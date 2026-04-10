import sensor, image, time

# ===================== 配置 =====================
road_threshold = (7, 96, 9, 115, -114, -78)   # 蓝色路面 LAB 阈值（已标定）

SEARCH_FRAMES   = 30       # 第一阶段采集帧数
RECT_THRESHOLD  = 5000     # find_rects 边缘强度阈值，越大越严格
BLOB_MIN_AREA   = 2000     # blob 备用方案最小面积

cols = 16 - 2
rows = 12 - 2

SAMPLE_R    = 2            # 格子中心区域采样半径，采样区域为 (2r+1)² = 25 px
ROAD_RATIO  = 0.4          # 区域内蓝色像素占比 >= 此值 → 路，否则 → 墙

# ==============================================

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)   # 160x120

# 先让自动增益/白平衡稳定后再锁定，避免初始帧色偏
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.skip_frames(time=500)

clock = time.clock()

corner_accumulator = []
maze_corners_avg   = None
frame_count        = 0
map_sent           = False   # 地图只发送一次


# ===================== 工具函数 =====================
def order_corners_clockwise(corners):
    """将任意顺序的4个角点排列为 [TL, TR, BR, BL]"""
    corners = list(corners)
    tl = min(corners, key=lambda c: c[0] + c[1])
    br = max(corners, key=lambda c: c[0] + c[1])
    bl = min(corners, key=lambda c: c[0] - c[1])
    tr = max(corners, key=lambda c: c[0] - c[1])
    return [tl, tr, br, bl]


def sample_road_ratio(binary_img, cx, cy, r):
    """
    统计 (cx, cy) 周围 (2r+1)² 区域内蓝色（路面）像素的占比。
    返回值 0.0~1.0，越高越可能是路。
    """
    total = 0
    road  = 0
    img_w = binary_img.width()
    img_h = binary_img.height()
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            px, py = cx + dx, cy + dy
            if 0 <= px < img_w and 0 <= py < img_h:
                total += 1
                if binary_img.get_pixel(px, py)[0] > 0:
                    road += 1
    return road / total if total > 0 else 0.0


# ===================== 主循环 =====================
while True:
    clock.tick()
    img = sensor.snapshot()

    # ========= 第一阶段：寻找迷宫边界（多帧平均） =========
    if frame_count < SEARCH_FRAMES:

        found = False

        # --- 方法一：find_rects 精准矩形检测 ---
        rects = img.find_rects(threshold=RECT_THRESHOLD)
        if rects:
            best_rect = max(rects, key=lambda r: r.magnitude())
            rw = best_rect.w()
            rh = best_rect.h()
            ratio = rw / rh if rh != 0 else 0
            # 过滤太小或宽高比异常的矩形
            if rw * rh > 3000 and 0.5 < ratio < 2.5:
                corners = order_corners_clockwise(best_rect.corners())
                corner_accumulator.append(corners)
                img.draw_rectangle(best_rect.rect(), color=(0, 255, 0))
                found = True

        # --- 方法二（备用）：对路面二值化后找最大 blob ---
        if not found:
            binary = img.copy().binary([road_threshold])
            binary.close(3)
            binary.open(2)
            binary.dilate(1)
            blobs = binary.find_blobs(
                [(100, 255)],
                pixels_threshold=500,
                area_threshold=500,
                merge=True
            )
            best_blob  = None
            best_score = 0
            for blob in blobs:
                w     = blob.w()
                h     = blob.h()
                area  = blob.pixels()
                ratio = w / h if h != 0 else 0
                if 0.5 < ratio < 2.0 and area > BLOB_MIN_AREA:
                    if area > best_score:
                        best_score = area
                        best_blob  = blob

            if best_blob:
                corners = order_corners_clockwise(best_blob.min_corners())
                corner_accumulator.append(corners)
                img.draw_rectangle(best_blob.rect(), color=(255, 128, 0))

        frame_count += 1

        # ========= 采集完成 → 计算各角点均值 =========
        if frame_count == SEARCH_FRAMES and corner_accumulator:
            maze_corners_avg = []
            valid_frames = len(corner_accumulator)
            for i in range(4):
                # 平均前每帧已经过 order_corners_clockwise，顺序一致
                x_sum = sum(c[i][0] for c in corner_accumulator)
                y_sum = sum(c[i][1] for c in corner_accumulator)
                maze_corners_avg.append((x_sum // valid_frames,
                                         y_sum // valid_frames))
            print("锁定角点:", maze_corners_avg)
            print("有效帧数:", valid_frames, "/", SEARCH_FRAMES)

    # ========= 第二阶段：透视矫正 + 网格判断 =========
    else:
        if maze_corners_avg:

            # 透视矫正：将迷宫四角拉伸到整幅画面
            img.rotation_corr(corners=maze_corners_avg, dest=(160, 120))

            # 矫正后重新对路面二值化（蓝色 = 路 = 1）
            binary = img.copy().binary([road_threshold])
            binary.close(2)
            binary.open(1)

            # 每格像素尺寸
            cell_w = img.width()  / cols   # 10.0 px
            cell_h = img.height() / rows   # 10.0 px

            # ===== 绘制网格线（黄色，与红/绿标记色区分） =====
            for i in range(cols + 1):
                x = int(i * cell_w)
                img.draw_line(x, 0, x, img.height(), color=(255, 255, 0))
            for j in range(rows + 1):
                y = int(j * cell_h)
                img.draw_line(0, y, img.width(), y, color=(255, 255, 0))

            # ===== 逐格判断：路(0) / 墙(1) =====
            maze_map = []
            for j in range(rows):
                row = []
                for i in range(cols):
                    cx = int((i + 0.5) * cell_w)
                    cy = int((j + 0.5) * cell_h)

                    # 区域采样，比单点更鲁棒
                    ratio = sample_road_ratio(binary, cx, cy, SAMPLE_R)

                    if ratio >= ROAD_RATIO:
                        row.append(0)   # 路
                        img.draw_circle(cx, cy, 2, color=(0, 255, 0))
                    else:
                        row.append(1)   # 墙
                        img.draw_circle(cx, cy, 2, color=(255, 0, 0))

                maze_map.append(row)

            # ===== 地图只打印一次（避免刷屏拖慢帧率） =====
            if not map_sent:
                print("地图 [0=路, 1=墙]:")
                for r in maze_map:
                    print(r)
                map_sent = True

    # print("FPS:", clock.fps())
