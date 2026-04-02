"""
步骤一：屏幕/迷宫边界定位 + 透视矫正锁定
目标：自动找到迷宫四角并透视拉正，在 IDE 帧缓冲区中直观验证效果。
验收标准：矫正后迷宫网格线看起来横平竖直，四边基本贴近画面边缘。
"""
import sensor, image, time

# ===================== 配置 =====================

# 已标定的蓝色路面阈值（用于备用方案）
road_threshold = (7, 96, 9, 115, -114, -78)

# find_rects 边缘强度阈值，找不到矩形时逐步降低
RECT_THRESHOLD  = 5000
RECT_THRESHOLD_LOW = 1000   # 降级阈值

# 多帧平均帧数
SEARCH_FRAMES = 40

# ---- 手动保底角点（来自 grid.py 实测，自动检测失败时使用）----
# 顺序：左上、右上、右下、左下
MANUAL_CORNERS = [
    (21,  8),
    (149, 14),
    (135, 99),
    (24,  96),
]

# =============================================

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)   # 160×120

# 先让相机自动稳定，再锁定曝光/白平衡
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.skip_frames(time=500)

clock = time.clock()

corner_accumulator = []
locked_corners     = None   # 锁定后不再变
frame_count        = 0
lock_source        = ""     # 记录是哪种方法锁定的


# ===================== 工具函数 =====================
def order_corners(corners):
    """将4个角点排列为 [左上, 右上, 右下, 左下]"""
    corners = list(corners)
    tl = min(corners, key=lambda c: c[0] + c[1])
    br = max(corners, key=lambda c: c[0] + c[1])
    bl = min(corners, key=lambda c: c[0] - c[1])
    tr = max(corners, key=lambda c: c[0] - c[1])
    return [tl, tr, br, bl]


def average_corners(accumulator):
    """对多帧角点取均值"""
    n = len(accumulator)
    result = []
    for i in range(4):
        x = sum(c[i][0] for c in accumulator) // n
        y = sum(c[i][1] for c in accumulator) // n
        result.append((x, y))
    return result


def corners_valid(corners, min_area=3000, ratio_range=(0.4, 3.0)):
    """
    检查角点围成的矩形是否合理：
    - 面积不能太小
    - 宽高比在合理范围内
    """
    xs = [c[0] for c in corners]
    ys = [c[1] for c in corners]
    w = max(xs) - min(xs)
    h = max(ys) - min(ys)
    if h == 0:
        return False
    area  = w * h
    ratio = w / h
    return area > min_area and ratio_range[0] < ratio < ratio_range[1]


# ===================== 主循环 =====================
while True:
    clock.tick()
    img = sensor.snapshot()

    # ========= 第一阶段：自动寻找迷宫角点 =========
    if locked_corners is None:

        found_this_frame = False

        # --- 方法①：find_rects 强阈值 ---
        rects = img.find_rects(threshold=RECT_THRESHOLD)
        if rects:
            best = max(rects, key=lambda r: r.w() * r.h())
            c = order_corners(best.corners())
            if corners_valid(c):
                corner_accumulator.append(c)
                img.draw_rectangle(best.rect(), color=(0, 255, 0))
                found_this_frame = True

        # --- 方法②：find_rects 降级阈值 ---
        if not found_this_frame:
            rects = img.find_rects(threshold=RECT_THRESHOLD_LOW)
            if rects:
                best = max(rects, key=lambda r: r.w() * r.h())
                c = order_corners(best.corners())
                if corners_valid(c):
                    corner_accumulator.append(c)
                    img.draw_rectangle(best.rect(), color=(255, 128, 0))
                    found_this_frame = True

        # --- 方法③：对蓝色路面 blob 取外接矩形 ---
        if not found_this_frame:
            binary = img.copy().binary([road_threshold])
            binary.close(3)
            binary.open(2)
            blobs = binary.find_blobs(
                [(100, 255)],
                pixels_threshold=500,
                area_threshold=500,
                merge=True
            )
            if blobs:
                best_blob = max(blobs, key=lambda b: b.pixels())
                c = order_corners(best_blob.min_corners())
                if corners_valid(c, min_area=2000):
                    corner_accumulator.append(c)
                    img.draw_rectangle(best_blob.rect(), color=(255, 0, 255))
                    found_this_frame = True

        # 在画面上显示进度
        progress = len(corner_accumulator)
        img.draw_string(2, 2,
            "LOCK %d/%d" % (progress, SEARCH_FRAMES),
            color=(255, 255, 0), scale=1)

        frame_count += 1

        # ========= 采集足够帧后锁定 =========
        if len(corner_accumulator) >= SEARCH_FRAMES:
            locked_corners = average_corners(corner_accumulator)
            lock_source = "AUTO"
            print("=== 自动锁定成功 ===")
            print("角点:", locked_corners)

        # 超时保底：帧数用完但收集帧不足，使用手动角点
        elif frame_count >= SEARCH_FRAMES * 2:
            locked_corners = MANUAL_CORNERS
            lock_source = "MANUAL"
            print("=== 自动检测失败，使用手动角点 ===")
            print("角点:", locked_corners)

    # ========= 第二阶段：透视矫正 + 效果展示 =========
    else:
        # 透视矫正：将锁定的四角拉满整个 160×120 画面
        img.rotation_corr(corners=locked_corners, dest=(160, 120))

        # 叠加来源标注（方便在 IDE 中确认）
        img.draw_string(2, 2, lock_source, color=(0, 255, 255), scale=1)

        # 叠加粗略网格辅助目视验证（16列×12行）
        cols, rows = 16, 12
        cw = img.width()  / cols
        ch = img.height() / rows
        for i in range(cols + 1):
            img.draw_line(int(i * cw), 0, int(i * cw), img.height(),
                          color=(200, 200, 200))
        for j in range(rows + 1):
            img.draw_line(0, int(j * ch), img.width(), int(j * ch),
                          color=(200, 200, 200))

    # print("FPS:", clock.fps())
