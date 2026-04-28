"""
PC 端全局定位脚本
读取 iVCam 虚拟摄像头画面，检测场地四角 AprilTag 做透视矫正，
检测车模 AprilTag 做定位，通过串口发送坐标给主控。

依赖安装：pip install opencv-python pupil-apriltags pyserial numpy

操作：
  Q  退出
  R  重新检测角点
  S  保存当前截图
  D  切换调试信息显示
"""

import cv2
import numpy as np
import time
import sys

try:
    from pupil_apriltags import Detector
except ImportError:
    try:
        from dt_apriltags import Detector
    except ImportError:
        print("缺少 apriltag 库，请执行:")
        print("  pip install pupil-apriltags")
        sys.exit(1)

# ===================== 配置 =====================

CAM_INDEX = 0               # iVCam 摄像头编号，找不到时改 1 或 2
CAM_W     = 1280             # 采集分辨率（iVCam 实测最高 1280x720）
CAM_H     = 720

SERIAL_PORT    = "COM3"
SERIAL_BAUD    = 115200
SERIAL_ENABLED = False       # 调试阶段先关闭

FIELD_W = 320                # 矫正后输出尺寸（与场地 3.2:2.4 = 4:3 一致）
FIELD_H = 240

# --- AprilTag ID 配置 ---
# 比赛规则：场地四角为 TAG16H5 的 ID 15,16,17,18
# 如果你的练习场用了不同 ID，在这里修改
# 设为 None 则自动发现模式：取画面中离四角最近的 4 个 tag
CORNER_IDS = {15, 16, 17, 18}  # 场地四角 tag ID
CAR_IDS    = {1, 2, 3, 4, 5}

GRID_COLS = 14
GRID_ROWS = 10

LOCK_FRAMES   = 10           # 多帧平均后锁定
CLAHE_ENABLED = True         # CLAHE 直方图增强（斜视角/光线不均时有效）
ROI_SCALE     = 2.0          # 角落区域放大倍数（远端小 tag 靠此检测）
CORNER_OUTWARD = 0.5         # 角点外推系数：0=tag外角，1=再外推一个半tag宽度，按需微调

DST_CORNERS = np.float32([
    [0,           0],            # 左上
    [FIELD_W - 1, 0],            # 右上
    [FIELD_W - 1, FIELD_H - 1],  # 右下
    [0,           FIELD_H - 1],  # 左下
])

# ===================== 初始化 =====================

detector = Detector(
    families="tag16h5",
    nthreads=4,
    quad_decimate=1.0,         # 不降采样，保留远端小标签细节
    quad_sigma=0.4,            # 轻度高斯模糊，抑制噪声帮助检测远端小标签
    decode_sharpening=0.5,     # 增强解码锐度，改善斜视角下的模糊标签
    refine_edges=True,         # 亚像素边缘精修，提高角点精度
)

clahe = cv2.createCLAHE(clipLimit=4.0, tileGridSize=(8, 8))

MIN_TAG_DECISION_MARGIN = 30  # 低于此置信度的检测结果丢弃（避免误识别）

# ===================== 工具函数 =====================

def scan_cameras(max_id=5):
    """扫描并列出可用摄像头"""
    found = []
    for i in range(max_id):
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret and frame is not None:
                h, w = frame.shape[:2]
                print(f"  摄像头 {i}: {w}x{h}")
                found.append(i)
            cap.release()
    return found


def _gamma(gray, gamma):
    lut = np.uint8(255 * (np.arange(256) / 255.0) ** (1.0 / gamma))
    return cv2.LUT(gray, lut)


def detect_all(gray):
    """多轮预处理 + ROI 放大检测 AprilTag，合并去重，过滤低置信度"""
    seen_ids = {}

    def _merge(dets, offset_x=0, offset_y=0, scale=1.0):
        for d in dets:
            if d.decision_margin < MIN_TAG_DECISION_MARGIN:
                continue
            tid = d.tag_id
            if tid not in seen_ids or d.decision_margin > seen_ids[tid].decision_margin:
                if scale != 1.0 or offset_x or offset_y:
                    d.center = (d.center[0] / scale + offset_x,
                                d.center[1] / scale + offset_y)
                    d.corners = d.corners / scale + np.array([offset_x, offset_y])
                seen_ids[tid] = d

    # 第 1 轮：原始灰度
    _merge(detector.detect(gray))

    # 第 2 轮：CLAHE 直方图均衡
    if CLAHE_ENABLED:
        _merge(detector.detect(clahe.apply(gray)))

    # 第 3 轮：Gamma 提亮
    _merge(detector.detect(_gamma(gray, 1.8)))

    # 第 4 轮：自适应阈值二值化
    thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                   cv2.THRESH_BINARY, 31, 5)
    _merge(detector.detect(thresh))

    # 第 5 轮：四角 ROI 放大检测（远端小 tag 在全图仅 ~20px，放大后可检测）
    h, w = gray.shape
    rh, rw = h * 2 // 3, w * 2 // 3
    roi_defs = [
        (0, 0, rw, rh),              # 左上
        (w - rw, 0, w, rh),          # 右上
        (0, h - rh, rw, h),          # 左下
        (w - rw, h - rh, w, h),      # 右下
    ]
    s = ROI_SCALE
    sharpen_k = np.array([[-1, -1, -1], [-1, 9, -1], [-1, -1, -1]])
    for (x1, y1, x2, y2) in roi_defs:
        crop = gray[y1:y2, x1:x2]
        big = cv2.resize(crop, None, fx=s, fy=s, interpolation=cv2.INTER_CUBIC)
        _merge(detector.detect(big), x1, y1, s)
        _merge(detector.detect(cv2.filter2D(big, -1, sharpen_k)), x1, y1, s)
        _merge(detector.detect(clahe.apply(big)), x1, y1, s)

    return list(seen_ids.values())


def detect_fast(gray):
    """锁定后的轻量检测：只跑原图+CLAHE，足够检测较大的车模 tag"""
    seen_ids = {}
    for img in [gray, clahe.apply(gray)]:
        for d in detector.detect(img):
            if d.decision_margin < MIN_TAG_DECISION_MARGIN:
                continue
            if d.tag_id not in seen_ids or d.decision_margin > seen_ids[d.tag_id].decision_margin:
                seen_ids[d.tag_id] = d
    return list(seen_ids.values())


def quad_area(pts):
    """Shoelace formula for quadrilateral area (4 points, ordered)"""
    n = len(pts)
    area = 0.0
    for i in range(n):
        j = (i + 1) % n
        area += pts[i][0] * pts[j][1]
        area -= pts[j][0] * pts[i][1]
    return abs(area) / 2.0


def pick_corner_tags(detections, img_w, img_h):
    """
    自动发现模式：从所有非车模 tag 中，选出围成最大面积四边形的 4 个。
    这 4 个就是场地角点（因为场地占画面最大区域）。
    """
    candidates = [d for d in detections if d.tag_id not in CAR_IDS]
    if len(candidates) < 4:
        return None

    from itertools import combinations

    best_area = 0
    best_group = None

    for combo in combinations(candidates, 4):
        centers = [d.center for d in combo]
        pts = [(c[0], c[1]) for c in centers]

        # Order: 左上 右上 右下 左下
        tl = min(pts, key=lambda p: p[0] + p[1])
        br = max(pts, key=lambda p: p[0] + p[1])
        tr = max(pts, key=lambda p: p[0] - p[1])
        bl = min(pts, key=lambda p: p[0] - p[1])
        ordered = [tl, tr, br, bl]

        area = quad_area(ordered)
        if area > best_area:
            best_area = area
            best_group = list(combo)

    return best_group


def order_corners_by_position(detections):
    """
    将四个 AprilTag 按空间位置排列为 [左上, 右上, 右下, 左下]。
    使用每个 tag 最靠近场地角的那个角点（而非中心），以提高透视变换精度。
    返回 (src_corners, id_map)
    """
    pts = [(det.center[0], det.center[1], det) for det in detections]

    tl_det = min(pts, key=lambda p: p[0] + p[1])[2]
    br_det = max(pts, key=lambda p: p[0] + p[1])[2]
    tr_det = max(pts, key=lambda p: p[0] - p[1])[2]
    bl_det = min(pts, key=lambda p: p[0] - p[1])[2]

    def _outer_corner(det, key_fn, use_min):
        """从 tag 的 4 个角点中选出最靠近场地角的那个，并按 CORNER_OUTWARD 向外推"""
        c = det.corners
        sel = min if use_min else max
        i = sel(range(4), key=lambda j: key_fn(c[j][0], c[j][1]))
        outer = c[i]
        center = np.array(det.center)
        direction = outer - center
        return outer + direction * CORNER_OUTWARD

    src = np.float32([
        _outer_corner(tl_det, lambda x, y: x + y, True),   # 左上：min(x+y)
        _outer_corner(tr_det, lambda x, y: x - y, False),   # 右上：max(x-y)
        _outer_corner(br_det, lambda x, y: x + y, False),   # 右下：max(x+y)
        _outer_corner(bl_det, lambda x, y: x - y, True),    # 左下：min(x-y)
    ])

    id_map = {
        "左上": tl_det.tag_id,
        "右上": tr_det.tag_id,
        "右下": br_det.tag_id,
        "左下": bl_det.tag_id,
    }

    return src, id_map


def tag_heading(det):
    """
    根据 AprilTag corners 计算 4 方向朝向。
    pupil_apriltags corners: [左下, 右下, 右上, 左上]（tag 自身坐标系）
    返回: 0=上, 1=右, 2=下, 3=左
    """
    c = det.corners
    top_mid = (c[2] + c[3]) / 2
    bot_mid = (c[0] + c[1]) / 2
    dx = top_mid[0] - bot_mid[0]
    dy = top_mid[1] - bot_mid[1]
    angle = np.degrees(np.arctan2(-dy, dx)) % 360
    if 45 <= angle < 135:
        return 0
    elif 135 <= angle < 225:
        return 3
    elif 225 <= angle < 315:
        return 2
    else:
        return 1


def draw_tag(img, det, color=(0, 255, 0), thickness=2):
    """在图像上绘制 AprilTag 边框和 ID"""
    corners = det.corners.astype(int)
    for i in range(4):
        cv2.line(img, tuple(corners[i]), tuple(corners[(i + 1) % 4]),
                 color, thickness)
    cx, cy = int(det.center[0]), int(det.center[1])
    cv2.putText(img, str(det.tag_id), (cx - 10, cy - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)


def send_car_position(ser, row, col, heading):
    """
    串口发送车模位置。
    协议: [0xAA][0x55][row][col][heading][xor_checksum]
    """
    if ser is None:
        return
    payload = bytes([row & 0xFF, col & 0xFF, heading & 0xFF])
    chk = 0
    for b in payload:
        chk ^= b
    ser.write(b'\xAA\x55' + payload + bytes([chk]))


# ===================== 主程序 =====================

def main():
    print("=== PC 全局定位 (iVCam + AprilTag) ===\n")

    # --- 扫描摄像头 ---
    print("扫描摄像头:")
    cams = scan_cameras()
    if not cams:
        print("未找到任何摄像头！请确认 iVCam 已连接。")
        return

    idx = CAM_INDEX
    if idx not in cams:
        idx = cams[0]
        print(f"CAM_INDEX={CAM_INDEX} 不可用，自动使用摄像头 {idx}")

    cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print(f"无法打开摄像头 {idx}")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"\n使用摄像头 {idx}，分辨率 {actual_w}x{actual_h}")

    # --- 串口 ---
    ser = None
    if SERIAL_ENABLED:
        try:
            import serial
            ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.1)
            print(f"串口 {SERIAL_PORT} 已打开")
        except Exception as e:
            print(f"串口打开失败: {e}")

    # --- 状态 ---
    M = None
    locked = False
    corner_history = []
    id_history = []
    discovered_corner_ids = set()
    show_debug = True

    HEADING_STR = ["U", "R", "D", "L"]

    mode_str = "自动发现" if CORNER_IDS is None else f"指定ID {CORNER_IDS}"
    print(f"角点模式: {mode_str}")
    print("\n快捷键: Q=退出  R=重新检测  S=截图  D=调试信息\n")

    fps_t = time.time()
    fps_count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = detect_all(gray) if not locked else detect_fast(gray)

        # --- 分类：角点 vs 车模 vs 其他 ---
        if CORNER_IDS is not None:
            corner_dets = [d for d in detections if d.tag_id in CORNER_IDS]
        else:
            corner_dets = []
        car_dets = [d for d in detections if d.tag_id in CAR_IDS]

        # ============ 阶段一：锁定四角 ============
        if not locked:
            for det in detections:
                is_corner = det.tag_id in CORNER_IDS if CORNER_IDS else det.tag_id not in CAR_IDS
                color = (0, 255, 0) if is_corner else (255, 0, 0)
                draw_tag(frame, det, color)

            # 确定角点候选
            if CORNER_IDS is not None:
                candidates = corner_dets
                enough = len(candidates) == 4
            else:
                picked = pick_corner_tags(detections, actual_w, actual_h)
                candidates = picked if picked else []
                enough = picked is not None

            if enough and len(candidates) == 4:
                src, cur_id_map = order_corners_by_position(candidates)
                corner_history.append(src)
                id_history.append(cur_id_map)

                progress = f"LOCKING {len(corner_history)}/{LOCK_FRAMES}"
                cv2.putText(frame, progress, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

                if len(corner_history) >= LOCK_FRAMES:
                    avg = np.mean(corner_history, axis=0).astype(np.float32)
                    M = cv2.getPerspectiveTransform(avg, DST_CORNERS)
                    locked = True

                    final_id_map = id_history[-1]
                    discovered_corner_ids = set(final_id_map.values())

                    print("=== 角点锁定成功 ===")
                    print(f"角点 ID: {final_id_map}")
                    print(f"平均角点坐标 (使用 tag 外侧角):")
                    for name, pt in zip(["左上","右上","右下","左下"], avg):
                        print(f"  {name}: ({pt[0]:.1f}, {pt[1]:.1f})")
            else:
                n_total = len(detections)
                n_non_car = len([d for d in detections if d.tag_id not in CAR_IDS])
                info = f"Tags:{n_total} (non-car:{n_non_car}/4)"
                all_ids = [d.tag_id for d in detections]
                cv2.putText(frame, info, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                if show_debug:
                    cv2.putText(frame, f"IDs: {all_ids}", (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

            disp = cv2.resize(frame, (960, 540))
            cv2.imshow("Global", disp)

        # ============ 阶段二：矫正 + 车模定位 ============
        else:
            warped = cv2.warpPerspective(frame, M, (FIELD_W, FIELD_H))

            # --- 在原图（高分辨率）中检测车模，变换坐标到矫正空间 ---
            car_info = None
            for det in car_dets:
                cx, cy = det.center
                pt = np.float32([[[cx, cy]]])
                wpt = cv2.perspectiveTransform(pt, M)[0][0]
                wx, wy = wpt

                if not (0 <= wx < FIELD_W and 0 <= wy < FIELD_H):
                    continue

                cell_w = FIELD_W / GRID_COLS
                cell_h = FIELD_H / GRID_ROWS
                col = max(0, min(int(wx / cell_w), GRID_COLS - 1))
                row = max(0, min(int(wy / cell_h), GRID_ROWS - 1))

                heading = tag_heading(det)
                car_info = (row, col, heading, det.tag_id)

                wcx, wcy = int(wx), int(wy)
                cv2.circle(warped, (wcx, wcy), 6, (0, 0, 255), 2)
                label = f"ID{det.tag_id} {HEADING_STR[heading]}"
                cv2.putText(warped, label, (wcx + 8, wcy + 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1)

                draw_tag(frame, det, (0, 0, 255))

            # --- 画网格 ---
            cell_w = FIELD_W / GRID_COLS
            cell_h = FIELD_H / GRID_ROWS
            for i in range(GRID_COLS + 1):
                x = int(i * cell_w)
                cv2.line(warped, (x, 0), (x, FIELD_H), (80, 80, 80), 1)
            for j in range(GRID_ROWS + 1):
                y = int(j * cell_h)
                cv2.line(warped, (0, y), (FIELD_W, y), (80, 80, 80), 1)

            # --- 在原图画锁定的角点 ---
            for det in [d for d in detections if d.tag_id in discovered_corner_ids]:
                draw_tag(frame, det, (0, 255, 0))

            # --- 状态栏 ---
            status = "LOCKED"
            if car_info:
                r, c, h, tid = car_info
                status += f" | CAR ID{tid} R{r}C{c} {HEADING_STR[h]}"
                send_car_position(ser, r, c, h)
            cv2.putText(warped, status, (4, 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.32, (0, 255, 255), 1)

            # --- 显示 ---
            disp_warped = cv2.resize(warped, (640, 480),
                                     interpolation=cv2.INTER_NEAREST)
            disp_orig = cv2.resize(frame, (960, 540))
            cv2.imshow("Global", disp_orig)
            cv2.imshow("Global - Corrected", disp_warped)

        # --- FPS ---
        fps_count += 1
        if time.time() - fps_t >= 2.0:
            fps = fps_count / (time.time() - fps_t)
            n_corner = len(corner_dets) if CORNER_IDS else len([d for d in detections if d.tag_id not in CAR_IDS])
            print(f"FPS: {fps:.1f}  tags:{len(detections)}  corners:{n_corner}  cars:{len(car_dets)}")
            fps_t = time.time()
            fps_count = 0

        # --- 按键 ---
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            locked = False
            M = None
            corner_history = []
            id_history = []
            discovered_corner_ids = set()
            print(">>> 重新检测角点")
        elif key == ord('s'):
            ts = int(time.time())
            cv2.imwrite(f"D:/Car/pic/global_{ts}.jpg", frame)
            if locked:
                cv2.imwrite(f"D:/Car/pic/corrected_{ts}.jpg", warped)
            print(f">>> 截图已保存 D:/Car/pic/global_{ts}.jpg")
        elif key == ord('d'):
            show_debug = not show_debug
            print(f">>> 调试信息: {'开' if show_debug else '关'}")

    cap.release()
    cv2.destroyAllWindows()
    if ser:
        ser.close()
    print("已退出")


if __name__ == "__main__":
    main()
