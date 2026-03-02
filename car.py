import sensor, image, time

# ====== 你需要用 Threshold Editor 自己调 ======
greeb_threshold  = (55, 79, -31, -105, 118, 19)
blue_threshold = (55, 79, -1, -67, -44, 17)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.set_brightness(0)
sensor.skip_frames(time=2000)

sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False, exposure_us=1000)

clock = time.clock()

while True:
    clock.tick()
    img = sensor.snapshot()

    greeb_blob = None
    blue_blob = None

    # ===== 找绿色 =====
    greeb_blobs = img.find_blobs([greeb_threshold],
                                 pixels_threshold=200,
                                 area_threshold=200,
                                 merge=True)

    if greeb_blobs:
        greeb_blob = max(greeb_blobs, key=lambda b: b.pixels())
        img.draw_rectangle(greeb_blob.rect(), color=(0,255,0))
        img.draw_cross(greeb_blob.cx(), greeb_blob.cy(), color=(0,255,0))

    # ===== 找蓝色 =====
    blue_blobs = img.find_blobs([blue_threshold],
                                pixels_threshold=200,
                                area_threshold=200,
                                merge=True)

    if blue_blobs:
        blue_blob = max(blue_blobs, key=lambda b: b.pixels())
        img.draw_rectangle(blue_blob.rect(), color=(0,0,255))
        img.draw_cross(blue_blob.cx(), blue_blob.cy(), color=(0,0,255))

    # ===== 如果两个都找到 =====
    if greeb_blob and blue_blob:

        # 中点
        center_x = (greeb_blob.cx() + blue_blob.cx()) // 2
        center_y = (greeb_blob.cy() + blue_blob.cy()) // 2

        # 画红色十字（中点）
        img.draw_cross(center_x, center_y, color=(255,0,0), size=15, thickness=2)

        # 画连接线
        img.draw_line(greeb_blob.cx(), greeb_blob.cy(),
                      blue_blob.cx(), blue_blob.cy(),
                      color=(255,0,0), thickness=2)

        # ===== 画整体外接矩形 =====
        x_min = min(greeb_blob.x(), blue_blob.x())
        y_min = min(greeb_blob.y(), blue_blob.y())

        x_max = max(greeb_blob.x() + greeb_blob.w(),
                    blue_blob.x() + blue_blob.w())

        y_max = max(greeb_blob.y() + greeb_blob.h(),
                    blue_blob.y() + blue_blob.h())

        width = x_max - x_min
        height = y_max - y_min

        img.draw_rectangle(x_min, y_min, width, height,
                           color=(255,0,0), thickness=2)

        print("Center:", center_x, center_y)

    print(clock.fps())
