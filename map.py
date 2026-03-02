import sensor, image, time

# ====== 阈值 ======
wall_threshold  = (14, 92, -3, 24, -17, 13)
box_threshold = (20, 92, 12, -5, 39, 111)
bomb_threshold = (20, 92, 20, 94, 28, 111)
target_threshold = (17, 89, 88, 104, -86, 82)

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

    # 生成二值掩码，仅显示指定阈值部分
    binary_img = img.binary([box_threshold])

    # 形态学处理
    # 1. 腐蚀: 去掉小亮点
    binary_img.erode(1)  # 参数是腐蚀迭代次数，可调大一点
    # 2. 膨胀: 扩大剩余区域
    binary_img.dilate(1)  # 参数是膨胀迭代次数
    # 3. 去除小噪点
    binary_img.remove_small_objects(min_size=30, connectivity=8)  # min_size 可调

    # 显示二值图像
    img.replace(binary_img)

    # 寻找指定阈值的目标（可以换成 box_threshold / wall_threshold 等）
    blobs = img.find_blobs([wall_threshold], pixels_threshold=50, area_threshold=50, merge=True)

    for blob in blobs:
        # 画出目标矩形和中心点
        img.draw_rectangle(blob.rect(), color=(255,0,0))
        img.draw_cross(blob.cx(), blob.cy(), color=(0,255,0))
        print("目标中心: ({}, {}), 宽高: ({}, {})".format(blob.cx(), blob.cy(), blob.w(), blob.h()))

    # 显示FPS
    print("FPS:", clock.fps())