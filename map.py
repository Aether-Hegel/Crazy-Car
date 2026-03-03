import sensor, image, time

# ====== 阈值 ======
# 假设我们把 type 定义为不同阈值对应的颜色类别
wall_threshold  = (14, 92, -3, 24, -17, 13)
box_threshold = (20, 92, 12, -5, 39, 111)
bomb_threshold = (20, 92, 20, 94, 28, 111)
target_threshold = (17, 89, 88, 104, -86, 82)
true_threshold = (21, 100, -128, 127, -128, 127)

# 用于示例，这里只分析 box_threshold
current_threshold = wall_threshold
current_type = 1  # 给每个阈值指定一个 type

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.set_brightness(0)
sensor.skip_frames(time=2000)

sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False, exposure_us=1000)

clock = time.clock()

PIXEL_THRESHOLD = 5
AREA_THRESHOLD = 5

while True:
    clock.tick()
    img = sensor.snapshot()

    # 二值化 + 形态学处理
    binary_img = img.binary([current_threshold])
    binary_img.open(1)
    binary_img.close(1)
    img.replace(binary_img)

    # 连通域分析
    blobs = img.find_blobs([true_threshold],
                           pixels_threshold=PIXEL_THRESHOLD,
                           area_threshold=AREA_THRESHOLD,
                           merge=True)

    # 打印 object_count
    print("object_count: {}".format(len(blobs)))

    # 遍历每个 blob，打印格式化信息
    for i, blob in enumerate(blobs):
        x, y, w, h = blob.rect()
        print("Object[{}] type:{} x:{} y:{} w:{} h:{}".format(i, current_type, x, y, w, h))

    # print("FPS:", clock.fps())

