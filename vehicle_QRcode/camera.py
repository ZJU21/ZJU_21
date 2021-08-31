import sensor, image, time,pyb
from pyb import UART

uart=UART(3,115200)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(30)
#sensor.set_vflip(True) # 垂直翻转
#sensor.set_hmirror(True) # 水平翻转
sensor.set_auto_gain(False)  # 必须关闭此功能，以防止图像冲洗…
clock = time.clock()

colors={"红色红色":0,
        "红色绿色":1,
        "红色蓝色":2,
        "绿色红色":3,
        "绿色绿色":4,
        "绿色蓝色":5,
        "蓝色红色":6,
        "蓝色绿色":7,
        "蓝色蓝色":8}


while(True):
    clock.tick()
    img = sensor.snapshot()
    for code in img.find_qrcodes():
        if code.payload() in colors:
            uart.write("c%d"%colors[code.payload()])
            for i in range(0,colors[code.payload()]):
                led = pyb.LED(3)
                led.on()
                time.sleep_ms(150)     #延时150ms
                led.off()           #暗灯
                time.sleep_ms(100)
            led = pyb.LED(1)
            led.on()
            time.sleep_ms(150)     #延时150ms
            led.off()           #暗灯
            time.sleep_ms(100)

