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

RED=(35, 57, 23, 88, 8, 64)
GREEN=(23, 72, -62, -13, 4, 43)
BLUE=(14, 57, -21, 27, -71, -18)
colors_thresholds=[[RED],[GREEN],[BLUE]]

colors_name={"红色红色":0,
            "红色绿色":1,
            "红色蓝色":2,
            "绿色红色":3,
            "绿色绿色":4,
            "绿色蓝色":5,
            "蓝色红色":6,
            "蓝色绿色":7,
            "蓝色蓝色":8}

colorsID=2    # 颜色代码编号，default=-1

work=False
getQR=False
getA=True
getB=False

def scanQR(img):
    print("star scan QRcode")
    global colorsID,colors_name
    for code in img.find_qrcodes():
        if code.payload() in colors_name:
            colorsID=colors_name[code.payload()]
            print("colors=",code.payload(),"codeID=",colorsID)
            uart.write("C%d\r\n"%colorsID)
            for i in range(0,colorsID):
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
            return True
    return False

def getNowColor(img):
    res_color,res=-1,{}
    for color in range(0,3):
        blobs=img.find_blobs(colors_thresholds[color])#,pixels_threshold=500)
        for blob in blobs:
            if res_color==-1 or blob.pixels()>res.pixels():
                res_color,res=color,blob
    return res_color,res

def scanThing(img,color):
    #print("start scan %d"%color)
    now_color,blob=getNowColor(img)
    #print("scan color ",color,";  now color:",now_color)
    if now_color!=color:
        return False
    #img.draw_rectangle(blob.rect(), color=(255,0,0))
    uart.write("%d %d %d\r\n"%(blob.cx(),blob.cy(),blob.pixels()))
    return True

print("start test?")
while (True):
    clock.tick()
    img = sensor.snapshot()
    while uart.any():
        ch=chr(uart.readchar())
        print("%c"%ch)
        if ch=='Q':
            getQR=True
        elif ch=='A':
            getA=True
        elif ch=='B':
            getB=True
        elif ch=='S':
            getA,getB=False,False
    if getQR:
        if (scanQR(img)):
            getQR=False
    elif getA:
        scanThing(img,colorsID//3)
    elif getB:
        scanThing(img,colorsID%3)