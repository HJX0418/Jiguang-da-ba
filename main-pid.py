#引入传感器、定时器、PID、Pin#
import sensor, image, time , pyb
from pid import PID
from machine import Pin
from pyb import Pin, Timer, delay, Servo

#--------------函数def-------------------#
#寻找距离2米，半径5cm的圆
def find_right_circles(blobs):
    right_R = 500
    d_R = 50
    right_circle = 0
    for blob in blobs:
        if abs(blob.r() - right_R) < d_R:
            right_circle = blob
            d_R = abs(blob.r() - right_R)
    return right_circle

#----------------------------------------#

#--------------变量设置-------------------#
time_set    =  2          #如果一直没有目标圆，3秒（30*100ms）后进入寻靶模式

down_multiple = 1          #舵机旋转方向反置时的系数
up_multiple   = 1
multiple_flag = 0

time_cnt    =  0          #计数初始化
right_blob  =  0          #正确色块初始化

max_degree  =  92         #舵机最大、最小角度
min_degree  =  -92
zero_degree = 0

max_pulse   =  2500       #脉冲宽度最大值、最小值、零值
min_pulse   =  500
zero_pulse  =  1500

find_flag   =  1          #寻靶模式标志位
degree_pulse = 110        #每增加或减少11为一度
find_pulse  =  100          #寻靶时每次运行的角度

roi_img = (32,24,256,192)    #图像感兴趣区
roi_light = roi_img          #激光感兴趣区

find_duty  = 2

light_threshold = (208, 255)
light_xerror = 0
light_yerror = 0
light_flag   = 0
#---------------------------------------#


#--------------初始化--------------------#
#摄像头初始化，QQVGA，关闭了白平衡#
sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.RGB565) # use RGB565.
sensor.set_framesize(sensor.QVGA) # use QQVGA for speed.
sensor.skip_frames(10) # Let new settings take affect.
sensor.set_auto_whitebal(False) # turn this off.
clock = time.clock() # Tracks FPS.

#上下舵机IO口设置
servo_down = Servo(1) # P7
servo_up = Servo(2)   # P8

#上下舵机当前角度初始化#
servo_down.calibration(min_pulse,max_pulse,zero_pulse)
servo_up.calibration(min_pulse,max_pulse,zero_pulse)

#down_pid = PID(p=0.55, i=0, imax=90) #脱机运行或者禁用图像传输，使用这个PID
#up_pid = PID(p=0.2, i=0, imax=90) #脱机运行或者禁用图像传输，使用这个PID
down_pid = PID(p=0.55, i=0, imax=90)#在线调试使用这个PID
up_pid   = PID(p=0.195, i=0, imax=90)#在线调试使用这个PID

#点亮绿灯作为程序运行的指令#
led_green = pyb.LED(2)    #Red LED = 3, Green LED = 2, Blue LED = 1, IR LEDs = 4.
#舵机位置初始化
down_current  =  zero_pulse
up_current    =  min_pulse
servo_down.pulse_width(down_current)
servo_up.pulse_width(up_current)

#---------------------------------------#

while True:

    clock.tick() # Track elapsed milliseconds between snapshots().
    led_green.on()


    img = sensor.snapshot() # Take a picture and return the image.

    #img.to_rgb565()


    #将图像转成灰度提高性能
    grayimg =img.to_grayscale()



    #霍夫变换寻找圆
    blobs = img.find_circles(roi_img ,threshold = 3400, x_margin = 10, y_margin = 10, r_margin = 10,
        r_min = 6, r_max = 8 , r_step = 2)

    #如果找到圆
    for right_blob in blobs :
        #如果有圆则寻靶标志位置零
        find_flag = 0

        roi_light = (right_blob.x()-15,right_blob.y()-15,66,66)
        for light_blob in img.find_blobs([light_threshold],roi= roi_light , pixels_threshold=0, area_threshold=0):
            pixels=light_blob.pixels()
            print(pixels)
            if 3< pixels <8 :
                img.draw_cross(light_blob.cx(),light_blob.cy())
                light_xerror = light_blob.cx()-right_blob.x()
                light_yerror = light_blob.cy()-right_blob.y()
        #计算圆与图像中心的误差，（24，100）是激光与openmv的机械误差
        target_width  = img.width()/2+24
        target_height = img.height()/2+100

        down_error = target_width -right_blob.x()
        up_error   = target_height-right_blob.y()

        #if (abs(light_xerror)>5 or abs(light_yerror)>5)and light_flag:
            #down_error = light_xerror
            #up_error   = light_yerror
        ##if abs(light_xerror)<5 and abs(light_yerror)<5:
            ##light_flag = 0
        #画出目标的十字
        img.draw_cross(int(img.width()/2), int(img.height()/2))

        #画出靶心
        img.draw_circle(right_blob.x(), right_blob.y(), right_blob.r(), color = (255, 0, 0))
        img.draw_cross(right_blob.x(), right_blob.y())

        #串口打印误差
        print("down_error: ", down_error)
        print("up_error: ", up_error)

        #pid计算输出角度
        down_output  =   down_pid.get_pid(down_error,1)
        up_output    =   up_pid.get_pid(up_error,1)
        print("down_output",down_output)
        print("up_output",down_output)

        #调整角度打靶
        up_current    = servo_up.pulse_width()
        down_current  = servo_down.pulse_width()


        servo_up.pulse_width(int(up_current+up_output))

        #如果上方舵机角度大于0，下方舵机减去误差；反之，增加；
        if servo_up.pulse_width() >= zero_pulse :
            servo_down.pulse_width(int(down_current-down_output))
        if servo_up.pulse_width() < zero_pulse :
            servo_down.pulse_width(int(down_current+down_output))
    right_blob = 0              #清空

    delay(1)
    time_cnt += 1;

    print("Time:",time_cnt,"ms")
    print("FPS: ",clock.fps())

    #初始化寻靶
    if find_flag and time_cnt % find_duty == 0  :
        multiple_flag = 0
        if time_cnt % (find_duty*2) == 0 :
            up_multiple *= -1
            if(down_current >= max_pulse):
                down_multiple  = -1
                up_current = max_pulse
                up_multiple = -1
                multiple_flag = 1
            if(down_current <= min_pulse):
                down_multiple  = 1
                up_current = min_pulse
                up_multiple = 1
                multiple_flag = 1
            down_current = servo_down.pulse_width()+find_pulse*down_multiple
        #if multiple_flag == 0:
            #up_current = servo_up.pulse_width()+find_pulse*up_multiple*2

        servo_down.pulse_width(down_current)
        servo_up.pulse_width(up_current)

        print("down_angle：",servo_down.angle(),"up_angle：",servo_up.angle())

