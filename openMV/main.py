import sensor, image, time
from pyb import UART
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
clock = time.clock()
uart = UART(3, 9600)
gat_dat = ""

number1=0
number2=0
gat_flag=0
while(True):

    clock.tick()
    img = sensor.snapshot().lens_corr(1.8)
    for c in img.find_circles(threshold = 1800, x_margin = 10, y_margin = 10, r_margin = 10,
            r_min = 2, r_max = 50, r_step = 2):
        area = (c.x()-c.r(), c.y()-c.r(), 2*c.r(), 2*c.r())
        #area为识别到的圆的区域，即圆的外接矩形框
        statistics = img.get_statistics(roi=area)#像素颜色统计
        #print(statistics)
        #(0,100,0,120,0,120)是红色的阈值，所以当区域内的众数（也就是最多的颜色），范围在这个阈值内，就说明是红色的圆。(42, 77, -5, 44, -35, 76)
        #l_mode()，a_mode()，b_mode()是L通道，A通道，B通道的众数。
        number1=number1+1
        if 14<statistics.l_mode()<95 and -7<statistics.a_mode()<22 and -5<statistics.b_mode()<30:#if the circle is red
            img.draw_circle(c.x(), c.y(), c.r(), color = (255, 0, 0))#识别到的红色圆形用红色的圆框出来


        else:
            img.draw_rectangle(area, color = (255, 255, 255))
            #将非红色的圆用白色的矩形框出来
    print("FPS %f" % clock.fps())
    number2=number2+1
    if 50<number2  and gat_flag==1:
       number2=0
       if  number1>20:
           number1=0
           output_str="%c%c%c" % (0xaa,'1',0xBB) #方式1
           uart.write(output_str)
           print("you want")
       else :
           number1=0
           output_str="%c%c%c" % (0xaa,'2',0xBB) #方式1
           uart.write(output_str)
           print("tongzhu")
    if uart.any():
      gat_dat = uart.read(3).decode()
      print(gat_dat[1])
      if gat_dat[1]=='1':
         gat_flag=1
      if gat_dat[1]=='0':
         gat_flag=0
