/*
 * File      : test_gpio.c
测试gpio接口在finsh中运行
1. test_output()
2. test_delay_1us()
3. test_delay_1s()
4. print_clk()  将当前的分频的所有时钟都打印出来
5. test_rtdelay_1s 调用RT_TICK_PER_SECOND实现1s 延时测试
6. mem_read(***) 读取***寄存器中32位数据并打印
 */


/*
file: test_pin.c
测试pin驱动， 在finsh 中运行
1. test_pin(52)   GPIO52引脚每隔1秒输出电平翻转
 */

#include <rtthread.h>
#include <stdlib.h> 
#include <rthw.h> 



#include "ls1c.h"
#include <drivers/pin.h>
#include "../drivers/drv_gpio.h"
#include "../libraries/ls1c_pwm.h"
#include "../libraries/ls1c_public.h"
#include "../libraries/ls1c_delay.h"
#include "../libraries/ls1c_clock.h"
#include "ls1c_uart.h"
#include "ls1c_pin.h"  
#include "car.h"

#define THREAD_PRIORITY         25
#define THREAD_STACK_SIZE       512
#define THREAD_TIMESLICE        5

static struct rt_thread thread1;
static rt_uint8_t thread1_stack[THREAD_STACK_SIZE];
static struct rt_thread thread2;
static rt_uint8_t thread2_stack[THREAD_STACK_SIZE];
rt_int8_t motor_flag=0,turn_flag=0,stop_flag=0,thing_flag=0,add_num=0,send_flag=0;
unsigned char uart3_tx_dat[10];

/*
串口1的中断函数
作用：与上位机通信
*/
void car_uart1_irqhandler(int IRQn, void *param)
{
    int ch=0;
    unsigned char dat[20];
    ls1c_uart_t uartx = uart_irqn_to_uartx(IRQn);
    void *uart_base = uart_get_base(uartx);
   unsigned char iir = reg_read_8(uart_base + LS1C_UART_IIR_OFFSET);
    // 判断是否为接收超时或接收到有效数据
    if ((IIR_RXTOUT & iir) || (IIR_RXRDY & iir))
    {
        // 是，则读取数据，并原样发送回去
        while (LSR_RXRDY &(reg_read_8(uart_base + LS1C_UART_LSR_OFFSET)) )
        {           
          
           dat[ch++]=reg_read_8(uart_base + LS1C_UART_DAT_OFFSET);    
        }
        if(dat[0]==0xCC&&dat[2]==0xBB)
        {
            if(dat[1]=='1')
            {
              motor_flag=0;
             uart3_send_dat(0);
            }

        }
        if(dat[0]==0xBB&&dat[2]==0xAA)
        {
          uart3_tx_dat[1]=dat[1];
         uart1_send_dat();
          //uart_putc(LS1C_UART3,'k');
        }
    }    
    return ;
}
//uart1向上位机发送数据：
void uart1_send_dat()
{
          uart_putc(LS1C_UART1,0xBB);
          uart_putc(LS1C_UART1,'0'+turn_flag);
          uart_putc(LS1C_UART1,'0'+motor_flag);
          uart_putc(LS1C_UART1,'0'+stop_flag);
          uart_putc(LS1C_UART1,'0'+thing_flag);
          uart_putc(LS1C_UART1,0xAA);

}

/*
串口3中断函数
作用：与openMV通信

*/
void car_uart3_irqhandler(int IRQn, void *param)
{
   int ch=0;
   unsigned char dat[10]="0";
    ls1c_uart_t uartx = uart_irqn_to_uartx(IRQn);
    void *uart_base = uart_get_base(uartx);
   unsigned char iir = reg_read_8(uart_base + LS1C_UART_IIR_OFFSET);
    // 判断是否为接收超时或接收到有效数据
    if ((IIR_RXTOUT & iir) || (IIR_RXRDY & iir))
    {
        // 是，则读取数据，并原样发送回去
        while (LSR_RXRDY & reg_read_8(uart_base + LS1C_UART_LSR_OFFSET))
        {
            dat[ch] = reg_read_8(uart_base + LS1C_UART_DAT_OFFSET);
            ch++;
        }
        //uart_print(LS1C_UART1,dat);
         if(dat[0]==0xAA&&dat[2]==0xBB)
         {
           if(dat[1]=='1')
           {
             add_num++;
             thing_flag=thing_flag+1;
           }
          if(dat[1]=='2')
           {
             add_num++;
             
           }
         }
         if(add_num==5)
         {
           if(thing_flag>2)  turn_flag=1;
           else turn_flag=2;
          uart3_send_dat(0);
          //uart1_send_dat();
          motor_flag=!motor_flag;
         }
         uart1_send_dat();
        //uart_putc(LS1C_UART1,add_num);

    }    
    return ;
}
void uart3_send_dat(int dat)
{
          uart_putc(LS1C_UART3,'!');
          uart_putc(LS1C_UART3,'0'+dat);
          uart_putc(LS1C_UART3,'!');
}
/*
  串口的初始化：
  UART1 P2 P3
  UART3 P0 P1
*/
void car_uart_init()
{
    ls1c_uart_info_t uart_info = {0};	
    unsigned int rx_gpio ;
    unsigned int tx_gpio ;
    int dat;
//串口初始化
        rx_gpio = 0;
        tx_gpio = 1;
        pin_set_remap(tx_gpio, PIN_REMAP_FOURTH);
        pin_set_remap(rx_gpio, PIN_REMAP_FOURTH);
        uart_info.UARTx = LS1C_UART3;
        uart_info.baudrate = 9600;
        uart_info.rx_enable = TRUE;  
        uart_init(&uart_info);
        rt_hw_interrupt_umask(LS1C_UART3_IRQ);		
        //uart_print((ls1c_uart_t)LS1C_UART3, "\r\nThis is uart3 sending string.\r\n");
        rt_hw_interrupt_install(LS1C_UART3_IRQ, car_uart3_irqhandler, RT_NULL, "UART3");


        rx_gpio = 2;
        tx_gpio = 3;
        pin_set_remap(tx_gpio, PIN_REMAP_FOURTH);
        pin_set_remap(rx_gpio, PIN_REMAP_FOURTH);
        uart_info.UARTx = LS1C_UART1;
        uart_info.baudrate = 9600;
        uart_info.rx_enable = TRUE;  
        uart_init(&uart_info);
        rt_hw_interrupt_umask(LS1C_UART1_IRQ);		
        //uart_print((ls1c_uart_t)LS1C_UART1, "\r\nThis is uart1 receive string:");
        rt_hw_interrupt_install(LS1C_UART1_IRQ, car_uart1_irqhandler, RT_NULL, "UART1"); 
}
void car_pin_init()
{
 // pin初始化
  
    hw_pin_init();
    // 把相应gpio设为输出模式



    rt_pin_mode(key_gpio, PIN_MODE_INPUT);//按键
    rt_pin_mode(key_led_L, PIN_MODE_INPUT);//寻迹左
    rt_pin_mode(key_led_R, PIN_MODE_INPUT);//寻迹右

    rt_pin_mode(motor_L1, PIN_MODE_OUTPUT);//其他输出
    rt_pin_mode(motor_L2, PIN_MODE_OUTPUT);//其他输出
    rt_pin_mode(motor_R1, PIN_MODE_OUTPUT);//其他输出
    rt_pin_mode(motor_R2, PIN_MODE_OUTPUT);//其他输出
    rt_pin_mode(led_gpio, PIN_MODE_OUTPUT);//其他输出

    car_set_pwm(0.7,0.7,1,1);
    car_uart_init();
}

/*
L298N驱动电机控制
*/
void car_set_pwm(float pWM1_L,float PWM2_R,int en_1,int en_2)
{
    pwm_info_t pwm1_info;
    pwm_info_t pwm2_info;

    pwm1_info.gpio = LS1C_PWM0_GPIO06;               // pwm引脚位gpio06
    pwm1_info.mode = PWM_MODE_NORMAL;                // 正常模式--连续输出pwm波形
    pwm1_info.duty = PWM2_R;                           // pwm占空比
    pwm1_info.period_ns = 5*1000*1000;               // pwm周期5ms

    // pwm初始化，初始化后立即产生pwm波形
    pwm_init(&pwm1_info);

    pwm2_info.gpio = LS1C_PWM1_GPIO92;               // pwm引脚位gpio06
    pwm2_info.mode = PWM_MODE_NORMAL;                // 正常模式--连续输出pwm波形
    pwm2_info.duty = pWM1_L;                           // pwm占空比
    pwm2_info.period_ns = 5*1000*1000;               // pwm周期5ms

    // pwm初始化，初始化后立即产生pwm波形
    pwm_init(&pwm2_info);

    if(en_1==1)  pwm_enable(&pwm1_info);
    else pwm_disable(&pwm1_info);

    if(en_2==1)  pwm_enable(&pwm2_info);
    else pwm_disable(&pwm2_info);
}


void scan_ledkey()
{
  rt_int8_t i;
   while(1)
   {
     if(rt_pin_read(key_led_L)==PIN_LOW&&rt_pin_read(key_led_R)==PIN_HIGH)
     {
       car_set_pwm(0.85,0.3,1,1);//右偏
     }
    if(rt_pin_read(key_led_L)==PIN_HIGH&&rt_pin_read(key_led_R)==PIN_LOW)
     {
       car_set_pwm(0.3,0.85,1,1);//左偏
    
     }
    if(rt_pin_read(key_led_L)==PIN_HIGH&&rt_pin_read(key_led_R)==PIN_HIGH)
     {
        //rt_pin_write(led_gpio, PIN_HIGH);
       car_set_pwm(0.45,0.45,1,1);
     }

    
   if(rt_pin_read(key_led_L)==PIN_LOW&&rt_pin_read(key_led_R)==PIN_LOW)//到达T型路口
     {
        delay_us(40);
        if(rt_pin_read(key_led_L)==PIN_LOW&&rt_pin_read(key_led_R)==PIN_LOW)//到达T型路口
        {
                  //motor_flag=0;
          if(stop_flag==1)
            {

             motor_flag=0;  
             uart1_send_dat();      
            }
          else    
          {
             
                
                  if(turn_flag==1)
                  {
                   
                    while(1)
                    {
                        car_set_pwm(0.4,0.9,1,1);
                       if(rt_pin_read(key_led_R)==PIN_HIGH)
                        {
                          stop_flag=1;
                          uart1_send_dat();
                            break;
                        }  
                    }
            
                  }
                if(turn_flag==2)
                  {
                     //if(send_flag==0)     uart1_send_dat(),send_flag=1;
                    while(1)
                    {
                    car_set_pwm(0.9,0.4,1,1);
                    if(rt_pin_read(key_led_L)==PIN_HIGH)
                      {
                        stop_flag=1;
                        uart1_send_dat();
                        break;
                      }  
                    }
  
                  }
                
            }
          }
        // car_set_pwm(0.8,0.8,1,1);
       // rt_pin_write(led_gpio, PIN_LOW);
     }
     rt_thread_delay(RT_TICK_PER_SECOND/100);
   };
}

void test_car1()
{

while(1)
{
     if(PIN_LOW == rt_pin_read(key_gpio))
        {
          rt_pin_write(led_gpio, PIN_HIGH);
          //motor_flag=!motor_flag;
          uart3_send_dat(1);
          uart1_send_dat();
          //turn_flag=2;
          //motor_flag=1;    
          while(!rt_pin_read(key_gpio));
        }
     if(motor_flag==1)
     {
        rt_pin_write(led_gpio, PIN_LOW);
        rt_pin_write(motor_L1, PIN_HIGH);
        rt_pin_write(motor_L2, PIN_LOW);
        rt_pin_write(motor_R1, PIN_HIGH);
        rt_pin_write(motor_R2, PIN_LOW);
     }
    else
    {
         rt_pin_write(led_gpio, PIN_HIGH);
         rt_pin_write(motor_L1, PIN_LOW);
        rt_pin_write(motor_L2, PIN_LOW);
        rt_pin_write(motor_R1, PIN_LOW);
        rt_pin_write(motor_R2, PIN_LOW);
    }

 rt_thread_delay(RT_TICK_PER_SECOND/10);
}
 
};

void car()
{

    rt_thread_t tid;  
    rt_err_t result;
car_pin_init();


 result = rt_thread_init(&thread1,     
                            "test_car1",    
                            test_car1,    
                            RT_NULL,    
                            &thread1_stack[0],    
                            sizeof(thread1_stack),    
                            THREAD_PRIORITY,    
                            THREAD_TIMESLICE);    
    if (RT_EOK == result)    
    {    
        rt_thread_startup(&thread1);    
    }    

 result = rt_thread_init(&thread2,     
                            "scan_ledkey",    
                            scan_ledkey,    
                            RT_NULL,    
                            &thread2_stack[0],    
                            sizeof(thread2_stack),    
                            THREAD_PRIORITY-1,    
                            THREAD_TIMESLICE);    
    if (RT_EOK == result)    
    {    
        rt_thread_startup(&thread2);    
    }    




   /* rt_kprintf("motor_L1:%d\n",gpio_get(motor_L1));
      rt_kprintf("motor_L2:%d\n",gpio_get(motor_L2));
      rt_kprintf("motor_R1:%d\n",gpio_get(motor_R1));
      rt_kprintf("motor_R2:%d\n",gpio_get(motor_R2));
    rt_thread_delay(1 * RT_TICK_PER_SECOND);*/

}
  

 





 #include  <finsh.h> 
FINSH_FUNCTION_EXPORT(car , car  e.g.car());
//FINSH_FUNCTION_EXPORT(test_pin_key , test_pin led-gpio52 e.g.test_pin(52,85));
/* 导出到 msh 命令列表中 */
MSH_CMD_EXPORT(car, car);
//MSH_CMD_EXPORT(test_pinkey_msh, test_pinkey_msh 52 85);
