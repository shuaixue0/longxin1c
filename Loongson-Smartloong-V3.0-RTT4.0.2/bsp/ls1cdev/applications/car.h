
#ifndef __CAR_H__
#define __CAR_H__


#define led_gpio 52
#define key_gpio 85

//寻迹开关
#define key_led_L 64
#define key_led_R 65


//L298N驱动控制
#define motor_L1 60
#define motor_L2 61
#define motor_R1 62
#define motor_R2 63

#define PWM_L  6
#define PWM_R  92

void car_uart1_irqhandler(int IRQn, void *param);
void car_uart3_irqhandler(int IRQn, void *param);
void car_uart_init();
void car_pin_init();
void car_set_pwm(float pWM1_L,float PWM2_R,int en_1,int en_2);
void uart1_send_dat();
void uart3_send_dat(int dat);

#endif