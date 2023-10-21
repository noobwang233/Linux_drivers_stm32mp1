#ifndef _LED_H
#define _LED_H 
#include <linux/types.h>   // 定义了u8 u32的头文件
//only used for LED0 PI
#define LED_ON  1
#define LED_OFF 0

extern void led_init(void);
extern void led_deinit(void);
extern void led_on(void);
extern void led_off(void);
extern void led_trigger(void);
extern u8 get_led_status(void);

#endif


