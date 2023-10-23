#ifndef _LED_H
#define _LED_H 
#include <linux/types.h>   // 定义了ssize_t的头文件
#include <linux/kernel.h>
#include <linux/ide.h>
#include <linux/init.h>    // 模块加载init和卸载exit相关头文件
#include <linux/module.h>
#include <linux/cdev.h>         // cdev相关头文件
#include <linux/device.h>   //设备号dev_t相关头文件
#include <linux/errno.h>    //错误相关头文件
//only used for LED0 PI
#define LED_ON  1
#define LED_OFF 0

struct led_dev_t
{
    struct file_operations fop;
    struct cdev cdev;
    struct class *class;
    struct device *device;
    int major;
    int minor;
    dev_t deviceID;
    struct device_node *np;
};

extern void led_init(struct led_dev_t *led_dev);
extern void led_deinit(void);
extern void led_on(void);
extern void led_off(void);
extern void led_trigger(void);
extern u8 get_led_status(void);

#endif


