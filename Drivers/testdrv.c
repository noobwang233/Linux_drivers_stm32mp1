#include <linux/types.h>   // 定义了ssize_t的头文件
#include <linux/kernel.h>
#include <linux/ide.h>
#include <linux/init.h>    // 模块加载init和卸载exit相关头文件
#include <linux/module.h>
#include "led.h"           //led的设备资源以及操控

#define TESTDRV_MAJOR 200     /* 驱动的主设备号 */
#define TESTDRV_NAME "leddrv" /* 驱动的主设备名称 */

static struct file_operations testdrv_fop;

static int testdrv_open(struct inode *inode, struct file *filp)
{
    printk("testdrv open!\r\n");
    return 0;
}

static int testdrv_release(struct inode *inode, struct file *filp)
{
    printk("testdrv close!\r\n");
    return 0;
}
//typedef int		__kernel_ssize_t;
//typedef __kernel_ssize_t	ssize_t;
static ssize_t testdrv_read(struct file *filp, char __user *buf, size_t cnt, loff_t *offt)
{
    int retvalue = 0;
    u8 status = 0;

    status = get_led_status();
    /* 向用户空间发送数据 */
    retvalue = copy_to_user(buf, &status, cnt);
    if(retvalue == 0)
    {
        printk("kernel senddata ok!\r\n");
    }
    else
    {
        printk("kernel senddata failed!\r\n");
    }

    printk("testdrv read!\r\n");
    return 0;
}

static ssize_t testdrv_write(struct file *filp, const char __user *buf, size_t cnt, loff_t *offt)
{
    int retvalue = 0;
    u8 cmd = 0;

    retvalue = copy_from_user(&cmd, buf, cnt);
    if(retvalue == 0)
    {
        printk("kernel recevdata:%d\r\n", cmd);
    }
    else
    {
        printk("kernel recevdata failed!\r\n");
    }

    switch (cmd)
    {
    case 0:
        led_off();
        printk("led off\n");
        break;
    case 1:
        led_on();
        printk("led on\n");
        break;
    default:
        led_trigger();
        printk("led trigger\n");
        break;
    }
    return 0;
}

static int __init testdrv_init(void)
{
    int retvalue = 0;

    /* 注册字符设备驱动 */
    led_init();
    printk("led init\n");
    retvalue = register_chrdev(TESTDRV_MAJOR, TESTDRV_NAME, &testdrv_fop);
    if(retvalue < 0)
    {
        printk("chrdevbase driver register failed\r\n");
        led_deinit();
        printk("led deinit\n");
    }
    printk("chrdevbase_init() success!\r\n");
    return 0;
}

static void __exit testdrv_exit(void)
{
    /* 注销字符设备驱动 */
    unregister_chrdev(TESTDRV_MAJOR, TESTDRV_NAME);
    led_deinit();
    printk("led deinit\n");
    printk("chrdevbase_exit() success!\r\n");
}

static struct file_operations testdrv_fop = {
    .owner = THIS_MODULE,
    .open = testdrv_open,
    .release = testdrv_release,
    .write = testdrv_write,
    .read = testdrv_read,
};

module_init(testdrv_init);
module_exit(testdrv_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("wt");
MODULE_INFO(intree, "Y");
