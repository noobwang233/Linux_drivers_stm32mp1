#include <linux/types.h>   // 定义了ssize_t的头文件
#include <linux/kernel.h>
#include <linux/ide.h>
#include <linux/init.h>    // 模块加载init和卸载exit相关头文件
#include <linux/module.h>
#include <linux/cdev.h>         // cdev相关头文件
#include <linux/device.h>   //设备号dev_t相关头文件
#include <linux/errno.h>    //错误相关头文件
#include <linux/gpio.h>     //gpio子系统相关头文件
#include <linux/of_gpio.h>  //of_gpio函数相关头文件
#include <linux/of.h>       //of_函数相关头文件

#define TESTDRV_MAJOR 200     /* 驱动的主设备号 */
#define TESTDRV_NAME "leddrv" /* 驱动的主设备名称 */
#define DEVICE_CNT    1       /* 设备数量 */

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
    int gpio; /* led 所使用的 GPIO 编号 */
};
struct led_dev_t led_dev_0;
static struct file_operations testdrv_fop;
static int testdrv_open(struct inode *inode, struct file *filp)
{
    filp->private_data = &led_dev_0;
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

    status = gpio_get_value(led_dev_0.gpio);
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
    case 0:// GPIO_ACTIVE_LOW
        gpio_set_value(led_dev_0.gpio, 0);
        printk("led on\n");
        break;
    case 1:
        gpio_set_value(led_dev_0.gpio, 1);
        printk("led off\n");
        break;
    default:
        gpio_set_value(led_dev_0.gpio, (gpio_get_value(led_dev_0.gpio)==0? 1:0));
        printk("led trigger\n");
        break;
    }
    return 0;
}

static int __init testdrv_init(void)
{
    int retvalue = 0;

    /* 注册字符设备驱动 */
    /* 查找设备结点 */
    led_dev_0.np = of_find_compatible_node(NULL , NULL , "led_gpio");
    if (led_dev_0.np == NULL)
    {
        pr_err("find device node failed\n");
    }
    printk("find device node successfully! \n");
    /* 解析gpio属性 */
    led_dev_0.gpio = of_get_named_gpio(led_dev_0.np, "gpio", 0);
    if (led_dev_0.gpio == 0)
    {
        pr_err("get gpio failed\n");
    }
    printk("get gpio %d successfully! \n", led_dev_0.gpio);
    /* 申请gpio */
    retvalue = gpio_request(led_dev_0.gpio, "led_gpio");
    if (retvalue != 0)
    {
        pr_err("request gpio failed\n");
    }
    printk("request gpio %d successfully! \n", led_dev_0.gpio);
    /* 设置为输出且默认为1 GPIO_ACTIVE_LOW 关闭led*/
    retvalue = gpio_direction_output(led_dev_0.gpio, 1);
    if (retvalue != 0)
    {
        pr_err("request gpio output failed\n");
        goto freegpio;
    }
    printk("request gpio %d output successfully! \n", led_dev_0.gpio);
    /*申请设备号*/
    led_dev_0.major=TESTDRV_MAJOR;
    if(led_dev_0.major != 0)
    {
        led_dev_0.deviceID = MKDEV(led_dev_0.major, 0);
        retvalue = register_chrdev_region(led_dev_0.deviceID, DEVICE_CNT, TESTDRV_NAME);
        if(retvalue < 0) 
        {
            pr_err("cannot register %s char driver [retvalue=%d]\n",TESTDRV_NAME, DEVICE_CNT);
            goto freegpio;
        }
    }
    else
    {
        retvalue = alloc_chrdev_region(&led_dev_0.deviceID, 0, DEVICE_CNT, TESTDRV_NAME);
        if(retvalue < 0) 
        {
            pr_err("cannot register %s char driver [retvalue=%d]\n",TESTDRV_NAME, DEVICE_CNT);
            goto freegpio;
        }
        led_dev_0.major = MAJOR(led_dev_0.deviceID);
        led_dev_0.minor = MINOR(led_dev_0.deviceID);
    }
    printk("newcheled major=%d,minor=%d\r\n",led_dev_0.major, led_dev_0.minor);
    /*初始化cdev*/
    led_dev_0.cdev.owner = THIS_MODULE;
    cdev_init(&led_dev_0.cdev, &testdrv_fop);
    /*注册cdev*/
    retvalue = cdev_add(&led_dev_0.cdev, led_dev_0.deviceID, DEVICE_CNT);
    if(retvalue < 0)
    {
        pr_err("add cdev %s faild\n",TESTDRV_NAME);
        goto unregister;
    }
    /*创建类*/
    led_dev_0.class = class_create(THIS_MODULE, TESTDRV_NAME);
    if (IS_ERR(led_dev_0.class)) {
        pr_err("create class %s faild\n",TESTDRV_NAME);
        goto del_cdev;
    }
    /*创建设备*/
    led_dev_0.device = device_create(led_dev_0.class, NULL, led_dev_0.deviceID, NULL, TESTDRV_NAME);
    if (IS_ERR(led_dev_0.device)) {
        pr_err("create device %s faild\n",TESTDRV_NAME);
        goto destroy_class;
    }
    printk("chrdevbase_init() success!\r\n");
    return 0;
destroy_class:
    class_destroy(led_dev_0.class);
del_cdev:
    cdev_del(&led_dev_0.cdev);
unregister:
    unregister_chrdev_region(led_dev_0.deviceID, DEVICE_CNT);
freegpio:
    gpio_free(led_dev_0.gpio);
    return -EIO;
}

static void __exit testdrv_exit(void)
{
    /* 注销字符设备驱动 */
    device_destroy(led_dev_0.class, led_dev_0.deviceID);
    class_destroy(led_dev_0.class);
    cdev_del(&led_dev_0.cdev);
    unregister_chrdev_region(led_dev_0.deviceID, DEVICE_CNT);
    gpio_free(led_dev_0.gpio);
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
