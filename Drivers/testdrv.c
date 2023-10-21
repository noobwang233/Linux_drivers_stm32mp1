#include <linux/types.h>   // 定义了ssize_t的头文件
#include <linux/kernel.h>
#include <linux/ide.h>
#include <linux/init.h>    // 模块加载init和卸载exit相关头文件
#include <linux/module.h>

#define CHRDEVBASE_MAJOR 200 /* 主设备号 */
#define CHRDEVBASE_NAME "testdrv" /* 设备名 */

static char readbuf[100]; /* 读缓冲区 */
static char writebuf[100]; /* 写缓冲区 */
static char kerneldata[] = {"testdrv kernel data!"};
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

    /* 向用户空间发送数据 */
    memcpy(readbuf, kerneldata, sizeof(kerneldata));
    retvalue = copy_to_user(buf, readbuf, cnt);
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

    retvalue = copy_from_user(writebuf, buf, cnt);
    if(retvalue == 0)
    {
        printk("kernel recevdata:%s\r\n", writebuf);
    }
    else
    {
        printk("kernel recevdata failed!\r\n");
    }

    printk("testdrv write!\r\n");
    return 0;
}

static int __init testdrv_init(void)
{
    int retvalue = 0;

    /* 注册字符设备驱动 */
    retvalue = register_chrdev(CHRDEVBASE_MAJOR, CHRDEVBASE_NAME, &testdrv_fop);
    if(retvalue < 0)
    {
        printk("chrdevbase driver register failed\r\n");
    }
    printk("chrdevbase_init() success!\r\n");
    return 0;
}

static void __exit testdrv_exit(void)
{
    /* 注销字符设备驱动 */
    unregister_chrdev(CHRDEVBASE_MAJOR, CHRDEVBASE_NAME);
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
