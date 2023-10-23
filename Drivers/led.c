#include "led.h"
#include <asm/io.h> //定义了ioremap相关
#include <linux/of.h>
#include <linux/of_address.h>

/* 映射后的寄存器虚拟地址指针 */
static void __iomem *MPU_AHB4_PERIPH_RCC_PI;
static void __iomem *GPIOI_MODER_PI;
static void __iomem *GPIOI_OTYPER_PI;
static void __iomem *GPIOI_OSPEEDR_PI;
static void __iomem *GPIOI_PUPDR_PI;
static void __iomem *GPIOI_BSRR_PI;

static u8 led_status = 0;

static void led_unmap(void)
{
    /* 取消映射 */
    iounmap(MPU_AHB4_PERIPH_RCC_PI);
    iounmap(GPIOI_MODER_PI);
    iounmap(GPIOI_BSRR_PI);
}

void led_init(struct led_dev_t *led_dev)
{
    u32 val = 0;
    u32 regdata[3];
    struct property *proper;
    int retvalue;
    const char *str;
//查找led_test设备结点
    led_dev->np = of_find_node_by_path("/led_test");
	if (led_dev->np) {
        printk("led_test node find!\r\n");
	}
    else
    {
        printk("led_test node not find!\r\n");
    }
    proper = of_find_property(led_dev->np, "compatible", NULL);

    if(proper == NULL) {
        printk("compatible property find failed\r\n");
    } else {
        printk("compatible = %s\r\n", (char*)proper->value);
    }

    /* 3、获取 status 属性内容 */
    retvalue = of_property_read_string(led_dev->np, "status", &str);
    if(retvalue < 0){
        printk("status read failed!\r\n");
    }
    else {
        printk("status = %s\r\n",str);
    }

    retvalue = of_property_read_u32_array(led_dev->np, "reg", regdata, 3);
    if(retvalue < 0) {
        printk("reg property read failed!\r\n");
    }
    else {
        u8 i = 0;
        printk("reg data:\r\n");
        for(i = 0; i < 3; i++)
            printk("%#X ", regdata[i]);
        printk("\r\n");
    }
//解析reg属性并地址映射
    MPU_AHB4_PERIPH_RCC_PI = of_iomap(led_dev->np,0);
    GPIOI_MODER_PI = of_iomap(led_dev->np, 1);
    GPIOI_OTYPER_PI = GPIOI_MODER_PI + 0x04;
    GPIOI_OSPEEDR_PI = GPIOI_MODER_PI + 0x08;
    GPIOI_PUPDR_PI = GPIOI_MODER_PI + 0x0C;
    GPIOI_BSRR_PI = of_iomap(led_dev->np, 2);

    /* 初始化 LED */
    /* 2、使能 PI 时钟 */
    val = readl(MPU_AHB4_PERIPH_RCC_PI);
    val &= ~(0X1 << 8); /* 清除以前的设置 */
    val |= (0X1 << 8); /* 设置新值 */
    writel(val, MPU_AHB4_PERIPH_RCC_PI);

    /* 3、设置 PI0 通用的输出模式。*/
    val = readl(GPIOI_MODER_PI);
    val &= ~(0X3 << 0); /* bit0:1 清零 */
    val |= (0X1 << 0); /* bit0:1 设置 01 */
    writel(val, GPIOI_MODER_PI);

    /* 4、设置 PI0 为推挽模式。*/
    val = readl(GPIOI_OTYPER_PI);
    val &= ~(0X1 << 0); /* bit0 清零，设置为上拉*/
    writel(val, GPIOI_OTYPER_PI);
    val = readl(GPIOI_OSPEEDR_PI);
    val &= ~(0X3 << 0); /* bit0:1 清零 */
    val |= (0x2 << 0); /* bit0:1 设置为 10 */
    writel(val, GPIOI_OSPEEDR_PI);

    /* 5、设置 PI0 为上拉。*/
    val = readl(GPIOI_PUPDR_PI);
    val &= ~(0X3 << 0); /* bit0:1 清零 */
    val |= (0x1 << 0); /*bit0:1 设置为 01 */
    writel(val,GPIOI_PUPDR_PI);

    /* 6、关闭LED。*/
    led_off();
}
void led_deinit()
{
    /* 取消映射 */
    led_unmap();
}
void led_on()
{
    u32 val;

    val = readl(GPIOI_BSRR_PI);
    val |= (1 << 16); 
    writel(val, GPIOI_BSRR_PI);
    led_status = LED_ON;
}
void led_off()
{
    u32 val;

    val = readl(GPIOI_BSRR_PI);
    val|= (1 << 0);
    writel(val, GPIOI_BSRR_PI);
    led_status = LED_OFF;
}
void led_trigger()
{
    if(led_status == LED_ON)
    {
        led_off();
    }
    else
    {
        led_on();
    }
}
u8 get_led_status()
{
    return led_status;
}