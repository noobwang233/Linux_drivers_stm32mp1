#include "led.h"
#include <asm/io.h> //定义了ioremap相关

/* 寄存器物理地址 */
#define PERIPH_BASE (0x40000000)
#define MPU_AHB4_PERIPH_BASE (PERIPH_BASE + 0x10000000)
#define RCC_BASE (MPU_AHB4_PERIPH_BASE + 0x0000)
#define RCC_MP_AHB4ENSETR (RCC_BASE + 0XA28)
#define GPIOI_BASE (MPU_AHB4_PERIPH_BASE + 0xA000)
#define GPIOI_MODER (GPIOI_BASE + 0x0000) 
#define GPIOI_OTYPER (GPIOI_BASE + 0x0004)
#define GPIOI_OSPEEDR (GPIOI_BASE + 0x0008) 
#define GPIOI_PUPDR (GPIOI_BASE + 0x000C) 
#define GPIOI_BSRR (GPIOI_BASE + 0x0018)

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
    iounmap(GPIOI_OTYPER_PI);
    iounmap(GPIOI_OSPEEDR_PI);
    iounmap(GPIOI_PUPDR_PI);
    iounmap(GPIOI_BSRR_PI);
}
static void led_map(void)
{
    MPU_AHB4_PERIPH_RCC_PI = ioremap(RCC_MP_AHB4ENSETR, 4);
    GPIOI_MODER_PI = ioremap(GPIOI_MODER, 4);
    GPIOI_OTYPER_PI = ioremap(GPIOI_OTYPER, 4);
    GPIOI_OSPEEDR_PI = ioremap(GPIOI_OTYPER, 4);
    GPIOI_PUPDR_PI = ioremap(GPIOI_PUPDR, 4);
    GPIOI_BSRR_PI = ioremap(GPIOI_BSRR, 4);
}

void led_init()
{
    u32 val = 0;

    /* 初始化 LED */
    /* 1、寄存器地址映射 */
    led_map();
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