# 1. 使用不同的开发板内核时, 一定要修改 KERN_DIR
# 2. KERN_DIR 中的内核要事先配置、编译, 为了能编译内核, 要先设置下列环境变量:
# 2.1 ARCH, 比如: export ARCH=arm64
# 2.2 CROSS_COMPILE, 比如: export CROSS_COMPILE=aarch64-linux-gnu06 # 2.3 PATH, 比如: export PATH=$PATH:/home/book/100ask_roc-rk3399-pc/ToolChain-6.3.1/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu/bin
# 注意: 不同的开发板不同的编译器上述 3 个环境变量不一定相同,
# 请参考各开发板的高级用户使用手册

KERNELDIR := /home/wt/Linux/Linux_drivers/linux-5.4.31
CURRENT_PATH := $(shell pwd)
ROOTFS := /home/wt/Linux/nfs/rootfs
NTFS := /home/wt/Linux/nfs/
TFTP := /home/wt/Linux/tftp/
obj-m += leddrv.o
#leddrv.ko 有testdrv.o和led.o生成
leddrv-objs = testdrv.o # led.o
build: kernel_modules

kernel_modules:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) modules
copy:
	sudo rm -rf  $(ROOTFS)/lib/modules/5.4.31/*.ko
	sudo cp *.ko $(ROOTFS)/lib/modules/5.4.31
	cp $(CURRENT_PATH)/../linux-5.4.31/arch/arm/boot/dts/stm32mp157d-atk.dtb $(NTFS)
	cp $(CURRENT_PATH)/../linux-5.4.31/arch/arm/boot/dts/stm32mp157d-atk.dtb $(TFTP)
	arm-none-linux-gnueabihf-gcc -o testapp testapp.c
	sudo cp testapp $(ROOTFS)/lib/modules/5.4.31
copy_dts:
	cp *.dts $(CURRENT_PATH)/../linux-5.4.31/arch/arm/boot/dts
clean:
	rm -rf *.o *.ko .*.o.d .*.cmd *.mod.* *.mod modules.order Module.symvers .tmp_versions *app
