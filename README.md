# linux-3.10
for sate210/tq210

make ARCH=arm sate210_defconfig
or
make ARCH=arm tq210_defconfig
make ARCH=arm CROSS_COMPILE=arm-buildroot-linux-gnueabihf- menuconfig
make ARCH=arm CROSS_COMPILE=arm-buildroot-linux-gnueabihf- zImage
make ARCH=arm CROSS_COMPILE=arm-buildroot-linux-gnueabihf- uImage
