# linux-3.10
for sate210

make ARCH=arm sate210_defconfig
make ARCH=arm CROSS_COMPILE=arm-buildroot-linux-gnueabihf- menuconfig
make ARCH=arm CROSS_COMPILE=arm-buildroot-linux-gnueabihf- zImage
make ARCH=arm CROSS_COMPILE=arm-buildroot-linux-gnueabihf- uImage
