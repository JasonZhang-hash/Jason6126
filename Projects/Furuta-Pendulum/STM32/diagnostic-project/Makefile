CC      = F:/Tools/ArmGNU-14.3.rel1/bin/arm-none-eabi-gcc
OBJCOPY = F:/Tools/ArmGNU-14.3.rel1/bin/arm-none-eabi-objcopy
CFLAGS  = -mcpu=cortex-m3 -mthumb -O0 -g -Wall -nostdlib -mfloat-abi=soft
LDLIBS  = -lgcc

all: furuta_diag.hex

furuta_diag.elf: main.c startup_stm32f103.s stm32f103c8.ld
	$(CC) $(CFLAGS) -T stm32f103c8.ld -o $@ main.c startup_stm32f103.s $(LDLIBS)

furuta_diag.hex: furuta_diag.elf
	$(OBJCOPY) -O ihex $< $@

flash: furuta_diag.hex
	F:/Tools/OpenOCD-0.12.0-7/xpack-openocd-0.12.0-7/bin/openocd.exe \
		-f interface/stlink.cfg \
		-f target/stm32f1x.cfg \
		-c "program furuta_diag.hex verify reset exit"

clean:
	rm -f furuta_diag.elf furuta_diag.hex
