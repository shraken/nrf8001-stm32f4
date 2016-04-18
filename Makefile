TARGET=main
EXECUTABLE=main.elf
BINARYOUT=main.bin
STLINK=~/Downloads/stlink
LIBPATH=/home/shraken/Downloads/STM32F4-workarea

CC=arm-none-eabi-gcc
LD=arm-none-eabi-gcc
AR=arm-none-eabi-ar
AS=arm-none-eabi-as
CP=arm-none-eabi-objcopy
OD=arm-none-eabi-objdump

BIN=$(CP) -O ihex 

DEFS = -DUSE_STDPERIPH_DRIVER -DSTM32F4XX -DHSE_VALUE=8000000
STARTUP = $(LIBPATH)/Libraries/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc_ride7/startup_stm32f40xx.s

MCU = cortex-m4
MCFLAGS = -mcpu=$(MCU) -mthumb -mlittle-endian -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb-interwork -std=c99
STM32_INCLUDES = -I$(LIBPATH)/Libraries/CMSIS/Device/ST/STM32F4xx/Include/ \
	-I$(LIBPATH)/Libraries/CMSIS/Include/ \
	-I$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/inc/ \
	-I./inc/ \
	-I./inc/BLE/

OPTIMIZE       = -Os

CFLAGS	= $(MCFLAGS)  $(OPTIMIZE)  $(DEFS) -I. -I./ $(STM32_INCLUDES)  -Wl,-T,stm32_flash.ld
AFLAGS	= $(MCFLAGS) 
#-mapcs-float use float regs. small increase in code size

SRC = ./src/main.c \
	./src/millis.c \
	./src/usart.c \
	./src/spi.c \
	./src/debug.c \
	./src/stm32f4xx_it.c \
	./src/system_stm32f4xx.c \
	./src/BLE/io_support.c \
	./src/BLE/acilib.c \
	./src/BLE/aci_queue.c \
	./src/BLE/aci_setup.c \
	./src/BLE/hal_aci_tl.c \
	./src/BLE/lib_aci.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/misc.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_adc.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_can.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_crc.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cryp_aes.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cryp.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cryp_des.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cryp_tdes.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dac.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dbgmcu.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dcmi.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dma.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_exti.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_flash.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_fsmc.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_gpio.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_hash.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_hash_md5.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_hash_sha1.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_i2c.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_iwdg.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_pwr.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rcc.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rng.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rtc.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_sdio.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_spi.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_syscfg.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_tim.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_usart.c \
	$(LIBPATH)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_wwdg.c 

OBJDIR = .
OBJ = $(SRC:%.c=$(OBJDIR)/%.o) 
OBJ += Startup.o

all: $(TARGET)

$(TARGET): $(EXECUTABLE)
	$(CP) -O ihex $^ $@
	$(CP) -O binary $(EXECUTABLE) $(BINARYOUT)

$(EXECUTABLE): $(SRC) $(STARTUP)
	$(CC) $(CFLAGS) $^ -lm -lc -lnosys -o $@

burn: 
	$(STLINK)/st-flash write $(BINARYOUT) 0x8000000

clean:
	rm -f Startup.lst  $(TARGET)  $(TARGET).lst $(AUTOGEN)  $(TARGET).out  $(TARGET).hex  $(TARGET).map \
	 $(TARGET).dmp  $(TARGET).elf  $(TARGET).bin
