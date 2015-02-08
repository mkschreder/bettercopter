KERNEL_SOURCE = martink
ARCH = $(firstword $(subst -, ,$(BUILD)))
CPU = $(word 2,$(subst -, ,$(BUILD)))
CONFIG = $(KERNEL_SOURCE)/configs/$(BUILD).config

-include $(CONFIG)

#ifdef BUILD
#	include $(KERNEL_SOURCE)/configs/$(BUILD).config
#else
#	include $(KERNEL_SOURCE)/.config
#endif

-include Makefile.config

MAKEFLAGS += -I$(KERNEL_SOURCE)/

CFLAGS := -Wstrict-prototypes -Wno-pointer-to-int-cast 
CXXFLAGS := 
LD := g++ 
COMMON_FLAGS := -DBUILD_$(subst -,_,$(BUILD)) -MD -ffunction-sections -Wall -Wno-int-to-pointer-cast -fdata-sections -Os -Wl,--gc-sections -fpermissive 

srctree := $(CURDIR)/$(KERNEL_SOURCE)/
-include $(KERNEL_SOURCE)/arch/Makefile
srctree := $(CURDIR)

obj-y := Application.o FlightController.o PID.o GPS.o LedIndicator.o ModeAltHold.o ModeStab.o PCLink.o ArmSwitch.o

BUILD_DIR := build

INCLUDES := -I. -I$(KERNEL_SOURCE) -I./$(KERNEL_SOURCE)/include/ 
APPDEPS := 

ifeq ($(CONFIG_SIMULATOR), y)
	INCLUDES += -Isimulator
	LDFLAGS += simulator/libquadcopter.a simulator/irrlicht/source/Irrlicht/libirrlicht.a -lBulletSoftBody -lBulletDynamics -lBulletCollision -lLinearMath -lm -lGL -lXxf86vm -lXext -lX11 -lpthread -ljpeg -lbz2
	APPDEPS += simulator/simulator
	APPNAME := simulator/simulator
else 
	INCLUDES += -Iinclude 
	APPDEPS += kernel
	APPNAME := firmware-$(BUILD)
endif
ifeq ($(CONFIG_AVR), y)
	INCLUDES += -Iinclude/c++ 
else
	INCLUDES += -Iinclude/c++
endif

CXXFLAGS += $(CPU_FLAGS) $(COMMON_FLAGS) $(INCLUDES) -std=c++11 -fno-exceptions -fno-rtti
CFLAGS += $(CPU_FLAGS) $(COMMON_FLAGS) $(INCLUDES) -std=c99 
#LDFLAGS := $(CPU_FLAGS) $(COMMON_FLAGS) $(LDFLAGS) -ffreestanding
LDFLAGS := -Wl,-Map=$(APPNAME).map $(CPU_FLAGS) $(COMMON_FLAGS) $(LDFLAGS)

obj-y := $(patsubst %, $(BUILD_DIR)/%, $(obj-y))

all: check kernel firmware;

firmware: check $(obj-y) $(APPDEPS) 
	#make -C $(KERNEL_SOURCE) build
	$(LDXX) -o $(APPNAME) $(LDFLAGS) $(obj-y) -Wl,--start-group $(KERNEL_SOURCE)/libk-$(ARCH)-$(CPU).a -lm -lc -lgcc -Wl,--end-group
	#avr-strip -s -R .comment -R .gnu.version --strip-unneeded $(APPNAME)
	
kernel_tree: 
	if [ -d $(KERNEL_SOURCE) ]; \
	then echo "Found kernel tree."; \
	else git clone https://github.com/mkschreder/martink.git martink; \
	echo "PLEASE MAKE SURE FOLLOWING ARE INSTALLED: libncurses5-dev avr-libc avr-gcc";\
	fi
	
kernel: 
	make -C $(KERNEL_SOURCE) BUILD=$(BUILD) build
	
simulator/simulator: $(obj-y) 
	make -C simulator
	make -C $(KERNEL_SOURCE) build
	$(LDXX) -o $(APPNAME)  $(obj-y) $(KERNEL_SOURCE)/built-in.o $(LDFLAGS) 
	
install: install-$(BUILD)

install-avr-atmega328p:
	avr-nm -a --demangle --print-size --size-sort -t d $(APPNAME) > $(APPNAME).info
	avr-objdump -d $(APPNAME) > $(APPNAME).asm
	avr-objcopy -j .text -j .data -O ihex $(APPNAME) $(APPNAME).hex 
	avr-size $(APPNAME) 
	sudo avrdude -p m328p -c usbasp -e -U lfuse:w:0xFF:m -U hfuse:w:0xD7:m -U efuse:w:0x05:m -U flash:w:$(APPNAME).hex
	#sudo avrdude -p m328p -b 115200 -c avrisp -P /dev/ttyUSB3 -e -U flash:w:$(APPNAME).hex
	#sudo avrdude -p ${CPU_AVRDUDE} -b 57600 -c arduino -P /dev/ttyUSB3 -e -U flash:w:${TARGET}.hex

install-arm-stm32f103: 
	make -C martink/scripts/stm32flash
	arm-none-eabi-size $(APPNAME)
	arm-none-eabi-objcopy -j .text -j .data $(APPNAME) $(APPNAME).bin -O binary
	arm-none-eabi-objdump -d $(APPNAME) > $(APPNAME).asm
	sudo martink/scripts/stm32flash/stm32flash -w $(APPNAME).bin -v -g 0x0 /dev/ttyUSB0

install-at91sam3: 
	#arm-none-eabi-strip $(APPNAME)
	arm-none-eabi-size $(APPNAME)
	arm-none-eabi-objcopy -j .text -j .data $(APPNAME) $(APPNAME).bin -O binary
	stty -F /dev/ttyACM0 raw ispeed 1200 ospeed 1200 cs8 -cstopb ignpar eol 255 eof 255
	bossac -U false -e -w -b $(APPNAME).bin -R

clean:
	rm -rf build/
	#make -C simulator clean
	make -C $(KERNEL_SOURCE) APP=$(PWD) clean
	
menuconfig: kernel_tree
	cp KConfig.app $(KERNEL_SOURCE)/
	make -C $(KERNEL_SOURCE) menuconfig
	
sim: $(obj-y) simulator/libquadcopter.a
	make -C simulator
	$(LD) $(LDFLAGS) -o simulator/quadcopter $(obj-y) 

#@which $(CC) > /dev/null

$(BUILD_DIR)/%.o: %.cpp martink/.config
	mkdir -p `dirname $@`
	$(CXX) -c -DBUILD=$(BUILD) $(CXXFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.c martink/.config
	mkdir -p `dirname $@`
	$(CC) -c -DBUILD=$(BUILD) $(CFLAGS) $< -o $@

-include $(obj-y:%.o=%.d)

check: 
ifeq (, $(shell which $(CC) 2>/dev/null))
	$(error "$(CC) not found! You may need to install it! (along with libc for target architecture!)")
endif
	if [ -e $(CONFIG) ]; then echo "Using $(CONFIG)"; else make -C $(KERNEL_SOURCE) BUILD=$(BUILD) menuconfig; fi; 
