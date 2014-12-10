KERNEL_SOURCE = martink

-include $(KERNEL_SOURCE)/.config

MAKEFLAGS += -I$(KERNEL_SOURCE)/

CFLAGS := -Wstrict-prototypes -Wno-pointer-to-int-cast 
CXXFLAGS := 
LD := g++ 
COMMON_FLAGS :=  -MD -ffunction-sections -Wall -Wno-int-to-pointer-cast -fdata-sections -Os -Wl,--relax,--gc-sections -fpermissive -fno-rtti -fno-exceptions

srctree := $(CURDIR)/$(KERNEL_SOURCE)/
include $(KERNEL_SOURCE)/arch/Makefile
srctree := $(CURDIR)

obj-y := main.o FlightController.o PID.o

BUILD_DIR := build

INCLUDES := -I. -I$(KERNEL_SOURCE) -I./$(KERNEL_SOURCE)/include/
APPDEPS := 

ifeq ($(CONFIG_SIMULATOR), y)
	INCLUDES += -Isimulator
	LDFLAGS += simulator/libquadcopter.a simulator/irrlicht/source/Irrlicht/libirrlicht.a -lBulletSoftBody -lBulletDynamics -lBulletCollision -lLinearMath -lm -lGL -lXxf86vm -lXext -lX11 -lpthread -ljpeg -lbz2
	APPDEPS += simulator/simulator
	APPNAME := simulator/simulator
else 
	INCLUDES += -Iinclude -Iinclude/c++ 
	APPDEPS += kernel
	APPNAME := firmware
endif

CXXFLAGS += $(CPU_FLAGS) $(COMMON_FLAGS) $(INCLUDES) -fno-rtti -fno-exceptions -std=c++11 
CFLAGS += $(CPU_FLAGS) $(COMMON_FLAGS) $(INCLUDES) -std=c99 
LDFLAGS := $(CPU_FLAGS) $(COMMON_FLAGS) $(LDFLAGS) 

obj-y := $(patsubst %, $(BUILD_DIR)/%, $(obj-y))

firmware: check $(obj-y) $(APPDEPS)
	#make -C $(KERNEL_SOURCE) build
	$(LDXX) -flto -static -nostartfiles -o $(APPNAME) $(LDFLAGS) -Wl,--start-group \
	-lc -lm -lgcc \
	$(KERNEL_SOURCE)/built-in.o $(obj-y) \
-Wl,--end-group 

kernel: 
	make -C $(KERNEL_SOURCE) build
	
simulator/simulator: $(obj-y) 
	make -C simulator
	make -C $(KERNEL_SOURCE) build
	$(LDXX) -o $(APPNAME)  $(obj-y) $(KERNEL_SOURCE)/built-in.o $(LDFLAGS) 

install_due: 
	#arm-none-eabi-strip $(APPNAME)
	arm-none-eabi-objcopy -O binary $(APPNAME) $(APPNAME).bin
	stty -F /dev/ttyACM0 raw ispeed 1200 ospeed 1200 cs8 -cstopb ignpar eol 255 eof 255
	bossac -U false -e -w -v -b $(APPNAME).bin -R
	
#simulator/libquadcopter.a: 
#	make -C simulator
	
flash: $(APPNAME)
	avr-objcopy -j .text -j .data -O ihex $(APPNAME) $(APPNAME).hex 
	avr-size -C -x $(APPNAME) 
	sudo avrdude -p m328p -c usbasp -e -U flash:w:$(APPNAME).hex
	#sudo avrdude -p m328p -b 115200 -c avrisp -P /dev/ttyUSB3 -e -U flash:w:$(APPNAME).hex
	#sudo avrdude -p ${CPU_AVRDUDE} -b 57600 -c arduino -P /dev/ttyUSB3 -e -U flash:w:${TARGET}.hex
	
clean:
	rm -rf build/
	#make -C simulator clean
	make -C $(KERNEL_SOURCE) APP=$(PWD) clean
	
menuconfig: 
	cp KConfig.app $(KERNEL_SOURCE)/
	make -C $(KERNEL_SOURCE) menuconfig
	
sim: $(obj-y) simulator/libquadcopter.a
	make -C simulator
	$(LD) $(LDFLAGS) -o simulator/quadcopter $(obj-y) 

#@which $(CC) > /dev/null

$(BUILD_DIR)/%.o: %.cpp martink/.config
	mkdir -p `dirname $@`
	$(CXX) -c $(CXXFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.c martink/.config
	mkdir -p `dirname $@`
	$(CC) -c $(CFLAGS) $< -o $@


check: 
ifeq (, $(shell which $(CC) 2>/dev/null))
	$(error "$(CC) not found! You may need to install it!")
endif
