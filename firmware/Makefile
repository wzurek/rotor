# output
TARGET = target
OBJDIR = $(TARGET)/libs

APP_NAME = rotor_firmware

# name of the port where Arduino Due is connected
USB_PORT=cu.usbmodemfd121

# sources
MAIN_CPP_SRCS = $(wildcard *.cpp)
MAIN_C_SRCS = $(wildcard *.c)
MAIN_OBJ_FILES = $(MAIN_C_SRCS:.c=.o) $(MAIN_CPP_SRCS:.cpp=.o)
MAIN_OBJS = $(patsubst %,$(OBJDIR)/%,$(MAIN_OBJ_FILES))

# adruino home
arduino = /Users/wzurek/Dev/ArdDue/hardware

# executables and options
GCC        = $(arduino)/tools/g++_arm_none_eabi/bin/arm-none-eabi-gcc
GCCOPTIONS = -c -g -Os -w -ffunction-sections -fdata-sections -nostdlib --param max-inline-insns-single=500                           -Dprintf=iprintf -mcpu=cortex-m3 -DF_CPU=84000000L -DARDUINO=152 -D__SAM3X8E__ -mthumb -DUSB_PID=0x003e -DUSB_VID=0x2341 -DUSBCON

GXX        = $(arduino)/tools/g++_arm_none_eabi/bin/arm-none-eabi-g++
GXXOPTIONS = -c -g -Os -w -ffunction-sections -fdata-sections -nostdlib --param max-inline-insns-single=500 -fno-rtti -fno-exceptions -Dprintf=iprintf -mcpu=cortex-m3 -DF_CPU=84000000L -DARDUINO=152 -D__SAM3X8E__ -mthumb -DUSB_PID=0x003e -DUSB_VID=0x2341 -DUSBCON

AR      = $(arduino)/tools/g++_arm_none_eabi/bin/arm-none-eabi-ar
OBJCOPY = $(arduino)/tools/g++_arm_none_eabi/bin/arm-none-eabi-objcopy

# Arduino core sources
ARDUINO_CORE_PATH = $(arduino)/arduino/sam/cores/arduino
ARDUINO_VARIAT_PATH = $(arduino)/arduino/sam/variants/arduino_due_x

CORE_C_SRCS = $(wildcard $(ARDUINO_CORE_PATH)/*.c)
CORE_CPP_SRCS = $(wildcard $(ARDUINO_CORE_PATH)/*.cpp)
CORE_USB_CPP_SRCS = $(wildcard $(ARDUINO_CORE_PATH)/USB/*.cpp)
CORE_VARIANT_SRCS = $(wildcard $(ARDUINO_VARIAT_PATH)/*.cpp)

CORE_INCLUDE = -I$(arduino)/arduino/sam/system/libsam \
			   -I$(arduino)/arduino/sam/system/CMSIS/CMSIS/Include/ \
			   -I$(arduino)/arduino/sam/system/CMSIS/Device/ATMEL/ \
			   -I$(arduino)/arduino/sam/variants/arduino_due_x \
			   -I$(arduino)/tools/g++_arm_none_eabi/lib/gcc/arm-none-eabi/4.4.1/include \
			   -I$(ARDUINO_CORE_PATH) \
			   -I$(ARDUINO_CORE_PATH)/USB

CORE_OBJ_FILES = $(CORE_C_SRCS:.c=.o) $(CORE_CPP_SRCS:.cpp=.o)
CORE_USB_OBJ_FILES = $(CORE_USB_CPP_SRCS:.cpp=.o)
CORE_VARIANT_OBJ_FILES = $(CORE_VARIANT_SRCS:.cpp=.o)
CORE_OBJS = $(patsubst $(ARDUINO_VARIAT_PATH)/%,  $(OBJDIR)/%, $(CORE_VARIANT_OBJ_FILES)) \
			$(patsubst $(ARDUINO_CORE_PATH)/%,    $(OBJDIR)/%, $(CORE_OBJ_FILES)) \
			$(patsubst $(ARDUINO_CORE_PATH)/USB/%,$(OBJDIR)/%, $(CORE_USB_OBJ_FILES))

# Wire library
Wire_LIB_PATH = $(arduino)/arduino/sam/libraries/Wire
Wire_LIB_SRCS = $(wildcard $(Wire_LIB_PATH)/*.cpp)
Wire_LIB_OBJ_FILES = $(Wire_LIB_SRCS:.cpp=.o)
Wire_LIB_OBJS = $(patsubst $(Wire_LIB_PATH)/%,  $(OBJDIR)/%, $(Wire_LIB_OBJ_FILES)) 

Wire_LIB_INCLUDE = -I$(Wire_LIB_PATH)

# compile Wire library
$(OBJDIR)/%.o:$(Wire_LIB_PATH)/%.cpp
	$(GXX) $(GXXOPTIONS) $(CORE_INCLUDE) $(Wire_LIB_INCLUDE) $< -o $@

# L3G Library
L3G_LIB_PATH = L3G
L3G_LIB_SRCS = $(wildcard $(L3G_LIB_PATH)/*.cpp)
L3G_LIB_OBJ_FILES = $(L3G_LIB_SRCS:.cpp=.o)
L3G_LIB_OBJS = $(patsubst $(L3G_LIB_PATH)/%,  $(OBJDIR)/%, $(L3G_LIB_OBJ_FILES))

L3G_LIB_INCLUDE = -I$(L3G_LIB_PATH)

# compile L3G library
$(OBJDIR)/%.o:$(L3G_LIB_PATH)/%.cpp
	$(GXX) $(GXXOPTIONS) $(CORE_INCLUDE) $(Wire_LIB_INCLUDE) $(Servo_LIB_INCLUDE) $(L3G_LIB_INCLUDE) $< -o $@
	
# LSM303 Library
LSM303_LIB_PATH = LSM303
LSM303_LIB_SRCS = $(wildcard $(LSM303_LIB_PATH)/*.cpp)
LSM303_LIB_OBJ_FILES = $(LSM303_LIB_SRCS:.cpp=.o)
LSM303_LIB_OBJS = $(patsubst $(LSM303_LIB_PATH)/%,  $(OBJDIR)/%, $(LSM303_LIB_OBJ_FILES))

LSM303_LIB_INCLUDE = -I$(LSM303_LIB_PATH)

# compile LSM303 library
$(OBJDIR)/%.o:$(LSM303_LIB_PATH)/%.cpp
	$(GXX) $(GXXOPTIONS) $(CORE_INCLUDE) $(Wire_LIB_INCLUDE) $(Servo_LIB_INCLUDE) $(LSM303_LIB_INCLUDE) $< -o $@


# Servo library
Servo_LIB_PATH = $(arduino)/arduino/sam/libraries/Servo
Servo_LIB_SRCS = $(wildcard $(Servo_LIB_PATH)/*.cpp)
Servo_LIB_OBJ_FILES = $(Servo_LIB_SRCS:.cpp=.o)
Servo_LIB_OBJS = $(patsubst $(Servo_LIB_PATH)/%,  $(OBJDIR)/%, $(Servo_LIB_OBJ_FILES)) 

Servo_LIB_INCLUDE = -I$(Servo_LIB_PATH)

# compile Servo library
$(OBJDIR)/%.o:$(Servo_LIB_PATH)/%.cpp
	$(GXX) $(GXXOPTIONS) $(CORE_INCLUDE) $(Servo_LIB_INCLUDE) $< -o $@



# package all libs into an archive	
$(TARGET)/libs.a: $(LSM303_LIB_OBJS) $(L3G_LIB_OBJS) $(SD_LIB_OBJS) $(Wire_LIB_OBJS) $(Servo_LIB_OBJS)
	$(AR) rcs $@ $?

# all - wil build $(APP_NAME).bin
all: $(APP_NAME).bin

# create target folders
$(TARGET):
	mkdir -p $(TARGET)
	
# create target/lib folder
$(OBJDIR): $(TARGET)
	mkdir -p $(OBJDIR)

# clean - remove target
clean:
	rm -rf $(TARGET)

# compile arduino core .c files
$(OBJDIR)/%.o:$(ARDUINO_CORE_PATH)/%.c
	$(GCC) $(GCCOPTIONS) $(CORE_INCLUDE) $< -o $@

# compile arduino core .cpp files
$(OBJDIR)/%.o:$(ARDUINO_CORE_PATH)/%.cpp
	$(GXX) $(GXXOPTIONS) $(CORE_INCLUDE) $< -o $@
	
# compile arduino core usb .cpp files
$(OBJDIR)/%.o:$(ARDUINO_CORE_PATH)/USB/%.cpp
	$(GXX) $(GXXOPTIONS) $(CORE_INCLUDE) $< -o $@
	
# compile arduino variant .cpp files
$(OBJDIR)/%.o:$(ARDUINO_VARIAT_PATH)/%.cpp
	$(GXX) $(GXXOPTIONS) $(CORE_INCLUDE) $< -o $@

# builds the archive with all arduino core	
$(TARGET)/core.a: $(CORE_OBJS)
	$(AR) rcs $@ $?

# builds the application code
$(OBJDIR)/%.o:%.cpp
	$(GCC) $(GCCOPTIONS) $(CORE_INCLUDE) $(SD_LIB_INCLUDE) $(LSM303_LIB_INCLUDE) $(L3G_LIB_INCLUDE) $(Wire_LIB_INCLUDE) $(Servo_LIB_INCLUDE) $< -o $@

# builds elf from compiled sources
$(APP_NAME).elf: $(OBJDIR) $(TARGET)/core.a  $(TARGET)/libs.a $(MAIN_OBJS)
	$(GXX) -Os -Wl,--gc-sections -mcpu=cortex-m3 \
		-T$(arduino)/arduino/sam/variants/arduino_due_x/linker_scripts/gcc/flash.ld \
		-Wl,-Map,$(TARGET)/$(APP_NAME).map -o $(TARGET)/$(APP_NAME).elf -L$(TARGET) -lm -lgcc -mthumb \
		-Wl,--cref \
		-Wl,--check-sections \
		-Wl,--gc-sections \
		-Wl,--entry=Reset_Handler \
		-Wl,--unresolved-symbols=report-all \
		-Wl,--warn-common \
		-Wl,--warn-section-align \
		-Wl,--warn-unresolved-symbols \
		-Wl,--start-group \
			$(OBJDIR)/syscalls_sam3.o \
			$(MAIN_OBJS) \
			$(TARGET)/core.a \
			$(TARGET)/libs.a \
			$(arduino)/arduino/sam/variants/arduino_due_x/libsam_sam3x8e_gcc_rel.a \
		-Wl,--end-group 

# build .bin from .elf
$(APP_NAME).bin: $(APP_NAME).elf
	$(OBJCOPY) -O binary $(TARGET)/$(APP_NAME).elf $(TARGET)/$(APP_NAME).bin

# uplod .bin into the Arduino
upload: $(APP_NAME).bin
	# reset the port to 1200 boud (due can be programmed only at this speed)
	stty -f /dev/$(USB_PORT) 1200
	# upload the bin
	$(arduino)/tools/bossac --port=$(USB_PORT) -U false -e -w -v -b $(TARGET)/$(APP_NAME).bin -R 
