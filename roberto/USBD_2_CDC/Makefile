OPENOCD_PATH		?= /opt/gnuarmeclipse/openocd/0.10.0-201510281129-dev
OPENOCD           	?= $(OPENOCD_PATH)/bin/openocd
OPENOCD_INTERFACE 	?= $(OPENOCD_PATH)/scripts/interface/stlink-v2.cfg


FORMAT = ihex

BUILDDIR = Build



OPENOCD_TARGET      ?= 	$(OPENOCD_PATH)/scripts/target/stm32f3x.cfg


TARGET 				= 	Charger
CPU 				= 	f3
PROCESSOR 			= 	-mthumb -mcpu=cortex-m4 -DTHUMB -DHSE_VALUE="((uint32_t)16000000)" -DSTM32F302xC -DARM_MATH_CM4 
#PROCESSOR 			+= -DEMPL -DUSE_DMP -DMPU9250 -DMPL_LOG_NDEBUG=1 -DEMPL_TARGET_STM32F3xx #-DREMOVE_LOGGING -DSTM32F303xC -DENABLE_EKF
LINKERFILE			=	Linkers/STM32F302CCTx_FLASH.ld


INCLUDES =  -IInc
INCLUDES += -IInc/$(CPU)
INCLUDES += -IInc/periph
INCLUDES += -IDrivers/CMSIS/Include


# Platform specific files
SRC 		= 	Drivers/CMSIS/Device/ST/STM32F3xx/Source/Templates/system_stm32f3xx.c

STARTUP  	= 	Drivers/CMSIS/Device/ST/STM32F3xx/Source/Templates/gcc/startup_stm32f302xc.S

INCLUDES 	+= -IDrivers/STM32F3xx_HAL_Driver/Inc
INCLUDES 	+= -IDrivers/STM32F3xx_HAL_Driver/Inc/Legacy
INCLUDES 	+= -IDrivers/CMSIS/Device/ST/STM32F3xx/Include

SRC 		+= Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal.c

PERIP 		+= adc adc_ex crc crc_ex cortex dma flash flash_ex gpio i2c i2c_ex pcd pcd_ex pwr pwr_ex rcc rcc_ex spi spi_ex tim tim_ex	
SRC 		+= $(foreach mod, $(PERIP), Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_$(mod).c)

SRC			+= Src/$(CPU)/stm32f3xx_hal_msp.c
SRC			+= Src/$(CPU)/stm32f3xx_it.c
	

# USB Middleware files
INCLUDES 	+= -IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc
SRC 		+= Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
SRC 		+= Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
SRC 		+= Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c

#INCLUDES 	+= -IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
#SRC 		+= Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c

# USB setup files and conf
SRC 		+= Src/periph/usbd_desc.c
SRC 		+= Src/periph/usbd_iad.c
SRC 		+= Src/periph/usbd_iad_if.c
SRC 		+= Src/periph/usbd_conf.c
SRC 		+= Src/periph/usb_device.c

#Error handler
SRC 		+= Src/periph/error.c


# User objects
CPPSRC 		+= Src/main.cpp
SRC			+= Src/$(CPU)/sys.c
#CPPSRC		+= Src/OLED.cpp

# Periph files
#SRC			+= Src/periph/dfu.c
#SRC			+= Src/periph/led.c
SRC 		+= Src/periph/gpio.c
#SRC 		+= Src/periph/i2c.c
#SRC 		+= Src/periph/spi.c
#SRC 		+= Src/periph/tim.c
#SRC 		+= Src/VCP.c
#SRC 		+= Src/periph/usart.c


#SRC 		+= Src/unit_tests.c

# Library hacks for ROS
CPPSRC 		+= Src/LibraryHacks.cpp

# Ros lib includes
INCLUDES 	+= -IDrivers/ros_lib
ROSLIB 		 = $(sort $(dir $(wildcard Drivers/ros_lib/*)))
INCLUDES 	+= $(foreach mod, $(ROSLIB), -I$(mod))
CPPSRC 		+= Drivers/ros_lib/duration.cpp
CPPSRC 		+= Drivers/ros_lib/time.cpp



CFLAGS 		+= $(PROCESSOR) -DUSE_STDPERIPH_DRIVER $(INCLUDES) -Os -g0 -Wall -fdiagnostics-color=auto

# flags only for C++ (arm-none-eabi-g++)
# CPPFLAGS = -fno-rtti -fno-exceptions
CPPFLAGS = -std=c++11  -Wno-pmf-conversions

# flags only for C (arm-none-eabi-gcc)
CONLYFLAGS 	= -Wno-pointer-sign -std=gnu11

#LDFLAGS 	=  $(PROCESSOR) --specs=nano.specs --specs=rdimon.specs -lc -lrdimon -u _printf_float
LDFLAGS 	=  $(PROCESSOR) --specs=nano.specs --specs=nosys.specs -lm -lgcc -lc -u _printf_float
LDFLAGS 	+= -T$(LINKERFILE)

ASFLAGS 	= $(PROCESSOR) -x assembler-with-cpp 



SHELL = sh
CC = arm-none-eabi-gcc
CPP = arm-none-eabi-g++
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SIZE = arm-none-eabi-size
NM = arm-none-eabi-nm
REMOVE = rm -f
COPY = cp


# Define Messages
# English
MSG_ERRORS_NONE = Errors: none
MSG_BEGIN = -------- begin --------
MSG_END = --------  end  --------
MSG_SIZE_BEFORE = Size before: 
MSG_SIZE_AFTER = Size after:
MSG_FLASH = Creating load file for Flash:
MSG_EXTENDED_LISTING = Creating Extended Listing:
MSG_SYMBOL_TABLE = Creating Symbol Table:
MSG_LINKING = Linking:
MSG_COMPILING = Compiling C:
MSG_COMPILING_ARM = "Compiling C (ARM-only):"
MSG_COMPILINGCPP = Compiling C++:
MSG_COMPILINGCPP_ARM = "Compiling C++ (ARM-only):"
MSG_ASSEMBLING = Assembling:
MSG_ASSEMBLING_ARM = "Assembling (ARM-only):"
MSG_CLEANING = Cleaning project:


ASRC = $(STARTUP)

# Define all object files.
COBJ      = $(SRC:%.c=$(BUILDDIR)/%.o) 
AOBJ      = $(ASRC:%.S=$(BUILDDIR)/%.o)
COBJARM   = $(SRCARM:%.c=$(BUILDDIR)/%.o)
AOBJARM   = $(ASRCARM:%.S=$(BUILDDIR)/%.o)
CPPOBJ    = $(CPPSRC:%.cpp=$(BUILDDIR)/%.o) 
CPPOBJARM = $(CPPSRCARM:%.cpp=$(BUILDDIR)/%.o)

# Define all listing files.
LST = $(ASRC:.S=.lst) $(ASRCARM:.S=.lst) $(SRC:.c=.lst) $(SRCARM:.c=.lst)
LST += $(CPPSRC:.cpp=.lst) $(CPPSRCARM:.cpp=.lst)

# Compiler flags to generate dependency files.
### GENDEPFLAGS = -Wp,-M,-MP,-MT,$(*F).o,-MF,.dep/$(@F).d
GENDEPFLAGS = -MD -MP -MF .dep/$(@F).d

#CFLAGS += $(GENDEPFLAGS)


# Default target.
all: begin gccversion sizebefore build sizeafter finished end

build: none-eabi hex lss sym bin dfu

none-eabi: $(BUILDDIR)/$(TARGET).none-eabi
hex: $(BUILDDIR)/$(TARGET).hex
lss: $(BUILDDIR)/$(TARGET).lss 
sym: $(BUILDDIR)/$(TARGET).sym
bin: $(BUILDDIR)/$(TARGET).bin
dfu: $(BUILDDIR)/$(TARGET).dfu

# Eye candy.
begin:
	@echo $(REV)
	@echo
	@mkdir -p $(BUILDDIR)
	@echo $(MSG_BEGIN)

finished:
	@echo $(MSG_ERRORS_NONE)

end:
	@echo $(MSG_END)
	@echo


# Display size of file.
HEXSIZE = $(SIZE) --target=$(FORMAT) $(BUILDDIR)/$(TARGET).hex
ELFSIZE = $(SIZE) -A $(BUILDDIR)/$(TARGET).none-eabi
sizebefore:
	@if [ -f $(BUILDDIR)/$(TARGET).none-eabi ]; then echo; echo $(MSG_SIZE_BEFORE); $(ELFSIZE); echo; fi

sizeafter:
	@if [ -f $(BUILDDIR)/$(TARGET).none-eabi ]; then echo; echo $(MSG_SIZE_AFTER); $(ELFSIZE); echo; fi


# Display compiler version information.
gccversion : 
	@$(CC) --version


# Create final output files (.hex, .eep) from ELF output file.
# TODO: handling the .eeprom-section should be redundant
%.hex: %.none-eabi
	@echo
	@echo $(MSG_FLASH) $@
	$(OBJCOPY) -O $(FORMAT) $< $@

%.bin: %.none-eabi
	$(OBJCOPY) $^ -O binary $@


# Create extended listing file from ELF output file.
# testing: option -C
%.lss: %.none-eabi
	@echo
	@echo $(MSG_EXTENDED_LISTING) $@
	$(OBJDUMP) -h -S -C $< > $@


# Create a symbol table from ELF output file.
%.sym: %.none-eabi
	@echo
	@echo $(MSG_SYMBOL_TABLE) $@
	$(NM) -n $< > $@

%.dfu: %.bin
	python2 Scripts/dfu-convert.py -b 0x8000000:$^ $@


# Link: create ELF output file from object files.
.SECONDARY : $(TARGET).none-eabi
.PRECIOUS : $(AOBJARM) $(AOBJ) $(COBJARM) $(COBJ) $(CPPOBJ) $(CPPOBJARM)
%.none-eabi:  $(AOBJARM) $(AOBJ) $(COBJARM) $(COBJ) $(CPPOBJ) $(CPPOBJARM)
	@echo
	@echo $(MSG_LINKING) $@
#	$(CC) $(THUMB) $(CFLAGS) $(CONLYFLAGS) $(AOBJARM) $(AOBJ) $(COBJARM) $(COBJ) $(CPPOBJ) $(CPPOBJARM) --output $@ $(LDFLAGS) $(LIBS)
	$(CPP) $(THUMB) $(CFLAGS) $(CPPFLAGS) $(AOBJARM) $(AOBJ) $(COBJARM) $(COBJ) $(CPPOBJ) $(CPPOBJARM) --output $@ $(LDFLAGS) $(LIBS)

# Compile: create object files from C source files. ARM/Thumb
$(COBJ) : $(BUILDDIR)/%.o : %.c
	@echo 
	@mkdir -p $(@D)
	@echo $(MSG_COMPILING) $<
	$(CC) -c $(THUMB) $(CFLAGS) $(CONLYFLAGS) $< -o $@ 

# Compile: create object files from C source files. ARM-only
$(COBJARM) : $(BUILDDIR)/%.o : %.c
	@echo
	@mkdir -p $(@D)
	@echo $(MSG_COMPILING_ARM) $<
	$(CC) -c $(CFLAGS) $(CONLYFLAGS) $< -o $@ 

# Compile: create object files from C++ source files. ARM/Thumb
$(CPPOBJ) : $(BUILDDIR)/%.o : %.cpp
	@echo
	@mkdir -p $(@D)
	@echo $(MSG_COMPILINGCPP) $<
	$(CPP) -c $(THUMB) $(CFLAGS) $(CPPFLAGS) $< -o $@ 

# Compile: create object files from C++ source files. ARM-only
$(CPPOBJARM) : $(BUILDDIR)/%.o : %.cpp
	@echo
	@mkdir -p $(@D)
	@echo $(MSG_COMPILINGCPP_ARM) $<
	$(CPP) -c $(CFLAGS) $(CPPFLAGS) $< -o $@ 


# Compile: create assembler files from C source files. ARM/Thumb
## does not work - TODO - hints welcome
##$(COBJ) : %.s : %.c
##	$(CC) $(THUMB) -S $(CFLAGS) $< -o $@


# Assemble: create object files from assembler source files. ARM/Thumb
$(AOBJ) : $(BUILDDIR)/%.o : %.S
	@echo
	@mkdir -p $(@D)
	@echo $(MSG_ASSEMBLING) $<
	$(CPP) -c $(THUMB) $(ASFLAGS) $< -o $@


# Assemble: create object files from assembler source files. ARM-only
$(AOBJARM) : $(BUILDDIR)/%.o : %.S
	@echo
	@mkdir -p $(@D)
	@echo $(MSG_ASSEMBLING_ARM) $<
	$(CPP) -c $(ASFLAGS) $< -o $@

flash: all
	#python2 Scripts/jump-dfu.py
	dfu-util -d 0483:df11 -a 0 -D $(BUILDDIR)/$(TARGET).dfu -s :leave

flash-swd: all
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) -c init -c targets -c "reset halt" \
	           -c "flash write_image erase "$(BUILDDIR)/$(TARGET)".hex" -c "verify_image "$(BUILDDIR)/$(TARGET)".hex" -c "reset run" -c shutdown

openocd:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) 
	#-c init -c targets

# Target: clean project.
clean: begin clean_list finished end


clean_list :
	@echo
	@echo $(MSG_CLEANING)
	$(REMOVE) -r $(BUILDDIR)
	$(REMOVE) $(LST)
	$(REMOVE) $(SRC:.c=.s)
	$(REMOVE) $(SRC:.c=.d)
	$(REMOVE) $(SRCARM:.c=.s)
	$(REMOVE) $(SRCARM:.c=.d)
	$(REMOVE) $(CPPSRC:.cpp=.s) 
	$(REMOVE) $(CPPSRC:.cpp=.d)
	$(REMOVE) $(CPPSRCARM:.cpp=.s) 
	$(REMOVE) $(CPPSRCARM:.cpp=.d)
	$(REMOVE) .dep/*


# Include the dependency files.
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)


# Listing of phony targets.
.PHONY : all begin finish end sizebefore sizeafter gccversion \
build none-eabi hex lss sym bin dfu flash-tag flash-anchor clean clean_list program \
tag anchor
