#############################################################
#
# Root Level Makefile ESP8266 + RTOS
#
# (c) by CHERTS <sleuthhound@gmail.com>
#
# Modified by Martin Winkelhofer
# W-Dimension / wdim / wdim0 / maarty.w@gmail.com
# - generation of flash_slot1.bin / flash_slot2.bin files and related md5 files
# - new flashinit (clear entire flash, flash FOTA bootloader v1.5 and ESP init) 
# - baud 921600, COM3
# - for ESP module with 1 MB flash memory
# - wfofgen section added (specific for this project - generation of wfof/wfof_data.h (WLCD images data))
#   (run first before "make all" or do "make rebuild")
#
#############################################################

BUILD_BASE	= build
FW_BASE		= firmware

# Base directory for the compiler
XTENSA_TOOLS_ROOT ?= c:/Espressif/xtensa-lx106-elf/bin

# base directory of the ESP8266 SDK package, absolute
SDK_BASE	?= c:/Espressif/ESP8266_RTOS_SDK
SDK_TOOLS	?= c:/Espressif/utils

# esptool path and port
ESPTOOL ?= $(SDK_TOOLS)/esptool.exe
ESPPORT ?= COM3
# Baud rate for programmer
BAUD ?= 921600

# SPI_SPEED = 40, 26, 20, 80
SPI_SPEED ?= 40
# SPI_MODE: qio, qout, dio, dout
SPI_MODE ?= qio
# SPI_SIZE_MAP
# 0 : 512 KB (256 KB + 256 KB)
# 1 : 256 KB
# 2 : 1024 KB (512 KB + 512 KB)
# 3 : 2048 KB (512 KB + 512 KB)
# 4 : 4096 KB (512 KB + 512 KB)
# 5 : 2048 KB (1024 KB + 1024 KB)
# 6 : 4096 KB (1024 KB + 1024 KB)
SPI_SIZE_MAP ?= 2

ifeq ($(SPI_SPEED), 26.7)
    freqdiv = 1
    flashimageoptions = -ff 26m
else
    ifeq ($(SPI_SPEED), 20)
        freqdiv = 2
        flashimageoptions = -ff 20m
    else
        ifeq ($(SPI_SPEED), 80)
            freqdiv = 15
            flashimageoptions = -ff 80m
        else
            freqdiv = 0
            flashimageoptions = -ff 40m
        endif
    endif
endif

ifeq ($(SPI_MODE), QOUT)
    mode = 1
    flashimageoptions += -fm qout
else
    ifeq ($(SPI_MODE), DIO)
		mode = 2
		flashimageoptions += -fm dio
    else
        ifeq ($(SPI_MODE), DOUT)
            mode = 3
            flashimageoptions += -fm dout
        else
            mode = 0
            flashimageoptions += -fm qio
        endif
    endif
endif

ifeq ($(SPI_SIZE_MAP), 1)
  size_map = 1
  flash = 256
  flashimageoptions += -fs 2m
else
  ifeq ($(SPI_SIZE_MAP), 2)
    size_map = 2
    flash = 1024
    flashimageoptions += -fs 8m
  else
    ifeq ($(SPI_SIZE_MAP), 3)
      size_map = 3
      flash = 2048
      flashimageoptions += -fs 16m
    else
      ifeq ($(SPI_SIZE_MAP), 4)
		size_map = 4
		flash = 4096
		flashimageoptions += -fs 32m
      else
        ifeq ($(SPI_SIZE_MAP), 5)
          size_map = 5
          flash = 2048
          flashimageoptions += -fs 16m
        else
          ifeq ($(SPI_SIZE_MAP), 6)
            size_map = 6
            flash = 4096
            flashimageoptions += -fs 32m
          else
            size_map = 0
            flash = 512
            flashimageoptions += -fs 4m
          endif
        endif
      endif
    endif
  endif
endif

# name for the target project
TARGET		= app

# which modules (subdirectories) of the project to include in compiling
#MODULES 	= user
MODULES 	= driver wfof user
#EXTRA_INCDIR = include
EXTRA_INCDIR = include/driver include

#LIBS		= cirom gccirom hal phy pp net80211 wpa main freertos lwip udhcp
# linking libgccirom.a instead of libgcc.a causes reset when working with flash memory (ie spi_flash_erase_sector)
LIBS		= gcc hal phy pp net80211 wpa crypto main freertos lwip minic

# compiler flags using during compilation of source files
CFLAGS		= -g -save-temps -std=gnu90 -Os -Wpointer-arith -Wundef -Werror -Wl,-EL -fno-inline-functions -nostdlib -mlongcalls -mtext-section-literals -mno-serialize-volatile -D__ets__ -DICACHE_FLASH
CXXFLAGS	= $(CFLAGS) -fno-rtti -fno-exceptions

# linker flags used to generate the main object file
LDFLAGS		= -nostdlib -Wl,--no-check-sections -u call_user_start -Wl,-static

# linker script used for the above linker step
LD_SCRIPT_SLOT1	= eagle.app.v6.new.1024.app1.ld
LD_SCRIPT_SLOT2	= eagle.app.v6.new.1024.app2.ld

# various paths from the SDK used in this project
SDK_LIBDIR	= lib
SDK_LDDIR	= ld
#SDK_INCDIR	= extra_include include include/espressif include/json include/udhcp include/lwip include/lwip/lwip include/lwip/ipv4 include/lwip/ipv6
SDK_INCDIR	= include include/espressif include/freertos include/json include/lwip include/lwip/ipv4 include/lwip/ipv6 include/nopoll include/spiffs include/ssl
 
# select which tools to use as compiler, librarian and linker
CC		:= $(XTENSA_TOOLS_ROOT)/xtensa-lx106-elf-gcc
CXX		:= $(XTENSA_TOOLS_ROOT)/xtensa-lx106-elf-g++
AR		:= $(XTENSA_TOOLS_ROOT)/xtensa-lx106-elf-ar
LD		:= $(XTENSA_TOOLS_ROOT)/xtensa-lx106-elf-gcc
OBJCOPY := $(XTENSA_TOOLS_ROOT)/xtensa-lx106-elf-objcopy
OBJDUMP := $(XTENSA_TOOLS_ROOT)/xtensa-lx106-elf-objdump

# MD5 tool (making .md5 checksum files of compiled binaries needed by FOTA upgrade using WHTTPD)
MD5TOOL ?= md5.exe

# --------------- no user configurable options below here ---------------

SRC_DIR		:= $(MODULES)
BUILD_DIR	:= $(addprefix $(BUILD_BASE)/,$(MODULES))

SDK_LIBDIR	:= $(addprefix $(SDK_BASE)/,$(SDK_LIBDIR))
SDK_INCDIR	:= $(addprefix -I$(SDK_BASE)/,$(SDK_INCDIR))

SRC			:= $(foreach sdir,$(SRC_DIR),$(wildcard $(sdir)/*.c*))
C_OBJ		:= $(patsubst %.c,%.o,$(SRC))
CXX_OBJ		:= $(patsubst %.cpp,%.o,$(C_OBJ))
OBJ			:= $(patsubst %.o,$(BUILD_BASE)/%.o,$(CXX_OBJ))
LIBS		:= $(addprefix -l,$(LIBS))
APP_AR		:= $(addprefix $(BUILD_BASE)/,$(TARGET)_app.a)

TARGET_OUT_SLOT1	:= $(addprefix $(BUILD_BASE)/,$(TARGET).out1)
TARGET_OUT_SLOT2	:= $(addprefix $(BUILD_BASE)/,$(TARGET).out2)

LD_SCRIPT_SLOT1		:= $(addprefix -T$(SDK_BASE)/$(SDK_LDDIR)/,$(LD_SCRIPT_SLOT1))
LD_SCRIPT_SLOT2		:= $(addprefix -T$(SDK_BASE)/$(SDK_LDDIR)/,$(LD_SCRIPT_SLOT2))

INCDIR				:= $(addprefix -I,$(SRC_DIR))
EXTRA_INCDIR		:= $(addprefix -I,$(EXTRA_INCDIR))
MODULE_INCDIR		:= $(addsuffix /include,$(INCDIR))

V ?= $(VERBOSE)
ifeq ("$(V)","1")
Q :=
vecho := @true
else
Q := @
vecho := @echo
endif

vpath %.c $(SRC_DIR)
vpath %.cpp $(SRC_DIR)

define compile-objects
$1/%.o: %.c
	$(vecho) "CC $$<"
	$(Q) $(CC) $(INCDIR) $(MODULE_INCDIR) $(EXTRA_INCDIR) $(SDK_INCDIR) $(CFLAGS)  -c $$< -o $$@
$1/%.o: %.cpp
	$(vecho) "C+ $$<"
	$(Q) $(CXX) $(INCDIR) $(MODULE_INCDIR) $(EXTRA_INCDIR) $(SDK_INCDIR) $(CXXFLAGS)  -c $$< -o $$@
endef

.PHONY: wfofgen all checkdirs flash flash_slot1 flash_slot2 flashinit rebuild clean

wfofgen:
	wfof/generator/wfof_gen.exe wfof/content 4 wfof/wfof_data.h wfof/wfof_idxs.h
	touch wfof/wfof.c

all: checkdirs $(TARGET_OUT_SLOT1) $(TARGET_OUT_SLOT2)

$(TARGET_OUT_SLOT1): $(APP_AR)
	$(vecho) "==== LD $@ for slot 1 ============================================"
	$(Q) $(LD) -L$(SDK_LIBDIR) $(LD_SCRIPT_SLOT1) $(LDFLAGS) -Wl,--start-group $(LIBS) $(APP_AR) -Wl,--end-group -o $@
	$(vecho) "Section info:"
	$(Q) $(OBJDUMP) -h -j .data -j .rodata -j .bss -j .text -j .irom0.text $@
	$(vecho) "------------------------------------------------------------------------------"
	$(Q) $(ESPTOOL) elf2image $(TARGET_OUT_SLOT1) -o$(FW_BASE)/slot1/ $(flashimageoptions)
	$(vecho) "------------------------------------------------------------------------------"
	$(vecho) "(slot1/0x00000.bin --------> 0x01000)"
	$(vecho) "(slot1/0x11000.bin --------> 0x11000)"
	$(SDK_TOOLS)/gen_flashbin.exe $(FW_BASE)/slot1/0x00000.bin $(FW_BASE)/slot1/0x11000.bin 0x10000
	$(Q) mv eagle.app.flash.bin $(FW_BASE)/flash_slot1.bin
	$(Q) $(MD5TOOL) -n -o$(FW_BASE)/flash_slot1.md5 $(FW_BASE)/flash_slot1.bin
	$(vecho) "flash_slot1.bin -----------> 0x01000 successully generated in folder $(FW_BASE)"

$(TARGET_OUT_SLOT2): $(APP_AR)
	$(vecho) "==== LD $@ for slot 2 ============================================"
	$(Q) $(LD) -L$(SDK_LIBDIR) $(LD_SCRIPT_SLOT2) $(LDFLAGS) -Wl,--start-group $(LIBS) $(APP_AR) -Wl,--end-group -o $@
	$(vecho) "Section info:"
	$(Q) $(OBJDUMP) -h -j .data -j .rodata -j .bss -j .text -j .irom0.text $@
	$(vecho) "------------------------------------------------------------------------------"
	$(Q) $(ESPTOOL) elf2image $(TARGET_OUT_SLOT2) -o$(FW_BASE)/slot2/ $(flashimageoptions)
	$(vecho) "------------------------------------------------------------------------------"
	$(vecho) "(slot2/0x00000.bin --------> 0x81000)"
	$(vecho) "(slot2/0x91000.bin --------> 0x91000)"
	$(SDK_TOOLS)/gen_flashbin.exe $(FW_BASE)/slot2/0x00000.bin $(FW_BASE)/slot2/0x91000.bin 0x10000
	$(Q) mv eagle.app.flash.bin $(FW_BASE)/flash_slot2.bin
	$(Q) $(MD5TOOL) -n -o$(FW_BASE)/flash_slot2.md5 $(FW_BASE)/flash_slot2.bin
	$(vecho) "flash_slot2.bin -----------> 0x81000 successully generated in folder $(FW_BASE)"

$(APP_AR): $(OBJ)
	$(vecho) "AR $@"
	$(Q) $(AR) cru $@ $^

checkdirs: $(BUILD_DIR) $(FW_BASE)

$(BUILD_DIR):
	$(Q) mkdir -p $@

$(FW_BASE):
	$(Q) mkdir -p $@
	$(Q) mkdir -p $@/slot1
	$(Q) mkdir -p $@/slot2

flash: flash_slot1 flash_slot2 

flash_slot1: all
	$(vecho) "==== Flashing application to FOTA slot 1 ====================================="
	$(vecho) "flash_slot1.bin ---------------> 0x01000"
	$(ESPTOOL) -p $(ESPPORT) -b $(BAUD) write_flash $(flashimageoptions) 0x01000 $(FW_BASE)/flash_slot1.bin

flash_slot2: all
	$(vecho) "==== Flashing application to FOTA slot 2 ====================================="
	$(vecho) "flash_slot2.bin ---------------> 0x81000"
	$(ESPTOOL) -p $(ESPPORT) -b $(BAUD) write_flash $(flashimageoptions) 0x81000 $(FW_BASE)/flash_slot2.bin

# ===============================================================
# From http://bbs.espressif.com/viewtopic.php?f=10&t=305
# master-device-key.bin is only need if using espressive services
# master_device_key.bin 0x3E000 is not used , write blank
# See 2A-ESP8266__IOT_SDK_User_Manual__EN_v1.1.0.pdf
# http://bbs.espressif.com/download/file.php?id=532
#
# System parameter area is the last 16KB of flash
# 512KB flash - system parameter area starts from 0x7C000 
# 	download blank.bin to 0x7E000 as initialization or erase entire flash before.
# 1024KB flash - system parameter area starts from 0xFC000 
# 	download blank.bin to 0xFE000 as initialization or erase entire flash before.
# 2048KB flash - system parameter area starts from 0x1FC000 
# 	download blank.bin to 0x1FE000 as initialization or erase entire flash before.
# 4096KB flash - system parameter area starts from 0x3FC000 
# 	download blank.bin to 0x3FE000 as initialization or erase entire flash before.
# ===============================================================

flashinit:
	$(vecho) "==== Clearing entire flash (all 0xFF) ========================================"
	$(ESPTOOL) -p $(ESPPORT) -b $(BAUD) erase_flash
	$(vecho) "==== Flashing FOTA bootloader and ESP init ==================================="
	$(vecho) "boot_v1.5.bin -----------------> 0x00000"
	$(vecho) "esp_init_data_default.bin -----> 0xFC000"
	$(ESPTOOL) -p $(ESPPORT) write_flash $(flashimageoptions) \
		0x00000 $(SDK_BASE)/bin/boot_v1.5.bin \
		0xFC000 $(SDK_BASE)/bin/esp_init_data_default.bin

rebuild: clean wfofgen all

clean:
	$(Q) rm -f $(APP_AR)
	$(Q) rm -f $(TARGET_OUT_SLOT1)
	$(Q) rm -f $(TARGET_OUT_SLOT2)
	$(Q) rm -f *.bin
	$(Q) rm -f *.sym
	$(Q) rm -f *.ii
	$(Q) rm -f *.i
	$(Q) rm -f *.s
	$(Q) rm -rf $(BUILD_DIR)
	$(Q) rm -rf $(BUILD_BASE)
	$(Q) rm -rf $(FW_BASE)

$(foreach bdir,$(BUILD_DIR),$(eval $(call compile-objects,$(bdir))))
