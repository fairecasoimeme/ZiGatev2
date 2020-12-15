###############################################################################
#
# MODULE:   Makefile
#
# DESCRIPTION: Makefile for the Control Bridge
# 
 ############################################################################
#
# This software is owned by NXP B.V. and/or its supplier and is protected
# under applicable copyright laws. All rights are reserved. We grant You,
# and any third parties, a license to use this software solely and
# exclusively on NXP products [NXP Microcontrollers such as  JN516x,
# JN517x, JN518x]. 
# You, and any third parties must reproduce the copyright and warranty notice
# and any other legend of ownership on each copy or partial copy of the 
# software.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Copyright NXP B.V. 2015. All rights reserved
#
############################################################################

TARGET  =    ControlBridge

###############################################################################
# User definable make parameters that may be overwritten from the command line
# Chip variant


# Set specific chip	(choose one)
#JENNIC_CHIP           ?= K32W061
#JENNIC_CHIP           ?= K32W041
JENNIC_CHIP           ?= JN5189
#JENNIC_CHIP           ?= JN5188

# Legacy chip family (always JN518x)
JENNIC_CHIP_FAMILY     ?= JN518x

# K32W0xx SDK settings
ifeq ($(findstring K32W0,$(JENNIC_CHIP)),K32W0)
# Chip/Device
SDK_DEVICE_FAMILY      ?= K32W061
# Board
SDK_BOARD              ?= k32w061dk6
# SDK folder
SDK_FOLDER             ?= K32W061DK6
endif

# JN518x SDK settings
ifeq ($(findstring JN518,$(JENNIC_CHIP)),JN518)
# Chip/Device
SDK_DEVICE_FAMILY      ?= JN5189
# Board
SDK_BOARD              ?= jn5189dk6
# SDK folder
SDK_FOLDER             ?= JN5189DK6
endif

ENDIAN                 ?= LITTLE_ENDIAN_PROCESSOR
DISABLE_LTO             = 1
ZBPRO_DEVICE_TYPE       = ZCR
JENNIC_MAC             ?= MiniMacShim
PDM_BUILD_TYPE         ?= _EEPROM
BAUD                   ?= 115200
DEBUG                  ?= SL
HEAP_SIZE              ?= 0xFA0
GP_SUPPORT			   ?= 1
OTA                    ?= 1
OTA_INTERNAL_STORAGE   ?= 1

# NFC NCI ICODE (new) functionality
APP_NCI_ICODE ?= 0
ifeq ($(APP_NCI_ICODE), 1)
CFLAGS  += -DAPP_NCI_ICODE
endif

# 32k clock configuration (1 = FRO, 0 = XTAL)
CFLAGS += -DgClkUseFro32K=1
# 32k FRO automatic calibration (0 = disable, 1 = enable)
CFLAGS += -DgClkRecalFro32K=0

# Use LowPower APIs (PWRM is used if not defined)
CFLAGS += -DAPP_LOW_POWER_API

###############################################################################
# un-comment the lines below for Debugger
# Debug options - define DEBUG to enable
# DEBUG ?=HW
#
# Define which UART to use for HW debug
# DEBUG_PORT ?= UART1
# DEBUG_MODE ?= SWD

NODE                   ?= COORDINATOR
CRC_NEW                ?= CRC_XOR
#CRC_NEW                ?= CCITT_CRC

###############################################################################
# Select the network stack (e.g. MAC, ZBPro)

# Framework features 
FRAMEWORK_SWITCH = 1

###############################################################################
# commands options
# enable optional commands as required per applicaion.
#
NETWORK_RECOVERY       ?= 0
STACK_MEASURE          ?= 0
APP_AHI_CONTROL        ?= 0

###############################################################################

CFLAGS  += -DUART_BAUD_RATE=$(BAUD)
CFLAGS  += -D$(NODE)
CFLAGS	+= -D$(CRC_NEW)
CFLAGS += -DJENNIC_DEBUG_ENABLE
ifeq ($(DEBUG), UART1)
CFLAGS  += -DDBG_ENABLE
$(info Building with debug UART1...)
CFLAGS  += -DUART_DEBUGGING
CFLAGS  += -DDEBUG_UART=DBG_E_UART_1
#CFLAGS  += -DDEBUG_APP_OTA
CFLAGS  += -DDEBUG_ZCL
CFLAGS	+= -DDEBUG_NWK_RECOVERY
#CFLAGS	+= -DDEBUG_PDM_EXTENDED
CFLAGS	+= -DDEBUG_ZB_CONTROLBRIDGE_TASK
CFLAGS  += -DTRACE_APP
#CFLAGS  += -DDEBUG_PDM_EXTENDED
CFLAGS += -DDEBUG_SERIAL_LINK
endif


ifeq ($(DEBUG), SL)
$(info Building with debug SL ...)
CFLAGS  += -DDEBUG_ZCL
CFLAGS	+= -DDEBUG_NWK_RECOVERY
#CFLAGS	+= -DDEBUG_PDM_EXTENDED
CFLAGS	+= -DDEBUG_ZB_CONTROLBRIDGE_TASK
CFLAGS  += -DTRACE_APPSTART
CFLAGS  += -DTRACE_APP
#CFLAGS  += -DDEBUG_PDM_EXTENDED
#CFLAGS += -DDEBUG_SERIAL_LINK
CFLAGS += -DDEBUG_CLD_ONOFF
endif 

ifeq ($(NETWORK_RECOVERY), 1)
CFLAGS	+= -DNETWORK_RECOVERY
endif

ifeq ($(STACK_MEASURE), 1)
CFLAGS	+= -DSTACK_MEASURE
endif

ifeq ($(APP_AHI_CONTROL), 1)
CFLAGS	+= -DAPP_AHI_CONTROL
endif

ifneq ($(GP_SUPPORT), 1)
ifeq ($(NODE), COORDINATOR)
$(info Building Node Coordinator Only...)
APP_ZPSCFG = ControlBridge_Coord.zpscfg
else
$(info Building Node Full functionality ...)
APP_ZPSCFG = ControlBridge_Full.zpscfg
endif
else
ifeq ($(NODE), COORDINATOR)
$(info Building Node Coordinator Only...)
APP_ZPSCFG = ControlBridge_Coord_GpProxy.zpscfg
else
$(info Building Node Full functionality ...)
APP_ZPSCFG = ControlBridge_Full_GpProxy.zpscfg
endif
endif

ifeq ($(GP_SUPPORT), 1)
CFLAGS  +=-DGP_PROXY_BASIC_DEVICE
$(info  "NOTE: Make sure that app_GP.cfg Manually Changed to have device ID as 0x0061 and also remove GP cluster id from in cluster list of Endpoint 242 for Proxy build")
CFLAGS  += -DGP_CERITIFICATION_TEST
CFLAGS  += -DCLD_GREENPOWER
APP_CLUSTERS_GREENPOWER_SRC ?=1
endif

###############################################################################
# Path definitions

# Use if application directory contains multiple targets

# Toolchain paths
TOOLCHAIN_DIR_PATH  = C:/NXP/MCUXpressoIDE_11.2.0_4120/ide/tools
TOOLCHAIN_PATH      = $(TOOLCHAIN_DIR_PATH)/bin

# SDK paths
SDK2_BASE_DIR       =  ../../Source/SDK/SDKPackages/$(SDK_FOLDER)

# Application paths
APP_BASE            =  ../..
APP_BLD_DIR         = $(APP_BASE)/Build/jn518x_mcux
APP_SRC_DIR         = $(APP_BASE)/Source/ControlBridge
APP_COMMON_SRC_DIR  = $(APP_BASE)/Source/Common
BOARD_DIR           = $(APP_BASE)/Source/board
AWK_SRC             = $(APP_BASE)/Source/Awk

# Tool paths
AWK_EXE             = $(APP_BASE)/../../gawk-3.1.6-1-bin/bin/gawk.exe

###############################################################################
# Application Source files
# Note: Path to source file is found using vpath below, so only .c filename is required

APPSRC = port_JN518x.c
APPSRC += pdum_gen.c
APPSRC += zps_gen.c
APPSRC += hardware_init.c
APPSRC += app_start.c
APPSRC += pin_mux.c
APPSRC += fsl_gpio.c
APPSRC += SerialLink.c
APPSRC += app_uart.c
APPSRC += app_Znc_cmds.c
APPSRC += app_general_events_handler.c
APPSRC += app_zcl_event_handler.c
APPSRC += pdum_apdu.S
APPSRC += app_ota_server.c
APPSRC += temp_sensor_drv.c
APPSRC += fsl_adc.c

ifeq ($(NETWORK_RECOVERY), 1)
APPSRC += app_network_recovery.c
endif

ifeq ($(STACK_MEASURE), 1)
APPSRC += StackMeasure.c
endif

ifeq ($(APP_AHI_CONTROL), 1)
APPSRC += app_ahi_commands.c
endif

ifeq ($(GP_SUPPORT), 1)
APPSRC += app_green_power.c
APPSRC += app_power_on_counter.c
endif

ifeq ($(APP_NCI_ICODE),1)
APPSRC += app_nci_icode.c
APPSRC += fsl_i2c.c
endif

ifeq ($(FRAMEWORK_SWITCH), 1)
APPSRC += usart_dma_rxbuffer.c
endif

LIBNAME = $(TARGET)_PDM$(PDM_BUILD_TYPE)_BAUD_$(BAUD)_DEBUG_$(DEBUG)

###############################################################################
# Standard Application header search paths

INCFLAGS += -I$(APP_SRC_DIR)
INCFLAGS += -I$(APP_COMMON_SRC_DIR)
INCFLAGS += -I$(BOARD_DIR)

###############################################################################
# Application libraries
# Specify additional Component libraries

#APPLIBS += 

###############################################################################
# NCI ICODE (new) library
ifeq ($(APP_NCI_ICODE),1)
# NCI library folder
APP_NCI_LIB_DIR  = $(APP_BASE)/Source/NFC
#APP_NCI_LIB_DIR   = $(SDK2_BASE_DIR)/middleware/wireless/framework/NFC
# NCI library (unused for 8x)
LDLIBS           += NCI_$(JENNIC_CHIP_FAMILY)
# NCI library path (unused for 8x)
LDFLAGS         += -L$(APP_NCI_LIB_DIR)/Library
#LDFLAGS          += -L$(APP_NCI_LIB_DIR)/JN518x/Library
# NCI include path
INCFLAGS         += -I$(APP_NCI_LIB_DIR)/Include
endif

###############################################################################
# Construct strings for output file names

# Construct binary features list 
ifeq ($(NODE), COORDINATOR)
TARGET_FEATURES:=$(TARGET_FEATURES)_Coord
else
TARGET_FEATURES:=$(TARGET_FEATURES)_Full
endif

ifeq ($(APP_NCI_ICODE),1)
TARGET_FEATURES:=$(TARGET_FEATURES)_NciIcode
endif

ifeq ($(GP_SUPPORT), 1)
TARGET_FEATURES:=$(TARGET_FEATURES)_GpProxy
endif

TARGET_FEATURES:=$(TARGET_FEATURES)_$(BAUD)

# Construct full file name
TARGET_FULL := $(TARGET)$(TARGET_FEATURES)

$(info Creating folders...)
APP_OUT_DIR = $(APP_BASE)/Binaries/$(TARGET_FULL)
APP_OBJ_DIR = $(APP_BASE)/Binaries/$(TARGET_FULL)/obj
$(info $(shell mkdir -p $(APP_OUT_DIR)))
$(info $(shell mkdir -p $(APP_OBJ_DIR)))

###############################################################################

# You should not need to edit below this line

APP_CLUSTERS_MEASUREMENT_AND_SENSING_SRC    ?= 1
APP_CLUSTERS_HVAC_SRC                       ?= 1
APP_CLUSTERS_IAS_SRC                        ?= 1
APP_CLUSTERS_SMART_ENERGY_SRC               ?= 1
APP_CLUSTERS_ENERGY_AT_HOME_SRC             ?= 1
APP_CLUSTERS_OTA_SRC                        ?= 1
APP_CLUSTER_LIGHTING_SRC                    ?= 1
APP_CLUSTERS_APPLIANCE_MANAGEMENT_SRC       ?= 1
APP_CLUSTERS_CLOSURES_SRC                   ?= 1
APP_CLUSTERS_SECURITY_AND_SAFETY_SRC        ?= 1
BDB_SUPPORT_OOBC                            ?= 1
BDB_SUPPORT_NWK_STEERING                    ?= 1
BDB_SUPPORT_NWK_FORMATION                   ?= 1
ifneq ($(NODE), COORDINATOR)
APP_CLUSTER_COMMISSIONING_SRC               ?= 1
# BDB features � Enable as required
BDB_SUPPORT_TOUCHLINK_INITIATOR_ROUTER      ?= 1
endif
ifeq ($(APP_NCI_ICODE), 1)
BDB_SUPPORT_OOBC ?= 1
endif

###############################################################################
###############################################################################
# Optional stack features to pull relevant libraries into the build.

OPTIONAL_STACK_FEATURES = $(shell $(ZPSCONFIG) -n $(TARGET) -f $(APP_SRC_DIR)/$(APP_ZPSCFG) -y )

###############################################################################
# Configure for the selected chip or chip family
ifeq ($(FRAMEWORK_SWITCH), 1)
include $(SDK2_BASE_DIR)/middleware/wireless/zigbee/BuildConfig/ZCL/Build/config_ZCL.mk
include $(SDK2_BASE_DIR)/middleware/wireless/zigbee/BDB/Build/config.mk
else
include $(SDK2_BASE_DIR)/middleware/wireless/zigbee3.0/BuildConfig/ZCL/Build/config_ZCL.mk
include $(SDK2_BASE_DIR)/middleware/wireless/zigbee3.0/BDB/Build/config.mk
endif

###############################################################################

TEMP         = $(APPSRC:.c=.o)
APPOBJS_TMP  = $(TEMP:.S=.o)
APPOBJS     := $(addprefix $(APP_OBJ_DIR)/,$(APPOBJS_TMP))

###############################################################################
# Application dynamic dependencies

APPDEPS_TMP = $(APPOBJS_TMP:.o=.d)
APPDEPS := $(addprefix $(APP_OBJ_DIR)/,$(APPDEPS_TMP))

###############################################################################
# Linker

# Add application libraries before chip specific libraries to linker so
# symbols are resolved correctly (i.e. ordering is significant for GCC)

LDLIBS := $(APPLIBS) $(LDLIBS)

IMAGE_TOOL               = $(SDK2_BASE_DIR)/tools/imagetool/dk6_image_tool.py
IMAGE_TOOL_CONFIGURATION = -s 294912

###############################################################################
# Dependency rules

.PHONY: all clean
# Path to directories containing application source 
vpath % $(APP_SRC_DIR):$(APP_COMMON_SRC_DIR):$(ZCL_SRC):$(ZCL_SRC_DIRS):$(BDB_SRC_DIR):$(ZIGBEE_BASE_SRC):$(BOARD_DIR)

all: $(APP_OUT_DIR)/$(TARGET_FULL).axf

-include $(APPDEPS)
%.d:
#    rm -f $*.o

$(APP_SRC_DIR)/pdum_gen.c $(APP_SRC_DIR)/pdum_gen.h: $(APP_SRC_DIR)/$(APP_ZPSCFG) $(PDUMCONFIG)
	$(info Configuring the PDUM ...)
	$(PDUMCONFIG) -z $(TARGET) -e LITTLE_ENDIAN_PROCESSOR -f $< -o $(APP_SRC_DIR)

$(APP_SRC_DIR)/zps_gen.c $(APP_SRC_DIR)/zps_gen.h: $(APP_SRC_DIR)/$(APP_ZPSCFG) $(ZPSCONFIG)
	$(info Configuring the Zigbee Protocol Stack ...)
	$(ZPSCONFIG) -n $(TARGET) -e LITTLE_ENDIAN_PROCESSOR -t $(JENNIC_CHIP) -l $(ZPS_NWK_LIB) -a $(ZPS_APL_LIB) -c $(TOOLCHAIN_DIR_PATH) -f $< -o $(APP_SRC_DIR)

pdum_gen.o zps_gen.o: CFLAGS += -fno-lto

$(APP_OBJ_DIR)/port_JN518x.o: port_JN518x.c 
	$(info Compiling $< into $@ ...)
	$(TOOLCHAIN_PATH)/$(CC) -c -o $@ $(CFLAGS) $(INCFLAGS) $< -MD -MF $(APP_OBJ_DIR)/$*.d -MP
	@echo Source Files > $(APP_OUT_DIR)/$(TARGET_FULL).log
	@echo $@ $< >> $(APP_OUT_DIR)/$(TARGET_FULL).log
	@echo

$(APP_OBJ_DIR)/%.o: %.c 
	$(info Compiling $< into $@ ...)
	$(TOOLCHAIN_PATH)/$(CC) -c -o $@ $(CFLAGS) $(INCFLAGS) $< -MD -MF $(APP_OBJ_DIR)/$*.d -MP
	@echo $@ $< >> $(APP_OUT_DIR)/$(TARGET_FULL).log
	@echo

$(APP_OBJ_DIR)/%.o: %.S
	$(info Assembling $< into $@ ...)
	$(TOOLCHAIN_PATH)/$(CC) -c -o $@ $(CFLAGS) $(INCFLAGS) $< -MD -MF $(APP_OBJ_DIR)/$*.d -MP
	@echo $@ $< >> $(APP_OUT_DIR)/$(TARGET_FULL).log
	@echo

$(APP_OUT_DIR)/$(TARGET_FULL).axf: $(APPOBJS) $(addsuffix.a,$(addprefix $(COMPONENTS_BASE_DIR)/Library/lib,$(APPLDLIBS))) 
	$(info Linking $@ ...)
	#$(TOOLCHAIN_PATH)/$(CC)  -Wl,--gc-sections $(LDFLAGS) -T "AppBuildNone_$(JENNIC_CHIP).ld" -T "$(JENNIC_CHIP)_rom.ld" -o $@ -Wl,--start-group $(APPOBJS) $(addprefix -l,$(LDLIBS)) -lm -Wl,--end-group -Wl,-Map,$(APP_OUT_DIR)/$(TARGET_FULL).map 
	$(TOOLCHAIN_PATH)/$(CC)  -Wl,--gc-sections $(LDFLAGS) -T "AppBuildNone_$(JENNIC_CHIP).ld" -o $@ -Wl,--start-group $(APPOBJS) $(addprefix -l,$(LDLIBS)) -lm -Wl,--end-group -Wl,-Map,$(APP_OUT_DIR)/$(TARGET_FULL).map 
	$(info Generating binary ...)
	$(TOOLCHAIN_PATH)/$(SIZE) $@  
	date +%c >> $(APP_BASE)/Doc/size.txt
	$(TOOLCHAIN_PATH)/$(SIZE) $@ >> $(APP_BASE)/Doc/size.txt
	-$(AWK_EXE) -f $(AWK_SRC)/JN51xx_MAP.awk $(APP_OUT_DIR)/$(TARGET_FULL).log $(APP_OUT_DIR)/$(TARGET_FULL).map > $(APP_OBJ_DIR)/temp.txt
	$(IMAGE_TOOL) -b 7 $(IMAGE_TOOL_CONFIGURATION) $(APP_OUT_DIR)/$(TARGET_FULL).axf 
	rm -f $(APP_BLD_DIR)/data.bin
	$(TOOLCHAIN_PATH)/$(OBJCOPY) -v -O binary $(APP_OUT_DIR)/$(TARGET_FULL).axf $(APP_OUT_DIR)/$(TARGET_FULL).bin
	$(TOOLCHAIN_PATH)/$(OBJDUMP) -d $(APP_OUT_DIR)/$(TARGET_FULL).axf > $(APP_OUT_DIR)/$(TARGET_FULL).dis
	
################################################################################

clean:
	rm -f $(APPOBJS)
	rm -f $(APPDEPS)
	rm -f $(APP_OUT_DIR)/$(TARGET_FULL).bin
	rm -f $(APP_OUT_DIR)/$(TARGET_FULL).axf
	rm -f $(APP_OUT_DIR)/$(TARGET_FULL).map
	rm -f $(APP_OUT_DIR)/$(TARGET_FULL).dis
	rm -f $(APP_OUT_DIR)/$(TARGET_FULL).html
	rm -f $(APP_OUT_DIR)/$(TARGET_FULL).log
	rm -f $(APP_OUT_DIR)/$(TARGET_FULL).txt
	rm -f $(APP_SRC_DIR)/pdum_gen.* $(APP_SRC_DIR)/zps_gen*.* $(APP_SRC_DIR)/pdum_apdu.*

###############################################################################

