#******************************************************************************
# Regulator
#
PROJECT_NAME	:= regulator
OUTDIR 			:= build
INCLUDES := \
	app/inc	\
	cm4/Device/ST/STM32F3xx/Include \
	cm4/Include \
	drivers/inc \
	FreeRTOS/inc \
	lib/inc \
	system/inc \
	task/inc \
	system/inc

LD_FILES 		:= -T ldscripts/STM32F373CC_FLASH.ld
LIBS			:= lib/IQmathLib-cm4.a
TOOLCHAIN_PATH	:= arm-none-eabi

CPU_FLAGS := \
	-mcpu=cortex-m4 -mthumb \
	-mfloat-abi=hard \
	-mfpu=fpv4-sp-d16 \
	-ffunction-sections \
	-fdata-sections \
	-MMD -MP

CC_FLAGS := \
	$(CPU_FLAGS) \
	-std=gnu11 \
	-fmessage-length=0 \
	-fsigned-char \
	-fsingle-precision-constant \
	-Wfloat-equal \
	-Wuninitialized \
	-Wextra

LDFLAGS := \
	$(CPU_FLAGS) \
	$(LD_FILES) \
	-Werror -Wall -Wextra \
	-Wl,--gc-sections \
	-Wl,-Map="$(OUTDIR)/$(PROJECT_NAME).map" \
	-nostartfiles \
	-Xlinker --gc-sections --specs=nano.specs \
	
#******************************************************************************
# Параметры сборки проекта.
#
APP_OPTIMIZATION			:= -g3 -O2
CM4_OPTIMIZATION			:= -g3 -O2
DRIVERS_OPTIMIZATION		:= -g3 -O2
FREERTOS_OPTIMIZATION		:= -g3 -O2
SYSTEM_OPTIMIZATION			:= -g3 -O2
TASK_OPTIMIZATION			:= -g3 -O2

#******************************************************************************
# toolchain
#
CC		= $(TOOLCHAIN_PATH)-gcc
CPP		= $(TOOLCHAIN_PATH)-g++
CCDEP	= $(TOOLCHAIN_PATH)-gcc
LD		= $(TOOLCHAIN_PATH)-gcc
AR		= $(TOOLCHAIN_PATH)-ar
AS		= $(TOOLCHAIN_PATH)-gcc
OBJCOPY	= $(TOOLCHAIN_PATH)-objcopy
OBJDUMP	= $(TOOLCHAIN_PATH)-objdump
GDB		= $(TOOLCHAIN_PATH)-gdb
SIZE	= $(TOOLCHAIN_PATH)-size

#******************************************************************************
# Build [app]
#
CURRENT_DIR		:= app
C_FILE			:= $(shell find $(CURRENT_DIR) -maxdepth 3 -type f -name "*.c" )
I_PATH			:= $(addprefix -I, $(INCLUDES)) 
OBJ_FILE		:= $(addprefix $(OUTDIR)/obj/, $(C_FILE))
APP_OBJ_FILE	:= $(patsubst %.c, %.obj, $(OBJ_FILE))
$(OUTDIR)/obj/$(CURRENT_DIR)/%.obj:	$(CURRENT_DIR)/%.c
	@echo [CC] $<
	@mkdir -p $(dir $@)
	@$(CC) $(CC_FLAGS) $(I_PATH) $(APP_OPTIMIZATION) -c $< -o $@
	
#******************************************************************************
# Build [cm4]
#
CURRENT_DIR		:= cm4
C_FILE			:= $(shell find $(CURRENT_DIR) -maxdepth 6 -type f -name "*.c" )
I_PATH			:= $(addprefix -I, $(INCLUDES)) 
OBJ_FILE		:= $(addprefix $(OUTDIR)/obj/, $(C_FILE))
CM4_OBJ_FILE	:= $(patsubst %.c, %.obj, $(OBJ_FILE))
$(OUTDIR)/obj/$(CURRENT_DIR)/%.obj:	$(CURRENT_DIR)/%.c
	@echo [CC] $<
	@mkdir -p $(dir $@)
	@$(CC) $(CC_FLAGS) $(I_PATH) $(CM4_OPTIMIZATION) -c $< -o $@
	
#******************************************************************************
# Build [drivers]
#
CURRENT_DIR		:= drivers
C_FILE			:= $(shell find $(CURRENT_DIR) -maxdepth 3 -type f -name "*.c" )
I_PATH			:= $(addprefix -I, $(INCLUDES)) 
OBJ_FILE		:= $(addprefix $(OUTDIR)/obj/, $(C_FILE))
DRIVERS_OBJ_FILE	:= $(patsubst %.c, %.obj, $(OBJ_FILE))
$(OUTDIR)/obj/$(CURRENT_DIR)/%.obj:	$(CURRENT_DIR)/%.c
	@echo [CC] $<
	@mkdir -p $(dir $@)
	@$(CC) $(CC_FLAGS) $(I_PATH) $(DRIVERS_OPTIMIZATION) -c $< -o $@

#******************************************************************************
# Build [freertos]
#
CURRENT_DIR		:= freertos
C_FILE			:= $(shell find $(CURRENT_DIR) -maxdepth 3 -type f -name "*.c" )
I_PATH			:= $(addprefix -I, $(INCLUDES)) 
OBJ_FILE		:= $(addprefix $(OUTDIR)/obj/, $(C_FILE))
FREERTOS_OBJ_FILE	:= $(patsubst %.c, %.obj, $(OBJ_FILE))
$(OUTDIR)/obj/$(CURRENT_DIR)/%.obj:	$(CURRENT_DIR)/%.c
	@echo [CC] $<
	@mkdir -p $(dir $@)
	@$(CC) $(CC_FLAGS) $(I_PATH) $(FREERTOS_OPTIMIZATION) -c $< -o $@

#******************************************************************************
# Build [system]
#
CURRENT_DIR		:= system
C_FILE			:= $(shell find $(CURRENT_DIR) -maxdepth 3 -type f -name "*.c" )
I_PATH			:= $(addprefix -I, $(INCLUDES)) 
OBJ_FILE		:= $(addprefix $(OUTDIR)/obj/, $(C_FILE))
SYSTEM_OBJ_FILE	:= $(patsubst %.c, %.obj, $(OBJ_FILE))
$(OUTDIR)/obj/$(CURRENT_DIR)/%.obj:	$(CURRENT_DIR)/%.c
	@echo [CC] $<
	@mkdir -p $(dir $@)
	@$(CC) $(CC_FLAGS) $(I_PATH) $(SYSTEM_OPTIMIZATION) -c $< -o $@
	
#******************************************************************************
# Build [system] *.S
#
CURRENT_DIR		:= system
C_FILE			:= $(shell find $(CURRENT_DIR) -maxdepth 3 -type f -name "*.S" )
I_PATH			:= $(addprefix -I, $(INCLUDES)) 
OBJ_FILE		:= $(addprefix $(OUTDIR)/obj/, $(C_FILE))
SYSTEMASM_OBJ_FILE	:= $(patsubst %.S, %.obj, $(OBJ_FILE))
$(OUTDIR)/obj/$(CURRENT_DIR)/%.obj:	$(CURRENT_DIR)/%.S
	@echo [CC] $<
	@mkdir -p $(dir $@)
	@$(CC) $(CC_FLAGS) $(I_PATH) $(SYSTEM_OPTIMIZATION) -c $< -o $@
	
#******************************************************************************
# Build [system]
#
#CURRENT_DIR		:= system
#C_FILE			:= $(shell find $(CURRENT_DIR) -maxdepth 3 -type f -name "*.c" )
#SOURCEEXP		:= $(suffix $(C_FILE))
#I_PATH			:= $(addprefix -I, $(INCLUDES)) 
#OBJ_FILE		:= $(addprefix $(OUTDIR)/obj/, $(C_FILE))
#SYSTEM_OBJ_FILE	:= $(patsubst %.c, %.obj, $(OBJ_FILE))
#$(OUTDIR)/obj/$(CURRENT_DIR)/%.obj:	$(CURRENT_DIR)/%.c
#	@echo [CC] $<
#	@mkdir -p $(dir $@)
#	@$(CC) $(CC_FLAGS) $(I_PATH) $(APP_OPTIMIZATION) -c $< -o $@
	
#******************************************************************************
# Build [task]
#
CURRENT_DIR		:= task
C_FILE			:= $(shell find $(CURRENT_DIR) -maxdepth 3 -type f -name "*.c" )
I_PATH			:= $(addprefix -I, $(INCLUDES))
OBJ_FILE		:= $(addprefix $(OUTDIR)/obj/, $(C_FILE))
TASK_OBJ_FILE	:= $(patsubst %.c, %.obj, $(OBJ_FILE))
$(OUTDIR)/obj/$(CURRENT_DIR)/%.obj:	$(CURRENT_DIR)/%.c
	@echo [CC] $<
	@mkdir -p $(dir $@)
	@$(CC) $(CC_FLAGS) $(I_PATH) $(TASK_OPTIMIZATION) -c $< -o $@
	
PROJECT_OBJ_FILE	:= $(APP_OBJ_FILE) $(CM4_OBJ_FILE) $(DRIVERS_OBJ_FILE) $(FREERTOS_OBJ_FILE) $(SYSTEM_OBJ_FILE) $(SYSTEMASM_OBJ_FILE) $(TASK_OBJ_FILE) $(LIBS)

#******************************************************************************
# Targets
#
$(PROJECT_NAME).elf:	$(PROJECT_OBJ_FILE)
	@$(LD) $(LDFLAGS) $(PROJECT_OBJ_FILE) -o $(OUTDIR)/$(PROJECT_NAME).elf
	@echo ' '

$(PROJECT_NAME).hex: $(PROJECT_NAME).elf
	@$(OBJCOPY) -O ihex $(OUTDIR)/$(PROJECT_NAME).elf $(OUTDIR)/$(PROJECT_NAME).hex
	@echo ' '

mainbuild: $(PROJECT_NAME).elf $(PROJECT_NAME).hex
	@echo 'Print Size:'
	@$(SIZE) --format=berkeley "$(OUTDIR)/$(PROJECT_NAME).elf"
	@echo ' '

all: prebuild mainbuild

rebuild: clean all
	@echo ' '

clean:	
	@rm -rf $(OUTDIR)
	@echo ' '

prebuild:
	-wscript.exe otherFiles\versionGen.vbs app\src\version.c app\inc\version.h
	@echo ' '

.PHONY: all rebuild clean
