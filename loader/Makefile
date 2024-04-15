# ARMGNU is a cross-compiler prefix.
ARMGNU ?= ../arm_toolchain/bin/aarch64-none-elf

COPS = -Wall -nostdlib -nostartfiles -ffreestanding -I../include -mgeneral-regs-only
ASMOPS = -I../include 

BUILD_DIR = ../build/loader
OUTDIR = ../out
SRC_DIR = ""

# $(info $$D = $(D))

.PHONY: all
all: loader.img

C_FILES = $(wildcard *.c $(foreach fd, $(SRC_DIR), $(fd)/*.c))
ASM_FILES =$(wildcard *.S $(foreach fd, $(SRC_DIR), $(fd)/*.S))

# The [-MMD] parameter instructs the gcc compiler 
# to create a dependency file for each generated object file. 
# A dependency file defines all of the dependencies for a particular 
# source file. These dependencies usually contain a list of all 
# included headers. We need to include all of the generated dependency 
# files so that make knows what exactly to recompile in case a header changes.
#
# This is a target for all of the C .o files followed by the recipe
$(BUILD_DIR)/%_c.o: %.c
	mkdir -p $(@D)
	$(ARMGNU)-gcc $(COPS) -MMD -c $< -o $@

# This is a target for all of the Assembly .S files followed by the recipe
$(BUILD_DIR)/%_s.o: %.S
	mkdir -p $(@D)
	$(ARMGNU)-gcc $(ASMOPS) -MMD -c $< -o $@

OBJ_FILES = $(C_FILES:%.c=$(BUILD_DIR)/%_c.o)
OBJ_FILES += $(ASM_FILES:%.S=$(BUILD_DIR)/%_s.o)

DEP_FILES = $(OBJ_FILES:%.o=%.d)
-include $(DEP_FILES)

loader.img: linker.ld $(OBJ_FILES)
	$(ARMGNU)-ld -T linker.ld -o $(BUILD_DIR)/kernel8.elf $(OBJ_FILES)
	$(ARMGNU)-objcopy $(BUILD_DIR)/kernel8.elf -O binary $(BUILD_DIR)/kernel8.img
	mkdir -p $(OUTDIR)
	cp $(BUILD_DIR)/kernel8.img $(OUTDIR)/kernel8.img
