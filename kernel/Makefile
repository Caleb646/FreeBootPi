# https://www.opensourceforu.com/2012/06/gnu-make-in-detail-for-beginners/

################ Dependencies ################

# Before executing the action (commands) corresponding to the desired target, 
# its dependencies must be met; when they are not met, the targets corresponding 
# to the unmet dependencies are executed before the given make target, to supply the missing dependencies.

# When a target is a filename, make compares the timestamps of the target file and 
# its dependency files. If the dependency filename is another target in the Makefile, 
# make then checks the timestamps of that target’s dependencies. It thus winds up 
# recursively checking all the way down the dependency tree, to the source code files, 
# to see if any of the files in the dependency tree are newer than their target 
# filenames. (Of course, if the dependency files don’t exist, then make knows it must 
# start executing the make targets from the “lowest” point in the dependency tree, to create them.)

################ Action Lines & Sub-Shells ################

# Each of the actions (shell commands written on a line) are executed 
# in a separate sub-shell. If an action changes the shell environment, 
# such a change is restricted to the sub-shell for that action line only. 
# For example, if one action line contains a command like cd newdir, the 
# current directory will be changed only for that line/action; for the 
# next line/action, the current directory will be unchanged.

################ Operators ################

# Simple assignment (:=)

# Conditional assignment (?=) statements assign the given value to the 
# variable only if the variable does not yet have a value.

# Appending (+=) operation appends texts to an existing variable. For example:
# CC = gcc
# CC += -W
# CC now holds the value gcc -W.

################ Patterns and Special Variables ################

# The % character can be used for wildcard pattern-matching, to provide generic targets. For example:
# %.o: %.c ## When % appears in the dependency list, it is replaced 
# with the same string that was used to perform substitution in the target.
# 		The % in make is a pattern match and it only works in pattern rules: 
#		pattern rules must have a % in the target (like the second rule you have, %.o : %.c). 
#		If you don't have a % in the target, then make just thinks that the % in the prerequisite 
#		list is a normal character like an a or b or whatever.

# special variables for matching filenames. Some of them are:

# $@ (full target name of the current target)
# $? (returns the dependencies that are newer than the current target)
# $* (returns the text that corresponds to % in the target)
# $< (name of the first dependency)
# $^ (name of all the dependencies with space as the delimiter)

################ Action Modifiers ################

# - (minus) — Prefixing this to any action causes any 
# error that occurs while executing the action to be ignored. 
# By default, execution of a Makefile stops when any command returns 
# a non-zero (error) value. If an error occurs, a message is printed, 
# with the status code of the command, and noting that the error has been ignored.

# @ (at) suppresses the standard print-action-to-standard-output 
# behaviour of make, for the action/command that is prefixed with @. 
# For example, to echo a custom message to standard output, we 
# want only the output of the echo command, and don’t want to print the 
# echo command line itself. @echo Message will print “Message” without the echo command line being printed.


# ARMGNU is a cross-compiler prefix.
 #aarch64-linux-gnu
ARMGNU ?= ../arm_toolchain/bin/aarch64-none-elf

# -Wall Show all warnings.

# -nostdlib Don't use the C standard library. Most of the 
# calls in the C standard library eventually interact with the operating system. 
# We are writing a bare-metal program, and we don't have any underlying operating system, 
# so the C standard library is not going to work for us anyway.

# -nostartfiles Don't use standard startup files. Startup 
# files are responsible for setting an initial stack pointer, 
# initializing static data, and jumping to the main entry point. We are going to do all of this by ourselves.

# -ffreestanding A freestanding environment is an environment 
# in which the standard library may not exist, and program startup 
# may not necessarily be at main. The option -ffreestanding directs 
# the compiler to not assume that standard functions have their usual definition.

# -Iinclude Search for header files in the include folder.

# -mgeneral-regs-only. Use only general-purpose registers. ARM processors 
# also have NEON registers. We don't want the compiler to use them because 
# they add additional complexity (since, for example, we will need to store the registers during a context switch).
COPS = -Wall -nostdlib -nostartfiles -ffreestanding -I../include -mgeneral-regs-only
ASMOPS = -I../include 

BUILD_DIR = ../build/kernel
OUTDIR = ../out
SRC_DIR = ""
SRC_DIR += peripherals

.PHONY: all
all: kernel.img

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

# $@ == ../build/kernel/peripherals/vpu_c.o
# $< == peripherals/vpu.c
# $(@D) == ../build/kernel/peripherals
# $(@F) == uart_c.o
$(BUILD_DIR)/%_c.o: %.c
	mkdir -p $(@D)
	$(ARMGNU)-gcc $(COPS) -MMD -c $< -o $@
	$(ARMGNU)-gcc $(COPS) -MMD -c -S $< -o $(@D)/$(@F).S
# $(info $$DDDDDDDDDDDDDDDDD = $@)
# $(info $$DDDDDDDDDDDDDDDDD = $<)
# $(info $$DDDDDDDDDDDDDDDDD = $(@D))
# $(info $$DDDDDDDDDDDDDDDDD = $(@F))

# This is a target for all of the Assembly .S files followed by the recipe
$(BUILD_DIR)/%_s.o: %.S
	mkdir -p $(@D)
	$(ARMGNU)-gcc $(ASMOPS) -MMD -c $< -o $@

OBJ_FILES = $(C_FILES:%.c=$(BUILD_DIR)/%_c.o)
OBJ_FILES += $(ASM_FILES:%.S=$(BUILD_DIR)/%_s.o)

DEP_FILES = $(OBJ_FILES:%.o=%.d)
-include $(DEP_FILES)

# $(ARMGNU)-ld -T $(SRC_DIR)/linker.ld -o kernel8.elf  $(OBJ_FILES)
# Use the OBJ_FILES array to build the kernel8.elf file. We use 
# the linker script src/linker.ld to define the basic layout of the resulting executable image

# $(ARMGNU)-objcopy kernel8.elf -O binary kernel8.img
# kernel8.elf is in the ELF format. The problem is that ELF files are 
# designed to be executed by an operating system. To write a bare-metal program, 
# we need to extract all executable and data sections from the ELF file and put them 
# into the kernel8.img image. The trailing 8 denotes ARMv8 which is a 64-bit 
# architecture. This filename tells the firmware to boot the processor into 64-bit mode.

# # 
kernel.img: linker.ld $(OBJ_FILES)
	mkdir -p $(OUTDIR)
	$(ARMGNU)-ld -Map $(OUTDIR)/kernel.map -T linker.ld -o $(BUILD_DIR)/kernel.elf $(OBJ_FILES)
	$(ARMGNU)-objcopy $(BUILD_DIR)/kernel.elf -O binary $(BUILD_DIR)/kernel.img
	cp $(BUILD_DIR)/kernel.img $(OUTDIR)/kernel.img