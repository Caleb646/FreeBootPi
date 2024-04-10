#!/bin/bash
set -e

make clean
make qemu

qemu-system-aarch64 -machine raspi4b -nographic -kernel ./bin/kernel8.img ## -serial null -serial mon:stdio