#!/bin/bash
set -e

make clean
make qemu

qemu-system-aarch64 -machine raspi4b -serial null -serial mon:stdio -kernel ./out/kernel8.img # -nographic 