#!/bin/bash

set -e
echo "Downloading and Installing x86_64 ARM Toolchain to $PWD"
curl -o $PWD/toolchain.tar.xz -OL https://developer.arm.com/-/media/Files/downloads/gnu/13.2.rel1/binrel/arm-gnu-toolchain-13.2.rel1-x86_64-aarch64-none-elf.tar.xz?rev=a05df3001fa34105838e6fba79ee1b23&hash=DCB97F9E407955B162E4652F4E78B6CCDF75E4FF

wait

tar -xf $PWD/toolchain.tar.xz
mv $PWD/arm-gnu-toolchain-13.2.Rel1-x86_64-aarch64-none-elf "$PWD/arm_toolchain"
rm $PWD/toolchain.tar.xz
