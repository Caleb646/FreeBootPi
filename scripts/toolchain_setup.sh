#!/bin/bash
set -e

echo "Downloading and Installing x86_64 ARM Toolchain to $PWD"
wget --quiet https://developer.arm.com/-/media/Files/downloads/gnu/13.2.rel1/binrel/arm-gnu-toolchain-13.2.rel1-x86_64-aarch64-none-elf.tar.xz?rev=a05df3001fa34105838e6fba79ee1b23&hash=DCB97F9E407955B162E4652F4E78B6CCDF75E4FF
tar -xf 'arm-gnu-toolchain-13.2.rel1-x86_64-aarch64-none-elf.tar.xz?rev=a05df3001fa34105838e6fba79ee1b23'
mv "arm-gnu-toolchain-13.2.rel1-x86_64-aarch64-none-elf" "arm_toolchain"
rm 'arm-gnu-toolchain-13.2.rel1-x86_64-aarch64-none-elf.tar.xz?rev=a05df3001fa34105838e6fba79ee1b23'
