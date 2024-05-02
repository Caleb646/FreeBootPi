BUILD_DIR = ./build
OUTDIR = ./out


.PHONY: all
all: kernel.img loader.img

.PHONY: kernel
kernel: kernel.img

.PHONY: loader
loader: loader.img

kernel.img:
	$(MAKE) -C kernel $@

loader.img:
	$(MAKE) -C loader $@

.PHONY: clean
clean:
	rm -rf $(BUILD_DIR) $(OUTDIR)

# Copy kernel.img and kernel8.img (loads the kernel over serial) 
# from wsl fs to windows fs
.PHONY: kmove
kmove:
	sudo cp -u -v ./out/kernel.img /mnt/c/Users/c.thomas/Downloads/
	sudo cp -u -v ./out/kernel8.img /mnt/c/Users/c.thomas/Downloads/
