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
