TOOLCHAIN ?= riscv64-unknown-elf-
CC := $(TOOLCHAIN)gcc
OBJCOPY := $(TOOLCHAIN)objcopy
SIZE := $(TOOLCHAIN)size
PYTHON ?= python3
ARCH_FLAGS := -march=rv32im -mabi=ilp32
CFLAGS := $(ARCH_FLAGS) -Os -g0 -ffreestanding -fno-builtin -fno-stack-protector \
          -ffunction-sections -fdata-sections -Wall -Wextra -Werror \
          -fno-ident -fno-unwind-tables -fno-asynchronous-unwind-tables
LDFLAGS := $(ARCH_FLAGS) -nostdlib -nostartfiles -Wl,-T,link.ld \
           -Wl,--gc-sections,--build-id=none,-Map,firmware.map
OBJECTS := start.o main.o astar.o nav_protocol.o layered_control.o layered_mmio.o

.PHONY: all clean size host-test
all: firmware.elf firmware.bin firmware.hex firmware_symbols.vh
firmware.elf: $(OBJECTS) link.ld
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@
firmware.bin: firmware.elf
	$(OBJCOPY) -O binary $< $@
firmware.hex: firmware.bin bin_to_hex.py
	$(PYTHON) bin_to_hex.py $< $@
firmware_symbols.vh: firmware.elf elf_symbols.py
	$(PYTHON) elf_symbols.py --nm $(TOOLCHAIN)nm $< $@
%.o: %.c astar.h mmio.h nav_protocol.h layered_control.h layered_mmio.h
	$(CC) $(CFLAGS) -c $< -o $@
start.o: start.S
	$(CC) $(ARCH_FLAGS) -c $< -o $@
size: firmware.elf
	$(SIZE) $<
host-test:
	$(MAKE) -C ../tests -f test_astar.mk test
clean:
	rm -f $(OBJECTS) firmware.elf firmware.bin firmware.hex firmware_lane?.hex firmware.map firmware_symbols.vh
