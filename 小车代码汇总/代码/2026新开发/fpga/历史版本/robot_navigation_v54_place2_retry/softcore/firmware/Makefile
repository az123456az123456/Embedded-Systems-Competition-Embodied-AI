TOOLCHAIN ?= riscv64-unknown-elf-
CC := $(TOOLCHAIN)gcc
OBJCOPY := $(TOOLCHAIN)objcopy
SIZE := $(TOOLCHAIN)size
PYTHON ?= python3
ARCH_FLAGS := -march=rv32im -mabi=ilp32
CFLAGS := $(ARCH_FLAGS) -Os -g0 -ffreestanding -fno-builtin -fno-stack-protector \
          -ffunction-sections -fdata-sections -Wall -Wextra -Werror \
          -fno-ident -fno-unwind-tables -fno-asynchronous-unwind-tables -fstack-usage -flto
LDFLAGS := $(ARCH_FLAGS) -flto -fstack-usage -nostdlib -nostartfiles -Wl,-T,link.ld -Wl,-T,navigation_budget.ld \
           -Wl,--gc-sections,--build-id=none,-Map,firmware.map
OBJECTS := start.o main.o astar.o nav_protocol.o layered_control.o layered_mmio.o \
           nav_exec_wire.o nav_exec_protocol.o nav_exec.o nav_math.o nav_stop.o \
           nav_local_pose.o nav_track.o nav_avoid.o nav_obstacle.o nav_exec_mmio.o nav_runtime.o freestanding.o

.PHONY: all clean size host-test test-gate verify-memory-image
all: firmware.elf firmware.bin firmware.hex firmware_symbols.vh verify-memory-image
firmware.elf: $(OBJECTS) link.ld navigation_budget.ld
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@
firmware.bin: firmware.elf
	$(OBJCOPY) -O binary $< $@
firmware.hex: firmware.bin bin_to_hex.py memory_profile.json
	$(PYTHON) bin_to_hex.py $< $@
verify-memory-image: firmware.hex verify_memory_image.py
	$(PYTHON) verify_memory_image.py firmware.hex
firmware_symbols.vh: firmware.elf elf_symbols.py
	$(PYTHON) elf_symbols.py --nm $(TOOLCHAIN)nm $< $@
nav_exec_wire.o: ../../common/nav_exec_wire.c ../../common/nav_exec_wire.h
	$(CC) $(CFLAGS) -c $< -o $@
freestanding.o: freestanding.c
	$(CC) $(filter-out -flto,$(CFLAGS)) -fno-lto -c $< -o $@
%.o: %.c astar.h mmio.h nav_protocol.h layered_control.h layered_mmio.h nav_exec_mmio.h nav_runtime.h
	$(CC) $(CFLAGS) -c $< -o $@
start.o: start.S
	$(CC) $(ARCH_FLAGS) -c $< -o $@
size: firmware.elf
	$(SIZE) $<
host-test:
	$(PYTHON) ../../verification/host/run_host.py
test-gate:
	$(PYTHON) ../../verification/run_gate.py
clean:
	rm -f $(OBJECTS) firmware.elf firmware.bin firmware.hex firmware_lane?.hex firmware_low.hex firmware_tail_lane?_??.hex firmware.memory.json firmware.map firmware_symbols.vh
