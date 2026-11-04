CC ?= cc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra -Werror -Wconversion -Wshadow
CROSS_COMPILE ?= aarch64-none-linux-gnu-

.PHONY: all test arm64 clean
all: softcore-monitor

softcore-monitor: softcore-monitor.c
	$(CC) $(CFLAGS) $< -o $@

arm64: softcore-monitor.c
	$(CROSS_COMPILE)gcc $(CFLAGS) $< -o softcore-monitor.aarch64
	$(CROSS_COMPILE)strip softcore-monitor.aarch64

test: softcore-monitor
	./softcore-monitor --self-test
	python3 test_monitor_cli.py

clean:
	rm -f softcore-monitor softcore-monitor.aarch64
