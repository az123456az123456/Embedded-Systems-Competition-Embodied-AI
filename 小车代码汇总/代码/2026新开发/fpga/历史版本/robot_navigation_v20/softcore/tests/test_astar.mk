HOST_CC ?= cc
TEST_BIN ?= /tmp/acg138-softcore-test-astar
.PHONY: test
test:
	$(HOST_CC) -std=c11 -O2 -Wall -Wextra -Werror -pedantic \
	  -fsanitize=address,undefined -fno-omit-frame-pointer \
	  test_astar.c ../firmware/astar.c -o $(TEST_BIN)
	ASAN_OPTIONS=detect_leaks=0 $(TEST_BIN)
