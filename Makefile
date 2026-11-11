CC ?= gcc
BUILD := build
APP := $(BUILD)/gps_kalman
TEST := $(BUILD)/run_tests
SRC := $(filter-out src/main.c,$(wildcard src/*.c))
OBJ := $(patsubst src/%.c,$(BUILD)/%.o,$(SRC))
CFLAGS ?= -std=c17 -O3 -pipe -march=native -mtune=native -flto=auto -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wstrict-prototypes -Werror=implicit-function-declaration
CPPFLAGS := -Iinclude -MMD -MP
LDLIBS := -lm
.PHONY: all run test clean debug sanitize format
all: $(APP)
$(APP): $(BUILD)/main.o $(OBJ) | $(BUILD)
	$(CC) $(CFLAGS) -flto=auto $^ $(LDLIBS) -o $@
$(TEST): $(BUILD)/tests/test_main.o $(OBJ) | $(BUILD)
	$(CC) $(CFLAGS) -flto=auto $^ $(LDLIBS) -o $@
$(BUILD)/%.o: src/%.c
	@mkdir -p $(@D)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@
$(BUILD)/tests/%.o: tests/%.c
	@mkdir -p $(@D)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@
$(BUILD):
	mkdir -p $@
run: $(APP)
	./$(APP)
test: $(TEST)
	./$(TEST)
debug: CFLAGS := -std=c17 -O0 -g3 -Wall -Wextra -Wpedantic -Wshadow -Wconversion

debug: clean all
sanitize: CFLAGS := -std=c17 -O1 -g3 -Wall -Wextra -Wpedantic -fsanitize=address,undefined -fno-omit-frame-pointer
sanitize: clean test
format:
	clang-format -i src/*.c include/*.h tests/*.c
clean:
	rm -rf $(BUILD) output
-include $(OBJ:.o=.d) $(BUILD)/main.d $(BUILD)/tests/test_main.d
