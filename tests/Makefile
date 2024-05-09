ifeq ($(OS),Windows_NT)
  ifeq ($(shell uname -s),) # not in a bash-like shell
	CLEANUP = del /F /Q
	MKDIR = mkdir
  else # in a bash-like shell, like msys
	CLEANUP = rm -f
	MKDIR = mkdir -p
  endif
	TARGET_EXTENSION=exe
else
	CLEANUP = rm -f
	MKDIR = mkdir -p
	TARGET_EXTENSION=out
endif

.PHONY: clean
.PHONY: test

UNITY_PATH = unity/src/
SRC_PATH = ../kernel/
TESTS_PATH = tests/
BUILD_PATH = build/
DEPS_PATH = build/depends/
OBJS_PATH = build/objs/
RESULTS_PATH = build/results/

BUILD_PATHS = $(BUILD_PATH) $(DEPS_PATH) $(OBJS_PATH) $(RESULTS_PATH)

SRCT = $(wildcard $(TESTS_PATH)*.c)

$(info $$D = $(SRCT))

COMPILE=gcc -c
LINK=gcc
DEPEND=gcc -MM -MG -MF
C_INCLUDE = -I.
C_INCLUDE += -I$(UNITY_PATH)
C_INCLUDE += -I$(SRC_PATH)
C_INCLUDE += -I../include # source include path
C_INCLUDE += -I./include # test include path
CFLAGS = $(C_INCLUDE) -DTEST
CFLAGS += -g # add debug information

# NOTE: format of test files -> test_mem.c and test_sync.c corresponds to src files -> mem.c and sync.c
RESULTS = $(patsubst $(TESTS_PATH)test_%.c, $(RESULTS_PATH)test_%.txt, $(SRCT))

PASSED = `grep -s PASS $(RESULTS_PATH)*.txt`
FAIL = `grep -s FAIL $(RESULTS_PATH)*.txt`
IGNORE = `grep -s IGNORE $(RESULTS_PATH)*.txt`

test: $(BUILD_PATHS) $(RESULTS)
	@echo "-----------------------\nIGNORES:\n-----------------------"
	@echo "$(IGNORE)"
	@echo "-----------------------\nFAILURES:\n-----------------------"
	@echo "$(FAIL)"
	@echo "-----------------------\nPASSED:\n-----------------------"
	@echo "$(PASSED)"
	@echo "\nDONE"

$(RESULTS_PATH)%.txt: $(BUILD_PATH)%.$(TARGET_EXTENSION)
	-./$< > $@ 2>&1

$(BUILD_PATH)test_%.$(TARGET_EXTENSION): $(OBJS_PATH)test_%.o $(OBJS_PATH)%.o $(OBJS_PATH)unity.o
	$(LINK) -o $@ $^

$(OBJS_PATH)%.o:: $(TESTS_PATH)%.c
	$(COMPILE) $(CFLAGS) $< -o $@

$(OBJS_PATH)%.o:: $(SRC_PATH)%.c
	$(COMPILE) $(CFLAGS) $< -o $@

$(OBJS_PATH)%.o:: $(UNITY_PATH)%.c $(UNITY_PATH)%.h
	$(COMPILE) $(CFLAGS) $< -o $@

$(DEPS_PATH)%.d:: $(TESTS_PATH)%.c
	$(DEPEND) $@ $<

$(BUILD_PATH):
	$(MKDIR) $(BUILD_PATH)

$(DEPS_PATH):
	$(MKDIR) $(DEPS_PATH)

$(OBJS_PATH):
	$(MKDIR) $(OBJS_PATH)

$(RESULTS_PATH):
	$(MKDIR) $(RESULTS_PATH)

clean:
	$(CLEANUP) $(OBJS_PATH)*.o
	$(CLEANUP) $(BUILD_PATH)*.$(TARGET_EXTENSION)
	$(CLEANUP) $(RESULTS_PATH)*.txt

.PRECIOUS: $(BUILD_PATH)test_%.$(TARGET_EXTENSION)
.PRECIOUS: $(DEPS_PATH)%.d
.PRECIOUS: $(OBJS_PATH)%.o
.PRECIOUS: $(RESULTS_PATH)test_%.txt
