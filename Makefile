FILES = src/mms/mms.cpp src/algorithms/floodfill.cpp src/algorithms/optimisation.cpp #append your new cpp file
TEST_FILES = test/test.cpp

OUT = build/main
TEST_OUT = build/test

INC = -I lib/Adafruit_VL53L0X/src

CC_FLAGS = -std=c++17 -Wall -O3
BUILD_DIR = ./build

DEBUG = 1 #DEBUG MODE (1/0) - important for LOG

.PHONY: all, clean, run_tests, main, test, build_directory, build_directory_win

all: clean #compiles everything
	$(MAKE) main
	$(MAKE) test

main: build_directory #only compiles the application
	g++ -o $(OUT) src/algorithms/main.cpp $(FILES) $(CC_FLAGS) $(INC) -D DEBUG=$(DEBUG) -D DONT_INCLUDE_LIBS=1

test: build_directory #only compiles the tests
	g++ -o $(TEST_OUT) $(FILES) $(TEST_FILES) $(CC_FLAGS) -D DEBUG=0 -D DONT_INCLUDE_LIBS=1

wmain: build_directory_win
	g++ -o build\main.exe src/algorithms/main.cpp $(FILES) $(CC_FLAGS) -D DEBUG=$(DEBUG) -D DONT_INCLUDE_LIBS=1

build_directory_win:
	@if not exist "build" mkdir build 

build_directory:
	[ -d $(BUILD_DIR) ] || mkdir -p $(BUILD_DIR)

run_tests:
	$(TEST_OUT)

clean:
	rm -rf build/* 

wclean:
	rmdir /s /q build