FILES = src/misc.cpp #append your new cpp file
TEST_FILES = test/test.cpp

OUT = build/main
TEST_OUT = build/test

CC_FLAGS = -std=c++17 -Wall -O3
BUILD_DIR = ./build

.PHONY: all, clean, run_tests, main, test

all: #compiles everything
	[ -d $(BUILD_DIR) ] || mkdir -p $(BUILD_DIR)
	g++ -o $(OUT) src/main.cpp $(FILES) $(CC_FLAGS) 
	g++ -o $(TEST_OUT) $(FILES) $(TEST_FILES) $(CC_FLAGS)

main: #only compiles the application
	[ -d $(BUILD_DIR) ] || mkdir -p $(BUILD_DIR)
	g++ -o $(OUT) src/main.cpp $(FILES) $(CC_FLAGS) 

test: #only compiles the tests
	[ -d $(BUILD_DIR) ] || mkdir -p $(BUILD_DIR)
	g++ -o $(TEST_OUT) $(FILES) $(TEST_FILES) $(CC_FLAGS)

run_tests:
	$(TEST_OUT)

clean:
	rm -rf build/* 