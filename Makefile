CXX      = g++
CXXFLAGS = -std=c++17 -Wall -Wno-unused-variable -Wno-unused-function \
           -I. -Itest/mocks
TEST_BIN = test/run_tests

.PHONY: test clean

test: $(TEST_BIN)
	./$(TEST_BIN)

$(TEST_BIN): test/test_ball_plate.cpp ball_plate.ino \
             test/mocks/Arduino.h test/mocks/Servo.h test/mocks/TouchScreen.h
	$(CXX) $(CXXFLAGS) test/test_ball_plate.cpp -o $(TEST_BIN)

clean:
	rm -f $(TEST_BIN)
