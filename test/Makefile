CXX      ?= g++
CXXFLAGS ?= -std=c++14 -Wall -Wextra -Wpedantic -O2

HEADERS := ../Clip.h ../MovingAverage.h ../PidController.h ../Trajectory.h

.PHONY: test clean

test: test_main
	./test_main

test_main: test_main.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) test_main.cpp -o test_main

clean:
	rm -f test_main
