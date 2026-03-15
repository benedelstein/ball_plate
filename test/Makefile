CXX = g++
CXXFLAGS = -std=c++11 -Wall -Wextra -g

test: test_all.cpp ../PIDController.h ../MovingAverage.h ../Trajectory.h
	$(CXX) $(CXXFLAGS) -o test_all test_all.cpp -lm

run: test
	./test_all

clean:
	rm -f test_all

.PHONY: run clean
