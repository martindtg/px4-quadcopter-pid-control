CXX := g++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra

all: simulate_attitude

simulate_attitude: simulate_attitude.cpp pid_controller.hpp
	$(CXX) $(CXXFLAGS) simulate_attitude.cpp -o simulate_attitude

run: all
	./simulate_attitude
	python3 analyze_px4_modes.py

clean:
	rm -f simulate_attitude
