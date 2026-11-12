CXX := g++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -Iinclude

all: bin/simulate_attitude

bin/simulate_attitude: src/simulate_attitude.cpp include/pid_controller.hpp
	@mkdir -p bin data
	$(CXX) $(CXXFLAGS) src/simulate_attitude.cpp -o bin/simulate_attitude

run: all
	./bin/simulate_attitude
	python3 python/analyze_px4_modes.py

clean:
	rm -rf bin
