## Simple build for PMSM demo

CXX := g++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -Iinclude
LDFLAGS :=

SRCS := src/main.cpp \
		src/simulation_logger.cpp \
		model/controllers/SVPWMController.cpp \
		model/components/MotorModel.cpp \
		model/components/InverterModel.cpp \
		model/components/TurbineModel.cpp \
		model/components/ShaftModel.cpp \
		model/components/PumpModel.cpp \
		model/solver/SystemSolver.cpp \
		model/solver/RK4Utils.cpp
OBJS := $(SRCS:%.cpp=build/%.o)
BIN := build/main

.PHONY: all run clean

all: $(BIN)

run: $(BIN)
	./$(BIN)

$(BIN): $(OBJS)
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

build/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf build
