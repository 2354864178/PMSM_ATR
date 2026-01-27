## Simple build for PMSM demo

CXX := g++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -Iinclude -Imodel
LDFLAGS :=

SRCS := src/main.cpp model/MotorModel.cpp model/TurbineModel.cpp model/ShaftModel.cpp model/PumpModel.cpp model/RectifierModel.cpp
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
