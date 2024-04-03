CXX = g++
CXXFLAGS = -Wall -std=c++11
CXXFLAGS += -Icore/include -Isimulation/include -Iutilities/include

OBJ_DIR = build
CORE_SRC = $(wildcard core/src/*.cpp)
SIMULATION_SRC = $(wildcard simulation/src/*.cpp)
UTILITIES_SRC = $(wildcard utilities/src/*.cpp)
CORE_OBJ = $(CORE_SRC:core/src/%.cpp=$(OBJ_DIR)/%.o)
SIMULATION_OBJ = $(SIMULATION_SRC:simulation/src/%.cpp=$(OBJ_DIR)/%.o)
UTILITIES_OBJ = $(UTILITIES_SRC:utilities/src/%.cpp=$(OBJ_DIR)/%.o)

OBJS = $(CORE_OBJ) $(SIMULATION_OBJ) $(UTILITIES_OBJ)
EXEC = simulationExec

all: $(EXEC)

$(EXEC): $(OBJS)
	@mkdir -p $(@D)
	$(CXX) $(LDFLAGS) -o $@ $^

$(OBJ_DIR)/%.o: core/src/%.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR)/%.o: simulation/src/%.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR)/%.o: utilities/src/%.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf $(OBJ_DIR)
