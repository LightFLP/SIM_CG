rwildcard=$(foreach d,$(wildcard $(1:=/*)),$(call rwildcard,$d,$2) $(filter $(subst *,%,$2),$d)) # Recursive wildcard


INC_WINDOWS = ./include/windows
INC_LINUX   = include/linux
INC_COMMON  = $(patsubst %,-I%, $(sort $(dir $(call rwildcard, ./include/common, *.h))))

ifeq ($(OS),Windows_NT)
	RM = rm -vf # might have to be adjusted to del for some Windows distros
	INCLUDE = $(INC_WINDOWS)
	CXX_EXTRA_FLAGS = -Llib -lfreeglut -lglu32 -lopengl32 -lpng12
else
	RM = rm -vf
	INCLUDE = $(INC_LINUX)
	CXX_EXTRA_FLAGS = -lglut -lGLU -lGL -lpng
endif

CXXFLAGS = -g -Wall -Wno-sign-compare -I$(INCLUDE) $(INC_COMMON) -DHAVE_CONFIG_H 
CXX = g++

SRC_DIR = src
BIN_DIR = bin

SOURCES = $(call rwildcard,$(SRC_DIR),*.cpp)

OBJ_DIR = $(patsubst src/%, obj/%,$(sort $(dir $(SOURCES))))
OBJECTS = $(patsubst src/%.cpp, obj/%.o, $(SOURCES))
EXECUTABLE = $(BIN_DIR)/project1.exe

all: $(OBJ_DIR) $(BIN_DIR) $(EXECUTABLE)

$(OBJ_DIR):
	mkdir -v $@

$(BIN_DIR):
	mkdir -v $@


obj/%.o: $(SRC_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) -o $@ -c $<

$(EXECUTABLE): $(OBJECTS)
	$(CXX) -o $@ $^ $(CXX_EXTRA_FLAGS)

clean:
	@$(RM) $(foreach DIR, $(OBJ_DIR), $(filter %.o, $(wildcard $(DIR)*.o)))
	@$(RM) $(BIN_DIR)/*exe
