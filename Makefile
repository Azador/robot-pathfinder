#
#
#

OBJ_DIR=obj

SOURCE_FILES=robot-pathfinder.cpp robot-map.cpp robot-geometry.cpp

ifeq ($(OS),Windows_NT)
CXX = cl.exe
LN = link.exe
OBJECT_EXTENSION=obj
EXECUTABLE_EXTENSION=.exe
CXX_OUTPUT_OPTION=/Fo
LN_OUTPUT_OPTION=/OUT:
CXX_FLAGS=/EHsc
else
CXX = g++
LN = g++
OBJECT_EXTENSION=o
EXECUTABLE_EXTENSION=
CXX_OUTPUT_OPTION=-o
LN_OUTPUT_OPTION=-o
CXX_FLAGS=-std=c++14
endif

OBJECT_FILES = $(SOURCE_FILES:%.cpp=$(OBJ_DIR)/%.$(OBJECT_EXTENSION))

.PHONY: clean all

all: $(OBJ_DIR)/robot-pathfinder$(EXECUTABLE_EXTENSION)

clean:
	rm -f $(OBJECT_FILES) 

$(OBJ_DIR):
	mkdir $(OBJ_DIR)

$(OBJ_DIR)/robot-pathfinder.$(OBJECT_EXTENSION): robot-pathfinder.h robot-map.h robot-geometry.h $(OBJ_DIR)

$(OBJ_DIR)/robot-map.$(OBJECT_EXTENSION): robot-map.h robot-geometry.h $(OBJ_DIR)

$(OBJ_DIR)/robot-geometry.$(OBJECT_EXTENSION): robot-geometry.h $(OBJ_DIR)

$(OBJ_DIR)/robot-pathfinder$(EXECUTABLE_EXTENSION): $(OBJECT_FILES)
	$(LN) $(LN_OUTPUT_OPTION)$@ $(OBJECT_FILES)
	
$(OBJ_DIR)/%.$(OBJECT_EXTENSION): %.cpp
	$(CXX) $(CXX_FLAGS) -c $(CXX_OUTPUT_OPTION)$@ $<
