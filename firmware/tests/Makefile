# output
TARGET = target
OBJDIR = $(TARGET)/lib
TESTDIR = $(TARGET)/tests

APP_NAME = rotor_firmware_tests

APP_EXE = $(TARGET)/$(APP_NAME)

# sources
MAIN_CPP_SRCS = $(wildcard *.cpp)
MAIN_C_SRCS = $(wildcard *.c)
MAIN_OBJ_FILES = $(MAIN_C_SRCS:.c=.o) $(MAIN_CPP_SRCS:.cpp=.o)
MAIN_OBJS = $(patsubst %,$(OBJDIR)/%,$(MAIN_OBJ_FILES))

# mocks
MOCKS_PATH = mocks
MOCKS_SRCS = $(wildcard $(MOCKS_PATH)/*.cpp)
MOCKS_OBJ_FILES = $(MOCKS_SRCS:.cpp=.o)
MOCKS_OBJS = $(patsubst $(MOCKS_PATH)/%,  $(OBJDIR)/%, $(MOCKS_OBJ_FILES))

MOCKS_INCLUDE = -I$(MOCKS_PATH)

# compile custom libs
$(OBJDIR)/%.o:$(MOCKS_PATH)/%.cpp
	$(GXX) $(GXXOPTIONS) $(MOCKS_INCLUDE) $< -o $@

$(OBJDIR)/ground_station.o:
	$(GXX) $(GXXOPTIONS) $(MOCKS_INCLUDE) ../libs/ground_station.cpp -o $(OBJDIR)/ground_station.o

# executables and options
GCC        = gcc
GCCOPTIONS = -c -g -Os -w

GXX        = g++
GXXOPTIONS = -c -g -Os -w

AR      = ar
OBJCOPY = objcopy

# all - wil build $(APP_NAME).bin
all: $(APP_EXE)

# create target folders
$(TARGET):
	mkdir -p $(TARGET)
	
# create target/lib folder
$(OBJDIR): $(TARGET)
	mkdir -p $(OBJDIR)

# create target/lib folder
$(TESTDIR): $(TARGET)
	mkdir -p $(TESTDIR)

# clean - remove target
clean:
	rm -rf $(TARGET)

# builds the application code
$(OBJDIR)/%.o: %.cpp
	$(GXX) $(GCCOPTIONS) -I$(MOCKS_PATH) -I../libs $< -o $@

# builds the exeutable
$(APP_EXE): $(OBJDIR) $(MAIN_OBJS) $(MOCKS_OBJS) $(OBJDIR)/ground_station.o
	$(GXX) $(MAIN_OBJS) $(MOCKS_OBJS) $(OBJDIR)/ground_station.o -o $(APP_EXE)
	
test: $(APP_EXE) $(TESTDIR)
	$(APP_EXE) | tee $(TESTDIR)/tests.txt	
	
