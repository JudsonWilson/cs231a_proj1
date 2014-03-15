.SUFFIXES:
.SUFFIXES: .cpp .o

CXX = g++
CXXFLAGS = -Wall -g -O2
RM = rm -f
INCLUDES = -I../src
LIBS = ../src/libRVO.a
PROGRAMS = ExampleBlocks ExampleCircle ExampleRoadmap

all: $(PROGRAMS)

ExampleBlocks: ExampleBlocks.o
	$(RM) ExampleBlocks
	$(CXX) $(INCLUDES) $(CXXFLAGS) -o $@ ExampleBlocks.o $(LIBS)

ExampleCircle: ExampleCircle.o
	$(RM) ExampleCircle
	$(CXX) $(INCLUDES) $(CXXFLAGS) -o $@ ExampleCircle.o $(LIBS)

ExampleRoadmap: ExampleRoadmap.o
	$(RM) ExampleRoadmap
	$(CXX) $(INCLUDES) $(CXXFLAGS) -o $@ ExampleRoadmap.o $(LIBS)
	
.cpp.o:
	$(CXX) $(INCLUDES) $(CXXFLAGS) -c -o $@ $<

clean:
	$(RM) $(PROGRAMS)
	$(RM) *.o

.PHONY: all clean

.NOEXPORT:
