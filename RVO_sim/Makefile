.SUFFIXES:
.SUFFIXES: .cpp .o

CXX = g++
CXXFLAGS = -Wall -g -O2
RM = rm -f
INCLUDES = -I./RVO2/src
LIBS = ./RVO2/src/libRVO.a
PROGRAMS = test parser sim

all: $(PROGRAMS)

sim: sim.o parser.o
	$(RM) sim
	$(CXX) $(INCLUDES) $(CXXFLAGS) -o $@ sim.o parser.o $(LIBS)

test: test.o parser.o
	$(RM) test
	$(CXX) $(INCLUDES) $(CXXFLAGS) -o $@ test.o $(LIBS)

parser: parser.o
	$(RM) parser
	$(CXX) $(INCLUDES) $(CXXFLAGS) -o $@ parser.o $(LIBS)
	
.cpp.o:
	$(CXX) $(INCLUDES) $(CXXFLAGS) -c -o $@ $<

clean:
	$(RM) $(PROGRAMS)
	$(RM) *.o

.PHONY: all clean

.NOEXPORT:
