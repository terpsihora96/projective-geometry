CXX     	= g++
CXXFLAGS 	= -std=c++17 -I include
LIBS		= -lglut -lGLU -lGL
WFLAGS 		= -Wall -Wextra -O3

slerp: slerp.cpp
	$(CXX) -o $@ $^ $(CXXFLAGS) $(WFLAGS) $(LIBS)
