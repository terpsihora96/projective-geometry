CXX     	= g++
CXXFLAGS 	= -std=c++17 -I include
LDLIBS  	= -lstdc++fs `pkg-config opencv --cflags --libs`
WFLAGS 		= -Wall -Wextra -O3

fix_distortion: fix_distortion.cpp
	$(CXX) -o $@ $^ $(CXXFLAGS) $(WFLAGS) $(LDLIBS) 
