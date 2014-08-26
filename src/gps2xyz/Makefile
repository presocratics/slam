CXXFLAGS = -std=c++11 -pedantic-errors
CXXFLAGS += -g
CXXFLAGS += $(shell pkg-config --cflags opencv)
LIBS = -lm
LIBS += $(shell pkg-config -libs opencv)

gps2xyz: main.o
	$(CXX) -o ../../bin/gps2xyz main.o $(CXXFLAGS) $(LIBS)

.cpp.o:
	$(CXX) $(CXXFLAGS) -c $< -o $@

.PHONY: clean
clean:
	rm -f *.o
	rm -f ../../track

