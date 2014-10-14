IDIR = ../../include

CXXFLAGS = -std=c++03 -pedantic-errors
CXXFLAGS += -g
CXXFLAGS += $(shell pkg-config --cflags opencv)
CXXFLAGS += -I$(IDIR)
LDIR = ../../lib/
LIBS += $(LDIR)libARC.a
LIBS += $(shell pkg-config --libs opencv)

slam: main.o libARC.a
	$(CXX) -o ../../bin/slam main.o $(CXXFLAGS) $(LIBS) 

.cpp.o:
	$(CXX) $(CXXFLAGS) -c $< -o $@

libARC.a:
	$(MAKE) -C ../ARC

.PHONY: clean
clean:
	rm -f *.o
	rm -f ../../bin/slam

