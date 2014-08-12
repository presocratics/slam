IDIR = ../../include

objects = main.o ourerr.o
CXXFLAGS = -Wall -Werror -std=c++03 -pedantic-errors
CXXFLAGS += -g
CXXFLAGS += $(shell pkg-config --cflags opencv)
CXXFLAGS += -I$(IDIR)
LDIR = ../../lib/
LIBS += $(LDIR)libARC.a
LIBS += $(shell pkg-config --libs opencv)

rotate: $(objects) 
	$(CXX) -o ../../bin/rotate $(objects) $(CXXFLAGS) $(LIBS)
%.o: %cpp
	$(CXX) $(CXXFLAGS) -o $@ -c $<  $(LIBS)
.PHONY: clean
clean:
	rm -f *.o
	rm -f ../../bin/rotate

