objects = img2body.o 
CXXFLAGS = -Wall -Werror
CXXFLAGS += -g
CXXFLAGS += $(shell pkg-config --cflags opencv)
CXXFLAGS += -I../../include
LIBS += ../../lib/libARC.a
LIBS += $(shell pkg-config --libs opencv)

img2body: $(objects) libARC.a
	$(CXX) -o ../../bin/img2body $(objects) $(CXXFLAGS) $(LIBS)

%.o: %cpp
	$(CXX) $(CXXFLAGS) -o $@ -c $<  $(LIBS)

libARC.a:
	$(MAKE) -C ../ARC

.PHONY: clean
clean:
	rm -f *.o
	rm -f ../../bin/img2body

