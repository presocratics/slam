objects = img2body.o Quaternion.o
CXXFLAGS = -Wall -Werror
CXXFLAGS += -g
CXXFLAGS += $(shell pkg-config --cflags opencv)
LIBS += $(shell pkg-config --libs opencv)

img2body: $(objects)
	$(CXX) -o ../../bin/img2body $(objects) $(CXXFLAGS) $(LIBS)
%.o: %cpp
	$(CXX) $(CXXFLAGS) -o $@ -c $<  $(LIBS)
.PHONY: clean
clean:
	rm -f *.o

