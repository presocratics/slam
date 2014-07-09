objects = feature.o featureIO.o \
		  imagesensor.o main.o \
		  ourerr.o Quaternion.o\
		  Sensors.o states.o \
		  view.o 
CXXFLAGS = -Wall -Werror
CXXFLAGS += -g
CXXFLAGS += $(shell pkg-config --cflags opencv)
LIBS += $(shell pkg-config --libs opencv)

slam: $(objects)
	$(CXX) -o ../../bin/slam $(objects) $(CXXFLAGS) $(LIBS)
%.o: %cpp
	$(CXX) $(CXXFLAGS) -o $@ -c $<  $(LIBS)
.PHONY: clean
clean:
	rm -f *.o

