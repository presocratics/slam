objects = main.o ourerr.o
CXXFLAGS = -Wall -Werror -std=c99
CXXFLAGS += -g

sensor-emu: $(objects)
	$(CXX) -o ../../bin/sensor-emu $(objects) $(CXXFLAGS) $(LIBS)
%.o: %cpp
	$(CXX) $(CXXFLAGS) -o $@ -c $<  $(LIBS)
.PHONY: clean
clean:
	rm -f *.o

