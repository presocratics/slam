objects = stamp.o
CXXFLAGS = -Wall -Werror
CXXFLAGS += -g

sensor-emu-stamp-feature: $(objects)
	$(CXX) -o ../../bin/sensor-emu-stamp-feature $(objects) $(CXXFLAGS) $(LIBS)
%.o: %cpp
	$(CXX) $(CXXFLAGS) -o $@ -c $<  $(LIBS)
.PHONY: clean
clean:
	rm -f *.o
	rm -f ../../bin/sensor-emu-stamp-feature

