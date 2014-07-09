objects = euler2qbw.o
CXXFLAGS = -Wall -Werror
CXXFLAGS += -g

euler2qbw: $(objects)
	$(CXX) -o ../../bin/euler2qbw $(objects) $(CXXFLAGS) $(LIBS)
%.o: %cpp
	$(CXX) $(CXXFLAGS) -o $@ -c $<  $(LIBS)
.PHONY: clean
clean:
	rm -f *.o

