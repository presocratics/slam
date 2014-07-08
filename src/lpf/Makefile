objects = lpf.o 
CXXFLAGS= -g

lpf: $(objects)
	$(CXX) -o ../../bin/lpf $(objects) $(CXXFLAGS) $(LIBS)
%.o: %cpp
	$(CXX) $(CXXFLAGS) -o $@ -c $<  $(LIBS)
.PHONY: clean
clean:
	rm -f *.o

