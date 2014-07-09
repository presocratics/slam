objects = main.o ourerr.o
CXXFLAGS = -Wall -Werror
CXXFLAGS += -g

multitap: $(objects)
	$(CXX) -o ../../bin/multitap $(objects) $(CXXFLAGS) $(LIBS)
%.o: %cpp
	$(CXX) $(CXXFLAGS) -o $@ -c $<  $(LIBS)
.PHONY: clean
clean:
	rm -f *.o

