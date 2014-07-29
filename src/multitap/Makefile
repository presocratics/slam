objects = main.o 
CXXFLAGS = -Wall -Werror
CXXFLAGS += -g
CXXFLAGS += -I../../include
LIBS += ../../lib/libARC.a

multitap: $(objects) libARC.a
	$(CXX) -o ../../bin/multitap $(objects) $(CXXFLAGS) $(LIBS)
%.o: %cpp
	$(CXX) $(CXXFLAGS) -o $@ -c $<  $(LIBS)
libARC.a:
	$(MAKE) -C ../ARC
.PHONY: clean
clean:
	rm -f *.o
	rm -f ../../bin/multitap

