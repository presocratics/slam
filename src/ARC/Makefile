SRC = feature.cpp featureIO.cpp imagesensor.cpp slamsensor.cpp ourerr.cpp \
		Quaternion.cpp Sensors.cpp states.cpp view.cpp
OBJ = $(SRC:.cpp=.o)
OUT = libARC.a
IDIR = ../../include
CXXFLAGS = -Wall -Werror -std=c++03 -pedantic-errors
CXXFLAGS += -g
CXXFLAGS += $(shell pkg-config --cflags opencv)
CXXFLAGS += -I$(IDIR)
LDIR = ../../lib/

LIBS += $(shell pkg-config --libs opencv)

$(OUT): $(OBJ)
	ar rcs $(LDIR)$(OUT) $(OBJ)


.cpp.o:
	$(CXX) $(INCLUDES) $(CXXFLAGS) -c $< -o $@



.PHONY: clean
clean:
	rm -f *.o
	rm -f ../../lib/libARC.a

