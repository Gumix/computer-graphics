CXX = g++
IDIR = /opt/local/include/SDL2/
LDIR = /opt/local/lib/
LIBS = -lSDL2
CFLAGS = -Wall -march=native -Ofast -Wa,-q
NAME = graphics
SRC = $(NAME).cpp
OBJ = $(NAME).o

$(NAME): $(OBJ)
	$(CXX) -o $@ $^ -L $(LDIR) $(LIBS)

$(OBJ): $(SRC)
	$(CXX) -c -o $@ $^ -I $(IDIR) $(CFLAGS)

.PHONY: clean

clean:
	rm $(NAME) $(OBJ)
