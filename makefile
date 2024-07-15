EXEC=demo
SRC=graphics.cpp algorithms.cpp demo.cpp

compile:
	g++ $(SRC) -o $(EXEC) -lSDL2 -lSDL2main -g -DDRAW_BOUNDS

random:
	g++ $(SRC) -o $(EXEC) -lSDL2 -lSDL2main -g -DDRAW_BOUNDS -DRANDOM_START

six:
	g++ $(SRC) -o $(EXEC) -lSDL2 -lSDL2main -g -DDRAW_BOUNDS -DRANDOM_START -DNUM_CONES=6


clean:
	rm $(EXEC)
