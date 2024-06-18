EXEC=demo
SRC=graphics.cpp algorithms.cpp demo.cpp

compile:
	g++ $(SRC) -o $(EXEC) -lSDL2 -lSDL2main -g -DDRAW_BOUNDS

clean:
	rm $(EXEC)
