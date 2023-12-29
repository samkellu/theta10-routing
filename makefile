EXEC=demo
SRC=demo.cpp

compile:
	g++ $(SRC) -o $(EXEC) -lSDL2 -lSDL2main -g

clean:
	rm $(EXEC)
