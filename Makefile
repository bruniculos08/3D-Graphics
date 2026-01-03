SDLFLAGS = -lSDL2
FILE ?= "render.cpp"

run:
	g++ $(SDLFLAGS) $(FILE)
	./a.out
	rm ./a.out