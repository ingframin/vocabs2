CC = gcc
CFLAGS = -I.
DEPS = drone.h vec2.h video.h

%.o: %c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

default: video.o main.o vec2.o drone.o 
	$(CC) video.o main.o drone.o vec2.o -o ./build/vocabs2 -lm -Wall -lSDL2main -lSDL2 -lSDL2_image

clean:
	rm ./build/vocabs2
	rm *.o
