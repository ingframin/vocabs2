CC = gcc
CFLAGS = -I.
DEPS = drone.h vec2.h video.h channel.h


default: 
	$(CC) video.c main.c vec2.c drone.c channel.c -o ./build/vocabs2 -lm -Wall -lSDL2main -lSDL2 -lSDL2_image

clean:
	rm ./build/vocabs2
