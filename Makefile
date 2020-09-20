CC = gcc-10
CFLAGS = -I.
DEPS = drone.h vec2.h video.h channel.h


default: 
	$(CC) main.c vec2.c drone.c -std=c99 -o ./build/vocabs2 -lm -Wall -Wfatal-errors -Wextra -fopenmp -finline-functions

clean:
	rm ./build/vocabs2

