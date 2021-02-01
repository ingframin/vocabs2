CC = clang
CFLAGS = -I.
DEPS = drone.h vec2.h video.h channel.h


default: 
	$(CC) main.c vec2.c drone.c -std=c11 -o ./build/vocabs2 -lm -Wall -Wfatal-errors -Wextra -finline-functions 

clean:
	rm ./build/vocabs2

