CC = gcc
CFLAGS = -I.
DEPS = drone.h vec2.h video.h channel.h


default: 
	$(CC) main.c vec2.c drone.c -std=c99 -o ./build/vocabs2 -lm -Wall -Wfatal-errors -Wextra

windows:
	gcc video.c main.c vec2.c drone.c channel.c -I./SDL2 -L./lib -std=c11 -o ./build/vocabs2.exe -lmingw32 -lm -Wall -lSDL2main -lSDL2 -lSDL2_image -w -Wall -Wfatal-errors -Wextra
clean:
	rm ./build/vocabs2

