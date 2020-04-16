CC = gcc
CFLAGS = -I.
DEPS = drone.h vec2.h

%.o: %c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

default: main.o vec2.o drone.o
	$(CC) main.o drone.o vec2.o -o ./build/vocabs2 -lm -Wall 

clean:
	rm ./build/vocabs2
	rm *.o
