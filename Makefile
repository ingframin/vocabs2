default: main.o vec2.o drone.o
	gcc main.o drone.o vec2.o -o ./build/vocabs2 -lm -Wall 

clean:
	rm ./build/vocabs2
	rm *.o
