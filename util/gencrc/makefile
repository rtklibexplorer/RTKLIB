# makefile for gencrc

CFLAGS = -std=c99 -Wall -O3 -pedantic
LDLIBS  = -lm

all        : gencrc genxor genmsk
gencrc     : gencrc.o
genxor     : genxor.o
genmsk     : genmsk.o

clean:
	rm -f gencrc genxor genmsk *.o

test:
	./gencrc -16 > crc16.c
	./gencrc -24 > crc24.c
	./genxor > xor.c
	./genmsk > msk.c
