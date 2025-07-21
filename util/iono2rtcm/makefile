# makefile for iono2rtcm

BINDIR = /usr/local/bin
SRC    = ../../src


# Build configuration options
DEBUG   ?= 0
PROFILE ?= 0

# Base compiler and linker flags
CFLAGS = -std=c99 -Wall -pedantic -I$(SRC) -DTRACE -DENAQZS -DENAGLO -DEXTLEX
LDFLAGS = -lm

# Add optimization/debug flags based on configuration
ifeq ($(DEBUG),1)
    CFLAGS += -g -O0 -DDEBUG
    LDFLAGS += -g
else
    CFLAGS += -O3 -DNDEBUG
endif

# Add profiling flags if requested
ifeq ($(PROFILE),1)
    CFLAGS += -pg
    LDFLAGS += -pg
endif

iono2rtcm   : trace.o rtkcmn.o rinex.o rtcm.o rtcm2.o rtcm3.o rtcm3e.o main.c
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

rtkcmn.o   : $(SRC)/rtkcmn.c
	$(CC) -c $(CFLAGS) $(SRC)/rtkcmn.c
trace.o   : $(SRC)/rtklib.h $(SRC)/trace.c
	$(CC) -c $(CFLAGS) $(SRC)/trace.c
rinex.o    : $(SRC)/rinex.c
	$(CC) -c $(CFLAGS) $(SRC)/rinex.c
rtcm.o     : $(SRC)/rtcm.c
	$(CC) -c $(CFLAGS) $(SRC)/rtcm.c
rtcm2.o    : $(SRC)/rtcm2.c
	$(CC) -c $(CFLAGS) $(SRC)/rtcm2.c
rtcm3.o    : $(SRC)/rtcm3.c
	$(CC) -c $(CFLAGS) $(SRC)/rtcm3.c
rtcm3e.o   : $(SRC)/rtcm3e.c
	$(CC) -c $(CFLAGS) $(SRC)/rtcm3e.c

install:
	cp iono2rtcm $(BINDIR)

clean:
	rm -f iono2rtcm iono2rtcm.exe *.o

test: test1 test2

test1:
	./iono2rtcm ../data/IONO00IGS1_ICD_example.txt > test.rtcm3
	diff -u test.rtcm3 ../data/IONO00IGS1_ICD_example.rtcm3
test2:
	./iono2rtcm ../data/IONO00IGS1_BNC.txt > test.rtcm3
	diff -u test.rtcm3 ../data/IONO00IGS1_BNC.rtcm3
