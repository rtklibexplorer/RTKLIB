# makefile for margelog

BINDIR = ../../bin
SRC    = ../../src
OPTIONS= -DTRACE # -DENAGLO -DENAQZS -DENAGAL -DENACMP -DENAIRN -DNFREQ=3 -DNEXOBS=3
CFLAGS = -std=c99 -Wall -O3 -pedantic -I$(SRC) $(OPTIONS)
LDLIBS  = -lm

margelog   : margelog.o rtkcmn.o trace.o sofa.o

rnx2rtcm.o : rnx2rtcm.c
	$(CC) -c $(CFLAGS) rnx2rtcm.c
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
sofa.o   : $(SRC)/sofa.c
	$(CC) -c $(CFLAGS) $(SRC)/sofa.c

install:
	cp margelog $(BINDIR)

clean:
	rm -f margelog margelog.exe *.o
