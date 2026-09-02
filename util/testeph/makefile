# makefile for diffeph dumpssr

BINDIR = ../../bin
SRC    = ../../src
CFLAGS = -std=c99 -Wall -O3 -pedantic -I$(SRC) -DTRACE -DENAGLO -DENAQZS -DENAGAL -DENACMP -DENAIRN
LDFLAGS = -lm

all        : dumpssr # diffeph dumpssr

diffeph    : diffeph.o rtkcmn.o trace.o ephemeris.o sbas.o rinex.o preceph.o rtcm.o rtcm2.o rtcm3.o ionex.o pntpos.o ppp.o ppp_ar.o
dumpssr    : dumpssr.o rtkcmn.o trace.o preceph.o rtcm.o rtcm2.o rtcm3.o rinex.o rtcm.o rtcm2.o rtcm3.o

diffeph.o   : diffeph.c
	$(CC) -c $(CFLAGS) diffeph.c
dumpssr.o   : dumpssr.c
	$(CC) -c $(CFLAGS) dumpssr.c

rtkcmn.o   : $(SRC)/rtklib.h $(SRC)/rtkcmn.c
	$(CC) -c $(CFLAGS) $(SRC)/rtkcmn.c
trace.o   : $(SRC)/rtklib.h $(SRC)/trace.c
	$(CC) -c $(CFLAGS) $(SRC)/trace.c
ephemeris.o: $(SRC)/rtklib.h $(SRC)/ephemeris.c
	$(CC) -c $(CFLAGS) $(SRC)/ephemeris.c
sbas.o     : $(SRC)/rtklib.h $(SRC)/sbas.c
	$(CC) -c $(CFLAGS) $(SRC)/sbas.c
rinex.o    : $(SRC)/rtklib.h $(SRC)/rinex.c
	$(CC) -c $(CFLAGS) $(SRC)/rinex.c
preceph.o  : $(SRC)/rtklib.h $(SRC)/preceph.c
	$(CC) -c $(CFLAGS) $(SRC)/preceph.c
rtcm.o     : $(SRC)/rtklib.h $(SRC)/rtcm.c
	$(CC) -c $(CFLAGS) $(SRC)/rtcm.c
rtcm2.o    : $(SRC)/rtklib.h $(SRC)/rtcm2.c
	$(CC) -c $(CFLAGS) $(SRC)/rtcm2.c
rtcm3.o    : $(SRC)/rtklib.h $(SRC)/rtcm3.c
	$(CC) -c $(CFLAGS) $(SRC)/rtcm3.c
ionex.o    : $(SRC)/rtklib.h $(SRC)/ionex.c
	$(CC) -c $(CFLAGS) $(SRC)/ionex.c
pntpos.o   : $(SRC)/rtklib.h $(SRC)/pntpos.c
	$(CC) -c $(CFLAGS) $(SRC)/pntpos.c
ppp.o      : $(SRC)/rtklib.h $(SRC)/ppp.c
	$(CC) -c $(CFLAGS) $(SRC)/ppp.c
ppp_ar.o   : $(SRC)/rtklib.h $(SRC)/ppp_ar.c
	$(CC) -c $(CFLAGS) $(SRC)/ppp_ar.c

install:
	cp diffeph $(BINDIR)
	cp dumpssr $(BINDIR)

clean:
	rm -f diffeph dumpssr *.o
