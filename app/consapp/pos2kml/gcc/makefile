# makefile for pos2kml

BINDIR = /usr/local/bin
SRC    = ../../../../src
CFLAGS = -std=c99 -Wall -O3 -pedantic -I$(SRC) -DTRACE
LDLIBS  = -lm -lrt

pos2kml    : pos2kml.o convkml.o convgpx.o solution.o geoid.o rtkcmn.o trace.o preceph.o

pos2kml.o  : ../pos2kml.c
	$(CC) -c $(CFLAGS) ../pos2kml.c
convkml.o  : $(SRC)/convkml.c
	$(CC) -c $(CFLAGS) $(SRC)/convkml.c
convgpx.o  : $(SRC)/convgpx.c
	$(CC) -c $(CFLAGS) $(SRC)/convgpx.c
solution.o : $(SRC)/solution.c
	$(CC) -c $(CFLAGS) $(SRC)/solution.c
geoid.o    : $(SRC)/geoid.c
	$(CC) -c $(CFLAGS) $(SRC)/geoid.c
rtkcmn.o   : $(SRC)/rtkcmn.c
	$(CC) -c $(CFLAGS) $(SRC)/rtkcmn.c
trace.o   : $(SRC)/trace.c
	$(CC) -c $(CFLAGS) $(SRC)/trace.c
preceph.o  : $(SRC)/preceph.c
	$(CC) -c $(CFLAGS) $(SRC)/preceph.c

pos2kml.o  : $(SRC)/rtklib.h $(SRC)/trace.h
convkml.o  : $(SRC)/rtklib.h $(SRC)/trace.h
convgpx.o  : $(SRC)/rtklib.h $(SRC)/trace.h
solution.o : $(SRC)/rtklib.h $(SRC)/trace.h
geoid.o    : $(SRC)/rtklib.h $(SRC)/trace.h
rtkcmn.o   : $(SRC)/rtklib.h $(SRC)/trace.h
preceph.o  : $(SRC)/rtklib.h $(SRC)/trace.h

install:
	cp pos2kml $(BINDIR)

clean:
	rm -f pos2kml pos2kml.exe *.o
