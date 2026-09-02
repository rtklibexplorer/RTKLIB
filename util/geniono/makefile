# makefile for geniono

BINDIR   =../../bin
RTKLIBDIR=../../src

CFLAGS = -Wall -O3 -I$(RTKLIBDIR) -DTRACE
LDFLAGS = -lm

BIN = estiono gengrid genstec rcvdcb

all        : $(BIN)
estiono    : estiono.o rtkcmn.o trace.o ephemeris.o preceph.o sbas.o rinex.o
genstec    : genstec.o rtkcmn.o trace.o ephemeris.o preceph.o sbas.o rinex.o
gengrid    : gengrid.o rtkcmn.o trace.o preceph.o
geniono    : geniono.o rtkcmn.o trace.o preceph.o
rcvdcb     : rcvdcb.o rtkcmn.o trace.o ephemeris.o preceph.o sbas.o rinex.o

rtkcmn.o   : $(RTKLIBDIR)/rtklib.h $(RTKLIBDIR)/rtkcmn.c
	$(CC) -c $(CFLAGS) $(RTKLIBDIR)/rtkcmn.c
trace.o   : $(RTKLIBDIR)/rtklib.h $(RTKLIBDIR)/trace.c
	$(CC) -c $(CFLAGS) $(RTKLIBDIR)/trace.c
ephemeris.o: $(RTKLIBDIR)/ephemeris.c
	$(CC) -c $(CFLAGS) $(RTKLIBDIR)/ephemeris.c
preceph.o  : $(RTKLIBDIR)/preceph.c
	$(CC) -c $(CFLAGS) $(RTKLIBDIR)/preceph.c
sbas.o     : $(RTKLIBDIR)/sbas.c
	$(CC) -c $(CFLAGS) $(RTKLIBDIR)/sbas.c
rinex.o    : $(RTKLIBDIR)/rinex.c
	$(CC) -c $(CFLAGS) $(RTKLIBDIR)/rinex.c

OBSDIR=/proj/data_gsi/2011
NAVDIR=/proj/data/2011
STA=0225

OBS=$(OBSDIR)/001/$(STA)0010.11o $(OBSDIR)/002/$(STA)0020.11o $(OBSDIR)/003/$(STA)0030.11o
NAV=$(NAVDIR)/001/brdc0010.11n $(NAVDIR)/002/brdc0020.11n $(NAVDIR)/003/brdc0030.11n

test1:
	./genstec $(OBS) $(NAV) -a igs05.atx -o iono_$(STA).stec

test2:
	./gengrid "stec/iono_*.stec" -d stec_grid

install:
	cp -f $(BIN) $(BINDIR)

clean:
	rm -f $(BIN) *.o *.trace

