# makefile for rtkrcv

BINDIR = /usr/local/bin
SRC    = ../../../../src

CTARGET= -DTRACE -DENAGLO -DENAQZS -DENACMP -DENAGAL -DENAIRN -DNFREQ=3 -DNEXOBS=3 -DSVR_REUSEADDR
#CTARGET= -DENAGLO -DENAQZS -DENACMP -DENAGAL -DENAIRN -DNFREQ=3 -DIERS_MODEL -DSVR_REUSEADDR

CFLAGS = -std=c99 -Wall -O3 -pedantic -Wno-unused-but-set-variable -I$(SRC) -I.. -DTRACE $(CTARGET) -g
LDLIBS  = -lm -lrt -lpthread
#LDLIBS  = ../../../lib/iers/gcc/iers.a -lm -lrt -lpthread

all        : rtkrcv
rtkrcv     : rtkrcv.o vt.o rtkcmn.o trace.o rtksvr.o rtkpos.o geoid.o solution.o lambda.o
rtkrcv     : sbas.o stream.o rcvraw.o rtcm.o preceph.o options.o pntpos.o ppp.o ppp_ar.o
rtkrcv     : novatel.o ublox.o crescent.o skytraq.o javad.o nvs.o binex.o
rtkrcv     : rt17.o ephemeris.o rinex.o ionex.o rtcm2.o rtcm3.o rtcm3e.o
rtkrcv     : tides.o septentrio.o swiftnav.o


rtkrcv.o   : ../rtkrcv.c
	$(CC) -c $(CFLAGS) ../rtkrcv.c
vt.o       : ../vt.c
	$(CC) -c $(CFLAGS) ../vt.c
rtkcmn.o   : $(SRC)/rtkcmn.c
	$(CC) -c $(CFLAGS) $(SRC)/rtkcmn.c
trace.o   : $(SRC)/trace.c
	$(CC) -c $(CFLAGS) $(SRC)/trace.c
rtksvr.o   : $(SRC)/rtksvr.c
	$(CC) -c $(CFLAGS) $(SRC)/rtksvr.c
rtkpos.o   : $(SRC)/rtkpos.c
	$(CC) -c $(CFLAGS) $(SRC)/rtkpos.c
geoid.o    : $(SRC)/geoid.c
	$(CC) -c $(CFLAGS) $(SRC)/geoid.c
solution.o : $(SRC)/solution.c
	$(CC) -c $(CFLAGS) $(SRC)/solution.c
lambda.o   : $(SRC)/lambda.c
	$(CC) -c $(CFLAGS) $(SRC)/lambda.c
sbas.o     : $(SRC)/sbas.c
	$(CC) -c $(CFLAGS) $(SRC)/sbas.c
stream.o   : $(SRC)/stream.c
	$(CC) -c $(CFLAGS) $(SRC)/stream.c
rcvraw.o : $(SRC)/rcvraw.c
	$(CC) -c $(CFLAGS) $(SRC)/rcvraw.c
rtcm.o     : $(SRC)/rtcm.c
	$(CC) -c $(CFLAGS) $(SRC)/rtcm.c
rtcm2.o    : $(SRC)/rtcm2.c
	$(CC) -c $(CFLAGS) $(SRC)/rtcm2.c
rtcm3.o    : $(SRC)/rtcm3.c
	$(CC) -c $(CFLAGS) $(SRC)/rtcm3.c
rtcm3e.o   : $(SRC)/rtcm3e.c
	$(CC) -c $(CFLAGS) $(SRC)/rtcm3e.c
preceph.o  : $(SRC)/preceph.c
	$(CC) -c $(CFLAGS) $(SRC)/preceph.c
options.o  : $(SRC)/options.c
	$(CC) -c $(CFLAGS) $(SRC)/options.c
pntpos.o   : $(SRC)/pntpos.c
	$(CC) -c $(CFLAGS) $(SRC)/pntpos.c
ppp.o      : $(SRC)/ppp.c
	$(CC) -c $(CFLAGS) $(SRC)/ppp.c
ppp_ar.o   : $(SRC)/ppp_ar.c
	$(CC) -c $(CFLAGS) $(SRC)/ppp_ar.c
novatel.o  : $(SRC)/rcv/novatel.c
	$(CC) -c $(CFLAGS) $(SRC)/rcv/novatel.c
ublox.o    : $(SRC)/rcv/ublox.c
	$(CC) -c $(CFLAGS) $(SRC)/rcv/ublox.c
swiftnav.o : $(SRC)/rcv/swiftnav.c
	$(CC) -c $(CFLAGS) $(SRC)/rcv/swiftnav.c
crescent.o : $(SRC)/rcv/crescent.c
	$(CC) -c $(CFLAGS) $(SRC)/rcv/crescent.c
skytraq.o  : $(SRC)/rcv/skytraq.c
	$(CC) -c $(CFLAGS) $(SRC)/rcv/skytraq.c
javad.o    : $(SRC)/rcv/javad.c
	$(CC) -c $(CFLAGS) $(SRC)/rcv/javad.c
nvs.o      : $(SRC)/rcv/nvs.c
	$(CC) -c $(CFLAGS) $(SRC)/rcv/nvs.c
binex.o    : $(SRC)/rcv/binex.c
	$(CC) -c $(CFLAGS) $(SRC)/rcv/binex.c
rt17.o     : $(SRC)/rcv/rt17.c
	$(CC) -c $(CFLAGS) $(SRC)/rcv/rt17.c
ephemeris.o: $(SRC)/ephemeris.c
	$(CC) -c $(CFLAGS) $(SRC)/ephemeris.c
rinex.o    : $(SRC)/rinex.c
	$(CC) -c $(CFLAGS) $(SRC)/rinex.c
ionex.o    : $(SRC)/ionex.c
	$(CC) -c $(CFLAGS) $(SRC)/ionex.c
tides.o    : $(SRC)/tides.c
	$(CC) -c $(CFLAGS) $(SRC)/tides.c
septentrio.o: $(SRC)/rcv/septentrio.c
	$(CC) -c $(CFLAGS) $(SRC)/rcv/septentrio.c

rtkrcv.o   : $(SRC)/rtklib.h $(SRC)/trace.h ../vt.h
rtkcmn.o   : $(SRC)/rtklib.h $(SRC)/trace.h
rtksvr.o   : $(SRC)/rtklib.h $(SRC)/trace.h
rtkpos.o   : $(SRC)/rtklib.h $(SRC)/trace.h
geoid.o    : $(SRC)/rtklib.h $(SRC)/trace.h
solution.o : $(SRC)/rtklib.h $(SRC)/trace.h
lambda.o   : $(SRC)/rtklib.h $(SRC)/trace.h
sbas.o     : $(SRC)/rtklib.h $(SRC)/trace.h
rcvraw.o   : $(SRC)/rtklib.h $(SRC)/trace.h
rtcm.o     : $(SRC)/rtklib.h $(SRC)/trace.h
rtcm2.o    : $(SRC)/rtklib.h $(SRC)/trace.h
rtcm3.o    : $(SRC)/rtklib.h $(SRC)/trace.h
rtcm3e.o   : $(SRC)/rtklib.h $(SRC)/trace.h
preceph.o  : $(SRC)/rtklib.h $(SRC)/trace.h
options.o  : $(SRC)/rtklib.h $(SRC)/trace.h
pntpos.o   : $(SRC)/rtklib.h $(SRC)/trace.h
ppp.o      : $(SRC)/rtklib.h $(SRC)/trace.h
ppp_ar.o   : $(SRC)/rtklib.h $(SRC)/trace.h
novatel.o  : $(SRC)/rtklib.h $(SRC)/trace.h
ublox.o    : $(SRC)/rtklib.h $(SRC)/trace.h
swiftnav.o : $(SRC)/rtklib.h $(SRC)/trace.h
crescent.o : $(SRC)/rtklib.h $(SRC)/trace.h
skytraq.o  : $(SRC)/rtklib.h $(SRC)/trace.h
javad.o    : $(SRC)/rtklib.h $(SRC)/trace.h
nvs.o      : $(SRC)/rtklib.h $(SRC)/trace.h
binex.o    : $(SRC)/rtklib.h $(SRC)/trace.h
rt17.o     : $(SRC)/rtklib.h $(SRC)/trace.h
septentrio.o: $(SRC)/rtklib.h $(SRC)/trace.h
ephemeris.o: $(SRC)/rtklib.h $(SRC)/trace.h
rinex.o    : $(SRC)/rtklib.h $(SRC)/trace.h
ionex.o    : $(SRC)/rtklib.h $(SRC)/trace.h
tides.o    : $(SRC)/rtklib.h $(SRC)/trace.h

install:
	cp rtkrcv $(BINDIR)

test1:
	./rtkrcv -t 4 -m 52001 -t 4

test2:
	./rtkrcv -p 2105 -m 52001

test3:
	./rtkrcv -o ../rtk_pb.conf

clean:
	rm -f rtkrcv rtkrcv.exe rtkrcv.nav *.o *.out *.trace

