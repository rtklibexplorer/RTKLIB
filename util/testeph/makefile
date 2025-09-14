# makefile for diffeph dumpssr

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



all        : diffeph dumpssr

diffeph    : rtkcmn.o trace.o ephemeris.o sbas.o rinex.o preceph.o rtcm.o rtcm2.o rtcm3.o
diffeph    : ionex.o pntpos.o ppp.o ppp_ar.o stec.o qzslex.o
dumpssr    : rtcm.o rtcm3.o rtcm3e.o rtkcmn.o trace.o preceph.o rtcm2.o dumpssr.c
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

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
rtcm3e.o    : $(SRC)/rtklib.h $(SRC)/rtcm3e.c
	$(CC) -c $(CFLAGS) $(SRC)/rtcm3e.c
ionex.o    : $(SRC)/rtklib.h $(SRC)/ionex.c
	$(CC) -c $(CFLAGS) $(SRC)/ionex.c
pntpos.o   : $(SRC)/rtklib.h $(SRC)/pntpos.c
	$(CC) -c $(CFLAGS) $(SRC)/pntpos.c
ppp.o      : $(SRC)/rtklib.h $(SRC)/ppp.c
	$(CC) -c $(CFLAGS) $(SRC)/ppp.c
ppp_ar.o   : $(SRC)/rtklib.h $(SRC)/ppp_ar.c
	$(CC) -c $(CFLAGS) $(SRC)/ppp_ar.c
stec.o     : $(SRC)/rtklib.h $(SRC)/stec.c
	$(CC) -c $(CFLAGS) $(SRC)/stec.c
qzslex.o   : $(SRC)/rtklib.h $(SRC)/qzslex.c
	$(CC) -c $(CFLAGS) $(SRC)/qzslex.c

install:
	cp -f diffeph.exe /usr/local/bin
	cp -f dumpssr.exe /usr/local/bin

TEST_INPUTS := IONO00IGS1_ICD_example IONO00IGS1_BNC
TMPFILE1 :=$(shell mktemp)
TMPFILE2 :=$(shell mktemp)

test: $(TEST_INPUTS)

$(TEST_INPUTS):
	echo "Running test for $@"
	./dumpssr ../data/$@.rtcm3 | tail -8 > ${TMPFILE1}
	tail -8  ../data/$@.txt > ${TMPFILE2}
	diff ${TMPFILE1} ${TMPFILE2}

clean:
	rm -f dumpssr diffeph *.o *.stackdump *.trace *.out *.exe $(TMPFILE1) $(TMPFILE2)
