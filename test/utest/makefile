# makefile for rtklib unit test

SRC    = ../../src
#CFLAGS = -std=c99 -Wall -O3 -pedantic -I$(SRC) -DENAGLO
CFLAGS = -std=c99 -Wall -O3 -pedantic -I$(SRC) -DTRACE -DENAGLO -DENAGAL -DENAQZS
LDLIBS = -lm -llapack -lblas
CC = gcc

BIN    = t_matrix t_time t_coord t_rinex t_lambda t_atmos t_misc t_preceph t_gloeph \
t_geoid t_ppp t_ionex t_tle t_ssr_ion

all        : $(BIN)
t_matrix   : t_matrix.o rtkcmn.o trace.o preceph.o
t_time     : t_time.o rtkcmn.o trace.o preceph.o
t_coord    : t_coord.o rtkcmn.o trace.o geoid.o preceph.o
t_rinex    : t_rinex.o rtkcmn.o trace.o rinex.o preceph.o
t_lambda   : t_lambda.o rtkcmn.o trace.o lambda.o preceph.o
t_atmos    : t_atmos.o rtkcmn.o trace.o preceph.o
t_misc     : t_misc.o rtkcmn.o trace.o preceph.o
t_preceph  : t_preceph.o rtkcmn.o trace.o preceph.o rinex.o ephemeris.o sbas.o
t_gloeph   : t_gloeph.o rtkcmn.o trace.o rinex.o ephemeris.o sbas.o preceph.o
t_geoid    : t_geoid.o rtkcmn.o trace.o preceph.o geoid.o
t_ppp      : t_ppp.o rtkcmn.o trace.o ephemeris.o preceph.o sbas.o ionex.o pntpos.o ppp.o ppp_ar.o
t_ppp      : lambda.o tides.o
t_ionex    : t_ionex.o rtkcmn.o trace.o preceph.o ionex.o
t_tle      : t_tle.o rtkcmn.o trace.o rinex.o ephemeris.o sbas.o preceph.o tle.o
t_ssr_ion  : t_ssr_ion.o rtkcmn.o trace.o preceph.o ephemeris.o sbas.o ionex.o pntpos.o ssr_ion.o

rtkcmn.o   : $(SRC)/rtklib.h $(SRC)/rtkcmn.c
	$(CC) -c $(CFLAGS) $(SRC)/rtkcmn.c
trace.o   : $(SRC)/rtklib.h $(SRC)/trace.c
	$(CC) -c $(CFLAGS) $(SRC)/trace.c
rinex.o    : $(SRC)/rtklib.h $(SRC)/rinex.c
	$(CC) -c $(CFLAGS) $(SRC)/rinex.c
rtkpos.o   : $(SRC)/rtklib.h $(SRC)/rtkpos.c
	$(CC) -c $(CFLAGS) $(SRC)/rtkpos.c
lambda.o   : $(SRC)/rtklib.h $(SRC)/lambda.c
	$(CC) -c $(CFLAGS) $(SRC)/lambda.c
geoid.o    : $(SRC)/rtklib.h $(SRC)/geoid.c
	$(CC) -c $(CFLAGS) $(SRC)/geoid.c
ephemeris.o: $(SRC)/rtklib.h $(SRC)/ephemeris.c
	$(CC) -c $(CFLAGS) $(SRC)/ephemeris.c
sbas.o     : $(SRC)/rtklib.h $(SRC)/sbas.c
	$(CC) -c $(CFLAGS) $(SRC)/sbas.c
preceph.o  : $(SRC)/rtklib.h $(SRC)/preceph.c
	$(CC) -c $(CFLAGS) $(SRC)/preceph.c
ppp.o      : $(SRC)/rtklib.h $(SRC)/ppp.c
	$(CC) -c $(CFLAGS) $(SRC)/ppp.c
ppp_ar.o   : $(SRC)/rtklib.h $(SRC)/ppp_ar.c
	$(CC) -c $(CFLAGS) $(SRC)/ppp_ar.c
pntpos.o   : $(SRC)/rtklib.h $(SRC)/pntpos.c
	$(CC) -c $(CFLAGS) $(SRC)/pntpos.c
ionex.o    : $(SRC)/rtklib.h $(SRC)/ionex.c
	$(CC) -c $(CFLAGS) $(SRC)/ionex.c
tle.o      : $(SRC)/rtklib.h $(SRC)/tle.c
	$(CC) -c $(CFLAGS) $(SRC)/tle.c
tides.o   : $(SRC)/rtklib.h $(SRC)/tides.c
	$(CC) -c $(CFLAGS) $(SRC)/tides.c
ssr_ion.o : $(SRC)/rtklib.h $(SRC)/ssr_ion.c
	$(CC) -c $(CFLAGS) $(SRC)/ssr_ion.c

utest : utest1 utest2 utest3 utest4 utest5 utest6 utest7 utest8
utest : utest9 utest10 utest11 utest12 utest14
utest : utest_ssr_ion

utest1 :
	./t_matrix  > utest1.out
utest2 :
	./t_time    > utest2.out
utest3 :
	./t_coord   > utest3.out
utest4 :
	./t_rinex   > utest4.out
utest5 :
	./t_lambda  > utest5.out
utest6 :
	./t_atmos   > utest6.out
utest7 :
	./t_misc    > utest7.out
utest8 :
	./t_preceph > utest8.out
utest9 :
	./t_gloeph  > utest9.out
utest10 :
#	./t_geoid   > utest10.out
utest11 :
	./t_ppp     > utest11.out
utest12 :
	./t_ionex   > utest12.out
utest14 :
	./t_tle     > utest14.out
utest_ssr_ion :
	./t_ssr_ion > utest_ssr_ion.out

clean :
	rm -f *.o *.out *.exe $(BIN) *.stackdump gmon.out

