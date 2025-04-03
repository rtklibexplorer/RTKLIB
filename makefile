#-------------------------------------------------------------------------------
#
# makefile
#
# Purpose:
#
#   Linux make file for software libraries
#
# Last modified:
#
#   2021/06/10  AHA  Created
#   2022/11/02  AHA  Added clean-up of old Qt makefiles
#   2022/11/22  AHA  Restored compilation of iers lib
#   2025/03/31  AHA  Switch to cmake
#
#-------------------------------------------------------------------------------

# Paths

RTKLIB     = .
RTKLIB_bin = $(RTKLIB)/bin
RTKLIB_bld = $(RTKLIB)/build

IERS       = $(RTKLIB)/lib/iers/gcc

GENCRC     = $(RTKLIB)/util/gencrc
GENIONO    = $(RTKLIB)/util/geniono
LOGFILE    = $(RTKLIB)/util/logfile
RNX2RTCM   = $(RTKLIB)/util/rnx2rtcm
SIMOBS     = $(RTKLIB)/util/simobs/gcc
TESTEPH    = $(RTKLIB)/util/testeph

UTEST      = $(RTKLIB)/test/utest

# Get number of parallel build jobs

ifneq ($(shell which nproc 2> /dev/null),)
  NJOBS = $(shell nproc)
else
  NJOBS = 1
endif

# Parallel compilation on Linux and Cygwin

PMAKE = make
OS = $(shell uname -o)
ifeq ($(OS),GNU/Linux)
  PMAKE = make -j $(NJOBS)
endif
ifeq ($(OS),Cygwin)
  PMAKE = make -j $(NJOBS)
endif

# Operating system dependent settings for qmake

ifeq ($(shell which qmake-qt5  2> /dev/null),)
  QMAKE = qmake
else
  QMAKE = qmake-qt5  # if qmake is not available (e.g. openSuse)
endif

# Targets

all: init \
	 iers_ apps_

utils: gencrc_ logfile_ rnx2rtcm_ simobs_ # geniono_ testeph_

utest: utest_

# Create directory tree

init:
	if [ ! -d "$(RTKLIB_bin)" ]; then mkdir -p $(RTKLIB_bin); fi
	if [ ! -d "$(RTKLIB_bld)" ]; then mkdir -p $(RTKLIB_bld); fi

iers_:
	cd $(IERS); $(PMAKE)

utest_:
	cd $(UTEST); $(PMAKE)

apps_:
	cd $(RTKLIB_bld); cmake ..; $(PMAKE)

gencrc_:
	cd $(GENCRC); $(PMAKE)

geniono_:
	cd $(GENIONO); $(PMAKE)

logfile_:
	cd $(LOGFILE); $(PMAKE)

rnx2rtcm_:
	cd $(RNX2RTCM); $(PMAKE)

simobs_:
	cd $(SIMOBS); $(PMAKE)

testeph_:
	cd $(TESTEPH); $(PMAKE)

# Clean up

clean:
	cd $(RTKLIB_bld); if [ -e makefile ]; then $(PMAKE) clean; fi
	rm -rf $(RTKLIB_bld)/*
	cd $(IERS);     make clean
	cd $(UTEST);    make clean
	cd $(GENCRC);   make clean
	cd $(GENIONO);  make clean
	cd $(LOGFILE);  make clean
	cd $(RNX2RTCM); make clean
	cd $(SIMOBS);   make clean
	cd $(TESTEPH);  make clean
