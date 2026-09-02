RTKLIB-EX (previously RTKLIB demo5): A version of RTKLIB optimized for low cost GNSS receivers (single, dual, or triple frequency), especially u-blox receivers and based on RTKLIB 2.4.3. This software is provided “AS IS” without any warranties of any kind so please be careful, especially if using it in any kind of real-time application. 

Releases and pre-releases for Windows executables are available at https://github.com/rtklibexplorer/RTKLIB/releases

Tutorials for this code, and sample GPS data sets are available at http://rtkexplorer.com/

The latest version of the user manual is at: https://rtkexplorer.com/pdfs/manual_demo5.pdf


WINDOWS: To build and install code for with Windows Embarcadero compiler:

GUIs:
1) Build executables with app/winapp/rtklib_winapp.groupproj project file
2) Install executables and DLLs to ../RTKLIB/bin by runnning app/winapp/install_winapp.bat

CUIs:
1) Build executables with app/consapp/rtklib_consapp.groupproj project file
2) Install executables to ../RTKLIB/bin by runnning app/consapp/install_consapp.bat



WINDOWS/LINUX CLI & GUI (except for Embarcadero GUI) using CMake

Qt5/6 modules required for GUI:
 - Core
 - Gui
 - Widgets
 - SerialPort
 - Xml
 - Concurrent
 - WebEngineWidgets or WebKitWidgets (optional for maps)
 - LinguistTools (optional for translations)


1) create a build directory
 > mkdir build
 > cd build/
2) setup CMake project
 > cmake ..
3) compile CLI & GUI
 > make
4) run tests
 > make test

LINUX: To build and install code (DEPRECATED)

CUIs:
1) cd app/consapp/<appName>/gcc
2) make

GUIs (Qt based):
1) cd app/qtapp
2) qmake
3) make
4) ./install_qtapp

Windows binaries can be found on the release page.
Pre-complied linux packages are available at https://build.opensuse.org/package/show/home:ReimannJens/rtklib-qt.

The last step will copy the compiled executables into a new directory RTKLIB_bin next to the rtklib source directory.

BENCHMARKS:
For evaluation and comparison purposes, config files (septentrio_urban.conf, f9p_urban.conf, trimble_urban.conf) are included in the data/config folder for real-time compatible solutions for the PPC-Dataset and UrbanNav Tokyo benchmarks.  Both benchmark datasets are available on Github.  Results with the latest code as of 8/24/26 using the PPC_Dataset offficial scoring for both benchmarks are:

Results: PPC-Dataset
   nagoya run1: 82.2%
   nagoya run2: 68.3%
   nagoya run3: 93.6%
    tokyo run1: 84.8%
    tokyo run2: 94.4%
    tokyo run3: 93.3%
    Overall score:  88.0%

Results: UrbanNav Tokyo
    F9P Odaiba: 83.6%
    F9P Shinjuku: 67.5%
    Trimble Odaiba: 81.8%
    Trimble Shinjuku: 72.1%
    Overall score:  75.7% 

For more information on running the benchmarks, see https://rtklibexplorer.wordpress.com/2026/07/09/benchmarking-rtk-solutions-with-the-ppc-dataset/ and https://rtklibexplorer.wordpress.com/2026/08/31/updates-to-urban-rtk-benchmarking-with-rtklib/

For more typical, post-processed, less severe RTK conditions, the f9p_ppk.conf config file in the same folder is recommended.

The CRX2RNX utility needs to be available in the search path for the decompression of CRX compressed RINEX files. For Linux the 'CRX2RNX' command needs to be renamed to 'crx2rnx'. This utility is available at https://terras.gsi.go.jp/ja/crx2rnx.html.
