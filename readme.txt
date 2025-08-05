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

1) create a build directory
 > mkdir build
 > cd build/
2) setup CMake project
 > cmake ..
3) compile CLI & GUI
 > make


LINUX: To build and install code (DEPRECATED)

CUIs:
1) cd app/consapp/<appName>/gcc
2) make

GUIs (Qt based - Beta):
1) cd app/qtapp
2) qmake
3) make
4) ./install_qtapp

Windows binaries can be found on the release page.
Pre-complied linux packages are available at https://build.opensuse.org/package/show/home:ReimannJens/rtklib-qt.

The last step will copy the compiled executables into a new directory RTKLIB_bin next to the rtklib source directory.


The CRX2RNX utility needs to be available in the search path for the decompression of CRX compressed RINEX files. For Linux the 'CRX2RNX' command needs to be renamed to 'crx2rnx'. This utility is available at https://terras.gsi.go.jp/ja/crx2rnx.html.
