# RTKLIB-EX (previously RTKLIB demo5)

A version of RTKLIB optimized for low cost GNSS receivers (single, dual, or triple frequency), especially u-blox receivers and based on RTKLIB 2.4.3. This software is provided “AS IS” without any warranties of any kind so please be careful, especially if using it in any kind of real-time application.

Releases and pre-releases for Windows executables are available at https://github.com/rtklibexplorer/RTKLIB/releases

Tutorials for this code, and sample GPS data sets are available at http://rtkexplorer.com/

The latest version of the user manual is at: https://rtkexplorer.com/pdfs/manual_demo5.pdf

==============================
INSTALLATION & COMPILATION
==============================

---
▶️ **Recommended: CMake (Windows, Linux, macOS)**
---

**Requirements**
- CMake ≥ 3.16
- C/C++ compiler (GCC, Clang, MSVC, ...)
- Qt5 or Qt6 (for GUI applications)
    - Required modules: Core, Gui, Widgets, SerialPort, Xml, Concurrent
    - Optional: WebEngineWidgets/WebKitWidgets (for maps), LinguistTools (for translations)

**Steps**
1. Create a build directory:
   ```sh
   mkdir build
   cd build
   ```
2. Configure the project:
   ```sh
   cmake ..
   ```
   - For Release build:
     ```sh
     cmake -DCMAKE_BUILD_TYPE=Release ..
     ```
   - To specify a custom Qt path:
     ```sh
     cmake -DQt5_DIR=/path/to/Qt5 ..
     ```
3. Build:
   ```sh
   make
   ```
4. (Optional) Run tests:
   ```sh
   make test
   ```

Executables will be available in the `bin/` directory after compilation.

---
▶️ **Windows (Embarcadero) Compilation**
---

**Graphical Applications (GUIs)**
1. Open `app/winapp/rtklib_winapp.groupproj` in Embarcadero
2. Build
3. Install to `../RTKLIB/bin` by running:
   ```bat
   app/winapp/install_winapp.bat
   ```

**Console Applications (CUIs)**
1. Open `app/consapp/rtklib_consapp.groupproj` in Embarcadero
2. Build
3. Install to `../RTKLIB/bin` by running:
   ```bat
   app/consapp/install_consapp.bat
   ```

---
▶️ **Alternative (Deprecated) Methods**
---

**Linux: Manual Compilation**

*Console*
```sh
cd app/consapp/<appName>/gcc
make
```

*Qt GUI*
```sh
cd app/qtapp
qmake
make
./install_qtapp
```

---
▶️ **Binaries & Packages**
---

- Windows: see the [releases page](https://github.com/rtklibexplorer/RTKLIB/releases)
- Linux: precompiled packages at https://build.opensuse.org/package/show/home:ReimannJens/rtklib-qt

---
▶️ **CRX2RNX Utility**
---

To decompress CRX-compressed RINEX files, the CRX2RNX utility must be available in your PATH.
On Linux, rename the command to `crx2rnx`.
Download: https://terras.gsi.go.jp/ja/crx2rnx.html
