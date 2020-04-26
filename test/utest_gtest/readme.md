## Utest using gtest
Porting utest here to use gtest, almost all tests in *.c files from utest are ported here, except:  
- the utest didn't contain any assert
- the utest print the results to a file or stdout for manually checking

## What's need
- cmake is required
- a c++ complier supported by cmake

> gtest and its lib is optional, because it would be downloading in the cmake build step

below envrionments has been tested
|OS|complier|
|----|----|
|ubuntu|g++|
|Windows 10|visual studio 2019|

## How to use
1. start from utest_gtest as working direcory
2. `mkdir bin` and `cd bin`
3. `cmake ..`
it would be a little slow the first time you run `cmake`, because we downloading the gtest project in this step
4. build and run the test
- if on Windows and using visual stuido, just open the *.sln and  
    1. click Build>Build solution
    2. click Test>Test Explorer
    3. click Test>Run all tests
- if on linux,
    1. `make` to generate `utest`
    2. `./utest` to run all tests
5. the results is as below

```bash
Running main() from F:\1-own\md\rtklib\addgtest\RTKLIB\test\utest_gtest\bin\googletest-src\googletest\src\gtest_main.cc
[==========] Running 52 tests from 14 test suites.
[----------] Global test environment set-up.
[----------] 1 test from Testionmodel
[ RUN      ] Testionmodel.utest1
[       OK ] Testionmodel.utest1 (0 ms)
[----------] 1 test from Testionmodel (0 ms total)

[----------] 1 test from Testionmapf
[ RUN      ] Testionmapf.utest2
[       OK ] Testionmapf.utest2 (0 ms)
[----------] 1 test from Testionmapf (0 ms total)

...

[----------] 3 tests from TEST_TLE
[ RUN      ] TEST_TLE.test_tle_read
[       OK ] TEST_TLE.test_tle_read (2 ms)
[ RUN      ] TEST_TLE.test_tle_pos
[       OK ] TEST_TLE.test_tle_pos (0 ms)
[ RUN      ] TEST_TLE.test_tle_pos_accuracy
[       OK ] TEST_TLE.test_tle_pos_accuracy (370 ms)
[----------] 3 tests from TEST_TLE (373 ms total)

[----------] Global test environment tear-down
[==========] 52 tests from 14 test suites ran. (10184 ms total)
[  PASSED  ] 52 tests.
```