#Commands to run in order to compile/run ELKA program

##View code in a directory
`ls` (might be `dir` on Windows)

##Change into another directory
`cd <name_of_dir>`
`cd build_elka` from `elka_firmware` dir in order to view built code
  Useful to view executable (.elf, .hex, .bin) files

##Compile ELKA
`>>make elka` or `>>make elka-debug`
`elka-debug` target allows you to view all commands run during the
build process

##Clean up (delete) all code from build process
`>>make clean`

#Files of use to debug build process

##Buildscript files
`elka_firmware/Makefile`
`elka_firmware/CMakeLists.txt`
`elka_firmware/src/CMakeLists.txt`
`elka_firmware/src/<name_of_library_dir>/CMakeLists.txt`

##Linker files
`ldscripts/stm32_flash.ld`

##Compiler/toolchain files for CMake
`toolchain/Toolchain-arm-none-eabi.cmake`

##Source files
`src/<name_of_library_dir>/*.c`

##Header files
`inc/<name_of_library_dir/*.h`

##Executable files
###Only available after program has been built
`build_elka/elka`
`build_elka/elka.bin`
`build_elka/elka.hex`
