.PHONY all:
all: elka update
  
elka: update
	@mkdir -p build_elka && cd build_elka && cmake -Wno-dev ../ -DCMAKE_TOOLCHAIN_FILE=../toolchain/Toolchain-arm-none-eabi.cmake
	@cd build_elka && make

elka-debug: update
	@mkdir -p build_elka && cd build_elka && cmake -Wno-dev ../ -DCMAKE_TOOLCHAIN_FILE=../toolchain/Toolchain-arm-none-eabi.cmake -DCMAKE_VERBOSE_MAKEFILE:BOOL=ON
	@cd build_elka && make


update:
	git submodule update --init --remote --recursive
	./tools/bootstrap.sh

clean:
	@rm -rf build_elka
