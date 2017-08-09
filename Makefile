.PHONY all:
all: elka

.PHONY elka:
elka:
	@mkdir -p build_elka && cd build_elka && cmake -Wno-dev ../ -DCMAKE_TOOLCHAIN_FILE=../toolchain/Toolchain-arm-none-eabi.cmake
	@cd build_elka && make && make elka_hex && make elka_bin

.PHONY elka-debug:
elka-debug:
	@mkdir -p build_elka && cd build_elka && cmake -Wno-dev ../ -DCMAKE_TOOLCHAIN_FILE=../toolchain/Toolchain-arm-none-eabi.cmake -DCMAKE_VERBOSE_MAKEFILE:BOOL=ON
	@cd build_elka && make && make elka_hex && make elka_bin

.PHONY upload:
upload:
	@cd build_elka && st-flash --format ihex write elka.hex

.PHONY upload-bin:
upload-bin:
	@cd build_elka && st-flash write elka.bin 0x8000000

clean:
	@rm -rf build_elka
