# Mavlink C++ sample

Example of sending/receiving heartbeats, sending vision position estimate, offboard mode (PX4 specific) and target position (offboard control signal).

For compilation you need to download [Mavlink lib](https://github.com/mavlink/c_library_v2) inside `mavlink` dir.

## Compilation

To compile project for specific board you need to specify CMake's toolchain file. Example toolchains for LicheeRV Nano (RISCV64) and Luckfox (ARMHF) located inside `./toolchains` dir.

For example, compile for RISCV64:

```bash
mkdir build && cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../toolchains/riscv64.toolchain.cmake
make
```

Result binary located inside `./build/bin/`.
