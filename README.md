# GPS Kalman Filter — C / Garuda Linux

A dependency-light C17 simulation of noisy GPS tracking with a 2D constant-velocity Kalman filter. All input is deterministic simulated data; no GPS hardware or external service is required.

## Garuda Linux setup

Garuda is Arch-based:

```bash
sudo pacman -S --needed base-devel clang
```

`gcc`, `make`, and the C math library are sufficient. `clang` is optional for formatting/static analysis.

## Build and run

```bash
make
make run
make test
```

The release build uses native-CPU optimization (`-march=native`, `-mtune=native`) and LTO. For portable binaries, override CFLAGS with `-march=x86-64-v3` or remove `-march=native`.

Useful targets: `make debug`, `make sanitize`, `make format`, `make clean`.

## Layout

- `src/`: C implementations
- `include/`: public module interfaces
- `tests/`: deterministic test suite
- `data/`: reserved for mock datasets
- `build/`: generated artifacts, ignored by Git

## Safety and design

The project uses explicit status codes, checked allocations, deterministic mock noise, bounded fixed-size Kalman math, and explicit ownership/freeing of simulation data.
