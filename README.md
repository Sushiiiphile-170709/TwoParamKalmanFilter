# gps_kalman (C)

A small C11 program that simulates a car driving with a noisy GPS receiver and
cleans up the track with a Kalman filter. No hardware and no data download are
needed: the drive, the GPS noise and the tunnels where GPS drops out are all
generated. It is a C port of an earlier C++ version and prints the same numbers
for the same options.

## Quick start (Garuda Linux)

```sh
sudo pacman -S --needed base-devel   # gcc, make; usually already installed
make test                            # build and run the 126 tests
make run                             # run the demo
```

`make run` writes `output/results.csv` and `output/track.svg`. Open the SVG in a
browser to see the true path, the noisy GPS dots, the filter's track and the
stretches where the GPS was lost.

The result for the default settings:

```
Simulated trip: 110.0 s, GPS every 1.0 s, noise sigma 5.0 m per axis, seed 42

  Position error (meters)               RMSE    mean     max       n
  Raw GPS fixes                         7.70    6.98   17.56      94
  Kalman estimate at the same fixes     6.27    5.66   12.39      94
  Kalman estimate during GPS outages   22.26   16.55   60.72     152
  Kalman estimate, every step          12.37    8.53   73.02    1101
```

The filter beats raw GPS by about 19 % here. The route is full of bends, which
the constant-velocity model does not know about. On a straight road the same
filter reaches about 4.8 m against 7.1 m raw. During an outage the estimate
coasts and drifts, so the error is larger there.

## Layout

```
Makefile
include/   headers (one per module)
src/       implementation, one .c file per header, plus main.c
tests/     Unity tests, one file per module
  vendor/unity/   Unity 2.6.0 (MIT), copied in so nothing has to be installed
data/      mock data: route and GPS outage files
build/     compiled files (not tracked by git)
output/    CSV and SVG written by the demo (not tracked by git)
```

| Module | What it does |
| --- | --- |
| `status` | error codes returned by every function that can fail |
| `mem` | allocation with overflow checks |
| `types` | shared structs: truth, GPS fixes, estimates, simulation result |
| `rng` | seedable random numbers (MT19937-64, Gaussian by the polar method) |
| `matrix` | small dense matrices (up to 4x4) with checked shapes |
| `geo` | latitude/longitude to local meters and back |
| `kalman_filter` | the constant-velocity Kalman filter |
| `simulator` | route, true motion, noisy GPS, outages |
| `tracker` | runs the filter over a simulation |
| `metrics` | RMSE, mean and max error |
| `csv_writer`, `svg_plot` | output files |
| `data_loader` | reads the route and outage CSV files from `data/` |
| `fsutil` | `mkdir -p` and safe path joining |

## Make targets

| Command | Result |
| --- | --- |
| `make` | builds `build/gps_kalman` |
| `make run` | builds and runs the demo on the files in `data/` |
| `make test` | builds and runs the tests |
| `make sanitize` | runs the tests with AddressSanitizer and UBSan (also leak checking) |
| `make analyze` | static analysis with `gcc -fanalyzer` |
| `make clean` | deletes `build/` and `output/` |
| `make help` | prints the target and option list |

Options go after `make`, for example `make NATIVE=0 LTO=0`:

| Option | Default | Meaning |
| --- | --- | --- |
| `CC` | `gcc` | compiler. The Makefile has a clang path (`CC=clang`), but it has only been tried with gcc |
| `NATIVE` | `1` | `-march=native -mtune=native`. Use `0` for a binary you will copy to another machine |
| `LTO` | `1` | link-time optimization |
| `DEBUG` | `0` | `1` gives `-O0 -g3` without hardening, for gdb |
| `WERROR` | `0` | `1` turns warnings into errors |
| `FASTLD` | empty | `mold`, `lld` or `gold` instead of GNU ld, if installed (not tried) |
| `V` | `0` | `1` prints the full compiler commands |

## What the Makefile does for Garuda

Garuda is Arch based, so the release build uses the same hardening flags Arch
uses for its packages, plus optimization for the local CPU:

- `-O3 -march=native -mtune=native -flto=auto -pipe`
- `-D_FORTIFY_SOURCE=3 -fstack-protector-strong -fstack-clash-protection -fcf-protection=full -fno-plt`
- position-independent executable, and linker flags `-z relro -z now -z noexecstack --as-needed`
- all CPU cores are used for the build unless you pass `-j` yourself

Floating-point flags are chosen so that results do not depend on the CPU:
`-ffp-contract=off` (no fused multiply-add) and no fast-math. `-fno-math-errno`
lets `sqrt` compile to a single instruction. If `gcc` is missing, `make` says
so and prints the `pacman` line above. The same Makefile works on other glibc
Linux systems (on Debian or Ubuntu the package is `build-essential`).

## Running the program directly

```sh
./build/gps_kalman [options]
```

| Option | Default | Meaning |
| --- | --- | --- |
| `--seed N` | 42 | seed for the GPS noise; the same seed gives the same data |
| `--gps-noise M` | 5 | true GPS noise, sigma per axis in meters |
| `--assumed-noise M` | same as `--gps-noise` | noise the filter believes in |
| `--accel-std A` | 1.0 | filter process noise in m/s^2 (higher reacts faster to turns, trusts GPS more) |
| `--origin-lat DEG`, `--origin-lon DEG` | 45, 10 | where the local frame is anchored |
| `--route FILE` | built in | route segments CSV |
| `--dropouts FILE` | built in | outage windows CSV, or `none` |
| `--out-dir DIR` | `output` | where `results.csv` and `track.svg` go |

Try a mismatch between the real and the assumed noise:
`./build/gps_kalman --gps-noise 12 --assumed-noise 3`.

## Mock data

`data/` holds the data that would otherwise come from a real drive:

- `default_route.csv`, `default_dropouts.csv`: the same route and outages that are built into the program. `make run` reads them from disk, and a test checks that the result equals the built-in version.
- `city_route.csv`, `city_dropouts.csv`: a slower route with tighter corners.

```sh
./build/gps_kalman --route data/city_route.csv --dropouts data/city_dropouts.csv
```

Route format, one segment per line: `duration_s,accel_mps2,yaw_rate_rad_s`. A
positive yaw rate turns left. Outage format: `start_s,end_s`. Blank lines,
lines starting with `#` and one header line are ignored. A bad line stops the
program with a message that includes the line number.

## Error handling and memory

- Functions that can fail return a `kf_status` (`status.h`) and put results in an out-parameter. On failure the output is left untouched or zeroed, as each header documents.
- Inputs are checked, including NaN and infinity. Bad sizes, NULL pointers, mismatched matrix shapes and singular matrices come back as error codes and never as undefined behavior.
- Matrices live on the stack (at most 4x4). The only heap data are the simulation arrays, sized exactly up front with overflow-checked arithmetic, and a run is capped at 5 million steps so a typo cannot ask for gigabytes.
- `kf_sim_result_free` and `kf_estimates_free` can be called twice safely.

## Tests

`make test` runs 126 tests in 10 files. They cover each module, the error
paths, the data files, the CSV and SVG output, and a few accuracy checks. For
example, the straight-road test compares the filter's error with the steady
state obtained from the Riccati equation. The random number tests compare
against values produced by C++ `std::mt19937_64` and `std::normal_distribution`,
including the output the C++ standard prescribes for the 10000th draw.

`make sanitize` runs the same tests under AddressSanitizer, UBSan and the leak
checker.

## Differences from the C++ version

- The numbers are the same. For six different option sets, stdout, `results.csv` and `track.svg` matched the C++ program byte for byte. This works because `rng.c` reproduces the C++ random number stream for a given seed.
- Exceptions became status codes.
- Matrices have a fixed capacity of 4x4 instead of being templates.
- Option values are parsed strictly (`--seed -1` or `--gps-noise abc` are errors). An invalid origin is rejected before any work is done.
- New: `--route`, `--dropouts` and the `data/` files.

## Limits

- The flat-earth conversion is fine for a trip of a few kilometers and wrong over long distances.
- The filter assumes constant velocity. Hard braking or sharp turns show up as error until the next GPS fix pulls the estimate back.
- Only 2D position is modeled. There is no altitude and no heading state.
