# Installation

## Windows

### Prerequisites

#### CMake

1. Download the latest version of CMake from the [official CMake website](https://cmake.org/).
2. Run the installer and follow the instruction. After installation, add CMake to the environment PATH.

#### Visual Studio

1. Install Visual Studio with C++ workload. This includes the C++ compiler (MSVC) and the Windows SDK.

> You can use MinGW instead of MSVC.

#### (Optional) Eigen

Please see the [below](#optional-eigen-1)

### Build from sources

```bash
mkdir build
cd build
cmake -G "Visual Studio 16 2019" .. 
cmake --build . --config Release
```

## Linux

### Prerequisites

```bash
sudo apt install -y build-essential cmake
```

#### (Optional) [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)

Eigen library is optional. If you want to use Eigen vector, install eigen library and just include header file
before ``rbpodo`` header file. Detail installation guide can be found in [this page](https://eigen.tuxfamily.org/dox/GettingStarted.html).

```bash
sudo apt install -y wget
wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.bz2
tar xvf eigen-3.4.0.tar.bz2
cd eigen-3.4.0
mkdir build 
cd build && cmake .. && sudo cmake --build . --target install
```

### Build from sources

```bash
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release .. 
make
```

#### Options

- ``BUILD_EXAMPLES``
- ``BUILD_EIGEN_EXAMPLES``

### Install the binaries

```bash
cd build 
sudo make install
```