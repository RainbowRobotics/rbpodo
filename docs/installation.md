# Installation

## Windows

### Prerequisites

#### Visual Studio MSCV compiler

... WIP

## Linux

### Prerequisites

```bash
sudo apt install -y build-essential cmake
```

##### (Optional) [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)

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