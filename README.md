# sdformat

SDFormat is an XML file format that describes environments, objects, and robots
in a manner suitable for robotic applications. SDFormat is capable of representing
and describing different physic engines, lighting properties, terrain, static
or dynamic objects, and articulated robots with various sensors, and acutators.
The format of SDFormat is also described by XML, which facilitates updates and
allows conversion from previous versions.

* SDFormat - The specification.
    * SDF - Synonym for SDFormat, though SDFormat should be preferred, as "SDF"
      is an acronym with other meanings.
* libsdformat - The C++ parsing code contained within this repository,
  which can be used to read SDFormat files and return a C++ interface.

[http://sdformat.org/](http://sdformat.org/)

Test coverage:

[![codecov](https://codecov.io/gh/osrf/sdformat/branch/master/graph/badge.svg)](https://codecov.io/gh/osrf/sdformat)

# Installation

We recommend following the Binary Installation instructions to get up and running as quickly and painlessly as possible.

The Source Installation instructions should be used if you need the very latest software improvements, you need to modify the code, or you plan to make a contribution.

## Binary Installation

### Ubuntu

On Ubuntu systems, `apt-get` can be used to install `sdformat`:
```
sudo apt install libsdformat<#>-dev libsdformat<#>
```

Be sure to replace `<#>` with a number value, such as 2 or 3, depending on
which version you need, or leave it empty for version 1.

### Windows

Install [Conda package management system](https://docs.conda.io/projects/conda/en/latest/user-guide/install/download.html).
Miniconda suffices.

Create if necessary, and activate a Conda environment:
```
conda create -n ign-ws
conda activate ign-ws
```

Install `sdformat`:
```
conda install libsdformat --channel conda-forge
```

You can view all the versions with
```
conda search libsdformat --channel conda-forge
```

and install a specific minor version with
```
conda install libsdformat=9.3.0 --channel conda-forge
```

## Source Installation

## UNIX

#### Build from Source

Standard installation can be performed in UNIX systems using the following
steps:
```
mkdir build
cd build
cmake ..
sudo make install
```

sdformat supported cmake parameters at configuring time:
 - `USE_INTERNAL_URDF` (`bool`) [default `False`]
   Use an internal copy of urdfdom 1.0.0 instead of look for one
   installed in the system
 - `USE_UPSTREAM_CFLAGS` (`bool`) [default `True`]
   Use the sdformat team compilation flags instead of the common set defined
   by cmake.

## Uninstallation

To uninstall the software installed with the previous steps:
```
cd build
sudo make uninstall
```

## Windows

### Prerequisites

Install [Conda package management system](https://docs.conda.io/projects/conda/en/latest/user-guide/install/download.html).
Miniconda suffices.

Create if necessary, and activate a Conda environment:
```
conda create -n ign-ws
conda activate ign-ws
```

Install prerequisites:
```
conda install urdfdom --channel conda-forge
```

Install Ignition dependencies:

You can view lists of dependencies:
```
conda search libsdformat --channel conda-forge --info
```

Install dependencies, replacing `<#>` with the desired versions:
```
conda install libignition-math<#> libignition-tools<#> --channel conda-forge
```

### Build from Source

This assumes you have created and activated a Conda environment while installing the Prerequisites.

1. Configure and build
  ```
  mkdir build
  cd build
  cmake .. -DBUILD_TESTING=OFF  # Optionally, -DCMAKE_INSTALL_PREFIX=path\to\install
  cmake --build . --config Release
  ```

2. Install
  ```
  cmake --install . --config Release
  ```
