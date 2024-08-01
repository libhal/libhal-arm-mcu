# libhal ARM-MCU platform library architecture

Version: 1

This document goes over the layout and architecture of a libhal platform
library for ARM Cortex-M microcontrollers. Platform libraries provide drivers
and APIs for controlling the microcontroller hardware. This includes,
controlling the MCU's operating speed, powering on/off peripherals, and
creating driver objects to control I/O. Drivers objects that control
peripherals are called peripheral drivers. These drivers can then be used
directly by application software or used to construct other drivers such as
sensors, motor controllers, displays, and anything else that uses the libhal
interfaces.

## History of this Repo

Prior to the creation of this repo, there are several other repos for each MCU in each family. This became difficult to manage and the rationale to keep these libraries apart from each other no longer made sense. Khalil made the move to merge all of the arm mcu libraries into one package so there is a central place to get packages for all arm mcu related libraries. The goal is to:

1. Reduce the number of repos maintainers have to work with
   1. Reduce the amount of code duplication (conanfile.py, READMEs, etc)
   2. Reduce dependency depth (libhal-armcortex is merged into this package)
   3. Reduce the number of PRs when libhal, or -util, change
2. Improve code reuse between chip families with identical hardware IP
3. Make a central place for all ARM MCU related issues and changes

## Layout of the Directory

### `.github/workflows/`

This directory contains GitHub Actions workflow files for continuous integration
(CI) and other automated tasks. The workflows currently included are:

- `deploy.yml`: This workflow is used for deploy released versions of the code
  to the JFrog artifactory. See the section:
- `ci.yml`: This workflow runs the CI pipeline, which includes building the
  project, running tests, and deploying the library to the `libhal-trunk`
  package repository.
- `take.yml`: This workflow is responsible for the "take" action, which assigns
  commits to the users that comment `.take` as a comment in an issue.

### `conanfile.py`

This is a [Conan](https://conan.io/) recipe file. Conan is a package manager for
C and C++ that helps manage dependencies in your project. This file defines how
Conan should build your project and its dependencies.

### `CMakeList.txt`

The root CMake build script for the library. It contains a call to the
[`libhal-cmake-util`](https://github.com/libhal/libhal-cmake-util) function
[`libhal_test_and_make_library()`](https://github.com/libhal/libhal-cmake-util?tab=readme-ov-file#libhal_test_and_make_library).

### `datasheets/`

This directory is intended for storing data sheets related to the device that
the library is being built for. This directory is meant to be a reference for
library developers and potentially users of the library, to gain information
about how the device behaves.

Many data sheets are subject to copyright and that must be considered when
adding the datasheet to a libhal repo. If the datasheet cannot be redistributed
on the repo for copyright and/or license reasons, then a markdown file with a
link to the datasheet (and potentially mirrors of it) is an acceptable
alternative.

<!--

### `demos/`

This directory contains demonstration applications showing how to use the device
library. It includes:

- `resource_list.hpp`: A header file defining the resource list required for
  the demo applications.
- `main.cpp`: The main entry point for the demo applications.
- `platforms/lpc4074.cpp` and `platforms/lpc4078.cpp`: Platform-specific
  implementations for the demo applications.
- `CMakeLists.txt`: Build file using the
  [`libhal_build_demos`](https://github.com/libhal/libhal-cmake-util?tab=readme-ov-file#libhal_test_and_make_library)
  function from
  [`libhal-cmake-util`](https://github.com/libhal/libhal-cmake-util).

### `include/libhal-arm-mcu/`

This directory contains the header files for the device library. This contains
the public APIs. Try and keep the public APIs as minimal as possible as
removing or changing something from this area will result in either an API or
ABI break.

### `src/`

This directory contains the source files for the device library. Implementation
details for the device library and any other private support libraries are
written here.

### `test_package/`

This directory contains a test package for the Conan recipe. This tests that
the Conan recipe is working correctly. The test package doesn't have to do
anything fancy. It just exists to ensure that the device library can be a
dependency of an application and successfully build. Make sure to at least
include one file from the public includes of this repo in order to determine
that your headers work. If possible, create an object or run a function in the
code to ensure that your APIs and types can be used in the package.

### `tests/`

This directory contains tests for the device library. It will always contain a `main.test.cpp` which is the entry point for the tests.

-->

## How to Deploy Binaries for a Release

1. On Github, click on the "Releases" button
2. Press "Draft a new release"
3. For the tag drop down, provide a version. This version should be in the
   [SEMVER](https://semver.org/) format like so "1.0.0". No "v1.0.0" or "1.0.
   0v" or anything like that.
4. Press "Create New Tag" on the bottom of the drop down to create a tag for
   this release.
5. Press "Generate Notes" to add release notes to the release. Feel free to add
   additional notes you believe are useful to developers reading over these
   changes.
6. Press "Public Release"
7. The release should now exist in the "releases" section of the repo.
8. Go to Actions and on the left hand bar, press "🚀 Deploy"
9. Press "Run Workflow"
10. For the "Use workflow from" drop down, press it and select "tag" then the
    "tag" version of your release.
11. Finally press "Run Workflow" and the `deploy.yml` action will build your
    device driver for all of the available libhal platforms and deploy to the JFrog Artifactory binary repository.
