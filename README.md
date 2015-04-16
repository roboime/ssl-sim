[![Travis](https://img.shields.io/travis/roboime/ssl-sim.svg)](https://travis-ci.org/roboime/ssl-sim)
[![AppVeyor](https://img.shields.io/appveyor/ci/jansegre/ssl-sim.svg)](https://ci.appveyor.com/project/jansegre/ssl-sim)
[![MPL License](https://img.shields.io/badge/license-MPL-blue.svg)](https://www.mozilla.org/MPL/2.0/)

ssl-sim
=======

THIS IS A WORK IN PROGRESS. NOT SUITABLE FOR USAGE YET.

This is a physics simulator for Robocup Small Size League.
Main goals are:

1. Fast, consistent and fast physics.
2. Simple API for usage in code to aid AI.
3. Stand-alone GUI which speaks default protocols (grSim and ssl-vision).
4. Integrated referee for fully unsupervised matches.

Compiling
=========

This project uses C++11 features, some of which do not work with gcc maybe
(see `src/utils/stack_vector.hh`, that's causing ICE on gcc).

Only `clang` is tested, though on older systems you'll need a recent libstc++
if that's what your system uses, though libc++ is desired it's not ABI compatible
on older systems.  That is a fairly known issue, for a working solution the
travis build script (`.travis.yml`) is a great example and reference.

Currently __OS X Yosemite__ and __Ubuntu 14.04/12.04__ are known to work.
If you make it work on other systems please do add them here.
