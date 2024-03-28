## building

Currently the project is setup to build with cmake and setup.py. Because fast_kinematics requires torch, which has a lot of dynamic links to shared libraries, please see `build.sh` for a list of shared libraries we need to exclude to make the build usable on other machines.

The the user will need to import torch before using this library as torch will load the shared libraries.