
Overview
=========

OpenPHRI: a generic framework to easily and safely control robots in interactions with humans

The license that applies to the whole package content is **GNULGPL**. Please look at the license.txt file at the root of this repository.

Installation and Usage
=======================

The procedures for installing the open-phri package and for using its components is based on the [PID](http://pid.lirmm.net/pid-framework/pages/introduction.html) build and deployment system called PID. Just follow and read the links to understand how to install, use and call its API and/or applications.

Here are the basic steps to have OpenPHRI up and running:
 * Clone the PID workspace: `git clone https://github.com/lirmm/pid-workspace.git` or `git clone git@github.com:lirmm/pid-workspace.git` to use SSH instead of HTTPS
 * Go to the `pid` directory: `cd pid-workspace/pid`
 * Configure the workspace: `cmake ..`
 * Deploy OpenPHRI and its dependencies: `make deploy package=open-phri`
 * Now the library is compiled and available under `pid-workspace/instal/your-architecture/open-phri/current-version/`. The dynamic library files are under `lib` while the headers are in the `include` folder.
 * If you want to try out the example applications, go to the package build directory: `cd pid-workspace/packages/open-phri/build` then turn the CMake option `BUILD_EXAMPLES` to `ON` with the help of: `ccmake ..`
 * Rebuild the package: `make build`
 * Find the applications under the `bin` folder of the install path `pid-workspace/instal/your-architecture/open-phri/current-version/`
 * Run apps (Start V-REP, open a scene (see OpenPHRI/share/scenes) and run the corresponding application e.g. `./apps/demo`)
   

About authors
=====================

open-phri has been developped by following authors: 
+ Benjamin Navarro (LIRMM)

Please contact Benjamin Navarro (navarro@lirmm.fr) - LIRMM for more information or questions.




