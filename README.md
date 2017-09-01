
Overview
=========

OpenPHRI: a generic framework to easily and safely control robots in interactions with humans

The license that applies to the whole package content is **GNULGPL**. Please look at the license.txt file at the root of this repository.

Installation and Usage
=======================

The procedures for installing the open-phri package and for using its components is based on the [PID](http://pid.lirmm.net/pid-framework/pages/introduction.html) build and deployment system called PID. Just follow and read the links to understand how to install, use and call its API and/or applications.

Until PID is publicly available (should arrive soon), pure CMake support is maintained. Make sure to have [Eigen3](http://eigen.tuxfamily.org/) installed on your system then use git, cmake and make as you would do with any project:
 * `git clone git@github.com:BenjaminNavarro/OpenPHRI.git`
 * `cd OpenPHRI/build`
 * `cmake ..`
 * [optional] `ccmake ..` (to configure build options)
 * `make` or `make -j` to parallelize the build
 * run apps (Start V-REP, open a scene (see OpenPHRI/share/scenes) and run the corresponding application e.g. `./apps/demo`)
   

About authors
=====================

open-phri has been developped by following authors: 
+ Benjamin Navarro (LIRMM)

Please contact Benjamin Navarro (navarro@lirmm.fr) - LIRMM for more information or questions.




