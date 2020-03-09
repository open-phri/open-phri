
Overview
=========

OpenPHRI: a generic framework to easily and safely control robots in interactions with humans

Disclamer
=========

OpenPHRI has been through a lot of changes recently so the Python bindings are not up to date and can't be used at the moment.

<<<<<<< HEAD
For an overview of the library, you can check the related [Robotics and Automation Magazine article](https://ieeexplore.ieee.org/ielx7/100/4600619/08360398.pdf), but keep in mind that the library went through some changes since its writing and so some parts might not be up-to-date.

You can find the documentation (work in progress) about OpenPHRI in this [Gitbook](https://openphri.gitbook.io/core).

DISCLAMER: version 1.0, the current release, has lots of improvements compared to initial release (0.x), but the Python bindings haven't been updated and so cannot be used. Since a version 2.0 with major usability improvements is on its way, the 1.0 will not be updated except for potential bugs fixes. Python bindings will be back with 2.0.
=======
Also, the documentation is currently being written and can be found on [GitBook](https://openphri.gitbook.io/core). 

Once the documentation is finished and the Python bindings are updated, the V1.0.0 will be released. In the meantime, don't easitate to create an issue if something requires a fix or some explanations or if you find that something is missing from the library.

Overview
=========
>>>>>>> cd11920 (code formatting)

OpenPHRI: a generic framework to easily and safely control robots in interactions with humans.

For an overview of the library, you can check the related [Robotics and Automation Magazine article](https://ieeexplore.ieee.org/ielx7/100/4600619/08360398.pdf), but keep in mind that the library went through some changes since its writing and so some parts might not be up-to-date.

The license that applies to the whole package content is **GNULGPL**. Please look at the license.txt file at the root of this repository.

| Master  | Integration  |
|:---:|:---:|
| [![Build Status](https://travis-ci.org/BenjaminNavarro/open-phri.svg?branch=master)](https://travis-ci.org/BenjaminNavarro/open-phri) |  [![Build Status](https://travis-ci.org/BenjaminNavarro/open-phri.svg?branch=integration)](https://travis-ci.org/BenjaminNavarro/open-phri)  |

The license that applies to the whole package content is **GNULGPL**. Please look at the license.txt file at the root of this repository.

Installation and Usage
=======================

The detailed procedures for installing the open-phri package and for using its components is based on the [PID](http://pid.lirmm.net/pid-framework/pages/install.html) build and deployment system called PID. Just follow and read the links to understand how to install, use and call its API and/or applications.

For a quick installation:

## Installing the project into an existing PID workspace

To get last version :
 ```
cd <path to pid workspace>/pid
make deploy package=open-phri
```

To get a specific version of the package :
 ```
cd <path to pid workspace>/pid
make deploy package=open-phri version=<version number>
```

## Standalone install
 ```
git clone https://github.com/open-phri/open-phri.git
cd open-phri
```

Then run the adequate install script depending on your system. For instance on linux:
```
sh share/install/standalone_install.sh
```

The pkg-config tool can be used to get all links and compilation flags for the libraries defined inthe project. To let pkg-config know these libraries, read the last output of the install_script and apply the given command. It consists in setting the PKG_CONFIG_PATH, for instance on linux do:
```
export PKG_CONFIG_PATH=<path to open-phri>/binaries/pid-workspace/share/pkgconfig:$PKG_CONFIG_PATH
```

Then, to get compilation flags run:

```
pkg-config --static --cflags open-phri_<name of library>
```

To get linker flags run:

```
pkg-config --static --libs open-phri_<name of library>
```


About authors
=====================

open-phri has been developped by following authors: 
+ Benjamin Navarro (LIRMM)

Please contact Benjamin Navarro (navarro@lirmm.fr) - LIRMM for more information or questions.



