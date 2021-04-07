
open-phri
==============

OpenPHRI: a generic framework to easily and safely control robots in interactions with humans



Disclamer
=========

OpenPHRI has been through a lot of changes recently so the Python bindings are not up to date and can't be used at the moment.

Also, the documentation is currently being written and can be found on [GitBook](https://openphri.gitbook.io/core). 

Once the documentation is finished and the Python bindings are updated, the V1.0.0 will be released. In the meantime, don't easitate to create an issue if something requires a fix or some explanations or if you find that something is missing from the library.

Overview
=========

For an overview of the library, you can check the related [Robotics and Automation Magazine article](https://ieeexplore.ieee.org/ielx7/100/4600619/08360398.pdf), but keep in mind that the library went through some changes since its writing and so some parts might not be up-to-date.

You can find the documentation (work in progress) about OpenPHRI in this [Gitbook](https://openphri.gitbook.io/core).

DISCLAMER: version 1.0, the current release, has lots of improvements compared to initial release (0.x), but the Python bindings haven't been updated and so cannot be used. Since a version 2.0 with major usability improvements is on its way, the 1.0 will not be updated except for potential bugs fixes. Python bindings will be back with 2.0.


Package Overview
================

The **open-phri** package contains the following:

 * Libraries:

   * open-phri (shared)

   * open-phri-vrep-driver (shared)

 * Examples:

   * force_control_example

   * joint_trajectory_generator_example

   * null_space_motion_example

   * obstacle_avoidance_example

   * path_follower_example

   * separation_distance_example

   * stop_constraint_example

   * trajectory_generator_example

   * velocity_constraint_example

 * Tests:

   * default_controller

   * interpolators

   * potential_field_generator

   * task_space

   * proxy_generators

   * universal_wrapper

   * constraints


Installation and Usage
======================

The **open-phri** project is packaged using [PID](http://pid.lirmm.net), a build and deployment system based on CMake.

If you wish to adopt PID for your develoment please first follow the installation procedure [here](http://pid.lirmm.net/pid-framework/pages/install.html).

If you already are a PID user or wish to integrate **open-phri** in your current build system, please read the appropriate section below.


## Using an existing PID workspace

This method is for developers who want to install and access **open-phri** from their PID workspace.

You can use the `deploy` command to manually install **open-phri** in the workspace:
```
cd <path to pid workspace>
pid deploy package=open-phri # latest version
# OR
pid deploy package=open-phri version=x.y.z # specific version
```
Alternatively you can simply declare a dependency to **open-phri** in your package's `CMakeLists.txt` and let PID handle everything:
```
PID_Dependency(open-phri) # any version
# OR
PID_Dependency(open-phri VERSION x.y.z) # any version compatible with x.y.z
```

If you need more control over your dependency declaration, please look at [PID_Dependency](https://pid.lirmm.net/pid-framework/assets/apidoc/html/pages/Package_API.html#pid-dependency) documentation.

Once the package dependency has been added, you can use the following components as component dependencies:
 * `open-phri/open-phri`
 * `open-phri/open-phri-vrep-driver`

You can read [PID_Component](https://pid.lirmm.net/pid-framework/assets/apidoc/html/pages/Package_API.html#pid-component) and [PID_Component_Dependency](https://pid.lirmm.net/pid-framework/assets/apidoc/html/pages/Package_API.html#pid-component-dependency) documentations for more details.
## Standalone installation

This method allows to build the package without having to create a PID workspace manually. This method is UNIX only.

All you need to do is to first clone the package locally and then run the installation script:
 ```
git clone https://github.com/open-phri/open-phri.git
cd open-phri
./share/install/standalone_install.sh
```
The package as well as its dependencies will be deployed under `binaries/pid-workspace`.

You can pass `--help` to the script to list the available options.

### Using **open-phri** in a CMake project
There are two ways to integrate **open-phri** in CMake project: the external API or a system install.

The first one doesn't require the installation of files outside of the package itself and so is well suited when used as a Git submodule for example.
Please read [this page](https://pid.lirmm.net/pid-framework/pages/external_API_tutorial.html#using-cmake) for more information.

The second option is more traditional as it installs the package and its dependencies in a given system folder which can then be retrived using `find_package(open-phri)`.
You can pass the `--install <path>` option to the installation script to perform the installation and then follow [these steps](https://pid.lirmm.net/pid-framework/pages/external_API_tutorial.html#third-step--extra-system-configuration-required) to configure your environment, find PID packages and link with their components.
### Using **open-phri** with pkg-config
You can pass `--pkg-config on` to the installation script to generate the necessary pkg-config files.
Upon completion, the script will tell you how to set the `PKG_CONFIG_PATH` environment variable for **open-phri** to be discoverable.

Then, to get the necessary compilation flags run:

```
pkg-config --static --cflags open-phri_<component>
```

```
pkg-config --variable=c_standard open-phri_<component>
```

```
pkg-config --variable=cxx_standard open-phri_<component>
```

To get the linker flags run:

```
pkg-config --static --libs open-phri_<component>
```

Where `<component>` is one of:
 * `open-phri`
 * `open-phri-vrep-driver`




Offline API Documentation
=========================

With [Doxygen](https://www.doxygen.nl) installed, the API documentation can be built locally by turning the `BUILD_API_DOC` CMake option `ON` and running the `doc` target, e.g
```
pid cd open-phri
pid -DBUILD_API_DOC=ON doc
```
The resulting documentation can be accessed by opening `<path to open-phri>/build/release/share/doc/html/index.html` in a web browser.

License
=======

The license that applies to the whole package content is **GNULGPL**. Please look at the [license.txt](./license.txt) file at the root of this repository for more details.

Authors
=======

**open-phri** has been developed by the following authors: 
+ Benjamin Navarro (LIRMM)

Please contact Benjamin Navarro (navarro@lirmm.fr) - LIRMM for more information or questions.
