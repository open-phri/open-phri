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
