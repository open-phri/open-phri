# Dependencies:
* C++ compiler with C++14 support (Unsolved bug with Clang (tested with v3.9.1) when using Vector6.Zero in python and passing it to C++ functions like NewVector6dPtr)
* CMake
* Eiegen3
* For python bindings:
 * minieigen (try `sudo easy_install minieigen`, if it complains about a missing boost_python-py36 library, symlink your boost_python3.so to boost_python-py36.so and run it again)
 * boost
 * python libs
