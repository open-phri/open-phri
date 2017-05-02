#!/bin/bash

cd build

# creating the doc
#
if [ "$PACKAGE_HAS_LIBRARIES" = true ] ; then
   cmake --build . --target doc
fi

# installing into the workspace
cmake --build . --target install

# creating the binary package
cmake --build . --target package

# installing the binary package
cmake --build . --target package_install

cd ..
