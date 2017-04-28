
#!/bin/bash

if [ "$PACKAGE_HAS_TESTS" = true ] ; then
    cd build && cmake --build . --target test && cd ..
fi

#
