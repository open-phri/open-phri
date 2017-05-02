
#!/bin/bash

cd build

# generating the coverage report of the package if there are tests on libraries
if [ "$PACKAGE_HAS_TESTS" = true ] && [ "$PACKAGE_HAS_LIBRARIES" = true ]; then
    #using && operator allows for failure to be fatal in shell execution 
    cmake --build . --target coverage && cd debug/share && cmake -E tar cvz coverage.tgz coverage_report/ && cd ../.. 
fi

cmake --build . --target staticchecks && cd release/share && cmake -E tar cvz staticchecks.tgz static_checks_report/ && cd ../..

cd ..
