
#!/bin/bash


cd build

if [ -d "./release/share/developper_info" ] ; then
  rm -Rf ./release/share/developper_info
fi

#preparing the final archive to upload as artifact
mkdir ./release/share/developper_info
cp ./release/share/dependencies.txt ./release/share/developper_info
cmake --build . --target staticchecks && cd ./release/share && cmake -E tar cvz staticchecks.tgz static_checks_report/ && cd ../..
cd release/share && cmake -E tar cvz staticchecks.tgz static_checks_report/ && mv staticchecks.tgz developper_info && cd ../..

if [ "$PACKAGE_HAS_LIBRARIES" = true ] && [ "$PACKAGE_HAS_TESTS" = true ] ; then
  # generating the coverage report of the package if there are tests on libraries
  cd debug/share && cmake -E tar cvz coverage.tgz coverage_report/ && mv coverage.tgz ../../release/share/developper_info && cd ../..
fi
#creating the final archive to upload as artifact
cd ./release/share && cmake -E tar cvf developper_info.tgz developper_info/ && cd ../..

cd ..
