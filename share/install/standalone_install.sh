#!/usr/bin/env bash

##################################################################################
#  --  get a workspace and defer this call to its standalone install script  --  #
##################################################################################
call_site_path=`pwd` #memorizing the call site

#going to the root folder of the package (simply finding the build folder)
script_dir_path_rel=`dirname "$0"`
script_dir_path=`(cd $script_dir_path_rel && pwd)`
cd $script_dir_path
while [ ! -d "./build" ] || [ ! -e "./CMakeLists.txt" ]
do
  cd ..
done
package_root_path=`pwd`
workspace_root_path=$package_root_path/binaries/pid-workspace
package_name=`basename $package_root_path`

#manage user arguments
with_tests="OFF"
with_sanitizers="OFF"
with_examples="OFF"

for an_arg in "$@"
do
  if [ $an_arg = "test" ]; then
    with_tests="ON"
  elif [ $an_arg = "sanitizers" ]; then
    with_sanitizers="ON"
  elif [ $an_arg = "example" ]; then
    with_examples="ON"
  elif [ $an_arg = "help" ]; then
    echo "Usage: `\n` + argument \"test\" builds and run tests `\n` + argument \"sanitizers\" to enable sanitizers `\n` + argument \"example\" builds the examples."
  fi
done

# #################################################################
# #  --  initializing the folder where workspace will be put  --  #
# #################################################################
echo "Preparing the build of $package_name ..."

#creating the folder for binaries if it does not exist
if [ ! -d "$package_root_path/binaries" ]; then
  mkdir $package_root_path/binaries
fi

#initializing the pid-workspace
if [ ! -d "$package_root_path/binaries/pid-workspace" ]; then
  # clone the workspace into the binaries folder
  (cd $package_root_path/binaries && git clone https://gite.lirmm.fr/pid/pid-workspace.git --branch master)
  # previous to an execution of cmake we need to set a link into the workspace that point to the current package
  (cd $workspace_root_path/packages && ln -s $package_root_path $package_name)
fi

# launch workspace configuration using the pkg-config plugin to generate adequate
cd pid && cmake -DPLUGIN_pkg_config=ON ..

#################################################################
#############  --  building the project  --  ####################
#################################################################
echo "Building $package_name ..."
# go to project build dir
cd $package_root_path/build && rm -Rf *
# configuring the package's project
cmake -DREQUIRED_PACKAGES_AUTOMATIC_DOWNLOAD=ON -DADDITIONNAL_DEBUG_INFO=OFF -DBUILD_AND_RUN_TESTS=$with_tests -DENABLE_SANITIZERS=$with_sanitizers -DENABLE_PARALLEL_BUILD=ON -DBUILD_EXAMPLES=$with_examples -DBUILD_API_DOC=OFF -DBUILD_STATIC_CODE_CHECKING_REPORT=OFF -DGENERATE_INSTALLER=OFF -DWORKSPACE_DIR="../binaries/pid-workspace" ..
# building the project
cmake --build . --target build -- force=true && echo "The path $package_root_path/binaries/pid-workspace/pid/share/pkgconfig must be added to your PKG_CONFIG_PATH environment variable. To make the change permanent, write the line \"export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:$package_root_path/binaries/pid-workspace/pid/share/pkgconfig\" into your .bashrc file."
#NB: provide the generated libraries as pkg-config modules

#go back to initial folder
cd $call_site_path
