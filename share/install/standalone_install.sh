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

cd $call_site_path #restore the call site

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

#forward call to the actual install script 
. $workspace_root_path/cmake/patterns/packages/run_standalone_install.sh $@