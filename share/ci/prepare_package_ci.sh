#!/bin/bash

#########################
#  --  Git prepare  --  #
#########################

# Print Git version
git --version
dir_path=`pwd`
dir_name=`basename $dir_path`

############################################################################################
#  --  initializing the folder where dependencies and installed artefacts will be put  --  #
############################################################################################

#creating the folder for binaries
if [ ! -d "./binaries" ]; then
  mkdir binaries
fi

#initializing the pid-workspace
if [ ! -d "./binaries/pid-workspace" ]; then
  cd binaries && git clone git@gite.lirmm.fr:pid/pid-workspace.git && cd pid-workspace/pid && git checkout master && cmake .. && cd ../../..
else
  cd binaries/pid-workspace/pid && git pull -f official master && cmake .. && cd ../../..
fi

# previous to an execution we need to set a link into the workspace that point to the current package
echo "creating link into binaries/pid-workspace/packages/$dir_name"
cd binaries/pid-workspace/packages && ln -s $dir_path $dir_name && cd ../../..
