#!/bin/bash

#########################
#  --  Git prepare  --  #
#########################

# Print Git version
git --version

############################################################################################
#  --  initializing the folder where dependencies and installed artefacts will be put  --  #
############################################################################################

#creating the folder for binaries
if [ ! -d "./binaries" ]; then
  mkdir binaries
fi

#initializing the pid-workspace 
if [ ! -d "./binaries/pid-workspace" ]; then
  cd binaries && git clone git@gite.lirmm.fr:passama/pid-workspace.git && cd pid-workspace/pid && git checkout development && cmake .. && cd ../../..
else
  cd binaries/pid-workspace/pid && git pull -f official development && cmake .. && cd ../../..
fi


