#!/bin/bash

cd build

if [ "$PACKAGE_HAS_SITE" = true ] ; then
  # managing publication of developpers info
  if [ "$PACKAGE_DEV_INFO_PUBLISHED" = true ]; then
    site_publish_static_checks=ON
    if [ "$PACKAGE_HAS_TESTS" = true ] ; then
      site_publish_coverage=ON
    else
      site_publish_coverage=OFF
    fi
  else
    site_publish_static_checks=OFF
    site_publish_coverage=OFF
  fi

  # managing publication of package binaries
  if [ "$PACKAGE_BINARIES_PUBLISHED" = true ]; then
    site_publish_binaries=ON
  else
    site_publish_binaries=OFF
  fi

  # publishing API doc as soon as there are libraries
  if [ "$PACKAGE_HAS_LIBRARIES" = true ]; then
      site_publish_api=ON
  else
      site_publish_api=OFF
  fi

  # configuring the package adequately
  cmake -DREQUIRED_PACKAGES_AUTOMATIC_DOWNLOAD=ON -DADDITIONNAL_DEBUG_INFO=OFF -DBUILD_AND_RUN_TESTS=$site_publish_coverage -DBUILD_TESTS_IN_DEBUG=$site_publish_coverage -DBUILD_COVERAGE_REPORT=$site_publish_coverage -DENABLE_PARALLEL_BUILD=ON -DBUILD_EXAMPLES=OFF -DBUILD_API_DOC=$site_publish_api -DBUILD_STATIC_CODE_CHECKING_REPORT=$site_publish_static_checks -DGENERATE_INSTALLER=$site_publish_binaries -DWORKSPACE_DIR="../binaries/pid-workspace" ..
fi

# always generating the dependencies file of the package
cmake --build . --target list_dependencies -- write_file=true

cd ..
