

#!/bin/bash

cd build

if [ "$PACKAGE_HAS_SITE" = true ] ; then
    if [ "$PACKAGE_HAS_TESTS" = true ] ; then
       cmake --build . --target coverage
       wiki_publish_coverage=true
    else
       wiki_publish_coverage=false
    fi
    
    if [ "$PACKAGE_HAS_LIBRARIES" = true ] ; then
	wiki_publish_doc=true
    else
        wiki_publish_doc=false
    fi
    
    cmake --build . --target site -- upload_coverage="$wiki_publish_coverage" upload_doc="$wiki_publish_doc" upload_staticchecks=true
fi

if [ "$PACKAGE_BINARIES_PUBLISHED" = true ] ; then
   publish_binaries=true #TODO
fi

cd ..
