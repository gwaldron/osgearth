SET(TARGET_SRC
    IntersectFeatureFilter.cpp    
)

SET(TARGET_H   	
    IntersectFeatureFilterOptions
)

SET(TARGET_LIBRARIES_VARS OSG_LIBRARY OSGDB_LIBRARY OPENTHREADS_LIBRARY) 

SETUP_PLUGIN(osgearth_featurefilter_intersect)

# to install public driver includes:
SET(LIB_NAME featurefilter_intersect)
SET(LIB_PUBLIC_HEADERS ${TARGET_H})

INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
