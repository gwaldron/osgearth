### DOCUMENTATION ###

set( DOCS_ROOT
    source/about.rst
    source/data.rst
    source/faq.rst
    source/howtos.rst
    source/index.rst
    source/install.rst
    source/ios.rst
    source/releasenotes.rst
    source/startup.rst
)

set( DOCS_DEVELOPER
    source/developer/coordinate_systems.rst
    source/developer/custom_driver.rst
    source/developer/index.rst
    source/developer/maps.rst
    source/developer/qt_integration.rst
    source/developer/shader_composition.rst
    source/developer/utilities.rst
)

set( DOCS_REFERENCES
    source/references/colorfilters.rst
    source/references/earthfile.rst
    source/references/envvars.rst
    source/references/index.rst
    source/references/symbology.rst
)

set( DOCS_REFRENCES_DRIVERS
    source/references/drivers/index.rst
)

set( DOCS_REFERENCES_DRIVERS_FEATURE
    source/references/drivers/feature/index.rst
    source/references/drivers/feature/ogr.rst
    source/references/drivers/feature/tfs.rst
    source/references/drivers/feature/wfs.rst
)

set( DOCS_REFERENCES_DRIVERS_LOADERS
    source/references/drivers/loaders/index.rst
)

set( DOCS_REFERENCES_DRIVERS_MODEL
    source/references/drivers/model/feature_geom.rst
    source/references/drivers/model/feature_model_shared_blocks.rst
    source/references/drivers/model/feature_model_shared_props.rst
    source/references/drivers/model/index.rst
    source/references/drivers/model/simple.rst
)

set( DOCS_REFERENCES_DRIVERS_TERRAIN
    source/references/drivers/terrain/index.rst
    source/references/drivers/terrain/mp.rst
    source/references/drivers/terrain/terrain_options_shared.rst
)

set( DOCS_REFERENCES_DRIVERS_TILE
    source/references/drivers/tile/agglite.rst
    source/references/drivers/tile/arcgis.rst
    source/references/drivers/tile/debug.rst
    source/references/drivers/tile/gdal.rst
    source/references/drivers/tile/index.rst
    source/references/drivers/tile/noise.rst
    source/references/drivers/tile/osg.rst
    source/references/drivers/tile/tilecache.rst
    source/references/drivers/tile/tileservice.rst
    source/references/drivers/tile/tms.rst
    source/references/drivers/tile/vpb.rst
    source/references/drivers/tile/wcs.rst
    source/references/drivers/tile/wms.rst
    source/references/drivers/tile/xyz.rst
)

set( DOCS_USER
    source/user/annotations.rst
    source/user/caching.rst
    source/user/earthfiles.rst
    source/user/elevation.rst
    source/user/features.rst
    source/user/imagery.rst
    source/user/index.rst
    source/user/maps.rst
    source/user/profile.rst
    source/user/spatialreference.rst
    source/user/tools.rst
)

source_group( Developer                    FILES ${DOCS_DEVELOPER} )
source_group( References                   FILES ${DOCS_REFERENCES} )
source_group( References\\Drivers          FILES ${DOCS_REFERENCES_DRIVERS} )
source_group( References\\Drivers\\Feature FILES ${DOCS_REFERENCES_DRIVERS_FEATURE} )
source_group( References\\Drivers\\Loaders FILES ${DOCS_REFERENCES_DRIVERS_LOADERS} )
source_group( References\\Drivers\\Model   FILES ${DOCS_REFERENCES_DRIVERS_MODEL} )
source_group( References\\Drivers\\Terrain FILES ${DOCS_REFERENCES_DRIVERS_TERRAIN} )
source_group( References\\Drivers\\Tile    FILES ${DOCS_REFERENCES_DRIVERS_TILE} )
source_group( User                         FILES ${DOCS_USER} )

add_custom_target( 
    Documentation
    SOURCES
        ${DOCS_ROOT}
        ${DOCS_DEVELOPER}
        ${DOCS_REFERENCES}
        ${DOCS_REFERENCES_DRIVERS}
        ${DOCS_REFERENCES_DRIVERS_FEATURE}
        ${DOCS_REFERENCES_DRIVERS_LOADERS}
        ${DOCS_REFERENCES_DRIVERS_MODEL}
        ${DOCS_REFERENCES_DRIVERS_TERRAIN}
        ${DOCS_REFERENCES_DRIVERS_TILE}
        ${DOCS_USER}
)
