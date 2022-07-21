### DOCUMENTATION ###

set(DOCS_ROOT
    3.0_upgrade_guide.md
    arcgis.md
    bing.md
    build.md
    cesiumion.md
    composite.md
    conf.py
    contourmap.md
    data.md
    earthfile.md
    elevation.md
    faq.md
    featureimage.md
    featuremodel.md
    gdal.md
    image.md
    index.md
    install.md
    layer.md
    layers.md
    license.md
    mbtiles.md
    osgearth_conv.md
    releasenotes.md
    requirements.txt
    support.md
    tiledfeaturemodel.md
    TileLayer.md
    tms.md
    visiblelayer.md
    wms.md
    xyz.md
)

# prepend directory name
set(temp "")
foreach(f ${DOCS_ROOT})
    list(APPEND temp "source/${f}")
endforeach(f)
set(DOCS_ROOT ${temp})

# todo if we need folders later
# source_group( Developer                    FILES ${DOCS_DEVELOPER} )
# source_group( References\\Drivers          FILES ${DOCS_REFERENCES_DRIVERS} )

add_custom_target(Documentation
    SOURCES
        ${DOCS_ROOT}
)
