### DOCUMENTATION ###

set(DOCS_ROOT
    source/3.0_upgrade_guide.md
    source/build.md
    source/data.md
    source/earthfile.md
    source/faq.md
    source/index.md
    source/install.md
    source/layers.md
    source/releasenotes.md
    source/support.md
)

# todo if we need folders later
# source_group( Developer                    FILES ${DOCS_DEVELOPER} )
# source_group( References\\Drivers          FILES ${DOCS_REFERENCES_DRIVERS} )

add_custom_target(Documentation
    SOURCES
        ${DOCS_ROOT}
)
