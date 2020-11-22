### DOCUMENTATION ###

set(DOCS_ROOT
    index.md
    install.md
    build.md
    releasenotes.md
    faq.md
    3.0_upgrade_guide.md
)

# todo if we need folders later
# source_group( Developer                    FILES ${DOCS_DEVELOPER} )
# source_group( References\\Drivers          FILES ${DOCS_REFERENCES_DRIVERS} )

add_custom_target(Documentation
    SOURCES
        ${DOCS_ROOT}
)
