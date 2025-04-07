/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include "DetailOptions"
#include "DetailExtension"

#include <osgDB/ReaderWriter>
#include <osgDB/Registry>
#include <osgDB/FileNameUtils>

namespace osgEarth { namespace Detail
{
    /**
     * Plugin entry point
     */
    class DetailPlugin : public osgDB::ReaderWriter
    {
    public: // Plugin stuff

        DetailPlugin() {
            supportsExtension( "osgearth_detail", "osgEarth Detail Texture Extension" );
        }
        
        const char* className() const {
            return "osgEarth Detail Texture Extension";
        }

        virtual ~DetailPlugin() { }

        ReadResult readObject(const std::string& filename, const osgDB::Options* dbOptions) const
        {
          if ( !acceptsExtension(osgDB::getLowerCaseFileExtension(filename)) )
                return ReadResult::FILE_NOT_HANDLED;

          return ReadResult( new DetailExtension(Extension::getConfigOptions(dbOptions)) );
        }
    };

    REGISTER_OSGPLUGIN(osgearth_detail, DetailPlugin)

} } // namespace osgEarth::Detail
