/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include "ViewpointsExtension"

#include <osgDB/ReaderWriter>
#include <osgDB/Registry>
#include <osgDB/FileNameUtils>

namespace osgEarth { namespace Viewpoints
{
    /**
     * Plugin entry point
     */
    class ViewpointsPlugin : public osgDB::ReaderWriter
    {
    public: // Plugin stuff

        ViewpointsPlugin() {
            supportsExtension( "osgearth_viewpoints", "osgEarth Viewpoints Extension" );
        }
        
        const char* className() const {
            return "osgEarth Viewpoints Extension";
        }

        virtual ~ViewpointsPlugin() { }

        ReadResult readObject(const std::string& filename, const osgDB::Options* dbOptions) const
        {
          if ( !acceptsExtension(osgDB::getLowerCaseFileExtension(filename)) )
                return ReadResult::FILE_NOT_HANDLED;

          return ReadResult( new ViewpointsExtension(Extension::getConfigOptions(dbOptions)) );
        }
    };

    REGISTER_OSGPLUGIN(osgearth_viewpoints, ViewpointsPlugin)

} } // namespace osgEarth::Viewpoints
