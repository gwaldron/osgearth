/* osgEarth
 * Copyright 2008-2014 Pelican Mapping
 * MIT License
 */
#include "RexTerrainEngineNode"
#include <osgEarth/Utils>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <string>

#define LC "[engine_rex] "

namespace osgEarth { namespace REX
{
    /**
     * osgEarth driver for the Rex terrain engine.
     */
    class RexTerrainEngineDriver : public osgDB::ReaderWriter
    {
    public:
        RexTerrainEngineDriver()
        {
            //nop
        }

        virtual const char* className() const
        {
            return "osgEarth Rex Terrain Engine";
        }

        virtual bool acceptsExtension(const std::string& extension) const
        {
            return osgDB::equalCaseInsensitive( extension, "osgearth_engine_rex" );
        }

        virtual ReadResult readObject(const std::string& uri, const Options* options) const
        {
            if ( "osgearth_engine_rex" == osgDB::getFileExtension( uri ) )
            {
                OE_INFO << LC << "Activated!" << std::endl;
                return ReadResult( new RexTerrainEngineNode() );
            }
            else
            {
                return ReadResult::FILE_NOT_HANDLED;
            }
        }    
    };

    REGISTER_OSGPLUGIN(osgearth_engine_rex, RexTerrainEngineDriver);

} } // namespace osgEarth::REX
