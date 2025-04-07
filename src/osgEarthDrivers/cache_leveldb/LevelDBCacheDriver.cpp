/* osgEarth
 * Copyright 2016 Pelican Mapping
 * MIT License
 */
#include "LevelDBCache"
#include <osgEarth/Cache>
#include <osgDB/Registry>
#include <osgDB/FileNameUtils>

namespace osgEarth { namespace Drivers { namespace LevelDBCache
{
    /**
     * This driver defers loading of the source data to the appropriate OSG plugin. You
     * must explicitly set an override profile when using this driver.
     *
     * For example, use this driver to load a simple jpeg file; then set the profile to
     * tell osgEarth its projection.
     */
    class LevelDBCacheDriver : public osgEarth::CacheDriver
    {
    public:
        LevelDBCacheDriver()
        {
            supportsExtension( "osgearth_cache_leveldb", "leveldb cache for osgEarth" );
        }

        virtual const char* className() const
        {
            return "leveldb cache for osgEarth";
        }

        virtual ReadResult readObject(const std::string& file_name, const Options* options) const
        {
            if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
                return ReadResult::FILE_NOT_HANDLED;

            return ReadResult( new LevelDBCacheImpl( getCacheOptions(options) ) );
        }
    };

    REGISTER_OSGPLUGIN(osgearth_cache_leveldb, LevelDBCacheDriver);

} } } // namespace osgEarth::Drivers::LevelDBCache
