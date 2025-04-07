/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include "RocksDBCache"
#include <osgEarth/Cache>
#include <osgDB/Registry>
#include <osgDB/FileNameUtils>

namespace osgEarth { namespace RocksDBCache
{
    /**
     * This driver defers loading of the source data to the appropriate OSG plugin. You
     * must explicitly set an override profile when using this driver.
     *
     * For example, use this driver to load a simple jpeg file; then set the profile to
     * tell osgEarth its projection.
     */
    class RocksDBCacheDriver : public osgEarth::CacheDriver
    {
    public:
        RocksDBCacheDriver()
        {
            supportsExtension( "osgearth_cache_rocksdb", "rocksdb cache for osgEarth" );
        }

        virtual const char* className() const
        {
            return "rocksdb cache for osgEarth";
        }

        virtual ReadResult readObject(const std::string& file_name, const Options* options) const
        {
            if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
                return ReadResult::FILE_NOT_HANDLED;

            return ReadResult( new RocksDBCacheImpl( getCacheOptions(options) ) );
        }
    };

    REGISTER_OSGPLUGIN(osgearth_cache_rocksdb, RocksDBCacheDriver);

} } // namespace osgEarth::RocksDBCache
