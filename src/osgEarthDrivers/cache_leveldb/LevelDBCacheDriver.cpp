/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2014 Pelican Mapping
 * http://osgearth.org
 *
 * osgEarth is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */
#include "LevelDBCache"
#include <osgEarth/Cache>
#include <osgDB/Registry>
#include <osgDB/FileNameUtils>

namespace osgEarth { namespace Drivers { namespace LevelDBCache
{
    /**
     * This driver defers loading of the source data to the appropriate OSG plugin. You
     * must explicity set an override profile when using this driver.
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

        virtual const char* className()
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
