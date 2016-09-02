/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2016 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/
#include <osgEarth/TileSource>
#include <osgEarth/FileUtils>
#include <osgEarth/Registry>
#include <osgEarth/ImageUtils>
#include <osgEarth/URI>

#include <osgEarthUtil/TileIndex>


#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/ImageOptions>

#include <sstream>
#include <stdlib.h>
#include <memory.h>

#include <osgEarthDrivers/gdal/GDALOptions>
#include "TileIndexOptions"


#define LC "[TileIndex driver] "

using namespace std;
using namespace osgEarth;
using namespace osgEarth::Drivers;
using namespace osgEarth::Util;

class TileIndexSource : public TileSource
{
public:
    TileIndexSource( const TileSourceOptions& options ):
      TileSource( options ),
      _options( options ),
	  _tileSourceCache( true )
    {
    }

    Status initialize( const osgDB::Options* dbOptions )
    {
        _dbOptions = Registry::instance()->cloneOrCreateOptions(dbOptions);
        if ( _options.url().isSet() )
        {
            _index = TileIndex::load( _options.url()->full() );        
            if (_index.valid() )
            {
                setProfile( osgEarth::Registry::instance()->getGlobalGeodeticProfile() );
                return STATUS_OK;
            }
        }
        return Status::Error("Failed to load TileIndex");
    }

    osg::Image* createImage( const TileKey&        key,
                             ProgressCallback*     progress)
    {        
        osg::Timer_t start = osg::Timer::instance()->tick();
        std::vector< std::string > files;
        _index->getFiles( key.getExtent(), files );        
        osg::Timer_t end = osg::Timer::instance()->tick();
        OE_DEBUG << "Got " << files.size() << " files in " << osg::Timer::instance()->delta_m( start, end) << " ms" << std::endl;

        // The result image
        osg::Image* result = 0;
        
        for (unsigned int i = 0; i < files.size(); i++)
        {            
            osg::ref_ptr< TileSource> source;
            {
                //Try to get the TileSource from the cache
                TileSourceCache::Record record;
                if (_tileSourceCache.get( files[i], record ))
                {
                    source = record.value().get();                    
                }
                else
                {
                    // Couldn't get it from the cache so open it.                    
                    GDALOptions opt;
                    opt.url() = files[i];
                    //Just force it to render so we don't have to worry about falling back
                    opt.maxDataLevelOverride() = 23;           
                    //Disable the l2 cache so that we don't run out of RAM so easily.
                    opt.L2CacheSize() = 0;

                    start = osg::Timer::instance()->tick();
                    source = osgEarth::TileSourceFactory::create( opt );                               
                    Status compStatus = source->open();
                    if (compStatus.isOK())
                    {
                        _tileSourceCache.insert( files[i], source.get() );                                                
                    }
                    else
                    {
                        OE_WARN << "Failed to open " << files[i] << std::endl;
                    }
                    end = osg::Timer::instance()->tick();
                    //OE_NOTICE << "init took " << osg::Timer::instance()->delta_m( start, end) << "ms" << std::endl;
                }               
            }

            
            
            start = osg::Timer::instance()->tick();
            osg::ref_ptr< osg::Image > image = source->createImage( key);
            end = osg::Timer::instance()->tick();
            OE_DEBUG << "createImage " << osg::Timer::instance()->delta_m( start, end) << "ms" << std::endl;
            if (image)
            {                                
                if (!result)
                {
                    // Initialize the result
                     result = new osg::Image( *image.get() );
                }
                else
                {
                    // Composite the new image with the result
                     ImageUtils::mix( result, image.get(), 1.0);
                }                
            }
            else
            {
                OE_DEBUG << "Failed to create image for " << files[i] << std::endl;
            }
        }

        return result;
    }

    //std::map< std::string, osg::ref_ptr< TileSource> > _tileSourceCache;
    typedef LRUCache< std::string, osg::ref_ptr< TileSource> > TileSourceCache;
    TileSourceCache _tileSourceCache;

    osg::ref_ptr< TileIndex > _index;
    TileIndexOptions _options;
    osg::ref_ptr<osgDB::Options> _dbOptions;
};


class ReaderWriterTileIndex : public TileSourceDriver
{
public:
    ReaderWriterTileIndex() {}

    virtual const char* className() const
    {
        return "TileIndex Reader";
    }

    virtual bool acceptsExtension(const std::string& extension) const
    {
        return osgDB::equalCaseInsensitive( extension, "osgearth_tileindex" );
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options ) const
    {        
        if ( !acceptsExtension( osgDB::getFileExtension( file_name ) ) )
        {
            return ReadResult::FILE_NOT_HANDLED;
        }
        
        return new TileIndexSource( getTileSourceOptions(options) );
    }
};

REGISTER_OSGPLUGIN(osgearth_tileindex, ReaderWriterTileIndex)
