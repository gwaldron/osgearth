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
#include <osgEarth/ImageUtils>
#include <osgEarth/Registry>

#include <osg/Notify>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>

#include <OpenThreads/Atomic>

#include <sstream>
#include <iomanip>
#include <string.h>

#include "QuadKeyOptions"

using namespace osgEarth;
using namespace osgEarth::Drivers;

#define LC "[QuadKey driver] "
#define OE_TEST OE_DEBUG


class QuadKeySource : public TileSource
{
public:
    QuadKeySource(const TileSourceOptions& options) : 
        TileSource(options), _options(options), _rotate_iter(0u), _rotateStart(0), _rotateEnd(0)
    {
        //nop
    }


    Status initialize(const osgDB::Options* dbOptions)
    {
        _dbOptions = Registry::instance()->cloneOrCreateOptions(dbOptions);        

        URI uri = _options.url().value();
        if ( uri.empty() )
        {
            return Status::Error( Status::ConfigurationError, "Fail: driver requires a valid \"url\" property" );
        }

        // The quadkey driver always uses spherical mercator.
        // Bing maps profile is spherical mercator with 2x2 tiles are the root.
        const Profile* profile = Profile::create(
            SpatialReference::get("spherical-mercator"),
            MERC_MINX, MERC_MINY, MERC_MAXX, MERC_MAXY,
            2, 2);

        setProfile( profile );


        
        _template = uri.full();
        
        _rotateStart = _template.find("[");
        _rotateEnd   = _template.find("]");
        if ( _rotateStart != std::string::npos && _rotateEnd != std::string::npos && _rotateEnd-_rotateStart > 1 )
        {
            _rotateString  = _template.substr(_rotateStart, _rotateEnd-_rotateStart+1);
            _rotateChoices = _template.substr(_rotateStart+1, _rotateEnd-_rotateStart-1);
        }

        _format = _options.format().isSet() 
            ? *_options.format()
            : osgDB::getLowerCaseFileExtension( uri.base() );

        return STATUS_OK;
    }


    osg::Image* createImage(const TileKey&     key,
                            ProgressCallback*  progress )
    {
        unsigned x, y;
        key.getTileXY( x, y );

        std::string location = _template;

        std::string quadkey = getQuadKey(key);

        // support OpenLayers template style:
        replaceIn( location, "${key}", quadkey );

        // failing that, legacy osgearth style:
        replaceIn( location, "{key}", quadkey );

        std::string cacheKey;

        if ( !_rotateChoices.empty() )
        {
            cacheKey = location;
            unsigned index = (++_rotate_iter) % _rotateChoices.size();
            replaceIn( location, _rotateString, Stringify() << _rotateChoices[index] );
        }


        URI uri( location, _options.url()->context() );
        if ( !cacheKey.empty() )
            uri.setCacheKey( cacheKey );

        OE_TEST << LC << "URI: " << uri.full() << ", key: " << uri.cacheKey() << std::endl;

        return uri.getImage( _dbOptions.get(), progress );
    }

    virtual std::string getExtension() const 
    {
        return _format;
    }

private:
    std::string getQuadKey(const TileKey& key)
    {
        unsigned int tile_x, tile_y;
        key.getTileXY(tile_x, tile_y);
        unsigned int lod = key.getLevelOfDetail();

        std::stringstream ss;
        for( unsigned i = (int)lod+1; i > 0; i-- )
        {
            char digit = '0';
            unsigned mask = 1 << (i-1);
            if ( (tile_x & mask) != 0 )
            {
                digit++;
            }
            if ( (tile_y & mask) != 0 )
            {
                digit += 2;
            }
            ss << digit;
        }
        return ss.str();
    }

    const QuadKeyOptions   _options;
    std::string            _format;
    std::string            _template;
    std::string            _rotateChoices;
    std::string            _rotateString;
    std::string::size_type _rotateStart, _rotateEnd;
    OpenThreads::Atomic    _rotate_iter;

    osg::ref_ptr<osgDB::Options> _dbOptions;
};




class QuadKeyTileSourceDriver : public TileSourceDriver
{
public:
    QuadKeyTileSourceDriver()
    {
        supportsExtension( "osgearth_quadkey", "QuadKey Driver" );
    }

    virtual const char* className() const
    {
        return "QuadKey Driver";
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        return new QuadKeySource( getTileSourceOptions(options) );
    }
};

REGISTER_OSGPLUGIN(osgearth_quadkey, QuadKeyTileSourceDriver)
