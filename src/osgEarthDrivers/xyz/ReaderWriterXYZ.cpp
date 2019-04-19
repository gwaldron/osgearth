/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2019 Pelican Mapping
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

#include "XYZOptions"

using namespace osgEarth;
using namespace osgEarth::Drivers;

#define LC "[XYZ driver] "
#define OE_TEST OE_DEBUG


class XYZSource : public TileSource
{
public:
    XYZSource(const TileSourceOptions& options) : 
      TileSource(options), _options(options), _rotate_iter(0u), _rotateStart(0), _rotateEnd(0)
      {
          //nop
      }


      Status initialize(const osgDB::Options* dbOptions)
      {
          _dbOptions = Registry::instance()->cloneOrCreateOptions(dbOptions);        

          URI xyzURI = _options.url().value();
          if ( xyzURI.empty() )
          {
              return Status::Error( Status::ConfigurationError, "Fail: driver requires a valid \"url\" property" );
          }

          // driver requires a profile.
          if ( !getProfile() )
          {
              return Status::Error( Status::ConfigurationError, "An explicit profile definition is required by the XYZ driver." );
          }

          _template = xyzURI.full();

          _rotateStart = _template.find("[");
          _rotateEnd   = _template.find("]");
          if ( _rotateStart != std::string::npos && _rotateEnd != std::string::npos && _rotateEnd-_rotateStart > 1 )
          {
              _rotateString  = _template.substr(_rotateStart, _rotateEnd-_rotateStart+1);
              _rotateChoices = _template.substr(_rotateStart+1, _rotateEnd-_rotateStart-1);
          }

          _format = _options.format().isSet() 
              ? *_options.format()
              : osgDB::getLowerCaseFileExtension( xyzURI.base() );

          return STATUS_OK;
      }


      osg::Image* createImage(const TileKey& key, ProgressCallback*  progress )
      {
          unsigned x, y;
          key.getTileXY( x, y );

          if ( _options.invertY() == true )
          {
              unsigned cols=0, rows=0;
              key.getProfile()->getNumTiles( key.getLevelOfDetail(), cols, rows );
              y = rows - y - 1;
          }

          std::string location = _template;

          // support OpenLayers template style:
          replaceIn( location, "${x}", Stringify() << x );
          replaceIn( location, "${y}", Stringify() << y );
          replaceIn( location, "${z}", Stringify() << key.getLevelOfDetail() );

          // failing that, legacy osgearth style:
          replaceIn( location, "{x}", Stringify() << x );
          replaceIn( location, "{y}", Stringify() << y );
          replaceIn( location, "{z}", Stringify() << key.getLevelOfDetail() );

          std::string cacheKey;

          if ( !_rotateChoices.empty() )
          {
              cacheKey = location;
              unsigned index = (++_rotate_iter) % _rotateChoices.size();
              replaceIn( location, _rotateString, Stringify() << _rotateChoices[index] );
          }


          URI uri( location, _options.url()->context() );
          if ( !cacheKey.empty() )
          {
              uri.setCacheKey(Cache::makeCacheKey(location, "uri"));
          }

          OE_TEST << LC << "URI: " << uri.full() << ", key: " << uri.cacheKey() << std::endl;

          return uri.getImage( _dbOptions.get(), progress );
      }

      virtual std::string getExtension() const 
      {
          return _format;
      }

      osg::HeightField* createHeightField( const TileKey& key, ProgressCallback* progress)
      {
          // MapBox encoded elevation PNG.
          // https://www.mapbox.com/blog/terrain-rgb/
          if (_options.elevationEncoding().value() == "mapbox")
          {
              if (getStatus().isError())
                  return 0L;

              osg::HeightField *hf = 0;
              osg::ref_ptr<osg::Image> image = createImage(key, progress);
              if (image.valid())
              {
                  // Allocate the heightfield.
                  hf = new osg::HeightField();
                  hf->allocate( image->s(), image->t() );

                  ImageUtils::PixelReader reader(image.get());
                  for (unsigned int c = 0; c < image->s(); c++)
                  {
                      for (unsigned int r = 0; r < image->t(); r++)
                      {
                          osg::Vec4 pixel = reader(c, r);
                          pixel.r() *= 255.0;
                          pixel.g() *= 255.0;
                          pixel.b() *= 255.0;
                          float h = -10000.0f + ((pixel.r() * 256.0f * 256.0f + pixel.g() * 256.0f + pixel.b()) * 0.1f);
                          hf->setHeight(c, r, h);
                      }
                  }
              }
              return hf;        
          }
          else
          {
              return TileSource::createHeightField( key, progress );
          }
      }

private:
    const XYZOptions       _options;
    std::string            _format;
    std::string            _template;
    std::string            _rotateChoices;
    std::string            _rotateString;
    std::string::size_type _rotateStart, _rotateEnd;
    OpenThreads::Atomic    _rotate_iter;

    osg::ref_ptr<osgDB::Options> _dbOptions;
};




class XYZTileSourceDriver : public TileSourceDriver
{
public:
    XYZTileSourceDriver()
    {
        supportsExtension( "osgearth_xyz", "XYZ Driver" );
    }

    virtual const char* className() const
    {
        return "XYZ Driver";
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        return new XYZSource( getTileSourceOptions(options) );
    }
};

REGISTER_OSGPLUGIN(osgearth_xyz, XYZTileSourceDriver)
