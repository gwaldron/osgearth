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
#include <osgEarth/Registry>
#include <osgEarth/JsonUtils>

#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>

#include <sstream>
#include <iomanip>
#include <string.h>

#include "CesiumIonOptions"

using namespace osgEarth;
using namespace osgEarth::Drivers;

#define LC "[CesiumIon driver] "
#define OE_TEST OE_DEBUG


class CesiumIonSource : public TileSource
{
public:
    CesiumIonSource(const TileSourceOptions& options) :
      TileSource(options),
      _options(options)
      {
          //nop
      }

      Status initialize(const osgDB::Options* dbOptions)
      {
          _dbOptions = Registry::instance()->cloneOrCreateOptions(dbOptions);

          if (_options.assetId().value().empty())
          {
              return Status::Error(Status::ConfigurationError, "Fail: driver requires a valid \"asset_id\" property");
          }

          if (_options.token().value().empty())
          {
              return Status::Error(Status::ConfigurationError, "Fail: driver requires a valid \"token\" property");
          }

          setProfile(osgEarth::Registry::instance()->getSphericalMercatorProfile());

          std::stringstream buf;
          buf << _options.server()->full();
          if (!endsWith(_options.server()->full(), "/")) buf << "/";
          buf << "v1/assets/" << *_options.assetId() << "/endpoint?access_token=" << *_options.token();
          URI endpoint(buf.str());
          OE_DEBUG << "Getting endpoint " << endpoint.full() << std::endl;

          ReadResult r = URI(endpoint).readString(_dbOptions.get());
          if (r.failed())
              return Status::Error(Status::ConfigurationError, "Failed to get metadata from asset endpoint");

          Json::Value doc;
          Json::Reader reader;
          if (!reader.parse(r.getString(), doc))
          {
              return Status::Error(Status::ConfigurationError, "Failed to parse metadata from asset endpoint");
          }

          _resourceUrl = doc["url"].asString();
          _resourceToken = doc["accessToken"].asString();

          // Configure the accept header
          std::stringstream buf2;
          buf2 << "*/*;access_token=" << _resourceToken;
          _acceptHeader = buf2.str();

          return STATUS_OK;
      }


      osg::Image* createImage(const TileKey&     key,
          ProgressCallback*  progress )
      {
          unsigned x, y;
          key.getTileXY( x, y );

          // Invert the y value
          unsigned cols=0, rows=0;
          key.getProfile()->getNumTiles( key.getLevelOfDetail(), cols, rows );
          y = rows - y - 1;

          std::string location = _resourceUrl;
          std::stringstream buf;
          buf << location;
          if (!endsWith(location, "/")) buf << "/";
          buf << key.getLevelOfDetail() << "/" << x << "/" << y << "." << *_options.format();

          URIContext context = _options.server()->context();
          context.addHeader("accept", _acceptHeader);
          URI uri(buf.str(), context);
          return uri.getImage(_dbOptions.get(), progress);
      }

      virtual std::string getExtension() const
      {
          return *_options.format();
      }

private:
    const CesiumIonOptions       _options;
    osg::ref_ptr<osgDB::Options> _dbOptions;
    std::string                  _acceptHeader;
    std::string                  _resourceToken;
    std::string                  _resourceUrl;
};

class CesiumIonTileSourceDriver : public TileSourceDriver
{
public:
    CesiumIonTileSourceDriver()
    {
        supportsExtension( "osgearth_cesiumion", "CesiumIon Driver" );
    }

    virtual const char* className() const
    {
        return "CesiumIon Driver";
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        return new CesiumIonSource( getTileSourceOptions(options) );
    }
};

REGISTER_OSGPLUGIN(osgearth_cesiumion, CesiumIonTileSourceDriver)
