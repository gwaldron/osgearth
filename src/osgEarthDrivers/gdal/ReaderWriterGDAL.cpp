/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2018 Pelican Mapping
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
#include <osgEarth/GDAL>

#define LC "[GDAL legacy driver] "

using namespace osgEarth;


class GDALTileSource : public TileSource
{
public:
    osg::ref_ptr<GDAL::Driver> _driver;
    TileSourceOptions _tileSourceOptions;
    GDAL::GDALLayerOptions<ImageLayer::Options> _gdalOptions;

    GDALTileSource(const ConfigOptions& options) :
        TileSource(options),
        _tileSourceOptions(options),
        _gdalOptions(options)
    {
    }

    virtual ~GDALTileSource()
    {
        _driver = 0L;
    }

    Status initialize(const osgDB::Options* readOptions)
    {
        _driver = new GDAL::Driver();

        _driver->setNoDataValue(getNoDataValue());
        _driver->setMinValidValue(getMinValidValue());
        _driver->setMaxValidValue(getMaxValidValue());

        optional<unsigned> maxDataLevel;
        _tileSourceOptions.getConfig().get("max_data_level", maxDataLevel);
        if (maxDataLevel.isSet())
        {
            _driver->setMaxDataLevel(maxDataLevel.get());
        }

        if (_tileSourceOptions.profile().isSet())
        {
            const Profile* profile = Profile::create(_tileSourceOptions.profile().get());
            _driver->setOverrideProfile(profile);
        }

        Status status = _driver->open(
            _gdalOptions,
            getPixelsPerTile(),
            getDataExtents(),
            readOptions);

        if (_driver->getProfile())
        {
            setProfile(_driver->getProfile());
        }

        return status;
    }

    osg::Image* createImage(const TileKey& key, ProgressCallback* progress)
    {
        osg::Image* image = _driver->createImage(key, getPixelsPerTile(), _tileSourceOptions.coverage()==true, progress);
        return image;
    }

    osg::HeightField* createHeightField(const TileKey& key, ProgressCallback* progress)
    {
        osg::HeightField* hf = _driver->createHeightField(key, getPixelsPerTile(), progress);
        return hf;
    }
};


class GDALTileSourceDriver : public TileSourceDriver
{
public:
    GDALTileSourceDriver() {}

    virtual const char* className() const
    {
        return "GDAL Tile Reader";
    }

    virtual bool acceptsExtension(const std::string& extension) const
    {
        return osgDB::equalCaseInsensitive( extension, "osgearth_gdal" );
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* opt) const
    {
        if ( !acceptsExtension( osgDB::getFileExtension( file_name ) ) )
        {
            return ReadResult::FILE_NOT_HANDLED;
        }
        return new GDALTileSource( getTileSourceOptions(opt) );
    }
};

REGISTER_OSGPLUGIN(osgearth_gdal, GDALTileSourceDriver)
