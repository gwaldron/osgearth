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
#include "TilePackageOptions"
#include "BundleReader"

#include <osgEarth/TileSource>
#include <osgEarth/Registry>
#include <osgEarth/URI>
#include <osgEarth/XmlUtils>

#include <osg/Notify>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

using namespace osgEarth;
using namespace osgEarth::Drivers;

#define LC "[ReaderWriterTilePackage] "

std::string padLeft(std::string value, unsigned int length)
{
    std::stringstream ss;
    if (value.size() < length)
    {
        for (unsigned int i = 0; i < (length - value.size()); i++)
        {
            ss << "0";
        }
        ss << value;
        return ss.str();
    }
    else
    {
        return value;
    }
}


/**
 *  TileSource that reads tiles from ArcGIS TilePackage format.  This currently only supports the exploded (aka unzipped) version.
 *  Thanks to https://github.com/consbio/tpkutils/tree/master/tpkutils for providing a great reference implementation
 */
class TilePackageSource : public TileSource
{
public:
    TilePackageSource(const TileSourceOptions& options) :
        TileSource(options),
        _options(options),
        _profileConf(ProfileOptions()),
        _tileSize(256),
        _bundleSize(128),
        _extension("png")
    {
    }

    // override
    Status initialize(const osgDB::Options* dbOptions)
    {

        _dbOptions = Registry::instance()->cloneOrCreateOptions(dbOptions);

        readConf();

        // establish a profile if we don't already have one:
        if (!getProfile())
        {
            const Profile* profile = NULL;

            if (_profileConf.isSet())
            {
                profile = Profile::create(_profileConf.get());
            }
            else
            {
                // finally, fall back on mercator
                profile = osgEarth::Registry::instance()->getSphericalMercatorProfile();
            }
            setProfile(profile);
        }

        return STATUS_OK;
    }

    /**
     * Reads layer info from the the conf file if it exists.
     */
    void readConf()
    {    
        std::string confPath = _options.url()->full() + "/conf.xml";

        osg::ref_ptr<XmlDocument> doc = XmlDocument::load(confPath);
        if (doc.valid())
        {
            Config conf = doc->getConfig();
            Config tileCacheInfo = conf.child("cacheinfo").child("tilecacheinfo");
            std::string wkt = tileCacheInfo.child("spatialreference").value("wkt");
            if (!wkt.empty())
            {
                SpatialReference* srs = SpatialReference::create(wkt);
                if (srs)
                {
                    if (srs->isMercator())
                    {                 
                        setProfile(osgEarth::Registry::instance()->getSphericalMercatorProfile());
                    }
                    else
                    {
                        setProfile(osgEarth::Registry::instance()->getGlobalGeodeticProfile());
                    }
                }
            }

            _tileSize = as<unsigned int>(tileCacheInfo.value("tilecols"), 256);

            std::string format = conf.child("cacheinfo").child("tileimageinfo").value("cachetileformat");
            if (format == "JPEG")
            {
                _extension = "jpg";
            }
            // All other cases will use png, this includes mixed mode.
            else
            {
                _extension = "png";
            }

            _bundleSize = as<unsigned int>(conf.child("cacheinfo").child("cachestorageinfo").value("packetsize"), 128);
        }
    }

    // override
    int getPixelsPerTile() const
    {
        return _tileSize;
    }

    // override
    osg::Image* createImage(const TileKey& key, ProgressCallback* progress)
    {
        // Try to figure out which bundle file the incoming tilekey is in.
        unsigned int numWide, numHigh;
        getProfile()->getNumTiles(key.getLevelOfDetail(), numWide, numHigh);

        std::stringstream buf;
        buf << _options.url()->full() << "/_alllayers/";
        buf << "L" << padLeft(toString<unsigned int>(key.getLevelOfDetail()), 2) << "/";

        unsigned int colOffset = static_cast<unsigned int>(floor(static_cast<double>(key.getTileX() / _bundleSize) * _bundleSize));
        unsigned int rowOffset = static_cast<unsigned int>(floor(static_cast<double>(key.getTileY() / _bundleSize) * _bundleSize));

        buf << "R" << padLeft(toHex(rowOffset), 4) << "C" << padLeft(toHex(colOffset), 4);
        buf << ".bundle";

        std::string bundleFile = buf.str();
        if (osgDB::fileExists(bundleFile))
        {
            BundleReader reader(bundleFile, _bundleSize);
            osg::Image* result = reader.readImage(key);
            return result;
        }

        return 0;
    }

    // override
    virtual std::string getExtension() const
    {
        return _extension;
    }

private:
    const TilePackageOptions _options;
    optional<ProfileOptions> _profileConf;
    osg::ref_ptr<osgDB::Options> _dbOptions;
    unsigned int _tileSize;
    std::string _extension;
    unsigned int _bundleSize;
};


class TilePackageTileSourceFactory : public TileSourceDriver
{
public:
    TilePackageTileSourceFactory()
    {
        supportsExtension("osgearth_tilepackage", "TilePackage");
    }

    virtual const char* className() const
    {
        return "TilePackage ReaderWriter";
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if (!acceptsExtension(osgDB::getLowerCaseFileExtension(file_name)))
            return ReadResult::FILE_NOT_HANDLED;

        return new TilePackageSource(getTileSourceOptions(options));
    }
};

REGISTER_OSGPLUGIN(osgearth_tilepackage, TilePackageTileSourceFactory)


