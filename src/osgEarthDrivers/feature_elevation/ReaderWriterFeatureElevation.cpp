/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2015 Pelican Mapping
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
#include <osgEarth/HeightFieldUtils>

#include <osgEarthFeatures/TransformFilter>

#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/ImageOptions>

#include <sstream>
#include <stdlib.h>
#include <memory.h>

#include "FeatureElevationOptions"

#define LC "[Featuer Elevation driver] "

using namespace std;
using namespace osgEarth;
using namespace osgEarth::Drivers;


class FeatureElevationTileSource : public TileSource
{
public:
    FeatureElevationTileSource( const TileSourceOptions& options ) :
      TileSource( options ),
      _options(options),
      _maxDataLevel(30)
    {
    }

    virtual ~FeatureElevationTileSource() { }


    Status initialize( const osgDB::Options* dbOptions )
    {
        Cache* cache = 0;

        _dbOptions = Registry::instance()->cloneOrCreateOptions( dbOptions );

        if ( _dbOptions.valid() )
        {
            // Set up a Custom caching bin for this TileSource
            cache = Cache::get( _dbOptions.get() );
            if ( cache )
            {
                Config optionsConf = _options.getConfig();

                std::string binId = Stringify() << std::hex << hashString(optionsConf.toJSON());
                _cacheBin = cache->addBin( binId );

                if ( _cacheBin.valid() )
                {
                    _cacheBin->apply( _dbOptions.get() );
                }
            }
        }


        if ( !_options.featureOptions().isSet() )
        {
            return Status::Error( Stringify() << LC << "Illegal: feature source is required" );
        }
    
        _features = FeatureSourceFactory::create( _options.featureOptions().value() );
        if ( !_features.valid() )
        {
            return Status::Error( Stringify() << "Illegal: no valid feature source provided");
        }

        //if ( _features->getGeometryType() != osgEarth::Symbology::Geometry::TYPE_POLYGON )
        //{
        //    Status::Error( Stringify() << "Illegal: only polygon features are currently supported");
        //    return false;
        //}

        _features->initialize( _dbOptions );

        // populate feature list
        osg::ref_ptr<FeatureCursor> cursor = _features->createFeatureCursor();
        while ( cursor.valid() && cursor->hasMore() )
        {
            Feature* f = cursor->nextFeature();
            if ( f && f->getGeometry() )
                _featureList.push_back(f);
        }

        if (_features->getFeatureProfile())
        {
            if (getProfile() && !getProfile()->getSRS()->isEquivalentTo(_features->getFeatureProfile()->getSRS()))
                OE_WARN << LC << "Specified profile does not match feature profile, ignoring specified profile." << std::endl;

            _extents = _features->getFeatureProfile()->getExtent();

            const Profile* profile = Profile::create(
                _extents.getSRS(),
                _extents.bounds().xMin(),
                _extents.bounds().yMin(),
                _extents.bounds().xMax(),
                _extents.bounds().yMax());

            setProfile( profile );
        }
        else if (getProfile())
        {
            _extents = getProfile()->getExtent();
        }
        else
        {
            return Status::Error( Stringify() << "Failed to establish a profile for " <<  this->getName() );
        }

        getDataExtents().push_back( DataExtent(_extents, 0, _maxDataLevel) );

        return STATUS_OK;
    }


    osg::Image* createImage( const TileKey&        key,
                             ProgressCallback*     progress)
    {
        return 0L;
    }


    osg::HeightField* createHeightField( const TileKey&        key,
                                         ProgressCallback*     progress)
    {
        if (key.getLevelOfDetail() > _maxDataLevel)
        {
            //OE_NOTICE << "Reached maximum data resolution key=" << key.getLevelOfDetail() << " max=" << _maxDataLevel <<  std::endl;
            return NULL;
        }

        int tileSize = _options.tileSize().value();

        //Allocate the heightfield
        osg::ref_ptr<osg::HeightField> hf = new osg::HeightField;
        hf->allocate(tileSize, tileSize);
        for (unsigned int i = 0; i < hf->getHeightList().size(); ++i) hf->getHeightList()[i] = NO_DATA_VALUE;

        if (intersects(key))
        {
            //Get the extents of the tile
            double xmin, ymin, xmax, ymax;
            key.getExtent().getBounds(xmin, ymin, xmax, ymax);

            // Iterate over the output heightfield and sample the data that was read into it.
            double dx = (xmax - xmin) / (tileSize-1);
            double dy = (ymax - ymin) / (tileSize-1);

            for (int c = 0; c < tileSize; ++c)
            {
                double geoX = xmin + (dx * (double)c);
                for (int r = 0; r < tileSize; ++r)
                {
                    double geoY = ymin + (dy * (double)r);

                    float h = NO_DATA_VALUE;


                    for (FeatureList::iterator f = _featureList.begin(); f != _featureList.end(); ++f)
                    {
                        osgEarth::Symbology::Polygon* p = dynamic_cast<osgEarth::Symbology::Polygon*>((*f)->getGeometry());

                        if (!p)
                        {
                            OE_WARN << LC << "NOT A POLYGON" << std::endl;
                        }
                        else
                        {
                            GeoPoint geo(key.getProfile()->getSRS(), geoX, geoY);
                            if (!key.getProfile()->getSRS()->isEquivalentTo(getProfile()->getSRS()))
                                geo.transform(getProfile()->getSRS());
                            
                            if (p->contains2D(geo.x(), geo.y()))
                            {
                                h = (*f)->getDouble(_options.attr().value());
                                break;
                            }
                        }
                    }

                    hf->setHeight(c, r, h);
                }
            }
        }
        return hf.release();
    }


    bool intersects(const TileKey& key)
    {
        return key.getExtent().intersects( _extents );
    }


private:

    GeoExtent _extents;

    const FeatureElevationOptions _options;

    osg::ref_ptr<FeatureSource> _features;
    FeatureList _featureList;

    osg::ref_ptr< CacheBin > _cacheBin;
    osg::ref_ptr< osgDB::Options > _dbOptions;

    unsigned int _maxDataLevel;
};


class ReaderWriterFeatureElevationTile : public TileSourceDriver
{
public:
    ReaderWriterFeatureElevationTile() {}

    virtual const char* className()
    {
        return "Feature Elevation Tile Reader";
    }

    virtual bool acceptsExtension(const std::string& extension) const
    {
        return osgDB::equalCaseInsensitive( extension, "osgearth_feature_elevation" );
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* opt) const
    {
        if ( !acceptsExtension( osgDB::getFileExtension( file_name ) ) )
        {
            return ReadResult::FILE_NOT_HANDLED;
        }
        return new FeatureElevationTileSource( getTileSourceOptions(opt) );
    }
};

REGISTER_OSGPLUGIN(osgearth_feature_elevation, ReaderWriterFeatureElevationTile)
