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

#include <osgEarth/Profiler>


class FeatureElevationTileSource : public TileSource
{
public:
    FeatureElevationTileSource( const TileSourceOptions& options ) :
      TileSource( options ),
      _options(options),
      _maxDataLevel(23),
      _offset(-0.1)
    {
        _offset = _options.offset().getOrUse(_offset);
    }

    virtual ~FeatureElevationTileSource() { }


    Status initialize(const osgDB::Options* readOptions)
    {
        Cache* cache = 0;

        if ( !_options.featureOptions().isSet() )
        {
            return Status::Error(Status::ConfigurationError, "Feature source is required" );
        }
    
        // create the driver:
        _features = FeatureSourceFactory::create( _options.featureOptions().value() );
        if ( !_features.valid() )
        {
            return Status::Error(Status::ServiceUnavailable, "Cannot find feature driver \"" + _options.featureOptions()->getDriver() + "\"");
        }

        // open the feature source:
        const Status& fstatus = _features->open(readOptions);
        if (fstatus.isError())
        {
            return fstatus;
        }

        if (_features->getFeatureProfile())
        {
			if (getProfile() && !getProfile()->getSRS()->isEquivalentTo(_features->getFeatureProfile()->getSRS()))
            {
                OE_WARN << LC << "Specified profile does not match feature profile, ignoring specified profile." << std::endl;
            }

            _extents = _features->getFeatureProfile()->getExtent();

			// If you didn't specify a profile (hint, you should have), use the feature profile.
			if ( !getProfile() )
			{
                OE_WARN << LC << "No profile specified; falling back on feature profile." << std::endl;

				const Profile* profile = Profile::create(
					_extents.getSRS(),
					_extents.bounds().xMin(),
					_extents.bounds().yMin(),
					_extents.bounds().xMax(),
					_extents.bounds().yMax());

				setProfile( profile );
			}
        }

        return STATUS_OK;
    }


    osg::Image* createImage(const TileKey& key, ProgressCallback* progress)
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

	    if (intersects(key))
        {
            //Get the extents of the tile
            double xmin, ymin, xmax, ymax;
            key.getExtent().getBounds(xmin, ymin, xmax, ymax);

            const SpatialReference* featureSRS = _features->getFeatureProfile()->getSRS();
            GeoExtent extentInFeatureSRS = key.getExtent().transform( featureSRS );

            const SpatialReference* keySRS = key.getProfile()->getSRS();
            
            // populate feature list
            // assemble a spatial query. It helps if your features have a spatial index.
            Query query;
            query.bounds() = extentInFeatureSRS.bounds();

		    FeatureList featureList;
            osg::ref_ptr<FeatureCursor> cursor = _features->createFeatureCursor(query);
            while ( cursor.valid() && cursor->hasMore() )
            {
                Feature* f = cursor->nextFeature();
                if ( f && f->getGeometry() )
                    featureList.push_back(f);
            }

            // We now have a feature list in feature SRS.

            bool transformRequired = !keySRS->isHorizEquivalentTo(featureSRS);
		    
			if (!featureList.empty())
			{
                //Only allocate the heightfield if we actually intersect any features.
                osg::ref_ptr<osg::HeightField> hf = new osg::HeightField;
                hf->allocate(tileSize, tileSize);
                for (unsigned int i = 0; i < hf->getHeightList().size(); ++i) hf->getHeightList()[i] = NO_DATA_VALUE;

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

						for (FeatureList::iterator f = featureList.begin(); f != featureList.end(); ++f)
						{
							osgEarth::Symbology::Polygon* boundary = dynamic_cast<osgEarth::Symbology::Polygon*>((*f)->getGeometry());

							if (!boundary)
							{
								OE_WARN << LC << "NOT A POLYGON" << std::endl;
							}
							else
							{
								GeoPoint geo(keySRS, geoX, geoY, 0.0, ALTMODE_ABSOLUTE);

                                if ( transformRequired )
                                    geo = geo.transform(featureSRS);

								if ( boundary->contains2D(geo.x(), geo.y()) )
								{
                                    h = (*f)->getDouble(_options.attr().value());

                                    if ( keySRS->isGeographic() )
                                    {                              
                                        // for a round earth, must adjust the final elevation accounting for the
                                        // curvature of the earth; so we have to adjust it in the feature boundary's
                                        // local tangent plane.
                                        Bounds bounds = boundary->getBounds();
                                        GeoPoint anchor( featureSRS, bounds.center().x(), bounds.center().y(), h, ALTMODE_ABSOLUTE );
                                        if ( transformRequired )
                                            anchor = anchor.transform(keySRS);

                                        // For transforming between ECEF and local tangent plane:
                                        osg::Matrix localToWorld, worldToLocal;
                                        anchor.createLocalToWorld(localToWorld);
                                        worldToLocal.invert( localToWorld );

                                        // Get the ECEF location of the anchor point:
                                        osg::Vec3d ecef;
                                        geo.toWorld( ecef );

                                        // Move it into Local Tangent Plane coordinates:
                                        osg::Vec3d local = ecef * worldToLocal;

                                        // Reset the Z to zero, since the LTP is centered on the "h" elevation:
                                        local.z() = 0.0;

                                        // Back into ECEF:
                                        ecef = local * localToWorld;

                                        // And back into lat/long/alt:
                                        geo.fromWorld( geo.getSRS(), ecef);

                                        h = geo.z();
                                    }
									break;
								}
							}
						}

						hf->setHeight(c, r, h + _offset);
					}
				}
                return hf.release();
			}	
        }
        return 0;        
    }


    bool intersects(const TileKey& key)
    {
        return key.getExtent().intersects( _extents );
    }


private:

    GeoExtent _extents;

    const FeatureElevationOptions _options;
    osg::ref_ptr<FeatureSource> _features;

    unsigned int _maxDataLevel;

    double _offset;
};


class ReaderWriterFeatureElevationTile : public TileSourceDriver
{
public:
    ReaderWriterFeatureElevationTile() {}

    virtual const char* className() const
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
