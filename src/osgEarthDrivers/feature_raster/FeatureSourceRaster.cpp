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
#include "RasterFeatureOptions"

#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgEarth/Registry>
#include <osgEarth/ImageUtils>
#include <osgEarthFeatures/FeatureSource>

#define LC "[Raster FeatureSource] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Drivers;


/**
 * A FeatureSource that reads features from a raster layer
 * 
 */
class RasterFeatureSource : public FeatureSource
{
public:
    RasterFeatureSource(const RasterFeatureOptions& options ) :
      FeatureSource( options ),
      _options     ( options )
    {           
    }

    virtual ~RasterFeatureSource()
    {               
        //nop
    }

    //override
    void initialize( const osgDB::Options* dbOptions )
    {
        _dbOptions = dbOptions ? osg::clone(dbOptions) : 0L;
    }


    /** Called once at startup to create the profile for this feature set. Successful profile
        creation implies that the datasource opened succesfully. */
    const FeatureProfile* createFeatureProfile()
    {
        const Profile* wgs84 = Registry::instance()->getGlobalGeodeticProfile();
        FeatureProfile* profile = new FeatureProfile( wgs84->getExtent() );
        profile->setProfile( Profile::create("wgs84", -180.0, -90.0, 180.0, 90.0, "", 1, 1) );
        int level = 12;
        profile->setFirstLevel(level);
        profile->setMaxLevel(level);
        profile->setTiled(true);
        return profile;
    }

    FeatureCursor* createFeatureCursor( const Symbology::Query& query )
    {
        OE_NOTICE << LC << " createFeatureCursor" << std::endl;
        unsigned w, h;
        query.tileKey()->getProfile()->getNumTiles( query.tileKey()->getLevelOfDetail(), w, h );      
        TileKey key = TileKey(query.tileKey()->getLevelOfDetail(), query.tileKey()->getTileX(), h - query.tileKey()->getTileY() -1, query.tileKey()->getProfile() );

#if 1
        // Debug
        Polygon* poly = new Polygon();
        poly->push_back(key.getExtent().xMin(), key.getExtent().yMin());
        poly->push_back(key.getExtent().xMax(), key.getExtent().yMin());
        poly->push_back(key.getExtent().xMax(), key.getExtent().yMax());
        poly->push_back(key.getExtent().xMin(), key.getExtent().yMax());
        FeatureList features;
        Feature* feature = new Feature(poly, SpatialReference::create("wgs84"));
        features.push_back( feature );
        return new FeatureListCursor( features );
#else

        osg::ref_ptr< osgEarth::ImageLayer > layer = query.getMap()->getImageLayerByName(*_options.layer());
        if (layer.valid())
        {
            GeoImage image = layer->createImage( key );
         
            FeatureList features;

            if (image.valid())
            {
                double pixWidth  = key.getExtent().width() / (double)image.getImage()->s();
                double pixHeight = key.getExtent().height() / (double)image.getImage()->t();
                ImageUtils::PixelReader reader(image.getImage());

                for (unsigned int r = 0; r < image.getImage()->t(); r++)
                {
                    double y = key.getExtent().yMin() + (double)r * pixHeight;

                    int startIndex = -1;
                    for (unsigned int c = 0; c < image.getImage()->s(); c++)
                    {
                        double x = key.getExtent().xMin() + (double)c * pixWidth;

                        osg::Vec4f color = reader(c, r);

                        
                        // If it's a forest emit a point.
                        if (color.r() == 100.0f || color.r() == 40.0f || color.r() == 50.0f || color.r() == 60.0f || color.r() == 70 || color.r() == 90.0f )
                        {
                            if (startIndex < 0)
                            {
                                startIndex = c;
                            }
                        }
                        else if (startIndex >= 0)
                        {
                            // Emit the polygon
                            Polygon* polygon = new Polygon();
                            double minX = key.getExtent().xMin() + (double)startIndex * pixWidth;
                            double maxX = key.getExtent().xMin() + (double)(c+1) * pixWidth;
                            Polygon* poly = new Polygon();
                            poly->push_back(minX, y);
                            poly->push_back(maxX, y);
                            poly->push_back(maxX, y+pixHeight);
                            poly->push_back(minX, y+pixHeight);
                            Feature* feature = new Feature(poly, SpatialReference::create("wgs84"));
                            features.push_back( feature );
                            startIndex = -1;
                        }
                    }

                    // Emit the polygon
                    if (startIndex > 0)
                    {
                        // Emit the polygon
                        Polygon* polygon = new Polygon();
                        double minX = key.getExtent().xMin() + (double)startIndex * pixWidth;
                        double maxX = key.getExtent().xMin() + (double)(image.getImage()->s()) * pixWidth;
                        Polygon* poly = new Polygon();
                        poly->push_back(minX, y);
                        poly->push_back(maxX, y);
                        poly->push_back(maxX, y+pixHeight);
                        poly->push_back(minX, y+pixHeight);
                        Feature* feature = new Feature(poly, SpatialReference::create("wgs84"));
                        features.push_back( feature );
                    }
                }
               
                if (!features.empty())
                {
                    OE_NOTICE << LC << "Returning " << features.size() << " features" << std::endl;
                    return new FeatureListCursor( features );
                }
            }
        }
        else
        {
            OE_NOTICE << LC << "Couldn't get layer " << *_options.layer() << std::endl;
        }
        return 0;
#endif
    }

    virtual bool supportsGetFeature() const
    {
        return false;
    }

    virtual Feature* getFeature( FeatureID fid )
    {
        return 0;
    }

    virtual bool isWritable() const
    {
        return false;
    }

    virtual const FeatureSchema& getSchema() const
    {
        //TODO:  Populate the schema from the DescribeFeatureType call
        return _schema;
    }

    virtual osgEarth::Symbology::Geometry::Type getGeometryType() const
    {
        return Geometry::TYPE_UNKNOWN;
    }



private:
    const RasterFeatureOptions       _options;    
    FeatureSchema                   _schema;
    osg::ref_ptr<osgDB::Options>    _dbOptions;    
};


class RasterFeatureSourceFactory : public FeatureSourceDriver
{
public:
    RasterFeatureSourceFactory()
    {
        supportsExtension( "osgearth_feature_raster", "Raster feature driver for osgEarth" );
    }

    virtual const char* className()
    {
        return "Raster Feature Reader";
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        return ReadResult( new RasterFeatureSource( getFeatureSourceOptions(options) ) );
    }
};

REGISTER_OSGPLUGIN(osgearth_feature_raster, RasterFeatureSourceFactory)

