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
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */
#include <osgEarthFeatures/ImageToFeatureLayer>
#include <osgEarth/ImageLayer>
#include <osgEarth/Registry>
#include <osgEarthFeatures/FeatureCursor>

#define LC "[ImageToFeatureLayer] " << getName() << ": "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;


namespace osgEarth {
    namespace Features {
        REGISTER_OSGEARTH_LAYER(image_to_feature, ImageToFeatureLayer);
    }
}

namespace
{
    // feature source driver to turn raster data into polygons
    class ImageToFeatureDriver : public FeatureSource
    {
    public:
        osg::observer_ptr<ImageLayer> _layer;

        ImageToFeatureDriver(const ImageToFeatureLayerOptions& options) :
            FeatureSource(options),
            _options(options)
        {
            //nop
        }

        FeatureCursor* createFeatureCursor(const Symbology::Query& query, ProgressCallback* progress)
        {
            TileKey key = *query.tileKey();

            osg::ref_ptr<ImageLayer> layer;
            if (_layer.lock(layer))
            {
                GeoImage image = layer->createImage(key, progress);

                FeatureList features;

                if (image.valid())
                {
                    double pixWidth = key.getExtent().width() / (double)image.getImage()->s();
                    double pixHeight = key.getExtent().height() / (double)image.getImage()->t();
                    ImageUtils::PixelReader reader(image.getImage());

                    for (unsigned int r = 0; r < (unsigned)image.getImage()->t(); r++)
                    {
                        double y = key.getExtent().yMin() + (double)r * pixHeight;

                        double minX = 0;
                        double maxX = 0;
                        float value = 0.0;

                        for (unsigned int c = 0; c < (unsigned)image.getImage()->s(); c++)
                        {
                            double x = key.getExtent().xMin() + (double)c * pixWidth;

                            osg::Vec4f color = reader(c, r);

                            // Starting a new row.  Initialize the values.
                            if (c == 0)
                            {
                                minX = x;
                                maxX = x + pixWidth;
                                value = color.r();
                            }
                            // Ending a row, finish the polygon.
                            else if (c == image.getImage()->s() - 1)
                            {
                                // Increment the maxX to finish the row.
                                maxX = x + pixWidth;
                                Polygon* poly = new Polygon();
                                poly->push_back(minX, y);
                                poly->push_back(maxX, y);
                                poly->push_back(maxX, y + pixHeight);
                                poly->push_back(minX, y + pixHeight);
                                Feature* feature = new Feature(poly, SpatialReference::create("wgs84"));
                                feature->set(_options.attribute().get(), value);
                                features.push_back(feature);
                                minX = x;
                                maxX = x + pixWidth;
                                value = color.r();
                            }
                            // The value is different, so complete the polygon and start a new one.
                            else if (color.r() != value)
                            {
                                Polygon* poly = new Polygon();
                                poly->push_back(minX, y);
                                poly->push_back(maxX, y);
                                poly->push_back(maxX, y + pixHeight);
                                poly->push_back(minX, y + pixHeight);
                                Feature* feature = new Feature(poly, SpatialReference::create("wgs84"));
                                feature->set(_options.attribute().get(), value);
                                features.push_back(feature);
                                minX = x;
                                maxX = x + pixWidth;
                                value = color.r();
                            }
                            // The value is the same as the previous value, continue the polygon by increasing the maxX.
                            else if (color.r() == value)
                            {
                                maxX = x + pixWidth;
                            }
                        }


                    }

                    if (!features.empty())
                    {
                        //OE_NOTICE << LC << "Returning " << features.size() << " features" << std::endl;
                        return new FeatureListCursor(features);
                    }
                }
            }
            return 0;
        }

        virtual bool supportsGetFeature() const
        {
            return false;
        }

        virtual Feature* getFeature(FeatureID fid)
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

    protected:

        //override
        Status initialize(const osgDB::Options* readOptions)
        {
            _readOptions = Registry::cloneOrCreateOptions(readOptions);

            // Establish the feature profile.
            const Profile* wgs84 = Registry::instance()->getGlobalGeodeticProfile();
            GeoExtent extent(wgs84->getSRS(), -180, -90, 180, 90);

            FeatureProfile* profile = new FeatureProfile(extent);
            profile->setProfile(Profile::create("wgs84", extent.xMin(), extent.yMin(), extent.xMax(), extent.yMax(), "", 1, 1));
            profile->setFirstLevel(_options.level().get());
            profile->setMaxLevel(_options.level().get());
            profile->setTiled(true);

            setFeatureProfile(profile);
            return Status::OK();
        }


    private:
        const ImageToFeatureLayerOptions& _options;
        FeatureSchema _schema;
        osg::ref_ptr<osgDB::Options> _readOptions;
    };
}


//.........................................................

ImageToFeatureLayer::ImageToFeatureLayer() :
FeatureSourceLayer(&_optionsConcrete),
_options(&_optionsConcrete)
{
    init();
}

ImageToFeatureLayer::ImageToFeatureLayer(const ImageToFeatureLayerOptions& inOptions) :
FeatureSourceLayer(&_optionsConcrete),
_options(&_optionsConcrete),
_optionsConcrete(inOptions)
{
    init();
}

void
ImageToFeatureLayer::init()
{
    FeatureSourceLayer::init();    
    setFeatureSource(new ImageToFeatureDriver(options()));
}

void
ImageToFeatureLayer::setImageLayer(ImageLayer* layer)
{
    ImageToFeatureDriver* driver = dynamic_cast<ImageToFeatureDriver*>(getFeatureSource());
    if (driver)
    {
        driver->_layer = layer;
    }
}

void
ImageToFeatureLayer::addedToMap(const Map* map)
{
    OE_DEBUG << LC << "addedToMap" << std::endl;

    if (options().imageLayer().isSet())
    {
        _imageLayerListener.listen(
            map,
            options().imageLayer().get(),
            this,
            &ImageToFeatureLayer::setImageLayer);
    }

    FeatureSourceLayer::addedToMap(map);
}

void
ImageToFeatureLayer::removedFromMap(const Map* map)
{
    FeatureSourceLayer::removedFromMap(map);
}
