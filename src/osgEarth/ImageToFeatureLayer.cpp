/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2020 Pelican Mapping
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
#include <osgEarth/ImageToFeatureLayer>
#include <osgEarth/ImageLayer>
#include <osgEarth/Registry>
#include <osgEarth/FeatureCursor>

#define LC "[ImageToFeatureSource] " << getName() << ": "

using namespace osgEarth;


namespace osgEarth {
    namespace Features {
        REGISTER_OSGEARTH_LAYER(imagetofeature, ImageToFeatureSource);
        REGISTER_OSGEARTH_LAYER(image_to_feature, ImageToFeatureSource);
    }
}

//.........................................................

Config
ImageToFeatureSource::Options::getConfig() const {
    Config conf = FeatureSource::Options::getConfig();
    image().set(conf, "image");
    conf.set("level", level());
    conf.set("attribute", attribute());
    return conf;
}

void
ImageToFeatureSource::Options::fromConfig(const Config& conf)
{
    level().init(0u);
    attribute().init("value");
    image().get(conf, "image");
    conf.get("level", level());
    conf.get("attribute", attribute());
}

//.........................................................

OE_LAYER_PROPERTY_IMPL(ImageToFeatureSource, unsigned, Level, level);
OE_LAYER_PROPERTY_IMPL(ImageToFeatureSource, std::string, Attribute, attribute);

void
ImageToFeatureSource::init()
{
    FeatureSource::init();
}

void
ImageToFeatureSource::setImageLayer(ImageLayer* layer)
{
    options().image().setLayer(layer);
}

ImageLayer*
ImageToFeatureSource::getImageLayer() const
{
    return options().image().getLayer();
}

Status
ImageToFeatureSource::openImplementation()
{
    Status parent = FeatureSource::openImplementation();
    if (parent.isError())
        return parent;

    // Establish the feature profile.
    osg::ref_ptr<const Profile> globalGeodetic = Profile::create(Profile::GLOBAL_GEODETIC);

    const GeoExtent& extent = globalGeodetic->getExtent();
    FeatureProfile* profile = new FeatureProfile(extent);
    profile->setTilingProfile(Profile::create(extent.getSRS(), extent.xMin(), extent.yMin(), extent.xMax(), extent.yMax(), 1, 1));
    profile->setFirstLevel(options().level().get());
    profile->setMaxLevel(options().level().get());

    setFeatureProfile(profile);

    return Status::NoError;
}

void
ImageToFeatureSource::addedToMap(const Map* map)
{
    options().image().addedToMap(map);
    FeatureSource::addedToMap(map);
}

void
ImageToFeatureSource::removedFromMap(const Map* map)
{
    FeatureSource::removedFromMap(map);
    options().image().removedFromMap(map);
}

FeatureCursor*
ImageToFeatureSource::createFeatureCursorImplementation(const Query& query, ProgressCallback* progress) const
{
    TileKey key = *query.tileKey();

    if (getImageLayer())
    {
        GeoImage image = getImageLayer()->createImage(key, progress);

        FeatureList features;

        if (image.valid())
        {
            double pixWidth = key.getExtent().width() / (double)image.getImage()->s();
            double pixHeight = key.getExtent().height() / (double)image.getImage()->t();

            ImageUtils::PixelReader reader(image.getImage());
            osg::Vec4f color;

            for (unsigned int r = 0; r < (unsigned)image.getImage()->t(); r++)
            {
                double y = key.getExtent().yMin() + (double)r * pixHeight;

                double minX = 0;
                double maxX = 0;
                float value = 0.0;

                for (unsigned int c = 0; c < (unsigned)image.getImage()->s(); c++)
                {
                    double x = key.getExtent().xMin() + (double)c * pixWidth;

                    reader(color, c, r);

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
                        feature->set(options().attribute().get(), value);
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
                        feature->set(options().attribute().get(), value);
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
                return new FeatureListCursor(std::move(features));
            }
        }
    }
    return 0;
}


Config
ImageToFeatureSource::getConfig() const
{
    Config c = FeatureSource::getConfig();
    return c;
}
