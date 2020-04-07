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
#include <osgEarth/FeatureElevationLayer>

using namespace osgEarth;

#define LC "[FeatureElevationLayer] "

#define OE_TEST OE_DEBUG

REGISTER_OSGEARTH_LAYER(featureelevation, FeatureElevationLayer);
REGISTER_OSGEARTH_LAYER(feature_elevation, FeatureElevationLayer);

//............................................................................

void
FeatureElevationLayer::Options::fromConfig(const Config& conf)
{
    attr().init("ELEVATION");
    offset().init(-1.0);

    conf.get("attr", attr());
    conf.get("offset", offset());
    featureSource().get(conf, "features");
}

Config
FeatureElevationLayer::Options::getConfig() const
{
    Config conf = ElevationLayer::Options::getConfig();
    conf.set("attr", attr());
    conf.set("offset", offset());
    featureSource().set(conf, "features");
    return conf;
}

//............................................................................

//OE_LAYER_PROPERTY_IMPL(FeatureElevationLayer, std::string, Attr, attr);
//OE_LAYER_PROPERTY_IMPL(FeatureElevationLayer, double, Offset, offset);

FeatureElevationLayer::~FeatureElevationLayer()
{
    //todo
}

void
FeatureElevationLayer::init()
{
    ElevationLayer::init();
    setTileSize(257u);

    // Global WGS84 profile
    setProfile(Profile::create("global-geodetic"));
}

Config
FeatureElevationLayer::getConfig() const
{
    Config c = ElevationLayer::getConfig();
    return c;
}

Status
FeatureElevationLayer::openImplementation()
{
    Status parent = ElevationLayer::openImplementation();
    if (parent.isError())
        return parent;

    Status fsStatus = options().featureSource().open(getReadOptions());
    if (fsStatus.isError())
        return fsStatus;

    FeatureSource* features = options().featureSource().getLayer();
    if (!features)
        return Status::ServiceUnavailable;

    if (features->getFeatureProfile())
    {
        if (getProfile() && !getProfile()->getSRS()->isEquivalentTo(features->getFeatureProfile()->getSRS()))
        {
            OE_WARN << LC << "Specified profile does not match feature profile, ignoring specified profile." << std::endl;
        }

        _extent = features->getFeatureProfile()->getExtent();

        // If you didn't specify a profile (hint, you should have), use the feature profile.
        if (!getProfile())
        {
            OE_WARN << LC << "No profile specified; falling back on feature profile." << std::endl;

            const Profile* profile = Profile::create(
                _extent.getSRS(),
                _extent.bounds().xMin(),
                _extent.bounds().yMin(),
                _extent.bounds().xMax(),
                _extent.bounds().yMax());

            setProfile(profile);
        }

        DataExtent de(_extent);
        dataExtents().push_back(de);
    }

    return Status::NoError;
}

void
FeatureElevationLayer::addedToMap(const Map* map)
{
    ElevationLayer::addedToMap(map);
    options().featureSource().addedToMap(map);
}

void
FeatureElevationLayer::removedFromMap(const Map* map)
{
    options().featureSource().removedFromMap(map);
    ElevationLayer::removedFromMap(map);
}

GeoHeightField
FeatureElevationLayer::createHeightFieldImplementation(const TileKey& key, ProgressCallback* progress) const
{
    if (getStatus().isError())
        return GeoHeightField(getStatus());

    FeatureSource* features = options().featureSource().getLayer();
    if (!features)
        return GeoHeightField(Status::ServiceUnavailable);

    int tileSize = getTileSize();

    if (intersects(key))
    {
        //Get the extents of the tile
        double xmin, ymin, xmax, ymax;
        key.getExtent().getBounds(xmin, ymin, xmax, ymax);

        const SpatialReference* featureSRS = features->getFeatureProfile()->getSRS();
        GeoExtent extentInFeatureSRS = key.getExtent().transform(featureSRS);

        const SpatialReference* keySRS = key.getProfile()->getSRS();

        // populate feature list
        // assemble a spatial query. It helps if your features have a spatial index.
        Query query;
        query.bounds() = extentInFeatureSRS.bounds();

        FeatureList featureList;
        osg::ref_ptr<FeatureCursor> cursor = features->createFeatureCursor(query, progress);
        while (cursor.valid() && cursor->hasMore())
        {
            Feature* f = cursor->nextFeature();
            if (f && f->getGeometry())
                featureList.push_back(f);
        }

        // We now have a feature list in feature SRS.

        bool transformRequired = !keySRS->isHorizEquivalentTo(featureSRS);

        if (!featureList.empty())
        {
            if (progress && progress->isCanceled())
                return GeoHeightField::INVALID;

            //Only allocate the heightfield if we actually intersect any features.
            osg::ref_ptr<osg::HeightField> hf = new osg::HeightField;
            hf->allocate(tileSize, tileSize);
            for (unsigned int i = 0; i < hf->getHeightList().size(); ++i) hf->getHeightList()[i] = NO_DATA_VALUE;

            // Iterate over the output heightfield and sample the data that was read into it.
            double dx = (xmax - xmin) / (tileSize - 1);
            double dy = (ymax - ymin) / (tileSize - 1);

            for (int c = 0; c < tileSize; ++c)
            {
                double geoX = xmin + (dx * (double)c);
                for (int r = 0; r < tileSize; ++r)
                {
                    double geoY = ymin + (dy * (double)r);

                    float h = NO_DATA_VALUE;

                    for (FeatureList::iterator f = featureList.begin(); f != featureList.end(); ++f)
                    {
                        if (progress && progress->isCanceled())
                            return GeoHeightField::INVALID;

                        osgEarth::Polygon* boundary = dynamic_cast<osgEarth::Polygon*>((*f)->getGeometry());

                        if (!boundary)
                        {
                            OE_WARN << LC << "NOT A POLYGON" << std::endl;
                        }
                        else
                        {
                            GeoPoint geo(keySRS, geoX, geoY, 0.0, ALTMODE_ABSOLUTE);

                            if (transformRequired)
                                geo = geo.transform(featureSRS);

                            if (boundary->contains2D(geo.x(), geo.y()))
                            {
                                h = (*f)->getDouble(options().attr().get());

                                if (keySRS->isGeographic())
                                {
                                    // for a round earth, must adjust the final elevation accounting for the
                                    // curvature of the earth; so we have to adjust it in the feature boundary's
                                    // local tangent plane.
                                    Bounds bounds = boundary->getBounds();
                                    GeoPoint anchor(featureSRS, bounds.center().x(), bounds.center().y(), h, ALTMODE_ABSOLUTE);
                                    if (transformRequired)
                                        anchor = anchor.transform(keySRS);

                                    // For transforming between ECEF and local tangent plane:
                                    osg::Matrix localToWorld, worldToLocal;
                                    anchor.createLocalToWorld(localToWorld);
                                    worldToLocal.invert(localToWorld);

                                    // Get the ECEF location of the anchor point:
                                    osg::Vec3d ecef;
                                    geo.toWorld(ecef);

                                    // Move it into Local Tangent Plane coordinates:
                                    osg::Vec3d local = ecef * worldToLocal;

                                    // Reset the Z to zero, since the LTP is centered on the "h" elevation:
                                    local.z() = 0.0;

                                    // Back into ECEF:
                                    ecef = local * localToWorld;

                                    // And back into lat/long/alt:
                                    geo.fromWorld(geo.getSRS(), ecef);

                                    h = geo.z();
                                }
                                break;
                            }
                        }
                    }

                    hf->setHeight(c, r, h + options().offset().get());
                }
            }
            return GeoHeightField(hf.release(), key.getExtent());
        }
    }
    return GeoHeightField::INVALID;
}

bool
FeatureElevationLayer::intersects(const TileKey& key) const
{
    return key.getExtent().intersects(_extent);
}
