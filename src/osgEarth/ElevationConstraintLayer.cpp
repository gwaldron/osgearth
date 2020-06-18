/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
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
#include <osgEarth/ElevationConstraintLayer>
#include <osgEarth/Session>
#include <osgEarth/FeatureCursor>
#include <osgEarth/Math>

using namespace osgEarth;

#undef LC
#define LC "[ElevationConstraintLayer] " << getName() << ": "


REGISTER_OSGEARTH_LAYER(elevationconstraint, ElevationConstraintLayer);
REGISTER_OSGEARTH_LAYER(elevation_constraint, ElevationConstraintLayer);

//........................................................................

Config
ElevationConstraintLayer::Options::getConfig() const
{
    Config conf = ImageLayer::Options::getConfig();
    featureSource().set(conf, "features");

    if (!filters().empty()) {
        Config temp;
        for(unsigned i=0; i<filters().size(); ++i)
            temp.add( filters()[i].getConfig() );
        conf.set( "filters", temp );
    }
    return conf;
}

void
ElevationConstraintLayer::Options::fromConfig(const Config& conf)
{
    magFilter() = osg::Texture::NEAREST;
    featureSource().get(conf, "features");

    const Config& filtersConf = conf.child("filters");
    for(ConfigSet::const_iterator i = filtersConf.children().begin(); i != filtersConf.children().end(); ++i)
        filters().push_back( ConfigOptions(*i) );
}

//........................................................................

namespace
{
    FeatureCursor* createCursor(FeatureSource* fs, FeatureFilterChain* chain, FilterContext& cx, const Query& query, ProgressCallback* progress)
    {
        FeatureCursor* cursor = fs->createFeatureCursor(query, progress);
        if (chain)
        {
            cursor = new FilteredFeatureCursor(cursor, chain, cx);
        }
        return cursor;
    }
}

void
ElevationConstraintLayer::init()
{
    ImageLayer::init();

    // Default profile (WGS84) if not set
    if (!getProfile())
    {
        setProfile(Profile::create("global-geodetic"));
    }

    _emptyImage = ImageUtils::createEmptyImage(1, 1, 2);
}

Status
ElevationConstraintLayer::openImplementation()
{
    Status parent = ImageLayer::openImplementation();
    if (parent.isError())
        return parent;

    // assert a feature source:
    Status fsStatus = options().featureSource().open(getReadOptions());
    if (fsStatus.isError())
        return fsStatus;

    _filterChain = FeatureFilterChain::create(options().filters(), getReadOptions());

    return Status::NoError;
}

void
ElevationConstraintLayer::addedToMap(const Map* map)
{
    ImageLayer::addedToMap(map);

    options().featureSource().addedToMap(map);

    if (getFeatureSource())
    {
        _session = new Session(map, NULL, getFeatureSource(), getReadOptions());
    }
}

void
ElevationConstraintLayer::removedFromMap(const Map* map)
{
    options().featureSource().removedFromMap(map);

    ImageLayer::removedFromMap(map);
}

void
ElevationConstraintLayer::setFeatureSource(FeatureSource* fs)
{
    if (getFeatureSource() != fs)
    {
        options().featureSource().setLayer(fs);

        if (fs && fs->getStatus().isError())
        {
            setStatus(fs->getStatus());
        }
    }
}

osg::Image*
ElevationConstraintLayer::createImageImplementationAux(const TileKey& key, ProgressCallback* progress) const
{
    if (getStatus().isError())
    {
        return 0L;
    }

    if (!getFeatureSource())
    {
        setStatus(Status::ServiceUnavailable, "No feature source");
        return 0L;
    }

    const FeatureProfile* featureProfile = getFeatureSource()->getFeatureProfile();
    if (!featureProfile)
    {
        setStatus(Status::ConfigurationError, "Feature profile is missing");
        return 0L;
    }

    const SpatialReference* featureSRS = featureProfile->getSRS();
    if (!featureSRS)
    {
        setStatus(Status::ConfigurationError, "Feature profile has no SRS");
        return 0L;
    }

    if (!_session.valid())
    {
        setStatus(Status::AssertionFailure, "_session is NULL - call support");
        return 0L;
    }

    FeatureList features;

    GeoExtent tileExtent = key.getExtent();
    double scale = 1.0 + 4.0 / getTileSize();
    tileExtent.scale(scale, scale);

    const GeoExtent& featuresExtent = _session->getFeatureSource()->getFeatureProfile()->getExtent();
    GeoExtent featuresExtentWGS84 = featuresExtent.transform( featuresExtent.getSRS()->getGeographicSRS() );
    GeoExtent imageExtentWGS84 = key.getExtent().transform( featuresExtent.getSRS()->getGeographicSRS() );
    GeoExtent queryExtentWGS84 = featuresExtentWGS84.intersectionSameSRS( imageExtentWGS84 );

    if (!queryExtentWGS84.isValid())
        return 0L;

    GeoExtent queryExtent = queryExtentWGS84.transform( featuresExtent.getSRS() );

    queryExtent = tileExtent.transform(featureProfile->getSRS());

    // TODO: this whole block needs to be elevated to a convenient location in the SDK
    if (featureProfile->getTilingProfile())
    {
        // Resolve the list of tile keys that intersect the incoming extent.
        std::vector<TileKey> intersectingKeys;
        featureProfile->getTilingProfile()->getIntersectingTiles(queryExtent, key.getLOD(), intersectingKeys);

        UnorderedSet<TileKey> featureKeys;
        for (int i = 0; i < intersectingKeys.size(); ++i)
        {
            if ((int)intersectingKeys[i].getLOD() > featureProfile->getMaxLevel())
                featureKeys.insert(intersectingKeys[i].createAncestorKey(featureProfile->getMaxLevel()));
            else
                featureKeys.insert(intersectingKeys[i]);
        }

        // Query and collect all the features we need for this tile.
        for (UnorderedSet<TileKey>::const_iterator i = featureKeys.begin(); i != featureKeys.end(); ++i)
        {
            Query query;
            query.tileKey() = *i;

            osg::ref_ptr<FeatureCursor> cursor = getFeatureSource()->createFeatureCursor(query, progress);
            if (cursor.valid())
            {
                cursor->fill(features);
            }
        }
    }
    else
    {
        // Set up the query; bounds must be in the feature SRS:
        Query query;
        query.bounds() = queryExtent.bounds();

        // Run the query and fill the list.
        osg::ref_ptr<FeatureCursor> cursor = getFeatureSource()->createFeatureCursor(query, progress);
        if (cursor.valid())
        {
            cursor->fill(features);
        }
    }

    int gridSize = getTileSize(); //TODO: option
    const osg::Vec4 zero(0,0,0,0);
    osg::Vec4 value(0,0,0,0);
    osg::Vec3d point;

    osg::ref_ptr<osg::Image> image = new osg::Image();
    image->allocateImage(gridSize, gridSize, 1, GL_RGBA, GL_FLOAT);
    image->setInternalTextureFormat(GL_RGBA16F_ARB); //16-bit is sufficient
    ImageUtils::PixelWriter write(image.get());
    write.assign(zero);

    // use a local tangent place to get accurate local coordinates
    const SpatialReference* local = key.getExtent().getSRS()->createTangentPlaneSRS(key.getExtent().getCentroid());
    GeoExtent localExtent = key.getExtent().transform(local);

    // transform all features into the LTP:
    for(FeatureList::iterator i = features.begin(); i != features.end(); ++i)
        i->get()->transform(local);

    double xbuffer = 0.5*(localExtent.width() / (gridSize-1));
    double ybuffer = 0.5*(localExtent.height() / (gridSize-1));
    double buffer2 = osg::maximum(xbuffer*xbuffer, ybuffer*ybuffer);


    // go through and for a bbox around each "vertex".
    // if any segment of the feature intersects this bbox
    // we can create a shifting vector to move the vertex
    // to the constraint.
    for (int t = 0; t < gridSize; ++t)
    {
        double v = (double)t/(double)(gridSize-1);
        double y = localExtent.yMin() + v * localExtent.height();

        for (int s = 0; s < gridSize; ++s)
        {
            double u = (double)s/(double)(gridSize-1);
            double x = localExtent.xMin() + u * localExtent.width();

            point.set(x, y, 0);

            double closest2 = DBL_MAX;

            for(FeatureList::iterator i = features.begin(); i != features.end(); ++i)
            {
                Feature* feature = i->get();

                if (feature)
                {
                    ConstGeometryIterator cgi(feature->getGeometry(), true);
                    while(cgi.hasMore())
                    {
                        const Geometry* geom = cgi.next();

                        ConstSegmentIterator csi(geom, false);
                        while (csi.hasMore())
                        {
                            Segment segment = csi.next();
                            Segment2d s2d(segment.first, segment.second);

                            osg::Vec3d c = s2d.closestPointTo(point);
                            osg::Vec3d shift = c-point;
                            double dist2 = shift.length2();
                            if (dist2 < buffer2 && dist2 < closest2)
                            {
                                closest2 = dist2;

                                value.set(
                                    shift.x(),
                                    shift.y(),
                                    shift.x() / localExtent.width(),
                                    shift.y() / localExtent.height() );
                            
                                write(value, s, t);
                            }
                        }
                    }
                }
            }
        }
    }
    return image.release();
}

GeoImage
ElevationConstraintLayer::createImageImplementation(const TileKey& key, ProgressCallback* progress) const
{
    osg::ref_ptr<osg::Image> primary =createImageImplementationAux(key, progress);
    if (!primary.valid())
    {
        return GeoImage();      // XXX
    }
    int gridSize = getTileSize(); //TODO: option
    osg::ref_ptr<osg::Image> parentImage = createImageImplementationAux(key.createParentKey(), progress);
    osg::ref_ptr<osg::Image> result = new osg::Image();
    result->allocateImage(gridSize, gridSize, 2, GL_RGBA, GL_FLOAT);
    result->setInternalTextureFormat(GL_RGBA16F_ARB); //16-bit is sufficient
    ImageUtils::PixelWriter write(result.get());
    ImageUtils::PixelReader readPrimary(primary.get());
    // First layer
    for (int t = 0; t < gridSize; ++t)
    {
        for (int s = 0; s < gridSize; ++s)
        {
            osg::Vec4 value = readPrimary(s, t);
            write(value, s, t, 0);
        }
    }
    // Scale and bias matrices, one for each TileKey quadrant.
    const osg::Matrixf scaleBias[4] =
    {
        osg::Matrixf(0.5f,0,0,0, 0,0.5f,0,0, 0,0,1.0f,0, 0.0f,0.5f,0,1.0f),
        osg::Matrixf(0.5f,0,0,0, 0,0.5f,0,0, 0,0,1.0f,0, 0.5f,0.5f,0,1.0f),
        osg::Matrixf(0.5f,0,0,0, 0,0.5f,0,0, 0,0,1.0f,0, 0.0f,0.0f,0,1.0f),
        osg::Matrixf(0.5f,0,0,0, 0,0.5f,0,0, 0,0,1.0f,0, 0.5f,0.0f,0,1.0f)
    };
    const osg::Matrixf& m = scaleBias[key.getQuadrant()];
    ImageUtils::PixelReader readParent(parentImage.get());
    for (int t = 0; t < gridSize; ++t)
    {
        float v = (float)t / (float)(gridSize-1);
        for (int s = 0; s < gridSize; ++s)
        {
            float u = (float)s / (float)(gridSize-1);
            float up = u*m(0,0) + m(3,0);
            float vp = v*m(1,1) + m(3,1);
            osg::Vec4 value = readParent(up, vp);
            write(value, s, t, 1);
        }
    }
    return GeoImage(result.get(), key.getExtent());
}
