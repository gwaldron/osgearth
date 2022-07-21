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
#include "FeatureSDFLayer"
#include "FeatureRasterizer"
#include <osgDB/WriteFile>
#include <osgDB/FileUtils>

using namespace osgEarth;
using namespace osgEarth::Util;

#undef LC
#define LC "[FeatureSDF] "

REGISTER_OSGEARTH_LAYER(featuresdf, FeatureSDFLayer);

Config
FeatureSDFLayer::Options::getConfig() const
{
    Config conf = ImageLayer::Options::getConfig();
    featureSource().set(conf, "features");
    styleSheet().set(conf, "styles");
    conf.set("inverted", inverted());

    if (filters().empty() == false)
    {
        Config temp;
        for (unsigned i = 0; i < filters().size(); ++i)
            temp.add(filters()[i].getConfig());
        conf.set("filters", temp);
    }

    return conf;
}

void
FeatureSDFLayer::Options::fromConfig(const Config& conf)
{
    inverted().setDefault(false);

    featureSource().get(conf, "features");
    styleSheet().get(conf, "styles");
    conf.get("inverted", inverted());

    const Config& filtersConf = conf.child("filters");
    for (ConfigSet::const_iterator i = filtersConf.children().begin(); i != filtersConf.children().end(); ++i)
        filters().push_back(ConfigOptions(*i));
}

void
FeatureSDFLayer::init()
{
    ImageLayer::init();

    // The SDF shader is no good. Too many memory barriers.
    //_sdfGenerator.setUseGPU(true);
    _sdfGenerator.setUseGPU(false);
}

void
FeatureSDFLayer::setInverted(bool value) {
    options().inverted() = value;
}

bool
FeatureSDFLayer::getInverted() const {
    return options().inverted().get();
}

Status
FeatureSDFLayer::openImplementation()
{
    Status parent = ImageLayer::openImplementation();
    if (parent.isError())
        return parent;

    // assert a feature source:
    Status fsStatus = options().featureSource().open(getReadOptions());
    if (fsStatus.isError())
        return fsStatus;

    Status ssStatus = options().styleSheet().open(getReadOptions());
    if (ssStatus.isError())
        return ssStatus;

    establishProfile();

    _filterChain = FeatureFilterChain::create(options().filters(), getReadOptions());

    return Status::NoError;
}

void
FeatureSDFLayer::establishProfile()
{
    if (getProfile() == nullptr && getFeatureSource() != nullptr)
    {
        const FeatureProfile* fp = getFeatureSource()->getFeatureProfile();
        if (fp)
        {
            if (fp->getTilingProfile())
            {
                setProfile(fp->getTilingProfile());
            }
            else if (fp->getSRS())
            {
                setProfile(Profile::create(fp->getSRS()));
            }
        }
        else
        {
            OE_WARN << LC << "Got a null feature profile from " << getFeatureSource()->getName()
                << "; did your feature layer open properly?"
                << std::endl;
        }
    }
}

void
FeatureSDFLayer::addedToMap(const Map* map)
{
    ImageLayer::addedToMap(map);

    options().featureSource().addedToMap(map);
    options().styleSheet().addedToMap(map);

    if (getFeatureSource())
    {
        establishProfile();
        _session = new Session(map, getStyleSheet(), getFeatureSource(), getReadOptions());
        updateSession();
    }
}

void
FeatureSDFLayer::removedFromMap(const Map* map)
{
    options().featureSource().removedFromMap(map);
    options().styleSheet().removedFromMap(map);

    ImageLayer::removedFromMap(map);
}

void
FeatureSDFLayer::setFeatureSource(FeatureSource* fs)
{
    if (getFeatureSource() != fs)
    {
        options().featureSource().setLayer(fs);
        _featureProfile = 0L;

        if (fs)
        {
            if (fs->getStatus().isError())
            {
                setStatus(fs->getStatus());
            }
            else
            {
                // with a new feature source, we need to re-establish
                // the data extents and open a new session.
                updateSession();
            }
        }
    }
}

void
FeatureSDFLayer::setStyleSheet(StyleSheet* value)
{
    if (getStyleSheet() != value)
    {
        options().styleSheet().setLayer(value);
        if (_session.valid())
        {
            _session->setStyles(getStyleSheet());
        }
    }
}

void
FeatureSDFLayer::updateSession()
{
    if (_session.valid() && getFeatureSource())
    {
        const FeatureProfile* fp = getFeatureSource()->getFeatureProfile();

        dataExtents().clear();

        if (fp)
        {
            // recalculate the data extents based on the feature source.
            if (fp->getTilingProfile() != NULL)
            {
                // Use specified profile's GeoExtent
                unsigned maxLevel = fp->getMaxLevel();
                if (options().maxDataLevel().isSet())
                    maxLevel = osg::maximum(maxLevel, options().maxDataLevel().get());
                dataExtents().push_back(DataExtent(fp->getTilingProfile()->getExtent(), fp->getFirstLevel(), maxLevel));
            }
            else if (fp->getExtent().isValid() == true)
            {
                // Use FeatureProfile's GeoExtent
                dataExtents().push_back(DataExtent(fp->getExtent()));
            }

#if 0 // hopefully fixed
            // warn the user if the feature data is tiled and the
            // layer profile doesn't match the feature source profile
            if (fp->isTiled() &&
                fp->getTilingProfile()->isHorizEquivalentTo(getProfile()) == false)
            {
                OE_WARN << LC << "Layer profile doesn't match feature tiling profile - data may not render properly" << std::endl;
                OE_WARN << LC << "(Feature tiling profile = " << fp->getTilingProfile()->toString() << ")" << std::endl;
            }
#endif
        }

        _session->setFeatureSource(getFeatureSource());
        _session->setStyles(getStyleSheet());
    }
}


/*
osg::Image* redToRGBA(const osg::Image* image)
{
    osg::ref_ptr<osg::Image> out = new osg::Image();
    out->allocateImage(image->s(), image->t(), 1, GL_RGBA, GL_UNSIGNED_BYTE);
    osg::Vec4f p;
    ImageUtils::PixelReader read(image);
    ImageUtils::PixelWriter write(out.get());
    ImageUtils::ImageIterator i(read);
    i.forEachPixel([&]()
        {
            read(p, i.s(), i.t());
            p.set(1.0, 1.0, 1.0, p.r());
            write(p, i.s(), i.t());
        }
    );
    return out.release();
}
*/

GeoImage
FeatureSDFLayer::createImageImplementation(
    const TileKey& key, 
    ProgressCallback* progress) const
{
    if (getStatus().isError())
    {
        return GeoImage::INVALID;
    }

    if (!getFeatureSource())
    {
        setStatus(Status::ServiceUnavailable, "No feature source");
        return GeoImage::INVALID;
    }

    const FeatureProfile* featureProfile = getFeatureSource()->getFeatureProfile();
    if (!featureProfile)
    {
        setStatus(Status::ConfigurationError, "Feature profile is missing");
        return GeoImage::INVALID;
    }

    const SpatialReference* featureSRS = featureProfile->getSRS();
    if (!featureSRS)
    {
        setStatus(Status::ConfigurationError, "Feature profile has no SRS");
        return GeoImage::INVALID;
    }

    if (!_session.valid())
    {
        setStatus(Status::AssertionFailure, "_session is NULL - call support");
        return GeoImage::INVALID;
    }

    // Rasterizer for rendering features to an image. We are going to make this
    // larger than the final SDF so we can properly calculate distances to features
    // just outside the extent.
    GeoExtent featuresExtent = key.getExtent();

#if 1
    featuresExtent.expand(
        key.getExtent().width(), 
        key.getExtent().height());

    FeatureRasterizer rasterizer(
        2 * getTileSize(),
        2 * getTileSize(),
        featuresExtent,
        Color(1, 1, 1, 0)); // background

#else
    FeatureRasterizer rasterizer(
        getTileSize(),
        getTileSize(),
        featuresExtent,
        Color(1, 1, 1, 0)); // background

#endif


    // Hello! If you are looking at this code, maybe you are wondering
    // why your SDF layer with multiple styles only seems to be applying
    // one of those styles. If so, that is because this code is wrong.
    // It rasterizes all the features to a single image.  Sadly doing 
    // this makes it so the SDF generator doesn't know which pixels came from
    // which features, making it impossible to apply different SDF distance
    // limits to different features.
    // The correct approach is to NN and SDF each style separately and then
    // multiply the SDFs together at the end. Someone should do that.
    GeoImage rasterizedFeatures;

    double toMeters = 1.0;
    const GeoExtent& sdfExtent = key.getExtent();

    // Poor man's degrees-to-meters conversion    
    if (sdfExtent.getSRS()->isGeographic())
    {
        double LAT = sdfExtent.yMin() >= 0.0 ? sdfExtent.yMin() : sdfExtent.yMax();
        toMeters = sdfExtent.getSRS()->getEllipsoid().longitudinalDegreesToMeters(1.0, LAT);
    }

    float minDistanceMeters = FLT_MAX;
    float maxDistanceMeters = -FLT_MAX;

    FeatureStyleSorter::Function rasterizeFeatures = [&](
        const Style& style,
        FeatureList& features,
        ProgressCallback* progress)
    {
        if (features.empty())
            return;

        // TODO: bin as line or poly??

        // Render features to a temporary image
        Style r_style;
        if (features.front()->getGeometry()->isLinear())
            r_style.getOrCreate<LineSymbol>()->stroke()->color() = Color::Black;
        else
            r_style.getOrCreate<PolygonSymbol>()->fill()->color() = Color::Black;

        // Compute the min and max distances across all the styles.
        float styleMinDist = style.get<RenderSymbol>()->sdfMinDistance()->eval();
        float styleMaxDist = style.get<RenderSymbol>()->sdfMaxDistance()->eval();
        if (styleMinDist < minDistanceMeters)
        {
            minDistanceMeters = styleMinDist;
        }
        if (styleMaxDist > maxDistanceMeters)
        {
            maxDistanceMeters = styleMaxDist;
        }

        rasterizer.render(
            features,
            r_style,
            _session->getFeatureSource()->getFeatureProfile(),
            _session->styles());
    };

    FeatureStyleSorter sorter;

    // rasterize each group of features into a unified image:
    sorter.sort(
        key,
        Distance(key.getExtent().width() / 2.0, key.getExtent().getSRS()->getUnits()),
        _session.get(), 
        _filterChain.get(), 
        rasterizeFeatures, 
        progress);

    rasterizedFeatures = rasterizer.finalize();

    // Convert the distances to pixels
    double metersPerPixel = toMeters * (key.getExtent().width() / (double)rasterizedFeatures.getImage()->s());

    // We couldn't compute a valid min/max distance from the features b/c no features were rendered
    // So initialize it to something reasonable.
    if (minDistanceMeters == FLT_MAX && maxDistanceMeters == -FLT_MAX)
    {
        minDistanceMeters = 0.0f;
        maxDistanceMeters = 1.0f;
    }

    float minPixels = minDistanceMeters / metersPerPixel;
    float maxPixels = maxDistanceMeters / metersPerPixel;
    // Prevent divide by zero error
    if (minPixels == maxPixels)
    {
        maxPixels += 1;
    }

    // Crop the image so it's the proper extent for the incoming tilekey    
    osg::ref_ptr< osg::Image > sdf = _sdfGenerator.createDistanceField(rasterizedFeatures.getImage(), minPixels, maxPixels);
    GeoImage newSDF(sdf.get(), featuresExtent);
    GeoImage cropped = newSDF.crop(key.getExtent(), false, 0, 0, false);

#if 0
    osgDB::makeDirectoryForFile(Stringify() << "out/" << getName() << "/" << key.str() << ".out.png");
    osgDB::writeImageFile(*rasterizedFeatures.getImage(), Stringify() << "out/" << getName() << "/" << key.str() << ".out.png");
    osg::ref_ptr < osg::Image > rgba = redToRGBA(sdf);
    osgDB::writeImageFile(*rgba.get(), Stringify() << "out/" << getName() << "/" << key.str() << ".sdf.png");
    rgba = redToRGBA(cropped.getImage());
    osgDB::writeImageFile(*rgba.get(), Stringify() << "out/" << getName() << "/" << key.str() << "_cropped.sdf.png");

#endif    
    return cropped;
}
