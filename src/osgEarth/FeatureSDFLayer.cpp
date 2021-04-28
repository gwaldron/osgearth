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
#include "TransformFilter"
#include "FeatureRasterizer"

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
    featureSource().get(conf, "features");
    styleSheet().get(conf, "styles");

    const Config& filtersConf = conf.child("filters");
    for (ConfigSet::const_iterator i = filtersConf.children().begin(); i != filtersConf.children().end(); ++i)
        filters().push_back(ConfigOptions(*i));
}

void
FeatureSDFLayer::init()
{
    ImageLayer::init();

    // Default profile (WGS84) if not set
    if (!getProfile())
    {
        setProfile(Profile::create("global-geodetic"));
    }
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

    //_program = new osg::Program();
    //_program->addShader(new osg::Shader(osg::Shader::COMPUTE, jfa_cs));

    return Status::NoError;
}

void
FeatureSDFLayer::establishProfile()
{
    if (getProfile() == nullptr && getFeatureSource() != nullptr)
    {
        const FeatureProfile* fp = getFeatureSource()->getFeatureProfile();

        if (fp->getTilingProfile())
        {
            setProfile(fp->getTilingProfile());
        }
        else if (fp->getSRS())
        {
            setProfile(Profile::create(fp->getSRS()));
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

            // warn the user if the feature data is tiled and the
            // layer profile doesn't match the feature source profile
            if (fp->isTiled() &&
                fp->getTilingProfile()->isHorizEquivalentTo(getProfile()) == false)
            {
                OE_WARN << LC << "Layer profile doesn't match feature tiling profile - data may not render properly" << std::endl;
                OE_WARN << LC << "(Feature tiling profile = " << fp->getTilingProfile()->toString() << ")" << std::endl;
            }
        }

        _session->setFeatureSource(getFeatureSource());
        _session->setStyles(getStyleSheet());
    }
}

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

    // allocate the image.
    osg::ref_ptr<osg::Image> sdf = new osg::Image();
    sdf->allocateImage(getTileSize(), getTileSize(), 1, GL_RED, GL_UNSIGNED_BYTE);
    sdf->setInternalTextureFormat(GL_R8);
    ImageUtils::PixelWriter write(sdf);
    write.assign(Color(1, 1, 1, 1));

    FeatureRasterizer rasterizer(sdf->s(), sdf->t(), key.getExtent(), Color(1, 1, 1, 0));
    osg::ref_ptr<osg::Image> rasterizedFeatures;
    osg::ref_ptr<osg::Image> nnfield;

    FeatureStyleSorter::Function rasterizeFeatures = [&](
        const Style& style,
        FeatureList& features,
        ProgressCallback* progress)
    {
        if (features.empty())
            return;

        // Transform to map SRS:
        FilterContext context(_session.get());
        context.setProfile(getFeatureSource()->getFeatureProfile());
        TransformFilter xform(key.getExtent().getSRS());
        xform.push(features, context);

        // Render features to a temporary image
        Style r_style;
        if (features.front()->getGeometry()->isLinear())
            r_style.getOrCreate<LineSymbol>()->stroke()->color() = Color::Black;
        else
            r_style.getOrCreate<PolygonSymbol>()->fill()->color() = Color::Black;

        rasterizer.render(
            _session.get(), 
            r_style,
            _session->getFeatureSource()->getFeatureProfile(), 
            features);
    };

    FeatureStyleSorter::Function renderSDF = [&](
        const Style& style,
        FeatureList& features,
        ProgressCallback* progress)
    {
        const GeoExtent& extent = key.getExtent();

        // Poor man's degrees-to-meters conversion
        double toMeters = 1.0;
        if (extent.getSRS()->isGeographic())
        {
            double R = extent.getSRS()->getEllipsoid()->getRadiusEquator();
            toMeters = (2.0 * osg::PI * R / 360.0) * cos(osg::DegreesToRadians(extent.yMin()));
        }

        _sdfGenerator.createDistanceField(
            nnfield.get(),
            sdf,
            extent.height() * toMeters,
            style.get<RenderSymbol>()->sdfMinDistance()->eval(),
            style.get<RenderSymbol>()->sdfMaxDistance()->eval(),
            progress);
    };

    FeatureStyleSorter sorter;

    // rasterize each group of features into a unified image:
    sorter.sort(
        key, 
        _session.get(), 
        _filterChain.get(), 
        rasterizeFeatures, 
        progress);

    rasterizedFeatures = rasterizer.finalize();

    // create a NN field from the rasterized data:
    _sdfGenerator.createNearestNeighborField(
        rasterizedFeatures.get(),
        nnfield,
        progress);

    // feed the NN field into each feature group to generate an SDF
    sorter.sort(
        key,
        _session.get(),
        _filterChain.get(),
        renderSDF,
        progress);

    return GeoImage(sdf.get(), key.getExtent());
}
