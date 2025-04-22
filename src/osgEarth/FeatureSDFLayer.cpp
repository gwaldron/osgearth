/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include "FeatureSDFLayer"
#include "FeatureRasterizer"
#include "FeatureStyleSorter"

using namespace osgEarth;
using namespace osgEarth::Util;

#undef LC
#define LC "[FeatureSDF] "

REGISTER_OSGEARTH_LAYER(featuresdf, FeatureSDFLayer);

Config
FeatureSDFLayer::Options::getConfig() const
{
    Config conf = ImageLayer::Options::getConfig();
    features().set(conf, "features");
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
    features().get(conf, "features");
    styleSheet().get(conf, "styles");

    const Config& filtersConf = conf.child("filters");
    for (ConfigSet::const_iterator i = filtersConf.children().begin(); i != filtersConf.children().end(); ++i)
        filters().push_back(ConfigOptions(*i));
}

void
FeatureSDFLayer::init()
{
    ImageLayer::init();

    // The SDF shader is no good. Too many memory barriers.
    _sdfGenerator.setUseGPU(false);
}

Status
FeatureSDFLayer::openImplementation()
{
    Status parent = ImageLayer::openImplementation();
    if (parent.isError())
        return parent;

    // assert a feature source:
    Status fsStatus = options().features().open(getReadOptions());
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

    options().features().addedToMap(map);
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
    options().features().removedFromMap(map);
    options().styleSheet().removedFromMap(map);

    ImageLayer::removedFromMap(map);
}

void
FeatureSDFLayer::setFeatureSource(FeatureSource* fs)
{
    if (getFeatureSource() != fs)
    {
        options().features().setLayer(fs);
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

        DataExtentList dataExtents;

        if (fp)
        {
            // recalculate the data extents based on the feature source.
            if (fp->getTilingProfile() != NULL)
            {
                // Use specified profile's GeoExtent
                unsigned maxLevel = fp->getMaxLevel();
                if (options().maxDataLevel().isSet())
                    maxLevel = osg::maximum(maxLevel, options().maxDataLevel().get());
                dataExtents.push_back(DataExtent(fp->getTilingProfile()->getExtent(), fp->getFirstLevel(), maxLevel));
            }
            else if (fp->getExtent().isValid() == true)
            {
                // Use FeatureProfile's GeoExtent
                dataExtents.push_back(DataExtent(fp->getExtent()));
            }
        }

        setDataExtents(dataExtents);

        _session->setFeatureSource(getFeatureSource());
        _session->setStyles(getStyleSheet());
    }
}

GeoImage
FeatureSDFLayer::createImageImplementation(const TileKey& key, ProgressCallback* progress) const
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

    // expand the extent to account for a reasonable SDF distance,
    // so we don't get tiling effects
    featuresExtent.expand(
        key.getExtent().width(), 
        key.getExtent().height());


    // Poor man's degrees-to-meters conversion    
    double toMeters = 1.0;
    const GeoExtent& sdfExtent = key.getExtent();
    if (sdfExtent.getSRS()->isGeographic())
    {
        double LAT = sdfExtent.yMin() >= 0.0 ? sdfExtent.yMin() : sdfExtent.yMax();
        toMeters = sdfExtent.getSRS()->getEllipsoid().longitudinalDegreesToMeters(1.0, LAT);
    }

    FilterContext context(_session.get(), key.getExtent());

    // to receive a separate SDF for each different style:
    std::vector<GeoImage> sdfs;

    FeatureStyleSorter::StyleFunction makeDistanceField = [&](const Style& style, FeatureList& features, ProgressCallback* progress)
    {
        if (features.empty())
            return;

        // Render features to a temporary image
        Style r_style;
        if (features.front()->getGeometry()->isLinear())
            r_style.getOrCreate<LineSymbol>()->stroke()->color() = Color::Black;
        else
            r_style.getOrCreate<PolygonSymbol>()->fill()->color() = Color::Black;

        FeatureRasterizer rasterizer(
            2 * getTileSize(),
            2 * getTileSize(),
            featuresExtent,
            Color(1, 1, 1, 0)); // background

        // rasterize the features.
        rasterizer.render(features, r_style, context);
        auto raster = rasterizer.finalize();

        // Next, based on the SDF parameters we are going to create a distance field [0..1]
        // where 0 maps to the min-distance and 1 maps to the the max-distance.
        auto render = style.get<RenderSymbol>();
        float minDistanceMeters = render ? render->sdfMinDistance()->eval() : FLT_MAX;
        float maxDistanceMeters = render ? render->sdfMaxDistance()->eval() : FLT_MAX;

        // Convert the distances to pixels
        double metersPerPixel = toMeters * (featuresExtent.width() / (double)raster.getImage()->s());

        // We couldn't compute a valid min/max distance from the features b/c no features were rendered
        // So initialize it to something reasonable.
        if (minDistanceMeters == FLT_MAX || maxDistanceMeters == FLT_MAX)
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

        osg::ref_ptr<osg::Image> sdf = _sdfGenerator.createDistanceField(raster.getImage(), minPixels, maxPixels);
        sdfs.emplace_back(sdf.get(), featuresExtent);
    };

    FeatureStyleSorter sorter;

    // rasterize each group of features into a unified image:
    sorter.sort(
        key,
        Distance(key.getExtent().width() / 2.0, key.getExtent().getSRS()->getUnits()),
        _session.get(), 
        _filterChain,
        nullptr,
        makeDistanceField,
        progress);

    if (sdfs.size() == 0)
    {
        return GeoImage::INVALID;
    }

    // we need to crop the resulting image back to the key extent and return it:

    if (sdfs.size() == 1)
    {
        return sdfs.front().crop(key.getExtent(), false, 0, 0, false);
    }

    // more than 1? composite them with a "less than" function.
    osg::Image* image = const_cast<osg::Image*>(sdfs[0].getImage());
    ImageUtils::PixelReader read_dst(image);
    ImageUtils::PixelWriter write_dst(image);
    osg::Vec4f src, dst;
    for(std::size_t k=1; k<sdfs.size(); ++k)
    {
        ImageUtils::PixelReader read_src(sdfs[k].getImage());
        read_dst.forEachPixel([&](auto& i)
        {
            read_src(src, i.s(), i.t(), i.r());
            read_dst(dst, i.s(), i.t(), i.r());
            if (src.r() < dst.r())
            {
                write_dst(src, i.s(), i.t(), i.r());
            }
        });
    }

    return sdfs.front().crop(key.getExtent(), false, 0, 0, false);
}
