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
#include <osgEarth/FeatureImageLayer>
#include <osgEarth/Session>
#include <osgEarth/StyleSheet>
#include <osgEarth/Registry>
#include <osgEarth/Progress>
#include <osgEarth/LandCover>
#include <osgEarth/Metrics>

using namespace osgEarth;

#define LC "[FeatureImageLayer] " << getName() << ": "


REGISTER_OSGEARTH_LAYER(featureimage, FeatureImageLayer);
REGISTER_OSGEARTH_LAYER(feature_image, FeatureImageLayer);

//........................................................................

Config
FeatureImageLayer::Options::getConfig() const
{
    Config conf = ImageLayer::Options::getConfig();
    featureSource().set(conf, "features");
    styleSheet().set(conf, "styles");
    conf.set("gamma", gamma());
    conf.set("sdf", sdf());
    conf.set("sdf_invert", sdf_invert());

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
FeatureImageLayer::Options::fromConfig(const Config& conf)
{
    gamma().setDefault(1.3);
    sdf().setDefault(false);
    sdf_invert().setDefault(false);

    featureSource().get(conf, "features");
    styleSheet().get(conf, "styles");
    conf.get("gamma", gamma());
    conf.get("sdf", sdf());
    conf.get("sdf_invert", sdf_invert());

    const Config& filtersConf = conf.child("filters");
    for (ConfigSet::const_iterator i = filtersConf.children().begin(); i != filtersConf.children().end(); ++i)
        filters().push_back(ConfigOptions(*i));
}

//........................................................................

FeatureImageLayer::Isolate::Isolate(const Isolate& rhs) :
    _session(rhs._session),
    _filterChain(rhs._filterChain)
{
}

void
FeatureImageLayer::init()
{
    ImageLayer::init();

    // Default profile (WGS84) if not set
    if (!getProfile())
    {
        setProfile(Profile::create(Profile::GLOBAL_GEODETIC));
    }
}

Status
FeatureImageLayer::openImplementation()
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

    _global._filterChain = FeatureFilterChain::create(
        options().filters(), 
        getReadOptions());

    return Status::NoError;
}

void
FeatureImageLayer::establishProfile()
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
FeatureImageLayer::addedToMap(const Map* map)
{
    ImageLayer::addedToMap(map);

    options().featureSource().addedToMap(map);
    options().styleSheet().addedToMap(map);

    if (getFeatureSource())
    {
        establishProfile();

        _global._session = new Session(
            map, 
            getStyleSheet(), 
            getFeatureSource(), 
            getReadOptions());

        updateSession();
    }
}

void
FeatureImageLayer::removedFromMap(const Map* map)
{
    options().featureSource().removedFromMap(map);
    options().styleSheet().removedFromMap(map);

    ImageLayer::removedFromMap(map);
}

void
FeatureImageLayer::setFeatureSource(FeatureSource* fs)
{
    if (getFeatureSource() != fs)
    {
        options().featureSource().setLayer(fs);
        //_data._featureProfile = 0L;

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
FeatureImageLayer::setStyleSheet(StyleSheet* value)
{
    if (getStyleSheet() != value)
    {
        options().styleSheet().setLayer(value);
        if (_global._session.valid())
        {
            _global._session->setStyles(getStyleSheet());
        }
    }
}

void
FeatureImageLayer::updateSession()
{
    if (_global._session.valid() && getFeatureSource())
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
        }

        _global._session->setFeatureSource(getFeatureSource());
        _global._session->setStyles(getStyleSheet());
    }
}

GeoImage
FeatureImageLayer::createImageImplementation(const TileKey& key, ProgressCallback* progress) const
{
    if (getStatus().isError())
    {
        return GeoImage::INVALID;
    }

    // take local refs to prevent threading issues.
    Isolate local(_global);

    if (!local._session.valid())
    {
        setStatus(Status::AssertionFailure, "session is NULL - call support");
        return GeoImage::INVALID;
    }

    if (!local._session->getFeatureSource())
    {
        setStatus(Status::ServiceUnavailable, "No feature source");
        return GeoImage::INVALID;
    }

    const FeatureProfile* featureProfile = local._session->getFeatureSource()->getFeatureProfile();
    if (featureProfile == nullptr)
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


    FeatureRasterizer* rasterizer = nullptr;

    osg::ref_ptr<osg::Image> image;
    if (local._session->styles())
    {
        for (auto& style : local._session->styles()->getStyles())
        {
            if (style.second.getSymbol<CoverageSymbol>())
            {
                image = LandCover::createImage(
                    getTileSize(),
                    getTileSize());

                rasterizer = new FeatureRasterizer(image.get(), key.getExtent());
                ImageUtils::PixelWriter writer(image.get());
                writer.assign(osg::Vec4(NO_DATA_VALUE, NO_DATA_VALUE, NO_DATA_VALUE, NO_DATA_VALUE));
                break;
            }
        }
    }

    if (!rasterizer)
    {
        rasterizer = new FeatureRasterizer(
            getTileSize(), 
            getTileSize(), 
            key.getExtent());
    }

    FeatureStyleSorter::Function renderer = [&](
        const Style& style,
        FeatureList& features,
        ProgressCallback* progress)
    {
        rasterizer->render(
            features,
            style,
            featureProfile);
    };

    FeatureStyleSorter sorter;

    sorter.sort(
        key,
        Distance(0, Units::METERS),
        local._session.get(),
        local._filterChain.get(),
        renderer,
        progress);

    GeoImage result = rasterizer->finalize();

#if 0
    // If the rasterizer didn't render anything at all return an invalid image
    // This was originally added to avoid caching images where the server had an error and returned no features
    // and also to just save on disk space for transparent images.
    // However, this is causing areas that are actually empty to fall back on parent tiles
    // which is incorrect.  For example, using the osm highways multilevel 
    // vector tile dataset will try to render roads at level 14, find no roads (b/c there aren't any in a tile) and then
    // will try level 13, 12, etc until it finally gets to a tile that contains "some" road data.
    if (ImageUtils::isEmptyImage(result.getImage()))
    {
        result = GeoImage::INVALID;
    }
#endif
    delete rasterizer;

    return result;
}
