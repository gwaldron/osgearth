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
#include <osgEarth/XYZFeatureSource>
#include <osgEarth/OgrUtils>
#include <osgEarth/GeometryUtils>
#include <osgEarth/FeatureCursor>
#include <osgEarth/Filter>
#include <osgEarth/MVT>
#include <osgEarth/Registry>

#define LC "[XYZFeatureSource] "

using namespace osgEarth;
#define OGR_SCOPED_LOCK GDAL_SCOPED_LOCK

//........................................................................

Config
XYZFeatureSource::Options::getConfig() const
{
    Config conf = FeatureSource::Options::getConfig();
    conf.set("url", _url);
    conf.set("format", _format);
    conf.set("min_level", _minLevel);
    conf.set("max_level", _maxLevel);
    return conf;
}

void
XYZFeatureSource::Options::fromConfig(const Config& conf)
{
    format().setDefault("json");

    conf.get("url", _url);
    conf.get("format", _format);
    conf.get("min_level", _minLevel);
    conf.get("max_level", _maxLevel);
}

//........................................................................

REGISTER_OSGEARTH_LAYER(xyzfeatures, XYZFeatureSource);

OE_LAYER_PROPERTY_IMPL(XYZFeatureSource, URI, URL, url);
OE_LAYER_PROPERTY_IMPL(XYZFeatureSource, std::string, Format, format);
OE_LAYER_PROPERTY_IMPL(XYZFeatureSource, int, MinLevel, minLevel);
OE_LAYER_PROPERTY_IMPL(XYZFeatureSource, int, MaxLevel, maxLevel);

Status
XYZFeatureSource::openImplementation()
{
    Status parent = FeatureSource::openImplementation();
    if (parent.isError())
        return parent;

    FeatureProfile* fp = 0L;

    // Try to get the results from the settings instead
    if (!options().profile().isSet())
    {
        return Status(Status::ConfigurationError, "XYZ driver requires an explicit profile");
    }

    if (!options().minLevel().isSet() || !options().maxLevel().isSet())
    {
        return Status(Status::ConfigurationError, "XYZ driver requires a min and max level");
    }

    _template = options().url()->full();

    _rotateStart = _template.find("[");
    _rotateEnd = _template.find("]");
    if (_rotateStart != std::string::npos && _rotateEnd != std::string::npos && _rotateEnd - _rotateStart > 1)
    {
        _rotateString = _template.substr(_rotateStart, _rotateEnd - _rotateStart + 1);
        _rotateChoices = _template.substr(_rotateStart + 1, _rotateEnd - _rotateStart - 1);
    }


    osg::ref_ptr<const Profile> profile = Profile::create(options().profile().get());
    fp = new FeatureProfile(profile->getExtent());
    fp->setFirstLevel(options().minLevel().get());
    fp->setMaxLevel(options().maxLevel().get());
    fp->setTilingProfile(profile.get());
    if (options().geoInterp().isSet())
    {
        fp->geoInterp() = options().geoInterp().get();
    }

    setFeatureProfile(fp);

    return Status::NoError;
}

void
XYZFeatureSource::init()
{
    FeatureSource::init();

    _rotateStart = 0.0;
    _rotateEnd = 0.0;
}


FeatureCursor*
XYZFeatureSource::createFeatureCursorImplementation(const Query& query, ProgressCallback* progress)
{
    FeatureCursor* result = 0L;

    URI uri = createURL(query);
    if (uri.empty()) return 0;

    OE_DEBUG << LC << uri.full() << std::endl;

    // read the data:
    ReadResult r = uri.readString(getReadOptions(), progress);

    const std::string& buffer = r.getString();
    const Config&      meta = r.metadata();

    bool dataOK = false;

    FeatureList features;
    if (!buffer.empty())
    {
        // Get the mime-type from the metadata record if possible
        std::string mimeType = r.metadata().value(IOMetadata::CONTENT_TYPE);
        //If the mimetype is empty then try to set it from the format specification
        if (mimeType.empty())
        {
            if (options().format().value() == "json")
                mimeType = "json";
            else if (options().format().value().compare("gml") == 0)
                mimeType = "text/xml";
            else if (options().format().value().compare("pbf") == 0)
                mimeType = "application/x-protobuf";
        }
        dataOK = getFeatures(buffer, *query.tileKey(), mimeType, features);
    }

    if (dataOK)
    {
        OE_DEBUG << LC << "Read " << features.size() << " features" << std::endl;
    }

    //If we have any filters, process them here before the cursor is created
    if (getFilters() && !getFilters()->empty() && !features.empty())
    {
        FilterContext cx;
        cx.setProfile(getFeatureProfile());
        cx.extent() = query.tileKey()->getExtent();

        for (FeatureFilterChain::const_iterator i = getFilters()->begin(); i != getFilters()->end(); ++i)
        {
            FeatureFilter* filter = i->get();
            cx = filter->push(features, cx);
        }
    }

    // If we have any features and we have an fid attribute, override the fid of the features
    if (options().fidAttribute().isSet())
    {
        for (FeatureList::iterator itr = features.begin(); itr != features.end(); ++itr)
        {
            std::string attr = itr->get()->getString(options().fidAttribute().get());
            FeatureID fid = as<FeatureID>(attr, 0);
            itr->get()->setFID(fid);
        }
    }

    //result = new FeatureListCursor(features);
    result = dataOK ? new FeatureListCursor(features) : 0L;

    return result;
}

bool
XYZFeatureSource::getFeatures(const std::string& buffer, const TileKey& key, const std::string& mimeType, FeatureList& features)
{
    if (mimeType == "application/x-protobuf" || mimeType == "binary/octet-stream" || mimeType == "application/octet-stream")
    {
#ifdef OSGEARTH_HAVE_MVT
        std::stringstream in(buffer);
        return MVT::readTile(in, key, features);
#else
        if (getStatus().isOK())
        {
            setStatus(Status::ResourceUnavailable, "osgEarth is not built with MVT/PBF support");
            OE_WARN << LC << getStatus().message() << std::endl;
        }
        return false;
#endif
    }
    else
    {
        // find the right driver for the given mime type
        OGR_SCOPED_LOCK;

        // find the right driver for the given mime type
        OGRSFDriverH ogrDriver =
            isJSON(mimeType) ? OGRGetDriverByName("GeoJSON") :
            isGML(mimeType) ? OGRGetDriverByName("GML") :
            0L;

        // fail if we can't find an appropriate OGR driver:
        if (!ogrDriver)
        {
            OE_WARN << LC << "Error reading TFS response; cannot grok content-type \"" << mimeType << "\""
                << std::endl;
            return false;
        }

        OGRDataSourceH ds = OGROpen(buffer.c_str(), FALSE, &ogrDriver);

        if (!ds)
        {
            OE_WARN << LC << "Error reading TFS response" << std::endl;
            return false;
        }

        // read the feature data.
        OGRLayerH layer = OGR_DS_GetLayer(ds, 0);
        if (layer)
        {
            const SpatialReference* srs = getFeatureProfile()->getSRS();

            OGR_L_ResetReading(layer);
            OGRFeatureH feat_handle;
            while ((feat_handle = OGR_L_GetNextFeature(layer)) != NULL)
            {
                if (feat_handle)
                {
                    osg::ref_ptr<Feature> f = OgrUtils::createFeature(feat_handle, getFeatureProfile(), *_options->rewindPolygons());
                    if (f.valid() && !isBlacklisted(f->getFID()))
                    {
                        features.push_back(f.release());
                    }
                    OGR_F_Destroy(feat_handle);
                }
            }
        }

        // Destroy the datasource
        OGR_DS_Destroy(ds);
    }

    return true;
}


std::string
XYZFeatureSource::getExtensionForMimeType(const std::string& mime)
{
    //OGR is particular sometimes about the extension of files when it's reading them so it's good to have
    //the temp file have an appropriate extension
    if ((mime.compare("text/xml") == 0) ||
        (mime.compare("text/xml; subtype=gml/2.1.2") == 0) ||
        (mime.compare("text/xml; subtype=gml/3.1.1") == 0)
        )
    {
        return ".xml";
    }
    else if ((mime.compare("application/json") == 0) ||
        (mime.compare("json") == 0) ||

        (mime.compare("application/x-javascript") == 0) ||
        (mime.compare("text/javascript") == 0) ||
        (mime.compare("text/x-javascript") == 0) ||
        (mime.compare("text/x-json") == 0)
        )
    {
        return ".json";
    }
    return "";
}

bool
XYZFeatureSource::isGML(const std::string& mime) const
{
    return
        startsWith(mime, "text/xml");
}


bool
XYZFeatureSource::isJSON(const std::string& mime) const
{
    return
        (mime.compare("application/json") == 0) ||
        (mime.compare("json") == 0) ||

        (mime.compare("application/x-javascript") == 0) ||
        (mime.compare("text/javascript") == 0) ||
        (mime.compare("text/x-javascript") == 0) ||
        (mime.compare("text/x-json") == 0);
}

URI
XYZFeatureSource::createURL(const Query& query)
{
    if (query.tileKey().isSet())
    {
        const TileKey &key = query.tileKey().get();
        unsigned int tileX = key.getTileX();
        unsigned int tileY = key.getTileY();
        unsigned int level = key.getLevelOfDetail();

        unsigned int numRows, numCols;
        key.getProfile()->getNumTiles(key.getLevelOfDetail(), numCols, numRows);
        unsigned inverted_tileY = numRows - tileY - 1;

        std::string location = _template;

        // support OpenLayers template style:
        replaceIn(location, "${x}", Stringify() << tileX);
        replaceIn(location, "${y}", Stringify() << tileY);
        replaceIn(location, "${-y}", Stringify() << inverted_tileY);
        replaceIn(location, "${z}", Stringify() << key.getLevelOfDetail());

        // failing that, legacy osgearth style:
        replaceIn(location, "{x}", Stringify() << tileX);
        replaceIn(location, "{y}", Stringify() << tileY);
        replaceIn(location, "{-y}", Stringify() << inverted_tileY);
        replaceIn(location, "{z}", Stringify() << key.getLevelOfDetail());

        std::string cacheKey;

        if (!_rotateChoices.empty())
        {
            cacheKey = location;
            unsigned index = (++_rotate_iter) % _rotateChoices.size();
            replaceIn(location, _rotateString, Stringify() << _rotateChoices[index]);
        }


        URI uri(location, options().url()->context());
        if (!cacheKey.empty())
        {
            uri.setCacheKey(Cache::makeCacheKey(location, "uri"));
        }

        return uri;
    }
    return URI();
}