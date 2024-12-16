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
#include <osgEarth/WFS>

#include <osgEarth/Registry>
#include <osgEarth/FileUtils>
#include <osgEarth/URI>
#include <osgEarth/XmlUtils>

#include <osgEarth/Filter>
#include <osgEarth/OgrUtils>

#include <osg/Notify>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <list>
#include <stdio.h>
#include <stdlib.h>

#include <ogr_api.h>

#define LC "[WFSFeatureSource] "

using namespace osgEarth;
using namespace osgEarth::WFS;

//........................................................................

#define ATTR_VERSION "version"
#define ELEM_CAPABILITY "capability"
#define ELEM_SERVICE "service"
#define ELEM_REQUEST "request"
#define ELEM_ABSTRACT "abstract"
#define ELEM_TILED "tiled"
#define ELEM_MAXLEVEL "maxlevel"
#define ELEM_FIRSTLEVEL "firstlevel"
#define ELEM_FORMAT "format"
#define ELEM_NAME "name"
#define ELEM_TITLE "title"
#define ELEM_SRS "srs"
#define ELEM_FEATURETYPELIST "featuretypelist"
#define ELEM_FEATURETYPE "featuretype"
#define ELEM_LATLONGBOUNDINGBOX "latlongboundingbox"
#define ATTR_MINX              "minx"
#define ATTR_MINY              "miny"
#define ATTR_MAXX              "maxx"
#define ATTR_MAXY              "maxy"

//........................................................................

WFS::FeatureType::FeatureType() :
_tiled(false),
_maxLevel(0),
_firstLevel(0)
{
    //nop
}

//........................................................................

WFS::Capabilities::Capabilities()
{
}

WFS::FeatureType*
WFS::Capabilities::getFeatureTypeByName(const std::string& name)
{
    for (WFS::FeatureTypeList::iterator itr = _featureTypes.begin(); itr != _featureTypes.end(); ++itr)
    {
        if (osgDB::equalCaseInsensitive(itr->get()->getName(),name)) return itr->get();
    }
    return NULL;
}

//........................................................................

WFS::Capabilities*
WFS::CapabilitiesReader::read(const URI& location, const osgDB::Options* dbOptions)
{
    // read the data into a string buffer and parse it from there
    std::string buffer = location.readString(dbOptions).getString();
    if (!buffer.empty())
    {
        std::stringstream buf(buffer);
        return read(buf);
    }
    else return 0L;
}

WFS::Capabilities*
WFS::CapabilitiesReader::read(std::istream &in)
{
    osg::ref_ptr<WFS::Capabilities> capabilities = new WFS::Capabilities;

    osg::ref_ptr<XmlDocument> doc = XmlDocument::load(in);
    if (!doc.valid() || doc->getChildren().empty())
    {
        OE_NOTICE << "Failed to load Capabilities " << std::endl;
        return 0;
    }

    //Get the Capabilities version
    osg::ref_ptr<XmlElement> e_root = static_cast<XmlElement*>(doc->getChildren()[0].get());
    capabilities->setVersion(e_root->getAttr(ATTR_VERSION));

    osg::ref_ptr<XmlElement> e_service = e_root->getSubElement(ELEM_SERVICE);
    if (!e_service.valid())
    {
        OE_NOTICE << "Could not find Service element" << std::endl;
        return 0;
    }


    //Read the parameters from the Service block
    capabilities->setName(e_service->getSubElementText(ELEM_NAME));
    capabilities->setAbstract(e_service->getSubElementText(ELEM_ABSTRACT));
    capabilities->setTitle(e_service->getSubElementText(ELEM_TITLE));

    //Read all the feature types
    osg::ref_ptr<XmlElement> e_feature_types = e_root->getSubElement(ELEM_FEATURETYPELIST);
    if (e_feature_types.valid())
    {
        XmlNodeList featureTypes = e_feature_types->getSubElements(ELEM_FEATURETYPE);
        for (XmlNodeList::const_iterator itr = featureTypes.begin(); itr != featureTypes.end(); itr++)
        {
            XmlElement* e_featureType = static_cast<XmlElement*>(itr->get());
            WFS::FeatureType* featureType = new WFS::FeatureType();
            featureType->setName(e_featureType->getSubElementText(ELEM_NAME));
            featureType->setTitle(e_featureType->getSubElementText(ELEM_TITLE));
            featureType->setAbstract(e_featureType->getSubElementText(ELEM_ABSTRACT));

            //NOTE:  TILED and MAXLEVEL aren't part of the WFS spec, these are enhancements to our server for tiled WFS access
            std::string tiledStr = e_featureType->getSubElementText(ELEM_TILED);
            if (tiledStr.compare("") != 0)
            {
                featureType->setTiled(as<bool>(tiledStr, false));
            }

            std::string maxLevelStr = e_featureType->getSubElementText(ELEM_MAXLEVEL);
            if (maxLevelStr.compare("") != 0)
            {
                featureType->setMaxLevel(as<int>(maxLevelStr, -1));
            }

            std::string firstLevelStr = e_featureType->getSubElementText(ELEM_FIRSTLEVEL);
            if (firstLevelStr.compare("") != 0)
            {
                featureType->setFirstLevel(as<int>(firstLevelStr, 0));
            }

            // Read the SRS
            std::string srsText = e_featureType->getSubElementText(ELEM_SRS);
            if (srsText.compare("") != 0)
            {
                featureType->setSRS(srsText);
            }

            osg::ref_ptr<XmlElement> e_bb = e_featureType->getSubElement(ELEM_LATLONGBOUNDINGBOX);
            if (e_bb.valid())
            {
                double minX, minY, maxX, maxY;
                minX = as<double>(e_bb->getAttr(ATTR_MINX), 0);
                minY = as<double>(e_bb->getAttr(ATTR_MINY), 0);
                maxX = as<double>(e_bb->getAttr(ATTR_MAXX), 0);
                maxY = as<double>(e_bb->getAttr(ATTR_MAXY), 0);
                featureType->setExtent(GeoExtent(osgEarth::SpatialReference::create(srsText), minX, minY, maxX, maxY));
            }

            capabilities->getFeatureTypes().push_back(featureType);
        }
    }


    return capabilities.release();
}


//........................................................................

Config
WFSFeatureSourceOptions::getConfig() const
{
    Config conf = FeatureSource::Options::getConfig();
    conf.set("url", url());
    conf.set("typename", typeName());
    conf.set("outputformat", outputFormat());
    conf.set("maxfeatures", maxFeatures());
    conf.set("disable_tiling", disableTiling());
    conf.set("request_buffer", buffer());
    return conf;
}

void
WFSFeatureSourceOptions::fromConfig(const Config& conf)
{
    maxFeatures().init(0u);
    disableTiling().init(false);
    buffer().init(0.0);

    conf.get("url", url());
    conf.get("typename", typeName());
    conf.get("outputformat", outputFormat());
    conf.get("maxfeatures", maxFeatures());
    conf.get("disable_tiling", disableTiling());
    conf.get("request_buffer", buffer());
}

//........................................................................

REGISTER_OSGEARTH_LAYER(wfsfeatures, WFSFeatureSource);

OE_LAYER_PROPERTY_IMPL(WFSFeatureSource, URI, URL, url);
OE_LAYER_PROPERTY_IMPL(WFSFeatureSource, std::string, TypeName, typeName);
OE_LAYER_PROPERTY_IMPL(WFSFeatureSource, unsigned, MaxFeatures, maxFeatures);
OE_LAYER_PROPERTY_IMPL(WFSFeatureSource, std::string, OutputFormat, outputFormat);
OE_LAYER_PROPERTY_IMPL(WFSFeatureSource, bool, DisableTiling, disableTiling);
OE_LAYER_PROPERTY_IMPL(WFSFeatureSource, double, Buffer, buffer);

void
WFSFeatureSource::init()
{
    FeatureSource::init();
}

Status
WFSFeatureSource::openImplementation()
{
    Status parent = FeatureSource::openImplementation();
    if (parent.isError())
        return parent;

    // parse the WFS capabilities URL
    std::string capUrl;
    if (options().url().isSet())
    {
        char sep = options().url()->full().find_first_of('?') == std::string::npos ? '?' : '&';

        capUrl =
            options().url()->full() +
            sep +
            "SERVICE=WFS&VERSION=1.0.0&REQUEST=GetCapabilities";
    }

    // read the WFS capabilities:
    _capabilities = WFS::CapabilitiesReader::read(capUrl, getReadOptions());
    if (!_capabilities.valid())
    {
        return Status(Status::ResourceUnavailable, Stringify() << "Failed to read WFS GetCapabilities from \"" << capUrl << "\"");
    }
    else
    {
        OE_INFO << "[osgEarth::WFS] Got capabilities from " << capUrl << std::endl;
    }

    // establish a feature profile
    FeatureProfile* fp = 0L;

    //Find the feature type by name
    osg::ref_ptr< WFS::FeatureType > featureType = _capabilities->getFeatureTypeByName(options().typeName().get());
    if (featureType.valid())
    {
        if (featureType->getExtent().isValid())
        {
            fp = new FeatureProfile(featureType->getExtent());

            bool disableTiling = options().disableTiling().isSetTo(true);

            if (featureType->getTiled() && !disableTiling)
            {
                fp->setFirstLevel(featureType->getFirstLevel());
                fp->setMaxLevel(featureType->getMaxLevel());
                fp->setTilingProfile(osgEarth::Profile::create(
                    osgEarth::SpatialReference::create("epsg:4326"),
                    featureType->getExtent().xMin(), featureType->getExtent().yMin(),
                    featureType->getExtent().xMax(), featureType->getExtent().yMax(),
                    1, 1));
            }
        }
    }

    // if nothing else, fall back on a global geodetic feature profile.
    if (!fp)
    {
        fp = new FeatureProfile(GeoExtent(SpatialReference::create("epsg:4326"), -180, -90, 180, 90));
    }

    if (options().geoInterp().isSet())
    {
        fp->geoInterp() = options().geoInterp().get();
    }

    setFeatureProfile(fp);

    return Status::NoError;
}



void
WFSFeatureSource::saveResponse(const std::string buffer, const std::string& filename) const
{
    std::ofstream fout;
    fout.open(filename.c_str(), std::ios::out | std::ios::binary);
    fout.write(buffer.c_str(), buffer.size());
    fout.close();
}

bool
WFSFeatureSource::getFeatures(const std::string& buffer, const std::string& mimeType, FeatureList& features) const
{
    bool json = isJSON(mimeType);
    bool gml = isGML(mimeType);

    // find the right driver for the given mime type
    OGRSFDriverH ogrDriver =
        json ? OGRGetDriverByName("GeoJSON") :
        gml ? OGRGetDriverByName("GML") :
        0L;

    // fail if we can't find an appropriate OGR driver:
    if (!ogrDriver)
    {
        OE_WARN << LC << "Error reading WFS response; cannot grok content-type \"" << mimeType << "\""
            << std::endl;
        return false;
    }

    auto feature_srs = getFeatureProfile()->getSRS();

    // GeoJSON is always WGS84 according to spec
    // https://datatracker.ietf.org/doc/html/rfc7946
    if (json)
    {
        feature_srs = osgEarth::SpatialReference::create("wgs84");
    }

    std::string tmpName;

    OGRDataSourceH ds = 0;
    //GML needs to be saved to a temp file to load from disk.  GeoJSON can be loaded directly from memory
    if (gml)
    {
        std::string ext = getExtensionForMimeType(mimeType);
        //Save the response to a temp file
        std::string tmpPath = getTempPath();
        tmpName = getTempName(tmpPath, ext);
        saveResponse(buffer, tmpName);
        ds = OGROpen(tmpName.c_str(), FALSE, &ogrDriver);
    }
    else if (json)
    {
        //Open GeoJSON directly from memory
        ds = OGROpen(buffer.c_str(), FALSE, &ogrDriver);
    }


    if (!ds)
    {
        OE_WARN << LC << "Error reading WFS response" << std::endl;
        return false;
    }

    // read the feature data.
    OGRLayerH layer = OGR_DS_GetLayer(ds, 0);
    if (layer)
    {
        OGR_L_ResetReading(layer);
        OGRFeatureH feat_handle;
        while ((feat_handle = OGR_L_GetNextFeature(layer)) != NULL)
        {
            if (feat_handle)
            {
                osg::ref_ptr<Feature> f = OgrUtils::createFeature(feat_handle, feature_srs,
                    getFeatureProfile()->geoInterp(), *_options->rewindPolygons());

                if (f.valid() && !isBlacklisted(f->getFID()))
                {
                    if (feature_srs != getFeatureProfile()->getSRS())
                    {
                        f->transform(getFeatureProfile()->getSRS());
                    }
                    features.push_back(f.release());
                }
                OGR_F_Destroy(feat_handle);
            }
        }
    }

    // Destroy the datasource
    OGR_DS_Destroy(ds);

    //Delete the temp file if one was created
    if (!tmpName.empty())
    {
        remove(tmpName.c_str());
    }

    return true;
}


std::string
WFSFeatureSource::getExtensionForMimeType(const std::string& mime) const
{
    //OGR is particular sometimes about the extension of files when it's reading them so it's good to have
    //the temp file have an appropriate extension
    if (isGML(mime))
    {
        return ".xml";
    }
    else if (isJSON(mime))
    {
        return ".json";
    }
    return "";
}

bool
WFSFeatureSource::isGML(const std::string& mime) const
{
    return
        startsWith(mime, "text/xml");
}


bool
WFSFeatureSource::isJSON(const std::string& mime) const
{
    return
        startsWith(mime, "application/json") ||
        startsWith(mime, "json") ||
        startsWith(mime, "application/x-javascript") ||
        startsWith(mime, "text/javascript") ||
        startsWith(mime, "text/x-javascript") ||
        startsWith(mime, "text/x-json");
}

std::string
WFSFeatureSource::createURL(const Query& query) const
{
    char sep = options().url()->full().find_first_of('?') == std::string::npos ? '?' : '&';

    std::stringstream buf;
    buf.imbue(std::locale::classic());

    buf << options().url()->full() << sep << "SERVICE=WFS&VERSION=1.0.0&REQUEST=GetFeature";
    buf << "&TYPENAME=" << options().typeName().get();

    std::string outputFormat = "geojson";
    if (options().outputFormat().isSet()) outputFormat = options().outputFormat().get();
    buf << "&OUTPUTFORMAT=" << outputFormat;

    // If the Query limit is set, use that.  Otherwise use the globally defined maxFeatures setting.
    if (query.limit().isSet())
    {
        buf << "&MAXFEATURES=" << query.limit().get();
    }
    else if (options().maxFeatures().isSet())
    {
        buf << "&MAXFEATURES=" << options().maxFeatures().get();
    }

    if (query.tileKey().isSet() && getFeatureProfile()->isTiled())
    {
        unsigned int tileX = query.tileKey().get().getTileX();
        unsigned int tileY = query.tileKey().get().getTileY();
        unsigned int level = query.tileKey().get().getLevelOfDetail();

        // Tiled WFS follows the same protocol as TMS, with the origin in the lower left of the profile.
        // osgEarth TileKeys are upper left origin, so we need to invert the tilekey to request the correct key.
        unsigned int numRows, numCols;
        query.tileKey().get().getProfile()->getNumTiles(level, numCols, numRows);
        tileY = numRows - tileY - 1;

        buf << "&Z=" << level <<
            "&X=" << tileX <<
            "&Y=" << tileY;
    }
    // BBOX and CQL_FILTER are mutually exclusive. Give CQL_FILTER priority if specified.
    // NOTE: CQL_FILTER is a non-standard vendor parameter. See:
    // http://docs.geoserver.org/latest/en/user/services/wfs/vendor.html
    else if (query.expression().isSet())
    {
        buf << "&CQL_FILTER=" << osgEarth::URI::urlEncode(query.expression().get());
    }
    else if (query.bounds().isSet())
    {
        double buffer = *options().buffer();
        buf << "&BBOX=" << std::setprecision(16)
            << query.bounds().get().xMin() - buffer << ","
            << query.bounds().get().yMin() - buffer << ","
            << query.bounds().get().xMax() + buffer << ","
            << query.bounds().get().yMax() + buffer;
    }

    std::string str;
    str = buf.str();
    return str;
}

FeatureCursor*
WFSFeatureSource::createFeatureCursorImplementation(const Query& query, ProgressCallback* progress) const
{
    FeatureCursor* result = 0L;

    std::string url = createURL(query);

    URI uri(url, options().url()->context());

    // read the data:
    ReadResult r = uri.readString(getReadOptions(), progress);

    const std::string& buffer = r.getString();
    const Config&      meta = r.metadata();

    bool dataOK = false;

    FeatureList features;
    if (!buffer.empty())
    {
        // Get the mime-type from the metadata record if possible
        const std::string& mimeType = r.metadata().value(IOMetadata::CONTENT_TYPE);
        dataOK = getFeatures(buffer, mimeType, features);
    }

    if (dataOK)
    {
        OE_NULL << LC << "Read " << features.size() << " features" << std::endl;
    }

#if 0 // Done in FeatureSource now
    //If we have any filters, process them here before the cursor is created
    if (!getFilters().empty() && !features.empty())
    {
        FilterContext cx;
        cx.setProfile(getFeatureProfile());
        cx = getFilters().push(features, cx);
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
#endif

    result = dataOK ? new FeatureListCursor(features) : 0L;

    return result;
}