/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2018 Pelican Mapping
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
#include <osgEarth/TFS>

#include <osgEarth/Registry>
#include <osgEarth/XmlUtils>
#include <osgEarth/FileUtils>
#include <osgEarth/Progress>

#include <osgEarth/Filter>
#include <osgEarth/BufferFilter>
#include <osgEarth/ScaleFilter>
#include <osgEarth/MVT>
#include <osgEarth/OgrUtils>
#include <osgEarth/FeatureCursor>

#include <osg/Notify>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <list>
#include <stdio.h>
#include <stdlib.h>

#include <ogr_api.h>

#ifdef WIN32
#include <windows.h>
#endif

#define LC "[TFS FeatureSource] "

using namespace osgEarth;
using namespace osgEarth::TFS;

#define OGR_SCOPED_LOCK GDAL_SCOPED_LOCK

//........................................................................

TFS::Layer::Layer() :
_firstLevel(0),
_maxLevel(8),
_title("layer"),
_srs(SpatialReference::create("EPSG:4326"))
{
    //nop
}

//........................................................................

bool
TFS::ReaderWriter::read(const URI& uri, const osgDB::ReaderWriter::Options *options, TFS::Layer &layer)
{
    osgEarth::ReadResult result = uri.readString(options);
    if (result.succeeded())
    {
        std::string str = result.getString();
        std::stringstream in( str.c_str()  );
        return read( in, layer);
    }    
    return false;
}

bool
TFS::ReaderWriter::read( std::istream &in, TFS::Layer &layer)
{
    osg::ref_ptr< XmlDocument > doc = XmlDocument::load( in );
    if (!doc.valid()) return false;

    osg::ref_ptr<XmlElement> e_layer = doc->getSubElement( "layer" );
    if (!e_layer.valid()) return false;

    layer.setTitle( e_layer->getSubElementText("title") );
    layer.setAbstract( e_layer->getSubElementText("abstract") );
    layer.setFirstLevel( as<unsigned int>(e_layer->getSubElementText("firstlevel"), 0) );
    layer.setMaxLevel( as<unsigned int>(e_layer->getSubElementText("maxlevel"), 0) );

    std::string srsString = e_layer->getSubElementText("srs");
    if (!srsString.empty())
    {
        const SpatialReference* srs = SpatialReference::create( srsString );
        if (srs)
        {
            layer.setSRS( srs );
        }
    }

     //Read the bounding box
    osg::ref_ptr<XmlElement> e_bounding_box = e_layer->getSubElement("boundingbox");
    if (e_bounding_box.valid())
    {
        double minX = as<double>(e_bounding_box->getAttr( "minx" ), 0.0);
        double minY = as<double>(e_bounding_box->getAttr( "miny" ), 0.0);
        double maxX = as<double>(e_bounding_box->getAttr( "maxx" ), 0.0);
        double maxY = as<double>(e_bounding_box->getAttr( "maxy" ), 0.0);
        layer.setExtent( GeoExtent( layer.getSRS(), minX, minY, maxX, maxY) );
    }    

    return true;
}

void
TFS::ReaderWriter::write(const TFS::Layer &layer, const std::string &location)
{
    std::string path = osgDB::getFilePath(location);
    if (!osgDB::fileExists(path) && !osgDB::makeDirectory(path))
    {
        OE_WARN << "Couldn't create path " << std::endl;
    }
    std::ofstream out(location.c_str());
    write(layer, out);
}

void
TFS::ReaderWriter::write(const TFS::Layer& layer, std::ostream& output)
{
    //Create the root XML document
    osg::ref_ptr<XmlDocument> doc = new XmlDocument();
    doc->setName("Layer");

    doc->addSubElement("Title", layer.getTitle());
    doc->addSubElement("Abstract", layer.getAbstract());
    doc->addSubElement("MaxLevel", toString<unsigned int>(layer.getMaxLevel()));
    doc->addSubElement("FirstLevel", toString<unsigned int>(layer.getFirstLevel()));

    osg::ref_ptr<XmlElement> e_bounding_box = new XmlElement("BoundingBox");
    e_bounding_box->getAttrs()["minx"] = toString(layer.getExtent().xMin());
    e_bounding_box->getAttrs()["miny"] = toString(layer.getExtent().yMin());
    e_bounding_box->getAttrs()["maxx"] = toString(layer.getExtent().xMax());
    e_bounding_box->getAttrs()["maxy"] = toString(layer.getExtent().yMax());
    doc->getChildren().push_back(e_bounding_box.get());

    doc->addSubElement("SRS", layer.getSRS()->getHorizInitString());

    doc->store(output);
}


//........................................................................

Config
TFSFeatureSourceOptions::getConfig() const
{
    Config conf = FeatureSource::Options::getConfig();
    conf.set("url", _url);
    conf.set("format", _format);
    conf.set("invert_y", _invertY);
    conf.set("min_level", _minLevel);
    conf.set("max_level", _maxLevel);
    return conf;
}

void
TFSFeatureSourceOptions::fromConfig(const Config& conf)
{
    format().init("json");

    conf.get("url", _url);
    conf.get("format", _format);
    conf.get("invert_y", _invertY);
    conf.get("min_level", _minLevel);
    conf.get("max_level", _maxLevel);
}

//........................................................................

REGISTER_OSGEARTH_LAYER(TFSFeatures, TFSFeatureSource);

OE_LAYER_PROPERTY_IMPL(TFSFeatureSource, URI, URL, url);
OE_LAYER_PROPERTY_IMPL(TFSFeatureSource, std::string, Format, format);
OE_LAYER_PROPERTY_IMPL(TFSFeatureSource, bool, InvertY, invertY);
OE_LAYER_PROPERTY_IMPL(TFSFeatureSource, int, MinLevel, minLevel);
OE_LAYER_PROPERTY_IMPL(TFSFeatureSource, int, MaxLevel, maxLevel);

void
TFSFeatureSource::init()
{
    FeatureSource::init();
    _layerValid = false;
}

Status
TFSFeatureSource::openImplementation()
{
    Status parent = FeatureSource::openImplementation();
    if (parent.isError())
        return parent;

    FeatureProfile* fp = 0L;

    // Try to read the TFS metadata:
    _layerValid = TFS::ReaderWriter::read(options().url().get(), _readOptions.get(), _layer);

    if (_layerValid)
    {
        OE_INFO << LC << "Read layer TFS " << _layer.getTitle() << " " << _layer.getAbstract() << " " << _layer.getFirstLevel() << " " << _layer.getMaxLevel() << " " << _layer.getExtent().toString() << std::endl;

        fp = new FeatureProfile(_layer.getExtent());
        fp->setFirstLevel(_layer.getFirstLevel());
        fp->setMaxLevel(_layer.getMaxLevel());
        fp->setTilingProfile(osgEarth::Profile::create(_layer.getSRS(), _layer.getExtent().xMin(), _layer.getExtent().yMin(), _layer.getExtent().xMax(), _layer.getExtent().yMax(), 1, 1));
        if (options().geoInterp().isSet())
        {
            fp->geoInterp() = options().geoInterp().get();
        }
    }
    else
    {
        // Try to get the results from the settings instead
        if (!options().profile().isSet())
        {
            return Status(Status::ConfigurationError, "TFS driver requires an explicit profile");
        }

        if (!options().minLevel().isSet() || !options().maxLevel().isSet())
        {
            return Status(Status::ConfigurationError, "TFS driver requires a min and max level");
        }

        osg::ref_ptr<const Profile> profile = Profile::create(*options().profile());
        if (!profile.valid())
        {
            return Status(Status::ConfigurationError, "Failed to establish valid Profile");
        }

        fp = new FeatureProfile(profile->getExtent());
        fp->setFirstLevel(*options().minLevel());
        fp->setMaxLevel(*options().maxLevel());
        fp->setTilingProfile(profile.get());
        if (options().geoInterp().isSet())
        {
            fp->geoInterp() = options().geoInterp().get();
        }
    }

    setFeatureProfile(fp);

    return Status::NoError;
}


FeatureCursor*
TFSFeatureSource::createFeatureCursor(const Query& query, ProgressCallback* progress)
{
    FeatureCursor* result = 0L;

    std::string url = createURL(query);

    // the URL wil lbe empty if it was invalid or outside the level bounds of the layer.
    if (url.empty())
        return 0L;

    OE_DEBUG << LC << url << std::endl;
    URI uri(url, options().url()->context());

    // read the data:
    ReadResult r = uri.readString(_readOptions.get(), progress);

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
            if (options().format().value() == "json") mimeType = "json";
            else if (options().format().value().compare("gml") == 0) mimeType = "text/xml";
            else if (options().format().value().compare("pbf") == 0) mimeType = "application/x-protobuf";
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
            FeatureID fid = as<long>(attr, 0);
            itr->get()->setFID(fid);
        }
    }

    result = new FeatureListCursor(features);
    return result;
}


bool
TFSFeatureSource::getFeatures(const std::string& buffer, const TileKey& key, const std::string& mimeType, FeatureList& features)
{
    if (mimeType == "application/x-protobuf" || mimeType == "binary/octet-stream")
    {
#ifdef OSGEARTH_HAVE_MVT
        std::stringstream in(buffer);
        return MVT::readTile(in, key, features);
#else
        if (getStatus().isOK())
        {
            setStatus(Status::ResourceUnavailable, "osgEarth is not built with MVT/PBF support (mime-type=application/x-protobuf)");
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
            const SpatialReference* srs = _layer.getSRS();

            OGR_L_ResetReading(layer);
            OGRFeatureH feat_handle;
            while ((feat_handle = OGR_L_GetNextFeature(layer)) != NULL)
            {
                if (feat_handle)
                {
                    osg::ref_ptr<Feature> f = OgrUtils::createFeature(feat_handle, getFeatureProfile());
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
TFSFeatureSource::getExtensionForMimeType(const std::string& mime)
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
TFSFeatureSource::isGML(const std::string& mime) const
{
    return
        startsWith(mime, "text/xml");
}


bool
TFSFeatureSource::isJSON(const std::string& mime) const
{
    return
        (mime.compare("application/json") == 0) ||
        (mime.compare("json") == 0) ||

        (mime.compare("application/x-javascript") == 0) ||
        (mime.compare("text/javascript") == 0) ||
        (mime.compare("text/x-javascript") == 0) ||
        (mime.compare("text/x-json") == 0);
}

std::string
TFSFeatureSource::createURL(const Query& query)
{
    if (query.tileKey().isSet())
    {
        const TileKey& key = query.tileKey().get();
        unsigned int tileX = key.getTileX();
        unsigned int tileY = key.getTileY();
        unsigned int level = key.getLevelOfDetail();

        // attempt to verify that the request is within the first and max level
        // of the data source.
        const FeatureProfile* fp = getFeatureProfile();
        if (fp && fp->isTiled())
        {
            if (fp->getFirstLevel() > (int)level || fp->getMaxLevel() < (int)level)
            {
                return "";
            }
        }

        // TFS follows the same protocol as TMS, with the origin in the lower left of the profile.
        // osgEarth TileKeys are upper left origin, so we need to invert the tilekey to request the correct key.            
        if (options().invertY() == false)
        {
            unsigned int numRows, numCols;
            key.getProfile()->getNumTiles(key.getLevelOfDetail(), numCols, numRows);
            tileY = numRows - tileY - 1;
        }

        std::stringstream buf;
        std::string path = osgDB::getFilePath(options().url()->full());
        buf << path << "/" << level << "/"
            << tileX << "/"
            << tileY
            << "." << options().format().get();
        return buf.str();
    }
    return "";
}
