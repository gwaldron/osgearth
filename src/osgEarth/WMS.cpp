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
#include "WMS"
#include <osgEarth/XmlUtils>
#include <osgEarth/Registry>
#include <osgEarth/StringUtils>
#include <osgEarth/Registry>
#include <osg/ImageSequence>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>

using namespace osgEarth;
using namespace osgEarth::Util;

#undef LC
#define LC "[WMS] "

//........................................................................

WMS::Style::Style()
{
}

WMS::Style::Style(const std::string& name, const std::string &title)
{
    _name = name;
    _title = title;
}

//........................................................................

WMS::Layer::Layer():
_minLon(0),
_minLat(0),
_maxLon(0),
_maxLat(0),
_minX(0),
_minY(0),
_maxX(0),
_maxY(0),
_parentLayer(0)
{
    //nop
}

void WMS::Layer::getLatLonExtents(double &minLon, double &minLat, double &maxLon, double &maxLat) const
{
    minLon = _minLon;
    minLat = _minLat;
    maxLon = _maxLon;
    maxLat = _maxLat;
}

void WMS::Layer::setLatLonExtents(double minLon, double minLat, double maxLon, double maxLat)
{
    _minLon = minLon;
    _minLat = minLat;
    _maxLon = maxLon;
    _maxLat = maxLat;
}

void WMS::Layer::getExtents(double &minX, double &minY, double &maxX, double &maxY) const
{
    minX = _minX;
    minY = _minY;
    maxX = _maxX;
    maxY = _maxY;
}

void WMS::Layer::setExtents(double minX, double minY, double maxX, double maxY)
{
    _minX = minX;
    _minY = minY;
    _maxX = maxX;
    _maxY = maxY;
}

//........................................................................

WMS::Capabilities::Capabilities()
{
}

std::string
WMS::Capabilities::suggestExtension() const
{
    //Default to png
    std::string ext = "png";

    //Find the first format that we have an osg ReaderWriter for
    for (unsigned int i = 0; i < _formats.size(); ++i)
    {
        std::string format = _formats[i];
        //Strip off the "image/"
        if ((format.length() > 6) && (format.compare(0,6,"image/") == 0))
        {
            format = format.substr(6);
            //See if we have a ReaderWriter for the extension
            osgDB::ReaderWriter* rw = osgDB::Registry::instance()->getReaderWriterForExtension( format );
            if (rw)
            {
                ext = format;
                OE_DEBUG << "suggestExtension found ReaderWriter for " << ext << std::endl;
                break;
            }
        }
    }
    return ext;
}

WMS::Layer*
WMS::Capabilities::getLayerByName(const std::string& name, const WMS::Layer::LayerList& layers) const
{
    for (WMS::Layer::LayerList::const_iterator i = layers.begin(); i != layers.end(); ++i)
    {
        if (osgDB::equalCaseInsensitive(i->get()->getName(), name)) return i->get();
        WMS::Layer *l = getLayerByName(name, i->get()->getLayers());
        if (l) return l;
    }
    return 0;
}

WMS::Layer*
WMS::Capabilities::getLayerByName(const std::string& name) const
{
    return getLayerByName(name, _layers);
}

WMS::Capabilities*
WMS::CapabilitiesReader::read(const URI& location, const osgDB::ReaderWriter::Options* options)
{
    WMS::Capabilities* caps = NULL;
    if (location.isRemote())
    {
        ReadResult rr = location.readString(options);
        if (rr.succeeded())
        {
            std::istringstream in(rr.getString());
            caps = read(in);
        }
    }
    else
    {
        if ((osgDB::fileExists(location.full())) && (osgDB::fileType(location.full()) == osgDB::REGULAR_FILE))
        {
            std::ifstream in(location.full().c_str());
            caps = read(in);
        }
    }
    return caps;
}

#define ATTR_VERSION "version"
#define ELEM_CAPABILITY "capability"
#define ELEM_REQUEST "request"
#define ELEM_ABSTRACT "abstract"
#define ELEM_GETMAP "getmap"
#define ELEM_FORMAT "format"
#define ELEM_LAYER "layer"
#define ELEM_NAME "name"
#define ELEM_TITLE "title"
#define ELEM_STYLE "style"
#define ELEM_SRS "srs"
#define ELEM_CRS "crs"
#define ELEM_LATLONBOUNDINGBOX "latlonboundingbox"
#define ELEM_GEOGRAPHICBOUNDINGBOX "ex_geographicboundingbox"
#define ELEM_BOUNDINGBOX "boundingbox"
#define ATTR_MINX "minx"
#define ATTR_MINY "miny"
#define ATTR_MAXX "maxx"
#define ATTR_MAXY "maxy"

#define ATTR_EASTLON "eastboundlongitude"
#define ATTR_WESTLON "westboundlongitude"
#define ATTR_NORTHLAT "northboundlatitude"
#define ATTR_SOUTHLAT "southboundlatitude"


void
WMS::CapabilitiesReader::readLayers(XmlElement* e, WMS::Layer* parentLayer, WMS::Layer::LayerList& layers)
{
    XmlNodeList layerNodes = e->getSubElements(ELEM_LAYER);
    for (XmlNodeList::const_iterator i = layerNodes.begin(); i != layerNodes.end(); i++)
    {
        XmlElement* e_layer = static_cast<XmlElement*>(i->get());

        WMS::Layer* layer = new WMS::Layer;
        layer->setName(e_layer->getSubElementText(ELEM_NAME));
        layer->setTitle(e_layer->getSubElementText(ELEM_TITLE));
        layer->setAbstract(e_layer->getSubElementText(ELEM_ABSTRACT));

        //Read all the supported styles
        XmlNodeList styles = e_layer->getSubElements(ELEM_STYLE);
        for (XmlNodeList::const_iterator styleitr = styles.begin(); styleitr != styles.end(); styleitr++)
        {
            XmlElement* e_style = static_cast<XmlElement*>(styleitr->get());
            std::string name = e_style->getSubElementText(ELEM_NAME);
            std::string title = e_style->getSubElementText(ELEM_TITLE);
            layer->getStyles().push_back(WMS::Style(name, title));
        }

        //Read all the supported SRS's
        XmlNodeList spatialReferences = e_layer->getSubElements(ELEM_SRS);
        for (XmlNodeList::const_iterator srsitr = spatialReferences.begin(); srsitr != spatialReferences.end(); ++srsitr)
        {
            std::string srs = static_cast<XmlElement*>(srsitr->get())->getText();
            layer->getSpatialReferences().push_back(srs);
        }

        //Read all the supported CRS's
        spatialReferences = e_layer->getSubElements(ELEM_CRS);
        for (XmlNodeList::const_iterator srsitr = spatialReferences.begin(); srsitr != spatialReferences.end(); ++srsitr)
        {
            std::string crs = static_cast<XmlElement*>(srsitr->get())->getText();
            layer->getSpatialReferences().push_back(crs);
        }

        if (parentLayer)
        {
            // Also add in any SRS that is defined in the parent layer.  Some servers, like GeoExpress from LizardTech will publish top level SRS's that also apply to the child layers
            for (WMS::Layer::SRSList::iterator itr = parentLayer->getSpatialReferences().begin(); itr != parentLayer->getSpatialReferences().end(); itr++)
            {
                std::string parentSRS = *itr;
                // Only add the SRS if it's not already present in the SRS list.
                if (std::find(layer->getSpatialReferences().begin(), layer->getSpatialReferences().end(), parentSRS) == layer->getSpatialReferences().end())
                {
                    layer->getSpatialReferences().push_back(parentSRS);
                }
            }
        }

        osg::ref_ptr<XmlElement> e_bb = e_layer->getSubElement(ELEM_LATLONBOUNDINGBOX);
        if (e_bb.valid())
        {
            double minX, minY, maxX, maxY;
            minX = as<double>(e_bb->getAttr(ATTR_MINX), 0);
            minY = as<double>(e_bb->getAttr(ATTR_MINY), 0);
            maxX = as<double>(e_bb->getAttr(ATTR_MAXX), 0);
            maxY = as<double>(e_bb->getAttr(ATTR_MAXY), 0);
            layer->setLatLonExtents(minX, minY, maxX, maxY);
        }
        else {
            osg::ref_ptr<XmlElement> e_gbb = e_layer->getSubElement(ELEM_GEOGRAPHICBOUNDINGBOX);
            if (e_gbb.valid())
            {
                double minX, minY, maxX, maxY;
                minX = as<double>(e_gbb->getSubElementText(ATTR_WESTLON), 0);
                minY = as<double>(e_gbb->getSubElementText(ATTR_SOUTHLAT), 0);
                maxX = as<double>(e_gbb->getSubElementText(ATTR_EASTLON), 0);
                maxY = as<double>(e_gbb->getSubElementText(ATTR_NORTHLAT), 0);
                layer->setLatLonExtents(minX, minY, maxX, maxY);
            }
            else if (parentLayer)
            {
                // inherit from parent if not specified locally
                double minX, minY, maxX, maxY;
                parentLayer->getLatLonExtents(minX, minY, maxX, maxY);
                layer->setLatLonExtents(minX, minY, maxX, maxY);
            }
        }

        e_bb = e_layer->getSubElement(ELEM_BOUNDINGBOX);
        if (e_bb.valid())
        {
            double minX, minY, maxX, maxY;
            minX = as<double>(e_bb->getAttr(ATTR_MINX), 0);
            minY = as<double>(e_bb->getAttr(ATTR_MINY), 0);
            maxX = as<double>(e_bb->getAttr(ATTR_MAXX), 0);
            maxY = as<double>(e_bb->getAttr(ATTR_MAXY), 0);
            layer->setExtents(minX, minY, maxX, maxY);
        }

        //Add the layer to the list and set its parent layer
        layers.push_back(layer);
        layer->setParentLayer(parentLayer);

        //Read any other layers that are in the layer node
        readLayers(e_layer, layer, layer->getLayers());
    }
}

WMS::Capabilities*
WMS::CapabilitiesReader::read(std::istream &in)
{
    osg::ref_ptr<WMS::Capabilities> capabilities = new WMS::Capabilities;

    osg::ref_ptr<XmlDocument> doc = XmlDocument::load( in );
    if (!doc.valid() || doc->getChildren().empty())
    {
        OE_WARN << LC << "Failed to load Capabilities " << std::endl;
        return 0;
    }

    //Get the Capabilities version
    osg::ref_ptr<XmlElement> e_root = static_cast<XmlElement*>(doc->getChildren()[0].get());
    capabilities->setVersion( e_root->getAttr(ATTR_VERSION ) );

    osg::ref_ptr<XmlElement> e_capability = e_root->getSubElement( ELEM_CAPABILITY );
    if (!e_capability.valid())
    {
        OE_WARN << LC << "Could not find Capability element" << std::endl;
        return 0;
    }

    //Get the supported formats
    osg::ref_ptr<XmlElement> e_request = e_capability->getSubElement( ELEM_REQUEST );
    if (e_request.valid())
    {
        osg::ref_ptr<XmlElement> e_getMap = e_request->getSubElement( ELEM_GETMAP );
        if ( e_getMap.valid() )
        {
            //Read all the formats
            XmlNodeList formats = e_getMap->getSubElements( ELEM_FORMAT );
            for( XmlNodeList::const_iterator i = formats.begin(); i != formats.end(); i++ )
            {
                std::string format = trim(static_cast<XmlElement*>( i->get() )->getText());
                capabilities->getFormats().push_back(format);
            }
        }
    }

    //Try to read the layers
    readLayers( e_capability.get(), 0, capabilities->getLayers());

    return capabilities.release();
}

//........................................................................

Config
WMS::WMSImageLayerOptions::getMetadata()
{
    return Config::readJSON( OE_MULTILINE(
        { "name" : "WMS (OGC Web Map Service)",
          "properties": [
            { "name": "url", "description": "Location of the TMS repository", "type": "string", "default": "" },
            { "name": "capabilities_url", "description": "Special URL for requesting capabilities data", "type": "string", "default": "" },
            { "name": "layers", "description": "List of layers to query from the WMS service", "type": "string", "default": "" },
            { "name": "style", "decription": "WMS style to request", "type": "string", "default": "" },
            { "name": "format", "description", "Image format to request", "type": "string", "default": "image/png" },
            { "name": "wms_format", "description", "Image format to request", "type": "string", "default": "image/png" },
            { "name": "wms_version", "description", "WMS service version", "type": "string", "default": "1.1.1" },
            { "name": "srs", "description", "SRS name to request", "type": "string", "default": "" },
            { "name": "crs", "description", "CRS name to request", "type": "string", "default": "" },
            { "name": "transparent", "description", "Whether to set the transparent flag in WMS requests", "type": "boolean", "default": "false" },
            { "name": "times", "description", "List of timestamps for WMS-T", "type": "string", "default": "" },
          ]
        }
    ) );
}

Config
WMS::WMSImageLayerOptions::getConfig() const
{
    Config conf = ImageLayer::Options::getConfig();
    conf.set("url", _url);
    conf.set("capabilities_url", _capabilitiesUrl);
    conf.set("layers", _layers);
    conf.set("style", _style);
    conf.set("format", _format);
    conf.set("wms_format", _wmsFormat);
    conf.set("wms_version", _wmsVersion);
    conf.set("srs", _srs);
    conf.set("crs", _crs);
    conf.set("transparent", _transparent);
    conf.set("times", _times);
    conf.set("seconds_per_frame", _secondsPerFrame);
    return conf;
}

void
WMS::WMSImageLayerOptions::fromConfig(const Config& conf)
{
    _wmsVersion.init("1.1.1");
    _transparent.init(true);
    _secondsPerFrame.init(1.0);

    conf.get("url", _url);
    conf.get("capabilities_url", _capabilitiesUrl);
    conf.get("layers", _layers);
    conf.get("style", _style);
    conf.get("format", _format);
    conf.get("wms_format", _wmsFormat);
    conf.get("wms_version", _wmsVersion);
    conf.get("srs", _srs);
    conf.get("crs", _crs);
    conf.get("transparent", _transparent);
    conf.get("times", _times);
    conf.get("time", _times); // alternative
    conf.get("seconds_per_frame", _secondsPerFrame);
}

//........................................................................

namespace osgEarth {  namespace WMS
{
    // All looping ImageSequences deriving from this class will be in sync due to
    // a shared reference time.
    struct SyncImageSequence : public osg::ImageSequence
    {
        SyncImageSequence() : osg::ImageSequence() { }
        virtual void update(osg::NodeVisitor* nv)
        {
            setReferenceTime( 0.0 );
            osg::ImageSequence::update( nv );
        }
    };
} } // namespace osgEarth::WMS

//........................................................................

//! Construct the WMS driver
WMS::Driver::Driver(const WMS::WMSImageLayerOptions& myOptions,
                    SequenceControl* sequence,
                    const osgDB::Options* readOptions)
{
    _sequence = sequence;
    _options = &myOptions;
    _readOptions = readOptions;
}

bool
WMS::Driver::isSequenced() const
{
    return _timesVec.size() > 1;
}

//! Connect to the WMS service, query capabilities, and prepare the driver
Status
WMS::Driver::open(osg::ref_ptr<const Profile>& profile,
                  DataExtentList& dataExtents)
{
    if (options().times().isSet())
    {
        StringTokenizer(options().times().get(), _timesVec, ",", "", false, true);
        OE_INFO << LC << "WMS-T: found " << _timesVec.size() << " times." << std::endl;

        for (unsigned i = 0; i < _timesVec.size(); ++i)
        {
            _seqFrameInfoVec.push_back(SequenceFrameInfo());
            _seqFrameInfoVec.back().timeIdentifier = _timesVec[i];
        }
    }

    // localize it since we might override them:
    _formatToUse = options().format().value();
    _srsToUse = options().wmsVersion().value() == "1.3.0" ? options().crs().value() : options().srs().value();
    if (_srsToUse.empty())
    {
        //If they didn't specify a CRS, see if they specified an SRS and try to use that
        _srsToUse = options().srs().value();
    }

    osg::ref_ptr<const Profile> result;

    char sep = options().url()->full().find_first_of('?') == std::string::npos ? '?' : '&';

    URI capUrl = options().capabilitiesUrl().value();
    if (capUrl.empty())
    {
        capUrl = URI(
            options().url()->full() +
            sep +
            std::string("SERVICE=WMS") +
            std::string("&VERSION=") + options().wmsVersion().value() +
            std::string("&REQUEST=GetCapabilities"));
    }

    //Try to read the WMS capabilities
    osg::ref_ptr<WMS::Capabilities> capabilities = WMS::CapabilitiesReader::read(capUrl, _readOptions.get());
    if (!capabilities.valid())
    {
        return Status::Error(Status::ResourceUnavailable, "Unable to read WMS GetCapabilities.");
    }
    else
    {
        OE_INFO << LC << "Got capabilities from " << capUrl.full() << std::endl;
    }

    if (_formatToUse.empty() && capabilities.valid())
    {
        _formatToUse = capabilities->suggestExtension();
        OE_INFO << LC << "No format specified, capabilities suggested extension " << _formatToUse << std::endl;
    }

    if (_formatToUse.empty())
        _formatToUse = "png";

    if (_srsToUse.empty())
        _srsToUse = "EPSG:4326";

    std::string wmsFormatToUse = options().wmsFormat().value();

    //Initialize the WMS request prototype
    std::stringstream buf;

    // first the mandatory keys:
    buf
        << std::fixed << options().url()->full() << sep
        << "SERVICE=WMS"
        << "&VERSION=" << options().wmsVersion().value()
        << "&REQUEST=GetMap"
        << "&LAYERS=" << options().layers().value()
        << "&FORMAT=" << (wmsFormatToUse.empty() ? std::string("image/") + _formatToUse : wmsFormatToUse)
        << "&STYLES=" << options().style().value()
        << (options().wmsVersion().value() == "1.3.0" ? "&CRS=" : "&SRS=") << _srsToUse
        << "&WIDTH=" << options().tileSize().get()
        << "&HEIGHT=" << options().tileSize().get()
        << "&BBOX=%lf,%lf,%lf,%lf";

    // then the optional keys:
    if (options().transparent().isSet())
        buf << "&TRANSPARENT=" << (options().transparent() == true ? "TRUE" : "FALSE");


    _prototype = "";
    _prototype = buf.str();

    //OE_NOTICE << "Prototype " << _prototype << std::endl;

    osg::ref_ptr<SpatialReference> wms_srs = SpatialReference::create(_srsToUse);

    // check for spherical mercator:
    if (wms_srs.valid() && wms_srs->isEquivalentTo(osgEarth::Registry::instance()->getSphericalMercatorProfile()->getSRS()))
    {
        result = osgEarth::Registry::instance()->getSphericalMercatorProfile();
    }
    else if (wms_srs.valid() && wms_srs->isEquivalentTo(osgEarth::Registry::instance()->getGlobalGeodeticProfile()->getSRS()))
    {
        result = osgEarth::Registry::instance()->getGlobalGeodeticProfile();
    }

    // Next, try to glean the extents from the layer list
    if (capabilities.valid())
    {
        StringTokenizer tok(",");
        StringVector tized;
        tok.tokenize(options().layers().value(), tized);

        for (StringVector::const_iterator itr = tized.begin(); itr != tized.end(); ++itr)
        {
            std::string layerName = *itr;
            WMS::Layer* layer = capabilities->getLayerByName(layerName);
            if (layer)
            {
                // Get the lat/lon extents
                double minLon, minLat, maxLon, maxLat;
                layer->getLatLonExtents(minLon, minLat, maxLon, maxLat);
                GeoExtent wgs84Extent(SpatialReference::create("wgs84"), minLon, minLat, maxLon, maxLat);
                dataExtents.push_back(DataExtent(wgs84Extent, 0));
            }
        }

        // If we don't have a profile yet, transform the lat/lon extents to
        // the requested srs and use it as the extents of the profile.
        if (!result.valid())
        {
            const SpatialReference* srs = SpatialReference::create(_srsToUse);
            GeoExtent totalExtent(srs);
            for (DataExtentList::const_iterator itr = dataExtents.begin(); itr != dataExtents.end(); ++itr)
            {
                GeoExtent dataExtent = *itr;
                GeoExtent nativeExtent;
                dataExtent.transform(srs, nativeExtent);
                totalExtent.expandToInclude(nativeExtent);
            }
            result = Profile::create(srs, totalExtent.xMin(), totalExtent.yMin(), totalExtent.xMax(), totalExtent.yMax());
        }
    }

    // Last resort: create a global extent profile (only valid for global maps)
    if (!result.valid() && wms_srs->isGeographic())
    {
        result = osgEarth::Registry::instance()->getGlobalGeodeticProfile();
    }


    // Use the override profile if one is passed in.
    if (profile.valid() == false)
    {
        profile = result.get();
    }

    if (profile.valid())
    {
        OE_INFO << LC << "Profile=" << profile->toString() << std::endl;

        return Status::OK();
    }
    else
    {
        return Status::Error("Unable to establish profile");
    }
}

//! fetch a tile image from the WMS service and report any exceptions.
osg::Image*
WMS::Driver::fetchTileImage(const TileKey&     key,
                            const std::string& extraAttrs,
                            ProgressCallback*  progress,
                            ReadResult&        out_response) const
{
    osg::ref_ptr<osg::Image> image;

    std::string uri = createURI(key);
    if (!extraAttrs.empty())
    {
        std::string delim = uri.find("?") == std::string::npos ? "?" : "&";
        uri = uri + delim + extraAttrs;
    }

    // Try to get the image first
    out_response = URI(uri, options().url()->context()).readImage(_readOptions.get(), progress);

    if (out_response.succeeded())
    {
        image = out_response.getImage();
    }

    return image.release();
}

//! Queries the WMS service for an image and returns an osg::Image
//! containing the result, or NULL if one could not be found
osg::Image*
WMS::Driver::createImage(const TileKey& key, ProgressCallback* progress) const
{
    osg::ref_ptr<osg::Image> image;

    if (_timesVec.size() > 1)
    {
        image = createImageSequence(key, progress);
    }
    else
    {
        std::string extras;
        if (_timesVec.size() == 1)
            extras = std::string("TIME=") + _timesVec[0];

        ReadResult response;
        image = fetchTileImage(key, extras, progress, response);
    }

    return image.release();
}


//! Creates an image from timestamped data
osg::Image*
WMS::Driver::createImageSequence(const TileKey& key, ProgressCallback* progress) const
{
    osg::ref_ptr< osg::ImageSequence > seq = new SyncImageSequence();

    seq->setLoopingMode(osg::ImageStream::LOOPING);
    seq->setLength(options().secondsPerFrame().value() * (double)_timesVec.size());
    if (_sequence->isSequencePlaying())
        seq->play();

    for (unsigned int r = 0; r < _timesVec.size(); ++r)
    {
        std::string extraAttrs = std::string("TIME=") + _timesVec[r];

        ReadResult response;
        osg::ref_ptr<osg::Image> image = fetchTileImage(key, extraAttrs, progress, response);
        if (image.get())
        {
            seq->addImage(image);
        }
    }

    // Just return an empty image if we didn't get any images
    unsigned size = seq->getNumImageData();

    if (size == 0)
    {
        return ImageUtils::createEmptyImage();
    }

    return seq.release();
}

//! Generates a URI for a tile key using the WMS request prototype
std::string
WMS::Driver::createURI(const TileKey& key) const
{
    double minx, miny, maxx, maxy;
    key.getExtent().getBounds(minx, miny, maxx, maxy);

    char buf[2048];
    sprintf(buf, _prototype.c_str(), minx, miny, maxx, maxy);

    std::string uri(buf);

    // url-ize the uri before returning it
    if (osgDB::containsServerAddress(uri))
        uri = replaceIn(uri, " ", "%20");

    return uri;
}

const WMS::WMSImageLayerOptions&
WMS::Driver::options() const
{
    return *_options;
}

const std::vector<SequenceFrameInfo>&
WMS::Driver::getSequenceFrameInfo() const
{
    return _seqFrameInfoVec;
}

int
WMS::Driver::getCurrentSequenceFrameIndex(const osg::FrameStamp* fs, double secondsPerFrame) const
{
    if (_seqFrameInfoVec.size() == 0)
        return 0;

    double len = secondsPerFrame * (double)_timesVec.size();
    double t = fmod(fs->getSimulationTime(), len) / len;
    return osg::clampBetween(
        (int)(t * (double)_seqFrameInfoVec.size()),
        (int)0,
        (int)_seqFrameInfoVec.size() - 1);
}


//........................................................................

REGISTER_OSGEARTH_LAYER(wmsimage, WMSImageLayer);

OE_LAYER_PROPERTY_IMPL(WMSImageLayer, URI, URL, url);
OE_LAYER_PROPERTY_IMPL(WMSImageLayer, URI, CapabilitiesURL, capabilitiesUrl);
OE_LAYER_PROPERTY_IMPL(WMSImageLayer, std::string, Layers, layers);
OE_LAYER_PROPERTY_IMPL(WMSImageLayer, std::string, Style, style);
OE_LAYER_PROPERTY_IMPL(WMSImageLayer, std::string, Format, format);
OE_LAYER_PROPERTY_IMPL(WMSImageLayer, std::string, SRS, srs);
OE_LAYER_PROPERTY_IMPL(WMSImageLayer, std::string, CRS, crs);
OE_LAYER_PROPERTY_IMPL(WMSImageLayer, bool, Transparent, transparent);
OE_LAYER_PROPERTY_IMPL(WMSImageLayer, std::string, Times, times);
OE_LAYER_PROPERTY_IMPL(WMSImageLayer, double, SecondsPerFrame, secondsPerFrame);


void
WMSImageLayer::init()
{
    ImageLayer::init();
    _isPlaying = false;
}

Status
WMSImageLayer::openImplementation()
{
    Status parent = ImageLayer::openImplementation();
    if (parent.isError())
        return parent;

    WMS::Driver* driver = new WMS::Driver(options(), this, getReadOptions());
    _driver = driver;

    osg::ref_ptr<const Profile> profile = getProfile();

    Status status = driver->open(
        profile,
        dataExtents());

    if (status.isError())
        return status;

    if (profile.get() != getProfile())
    {
        setProfile(profile.get());
    }

    return Status::NoError;
}

GeoImage
WMSImageLayer::createImageImplementation(const TileKey& key, ProgressCallback* progress) const
{
    osg::ref_ptr<osg::Image> image;
    if (_driver.valid())
    {
        WMS::Driver* driver = static_cast<WMS::Driver*>(_driver.get());
        image = driver->createImage(key, progress);
    }
    return GeoImage(image.get(), key.getExtent());
}

/** Whether the implementation supports these methods */
bool
WMSImageLayer::supportsSequenceControl() const
{
    WMS::Driver* driver = static_cast<WMS::Driver*>(_driver.get());
    return driver->isSequenced();
}

/** Starts playback */
void
WMSImageLayer::playSequence()
{
    _isPlaying = true;
}

/** Stops playback */
void
WMSImageLayer::pauseSequence()
{
    _isPlaying = false;
}

/** Seek to a specific frame */
void
WMSImageLayer::seekToSequenceFrame(unsigned frame)
{
    //todo
}

/** Whether the object is in playback mode */
bool
WMSImageLayer::isSequencePlaying() const
{
    return _isPlaying;
}

/** Gets data about the current frame in the sequence */
const std::vector<SequenceFrameInfo>&
WMSImageLayer::getSequenceFrameInfo() const
{
    WMS::Driver* driver = static_cast<WMS::Driver*>(_driver.get());
    return driver->getSequenceFrameInfo();
}

/** Index of current frame */
int
WMSImageLayer::getCurrentSequenceFrameIndex(const osg::FrameStamp* fs) const
{
    WMS::Driver* driver = static_cast<WMS::Driver*>(_driver.get());
    return driver->getCurrentSequenceFrameIndex(fs, options().secondsPerFrame().get());
}
