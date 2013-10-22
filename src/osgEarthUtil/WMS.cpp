/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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

#include <osgEarthUtil/WMS>
#include <osgEarth/XmlUtils>

#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace std;

namespace
{
    WMSLayer* getLayerByName(const string &name, WMSLayer::LayerList& layers)
    {
        for (WMSLayer::LayerList::iterator i = layers.begin(); i != layers.end(); ++i)
        {
            if (osgDB::equalCaseInsensitive(i->get()->getName(),name)) return i->get();
            WMSLayer *l = getLayerByName(name, i->get()->getLayers());
            if (l) return l;
        }
        return 0;
    }
}

WMSStyle::WMSStyle()
{
}

WMSStyle::WMSStyle(const std::string& name, const std::string &title)
{
    _name = name;
    _title = title;
}

WMSLayer::WMSLayer():
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
}

void WMSLayer::getLatLonExtents(double &minLon, double &minLat, double &maxLon, double &maxLat)
{
    minLon = _minLon;
    minLat= _minLat;
    maxLon = _maxLon;
    maxLat = _maxLat;
}

void WMSLayer::setLatLonExtents(double minLon, double minLat, double maxLon, double maxLat)
{
    _minLon = minLon;
    _minLat = minLat;
    _maxLon = maxLon;
    _maxLat = maxLat;
}

void WMSLayer::getExtents(double &minX, double &minY, double &maxX, double &maxY)
{
    minX = _minX;
    minY = _minY;
    maxX = _maxX;
    maxY = _maxY;
}

void WMSLayer::setExtents(double minX, double minY, double maxX, double maxY)
{
    _minX = minX;
    _minY = minY;
    _maxX = maxX;
    _maxY = maxY;
}


WMSCapabilities::WMSCapabilities()
{
}

std::string WMSCapabilities::suggestExtension()
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

WMSLayer*
WMSCapabilities::getLayerByName(const std::string &name)
{
    return ::getLayerByName(name, _layers);
}

WMSCapabilities* 
WMSCapabilitiesReader::read( const std::string &location, const osgDB::ReaderWriter::Options* options )
{
    WMSCapabilities *caps = NULL;
    if ( osgDB::containsServerAddress( location ) )
    {
        ReadResult rr = URI(location).readString( options );
        if ( rr.succeeded() )
        {
            std::istringstream in( rr.getString() );
            caps = read( in );
        }
    }
    else
    {
        if ((osgDB::fileExists(location)) && (osgDB::fileType(location) == osgDB::REGULAR_FILE))
        {
            std::ifstream in( location.c_str() );
            caps = read( in );
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
#define ATTR_MINX              "minx"
#define ATTR_MINY              "miny"
#define ATTR_MAXX              "maxx"
#define ATTR_MAXY              "maxy"

#define ATTR_EASTLON           "eastboundlongitude"
#define ATTR_WESTLON           "westboundlongitude"
#define ATTR_NORTHLAT          "northboundlatitude"
#define ATTR_SOUTHLAT          "southboundlatitude"

static void
readLayers(XmlElement* e, WMSLayer* parentLayer, WMSLayer::LayerList& layers)
{
    XmlNodeList layerNodes = e->getSubElements( ELEM_LAYER );
    for( XmlNodeList::const_iterator i = layerNodes.begin(); i != layerNodes.end(); i++ )
    {
        XmlElement* e_layer = static_cast<XmlElement*>( i->get() );

        WMSLayer *layer = new WMSLayer;
        layer->setName( e_layer->getSubElementText( ELEM_NAME ) );
        layer->setTitle( e_layer->getSubElementText( ELEM_TITLE ) );
        layer->setAbstract( e_layer->getSubElementText( ELEM_ABSTRACT ) );        

        //Read all the supported styles
        XmlNodeList styles = e_layer->getSubElements( ELEM_STYLE );
        for( XmlNodeList::const_iterator styleitr = styles.begin(); styleitr != styles.end(); styleitr++ )
        {
            XmlElement* e_style = static_cast<XmlElement*>( styleitr->get() );
            string name = e_style->getSubElementText( ELEM_NAME );
            string title = e_style->getSubElementText( ELEM_TITLE );
            layer->getStyles().push_back(WMSStyle(name,title));
        }

        //Read all the supported SRS's
        XmlNodeList spatialReferences = e_layer->getSubElements( ELEM_SRS );
        for (XmlNodeList::const_iterator srsitr = spatialReferences.begin(); srsitr != spatialReferences.end(); ++srsitr)
        {
            string srs = static_cast<XmlElement*>( srsitr->get() )->getText();
            layer->getSpatialReferences().push_back(srs);
        }

        //Read all the supported CRS's
        spatialReferences = e_layer->getSubElements( ELEM_CRS );
        for (XmlNodeList::const_iterator srsitr = spatialReferences.begin(); srsitr != spatialReferences.end(); ++srsitr)
        {
            string crs = static_cast<XmlElement*>( srsitr->get() )->getText();
            layer->getSpatialReferences().push_back(crs);
        }

        osg::ref_ptr<XmlElement> e_bb = e_layer->getSubElement( ELEM_LATLONBOUNDINGBOX );
        if (e_bb.valid())
        {
            double minX, minY, maxX, maxY;
            minX = as<double>(e_bb->getAttr( ATTR_MINX ), 0);
            minY = as<double>(e_bb->getAttr( ATTR_MINY ), 0);
            maxX = as<double>(e_bb->getAttr( ATTR_MAXX ), 0);
            maxY = as<double>(e_bb->getAttr( ATTR_MAXY ), 0);
            layer->setLatLonExtents(minX, minY, maxX, maxY);
        }
        else {
            osg::ref_ptr<XmlElement> e_gbb = e_layer->getSubElement( ELEM_GEOGRAPHICBOUNDINGBOX );
            if (e_gbb.valid())
            {
                double minX, minY, maxX, maxY;
                minX = as<double>(e_gbb->getSubElementText( ATTR_WESTLON ), 0);
                minY = as<double>(e_gbb->getSubElementText( ATTR_SOUTHLAT ), 0);
                maxX = as<double>(e_gbb->getSubElementText( ATTR_EASTLON ), 0);
                maxY = as<double>(e_gbb->getSubElementText( ATTR_NORTHLAT ), 0);
                layer->setLatLonExtents(minX, minY, maxX, maxY);
            }
        }

        e_bb = e_layer->getSubElement( ELEM_BOUNDINGBOX );
        if (e_bb.valid())
        {
            double minX, minY, maxX, maxY;
            minX = as<double>(e_bb->getAttr( ATTR_MINX ), 0);
            minY = as<double>(e_bb->getAttr( ATTR_MINY ), 0);
            maxX = as<double>(e_bb->getAttr( ATTR_MAXX ), 0);
            maxY = as<double>(e_bb->getAttr( ATTR_MAXY ), 0);
            layer->setExtents(minX, minY, maxX, maxY);
        }

        //Add the layer to the list and set its parent layer
        layers.push_back(layer);
        layer->setParentLayer( parentLayer );

        //Read any other layers that are in the layer node
        readLayers( e_layer, layer, layer->getLayers());
    }
}




WMSCapabilities*
WMSCapabilitiesReader::read(std::istream &in)
{
    osg::ref_ptr<WMSCapabilities> capabilities = new WMSCapabilities;

    osg::ref_ptr<XmlDocument> doc = XmlDocument::load( in );
    if (!doc.valid() || doc->getChildren().empty())
    {
        OE_NOTICE << "Failed to load Capabilities " << std::endl;
        return 0;
    }

    //Get the Capabilities version
    osg::ref_ptr<XmlElement> e_root = static_cast<XmlElement*>(doc->getChildren()[0].get());
    capabilities->setVersion( e_root->getAttr(ATTR_VERSION ) );

    osg::ref_ptr<XmlElement> e_capability = e_root->getSubElement( ELEM_CAPABILITY );
    if (!e_capability.valid())
    {
        OE_NOTICE << "Could not find Capability element" << std::endl;
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
                string format = trim(static_cast<XmlElement*>( i->get() )->getText());
                capabilities->getFormats().push_back(format);
            }
        }
    }

    //Try to read the layers
    readLayers( e_capability.get(), 0, capabilities->getLayers());

    return capabilities.release();
}
