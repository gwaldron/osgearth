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

#include <osgEarthUtil/WFS>
#include <osgEarth/XmlUtils>

#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace std;


WFSCapabilities::WFSCapabilities()
{
}

WFSFeatureType*
WFSCapabilities::getFeatureTypeByName(const std::string& name)
{
    for (FeatureTypeList::iterator itr = _featureTypes.begin(); itr != _featureTypes.end(); ++itr)
    {
        if (osgDB::equalCaseInsensitive(itr->get()->getName(),name)) return itr->get();
    }
    return NULL;
}

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

/**************************************************************************************/
WFSFeatureType::WFSFeatureType()
{    
}
/**************************************************************************************/

WFSCapabilities* 
WFSCapabilitiesReader::read( const URI& location, const osgDB::Options* dbOptions )
{
    // read the data into a string buffer and parse it from there
    std::string buffer = location.readString(dbOptions).getString();
    if ( !buffer.empty() )
    {
        std::stringstream buf(buffer);
        return read(buf);
    }
    else return 0L;
}

WFSCapabilities*
WFSCapabilitiesReader::read(std::istream &in)
{
    osg::ref_ptr<WFSCapabilities> capabilities = new WFSCapabilities;

    osg::ref_ptr<XmlDocument> doc = XmlDocument::load( in );
    if (!doc.valid() || doc->getChildren().empty())
    {
        OE_NOTICE << "Failed to load Capabilities " << std::endl;
        return 0;
    }

    //Get the Capabilities version
    osg::ref_ptr<XmlElement> e_root = static_cast<XmlElement*>(doc->getChildren()[0].get());
    capabilities->setVersion( e_root->getAttr(ATTR_VERSION ) );

    osg::ref_ptr<XmlElement> e_service = e_root->getSubElement( ELEM_SERVICE );
    if (!e_service.valid())
    {
        OE_NOTICE << "Could not find Service element" << std::endl;
        return 0;
    }
    

    //Read the parameters from the Service block
    capabilities->setName( e_service->getSubElementText(ELEM_NAME ) );
    capabilities->setAbstract( e_service->getSubElementText( ELEM_ABSTRACT ) );
    capabilities->setTitle( e_service->getSubElementText( ELEM_TITLE ) );    

    //Read all the feature types    
    osg::ref_ptr<XmlElement> e_feature_types = e_root->getSubElement( ELEM_FEATURETYPELIST );
    if (e_feature_types.valid())
    {
        XmlNodeList featureTypes = e_feature_types->getSubElements( ELEM_FEATURETYPE );
        for( XmlNodeList::const_iterator itr = featureTypes.begin(); itr != featureTypes.end(); itr++ )
        {
            XmlElement* e_featureType = static_cast<XmlElement*>( itr->get() );
            WFSFeatureType* featureType = new WFSFeatureType();
            featureType->setName( e_featureType->getSubElementText( ELEM_NAME ) );
            featureType->setTitle( e_featureType->getSubElementText( ELEM_TITLE ) );
            featureType->setAbstract( e_featureType->getSubElementText( ELEM_ABSTRACT ) );

            //NOTE:  TILED and MAXLEVEL aren't part of the WFS spec, these are enhancements to our server for tiled WFS access
            std::string tiledStr = e_featureType->getSubElementText(ELEM_TILED);
            if (tiledStr.compare("") != 0)
            {
                featureType->setTiled( as<bool>(tiledStr, false) );
            }

            std::string maxLevelStr = e_featureType->getSubElementText(ELEM_MAXLEVEL);
            if (maxLevelStr.compare("") != 0)
            {
                featureType->setMaxLevel( as<int>(maxLevelStr, -1));
            }

            std::string firstLevelStr = e_featureType->getSubElementText(ELEM_FIRSTLEVEL);
            if (firstLevelStr.compare("") != 0)
            {
                featureType->setFirstLevel( as<int>(firstLevelStr, 0));
            }

            // Read the SRS            
            std::string srsText = e_featureType->getSubElementText(ELEM_SRS);
            if (srsText.compare("") != 0)
            {                
                featureType->setSRS( srsText );                
            }

            osg::ref_ptr<XmlElement> e_bb = e_featureType->getSubElement( ELEM_LATLONGBOUNDINGBOX );
            if (e_bb.valid())
            {
                double minX, minY, maxX, maxY;
                minX = as<double>(e_bb->getAttr( ATTR_MINX ), 0);
                minY = as<double>(e_bb->getAttr( ATTR_MINY ), 0);
                maxX = as<double>(e_bb->getAttr( ATTR_MAXX ), 0);
                maxY = as<double>(e_bb->getAttr( ATTR_MAXY ), 0);                
                featureType->setExtent( GeoExtent( osgEarth::SpatialReference::create( srsText ), minX, minY, maxX, maxY) );
            }                       

            capabilities->getFeatureTypes().push_back( featureType );
        }        
    }


    return capabilities.release();
}
