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

#include <osgEarthUtil/TFS>
#include <osgEarth/XmlUtils>

#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace std;


TFSLayer::TFSLayer():
_firstLevel(0),
_maxLevel(8),
_title("layer"),
_srs( SpatialReference::create("EPSG:4326") )
{
}

/************************************************/

bool
TFSReaderWriter::read(const URI& uri, const osgDB::ReaderWriter::Options *options, TFSLayer &layer)
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
TFSReaderWriter::read( std::istream &in, TFSLayer &layer)
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


static XmlDocument*
tfsToXmlDocument(const TFSLayer &layer)
{
    //Create the root XML document
    osg::ref_ptr<XmlDocument> doc = new XmlDocument();
    doc->setName( "Layer" );
        
    doc->addSubElement( "Title", layer.getTitle() );
    doc->addSubElement( "Abstract", layer.getAbstract() );
    doc->addSubElement( "MaxLevel", toString<unsigned int>(layer.getMaxLevel() ));
    doc->addSubElement( "FirstLevel", toString<unsigned int>(layer.getFirstLevel() ));
    
    osg::ref_ptr<XmlElement> e_bounding_box = new XmlElement( "BoundingBox" );
    e_bounding_box->getAttrs()["minx"] = toString(layer.getExtent().xMin());
    e_bounding_box->getAttrs()["miny"] = toString(layer.getExtent().yMin());
    e_bounding_box->getAttrs()["maxx"] = toString(layer.getExtent().xMax());
    e_bounding_box->getAttrs()["maxy"] = toString(layer.getExtent().yMax());
    doc->getChildren().push_back(e_bounding_box.get() );

    doc->addSubElement( "SRS", layer.getSRS()->getHorizInitString() );
    
    return doc.release();
}

void
TFSReaderWriter::write(const TFSLayer &layer, const std::string &location)
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
TFSReaderWriter::write(const TFSLayer &layer, std::ostream &output)
{
    osg::ref_ptr<XmlDocument> doc = tfsToXmlDocument(layer);    
    doc->store(output);
}
