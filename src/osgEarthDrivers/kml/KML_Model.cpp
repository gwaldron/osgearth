/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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
#include "KML_Model"

#include <osgEarthSymbology/ModelSymbol>

using namespace osgEarth_kml;
using namespace osgEarth::Symbology;

void
KML_Model::parseCoords( xml_node<>* node, KMLContext& cx )
{
    PointSet* point = new PointSet();

    xml_node<>* location = node->first_node("location", 0, false);
    if (location)
    {
        double latitude  = as<double>(getValue(location, "latitude"), 0.0);
        double longitude = as<double>(getValue(location, "longitude"), 0.0);
        double altitude  = as<double>(getValue(location, "altitude"), 0.0);
        point->push_back( osg::Vec3d(longitude, latitude, altitude));
    }    
    _geom = point;
}

void
KML_Model::parseStyle(xml_node<>* node, KMLContext& cx, Style& style)
{    
    ModelSymbol* model = 0L;
    
    std::string url = KMLUtils::parseLink(node);
    if ( !url.empty() )
    {
        if ( !model ) model = style.getOrCreate<ModelSymbol>();
        model->url()->setLiteral( url );
        model->url()->setURIContext( URIContext(cx._referrer) );

    }

    xml_node<>* scale = node->first_node("scale", 0, false);
    if (scale)
    {
        if ( !model ) model = style.getOrCreate<ModelSymbol>();
        //TODO:  Support XYZ scale instead of single value
        model->scale() = as<double>(getValue(scale, "x"), 1.0);
    }

    xml_node<>* orientation = node->first_node("orientation", 0, false);
    if (orientation)
    {
        if ( !model ) model = style.getOrCreate<ModelSymbol>();
        
        double h = as<double>(getValue(orientation, "heading"), 0.0);
        if ( !osg::equivalent(h, 0.0) )
            model->heading() = NumericExpression( h );

        double p = as<double>(getValue(orientation, "tilt"), 0.0);
        if ( !osg::equivalent(p, 0.0) )
            model->pitch() = NumericExpression( p );

        double r = as<double>(getValue(orientation, "roll"), 0.0);
        if ( !osg::equivalent(r, 0.0) )
            model->roll() = NumericExpression( r );
    }

    // Read and store file path aliases from a KML ResourceMap.
    xml_node<>* resource_map = node->first_node("resourcemap", 0, false);
    if ( resource_map )
    {
		for (xml_node<>* n = resource_map->first_node("alias", 0, false); n; n = n->next_sibling("alias", 0, false))
		{
			std::string source = getValue(n, "sourcehref");
            std::string target = getValue(n, "targethref");
            if ( !source.empty() || !target.empty() )
            {
                if ( !model ) model = style.getOrCreate<ModelSymbol>();
                model->uriAliasMap()->insert( source, target );
            }
		}
    }

    KML_Geometry::parseStyle(node, cx, style);
}
