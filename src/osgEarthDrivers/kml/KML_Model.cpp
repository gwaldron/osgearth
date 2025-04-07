/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include "KML_Model"

#include <osgEarth/ModelSymbol>

using namespace osgEarth_kml;

void
KML_Model::parseCoords( xml_node<>* node, KMLContext& cx )
{
    Point* point = new Point();

    xml_node<>* location = node->first_node("location", 0, false);
    if (location)
    {
        double latitude  = as<double>(getValue(location, "latitude"), 0.0);
        double longitude = as<double>(getValue(location, "longitude"), 0.0);
        double altitude  = as<double>(getValue(location, "altitude"), 0.0);
        point->set( osg::Vec3d(longitude, latitude, altitude));
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
        model->url().mutable_value().setLiteral( url );
        model->url().mutable_value().setURIContext( URIContext(cx._referrer) );

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
                model->uriAliasMap().mutable_value().insert( source, target );
            }
		}
    }

    KML_Geometry::parseStyle(node, cx, style);
}
