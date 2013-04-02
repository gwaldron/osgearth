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
#include "KML_Model"

#include <osgEarthSymbology/ModelSymbol>

using namespace osgEarth_kml;
using namespace osgEarth::Symbology;

void
KML_Model::parseCoords( const Config& conf, KMLContext& cx )
{
    PointSet* point = new PointSet();

    Config location = conf.child("location");
    if (!location.empty())
    {
        double latitude  = location.value("latitude",  0.0);
        double longitude = location.value("longitude", 0.0);
        double altitude  = location.value("altitude", 0.0); 
        point->push_back( osg::Vec3d(longitude, latitude, altitude));
    }    
    _geom = point;
}

void
KML_Model::parseStyle(const Config& conf, KMLContext& cx, Style& style)
{    
    ModelSymbol* model = 0L;
    
    std::string url = KMLUtils::parseLink(conf);
    if ( !url.empty() )
    {
        if ( !model ) model = style.getOrCreate<ModelSymbol>();
        model->url()->setLiteral( url );
        model->url()->setURIContext( URIContext(conf.referrer()) );
    }

    Config scale = conf.child("scale");
    if (!scale.empty())
    {
        if ( !model ) model = style.getOrCreate<ModelSymbol>();
        //TODO:  Support XYZ scale instead of single value
        model->scale() = scale.value("x", 1.0);
    }

    Config orientation = conf.child("orientation");
    if (!orientation.empty())
    {
        if ( !model ) model = style.getOrCreate<ModelSymbol>();
        
        double h = orientation.value("heading", 0);
        if ( !osg::equivalent(h, 0.0) )
            model->heading() = NumericExpression( h );

        double p = orientation.value("tilt", 0);
        if ( !osg::equivalent(p, 0.0) )
            model->pitch() = NumericExpression( p );

        double r = orientation.value("roll", 0);
        if ( !osg::equivalent(r, 0.0) )
            model->roll() = NumericExpression( r );
    }

    // Read and store file path aliases from a KML ResourceMap.
    Config resource_map = conf.child("resourcemap");
    if ( !resource_map.empty() )
    {
        const ConfigSet aliases = resource_map.children("alias");
        for( ConfigSet::const_iterator i = aliases.begin(); i != aliases.end(); ++i )
        {
            std::string source = i->value("sourcehref");
            std::string target = i->value("targethref");
            if ( !source.empty() || !target.empty() )
            {
                if ( !model ) model = style.getOrCreate<ModelSymbol>();
                model->uriAliasMap()->insert( source, target );
            }
        }
    }

    KML_Geometry::parseStyle(conf, cx, style);
}
