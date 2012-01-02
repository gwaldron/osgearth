/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2010 Pelican Mapping
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

#include <osgEarthSymbology/MarkerSymbol>

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
        double altitude  = location.value("alttiude", 0.0); 
        point->push_back( osg::Vec3d(longitude, latitude, altitude));
    }    
    _geom = point;
}

void
KML_Model::parseStyle(const Config& conf, KMLContext& cx, Style& style)
{    
    MarkerSymbol* marker = 0L;
    
    Config link = conf.child("link");
    if (!link.empty())
    {
        if ( !marker ) marker = style.getOrCreate<MarkerSymbol>();
        marker->url() = StringExpression( link.value("href") );
        marker->url()->setURIContext( URIContext(conf.referrer()) );        
    }

    Config scale = conf.child("scale");
    if (!scale.empty())
    {
        if ( !marker ) marker = style.getOrCreate<MarkerSymbol>();
        //TODO:  Support XYZ scale instead of single value
        marker->scale() = scale.value("x", 1.0);
    }

    Config orientation = conf.child("orientation");
    if (!orientation.empty())
    {
        if ( !marker ) marker = style.getOrCreate<MarkerSymbol>();
        double h = orientation.value("heading", 0);
        double p = orientation.value("tilt", 0);
        double r = orientation.value("roll", 0);        
        marker->orientation() = osg::Vec3d(h,p,r);
    }

    // since we know this is a model, not an icon.
    if ( marker )
        marker->isModel() = true;

    KML_Geometry::parseStyle(conf, cx, style);
}
