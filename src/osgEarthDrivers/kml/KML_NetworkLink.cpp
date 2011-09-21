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
#include "KML_NetworkLink"
#include <osg/PagedLOD>

void
KML_NetworkLink::build( const Config& conf, KMLContext& cx )
{
    std::string name = conf.value("name");

    // parse the bounds:
    const Config& regionConf = conf.child("region");
    if ( regionConf.empty() )
        return;
    const Config& llaBoxConf = regionConf.child("latlonaltbox");
    if ( llaBoxConf.empty() )
        return;
    GeoExtent llaExtent(
        cx._mapNode->getMap()->getProfile()->getSRS()->getGeographicSRS(),
        conf.value<double>(regionConf.value("west"),  0.0),
        conf.value<double>(regionConf.value("south"), 0.0),
        conf.value<double>(regionConf.value("east"),  0.0),
        conf.value<double>(regionConf.value("north"), 0.0) );
    GeoExtent mapExtent = llaExtent.transform( cx._mapNode->getMap()->getProfile()->getSRS() );
    double x, y;
    mapExtent.getCentroid( x, y );
    osg::Vec3d lodCenter;
    cx._mapNode->getMap()->mapPointToWorldPoint( osg::Vec3d(x,y,0), lodCenter );

    // parse the LOD ranges:
    float minRange = 0, maxRange = 1e6;
    const Config& lodConf = conf.child("lod");
    if ( !lodConf.empty() ) 
    {
        // swapped
        maxRange = conf.value<float>( "minlodpixels", maxRange );
        minRange = conf.value<float>( "maxlodpixels", minRange );
    }

    // parse the link:
    const Config& linkConf = conf.child("link");
    if ( linkConf.empty() )
        return;
    std::string href = linkConf.value("href");
    if ( href.empty() )
        return;

    // build the node
    osg::PagedLOD* plod = new osg::PagedLOD();
    plod->setRangeMode( osg::LOD::PIXEL_SIZE_ON_SCREEN );
    if ( !endsWith(href, ".kml") )
        href += "&.kml";
    plod->setFileName( 0, href );
    plod->setRange( 0, minRange, maxRange );
    plod->setCenter( lodCenter );
    osgDB::Options* options = new osgDB::Options();
    options->setPluginData( "osgEarth::MapNode", cx._mapNode );
    plod->setDatabaseOptions( options );
    cx._groupStack.top()->addChild( plod );
}
