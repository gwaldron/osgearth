/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2014 Pelican Mapping
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
#include <osgEarth/GeoMath>
#include <osgEarth/Registry>
#include <osg/PagedLOD>
#include <osg/ProxyNode>
#include <osg/Version>

#undef  LC
#define LC "[KML_NetworkLink] "

using namespace osgEarth_kml;

void
KML_NetworkLink::build( const Config& conf, KMLContext& cx )
{
    std::string name = conf.value("name");

    // parse the link:
    std::string href = KMLUtils::parseLink(conf);

    // "open" determines whether to load it immediately
    bool open = conf.value<bool>("open", false);

    // if it's region-bound, parse it as a paged LOD:
    const Config& regionConf = conf.child("region");
    if ( !regionConf.empty() )
    {
        const Config& llaBoxConf = regionConf.child("latlonaltbox");
        if ( llaBoxConf.empty() )
            return;

        const SpatialReference* geoSRS = cx._mapNode->getMapSRS()->getGeographicSRS();

        GeoExtent llaExtent(
            geoSRS,
            llaBoxConf.value<double>("west",  0.0),
            llaBoxConf.value<double>("south", 0.0),
            llaBoxConf.value<double>("east",  0.0),
            llaBoxConf.value<double>("north", 0.0) );

        // find the ECEF LOD center point:
        double x, y;
        llaExtent.getCentroid( x, y );
        osg::Vec3d lodCenter;
        llaExtent.getSRS()->transform( osg::Vec3d(x,y,0), geoSRS->getECEF(), lodCenter );
        //llaExtent.getSRS()->transformToECEF( osg::Vec3d(x,y,0), lodCenter );

        // figure the tile radius:
        double d = 0.5 * GeoMath::distance(
            osg::DegreesToRadians(llaExtent.yMin()), osg::DegreesToRadians(llaExtent.xMin()),
            osg::DegreesToRadians(llaExtent.yMax()), osg::DegreesToRadians(llaExtent.xMax()) );

        // parse the LOD ranges:
        float minRange = 0, maxRange = 1e6;
        const Config& lodConf = regionConf.child("lod");
        if ( !lodConf.empty() ) 
        {
            // swapped
            minRange = lodConf.value<float>( "minlodpixels", 0.0f );
            if ( minRange < 0.0f )
                minRange = 0.0f;
            maxRange = lodConf.value<float>( "maxlodpixels", FLT_MAX );
            if ( maxRange < 0.0f )
                maxRange = FLT_MAX;
        }

        // build the node
        osg::PagedLOD* plod = new osg::PagedLOD();
        plod->setRangeMode( osg::LOD::PIXEL_SIZE_ON_SCREEN );

        plod->setFileName( 0, href );
        plod->setRange( 0, minRange, maxRange );
        plod->setCenter( lodCenter );
        plod->setRadius( d );

        osgDB::Options* options = Registry::instance()->cloneOrCreateOptions();
        options->setPluginData( "osgEarth::MapNode", cx._mapNode );
        plod->setDatabaseOptions( options );

        OE_DEBUG << LC << 
            "PLOD: radius = " << d << ", minRange=" << minRange << ", maxRange=" << maxRange << std::endl;

        cx._groupStack.top()->addChild( plod );
    }

    else 
    {
        osg::ProxyNode* proxy = new osg::ProxyNode();
        proxy->setFileName( 0, href );                

        osgDB::Options* options = Registry::instance()->cloneOrCreateOptions();
        options->setPluginData( "osgEarth::MapNode", cx._mapNode );
        proxy->setDatabaseOptions( options );

        cx._groupStack.top()->addChild( proxy );
    }

}
