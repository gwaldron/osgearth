/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
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
KML_NetworkLink::build( xml_node<>* node, KMLContext& cx )
{
	if (!node) return;

    std::string name = getValue(node, "name");

    // parse the link:
    std::string href = KMLUtils::parseLink(node);
    
    if ( !href.empty())
    {
        href = URIContext(cx._referrer).getOSGPath(href);
    }

    // "open" determines whether to load it immediately
    bool open = as<bool>(getValue(node, "open"), false);

    // if it's region-bound, parse it as a paged LOD:
    xml_node<>* region = node->first_node("region", 0, false);
    if ( region )
    {
        xml_node<>* llaBox = region->first_node("latlonaltbox", 0, false);
        if ( !llaBox )
            return;

        const SpatialReference* geoSRS = cx._srs;

        GeoExtent llaExtent(
            geoSRS,
            as<double>(getValue(llaBox, "west"), 0.0),
			as<double>(getValue(llaBox, "south"), 0.0),
			as<double>(getValue(llaBox, "east"), 0.0),
			as<double>(getValue(llaBox, "north"), 0.0));

        // find the ECEF LOD center point:
        double x, y;
        llaExtent.getCentroid( x, y );
        osg::Vec3d lodCenter;
        llaExtent.getSRS()->transform( osg::Vec3d(x,y,0), geoSRS->getGeocentricSRS(), lodCenter );
        //llaExtent.getSRS()->transformToECEF( osg::Vec3d(x,y,0), lodCenter );

        // figure the tile radius:
        double d = 0.5 * GeoMath::distance(
            osg::DegreesToRadians(llaExtent.yMin()), osg::DegreesToRadians(llaExtent.xMin()),
            osg::DegreesToRadians(llaExtent.yMax()), osg::DegreesToRadians(llaExtent.xMax()) );

        // parse the LOD ranges:
        float minRange = 0, maxRange = 1e6;
        xml_node<>* lod = region->first_node("lod", 0, false);
        if ( lod ) 
        {
            // swapped
            minRange = as<float>(getValue(lod, "minlodpixels"), 0.0f);
            if ( minRange < 0.0f )
                minRange = 0.0f;
			maxRange = as<float>(getValue(lod, "maxlodpixels"), FLT_MAX);
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

        cx._groupStack.top()->addChild( plod );
    }

    else 
    {
        osg::ProxyNode* proxy = new osg::ProxyNode();
        proxy->setFileName( 0, href );                

        //osgDB::Options* options = Registry::instance()->cloneOrCreateOptions();
        //options->setPluginData( "osgEarth::MapNode", cx._mapNode );
        //proxy->setDatabaseOptions( options );

        cx._groupStack.top()->addChild( proxy );
    }

}
