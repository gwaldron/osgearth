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
#include <osgEarthUtil/UTMGraticule>
#include <osgEarthUtil/MGRSFormatter>

#include <osgEarthFeatures/GeometryCompiler>
#include <osgEarthFeatures/TextSymbolizer>

#include <osgEarthSymbology/Geometry>
#include <osgEarthAnnotation/LabelNode>

#include <osgEarth/Registry>
#include <osgEarth/DepthOffset>
#include <osgEarth/ECEF>
#include <osgEarth/NodeUtils>
#include <osgEarth/Utils>
#include <osgEarth/CullingUtils>
#include <osgEarth/DrapeableNode>
#include <osgEarth/ThreadingUtils>

#include <OpenThreads/Mutex>
#include <OpenThreads/ScopedLock>
#include <osg/PagedLOD>
#include <osg/Depth>
#include <osg/Program>
#include <osgDB/FileNameUtils>

#define LC "[UTMGraticule] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;
using namespace osgEarth::Annotation;

Threading::Mutex UTMGraticule::s_graticuleMutex;
UTMGraticule::UTMGraticuleRegistry UTMGraticule::s_graticuleRegistry;

//---------------------------------------------------------------------------

UTMGraticuleOptions::UTMGraticuleOptions( const Config& conf ) :
ConfigOptions( conf ),
_textScale   ( 1.0f )
{
    mergeConfig( _conf );
}

void
UTMGraticuleOptions::mergeConfig( const Config& conf )
{
    //todo
}

Config
UTMGraticuleOptions::getConfig() const
{
    Config conf = ConfigOptions::newConfig();
    conf.key() = "utm_graticule";
    //todo
    return conf;
}

//---------------------------------------------------------------------------


UTMGraticule::UTMGraticule( MapNode* mapNode ) :
_mapNode   ( mapNode ),
_root      ( 0L )
{
    init();
}

UTMGraticule::UTMGraticule( MapNode* mapNode, const UTMGraticuleOptions& options ) :
_mapNode   ( mapNode ),
_root      ( 0L )
{
    _options = options;
    init();
}

void
UTMGraticule::init()
{
    // safely generate a unique ID for this graticule:
    _id = Registry::instance()->createUID();
    {
        Threading::ScopedMutexLock lock( s_graticuleMutex );
        s_graticuleRegistry[_id] = this;
    }

    // make the shared depth attr:
    _depthAttribute = new osg::Depth(osg::Depth::LEQUAL,0,1,false);

    // this will intialize the graph.
    rebuild();
}

void
UTMGraticule::setMapNode( MapNode* mapNode )
{
    _mapNode = mapNode;
    rebuild();
}

void
UTMGraticule::setOptions( const UTMGraticuleOptions& options )
{
    _options = options;
    rebuild();
}

void
UTMGraticule::rebuild()
{
    // clear everything out
    this->removeChildren( 0, this->getNumChildren() );

    // requires a map node
    if ( !getMapNode() )
    {
        return;
    }

    // requires a geocentric map
    if ( !getMapNode()->isGeocentric() )
    {
        OE_WARN << LC << "Projected map mode is not yet supported" << std::endl;
        return;
    }

    const Profile* mapProfile = getMapNode()->getMap()->getProfile();

    _profile = Profile::create(
        mapProfile->getSRS(),
        mapProfile->getExtent().xMin(),
        mapProfile->getExtent().yMin(),
        mapProfile->getExtent().xMax(),
        mapProfile->getExtent().yMax(),
        8, 4 );

    _featureProfile = new FeatureProfile(_profile->getSRS());

    //todo: do this right..
    osg::StateSet* set = this->getOrCreateStateSet();
    set->setMode( GL_LIGHTING, 0 );
    set->setMode( GL_BLEND, 1 );

    // set up default options if the caller did not supply them
    if ( !_options.isSet() )
    {
        _options->primaryStyle() = Style();

        LineSymbol* line = _options->primaryStyle()->getOrCreate<LineSymbol>();
        line->stroke()->color() = Color::Gray;
        line->stroke()->width() = 1.0;
        line->tessellation() = 20;

        AltitudeSymbol* alt = _options->primaryStyle()->getOrCreate<AltitudeSymbol>();

        TextSymbol* text = _options->primaryStyle()->getOrCreate<TextSymbol>();
        text->fill()->color() = Color(Color::White, 0.3f);
        text->halo()->color() = Color(Color::Black, 0.2f);
        text->alignment() = TextSymbol::ALIGN_CENTER_CENTER;
    }


    // rebuild the graph:
    _root = new DrapeableNode( getMapNode(), false );
    this->addChild( _root );

    // build the base Grid Zone Designator (GZD) loolup table. This is a table
    // that maps the GZD string to its extent.
    static std::string s_gzdRows( "CDEFGHJKLMNPQRSTUVWX" );
    const SpatialReference* geosrs = _profile->getSRS()->getGeographicSRS();

    // build the lateral zones:
    for( unsigned zone = 0; zone < 60; ++zone )
    {
        for( unsigned row = 0; row < s_gzdRows.size(); ++row )
        {
            double yMaxExtra = row == s_gzdRows.size()-1 ? 4.0 : 0.0; // extra 4 deg for row X

            GeoExtent cellExtent(
                geosrs,
                -180.0 + double(zone)*6.0,
                -80.0  + row*8.0,
                -180.0 + double(zone+1)*6.0,
                -80.0  + double(row+1)*8.0 + yMaxExtra );

            _gzd[ Stringify() << (zone+1) << s_gzdRows[row] ] = cellExtent;
        }        
    }

    // the polar zones (UPS):
    _gzd["1Y"] = GeoExtent( geosrs, -180.0,  84.0,   0.0,  90.0 );
    _gzd["1Z"] = GeoExtent( geosrs,    0.0,  84.0, 180.0,  90.0 );
    _gzd["1A"] = GeoExtent( geosrs, -180.0, -90.0,   0.0, -80.0 );
    _gzd["1B"] = GeoExtent( geosrs,    0.0, -90.0, 180.0, -80.0 );

    // replace the "exception" zones in Norway and Svalbard
    _gzd["31V"] = GeoExtent( geosrs, 0.0, 56.0, 3.0, 64.0 );
    _gzd["32V"] = GeoExtent( geosrs, 3.0, 56.0, 12.0, 64.0 );
    _gzd["31X"] = GeoExtent( geosrs, 0.0, 72.0, 9.0, 84.0 );
    _gzd["33X"] = GeoExtent( geosrs, 9.0, 72.0, 21.0, 84.0 );
    _gzd["35X"] = GeoExtent( geosrs, 21.0, 72.0, 33.0, 84.0 );
    _gzd["37X"] = GeoExtent( geosrs, 33.0, 72.0, 42.0, 84.0 );

    // ..and remove the non-existant zones:
    _gzd.erase( "32X" );
    _gzd.erase( "34X" );
    _gzd.erase( "36X" );

    // now build the lateral tiles for the GZD level.
    for( SectorTable::iterator i = _gzd.begin(); i != _gzd.end(); ++i )
    {
        osg::Node* tile = buildGZDTile( i->first, i->second );
        if ( tile )
            _root->addChild( tile );
    }

    //DepthOffsetUtils::prepareGraph( _root );
}


osg::Node*
UTMGraticule::buildGZDTile( const std::string& name, const GeoExtent& extent )
{
    osg::Group* group = new osg::Group();

    Style lineStyle;
    lineStyle.add( _options->primaryStyle()->get<LineSymbol>() );
    lineStyle.add( _options->primaryStyle()->get<AltitudeSymbol>() );

    bool hasText = _options->primaryStyle()->get<TextSymbol>() != 0L;

    GeometryCompiler compiler;
    osg::ref_ptr<Session> session = new Session( getMapNode()->getMap() );
    FilterContext context( session.get(), _featureProfile.get(), extent );

    // make sure we get sufficient tessellation:
    compiler.options().maxGranularity() = 1.0;

    FeatureList features;

    // longitudinal line:
    LineString* lon = new LineString(2);
    lon->push_back( osg::Vec3d(extent.xMin(), extent.yMax(), 0) );
    lon->push_back( osg::Vec3d(extent.xMin(), extent.yMin(), 0) );
    Feature* lonFeature = new Feature(lon, extent.getSRS());
    lonFeature->geoInterp() = GEOINTERP_GREAT_CIRCLE;
    features.push_back( lonFeature );

    // latitudinal line:
    LineString* lat = new LineString(2);
    lat->push_back( osg::Vec3d(extent.xMin(), extent.yMin(), 0) );
    lat->push_back( osg::Vec3d(extent.xMax(), extent.yMin(), 0) );
    Feature* latFeature = new Feature(lat, extent.getSRS());
    latFeature->geoInterp() = GEOINTERP_RHUMB_LINE;
    features.push_back( latFeature );

    // top lat line at 84N
    if ( extent.yMax() == 84.0 )
    {
        LineString* lat = new LineString(2);
        lat->push_back( osg::Vec3d(extent.xMin(), extent.yMax(), 0) );
        lat->push_back( osg::Vec3d(extent.xMax(), extent.yMax(), 0) );
        Feature* latFeature = new Feature(lat, extent.getSRS());
        latFeature->geoInterp() = GEOINTERP_RHUMB_LINE;
        features.push_back( latFeature );
    }

    osg::Node* geomNode = compiler.compile(features, lineStyle, context);
    if ( geomNode ) 
        group->addChild( geomNode );

    // get the geocentric tile center:
    osg::Vec3d tileCenter;
    extent.getCentroid( tileCenter.x(), tileCenter.y() );

    const SpatialReference* ecefSRS = extent.getSRS()->getECEF();
    
    osg::Vec3d centerECEF;
    extent.getSRS()->transform( tileCenter, ecefSRS, centerECEF );
    //extent.getSRS()->transformToECEF( tileCenter, centerECEF );

    if ( hasText )
    {
        osg::Vec3d west, east;
        extent.getSRS()->transform( osg::Vec3d(extent.xMin(),tileCenter.y(),0), ecefSRS, west );
        extent.getSRS()->transform( osg::Vec3d(extent.xMax(),tileCenter.y(),0), ecefSRS, east );
        //extent.getSRS()->transformToECEF(osg::Vec3d(extent.xMin(),tileCenter.y(),0), west );
        //extent.getSRS()->transformToECEF(osg::Vec3d(extent.xMax(),tileCenter.y(),0), east );

        TextSymbol* textSym = _options->primaryStyle()->getOrCreate<TextSymbol>();
        textSym->size() = (west-east).length() / 3.0;

        TextSymbolizer ts( textSym );
        
        osg::Geode* textGeode = new osg::Geode();
        textGeode->getOrCreateStateSet()->setRenderBinDetails( 9998, "DepthSortedBin" );   
        textGeode->getOrCreateStateSet()->setAttributeAndModes( _depthAttribute, 1 );
        
        osg::Drawable* d = ts.create(name);
        d->getOrCreateStateSet()->setRenderBinToInherit();

        textGeode->addDrawable(d);
        osg::Matrixd centerL2W;
        ecefSRS->createLocalToWorld( centerECEF, centerL2W );
        osg::MatrixTransform* mt = new osg::MatrixTransform(centerL2W);
        mt->addChild(textGeode);
       
        group->addChild(mt);
    }

    group = buildGZDChildren( group, name );

    group = ClusterCullingFactory::createAndInstall( group, centerECEF )->asGroup();

    return group;
}


