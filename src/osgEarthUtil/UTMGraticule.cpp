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
#include <osgEarthUtil/UTMGraticule>
#include <osgEarthUtil/Formatters>

#include <osgEarthFeatures/GeometryCompiler>
#include <osgEarthFeatures/TextSymbolizer>

#include <osgEarthSymbology/Geometry>
#include <osgEarthAnnotation/LabelNode>
#include <osgEarthAnnotation/Decluttering>

#include <osgEarth/Registry>
#include <osgEarth/ECEF>
#include <osgEarth/FindNode>
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

static Threading::Mutex s_graticuleMutex;
typedef std::map<unsigned int, osg::ref_ptr<UTMGraticule> > UTMGraticuleRegistry;
static UTMGraticuleRegistry s_graticuleRegistry;

#define UTM_GRATICULE_EXTENSION "osgearthutil_utm_graticule"
#define TEXT_MARKER "t"
#define GRID_MARKER "g"

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
    if ( !_mapNode.valid() )
    {
        OE_WARN << LC << "Illegal NULL map node" << std::endl;
        return;
    }

    if ( !_mapNode->isGeocentric() )
    {
        OE_WARN << LC << "Projected map mode is not yet supported" << std::endl;
        return;
    }

    // safely generate a unique ID for this graticule:
    _id = Registry::instance()->createUID();
    {
        Threading::ScopedMutexLock lock( s_graticuleMutex );
        s_graticuleRegistry[_id] = this;
    }

    const Profile* mapProfile = _mapNode->getMap()->getProfile();

    _profile = Profile::create(
        mapProfile->getSRS(),
        mapProfile->getExtent().xMin(),
        mapProfile->getExtent().yMin(),
        mapProfile->getExtent().xMax(),
        mapProfile->getExtent().yMax(),
        mapProfile->getVerticalSRS(),
        8, 4 );

    _featureProfile = new FeatureProfile(_profile->getSRS());

    //todo: do this right..
    osg::StateSet* set = this->getOrCreateStateSet();
    set->setRenderBinDetails( 9999, "RenderBin" );
    set->setMode( GL_LIGHTING, 0 );

    // set up default options if the caller did not supply them
    if ( !_options.isSet() )
    {
        _options->lineStyle() = Style();

        LineSymbol* line = _options->lineStyle()->getOrCreate<LineSymbol>();
        line->stroke()->color() = Color::Gray;
        line->stroke()->width() = 1.0;

        AltitudeSymbol* alt = _options->lineStyle()->getOrCreate<AltitudeSymbol>();
        alt->verticalOffset() = NumericExpression(5000.0);

        _options->textStyle() = Style();
        _options->textStyle()->addSymbol( alt );

        TextSymbol* text = _options->textStyle()->getOrCreate<TextSymbol>();
        text->fill()->color() = Color(Color::Gray,0.6f);
        text->alignment() = TextSymbol::ALIGN_CENTER_CENTER;
    }

    // this will intialize the graph.
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
    // intialize the container if necessary
    if ( !_root )
    {
        _root = new DrapeableNode( _mapNode.get(), false );
        this->addChild( _root );
    }
    else
    {
        _root->removeChildren(0, _root->getNumChildren());
    }

    // build the base grid.
    static std::string s_utmRows( "CDEFGHJKLMNPQRSTUVWX" );
    std::map<std::string, GeoExtent> zoneMap;
    const SpatialReference* geosrs = _profile->getSRS()->getGeographicSRS();

    // build the lateral zones:
    for( unsigned zone = 0; zone < 60; ++zone )
    {
        for( unsigned row = 0; row < s_utmRows.size(); ++row )
        {
            double yMaxExtra = row == s_utmRows.size()-1 ? 4.0 : 0.0; // extra 4 deg for row X

            GeoExtent cellExtent(
                geosrs,
                -180.0 + double(zone)*6.0,
                -80.0  + row*8.0,
                -180.0 + double(zone+1)*6.0,
                -80.0  + double(row+1)*8.0 + yMaxExtra );

            zoneMap[ Stringify() << (zone+1) << s_utmRows[row] ] = cellExtent;
        }        
    }

    // replace the "exception" zones in Norway and Svalbard
    zoneMap["31V"] = GeoExtent( geosrs, 0.0, 56.0, 3.0, 64.0 );
    zoneMap["32V"] = GeoExtent( geosrs, 3.0, 56.0, 12.0, 64.0 );
    zoneMap["31X"] = GeoExtent( geosrs, 0.0, 72.0, 9.0, 84.0 );
    zoneMap["33X"] = GeoExtent( geosrs, 9.0, 72.0, 21.0, 84.0 );
    zoneMap["35X"] = GeoExtent( geosrs, 21.0, 72.0, 33.0, 84.0 );
    zoneMap["37X"] = GeoExtent( geosrs, 33.0, 72.0, 42.0, 84.0 );

    // ..and remove the non-existant zones:
    zoneMap.erase( "32X" );
    zoneMap.erase( "34X" );
    zoneMap.erase( "36X" );

    // now build the lateral tiles.
    for( std::map<std::string,GeoExtent>::iterator i = zoneMap.begin(); i != zoneMap.end(); ++i )
    {
        osg::Node* tile = buildUTMTile( i->first, i->second );
        if ( tile )
            _root->addChild( tile );
    }

    // and the polar tiles.
    _root->addChild( buildUPSTiles() );
}


osg::Node*
UTMGraticule::buildUTMTile( const std::string& name, const GeoExtent& extent )
{
    osg::Group* group = new osg::Group();

    const Style& lineStyle = *_options->lineStyle();
    Style textStyle = *_options->textStyle();

    bool hasText = textStyle.get<TextSymbol>() != 0L;

    GeometryCompiler compiler;
    osg::ref_ptr<Session> session = new Session( _mapNode->getMap() );
    FilterContext context( session.get(), _featureProfile.get(), extent );

    // make sure we get sufficient tessellation:
    compiler.options().maxGranularity() = 1.0;

    FeatureList features;

    // longitudinal line:
    LineString* lon = new LineString(2);
    lon->push_back( osg::Vec3d(extent.xMin(), extent.yMax(), 0) );
    lon->push_back( osg::Vec3d(extent.xMin(), extent.yMin(), 0) );
    Feature* lonFeature = new Feature(lon);
    lonFeature->geoInterp() = GEOINTERP_GREAT_CIRCLE;
    features.push_back( lonFeature );

    // latitudinal line:
    LineString* lat = new LineString(2);
    lat->push_back( osg::Vec3d(extent.xMin(), extent.yMin(), 0) );
    lat->push_back( osg::Vec3d(extent.xMax(), extent.yMin(), 0) );
    Feature* latFeature = new Feature(lat);
    latFeature->geoInterp() = GEOINTERP_RHUMB_LINE;
    features.push_back( latFeature );

    // top lat line at 84N
    if ( extent.yMax() == 84.0 )
    {
        LineString* lat = new LineString(2);
        lat->push_back( osg::Vec3d(extent.xMin(), extent.yMax(), 0) );
        lat->push_back( osg::Vec3d(extent.xMax(), extent.yMax(), 0) );
        Feature* latFeature = new Feature(lat);
        latFeature->geoInterp() = GEOINTERP_RHUMB_LINE;
        features.push_back( latFeature );
    }

    osg::Node* geomNode = compiler.compile(features, lineStyle, context);
    if ( geomNode ) 
        group->addChild( geomNode );

    // get the geocentric tile center:
    osg::Vec3d tileCenter;
    extent.getCentroid( tileCenter.x(), tileCenter.y() );
    
    osg::Vec3d centerECEF;
    extent.getSRS()->transformToECEF( tileCenter, centerECEF );

    if ( hasText )
    {
        osg::Vec3d west, east;
        extent.getSRS()->transformToECEF(osg::Vec3d(extent.xMin(),tileCenter.y(),0), west );
        extent.getSRS()->transformToECEF(osg::Vec3d(extent.xMax(),tileCenter.y(),0), east );

        TextSymbol* textSym = textStyle.get<TextSymbol>();
        textSym->size() = (west-east).length() / 3.0;

        TextSymbolizer ts( textSym );
        
        osg::Geode* geode = new osg::Geode();
        osg::Drawable* d = ts.create(name);
        geode->addDrawable(d);
        osg::MatrixTransform* mt = new osg::MatrixTransform(ECEF::createLocalToWorld(centerECEF));
        mt->addChild(geode);
       
        group->addChild(mt);
    }

    osg::NodeCallback* ccc = 0L;
    // set up cluster culling.
    if ( extent.getSRS()->isGeographic() && extent.width() < 90.0 && extent.height() < 90.0 )
    {
        ccc = ClusterCullingFactory::create( group, centerECEF );
    }

#if 0
    // add a paging node for higher LODs:
    if ( key.getLevelOfDetail() + 1 < _options->levels().size() )
    {
        const GeodeticGraticuleOptions::Level& nextLevel = _options->levels()[key.getLevelOfDetail()+1];

        osg::BoundingSphere bs = group->getBound();

        std::string uri = Stringify() << key.str() << "_" << getID() << "." << GRID_MARKER << "." << GRATICULE_EXTENSION;

        osg::PagedLOD* plod = new osg::PagedLOD();
        plod->setCenter( bs.center() );
        plod->addChild( group, std::max(level._minRange,nextLevel._maxRange), FLT_MAX );
        plod->setFileName( 1, uri );
        plod->setRange( 1, 0, nextLevel._maxRange );
        group = plod;
    }

    // or, if this is the deepest level and there's a minRange set, we need an LOD:
    else if ( level._minRange > 0.0f )
    {
        osg::LOD* lod = new osg::LOD();
        lod->addChild( group, level._minRange, FLT_MAX );
        group = lod;
    }
#endif

    if ( ccc )
    {
        osg::Group* cccGroup = new osg::Group();
        cccGroup->addCullCallback( ccc );
        cccGroup->addChild( group );
        group = cccGroup;
    }

    return group;
}

osg::Node*
UTMGraticule::buildUPSTiles()
{
    osg::Group* group = new osg::Group();

    const Style& lineStyle = *_options->lineStyle();
    Style textStyle = *_options->textStyle();
    TextSymbol* ts = textStyle.getOrCreate<TextSymbol>();
    const SpatialReference* geosrs = _mapNode->getMapSRS()->getGeographicSRS();

    osg::Vec3d northECEF(0, 0, geosrs->getEllipsoid()->getRadiusPolar());

    GeometryCompiler compiler;
    compiler.options().maxGranularity() = 1.0;
    osg::ref_ptr<Session> session = new Session( _mapNode->getMap() );
    GeoExtent extent( _mapNode->getMap()->getProfile()->getExtent() );
    FilterContext context( session.get(), _featureProfile.get(), extent );
    TextSymbolizer symbolizer( textStyle.get<TextSymbol>() );

    osg::ref_ptr<Feature> northF = new Feature(new LineString(2));
    northF->getGeometry()->push_back( osg::Vec3d(0.0, 84.0, 0) );
    northF->getGeometry()->push_back( osg::Vec3d(180.0, 84, 0) );
    northF->geoInterp() = GEOINTERP_GREAT_CIRCLE;
    osg::Node* northNode = compiler.compile(northF.get(), lineStyle, context);
    if ( northNode )
    {
        osg::Group* northGroup = new osg::Group();
        northGroup->addChild( northNode );
        
        osg::Geode* geode = new osg::Geode();
        ts->size() = 120000.0;

        osgText::Text* yText = symbolizer.create("Y");
        osg::Vec3d yPos( -90, 87.0, 0 );
        geosrs->transformToECEF(yPos, yPos);
        osg::Matrixd yLocal2World = ECEF::createLocalToWorld( yPos );
        yText->setRotation( yLocal2World.getRotate() );
        yText->setPosition( yPos - northECEF );
        geode->addDrawable( yText );

        osgText::Text* zText = symbolizer.create("Z");
        osg::Vec3d zPos(  90, 87.0, 0 );
        geosrs->transformToECEF(zPos, zPos);
        osg::Matrixd zLocal2World = ECEF::createLocalToWorld( zPos );
        zText->setRotation( yLocal2World.getRotate() * osg::Quat(osg::PI,osg::Vec3(0,0,1)) );
        zText->setPosition( zPos - northECEF );
        geode->addDrawable( zText );

        osg::MatrixTransform* mt = new osg::MatrixTransform( osg::Matrix::translate(northECEF) );
        mt->addChild( geode );

        northGroup->addChild( mt );
        northGroup = ClusterCullingFactory::createAndInstall(northGroup, northECEF)->asGroup();

        group->addChild( northGroup );
    }

    osg::ref_ptr<Feature> southF = new Feature(new LineString(2));
    southF->getGeometry()->push_back( osg::Vec3d(0.0, -80.0, 0) );
    southF->getGeometry()->push_back( osg::Vec3d(180.0, -80.0, 0) );
    southF->geoInterp() = GEOINTERP_GREAT_CIRCLE;
    osg::Node* southNode = compiler.compile(southF.get(), lineStyle, context);
    if ( southNode )
    {
        osg::Vec3d southECEF = -northECEF;

        osg::Group* southGroup = new osg::Group();
        southGroup->addChild( southNode );
        
        osg::Geode* geode = new osg::Geode();
        ts->size() = 120000.0;

        osgText::Text* aText = symbolizer.create("A");
        osg::Vec3d aPos( -90, -85.0, 0 );
        geosrs->transformToECEF(aPos, aPos);
        osg::Matrixd aLocal2World = ECEF::createLocalToWorld( aPos );
        aText->setRotation( aLocal2World.getRotate() );
        aText->setPosition( aPos - southECEF );
        geode->addDrawable( aText );

        osgText::Text* bText = symbolizer.create("B");
        osg::Vec3d bPos(  90, -85.0, 0 );
        geosrs->transformToECEF(bPos, bPos);
        osg::Matrixd bLocal2World = ECEF::createLocalToWorld( bPos );
        bText->setRotation( osg::Quat(osg::PI,osg::Vec3(0,0,1)) * bLocal2World.getRotate() );
        bText->setPosition( bPos - southECEF );
        geode->addDrawable( bText );

        osg::MatrixTransform* mt = new osg::MatrixTransform( osg::Matrix::translate(southECEF) );
        mt->addChild( geode );

        southGroup->addChild( mt );
        southGroup = ClusterCullingFactory::createAndInstall(southGroup, southECEF)->asGroup();

        group->addChild( southGroup );
    }

    return group;
}


#if 0
osg::Node*
GeodeticGraticule::buildChildren( unsigned level, unsigned x, unsigned y ) const
{
    osg::ref_ptr<MapNode> mapNodeSafe = _mapNode.get();
    if ( mapNodeSafe.valid() )
    {
        TileKey parent(level, x, y, _profile.get());
        osg::Group* g = new osg::Group();
        for( unsigned q=0; q<4; ++q )
        {
            TileKey child = parent.createChildKey( q );
            osg::Node* n = buildTile(child, mapNodeSafe->getMap() );
            if ( n )
                g->addChild( n );
        }
        return g;
    }
    else return 0L;
}

void
GeodeticGraticule::traverse( osg::NodeVisitor& nv )
{
    if ( nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
    {
    }
    osg::Group::traverse( nv );
}
#endif

/**************************************************************************/

#if 0
namespace osgEarth { namespace Util
{
    // OSG Plugin for loading subsequent graticule levels
    class GeodeticGraticuleFactory : public osgDB::ReaderWriter
    {
    public:
        virtual const char* className()
        {
            supportsExtension( UTM_GRATICULE_EXTENSION, "osgEarth UTM raticule" );
            return "osgEarth UTM graticule LOD loader";
        }

        virtual bool acceptsExtension(const std::string& extension) const
        {
            return osgDB::equalCaseInsensitive(extension, UTM_GRATICULE_EXTENSION);
        }

        virtual ReadResult readNode(const std::string& uri, const Options* options) const
        {        
            std::string ext = osgDB::getFileExtension( uri );
            if ( !acceptsExtension( ext ) )
                return ReadResult::FILE_NOT_HANDLED;

            // the graticule definition is formatted: LEVEL_ID.MARKER.EXTENSION
            std::string def = osgDB::getNameLessExtension( uri );
            
            std::string marker = osgDB::getFileExtension( def );
            def = osgDB::getNameLessExtension( def );

            int levelNum, x, y, id;
            sscanf( def.c_str(), "%d/%d/%d_%d", &levelNum, &x, &y, &id );

            // look up the graticule referenced in the location name:
            UTMGraticule* graticule = 0L;
            {
                Threading::ScopedMutexLock lock( s_graticuleMutex );
                UTMGraticuleRegistry::iterator i = s_graticuleRegistry.find(id);
                if ( i != s_graticuleRegistry.end() )
                    graticule = i->second.get();
            }

            osg::Node* result = graticule->buildChildren( levelNum, x, y );
            return result ? ReadResult(result) : ReadResult::ERROR_IN_READING_FILE;
        }
    };
    REGISTER_OSGPLUGIN(UTM_GRATICULE_EXTENSION, UTMGraticuleFactory)

} } // namespace osgEarth::Util
#endif

