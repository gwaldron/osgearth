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
#include <osgEarthUtil/GeodeticGraticule>
#include <osgEarthUtil/LatLongFormatter>

#include <osgEarthFeatures/GeometryCompiler>
#include <osgEarthSymbology/Geometry>
#include <osgEarthAnnotation/LabelNode>

#include <osgEarth/Registry>
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

#define LC "[GeodeticGraticule] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;
using namespace osgEarth::Annotation;

static Threading::Mutex s_graticuleMutex;
typedef std::map<unsigned int, osg::ref_ptr<GeodeticGraticule> > GeodeticGraticuleRegistry;
static GeodeticGraticuleRegistry s_graticuleRegistry;

#define GRATICULE_EXTENSION "osgearthutil_geodetic_graticule"
#define TEXT_MARKER "t"
#define GRID_MARKER "g"

//---------------------------------------------------------------------------

GeodeticGraticuleOptions::GeodeticGraticuleOptions( const Config& conf ) :
ConfigOptions( conf )
{
    mergeConfig( _conf );
}

void
GeodeticGraticuleOptions::mergeConfig( const Config& conf )
{
    //todo
}

Config
GeodeticGraticuleOptions::getConfig() const
{
    Config conf = ConfigOptions::newConfig();
    conf.key() = "geodetic_graticule";
    //todo
    return conf;
}

void
GeodeticGraticuleOptions::addLevel( float maxRange, float minRange, unsigned subFactor, const Style& lineStyle, const Style& textStyle )
{
    Level level;
    level._maxRange = maxRange;
    level._minRange = minRange;
    level._subdivisionFactor = subFactor;
    if ( !lineStyle.empty() )
        level._lineStyle = lineStyle;
    if ( !textStyle.empty() )
        level._textStyle = textStyle;

    _levels.push_back(level);
}

//---------------------------------------------------------------------------

GeodeticGraticule::GeodeticGraticule( MapNode* mapNode ) :
_mapNode   ( mapNode ),
_root      ( 0L )
{
    init();
}

GeodeticGraticule::GeodeticGraticule( MapNode* mapNode, const GeodeticGraticuleOptions& options ) :
_mapNode   ( mapNode ),
_root      ( 0L )
{
    _options = options;
    init();
}

void
GeodeticGraticule::init()
{
    // safely generate a unique ID for this graticule:
    _id = Registry::instance()->createUID();
    {
        Threading::ScopedMutexLock lock( s_graticuleMutex );
        s_graticuleRegistry[_id] = this;
    }

    // this will intialize the graph.
    rebuild();
}

void
GeodeticGraticule::setMapNode( MapNode* mapNode )
{
    _mapNode = mapNode;
    rebuild();
}

void
GeodeticGraticule::setOptions( const GeodeticGraticuleOptions& options )
{
    _options = options;
    rebuild();
}

void
GeodeticGraticule::rebuild()
{
    this->removeChildren( 0, this->getNumChildren() );

    if ( !getMapNode() )
    {
        OE_WARN << LC << "Illegal NULL map node" << std::endl;
        return;
    }

    if ( !getMapNode()->isGeocentric() )
    {
        OE_WARN << LC << "Projected map mode is not yet supported" << std::endl;
        return;
    }

    const Profile* mapProfile = _mapNode->getMap()->getProfile();

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
        TextSymbol* text = _options->textStyle()->getOrCreate<TextSymbol>();
        text->alignment() = TextSymbol::ALIGN_CENTER_CENTER;

        if ( _mapNode->isGeocentric() )
        {
            double r = _mapNode->getMapSRS()->getEllipsoid()->getRadiusEquator();
            _options->addLevel( FLT_MAX, 0.0f, 1u );
            double d = 4.5*r;
            for(int i=0; i<3; i++)
            {
                d *= 0.5;
                _options->addLevel( d, d*0.25 );
            }
        }
        else
        {
            //todo?
        }
    }

    DrapeableNode* drapeable = new DrapeableNode();
    drapeable->setDrapingEnabled( false );
    _root = drapeable;
    this->addChild( _root );

    // need at least one level
    if ( _options->levels().size() < 1 )
        return;

    const GeodeticGraticuleOptions::Level& level0 = _options->levels()[0];

    // build the top level cell grid.
    unsigned tilesX, tilesY;
    _profile->getNumTiles( 0, tilesX, tilesY );

    for( unsigned tx = 0; tx < tilesX; ++tx )
    {
        for( unsigned ty = 0; ty < tilesY; ++ty )
        {
            TileKey key( 0, tx, ty, _profile.get() );
            osg::Node* tile = buildTile( key, getMapNode()->getMap() );
            if ( tile )
                _root->addChild( tile );
        }
    }
}


osg::Node*
GeodeticGraticule::buildTile( const TileKey& key, Map* map ) const
{
    if ( _options->levels().size() <= key.getLevelOfDetail() )
    {
        OE_WARN << LC << "Tried to create cell at non-existant level " << key.getLevelOfDetail() << std::endl;
        return 0L;
    }

    const GeodeticGraticuleOptions::Level& level = _options->levels()[key.getLevelOfDetail()]; //_levels[key.getLevelOfDetail()];


    // the "-2" here is because normal tile paging gives you one subdivision already,
    // so we only need to account for > 1 subdivision factor.
    unsigned cellsPerTile = level._subdivisionFactor <= 2u ? 1u : 1u << (level._subdivisionFactor-2u);
    unsigned cellsPerTileX = std::max(1u, cellsPerTile);
    unsigned cellsPerTileY = std::max(1u, cellsPerTile);


    GeoExtent tileExtent = key.getExtent();

    FeatureList latLines;
    FeatureList lonLines;

    static LatLongFormatter s_llf(LatLongFormatter::FORMAT_DECIMAL_DEGREES);
    
    double cellWidth = tileExtent.width() / cellsPerTileX;
    double cellHeight = tileExtent.height() / cellsPerTileY;

    const Style& lineStyle = level._lineStyle.isSet() ? *level._lineStyle : *_options->lineStyle();
    const Style& textStyle = level._textStyle.isSet() ? *level._textStyle : *_options->textStyle();

    bool hasText = textStyle.get<TextSymbol>() != 0L;

    osg::ref_ptr<osg::Group> labels;
    if ( hasText )
    {
        labels = new osg::Group();
    }

    // spatial ref for features:
    const SpatialReference* geoSRS = tileExtent.getSRS()->getGeographicSRS();

    // longitude lines
    for( unsigned cx = 0; cx < cellsPerTileX; ++cx )
    {
        double clon = tileExtent.xMin() + cellWidth * (double)cx;
        LineString* lon = new LineString(2);
        lon->push_back( osg::Vec3d(clon, tileExtent.yMin(), 0) );
        lon->push_back( osg::Vec3d(clon, tileExtent.yMax(), 0) );
        lonLines.push_back( new Feature(lon, geoSRS) );

        if ( hasText )
        {
            for( unsigned cy = 0; cy < cellsPerTileY; ++cy )
            {
                double clat = tileExtent.yMin() + (0.5*cellHeight) + cellHeight*(double)cy;
                LabelNode* label = new LabelNode( 
                    _mapNode.get(),
                    GeoPoint(geoSRS, clon, clat),
                    s_llf.format(clon, false),
                    textStyle );
                labels->addChild( label );
            }
        }
    }

    // latitude lines
    for( unsigned cy = 0; cy < cellsPerTileY; ++cy )
    {
        double clat = tileExtent.yMin() + cellHeight * (double)cy;
        if ( clat == key.getProfile()->getExtent().yMin() )
            continue;

        LineString* lat = new LineString(2);
        lat->push_back( osg::Vec3d(tileExtent.xMin(), clat, 0) );
        lat->push_back( osg::Vec3d(tileExtent.xMax(), clat, 0) );
        latLines.push_back( new Feature(lat, geoSRS) );

        if ( hasText )
        {
            for( unsigned cx = 0; cx < cellsPerTileX; ++cx )
            {
                double clon = tileExtent.xMin() + (0.5*cellWidth) + cellWidth*(double)cy;
                LabelNode* label = new LabelNode( 
                    _mapNode.get(), 
                    GeoPoint(geoSRS, clon, clat),
                    s_llf.format(clat, true),
                    textStyle );
                labels->addChild( label );
            }
        }
    }

    osg::Group* group = new osg::Group();

    GeometryCompiler compiler;
    osg::ref_ptr<Session> session = new Session( map );
    FilterContext context( session.get(), _featureProfile.get(), tileExtent );

    // make sure we get sufficient tessellation:
    compiler.options().maxGranularity() = std::min(cellWidth, cellHeight) / 16.0;

    compiler.options().geoInterp() = GEOINTERP_GREAT_CIRCLE;
    osg::Node* lonNode = compiler.compile(lonLines, lineStyle, context);
    if ( lonNode )
        group->addChild( lonNode );

    compiler.options().geoInterp() = GEOINTERP_RHUMB_LINE;
    osg::Node* latNode = compiler.compile(latLines, lineStyle, context);
    if ( latNode )
        group->addChild( latNode );

    // add the labels.
    if ( labels.valid() )
        group->addChild( labels.get() );

    // get the geocentric tile center:
    osg::Vec3d tileCenter;
    tileExtent.getCentroid( tileCenter.x(), tileCenter.y() );

    const SpatialReference* ecefSRS = tileExtent.getSRS()->getECEF();
    osg::Vec3d centerECEF;
    tileExtent.getSRS()->transform( tileCenter, ecefSRS, centerECEF );
    //tileExtent.getSRS()->transformToECEF( tileCenter, centerECEF );

    osg::NodeCallback* ccc = 0L;
    // set up cluster culling.
    if ( tileExtent.getSRS()->isGeographic() && tileExtent.width() < 90.0 && tileExtent.height() < 90.0 )
    {
        ccc = ClusterCullingFactory::create( group, centerECEF );
    }

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

/**************************************************************************/

namespace osgEarth { namespace Util
{
    // OSG Plugin for loading subsequent graticule levels
    class GeodeticGraticuleFactory : public osgDB::ReaderWriter
    {
    public:
        GeodeticGraticuleFactory()
        {
            supportsExtension( GRATICULE_EXTENSION, "osgEarth graticule" );
        }

    public: // osgDB::ReaderWriter

        const char* className() const
        {
            return "osgEarth graticule LOD loader";
        }

        bool acceptsExtension(const std::string& extension) const
        {
            return osgDB::equalCaseInsensitive(extension, GRATICULE_EXTENSION);
        }

        ReadResult readNode(const std::string& uri, const Options* options) const
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
            GeodeticGraticule* graticule = 0L;
            {
                Threading::ScopedMutexLock lock( s_graticuleMutex );
                GeodeticGraticuleRegistry::iterator i = s_graticuleRegistry.find(id);
                if ( i != s_graticuleRegistry.end() )
                    graticule = i->second.get();
            }

            osg::Node* result = 0L;
            if (graticule)
                result = graticule->buildChildren( levelNum, x, y );

            return result ? ReadResult(result) : ReadResult::ERROR_IN_READING_FILE;
        }
    };
    REGISTER_OSGPLUGIN(GRATICULE_EXTENSION, GeodeticGraticuleFactory)

} } // namespace osgEarth::Util

