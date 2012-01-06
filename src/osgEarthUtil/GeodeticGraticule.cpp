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
#include <osgEarthUtil/GeodeticGraticule>
#include <osgEarthUtil/AutoClipPlaneHandler>
#include <osgEarthFeatures/GeometryCompiler>
#include <osgEarthSymbology/Geometry>
#include <osgEarth/Registry>
#include <osgEarth/FindNode>
#include <osgEarth/Utils>
#include <osgEarth/CullingUtils>
#include <osgEarth/DrapeableNode>
#include <OpenThreads/Mutex>
#include <OpenThreads/ScopedLock>
#include <osg/PagedLOD>
#include <osg/ProxyNode>
#include <osg/MatrixTransform>
#include <osg/Depth>
#include <osg/Program>
#include <osg/LineStipple>
#include <osgDB/FileNameUtils>
#include <osgUtil/Optimizer>
#include <osgText/Text>
#include <sstream>
#include <iomanip>

#define LC "[GeodeticGraticule] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;
using namespace OpenThreads;

static Mutex s_graticuleMutex;
typedef std::map<unsigned int, osg::ref_ptr<GeodeticGraticule> > GeodeticGraticuleRegistry;
static GeodeticGraticuleRegistry s_graticuleRegistry;

#define GRATICULE_EXTENSION "osgearthutil_geodetic_graticule"
#define TEXT_MARKER "t"
#define GRID_MARKER "g"

//---------------------------------------------------------------------------

namespace
{
    char s_vertexShader[] =
        "varying vec3 Normal; \n"
        "void main(void) \n"
        "{ \n"
        "    Normal = normalize( gl_NormalMatrix * gl_Normal ); \n"
        "    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex; \n"
        "    gl_FrontColor = gl_Color; \n"
        "} \n";

    char s_fragmentShader[] =
        "varying vec3 Normal; \n"
        "void main(void) \n"
        "{ \n"
        "    gl_FragColor = gl_Color; \n"
        "} \n";
}

//---------------------------------------------------------------------------

GeodeticGraticule::GeodeticGraticule( MapNode* mapNode ) :
_mapNode   ( mapNode ),
_autoLevels( true ),
_textColor ( 1,1,0,1 ),
_root      ( 0L )
{
    // safely generate a unique ID for this graticule:
    _id = Registry::instance()->createUID();
    {
        ScopedLock<Mutex> lock( s_graticuleMutex );
        s_graticuleRegistry[_id] = this;
    }

    LineSymbol* line = _style.getOrCreate<LineSymbol>();
    line->stroke()->color() = Color::Gray;
    line->stroke()->width() = 1.0;

    AltitudeSymbol* alt = _style.getOrCreate<AltitudeSymbol>();
    alt->verticalOffset() = NumericExpression(25000.0);

    //TextSymbol* text = _style.getOrCreate<TextSymbol>();

    const Profile* mapProfile = mapNode->getMap()->getProfile();
    _profile = Profile::create(
        mapProfile->getSRS(),
        mapProfile->getExtent().xMin(),
        mapProfile->getExtent().yMin(),
        mapProfile->getExtent().xMax(),
        mapProfile->getExtent().yMax(),
        mapProfile->getVerticalSRS(),
        8, 4 );
    //_profile = mapNode->getMap()->getProfile();
    _featureProfile = new FeatureProfile(_profile->getSRS());

    if ( _mapNode->isGeocentric() )
    {
        double r = _mapNode->getMapSRS()->getEllipsoid()->getRadiusEquator();

        addLevel( FLT_MAX, 4, 4 );

        double d = 4.5*r;
        for(int i=0; i<3; i++)
        {
            d *= 0.5;
            addLevel( d, 4, 4 );
        }
    }
    else
    {
        // ??? calculate an optimal # of cells per tile
    }

    osg::StateSet* set = this->getOrCreateStateSet();
    set->setRenderBinDetails( 9999, "RenderBin" );
    //set->setAttributeAndModes( 
    //    new osg::Depth( osg::Depth::LEQUAL, 1 ) );
        //new osg::Depth( osg::Depth::ALWAYS ), 
        //osg::StateAttribute::ON | osg::StateAttribute::PROTECTED );
    set->setMode( GL_LIGHTING, 0 );

    //osg::Program* program = new osg::Program();
    //program->addShader( new osg::Shader( osg::Shader::VERTEX, s_vertexShader ) );
    //program->addShader( new osg::Shader( osg::Shader::FRAGMENT, s_fragmentShader ) );
    //set->setAttributeAndModes( program, osg::StateAttribute::ON );

    init();
}

void
GeodeticGraticule::init()
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

    // need at least one level
    if ( _levels.size() < 1 )
        return;

    const Level& level0 = _levels[0];

    // build the top level cell grid.
    unsigned tilesX, tilesY;
    _profile->getNumTiles( 0, tilesX, tilesY );

    for( unsigned tx = 0; tx < tilesX; ++tx )
    {
        for( unsigned ty = 0; ty < tilesY; ++ty )
        {
            TileKey key( 0, tx, ty, _profile.get() );
            osg::Node* tile = buildTile( key, _mapNode->getMap() );
            if ( tile )
                _root->addChild( tile );
        }
    }
}

osg::Node*
GeodeticGraticule::buildTile( const TileKey& key, Map* map ) const
{
    if ( _levels.size() <= key.getLevelOfDetail() )
    {
        OE_WARN << LC << "Tried to create cell at non-existant level " << key.getLevelOfDetail() << std::endl;
        return 0L;
    }

    const Level& level = _levels[key.getLevelOfDetail()];

    GeoExtent tileExtent = key.getExtent();

    FeatureList latLines;
    FeatureList lonLines;

    double cellWidth = tileExtent.width() / level._cellsPerTileX;
    double cellHeight = tileExtent.height() / level._cellsPerTileY;

    for( unsigned cx = 0; cx < level._cellsPerTileX; ++cx )
    {
        double clon = tileExtent.xMin() + cellWidth * (double)cx;
        LineString* lon = new LineString(2);
        lon->push_back( osg::Vec3d(clon, tileExtent.yMin(), 0) );
        lon->push_back( osg::Vec3d(clon, tileExtent.yMax(), 0) );
        lonLines.push_back( new Feature(lon) );
    }

    for( unsigned cy = 0; cy < level._cellsPerTileY; ++cy )
    {
        double clat = tileExtent.yMin() + cellHeight * (double)cy;
        if ( clat == key.getProfile()->getExtent().yMin() )
            continue;

        LineString* lat = new LineString(2);
        lat->push_back( osg::Vec3d(tileExtent.xMin(), clat, 0) );
        lat->push_back( osg::Vec3d(tileExtent.xMax(), clat, 0) );
        latLines.push_back( new Feature(lat) );
    }

    osg::Group* group = new osg::Group();

    GeometryCompiler compiler;
    osg::ref_ptr<Session> session = new Session( map );
    FilterContext context( session.get(), _featureProfile.get(), tileExtent );

    // make sure we get sufficient tessellation:
    compiler.options().maxGranularity() = std::min(cellWidth, cellHeight) / 2.0;

    compiler.options().geoInterp() = GEOINTERP_GREAT_CIRCLE;
    osg::Node* lonNode = compiler.compile(lonLines, level._style, context);
    if ( lonNode )
        group->addChild( lonNode );

    compiler.options().geoInterp() = GEOINTERP_RHUMB_LINE;
    osg::Node* latNode = compiler.compile(latLines, level._style, context);
    if ( latNode )
        group->addChild( latNode );

    // get the geocentric tile center:
    osg::Vec3d tileCenter;
    tileExtent.getCentroid( tileCenter.x(), tileCenter.y() );
    osg::Vec3d centerECEF;
    tileExtent.getSRS()->transformToECEF( tileCenter, centerECEF );

    osg::NodeCallback* ccc = 0L;
    // set up cluster culling.
    if ( tileExtent.getSRS()->isGeographic() && tileExtent.width() < 90.0 && tileExtent.height() < 90.0 )
    {
        ccc = ClusterCullingFactory::create( group, centerECEF );
    }

    // add a paging node for higher LODs:
    if ( key.getLevelOfDetail() + 1 < _levels.size() )
    {
        const Level& nextLevel = _levels[key.getLevelOfDetail()+1];

        osg::BoundingSphere bs = group->getBound();

        std::string uri = Stringify() << key.str() << "_" << getID() << "." << GRID_MARKER << "." << GRATICULE_EXTENSION;

        osg::PagedLOD* plod = new osg::PagedLOD();
        plod->setCenter( bs.center() );
        plod->addChild( group, nextLevel._maxRange, FLT_MAX );
        plod->setFileName( 1, uri );
        plod->setRange( 1, 0, nextLevel._maxRange );
        group = plod;
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


#if 0
            GeoExtent cellExtent(
                key.getExtent().getSRS(),
                tileExtent.xMin() + cellWidth  * (double)cx,
                tileExtent.yMin() + cellHeight *ush_ (double)cy,
                tileExtent.xMin() + cellWidth  * (double)(cx + 1),
                tileExtent.yMin() + cellHeight * (double)(cy + 1));
#endif


void
GeodeticGraticule::setFirstLevel(float maxRange,
                                 unsigned tilesX,        unsigned tilesY,
                                 unsigned cellsPerTileX, unsigned cellsPerTileY )
{
    _levels.clear();
    Level level0;
    level0._maxRange = maxRange;
    level0._tilesX = std::max(1u, tilesX);
    level0._tilesY = std::max(1u, tilesY);
    level0._cellsPerTileX = std::max(1u, cellsPerTileX);
    level0._cellsPerTileY = std::max(1u, cellsPerTileY);
    _levels.push_back(level0);
}



void
GeodeticGraticule::addLevel(float maxRange,
                            unsigned cellsPerTileX, unsigned cellsPerTileY )
{
    if ( _autoLevels )
    {
        _autoLevels = false;
        _levels.clear();
    }

    Level level;
    level._maxRange = maxRange;
    level._cellsPerTileX = std::max(1u, cellsPerTileX);
    level._cellsPerTileY = std::max(1u, cellsPerTileY);
    _profile->getNumTiles( _levels.size(), level._tilesX, level._tilesY );

    level._style.getOrCreate<LineSymbol>()->stroke()->color() = 
        _levels.size() == 0 ? Color::White :
        _levels.size() == 1 ? Color::Yellow :
        _levels.size() == 2 ? Color::Lime :
        Color::Cyan;

    level._style.addSymbol( _style.get<AltitudeSymbol>() );

    _levels.push_back( level );
}


bool
GeodeticGraticule::getLevel( unsigned level, GeodeticGraticule::Level& out_level ) const
{
    if ( level < _levels.size() )
    {
        out_level = _levels[level];
        return true;
    }
    else
    {
        return false;
    }
}

void
GeodeticGraticule::traverse( osg::NodeVisitor& nv )
{
    if ( nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
    {
    }
    osg::Group::traverse( nv );
}

namespace
{
    Geometry*
    createCellGeometry( const GeoExtent& tex, double lw, const GeoExtent& profEx, bool isGeocentric )
    {            
        LineString* geom = 0L;

        if ( tex.yMin() == profEx.yMin() )
        {
            geom = new LineString(2);
            geom->push_back( osg::Vec3d( tex.xMin(), tex.yMax(), 0 ) );
            geom->push_back( osg::Vec3d( tex.xMin(), tex.yMin(), 0 ) );
        }
        else
        {
            geom = new LineString(3);
            geom->push_back( osg::Vec3d( tex.xMin(), tex.yMax(), 0 ) );
            geom->push_back( osg::Vec3d( tex.xMin(), tex.yMin(), 0 ) );
            geom->push_back( osg::Vec3d( tex.xMax(), tex.yMin(), 0 ) );
        }

        return geom;
    }

    struct HardCodeCellBoundCB : public osg::Node::ComputeBoundingSphereCallback
    {
        HardCodeCellBoundCB( const osg::BoundingSphere& bs ) : _bs(bs) { }
        virtual osg::BoundingSphere computeBound(const osg::Node&) const { return _bs; }
        osg::BoundingSphere _bs;
    };

    osg::Node*
    createTextTransform(double x, double y, double value, 
                        const osg::EllipsoidModel* ell, 
                        float size, const osg::Vec4f& color,
                        float rotation =0.0f )
    {    
        osg::Vec3d pos;
        if ( ell ) // is geocentric
        {
            ell->convertLatLongHeightToXYZ(
                osg::DegreesToRadians( y ),
                osg::DegreesToRadians( x ),
                0,
                pos.x(), pos.y(), pos.z() );
        }
        else
        {
            pos.set( x, y, 0 );
        }

        osgText::Text* t = new osgText::Text();
        t->setFont( Registry::instance()->getDefaultFont() );
        t->setAlignment( osgText::Text::CENTER_BOTTOM );
        t->setCharacterSizeMode( osgText::Text::SCREEN_COORDS );
        t->setCharacterSize( size );
        t->setBackdropType( osgText::Text::OUTLINE );
        t->setBackdropColor( osg::Vec4f(0,0,0,1) );
        t->setColor( color );

        std::stringstream buf;
        buf << std::fixed << std::setprecision(3) << value;
        std::string bufStr;
        bufStr = buf.str();
        t->setText( bufStr );

        if ( rotation != 0.0f ) 
        {
            osg::Quat rot;
            rot.makeRotate( osg::DegreesToRadians(rotation), 0, 0, 1 );
            t->setRotation( rot );
        }

        osg::Geode* geode = new osg::Geode();
        geode->addDrawable( t );

        osg::Matrixd L2W;
        ell->computeLocalToWorldTransformFromXYZ(
            pos.x(), pos.y(), pos.z(), L2W );

        osg::MatrixTransform* xform = new osg::MatrixTransform();
        xform->setMatrix( L2W );
        xform->addChild( geode );     

        // in geocentric mode, install a plane culler
        if ( ell )
            xform->setCullCallback( new CullNodeByNormal(pos) );

        return xform;
    }
}

/**************************************************************************/

namespace osgEarth { namespace Util
{
    // OSG Plugin for loading subsequent graticule levels
    class GeodeticGraticuleFactory : public osgDB::ReaderWriter
    {
    public:
        virtual const char* className()
        {
            supportsExtension( GRATICULE_EXTENSION, "osgEarth graticule" );
            return "osgEarth graticule LOD loader";
        }

        virtual bool acceptsExtension(const std::string& extension) const
        {
            return osgDB::equalCaseInsensitive(extension, GRATICULE_EXTENSION);
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
            GeodeticGraticule* graticule = 0L;
            {
                ScopedLock<Mutex> lock( s_graticuleMutex );
                GeodeticGraticuleRegistry::iterator i = s_graticuleRegistry.find(id);
                if ( i != s_graticuleRegistry.end() )
                    graticule = i->second.get();
            }

            osg::Node* result = graticule->buildChildren( levelNum, x, y );
            return result ? ReadResult(result) : ReadResult::ERROR_IN_READING_FILE;

#if 0
            if ( marker == GRID_MARKER )
            {
                osg::Node* result = graticule->createGridLevel( levelNum );
                return result ? ReadResult( result ) : ReadResult::ERROR_IN_READING_FILE;
            }
            else if ( marker == TEXT_MARKER )
            {
                osg::Node* result = graticule->createTextLevel( levelNum );
                return result ? ReadResult( result ) : ReadResult::ERROR_IN_READING_FILE;
            }
            else
            {
                OE_NOTICE << "oh no! no markers" << std::endl;
                return ReadResult::FILE_NOT_HANDLED;
            }
#endif
        }
    };
    REGISTER_OSGPLUGIN(GRATICULE_EXTENSION, GeodeticGraticuleFactory)

} } // namespace osgEarth::Util

