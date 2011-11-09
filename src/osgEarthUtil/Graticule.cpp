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
#include <osgEarthUtil/Graticule>
#include <osgEarthUtil/AutoClipPlaneHandler>
#include <osgEarthFeatures/BuildGeometryFilter>
#include <osgEarthFeatures/TransformFilter>
#include <osgEarthFeatures/ResampleFilter>
#include <osgEarthSymbology/Geometry>
#include <osgEarth/Registry>
#include <osgEarth/FindNode>
#include <osgEarth/Utils>
#include <OpenThreads/Mutex>
#include <OpenThreads/ScopedLock>
#include <osg/PagedLOD>
#include <osg/ProxyNode>
#include <osg/MatrixTransform>
#include <osg/Depth>
#include <osg/Program>
#include <osg/LineStipple>
#include <osg/ClusterCullingCallback>
#include <osgDB/FileNameUtils>
#include <osgUtil/Optimizer>
#include <osgText/Text>
#include <sstream>
#include <iomanip>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;
using namespace OpenThreads;

static Mutex s_graticuleMutex;
typedef std::map<unsigned int, osg::ref_ptr<Graticule> > GraticuleRegistry;
static GraticuleRegistry s_graticuleRegistry;

#define GRATICLE_EXTENSION "osgearthutil_graticule"
#define TEXT_MARKER "t"
#define GRID_MARKER "g"

#ifdef __GNUC__
#define LIKELY_UNUSED_VARIABLE __attribute__ ((unused))
#else
#define LIKELY_UNUSED_VARIABLE
#endif

//---------------------------------------------------------------------------

namespace
{
    LIKELY_UNUSED_VARIABLE char s_vertexShader[] =
        "varying vec3 Normal; \n"
        "void main(void) \n"
        "{ \n"
        "    Normal = normalize( gl_NormalMatrix * gl_Normal ); \n"
        "    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex; \n"
        "    gl_FrontColor = gl_Color; \n"
        "} \n";

    LIKELY_UNUSED_VARIABLE char s_fragmentShader[] =
        "varying vec3 Normal; \n"
        "void main(void) \n"
        "{ \n"
        "    gl_FragColor = gl_Color; \n"
        "} \n";
}

//---------------------------------------------------------------------------

Graticule::Graticule( const Map* map ) :
_autoLevels( true ),
_map( map ),
_textColor( 1,1,0,1 )
{
    // safely generate a unique ID for this graticule:
    _id = Registry::instance()->createUID();
    {
        ScopedLock<Mutex> lock( s_graticuleMutex );
        s_graticuleRegistry[_id] = this;
    }

    setLineColor( osg::Vec4f(1,1,1,0.7) );
    setTextColor( osg::Vec4f(1,1,0,1) );

    if ( _map->isGeocentric() )
    {
        double r = map->getProfile()->getSRS()->getEllipsoid()->getRadiusEquator();

        int x=8, y=4;
        double d = 3.5*r;
        double lw=0.15;
        addLevel( FLT_MAX, x, y, lw );
        for(int i=0; i<9; i++)
        {
            x *= 2, y *= 2;
            lw *= 0.5;
            d *= 0.5;
            addLevel( r+d, x, y, lw );
        }
    }

    // Prime the grid:
    {
        std::stringstream buf;
        buf << "0_" << _id << "." << GRID_MARKER << "." << GRATICLE_EXTENSION;
        std::string bufStr = buf.str();
        osg::ProxyNode* proxy = new osg::ProxyNode();
        proxy->setFileName( 0, bufStr );
        proxy->setCenterMode( osg::ProxyNode::USER_DEFINED_CENTER );
        proxy->setCenter( osg::Vec3(0,0,0) );
        proxy->setRadius( 1e10 );
        this->addChild( proxy );
    }

    // Prime the text:
    {
        std::stringstream buf;
        buf << "0_" << _id << "." << TEXT_MARKER << "." << GRATICLE_EXTENSION;
        std::string bufStr = buf.str();

        osg::ProxyNode* proxy = new osg::ProxyNode();
        proxy->setFileName( 0, bufStr );
        proxy->setCenterMode( osg::ProxyNode::USER_DEFINED_CENTER );
        proxy->setCenter( osg::Vec3(0,0,0) );
        proxy->setRadius( 1e10 );

        this->addChild( proxy );
    }

    osg::StateSet* set = this->getOrCreateStateSet();
    set->setRenderBinDetails( 9999, "RenderBin" );
    set->setAttributeAndModes( 
        new osg::Depth( osg::Depth::ALWAYS ), 
        osg::StateAttribute::ON | osg::StateAttribute::PROTECTED );
    set->setMode( GL_LIGHTING, 0 );

    //osg::Program* program = new osg::Program();
    //program->addShader( new osg::Shader( osg::Shader::VERTEX, s_vertexShader ) );
    //program->addShader( new osg::Shader( osg::Shader::FRAGMENT, s_fragmentShader ) );
    //set->setAttributeAndModes( program, osg::StateAttribute::ON );

    this->addEventCallback( new AutoClipPlaneCallback( _map.get() ) );
}

void
Graticule::addLevel( float maxRange, unsigned int cellsX, unsigned int cellsY, double lineWidth )
{
    if ( _autoLevels )
    {
        _autoLevels = false;
        _levels.clear();
    }

    Level level;
    level._maxRange = maxRange;
    level._cellsX = cellsX;
    level._cellsY = cellsY;
    level._lineWidth = lineWidth;

    for( std::vector<Level>::iterator i = _levels.begin(); i != _levels.end(); ++i ) 
    {
        if ( maxRange > i->_maxRange )
        {
            _levels.insert( i, level );
            return;
        }
    }
    _levels.push_back( level );
}

bool
Graticule::getLevel( unsigned int level, Graticule::Level& out_level ) const
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
        t->setFont( "fonts/arial.ttf" );
        t->setAlignment( osgText::Text::CENTER_BOTTOM );
        t->setCharacterSizeMode( osgText::Text::SCREEN_COORDS );
        t->setCharacterSize( size );
        t->setBackdropType( osgText::Text::OUTLINE );
        t->setBackdropColor( osg::Vec4f(0,0,0,1) );
        t->setColor( color );

        std::stringstream buf;
        buf << std::fixed << std::setprecision(3) << value;
        std::string bufStr = buf.str();
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

void
Graticule::setLineColor( const osg::Vec4f& color )
{
    _lineStyle.getOrCreateSymbol<LineSymbol>()->stroke()->color() = color;
}

osg::Node*
Graticule::createGridLevel( unsigned int levelNum ) const
{
    if ( !_map->isGeocentric() )
    {
        OE_WARN << "Graticule: only supports geocentric maps" << std::endl;
        return 0L;
    }

    Graticule::Level level;
    if ( !getLevel( levelNum, level ) )
        return 0L;

    OE_DEBUG << "Graticule: creating grid level " << levelNum << std::endl;

    osg::Group* group = new osg::Group();

    const Profile* mapProfile = _map->getProfile();
    const GeoExtent& pex = mapProfile->getExtent();

    double tw = pex.width() / (double)level._cellsX;
    double th = pex.height() / (double)level._cellsY;

    for( unsigned int x=0; x<level._cellsX; ++x )
    {
        for( unsigned int y=0; y<level._cellsY; ++y )
        {
            GeoExtent tex(
                mapProfile->getSRS(),
                pex.xMin() + tw * (double)x,
                pex.yMin() + th * (double)y,
                pex.xMin() + tw * (double)(x+1),
                pex.yMin() + th * (double)(y+1) );

            Geometry* geom = createCellGeometry( tex, level._lineWidth, pex, _map->isGeocentric() );

            Feature* feature = new Feature();
            feature->setGeometry( geom );
            FeatureList features;
            features.push_back( feature );

            FilterContext cx;
            cx.profile() = new FeatureProfile( tex );
            //cx.isGeocentric() = _map->isGeocentric();

            if ( _map->isGeocentric() )
            {
                // We need to make sure that on a round globe, the points are sampled such that
                // long segments follow the curvature of the earth.
                ResampleFilter resample;
                resample.maxLength() = tex.width() / 10.0;
                cx = resample.push( features, cx );
            }

            TransformFilter xform( mapProfile->getSRS() );
            //xform.setMakeGeocentric( _map->isGeocentric() );
            //xform.setLocalizeCoordinates( true );
            cx = xform.push( features, cx );

            osg::ref_ptr<osg::Node> output;
            BuildGeometryFilter bg;
            bg.setStyle( _lineStyle );
            //cx = bg.push( features, cx );
            output = bg.push( features, cx ); //.getNode();

            if ( _map->isGeocentric() )
            {
                // get the geocentric control point:
                double cplon, cplat, cpx, cpy, cpz;
                tex.getCentroid( cplon, cplat );
                tex.getSRS()->getEllipsoid()->convertLatLongHeightToXYZ(
                    osg::DegreesToRadians( cplat ), osg::DegreesToRadians( cplon ), 0.0, cpx, cpy, cpz );
                osg::Vec3 controlPoint(cpx, cpy, cpz);

                // get the horizon point:
                tex.getSRS()->getEllipsoid()->convertLatLongHeightToXYZ(
                    osg::DegreesToRadians( tex.yMin() ), osg::DegreesToRadians( tex.xMin() ), 0.0,
                    cpx, cpy, cpz );
                osg::Vec3 horizonPoint(cpx, cpy, cpz);

                // the deviation is the dot product of the control vector and the vector from the
                // control point to the horizon point.
                osg::Vec3 controlPointNorm = controlPoint; controlPointNorm.normalize();
                osg::Vec3 horizonVecNorm = horizonPoint - controlPoint; horizonVecNorm.normalize();                
                float deviation = controlPointNorm * horizonVecNorm;

                // construct the culling callback using the deviation.
                osg::ClusterCullingCallback* ccc = new osg::ClusterCullingCallback();
                ccc->set( controlPoint, controlPointNorm, deviation, (controlPoint-horizonPoint).length() );

                // need a new group, because never put a cluster culler on a matrixtransform (doesn't work)
                osg::Group* me = new osg::Group();
                me->setCullCallback( ccc );
                me->addChild( output.get() );
                output = me;
            }

            group->addChild( output.get() );
        }
    }

    // organize it for better culling
    osgUtil::Optimizer opt;
    opt.optimize( group, osgUtil::Optimizer::SPATIALIZE_GROUPS );

    osg::Node* result = group;

    if ( levelNum < getNumLevels() )
    {
        Graticule::Level nextLevel;
        if ( getLevel( levelNum+1, nextLevel ) )
        {
            osg::PagedLOD* plod = new osg::PagedLOD();
            plod->addChild( group, nextLevel._maxRange, level._maxRange );
            std::stringstream buf;
            buf << levelNum+1 << "_" << getID() << "." << GRID_MARKER << "." << GRATICLE_EXTENSION;
            std::string bufStr = buf.str();
            plod->setFileName( 1, bufStr );
            plod->setRange( 1, 0, nextLevel._maxRange );
            result = plod;
        }
    }

    return result;
}

osg::Node*
Graticule::createTextLevel( unsigned int levelNum ) const
{
    if ( !_map->isGeocentric() )
    {
        OE_WARN << "Graticule: only supports geocentric maps" << std::endl;
        return 0L;
    }

    Graticule::Level level;
    if ( !getLevel( levelNum, level ) )
        return 0L;

    OE_DEBUG << "Graticule: creating text level " << levelNum << std::endl;

    osg::Group* group = new osg::Group();

    const Profile* mapProfile = _map->getProfile();
    const GeoExtent& pex = mapProfile->getExtent();

    double tw = pex.width() / (double)level._cellsX;
    double th = pex.height() / (double)level._cellsY;

    const osg::EllipsoidModel* ell = _map->getProfile()->getSRS()->getEllipsoid();

    for( unsigned int x=0; x<level._cellsX; ++x )
    {
        for( unsigned int y=0; y<level._cellsY; ++y )
        {
            GeoExtent tex(
                mapProfile->getSRS(),
                pex.xMin() + tw * (double)x,
                pex.yMin() + th * (double)y,
                pex.xMin() + tw * (double)(x+1),
                pex.yMin() + th * (double)(y+1) );

            double offset = 2.0 * level._lineWidth;

            double cx, cy;
            tex.getCentroid( cx, cy );

            // y value on the x-axis:
            group->addChild( createTextTransform(
                cx,
                tex.yMin() + offset,
                tex.yMin(),
                ell,
                20.0f,
                _textColor ) );

            // x value on the y-axis:
            group->addChild( createTextTransform(
                tex.xMin() + offset,
                cy,
                tex.xMin(),
                ell, 
                20.0f,
                _textColor,
                -90.0f ) );
        }
    }

    // organize it for better culling
    osgUtil::Optimizer opt;
    opt.optimize( group, osgUtil::Optimizer::SPATIALIZE_GROUPS );

    osg::Node* result = group;

    if ( levelNum+1 < getNumLevels() )
    {
        Graticule::Level nextLevel;
        if ( getLevel( levelNum+1, nextLevel ) )
        {
            osg::PagedLOD* plod = new osg::PagedLOD();
            plod->addChild( group, nextLevel._maxRange, level._maxRange );
            std::stringstream buf;
            buf << levelNum+1 << "_" << getID() << "." << TEXT_MARKER << "." << GRATICLE_EXTENSION;
            std::string bufStr = buf.str();
            plod->setFileName( 1, bufStr );
            plod->setRange( 1, 0, nextLevel._maxRange );
            result = plod;
        }
    }
    return result;
}

/**************************************************************************/

namespace osgEarth { namespace Util
{
    // OSG Plugin for loading subsequent graticule levels
    class GraticuleFactory : public osgDB::ReaderWriter
    {
    public:
        virtual const char* className()
        {
            supportsExtension( GRATICLE_EXTENSION, "osgEarth graticule" );
            return "osgEarth graticule LOD loader";
        }

        virtual bool acceptsExtension(const std::string& extension) const
        {
            return osgDB::equalCaseInsensitive(extension, GRATICLE_EXTENSION);
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

            int levelNum, id;
            sscanf( def.c_str(), "%d_%d", &levelNum, &id );

            // look up the graticule referenced in the location name:
            Graticule* graticule = 0L;
            {
                ScopedLock<Mutex> lock( s_graticuleMutex );
                GraticuleRegistry::iterator i = s_graticuleRegistry.find(id);
                if ( i != s_graticuleRegistry.end() )
                    graticule = i->second.get();
            }

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
        }
    };
    REGISTER_OSGPLUGIN(osgearthutil_graticule, GraticuleFactory)

} } // namespace osgEarth::Util

