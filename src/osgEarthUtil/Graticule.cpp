/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2009 Pelican Ventures, Inc.
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
#include <osgEarthFeatures/StencilVolumeNode>
#include <osgEarthFeatures/BufferFilter>
#include <osgEarthFeatures/TransformFilter>
#include <osgEarthFeatures/ResampleFilter>
#include <osgEarthSymbology/StencilVolumeNode>
#include <OpenThreads/Mutex>
#include <OpenThreads/ScopedLock>
#include <osg/PagedLOD>
#include <osg/ProxyNode>
#include <osg/MatrixTransform>
#include <osg/Depth>
#include <osgDB/FileNameUtils>
#include <osgUtil/Optimizer>
#include <osgText/Text>
#include <sstream>
#include <iomanip>

using namespace osgEarth;
using namespace osgEarthUtil;
using namespace osgEarth::Features;
using namespace OpenThreads;

static Mutex s_graticuleMutex;
static unsigned int s_graticuleIdGen = 0;
typedef std::map<unsigned int, osg::ref_ptr<Graticule> > GraticuleRegistry;
static GraticuleRegistry s_graticuleRegistry;

#define GRATICLE_EXTENSION "osgearthutil_graticule"
#define TEXT_MARKER "t"
#define GRID_MARKER "g"

/**************************************************************************/

Graticule::Graticule( const Map* map ) :
_map( map ),
_autoLevels( true )
{
    // safely generate a unique ID for this graticule:
    {
        ScopedLock<Mutex> lock( s_graticuleMutex );
        _id = s_graticuleIdGen++;
        s_graticuleRegistry[_id] = this;
    }

    if ( _map->isGeocentric() )
    {
        double r = map->getProfile()->getSRS()->getEllipsoid()->getRadiusEquator();

        int x=8, y=4;
        double d = 3.5*r;
        double lw=0.15;
        addLevel( FLT_MAX, x, y, lw );
        for(int i=0; i<4; i++)
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

        StencilVolumeNode* sv = new StencilVolumeNode();
        sv->addVolumes( proxy );
        sv->addChild( osgEarth::Symbology::StencilVolumeNode::createFullScreenQuad( osg::Vec4f(1,1,1,0.5) ) ); 
        //createColorNode( osg::Vec4f(1,1,1,0.5) ) );
        this->addChild( sv );
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
        osg::StateSet* set = proxy->getOrCreateStateSet();
        set->setRenderBinDetails( 999, "RenderBin" );
        set->setAttributeAndModes( 
            new osg::Depth( osg::Depth::ALWAYS ), 
            osg::StateAttribute::ON | osg::StateAttribute::PROTECTED );
        set->setMode( GL_LIGHTING, 0 );

        this->addChild( proxy );
    }
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

static Geometry*
createCellGeometry( const GeoExtent& tex, double lw, const GeoExtent& profEx, bool isGeocentric )
{            
    Polygon* geom = 0L;

    if ( isGeocentric )
    {
        if ( tex.yMin() == profEx.yMin() )
        {
            geom = new Polygon(3);
            geom->push_back( osg::Vec3d( tex.xMin()+lw/2, tex.yMin()+lw, 0 ) );
            geom->push_back( osg::Vec3d( tex.xMin()+lw, tex.yMax(), 0 ) );
            geom->push_back( osg::Vec3d( tex.xMin(), tex.yMax(), 0 ) );
        }
        else if ( tex.yMax() == profEx.yMax() )
        {
            geom = new Polygon(5);
            geom->push_back( osg::Vec3d( tex.xMin(), tex.yMin(), 0 ) );
            geom->push_back( osg::Vec3d( tex.xMax(), tex.yMin(), 0 ) );
            geom->push_back( osg::Vec3d( tex.xMax(), tex.yMin()+lw, 0 ) );
            geom->push_back( osg::Vec3d( tex.xMin()+lw, tex.yMin()+lw, 0 ) );
            geom->push_back( osg::Vec3d( tex.xMin()+lw/2, tex.yMax()-lw, 0 ) );            
        }
        else
        {
            geom = new Polygon(6);
            geom->push_back( osg::Vec3d( tex.xMin(), tex.yMin(), 0 ) );
            geom->push_back( osg::Vec3d( tex.xMax(), tex.yMin(), 0 ) );
            geom->push_back( osg::Vec3d( tex.xMax(), tex.yMin()+lw, 0 ) );
            geom->push_back( osg::Vec3d( tex.xMin()+lw, tex.yMin()+lw, 0 ) );
            geom->push_back( osg::Vec3d( tex.xMin()+lw, tex.yMax(), 0 ) );
            geom->push_back( osg::Vec3d( tex.xMin(), tex.yMax(), 0 ) );
        }
    }
    else
    {
        if ( tex.yMin() == profEx.yMin() )
        {
            geom = new Polygon(4);
            geom->push_back( osg::Vec3d( tex.xMin(), tex.yMin(), 0 ) );
            geom->push_back( osg::Vec3d( tex.xMin()+lw, tex.yMin(), 0 ) );
            geom->push_back( osg::Vec3d( tex.xMin()+lw, tex.yMax(), 0 ) );
            geom->push_back( osg::Vec3d( tex.xMin(), tex.yMax(), 0 ) );
        }
        else
        {
            geom = new Polygon(6);
            geom->push_back( osg::Vec3d( tex.xMin(), tex.yMin(), 0 ) );
            geom->push_back( osg::Vec3d( tex.xMax(), tex.yMin(), 0 ) );
            geom->push_back( osg::Vec3d( tex.xMax(), tex.yMin()+lw, 0 ) );
            geom->push_back( osg::Vec3d( tex.xMin()+lw, tex.yMin()+lw, 0 ) );
            geom->push_back( osg::Vec3d( tex.xMin()+lw, tex.yMax(), 0 ) );
            geom->push_back( osg::Vec3d( tex.xMin(), tex.yMax(), 0 ) );
        }
    }

    return geom;
}

struct HardCodeCellBoundCB : public osg::Node::ComputeBoundingSphereCallback {
    HardCodeCellBoundCB( const osg::BoundingSphere& bs ) : _bs(bs) { }
    virtual osg::BoundingSphere computeBound(const osg::Node&) const { return _bs; }
    osg::BoundingSphere _bs;
};

struct CullPlaneCallback : public osg::NodeCallback
{
    osg::Vec3d _n;

    CullPlaneCallback( const osg::Vec3d& planeNormal ) : _n(planeNormal) {
        _n.normalize();
    }

    void operator()(osg::Node* node, osg::NodeVisitor* nv) {
        if ( !nv || nv->getEyePoint() * _n > 0 )
            traverse(node,nv); 
    }
};

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

    OE_INFO << "Graticule: creating grid level " << levelNum << std::endl;

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

            double ox = level._lineWidth;
            double oy = level._lineWidth;

            Geometry* geom = createCellGeometry( tex, level._lineWidth, pex, _map->isGeocentric() );

            Feature* feature = new Feature();
            feature->setGeometry( geom );
            FeatureList features;
            features.push_back( feature );

            FilterContext cx;
            cx.profile() = new FeatureProfile( tex );
            cx.isGeocentric() = _map->isGeocentric();

            if ( _map->isGeocentric() )
            {
                // We need to make sure that on a round globe, the points are sampled such that
                // long segments follow the curvature of the earth.
                ResampleFilter resample;
                resample.maxLength() = tex.width() / 10.0;
                resample.perturbationThreshold() = level._lineWidth/1000.0;
                cx = resample.push( features, cx );
            }

            TransformFilter xform( mapProfile->getSRS(), _map->isGeocentric() );
            cx = xform.push( features, cx );

            Bounds bounds = feature->getGeometry()->getBounds();
            double exDist = bounds.radius()/2.0;

            osg::Node* cellVolume = StencilVolumeFactory::createVolume(
                feature->getGeometry(),
                -exDist,
                exDist*2,
                cx );

            osg::Node* child = cellVolume;

            if ( cx.hasReferenceFrame() )
            {
                osg::MatrixTransform* xform = new osg::MatrixTransform( cx.inverseReferenceFrame() );
                xform->addChild( child );

                // the transform matrix here does NOT include a rotation, so we need to get the normal
                // for the cull plane callback.
                osg::Vec3d normal = xform->getBound().center();
                xform->setCullCallback( new CullPlaneCallback( normal ) );

                child = xform;
            }

            group->addChild( child );
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
            buf << levelNum+1 << "_" << getID() << "." << GRID_MARKER << "." << GRATICLE_EXTENSION;
            std::string bufStr = buf.str();
            plod->setFileName( 1, bufStr );
            plod->setRange( 1, 0, nextLevel._maxRange );
            result = plod;
        }
    }

    return result;
}

static osg::Node*
createTextTransform( double x, double y, double value, const osg::EllipsoidModel* ell, float size, float rotation =0.0f )
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
    t->setColor( osg::Vec4f(1,1,1,1) );

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

    // Note: the Transform already includes the rotation, so all we need to cluster cull
    // is the local up vector.
    xform->setCullCallback( new CullPlaneCallback( osg::Vec3d(0,0,1) ) );

    return xform;
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

    OE_INFO << "Graticule: creating text level " << levelNum << std::endl;

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
                20.0f ) );

            // x value on the y-axis:
            group->addChild( createTextTransform(
                tex.xMin() + offset,
                cy,
                tex.xMin(),
                ell, 
                20.0f,
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

namespace osgEarthUtil
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
}

