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
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#include <osgEarthAnnotation/GeoPositionNode>
#include <osgEarthAnnotation/AnnotationUtils>
#include <osgEarthAnnotation/AnnotationSettings>
#include <osgEarthSymbology/Color>
#include <osgEarth/ThreadingUtils>
#include <osgEarth/CullingUtils>
#include <osgEarth/MapNode>
#include <osgEarth/TerrainEngineNode>

#include <osgText/Text>
#include <osg/ComputeBoundsVisitor>
#include <osgUtil/IntersectionVisitor>
#include <osg/OcclusionQueryNode>
#include <osg/Point>
#include <osg/Depth>
#include <osg/Switch>

#define LC "[GeoPositionNode] "

using namespace osgEarth;
using namespace osgEarth::Annotation;

#define DEFAULT_OCCLUSION_CULLING false
#define DEFAULT_HORIZON_CULLING true

GeoPositionNode::GeoPositionNode() :
AnnotationNode(),
_geoxform(0L),
_paxform(0L),
_occlusionCullingRequested( DEFAULT_OCCLUSION_CULLING ),
_horizonCullingRequested( DEFAULT_HORIZON_CULLING )
{
    init();
}

GeoPositionNode::GeoPositionNode(MapNode* mapNode) :
AnnotationNode(),
_geoxform(0L),
_paxform(0L),
_occlusionCullingRequested( DEFAULT_OCCLUSION_CULLING ),
_horizonCullingRequested( DEFAULT_HORIZON_CULLING )
{
    init();
    GeoPositionNode::setMapNode( mapNode );
}

GeoPositionNode::GeoPositionNode(MapNode* mapNode, const GeoPoint& position ) :
AnnotationNode(),
_geoxform(0L),
_paxform(0L),
_occlusionCullingRequested( DEFAULT_OCCLUSION_CULLING ),
_horizonCullingRequested( DEFAULT_HORIZON_CULLING )
{
    init();
    GeoPositionNode::setMapNode( mapNode );
    _geoxform->setPosition( position );
}

GeoPositionNode::GeoPositionNode(const GeoPositionNode& rhs, const osg::CopyOp& copy) :
AnnotationNode(rhs, copy),
_geoxform(0L),
_paxform(0L),
_occlusionCullingRequested( DEFAULT_OCCLUSION_CULLING ),
_horizonCullingRequested( DEFAULT_HORIZON_CULLING )
{
    //nop - UNUSED
}

void
GeoPositionNode::init()
{    
    _geoxform = new GeoTransform();
    _geoxform->setAutoRecomputeHeights( true );
    this->addChild( _geoxform );

    _paxform = new osg::PositionAttitudeTransform();
    _geoxform->addChild( _paxform );
    
    this->getOrCreateStateSet()->setMode( GL_LIGHTING, 0 );
}

void
GeoPositionNode::setMapNode( MapNode* mapNode )
{
    MapNode* oldMapNode = getMapNode();
    if ( oldMapNode != mapNode )
    {
        AnnotationNode::setMapNode( mapNode );

        bool occlusionCullingRequested = _occlusionCullingRequested;
        // the occlusion culler depends on the mapnode, so re-initialize it:
        setOcclusionCulling( false );
        if ( occlusionCullingRequested )
        {
            setOcclusionCulling( true );
        }

        if ( mapNode )
            _geoxform->setTerrain( getMapNode()->getTerrain() );
        else
            _geoxform->setTerrain( 0L );
    }
}

void
GeoPositionNode::applyStyle(const Style& style)
{
    const TextSymbol* text = style.get<TextSymbol>();

    // check for decluttering.
    if ( text && text->declutter() == false )
    {
        setPriority( FLT_MAX );
    }


    // check for occlusion culling
    if ( text && text->occlusionCull().isSet() )
    {
        setOcclusionCulling( *text->occlusionCull() );

        if (text->occlusionCullAltitude().isSet())
        {
            setOcclusionCullingMaxAltitude( *text->occlusionCullAltitude() );
        }
    }

    const IconSymbol* icon = style.get<IconSymbol>();

    if ( icon && icon->declutter() == false )
    {
        setPriority( FLT_MAX );
    }

    // check for occlusion culling
    if ( icon && icon->occlusionCull().isSet() )
    {
        this->setOcclusionCulling( *icon->occlusionCull() );

        if (icon->occlusionCullAltitude().isSet())
        {
            setOcclusionCullingMaxAltitude( *icon->occlusionCullAltitude() );
        }
    }

    // up the chain
    AnnotationNode::applyStyle( style );
}

bool
GeoPositionNode::getOcclusionCulling() const
{
    return _occlusionCullingRequested;
}

void
GeoPositionNode::setOcclusionCulling( bool value )
{
    if (_occlusionCullingRequested != value)
    {
        _occlusionCullingRequested = value;

        if ( _occlusionCullingRequested )
        {
            _occlusionCuller = new OcclusionCullingCallback( _geoxform );
            _occlusionCuller->setMaxAltitude( getOcclusionCullingMaxAltitude() );
            addCullCallback( _occlusionCuller.get()  );
        }
        else if (_occlusionCuller.valid())
        {
            removeCullCallback( _occlusionCuller.get() );
            _occlusionCuller = 0;
        }
    }
}

double
GeoPositionNode::getOcclusionCullingMaxAltitude() const
{
    if (_occlusionCullingMaxAltitude.isSet())
    {
        return *_occlusionCullingMaxAltitude;
    }
    return AnnotationSettings::getOcclusionCullingMaxAltitude();
}

void GeoPositionNode::setOcclusionCullingMaxAltitude( double occlusionCullingMaxAltitude )
{
    _occlusionCullingMaxAltitude = occlusionCullingMaxAltitude;
    if ( _occlusionCuller.valid() )
    {
        _occlusionCuller->setMaxAltitude( getOcclusionCullingMaxAltitude() );         
    }
}


GeoPositionNode::GeoPositionNode(MapNode* mapNode, const Config& conf) :
AnnotationNode          ( conf ),
_horizonCullingRequested( true )
{
    init();
    GeoPositionNode::setMapNode( mapNode );
    setConfig(conf);
}

void
GeoPositionNode::setConfig(const Config& conf)
{
    //AnnotationNode::setConfig(conf);

    if ( conf.hasChild( "position" ) )
    {
        setPosition( GeoPoint(conf.child("position")) );
    }
    else
    {
        if (conf.hasValue("lat") && conf.hasValue("long"))
        {
            setPosition( GeoPoint(SpatialReference::get("wgs84"),
                conf.value("long", 0.0), conf.value("lat", 0.0)) );
        }
    }

    if ( conf.hasChild( "scale" ) )
    {
        const Config* c = conf.child_ptr("scale");
        osg::Vec3f s( c->value("x", 1.0f), c->value("y", 1.0f), c->value("z", 1.0f) );
        getPositionAttitudeTransform()->setScale( s );
    }

    if ( conf.hasChild( "local_offset" ) )
    {
        const Config* c = conf.child_ptr("local_offset");
        osg::Vec3d o( c->value("x", 0.0), c->value("y", 0.0), c->value("z", 0.0) );
        getPositionAttitudeTransform()->setPosition( o );
    }

    if ( conf.hasChild( "local_rotation" ) )
    {
        const Config* c = conf.child_ptr("local_rotation");
        osg::Quat q( c->value("x", 0.0), c->value("y", 0.0), c->value("z", 0.0), c->value("w", 1.0) );
        getPositionAttitudeTransform()->setAttitude( q );
    }
}

Config
GeoPositionNode::getConfig() const
{
    Config conf = AnnotationNode::getConfig();

    conf.addObj( "position", getGeoTransform()->getPosition() );

    const osg::Vec3d& scale = getPositionAttitudeTransform()->getScale();
    if ( scale.x() != 1.0f || scale.y() != 1.0f || scale.z() != 1.0f )
    {
        Config c( "scale" );
        c.add( "x", scale.x() );
        c.add( "y", scale.y() );
        c.add( "z", scale.z() );
        conf.add( c );
    }

    const osg::Vec3d& offset = getPositionAttitudeTransform()->getPosition();
    if ( offset != osg::Vec3d(0,0,0) )
    {
        Config c( "local_offset" );
        c.set( "x", offset.x() );
        c.set( "y", offset.y() );
        c.set( "z", offset.z() );
        conf.add( c );
    }

    const osg::Quat& rot = getPositionAttitudeTransform()->getAttitude();
    if ( !rot.zeroRotation() )
    {
        Config c( "local_rotation" );
        c.set( "x", rot.x() );
        c.set( "y", rot.y() );
        c.set( "z", rot.z() );
        c.set( "w", rot.w() );
        conf.add( c );
    }

    return conf;
}
