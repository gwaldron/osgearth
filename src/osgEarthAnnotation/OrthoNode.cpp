/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2015 Pelican Mapping
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

#include <osgEarthAnnotation/OrthoNode>
#include <osgEarthAnnotation/AnnotationUtils>
#include <osgEarthAnnotation/AnnotationSettings>
#include <osgEarthSymbology/Color>
#include <osgEarth/ThreadingUtils>
#include <osgEarth/CullingUtils>
#include <osgEarth/MapNode>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/Decluttering>

#include <osgText/Text>
#include <osg/ComputeBoundsVisitor>
#include <osgUtil/IntersectionVisitor>
#include <osg/OcclusionQueryNode>
#include <osg/Point>
#include <osg/Depth>
#include <osg/Switch>

#define LC "[OrthoNode] "

using namespace osgEarth;
using namespace osgEarth::Annotation;

#define DEFAULT_HORIZON_CULLING   true
#define DEFAULT_OCCLUSION_CULLING false


OrthoNode::OrthoNode(MapNode* mapNode, const GeoPoint& position ) :
AnnotationNode(),
_occlusionCullingRequested( DEFAULT_OCCLUSION_CULLING ),
_horizonCullingRequested  ( DEFAULT_HORIZON_CULLING )
{
    init();
    OrthoNode::setMapNode( mapNode );
    _geoxform->setPosition( position );
}


OrthoNode::OrthoNode() :
AnnotationNode(),
_occlusionCullingRequested( DEFAULT_OCCLUSION_CULLING ),
_horizonCullingRequested  ( DEFAULT_HORIZON_CULLING )
{
    init();
}


void
OrthoNode::init()
{    
    _geoxform = new GeoTransform();
    _geoxform->setAutoRecomputeHeights( true );
    this->addChild( _geoxform );

    _paxform = new osg::PositionAttitudeTransform();
    _geoxform->addChild( _paxform );

    // Callback to cull ortho nodes that are not visible over the geocentric horizon
    _horizonCuller = new HorizonCullCallback();
    if ( getMapNode() )
        _horizonCuller->setHorizon( new Horizon(getMapNode()->getMapSRS()) );

    setHorizonCulling( _horizonCullingRequested );
    _geoxform->addCullCallback( _horizonCuller.get() );
    
    this->getOrCreateStateSet()->setMode( GL_LIGHTING, 0 );
}

void
OrthoNode::setMapNode( MapNode* mapNode )
{
    MapNode* oldMapNode = getMapNode();
    if ( oldMapNode != mapNode )
    {
        AnnotationNode::setMapNode( mapNode );

        // the occlusion culler depends on the mapnode, so re-initialize it:
        setOcclusionCulling( false );
        if ( _occlusionCullingRequested )
        {
            setOcclusionCulling( true );
        }

        // same goes for the horizon culler:
        if ( getMapNode() )
        {
            _horizonCuller->setHorizon( new Horizon(*getMapNode()->getMapSRS()->getEllipsoid()) );
            setHorizonCulling( _horizonCullingRequested );

            _geoxform->setTerrain( getMapNode()->getTerrain() );
        }
        else
        {
            _geoxform->setTerrain( 0L );
        }
    }
}

void
OrthoNode::applyStyle(const Style& style)
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
OrthoNode::getHorizonCulling() const
{
    return _horizonCullingRequested;
}

void
OrthoNode::setHorizonCulling(bool value)
{
    _horizonCullingRequested = value;

    _horizonCuller->setEnabled(
        _horizonCullingRequested &&
        getMapNode() &&
        getMapNode()->isGeocentric() );
}

bool
OrthoNode::getOcclusionCulling() const
{
    return _occlusionCullingRequested;
}

void
OrthoNode::setOcclusionCulling( bool value )
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
OrthoNode::getOcclusionCullingMaxAltitude() const
{
    if (_occlusionCullingMaxAltitude.isSet())
    {
        return *_occlusionCullingMaxAltitude;
    }
    return AnnotationSettings::getOcclusionCullingMaxAltitude();
}

void OrthoNode::setOcclusionCullingMaxAltitude( double occlusionCullingMaxAltitude )
{
    _occlusionCullingMaxAltitude = occlusionCullingMaxAltitude;
    if ( _occlusionCuller.valid() )
    {
        _occlusionCuller->setMaxAltitude( getOcclusionCullingMaxAltitude() );         
    }
}



OrthoNode::OrthoNode(MapNode* mapNode, const Config& conf) :
AnnotationNode          ( conf ),
_horizonCullingRequested( true )
{
    init();
    OrthoNode::setMapNode( mapNode );

    if ( conf.hasChild( "position" ) )
    {
        setPosition( GeoPoint(conf.child("position")) );
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

    bool hc = conf.value("horizon_culling", DEFAULT_HORIZON_CULLING);
    if ( hc != getHorizonCulling() )
    {
        setHorizonCulling( hc );
    }
}

Config
OrthoNode::getConfig() const
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

    if ( getHorizonCulling() != DEFAULT_HORIZON_CULLING )
    {
        conf.add( "horizon_culling", getHorizonCulling() );
    }

    return conf;
}
