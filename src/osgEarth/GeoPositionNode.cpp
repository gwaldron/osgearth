/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/

#include <osgEarth/GeoPositionNode>
#include <osgEarth/AnnotationSettings>
#include <osgEarth/Color>
#include <osgEarth/CullingUtils>
#include <osgEarth/MapNode>

#define LC "[GeoPositionNode] "

using namespace osgEarth;

#define DEFAULT_OCCLUSION_CULLING false
#define DEFAULT_HORIZON_CULLING true

namespace
{
    template<class T> bool hasCallback(osg::Node* node) {
        if (!node) return false;
        for (auto* cb = node->getCullCallback(); cb != nullptr; cb = cb->getNestedCallback()) {
            if (dynamic_cast<T*>(cb)) return true;
        }
        return false;
    }
}

GeoPositionNode::GeoPositionNode() :
AnnotationNode()
{
    construct();
}

GeoPositionNode::GeoPositionNode(const Config& conf, const osgDB::Options* options) :
AnnotationNode( conf, options )
{
    construct();
    setConfig(conf);
}

void
GeoPositionNode::construct()
{
    _geoxform = 0L;
    _paxform = 0L;

    _occlusionCullingRequested = DEFAULT_OCCLUSION_CULLING;
    _horizonCullingRequested = DEFAULT_HORIZON_CULLING;

    this->removeChildren(0, this->getNumChildren());

    _geoxform = new GeoTransform();
    this->addChild( _geoxform );

    _paxform = new osg::PositionAttitudeTransform();
    _geoxform->addChild( _paxform );
}

void
GeoPositionNode::setMapNode( MapNode* mapNode )
{
    if (mapNode != getMapNode())
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

void
GeoPositionNode::setPosition(const GeoPoint& pos)
{
    _geoxform->setPosition(pos);
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
            if (_occlusionCuller.valid())
            {
                removeCullCallback( _occlusionCuller.get() );
            }

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

void
GeoPositionNode::setConfig(const Config& conf)
{
    //AnnotationNode::setConfig(conf);

    if (conf.hasValue("name"))
    {
        setName(conf.value("name"));
    }

    if ( conf.hasChild( "position" ) )
    {
        setPosition( GeoPoint(conf.child("position")) );
    }
    else if (conf.hasChild("location"))
    {
        setPosition(GeoPoint(conf.child("location")));
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

    conf.set( "position", getGeoTransform()->getPosition() );

    const osg::Vec3d& scale = getPositionAttitudeTransform()->getScale();
    if ( scale.x() != 1.0f || scale.y() != 1.0f || scale.z() != 1.0f )
    {
        Config c( "scale" );
        c.set( "x", scale.x() );
        c.set( "y", scale.y() );
        c.set( "z", scale.z() );
        conf.set( c );
    }

    const osg::Vec3d& offset = getPositionAttitudeTransform()->getPosition();
    if ( offset != osg::Vec3d(0,0,0) )
    {
        Config c( "local_offset" );
        c.set( "x", offset.x() );
        c.set( "y", offset.y() );
        c.set( "z", offset.z() );
        conf.set( c );
    }

    const osg::Quat& rot = getPositionAttitudeTransform()->getAttitude();
    if ( !rot.zeroRotation() )
    {
        Config c( "local_rotation" );
        c.set( "x", rot.x() );
        c.set( "y", rot.y() );
        c.set( "z", rot.z() );
        c.set( "w", rot.w() );
        conf.set( c );
    }

    return conf;
}
