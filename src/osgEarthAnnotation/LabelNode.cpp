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

#include <osgEarthAnnotation/LabelNode>
#include <osgEarthAnnotation/AnnotationUtils>
#include <osgEarthAnnotation/AnnotationRegistry>
#include <osgEarthSymbology/Color>
#include <osgEarth/Registry>
#include <osgEarth/ShaderGenerator>
#include <osgEarth/GeoMath>
#include <osgText/Text>
#include <osg/Depth>
#include <osgUtil/IntersectionVisitor>
#include <osgUtil/LineSegmentIntersector>

#define LC "[LabelNode] "

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Symbology;


//-------------------------------------------------------------------

LabelNode::LabelNode(MapNode*            mapNode,
                     const GeoPoint&     position,
                     const std::string&  text,
                     const Style&        style ) :

GeoPositionNode( mapNode, position ),
_text             ( text ),
_labelRotationRad ( 0. ),
_followFixedCourse( false )
{
    init( style );
}

LabelNode::LabelNode(MapNode*            mapNode,
                     const GeoPoint&     position,
                     const std::string&  text,
                     const TextSymbol*   symbol ) :

GeoPositionNode( mapNode, position ),
_text             ( text ),
_labelRotationRad ( 0. ),
_followFixedCourse( false )
{
    Style style;
    style.add( const_cast<TextSymbol*>(symbol) );
    init( style );
}

LabelNode::LabelNode(const std::string&  text,
                     const Style&        style ) :
GeoPositionNode(),
_text             ( text ),
_labelRotationRad ( 0. ),
_followFixedCourse( false )
{
    init( style );
}

LabelNode::LabelNode(MapNode*            mapNode,
                     const GeoPoint&     position,
                     const Style&        style ) :
GeoPositionNode   ( mapNode, position ),
_labelRotationRad ( 0. ),
_followFixedCourse( false )
{
    init( style );
}

LabelNode::LabelNode(MapNode*            mapNode,
                     const Style&        style ) :
GeoPositionNode   ( mapNode, GeoPoint::INVALID ),
_labelRotationRad ( 0. ),
_followFixedCourse( false )
{
    init( style );
}

void
LabelNode::init( const Style& style )
{
    ScreenSpaceLayout::activate( this->getOrCreateStateSet() );

    _geode = new osg::Geode();

    // ensure that (0,0,0) is the bounding sphere control/center point.
    // useful for things like horizon culling.
    _geode->setComputeBoundingSphereCallback(new ControlPointCallback());

    getPositionAttitudeTransform()->addChild( _geode.get() );

    osg::StateSet* stateSet = _geode->getOrCreateStateSet();
    stateSet->setAttributeAndModes( new osg::Depth(osg::Depth::ALWAYS, 0, 1, false), 1 );    

    setStyle( style );
}

void
LabelNode::setText( const std::string& text )
{
    if ( !_dynamic && getNumParents() > 0 )
    {
        OE_WARN << LC << "Illegal state: cannot change a LabelNode that is not dynamic" << std::endl;
        return;
    }

    osgText::Text* d = dynamic_cast<osgText::Text*>(_geode->getDrawable(0));
    if ( d )
    {
        d->setText( text );
        d->dirtyDisplayList();
        _text = text;
    }
}

void
LabelNode::setStyle( const Style& style )
{
    if ( !_dynamic && getNumParents() > 0 )
    {
        OE_WARN << LC << "Illegal state: cannot change a LabelNode that is not dynamic" << std::endl;
        return;
    }

    _geode->removeDrawables( 0, _geode->getNumDrawables() );

    _style = style;

    const TextSymbol* symbol = _style.get<TextSymbol>();

    if ( _text.empty() )
        _text = symbol->content()->eval();

    if ( symbol && symbol->onScreenRotation().isSet() )
    {
        _labelRotationRad = osg::DegreesToRadians(symbol->onScreenRotation()->eval());
    }

    // In case of a label must follow a course on map, we project a point from the position
    // with the given bearing. Then during culling phase we compute both points on the screen
    // and then we can deduce the screen rotation
    // may be optimized...
    else if ( symbol && symbol->geographicCourse().isSet() )
    {
        _followFixedCourse = true;
        _labelRotationRad = osg::DegreesToRadians ( symbol->geographicCourse()->eval() );

        double latRad;
        double longRad;
        GeoMath::destination( osg::DegreesToRadians( getPosition().y() ),
                              osg::DegreesToRadians( getPosition().x() ),
                              _labelRotationRad,
                              2500.,
                              latRad,
                              longRad );
        _geoPointProj.set ( osgEarth::SpatialReference::get("wgs84"),
                                       osg::RadiansToDegrees(longRad),
                                       osg::RadiansToDegrees(latRad),
                                       0,
                                       osgEarth::ALTMODE_ABSOLUTE );
    }

    osg::Drawable* t = AnnotationUtils::createTextDrawable( _text, symbol, osg::Vec3(0,0,0) );
    _geode->addDrawable(t);
    _geode->setCullingActive(false);
    
    applyStyle( _style );

    setLightingIfNotSet( false );

    Registry::shaderGenerator().run(
        this,
        "osgEarth.LabelNode",
        Registry::stateSetCache() );

    updateLayoutData();
}

void
LabelNode::setPriority(float value)
{
    GeoPositionNode::setPriority(value);
    updateLayoutData();
}

void
LabelNode::updateLayoutData()
{
    if (!_dataLayout.valid())
    {
        _dataLayout = new ScreenSpaceLayoutData();
    }

    // re-apply annotation drawable-level stuff as neccesary.
    for (unsigned i = 0; i < _geode->getNumDrawables(); ++i)
    {
        _geode->getDrawable(i)->setUserData(_dataLayout.get());
    }
    
    _dataLayout->setPriority(getPriority());
    const TextSymbol* ts = getStyle().get<TextSymbol>();
    if (ts)
    {
        _dataLayout->setPixelOffset(ts->pixelOffset().get());
        _dataLayout->setRotationRad(_labelRotationRad);
    }
}

void
LabelNode::setDynamic( bool dynamic )
{
    GeoPositionNode::setDynamic( dynamic );

    osgText::Text* d = dynamic_cast<osgText::Text*>(_geode->getDrawable(0));
    if ( d )
    {
        d->setDataVariance( dynamic ? osg::Object::DYNAMIC : osg::Object::STATIC );
    }    
}

void
LabelNode::traverse(osg::NodeVisitor &nv)
{
    if(_followFixedCourse)
    {
        osgUtil::CullVisitor* cv = NULL;
        if ( nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
        {
            cv = Culling::asCullVisitor(nv);
            osg::Camera* camera = cv->getCurrentCamera();

            osg::Matrix matrix;
            matrix.postMult(camera->getViewMatrix());
            matrix.postMult(camera->getProjectionMatrix());
            if (camera->getViewport())
                matrix.postMult(camera->getViewport()->computeWindowMatrix());

            GeoPoint pos( osgEarth::SpatialReference::get("wgs84"),
                          getPosition().x(),
                          getPosition().y(),
                          0,
                          osgEarth::ALTMODE_ABSOLUTE );

            osg::Vec3d refOnWorld; pos.toWorld(refOnWorld);
            osg::Vec3d projOnWorld; _geoPointProj.toWorld(projOnWorld);
            osg::Vec3d refOnScreen = refOnWorld * matrix;
            osg::Vec3d projOnScreen = projOnWorld * matrix;
            projOnScreen -= refOnScreen;
            _labelRotationRad = atan2 (projOnScreen.y(), projOnScreen.x());
            updateLayoutData();
        }
    }
    GeoPositionNode::traverse(nv);
}


//-------------------------------------------------------------------

OSGEARTH_REGISTER_ANNOTATION( label, osgEarth::Annotation::LabelNode );


LabelNode::LabelNode(MapNode*              mapNode,
                     const Config&         conf,
                     const osgDB::Options* dbOptions ) :
GeoPositionNode( mapNode, conf ),
_labelRotationRad ( 0. ),
_followFixedCourse( false )
{
    optional<Style> style;

    conf.getObjIfSet( "style", style );
    conf.getIfSet   ( "text",  _text );

    init( *style );
}

Config
LabelNode::getConfig() const
{
    Config conf( "label" );
    conf.add   ( "text",   _text );
    conf.addObj( "style",  _style );

    return conf;
}
