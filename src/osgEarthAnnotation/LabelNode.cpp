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

#include <osgEarthAnnotation/LabelNode>
#include <osgEarthAnnotation/AnnotationUtils>
#include <osgEarthAnnotation/AnnotationRegistry>
#include <osgEarthAnnotation/BboxDrawable>
#include <osgEarthSymbology/Color>
#include <osgEarthSymbology/BBoxSymbol>
#include <osgEarth/Registry>
#include <osgEarth/ShaderGenerator>
#include <osgEarth/GeoMath>
#include <osgEarth/Utils>
#include <osgEarth/ScreenSpaceLayout>
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

LabelNode::LabelNode(const LabelNode& rhs, const osg::CopyOp& op) :
GeoPositionNode(rhs, op),
_labelRotationRad(0.),
_followFixedCourse(false)
{
    //nop - unused
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
        const TextSymbol* symbol = _style.get<TextSymbol>();

        osgText::String::Encoding textEncoding = osgText::String::ENCODING_UNDEFINED;
        if (symbol && symbol->encoding().isSet())
        {
            textEncoding = AnnotationUtils::convertTextSymbolEncoding(symbol->encoding().value());
        }

        d->setText(text, textEncoding);

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
    }

    osg::Drawable* text = AnnotationUtils::createTextDrawable( _text, symbol, osg::Vec3(0,0,0) );

    const BBoxSymbol* bboxsymbol = _style.get<BBoxSymbol>();
    if ( bboxsymbol && text )
    {
        osg::Drawable* bboxGeom = new BboxDrawable( Utils::getBoundingBox(text), *bboxsymbol );
        _geode->addDrawable(bboxGeom);
    }

    _geode->addDrawable(text);
    _geode->setCullingActive(false);

    applyStyle( _style );

    setLightingIfNotSet( false );

    Registry::shaderGenerator().run(
        this,
        "osgEarth.LabelNode",
        Registry::stateSetCache() );

    updateLayoutData();
    dirty();
}

void
LabelNode::dirty()
{
    GeoPositionNode::dirty();
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
    
    GeoPoint location = getPosition();
    location.makeGeographic();
    double latRad;
    double longRad;
    GeoMath::destination(osg::DegreesToRadians(location.y()),
        osg::DegreesToRadians(location.x()),
        _labelRotationRad,
        2500.,
        latRad,
        longRad);

    _geoPointProj.set(osgEarth::SpatialReference::get("wgs84"),
        osg::RadiansToDegrees(longRad),
        osg::RadiansToDegrees(latRad),
        0,
        osgEarth::ALTMODE_ABSOLUTE);

    _geoPointLoc.set(osgEarth::SpatialReference::get("wgs84"),
        //location.getSRS(),
        location.x(),
        location.y(),
        0,
        osgEarth::ALTMODE_ABSOLUTE);

    const TextSymbol* ts = getStyle().get<TextSymbol>();
    if (ts)
    {
        _dataLayout->setPixelOffset(ts->pixelOffset().get());
        
        if (_followFixedCourse)
        {
            osg::Vec3d p0, p1;
            _geoPointLoc.toWorld(p0);
            _geoPointProj.toWorld(p1);
            _dataLayout->setAnchorPoint(p0);
            _dataLayout->setProjPoint(p1);
        }
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
