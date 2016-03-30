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
#include <osgEarth/Decluttering>
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
_text    ( text )
{
    init( style );
}

LabelNode::LabelNode(MapNode*            mapNode,
                     const GeoPoint&     position,
                     const std::string&  text,
                     const TextSymbol*   symbol ) :

GeoPositionNode( mapNode, position ),
_text    ( text )
{
    Style style;
    style.add( const_cast<TextSymbol*>(symbol) );
    init( style );
}

LabelNode::LabelNode(const std::string&  text,
                     const Style&        style ) :
GeoPositionNode(),
_text    ( text )
{
    init( style );
}

LabelNode::LabelNode(MapNode*            mapNode,
                     const GeoPoint&     position,
                     const Style&        style ) :
GeoPositionNode( mapNode, position )
{
    init( style );
}

LabelNode::LabelNode(MapNode*            mapNode,
                     const Style&        style ) :
GeoPositionNode( mapNode, GeoPoint::INVALID )
{
    init( style );
}

void
LabelNode::init( const Style& style )
{
    Decluttering::setEnabled( this->getOrCreateStateSet(), true );

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
    osg::ref_ptr<LayoutData> data = new LayoutData();
    data->_priority = getPriority();
    const TextSymbol* ts = getStyle().get<TextSymbol>();
    if (ts) data->_pixelOffset = ts->pixelOffset().get();

    // re-apply annotation drawable-level stuff as neccesary.
    for (unsigned i = 0; i<_geode->getNumDrawables(); ++i)
    {
        _geode->getDrawable(i)->setUserData(data.get());
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
GeoPositionNode( mapNode, conf )
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
