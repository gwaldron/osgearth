/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2019 Pelican Mapping
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
#include <osgEarth/ShaderGenerator>
#include <osgEarth/GeoMath>
#include <osgEarth/Utils>
#include <osgEarth/ScreenSpaceLayout>
#include <osgEarth/Lighting>
#include <osgEarth/Shaders>
#include <osgText/Text>
#include <osg/Depth>
#include <osgUtil/IntersectionVisitor>
#include <osgUtil/LineSegmentIntersector>

#define LC "[LabelNode] "

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Symbology;


//-------------------------------------------------------------------

// Globally shared stateset for LabelNode geometry
osg::observer_ptr<osg::StateSet> LabelNode::s_geodeStateSet;

LabelNode::LabelNode() :
GeoPositionNode()
{
    construct();
    compile();
}

LabelNode::LabelNode(const std::string& text,
                     const Style& style) :
GeoPositionNode()
{
    construct();

    _text = text;
    _style = style;

    compile();
}

LabelNode::LabelNode(const GeoPoint& position,
                     const std::string& text,
                     const Style& style) :
GeoPositionNode()
{
    construct();

    _text = text;
    _style = style;
    setPosition(position);

    compile();
}

void
LabelNode::construct()
{
    _labelRotationRad = 0.0f;
    _followFixedCourse = false;

    // This class makes its own shaders
    ShaderGenerator::setIgnoreHint(this, true);

    // Initialize the shared stateset as necessary
    osg::ref_ptr<osg::StateSet> geodeStateSet;
    if (s_geodeStateSet.lock(geodeStateSet) == false)
    {
        static Threading::Mutex s_mutex;
        Threading::ScopedMutexLock lock(s_mutex);

        if (s_geodeStateSet.lock(geodeStateSet) == false)
        {
            s_geodeStateSet = geodeStateSet = new osg::StateSet();

            // draw in the screen-space bin
            ScreenSpaceLayout::activate(geodeStateSet.get());

            // completely disable depth buffer
            geodeStateSet->setAttributeAndModes( new osg::Depth(osg::Depth::ALWAYS, 0, 1, false), 1 ); 

            // Disable lighting for place label bbox
            geodeStateSet->setDefine(OE_LIGHTING_DEFINE, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);
        }
    }

    _geode = new osg::Geode();
    _geode->setStateSet(geodeStateSet.get());

    // ensure that (0,0,0) is the bounding sphere control/center point.
    // useful for things like horizon culling.
    _geode->setComputeBoundingSphereCallback(new ControlPointCallback());

    getPositionAttitudeTransform()->addChild( _geode.get() );
}

void
LabelNode::setText( const std::string& text )
{
    if ( !_dynamic && getNumParents() > 0 )
    {
        OE_WARN << LC << "Illegal state: cannot change a LabelNode that is not dynamic" << std::endl;
        return;
    }

    for (unsigned int i=0; i < _geode->getNumChildren(); i++)
    {
        osgText::Text* d = dynamic_cast<osgText::Text*>(_geode->getChild(i));
        if ( d )
        {
            const TextSymbol* symbol = _style.get<TextSymbol>();

            osgText::String::Encoding textEncoding = osgText::String::ENCODING_UNDEFINED;
            if (symbol && symbol->encoding().isSet())
            {
                textEncoding = AnnotationUtils::convertTextSymbolEncoding(symbol->encoding().value());
            }

            d->setText(text, textEncoding);

            _text = text;
            return;
        }
    }
}

void
LabelNode::compile()
{
    _geode->removeChildren(0, _geode->getNumChildren());

    const TextSymbol* symbol = _style.get<TextSymbol>();

    if (symbol)
    {
        if ( _text.empty() )
            _text = symbol->content()->eval();

        if ( symbol->onScreenRotation().isSet() )
        {
            _labelRotationRad = osg::DegreesToRadians(symbol->onScreenRotation()->eval());
        }

        // In case of a label must follow a course on map, we project a point from the position
        // with the given bearing. Then during culling phase we compute both points on the screen
        // and then we can deduce the screen rotation
        // may be optimized...
        else if ( symbol->geographicCourse().isSet() )
        {
            _followFixedCourse = true;
            _labelRotationRad = osg::DegreesToRadians ( symbol->geographicCourse()->eval() );
        }
    }

    osg::Drawable* text = AnnotationUtils::createTextDrawable( _text, symbol, osg::Vec3(0,0,0) );

    const BBoxSymbol* bboxsymbol = _style.get<BBoxSymbol>();
    if ( bboxsymbol && text )
    {
        osg::Drawable* bboxGeom = new BboxDrawable(text->getBoundingBox(), *bboxsymbol );
        if (bboxGeom)
        {
            _geode->addDrawable(bboxGeom);
        }
    }

    if (text)
    {
        if (_dynamic)
        {
            text->setDataVariance(osg::Object::DYNAMIC);
        }
        _geode->addDrawable(text);
    }

    applyStyle( _style );

    updateLayoutData();
    dirty();
}

void
LabelNode::setStyle( const Style& style )
{
    if ( !_dynamic && getNumParents() > 0 )
    {
        OE_WARN << LC << "Illegal state: cannot change a LabelNode that is not dynamic" << std::endl;
        return;
    }

    _style = style;

    compile();
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
    for (unsigned i = 0; i < _geode->getNumChildren(); ++i)
    {
        _geode->getChild(i)->setUserData(_dataLayout.get());
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

    if (_geode.valid())
    {
        for(unsigned i=0; i<_geode->getNumChildren(); ++i)
        {
            osg::Node* node = _geode->getChild(i);
            if (node)
            {
                node->setDataVariance(dynamic ? osg::Object::DYNAMIC : osg::Object::STATIC);
            }
        }
    }
}

//-------------------------------------------------------------------

OSGEARTH_REGISTER_ANNOTATION( label, osgEarth::Annotation::LabelNode );


LabelNode::LabelNode(const Config&         conf,
                     const osgDB::Options* dbOptions ) :
GeoPositionNode( conf, dbOptions )
{
    construct();

    conf.get("style", _style);
    conf.get("text", _text);

    compile();
}

Config
LabelNode::getConfig() const
{
    Config conf = GeoPositionNode::getConfig();
    conf.key() = "label";
    conf.set( "text",   _text );
    conf.set( "style",  _style );
    return conf;
}
