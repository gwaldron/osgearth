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

#include <osgEarthAnnotation/PlaceNode>
#include <osgEarthAnnotation/AnnotationUtils>
#include <osgEarthAnnotation/AnnotationRegistry>
#include <osgEarthAnnotation/BboxDrawable>
#include <osgEarthFeatures/TextSymbolizer>
#include <osgEarth/Utils>
#include <osgEarth/Registry>
#include <osgEarth/ShaderGenerator>
#include <osgEarth/GeoMath>
#include <osgEarth/ScreenSpaceLayout>
#include <osgEarth/NodeUtils>
#include <osgEarth/Lighting>
#include <osgEarth/Shaders>

#include <osg/Depth>
#include <osgText/Text>

#define LC "[PlaceNode] "

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

namespace
{
    const char* iconVS =
        "#version 330\n"
        "out vec2 oe_PlaceNode_texcoord; \n"
        "void oe_PlaceNode_icon_VS(inout vec4 vertex) { \n"
        "    oe_PlaceNode_texcoord = gl_MultiTexCoord0.st; \n"
        "} \n";

    const char* iconFS =
        "#version 330\n"
        "in vec2 oe_PlaceNode_texcoord; \n"
        "uniform sampler2D oe_PlaceNode_tex; \n"
        "void oe_PlaceNode_icon_FS(inout vec4 color) { \n"
        "    color = texture(oe_PlaceNode_tex, oe_PlaceNode_texcoord); \n"
        "} \n";
}

osg::ref_ptr<osg::StateSet> PlaceNode::_geodeStateSet;
osg::ref_ptr<osg::StateSet> PlaceNode::_textStateSet;
osg::ref_ptr<osg::StateSet> PlaceNode::_imageStateSet;

PlaceNode::PlaceNode() :
_geode            ( 0L ),
_labelRotationRad ( 0. ),
_followFixedCourse( false ),
_imageDrawable(0L),
_bboxDrawable(0L),
_textDrawable(0L)
{
    //nop
}

PlaceNode::PlaceNode(MapNode*           mapNode,
                     const GeoPoint&    position,
                     osg::Image*        image,
                     const std::string& text,
                     const Style&       style ) :

GeoPositionNode( mapNode ),
_image   ( image ),
_text    ( text ),
_style   ( style ),
_geode            ( 0L ),
_labelRotationRad ( 0. ),
_followFixedCourse( false ),
_imageDrawable(0L),
_bboxDrawable(0L),
_textDrawable(0L)
{
    init();
    setPosition(position);
}

PlaceNode::PlaceNode(MapNode*           mapNode,
                     const GeoPoint&    position,
                     const std::string& text,
                     const Style&       style ) :

GeoPositionNode( mapNode ),
_text    ( text ),
_style   ( style ),
_geode            ( 0L ),
_labelRotationRad ( 0. ),
_followFixedCourse( false ),
_imageDrawable(0L),
_bboxDrawable(0L),
_textDrawable(0L)
{
    init();
    setPosition(position);
}

PlaceNode::PlaceNode(MapNode*              mapNode,
                     const GeoPoint&       position,
                     const Style&          style,
                     const osgDB::Options* dbOptions ) :
GeoPositionNode ( mapNode ),
_style    ( style ),
_dbOptions        ( dbOptions ),
_geode            ( 0L ),
_labelRotationRad ( 0. ),
_followFixedCourse( false ),
_imageDrawable(0L),
_bboxDrawable(0L),
_textDrawable(0L)
{
    init();
    setPosition(position);
}

void
PlaceNode::setupState()
{
    if (!_geodeStateSet.valid())
    {
        static Threading::Mutex s_mutex;
        s_mutex.lock();
        if (!_geodeStateSet.valid())
        {
            _geodeStateSet = new osg::StateSet();

            // draw in the screen-space bin
            ScreenSpaceLayout::activate(_geodeStateSet.get());

            // completely disable depth buffer
            _geodeStateSet->setAttributeAndModes( new osg::Depth(osg::Depth::ALWAYS, 0, 1, false), 1 ); 

            // Disable lighting for place nodes by default
            _geodeStateSet->setDefine(OE_LIGHTING_DEFINE, osg::StateAttribute::ON);
            
            // shared stateset for the text
            {
                _textStateSet = new osg::StateSet();
                TextSymbolizer::installShaders(_textStateSet.get());
            }

            // shared stateset for the icon
            {
                _imageStateSet = new osg::StateSet();
                VirtualProgram* vp = VirtualProgram::getOrCreate(_imageStateSet.get());
                vp->setFunction("oe_PlaceNode_icon_VS", iconVS, ShaderComp::LOCATION_VERTEX_MODEL);
                vp->setFunction("oe_PlaceNode_icon_FS", iconFS, ShaderComp::LOCATION_FRAGMENT_COLORING);
                _imageStateSet->addUniform(new osg::Uniform("oe_PlaceNode_tex", 0));
            }
        }
        s_mutex.unlock();
    }
}

void
PlaceNode::init()
{
    setupState();

    osgEarth::clearChildren( getPositionAttitudeTransform() );

    _geode = new osg::Group();
    _geode->setCullingActive(false);
    _geode->setStateSet(_geodeStateSet.get());

    // ensure that (0,0,0) is the bounding sphere control/center point.
    // useful for things like horizon culling.
    _geode->setComputeBoundingSphereCallback(new ControlPointCallback());

    getPositionAttitudeTransform()->addChild(_geode);

    _imageDrawable = 0L;
    _bboxDrawable = 0L;
    _textDrawable = 0L;

    const TextSymbol* symbol = _style.get<TextSymbol>();

    // If there's no explicit text, look to the text symbol for content.
    if ( _text.empty() && symbol )
    {
        _text = symbol->content()->eval();
    }

    // Handle the rotation if any
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

    osg::ref_ptr<const InstanceSymbol> instance = _style.get<InstanceSymbol>();

    const IconSymbol* icon = instance->asIcon();

    if ( !_image.valid() )
    {
        URI imageURI;

        if ( icon )
        {
            if ( icon->url().isSet() )
            {
                imageURI = icon->url()->evalURI();
            }
            else if (icon->getImage())
            {
                _image = icon->getImage();
            }
        }

        if ( !imageURI.empty() )
        {
            _image = imageURI.getImage( _dbOptions.get() );
        }
    }

    osg::BoundingBox imageBox(0,0,0,0,0,0);

    // found an image; now format it:
    if ( _image.get() )
    {
        // Scale the icon if necessary
        double scale = 1.0;
        if ( icon && icon->scale().isSet() )
        {
            scale = icon->scale()->eval();
        }

        double s = scale * _image->s();
        double t = scale * _image->t();

        // this offset anchors the image at the bottom
        osg::Vec2s offset;
        if ( !icon || !icon->alignment().isSet() )
        {	
            // default to bottom center
            offset.set(0.0, t / 2.0);
        }
        else
        {	// default to bottom center
            switch (icon->alignment().value())
            {
            case IconSymbol::ALIGN_LEFT_TOP:
                offset.set((s / 2.0), -(t / 2.0));
                break;
            case IconSymbol::ALIGN_LEFT_CENTER:
                offset.set((s / 2.0), 0.0);
                break;
            case IconSymbol::ALIGN_LEFT_BOTTOM:
                offset.set((s / 2.0), (t / 2.0));
                break;
            case IconSymbol::ALIGN_CENTER_TOP:
                offset.set(0.0, -(t / 2.0));
                break;
            case IconSymbol::ALIGN_CENTER_CENTER:
                offset.set(0.0, 0.0);
                break;
            case IconSymbol::ALIGN_CENTER_BOTTOM:
            default:
                offset.set(0.0, (t / 2.0));
                break;
            case IconSymbol::ALIGN_RIGHT_TOP:
                offset.set(-(s / 2.0), -(t / 2.0));
                break;
            case IconSymbol::ALIGN_RIGHT_CENTER:
                offset.set(-(s / 2.0), 0.0);
                break;
            case IconSymbol::ALIGN_RIGHT_BOTTOM:
                offset.set(-(s / 2.0), (t / 2.0));
                break;
            }
        }

        // Apply a rotation to the marker if requested:
        double heading = 0.0;
        if ( icon && icon->heading().isSet() )
        {
            heading = osg::DegreesToRadians( icon->heading()->eval() );
        }

        //We must actually rotate the geometry itself and not use a MatrixTransform b/c the 
        //decluttering doesn't respect Transforms above the drawable.        
        _imageDrawable = AnnotationUtils::createImageGeometry(_image.get(), offset, 0, heading, scale);
        if (_imageDrawable)
        {
            //osg::Group* imageGroup = new osg::Group();
            //imageGroup->setStateSet(_imageStateSet.get());
            //imageGroup->addChild(_imageDrawable);
            //_geode->addChild(imageGroup);
            _imageDrawable->getOrCreateStateSet()->merge(*_imageStateSet.get());
            _geode->addChild(_imageDrawable);
            imageBox = osgEarth::Utils::getBoundingBox(_imageDrawable);
        }    
    }

    if ( _image.valid() )
    {
        TextSymbol* textSymbol = _style.getOrCreate<TextSymbol>();
        if ( !textSymbol->alignment().isSet() )
            textSymbol->alignment() = textSymbol->ALIGN_LEFT_CENTER;
    }

    _textDrawable = AnnotationUtils::createTextDrawable(
            _text,
            _style.get<TextSymbol>(),
            imageBox );

    const BBoxSymbol* bboxsymbol = _style.get<BBoxSymbol>();
    if ( bboxsymbol && _textDrawable )
    {
        _bboxDrawable = new BboxDrawable( osgEarth::Utils::getBoundingBox(_textDrawable), *bboxsymbol );
        _geode->addChild(_bboxDrawable);
    }

    if ( _textDrawable )
    {
        //_textDrawable->setStateSet(_textStateSet.get());
        _geode->addChild( _textDrawable );
    }

    setDefaultLighting( false );

    applyStyle( _style );

    setPriority(getPriority());

    if ( _dynamic )
        setDynamic( _dynamic );

    updateLayoutData();
}

void
PlaceNode::dirty()
{
    GeoPositionNode::dirty();
    updateLayoutData();
}

void
PlaceNode::setPriority(float value)
{
    GeoPositionNode::setPriority(value);
    updateLayoutData();
}

void
PlaceNode::updateLayoutData()
{
    if (!_dataLayout.valid())
    {
        _dataLayout = new ScreenSpaceLayoutData();
    }

    // re-apply annotation drawable-level stuff as neccesary.
    if (_imageDrawable)
        _imageDrawable->setUserData(_dataLayout.get());

    if (_bboxDrawable)
        _bboxDrawable->setUserData(_dataLayout.get());

    if (_textDrawable)
        _textDrawable->setUserData(_dataLayout.get());

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
PlaceNode::setText( const std::string& text )
{
    if ( !_dynamic && !_geode )
    {
        OE_WARN << LC << "Illegal state: cannot change a LabelNode that is not dynamic" << std::endl;
        return;
    }

    _text = text;

    if (_textDrawable)
    {
		TextSymbol* symbol =  _style.getOrCreate<TextSymbol>();
		osgText::String::Encoding text_encoding = osgText::String::ENCODING_UNDEFINED;
		if ( symbol && symbol->encoding().isSet() )
		{
			text_encoding = AnnotationUtils::convertTextSymbolEncoding(symbol->encoding().value());
		}

        _textDrawable->setText( text, text_encoding );
    }
}


void
PlaceNode::setStyle(const Style& style)
{
    // changing the style requires a complete rebuild.
    _style = style;
    init();
}


void
PlaceNode::setIconImage(osg::Image* image)
{
    if (_image != image)
    {
        _image = image;
        if (_imageDrawable)
        {            
            osg::Texture2D* texture = dynamic_cast<osg::Texture2D*>(_imageDrawable->getStateSet()->getTextureAttribute(0, osg::StateAttribute::TEXTURE));
            if (texture)
            {                
                texture->setImage(_image);
            }            
        }
        else
        {
            init();
        }
    }
}


void
PlaceNode::setDynamic( bool value )
{
    GeoPositionNode::setDynamic( value );
    
    osg::Object::DataVariance dv = value ? osg::Object::DYNAMIC : osg::Object::STATIC;

    if (_textDrawable)
        _textDrawable->setDataVariance(dv);

    if (_bboxDrawable)
        _bboxDrawable->setDataVariance(dv);

    if (_imageDrawable)
        _imageDrawable->setDataVariance(dv);
}

//-------------------------------------------------------------------

OSGEARTH_REGISTER_ANNOTATION( place, osgEarth::Annotation::PlaceNode );


PlaceNode::PlaceNode(MapNode*              mapNode,
                     const Config&         conf,
                     const osgDB::Options* dbOptions) :
GeoPositionNode ( mapNode, conf ),
_dbOptions( dbOptions ),
_labelRotationRad ( 0. ),
_followFixedCourse( false )
{
    conf.getObjIfSet( "style",  _style );
    conf.getIfSet   ( "text",   _text );

    optional<URI> imageURI;
    conf.getIfSet( "icon", imageURI );
    if ( imageURI.isSet() )
    {
        _image = imageURI->getImage();
        if ( _image.valid() )
            _image->setFileName( imageURI->base() );
    }

    init();
    setPosition(getPosition());
}

void
PlaceNode::setConfig(const Config& conf)
{
    GeoPositionNode::setConfig(conf);

    conf.getObjIfSet( "style",  _style );
    conf.getIfSet   ( "text",   _text );

    optional<URI> imageURI;
    conf.getIfSet( "icon", imageURI );
    if ( imageURI.isSet() )
    {
        _image = imageURI->getImage();
        if ( _image.valid() )
            _image->setFileName( imageURI->base() );
    }

    init();
}

Config
PlaceNode::getConfig() const
{
    Config conf( "place" );
    conf.add   ( "text",   _text );
    conf.addObj( "style",  _style );
    if ( _image.valid() ) {
        if ( !_image->getFileName().empty() )
            conf.add( "icon", _image->getFileName() );
        else if ( !_image->getName().empty() )
            conf.add( "icon", _image->getName() );
    }

    return conf;
}


#undef  LC
#define LC "[PlaceNode Serializer] "

#include <osgDB/ObjectWrapper>
#include <osgDB/InputStream>
#include <osgDB/OutputStream>

namespace osgEarth { namespace Serializers { namespace PlaceNode
{
    // functions
    static bool checkConfig(const osgEarth::Annotation::PlaceNode& node)
    {
        return true;
    }

    static bool readConfig(osgDB::InputStream& is, osgEarth::Annotation::PlaceNode& node)
    {
        std::string json;
        is >> json;
        Config conf;
        conf.fromJSON(json);
        node.setConfig(conf);
        return true;
    }

    static bool writeConfig(osgDB::OutputStream& os, const osgEarth::Annotation::PlaceNode& node)
    {
        os << node.getConfig().toJSON(false) << std::endl;
        return true;
    }

    REGISTER_OBJECT_WRAPPER(
        PlaceNode,
        new osgEarth::Annotation::PlaceNode,
        osgEarth::Annotation::PlaceNode,
        "osg::Object osg::Node osg::Group osgEarth::Annotation::AnnotationNode osgEarth::Annotation::GeoPositionNode osgEarth::Annotation::PlaceNode")
    {
        ADD_USER_SERIALIZER(Config);
    }
} } }
