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
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */
#include <osgEarthUtil/Controls>
#include <osgEarth/NodeUtils>
#include <osg/Geometry>
#include <osg/NodeCallback>
#include <osg/Depth>
#include <osg/TextureRectangle>
#include <osgGA/GUIEventHandler>
#include <osgText/Text>
#include <osgUtil/RenderBin>
#include <osgUtil/Statistics>
#include <osgEarthSymbology/Style>
#include <osgEarthSymbology/Geometry>
#include <osgEarthSymbology/GeometryRasterizer>
#include <osgEarthFeatures/PolygonizeLines>
#include <osg/Version>
#include <osgEarth/Common>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/Utils>
#include <osgEarth/CullingUtils>
#include <osgEarth/ShaderGenerator>
#include <osgEarth/VirtualProgram>

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;
using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;

#define LC "[Controls] "

// ---------------------------------------------------------------------------

namespace
{
    void calculateRotatedSize( float w, float h, float angle_rad, float& out_w, float& out_h )
    {
        float x1 = -w/2, x2 = w/2, x3 =  w/2, x4 = -w/2;
        float y1 =  h/2, y2 = h/2, y3 = -h/2, y4 = -h/2;

        float cosa = cos(angle_rad);
        float sina = sin(angle_rad);

        float
            x11 =  x1*cosa + y1*sina,
            y11 = -x1*sina + y1*cosa,
            x21 =  x2*cosa + y2*sina,
            y21 = -x2*sina + y2*cosa,
            x31 =  x3*cosa + y3*sina,
            y31 = -x3*sina + y3*cosa,
            x41 =  x4*cosa + y4*sina,
            y41 = -x4*sina + y3*cosa;

        float xmin = std::min(x11, std::min(x21, std::min(x31, x41)));
        float ymin = std::min(y11, std::min(y21, std::min(y31, y41)));

        float xmax = std::max(x11, std::max(x21, std::max(x31, x41)));
        float ymax = std::max(y11, std::max(y21, std::max(y31, y41)));

        out_w = xmax-xmin;
        out_h = ymax-ymin;
    }

    void rot( float x, float y, const osg::Vec2f& c, float angle_rad, osg::Vec3f& out )
    {
        float cosa = cos(angle_rad);
        float sina = sin(angle_rad);
        out.x() = (c.x()-x)*cosa - (c.y()-y)*sina + c.x();
        out.y() = (c.y()-y)*cosa + (c.x()-x)*sina + c.y();
        out.z() = 0.0f;
    }

    // Convenience method to create safe Control geometry.
    // Since Control geometry can change, we need to always set it
    // to DYNAMIC data variance.
    osg::Geometry* newGeometry()
    {
        osg::Geometry* geom = new osg::Geometry();
        geom->setUseVertexBufferObjects( true );
        geom->setUseDisplayList( false );
        geom->setDataVariance( osg::Object::DYNAMIC );
        return geom;
    }
}

// ---------------------------------------------------------------------------

float
UVec2f::x( const osg::Vec2f& size ) const
{
    if ( _xunits == UNITS_PIXELS )
        return _v[0];
    else if ( _xunits == UNITS_FRACTION )
        return _v[0] * size.x();
    else // UNITS_INSET_PIXELS
        return size.x() - _v[0] - 1.0f;
}

float
UVec2f::x( const ControlContext& cx ) const
{
    return cx._vp ? x( osg::Vec2f(cx._vp->width(), cx._vp->height()) ) : _v[0];
}

float
UVec2f::y( const osg::Vec2f& size ) const
{
    if ( _yunits == UNITS_PIXELS )
        return _v[1];
    else if ( _yunits == UNITS_FRACTION )
        return _v[1] * size.y();
    else // UNITS_INSET_PIXELS
        return size.y() - _v[1] - 1.0f;
}

float
UVec2f::y( const ControlContext& cx ) const
{
    return cx._vp ? y( osg::Vec2f(cx._vp->width(), cx._vp->height()) ) : _v[1];
}

UVec2f
UVec2f::asPixels( const osg::Vec2f& size ) const
{
    return UVec2f( x(size), y(size), UNITS_PIXELS, UNITS_PIXELS );
}

UVec2f
UVec2f::asPixels( const ControlContext& cx ) const
{
    return UVec2f( x(cx), y(cx), UNITS_PIXELS, UNITS_PIXELS );
}


// ---------------------------------------------------------------------------

Control::Control()
{
    init();
}

Control::Control( const Alignment& halign, const Alignment& valign, const Gutter& padding )
{
    init();

    setHorizAlign( halign );
    setVertAlign( valign );
    setPadding( padding );
}

void
Control::init()
{
    _x.init(0);
    _y.init(0);
    _width.init(1);
    _height.init(1);
    _valign.init( ALIGN_NONE );
    _halign.init( ALIGN_NONE );
    _backColor.init( osg::Vec4(0,0,0,0) );
    _foreColor.init( osg::Vec4(1,1,1,1) );
    _activeColor.init( osg::Vec4(.4,.4,.4,1) );

    _margin = Gutter(0);
    _padding = Gutter(2);
    _hfill = false;
    _vfill = false;    
    _visible = true;
    _active = false;
    _absorbEvents = true;
    _dirty = true;
    _borderWidth = 1.0f;

    _geode = new osg::Geode();
    this->addChild( _geode );
    
#ifdef OSG_GLES2_AVAILABLE
    _alphaEffect = new AlphaEffect(this->getOrCreateStateSet());
#endif
}

void
Control::setVisible( bool value ) {
    if ( value != _visible ) {
        _visible = value;
        dirty();
    }
}

void
Control::setX( float value ) {
    if ( value != _x.value() ) {
        _x = value;
        dirty();
    }
}

void
Control::setY( float value ) {
    if ( value != _y.value() ) {
        _y = value;
        dirty();
    }
}

void
Control::setPosition( float x, float y ) {
    setX( x );
    setY( y );
}

void
Control::setWidth( float value ) {
    if ( value != _width.value() ) {
        _width = value;
        dirty();
    }
}

void 
Control::setHeight( float value ) {
    if ( value != _height.value() ) {
        _height = value;
        dirty();
    }
}

void
Control::setSize( float w, float h ) {
    setWidth( w );
    setHeight( h );
}

void
Control::setMargin( const Gutter& value ) {
    if ( value != _margin ) {
        _margin = value;
        dirty();
    }
}

void
Control::setMargin( Side side, float value ) {
    switch(side) {
        case SIDE_TOP:
            if ( _margin.top() != value ) {
                _margin.top() = value;
                dirty();
            }
            break;
        case SIDE_BOTTOM:
            if ( _margin.bottom() != value ) {
                _margin.bottom() = value;
                dirty();
            }
            break;
        case SIDE_LEFT:
            if ( _margin.left() != value ) {
                _margin.left() = value;
                dirty();
            }
            break;
        case SIDE_RIGHT:
            if ( _margin.right() != value ) {
                _margin.right() = value;
                dirty();
            }
            break;
    }
}

void
Control::setPadding( const Gutter& value )
{
    if ( value != _padding ) {
        _padding = value;
        dirty();
    }
}

void
Control::setPadding( float value ) {
    Gutter g(value);
    if ( g != _padding ) {
        _padding = g;
        dirty();
    }
}

void
Control::setPadding( Side side, float value ) {
    switch(side) {
        case SIDE_TOP:
            if ( _padding.top() != value ) {
                _padding.top() = value;
                dirty();
            }
            break;
        case SIDE_BOTTOM:
            if ( _padding.bottom() != value ) {
                _padding.bottom() = value;
                dirty();
            }
            break;
        case SIDE_LEFT:
            if ( _padding.left() != value ) {
                _padding.left() = value;
                dirty();
            }
            break;
        case SIDE_RIGHT:
            if ( _padding.right() != value ) {
                _padding.right() = value;
                dirty();
            }
            break;
    }
}

void
Control::setHorizAlign( const Alignment& value ) {
    if ( !_halign.isSetTo( value ) ) {
        _halign = value;
        _x.unset();  // horiz align is mutex with abs positioning
        dirty();
    }
}

void
Control::setVertAlign( const Alignment& value ) {
    if ( !_valign.isSetTo( value ) ) {
        _valign = value;
        _y.unset(); // vert align is mutex with abs positioning
        dirty();
    }
}

void
Control::setAlign(const Alignment& h, const Alignment& v) {
    setHorizAlign( h );
    setVertAlign ( v );
}

void
Control::setHorizFill( bool hfill, float minWidth ) {
    if ( hfill != _hfill || !_width.isSetTo(minWidth) ) { //minWidth != _width.value() ) {
        _hfill = hfill;
        if ( hfill )
            setWidth( minWidth );
        else
            _width.unset();
        dirty();
    }
}

void
Control::setVertFill( bool vfill, float minHeight ) {
    if ( vfill != _hfill || minHeight != _height.value() ) {
        _vfill = vfill;
        if ( vfill )
            setHeight( minHeight );
        else
            _height.unset();
        dirty();
    }
}

bool 
Control::parentIsVisible() const
{
    bool visible = true;

    // ------------------------------------------------------------------------
    // -- If visible through any parent, consider it visible and return true --
    // -- Also visible if the parent is not a control (is a top-level)       --
    // ------------------------------------------------------------------------
    for( unsigned i=0; i<getNumParents(); ++i )
    {
        const Control* c = dynamic_cast<const Control*>( getParent(i) );

        // ----------------------------------------
        // -- Parent not a control, keep looking --
        // ----------------------------------------
        if( c == NULL )
            continue;
        
        // -----------------------------------------
        // -- If this path is visible, we're done --
        // -----------------------------------------
        if( c->visible() && c->parentIsVisible() )
        {
            return true;
        }
        else
        {
            // ---------------------------------------------
            // -- If their is a parent control, but it's  --
            // -- not visible, change our assumption but  --
            // -- keep looking at other parent controls   --
            // ---------------------------------------------            
            visible = false;
        }
    }

    return visible;
}


void
Control::setForeColor( const osg::Vec4f& value ) {
    if ( value != _foreColor.value() ) {
        _foreColor = value;
        dirty();
    }
}

void
Control::setBackColor( const osg::Vec4f& value ) {
    if ( value != _backColor.value() ) {
        _backColor = value;
        dirty();
    }
}

void
Control::setActiveColor( const osg::Vec4f& value ) {
    if ( value != _activeColor.value() ) {
        _activeColor = value;
        if ( _active )
            dirty();
    }
}

void
Control::setBorderColor( const osg::Vec4f& value ) {
    if ( value != _borderColor.value() ) {
        _borderColor = value;
        dirty();
    }
}

void
Control::addEventHandler( ControlEventHandler* handler, bool fire )
{
    _eventHandlers.push_back( handler );
    if ( fire )
        fireValueChanged( handler );
}

void
Control::setActive( bool value ) {
    if ( value != _active ) {
        _active = value;
        if ( _activeColor.isSet() )
            dirty();
    }
}

void
Control::setBorderWidth( float value ) {
    if ( value != _borderWidth ) {
        _borderWidth = value;
        dirty();
    }
}

namespace
{
    void dirtyParent(osg::Group* p)
    {
        if ( p )
        {
            Control* c = dynamic_cast<Control*>( p );
            if ( c )
            {
                c->dirty();
            }
            else if ( dynamic_cast<ControlCanvas*>( p ) )
            {
                return;
            }
            else
            {
                for( unsigned i=0; i<p->getNumParents(); ++i )
                {
                    dirtyParent( p->getParent(i) );
                }
            }
        }
    }
}

void
Control::dirty()
{
    _dirty = true;
    for(unsigned i=0; i<getNumParents(); ++i)
    {
        dirtyParent( getParent(i) );
    }
}

void
Control::calcSize(const ControlContext& cx, osg::Vec2f& out_size)
{
    if ( visible() == true )
    {
        _renderSize.set( 
            width().value()  + padding().x(),
            height().value() + padding().y() );

        out_size.set(
            _renderSize.x() + margin().x(),
            _renderSize.y() + margin().y() );
    }
    else
    {
        out_size.set(0,0);
    }
}

void
Control::calcPos(const ControlContext& cx, const osg::Vec2f& cursor, const osg::Vec2f& parentSize)
{
    if ( _x.isSet() )
    {
        _renderPos.x() = cursor.x() + margin().left() + padding().left() + *x();
    }
    else
    {
        if ( _halign == ALIGN_CENTER )
        {
            _renderPos.x() = cursor.x() + 0.5*(parentSize.x() - _renderSize.x());
        }
        else if ( _halign == ALIGN_RIGHT )
        {
            _renderPos.x() = cursor.x() + parentSize.x() - margin().right() - _renderSize.x() + padding().left();
        }
        else
        {
            _renderPos.x() = cursor.x() + margin().left() + padding().left();
        }
    }

    if ( _y.isSet() )
    {
        _renderPos.y() = cursor.y() + margin().top() + padding().top() + *y();
    }
    else
    {
        if ( _valign == ALIGN_CENTER )
        {
            _renderPos.y() = cursor.y() + 0.5*parentSize.y() - 0.5*(_renderSize.y() - padding().y());
        }
        else if ( _valign == ALIGN_BOTTOM )
        {
            _renderPos.y() = cursor.y() + parentSize.y() - margin().bottom() - _renderSize.y() + padding().top();
        }
        else
        {
            _renderPos.y() = cursor.y() + margin().top() + padding().top();
        }
    }
}
        
bool
Control::intersects( float x, float y ) const
{
    return
        x >= _renderPos.x() - padding().left() && x <= _renderPos.x() - padding().left() + _renderSize.x() &&
        y >= _renderPos.y() - padding().top() && y <= _renderPos.y() - padding().top() + _renderSize.y();
}

void
Control::draw(const ControlContext& cx)
{
    clearGeode();

    // by default, rendering a Control directly results in a colored quad. Usually however
    // you will not render a Control directly, but rather one of its subclasses.
    if ( visible()  && parentIsVisible() )
    {
        if ( !(_backColor.isSet() && _backColor->a() == 0) && _renderSize.x() > 0 && _renderSize.y() > 0 )
        {
            float vph = cx._vp->height();

            // draw the background poly:
            {
                _geom = newGeometry();

                float rx = _renderPos.x() - padding().left();
                float ry = _renderPos.y() - padding().top();

                osg::Vec3Array* verts = new osg::Vec3Array(4);
                _geom->setVertexArray( verts );
                (*verts)[0].set( rx, vph - ry, 0 );
                (*verts)[1].set( rx, vph - ry - _renderSize.y(), 0 );
                (*verts)[2].set( rx + _renderSize.x(), vph - ry - _renderSize.y(), 0 );
                (*verts)[3].set( rx + _renderSize.x(), vph - ry, 0 );
                _geom->addPrimitiveSet( new osg::DrawArrays( GL_QUADS, 0, 4 ) );

                osg::Vec4Array* colors = new osg::Vec4Array(1);
                (*colors)[0] = _active && _activeColor.isSet() ? _activeColor.value() : _backColor.value();
                _geom->setColorArray( colors );
                _geom->setColorBinding( osg::Geometry::BIND_OVERALL );

                getGeode()->addDrawable( _geom.get() );
            }

            // draw the border:
            if ( _borderColor.isSet() && _borderWidth > 0.0f )
            {
                float rx = _renderPos.x() - padding().left();
                float ry = _renderPos.y() - padding().top();

                osg::ref_ptr<osg::Vec3Array> verts = new osg::Vec3Array(5);
                (*verts)[0].set( rx, vph - ry, 0 );
                (*verts)[1].set( rx, vph - ry - _renderSize.y(), 0 );
                (*verts)[2].set( rx + _renderSize.x(), vph - ry - _renderSize.y(), 0 );
                (*verts)[3].set( rx + _renderSize.x(), vph - ry, 0 );
                (*verts)[4].set( rx, vph - ry, 0 );

                Stroke stroke;
                stroke.color() = *_borderColor;
                stroke.width() = _borderWidth;
                stroke.lineCap() = Stroke::LINECAP_SQUARE;
                stroke.lineJoin() = Stroke::LINEJOIN_MITRE;

                PolygonizeLinesOperator makeBorder(stroke);
                osg::Geometry* geom = makeBorder( verts.get(), 0L );

                getGeode()->addDrawable( geom );
            }
        }

        _dirty = false;
    }
}

bool
Control::handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa, ControlContext& cx )
{
    bool handled = false;

    if( !visible() || !parentIsVisible() )
        return false;

    if ( _eventHandlers.size() > 0 )
    {    
        handled = true;

        if ( !_active )
        {
            if ( ea.getEventType() == osgGA::GUIEventAdapter::MOVE )
            {
                cx._active.push( this );
            }
        }
        else 
        {            
            if ( ea.getEventType() == osgGA::GUIEventAdapter::RELEASE )
            {
              float canvasY = cx._vp->height() - (ea.getY() - cx._view->getCamera()->getViewport()->y());
              float canvasX = ea.getX() - cx._view->getCamera()->getViewport()->x();

                for( ControlEventHandlerList::const_iterator i = _eventHandlers.begin(); i != _eventHandlers.end(); ++i )
                {
                    osg::Vec2f relXY( canvasX - _renderPos.x(), canvasY - _renderPos.y() );
                    i->get()->onClick( this, relXY, ea.getButtonMask() );
                    aa.requestRedraw();
                }
            }
        }
    }

    return handled || _absorbEvents;
}

// ---------------------------------------------------------------------------

namespace
{
    // override osg Text to get at some of the internal properties
    struct LabelText : public osgText::Text
    {
        LabelText() : osgText::Text() { setDataVariance(osg::Object::DYNAMIC); }
        const osg::BoundingBox& getTextBB() const { return _textBB; }
        const osg::Matrix& getATMatrix(int contextID) const { return _autoTransformCache[contextID]._matrix; }
    };

    // writes a value to a label
    struct ValueLabelHandler : public ControlEventHandler
    {
        osg::observer_ptr<LabelControl> _label;
        ValueLabelHandler( LabelControl* label ) : _label(label) { }
        void onValueChanged( class Control* control, bool value ) { 
            if ( _label.valid() ) _label->setText( Stringify() << value ); }
        void onValueChanged( class Control* control, double value ) {
            if ( _label.valid() ) _label->setText( Stringify() << std::setprecision(16) << value ); }
        void onValueChanged( class Control* control, float value ) {
            if ( _label.valid() ) _label->setText( Stringify() << value ); }
        void onValueChanged( class Control* control, int value ) {
            if ( _label.valid() ) _label->setText( Stringify() << value ); }
        void onValueChanged( class Control* control, const osg::Vec3f& value ) {
            if ( _label.valid() ) _label->setText( Stringify() << std::setprecision(8) << value.x() << ", " << value.y() << ", " << value.z() ); }
        void onValueChanged( class Control* control, const osg::Vec2f& value ) {
            if ( _label.valid() ) _label->setText( Stringify() << std::setprecision(8) << value.x() << ", " << value.y() ); }
        void onValueChanged( class Control* control, const osg::Vec3d& value ) {
            if ( _label.valid() ) _label->setText( Stringify() << std::setprecision(16) << value.x() << ", " << value.y() << ", " << value.z() ); }
        void onValueChanged( class Control* control, const osg::Vec2d& value ) {
            if ( _label.valid() ) _label->setText( Stringify() << std::setprecision(16) << value.x() << ", " << value.y() ); }
        void onValueChanged( class Control* control, const std::string& value ) {
            if ( _label.valid() ) _label->setText( value ); }
    };
}

LabelControl::LabelControl(const std::string& text,
                           float              fontSize,
                           const osg::Vec4f&  foreColor):
_text    ( text ),
_fontSize( fontSize ),
_encoding( osgText::String::ENCODING_UNDEFINED ),
_backdropType( osgText::Text::OUTLINE ),
_backdropImpl( osgText::Text::NO_DEPTH_BUFFER ),
_backdropOffset( 0.03f )
{    
    setFont( Registry::instance()->getDefaultFont() );    
    setForeColor( foreColor );
    setBackColor( osg::Vec4f(0,0,0,0) );
}

LabelControl::LabelControl(const std::string& text,
                           const osg::Vec4f&  foreColor,
                           float              fontSize ):
_text    ( text ),
_fontSize( fontSize ),
_encoding( osgText::String::ENCODING_UNDEFINED ),
_backdropType( osgText::Text::OUTLINE ),
_backdropImpl( osgText::Text::NO_DEPTH_BUFFER ),
_backdropOffset( 0.03f )
{    	
    setFont( Registry::instance()->getDefaultFont() );   
    setForeColor( foreColor );
    setBackColor( osg::Vec4f(0,0,0,0) );
}

LabelControl::LabelControl(Control*           valueControl,
                           float              fontSize,
                           const osg::Vec4f&  foreColor):
_fontSize( fontSize ),
_encoding( osgText::String::ENCODING_UNDEFINED ),
_backdropType( osgText::Text::OUTLINE ),
_backdropImpl( osgText::Text::NO_DEPTH_BUFFER ),
_backdropOffset( 0.03f )
{
    setFont( Registry::instance()->getDefaultFont() );    
    setForeColor( foreColor );
    setBackColor( osg::Vec4f(0,0,0,0) );

    if ( valueControl )
        valueControl->addEventHandler( new ValueLabelHandler(this), true );
}

LabelControl::LabelControl(Control*           valueControl,
                           const osg::Vec4f&  foreColor,
                           float              fontSize ):
_fontSize( fontSize ),
_encoding( osgText::String::ENCODING_UNDEFINED ),
_backdropType( osgText::Text::OUTLINE ),
_backdropImpl( osgText::Text::NO_DEPTH_BUFFER ),
_backdropOffset( 0.03f )
{    	
    setFont( Registry::instance()->getDefaultFont() );   
    setForeColor( foreColor );
    setBackColor( osg::Vec4f(0,0,0,0) );

    if ( valueControl )
        valueControl->addEventHandler( new ValueLabelHandler(this), true );
}


void
LabelControl::setText( const std::string& value )
{
    if ( value != _text ) {
        _text = value;
        dirty();
    }
}

void
LabelControl::setEncoding( osgText::String::Encoding value )
{
    if ( value != _encoding ) {
        _encoding = value;
        dirty();
    }
}

void
LabelControl::setFont( osgText::Font* value )
{
    if ( value != _font.get() ) {
        _font = value;
        dirty();
    }
}

void
LabelControl::setFontSize( float value )
{
    if ( value != _fontSize ) {
        _fontSize = value;
        dirty();
    }
}

void
LabelControl::setHaloColor( const osg::Vec4f& value )
{
    if ( !_haloColor.isSet() || *_haloColor != value ) {
        _haloColor = value;
        dirty();
    }
}

void
LabelControl::setTextBackdropImplementation(osgText::Text::BackdropImplementation value)
{
    if( _backdropImpl != value ) {
        _backdropImpl = value;
        dirty();
    }
}

void
LabelControl::setTextBackdropType(osgText::Text::BackdropType value)
{
    if( _backdropType != value ) {
        _backdropType = value;
        dirty();
    }
}

void 
LabelControl::setTextBackdropOffset(float offsetValue) 
{
    if ( offsetValue != _backdropOffset ) {
        _backdropOffset = offsetValue;
        dirty();
    }
}

void
LabelControl::calcSize(const ControlContext& cx, osg::Vec2f& out_size)
{
    if ( visible() == true )
    {
        // we have to create the drawable during the layout pass so we can calculate its size.
        LabelText* t = new LabelText();

        t->setText( _text, _encoding );
        // yes, object coords. screen coords won't work because the bounding box will be wrong.
        t->setCharacterSizeMode( osgText::Text::OBJECT_COORDS );
        t->setCharacterSize( _fontSize );

        // always align to top. layout alignment gets calculated layer in Control::calcPos().
        t->setAlignment( osgText::Text::LEFT_TOP ); 
        t->setColor( foreColor().value() );

        // set up the font. When you do this, OSG automatically tries to put the text object
        // in the transparent render bin. We do not want that, so we will set it back to
        // INHERIT.
        if ( _font.valid() )
            t->setFont( _font.get() );

        if ( t->getStateSet() )
            t->getStateSet()->setRenderBinToInherit();

        // set up the backdrop halo:
        if ( haloColor().isSet() )
        {
            t->setBackdropType( _backdropType );
            t->setBackdropImplementation( _backdropImpl );
            t->setBackdropOffset( _backdropOffset );
            t->setBackdropColor( haloColor().value() );
        }

        osg::BoundingBox bbox = t->getTextBB();
        if ( cx._viewContextID != ~0u )
        {
            //the Text's autoTransformCache matrix puts some mojo on the bounding box
            osg::Matrix m = t->getATMatrix( cx._viewContextID );
            _bmin = osg::Vec3( bbox.xMin(), bbox.yMin(), bbox.zMin() ) * m;
            _bmax = osg::Vec3( bbox.xMax(), bbox.yMax(), bbox.zMax() ) * m;
        }
        else
        {
            _bmin = osg::Vec3( bbox.xMin(), bbox.yMin(), bbox.zMin() );
            _bmax = osg::Vec3( bbox.xMax(), bbox.yMax(), bbox.zMax() );
        }

        _renderSize.set(
            (_bmax.x() - _bmin.x()) + padding().x(),
            (_bmax.y() - _bmin.y()) + padding().y() );

        // If width explicitly set and > measured width of label text - use it.
        if (width().isSet() && width().get() > _renderSize.x()) _renderSize.x() = width().get();

        _drawable = t;

        out_size.set(
            margin().x() + _renderSize.x(),
            margin().y() + _renderSize.y() );
    }
    else
    {
        out_size.set(0,0);
    }
}

void
LabelControl::draw( const ControlContext& cx )
{
    Control::draw( cx );

    if ( _drawable.valid() && visible() && parentIsVisible() )
    {
        float vph = cx._vp->height();

        LabelText* t = static_cast<LabelText*>( _drawable.get() );
        osg::BoundingBox bbox = t->getTextBB();
        t->setPosition( osg::Vec3( _renderPos.x(), vph - _renderPos.y(), 0 ) );
        getGeode()->addDrawable( _drawable.get() );
    }
}

// ---------------------------------------------------------------------------

ButtonControl::ButtonControl(const std::string&   text,
                             float                fontSize,
                             const osg::Vec4f&    foreColor,
                             const osg::Vec4f&    backColor,
                             const osg::Vec4f&    activeColor,
                             ControlEventHandler* handler) :
LabelControl(text, fontSize, foreColor)
{
    setBackColor( backColor );
    setActiveColor( activeColor );
    setPadding( 6.0f );
    if ( handler )
        this->addEventHandler( handler );
}

ButtonControl::ButtonControl(const std::string&   text,
                             const osg::Vec4f&    foreColor,
                             const osg::Vec4f&    backColor,
                             const osg::Vec4f&    activeColor,
                             float                fontSize,
                             ControlEventHandler* handler) :
LabelControl(text, foreColor, fontSize)
{
    setBackColor( backColor );
    setActiveColor( activeColor );
    setPadding( 6.0f );
    if ( handler )
        this->addEventHandler( handler );
}

ButtonControl::ButtonControl(const std::string&   text,
                             ControlEventHandler* handler) :
LabelControl(text)
{
    setForeColor( Color::White );
    setBackColor( Color::DarkGray );
    setActiveColor( Color::Blue );
    setPadding( 6.0f );
    if ( handler )
        this->addEventHandler( handler );
}

// ---------------------------------------------------------------------------

ImageControl::ImageControl( osg::Image* image ) :
_rotation     ( 0.0, Units::RADIANS ),
_fixSizeForRot( false ),
_opacity      ( 1.0f )
{
    setImage( image );
}

void
ImageControl::setImage( osg::Image* image )
{
    if ( image != _image.get() ) {
        _image = image;
        dirty();
    }
}

void
ImageControl::setRotation( const Angular& angle )
{
    if ( angle != _rotation ) {
        _rotation = angle;
        dirty();
    }
}

void
ImageControl::setFixSizeForRotation( bool value ) 
{
    if ( _fixSizeForRot != value ) {
        _fixSizeForRot = value;
        dirty();
    }
}

void
ImageControl::calcSize(const ControlContext& cx, osg::Vec2f& out_size)
{
    if ( visible() == true )
    {
        _renderSize.set( 0, 0 );

        //First try the explicit settings
        if (width().isSet() && height().isSet())
        {
            _renderSize.set(width().value(), height().value());
        }
        //Second try the size of the image itself
        else if (_image.valid())
        {
            _renderSize.set( _image->s(), _image->t() );
        }
        //Lastly just use the default values for width and height
        else
        {
            _renderSize.set( width().value(), height().value());
        }

        //if there's a rotation angle, rotate
        float rot = _fixSizeForRot ? osg::PI_4 : _rotation.as(Units::RADIANS);
        if ( rot != 0.0f )
        {
            calculateRotatedSize( 
                _renderSize.x(), _renderSize.y(), 
                rot,
                _renderSize.x(), _renderSize.y() );
        }
        
        out_size.set(
            margin().left() + margin().right() + _renderSize.x(),
            margin().top() + margin().bottom() + _renderSize.y() );

        //_dirty = false;
    }
    else
    {
        out_size.set(0,0);
    }
}

#undef IMAGECONTROL_TEXRECT

void
ImageControl::draw( const ControlContext& cx )
{
    Control::draw( cx );

    if ( visible() && parentIsVisible() && _image.valid() )
    {
        //TODO: this is not precisely correct..images get deformed slightly..
        osg::Geometry* g = newGeometry();

        float rx = osg::round( _renderPos.x() );
        float ry = osg::round( _renderPos.y() );
        float vph = cx._vp->height();

        osg::Vec3Array* verts = new osg::Vec3Array(4);
        g->setVertexArray( verts );

        if ( _rotation.as(Units::RADIANS) != 0.0f || _fixSizeForRot == true )
        {
            osg::Vec2f rc( rx+_renderSize.x()/2, (vph-ry)-_renderSize.y()/2 );
            float ra = osg::PI - _rotation.as(Units::RADIANS);

            rx += 0.5*_renderSize.x() - 0.5*(float)_image->s();
            ry += 0.5*_renderSize.y() - 0.5*(float)_image->t();

            rot( rx, vph-ry, rc, ra, (*verts)[0] );
            rot( rx, vph-ry-_image->t(), rc, ra, (*verts)[1] );
            rot( rx+_image->s(), vph-ry-_image->t(), rc, ra, (*verts)[2] );
            rot( rx+_image->s(), vph-ry, rc, ra, (*verts)[3] );
        }
        else
        {
            (*verts)[0].set( rx, vph - ry, 0 );
            (*verts)[1].set( rx, vph - ry - _renderSize.y(), 0 );
            (*verts)[2].set( rx + _renderSize.x(), vph - ry - _renderSize.y(), 0 );
            (*verts)[3].set( rx + _renderSize.x(), vph - ry, 0 );
        }

        g->addPrimitiveSet( new osg::DrawArrays( GL_QUADS, 0, 4 ) );

        osg::Vec4Array* c = new osg::Vec4Array(1);
        (*c)[0] = osg::Vec4f(1,1,1,1);
        g->setColorArray( c );
        g->setColorBinding( osg::Geometry::BIND_OVERALL );

        bool flip = _image->getOrigin()==osg::Image::TOP_LEFT;

        osg::Vec2Array* t = new osg::Vec2Array(4);

#ifdef IMAGECONTROL_TEXRECT

        (*t)[0].set( 0, flip? 0: _image->t()-1 );
        (*t)[1].set( 0, flip? _image->t()-1: 0 );
        (*t)[2].set( _image->s()-1, flip? _image->t()-1: 0 );
        (*t)[3].set( _image->s()-1, flip? 0: _image->t()-1 );
        osg::TextureRectangle* tex = new osg::TextureRectangle( _image.get() );

#else

        (*t)[0].set( 0, flip? 0 : 1 );
        (*t)[1].set( 0, flip? 1 : 0 );
        (*t)[2].set( 1, flip? 1 : 0 );
        (*t)[3].set( 1, flip? 0 : 1 );
        osg::Texture2D* tex = new osg::Texture2D( _image.get() );
#endif

        g->setTexCoordArray( 0, t );

        tex->setResizeNonPowerOfTwoHint(false);

        tex->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR );
        tex->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
        g->getOrCreateStateSet()->setTextureAttributeAndModes( 0, tex, osg::StateAttribute::ON );

        osg::TexEnv* texenv = new osg::TexEnv( osg::TexEnv::MODULATE );
        g->getStateSet()->setTextureAttributeAndModes( 0, texenv, osg::StateAttribute::ON );

        getGeode()->addDrawable( g );

        _dirty = false;
    }
}

// ---------------------------------------------------------------------------

HSliderControl::HSliderControl( float min, float max, float value, ControlEventHandler* handler) :
_min(min),
_max(max),
_value(value)
{
    setHorizFill( true );
    setVertAlign( ALIGN_CENTER );
    setHeight( 20.0f );

    if ( handler )
        addEventHandler( handler );
}

void
HSliderControl::fireValueChanged( ControlEventHandler* oneHandler )
{
    if ( oneHandler )
    {
        oneHandler->onValueChanged( this, _value );
    }
    else
    {
        for( ControlEventHandlerList::const_iterator i = _eventHandlers.begin(); i != _eventHandlers.end(); ++i )
        {
            i->get()->onValueChanged( this, _value );
        }
    }
}

void
HSliderControl::setValue( float value, bool notify )
{
    if ( value != _value )
    {
        _value = value;
        if ( notify )
            fireValueChanged();
        dirty();
    }
}

void
HSliderControl::setMin( float min, bool notify )
{
    if ( min != _min )
    {
        _min = min;
        if ( _min >= _max )
            _max = _min+1.0f;

        if ( _value < _min || _value > _max ) 
        {
            _value = _min;
            if ( notify )
                fireValueChanged();
        }
        dirty();
    }
}

void
HSliderControl::setMax( float max, bool notify )
{
    if ( max != _max )
    {
        _max = max;
        if ( _max <= _min )
            _max = _min+1.0f;

        if ( _value < _min || _value > _max )
        {
            _value = _max;
            if ( notify )
                fireValueChanged();
        }
        dirty();
    }
}

void
HSliderControl::draw( const ControlContext& cx )
{
    Control::draw( cx );

    if ( visible() && parentIsVisible())
    {
        osg::ref_ptr<osg::Geometry> g = newGeometry();

        float rx = osg::round( _renderPos.x() );
        float ry = osg::round( _renderPos.y() );
        float rw = osg::round( _renderSize.x() - padding().x() );
        float rh = osg::round( _renderSize.y() - padding().y() );

        if ( rw > 0.0f && rh > 0.0f )
        {
            float vph = cx._vp->height();

            osg::Vec3Array* verts = new osg::Vec3Array(8);
            g->setVertexArray( verts );

            (*verts)[0].set( rx, vph - ry, 0 );
            (*verts)[1].set( rx, vph - (ry + rh), 0 );
            (*verts)[2].set( rx + rw, vph - (ry + rh), 0 );
            (*verts)[3].set( rx + rw, vph - ry, 0 );
            g->addPrimitiveSet( new osg::DrawArrays( GL_LINE_LOOP, 0, 4 ) );

            float hx = rx + rw * ( (_value-_min)/(_max-_min) );

            (*verts)[4].set( hx-4, vph - ry + 3, 0 );
            (*verts)[5].set( hx-4, vph - (ry + rh + 3), 0 );
            (*verts)[6].set( hx+4, vph - (ry + rh + 3), 0 );
            (*verts)[7].set( hx+4, vph - ry + 3, 0 );
            g->addPrimitiveSet( new osg::DrawArrays( GL_QUADS, 4, 4 ) );

            osg::Vec4Array* c = new osg::Vec4Array(1);
            (*c)[0] = *foreColor();
            g->setColorArray( c );
            g->setColorBinding( osg::Geometry::BIND_OVERALL );

            getGeode()->addDrawable( g.get() );
        }
    }
}

bool
HSliderControl::handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa, ControlContext& cx )
{
    if( !visible() || !parentIsVisible() )
        return false;

    if ( ea.getEventType() == osgGA::GUIEventAdapter::DRAG )
    {
        float canvasX = ea.getX() - cx._view->getCamera()->getViewport()->x();
        float relX = canvasX - _renderPos.x();

        if ( _min < _max )
            setValue( osg::clampBetween(_min + (_max-_min) * ( relX/_renderSize.x() ), _min, _max) );
        else
            setValue( osg::clampBetween(_min - (_min-_max) * ( relX/_renderSize.x() ), _max, _min) );

        aa.requestRedraw();

        return true;
    }
    return Control::handle( ea, aa, cx );
}

// ---------------------------------------------------------------------------

CheckBoxControl::CheckBoxControl( bool value ) :
_value( value )
{
    setWidth( 16 );
    setHeight( 16 );
}

CheckBoxControl::CheckBoxControl( bool value, ControlEventHandler* handler ) :
_value( value )
{
    this->addEventHandler( handler );
    setWidth( 16 );
    setHeight( 16 );
}

void
CheckBoxControl::fireValueChanged( ControlEventHandler* oneHandler )
{
    if ( oneHandler )
    {
        oneHandler->onValueChanged( this, _value );
    }
    else
    {
        for( ControlEventHandlerList::const_iterator i = _eventHandlers.begin(); i != _eventHandlers.end(); ++i )
        {
            i->get()->onValueChanged( this, _value );
        }
    }
}

void
CheckBoxControl::setValue( bool value )
{
    if ( value != _value )
    {
        _value = value;
        fireValueChanged();
        dirty();
    }
}

void
CheckBoxControl::draw( const ControlContext& cx )
{
    Control::draw( cx );

    if ( visible() && parentIsVisible() )
    {
        osg::Geometry* g = newGeometry();

        float rx = osg::round( _renderPos.x() );
        float ry = osg::round( _renderPos.y() );
        float rw = _renderSize.x() - padding().x();
        float rh = _renderSize.y() - padding().y();
        float vph = cx._vp->height(); // - padding().bottom();

        osg::Vec3Array* verts = new osg::Vec3Array(4);
        g->setVertexArray( verts );

        (*verts)[0].set( rx, vph - ry, 0 );
        (*verts)[1].set( rx + rw, vph - ry, 0 );
        (*verts)[2].set( rx + rw, vph - (ry + rh), 0 );
        (*verts)[3].set( rx, vph - (ry + rh), 0 );

        g->addPrimitiveSet( new osg::DrawArrays( GL_LINE_LOOP, 0, 4 ) );

        if ( _value )
        {
            osg::DrawElementsUByte* e = new osg::DrawElementsUByte( GL_LINES );
            e->push_back( 0 );
            e->push_back( 2 );
            e->push_back( 1 );
            e->push_back( 3 );
            g->addPrimitiveSet( e );
        }

        osg::Vec4Array* c = new osg::Vec4Array(1);
        (*c)[0] = *foreColor();
        g->setColorArray( c );
        g->setColorBinding( osg::Geometry::BIND_OVERALL );

        getGeode()->addDrawable( g );
    }
}

bool
CheckBoxControl::handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa, ControlContext& cx )
{
    if( !visible() || !parentIsVisible() )
        return false;

    if ( ea.getEventType() == osgGA::GUIEventAdapter::PUSH )
    {
        setValue( !_value );
        aa.requestRedraw();
        return true;
    }
    return Control::handle( ea, aa, cx );
}

// ---------------------------------------------------------------------------

Frame::Frame()
{
    setPadding( 0 );
}

void
Frame::calcPos(const ControlContext& context, const osg::Vec2f& cursor, const osg::Vec2f& parentSize)
{
    _renderPos = cursor;
}

void
Frame::draw( const ControlContext& cx )
{
    if ( !getImage() || getImage()->s() != _renderSize.x() || getImage()->t() != _renderSize.y() )
    {
        // a simple colored border frame
        osg::ref_ptr<Geometry> geom = new Ring();
        geom->push_back( osg::Vec3d( 0, 0, 0 ) );
        geom->push_back( osg::Vec3d( _renderSize.x()-1, 0, 0 ) );
        geom->push_back( osg::Vec3d( _renderSize.x()-1, _renderSize.y()-1, 0 ) );
        geom->push_back( osg::Vec3d( 0, _renderSize.y()-1, 0 ) );

        Style style;
        LineSymbol* line = style.getOrCreate<LineSymbol>();
        line->stroke()->color() = Color::White;
        line->stroke()->width() = 2.5f;
        GeometryRasterizer ras( (int)_renderSize.x(), (int)_renderSize.y(), style );
        ras.draw( geom.get() );

        osg::Image* image = ras.finalize();
        const_cast<Frame*>(this)->setImage( image );
    }

    Control::draw( cx );       // draws the background
    ImageControl::draw( cx );  // draws the border
}

// ---------------------------------------------------------------------------

RoundedFrame::RoundedFrame()
{
    //nop
}

void
RoundedFrame::draw( const ControlContext& cx )
{
    if ( Geometry::hasBufferOperation() )
    {
        if ( !getImage() || getImage()->s() != _renderSize.x() || getImage()->t() != _renderSize.y() )
        {
            // create a rounded rectangle by buffering a rectangle. "buffer" value affects how rounded
            // the corners are.
            float buffer = Geometry::hasBufferOperation() ? 10.0f : 0.0f;

            osg::ref_ptr<Geometry> geom = new Polygon();
            geom->push_back( osg::Vec3d( buffer, buffer, 0 ) );
            geom->push_back( osg::Vec3d( _renderSize.x()-1-buffer, buffer, 0 ) );
            geom->push_back( osg::Vec3d( _renderSize.x()-1-buffer, _renderSize.y()-1-buffer, 0 ) );
            geom->push_back( osg::Vec3d( buffer, _renderSize.y()-1-buffer, 0 ) );

            BufferParameters bp;
            bp._capStyle = BufferParameters::CAP_ROUND;
            geom->buffer( buffer-1.0f, geom, bp );

            GeometryRasterizer ras( (int)_renderSize.x(), (int)_renderSize.y() );
            ras.draw( geom.get(), backColor().value() );

            osg::Image* image = ras.finalize();
            const_cast<RoundedFrame*>(this)->setImage( image );
        }

        ImageControl::draw( cx );
    }
    else
    {
        // fallback: draw a non-rounded frame.
        Frame::draw( cx );
    }
}

// ---------------------------------------------------------------------------

Container::Container() :
_spacing( 5.0f )
{
    //nop
}

Container::Container( const Alignment& halign, const Alignment& valign, const Gutter& padding, float spacing )
: Control( halign, valign, padding )
{
    this->setChildSpacing( spacing );
}

void
Container::getChildren(std::vector<Control*>& out)
{
    for(unsigned i=1; i<getNumChildren(); ++i )
    {
        Control* c = dynamic_cast<Control*>( getChild(i) );
        if ( c ) out.push_back( c );
    }
}

void
Container::setChildSpacing( float value )
{
    if ( value != _spacing ) {
        _spacing = value;
        dirty();
    }
}

void
Container::setChildHorizAlign( Alignment value )
{
    if ( !_childhalign.isSet() || _childhalign != value )
    {
        _childhalign = value;
        applyChildAligns();
    }
}

void
Container::setChildVertAlign( Alignment value )
{
    if ( !_childvalign.isSet() || _childvalign != value )
    {
        _childvalign = value;
        applyChildAligns();
    }
}

void
Container::applyChildAligns()
{
    if ( _childhalign.isSet() || _childvalign.isSet() )
    {
        std::vector<Control*> children;
        getChildren( children );
        for( std::vector<Control*>::iterator i = children.begin(); i != children.end(); ++i )
        {
            Control* child = (*i);

            if ( _childvalign.isSet() && !child->vertAlign().isSet() )
                child->setVertAlign( *_childvalign );

            if ( _childhalign.isSet() && !child->horizAlign().isSet() )
                child->setHorizAlign( *_childhalign );
        }
        dirty();
    }
}

void
Container::calcSize(const ControlContext& cx, osg::Vec2f& out_size)
{
    if ( visible() == true )
    {
        float w = width().isSet()  ? std::max( width().value(),  _renderSize.x() ) : _renderSize.x();
        float h = height().isSet() ? std::max( height().value(), _renderSize.y() ) : _renderSize.y();

        _renderSize.set(
            w + padding().x(),
            h + padding().y() );

        out_size.set(
            _renderSize.x() + margin().x(),
            _renderSize.y() + margin().y() );
    }
}

void
Container::calcFill(const ControlContext& cx)
{
    for( unsigned i=1; i<getNumChildren(); ++i )
    {
        Control* child = dynamic_cast<Control*>( getChild(i) );
        if ( child )
        {
            child->calcFill( cx );
        }
    }
}

void
Container::calcPos(const ControlContext& context, const osg::Vec2f& cursor, const osg::Vec2f& parentSize)
{
    Control::calcPos( context, cursor, parentSize );
}

void
Container::draw( const ControlContext& cx )
{
    Control::draw( cx );
}

bool
Container::handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa, ControlContext& cx )
{
    if( !visible() || !parentIsVisible() )
        return false;

    bool handled = false;

    float canvasY = cx._vp->height() - (ea.getY() - cx._view->getCamera()->getViewport()->y());
    float canvasX = ea.getX() - cx._view->getCamera()->getViewport()->x();

    std::vector<Control*> children;
    getChildren( children );
    //OE_NOTICE << "handling " << children.size() << std::endl;
    for( std::vector<Control*>::reverse_iterator i = children.rbegin(); i != children.rend(); ++i )
    {
        Control* child = *i;
        //Control* child = dynamic_cast<Control*>( getChild(i) );
        if ( child )
        {
            if (ea.getEventType() == osgGA::GUIEventAdapter::FRAME || child->intersects( canvasX, canvasY ) )
                handled = child->handle( ea, aa, cx );
            if ( handled )
                break;
        }
    }

    return handled ? handled : Control::handle( ea, aa, cx );
}

void
Container::addControls( const ControlVector& controls )
{
    for( ControlVector::const_iterator i = controls.begin(); i != controls.end(); ++i )
    {
        addControl( i->get() );
    }
}






void Container::setVisible(bool visibility) 
{ 
        Control::setVisible(visibility); 
        std::vector<osgEarth::Util::Controls::Control*> out; 
        getChildren(out); 
        for (int i = 0; i < (int) out.size(); i++) 
        { 
                Container* container = dynamic_cast<Container*>( out.at(i) ); 
                if (container) { 
                        container->setVisible(visibility); 
                } else { 
                        out.at(i)->setVisible(visibility); 
                } 
        } 
}

// ---------------------------------------------------------------------------

VBox::VBox()
{
    //nop
}

VBox::VBox( const Alignment& halign, const Alignment& valign, const Gutter& padding, float spacing ) :
Container( halign, valign, padding, spacing )
{
    //nop
}

Control*
VBox::addControlImpl( Control* control, int index )
{
    insertChild( index, control );
    applyChildAligns();
    dirty();
    return control;
}

void
VBox::clearControls()
{
    removeChildren( 1, getNumChildren()-1 );
    dirty();
}

void
VBox::calcSize(const ControlContext& cx, osg::Vec2f& out_size)
{
    if ( visible() )
    {
        _renderSize.set( 0, 0 );

        // collect all the members, growing the container size vertically
        for( unsigned i=1; i<getNumChildren(); ++i )
        {
            Control* child = dynamic_cast<Control*>( getChild(i) );
            if ( child )
            {
                osg::Vec2f childSize;
                bool first = i == 1; //_controls.begin();

                child->calcSize( cx, childSize );

                _renderSize.x() = osg::maximum( _renderSize.x(), childSize.x() );
                _renderSize.y() += first ? childSize.y() : childSpacing() + childSize.y();
            }
        }

        Container::calcSize( cx, out_size );
    }
    else
    {
        out_size.set(0,0);
    }
}

void
VBox::calcFill(const ControlContext& cx)
{
    float used_x = padding().x();
    float used_y = padding().y() - childSpacing();

    Control* hc = 0L;
    Control* vc = 0L;

    for( unsigned i=1; i<getNumChildren(); ++i )
    {
        Control* child = dynamic_cast<Control*>( getChild(i) );
        if ( child )
        {
            used_y += child->margin().y() + childSpacing();
            if ( !hc && child->horizFill() )
            {
                hc = child;
                used_x += child->margin().x();
            }

            if ( !vc && child->vertFill() )
                vc = child;
            else
                used_y += child->renderSize().y();
        }
    }

    if ( hc && renderWidth(hc) < (_renderSize.x() - used_x) )
        renderWidth(hc) = _renderSize.x() - used_x;

    if ( vc && renderHeight(vc) < (_renderSize.y() - used_y) )
        renderHeight(vc) = _renderSize.y() - used_y;
   
    Container::calcFill( cx );
}

void
VBox::calcPos(const ControlContext& cx, const osg::Vec2f& cursor, const osg::Vec2f& parentSize)
{
    Container::calcPos( cx, cursor, parentSize );

    osg::Vec2f childCursor = _renderPos;

    osg::Vec2f renderArea = _renderSize - padding().size();

    for( unsigned i=1; i<getNumChildren(); ++i )
    {
        Control* child = dynamic_cast<Control*>( getChild(i) );
        if ( child )
        {
            child->calcPos( cx, childCursor, renderArea ); // GW1
            float deltaY = child->margin().top() + child->renderSize().y() + child->margin().bottom() + childSpacing();
            childCursor.y() += deltaY;
            renderArea.y() -= deltaY;
        }
    }
}

void
VBox::draw( const ControlContext& cx )
{
   
        Container::draw( cx );

        for( unsigned i=1; i<getNumChildren(); ++i )
        {
            Control* c = dynamic_cast<Control*>(getChild(i));
            if ( c )
                c->draw( cx );
        }
     
}

// ---------------------------------------------------------------------------

HBox::HBox()
{
    //nop
}

HBox::HBox( const Alignment& halign, const Alignment& valign, const Gutter& padding, float spacing ) :
Container( halign, valign, padding, spacing )
{
    //nop
}

Control*
HBox::addControlImpl( Control* control, int index )
{
    insertChild(index, control);
    applyChildAligns();
    dirty();
    return control;
}

void
HBox::clearControls()
{
    removeChildren(1, getNumChildren()-1);
    dirty();
}

void
HBox::calcSize(const ControlContext& cx, osg::Vec2f& out_size)
{
    if ( visible() )
    {
        _renderSize.set( 0, 0 );

        // collect all the members, growing the container is its orientation.
        for( unsigned i=1; i<getNumChildren(); ++i )
        {
            Control* child = dynamic_cast<Control*>( getChild(i) );
            if ( child )
            {
                osg::Vec2f childSize;
                bool first = i == 1;

                child->calcSize( cx, childSize );

                _renderSize.x() += first ? childSize.x() : childSpacing() + childSize.x();
                _renderSize.y() = osg::maximum( _renderSize.y(), childSize.y() );
            }
        }
    
        // If width explicitly set and > total width of children - use it
        if (width().isSet() && width().get() > _renderSize.x()) _renderSize.x() = width().get();

        Container::calcSize( cx, out_size );
    }
    else
    {
        out_size.set(0,0);
    }
}

void
HBox::calcFill(const ControlContext& cx)
{
    float used_x = padding().x() - childSpacing();
    float used_y = padding().y();

    Control* hc = 0L;
    Control* vc = 0L;

    for( unsigned i=1; i<getNumChildren(); ++i )
    {
        Control* child = dynamic_cast<Control*>( getChild(i) );
        if ( child )
        {
            used_x += child->margin().x() + childSpacing();
            if ( !hc && child->horizFill() )
                hc = child;
            else
                used_x += child->renderSize().x();

            if ( !vc && child->vertFill() )
            {
                vc = child;
                used_y += child->margin().y();
            }
        }
    }

    if ( hc && renderWidth(hc) < (_renderSize.x() - used_x) )
        renderWidth(hc) = _renderSize.x() - used_x;

    if ( vc && renderHeight(vc) < (_renderSize.y() - used_y) )
        renderHeight(vc) = _renderSize.y() - used_y;
   
    Container::calcFill( cx );
}

void
HBox::calcPos(const ControlContext& cx, const osg::Vec2f& cursor, const osg::Vec2f& parentSize)
{
    Container::calcPos( cx, cursor, parentSize );

    osg::Vec2f childCursor = _renderPos;

    osg::Vec2f renderArea = _renderSize - padding().size();

    for( unsigned i=1; i<getNumChildren(); ++i )
    {
        Control* child = dynamic_cast<Control*>( getChild(i) );
        if ( child )
        {
            child->calcPos( cx, childCursor, renderArea );
            float deltaX = child->margin().left() + child->renderSize().x() + child->margin().right() + childSpacing();
            childCursor.x() += deltaX;
            renderArea.x() -= deltaX;
        }
    }
}

void
HBox::draw( const ControlContext& cx )
{
    
        Container::draw( cx );

        for( unsigned i=1; i<getNumChildren(); ++i )
        {
            Control* c = dynamic_cast<Control*>(getChild(i));
            if ( c )
                c->draw( cx );
        }
    
}

// ---------------------------------------------------------------------------

Grid::Grid() :
Container(),
_maxCols(0)
{
    setChildHorizAlign( ALIGN_LEFT );
    setChildVertAlign( ALIGN_CENTER );
}

Grid::Grid( const Alignment& halign, const Alignment& valign, const Gutter& padding, float spacing ) :
Container( halign, valign, padding, spacing ),
_maxCols(0)
{
    //nop
}

void
Grid::getChildren(std::vector<Control*>& out)
{
    for(unsigned i=1; i<getNumChildren(); ++i )
    {
        osg::Group* row = getChild(i)->asGroup();
        if ( row )
        {
            for( unsigned j=0; j<row->getNumChildren(); ++j )
            {
                Control* c = dynamic_cast<Control*>( row->getChild(j) );
                if ( c ) out.push_back( c );
            }
        }
    }
}

unsigned
Grid::getNumRows() const
{
    return getNumChildren()-1;
}

unsigned
Grid::getNumColumns() const
{
    if ( getNumRows() == 0 )
        return 0;
    else
        return const_cast<Grid*>(this)->getRow(0)->getNumChildren();
}

osg::Group*
Grid::getRow(unsigned index)
{
    return getNumChildren() >= 2+index ? getChild(1+index)->asGroup() : 0L;
}

Control*
Grid::setControlImpl( int col, int row, Control* child )
{
    if ( child )
    {
        expandToInclude( col, row );
        osg::Group* rowGroup = getRow(row);
        rowGroup->setChild( col, child );
        applyChildAligns();

        dirty();
    }

    return child;
}

Control*
Grid::getControl(int col, int row)
{
    if ( row < (int)getNumChildren()+1 )
    {
        osg::Group* rowGroup = getRow(row);
        if ( col < (int)rowGroup->getNumChildren() )
        {
            return dynamic_cast<Control*>( rowGroup->getChild(col) );
        }
    }
    return 0L;
}

void
Grid::expandToInclude( int col, int row )
{
    // ensure all rows have sufficient columns:
    if ( col+1 > (int)_maxCols )
    {
        _maxCols = col+1;
    }

    // and that we have sufficient rows:
    unsigned maxRows = std::max( (unsigned)getNumRows(), (unsigned)(row+1) );

    // expand everything and use empty groups as placeholders
    for( unsigned r=0; r<maxRows; ++r )
    {
        osg::Group* rowGroup = getRow(r);
        if ( !rowGroup )
        {
            rowGroup = new osg::Group();
            addChild( rowGroup );
        }
        while ( rowGroup->getNumChildren() < _maxCols )
        {
            rowGroup->addChild( new osg::Group() );
        }
    }
}

Control*
Grid::addControlImpl( Control* control, int index )
{
    // creates a new row and puts the control in its first column (index is ignored)
    return setControlImpl( 0, getNumRows(), control );
}

void
Grid::addControls( const ControlVector& controls )
{
    unsigned row = getNumRows();
    unsigned col = 0;
    for( ControlVector::const_iterator i = controls.begin(); i != controls.end(); ++i, ++col )
    {
        if ( i->valid() )
        {
            setControlImpl( col, row, i->get() );
        }
    }
}

void
Grid::clearControls()
{
    removeChildren(1, getNumChildren()-1);
    dirty();
}

void
Grid::calcSize( const ControlContext& cx, osg::Vec2f& out_size )
{
    
        _renderSize.set( 0, 0 );

        int nRows = (int)getNumRows();
        int nCols = (int)getNumColumns();

        _rowHeights.assign( nRows, 0.0f );
        _colWidths.assign ( nCols, 0.0f );

        if ( nRows > 0 && nCols > 0 )
        {
            for( int r=0; r<nRows; ++r )
            { 
                for( int c=0; c<nCols; ++c )
                {
                    Control* child = getControl(c, r);
                    if ( child )
                    {
                        osg::Vec2f childSize;
                        child->calcSize( cx, childSize );

                        if ( childSize.x() > _colWidths[c] )
                            _colWidths[c] = childSize.x();
                        if ( childSize.y() > _rowHeights[r] )
                            _rowHeights[r] = childSize.y();
                    }
                }
            }

            for( int c=0; c<nCols; ++c )
                _renderSize.x() += _colWidths[c];
            _renderSize.x() += childSpacing() * (nCols-1);

            for( int r=0; r<nRows; ++r )
                _renderSize.y() += _rowHeights[r];
            _renderSize.y() += childSpacing() * (nRows-1);
        }

        Container::calcSize( cx, out_size );
    
}

void
Grid::calcFill(const ControlContext& cx)
{
    Container::calcFill( cx );

    int nRows = (int)getNumRows();
    int nCols = (int)getNumColumns();

    for( int r=0; r<nRows; ++r )
    {
        for( int c=0; c<nCols; ++c )
        {
            Control* child = getControl(c, r);

            if ( child )
            {
                if ( child->horizFill() )
                    renderWidth(child) = _colWidths[c] - child->margin().x();
                if ( child->vertFill() )
                    renderHeight(child) = _rowHeights[r] - child->margin().y();
            }
        }
    }
}

void
Grid::calcPos( const ControlContext& cx, const osg::Vec2f& cursor, const osg::Vec2f& parentSize )
{
    Container::calcPos( cx, cursor, parentSize );

    int nRows = (int)getNumRows();
    int nCols = (int)getNumColumns();

    osg::Vec2f childCursor = _renderPos;

    for( int r=0; r<nRows; ++r )
    {
        for( int c=0; c<nCols; ++c )
        {
            Control* child = getControl(c, r);
            if ( child )
            {
                osg::Vec2f cellSize( _colWidths[c], _rowHeights[r] );
                child->calcPos( cx, childCursor, cellSize );
            }
            childCursor.x() += _colWidths[c] + childSpacing();
        }
        childCursor.x() = _renderPos.x();
        childCursor.y() += _rowHeights[r] + childSpacing();
    }
}





void
Grid::draw( const ControlContext& cx )
{
    
        Container::draw( cx );

        for( unsigned i=1; i<getNumChildren(); ++i )
        {
            osg::Group* rowGroup = getChild(i)->asGroup();
            if ( rowGroup )
            {
                for( unsigned j=0; j<rowGroup->getNumChildren(); ++j )
                {
                    Control* c = dynamic_cast<Control*>( rowGroup->getChild(j) );
                    if ( c )
                    {
                        c->draw( cx );
                    }
                }
            }
        }
 
}

// ---------------------------------------------------------------------------

ControlCanvas::EventCallback::EventCallback(ControlCanvas* canvas) : 
_canvas   ( canvas ),
_firstTime( true ),
_width    ( 0 ),
_height   ( 0 )
{
    //nop
}

// version helper.
#if OSG_VERSION_GREATER_THAN(3,3,0)
#   define AS_ADAPTER(e) e->asGUIEventAdapter()
#else
#   define AS_ADAPTER(e) e
#endif

void
ControlCanvas::EventCallback::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
    osgGA::EventVisitor* ev = static_cast<osgGA::EventVisitor*>(nv);

    osg::ref_ptr<ControlCanvas> canvas;
    if ( _canvas.lock(canvas) )
    {
        const osgGA::EventQueue::Events& events = ev->getEvents();
        if ( events.size() > 0 )
        {
            osg::ref_ptr<ControlCanvas> canvas;
            if ( _canvas.lock(canvas) )
            {
                osgGA::GUIActionAdapter* aa = ev->getActionAdapter();

                for(osgGA::EventQueue::Events::const_iterator e = events.begin(); e != events.end(); ++e)
                {
                    osgGA::GUIEventAdapter* ea = AS_ADAPTER(e->get());

                    // check for a resize each frame. Don't rely on the RESIZE event;
                    // it does always convey the new viewport dimensions (they aren't
                    // always available until the following FRAME event)
                    if ( ea->getEventType() == ea->FRAME )
                    {
                        handleResize(aa->asView(), canvas.get());
                    }

                    if (canvas->handle( *ea, *aa ))
                    {
                        e->get()->setHandled(true);
                    }
                }
            }
        }
    }

    traverse(node,nv);
}

void 
ControlCanvas::EventCallback::handleResize(osg::View* view, ControlCanvas* canvas)
{
    osg::Camera* cam = view->getCamera();

    if ( cam && cam->getViewport() )
    {
        const osg::Viewport* vp = cam->getViewport();
        if ( _firstTime || vp->width() != _width || vp->height() != _height )
        {
            canvas->setProjectionMatrix(osg::Matrix::ortho2D( 0, vp->width()-1, 0, vp->height()-1 ) );

            ControlContext cx;
            cx._view = view;
            cx._vp = new osg::Viewport( 0, 0, vp->width(), vp->height() );

            osg::GraphicsContext* gc = view->getCamera()->getGraphicsContext();
            if ( !gc && view->getNumSlaves() > 0 )
                gc = view->getSlave(0)._camera->getGraphicsContext();

            if ( gc )
                cx._viewContextID = gc->getState()->getContextID();
            else
                cx._viewContextID = ~0u;

            canvas->setControlContext( cx );

            _width  = (int)vp->width();
            _height = (int)vp->height();
        }

        if ( vp->width() != 0 && vp->height() != 0 )
        {
            _firstTime = false;
        }
    }
}

// ---------------------------------------------------------------------------

ControlNode::ControlNode(Control* control, float priority ) :
_control ( control ),
_priority( priority )
{
    setCullingActive( false );
}

osg::BoundingSphere
ControlNode::computeBound() const
{
    return osg::BoundingSphere( osg::Vec3(0,0,0), 0.5 );
}

void
ControlNode::traverse( osg::NodeVisitor& nv )
{
    if ( nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
    {
        static osg::Vec3d s_zero(0,0,0);
        static osg::Vec4d s_zero_w(0,0,0,1);
        osgUtil::CullVisitor* cv = Culling::asCullVisitor(nv);

        // pull up the per-view data for this view:
        TravSpecificData& data = _travDataMap[cv->getCurrentCamera()];

        // if it's uninitialized, find the corresponding control canvas and 
        // cache a reference to its control node bin:
        if ( !data._canvas.valid() )
        {
            data._canvas = osgEarth::findTopMostNodeOfType<ControlCanvas>( cv->getCurrentCamera() );
            if ( data._canvas.valid() )
            {
                ControlNodeBin* bin = static_cast<ControlCanvas*>(data._canvas.get())->getControlNodeBin();
                bin->addNode( this );
            }
        }

        if ( data._canvas.valid() )
        {
            // calculate its screen position:
            osg::Vec4d clip = s_zero_w * (*cv->getModelViewMatrix()) * (*cv->getProjectionMatrix());
            osg::Vec3d clip_ndc( clip.x()/clip.w(), clip.y()/clip.w(), clip.z()/clip.w() );
            data._screenPos = clip_ndc * cv->getWindowMatrix();

            if ( clip_ndc.z() > 1.0 ) // node is behind the near clip plane
            {
                data._obscured = true;
            }
            else if ( data._obscured == true )
            {
                data._obscured = false;
                data._visibleTime = cv->getFrameStamp()->getReferenceTime();
            }
        }

        data._visitFrame = cv->getFrameStamp()->getFrameNumber();
    }

    // ControlNode has no children, so no point in calling traverse.
    osg::Node::traverse(nv);
}

ControlNode::TravSpecificData::TravSpecificData() :
_obscured   ( true ),
_visibleTime( 0.0 ),
_screenPos  ( 0.0, 0.0, 0.0 ),
_visitFrame(0)
{
    //nop
}

// ---------------------------------------------------------------------------

/**
 * A custom render bin for Controls, that sorts drawables by traversal order,
 * providing an unambiguous draw order.
 */
#define OSGEARTH_CONTROLS_BIN "osgEarth::Utils::Controls::bin"

namespace
{    
    struct osgEarthControlsRenderBin : public osgUtil::RenderBin
    {
        osgEarthControlsRenderBin()
        {
            this->setName( OSGEARTH_CONTROLS_BIN );
            this->setSortMode( osgUtil::RenderBin::TRAVERSAL_ORDER );
        }
    };
}

static osgEarthRegisterRenderBinProxy<osgEarthControlsRenderBin> s_regbin( OSGEARTH_CONTROLS_BIN );

// ---------------------------------------------------------------------------

ControlNodeBin::ControlNodeBin() :
_sortingEnabled( true ),
_sortByDistance( true ),
_fading        ( true )
{
    _group = new Group();

    osg::StateSet* stateSet = _group->getOrCreateStateSet();

    //TODO: appears to be unused
    osg::Uniform* defaultOpacity = new osg::Uniform( osg::Uniform::FLOAT, "oe_controls_opacity" );
    defaultOpacity->set( 1.0f );
    stateSet->addUniform( defaultOpacity );

    osg::Uniform* defaultVisibleTime = new osg::Uniform( osg::Uniform::FLOAT, "oe_controls_visibleTime" );
    defaultVisibleTime->set( 0.0f );
    stateSet->addUniform( defaultVisibleTime );    
}

void
ControlNodeBin::setFading( bool value )
{
    _fading = value;
}

void
ControlNodeBin::draw( const ControlContext& context, bool newContext, int bin )
{
    const osg::Viewport* vp = context._vp.get();
    osg::Vec2f surfaceSize( context._vp->width(), context._vp->height() );

    // we don't really need to keep this list in the object, but that prevents it from having to
    // reallocate it each time
    _taken.clear();

    ControlNodeCollection* drawList = 0L;
    ControlNodeCollection byDepth;

    if ( _sortingEnabled && _sortByDistance )
    {
        for( ControlNodeCollection::iterator i = _controlNodes.begin(); i != _controlNodes.end();)
        {
            ControlNode* node = i->second.get();
            if ( node->getNumParents() == 0 )
            {
              // Save the iterator and erase it before we increment it.  erase will invalidate i so we can't increment it directly.
              ControlNodeCollection::iterator saveItr = i;
              ++saveItr;
              _renderNodes.erase( node );
              // Erase the current iterator
              _controlNodes.erase( i );
              // Assign i to the incremented iterator
              i = saveItr;
            }
            else
            {
                ControlNode::TravSpecificData& nodeData = node->getData( context._view->getCamera() );
                byDepth.insert( ControlNodePair(nodeData._screenPos.z(), node) );
                i++;
            }
        }

        drawList = &byDepth;
    }
    else
    {
        drawList = &_controlNodes;
    }

    for( ControlNodeCollection::iterator i = drawList->begin(); i != drawList->end(); ) 
    {
        ControlNode* node = i->second.get();
        osg::MatrixTransform* xform = _renderNodes[node];

        // check to see if the node as removed
        bool nodeActive = node->getNumParents() > 0;

        if ( nodeActive )
        {
          ControlNode::TravSpecificData& nodeData = node->getData( context._view->getCamera() );
          Control* control = node->getControl();

          // if the context changed (e.g., viewport resize), we need to mark all nodes as dirty
          // even if they're obscured...that way they will regenerate properly next time
          if ( newContext )
          {
              control->dirty();
          }

          bool visible = true;

          if ( context._frameStamp->getFrameNumber() - nodeData._visitFrame > 2 )
          {
              visible = false;
          }

          else if ( nodeData._obscured == false )
          {
              const osg::Vec3f& nPos = nodeData._screenPos;
              const osg::Vec2f& size = control->renderSize();

              // calculate the rendering offset based on alignment:
              float x = 0.f, y = 0.f;

              if ( node->anchorPoint().isSet() )
              {
                  //TODO!!
              }
              else
              {
                  x =
                    control->horizAlign() == Control::ALIGN_LEFT  ? nPos.x() - size.x() :
                    control->horizAlign() == Control::ALIGN_RIGHT ? nPos.x() :
                    nPos.x() - size.x()*0.5;

                  y =
                    control->vertAlign() == Control::ALIGN_BOTTOM ? nPos.y() :
                    control->vertAlign() == Control::ALIGN_TOP    ? nPos.y() + size.y() :
                    nPos.y() + size.y()*0.5;
              }

              xform->setMatrix( osg::Matrixd::translate(x, y-context._vp->height(), 0) );

              osg::BoundingBox bbox( x, y, 0.0, x+size.x(), y+size.y(), 1.0 );
              if ( _sortingEnabled )
              {
                  // prevent overlap.
                  for( std::vector<osg::BoundingBox>::iterator u = _taken.begin(); u != _taken.end(); ++u )
                  {
                      if ( u->intersects( bbox ) )
                      {
                          nodeData._obscured = true;
                          break;
                      }
                  }
              }

              if ( nodeData._obscured == false )
              {
                  if ( _sortingEnabled )
                    _taken.push_back( bbox );

                  // the geode holding this node's geometry:
                  osg::Geode* geode = static_cast<osg::Geode*>( xform->getChild(0) );

                  // if the control changed, we need to rebuild its drawables:
                  if ( control->isDirty() )
                  {
                      // clear out the geode:
                      geode->removeDrawables( 0, geode->getNumDrawables() );

                      // calculate the size of the control in screen space:
                      osg::Vec2f dummySize;
                      control->calcSize( context, dummySize );
                      control->calcFill( context );

                      // only need to do this if the control has children ... (pos is always 0,0)
                      control->calcPos( context, osg::Vec2f(0,0), size );
                   
                      // build the drawables for the geode and insert them:
                      control->draw( context );
                  }

                  if ( _fading )
                  {
                      // update the "visible time" uniform if it's changed. this will cause the
                      // shader to "fade in" the label when it becomes visible.
                      if ( !nodeData._uniform.valid() )
                      {
                          nodeData._uniform = new osg::Uniform( osg::Uniform::FLOAT, "visibleTime" );
                          geode->getOrCreateStateSet()->addUniform( nodeData._uniform.get() );
                      }

                      float oldValue;
                      nodeData._uniform->get( oldValue );
                      if ( oldValue != nodeData._visibleTime )
                          nodeData._uniform->set( nodeData._visibleTime );
                  }
              }

              visible = !nodeData._obscured;
          }
          
          // adjust the visibility
          xform->setNodeMask( visible ? ~0 : 0 );

          ++i;
        }
        else
        {
          _renderNodes.erase( node );
          _controlNodes.erase( i++ );
        }
    }
}

void
ControlNodeBin::addNode( ControlNode* controlNode )
{
    // if we see a node with a non-zero priority, assume we're sorting
    // by priority.
    if ( controlNode->getPriority() != 0.0f )
        _sortByDistance = false;

    // record the node in priority order.
    ControlNodeCollection::iterator ptr = _controlNodes.insert(
        ControlNodePair( -controlNode->getPriority(), controlNode ) );

    // record it in the index.
    _index.insert( ControlIndexPair(controlNode->getControl(), ptr) );

    // create and cache a transform/geode pair for the node. the xform will position
    // the geode in 2D space.
    osg::MatrixTransform* xform = new osg::MatrixTransform();
    osg::Geode* geode = new osg::Geode();
    xform->addChild( geode );
    _renderNodes.insert( RenderNodePair(controlNode, xform) );

    // put it in the render graph.
    _group->addChild( xform );
}

// ---------------------------------------------------------------------------

ControlCanvas*
ControlCanvas::getOrCreate(osg::View* view)
{
    if ( !view )
        return 0L;

    if ( !view->getCamera() )
        return 0L;

    ControlCanvas* canvas = osgEarth::findTopMostNodeOfType<ControlCanvas>(view->getCamera());
    if ( canvas )
        return canvas;

    canvas = new ControlCanvas();

    // ControlCanvas does NOT work as a direct child of the View's camera.
    osg::Group* group = 0L;
    if ( view->getCamera()->getNumChildren() > 0 )
    {
        group = view->getCamera()->getChild(0)->asGroup();
        if ( !group )
        {
            group = new osg::Group();
            osgEarth::insertGroup(group, view->getCamera());
        }
    }
    else
    {
        group = new osg::Group();
        view->getCamera()->addChild(group);
    }

    group->addChild( canvas );
    return canvas;
}

ControlCanvas*
ControlCanvas::get(osg::View* view)
{
#if 1 // for now, to avoid breaking lots of code
    return getOrCreate(view);
#else
    if ( !view )
        return 0L;

    if ( !view->getCamera() )
        return 0L;

    ControlCanvas* canvas = osgEarth::findTopMostNodeOfType<ControlCanvas>(view->getCamera());
    if ( canvas )
        return canvas;

    return 0L;
#endif
}


ControlCanvas::ControlCanvas()
{
    init();
}

void
ControlCanvas::init()
{
    _contextDirty  = true;
    _updatePending = false;

    // deter the optimizer
    this->setDataVariance( osg::Object::DYNAMIC );

    this->addEventCallback( new EventCallback(this) );

    setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    setViewMatrix(osg::Matrix::identity());
    setClearMask(GL_DEPTH_BUFFER_BIT);
    setRenderOrder(osg::Camera::POST_RENDER, 25000);
    setAllowEventFocus( true );
    
    // register for event traversals.
    ADJUST_EVENT_TRAV_COUNT( this, 1 );

    osg::StateSet* ss = getOrCreateStateSet();
    ss->setMode( GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE );
    ss->setMode( GL_BLEND, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE );
    ss->setAttributeAndModes( new osg::Depth( osg::Depth::ALWAYS, 0, 1, false ) );
    ss->setRenderBinDetails( 0, "TraversalOrderBin" );

    _controlNodeBin = new ControlNodeBin();
    this->addChild( _controlNodeBin->getControlGroup() );
   
#ifndef OSG_GLES2_AVAILABLE
    // don't use shaders unless we have to.
    this->getOrCreateStateSet()->setAttributeAndModes(
        new osg::Program(), 
        osg::StateAttribute::OFF|osg::StateAttribute::OVERRIDE);
#endif
}

ControlCanvas::~ControlCanvas()
{
    //nop
}

void
ControlCanvas::setAllowControlNodeOverlap( bool value )
{
    getControlNodeBin()->_sortingEnabled = !value;
}

Control*
ControlCanvas::addControlImpl( Control* control )
{
    control->dirty();
    this->addChild( control );
    return control;
}

void
ControlCanvas::removeControl( Control* control )
{
    removeChild( control );
}

Control*
ControlCanvas::getControlAtMouse( float x, float y )
{
    for( osg::NodeList::iterator i = _children.begin(); i != _children.end(); ++i )
    {
        Control* control = dynamic_cast<Control*>( i->get() );
        if ( control->intersects( x, _context._vp->height() - y ) )
        {
            return control;
        }
    }
    return 0L;
}

bool
ControlCanvas::handle(const osgGA::GUIEventAdapter& ea,
                      osgGA::GUIActionAdapter&      aa)
{
    if ( !_context._vp )
        return false;

    for( unsigned i=getNumChildren()-1; i>0; --i )
    {
        Control* control = static_cast<Control*>( getChild(i) );
        if ( control->isDirty() )
        {
            aa.requestRedraw();
            break;
        }
    }

    bool handled = false;

    //Send a frame event to all controls
    if ( ea.getEventType() == osgGA::GUIEventAdapter::FRAME )
    {
        for( unsigned i=1; i<getNumChildren(); ++i )
        {
            Control* control = static_cast<Control*>( getChild(i) );
            control->handle(ea, aa, _context);
        }
        return handled;
    }


    float canvasY = _context._vp->height() - (ea.getY() - _context._view->getCamera()->getViewport()->y());
    float canvasX = ea.getX() - _context._view->getCamera()->getViewport()->x();

    for( unsigned i=getNumChildren()-1; i>0; --i )
    {
        Control* control = static_cast<Control*>( getChild(i) );
        if ( control->intersects( canvasX, canvasY ) )
        {
            handled = control->handle( ea, aa, _context );
            if ( handled )
                break;
        }
    }

    if ( _context._active.size() > 1 )
    {
        _context._active.front()->setActive( false );
        _context._active.pop();
    }

    if ( _context._active.size() > 0 )
    {
        bool hit = _context._active.front()->intersects( canvasX, canvasY );
        _context._active.front()->setActive( hit );
        if ( !hit )
            _context._active.pop();
    }

    return handled; //_context._active.size() > 0;
}

void
ControlCanvas::update(const osg::FrameStamp* frameStamp)
{
    _context._frameStamp = frameStamp;

    if ( !_context._vp )
        return;

    int bin = 0;
    for( unsigned i=1; i<getNumChildren(); ++i )
    {
        Control* control = static_cast<Control*>( getChild(i) );

        if ( control->isDirty() || _contextDirty )
        {
            osg::Vec2f size;
            control->calcSize( _context, size );
            control->calcFill( _context );

            osg::Vec2f surfaceSize( _context._vp->width(), _context._vp->height() );
            control->calcPos( _context, osg::Vec2f(0,0), surfaceSize );

            control->draw( _context );
        }
    }

    if ( _controlNodeBin.valid() )
    {
        _controlNodeBin->draw( _context, _contextDirty, bin );
    }

#ifdef OSG_GLES2_AVAILABLE
    // shaderize.
    // we don't really need to rebuild shaders on every dirty; we could probably
    // just do it on add/remove controls; but that's an optimization for later
    Registry::shaderGenerator().run( this, "osgEarth.ControlCanvas" );
#endif

    _contextDirty = false;
}

void
ControlCanvas::traverse(osg::NodeVisitor& nv)
{
    switch( nv.getVisitorType() )
    {
        case osg::NodeVisitor::EVENT_VISITOR:
        {
            if ( !_updatePending )
            {
                bool needsUpdate = _contextDirty;
                if ( !needsUpdate )
                {
                    for( unsigned i=1; i<getNumChildren(); ++i )
                    {
                        Control* control = static_cast<Control*>( getChild(i) );
                        if ( control->isDirty() )
                        {
                            needsUpdate = true;
                            break;
                        }
                    }
                }

                if ( needsUpdate )
                {
                    _updatePending = true;
                    ADJUST_UPDATE_TRAV_COUNT( this, 1 );
                }
            }
        }
        break;

    case osg::NodeVisitor::UPDATE_VISITOR:
        {
            update( nv.getFrameStamp() );
            ADJUST_UPDATE_TRAV_COUNT( this, -1 );
            _updatePending = false;
        }
        break;

    default:
    case osg::NodeVisitor::CULL_VISITOR:
        {
        }
        break;
    }

    osg::Camera::traverse( nv );
}

void
ControlCanvas::setControlContext( const ControlContext& cx )
{
    _context = cx;
    _contextDirty = true;
}
