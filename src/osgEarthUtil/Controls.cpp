/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2010 Pelican Mapping
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
#include <osgEarth/FindNode>
#include <osg/Geometry>
#include <osg/NodeCallback>
#include <osg/Depth>
#include <osg/TextureRectangle>
#include <osgGA/GUIEventHandler>
#include <osgText/Text>
#include <osgEarthSymbology/Geometry>
#include <osgEarthSymbology/GeometryRasterizer>

using namespace osgEarth;
using namespace osgEarth::Symbology;
using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;

#define LC "[Controls] "

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
}

// ---------------------------------------------------------------------------

Control::Control() :
_x(0), _y(0), _width(1), _height(1),
_margin( Gutter(0) ),
_padding( Gutter(2) ),
_visible( true ),
_valign( ALIGN_NONE ),
_halign( ALIGN_NONE ),
_backColor( osg::Vec4f(0,0,0,0) ),
_foreColor( osg::Vec4f(1,1,1,1) ),
_activeColor( osg::Vec4f(.4,.4,.4,1) ),
_active( false ),
_absorbEvents( false )
{
    //nop
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
Control::addEventHandler( ControlEventHandler* handler )
{
    _eventHandlers.push_back( handler );
}

bool
Control::getParent( osg::ref_ptr<Control>& out ) const
{
    out = _parent.get();
    return out.valid();
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
Control::dirty()
{
    _dirty = true;
    osg::ref_ptr<Control> parent;
    if ( getParent( parent ) )
        parent->dirty();
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
            _renderPos.y() = cursor.y() + 0.5*(parentSize.y() - _renderSize.y());
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
Control::draw(const ControlContext& cx, DrawableList& out )
{
    // by default, rendering a Control directly results in a colored quad. Usually however
    // you will not render a Control directly, but rather one of its subclasses.
    if ( visible() == true )
    {
        if ( !(_backColor.isSet() && _backColor->a() == 0) && _renderSize.x() > 0 && _renderSize.y() > 0 )
        {
            float vph = cx._vp->height(); // - padding().bottom();

            _geom = new osg::Geometry();

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

            out.push_back( _geom.get() );
        }

        _dirty = false;
    }
}

bool
Control::handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa, ControlContext& cx )
{
    bool handled = false;    

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
                for( ControlEventHandlerList::const_iterator i = _eventHandlers.begin(); i != _eventHandlers.end(); ++i )
                {
                    i->get()->onClick( this, ea.getButtonMask() );
                }
            }
        }
    }

    return handled || _absorbEvents;
}

// ---------------------------------------------------------------------------

// override osg Text to get at some of the internal properties
struct LabelText : public osgText::Text
{
    const osg::BoundingBox& getTextBB() const { return _textBB; }
    const osg::Matrix& getATMatrix(int contextID) const { return _autoTransformCache[contextID]._matrix; }
};

LabelControl::LabelControl(const std::string& text,
                           float fontSize,
                           const osg::Vec4f& foreColor)
{
    setText( text );
    setFont( osgText::readFontFile( "arial.ttf" ) ); // TODO: cache this?
    setFontSize( fontSize );
    setBackColor( osg::Vec4f(0,0,0,0) );
    setForeColor( foreColor );
}

LabelControl::LabelControl(const std::string& text,
                           const osg::Vec4f& foreColor)
{
    setText( text );
    setFont( osgText::readFontFile( "arial.ttf" ) ); // TODO: cache this?
    setFontSize( 18.0f );
    setBackColor( osg::Vec4f(0,0,0,0) );
    setForeColor( foreColor );
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
LabelControl::calcSize(const ControlContext& cx, osg::Vec2f& out_size)
{
    if ( visible() == true )
    {
        // we have to create the drawable during the layout pass so we can calculate its size.
        LabelText* t = new LabelText();

        t->setText( _text );
        // yes, object coords. screen coords won't work becuase the bounding box will be wrong.
        t->setCharacterSizeMode( osgText::Text::OBJECT_COORDS );
        t->setCharacterSize( _fontSize );
        // always align to top. layout alignment gets calculated layer in Control::calcPos().
        t->setAlignment( osgText::Text::LEFT_TOP ); 
        t->setColor( foreColor().value() );
        if ( _font.valid() )
            t->setFont( _font.get() );

        if ( backColor().isSet() )
        {
            t->setBackdropType( osgText::Text::OUTLINE );
            t->setBackdropColor( backColor().value() );
        }

        osg::BoundingBox bbox = t->getTextBB();
        if ( cx._viewContextID != ~0u )
        {
            //the Text's autoTransformCache matrix puts some mojo on the bounding box
            osg::Matrix m = t->getATMatrix( cx._viewContextID );
            _bmin = osg::Vec3( bbox.xMin(), bbox.yMin(), bbox.zMin() ) * m;
            _bmax = osg::Vec3( bbox.xMax(), bbox.yMax(), bbox.zMax() ) * m;
            //_renderSize.set( _bmax.x() - _bmin.x(), _bmax.y() - _bmin.y() );
        }
        else
        {
            _bmin = osg::Vec3( bbox.xMin(), bbox.yMin(), bbox.zMin() );
            _bmax = osg::Vec3( bbox.xMax(), bbox.yMax(), bbox.zMax() );
            //_renderSize.set( bbox.xMax()-bbox.xMin(), bbox.yMax()-bbox.yMin() );
        }

        _renderSize.set(
            padding().left() + (_bmax.x() - _bmin.x()) + padding().right(),
            padding().left() + (_bmax.y() - _bmin.y()) + padding().right() );

        _drawable = t;

        out_size.set(
            margin().left() + margin().right() + _renderSize.x(),
            margin().top() + margin().bottom() + _renderSize.y() );
    }
    else
    {
        out_size.set(0,0);
    }

    //_dirty = false;
}

void
LabelControl::draw( const ControlContext& cx, DrawableList& out )
{
    if ( _drawable.valid() && visible() == true )
    {
        Control::draw( cx, out );

        float vph = cx._vp->height(); // - padding().bottom();

        LabelText* t = static_cast<LabelText*>( _drawable.get() );
        osg::BoundingBox bbox = t->getTextBB();
        t->setPosition( osg::Vec3( _renderPos.x(), vph - _renderPos.y(), 0 ) );
        out.push_back( _drawable.get() );
    }
}

// ---------------------------------------------------------------------------

ImageControl::ImageControl( osg::Image* image ) :
_rotation_rad( 0.0f ),
_fixSizeForRot( false )
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
ImageControl::setRotation( float value_deg )
{
    float rad = osg::DegreesToRadians(value_deg);
    if ( _rotation_rad != rad ) {
        _rotation_rad = rad;
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
        float rot = _fixSizeForRot ? osg::PI_4 : _rotation_rad;
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

void
ImageControl::draw( const ControlContext& cx, DrawableList& out )
{
    if ( visible() == true && _image.valid() )
    {
        //TODO: this is not precisely correct..images get deformed slightly..
        osg::Geometry* g = new osg::Geometry();

        float rx = osg::round( _renderPos.x() );
        float ry = osg::round( _renderPos.y() );
        float vph = cx._vp->height();

        osg::Vec3Array* verts = new osg::Vec3Array(4);
        g->setVertexArray( verts );

        if ( _rotation_rad != 0.0f || _fixSizeForRot == true )
        {
            osg::Vec2f rc( rx+_renderSize.x()/2, (vph-ry)-_renderSize.y()/2 );
            float ra = osg::PI - _rotation_rad;

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
        (*t)[0].set( 0, flip? 0: _image->t()-1 );
        (*t)[1].set( 0, flip? _image->t()-1: 0 );
        (*t)[2].set( _image->s()-1, flip? _image->t()-1: 0 );
        (*t)[3].set( _image->s()-1, flip? 0: _image->t()-1 );
        g->setTexCoordArray( 0, t );

        osg::TextureRectangle* texrec = new osg::TextureRectangle( _image.get() );
        texrec->setFilter( osg::Texture::MIN_FILTER, osg::Texture::NEAREST );
        texrec->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
        g->getOrCreateStateSet()->setTextureAttributeAndModes( 0, texrec, osg::StateAttribute::ON );

        osg::TexEnv* texenv = new osg::TexEnv( osg::TexEnv::MODULATE );
        g->getStateSet()->setTextureAttributeAndModes( 0, texenv, osg::StateAttribute::ON );

        out.push_back( g );

        _dirty = false;
    }
}

// ---------------------------------------------------------------------------

HSliderControl::HSliderControl( float min, float max, float value ) :
_min(min),
_max(max),
_value(value)
{
   if ( _max <= _min )
       _max = _min+1.0f;
   if ( _value < _min )
       _value = _min;
   if ( _value > _max )
       _value = _max;
}

void
HSliderControl::fireValueChanged()
{
    for( ControlEventHandlerList::const_iterator i = _eventHandlers.begin(); i != _eventHandlers.end(); ++i )
    {
        i->get()->onValueChanged( this, _value );
    }
}

void
HSliderControl::setValue( float value )
{
    value = osg::clampBetween( value, _min, _max );
    if ( value != _value )
    {
        _value = value;
        fireValueChanged();
        dirty();
    }
}

void
HSliderControl::setMin( float min )
{
    if ( min != _min )
    {
        _min = min;
        if ( _min >= _max )
            _max = _min+1.0f;

        if ( _value < _min || _value > _max ) 
        {
            _value = _min;
            fireValueChanged();
        }
        dirty();
    }
}

void
HSliderControl::setMax( float max )
{
    if ( max != _max )
    {
        _max = max;
        if ( _max <= _min )
            _max = _min+1.0f;

        if ( _value < _min || _value > _max )
        {
            _value = _max;
            fireValueChanged();
        }
        dirty();
    }
}

void
HSliderControl::draw( const ControlContext& cx, DrawableList& out )
{
    Control::draw( cx, out );

    if ( visible() == true )
    {
        osg::Geometry* g = new osg::Geometry();

        float rx = osg::round( _renderPos.x() );
        float ry = osg::round( _renderPos.y() );
        float rw = osg::round( _renderSize.x() - padding().x() );
        float rh = osg::round( _renderSize.y() - padding().y() );

        float vph = cx._vp->height(); // - padding().bottom();
        //float hy = vph - (_renderPos.y() + 0.5 * _renderSize.y());

        osg::Vec3Array* verts = new osg::Vec3Array(8);
        g->setVertexArray( verts );

        (*verts)[0].set( rx, vph - ry, 0 );
        (*verts)[1].set( rx + rw, vph - ry, 0 );
        (*verts)[2].set( rx + rw, vph - (ry + rh), 0 );
        (*verts)[3].set( rx, vph - (ry + rh), 0 );
        g->addPrimitiveSet( new osg::DrawArrays( GL_LINE_LOOP, 0, 4 ) );

        float hx = rx + rw * ( (_value-_min)/(_max-_min) );

        (*verts)[4].set( hx-4, vph - ry + 3, 0 );
        (*verts)[5].set( hx+4, vph - ry + 3, 0 );
        (*verts)[6].set( hx+4, vph - (ry + rh + 3), 0 );
        (*verts)[7].set( hx-4, vph - (ry + rh + 3), 0 );
        g->addPrimitiveSet( new osg::DrawArrays( GL_QUADS, 4, 4 ) );

        osg::Vec4Array* c = new osg::Vec4Array(1);
        (*c)[0] = *foreColor();
        g->setColorArray( c );
        g->setColorBinding( osg::Geometry::BIND_OVERALL );

        out.push_back( g );
    }
}

bool
HSliderControl::handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa, ControlContext& cx )
{
    if ( ea.getEventType() == osgGA::GUIEventAdapter::DRAG )
    {
        float relX = ea.getX() - _renderPos.x();

        setValue( _min + (_max-_min) * ( relX/_renderSize.x() ) );
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

void
CheckBoxControl::fireValueChanged()
{
    for( ControlEventHandlerList::const_iterator i = _eventHandlers.begin(); i != _eventHandlers.end(); ++i )
    {
        i->get()->onValueChanged( this, _value );
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
CheckBoxControl::draw( const ControlContext& cx, DrawableList& out )
{
    Control::draw( cx, out );

    if ( visible() == true )
    {
        osg::Geometry* g = new osg::Geometry();

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

        out.push_back( g );
    }
}

bool
CheckBoxControl::handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa, ControlContext& cx )
{
    if ( ea.getEventType() == osgGA::GUIEventAdapter::PUSH )
    {
        setValue( !_value );
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
Frame::draw( const ControlContext& cx, DrawableList& out )
{
    if ( !getImage() || getImage()->s() != _renderSize.x() || getImage()->t() != _renderSize.y() )
    {
        // a simple colored border frame
        osg::ref_ptr<Geometry> geom = new Ring();
        geom->push_back( osg::Vec3d( 0, 0, 0 ) );
        geom->push_back( osg::Vec3d( _renderSize.x()-1, 0, 0 ) );
        geom->push_back( osg::Vec3d( _renderSize.x()-1, _renderSize.y()-1, 0 ) );
        geom->push_back( osg::Vec3d( 0, _renderSize.y()-1, 0 ) );

        GeometryRasterizer ras( (int)_renderSize.x(), (int)_renderSize.y() );
        ras.draw( geom.get() );

        osg::Image* image = ras.finalize();
        const_cast<Frame*>(this)->setImage( image );
    }

    Control::draw( cx, out );       // draws the background
    ImageControl::draw( cx, out );  // draws the border
}

// ---------------------------------------------------------------------------

RoundedFrame::RoundedFrame()
{
    //nop
}

void
RoundedFrame::draw( const ControlContext& cx, DrawableList& out )
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

        ImageControl::draw( cx, out );
    }
    else
    {
        // fallback: draw a non-rounded frame.
        Frame::draw( cx, out );
    }
}

// ---------------------------------------------------------------------------

Container::Container() :
_spacing( 1 )
{
    //nop
}

void
Container::setFrame( Frame* frame )
{
    if ( frame != _frame.get() ) {
        _frame = frame;
        dirty();
    }
}

void
Container::setSpacing( float value )
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

        for( ControlList::const_iterator i = children().begin(); i != children().end(); ++i )
            if ( !i->get()->horizAlign().isSet() )
                i->get()->setHorizAlign( value );

        dirty();
    }
}

void
Container::setChildVertAlign( Alignment value )
{
    if ( !_childvalign.isSet() || _childvalign != value )
    {
        _childvalign = value;

        for( ControlList::const_iterator i = children().begin(); i != children().end(); ++i )
            if ( !i->get()->vertAlign().isSet() )
                i->get()->setVertAlign( value );

        dirty();
    }
}

void
Container::calcSize(const ControlContext& cx, osg::Vec2f& out_size)
{
    if ( visible() == true )
    {
        if ( _frame.valid() )
        {
            _frame->setWidth( _renderSize.x() );
            _frame->setHeight( _renderSize.y() );

            osg::Vec2f dummy;
            _frame->calcSize( cx, dummy );
        }

        // no need to set the output vars.

        //_dirty = false;  
    }
}

void
Container::calcPos(const ControlContext& context, const osg::Vec2f& cursor, const osg::Vec2f& parentSize)
{
    Control::calcPos( context, cursor, parentSize );

    // process the frame.. it's not a child of the container
    if ( visible() == true && _frame.valid() )
    {
        _frame->calcPos( context, _renderPos - padding().offset(), parentSize );
    }
}

void
Container::draw( const ControlContext& cx, DrawableList& out )
{
    if ( visible() == true )
    {
        Control::draw( cx, out );
        if ( _frame.valid() )
            _frame->draw( cx, out );
    }
}

bool
Container::handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa, ControlContext& cx )
{
    bool handled = false;
    for( ControlList::const_reverse_iterator i = children().rbegin(); i != children().rend(); ++i )
    {
        Control* child = i->get();
        if (ea.getEventType() == osgGA::GUIEventAdapter::FRAME || child->intersects( ea.getX(), cx._vp->height() - ea.getY() ) )
            handled = child->handle( ea, aa, cx );
        if ( handled )
            break;
    }

    return handled ? handled : Control::handle( ea, aa, cx );
}

// ---------------------------------------------------------------------------

VBox::VBox()
{
    //nop
}

void
VBox::addControl( Control* control, int index )
{
    if ( index < 0 )
        _controls.push_back( control );
    else
        _controls.insert( _controls.begin() + osg::minimum(index,(int)_controls.size()-1), control );
    control->setParent( this );
    dirty();
}

void
VBox::clearControls()
{
    _controls.clear();
    dirty();
}

void
VBox::calcSize(const ControlContext& cx, osg::Vec2f& out_size)
{
    if ( visible() == true )
    {
        _renderSize.set( 0, 0 );

        // collect all the members, growing the container size vertically
        for( ControlList::const_iterator i = _controls.begin(); i != _controls.end(); ++i )
        {
            Control* child = i->get();
            osg::Vec2f childSize;
            bool first = i == _controls.begin();

            child->calcSize( cx, childSize );

            _renderSize.x() = osg::maximum( _renderSize.x(), childSize.x() );
            _renderSize.y() += first ? childSize.y() : spacing() + childSize.y();
        }

        _renderSize.set(
            _renderSize.x() + padding().left() + padding().right(),
            _renderSize.y() + padding().top() + padding().bottom() );

        out_size.set(
            _renderSize.x() + margin().left() + margin().right(),
            _renderSize.y() + margin().top() + margin().bottom() );

        Container::calcSize( cx, out_size );
    }
    else
    {
        out_size.set(0,0);
    }
}

void
VBox::calcPos(const ControlContext& cx, const osg::Vec2f& cursor, const osg::Vec2f& parentSize)
{
    Container::calcPos( cx, cursor, parentSize );

    osg::Vec2f childCursor = _renderPos;

    for( ControlList::const_iterator i = _controls.begin(); i != _controls.end(); ++i )
    {
        Control* child = i->get();
        child->calcPos( cx, childCursor, _renderSize );
        childCursor.y() += child->margin().top() + child->renderSize().y() + child->margin().bottom() + spacing();
    }
}

void
VBox::draw( const ControlContext& cx, DrawableList& out )
{
    if ( visible() == true )
    {
        Container::draw( cx, out );
        for( ControlList::const_iterator i = _controls.begin(); i != _controls.end(); ++i )
            i->get()->draw( cx, out );
    }
}

// ---------------------------------------------------------------------------

HBox::HBox()
{
    //nop
}

void
HBox::addControl( Control* control, int index )
{
    if ( index < 0 )
        _controls.push_back( control );
    else
        _controls.insert( _controls.begin() + osg::minimum(index,(int)_controls.size()-1), control );
    control->setParent( this );
    dirty();
}

void
HBox::clearControls()
{
    _controls.clear();
    dirty();
}

void
HBox::calcSize(const ControlContext& cx, osg::Vec2f& out_size)
{
    if ( visible() == true )
    {
        _renderSize.set( 0, 0 );

        // collect all the members, growing the container is its orientation.
        for( ControlList::const_iterator i = _controls.begin(); i != _controls.end(); ++i )
        {
            Control* child = i->get();
            osg::Vec2f childSize;
            bool first = i == _controls.begin();

            child->calcSize( cx, childSize );

            _renderSize.x() += first ? childSize.x() : spacing() + childSize.x();
            _renderSize.y() = osg::maximum( _renderSize.y(), childSize.y() );
        }

        _renderSize.set(
            _renderSize.x() + padding().left() + padding().right(),
            _renderSize.y() + padding().top() + padding().bottom() );

        out_size.set(
            _renderSize.x() + margin().left() + margin().right(),
            _renderSize.y() + margin().top() + margin().bottom() );

        Container::calcSize( cx, out_size );
    }
}

void
HBox::calcPos(const ControlContext& cx, const osg::Vec2f& cursor, const osg::Vec2f& parentSize)
{
    Container::calcPos( cx, cursor, parentSize );

    osg::Vec2f childCursor = _renderPos;
        //_renderPos.x() + padding().left(),
        //_renderPos.y() + padding().top() );

    // collect all the members
    for( ControlList::const_iterator i = _controls.begin(); i != _controls.end(); ++i )
    {
        Control* child = i->get();
        child->calcPos( cx, childCursor, _renderSize );
        childCursor.x() += child->margin().left() + child->renderSize().x() + child->margin().right() + spacing();
    }
}

void
HBox::draw( const ControlContext& cx, DrawableList& out ) 
{
    Container::draw( cx, out );
    for( ControlList::const_iterator i = _controls.begin(); i != _controls.end(); ++i )
        i->get()->draw( cx, out );
}

// ---------------------------------------------------------------------------

Grid::Grid()
{
    //nop
}

void
Grid::setControl( int col, int row, Control* child )
{
    if ( !child ) return;

    expandToInclude( col, row );

    Control* oldControl = cell( col, row ).get();
    if ( oldControl ) {
        ControlList::iterator i = std::find( _children.begin(), _children.end(), oldControl );
        if ( i != _children.end() ) 
            _children.erase( i );
    }

    cell( col, row ) = child;
    _children.push_back( child );

    child->setParent( this );

    dirty();
}

osg::ref_ptr<Control>&
Grid::cell(int col, int row)
{
    return _rows[row][col];
}

void
Grid::expandToInclude( int col, int row )
{
    while( (int)_rows.size() <= row )
        _rows.push_back( Row() );

    for( RowVector::iterator i = _rows.begin(); i != _rows.end(); ++i ) {
        Row& row = *i;
        while( (int)row.size() <= col )
            row.push_back( 0L );
    }
}

void
Grid::addControl( Control* control, int index )
{
    // creates a new row and puts the control in its first column
    setControl( _rows.size(), 0, control );
}

void
Grid::clearControls()
{
    _rows.clear();
    _children.clear();
    _rowHeights.clear();
    _colWidths.clear();
    dirty();
}

void
Grid::calcSize( const ControlContext& cx, osg::Vec2f& out_size )
{
    if ( visible() == true )
    {
        _renderSize.set( 0, 0 );

        int numRows = _rows.size();
        int numCols = numRows > 0 ? _rows[0].size() : 0;

        _rowHeights.assign( numRows, 0.0f );
        _colWidths.assign( numCols, 0.0f );

        if ( numRows > 0 && numCols > 0 )
        {
            for( int r=0; r<numRows; ++r )
            { 
                for( int c=0; c<numCols; ++c )
                {
                    Control* child = cell(c,r).get();
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

            for( int c=0; c<numCols; ++c )
                _renderSize.x() += _colWidths[c];
            _renderSize.x() += spacing() * (numCols-1);

            for( int r=0; r<numRows; ++r )
                _renderSize.y() += _rowHeights[r];
            _renderSize.y() += spacing() * (numRows-1);
        }
        
        _renderSize.set(
            _renderSize.x() + padding().x(),
            _renderSize.y() + padding().y() );

        out_size.set(
            _renderSize.x() + margin().x(),
            _renderSize.y() + margin().y() );

        Container::calcSize( cx, out_size );
    }
}

void
Grid::calcPos( const ControlContext& cx, const osg::Vec2f& cursor, const osg::Vec2f& parentSize )
{
    Container::calcPos( cx, cursor, parentSize );

    int numRows = _rows.size();
    int numCols = numRows > 0 ? _rows[0].size() : 0;

    osg::Vec2f childCursor = _renderPos;

    for( int r=0; r<numRows; ++r )
    {
        for( int c=0; c<numCols; ++c )
        {
            Control* child = cell(c,r).get();
            if ( child )
            {
                osg::Vec2f cellSize( _colWidths[c], _rowHeights[r] );
                child->calcPos( cx, childCursor, cellSize );
            }
            childCursor.x() += _colWidths[c] + spacing();
        }
        childCursor.x() = _renderPos.x();
        childCursor.y() += _rowHeights[r] + spacing();
    }
}

void
Grid::draw( const ControlContext& cx, DrawableList& out )
{
    Container::draw( cx, out );
    for( ControlList::const_iterator i = _children.begin(); i != _children.end(); ++i )
        i->get()->draw( cx, out );
}

// ---------------------------------------------------------------------------

namespace osgEarth { namespace Util { namespace Controls
{
    // This handler keeps an eye on the viewport and informs the control surface when it changes.
    // We need this info since controls position from the upper-left corner.
    struct ViewportHandler : public osgGA::GUIEventHandler
    {
        ViewportHandler( ControlCanvas* cs ) : _cs(cs), _width(0), _height(0), _first(true) { }

        bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
        {
            if ( ea.getEventType() == osgGA::GUIEventAdapter::RESIZE || _first )
            {
                osg::Camera* cam = aa.asView()->getCamera();
                if ( cam && cam->getViewport() )
                {
                    const osg::Viewport* vp = cam->getViewport();
                    if ( _first || vp->width() != _width || vp->height() != _height )
                    {
                        _cs->setProjectionMatrix(osg::Matrix::ortho2D( 0, vp->width()-1, 0, vp->height()-1 ) );

                        ControlContext cx;
                        cx._view = aa.asView();
                        cx._vp = new osg::Viewport( 0, 0, vp->width(), vp->height() );
                        cx._viewContextID = aa.asView()->getCamera()->getGraphicsContext()->getState()->getContextID();
                        _cs->setControlContext( cx );

                        _width  = (int)vp->width();
                        _height = (int)vp->height();
                    }
                    if ( vp->width() != 0 && vp->height() != 0 )
                    {
                        _first = false;
                    }
                }
            }
            return false;
        }
        ControlCanvas* _cs;
        int _width, _height;
        bool _first;
    };

    struct ControlCanvasEventHandler : public osgGA::GUIEventHandler
    {
        ControlCanvasEventHandler( ControlCanvas* cs ) : _cs(cs) { }

        bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
        {
            return _cs->handle( ea, aa );
        }

        ControlCanvas* _cs;
    };

    // This callback installs a control canvas under a view.
    struct CanvasInstaller : public osg::NodeCallback
    {
        CanvasInstaller( ControlCanvas* canvas, osg::Camera* camera )
            : _canvas( canvas )
        {
            _oldCallback = camera->getUpdateCallback();
            camera->setUpdateCallback( this );
        }
        
        void operator()( osg::Node* node, osg::NodeVisitor* nv )
        {
            osg::Camera* camera = static_cast<osg::Camera*>(node);
            osgViewer::View* view2 = dynamic_cast<osgViewer::View*>(camera->getView());
            install( view2, _canvas.get() );
            camera->setUpdateCallback( _oldCallback.get() );
        }

        static void install( osgViewer::View* view2, ControlCanvas* canvas )
        {
            osg::Node* node = view2->getSceneData();
            osg::Group* group = new osg::Group();
            if ( node )
                group->addChild( node );
            group->addChild( canvas );
            view2->setSceneData( group );
        }

        osg::ref_ptr<ControlCanvas>     _canvas;
        osg::ref_ptr<osg::NodeCallback> _oldCallback;
    };

} } } // namespace osgEarth::Util::Controls

// ---------------------------------------------------------------------------

ControlNode::ControlNode( Control* control, float priority ) :
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
        osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>( &nv );

        // pull up the per-view data for this view:
        PerViewData& data = _perViewData[cv->getCurrentCamera()->getView()];

        // if it's uninitialized, find the corresponding control canvas and 
        // cache a reference to its control node bin:
        if ( !data._visibleNodes.valid() )
        {
            ControlCanvas* canvas = ControlCanvas::get( cv->getCurrentCamera()->getView(), true );
            if ( canvas )
            {
                ControlNodeBin* bin = canvas->getControlNodeBin();
                bin->addNode( this );
                data._visibleNodes = bin->getVisibleNodesVector();       
            }
        }

        if ( data._visibleNodes.valid() )
        {
            // normal-cull it:
            // TODO: only do this in geocentric mode.....
            osg::Matrixd local2world = osg::computeLocalToWorld( nv.getNodePath() );            
            double dp = cv->getEyePoint() * local2world.getTrans();

            if ( dp < 0.0 )
            {
                data._obscured = true;
            }

            else
            {
                // calculate its screen position:
                osg::Vec3f sp = s_zero * (*cv->getMVPW());
                data._screenPos.set( sp.x(), sp.y() );

                if ( data._obscured == true )
                {
                    data._obscured = false;
                    data._visibleTime = cv->getFrameStamp()->getReferenceTime();
                }

                data._visibleNodes->push_back( this );
            }
        }
    }

    // ControlNode has no children, so no point in calling traverse.
}

ControlNode::PerViewData::PerViewData() :
_obscured   ( true ),
_visibleTime( 0.0 ),
_screenPos  ( 0.0, 0.0 )
{
    //nop
}

// ---------------------------------------------------------------------------

namespace
{
    // ControlNodeBin shaders.

    char* s_controlVertexShader =
        "void main() \n"
        "{ \n"
        "    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex; \n"
        "    gl_TexCoord[0] = gl_MultiTexCoord0; \n"
        "    gl_FrontColor = gl_Color; \n"
        "} \n";

    char* s_controlFragmentShader =
        "uniform sampler2D tex0; \n"
        "uniform float visibleTime; \n"
        "uniform float osg_FrameTime; \n"
        "void main() \n"
        "{ \n"
        "    float opacity = clamp( osg_FrameTime - visibleTime, 0.0, 1.0 ); \n"
        "    vec4 texel = texture2D(tex0, gl_TexCoord[0].st); \n"       
        "    gl_FragColor = vec4(gl_Color.rgb, texel.a * opacity); \n"
        "} \n";
}

// ---------------------------------------------------------------------------

ControlNodeBin::ControlNodeBin()
{
    _group = new Group();
    _visibleNodes = new RefNodeVector();

    osg::Program* program = new osg::Program();
    program->addShader( new osg::Shader( osg::Shader::VERTEX, s_controlVertexShader ) );
    program->addShader( new osg::Shader( osg::Shader::FRAGMENT, s_controlFragmentShader ) );

    osg::StateSet* stateSet = _group->getOrCreateStateSet();
    stateSet->setAttributeAndModes( program, osg::StateAttribute::ON );

    osg::Uniform* defaultOpacity = new osg::Uniform( osg::Uniform::FLOAT, "opacity" );
    defaultOpacity->set( 1.0f );
    stateSet->addUniform( defaultOpacity );

    osg::Uniform* defaultVisibleTime = new osg::Uniform( osg::Uniform::FLOAT, "visibleTime" );
    defaultVisibleTime->set( 0.0f );
    stateSet->addUniform( defaultVisibleTime );
}

void
ControlNodeBin::draw( const ControlContext& context, bool newContext, int bin )
{
    const osg::Viewport* vp = context._vp;
    osg::Vec2f surfaceSize( context._vp->width(), context._vp->height() );

    // we don't really need to keep this list in the object, but that prevents it from having to
    // reallocate it each time
    _taken.clear();

    for( ControlNodeCollection::iterator i = _controlNodes.begin(); i != _controlNodes.end(); ++i )
    {
        ControlNode* node = i->second;
        osg::MatrixTransform* xform = _renderNodes[node];

        ControlNode::PerViewData& nodeData = node->getData( context._view );
        Control* control = node->getControl();

        // if the context changed (e.g., viewport resize), we need to mark all nodes as dirty
        // even if they're obscured...that way they will regenerate properly next time
        if ( newContext )
        {
            control->dirty();
        }

        if ( nodeData._obscured == false )
        {
            const osg::Vec2f& nPos = nodeData._screenPos; //node->getScreenPos( _view ); //_screenPos;
            const osg::Vec2f& size = control->renderSize();

            float x = nPos.x()-size.x()*0.5;
            float y = nPos.y();
            xform->setMatrix( osg::Matrixd::translate(x, y-context._vp->height(), 0) );

            osg::BoundingBox bbox( x, y, 0.0, x+size.x(), y+size.y(), 1.0 );

            for( std::vector<osg::BoundingBox>::iterator u = _taken.begin(); u != _taken.end(); ++u )
            {
                if ( u->intersects( bbox ) )
                {
                    nodeData._obscured = true;
                    break;
                }
            }

            if ( nodeData._obscured == false )
            {
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

                    // only need to do this if the control has children ... (pos is always 0,0)
                    control->calcPos( context, osg::Vec2f(0,0), surfaceSize );
                 
                    // build the drawables for the geode and insert them:
                    DrawableList drawables;
                    control->draw( context, drawables );

                    for( DrawableList::iterator j = drawables.begin(); j != drawables.end(); ++j )
                    {
                        j->get()->setDataVariance( osg::Object::DYNAMIC );

                        osg::StateSet* stateSet = j->get()->getOrCreateStateSet();
                        stateSet->setRenderBinDetails( bin, "RenderBin" );
                        geode->addDrawable( j->get() );
                    }
                }

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
        
        // adjust the visibility based on whether the node is visible
        xform->setNodeMask( nodeData._obscured ? 0 : ~0 );
    }

    _visibleNodes->clear();
}

void
ControlNodeBin::addNode( ControlNode* controlNode )
{
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

ControlCanvas::ViewCanvasMap ControlCanvas::_viewCanvasMap;
OpenThreads::Mutex           ControlCanvas::_viewCanvasMapMutex;

ControlCanvas*
ControlCanvas::get( osg::View* view, bool installInSceneData )
{
    ControlCanvas* canvas = 0L;

    OpenThreads::ScopedLock<OpenThreads::Mutex> lock( _viewCanvasMapMutex );

    ViewCanvasMap::iterator i = _viewCanvasMap.find( view );
    if ( i != _viewCanvasMap.end() )
    {
        canvas = i->second;
    }

    else
    {
        // Not found, so create one. If requested, add a callback that will
        // automatically install it in the view's scene data during the 
        // next update traversal.
        osgViewer::View* view2 = dynamic_cast<osgViewer::View*>(view);
        if ( view2 )
        {
            canvas = new ControlCanvas( view2 );
            _viewCanvasMap[view] = canvas;

            if ( installInSceneData )
                new CanvasInstaller( canvas, view->getCamera() );
        }
    }

    return canvas;
}

// ---------------------------------------------------------------------------

struct UpdateHook : public osg::NodeCallback {
    void operator()(osg::Node* node, osg::NodeVisitor* nv ) {
        static_cast<ControlCanvas*>(node)->update();
    }
};

ControlCanvas::ControlCanvas( osgViewer::View* view ) :
_contextDirty(true)
{
    view->addEventHandler( new ViewportHandler(this) );
    view->addEventHandler( new ControlCanvasEventHandler(this) );

    setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    setViewMatrix(osg::Matrix::identity());
    setClearMask(GL_DEPTH_BUFFER_BIT);
    setRenderOrder(osg::Camera::POST_RENDER); 
    setAllowEventFocus( true );
    
    // activate the update traversal
    ADJUST_UPDATE_TRAV_COUNT( this, 1 );

    osg::StateSet* ss = getOrCreateStateSet();
    ss->setMode( GL_LIGHTING, 0 );
    ss->setMode( GL_BLEND, 1 );
    ss->setAttributeAndModes( new osg::Depth( osg::Depth::LEQUAL, 0, 1, false ) );

    _controlNodeBin = new ControlNodeBin();
    this->addChild( _controlNodeBin->getControlGroup() );

    // register this canvas.
    {
        OpenThreads::ScopedLock<OpenThreads::Mutex> lock( _viewCanvasMapMutex );
        _viewCanvasMap[view] = this;
    }

    //this->setUpdateCallback( new UpdateHook );
}

ControlCanvas::~ControlCanvas()
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock( _viewCanvasMapMutex );
    _viewCanvasMap.erase( _context._view );
}

void
ControlCanvas::addControl( Control* control )
{
    osg::Geode* geode = new osg::Geode();
    _geodeTable[control] = geode;
    addChild( geode );
    control->dirty();
    _controls.push_back( control );
}

void
ControlCanvas::removeControl( Control* control )
{
    GeodeTable::iterator i = _geodeTable.find( control );
    if ( i != _geodeTable.end() )
    {
         removeChild( i->second );
         _geodeTable.erase( i );
    }
    ControlList::iterator j = std::find( _controls.begin(), _controls.end(), control );
    if ( j != _controls.end() )
        _controls.erase( j );
}

Control*
ControlCanvas::getControlAtMouse( float x, float y ) const
{
    for( ControlList::const_iterator i = _controls.begin(); i != _controls.end(); ++i )
    {
        Control* control = i->get();
        if ( control->intersects( x, _context._vp->height() - y ) )
            return control;
    }
    return 0L;
}

bool
ControlCanvas::handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
{
    bool handled = false;
    //Send a frame event to all controls
    if ( ea.getEventType() == osgGA::GUIEventAdapter::FRAME )
    {
        for( ControlList::reverse_iterator i = _controls.rbegin(); i != _controls.rend(); ++i )
        {
            i->get()->handle(ea, aa, _context);
        }
        return handled;
    }


    float invY = _context._vp->height() - ea.getY();

    for( ControlList::reverse_iterator i = _controls.rbegin(); i != _controls.rend(); ++i )
    {
        Control* control = i->get();
        if ( control->intersects( ea.getX(), invY ) )
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
        bool hit = _context._active.front()->intersects( ea.getX(), invY );
        _context._active.front()->setActive( hit );
        if ( !hit )
            _context._active.pop();
    }

    return handled; //_context._active.size() > 0;
}

void
ControlCanvas::update()
{
    int bin = 999999;
    for( ControlList::iterator i = _controls.begin(); i != _controls.end(); ++i )
    {
        Control* control = i->get();
        if ( control->isDirty() || _contextDirty )
        {
            osg::Vec2f size;
            control->calcSize( _context, size );

            osg::Vec2f surfaceSize( _context._vp->width(), _context._vp->height() );
            control->calcPos( _context, osg::Vec2f(0,0), surfaceSize );

            osg::Geode* geode = _geodeTable[control];
            geode->removeDrawables( 0, geode->getNumDrawables() );
            DrawableList drawables;
            control->draw( _context, drawables );

            for( DrawableList::iterator j = drawables.begin(); j != drawables.end(); ++j )
            {
                j->get()->setDataVariance( osg::Object::DYNAMIC );
                j->get()->getOrCreateStateSet()->setRenderBinDetails( bin++, "RenderBin" );
                geode->addDrawable( j->get() );
            }
        }
    }

    if ( _controlNodeBin.valid() )
    {
        _controlNodeBin->draw( _context, _contextDirty, bin );
    }

    _contextDirty = false;
}

void
ControlCanvas::traverse( osg::NodeVisitor& nv )
{
    if ( nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR )
    {
        update();
    }

    osg::Camera::traverse( nv );
}

void
ControlCanvas::setControlContext( const ControlContext& cx )
{
    _context = cx;
    _contextDirty = true;
}
