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
using namespace osgEarthUtil;
using namespace osgEarthUtil::Controls;

#define LC "[osgEarth::Controls] "

// ---------------------------------------------------------------------------

Control::Control() :
_x(0), _y(0), _width(1), _height(1),
_margin( Gutter(0) ),
_halign( ALIGN_NONE ),
_valign( ALIGN_NONE ),
_backColor( osg::Vec4f(0,0,0,0) ),
_foreColor( osg::Vec4f(1,1,1,1) ),
_visible( true )
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
Control::setMargin( const Gutter& value ) {
    if ( value != _margin ) {
        _margin = value;
        dirty();
    }
}

void
Control::setHorizAlign( const Alignment& value ) {
    if ( value != _halign ) {
        _halign = value;
        _x.unset();  // horiz align is mutex with abs positioning
        dirty();
    }
}

void
Control::setVertAlign( const Alignment& value ) {
    if ( value != _valign ) {
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

bool
Control::getParent( osg::ref_ptr<Control>& out ) const
{
    out = _parent.get();
    return out.valid();
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
        _renderSize.set( width().value(), height().value() );

        out_size.set(
            _renderSize.x() + margin().left() + margin().right(),
            _renderSize.y() + margin().top() + margin().bottom() );            
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
        _renderPos.x() = cursor.x() + margin().left() + *x();
    }
    else
    {
        if ( _halign == ALIGN_CENTER )
        {
            _renderPos.x() = cursor.x() + 0.5*(parentSize.x() - _renderSize.x());
        }
        else if ( _halign == ALIGN_RIGHT )
        {
            _renderPos.x() = cursor.x() + parentSize.x() - margin().right() - _renderSize.x();
        }
        else
        {
            _renderPos.x() = cursor.x() + margin().left();
        }
    }

    if ( _y.isSet() )
    {
        _renderPos.y() = cursor.y() + margin().top() + *y();
    }
    else
    {
        if ( _valign == ALIGN_CENTER )
        {
            _renderPos.y() = cursor.y() + 0.5*(parentSize.y() - _renderSize.y());
        }
        else if ( _valign == ALIGN_BOTTOM )
        {
            _renderPos.y() = cursor.y() + parentSize.y() - margin().bottom() - _renderSize.y();
        }
        else
        {
            _renderPos.y() = cursor.y() + margin().top();
        }
    }
}

void
Control::draw(const ControlContext& cx, DrawableList& out )
{
    // by default, rendering a Control directly results in a colored quad. Usually however
    // you will not render a Control directly, but rather one of its subclasses.
    if ( visible() == true && !(_backColor.isSet() && _backColor->a() == 0) && _renderSize.x() > 0 && _renderSize.y() > 0 )
    {
        float vph = cx._vp->height();

        osg::Geometry* geom = new osg::Geometry();

        osg::Vec3Array* verts = new osg::Vec3Array(4);
        geom->setVertexArray( verts );
        (*verts)[0].set( _renderPos.x(), vph - _renderPos.y(), 0 );
        (*verts)[1].set( _renderPos.x(), vph - _renderPos.y() - _renderSize.y(), 0 );
        (*verts)[2].set( _renderPos.x() + _renderSize.x(), vph - _renderPos.y() - _renderSize.y(), 0 );
        (*verts)[3].set( _renderPos.x() + _renderSize.x(), vph - _renderPos.y(), 0 );
        geom->addPrimitiveSet( new osg::DrawArrays( GL_QUADS, 0, 4 ) );

        osg::Vec4Array* colors = new osg::Vec4Array(1);
        (*colors)[0] = _backColor.value();
        geom->setColorArray( colors );
        geom->setColorBinding( osg::Geometry::BIND_OVERALL );

        out.push_back( geom );
    }
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

        osg::BoundingBox bbox = t->getTextBB();
        if ( cx._viewContextID != ~0 )
        {
            //the Text's autoTransformCache matrix puts some mojo on the bounding box
            osg::Matrix m = t->getATMatrix( cx._viewContextID );
            osg::Vec3 bmin = osg::Vec3( bbox.xMin(), bbox.yMin(), bbox.zMin() ) * m;
            osg::Vec3 bmax = osg::Vec3( bbox.xMax(), bbox.yMax(), bbox.zMax() ) * m;
            _renderSize.set( bmax.x() - bmin.x(), bmax.y() - bmin.y() );
        }
        else
        {
            _renderSize.set( bbox.xMax()-bbox.xMin(), bbox.yMax()-bbox.yMin() );
        }

        _drawable = t;

        out_size.set(
            margin().left() + margin().right() + _renderSize.x(),
            margin().top() + margin().bottom() + _renderSize.y() );
    }
    else
    {
        out_size.set(0,0);
    }

    _dirty = false;
}

void
LabelControl::draw( const ControlContext& cx, DrawableList& out )
{
    if ( _drawable.valid() && visible() == true )
    {
        //TODO: support alignment here
        float vph = cx._vp->height();

        LabelText* t = static_cast<LabelText*>( _drawable.get() );
        osg::BoundingBox bbox = t->getTextBB();
        t->setPosition( osg::Vec3( _renderPos.x(), vph - _renderPos.y(), 0 ) );
        out.push_back( _drawable.get() );

        Control::draw( cx, out );
    }
}

// ---------------------------------------------------------------------------

ImageControl::ImageControl( osg::Image* image )
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
ImageControl::calcSize(const ControlContext& cx, osg::Vec2f& out_size)
{
    if ( visible() == true )
    {
        _renderSize.set( 0, 0 );

        if ( _image.valid() )
        {
            _renderSize.set( _image->s(), _image->t() );
        }

        _renderSize.set(
            osg::maximum( _renderSize.x(), width().value() ),
            osg::maximum( _renderSize.y(), height().value() ) );

        out_size.set(
            margin().left() + margin().right() + _renderSize.x(),
            margin().top() + margin().bottom() + _renderSize.y() );

        _dirty = false;
    }
    else
    {
        out_size.set(0,0);
    }
}

void
ImageControl::draw( const ControlContext& cx, DrawableList& out )
{
    if ( visible() == true )
    {
        //TODO: this is not precisely correct..images get deformed slightly..
        osg::Geometry* g = new osg::Geometry();

        float rx = osg::round( _renderPos.x() );
        float ry = osg::round( _renderPos.y() );
        float vph = cx._vp->height();

        osg::Vec3Array* verts = new osg::Vec3Array(4);
        g->setVertexArray( verts );
        (*verts)[0].set( rx, vph - ry, 0 );
        (*verts)[1].set( rx, vph - ry - _renderSize.y(), 0 );
        (*verts)[2].set( rx + _renderSize.x(), vph - ry - _renderSize.y(), 0 );
        (*verts)[3].set( rx + _renderSize.x(), vph - ry, 0 );
        g->addPrimitiveSet( new osg::DrawArrays( GL_QUADS, 0, 4 ) );

        osg::Vec4Array* c = new osg::Vec4Array(1);
        (*c)[0] = osg::Vec4f(1,1,1,1);
        g->setColorArray( c );
        g->setColorBinding( osg::Geometry::BIND_OVERALL );

        osg::Vec2Array* t = new osg::Vec2Array(4);
        (*t)[0].set( 0, _renderSize.y()-1 );
        (*t)[1].set( 0, 0 );
        (*t)[2].set( _renderSize.x()-1, 0 );
        (*t)[3].set( _renderSize.x()-1, _renderSize.y()-1 );
        g->setTexCoordArray( 0, t );

        osg::TextureRectangle* texrec = new osg::TextureRectangle( _image.get() );
        texrec->setFilter( osg::Texture::MIN_FILTER, osg::Texture::NEAREST );
        texrec->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
        g->getOrCreateStateSet()->setTextureAttributeAndModes( 0, texrec, osg::StateAttribute::ON );

        osg::TexEnv* texenv = new osg::TexEnv( osg::TexEnv::MODULATE );
        g->getStateSet()->setTextureAttributeAndModes( 0, texenv, osg::StateAttribute::ON );

        out.push_back( g );
    }
}

// ---------------------------------------------------------------------------

Frame::Frame()
{
    //nop
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

        GeometryRasterizer ras( _renderSize.x(), _renderSize.y() );
        ras.draw( geom.get() );

        osg::Image* image = ras.finalize();
        if ( image )
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

            GeometryRasterizer ras( _renderSize.x(), _renderSize.y() );
            ras.draw( geom.get(), backColor().value() );

            osg::Image* image = ras.finalize();
            if ( image )
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
_padding( Gutter(1) ),
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
Container::setPadding( const Gutter& value )
{
    if ( value != _padding ) {
        _padding = value;
        dirty();
    }
}

void
Container::setPadding( float value ) {
    Gutter g(value);
    if ( g != _padding ) {
        _padding = g;
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

        _dirty = false;  
    }
}

void
Container::calcPos(const ControlContext& context, const osg::Vec2f& cursor, const osg::Vec2f& parentSize)
{
    Control::calcPos( context, cursor, parentSize );

    // process the frame.. it's not a child of the container
    if ( visible() == true && _frame.valid() )
    {
        _frame->calcPos( context, _renderPos, parentSize );
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

// ---------------------------------------------------------------------------

VBox::VBox()
{
    //nop
}

void
VBox::addControl( Control* control )
{
    _controls.push_back( control );
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

    osg::Vec2f childCursor( 
        _renderPos.x() + padding().left(),
        _renderPos.y() + padding().top() );

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
HBox::addControl( Control* control )
{
    _controls.push_back( control );
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

    osg::Vec2f childCursor( 
        _renderPos.x() + padding().left(),
        _renderPos.y() + padding().top() );

    // collect all the members, growing the container is its orientation.
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

// binds the update traversal to the update() method
struct ControlUpdater : public osg::NodeCallback
{
    void operator()( osg::Node* node, osg::NodeVisitor* nv )
    {
        static_cast<ControlSurface*>(node)->update();
    }
};

// This handler keeps an eye on the viewport and informs the control surface when it changes.
// We need this info since controls position from the upper-left corner.
struct ViewportHandler : public osgGA::GUIEventHandler
{
    ViewportHandler( ControlSurface* cs ) : _cs(cs), _width(0), _height(0), _first(true) { }

    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
    {
        if ( ea.getEventType() == osgGA::GUIEventAdapter::RESIZE || _first )
        {
            int w = ea.getWindowWidth();
            int h = ea.getWindowHeight();
            if ( w != _width || h != _height )
            {
                _cs->setProjectionMatrix(osg::Matrix::ortho2D( 0, w-1, 0, h-1 ) );

                ControlContext cx;
                cx._vp = new osg::Viewport( 0, 0, w, h );
                cx._viewContextID = aa.asView()->getCamera()->getGraphicsContext()->getState()->getContextID();
                _cs->setControlContext( cx );

                _width = w;
                _height = h;
            }
            if ( w != 0 && h != 0 )
            {
                _first = false;
            }
        }
        return false;
    }
    ControlSurface* _cs;
    int _width, _height;
    bool _first;
};

// ---------------------------------------------------------------------------

ControlSurface::ControlSurface( osgViewer::View* view ) :
_contextDirty(true)
{
    view->addEventHandler( new ViewportHandler(this) );

    setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    setViewMatrix(osg::Matrix::identity());
    setClearMask(GL_DEPTH_BUFFER_BIT);
    setRenderOrder(osg::Camera::POST_RENDER); 
    setAllowEventFocus( true );
    
    this->setUpdateCallback( new ControlUpdater );

    getOrCreateStateSet()->setMode( GL_LIGHTING, 0 );
    getOrCreateStateSet()->setMode( GL_BLEND, 1 );

    getOrCreateStateSet()->setAttributeAndModes( new osg::Depth( osg::Depth::LEQUAL, 0, 1, false ) );
}

void
ControlSurface::addControl( Control* control )
{
    osg::Geode* geode = new osg::Geode();
    _geodeTable[control] = geode;
    addChild( geode );
    control->dirty();
    _controls.push_back( control );
}

void
ControlSurface::removeControl( Control* control )
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

void
ControlSurface::update()
{
    //int bin = 999999;
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
      //          j->get()->getOrCreateStateSet()->setRenderBinDetails( bin++, "RenderBin" );
                geode->addDrawable( j->get() );
            }
        }
    }
    _contextDirty = false;
}

void
ControlSurface::setControlContext( const ControlContext& cx )
{
    _context = cx;
    _contextDirty = true;
}
