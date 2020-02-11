/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2020 Pelican Mapping
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
#include <osgEarth/TextSymbol>
#include <osgEarth/Style>

using namespace osgEarth;

OSGEARTH_REGISTER_SIMPLE_SYMBOL(text, TextSymbol);

TextSymbol::TextSymbol(const TextSymbol& rhs,const osg::CopyOp& copyop):
Symbol(rhs, copyop),
_fill(rhs._fill),
_halo(rhs._halo),
_haloOffset(rhs._haloOffset),
_haloBackdropType(rhs._haloBackdropType),
_haloImplementation(rhs._haloImplementation),
_font(rhs._font),
_size(rhs._size),
_content(rhs._content),
_priority(rhs._priority),
_pixelOffset(rhs._pixelOffset),
_onScreenRotation(rhs._onScreenRotation),
_geographicCourse(rhs._geographicCourse),
_provider(rhs._provider),
_encoding(rhs._encoding),
_alignment(rhs._alignment),
_layout(rhs._layout),
_declutter(rhs._declutter),
_occlusionCull(rhs._occlusionCull),
_occlusionCullAltitude(rhs._occlusionCullAltitude)
{
}

TextSymbol::TextSymbol( const Config& conf ) :
Symbol                ( conf ),
_fill                 ( Fill( 1, 1, 1, 1 ) ),
_halo                 ( Stroke( 0.3, 0.3, 0.3, 1) ),
_haloOffset           ( 0.0625f ),
_haloBackdropType     ( osgText::Text::OUTLINE ),
_haloImplementation   ( osgText::Text::DELAYED_DEPTH_WRITES ),
_size                 ( 16.0f ),
_alignment            ( ALIGN_BASE_LINE ),
_layout               ( LAYOUT_LEFT_TO_RIGHT ),
_provider             ( "annotation" ),
_encoding             ( ENCODING_ASCII ),
_declutter            ( true ),
_occlusionCull        ( false ),
_occlusionCullAltitude( 200000 ),
_onScreenRotation     ( 0.0 ),
_geographicCourse     ( 0.0 )
{
    mergeConfig(conf);
}

Config
TextSymbol::getConfig() const
{
    Config conf = Symbol::getConfig();
    conf.key() = "text";
    conf.set( "fill", _fill );
    conf.set( "halo", _halo );
    conf.set( "halo_offset", _haloOffset );
    conf.set( "halo_backdrop_type", "bottom_right",  _haloBackdropType, osgText::Text::DROP_SHADOW_BOTTOM_RIGHT );
    conf.set( "halo_backdrop_type", "center_right",  _haloBackdropType, osgText::Text::DROP_SHADOW_CENTER_RIGHT );
    conf.set( "halo_backdrop_type", "top_right",     _haloBackdropType, osgText::Text::DROP_SHADOW_TOP_RIGHT );
    conf.set( "halo_backdrop_type", "bottom_center", _haloBackdropType, osgText::Text::DROP_SHADOW_BOTTOM_CENTER );
    conf.set( "halo_backdrop_type", "top_center",    _haloBackdropType, osgText::Text::DROP_SHADOW_TOP_CENTER );
    conf.set( "halo_backdrop_type", "bottom_left",   _haloBackdropType, osgText::Text::DROP_SHADOW_BOTTOM_LEFT );
    conf.set( "halo_backdrop_type", "center_left",   _haloBackdropType, osgText::Text::DROP_SHADOW_CENTER_LEFT );
    conf.set( "halo_backdrop_type", "top_left",      _haloBackdropType, osgText::Text::DROP_SHADOW_TOP_LEFT );
    conf.set( "halo_backdrop_type", "outline",       _haloBackdropType, osgText::Text::OUTLINE );
    conf.set( "halo_backdrop_type", "none",          _haloBackdropType, osgText::Text::NONE );
    conf.set( "halo_implementation", "polygon_offset",       _haloImplementation, osgText::Text::POLYGON_OFFSET );
    conf.set( "halo_implementation", "no_depth_buffer",      _haloImplementation, osgText::Text::NO_DEPTH_BUFFER );
    conf.set( "halo_implementation", "depth_range",          _haloImplementation, osgText::Text::DEPTH_RANGE );
    conf.set( "halo_implementation", "stencil_buffer",       _haloImplementation, osgText::Text::STENCIL_BUFFER );
    conf.set( "halo_implementation", "delayed_depth_writes", _haloImplementation, osgText::Text::DELAYED_DEPTH_WRITES );
    conf.set( "font", _font );
    conf.set( "size", _size );
    conf.set( "content", _content );
    conf.set( "priority", _priority );

    conf.set( "encoding", "ascii", _encoding, ENCODING_ASCII );
    conf.set( "encoding", "utf8",  _encoding, ENCODING_UTF8 );
    conf.set( "encoding", "utf16", _encoding, ENCODING_UTF16 );
    conf.set( "encoding", "utf32", _encoding, ENCODING_UTF32 );

    conf.set( "alignment", "left_top",                _alignment, ALIGN_LEFT_TOP );
    conf.set( "alignment", "left_center",             _alignment, ALIGN_LEFT_CENTER );
    conf.set( "alignment", "left_bottom",             _alignment, ALIGN_LEFT_BOTTOM );
    conf.set( "alignment", "center_top",              _alignment, ALIGN_CENTER_TOP );
    conf.set( "alignment", "center_center",           _alignment, ALIGN_CENTER_CENTER );
    conf.set( "alignment", "center_bottom",           _alignment, ALIGN_CENTER_BOTTOM );
    conf.set( "alignment", "right_top",               _alignment, ALIGN_RIGHT_TOP );
    conf.set( "alignment", "right_center",            _alignment, ALIGN_RIGHT_CENTER );
    conf.set( "alignment", "right_bottom",            _alignment, ALIGN_RIGHT_BOTTOM );
    conf.set( "alignment", "left_base_line",          _alignment, ALIGN_LEFT_BASE_LINE );
    conf.set( "alignment", "center_base_line",        _alignment, ALIGN_CENTER_BASE_LINE );
    conf.set( "alignment", "right_base_line",         _alignment, ALIGN_RIGHT_BASE_LINE );
    conf.set( "alignment", "left_bottom_base_line",   _alignment, ALIGN_LEFT_BOTTOM_BASE_LINE );
    conf.set( "alignment", "center_bottom_base_line", _alignment, ALIGN_CENTER_BOTTOM_BASE_LINE );
    conf.set( "alignment", "right_bottom_base_line",  _alignment, ALIGN_RIGHT_BOTTOM_BASE_LINE );
    conf.set( "alignment", "base_line",               _alignment, ALIGN_BASE_LINE );

    conf.set( "layout", "ltr",  _layout, LAYOUT_LEFT_TO_RIGHT );
    conf.set( "layout", "rtl",  _layout, LAYOUT_RIGHT_TO_LEFT );
    conf.set( "layout", "vertical",  _layout, LAYOUT_VERTICAL );

    conf.set( "declutter", _declutter );

    conf.set( "provider", _provider );
    if ( _pixelOffset.isSet() ) {
        conf.set( "pixel_offset_x", toString(_pixelOffset->x()) );
        conf.set( "pixel_offset_y", toString(_pixelOffset->y()) );
    }

    conf.set( "rotation", _onScreenRotation );
    conf.set( "geographic-course", _geographicCourse );

    conf.set( "text-occlusion-cull", _occlusionCull );
    conf.set( "text-occlusion-cull-altitude", _occlusionCullAltitude );

    return conf;
}

void
TextSymbol::mergeConfig( const Config& conf )
{
    conf.get( "fill", _fill );
    conf.get( "halo", _halo );
    conf.get( "halo_offset", _haloOffset );
    conf.get( "halo_backdrop_type", "right_bottom",  _haloBackdropType, osgText::Text::DROP_SHADOW_BOTTOM_RIGHT );
    conf.get( "halo_backdrop_type", "right_center",  _haloBackdropType, osgText::Text::DROP_SHADOW_CENTER_RIGHT );
    conf.get( "halo_backdrop_type", "right_top",     _haloBackdropType, osgText::Text::DROP_SHADOW_TOP_RIGHT );
    conf.get( "halo_backdrop_type", "center_bottom", _haloBackdropType, osgText::Text::DROP_SHADOW_BOTTOM_CENTER );
    conf.get( "halo_backdrop_type", "center_top",    _haloBackdropType, osgText::Text::DROP_SHADOW_TOP_CENTER );
    conf.get( "halo_backdrop_type", "left_bottom",   _haloBackdropType, osgText::Text::DROP_SHADOW_BOTTOM_LEFT );
    conf.get( "halo_backdrop_type", "left_center",   _haloBackdropType, osgText::Text::DROP_SHADOW_CENTER_LEFT );
    conf.get( "halo_backdrop_type", "left_top",      _haloBackdropType, osgText::Text::DROP_SHADOW_TOP_LEFT );
    conf.get( "halo_backdrop_type", "outline",       _haloBackdropType, osgText::Text::OUTLINE );
    conf.get( "halo_backdrop_type", "none",          _haloBackdropType, osgText::Text::NONE );
    conf.get( "halo_implementation", "polygon_offset",       _haloImplementation, osgText::Text::POLYGON_OFFSET );
    conf.get( "halo_implementation", "no_depth_buffer",      _haloImplementation, osgText::Text::NO_DEPTH_BUFFER );
    conf.get( "halo_implementation", "depth_range",          _haloImplementation, osgText::Text::DEPTH_RANGE );
    conf.get( "halo_implementation", "stencil_buffer",       _haloImplementation, osgText::Text::STENCIL_BUFFER );
    conf.get( "halo_implementation", "delayed_depth_writes", _haloImplementation, osgText::Text::DELAYED_DEPTH_WRITES );
    conf.get( "font", _font );
    conf.get( "size", _size );
    conf.get( "content", _content );
    conf.get( "priority", _priority );

    conf.get( "encoding", "ascii", _encoding, ENCODING_ASCII );
    conf.get( "encoding", "utf8",  _encoding, ENCODING_UTF8 );
    conf.get( "encoding", "utf16", _encoding, ENCODING_UTF16 );
    conf.get( "encoding", "utf32", _encoding, ENCODING_UTF32 );

    conf.get( "alignment", "left_top",                _alignment, ALIGN_LEFT_TOP );
    conf.get( "alignment", "left_center",             _alignment, ALIGN_LEFT_CENTER );
    conf.get( "alignment", "left_bottom",             _alignment, ALIGN_LEFT_BOTTOM );
    conf.get( "alignment", "center_top",              _alignment, ALIGN_CENTER_TOP );
    conf.get( "alignment", "center_center",           _alignment, ALIGN_CENTER_CENTER );
    conf.get( "alignment", "center_bottom",           _alignment, ALIGN_CENTER_BOTTOM );
    conf.get( "alignment", "right_top",               _alignment, ALIGN_RIGHT_TOP );
    conf.get( "alignment", "right_center",            _alignment, ALIGN_RIGHT_CENTER );
    conf.get( "alignment", "right_bottom",            _alignment, ALIGN_RIGHT_BOTTOM );
    conf.get( "alignment", "left_base_line",          _alignment, ALIGN_LEFT_BASE_LINE );
    conf.get( "alignment", "center_base_line",        _alignment, ALIGN_CENTER_BASE_LINE );
    conf.get( "alignment", "right_base_line",         _alignment, ALIGN_RIGHT_BASE_LINE );
    conf.get( "alignment", "left_bottom_base_line",   _alignment, ALIGN_LEFT_BOTTOM_BASE_LINE );
    conf.get( "alignment", "center_bottom_base_line", _alignment, ALIGN_CENTER_BOTTOM_BASE_LINE );
    conf.get( "alignment", "right_bottom_base_line",  _alignment, ALIGN_RIGHT_BOTTOM_BASE_LINE );
    conf.get( "alignment", "base_line" ,              _alignment, ALIGN_BASE_LINE );

    conf.get( "layout", "ltr",  _layout, LAYOUT_LEFT_TO_RIGHT );
    conf.get( "layout", "rtl",  _layout, LAYOUT_RIGHT_TO_LEFT );
    conf.get( "layout", "vertical",  _layout, LAYOUT_VERTICAL );

    conf.get( "declutter", _declutter );

    conf.get( "provider", _provider );
    if ( conf.hasValue( "pixel_offset_x" ) )
        _pixelOffset = osg::Vec2s( conf.value<short>("pixel_offset_x",0), 0 );
    if ( conf.hasValue( "pixel_offset_y" ) )
        _pixelOffset = osg::Vec2s( _pixelOffset->x(), conf.value<short>("pixel_offset_y",0) );

    conf.get( "rotation", _onScreenRotation );
    conf.get( "geographic-course", _geographicCourse );

    conf.get( "text-occlusion-cull", _occlusionCull );
    conf.get( "text-occlusion-cull-altitude", _occlusionCullAltitude );
}


void
TextSymbol::parseSLD(const Config& c, Style& style)
{
    TextSymbol defaults;

    if ( match(c.key(), "text-fill") || match(c.key(), "text-color") ) {
        style.getOrCreate<TextSymbol>()->fill()->color() = Color(c.value());
    }
    else if ( match(c.key(), "text-fill-opacity") ) {
        style.getOrCreate<TextSymbol>()->fill()->color().a() = as<float>( c.value(), 1.0f );
    }
    else if ( match(c.key(), "text-size") ) {
        style.getOrCreate<TextSymbol>()->size() = NumericExpression( c.value() );
    }
    else if ( match(c.key(), "text-font") ) {
        style.getOrCreate<TextSymbol>()->font() = c.value();
    }
    else if ( match(c.key(), "text-halo") || match(c.key(), "text-halo-color") ) {
        style.getOrCreate<TextSymbol>()->halo()->color() = htmlColorToVec4f( c.value() );
    }
    else if ( match(c.key(), "text-halo-offset") ) {
        style.getOrCreate<TextSymbol>()->haloOffset() = as<float>(c.value(), defaults.haloOffset().get() );
    }
    else if ( match(c.key(), "text-halo-backdrop-type") ) {
        if      ( match(c.value(), "right-bottom") )
            style.getOrCreate<TextSymbol>()->haloBackdropType() = osgText::Text::DROP_SHADOW_BOTTOM_RIGHT;
        else if ( match(c.value(), "right-center") )
            style.getOrCreate<TextSymbol>()->haloBackdropType() = osgText::Text::DROP_SHADOW_CENTER_RIGHT;
        else if ( match(c.value(), "right-top") )
            style.getOrCreate<TextSymbol>()->haloBackdropType() = osgText::Text::DROP_SHADOW_TOP_RIGHT;
        else if ( match(c.value(), "center-bottom") )
            style.getOrCreate<TextSymbol>()->haloBackdropType() = osgText::Text::DROP_SHADOW_BOTTOM_CENTER;
        else if ( match(c.value(), "center-top") )
            style.getOrCreate<TextSymbol>()->haloBackdropType() = osgText::Text::DROP_SHADOW_TOP_CENTER;
        else if ( match(c.value(), "left-bottom") )
            style.getOrCreate<TextSymbol>()->haloBackdropType() = osgText::Text::DROP_SHADOW_BOTTOM_LEFT;
        else if ( match(c.value(), "left-center") )
            style.getOrCreate<TextSymbol>()->haloBackdropType() = osgText::Text::DROP_SHADOW_CENTER_LEFT;
        else if ( match(c.value(), "left-top") )
            style.getOrCreate<TextSymbol>()->haloBackdropType() = osgText::Text::DROP_SHADOW_TOP_LEFT;
        else if ( match(c.value(), "outline") )
            style.getOrCreate<TextSymbol>()->haloBackdropType() = osgText::Text::OUTLINE;
        else if ( match(c.value(), "none") )
            style.getOrCreate<TextSymbol>()->haloBackdropType() = osgText::Text::NONE;
    }
    else if ( match(c.key(), "text-halo-implementation") ) {
        if      ( match(c.value(), "polygon-offset") )
            style.getOrCreate<TextSymbol>()->haloImplementation() = osgText::Text::POLYGON_OFFSET;
        else if ( match(c.value(), "no-depth-buffer") )
            style.getOrCreate<TextSymbol>()->haloImplementation() = osgText::Text::NO_DEPTH_BUFFER;
        else if ( match(c.value(), "depth-range") )
            style.getOrCreate<TextSymbol>()->haloImplementation() = osgText::Text::DEPTH_RANGE;
        else if ( match(c.value(), "stencil-buffer") )
            style.getOrCreate<TextSymbol>()->haloImplementation() = osgText::Text::STENCIL_BUFFER;
        else if ( match(c.value(), "delayed-depth-writes") )
            style.getOrCreate<TextSymbol>()->haloImplementation() = osgText::Text::DELAYED_DEPTH_WRITES;
    }
    else if ( match(c.key(), "text-align") ) {
        if      ( match(c.value(), "left-top") )
            style.getOrCreate<TextSymbol>()->alignment() = TextSymbol::ALIGN_LEFT_TOP;
        else if ( match(c.value(), "left-center") )
            style.getOrCreate<TextSymbol>()->alignment() = TextSymbol::ALIGN_LEFT_CENTER;
        else if ( match(c.value(), "left-bottom") )
            style.getOrCreate<TextSymbol>()->alignment() = TextSymbol::ALIGN_LEFT_BOTTOM;
        else if ( match(c.value(), "center-top")  )
            style.getOrCreate<TextSymbol>()->alignment() = TextSymbol::ALIGN_CENTER_TOP;
        else if ( match(c.value(), "center-center") )
            style.getOrCreate<TextSymbol>()->alignment() = TextSymbol::ALIGN_CENTER_CENTER;
        else if ( match(c.value(), "center-bottom") )
            style.getOrCreate<TextSymbol>()->alignment() = TextSymbol::ALIGN_CENTER_BOTTOM;
        else if ( match(c.value(), "right-top") )
            style.getOrCreate<TextSymbol>()->alignment() = TextSymbol::ALIGN_RIGHT_TOP;
        else if ( match(c.value(), "right-center") )
            style.getOrCreate<TextSymbol>()->alignment() = TextSymbol::ALIGN_RIGHT_CENTER;
        else if ( match(c.value(), "right-bottom") )
            style.getOrCreate<TextSymbol>()->alignment() = TextSymbol::ALIGN_RIGHT_BOTTOM;
        else if ( match(c.value(), "left-base-line") )
            style.getOrCreate<TextSymbol>()->alignment() = TextSymbol::ALIGN_LEFT_BASE_LINE;
        else if ( match(c.value(), "center-base-line") )
            style.getOrCreate<TextSymbol>()->alignment() = TextSymbol::ALIGN_CENTER_BASE_LINE;
        else if ( match(c.value(), "right-base-line") )
            style.getOrCreate<TextSymbol>()->alignment() = TextSymbol::ALIGN_RIGHT_BASE_LINE;
        else if ( match(c.value(), "left-bottom-base-line") )
            style.getOrCreate<TextSymbol>()->alignment() = TextSymbol::ALIGN_LEFT_BOTTOM_BASE_LINE;
        else if ( match(c.value(), "center-bottom-base-line") )
            style.getOrCreate<TextSymbol>()->alignment() = TextSymbol::ALIGN_CENTER_BOTTOM_BASE_LINE;
        else if ( match(c.value(), "right-bottom-base-line") )
            style.getOrCreate<TextSymbol>()->alignment() = TextSymbol::ALIGN_RIGHT_BOTTOM_BASE_LINE;
        else if ( match(c.value(), "base-line" ) )
            style.getOrCreate<TextSymbol>()->alignment() = TextSymbol::ALIGN_BASE_LINE;
    }
    else if ( match(c.key(), "text-layout") ) {
        if ( match(c.value(), "ltr") )
            style.getOrCreate<TextSymbol>()->layout() = TextSymbol::LAYOUT_LEFT_TO_RIGHT;
        else if ( match(c.value(), "rtl" ) )
            style.getOrCreate<TextSymbol>()->layout() = TextSymbol::LAYOUT_RIGHT_TO_LEFT;
        else if ( match(c.value(), "vertical" ) )
            style.getOrCreate<TextSymbol>()->layout() = TextSymbol::LAYOUT_VERTICAL;
    }
    else if ( match(c.key(), "text-content") || match(c.key(), "text") ) {
        style.getOrCreate<TextSymbol>()->content() = StringExpression( c.value() );
    }
    else if ( match(c.key(), "text-priority") ) {
        style.getOrCreate<TextSymbol>()->priority() = NumericExpression( c.value() );
    }
    else if ( match(c.key(), "text-provider") ) {
        style.getOrCreate<TextSymbol>()->provider() = c.value();
    }
    else if ( match(c.key(), "text-encoding") ) {
        if      (match(c.value(), "utf-8"))
            style.getOrCreate<TextSymbol>()->encoding() = TextSymbol::ENCODING_UTF8;
        else if (match(c.value(), "utf-16"))
            style.getOrCreate<TextSymbol>()->encoding() = TextSymbol::ENCODING_UTF16;
        else if (match(c.value(), "utf-32"))
            style.getOrCreate<TextSymbol>()->encoding() = TextSymbol::ENCODING_UTF32;
        else if (match(c.value(), "ascii"))
            style.getOrCreate<TextSymbol>()->encoding() = TextSymbol::ENCODING_ASCII;
        else
            style.getOrCreate<TextSymbol>()->encoding() = TextSymbol::ENCODING_ASCII;
    }
    else if ( match(c.key(), "text-declutter") ) {
        style.getOrCreate<TextSymbol>()->declutter() = as<bool>(c.value(), defaults.declutter().get() );
    }
    else if ( match(c.key(), "text-occlusion-cull") ) {
        style.getOrCreate<TextSymbol>()->occlusionCull() = as<bool>(c.value(), defaults.occlusionCull().get() );
    }
    else if ( match(c.key(), "text-occlusion-cull-altitude") ) {
        style.getOrCreate<TextSymbol>()->occlusionCullAltitude() = as<double>(c.value(), defaults.occlusionCullAltitude().get() );
    }
    else if ( match(c.key(), "text-script") ) {
        style.getOrCreate<TextSymbol>()->script() = StringExpression(c.value());
    }
    else if ( match(c.key(), "text-offset-x") ) {
        style.getOrCreate<TextSymbol>()->pixelOffset()->x() = as<double>(c.value(), defaults.pixelOffset()->x() );
    }
    else if ( match(c.key(), "text-offset-y") ) {
        style.getOrCreate<TextSymbol>()->pixelOffset()->y() = as<double>(c.value(), defaults.pixelOffset()->y() );
    }
    else if ( match(c.key(), "text-rotation") ) {
        style.getOrCreate<TextSymbol>()->onScreenRotation() = NumericExpression( c.value() );
    }
    else if ( match(c.key(), "text-geographic-course") ) {
        style.getOrCreate<TextSymbol>()->geographicCourse() = NumericExpression( c.value() );
    }
}
