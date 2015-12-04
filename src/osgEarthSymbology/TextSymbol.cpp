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
#include <osgEarthSymbology/TextSymbol>
#include <osgEarthSymbology/Style>

using namespace osgEarth;
using namespace osgEarth::Symbology;

OSGEARTH_REGISTER_SIMPLE_SYMBOL(text, TextSymbol);

TextSymbol::TextSymbol(const TextSymbol& rhs,const osg::CopyOp& copyop):
Symbol(rhs, copyop),
_fill(rhs._fill),
_halo(rhs._halo),
_haloOffset(rhs._haloOffset),
_font(rhs._font),
_size(rhs._size),
_content(rhs._content),
_priority(rhs._priority),
_removeDuplicateLabels(rhs._removeDuplicateLabels),
_pixelOffset(rhs._pixelOffset),
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
_haloOffset           ( 0.07f ),
_size                 ( 16.0f ),
_removeDuplicateLabels( false ),
_provider             ( "annotation" ),
_encoding             ( ENCODING_ASCII ),
_alignment            ( ALIGN_BASE_LINE ),
_layout               ( LAYOUT_LEFT_TO_RIGHT ),
_declutter            ( true ),
_occlusionCull        ( false ),
_occlusionCullAltitude( 200000 )
{
    mergeConfig(conf);
}

Config 
TextSymbol::getConfig() const
{
    Config conf = Symbol::getConfig();
    conf.key() = "text";
    conf.addObjIfSet( "fill", _fill );
    conf.addObjIfSet( "halo", _halo );
    conf.addIfSet( "halo_offset", _haloOffset );
    conf.addIfSet( "font", _font );
    conf.addObjIfSet( "size", _size );
    conf.addObjIfSet( "content", _content );
    conf.addObjIfSet( "priority", _priority );
    conf.addIfSet( "remove_duplicate_labels", _removeDuplicateLabels );

    conf.addIfSet( "encoding", "ascii", _encoding, ENCODING_ASCII );
    conf.addIfSet( "encoding", "utf8",  _encoding, ENCODING_UTF8 );
    conf.addIfSet( "encoding", "utf16", _encoding, ENCODING_UTF16 );
    conf.addIfSet( "encoding", "utf32", _encoding, ENCODING_UTF32 );

    conf.addIfSet( "alignment", "left_top",                _alignment, ALIGN_LEFT_TOP );
    conf.addIfSet( "alignment", "left_center",             _alignment, ALIGN_LEFT_CENTER );
    conf.addIfSet( "alignment", "left_bottom",             _alignment, ALIGN_LEFT_BOTTOM );
    conf.addIfSet( "alignment", "center_top",              _alignment, ALIGN_CENTER_TOP );
    conf.addIfSet( "alignment", "center_center",           _alignment, ALIGN_CENTER_CENTER );
    conf.addIfSet( "alignment", "center_bottom",           _alignment, ALIGN_CENTER_BOTTOM );
    conf.addIfSet( "alignment", "right_top",               _alignment, ALIGN_RIGHT_TOP );
    conf.addIfSet( "alignment", "right_center",            _alignment, ALIGN_RIGHT_CENTER );
    conf.addIfSet( "alignment", "right_bottom",            _alignment, ALIGN_RIGHT_BOTTOM );
    conf.addIfSet( "alignment", "left_base_line",          _alignment, ALIGN_LEFT_BASE_LINE );
    conf.addIfSet( "alignment", "center_base_line",        _alignment, ALIGN_CENTER_BASE_LINE );
    conf.addIfSet( "alignment", "right_base_line",         _alignment, ALIGN_RIGHT_BASE_LINE );
    conf.addIfSet( "alignment", "left_bottom_base_line",   _alignment, ALIGN_LEFT_BOTTOM_BASE_LINE );
    conf.addIfSet( "alignment", "center_bottom_base_line", _alignment, ALIGN_CENTER_BOTTOM_BASE_LINE );
    conf.addIfSet( "alignment", "right_bottom_base_line",  _alignment, ALIGN_RIGHT_BOTTOM_BASE_LINE );
    conf.addIfSet( "alignment", "base_line",               _alignment, ALIGN_BASE_LINE );

    conf.addIfSet( "layout", "ltr",  _layout, LAYOUT_LEFT_TO_RIGHT );
    conf.addIfSet( "layout", "rtl",  _layout, LAYOUT_RIGHT_TO_LEFT );
    conf.addIfSet( "layout", "vertical",  _layout, LAYOUT_VERTICAL );

    conf.addIfSet( "declutter", _declutter );

    conf.addIfSet( "provider", _provider );
    if ( _pixelOffset.isSet() ) {
        conf.add( "pixel_offset_x", toString(_pixelOffset->x()) );
        conf.add( "pixel_offset_y", toString(_pixelOffset->y()) );
    }

    conf.addIfSet( "text-occlusion-cull", _occlusionCull );
    conf.addIfSet( "text-occlusion-cull-altitude", _occlusionCullAltitude );

    return conf;
}

void 
TextSymbol::mergeConfig( const Config& conf )
{
    conf.getObjIfSet( "fill", _fill );
    conf.getObjIfSet( "halo", _halo );
    conf.getIfSet( "halo_offset", _haloOffset );
    conf.getIfSet( "font", _font );
    conf.getObjIfSet( "size", _size );
    conf.getObjIfSet( "content", _content );
    conf.getObjIfSet( "priority", _priority );
    conf.getIfSet( "remove_duplicate_labels", _removeDuplicateLabels );

    conf.getIfSet( "encoding", "ascii", _encoding, ENCODING_ASCII );
    conf.getIfSet( "encoding", "utf8",  _encoding, ENCODING_UTF8 );
    conf.getIfSet( "encoding", "utf16", _encoding, ENCODING_UTF16 );
    conf.getIfSet( "encoding", "utf32", _encoding, ENCODING_UTF32 );

    conf.getIfSet( "alignment", "left_top",                _alignment, ALIGN_LEFT_TOP );
    conf.getIfSet( "alignment", "left_center",             _alignment, ALIGN_LEFT_CENTER );
    conf.getIfSet( "alignment", "left_bottom",             _alignment, ALIGN_LEFT_BOTTOM );
    conf.getIfSet( "alignment", "center_top",              _alignment, ALIGN_CENTER_TOP );
    conf.getIfSet( "alignment", "center_center",           _alignment, ALIGN_CENTER_CENTER );
    conf.getIfSet( "alignment", "center_bottom",           _alignment, ALIGN_CENTER_BOTTOM );
    conf.getIfSet( "alignment", "right_top",               _alignment, ALIGN_RIGHT_TOP );
    conf.getIfSet( "alignment", "right_center",            _alignment, ALIGN_RIGHT_CENTER );
    conf.getIfSet( "alignment", "right_bottom",            _alignment, ALIGN_RIGHT_BOTTOM );
    conf.getIfSet( "alignment", "left_base_line",          _alignment, ALIGN_LEFT_BASE_LINE );
    conf.getIfSet( "alignment", "center_base_line",        _alignment, ALIGN_CENTER_BASE_LINE );
    conf.getIfSet( "alignment", "right_base_line",         _alignment, ALIGN_RIGHT_BASE_LINE );
    conf.getIfSet( "alignment", "left_bottom_base_line",   _alignment, ALIGN_LEFT_BOTTOM_BASE_LINE );
    conf.getIfSet( "alignment", "center_bottom_base_line", _alignment, ALIGN_CENTER_BOTTOM_BASE_LINE );
    conf.getIfSet( "alignment", "right_bottom_base_line",  _alignment, ALIGN_RIGHT_BOTTOM_BASE_LINE );
    conf.getIfSet( "alignment", "base_line" ,              _alignment, ALIGN_BASE_LINE );

    conf.getIfSet( "layout", "ltr",  _layout, LAYOUT_LEFT_TO_RIGHT );
    conf.getIfSet( "layout", "rtl",  _layout, LAYOUT_RIGHT_TO_LEFT );
    conf.getIfSet( "layout", "vertical",  _layout, LAYOUT_VERTICAL );

    conf.getIfSet( "declutter", _declutter );

    conf.getIfSet( "provider", _provider );
    if ( conf.hasValue( "pixel_offset_x" ) )
        _pixelOffset = osg::Vec2s( conf.value<short>("pixel_offset_x",0), 0 );
    if ( conf.hasValue( "pixel_offset_y" ) )
        _pixelOffset = osg::Vec2s( _pixelOffset->x(), conf.value<short>("pixel_offset_y",0) );

    conf.getIfSet( "text-occlusion-cull", _occlusionCull );
    conf.getIfSet( "text-occlusion-cull-altitude", _occlusionCullAltitude );
}


void
TextSymbol::parseSLD(const Config& c, Style& style)
{
    if ( match(c.key(), "text-fill") ) {
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
    else if ( match(c.key(), "text-halo") ) {
        style.getOrCreate<TextSymbol>()->halo()->color() = htmlColorToVec4f( c.value() );
    }
    else if ( match(c.key(), "text-halo-offset") ) {
        style.getOrCreate<TextSymbol>()->haloOffset() = as<float>(c.value(), 0.0625f);
    }
    else if ( match(c.key(), "text-remove-duplicate-labels") ) {
        if ( c.value() == "true" )
            style.getOrCreate<TextSymbol>()->removeDuplicateLabels() = true;
        else if (c.value() == "false")
            style.getOrCreate<TextSymbol>()->removeDuplicateLabels() = false;
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
    else if ( match(c.key(), "text-content") ) {        
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
        style.getOrCreate<TextSymbol>()->declutter() = as<bool>(c.value(), true);
    }
    else if ( match(c.key(), "text-occlusion-cull") ) {
        style.getOrCreate<TextSymbol>()->occlusionCull() = as<bool>(c.value(), false);
    }
    else if ( match(c.key(), "text-occlusion-cull-altitude") ) {
        style.getOrCreate<TextSymbol>()->occlusionCullAltitude() = as<double>(c.value(), 200000.0);
    }
    else if ( match(c.key(), "text-script") ) {
        style.getOrCreate<TextSymbol>()->script() = StringExpression(c.value());
    }
    else if ( match(c.key(), "text-offset-x") ) {
        style.getOrCreate<TextSymbol>()->pixelOffset()->x() = as<double>(c.value(), 0.0);
    }
    else if ( match(c.key(), "text-offset-y") ) {
        style.getOrCreate<TextSymbol>()->pixelOffset()->y() = as<double>(c.value(), 0.0);
    }
}
