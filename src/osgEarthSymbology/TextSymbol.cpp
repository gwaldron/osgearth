/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2012 Pelican Mapping
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
#include <osgEarthSymbology/TextSymbol>

using namespace osgEarth;
using namespace osgEarth::Symbology;

TextSymbol::TextSymbol( const Config& conf ) :
Symbol                ( conf ),
_fill                 ( Fill( 1, 1, 1, 1 ) ),
_halo                 ( Stroke( 0.3, 0.3, 0.3, 1) ),
_haloOffset           ( 0.07f ),
_size                 ( 16.0f ),
_removeDuplicateLabels( false ),
_alignment            ( ALIGN_BASE_LINE ),
_provider             ( "annotation" ),
_encoding             ( ENCODING_ASCII ),
_declutter            ( true )
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
    conf.addIfSet( "size", _size );
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

    conf.addIfSet( "declutter", _declutter );

    conf.addIfSet( "provider", _provider );
    if ( _pixelOffset.isSet() ) {
        conf.add( "pixel_offset_x", toString(_pixelOffset->x()) );
        conf.add( "pixel_offset_y", toString(_pixelOffset->y()) );
    }
    return conf;
}

void 
TextSymbol::mergeConfig( const Config& conf )
{
    conf.getObjIfSet( "fill", _fill );
    conf.getObjIfSet( "halo", _halo );
    conf.getIfSet( "halo_offset", _haloOffset );
    conf.getIfSet( "font", _font );
    conf.getIfSet( "size", _size );
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

    conf.getIfSet( "declutter", _declutter );

    conf.getIfSet( "provider", _provider );
    if ( conf.hasValue( "pixel_offset_x" ) )
        _pixelOffset = osg::Vec2s( conf.value<short>("pixel_offset_x",0), 0 );
    if ( conf.hasValue( "pixel_offset_y" ) )
        _pixelOffset = osg::Vec2s( _pixelOffset->x(), conf.value<short>("pixel_offset_y",0) );
}
