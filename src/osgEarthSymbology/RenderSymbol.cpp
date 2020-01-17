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
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */
#include <osgEarthSymbology/RenderSymbol>
#include <osgEarthSymbology/Style>

using namespace osgEarth;
using namespace osgEarth::Symbology;

OSGEARTH_REGISTER_SIMPLE_SYMBOL(render, RenderSymbol);

RenderSymbol::RenderSymbol(const RenderSymbol& rhs,const osg::CopyOp& copyop):
Symbol(rhs, copyop),
_depthTest(rhs._depthTest),
_lighting(rhs._lighting),
_depthOffset(rhs._depthOffset),
_backfaceCulling(rhs._backfaceCulling),
_order(rhs._order),
_clipPlane(rhs._clipPlane),
_minAlpha(rhs._minAlpha),
_renderBin(rhs._renderBin),
_transparent(rhs._transparent),
_decal(rhs._decal),
_maxCreaseAngle(rhs._maxCreaseAngle),
_maxAltitude(rhs._maxAltitude)
{
}

RenderSymbol::RenderSymbol(const Config& conf) :
Symbol          ( conf ),
_depthTest      ( true ),
_lighting       ( true ),
_backfaceCulling( true ),
_order          ( 0 ),
_clipPlane      ( 0 ),
_minAlpha       ( 0.0f ),
_transparent    ( false ),
_decal          ( false ),
_maxCreaseAngle ( 0.0 ),
_maxAltitude    ( Distance(FLT_MAX, Units::METERS) )
{
    mergeConfig(conf);
}

Config 
RenderSymbol::getConfig() const
{
    Config conf = Symbol::getConfig();
    conf.key() = "render";
    conf.set( "depth_test",       _depthTest );
    conf.set( "lighting",         _lighting );
    conf.set( "depth_offset",     _depthOffset );
    conf.set( "backface_culling", _backfaceCulling );
    conf.set( "order",            _order );
    conf.set( "clip_plane",       _clipPlane );
    conf.set( "min_alpha",        _minAlpha );
    conf.set( "render_bin",       _renderBin );
    conf.set( "transparent",      _transparent );
    conf.set( "decal",            _decal);
    conf.set( "max_crease_angle", _maxCreaseAngle);
    conf.set( "max_altitude",     _maxAltitude);
    return conf;
}

void 
RenderSymbol::mergeConfig( const Config& conf )
{
    conf.get( "depth_test",       _depthTest );
    conf.get( "lighting",         _lighting );
    conf.get( "depth_offset",     _depthOffset );
    conf.get( "backface_culling", _backfaceCulling );
    conf.get( "order",            _order );
    conf.get( "clip_plane",       _clipPlane );
    conf.get( "min_alpha",        _minAlpha );
    conf.get( "render_bin",       _renderBin );
    conf.get( "transparent",      _transparent );
    conf.get( "decal",            _decal);
    conf.get( "max_crease_angle", _maxCreaseAngle);
    conf.get( "max_altitude",     _maxAltitude);
}

void
RenderSymbol::parseSLD(const Config& c, Style& style)
{
    RenderSymbol defaults;

    if ( match(c.key(), "render-depth-test") ) {
        style.getOrCreate<RenderSymbol>()->depthTest() = as<bool>(c.value(), *defaults.depthTest());
    }
    else if ( match(c.key(), "render-lighting") ) {
        style.getOrCreate<RenderSymbol>()->lighting() = as<bool>(c.value(), *defaults.lighting());
    }
    else if ( match(c.key(), "render-depth-offset") ) {
        style.getOrCreate<RenderSymbol>()->depthOffset()->enabled() = as<bool>(c.value(), *defaults.depthOffset()->enabled() );
    }
    else if ( match(c.key(), "render-depth-offset-min-bias") ) {
        float value; Units units;
        if (Units::parse(c.value(), value, units, Units::METERS))
            style.getOrCreate<RenderSymbol>()->depthOffset()->minBias() = Distance(value, units);
        style.getOrCreate<RenderSymbol>()->depthOffset()->automatic() = false;
    }
    else if ( match(c.key(), "render-depth-offset-max-bias") ) {
        float value; Units units;
        if (Units::parse(c.value(), value, units, Units::METERS))
            style.getOrCreate<RenderSymbol>()->depthOffset()->maxBias() = Distance(value, units);
    }
    else if ( match(c.key(), "render-depth-offset-min-range") ) {
        float value; Units units;
        if (Units::parse(c.value(), value, units, Units::METERS))
            style.getOrCreate<RenderSymbol>()->depthOffset()->minRange() = Distance(value, units);
    }
    else if ( match(c.key(), "render-depth-offset-max-range") ) {
        float value; Units units;
        if (Units::parse(c.value(), value, units, Units::METERS))
            style.getOrCreate<RenderSymbol>()->depthOffset()->maxRange() = Distance(value, units);
    }
    else if ( match(c.key(), "render-depth-offset-auto") ) {
        style.getOrCreate<RenderSymbol>()->depthOffset()->automatic() = as<bool>(c.value(), *defaults.depthOffset()->automatic() );
    }
    else if ( match(c.key(), "render-backface-culling") ) {
        style.getOrCreate<RenderSymbol>()->backfaceCulling() = as<bool>(c.value(), *defaults.backfaceCulling() );
    }
    else if ( match(c.key(), "render-order") ) {
        style.getOrCreate<RenderSymbol>()->order() = !c.value().empty() ? NumericExpression(c.value()) : *defaults.order();
    }
    else if ( match(c.key(), "render-clip-plane") ) {
        style.getOrCreate<RenderSymbol>()->clipPlane() = as<unsigned>(c.value(), *defaults.clipPlane() );
    }
    else if ( match(c.key(), "render-min-alpha") ) {
        style.getOrCreate<RenderSymbol>()->minAlpha() = as<float>(c.value(), *defaults.minAlpha() );
    }
    else if ( match(c.key(), "render-bin") ) {
        style.getOrCreate<RenderSymbol>()->renderBin() = c.value();
    }
    else if ( match(c.key(), "render-transparent") ) {
        style.getOrCreate<RenderSymbol>()->transparent() = as<bool>(c.value(), *defaults.transparent() );
    }
    else if (match(c.key(), "render-decal")) {
        style.getOrCreate<RenderSymbol>()->decal() = as<bool>(c.value(), *defaults.decal());
    }
    else if (match(c.key(), "render-max-crease-angle")) {
        float value; Units units;
        if (Units::parse(c.value(), value, units, Units::METERS))
            style.getOrCreate<RenderSymbol>()->maxCreaseAngle() = Angle(value, units);
    }
    else if (match(c.key(), "render-max-altitude")) {
        float value; Units units;
        if (Units::parse(c.value(), value, units, Units::METERS))
            style.getOrCreate<RenderSymbol>()->maxAltitude() = Distance(value, units);
    }
}
