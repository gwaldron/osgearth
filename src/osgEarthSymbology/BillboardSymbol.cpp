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
#include <osgEarthSymbology/BillboardSymbol>
#include <osgEarthSymbology/BillboardResource>
#include <osgEarthSymbology/Style>
#include <osgEarth/URI>

using namespace osgEarth;
using namespace osgEarth::Symbology;

OSGEARTH_REGISTER_SIMPLE_SYMBOL(billboard, BillboardSymbol);


BillboardSymbol::BillboardSymbol(const BillboardSymbol& rhs,const osg::CopyOp& copyop):
InstanceSymbol(rhs, copyop)//,
//_image        (rhs._image.get())
{
    //nop
}

BillboardSymbol::BillboardSymbol( const Config& conf ) :
InstanceSymbol( conf )
{
    mergeConfig( conf );
}

Config 
BillboardSymbol::getConfig() const
{
    Config conf = InstanceSymbol::getConfig();
    conf.key() = "billboard";
    conf.set( "width", _width );
    conf.set( "height", _height );
    conf.set( "top_url", _topURL );
    //conf.setNonSerializable( "BillboardSymbol::image", _image.get() );
    return conf;
}

void 
BillboardSymbol::mergeConfig( const Config& conf )
{
    conf.get( "width", _width );
    conf.get( "height", _height );
    conf.get( "top_url", _topURL );
    //_image = conf.getNonSerializable<osg::Image>( "BillboardSymbol::image" );
}

InstanceResource*
BillboardSymbol::createResource() const
{
    return new BillboardResource();
}

void
BillboardSymbol::parseSLD(const Config& c, Style& style)
{
    if ( match(c.key(), "billboard-image") ) {
        style.getOrCreate<BillboardSymbol>()->url() = c.value();
        style.getOrCreate<BillboardSymbol>()->url()->setURIContext( c.referrer() );
    }
    else if (match(c.key(), "billboard-top-image")) {
        style.getOrCreate<BillboardSymbol>()->topURL() = c.value();
        style.getOrCreate<BillboardSymbol>()->topURL()->setURIContext(c.referrer());
    }
    else if ( match(c.key(), "billboard-width") ) {
        style.getOrCreate<BillboardSymbol>()->width() = as<float>(c.value(), 10.0f);
    }
    else if ( match(c.key(), "billboard-height") ) {
        style.getOrCreate<BillboardSymbol>()->height() = as<float>(c.value(), 10.0f);
    }
}

