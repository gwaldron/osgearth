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
#include <osgEarthFeatures/ConvertTypeFilter>
#include <list>
#include <deque>

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

OSGEARTH_REGISTER_SIMPLE_FEATUREFILTER(convert, ConvertTypeFilter)


ConvertTypeFilter::ConvertTypeFilter() :
_toType( Geometry::TYPE_UNKNOWN )
{
    //NOP
}

ConvertTypeFilter::ConvertTypeFilter( const Geometry::Type& toType ) :
_toType( toType )
{
    // NOP
}

ConvertTypeFilter::ConvertTypeFilter( const ConvertTypeFilter& rhs ) :
_toType( rhs._toType )
{
    //NOP
}

ConvertTypeFilter::ConvertTypeFilter( const Config& conf):
_toType( Geometry::TYPE_UNKNOWN )
{
    if (conf.key() == "convert")
    {
        optional<Geometry::Type> type = Geometry::TYPE_POINTSET;
        conf.getIfSet( "type", "point",   type, Geometry::TYPE_POINTSET );
        conf.getIfSet( "type", "line",    type, Geometry::TYPE_LINESTRING );
        conf.getIfSet( "type", "polygon", type, Geometry::TYPE_POLYGON );
        _toType = *type;        
    }
}

Config ConvertTypeFilter::getConfig() const
{
    Config config( "convert" );
    optional<Geometry::Type> type( _toType, _toType); // weird optional ctor :)
    config.addIfSet( "type", "point",   type, Geometry::TYPE_POINTSET );
    config.addIfSet( "type", "line",    type, Geometry::TYPE_LINESTRING );
    config.addIfSet( "type", "polygon", type, Geometry::TYPE_POLYGON );    

    return config;
}

FilterContext
ConvertTypeFilter::push( FeatureList& input, FilterContext& context )
{
    if ( !isSupported() )
    {
        OE_WARN << "ConvertTypeFilter support not enabled" << std::endl;
        return context;
    }

    bool ok = true;
    for( FeatureList::iterator i = input.begin(); i != input.end(); ++i )
    {
        Feature* input = i->get();
        if ( input && input->getGeometry() && input->getGeometry()->getComponentType() != _toType )
        {
            input->setGeometry( input->getGeometry()->cloneAs(_toType) );
        }
    }

    return context;
}
