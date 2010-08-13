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
#include <osgEarthFeatures/Feature>
#include <algorithm>

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

static
std::string EMPTY_STRING;


FeatureProfile::FeatureProfile( const GeoExtent& extent ) :
_extent( extent )
{
    //nop
}

/****************************************************************************/

Feature::Feature( long fid ) :
_fid( fid ),
_style( new Style() )
{
    //NOP
}

Feature::Feature( const Feature& rhs, const osg::CopyOp& copyOp ) :
_fid( rhs._fid ),
_attrs( rhs._attrs ),
_style( rhs._style )
{
    if ( rhs._geom.valid() )
        _geom = dynamic_cast<Geometry*>( copyOp( rhs._geom.get() ) );
}

long
Feature::getFID() const 
{
    return _fid;
}

void
Feature::setAttr( const std::string& name, const std::string& value )
{
    _attrs[name] = value;
}

const std::string&
Feature::getAttr( const std::string& name ) const
{
    AttributeTable::const_iterator i = _attrs.find(name);
    return i != _attrs.end()? i->second : EMPTY_STRING;
}

