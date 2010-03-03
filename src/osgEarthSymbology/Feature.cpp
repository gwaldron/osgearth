/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2009 Pelican Ventures, Inc.
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
#include <osgEarthSymbology/Feature>

using namespace osgEarth;
using namespace osgEarth::Symbology;
/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2009 Pelican Ventures, Inc.
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
#include <osgEarth/Common>
#include <algorithm>

using namespace osgEarth;
using namespace osgEarth::Symbology;

static
std::string EMPTY_STRING;

/****************************************************************************/

Feature::Feature( long fid ) :
_fid( fid )
{
    //NOP
}

Feature::Feature( const Feature& rhs, const osg::CopyOp& copyOp ) :
_fid( rhs._fid ),
_attrs( rhs._attrs )
{
    if ( rhs._geometry.valid() )
        _geometry = dynamic_cast<Geometry*>( copyOp( rhs._geometry.get() ) );
}


template<typename T>
T
Feature::getAttr( const std::string& name, const T& defaultValue ) const
{
    AttributeTable::const_iterator i = _attrs.find(name);
    return i != _attrs.end() ? osgEarth::as<T>( *i ) : defaultValue;
}

template<typename T>
bool
Feature::getAttr( const std::string& name, T& outputValue ) const
{
    AttributeTable::const_iterator i = _attrs.find(name);
    if ( i != _attrs.end() ) {
        outputValue = osgEarth::as<T>( *i );
        return true;
    }
    else {
        return false;
    }
}

template<typename T>
void
Feature::setAttr( const std::string& name, const T& value )
{
    _attrs[name] = osgEarth::toString( value );
}


