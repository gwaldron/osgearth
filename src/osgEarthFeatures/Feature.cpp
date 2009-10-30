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

using namespace osgEarthFeatures;

static
std::string EMPTY_STRING;


FeatureProfile::FeatureProfile(const SpatialReference* srs,
                               FeatureProfile::GeometryType type,
                               int dim,
                               bool multiGeometry ) :
_srs( srs ),
_geomType( type ),
_dimensionality( dim ),
_multiGeometry( multiGeometry )
{
    //nop
}

const SpatialReference*
FeatureProfile::getSRS() const 
{
    return _srs.get();
}

FeatureProfile::GeometryType
FeatureProfile::getGeometryType() const
{
    return _geomType;
}

int
FeatureProfile::getDimensionality() const
{
    return _dimensionality;
}

bool
FeatureProfile::isMultiGeometry() const
{
    return _multiGeometry;
}

/****************************************************************************/

Feature::Feature( long fid ) :
_fid( fid )
{
    //NOP
}

Feature::Feature( const Feature& rhs, const osg::CopyOp& copyOp ) :
_fid( rhs._fid )
{
    // copy the parts using the copy-operator:
    _parts.reserve( rhs._parts.size() );
    for( FeatureGeomParts::const_iterator p = rhs._parts.begin(); p != rhs._parts.end(); p++ )
    {
        _parts.push_back( static_cast<osg::Vec3dArray*>( copyOp( p->get() ) ) );
    }

    // always copied by value
    _attrs = rhs._attrs;
}

long
Feature::getFID() const 
{
    return _fid;
}

void
Feature::setPart( int part, osg::Vec3dArray* points )
{
    if ( part >= 0 && points )
    {
        if ( part+1 > (int)_parts.size() )
            _parts.resize( part+1 );

        _parts[part] = points;
    }
}

void
Feature::addPart( osg::Vec3dArray* points )
{
    if ( points )
        setPart( _parts.size(), points );
}

int
Feature::getNumParts() const
{
    return _parts.size();
}

osg::Vec3dArray*
Feature::getPart( int part ) const
{ 
    return part >= 0 && part < (int)_parts.size()? _parts[part] : 0L;
}


void
Feature::setAttr( const std::string& name, const std::string& value )
{
    _attrs[name] = value;
}

const std::string&
Feature::getAttr( const std::string& name ) const
{
    FeatureAttributes::const_iterator i = _attrs.find(name);
    return i != _attrs.end()? i->second : EMPTY_STRING;
}

