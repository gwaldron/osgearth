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
#include <algorithm>

using namespace osgEarth;
using namespace osgEarth::Features;

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

int
FeatureGeometry::getTotalPointCount() const
{
    int count = 0;
    for( const_iterator i = begin(); i != end(); ++i )
    {
        count += i->get()->size();
    }
    return count;
}

bool
FeatureGeometry::isClosed( const osg::Vec3dArray* part ) 
{
    return part && part->size() > 1 && part->front() == part->back();
    //return ( part && part->size() > 1 && *(part->begin()) == *(part->end()-1) );
}

double
FeatureGeometry::getSignedAreaOfOpenPolygon( const osg::Vec3dArray* part )
{
    double sum = 0.0;
    for( osg::Vec3dArray::const_iterator i = part->begin(); i != part->end(); ++i )
    {
        const osg::Vec3d& p0 = *i;
        const osg::Vec3d& p1 = i != part->end()-1? *(i+1) : *(part->begin());
        sum += p0.x()*p1.y() - p1.x()*p0.y();
    }
    return 0.5 * sum;
}

bool
FeatureGeometry::isCCW( const osg::Vec3dArray* part )
{
    return getSignedAreaOfOpenPolygon( part ) > 0.0;
}

void
FeatureGeometry::openPart( osg::Vec3dArray* part )
{
    if ( part->size() > 2 && part->front() == part->back() )
    {
        part->erase( part->end()-1 );
    }
}

void
FeatureGeometry::rewindOpenPolygon( osg::Vec3dArray* poly, bool makeCCW )
{
    if ( isCCW( poly ) != makeCCW )
        std::reverse( poly->begin(), poly->end() );
}

void 
FeatureGeometry::normalizePolygon()
{
    for( FeatureGeometry::iterator i = begin(); i != end(); ++i )
    {
        osg::Vec3dArray* part = i->get();
        openPart( part );
        rewindOpenPolygon( part, i == begin() );
    }
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
    for( FeatureGeometry::const_iterator p = rhs._parts.begin(); p != rhs._parts.end(); p++ )
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
Feature::setGeometry( const FeatureGeometry& newGeom )
{
    _parts = newGeom;
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

const FeatureGeometry&
Feature::getGeometry() const 
{
    return _parts;
}

FeatureGeometry&
Feature::getGeometry()
{
    return _parts;
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

