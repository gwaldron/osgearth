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

#include <osgEarth/SpatialReference>
#include <osgEarth/Registry>
#include <ogr_api.h>
#include <ogr_spatialref.h>
#include <algorithm>
#include <OpenThreads/ScopedLock>
#include <osg/Notify>

using namespace osgEarth;


#define OGR_SCOPE_LOCK() \
    OpenThreads::ScopedLock<OpenThreads::ReentrantMutex> _slock( osgEarth::Registry::instance()->getOGRMutex() )


SpatialReference*
SpatialReference::createFromPROJ4( const std::string& init )
{
    SpatialReference* result = NULL;
    OGR_SCOPE_LOCK();
	void* handle = OSRNewSpatialReference( NULL );
    if ( OSRImportFromProj4( handle, init.c_str() ) == OGRERR_NONE )
	{
        result = new SpatialReference( handle, true );
	}
	else 
	{
		osg::notify(osg::WARN) << "Unable to create spatial reference from PROJ4: " << init << std::endl;
		OSRDestroySpatialReference( handle );
	}
    return result;
}

SpatialReference*
SpatialReference::createFromWKT( const std::string& init )
{
    SpatialReference* result = NULL;
    OGR_SCOPE_LOCK();
	void* handle = OSRNewSpatialReference( NULL );
    char buf[4096];
    char* buf_ptr = &buf[0];
	strcpy( buf, init.c_str() );
	if ( OSRImportFromWkt( handle, &buf_ptr ) == OGRERR_NONE )
	{
        result = new SpatialReference( handle, true );
	}
	else 
	{
		osg::notify(osg::WARN) << "Unable to create spatial reference from WKT: " << init << std::endl;
		OSRDestroySpatialReference( handle );
	}
    return result;
}

SpatialReference*
SpatialReference::create( const std::string& init )
{
    std::string temp = init;
    std::transform( temp.begin(), temp.end(), temp.begin(), ::tolower );

    if ( temp.find( "init=" ) == 0 )
        return createFromPROJ4( init );
    if ( temp.find( "epsg:" ) == 0 )
        return createFromPROJ4( "init=" + init );
    if ( temp.find( "projcs" ) == 0 || temp.find( "geogcs" ) == 0 )
        return createFromWKT( init );
    else
        return NULL;
}

/****************************************************************************/


SpatialReference::SpatialReference( void* handle, bool take_ownership ) :
_handle( handle ),
_owns_handle( take_ownership )
{
    //TODO
}

SpatialReference::~SpatialReference()
{
	if ( _handle && _owns_handle )
	{
      OGR_SCOPE_LOCK();
      OSRDestroySpatialReference( _handle );
	}
	_handle = NULL;
}

bool
SpatialReference::isGeographic() const 
{
    if ( !_initialized )
        const_cast<SpatialReference*>(this)->init();
    return _is_geographic;
}

bool
SpatialReference::isProjected() const
{
    if ( !_initialized )
        const_cast<SpatialReference*>(this)->init();
    return !_is_geographic;
}

const std::string&
SpatialReference::getName() const
{
    if ( !_initialized )
        const_cast<SpatialReference*>(this)->init();
    return _name;
}

const osg::EllipsoidModel*
SpatialReference::getEllipsoid() const
{
    if ( !_initialized )
        const_cast<SpatialReference*>(this)->init();
    return _ellipsoid.get();
}

bool
SpatialReference::transformTo( double x, double y, const SpatialReference* out_srs, double& out_x, double& out_y ) const
{        
    OGR_SCOPE_LOCK();

    void* xform_handle = OCTNewCoordinateTransformation( _handle, out_srs->_handle );
    if ( !xform_handle )
    {
        osg::notify( osg::WARN )
            << "[osgEarth::SpatialReference] SRS xform not possible" << std::endl
            << "    From => " << getName() << std::endl
            << "    To   => " << out_srs->getName() << std::endl;
        return false;
    }

    double temp_x = x;
    double temp_y = y;
    double temp_z = 0.0;
    bool result;

    if ( OCTTransform( xform_handle, 1, &temp_x, &temp_y, &temp_z ) )
    {
        result = true;
        out_x = temp_x;
        out_y = temp_y;
    }
    else
    {
        osg::notify( osg::WARN ) << "[osgEarth::SpatialReference] Failed to xform a point from "
            << getName() << " to " << out_srs->getName()
            << std::endl;
        result = false;
    }

    OCTDestroyCoordinateTransformation( xform_handle );
    return result;
}

static std::string
getOGRAttrValue( void* _handle, const std::string& name, int child_num )
{
    OGR_SCOPE_LOCK();
	const char* val = OSRGetAttrValue( _handle, name.c_str(), child_num );
	return val? std::string( val ) : "";
}

void
SpatialReference::init()
{
    OGR_SCOPE_LOCK();

    int err;
    double semi_major_axis = OSRGetSemiMajor( _handle, &err );
    double semi_minor_axis = OSRGetSemiMinor( _handle, &err );
    _ellipsoid = new osg::EllipsoidModel( semi_major_axis, semi_minor_axis );
    
    _is_geographic = OSRIsGeographic( _handle ) != 0;

    _name = _is_geographic? 
        getOGRAttrValue( _handle, "GEOGCS", 0 ) : 
        getOGRAttrValue( _handle, "PROJCS", 0 );

    _initialized = true;
}
