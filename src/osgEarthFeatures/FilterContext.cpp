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
#include <osgEarthFeatures/FilterContext>
#include <osgEarthFeatures/Session>
#include <osgEarthSymbology/ResourceCache>
#include <osgEarth/Registry>

using namespace osgEarth;
using namespace osgEarth::Features;

FilterContext::FilterContext() :
_session(0L),
_profile(0L),
_isGeocentric(false),
_index(0L),
_shaderPolicy(osgEarth::SHADERPOLICY_GENERATE)
{
    //nop
}

FilterContext::FilterContext(Session*               session,
                             const FeatureProfile*  profile,
                             const GeoExtent&       workingExtent,
                             FeatureIndexBuilder*   index ) :
_session     ( session ),
_profile     ( profile ),
_extent      ( workingExtent, workingExtent ),
_isGeocentric( false ),
_index       ( index ),
_shaderPolicy( osgEarth::SHADERPOLICY_GENERATE )
{
    if ( session )
    {
        if ( session->getResourceCache() )
        {
            _resourceCache = session->getResourceCache();
        }
        else
        {
            _resourceCache = new ResourceCache();
        }
    }

    // attempt to establish a working extent if we don't have one:

    if (!_extent->isValid() &&
        profile &&
        profile->getExtent().isValid() )
    {
        _extent = profile->getExtent();
    }

    if (!_extent->isValid() &&
        session && 
        session->getMapInfo().getProfile() )
    {
        _extent = session->getMapInfo().getProfile()->getExtent();
    }

    // if the session is set, push its name as the first bc.
    if ( _session.valid() )
    {
        pushHistory( _session->getName() );
    }
}

FilterContext::FilterContext( const FilterContext& rhs ) :
_profile              ( rhs._profile.get() ),
_session              ( rhs._session.get() ),
_isGeocentric         ( rhs._isGeocentric ),
_extent               ( rhs._extent ),
_referenceFrame       ( rhs._referenceFrame ),
_inverseReferenceFrame( rhs._inverseReferenceFrame ),
_resourceCache        ( rhs._resourceCache.get() ),
_index                ( rhs._index ),
_shaderPolicy         ( rhs._shaderPolicy ),
_history              ( rhs._history ),
_outputSRS            ( rhs._outputSRS.get() )
{
    //nop
}

FilterContext::~FilterContext()
{
    //nop
}

void
FilterContext::setProfile(const FeatureProfile* value)
{
    _profile = value;
}

Session*
FilterContext::getSession()
{
    return _session.get();
}

const Session*
FilterContext::getSession() const
{
    return _session.get();
}

bool
FilterContext::isGeoreferenced() const
{ 
    return _session.valid() && _profile.valid();
}

const SpatialReference*
FilterContext::getOutputSRS() const
{
    if (_outputSRS.valid())
        return _outputSRS.get();

    if (_session.valid() && _session->getMapSRS())
        return _session->getMapSRS();

    if (_profile.valid() && _profile->getSRS())
        return _profile->getSRS();

    if (_extent.isSet())
        return _extent->getSRS();

    return SpatialReference::get("wgs84");
}

const osgDB::Options*
FilterContext::getDBOptions() const
{
    return _session.valid() ? _session->getDBOptions() : 0L;
}

void
FilterContext::toLocal( Geometry* geom ) const
{
    if ( hasReferenceFrame() )
    {
        GeometryIterator gi( geom );
        while( gi.hasMore() )
        {
            Geometry* g = gi.next();
            for( osg::Vec3dArray::iterator i = g->begin(); i != g->end(); ++i )
                *i = *i * _referenceFrame;
        }
    }
}

void
FilterContext::toWorld( Geometry* geom ) const
{
    if ( hasReferenceFrame() )
    {
        GeometryIterator gi( geom );
        while( gi.hasMore() )
        {
            Geometry* g = gi.next();
            for( osg::Vec3dArray::iterator i = g->begin(); i != g->end(); ++i )
                *i = *i * _inverseReferenceFrame;
        }
    }
}

osg::Vec3d
FilterContext::toMap( const osg::Vec3d& point ) const
{
    osg::Vec3d world = toWorld(point);
    osg::Vec3d map;
    _extent->getSRS()->transformFromWorld( world, map );
    return map;
}

osg::Vec3d
FilterContext::fromMap( const osg::Vec3d& point ) const
{
    osg::Vec3d world;
    _extent->getSRS()->transformToWorld( point, world );
    return toLocal(world);
}

ResourceCache*
FilterContext::resourceCache()
{
    return _resourceCache.get();
}

std::string
FilterContext::toString() const
{
    std::stringstream buf;

    buf << std::fixed
        << "CONTEXT: ["
        << "profile extent = "   << profile()->getExtent().toString()
        << ", working extent = " << extent()->toString()
        << ", geocentric = "     << osgEarth::toString(_isGeocentric)
        << ", history = "        << getHistory()
        << "]";

    std::string str;
    str = buf.str();
    return str;
}

std::string
FilterContext::getHistory() const
{
    std::stringstream buf;
    for(unsigned i=0; i<_history.size(); ++i)
    {
        if ( i > 0 ) buf << " : ";
        buf << _history[i];
    }
    return buf.str();
}
