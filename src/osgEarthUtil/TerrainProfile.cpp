/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2014 Pelican Mapping
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
#include <osgEarthUtil/TerrainProfile>
#include <osgEarth/MapNode>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/GeoMath>

using namespace osgEarth;
using namespace osgEarth::Util;

/***************************************************/
TerrainProfile::TerrainProfile():
_spacing( 1.0 )
{
}

TerrainProfile::TerrainProfile(const TerrainProfile& rhs):
_spacing( rhs._spacing ),
_elevations( rhs._elevations )
{
}

void
TerrainProfile::clear()
{
    _elevations.clear();
}

void
TerrainProfile::addElevation( double distance, double elevation )
{
    _elevations.push_back( DistanceHeight(distance, elevation));
}

double
TerrainProfile::getElevation( int i ) const
{
    if (i >= 0 && i < static_cast<int>(_elevations.size())) return _elevations[i].second;
    return DBL_MAX;    
}

double
TerrainProfile::getDistance( int i ) const
{
    if (i >= 0 && i < static_cast<int>(_elevations.size())) return _elevations[i].first;
    return DBL_MAX;    
}

double
TerrainProfile::getTotalDistance() const
{
    return _elevations.empty() ? 0.0 : _elevations.back().first;
}

unsigned int
TerrainProfile::getNumElevations() const
{
    return _elevations.size();
}

void
TerrainProfile::getElevationRanges(double &min, double &max ) const
{
    min = DBL_MAX;
    max = -DBL_MAX;

    for (unsigned int i = 0; i < _elevations.size(); i++)
    {
        if (_elevations[i].second < min) min = _elevations[i].second;
        if (_elevations[i].second > max) max = _elevations[i].second;
    }
}

/***************************************************/
TerrainProfileCalculator::TerrainProfileCalculator(MapNode* mapNode, const GeoPoint& start, const GeoPoint& end):
_mapNode( mapNode ),
_start( start),
_end( end )
{        
    _mapNode->getTerrain()->addTerrainCallback( this );        
    recompute();
}

TerrainProfileCalculator::TerrainProfileCalculator(MapNode* mapNode):
_mapNode( mapNode )
{
    _mapNode->getTerrain()->addTerrainCallback( this );
}

TerrainProfileCalculator::~TerrainProfileCalculator()
{
    _mapNode->getTerrain()->removeTerrainCallback( this );
}

void TerrainProfileCalculator::setMapNode( osgEarth::MapNode* mapNode )
{
  if (_mapNode.valid())
      _mapNode->getTerrain()->removeTerrainCallback( this );

  _mapNode = mapNode;
  if (_mapNode.valid())
  {
      _mapNode->getTerrain()->addTerrainCallback( this );
      recompute();
  }
}

void TerrainProfileCalculator::addChangedCallback( ChangedCallback* callback )
{
    _changedCallbacks.push_back( callback );
}

void TerrainProfileCalculator::removeChangedCallback( ChangedCallback* callback )
{
    ChangedCallbackList::iterator i = std::find( _changedCallbacks.begin(), _changedCallbacks.end(), callback);
    if (i != _changedCallbacks.end())
    {
        _changedCallbacks.erase( i );
    }    
}

const TerrainProfile& TerrainProfileCalculator::getProfile() const
{
    return _profile;
}

const GeoPoint& TerrainProfileCalculator::getStart() const
{
    return _start;
}

GeoPoint TerrainProfileCalculator::getStart(AltitudeMode altMode) const
{
    if( _start.altitudeMode() != altMode )
    {
        double newAlt = 0.0;
        _start.transformZ(altMode, _mapNode->getTerrain(), newAlt);
        return GeoPoint(_start.getSRS(), _start.x(), _start.y(), newAlt);
    }
    return _start;
}

const GeoPoint& TerrainProfileCalculator::getEnd() const
{
    return _end;
}

GeoPoint TerrainProfileCalculator::getEnd(AltitudeMode altMode) const
{
    if( _end.altitudeMode() != altMode )
    {
        double newAlt = 0.0;
        _end.transformZ(altMode, _mapNode->getTerrain(), newAlt);
        return GeoPoint(_end.getSRS(), _end.x(), _end.y(), newAlt);
    }
    return _end;
}

void TerrainProfileCalculator::setStartEnd(const GeoPoint& start, const GeoPoint& end)
{
    if (_start != start || _end != end)
    {
        _start = start;
        _end = end;
        recompute();
    }
}

void TerrainProfileCalculator::onTileAdded(const osgEarth::TileKey& tileKey, osg::Node* terrain, TerrainCallbackContext&)
{
    if (_start.isValid() && _end.isValid())
    {
        GeoExtent extent( _start.getSRS());
        extent.expandToInclude(_start.x(), _start.y());
        extent.expandToInclude(_end.x(), _end.y());

        if (tileKey.getExtent().intersects( extent ))
        {
            recompute();
        }
    }
}

void TerrainProfileCalculator::recompute()
{
    if (_start.isValid() && _end.isValid())
    {
        computeTerrainProfile( _mapNode.get(), _start, _end, _profile);

        for( ChangedCallbackList::iterator i = _changedCallbacks.begin(); i != _changedCallbacks.end(); i++ )
        {
            if ( i->get() )
                i->get()->onChanged(this);
        }
    }
    else
    {
        _profile.clear();
    }
}

void TerrainProfileCalculator::computeTerrainProfile( osgEarth::MapNode* mapNode, const GeoPoint& start, const GeoPoint& end, TerrainProfile& profile)
{
    osg::Vec3d startvec, endvec;
    start.toWorld( startvec, mapNode->getTerrain() );
    end.toWorld( endvec, mapNode->getTerrain() );
    osgSim::ElevationSlice slice;
    slice.setStartPoint( startvec );
    slice.setEndPoint( endvec );
    slice.setDatabaseCacheReadCallback( 0 );
    slice.computeIntersections( mapNode->getTerrainEngine());

    profile.clear();
    for (unsigned int i = 0; i < slice.getDistanceHeightIntersections().size(); i++)
    {
        profile.addElevation( slice.getDistanceHeightIntersections()[i].first, slice.getDistanceHeightIntersections()[i].second);
    }
}
