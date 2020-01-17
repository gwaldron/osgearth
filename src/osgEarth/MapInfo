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

#ifndef OSGEARTH_MAP_INFO_H
#define OSGEARTH_MAP_INFO_H 1

#include <osgEarth/Common>
#include <osgEarth/GeoCommon>
#include <osgEarth/Profile>

namespace osgEarth
{
    class Map;

    /**
     * A convenience class that combines a general geospatial profile and additional 
     * information about the map itself.
     */
    class OSGEARTH_EXPORT MapInfo
    {
    public:
        //! Constructs a new MapInfo object from the data in a Map
        MapInfo(const Map* map);

        //! Copy constructor
        MapInfo(const MapInfo& rhs);

        //! Dtor
        virtual ~MapInfo() { }

        //! Sets the map info to reflect a Map object
        void setMap(const Map* map);
        
        //! Map's profile
        const Profile* getProfile() const { return _profile.get(); }

        //! Map's profile SRS
        const SpatialReference* getSRS() const { return _profile->getSRS(); }

        //! True if the map's visualization is geocentric
        bool isGeocentric() const { return _isGeocentric; }

        //! True if the map's SRS is projected
        bool isProjectedSRS() const { return !isGeographicSRS(); }

        //! True if the map's SRS Is geographic (lat/long)
        bool isGeographicSRS() const { return _profile->getSRS()->isGeographic(); }

        //! Elevation interpolation method used by the map
        ElevationInterpolation getElevationInterpolation() const { return _elevationInterpolation;}

    private:
        osg::ref_ptr<const Profile> _profile;
        bool _isGeocentric, _isCube;
        ElevationInterpolation _elevationInterpolation;
    };
}

#endif // OSGEARTH_MAP_CALLBACK_H
