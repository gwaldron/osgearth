/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
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
#include "SelectionInfo"
#include <osgEarth/TileKey>

using namespace osgEarth::Drivers::RexTerrainEngine;
using namespace osgEarth;

#define LC "[SelectionInfo] "

const double SelectionInfo::_morphStartRatio = 0.66;

const SelectionInfo::LOD&
SelectionInfo::getLOD(unsigned lod) const
{
    static SelectionInfo::LOD s_dummy;

    if (lod-_firstLOD >= _lods.size())
    {
        // note, this can happen if firstLOD() is set
        OE_DEBUG << LC <<"Index out of bounds"<<std::endl;
        return s_dummy;
    }
    return _lods[lod-_firstLOD];
}

void
SelectionInfo::initialize(unsigned firstLod, unsigned maxLod, const Profile* profile, double mtrf)
{
    if (getNumLODs() > 0)
    {
        OE_INFO << LC <<"Error: Selection Information already initialized"<<std::endl;
        return;
    }

    if (firstLod > maxLod)
    {
        OE_INFO << LC <<"Error: Inconsistent First and Max LODs"<<std::endl;
        return;
    }

    _firstLOD = firstLod;

    double fLodNear = 0;
    float fRatio = 1.0;

    unsigned numLods = maxLod + 1u;

    _lods.resize(numLods);

    for (unsigned lod = 0; lod <= maxLod; ++lod)
    {
        TileKey key(lod, 0, 0, profile);
        GeoExtent e = key.getExtent();
        GeoCircle c = e.computeBoundingGeoCircle();
        double range = c.getRadius() * mtrf * 2.0;

        _lods[lod]._visibilityRange = range;
    }
    
    double lodNear = 0;
    double prevPos = lodNear;
    for (int i=(int)(numLods-1); i>=0; --i)
    {
        _lods[i]._morphEnd   = _lods[i]._visibilityRange;
        _lods[i]._morphStart = prevPos + (_lods[i]._morphEnd - prevPos) * _morphStartRatio;
        prevPos = _lods[i]._morphStart;
    }
}
