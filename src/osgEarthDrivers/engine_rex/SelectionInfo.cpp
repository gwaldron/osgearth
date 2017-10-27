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
#include "SelectionInfo"
#include "SurfaceNode"
#include "RexTerrainEngineOptions"

#include <osgEarth/Profile>

using namespace osgEarth::Drivers::RexTerrainEngine;
using namespace osgEarth;

#define LC "[SelectionInfo] "

const unsigned SelectionInfo::_uiLODForMorphingRoundEarth = 0;
const double   SelectionInfo::_fLodLowerBound   = 12.0;
const double   SelectionInfo::_fMorphStartRatio = 0.66;

unsigned SelectionInfo::getLODForMorphing(bool isProjected)
{
    return (isProjected) ? 0 : _uiLODForMorphingRoundEarth;
}

VisParameters SelectionInfo::visParameters(unsigned lod) const
{
    if (lod-_uiFirstLOD>=_vecVisParams.size())
    {
        // note, this can happen if firstLOD() is set
        OE_DEBUG << LC <<"Index out of bounds"<<std::endl;
        return VisParameters();
    }
    return _vecVisParams[lod-_uiFirstLOD];
}

bool SelectionInfo::initialized(void) const
{
    return _vecVisParams.size()>0;
}

void SelectionInfo::initialize(unsigned uiFirstLod, unsigned uiMaxLod, const Profile* profile, double mtrf)
{
    if (initialized())
    {
        OE_INFO << LC <<"Error: Selection Information already initialized"<<std::endl;
        return;
    }
    if (uiFirstLod>uiMaxLod)
    {
        OE_INFO << LC <<"Error: Inconsistent First and Max LODs"<<std::endl;
        return;
    }

    _uiFirstLOD = uiFirstLod;

    double fLodNear = 0;
    float fRatio = 1.0;

    _numLods = uiMaxLod+1u; // - uiFirstLod;

    _vecVisParams.resize(_numLods);

    for (unsigned lod = 0; lod <= uiMaxLod; ++lod)
    {
        TileKey key(lod, 0, 0, profile);
        GeoExtent e = key.getExtent();
        GeoCircle c = e.computeBoundingGeoCircle();
        double range = c.getRadius() * mtrf * 2.0;

        _vecVisParams[lod]._visibilityRange = range;
        _vecVisParams[lod]._visibilityRange2 = range*range;
    }
    
    fLodNear = 0;
    double fPrevPos = fLodNear;
    for (int i=(int)(_numLods-1); i>=0; --i)
    {
        _vecVisParams[i]._fMorphEnd   = _vecVisParams[i]._visibilityRange;
        _vecVisParams[i]._fMorphStart = fPrevPos + (_vecVisParams[i]._fMorphEnd - fPrevPos) * _fMorphStartRatio;
        fPrevPos = _vecVisParams[i]._fMorphStart;
    }
}
