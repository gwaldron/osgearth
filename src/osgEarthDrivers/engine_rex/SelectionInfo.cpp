#include "SelectionInfo"
#include "SurfaceNode"
#include "RexTerrainEngineOptions"

using namespace osgEarth::Drivers::RexTerrainEngine;
using namespace osgEarth;

#define LC "[SelectionInfo] "

const unsigned SelectionInfo::_uiLODForMorphingRoundEarth = 0;
const double   SelectionInfo::_fLodLowerBound   = 12.0;
const double   SelectionInfo::_fMorphStartRatio = 0.66;

unsigned SelectionInfo::lodForMorphing(bool isProjected)
{
    return (isProjected) ? 0 : _uiLODForMorphingRoundEarth;
}

double SelectionInfo::morphStartRatio(void) 
{
    return _fMorphStartRatio;
}

unsigned SelectionInfo::numLods(void) const
{
    return _numLods;
}

unsigned SelectionInfo::gridDimX(void) const
{
    return _uiGridDimensions.first;
}

unsigned SelectionInfo::gridDimY(void) const
{
    return _uiGridDimensions.second;
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

void SelectionInfo::initialize(unsigned uiFirstLOD, unsigned uiMaxLod, unsigned uiTileSize, double fLodFar)
{
    if (initialized())
    {
        OE_INFO << LC <<"Error: Selection Information already initialized"<<std::endl;
        return;
    }

    if (fLodFar<0)
    {
        OE_INFO << LC <<"Error: Invalid fLodFar hint"<<std::endl;
        return;
    }
    if (uiFirstLOD>uiMaxLod)
    {
        OE_INFO << LC <<"Error: Inconsistent First and Max LODs"<<std::endl;
        return;
    }
    _uiGridDimensions.first  = uiTileSize;
    _uiGridDimensions.second = uiTileSize;

    _uiFirstLOD = uiFirstLOD;

    double fLodNear = 0;
    float fRatio = 1.0;

    unsigned currLOD = _uiFirstLOD;
    while(currLOD<=uiMaxLod)
    {
        double fVisibility = fLodNear + fRatio*(fLodFar-fLodNear);
        if (fVisibility<_fLodLowerBound)
        {
            break;
        }
        fRatio*= 0.5;
        ++currLOD;
    }

    _numLods = currLOD-_uiFirstLOD;

    fLodNear = 0;
    fRatio = 1.0;
    _vecVisParams.resize(_numLods);
    for( int i = 0; i < (int)_numLods; ++i )
    {
        _vecVisParams[i]._visibilityRange = fLodNear + fRatio*(fLodFar-fLodNear);
        _vecVisParams[i]._visibilityRange2 = _vecVisParams[i]._visibilityRange * _vecVisParams[i]._visibilityRange;
        fRatio *= 0.5;
    }

    double fPrevPos = fLodNear;
    for (int i=(int)(_numLods-1); i>=0; --i)
    {
        _vecVisParams[i]._fMorphEnd   = _vecVisParams[i]._visibilityRange;
        _vecVisParams[i]._fMorphStart = fPrevPos + (_vecVisParams[i]._fMorphEnd - fPrevPos) * _fMorphStartRatio;

        fPrevPos = _vecVisParams[i]._fMorphStart;
    }
    for( int i = 0; i < (int)_numLods; ++i ) 
    {
        OE_DEBUG << "LOD[" << i+_uiFirstLOD<<"] = "<<_vecVisParams[i]._visibilityRange
                 <<" Start: "<<_vecVisParams[i]._fMorphStart
                 <<" End  : "<<_vecVisParams[i]._fMorphEnd
                 <<std::endl;
    }
}