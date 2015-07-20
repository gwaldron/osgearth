#include "SelectionInfo"
#include "SurfaceNode"
#include "RexTerrainEngineOptions"

using namespace osgEarth::Drivers::RexTerrainEngine;
using namespace osgEarth;

#define LC "[SelectionInfo] "

const unsigned SelectionInfo::_uiLODForMorphingRoundEarth = 5;
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
    if (lod>=_vecVisParams.size())
    {
        OE_INFO << LC <<"Index oout of bounds"<<std::endl;
        return VisParameters();
    }
    return _vecVisParams[lod];
}

bool SelectionInfo::initialized(void) const
{
    return _vecVisParams.size()>0;
}

void SelectionInfo::initialize(unsigned uiTileSize, unsigned int uiMaxLod, double fLodFar)
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
    _uiGridDimensions.first  = uiTileSize;
    _uiGridDimensions.second = uiTileSize;

    double fLodNear = 0;
    float fRatio = 1.0;

    unsigned numLods = 0;
    while(numLods<uiMaxLod)
    {
        double fVisibility = fLodNear + fRatio*(fLodFar-fLodNear);
        if (fVisibility<_fLodLowerBound)
        {
            break;
        }
        fRatio*= 0.5;
        ++numLods;
    }

    _numLods = numLods;

    fLodNear = 0;
    fRatio = 1.0;
    _vecVisParams.resize(_numLods);
    for( int i = 0; i < _numLods; ++i )
    {
        _vecVisParams[i]._fVisibility = fLodNear + fRatio*(fLodFar-fLodNear);
        fRatio*= 0.5;
    }

    double fPrevPos = fLodNear;
    for (int i=_numLods-1; i>=0; --i)
    {
        _vecVisParams[i]._fMorphEnd   = _vecVisParams[i]._fVisibility;
        _vecVisParams[i]._fMorphStart = fPrevPos + (_vecVisParams[i]._fMorphEnd - fPrevPos) * _fMorphStartRatio;

        fPrevPos = _vecVisParams[i]._fMorphStart;
    }
    for( int i = 0; i < _numLods; ++i ) 
    {
        OE_INFO << "LOD[" << i<<"] = "<<_vecVisParams[i]._fVisibility
                <<" Start: "<<_vecVisParams[i]._fMorphStart
                <<" End  : "<<_vecVisParams[i]._fMorphEnd
                <<std::endl;
    }
}