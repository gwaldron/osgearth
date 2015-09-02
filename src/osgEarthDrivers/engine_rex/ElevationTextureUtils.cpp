#include "ElevationTextureUtils"

#include <osgEarth/ImageUtils>
#include <osgEarth/TileKey>

#include <osg/Texture>

#include <numeric>

using namespace osgEarth::Drivers::RexTerrainEngine;
using namespace osgEarth;

#define LC "[ElevationTexureUtils] "

ElevationImageReader::ElevationImageReader(const osg::Image* image)
: _pixelReader(image)
{
    init(image, osg::Matrixf::identity());
}


ElevationImageReader::ElevationImageReader(const osg::Image* image, const osg::Matrix& matrixScaleBias)
: _pixelReader(image)
{
    init(image, matrixScaleBias);
}

void
ElevationImageReader::init(const osg::Image* image, const osg::Matrix& matrixScaleBias)
{
    double s_offset = matrixScaleBias(3,0) * (double)image->s();
    double t_offset = matrixScaleBias(3,1) * (double)image->t();
    double s_span   = matrixScaleBias(0,0) * (double)image->s();
    double t_span   = matrixScaleBias(1,1) * (double)image->t();

    // if the window is smaller than one pixel, forget it.
    if ( s_span < 4.0 || t_span < 4.0 )
    {
        _valid = false;
        return;
    }

    // starting column and row:
    _startCol = osg::clampAbove( ((int)s_offset)-1, 0 );
    _startRow = osg::clampAbove( ((int)t_offset)-1, 0 );

    // ending column and row
    _endCol = osg::clampBelow( _startCol + ((int)s_span) + 1, image->s()-1 );
    _endRow = osg::clampBelow( _startRow + ((int)t_span) + 1, image->t()-1 );

    OE_DEBUG << LC << std::dec 
        << "scale=" << s_offset << ", " << t_offset
        << "; span = " << s_span << ", " << t_span
        << "; c0=" << startCol() << ", r0=" << startRow() << "; c1=" << endCol() << ", r1=" << endRow() << "\n";

    _valid = true;
}

float
ElevationImageReader::elevationN(float s, float t)
{
    int col = startCol() + (endCol()-startCol())*s;
    int row = startRow() + (endRow()-startRow())*t;
    return elevation(col, row);
}

bool
ElevationTexureUtils::findExtrema(osg::Texture* elevationTexture, const osg::Matrix& matrixScaleBias, const TileKey& tileKey, osg::Vec2f& extrema)
{
    if (elevationTexture==0)
    {
        OE_DEBUG << LC << "findExtrema ERROR!! Elevation Texture is NULL"<<std::endl;
        return false;
    }
    // Searches a texture image (using a texture matrix) for the min and max elevation values.
    extrema.set( FLT_MAX, -FLT_MAX );

    osg::Image* image = elevationTexture->getImage(0);
    if ( image )
    {
        ElevationImageReader reader(image, matrixScaleBias);

        if (reader.valid()==false)
        {
            OE_DEBUG << LC << "findExtrema ERROR!! Elevation Texture extents too small"<<std::endl;
            return false;
        }

        OE_DEBUG << LC << "findExtrema: "<<tileKey.str()<<std::endl;

        for(int col=reader.startCol(); col <= reader.endCol(); ++col)
        {
            for(int row=reader.startRow(); row <= reader.endRow(); ++row)
            {
                float elevation = reader.elevation(col, row);
                if ( elevation < extrema[0] ) extrema[0] = elevation;
                if ( elevation > extrema[1] ) extrema[1] = elevation;
            }
        }

        if ( extrema[0] > extrema[1] )
        {
            OE_WARN << LC << "findExtrema ERROR!! (" << tileKey.str() << ") c0=" << reader.startCol() 
                                                                    << ", r0=" << reader.startRow() 
                                                                    << "; c1=" << reader.endCol() 
                                                                    << ", r1=" << reader.endRow() 
                                                                    << ", s=" << image->s() 
                                                                    << ", t=" << image->t() << "\n";
        }
    }
    else
    {
        OE_WARN << LC << "findExtrema ERROR!! (" << tileKey.str() << ") no tex image available\n";
    }

    OE_DEBUG << LC <<tileKey.getLOD()<< " Extrema Min: "<<extrema[0]<<" Max: "<<extrema[1]<<std::endl;

    return extrema[0] <= extrema[1];
}