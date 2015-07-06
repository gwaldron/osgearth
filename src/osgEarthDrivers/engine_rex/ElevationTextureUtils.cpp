#include "ElevationTextureUtils"

#include <osgEarth/ImageUtils>
#include <osgEarth/TileKey>

#include <osg/Texture>

#include <numeric>

using namespace osgEarth::Drivers::RexTerrainEngine;
using namespace osgEarth;

#define LC "[ElevationTexureUtils] "

bool
ElevationTexureUtils::findExtrema(osg::Texture* elevationTexture, const osg::Matrix& matrixScaleBias, const TileKey& tileKey, osg::Vec2f& extrema)
{
    if (elevationTexture==0)
    {
        OE_DEBUG << LC << "Elevation Texture is NULL"<<std::endl;
        return false;
    }
    // Searches a texture image (using a texture matrix) for the min and max elevation values.
    extrema.set( FLT_MAX, -FLT_MAX );

    osg::Image* image = elevationTexture->getImage(0);
    if ( image )
    {
        double s_offset = matrixScaleBias(3,0) * (double)image->s();
        double t_offset = matrixScaleBias(3,1) * (double)image->t();
        double s_span   = matrixScaleBias(0,0) * (double)image->s();
        double t_span   = matrixScaleBias(1,1) * (double)image->t();

        // if the window is smaller than one pixel, forget it.
        if ( s_span < 4.0 || t_span < 4.0 )
            return false;

        ImageUtils::PixelReader read(image);

        // starting column and row:
        int startCol = osg::clampAbove( ((int)s_offset)-1, 0 );
        int startRow = osg::clampAbove( ((int)t_offset)-1, 0 );

        // ending column and row
        int endCol = osg::clampBelow( startCol + ((int)s_span) + 1, image->s()-1 );
        int endRow = osg::clampBelow( startRow + ((int)t_span) + 1, image->t()-1 );

        OE_DEBUG << LC 
            << "find: key=" << tileKey.str() << std::dec 
            << "scale=" << s_offset << ", " << t_offset
            << "; span = " << s_span << ", " << t_span
            << "; c0=" << startCol << ", r0=" << startRow << "; c1=" << endCol << ", r1=" << endRow << "\n";

        for(int col=startCol; col <= endCol; ++col)
        {
            for(int row=startRow; row <= endRow; ++row)
            {
                float elevation = read(col, row).r();
                if ( elevation < extrema[0] ) extrema[0] = elevation;
                if ( elevation > extrema[1] ) extrema[1] = elevation;
            }
        }

        if ( extrema[0] > extrema[1] )
        {
            OE_WARN << LC << "findExtrema ERROR (" << tileKey.str() << ") c0=" << startCol << ", r0=" << startRow << "; c1=" << endCol << ", r1=" << endRow << ", s=" << image->s() << ", t=" << image->t() << "\n";
        }
    }
    else
    {
        OE_WARN << LC << "findExtrema ERROR (" << tileKey.str() << ") no tex image available\n";
    }

    return extrema[0] <= extrema[1];
}