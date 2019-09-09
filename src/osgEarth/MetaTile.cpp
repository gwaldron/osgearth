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
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/
#include <osgEarth/MetaTile>

using namespace osgEarth;

#define LC "[MetaImage] "


MetaImage::Tile::Tile() :
_valid(false),
_read(0L)
{
    //nop
}

bool
MetaImage::setImage(int x, int y, osg::Image* image, const osg::Matrix& scaleBias)
{
    if (image!=0L && x >= -1 && x <= 1 && y >= -1 && y <= 1)
    {
        x += 1, y += 1;

        // We store this reference just in case the caller does not hold on to it,
        // in which case the PixelReader would have a dangling pointer and crash.
        _tiles[x][y]._imageRef = image;

        _tiles[x][y]._read.setImage(image);
        _tiles[x][y]._scaleBias = scaleBias;
        _tiles[x][y]._valid = true;

        return true;
    }
    else
    {
        OE_WARN << LC << "ILLEGAL call to MetaImage.setImage\n";
        return false;
    }
}

osg::Image*
MetaImage::getImage(int x, int y) const
{
    x = osg::clampBetween(x+1, 0, 2);
    y = osg::clampBetween(y+1, 0, 2);
    return _tiles[x][y]._imageRef.get();
}

const osg::Matrix&
MetaImage::getScaleBias(int x, int y) const
{
    x = osg::clampBetween(x+1, 0, 2);
    y = osg::clampBetween(y+1, 0, 2);
    return _tiles[x][y]._scaleBias;
}

bool
MetaImage::read(double u, double v, osg::Vec4& output) const
{
    // clamp the input coordinates to the legal range:
    u = osg::clampBetween(u, -1.0, 2.0);
    v = osg::clampBetween(v, -1.0, 2.0);

    // resolve the tile to sample:
    int x = u < 0.0 ? 0 : u <= 1.0 ? 1 : 2;
    int y = v < 0.0 ? 2 : v <= 1.0 ? 1 : 0;

    const Tile& tile = _tiles[x][y];

    if (!tile._valid)
        return false;

    // transform the coordinates to the tile:
    u += u < 0.0 ? 1.0 : u > 1.0 ? -1.0 : 0.0;
    v += v < 0.0 ? 1.0 : v > 1.0 ? -1.0 : 0.0;

    // scale/bias to this tile's extent:
    u = u * tile._scaleBias(0, 0) + tile._scaleBias(3, 0);
    v = v * tile._scaleBias(1, 1) + tile._scaleBias(3, 1);

    output = tile._read(u, v);
    return true;
}

void
MetaImage::dump() const
{
    for (int x = 0; x <= 2; ++x) {
        for (int y = 0; y <= 2; ++y) {
            const Tile& tile = _tiles[x][y];
            if (!tile._valid) {
                OE_INFO << "    [" << x << "][" << y << "]: invalid\n";
            }
            else {
                OE_INFO << "    [" << x << "][" << y << "]: "
                    << "s=" << tile._scaleBias(0,0) 
                    << ", b=" << tile._scaleBias(3, 0) << " "
                    << tile._scaleBias(3, 1) << "\n";
            }                
        }
    }
}
