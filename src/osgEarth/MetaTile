/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
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
#ifndef OSGEARTH_METATILE_H
#define OSGEARTH_METATILE_H

#include <osgEarth/Common>
#include <osgEarth/ImageUtils>
#include <osgEarth/GeoData>
#include <osgEarth/TileKey>
#include <osg/Image>

namespace osgEarth
{
    class ImageLayer;

    /**
     * Metadata groups a collection of adjacent data tiles
     * together to facilitate operations that overlap multiple tiles.
     */
    class OSGEARTH_EXPORT MetaImage
    {
    public:
        //! Construct a new Metatiled image
        MetaImage() { }

        //! Sets the data elevation at location (x,y), where (0,0) is the center.
        bool setImage(int x, int y, osg::Image* image, const osg::Matrix& scaleBias);

        //! Gets the image at the neighbor location (x,y).
        osg::Image* getImage(int x, int y) const;

        //! Gets the positioning matrix for neightbor location (x,y).
        const osg::Matrix& getScaleBias(int x, int y) const;

        //! Reads the data from parametric location (u,v), where [u,v] in [-1, +2].
        //! Returns true upon success with the value in [output];
        //! false if there is no tile at the read location.
        bool read(double u, double v, osg::Vec4& output) const;

        void dump() const;

    private:
        //virtual ~MetaImage() { }

        struct Tile {
            Tile();
            bool _valid;
            osg::ref_ptr<osg::Image> _imageRef;
            ImageUtils::PixelReader _read;
            osg::Matrix _scaleBias;
        };

        Tile _tiles[3][3]; // col, row
    };

} // namespace osgEarth

#endif // OSGEARTH_METATILE_H
