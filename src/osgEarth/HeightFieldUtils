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
#ifndef OSGEARTH_HEIGHTFIELDUTILS_H
#define OSGEARTH_HEIGHTFIELDUTILS_H

#include <osgEarth/Common>
#include <osgEarth/GeoData>
#include <osg/Shape>
#include <osg/CoordinateSystemNode>
#include <osg/ClusterCullingCallback>

namespace osgEarth
{
    struct HeightFieldNeighborhood
    {
        osg::ref_ptr<osg::HeightField> _center;
        osg::ref_ptr<osg::HeightField> _neighbors[8];

        osg::HeightField* getNeighbor(int xoffset, int yoffset) const {
            if ( xoffset == 0 && yoffset == 0 ) return _center.get();
            int index = 3*(yoffset+1)+(xoffset+1);
            if (index > 4) index--;
            return _neighbors[index].get();
        }

        void setNeighbor(int xoffset, int yoffset, osg::HeightField* hf) {
            if ( xoffset == 0 && yoffset == 0 ) {
                _center = hf;
            }
            else {
                int index = 3*(yoffset+1)+(xoffset+1);
                if (index > 4) index--;
                _neighbors[index] = hf;
            }
        }

        bool getNeighborForNormalizedLocation(double nx, double ny, osg::HeightField*& hfPtr, double& out_nx, double& out_ny) const {
            int xoffset = nx < 0.0 ? -1 : nx > 1.0 ? 1 : 0;
            int yoffset = ny < 0.0 ? 1 : ny > 1.0 ? -1 : 0;
            if ( xoffset != 0 || yoffset != 0 )
                hfPtr = getNeighbor(xoffset, yoffset);
            else
                hfPtr = _center.get();
            out_nx = nx < 0.0 ? 1.0+nx : nx > 1.0 ? nx-1.0 : nx;
            out_ny = ny < 0.0 ? 1.0+ny : ny > 1.0 ? ny-1.0 : ny;
            return hfPtr != 0L;
        }
    };


    class OSGEARTH_EXPORT HeightFieldUtils
    {
    public:
        /**
         * Gets the interpolated height value at the specified fractional pixel position.
         */
        static float getHeightAtPixel(
            const osg::HeightField* hf, 
            double c, double r, 
            ElevationInterpolation interpolation = INTERP_BILINEAR);

        /**
         * Gets the height value at the specified column and row, but instead of reading
         * the actual height, interpolates a height based on the neighbors.
         */
        static bool getInterpolatedHeight(
            const osg::HeightField* hf, 
            unsigned c, unsigned r, 
            float& out_height,
            ElevationInterpolation interpoltion = INTERP_BILINEAR);
        
        /**
         * Gets the interpolated height value at the specific geolocation.
         */
        static float getHeightAtLocation(
            const osg::HeightField* hf, 
            double x, double y, 
            double llx, double lly,
            double dx, double dy,
            ElevationInterpolation interpolation = INTERP_BILINEAR);

        /**
         * Gets the normal vector at a geolocation
         */
        static osg::Vec3 getNormalAtLocation(
            const HeightFieldNeighborhood& hood,
            double x, double y, 
            double llx, double lly,
            double dx, double dy,
            ElevationInterpolation interpolation = INTERP_BILINEAR);

        /**
         * Gets the interpolated elevation at the specified "normalized unit location".
         * i.e., nx => [0.0, 1.0], ny => [0.0, 1.0] where 0.0 and 1.0 are the opposing
         * endposts of the heightfield.
         */
        static float getHeightAtNormalizedLocation(
            const osg::HeightField* hf,
            double nx, double ny,
            ElevationInterpolation interp = INTERP_BILINEAR);

        /**
         * Gets the interpolated elevation at the specified "normalized unit location".
         * i.e., nx => [-1.0...2.0], ny => [-1.0...2.0] since it can query neighbors
         * as well. Returns FALSE if there's no heightfield in the hood.
         */
        static bool getHeightAtNormalizedLocation(
            const HeightFieldNeighborhood& hood,
            double nx, double ny,
            float& output,
            ElevationInterpolation interp = INTERP_BILINEAR);

        /**
         * Scales all the height values in a heightfield from scalar units to "linear degrees".
         * The only purpose of this is to show reasonable height values in a projected
         * Plate Carre map (in which vertical units are not well defined).
         */
        static void scaleHeightFieldToDegrees( osg::HeightField* hf );
        
        /**
         * Creates a heightfield containing MSL heights for the specified extent.
         *
         * @param numCols Width of the heightfield, not including the border
         * @param numRows Height of the heightfield, not including the border
         * @param border Size (in samples) of the border (on each side). This will increase the
         *               actual numRows and numCols.
         * @param expressHeightsAsHAE
         *      If the SRS (in GeoExtent) has a vertical datum, and expressHeightsAsHAE==true,
         *      the height values will be those of its reference geoid. If there is no vertical
         *      datum, or is expressHeightsAsHAE==false, we assume that MSL == the reference
         *      ellipsoid and all the HF values will be zero.
         */
        static osg::HeightField* createReferenceHeightField( 
            const GeoExtent& ex, 
            unsigned         numCols,
            unsigned         numRows,
            unsigned         border,
            bool             expressHeightsAsHAE =true);

        /**
         * Subsamples a heightfield to the specified extent.
         */
        static osg::HeightField* createSubSample(
            const osg::HeightField* input, 
            const GeoExtent&        inputEx,
            const GeoExtent&        outputEx,
            ElevationInterpolation  interpolation = INTERP_BILINEAR);

        /**
         * Resizes a heightfield, keeping the corner values the same and
         * resampling the internal posts.
         */
        static osg::HeightField* resampleHeightField(
            osg::HeightField* input,
            const GeoExtent& inputEx,
            int newX,
            int newY,
            ElevationInterpolation interp = INTERP_BILINEAR );

        /**
         * Resolves any "invalid" height values in the hieghtfield, replacing them
         * with geodetic (ellipsoid) relative values from a Geoid (or zero if no geoid).
         */
        static void resolveInvalidHeights(
            osg::HeightField* grid,
            const GeoExtent&  extent,
            float             invalidValue,
            const Geoid*      geoid );
        
        /**
         * Creates a new cluster culler based on a heightfield.
         * Cluster cullers are for geocentric maps only, and therefore requires
         * the ellipsoid model.
         */
        static osg::NodeCallback* createClusterCullingCallback(
            const osg::HeightField*    grid, 
            const osg::EllipsoidModel* em, 
            float                      verticalScale =1.0f );

        /**
         * Convert a heightfield (and its neighbors) into a normal map* image.
         */
        static NormalMap* convertToNormalMap(
            const HeightFieldNeighborhood& hood,
            const SpatialReference*        hoodSRS);
        
        /**
         * Reads elevation data from one image and writes a normal/curvature map
         * for it to another image of the same size
         */
        static void createNormalMap(
            const osg::Image* elevation,
            NormalMap*        normalMap,
            const GeoExtent&  extent);


        /**
         * Utility function that will take sample points used for interpolation and copy valid values into any of the samples that are NO_DATA_VALUE.
         * Returns false if all values are NO_DATA_VALUE.
         * Returns true if all the values are valid or if we were able to replace NO_DATA_VALUE samples with valid values.
         **/
        static bool validateSamples(float &a, float &b, float &c, float &d);
    };
}

#endif //OSGEARTH_HEIGHTFIELDUTILS_H
