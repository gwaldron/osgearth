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
#ifndef OSGEARTH_UTIL_GEODETIC_LABELING_ENGINE_H
#define OSGEARTH_UTIL_GEODETIC_LABELING_ENGINE_H 1

#include <osgEarthUtil/Common>
#include <osgEarthUtil/GraticuleLabelingEngine>
#include <osgEarthUtil/LatLongFormatter>
#include <osgEarth/GeoData>
#include <osgEarth/MapNode>
#include <osgEarth/Containers>
#include <osgEarthAnnotation/LabelNode>

namespace osgEarth { namespace Util
{
    using namespace osgEarth;
    using namespace osgEarth::Annotation;

    /**
     * Node that plots geodetic coordinats labels along the edge of the
     * viewport when you are looking straight down on a zoomed-in area.
     */
    class GeodeticLabelingEngine : public GraticuleLabelingEngine
    {
    public:
        //! Construct a new labeling engine with the map's SRS
        GeodeticLabelingEngine(const SpatialReference* srs);

        virtual bool updateLabels(const osg::Vec3d& LL_world, osg::Vec3d& UL_world, osg::Vec3d& LR_world, ClipSpace& window, CameraData& data);

        double getResolution() const;
        void setResolution(double resolution);

    protected:
        std::string getText(const GeoPoint& location, bool lat);

        double _resolution;

        osg::ref_ptr<LatLongFormatter> _formatter;
    };

} } // namespace osgEarth::Util

#endif // GeodeticLabelingEngine
