/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2016 Pelican Mapping
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
#ifndef OSGEARTH_CASCADE_DRAPING_DECORATOR
#define OSGEARTH_CASCADE_DRAPING_DECORATOR

#include <osgEarth/Common>
#include <osgEarth/Containers>
#include <osgEarth/DrapingCullSet>
#include <osg/Group>

namespace osg {
    class Plane;
}
namespace osgUtil {
    class CullVisitor;
}

namespace osgEarth
{
    class TerrainResources;

    /**
     * Projects geometry from a separate scene graph onto the terrain.
     */
    class OSGEARTH_EXPORT CascadeDrapingDecorator : public osg::Group
    {
    public:
        //! Constructs a decorator
        CascadeDrapingDecorator(const SpatialReference* srs, TerrainResources*);

        //! Sets the maximum number of texture cascases to use (default=4, max=4)
        void setMaxNumCascades(unsigned value);

        //! Number of multisamples to use to improve image quality (default=4)
        void setNumMultiSamples(unsigned value);

        //! Texture size for draping target (default=1024)
        void setTextureSize(unsigned value);

        //! Whether to mipmap projected textures (default=false)
        void setUseMipMaps(bool value);

        //! Whether to use projection matrix fitting
        void setUseProjectionFitting(bool value);
        bool getUseProjectionFitting() const { return _useProjectionFitting; }

        //! Sets the minimum n/f ratio for projection fitting
        void setMinimumNearFarRatio(double value);

        //! Debugging method (used by osgearth_overlayviewer to visualize cascades)
        osg::Node* getDump();

    public: // osg::Node

        void traverse( osg::NodeVisitor& nv );

    protected:

        virtual ~CascadeDrapingDecorator() { }

        // Data for one RTT cascade
        struct Cascade
        {
            double _minClipY, _maxClipY;
            //double _minX, _minY, _maxX, _maxY;
            osg::BoundingBoxd _box;
            double _widthNDC, _heightNDC;

            void expandToInclude(const osg::Vec3d&);
            void computeProjection(const osg::Matrix&, const osg::Matrix&, const osg::EllipsoidModel&, const osg::Plane&, double, const osg::BoundingBoxd&);
            void computeClipCoverage(const osg::Matrix&, const osg::Matrix&);
            void makeProj(double dp);

            osg::ref_ptr<osg::Camera> _rtt;
            osg::Matrix _rttProj;
            osg::ref_ptr<osg::StateSet> _stateSet;
        };

        // RTT configuration for a single master camera.
        // (i.e. there will be one of these for each Camera that traverses DrapingDecorator)
        struct CameraLocal : public osg::Observer
        {
            Cascade _cascades[8];
            unsigned _numCascades;
            unsigned _maxCascades;
            osg::ref_ptr<osg::StateSet> _rttSS;
            osg::ref_ptr<osg::StateSet> _terrainSS;
            osg::Matrix _projMatrixLastFrame;

            void initialize(osg::Camera* camera, CascadeDrapingDecorator&);
            void traverse(osgUtil::CullVisitor*, CascadeDrapingDecorator&, const osg::BoundingSphere&);
            void clear();
            void constrainRttBoxToFrustum(const osg::Matrix&, const osg::Matrix&, const osg::EllipsoidModel&, bool, osg::BoundingBoxd& rttBox);
            void constrainRttBoxToBounds(const osg::Matrix&, const osg::BoundingSphere&, osg::BoundingBoxd& rttBox);
            bool intersectTerrain(CascadeDrapingDecorator& dec, const osg::Vec3d& startWorld, const osg::Vec3d& endWorld, osg::Vec3d& output);

            void dump(const osg::Camera* cam, CascadeDrapingDecorator&);

            osg::observer_ptr<osg::Camera> _token;            
            void objectDeleted(void*);
            ~CameraLocal();
        };
        
        int _unit;
        unsigned _multisamples;
        unsigned _maxCascades;
        unsigned _texSize;
        bool _mipmapping;
        double _maxHorizonDistance;
        PerObjectFastMap<const osg::Camera*, CameraLocal> _data;
        osg::ref_ptr<osg::Group> _dump;
        bool _debug;
        osg::ref_ptr<const SpatialReference> _srs;
        bool _constrainMaxYToFrustum;
        bool _constrainRttBoxToDrapingSetBounds;
        bool _useProjectionFitting;
        double _minNearFarRatio;

        // tracks drapable objects in the scene graph
        mutable DrapingManager _manager;
        DrapingManager& getDrapingManager() { return _manager; }
        friend class MapNode;

        osg::observer_ptr<TerrainResources> _resources;
        void reserveTextureImageUnit();
    };

} // namespace osgEarth

#endif //OSGEARTH_CASCADE_DRAPING_DECORATOR
