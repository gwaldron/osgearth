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
#ifndef OSGEARTH_CLAMPING_H
#define OSGEARTH_CLAMPING_H

#include <osgEarth/Common>
#include <osgEarth/Containers>
#include <osgEarth/ClampableNode>
#include <osg/Node>
#include <osg/Geometry>
#include <osg/ObserverNodePath>
#include <osg/Camera>

namespace osgEarth
{
    /**
     * Support for GPU Clamping.
     *
     * GPU Clamping makes use of a custom vertex attribute. The contents of this 
     * vec4 attribute are:
     *   attr[0] : X coordinate of reference vertex
     *   attr[1] : Y coordinate of reference vertex
     *   attr[2] : Vertical offset to apply after clamping
     *   attr[3] : 0 = clamp vertex directly; 1 = clamp indirectly using reference vertex
     */
    class OSGEARTH_EXPORT Clamping
    {
    public: // mutable

        // GLSL attribute binding location for clamping vertex attributes
        static int AnchorAttrLocation;

        // GLSL attribute binding location for clamping vertex heights
        static int HeightsAttrLocation;

    public: // non-mutable

        // Name of the clamping anchor vertex attribute
        static const char* AnchorAttrName;

        // Name of the boolean uniform that tells the shader whether the
        // geometry has clamping attributes.
        static const char* HasAttrsUniformName;

        // Name of the uniform used to apply an altitude offset to clamped geometry
        static const char* AltitudeOffsetUniformName;

        // Name of the clamping heights array attribute
        static const char* HeightsAttrName;

        // When setting clamping vertex attributes, use these constants in
        // the alpha channel to indicate whether the vertex is to be clamped
        // to the reference vertex in the attribute data, or to the ground.
        static const float ClampToAnchor;
        static const float ClampToGround;

    public: // utility methods

        // Installs a uniform on the stateset indicating that geometry has the
        // clamping vertex attribute installed.
        static void installHasAttrsUniform(
            osg::StateSet* stateset );

        static void applyDefaultClampingAttrs(
            osg::Node* node,
            float      verticalOffset =0.0f);

        static void applyDefaultClampingAttrs(
            osg::Geometry* geom,
            float          verticalOffset =0.0f);

        static void setHeights(
            osg::Geometry*   geom,
            osg::FloatArray* heights );
    };
    

    /**
     * Culling set for tracking groups whose contents should be GPU-clamped
     * The ClampableNode class inserts scene graphs into this objects during
     * the CULL traversal; then the ClampingTechnique traverses those graphs.
     */
    class OSGEARTH_INTERNAL ClampingCullSet
    {
    public:
        struct Entry
        {            
            osg::ref_ptr<osg::Group>     _node;
            osg::ref_ptr<osg::RefMatrix> _matrix;
            osg::ObserverNodePath        _path;
            int                          _frame;
        };

    public:
        ClampingCullSet();
        ~ClampingCullSet() { } // not virtual

        /** Pushes a node and its matrix into the cull set */
        void push(ClampableNode* node, const osg::NodePath& path, const osg::FrameStamp* stamp);

        /** Runs a node visitor on the cull set, optionally popping as it goes along. */
        void accept(osg::NodeVisitor& nv);

        /** Bounds of this set */
        const osg::BoundingSphere& getBound() const { return _bs; }

        /** Number of elements in the set */
        unsigned size() const { return _entries.size(); }

    private:
        std::vector<Entry>  _entries;
        osg::BoundingSphere _bs;
        bool                _frameCulled;
    };

    /**
     * Houses all the active ClampingCullSets under an osgEarth TerrainEngine.
     * ClampableNode will use this object to gain access to a ClampingCullSet
     * during the CULL traversal.
     */
    class OSGEARTH_INTERNAL ClampingManager
    {
    public:
        //! Gets the cull set associated with a camera.
        ClampingCullSet& get(const osg::Camera*);

    private:
        PerObjectFastMap<const osg::Camera*, ClampingCullSet> _sets;
    };
}

#endif // OSGEARTH_CLAMPING_H
