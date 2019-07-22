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
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */
#ifndef OSGEARTH_POINTDRAWABLE_H
#define OSGEARTH_POINTDRAWABLE_H 1

#include <osgEarth/Common>
#include <osg/Array>
#include <osg/Geometry>
#include <osg/Version>
#include <osg/Geode>

namespace osgEarth
{
    /**
     * Drawable that renders points.
     *
     * Note: Use the provided functions whenever possible. Do not access the
     * underlying Geometry arrays directly. The implementation uses special
     * formatting internally and accessing the arrays directly will probably
     * cause trouble.
     */
    class OSGEARTH_EXPORT PointDrawable : public osg::Geometry
    {
    public:
        //! Create new PointDrawable
        PointDrawable();

        //! Copy constructor
        PointDrawable(const PointDrawable& rhs, const osg::CopyOp& copy);

        //! Width in pixels of the point
        void setPointSize(float width_pixels);
        float getPointSize() const { return _width; }

        //! Point smoothing (Anti-aliasing/rounding)
        void setPointSmooth(bool value);
        bool getPointSmooth() const { return _smooth; }

        //! Sets the overall color of the drawable
        void setColor(const osg::Vec4& color);
        const osg::Vec4& getColor() const { return _color; }
        
        //! Append a vertex to the drawable
        void pushVertex(const osg::Vec3& vert);

        //! Set the value of a vertex at index i
        void setVertex(unsigned i, const osg::Vec3& vert);

        //! Gets the vertex at index i
        const osg::Vec3& getVertex(unsigned i) const;

        //! Sets the color of a vertex at index i
        void setColor(unsigned i, const osg::Vec4& color);

        //! Copy a vertex array into the drawable
        void importVertexArray(const osg::Vec3Array* verts);
        
        //! Copy a vertex attribute array into the drawable
        template<typename T>
        void importVertexAttribArray(unsigned location, const T* data);
        
        //! Allocate space for vertices
        void allocate(unsigned numVerts);

        //! Clears all data
        void clear();

        //! Number of vertices in the drawable
        unsigned getNumVerts() const;

        //! Number of vertices in the drawable
        unsigned size() const { return getNumVerts(); }

        //! Appends a vertex to an attribute array. Use this instead of adding
        //! to the array directly!
        template<typename T>
        void pushVertexAttrib(T* vaa, const typename T::ElementDataType& value);

        //! Gets the value of the vertex attribute from an array at index i
        template<typename T>
        const typename T::ElementDataType& getVertexAttrib(const T* vaa, unsigned i) const;

        //! Pre-allocate space for vertices
        void reserve(unsigned size);

        //! Index of the first vertex to draw (default = 0)
        void setFirst(unsigned index);
        unsigned getFirst() const;

        //! Number of vertices to draw (default = 0, which means draw to the
        //! end of the line
        void setCount(unsigned count);
        unsigned getCount() const;

        //! Rebuild the primitive sets for this drawable. You MUST call this
        //! after adding new data to the drawable!
        void dirty();
        void finish() { dirty(); }

    public: // osg::Geometry

        virtual void drawImplementation(osg::RenderInfo& ri) const;

    public: // osg::Node

        //! Replace methods from META_Node so we can override accept
        virtual osg::Object* cloneType() const { return new PointDrawable(); }
        virtual osg::Object* clone(const osg::CopyOp& copyop) const { return new PointDrawable(*this,copyop); }
        virtual bool isSameKindAs(const osg::Object* obj) const { return dynamic_cast<const PointDrawable*>(obj)!=NULL; }
        virtual const char* className() const { return "PointDrawable"; }
        virtual const char* libraryName() const { return "osgEarth"; }

        //! Override Node::accept to include the singleton GPU statset
        virtual void accept(osg::NodeVisitor& nv);

    public: // osg::Object
        
        virtual void compileGLObjects(osg::RenderInfo& renderInfo) const;
        virtual void resizeGLObjectBuffers(unsigned maxSize);
        virtual void releaseGLObjects(osg::State* state) const;

    protected:

        //! destructor
        virtual ~PointDrawable();

    private:
        bool _gpu;
        osg::Vec4 _color;
        float _width;
        bool _smooth;
        unsigned _first;
        unsigned _count;
        osg::Vec3Array* _current;
        osg::Vec4Array* _colors;

        void initialize();
        void setupState();
        void updateFirstCount();

        static osg::observer_ptr<osg::StateSet> s_sharedStateSet;
        osg::ref_ptr<osg::StateSet> _sharedStateSet;
        mutable bool _sharedStateSetCompiled;

        void checkSharedStateSet(osg::State*) const;
    };


    /**
     * Group for importing PointDrawables from osg::Geometry or for
     * optimizing multiple PointDrawables for performance.
     *
     * Note: PointGroup inherits from Geode (for now) to maintain support for
     * the OSG 3.4 MergeGeometryVisitor (which only works on Geodes). Once we
     * make OSG 3.6 the minimum supported version, we can change this to Group.
     */
    class OSGEARTH_EXPORT PointGroup : public osg::Geode
    {
    public:
        META_Node(osgEarth, PointGroup);

        //! Construct a new line group
        PointGroup();

        //! Copy constructor
        PointGroup(const PointGroup& rhs, const osg::CopyOp& copy);

        //! Imports any GL point drawables from a node graph, converts them
        //! to PointDrawables, and adds them to this group. 
        //!
        //! If you set removePrimitiveSets to true, it will remove all line-based
        //! primitive sets from the node after import.
        //!
        //! This method is for quickly importing legacy scene graphs; if you are
        //! writing new code, use the PointDrawable API directly instead!
        void import(osg::Node* node, bool removePrimitiveSets =false);
        
        //! Optimize the drawables under this group for performance.
        //! Only call this after you finish adding drawables to your group.
        //! It will attempt to combine drawables and state sets, but it will also
        //! render the graph henceforth immutable.
        void optimize();

        //! Get child i as a LineDrawable
        PointDrawable* getPointDrawable(unsigned i);

    protected:
        //! destructor
        virtual ~PointGroup();
    };

    
    // Template implementations ..........................................

    template<typename T>
    void PointDrawable::pushVertexAttrib(T* vaa, const typename T::ElementDataType& value)
    {
        vaa->push_back(value);
    }

    template<typename T>
    const typename T::ElementDataType& PointDrawable::getVertexAttrib(const T* vaa, unsigned i) const
    {
        return (*vaa)[i];
    }

    template<typename T>
    void PointDrawable::importVertexAttribArray(unsigned location, const T* data)
    {
        T* vaa = osg::cloneType(data);
        setVertexAttribArray(location, vaa);
        for (unsigned i=0; i < data->getNumElements(); ++i)
            pushVertexAttrib(vaa, (*data)[i]);
    }

} // namespace osgEarth

#endif // OSGEARTH_POINTDRAWABLE_H
