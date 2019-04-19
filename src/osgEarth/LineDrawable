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
#ifndef OSGEARTH_LINEDRAWABLE_H
#define OSGEARTH_LINEDRAWABLE_H 1

#include <osgEarth/Common>
#include <osg/Array>
#include <osg/Geometry>
#include <osg/Version>
#include <osg/Geode>

namespace osgEarth
{
    /**
     * Drawable that renders lines using the GPU. It will fall back on rendering
     * OpenGL lines when shader-based rendering is unavailable.
     *
     * Note: If you use this class you must have the oe_ViewportSize
     * uniform set. MapNode sets it automatically so any LineDrawable under
     * a MapNode is fine. Otherwise, use the osgEarth::InstallViewportSizeUniform
     * callback on your scene graph.
     *
     * Note: Use the provided functions whenever possible. Do not access the
     * underlying Geometry arrays directly. The implementation uses special
     * formatting internally and accessing the arrays directly will probably
     * cause trouble.
     */
    class OSGEARTH_EXPORT LineDrawable : public osg::Geometry
    {
    public:

        //! Create new LineDrawable in GL_LINE_STRIP mode
        LineDrawable();

        //! Create a new LineDrawable.
        //! @param[in ] mode GL line mode: GL_LINE_STRIP or GL_LINE_LOOP
        LineDrawable(GLenum mode);

        //! Copy constructor
        LineDrawable(const LineDrawable& rhs, const osg::CopyOp& copy);

        //! Width in pixels of the line
        void setLineWidth(float width_pixels);
        float getLineWidth() const { return _width; }

        //! Stippling pattern for the line (default is 0xFFFF)
        void setStipplePattern(GLushort pattern);
        GLushort getStipplePattern() const { return _pattern; }

        //! Stippling factor for the line (default is 1)
        void setStippleFactor(GLint factor);
        GLint getStippleFactor() const { return _factor; }

        //! Line smoothing (antialiasing)
        void setLineSmooth(bool value);
        bool getLineSmooth() const { return _smooth; }

        //! Sets the overall color of the line
        void setColor(const osg::Vec4& color);
        const osg::Vec4& getColor() const { return _color; }
        
        //! Append a vertex to the line
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

        //! Sets a line width on a custom stateset that will apply to
        //! all LineDrawables used with that state set.
        static void setLineWidth(osg::StateSet* stateSet, float value, int overrideFlags=osg::StateAttribute::ON);

    public:

        //! Binding location for "previous" vertex attribute (default = 9)
        static int PreviousVertexAttrLocation;

        //! Binding location for "next" vertex attribute (default = 10)
        static int NextVertexAttrLocation;

    public: // osg::Node

        //! Replace methods from META_Node so we can override accept
        virtual osg::Object* cloneType() const { return new LineDrawable(); }
        virtual osg::Object* clone(const osg::CopyOp& copyop) const { return new LineDrawable(*this,copyop); }
        virtual bool isSameKindAs(const osg::Object* obj) const { return dynamic_cast<const LineDrawable*>(obj)!=NULL; }
        virtual const char* className() const { return "LineDrawable"; }
        virtual const char* libraryName() const { return "osgEarth"; }

        //! Override Node::accept to include the singleton GPU statset
        virtual void accept(osg::NodeVisitor& nv);

    public: // osg::Object

        virtual void resizeGLObjectBuffers(unsigned maxSize);
        virtual void releaseGLObjects(osg::State* state) const;

    public:
            
        //! GL mode (for serializer only; do not use)
        void setMode(GLenum mode);
        GLenum getMode() const { return _mode; }

    protected:

        //! destructor
        virtual ~LineDrawable();

    private:
        GLenum _mode;
        bool _gpu;
        osg::Vec4 _color;
        GLint _factor;
        GLushort _pattern;
        float _width;
        bool _smooth;
        unsigned _first;
        unsigned _count;
        osg::Vec3Array* _current;
        osg::Vec3Array* _previous;
        osg::Vec3Array* _next;
        osg::Vec4Array* _colors;

        void initialize();
        void setupShaders();

        friend class LineGroup;

        unsigned actualVertsPerVirtualVert(unsigned) const;
        unsigned numVirtualVerts(const osg::Array*) const;
        unsigned getRealIndex(unsigned) const;
        void updateFirstCount();

        static osg::observer_ptr<osg::StateSet> s_gpuStateSet;
        osg::ref_ptr<osg::StateSet> _gpuStateSet;
    };


    /**
     * Group for importing LineDrawables from osg::Geometry or for
     * optimizing multiple LineDrawables for performance.
     *
     * Note: LineGroup inherits from Geode (for now) to maintain support for
     * the OSG 3.4 MergeGeometryVisitor (which only works on Geodes). Once we
     * make OSG 3.6 the minimum supported version, we can change this to Group.
     */
    class OSGEARTH_EXPORT LineGroup : public osg::Geode
    {
    public:
        META_Node(osgEarth, LineGroup);

        //! Construct a new line group
        LineGroup();

        //! Copy constructor
        LineGroup(const LineGroup& rhs, const osg::CopyOp& copy);

        //! Imports any GL line drawables from a node graph, converts them
        //! to LineDrawables, and adds them to this LineGroup. If will detect
        //! and set any LineWidth or LineStipple attributes it finds, but will not
        //! find any attributes that occur in the graph above the imported node.
        //!
        //! If you set removePrimitiveSets to true, it will remove all line-based
        //! primitive sets from the node after import.
        //!
        //! This method is for quickly importing legacy scene graphs; if you are
        //! writing new code, use the LineDrawable API directly instead!
        void import(osg::Node* node, bool removePrimitiveSets =false);
        
        //! Optimize the LineDrawables under this group for performance.
        //! Only call this after you finish adding drawables to your group.
        //! It will attempt to combine drawables and state sets, but it will also
        //! render the graph henceforth immutable.
        void optimize();

        //! Get child i as a LineDrawable
        LineDrawable* getLineDrawable(unsigned i);

    protected:
        //! destructor
        virtual ~LineGroup();
    };

    
    // Template implementations ..........................................

    template<typename T>
    void LineDrawable::pushVertexAttrib(T* vaa, const typename T::ElementDataType& value)
    {
        unsigned nvv = numVirtualVerts(vaa);
        unsigned num = actualVertsPerVirtualVert(nvv);
        for (unsigned i = 0; i<num; ++i)
            vaa->push_back(value);
    }

    template<typename T>
    const typename T::ElementDataType& LineDrawable::getVertexAttrib(const T* vaa, unsigned i) const
    {
        return (*vaa)[getRealIndex(i)];
    }

    template<typename T>
    void LineDrawable::importVertexAttribArray(unsigned location, const T* data)
    {
        T* vaa = osg::cloneType(data);
        setVertexAttribArray(location, vaa);
        for (unsigned i=0; i < data->getNumElements(); ++i)
            pushVertexAttrib(vaa, (*data)[i]);
    }

} // namespace osgEarth

#endif // OSGEARTH_LINEDRAWABLE_H
