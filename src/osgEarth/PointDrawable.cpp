
/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2018 Pelican Mapping
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
#include <osgEarth/PointDrawable>
#include <osgEarth/Shaders>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/GLUtils>
#include <osgEarth/CullingUtils>
#include <osgEarth/LineFunctor>
#include <osg/PointSprite>
#include <osg/Point>
#include <osgDB/ObjectWrapper>
#include <osgUtil/Optimizer>


#if defined(OSG_GLES1_AVAILABLE) || defined(OSG_GLES2_AVAILABLE) || defined(OSG_GLES3_AVAILABLE)
#define OE_GLES_AVAILABLE
#endif

// Comment this out to test the non-GLSL path
#define USE_GPU 1

using namespace osgEarth;

#define LC "[PointGroup] "

namespace osgEarth { namespace Serializers { namespace PointGroup
{
    REGISTER_OBJECT_WRAPPER(
        PointGroup,
        new osgEarth::PointGroup,
        osgEarth::PointGroup,
        "osg::Object osg::Node osg::Group osg::Geode osgEarth::PointGroup")
    {
        // no properties
    }
} } }

PointGroup::PointGroup()
{
    //nop
}

PointGroup::PointGroup(const PointGroup& rhs, const osg::CopyOp& copy) :
osg::Geode(rhs, copy)
{
    //nop
}

PointGroup::~PointGroup()
{
    //nop
}

namespace
{
    struct ImportPointsOperator
    {
        osg::Vec3Array* _verts;
        osg::Vec4Array* _colors;
        PointGroup* _group;
        PointDrawable* _drawable;

        virtual void setGeometry(osg::Geometry* geom)
        {
            _verts = dynamic_cast<osg::Vec3Array*>(geom->getVertexArray());
            _colors = dynamic_cast<osg::Vec4Array*>(geom->getColorArray());
        }

        void point(unsigned i1)
        {
            if (_verts)
            {
                _drawable->pushVertex((*_verts)[i1]);
                if (_colors && _colors->size() == _verts->size())
                {
                    _drawable->setColor(_drawable->getNumVerts()-1, (*_colors)[_colors->size()-1]);
                }
            }
        }
    };

    typedef PointIndexFunctor<ImportPointsOperator> ImportPointsFunctorBase;

    struct ImportPointsFunctor : public ImportPointsFunctorBase
    {
        osg::Point* _size;

        ImportPointsFunctor(osg::Geometry* input, PointGroup* group, osg::Point* size)
        {
            setGeometry(input);
            _group = group;
            _drawable = 0L;
            _size = size;
        }

        void emit()
        {
            if (_colors && _colors->getBinding() == osg::Array::BIND_OVERALL && _colors->size() > 0)
            {
                _drawable->setColor((*_colors)[0]);
            }

            _drawable->dirty();

            if (_size)
            {
                _drawable->setPointSize(_size->getSize());
            }

            _group->addChild(_drawable);
        }

        virtual void drawArrays(GLenum mode,GLint first,GLsizei count)
        {
            if (mode == GL_POINTS)
            {
                _drawable = new PointDrawable();
                ImportPointsFunctorBase::drawArrays(mode, first, count);
                emit();
            }
        }

        virtual void drawElements(GLenum mode,GLsizei count,const GLubyte* indices)
        {
            if (mode == GL_POINTS)
            {
                _drawable = new PointDrawable();
                ImportPointsFunctorBase::drawElements(mode, count, indices);
                emit();
            }
        }

        virtual void drawElements(GLenum mode,GLsizei count,const GLushort* indices)
        {
            if (mode == GL_POINTS)
            {
                _drawable = new PointDrawable();
                ImportPointsFunctorBase::drawElements(mode, count, indices);
                emit();
            }
        }

        virtual void drawElements(GLenum mode,GLsizei count,const GLuint* indices)
        {
            if (mode == GL_POINTS)
            {
                _drawable = new PointDrawable();
                ImportPointsFunctorBase::drawElements(mode, count, indices);
                emit();
            }
        }
    };

    

    struct ImportPointsVisitor : public osg::NodeVisitor
    {
        PointGroup* _group;
        bool _removePrimSets;
        typedef std::pair<osg::Node*, osg::Point*> Size;
        std::stack<Size> _size;

        ImportPointsVisitor(PointGroup* group, bool removePrimSets) : _group(group), _removePrimSets(removePrimSets)
        {
            setTraversalMode(TRAVERSE_ALL_CHILDREN);
            setNodeMaskOverride(~0);
        }

        void apply(osg::Node& node)
        {
            pushState(node);
            traverse(node);
            popState(node);
        }

        void apply(osg::Drawable& drawable)
        {
            pushState(drawable);
            osg::Geometry* geom = drawable.asGeometry();
            if (geom)
            {
                ImportPointsFunctor import(
                    geom,
                    _group,
                    _size.empty() ? 0L : _size.top().second);

                drawable.accept(import);

                if (_removePrimSets)
                {
                    for (int i = 0; i < (int)geom->getNumPrimitiveSets(); ++i)
                    {
                        GLenum mode = geom->getPrimitiveSet(i)->getMode();
                        if (mode == GL_POINTS)
                        {
                            geom->removePrimitiveSet(i--);
                        }
                    }
                }
            }
            popState(drawable);
        }

        void pushState(osg::Node& node)
        {
            osg::StateSet* ss = node.getStateSet();
            if (ss)
            {
                osg::Point* size = dynamic_cast<osg::Point*>(ss->getAttribute(osg::StateAttribute::POINT));
                if (size)
                {
                    _size.push(std::make_pair(&node, size));
                }
            }
        }

        void popState(osg::Node& node)
        {
            if (!_size.empty() && _size.top().first == &node)
                _size.pop();
        }
    };
}

void
PointGroup::import(osg::Node* node, bool removePrimitiveSets)
{
    if (node)
    {
        ImportPointsVisitor visitor(this, removePrimitiveSets);
        node->accept(visitor);
    }
}

void
PointGroup::optimize()
{
    // Optimize state sharing so the MergeGeometryVisitor can work better.
    // Without this step, the #defines used for width and stippling will
    // hold up the merge.
    osg::ref_ptr<StateSetCache> cache = new StateSetCache();
    cache->optimize(this);

    // Merge all non-dynamic drawables to reduce the total number of 
    // OpenGL calls.
    osgUtil::Optimizer::MergeGeometryVisitor mg;
    mg.setTargetMaximumNumberOfVertices(Registry::instance()->getMaxNumberOfVertsPerDrawable());
    accept(mg);
}

PointDrawable*
PointGroup::getPointDrawable(unsigned i)
{
    return i < getNumChildren() ? dynamic_cast<PointDrawable*>(getChild(i)) : 0L;
}

//...................................................................

#undef  LC
#define LC "[PointDrawable] "


namespace osgEarth { namespace Serializers { namespace PointDrawable
{
    REGISTER_OBJECT_WRAPPER(
        PointDrawable,
        new osgEarth::PointDrawable,
        osgEarth::PointDrawable,
        "osg::Object osg::Node osg::Drawable osg::Geometry osgEarth::PointDrawable")
    {
        ADD_VEC4_SERIALIZER( Color, osg::Vec4(1,1,1,1) );
        ADD_FLOAT_SERIALIZER( PointSize, 1.0f );
        ADD_UINT_SERIALIZER( First, 0u );
        ADD_UINT_SERIALIZER( Count, 0u );
    }
} } }

//...................................................................

namespace
{
    static bool s_isCoreProfile;
}

PointDrawable::PointDrawable() :
osg::Geometry(),
_gpu(false),
_color(1, 1, 1, 1),
_width(1.0f),
_smooth(false),
_first(0u),
_count(0u),
_current(NULL),
_colors(NULL),
_sharedStateSetCompiled(false)
{
#ifdef USE_GPU
    _gpu = Registry::capabilities().supportsGLSL();
#endif
    setupState();
}

PointDrawable::PointDrawable(const PointDrawable& rhs, const osg::CopyOp& copy) :
osg::Geometry(rhs, copy),
_gpu(rhs._gpu),
_color(rhs._color),
_width(rhs._width),
_smooth(rhs._smooth),
_first(rhs._first),
_count(rhs._count),
_current(NULL),
_colors(NULL),
_sharedStateSetCompiled(rhs._sharedStateSetCompiled)
{
    _current = static_cast<osg::Vec3Array*>(getVertexArray());
    setupState();
}

PointDrawable::~PointDrawable()
{
    //nop
}

void
PointDrawable::initialize()
{
    // Already initialized?
    if (_current)
        return;

    // See if the arrays already exist:
    _current = static_cast<osg::Vec3Array*>(getVertexArray());

    setUseVertexBufferObjects(_supportsVertexBufferObjects);
    setUseDisplayList(false);

    if (!_current)
    {
        _current = new osg::Vec3Array();
        _current->setBinding(osg::Array::BIND_PER_VERTEX);
        setVertexArray(_current);

        _colors = new osg::Vec4Array();
        _colors->setBinding(osg::Array::BIND_PER_VERTEX);
        setColorArray(_colors);
    }
}

void
PointDrawable::setPointSize(float value)
{
    if (_width != value)
    {
        _width = value;
        GLUtils::setPointSize(getOrCreateStateSet(), value, 1);
    }
}

void
PointDrawable::setPointSmooth(bool value)
{
    if (_smooth != value)
    {
        _smooth = value;
        GLUtils::setPointSmooth(getOrCreateStateSet(), value? 1 : 0);
    }
}

void
PointDrawable::setColor(const osg::Vec4& color)
{
    if (_color != color)
    {
        initialize();

        _color = color;
        if (_colors && !_colors->empty())
        {
            _colors->assign(_colors->size(), _color);
            _colors->dirty();
        }
    }
}

void
PointDrawable::setColor(unsigned vi, const osg::Vec4& color)
{
    (*_colors)[vi] = color;
    _colors->dirty();
}

void
PointDrawable::setFirst(unsigned value)
{
    _first = value;
    updateFirstCount();
}

unsigned
PointDrawable::getFirst() const
{
    return _first;
}

void
PointDrawable::setCount(unsigned value)
{
    _count = value;
    updateFirstCount();
}

unsigned
PointDrawable::getCount() const
{
    return _count;
}

void
PointDrawable::updateFirstCount()
{
    if (getNumPrimitiveSets() > 0)
    {
        osg::DrawArrays* da = dynamic_cast<osg::DrawArrays*>(getPrimitiveSet(0));
        if (da)
        {
            unsigned first = _first < _current->size() ? _first : 0u;
            unsigned count = _count <= _current->size()-first ? _count : _current->size()-first;
            da->setFirst(first);
            da->setCount(count);
            da->dirty();
        }
    }
}

void
PointDrawable::pushVertex(const osg::Vec3& vert)
{
    initialize();

    _current->push_back(vert);
    _current->dirty();

    _colors->push_back(_color);
    _colors->dirty();

    dirtyBound();
}

void
PointDrawable::insert(unsigned where, const osg::Vec3& vert)
{
  initialize();

  _current->insert(_current->begin() + where, vert);
  _current->dirty();

  _colors->insert(_colors->begin() + where, _color);
  _colors->dirty();

  dirtyBound();
}

void
PointDrawable::setVertex(unsigned vi, const osg::Vec3& vert)
{
    initialize();

    // if we've already called dirty() that means we are editing a completed
    // drawable and therefore need dynamic variance.
    if (getNumPrimitiveSets() > 0u && getDataVariance() != DYNAMIC)
    {
        setDataVariance(DYNAMIC);
    }

    if (vi < _current->size())
    {
        (*_current)[vi] = vert;
        _current->dirty();
    }

    dirtyBound();
}

const osg::Vec3&
PointDrawable::getVertex(unsigned index) const
{
    return (*_current)[index];
}

void
PointDrawable::importVertexArray(const osg::Vec3Array* verts)
{
    initialize();

    _current->clear();
    _colors->clear();

    if (verts && verts->size() > 0)
    {
        reserve(verts->size());

        for (osg::Vec3Array::const_iterator i = verts->begin(); i != verts->end(); ++i)
        {
            pushVertex(*i);
        }
    }

    dirty();
}

void
PointDrawable::allocate(unsigned numVerts)
{
    initialize();

    unsigned num = getNumVerts();
    if (numVerts >= num)
    {
        for (unsigned i = num; i < numVerts; ++i)
        {
            pushVertex(osg::Vec3(0,0,0));
        }
    }
    else
    {
        clear();
        for (unsigned i = 0; i < numVerts; ++i)
        {
            pushVertex(osg::Vec3(0,0,0));
        }
    }

    dirty();
}

// Calculates the "virtual" number of vertices in this drawable.
// The actual number of vertices depends on the GL mode.
unsigned
PointDrawable::getNumVerts() const
{
    if (!_current || _current->empty())
        return 0u;

    return _current->size();
}

void
PointDrawable::reserve(unsigned size)
{
    initialize();

    unsigned actualSize = size;

    if (actualSize > _current->size())
    {
        ArrayList arrays;
        getArrayList(arrays);
        for (ArrayList::iterator i = arrays.begin(); i != arrays.end(); ++i)
            i->get()->reserveArray(actualSize);
    }
}

void
PointDrawable::clear()
{
    initialize();

    unsigned n = getNumVerts();
    if (n > 0u)
    {
        ArrayList arrays;
        getArrayList(arrays);
        for (ArrayList::iterator i = arrays.begin(); i != arrays.end(); ++i)
        {
            i->get()->resizeArray(0);
        }
        reserve(n);
    }
}

void
PointDrawable::dirty()
{
    initialize();

    dirtyBound();

    _current->dirty();

    // rebuild primitive sets.
    if (getNumPrimitiveSets() > 0)
    {
        removePrimitiveSet(0, 1);
    }

    ArrayList arrays;
    getArrayList(arrays);
    for (unsigned i = 0; i<arrays.size(); ++i)
        arrays[i]->dirty();

    addPrimitiveSet(new osg::DrawArrays(GL_POINTS, _first, _count > 0u? _count : _current->size()));
}

osg::observer_ptr<osg::StateSet> PointDrawable::s_sharedStateSet;

void
PointDrawable::setupState()
{
    // Create the singleton state set for the shader. This stateset will be
    // shared by all PointDrawable instances so OSG will sort them together.
    if (!_sharedStateSet.valid())
    {
        if (s_sharedStateSet.lock(_sharedStateSet) == false)
        {
            static Threading::Mutex s_mutex;
            Threading::ScopedMutexLock lock(s_mutex);

            if (s_sharedStateSet.lock(_sharedStateSet) == false)
            {
                s_sharedStateSet = _sharedStateSet = new osg::StateSet();

                s_sharedStateSet->setTextureAttributeAndModes(0, new osg::PointSprite(), osg::StateAttribute::ON);

                if (_gpu)
                {
                    VirtualProgram* vp = VirtualProgram::getOrCreate(_sharedStateSet.get());
                    vp->setName("osgEarth::PointDrawable");
                    Shaders shaders;
                    shaders.load(vp, shaders.PointDrawable);
                    _sharedStateSet->setMode(GL_PROGRAM_POINT_SIZE, 1);
                }

                s_isCoreProfile = Registry::capabilities().isCoreProfile();
            }
        }
    }
}

void
PointDrawable::drawImplementation(osg::RenderInfo& ri) const
{
    checkSharedStateSet(ri.getState());

    // in the compatibility profile, we have to expressly enable point sprites;
    // not sure why this doesn't happen in osg::PointSprite
    if (!s_isCoreProfile)
    {
        glEnable(GL_POINT_SPRITE_ARB);
    }

    osg::Geometry::drawImplementation(ri);
}

void
PointDrawable::checkSharedStateSet(osg::State* state) const
{
    if (_sharedStateSet.valid() && !_sharedStateSetCompiled)
    {
        static Threading::Mutex s_mutex;
        Threading::ScopedMutexLock lock(s_mutex);

        if (!_sharedStateSetCompiled)
        {
            osg::PointSprite* sprite = dynamic_cast<osg::PointSprite*>(
                _sharedStateSet->getTextureAttribute(0, osg::StateAttribute::POINTSPRITE));

            if (sprite)
            {
                sprite->checkValidityOfAssociatedModes(*state);
            }

            _sharedStateSet->compileGLObjects(*state);
            _sharedStateSetCompiled = true;
        }
    }
}

void
PointDrawable::accept(osg::NodeVisitor& nv)
{
    if (nv.validNodeMask(*this))
    { 
        osgUtil::CullVisitor* cv = Culling::asCullVisitor(nv);

        nv.pushOntoNodePath(this);

        // inject our shared stateset into the cull visitor.
        if (cv)
            cv->pushStateSet(_sharedStateSet.get());

        nv.apply(*this); 

        if (cv)
            cv->popStateSet();

        nv.popFromNodePath();
    }
}

void
PointDrawable::compileGLObjects(osg::RenderInfo& ri) const
{
    checkSharedStateSet(ri.getState());
    osg::Geometry::compileGLObjects(ri);
}

void
PointDrawable::resizeGLObjectBuffers(unsigned maxSize)
{
    osg::Geometry::resizeGLObjectBuffers(maxSize);
    if (_sharedStateSet.valid())
        _sharedStateSet->resizeGLObjectBuffers(maxSize);
}

void
PointDrawable::releaseGLObjects(osg::State* state) const
{
    osg::Geometry::releaseGLObjects(state);
    if (_sharedStateSet.valid())
        _sharedStateSet->releaseGLObjects(state);
}
