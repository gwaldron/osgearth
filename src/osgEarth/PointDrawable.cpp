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
#include <osgDB/ObjectWrapper>
#include <osg/PointSprite>


#if defined(OSG_GLES1_AVAILABLE) || defined(OSG_GLES2_AVAILABLE) || defined(OSG_GLES3_AVAILABLE)
#define OE_GLES_AVAILABLE
#endif

#define USE_GPU 1

using namespace osgEarth;

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

PointDrawable::PointDrawable() :
osg::Geometry(),
_gpu(false),
_color(1, 1, 1, 1),
_width(1.0f),
_first(0u),
_count(0u),
_current(NULL),
_colors(NULL)
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
_first(rhs._first),
_count(rhs._count),
_current(NULL),
_colors(NULL)
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
            unsigned first = _first >= 0u ? _first : 0u;
            unsigned count = _count > 0u? _count : _current->size()-first;
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

osg::ref_ptr<osg::StateSet> PointDrawable::_sharedStateSet;

void
PointDrawable::setupState()
{
    // Create the singleton state set for the line shader. This stateset will be
    // shared by all LineDrawable instances so OSG will sort them together.
    if (!_sharedStateSet.valid())
    {
        static Threading::Mutex s_mutex;
        s_mutex.lock();
        if (!_sharedStateSet.valid())
        {
            _sharedStateSet = new osg::StateSet();

            _sharedStateSet->setTextureAttributeAndModes(0, new osg::PointSprite(), osg::StateAttribute::ON);

            if (_gpu)
            {
                VirtualProgram* vp = VirtualProgram::getOrCreate(_sharedStateSet.get());
                Shaders shaders;
                shaders.load(vp, shaders.PointDrawable);
                _sharedStateSet->getOrCreateUniform("oe_PointDrawable_limits", osg::Uniform::FLOAT_VEC2)->set(osg::Vec2f(-1,-1));
                _sharedStateSet->setMode(GL_VERTEX_PROGRAM_POINT_SIZE, 1);
            }
            else
            {
                //todo
            }
        }
        s_mutex.unlock();
    }
}

void
PointDrawable::accept(osg::NodeVisitor& nv)
{
    if (nv.validNodeMask(*this))
    { 
        osgUtil::CullVisitor* cv = Culling::asCullVisitor(nv);

        nv.pushOntoNodePath(this);

        if (cv)
            cv->pushStateSet(_sharedStateSet.get());

        nv.apply(*this); 

        if (cv)
            cv->popStateSet();

        nv.popFromNodePath();
    }
}
