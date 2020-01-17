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
#include <osgEarth/LineDrawable>
#include <osgEarth/Shaders>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/LineFunctor>
#include <osgEarth/GLUtils>
#include <osgEarth/CullingUtils>

#include <osg/LineStipple>
#include <osg/LineWidth>
#include <osgUtil/Optimizer>

#include <osgDB/ObjectWrapper>


#if defined(OSG_GLES1_AVAILABLE) || defined(OSG_GLES2_AVAILABLE) || defined(OSG_GLES3_AVAILABLE)
#define OE_GLES_AVAILABLE
#endif

using namespace osgEarth;


// References:
// https://mattdesl.svbtle.com/drawing-lines-is-hard
// https://github.com/mattdesl/webgl-lines


#define LC "[LineGroup] "

// Comment this out to test the non-GLSL path
#define USE_GPU

namespace osgEarth { namespace Serializers { namespace LineGroup
{
    REGISTER_OBJECT_WRAPPER(
        LineGroup,
        new osgEarth::LineGroup,
        osgEarth::LineGroup,
        "osg::Object osg::Node osg::Group osg::Geode osgEarth::LineGroup")
    {
        // no properties
    }
} } }

LineGroup::LineGroup()
{
    //nop
}

LineGroup::LineGroup(const LineGroup& rhs, const osg::CopyOp& copy) :
    osg::Geode(rhs, copy)
{
    //nop
}

LineGroup::~LineGroup()
{
    //nop
}

namespace
{
    struct ImportLinesOperator
    {
        osg::Vec3Array* _verts;
        osg::Vec4Array* _colors;
        LineGroup* _group;
        LineDrawable* _drawable;

        virtual void setGeometry(osg::Geometry* geom)
        {
            _verts = dynamic_cast<osg::Vec3Array*>(geom->getVertexArray());
            _colors = dynamic_cast<osg::Vec4Array*>(geom->getColorArray());
        }

        void line(unsigned i1, unsigned i2)
        {
            if (_verts)
            {
                _drawable->pushVertex((*_verts)[i1]);
                _drawable->pushVertex((*_verts)[i2]);
                if (_colors && _colors->size() == _verts->size())
                {
                    _drawable->setColor(_drawable->getNumVerts()-2, (*_colors)[_colors->size()-2]);
                    _drawable->setColor(_drawable->getNumVerts()-1, (*_colors)[_colors->size()-1]);
                }
            }
        }
    };

    typedef LineIndexFunctor<ImportLinesOperator> ImportLinesFunctorBase;

    struct ImportLinesFunctor : public ImportLinesFunctorBase
    {
        osg::LineWidth* _width;
        osg::LineStipple* _stipple;

        ImportLinesFunctor(osg::Geometry* input, LineGroup* group, osg::LineWidth* width, osg::LineStipple* stipple)
        {
            setGeometry(input);
            _group = group;
            _drawable = 0L;
            _width = width;
            _stipple = stipple;
        }

        void emit()
        {
            if (_colors && _colors->getBinding() == osg::Array::BIND_OVERALL && _colors->size() > 0)
            {
                _drawable->setColor((*_colors)[0]);
            }

            _drawable->dirty();

            if (_width)
            {
                _drawable->setLineWidth(_width->getWidth());
            }

            if (_stipple)
            {
                _drawable->setStipplePattern(_stipple->getPattern());
                _drawable->setStippleFactor(_stipple->getFactor());
            }

            _group->addChild(_drawable);
        }

        virtual void drawArrays(GLenum mode,GLint first,GLsizei count)
        {
            if (mode == GL_LINES || mode == GL_LINE_STRIP || mode == GL_LINE_LOOP)
            {
                _drawable = new LineDrawable(GL_LINES);
                ImportLinesFunctorBase::drawArrays(mode, first, count);
                emit();
            }
        }

        virtual void drawElements(GLenum mode,GLsizei count,const GLubyte* indices)
        {
            if (mode == GL_LINES || mode == GL_LINE_STRIP || mode == GL_LINE_LOOP)
            {
                _drawable = new LineDrawable(GL_LINES);
                ImportLinesFunctorBase::drawElements(mode, count, indices);
                emit();
            }
        }

        virtual void drawElements(GLenum mode,GLsizei count,const GLushort* indices)
        {
            if (mode == GL_LINES || mode == GL_LINE_STRIP || mode == GL_LINE_LOOP)
            {
                _drawable = new LineDrawable(GL_LINES);
                ImportLinesFunctorBase::drawElements(mode, count, indices);
                emit();
            }
        }

        virtual void drawElements(GLenum mode,GLsizei count,const GLuint* indices)
        {
            if (mode == GL_LINES || mode == GL_LINE_STRIP || mode == GL_LINE_LOOP)
            {
                _drawable = new LineDrawable(GL_LINES);
                ImportLinesFunctorBase::drawElements(mode, count, indices);
                emit();
            }
        }
    };



    struct ImportLinesVisitor : public osg::NodeVisitor
    {
        LineGroup* _group;
        bool _removePrimSets;
        typedef std::pair<osg::Node*, osg::LineWidth*> Width;
        typedef std::pair<osg::Node*, osg::LineStipple*> Stipple;
        std::stack<Width> _width;
        std::stack<Stipple> _stipple;

        ImportLinesVisitor(LineGroup* group, bool removePrimSets) : _group(group), _removePrimSets(removePrimSets)
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
                ImportLinesFunctor import(
                    geom,
                    _group,
                    _width.empty() ? 0L : _width.top().second,
                    _stipple.empty() ? 0L : _stipple.top().second);

                drawable.accept(import);

                if (_removePrimSets)
                {
                    for (int i = 0; i < (int)geom->getNumPrimitiveSets(); ++i)
                    {
                        GLenum mode = geom->getPrimitiveSet(i)->getMode();
                        if (mode == GL_LINES || mode == GL_LINE_STRIP || mode == GL_LINE_LOOP)
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
                osg::LineWidth* width = dynamic_cast<osg::LineWidth*>(ss->getAttribute(osg::StateAttribute::LINEWIDTH));
                if (width)
                {
                    _width.push(std::make_pair(&node, width));
                }
                osg::LineStipple* stipple = dynamic_cast<osg::LineStipple*>(ss->getAttribute(osg::StateAttribute::LINESTIPPLE));
                if (stipple)
                {
                    _stipple.push(std::make_pair(&node, stipple));
                }
            }
        }

        void popState(osg::Node& node)
        {
            if (!_width.empty() && _width.top().first == &node)
                _width.pop();
            if (!_stipple.empty() && _stipple.top().first == &node)
                _stipple.pop();
        }
    };
}

void
LineGroup::import(osg::Node* node, bool removePrimitiveSets)
{
    if (node)
    {
        ImportLinesVisitor visitor(this, removePrimitiveSets);
        node->accept(visitor);
    }
}

void
LineGroup::optimize()
{
    // Optimize state sharing so the MergeGeometryVisitor can work better.
    // Without this step, the #defines used for width and stippling will
    // hold up the merge.
    osg::ref_ptr<StateSetCache> cache = new StateSetCache();
    cache->optimize(this);

    // Merge all non-dynamic drawables to reduce the total number of 
    // OpenGL calls.
    osgUtil::Optimizer::MergeGeometryVisitor mg;
    mg.setTargetMaximumNumberOfVertices(65536);
    accept(mg);
}

LineDrawable*
LineGroup::getLineDrawable(unsigned i)
{
    return i < getNumChildren() ? dynamic_cast<LineDrawable*>(getChild(i)) : 0L;
}

//...................................................................

#undef  LC
#define LC "[LineDrawable] "


namespace osgEarth { namespace Serializers { namespace LineDrawable
{
    REGISTER_OBJECT_WRAPPER(
        LineDrawable,
        new osgEarth::LineDrawable,
        osgEarth::LineDrawable,
        "osg::Object osg::Node osg::Drawable osg::Geometry osgEarth::LineDrawable")
    {
        ADD_UINT_SERIALIZER( Mode, GL_LINE_STRIP );
        ADD_INT_SERIALIZER( StippleFactor, 1 );
        ADD_USHORT_SERIALIZER( StipplePattern, 0xFFFF );
        ADD_VEC4_SERIALIZER( Color, osg::Vec4(1,1,1,1) );
        ADD_FLOAT_SERIALIZER( LineWidth, 1.0f );
        ADD_UINT_SERIALIZER( First, 0u );
        ADD_UINT_SERIALIZER( Count, 0u );
    }
} } }

// static attribute binding locations. Changable by the user.
int LineDrawable::PreviousVertexAttrLocation = 9;
int LineDrawable::NextVertexAttrLocation = 10;

LineDrawable::LineDrawable() :
    osg::Geometry(),
    _mode(GL_LINE_STRIP),
    _gpu(false),
    _factor(1),
    _pattern(0xFFFF),
    _color(1, 1, 1, 1),
    _width(1.0f),
    _smooth(false),
    _first(0u),
    _count(0u),
    _current(NULL),
    _previous(NULL),
    _next(NULL),
    _colors(NULL)
{
#ifdef USE_GPU
    _gpu = Registry::capabilities().supportsGLSL();
    setupShaders();
#endif
}

LineDrawable::LineDrawable(GLenum mode) :
    osg::Geometry(),
    _mode(mode),
    _gpu(false),
    _factor(1),
    _pattern(0xFFFF),
    _color(1,1,1,1),
    _width(1.0f),
    _smooth(false),
    _first(0u),
    _count(0u),
    _current(NULL),
    _previous(NULL),
    _next(NULL),
    _colors(NULL)
{
#ifdef USE_GPU
    _gpu = 
        Registry::capabilities().supportsGLSL() &&
        (_mode == GL_LINES || _mode == GL_LINE_STRIP || _mode == GL_LINE_LOOP);

    setupShaders();
#endif
}

LineDrawable::LineDrawable(const LineDrawable& rhs, const osg::CopyOp& copy) :
    osg::Geometry(rhs, copy),
    _mode(rhs._mode),
    _gpu(rhs._gpu),
    _color(rhs._color),
    _factor(rhs._factor),
    _pattern(rhs._pattern),
    _width(rhs._width),
    _smooth(rhs._smooth),
    _first(rhs._first),
    _count(rhs._count),
    _current(NULL),
    _previous(NULL),
    _next(NULL),
    _colors(NULL)
{
    _current = static_cast<osg::Vec3Array*>(getVertexArray());

    if (_gpu)
    {
        _previous = static_cast<osg::Vec3Array*>(getVertexAttribArray(PreviousVertexAttrLocation));
        _next = static_cast<osg::Vec3Array*>(getVertexAttribArray(NextVertexAttrLocation));
    }

    setupShaders();
}

LineDrawable::~LineDrawable()
{
    //nop
}

void
LineDrawable::initialize()
{
    // Already initialized?
    if (_current)
        return;

    // See if the arrays already exist:
    _current = static_cast<osg::Vec3Array*>(getVertexArray());
    if (_gpu)
    {
        _previous = static_cast<osg::Vec3Array*>(getVertexAttribArray(PreviousVertexAttrLocation));
        _next = static_cast<osg::Vec3Array*>(getVertexAttribArray(NextVertexAttrLocation));
    }

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

        if (_gpu)
        {
            _previous = new osg::Vec3Array();
            _previous->setBinding(osg::Array::BIND_PER_VERTEX);
            _previous->setNormalize(false);
            setVertexAttribArray(PreviousVertexAttrLocation, _previous);  

            _next = new osg::Vec3Array();
            _next->setBinding(osg::Array::BIND_PER_VERTEX);
            _next->setNormalize(false);
            setVertexAttribArray(NextVertexAttrLocation, _next);
        }
    }
}

void
LineDrawable::setMode(GLenum mode)
{
    if (_mode != mode)
    {
        _mode = mode;
    }
}

void
LineDrawable::setLineWidth(float value)
{
    if (_width != value)
    {
        _width = value;
        GLUtils::setLineWidth(getOrCreateStateSet(), value, 1);
    }
}

void
LineDrawable::setLineWidth(osg::StateSet* stateSet, float value, int overrideFlags)
{
    GLUtils::setLineWidth(stateSet, value, overrideFlags);
}

void
LineDrawable::setStipplePattern(GLushort pattern)
{
    //if (_pattern != pattern)
    {
        _pattern = pattern;
        GLUtils::setLineStipple(getOrCreateStateSet(), _factor, _pattern, 1);
    }
}

void
LineDrawable::setStippleFactor(GLint factor)
{
    if (_factor != factor)
    {
        _factor = factor;
        GLUtils::setLineStipple(getOrCreateStateSet(), _factor, _pattern, 1);
    }
}

void
LineDrawable::setLineSmooth(bool value)
{
    if (_smooth != value)
    {
        _smooth = value;
        GLUtils::setLineSmooth(getOrCreateStateSet(), _smooth? 1 : 0);
    }
}

void
LineDrawable::setColor(const osg::Vec4& color)
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
LineDrawable::setColor(unsigned vi, const osg::Vec4& color)
{
    // test against overall color:
    if (color == _color)
        return;

    bool dirty = false;
    if (_gpu)
    {
        if (_mode == GL_LINE_STRIP || _mode == GL_LINE_LOOP)
        {
            if (vi == 0)
            {
                for (unsigned i=0; i<2; ++i)
                {
                    if ((*_colors)[i] != color)
                    {
                        (*_colors)[i] = color;
                        dirty = true;
                    }
                }
                for (unsigned i=_colors->size()-2; i<_colors->size(); ++i)
                {
                    if ((*_colors)[i] != color)
                    {
                        (*_colors)[i] = color;
                        dirty = true;
                    }
                }
            }
            else
            {
                for (unsigned i=vi*4-2; i<vi*4+2; ++i)
                {
                    if ((*_colors)[i] != color)
                    {
                        (*_colors)[i] = color;
                        dirty = true;
                    }
                }
            }
        }
        else // GL_LINES
        {
            if ((*_colors)[vi*2u] != color)
            {
                (*_colors)[vi*2u] = color;
                dirty = true;
            }
            if ((*_colors)[vi*2u+1] != color)
            {
                (*_colors)[vi*2u+1] = color;
                dirty = true;
            }
        }
    }
    else
    {
        if ((*_colors)[vi] != color)
        {
            (*_colors)[vi] = color;
            dirty = true;
        }
    }

    if (dirty)
    {
        _colors->dirty();

        // reset "overall" color
        _color.set(-1,-1,-1,-1);
    }
}

void
LineDrawable::setFirst(unsigned value)
{
    _first = value;
    updateFirstCount();
}

unsigned
LineDrawable::getFirst() const
{
    return _first;
}

void
LineDrawable::setCount(unsigned value)
{
    _count = value;
    updateFirstCount();
}

unsigned
LineDrawable::getCount() const
{
    return _count;
}

void
LineDrawable::updateFirstCount()
{
    if (_gpu)
    {
        osg::StateSet* ss = getOrCreateStateSet();
        ss->setDataVariance(ss->DYNAMIC);

        osg::Uniform* u = ss->getUniform("oe_LineDrawable_limits");
        if (!u)
        {
            u = new osg::Uniform(osg::Uniform::FLOAT_VEC2, "oe_LineDrawable_limits");
            ss->addUniform(u, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
        }

        if (_mode == GL_LINE_STRIP)
        {
            u->set(osg::Vec2(4u*_first + 2u, 4u*(_first+_count-1u)+1u));
        }
        else if (_mode == GL_LINES)
        {
            u->set(osg::Vec2(2u * _first, _count > 1u ? 2u * (_first + _count) - 1u : 0u));
        }
    }
    else
    {
        if (getNumPrimitiveSets() > 0)
        {
            osg::DrawArrays* da = dynamic_cast<osg::DrawArrays*>(getPrimitiveSet(0));
            if (da)
            {
                da->setFirst(_first);
                da->dirty();
            }
        }
    }
}

void
LineDrawable::pushVertex(const osg::Vec3& vert)
{
    initialize();

    if (_gpu)
    {
        if (_mode == GL_LINE_STRIP)
        {
            bool first = _current->empty();

            _previous->push_back(first ? vert : _current->back());
            _previous->push_back(first ? vert : _current->back());
            _previous->push_back(first ? vert : _current->back());
            _previous->push_back(first ? vert : _current->back());

            if (!first)
            {
                *(_next->end() - 4) = vert;
                *(_next->end() - 3) = vert;
                *(_next->end() - 2) = vert;
                *(_next->end() - 1) = vert;
            }

            _current->push_back(vert);
            _current->push_back(vert);
            _current->push_back(vert);
            _current->push_back(vert);

            _next->push_back(vert);
            _next->push_back(vert);
            _next->push_back(vert);
            _next->push_back(vert);

            _colors->push_back(_color);
            _colors->push_back(_color);
            _colors->push_back(_color);
            _colors->push_back(_color);
        }

        else if (_mode == GL_LINE_LOOP)
        {
            bool first = _current->empty();

            _previous->push_back(first ? vert : _current->back());
            _previous->push_back(first ? vert : _current->back());
            _previous->push_back(first ? vert : _current->back());
            _previous->push_back(first ? vert : _current->back());

            if (!first)
            {
                *(_next->end() - 4) = vert;
                *(_next->end() - 3) = vert;
                *(_next->end() - 2) = vert;
                *(_next->end() - 1) = vert;

                (*_previous)[0] = vert;
                (*_previous)[1] = vert;
                (*_previous)[2] = vert;
                (*_previous)[3] = vert;
            }

            _current->push_back(vert);
            _current->push_back(vert);
            _current->push_back(vert);
            _current->push_back(vert);

            _next->push_back(_current->front());
            _next->push_back(_current->front());
            _next->push_back(_current->front());
            _next->push_back(_current->front());

            _colors->push_back(_color);
            _colors->push_back(_color);
            _colors->push_back(_color);
            _colors->push_back(_color);
        }

        else if (_mode == GL_LINES)
        {
            bool first = ((_current->size() / 2) & 0x01) == 0;

            if (first)
            {
                _previous->push_back(vert);
                _previous->push_back(vert);

                _next->push_back(vert);
                _next->push_back(vert);
            }
            else
            {
                _previous->push_back(_current->back());
                _previous->push_back(_current->back());

                *(_next->end() - 2) = vert;
                *(_next->end() - 1) = vert;

                _next->push_back(vert);
                _next->push_back(vert);
            }

            _current->push_back(vert);
            _current->push_back(vert);

            _colors->push_back(_color);
            _colors->push_back(_color);
        }

        _previous->dirty();
        _next->dirty();
    }

    else
    {
        _current->push_back(vert);
        _colors->push_back(_color);
    }

    _current->dirty();
    _colors->dirty();
}

void
LineDrawable::setVertex(unsigned vi, const osg::Vec3& vert)
{
    initialize();

    // if we've already called dirty() that means we are editing a completed
    // drawable and therefore need dynamic variance.
    if (getNumPrimitiveSets() > 0u && getDataVariance() != DYNAMIC)
    {
        setDataVariance(DYNAMIC);
    }

    unsigned size = _current->size();
    unsigned numVerts = getNumVerts();

    // "vi" = virtual index, "ri" = real index.

    if (vi < numVerts)
    {
        if (_gpu)
        {
            if (_mode == GL_LINE_STRIP)
            {
                unsigned ri = vi*4u;
                unsigned rnum = 4u; // number of real verts to set

                                    // update the main verts:
                for (unsigned n = ri; n < ri+rnum; ++n)
                {
                    (*_current)[n] = vert;
                }
                _current->dirty();

                // update next/previous verts:
                if (numVerts == 1u)
                {
                    for(unsigned i=0; i<4; ++i)
                    {
                        (*_next)[i] = (*_previous)[i] = vert;
                    }
                    _next->dirty();
                    _previous->dirty();
                }
                else 
                {
                    if (vi > 0u)
                    {
                        unsigned rni = ri-4u;
                        for (unsigned n = 0; n < rnum; ++n)
                        {
                            (*_next)[rni + n] = vert;
                        }
                        _next->dirty();
                    }
                    else // if vi == 0
                    {
                        for (unsigned n = 0; n < rnum; ++n)
                        {
                            (*_previous)[n] = vert;
                        }
                        _previous->dirty();
                    }

                    if (vi < numVerts-1)
                    {
                        unsigned rpi = ri+4u;
                        for (unsigned n = 0; n < rnum; ++n)
                        {
                            (*_previous)[rpi + n] = vert;
                        }
                        _previous->dirty();
                    }
                    else // if (vi == numVerts-1)
                    {
                        for (unsigned n = 0; n < rnum; ++n)
                        {
                            (*_next)[ri+n] = vert;
                        }
                        _next->dirty();
                    }
                }
            }

            else if (_mode == GL_LINE_LOOP)
            {
                unsigned ri = vi*4u; // starting real index
                unsigned rnum = 4u; // number of real verts to set

                                    // update the main verts:
                for (unsigned n = ri; n < ri+rnum; ++n)
                {
                    (*_current)[n] = vert;
                }
                _current->dirty();

                // update next/previous verts:
                if (numVerts == 1u)
                {
                    for(unsigned i=0; i<4; ++i)
                    {
                        (*_next)[i] = (*_previous)[i] = vert;
                    }
                }
                else 
                {
                    unsigned rni = vi==0u? numVerts-4u : ri-4u;
                    unsigned rpi = vi==numVerts-1u ? 0u : ri+4u;

                    for(unsigned n=0; n<rnum; ++n)
                    {
                        (*_next)[rni+n] = vert;
                        (*_previous)[rpi+n] = vert;
                    }
                }

                _next->dirty();
                _previous->dirty();
            }

            else if (_mode == GL_LINES)
            {
                unsigned ri = vi * 2u;

                (*_current)[ri] = vert;
                (*_current)[ri + 1] = vert;
                _current->dirty();

                bool first = (vi & 0x01) == 0;
                if (first)
                {
                    (*_previous)[ri] = vert;
                    (*_previous)[ri+1] = vert;
                    (*_previous)[ri+2] = vert;
                    (*_previous)[ri+3] = vert;
                    _previous->dirty();
                }
                else
                {
                    (*_next)[ri+1] = vert;
                    (*_next)[ri] = vert;
                    (*_next)[ri-1] = vert;
                    (*_next)[ri-2] = vert;
                    _next->dirty();
                }
            }
        }

        else
        {
            (*_current)[vi] = vert;
            _current->dirty();
        }

        dirtyBound();
    }
}

const osg::Vec3&
LineDrawable::getVertex(unsigned index) const
{
    return (*_current)[getRealIndex(index)];
}

unsigned
LineDrawable::getRealIndex(unsigned index) const
{
    if (_gpu)
        return (_mode == GL_LINE_STRIP || _mode == GL_LINE_LOOP) ? index*4u : index*2u;
    else
        return index;
}

void
LineDrawable::importVertexArray(const osg::Vec3Array* verts)
{
    initialize();

    _current->clear();
    _colors->clear();
    if (verts && verts->size() > 0)
    {
        if (_gpu)
        {
            _previous->clear();
            _next->clear();
        }

        reserve(verts->size());

        for (osg::Vec3Array::const_iterator i = verts->begin(); i != verts->end(); ++i)
        {
            pushVertex(*i);
        }
    }

    dirty();
}

void
LineDrawable::allocate(unsigned numVerts)
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
LineDrawable::getNumVerts() const
{
    if (!_current || _current->empty())
        return 0u;

    if (_gpu)
    {
        if (_mode == GL_LINE_STRIP || _mode == GL_LINE_LOOP)
            return _current->size()/4; //_current->size() == 2 ? 1 : (_current->size()+2)/4;
        else
            return _current->size()/2;
    }
    else
    {
        return _current->size();
    }
}

unsigned
LineDrawable::actualVertsPerVirtualVert(unsigned index) const
{
    if (_gpu)
        if (_mode == GL_LINE_STRIP || _mode == GL_LINE_LOOP)
            return 4u; //index == 0u? 2u : 4u;
        else 
            return 2u;
    else
        return 1u;
}

unsigned
LineDrawable::numVirtualVerts(const osg::Array* a) const
{
    unsigned n = a->getNumElements();
    if (n == 0u)
        return 0u;

    if (_gpu)
        if (_mode == GL_LINE_STRIP || _mode == GL_LINE_LOOP)
            return n/4u; //n == 2u ? 1u : (n+2u)/4u;
        else
            return n/2u;
    else
        return n;
}

void
LineDrawable::reserve(unsigned size)
{
    initialize();

    unsigned actualSize = size;
    if (_gpu)
    {
        actualSize = (_mode == GL_LINE_STRIP || _mode == GL_LINE_LOOP) ? size*4u : size*2u;
    }

    if (actualSize > _current->size())
    {
        ArrayList arrays;
        getArrayList(arrays);
        for (ArrayList::iterator i = arrays.begin(); i != arrays.end(); ++i)
            i->get()->reserveArray(actualSize);
    }
}

void
LineDrawable::clear()
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

namespace
{
    osg::DrawElements* makeDE(unsigned size)
    {
        osg::DrawElements* de =
#ifndef OE_GLES_AVAILABLE
            size > 0xFFFF ? (osg::DrawElements*)new osg::DrawElementsUInt(GL_TRIANGLES) :
#endif
            size > 0xFF ?   (osg::DrawElements*)new osg::DrawElementsUShort(GL_TRIANGLES) :
            (osg::DrawElements*)new osg::DrawElementsUByte(GL_TRIANGLES);
        de->reserveElements(size);
        return de;
    }
}

void
LineDrawable::dirty()
{
    initialize();

    dirtyBound();

    _current->dirty();

    if (_gpu)
    {
        _previous->dirty();
        _next->dirty();
    }

    // rebuild primitive sets.
    if (getNumPrimitiveSets() > 0)
    {
        removePrimitiveSet(0, 1);
    }

    if (_gpu && _current->size() >= 4)
    {
        // IMPORTANT!
        // Don't change the order of the elements! Because of the way
        // GPU line stippling works, it is critical that the provoking vertex
        // be at the beginning of each line segment. In this case we are using
        // GL_TRIANGLES and thus the provoking vertex (PV) is the FINAL vert
        // in each triangle.

        if (_mode == GL_LINE_STRIP)
        {
            unsigned numEls = (getNumVerts()-1)*6;
            osg::DrawElements* els = makeDE(numEls);  

            for (int e = 2; e < _current->size() - 2; e += 4)
            {
                els->addElement(e+3);
                els->addElement(e+1);
                els->addElement(e+0); // PV
                els->addElement(e+2);
                els->addElement(e+3);
                els->addElement(e+0); // PV
            }

            addPrimitiveSet(els);
        }

        else if (_mode == GL_LINE_LOOP)
        {
            unsigned numEls = getNumVerts()*6;
            osg::DrawElements* els = makeDE(numEls); 

            int e;
            for (e = 2; e < _current->size() - 2; e += 4)
            {
                els->addElement(e+3);
                els->addElement(e+1);
                els->addElement(e+0); // PV
                els->addElement(e+2);
                els->addElement(e+3);
                els->addElement(e+0); // PV
            }

            els->addElement(1);
            els->addElement(e+1);
            els->addElement(e+0); // PV
            els->addElement(0);
            els->addElement(1);
            els->addElement(e+0); // PV

            addPrimitiveSet(els);
        }

        else if (_mode == GL_LINES)
        {
            // if there are an odd number of verts, ignore the last one.
            unsigned numVerts = getNumVerts();
            if (numVerts & 0x01) --numVerts;

            if (numVerts > 0u)
            {
                unsigned numEls = (numVerts/2)*6;
                osg::DrawElements* els = makeDE(numEls);  

                for(unsigned e=0; e<numVerts*2u; e += 4)
                {
                    //for (int e = 0; e < _current->size(); e += 4)
                    els->addElement(e+3);
                    els->addElement(e+1);
                    els->addElement(e+0); // PV
                    els->addElement(e+2);
                    els->addElement(e+3);
                    els->addElement(e+0); // PV
                }

                addPrimitiveSet(els);
            }
        }
    }

    else
    {
        ArrayList arrays;
        getArrayList(arrays);
        for (unsigned i = 0; i<arrays.size(); ++i)
            arrays[i]->dirty();

        addPrimitiveSet(new osg::DrawArrays(_mode, _first, _count > 0u? _count : _current->size()));
    }
}

osg::observer_ptr<osg::StateSet> LineDrawable::s_gpuStateSet;

void
LineDrawable::setupShaders()
{
    // Create the singleton state set for the line shader. This stateset will be
    // shared by all LineDrawable instances so OSG will sort them together.
    if (_gpu && !_gpuStateSet.valid())
    {
        if (s_gpuStateSet.lock(_gpuStateSet) == false)
        {
            // serialize access and double-check:
            static Threading::Mutex s_mutex;
            Threading::ScopedMutexLock lock(s_mutex);

            if (s_gpuStateSet.lock(_gpuStateSet) == false)
            {
                s_gpuStateSet = _gpuStateSet = new osg::StateSet();

                VirtualProgram* vp = VirtualProgram::getOrCreate(s_gpuStateSet.get());
                vp->setName("osgEarth::LineDrawable");
                Shaders shaders;
                shaders.load(vp, shaders.LineDrawable);
                vp->addBindAttribLocation("oe_LineDrawable_prev", LineDrawable::PreviousVertexAttrLocation);
                vp->addBindAttribLocation("oe_LineDrawable_next", LineDrawable::NextVertexAttrLocation);
                s_gpuStateSet->getOrCreateUniform("oe_LineDrawable_limits", osg::Uniform::FLOAT_VEC2)->set(osg::Vec2f(-1, -1));
                s_gpuStateSet->setMode(GL_CULL_FACE, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE | osg::StateAttribute::PROTECTED);
            }
        }
    }
}

void
LineDrawable::accept(osg::NodeVisitor& nv)
{
    if (nv.validNodeMask(*this))
    { 
        // Only push the shader if necessary.
        // The reason for this approach is go we can inject the singleton 
        // LineDrawable shader yet still allow the user to customize 
        // the node's StateSet.
        bool shade =
            _gpu &&
            nv.getVisitorType() == nv.CULL_VISITOR &&
            _gpuStateSet.valid();

        osgUtil::CullVisitor* cv = shade? Culling::asCullVisitor(nv) : 0L;

        nv.pushOntoNodePath(this);

        if (cv)
            cv->pushStateSet(_gpuStateSet.get());

        nv.apply(*this); 

        if (cv)
            cv->popStateSet();

        nv.popFromNodePath();
    }
}


void
LineDrawable::resizeGLObjectBuffers(unsigned maxSize)
{
    osg::Geometry::resizeGLObjectBuffers(maxSize);
    if (_gpuStateSet.valid())
        _gpuStateSet->resizeGLObjectBuffers(maxSize);
}

void
LineDrawable::releaseGLObjects(osg::State* state) const
{
    osg::Geometry::releaseGLObjects(state);
    if (_gpuStateSet.valid())
        _gpuStateSet->releaseGLObjects(state);
}
