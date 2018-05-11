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
#include <osgEarth/LineDrawable>
#include <osgEarth/Shaders>
#include <osgEarth/VirtualProgram>
#include <osgEarth/Utils>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/StateSetCache>
#include <osgEarth/LineFunctor>
#include <osgEarth/GLUtils>

#include <osg/Depth>
#include <osg/CullFace>
#include <osg/LineStipple>
#include <osg/LineWidth>
#include <osg/TemplatePrimitiveFunctor>
#include <osgUtil/Optimizer>

#include <osgDB/ObjectWrapper>
#include <osgDB/InputStream>
#include <osgDB/OutputStream>

#include <stack>

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
#ifdef USE_GPU
    if (Registry::capabilities().supportsGLSL())
    {
        LineDrawable::installShader(getOrCreateStateSet());
    }
#endif
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

//#define USE_GPU 1

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
_first(0u),
_count(0u),
_current(NULL),
_previous(NULL),
_next(NULL)
{
#ifdef USE_GPU
    _gpu = Registry::capabilities().supportsGLSL();
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
_first(0u),
_count(0u),
_current(NULL),
_previous(NULL),
_next(NULL)
{
#ifdef USE_GPU
    _gpu = Registry::capabilities().supportsGLSL();
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
_first(rhs._first),
_count(rhs._count)
{
    _current = static_cast<osg::Vec3Array*>(getVertexArray());

    if (_gpu)
    {
        _previous = static_cast<osg::Vec3Array*>(getVertexAttribArray(PreviousVertexAttrLocation));
        _next = static_cast<osg::Vec3Array*>(getVertexAttribArray(NextVertexAttrLocation));
    }
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

    if (_mode != GL_LINE_STRIP &&
        _mode != GL_LINE_LOOP &&
        _mode != GL_LINES)
    {
        OE_WARN << LC << "Illegal mode (" << _mode << "). Use GL_LINE_STRIP, GL_LINE_LOOP, or GL_LINES" << std::endl;
        _mode = GL_LINE_STRIP;
    }

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
        _mode = mode;
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
    //bool gpu = Registry::capabilities().supportsGLSL();
    //if (gpu)
    //    stateSet->setDefine("OE_LINES_WIDTH", Stringify() << value, overrideFlags);
    //else
    //    stateSet->setAttributeAndModes(new osg::LineWidth(value), overrideFlags);
}

void
LineDrawable::setStipplePattern(GLushort pattern)
{
    //if (_pattern != pattern)
    {
        _pattern = pattern;
        GLUtils::setLineStipple(getOrCreateStateSet(), _factor, _pattern, 1);
        //if (_gpu)
        //    getOrCreateStateSet()->setDefine("OE_LINES_STIPPLE_PATTERN", Stringify() << _pattern);
        //else
        //    getOrCreateStateSet()->setAttributeAndModes(new osg::LineStipple(_factor, _pattern));
    }
}

void
LineDrawable::setStippleFactor(GLint factor)
{
    if (_factor != factor)
    {
        _factor = factor;
        GLUtils::setLineStipple(getOrCreateStateSet(), _factor, _pattern, 1);
        //if (_gpu)
        //    getOrCreateStateSet()->setDefine("OE_LINES_STIPPLE_FACTOR", Stringify() << _factor );
        //else
        //    getOrCreateStateSet()->setAttributeAndModes(new osg::LineStipple(_factor, _pattern));
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
    if (_gpu)
    {
        if (_mode == GL_LINE_STRIP)
        {
            if (vi == 0)
                for (unsigned i=0; i<2; ++i)
                    (*_colors)[i] = color;
            else
                for (unsigned i=vi*4-2; i<vi*4+2; ++i)
                    (*_colors)[i] = color;
        }
        else
        {
            (*_colors)[vi*2u] = color;
            (*_colors)[vi*2u+1] = color;
        }
    }
    else
    {
        (*_colors)[vi] = color;
    }
    _colors->dirty();
}

void
LineDrawable::setFirst(unsigned value)
{
    _first = value;

    if (_gpu)
    {
        osg::StateSet* ss = getOrCreateStateSet();
        ss->setDefine("OE_LINES_USE_LIMITS");
        osg::Uniform* u = ss->getOrCreateUniform("oe_GPULines_limits", osg::Uniform::FLOAT_VEC2);
        if (_mode == GL_LINE_STRIP)
        {
            u->set(osg::Vec2(4u * _first, _count > 1u ? 4u * (_first + _count - 1u) - 2u : 0u));
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

unsigned
LineDrawable::getFirst() const
{
    return _first;
}

void
LineDrawable::setCount(unsigned value)
{
    _count = value;

    if (_gpu)
    {
        osg::StateSet* ss = getOrCreateStateSet();
        ss->setDefine("OE_LINES_USE_LIMITS");
        osg::Uniform* u = ss->getOrCreateUniform("oe_GPULines_limits", osg::Uniform::FLOAT_VEC2);
        if (_mode == GL_LINE_STRIP)
        {
            u->set(osg::Vec2(4u * _first, _count > 1u ? 4u * (_first + _count - 1u) - 1u : 0u));
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

unsigned
LineDrawable::getCount() const
{
    return _count;
}

void
LineDrawable::pushVertex(const osg::Vec3& vert)
{
    initialize();

    if (_gpu)
    {
        if (_current->empty())
        {
            _current->push_back(vert);
            _current->push_back(vert);

            _previous->push_back(vert);
            _previous->push_back(vert);

            _next->push_back(vert);
            _next->push_back(vert);

            _colors->push_back(_color);
            _colors->push_back(_color);
        }
        else
        {
            if (_mode == GL_LINE_STRIP)
            {
                // close out previous segment:
                *(_next->end() - 1) = vert;
                *(_next->end() - 2) = vert;
                if (_next->size() >= 4)
                {
                    *(_next->end()-3) = vert;
                    *(_next->end()-4) = vert;
                }

                _previous->push_back(_current->back());
                _previous->push_back(_current->back());
                _previous->push_back(_current->back());
                _previous->push_back(_current->back());

                _next->push_back(vert);
                _next->push_back(vert);
                _next->push_back(vert);
                _next->push_back(vert);

                _current->push_back(vert);
                _current->push_back(vert);
                _current->push_back(vert);
                _current->push_back(vert);

                _colors->push_back(_color);
                _colors->push_back(_color);
                _colors->push_back(_color);
                _colors->push_back(_color);
            }

            else if (_mode == GL_LINE_LOOP)
            {
                *(_next->end() - 1) = vert;
                *(_next->end() - 2) = vert;

                *(_previous->begin() + 0) = vert;
                *(_previous->begin() + 1) = vert;

                _previous->push_back(_current->back());
                _previous->push_back(_current->back());

                _next->push_back(_current->front());
                _next->push_back(_current->front());

                _current->push_back(vert);
                _current->push_back(vert);

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

                    *(_next->end()-2) = vert;
                    *(_next->end()-1) = vert;

                    _next->push_back(vert);
                    _next->push_back(vert);
                }

                _current->push_back(vert);
                _current->push_back(vert);

                _colors->push_back(_color);
                _colors->push_back(_color);
            }
        }
    }

    else
    {
        _current->push_back(vert);
        _colors->push_back(_color);
    }
}

void
LineDrawable::setVertex(unsigned vi, const osg::Vec3& vert)
{
    initialize();

    unsigned size = _current->size();
    unsigned numVerts = getNumVerts();
    
    // "vi" = virtual index, "ri" = real index.

    if (vi < numVerts)
    {
        if (_gpu)
        {
            if (_mode == GL_LINE_STRIP)
            {
                unsigned ri = vi == 0u ? 0u : (vi * 4u) - 2u; // starting real index
                unsigned rnum = actualVertsPerVirtualVert(vi); // number of real verts to set

                // update the main verts:
                for (unsigned n = ri; n < ri+rnum; ++n)
                    (*_current)[n] = vert;
                _current->dirty();

                // update next/previous verts:
                if (numVerts == 1u)
                {
                    (*_next)[0] = vert, (*_next)[1] = vert;
                    (*_previous)[0] = vert, (*_previous)[1] = vert;
                }
                else 
                {
                    if (vi == 0u)
                    {
                        for (unsigned n = 0; n < rnum; ++n)
                            (*_previous)[n] = vert;
                        _previous->dirty();
                    }
                    else if (vi < numVerts - 1)
                    {
                        for (unsigned n = ri + rnum; n < ri + rnum + 4; ++n)
                            (*_previous)[n] = vert;
                        _previous->dirty();
                    }

                    if (vi == numVerts - 1)
                    {
                        for (unsigned n = 0; n < rnum; ++n)
                            (*_next)[n] = vert;
                        _next->dirty();
                    }
                    else if (vi > 0)
                    {
                        unsigned prevNum = actualVertsPerVirtualVert(vi-1);
                        for (unsigned n = ri - prevNum; n < ri; ++n)
                            (*_next)[n] = vert;
                        _next->dirty();
                    }
                }
            }

            else if (_mode == GL_LINE_LOOP)
            {
                unsigned ri = vi * 2u;

                (*_current)[ri] = vert;
                (*_current)[ri + 1] = vert;
                _current->dirty();

                if (vi > 0)
                {
                    (*_next)[ri - 1] = vert;
                    (*_next)[ri - 2] = vert;
                    _next->dirty();
                }
                else
                {
                    (*_next)[size - 1] = vert;
                    (*_next)[size - 2] = vert;
                    _next->dirty();
                }
                
                if (vi < numVerts - 1)
                {
                    (*_previous)[ri + 2] = vert;
                    (*_previous)[ri + 3] = vert;
                    _previous->dirty();
                }
                else
                {
                    (*_previous)[0] = vert;
                    (*_previous)[1] = vert;
                    _previous->dirty();
                }
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
    if (_gpu)
        return _mode == GL_LINE_STRIP ? (*_current)[index * 4u] : (*_current)[index * 2u];
    else
        return (*_current)[index];
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
    for (unsigned i = num; i < numVerts; ++i)
    {
        pushVertex(osg::Vec3(0,0,0));
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
        if (_mode == GL_LINE_STRIP)
            return _current->size() == 2 ? 1 : (_current->size()+2)/4;
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
        if (_mode == GL_LINE_STRIP)
            return index == 0u? 2u : 4u;
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
        if (_mode == GL_LINE_STRIP)
            return n == 2u ? 1u : (n+2u)/4u;
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
        actualSize = (_mode == GL_LINE_STRIP) ? (size*4u)-2u : size*2u;
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

            for (int e = 0; e < _current->size() - 4; e += 4)
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
            for (e = 0; e < _current->size()-2; e += 2)
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
            unsigned numEls = (getNumVerts()/2)*6;
            osg::DrawElements* els = makeDE(numEls);  

            for (int e = 0; e < _current->size(); e += 4)
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

void
LineDrawable::installShader()
{
    initialize();

    if (_gpu)
    {
        installShader(getOrCreateStateSet());
    }
}

void
LineDrawable::installShader(osg::StateSet* stateSet)
{
    // install the shader components for GPU lines.
    VirtualProgram* vp = VirtualProgram::getOrCreate(stateSet);
    Shaders shaders;
    shaders.load(vp, shaders.LineDrawable);
    vp->addBindAttribLocation("oe_GPULines_prev", LineDrawable::PreviousVertexAttrLocation);
    vp->addBindAttribLocation("oe_GPULines_next", LineDrawable::NextVertexAttrLocation);
    stateSet->setMode(GL_CULL_FACE, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE | osg::StateAttribute::PROTECTED);
    stateSet->setDefine("OE_GPU_LINES");
}
