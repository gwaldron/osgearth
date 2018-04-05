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

#include <osg/Depth>
#include <osg/CullFace>
#include <osg/LineStipple>
#include <osg/LineWidth>
#include <osgUtil/Optimizer>

#define LC "[LineDrawable] "

#if defined(OSG_GLES1_AVAILABLE) || defined(OSG_GLES2_AVAILABLE) || defined(OSG_GLES3_AVAILABLE)
#define OE_GLES_AVAILABLE
#endif

using namespace osgEarth;


// References:
// https://mattdesl.svbtle.com/drawing-lines-is-hard
// https://github.com/mattdesl/webgl-lines


LineGroup::LineGroup()
{
    if (Registry::capabilities().supportsGLSL())
    {
        LineDrawable::installShader(getOrCreateStateSet());
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

//...................................................................

#undef  LC
#define LC "[LineDrawable] "

// static attribute binding locations. Changable by the user.
int LineDrawable::PreviousVertexAttrLocation = 9;
int LineDrawable::NextVertexAttrLocation = 10;

LineDrawable::LineDrawable(GLenum mode) :
osg::Geometry(),
_mode(mode),
_gpu(true),
_factor(1),
_pattern(0xFFFF),
_color(1,1,1,1),
_width(1.0f)
{
    if (mode != GL_LINE_STRIP &&
        mode != GL_LINE_LOOP &&
        mode != GL_LINES)
    {
        OE_WARN << LC << "Illegal mode (" << mode << "). Use GL_LINE_STRIP, GL_LINE_LOOP, or GL_LINES" << std::endl;
        _mode = GL_LINE_STRIP;
    }
    
    // check for GLSL support:
    _gpu = Registry::capabilities().supportsGLSL();

    setUseVertexBufferObjects(_supportsVertexBufferObjects);
    setUseDisplayList(false);

    _current = new osg::Vec3Array();
    _current->setBinding(osg::Array::BIND_PER_VERTEX);
    setVertexArray(_current);

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

LineDrawable::LineDrawable(const LineDrawable& rhs, const osg::CopyOp& copy) :
osg::Geometry(rhs, copy),
_mode(rhs._mode),
_gpu(rhs._gpu),
_color(rhs._color),
_factor(rhs._factor),
_pattern(rhs._pattern),
_width(rhs._width)
{
    _current = static_cast<osg::Vec3Array*>(getVertexArray());

    if (_gpu)
    {
        _previous = static_cast<osg::Vec3Array*>(getVertexAttribArray(PreviousVertexAttrLocation));
        _next = static_cast<osg::Vec3Array*>(getVertexAttribArray(NextVertexAttrLocation));
    }
}

void
LineDrawable::setLineWidth(float value)
{
    _width = value;
    if (_gpu)
        getOrCreateStateSet()->setDefine("OE_GPULINES_WIDTH", Stringify() << _width);
    else
        getOrCreateStateSet()->setAttributeAndModes(new osg::LineWidth(_width));
}

void
LineDrawable::setStipplePattern(GLushort pattern)
{
    _pattern = pattern;
    if (_gpu)
        getOrCreateStateSet()->setDefine("OE_GPULINES_STIPPLE_PATTERN", Stringify() << _pattern);
    else
        getOrCreateStateSet()->setAttributeAndModes(new osg::LineStipple(_factor, _pattern));
}

void
LineDrawable::setStippleFactor(GLint factor)
{
    _factor = factor;
    if (_gpu)
        getOrCreateStateSet()->setDefine("OE_GPULINES_STIPPLE_FACTOR", Stringify() << _factor );
    else
        getOrCreateStateSet()->setAttributeAndModes(new osg::LineStipple(_factor, _pattern));
}

void
LineDrawable::setColor(const osg::Vec4& color)
{
    _color = color;
}

void
LineDrawable::pushVertex(const osg::Vec3& vert)
{
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
        }
        else
        {
            if (_mode == GL_LINE_STRIP)
            {
                *(_next->end() - 1) = vert;
                *(_next->end() - 2) = vert;

                _previous->push_back(_current->back());
                _previous->push_back(_current->back());

                _next->push_back(vert);
                _next->push_back(vert);

                _current->push_back(vert);
                _current->push_back(vert);
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
            }
        }
    }

    else
    {
        _current->push_back(vert);
    }
}

void
LineDrawable::setVertex(unsigned i, const osg::Vec3& vert)
{
    unsigned size = _current->size();
    unsigned numVerts = _gpu? size/2u : size;

    if (i < numVerts)
    {
        if (_gpu)
        {
            unsigned k = i*2u;

            (*_current)[k] = vert;
            (*_current)[k+1] = vert;
            _current->dirty();

            if (_mode == GL_LINE_STRIP)
            {
                if (k > 0)
                {
                    (*_next)[k - 1] = vert;
                    (*_next)[k - 2] = vert;
                    _next->dirty();
                }
                if (i < numVerts - 1)
                {
                    (*_previous)[k + 2] = vert;
                    (*_previous)[k + 3] = vert;
                    _previous->dirty();
                }
            }

            else if (_mode == GL_LINE_LOOP)
            {
                if (k > 0)
                {
                    (*_next)[k - 1] = vert;
                    (*_next)[k - 2] = vert;
                    _next->dirty();
                }
                else
                {
                    (*_next)[size - 1] = vert;
                    (*_next)[size - 2] = vert;
                    _next->dirty();
                }
                
                if (i < numVerts - 1)
                {
                    (*_previous)[k + 2] = vert;
                    (*_previous)[k + 3] = vert;
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
                bool first = (i & 0x01) == 0;
                if (first)
                {
                    (*_previous)[k] = vert;
                    (*_previous)[k+1] = vert;
                    (*_previous)[k+2] = vert;
                    (*_previous)[k+3] = vert;
                    _previous->dirty();
                }
                else
                {
                    (*_next)[k+1] = vert;
                    (*_next)[k] = vert;
                    (*_next)[k-1] = vert;
                    (*_next)[k-2] = vert;
                    _next->dirty();
                }
            }
        }

        else
        {
            (*_current)[i] = vert;
            _current->dirty();
        }

        dirtyBound();
    }
}

void
LineDrawable::setVerts(const osg::Vec3Array* verts)
{
    _current->clear();
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
}

void
LineDrawable::reserve(unsigned size)
{
    if (size > _current->size())
    {
        unsigned actualSize = _gpu? size*2u : size;

        _current->reserve(actualSize);
        if (_gpu)
        {
            _previous->reserve(actualSize);
            _next->reserve(actualSize);
        }
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
            unsigned numEls = ((_current->size()/2)-1) * 6;
            osg::DrawElements* els = makeDE(numEls);  

            for (int e = 0; e < _current->size()-2; e += 2)
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
            unsigned numEls = (_current->size()/2) * 6;
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
            unsigned numEls = (_current->size()/4) * 6;
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
        addPrimitiveSet(new osg::DrawArrays(_mode, 0, _current->size()));
    }
    
    // update color array if necessary:
    osg::Vec4Array* colors = static_cast<osg::Vec4Array*>(getColorArray());
    if (colors == 0L)
    {
        colors = new osg::Vec4Array();
        colors->setBinding(osg::Array::BIND_PER_VERTEX);
        setColorArray(colors);
    }
    colors->assign(_current->size(), _color);
    colors->dirty();
}

void
LineDrawable::installShader()
{
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
