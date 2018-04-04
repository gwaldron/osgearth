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
#include "LineDrawable"
#include "Shaders"
#include "VirtualProgram"
#include "Utils"

#include <osg/Depth>
#include <osg/CullFace>
#include <osg/LineStipple>
#include <osg/LineWidth>

#define LC "[LineDrawable] "

#if defined(OSG_GLES1_AVAILABLE) || defined(OSG_GLES2_AVAILABLE) || defined(OSG_GLES3_AVAILABLE)
#define OE_GLES_AVAILABLE
#endif

//#if !defined(OSG_GL_FIXED_FUNCTION_AVAILABLE)
#define OE_USE_GPU_LINES
//#endif

using namespace osgEarth;


// References:
// https://mattdesl.svbtle.com/drawing-lines-is-hard
// https://github.com/mattdesl/webgl-lines


LineGroup::LineGroup() :
_gpu(false)
{
#ifdef OE_USE_GPU_LINES
    _gpu = true;
#endif

    if (_gpu)
    {
        // install the shader components for GPU lines.
        osg::StateSet* ss = getOrCreateStateSet();
        VirtualProgram* vp = VirtualProgram::getOrCreate(ss);
        Shaders shaders;
        shaders.load(vp, shaders.LineDrawable);
        vp->addBindAttribLocation("oe_GPULines_prev", LineDrawable::PreviousVertexAttrLocation);
        vp->addBindAttribLocation("oe_GPULines_next", LineDrawable::NextVertexAttrLocation);
        ss->setMode(GL_CULL_FACE, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE | osg::StateAttribute::PROTECTED);
        // Why don't we write to the depth buffer? -gw
        //ss->setAttributeAndModes(new osg::Depth(osg::Depth::LEQUAL, 0.0, 1.0, false));
        ss->setDefine("OE_GPU_LINES");
    }
}

//...................................................................

#undef  LC
#define LC "[LineDrawable] "

// static attribute binding locations. Changable by the user.
int LineDrawable::PreviousVertexAttrLocation = 9;
int LineDrawable::NextVertexAttrLocation = 10;

LineDrawable::LineDrawable(GLenum mode) :
osg::Drawable(),
_mode(mode),
_gpu(false)
{
    if (mode != GL_LINE_STRIP &&
        mode != GL_LINE_LOOP &&
        mode != GL_LINES)
    {
        OE_WARN << LC << "Illegal mode (" << mode << "). Use GL_LINE_STRIP, GL_LINE_LOOP, or GL_LINES" << std::endl;
    }

#ifdef OE_USE_GPU_LINES
    _gpu = true;
#endif

    setUseDisplayList(false);

    _geom = new osg::Geometry();

    _geom->setUseVertexBufferObjects(true);
    _geom->setUseDisplayList(false);

    _current = new osg::Vec3Array();
    _current->setBinding(osg::Array::BIND_PER_VERTEX);
    _geom->setVertexArray(_current.get());

    if (_gpu)
    {
        _previous = new osg::Vec3Array();
        _previous->setBinding(osg::Array::BIND_PER_VERTEX);
        _previous->setNormalize(false);
        _geom->setVertexAttribArray(PreviousVertexAttrLocation, _previous.get());    

        _next = new osg::Vec3Array();
        _next->setBinding(osg::Array::BIND_PER_VERTEX);
        _next->setNormalize(false);
        _geom->setVertexAttribArray(NextVertexAttrLocation, _next.get());
    }
}

LineDrawable::LineDrawable(const LineDrawable& rhs, const osg::CopyOp& copy) :
osg::Drawable(rhs, copy),
_geom(static_cast<osg::Geometry*>(copy(rhs._geom.get()))),
_mode(rhs._mode),
_gpu(rhs._gpu)
{
    _current = static_cast<osg::Vec3Array*>(_geom->getVertexArray());

    if (_gpu)
    {
        _previous = static_cast<osg::Vec3Array*>(_geom->getVertexAttribArray(PreviousVertexAttrLocation));
        _next = static_cast<osg::Vec3Array*>(_geom->getVertexAttribArray(NextVertexAttrLocation));
    }
}

void
LineDrawable::setLineWidth(float width)
{
    if (_gpu)
        getOrCreateStateSet()->setDefine("OE_GPULINES_WIDTH", Stringify() << width);
    else
        getOrCreateStateSet()->setAttributeAndModes(new osg::LineWidth(width));
}

void
LineDrawable::setStippling(short factor, short pattern)
{
    if (_gpu)
    {
        getOrCreateStateSet()->setDefine("OE_GPULINES_STIPPLE_PATTERN", Stringify() << pattern );
        getOrCreateStateSet()->setDefine("OE_GPULINES_STIPPLE_FACTOR", Stringify() << factor );
    }
    else
    {
        getOrCreateStateSet()->setAttributeAndModes(new osg::LineStipple(factor, pattern));
    }
}

void
LineDrawable::setColor(const osg::Vec4& color)
{
    osg::Vec4Array* colors = dynamic_cast<osg::Vec4Array*>(_geom->getColorArray());

    if (_geom->getColorArray() == 0L)
    {
        colors = new osg::Vec4Array();
        colors->setBinding(osg::Array::BIND_OVERALL);
        _geom->setColorArray(colors);
    }

    colors->clear();
    colors->push_back(color);
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

            (*_current.get())[k] = vert;
            (*_current.get())[k+1] = vert;
            _current->dirty();

            if (k > 0)
            {
                (*_next)[k-1] = vert;
                (*_next)[k-2] = vert;
                _next->dirty();
            }
            else if (_mode == GL_LINE_LOOP)
            {
                (*_next)[size-1] = vert;
                (*_next)[size-2] = vert;
                _next->dirty();
            }

            if (i < numVerts-1)
            {
                (*_previous)[k+2] = vert;
                (*_previous)[k+3] = vert;
                _previous->dirty();
            }
            else if (_mode == GL_LINE_LOOP)
            {
                (*_previous)[0] = vert;
                (*_previous)[1] = vert;
                _previous->dirty();
            }
        }

        else
        {
            (*_current)[i] = vert;
            _current->dirty();
        }

        _geom->dirtyBound();
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
    _geom->dirtyBound();

    _current->dirty();

    if (_gpu)
    {
        _previous->dirty();
        _next->dirty();
    }

    // rebuild primitive sets.
    if (_geom->getNumPrimitiveSets() > 0)
    {
        _geom->removePrimitiveSet(0, 1);
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
            
            _geom->addPrimitiveSet(els);
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
            
            _geom->addPrimitiveSet(els);
        }

        else if (_mode == GL_LINES)
        {
            for (int e = 0; e < _current->size(); e += 4)
            {
                osg::DrawElements* els = makeDE(6u);
                els->addElement(e+3);
                els->addElement(e+1);
                els->addElement(e+0); // PV
                els->addElement(e+2);
                els->addElement(e+3);
                els->addElement(e+0); // PV
                _geom->addPrimitiveSet(els);
            }
        }

#if 0
        // rebuild the previous/next lists
        _previous->clear();
        _next->clear();
        for (int e = 0; e < _current->size(); e += 2)
        {
            osg::Vec3Array::iterator c = _current->begin() + e;
            osg::Vec3Array::iterator p = _previous->begin() + e;
            osg::Vec3Array::iterator n = _next->begin() + e;

            if (_mode == GL_LINE_STRIP)
            {
                *p = *(p + 1) = (e == 0) ? *c : *(c-1);
                //if (e == 0)
                //    *p = *(p+1) = *c;
                //else
                //    *p = *(p+1) = *(c-1);
                *n = *(n + 1) = (e == _current->size() - 2) ? *c : *(c+2);
                //if (e == _current->size()-2)
                //    *n = *(n+1) = *c;
                //else
                //    *n = *(n+1) = *(c+2);
            }

            else if (_mode = GL_LINE_LOOP)
            {
                if (e == 0)
                    *p = *(p+1) = _current->back();
                else
                    *p = *(p+1) = *(c-1);

                if (e == _current->size()-2)
                    *n = *(n + 1) = _current->front();
                else
                    *n = *(n + 1) = *(c+2);
            }

            else if (_mode == GL_LINES)
            {

            }


                    (*_previous)[e] = (*_current)[e];
                    (*_previous)[e + 1] = (*_current)[e + 1];
                }
                else
                {
                    (*_previous())
                }
            }
            else if (_mode == GL_LINE_LOOP)
            {

            }
            else if (_mode == GL_LINES)
            {

            }
        }
#endif
    }

    else
    {
        _geom->addPrimitiveSet(new osg::DrawArrays(_mode, 0, _current->size()));
    }
}

void
LineDrawable::installShader()
{
    if (_gpu)
    {
        // install the shader components for GPU lines.
        osg::StateSet* ss = getOrCreateStateSet();
        VirtualProgram* vp = VirtualProgram::getOrCreate(ss);
        Shaders shaders;
        shaders.load(vp, shaders.LineDrawable);
        vp->addBindAttribLocation("oe_GPULines_prev", LineDrawable::PreviousVertexAttrLocation);
        vp->addBindAttribLocation("oe_GPULines_next", LineDrawable::NextVertexAttrLocation);
        ss->setMode(GL_CULL_FACE, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE | osg::StateAttribute::PROTECTED);
        // Why don't we write to the depth buffer? -gw
        //ss->setAttributeAndModes(new osg::Depth(osg::Depth::LEQUAL, 0.0, 1.0, false));
        ss->setDefine("OE_GPU_LINES");
    }
}

void
LineDrawable::drawImplementation(osg::RenderInfo& ri) const
{
    //osg::GLExtensions* gl = osg::GLExtensions::Get(ri.getContextID(), true);
    //gl->glPatchParameteri(1, 2);
    _geom->drawImplementation(ri);
}