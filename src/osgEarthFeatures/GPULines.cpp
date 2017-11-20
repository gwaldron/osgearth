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
#include <osgEarthFeatures/GPULines>
#include <osgEarthFeatures/FeatureSourceIndexNode>
#include <osgEarthFeatures/Shaders>

#include <osgEarth/VirtualProgram>
#include <osgEarth/Utils>

#include <osg/Depth>
#include <osg/CullFace>

#define LC "[GPULines] "

#if defined(OSG_GLES1_AVAILABLE) || defined(OSG_GLES2_AVAILABLE) || defined(OSG_GLES3_AVAILABLE)
#define OE_GLES_AVAILABLE
#endif

using namespace osgEarth::Features;


// References:
// https://mattdesl.svbtle.com/drawing-lines-is-hard
// https://github.com/mattdesl/webgl-lines


// static attribute binding locations. Changable by the user.
int GPULines::PreviousVertexAttrLocation = 9;
int GPULines::NextVertexAttrLocation = 10;
int GPULines::WidthAttrLocation = 11;


GPULinesOperator::GPULinesOperator()
{
    //nop
}

GPULinesOperator::GPULinesOperator(const Stroke& stroke) :
_stroke( stroke )
{
    //nop
}

osg::Geometry*
GPULinesOperator::operator()(osg::Vec3Array* input, bool isLoop) const
{
    if (input == 0L || input->size() <= 1)
        return 0L;

    osg::Geometry* geom = new osg::Geometry();
    geom->setUseVertexBufferObjects(true);
    geom->setUseDisplayList(false);

    bool frontEqualsBack = input->front() == input->back();

    // is this a line loop even though the caller doesn't know it?
    if (frontEqualsBack)
        isLoop = true;
    
    // if this is a loop and the first and last elements are equal, trim the back:
    int inputSize = input->size() - (frontEqualsBack? 1 : 0);

    // two output elements per input element:
    int outputSize = inputSize * 2;

    osg::Vec3Array* positions = new osg::Vec3Array();
    positions->reserve(outputSize);
    geom->setVertexArray(positions);

    osg::Vec3Array* previous = new osg::Vec3Array();
    previous->reserve(outputSize);
    previous->setBinding(previous->BIND_PER_VERTEX);
    geom->setVertexAttribArray(GPULines::PreviousVertexAttrLocation, previous);
    geom->setVertexAttribBinding(GPULines::PreviousVertexAttrLocation, osg::Geometry::BIND_PER_VERTEX);
    geom->setVertexAttribNormalize(GPULines::PreviousVertexAttrLocation, false);

    osg::Vec3Array* next = new osg::Vec3Array();
    next->reserve(outputSize);
    next->setBinding(next->BIND_PER_VERTEX);
    geom->setVertexAttribArray(GPULines::NextVertexAttrLocation, next);
    geom->setVertexAttribBinding(GPULines::NextVertexAttrLocation, osg::Geometry::BIND_PER_VERTEX);
    geom->setVertexAttribNormalize(GPULines::NextVertexAttrLocation, false);

    osg::FloatArray* widths = new osg::FloatArray();
    widths->reserve(outputSize);
    widths->setBinding(next->BIND_PER_VERTEX);
    geom->setVertexAttribArray(GPULines::WidthAttrLocation, widths);
    geom->setVertexAttribBinding(GPULines::WidthAttrLocation, osg::Geometry::BIND_PER_VERTEX);
    geom->setVertexAttribNormalize(GPULines::WidthAttrLocation, false);

    float thickness = _stroke.width().get();

    // first populate the positions and direction attributes:
    for (int i = 0; i < inputSize; ++i)
    {
        const osg::Vec3& c = (*input)[i];
        positions->push_back(c);      
        positions->push_back(c);

        // opposite directions:
        widths->push_back(-thickness); 
        widths->push_back(thickness);

        int prevIndex = i-1;
        if (prevIndex < 0) 
            prevIndex = isLoop ? inputSize-1 : i;

        const osg::Vec3& p = (*input)[prevIndex];
        previous->push_back(p);
        previous->push_back(p);

        int nextIndex = i+1;
        if (nextIndex > inputSize-1)
            nextIndex = isLoop ? 0 : i;

        const osg::Vec3& n = (*input)[nextIndex];
        next->push_back(n);
        next->push_back(n);
    }

    // generate the triangle set.
    unsigned numEls = (inputSize + (isLoop? 1 : 0)) * 6;
    osg::DrawElements* els = 
#ifndef OE_GLES_AVAILABLE
        numEls > 0xFFFF ? (osg::DrawElements*)new osg::DrawElementsUInt  ( GL_TRIANGLES ) :
#endif
        numEls > 0xFF   ? (osg::DrawElements*)new osg::DrawElementsUShort( GL_TRIANGLES ) :
                          (osg::DrawElements*)new osg::DrawElementsUByte ( GL_TRIANGLES );
    els->reserveElements(numEls);

    // IMPORTANT!
    // Don't change the order of these elements! Because of the way
    // GPU line stippling works, it is critical that the provoking vertex
    // be at the beginning of each line segment. In this case we are using
    // GL_TRIANGLES and thus the provoking vertex (PV) is the FINAL vert
    // in each triangle.
    int e;
    for (e = 0; e < positions->size()-2; e += 2)
    {
        els->addElement(e+3);
        els->addElement(e+1);
        els->addElement(e+0); // PV
        els->addElement(e+2);
        els->addElement(e+3);
        els->addElement(e+0); // PV
    }

    if (isLoop)
    {
        els->addElement(1);
        els->addElement(e+1);
        els->addElement(e+0); // PV
        els->addElement(0);
        els->addElement(1);
        els->addElement(e+0); // PV
    }

    geom->addPrimitiveSet(els);
    
    if (_stroke.stipplePattern().isSet())
    {
        osg::StateSet* ss = geom->getOrCreateStateSet();

        ss->setDefine(
            "OE_GPULINES_STIPPLE_PATTERN", 
            Stringify() << _stroke.stipplePattern().get() );

        ss->setDefine(
            "OE_GPULINES_STIPPLE_FACTOR",
            Stringify() << _stroke.stippleFactor().get() );
    }

    return geom;
}


void
GPULinesOperator::installShaders(osg::Node* node) const
{
    osg::StateSet* ss = node->getOrCreateStateSet();
    VirtualProgram* vp = VirtualProgram::getOrCreate(ss);
    GPULineShaders shaders;
    shaders.loadAll(vp);
    vp->addBindAttribLocation("oe_GPULines_prev", GPULines::PreviousVertexAttrLocation);
    vp->addBindAttribLocation("oe_GPULines_next", GPULines::NextVertexAttrLocation);
    vp->addBindAttribLocation("oe_GPULines_width", GPULines::WidthAttrLocation);
    ss->setMode(GL_CULL_FACE, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE | osg::StateAttribute::PROTECTED);
    ss->setAttributeAndModes(new osg::Depth(osg::Depth::LEQUAL, 0.0, 1.0, false));
    ss->setDefine("OE_GPU_LINES");
}
