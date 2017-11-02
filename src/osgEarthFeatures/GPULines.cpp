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
#include <osgEarthFeatures/Session>
#include <osgEarthFeatures/Shaders>
#include <osgEarth/VirtualProgram>
#include <osgEarth/Utils>
#include <osg/CullFace>

#define LC "[GPULines] "

using namespace osgEarth::Features;


// References:
// https://mattdesl.svbtle.com/drawing-lines-is-hard
// https://github.com/mattdesl/webgl-lines

GPULinesOperator::GPULinesOperator()
{
    //nop
}

GPULinesOperator::GPULinesOperator(const Stroke& stroke) :
_stroke( stroke )
{
    //nop
}

#define SLOT_PREVIOUS 9
#define SLOT_NEXT 10
#define SLOT_SIGNED_WIDTH 11

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
    geom->setVertexAttribArray(SLOT_PREVIOUS, previous);
    geom->setVertexAttribBinding(SLOT_PREVIOUS, osg::Geometry::BIND_PER_VERTEX);
    geom->setVertexAttribNormalize(SLOT_PREVIOUS, false);

    osg::Vec3Array* next = new osg::Vec3Array();
    next->reserve(outputSize);
    next->setBinding(next->BIND_PER_VERTEX);
    geom->setVertexAttribArray(SLOT_NEXT, next);
    geom->setVertexAttribBinding(SLOT_NEXT, osg::Geometry::BIND_PER_VERTEX);
    geom->setVertexAttribNormalize(SLOT_NEXT, false);

    osg::FloatArray* direction = new osg::FloatArray();
    direction->reserve(outputSize);
    direction->setBinding(next->BIND_PER_VERTEX);
    geom->setVertexAttribArray(SLOT_SIGNED_WIDTH, direction);
    geom->setVertexAttribBinding(SLOT_SIGNED_WIDTH, osg::Geometry::BIND_PER_VERTEX);
    geom->setVertexAttribNormalize(SLOT_SIGNED_WIDTH, false);

    float thickness = _stroke.width().get();

    // first populate the positions and direction attributes:
    for (int i = 0; i < inputSize; ++i)
    {
        const osg::Vec3& c = (*input)[i];
        positions->push_back(c);      
        positions->push_back(c);

        // opposite directions:
        direction->push_back(-thickness); 
        direction->push_back(thickness);

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

#if 0
    geom->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLE_STRIP, 0, positions->size()));
#else
    // generate the triangle set.
    unsigned elSize = (inputSize + (isLoop? 1 : 0)) * 6;
    osg::DrawElements* els = 
        elSize > 0xFFFF ? (osg::DrawElements*)new osg::DrawElementsUInt  ( GL_TRIANGLES ) :
        elSize > 0xFF   ? (osg::DrawElements*)new osg::DrawElementsUShort( GL_TRIANGLES ) :
                          (osg::DrawElements*)new osg::DrawElementsUByte ( GL_TRIANGLES );
    els->reserveElements(elSize);

    int e;
    for (e = 0; e < positions->size()-2; e += 2)
    {
        els->addElement(e+0);
        els->addElement(e+2);
        els->addElement(e+1);
        els->addElement(e+1);
        els->addElement(e+2);
        els->addElement(e+3);
    }

    if (isLoop)
    {
        els->addElement(e+0);
        els->addElement(0);
        els->addElement(e+1);
        els->addElement(e+1);
        els->addElement(0);
        els->addElement(1);
    }

    geom->addPrimitiveSet(els);
#endif

    return geom;
}


void
GPULinesOperator::installShaders(osg::Node* node) const
{
    osg::StateSet* ss = node->getOrCreateStateSet();
    VirtualProgram* vp = VirtualProgram::getOrCreate(ss);
    GPULineShaders shaders;
    shaders.loadAll(vp);
    vp->addBindAttribLocation("oe_GPULines_prev", SLOT_PREVIOUS);
    vp->addBindAttribLocation("oe_GPULines_next", SLOT_NEXT);
    vp->addBindAttribLocation("oe_GPULines_width", SLOT_SIGNED_WIDTH);
    ss->setMode(GL_CULL_FACE, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE | osg::StateAttribute::PROTECTED);

    //const char* frag =
    //    "in vec4 lineColor;\n"
    //    "void colorLines(inout vec4 color) { color = lineColor; }\n";
    //vp->setFunction("colorLines", frag, ShaderComp::LOCATION_FRAGMENT_COLORING);
}
