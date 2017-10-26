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

#define SLOT_PREVIOUS 6
#define SLOT_NEXT 7
#define SLOT_SIGNED_WIDTH 8

osg::Geometry*
GPULinesOperator::operator()(osg::Vec3Array* input, bool closeTheLoop) const
{
    if (input == 0L || input->size() <= 1)
        return 0L;

    osg::Geometry* geom = new osg::Geometry();
    geom->setUseVertexBufferObjects(true);
    geom->setUseDisplayList(false);

    int outputSize = input->size() * 2 + (closeTheLoop? 2 : 0);

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
    for (int i = 0; i < input->size(); ++i)
    {
        const osg::Vec3& c = (*input)[i];
        positions->push_back(c);
        direction->push_back(-thickness);       
        positions->push_back(c);
        direction->push_back(thickness);
    }

    // if we need to close the loop, dupliate the front at the back:
    if (closeTheLoop && positions->front() != positions->back())
    {
        positions->push_back(positions->front());
        direction->push_back(-thickness);
        positions->push_back(positions->front());
        direction->push_back(thickness);
    }

    // generate the previous and next attributes:
    for (int i = 0; i < positions->size(); i += 2)
    {
        const osg::Vec3& p = (*positions)[osg::clampBetween(i - 1, 0, (int)positions->size()-1)];
        const osg::Vec3& n = (*positions)[osg::clampBetween(i + 2, 0, (int)positions->size()-1)];

        previous->push_back(p);
        previous->push_back(p);
        next->push_back(n);
        next->push_back(n);
    }

    // generate the triangle set.
    unsigned elSize = (positions->size() - 2) * 3;
    osg::DrawElements* els = 
        elSize > 0xFFFF ? (osg::DrawElements*)new osg::DrawElementsUShort( GL_TRIANGLES ) :
        elSize > 0xFF   ? (osg::DrawElements*)new osg::DrawElementsUShort( GL_TRIANGLES ) :
                          (osg::DrawElements*)new osg::DrawElementsUByte ( GL_TRIANGLES );
    els->reserveElements(elSize);
    for (int i = 0; i < positions->size() - 2; i += 2)
    {
        els->addElement(i+0);
        els->addElement(i+1);
        els->addElement(i+2);
        els->addElement(i+2);
        els->addElement(i+1);
        els->addElement(i+3);
    }
    geom->addPrimitiveSet(els);

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
}
