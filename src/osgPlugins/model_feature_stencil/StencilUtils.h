/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2009 Pelican Ventures, Inc.
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
#ifndef OSGEARTH_MODEL_FEATURE_STENCIL_STENCIL_UTILS_H
#define OSGEARTH_MODEL_FEATURE_STENCIL_STENCIL_UTILS_H 1

#include <osg/Node>
#include <osg/Geode>
#include <osgEarthFeatures/Geometry>
#include <osgEarthFeatures/Filter>

using namespace osgEarth::Features;

struct StencilUtils
{
    static osg::Node* createGeometryPass( 
        osg::Node* geometry,
        int& ref_renderBin );

    static osg::Node* createMaskPass(
        const osg::Vec4ub& color,
        int& ref_renderBin );

    static osg::Geode* createVolume(
        Geometry*            geom,
        double               offset,
        double               height,
        const FilterContext& context );
};

#endif // OSGEARTH_MODEL_FEATURE_STENCIL_STENCIL_UTILS_H
