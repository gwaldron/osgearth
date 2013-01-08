/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2011 Pelican Mapping
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
#include "LODFactorCallback"

#include <osgEarth/CullingUtils>
#include <osg/Math>
#include <osg/PagedLOD>
#include <osg/StateSet>
#include <osg/Uniform>

using namespace osgEarth_engine_osgterrain;

// This callback sets a uniform, osgearth_LODRangeFactor, based on the
// distance from the camera and its relation to the minimum and
// maximum distance for a tile. The maximum distance isn't actually
// available, so 2 * min distance is used as an estimate. The range
// factor's value goes from 0 - at the maximum range - to 1 for the
// minimum range.

void LODFactorCallback::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
    // test the type since this is not always a PagedLOD.
    osg::PagedLOD* lod = static_cast<osg::PagedLOD*>(node);
    osgUtil::CullVisitor* cv = osgEarth::Culling::asCullVisitor(nv);
    osg::LOD::RangeMode rangeMode = lod->getRangeMode();
    float requiredRange = 0.0f;
    float rangeFactor = 1.0f;
    const osg::LOD::RangeList& rangeList = lod->getRangeList();
    if (rangeMode == osg::LOD::DISTANCE_FROM_EYE_POINT)
    {
        requiredRange = cv->getDistanceToViewPoint(lod->getCenter(), true);
    }
    else if (cv->getLODScale() > 0.0f)
    {
        requiredRange = cv->clampedPixelSize(lod->getBound()) / cv->getLODScale();
    }
    else
    {
        // The comment in osg/PagedLOD.cpp says that this algorithm
        // finds the highest res tile, but it actually finds the
        // lowest res tile!
        for (osg::LOD::RangeList::const_iterator itr = rangeList.begin(), end = rangeList.end();
            itr != end;
            ++itr)
        {
            requiredRange = osg::maximum(requiredRange, itr->first);
        }
    }
    // We're counting on only finding one valid LOD, unlike the
    // general OSG behavior.
    if (!rangeList.empty() && rangeList[0].first <= requiredRange
        && requiredRange < rangeList[0].second)
    {
        rangeFactor = 1.0f - (requiredRange - rangeList[0].first) / rangeList[0].first;
        rangeFactor = osg::clampTo(rangeFactor, 0.0f, 1.0f);
    }
    osg::ref_ptr<osg::Uniform> ufact
        = new osg::Uniform("osgearth_LODRangeFactor", rangeFactor);
    osg::ref_ptr<osg::StateSet> ss = new osg::StateSet;
    ss->addUniform(ufact.get());

    cv->pushStateSet(ss.get());
    traverse(node, nv);
    cv->popStateSet();
}
