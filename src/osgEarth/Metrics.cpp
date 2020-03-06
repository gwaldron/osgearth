/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2008-2012 Pelican Mapping
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


#include <osgEarth/Metrics>
#include <osgViewer/ViewerBase>
#include <osgViewer/View>
#include <osgEarth/Memory>

using namespace osgEarth::Util;

#define LC "[Metrics] "

static bool s_metricsEnabled = true;

bool Metrics::enabled()
{
    return s_metricsEnabled;
}

void Metrics::setEnabled(bool enabled)
{
    s_metricsEnabled = enabled;
}

void Metrics::frame()
{
    OE_PROFILING_FRAME_MARK;
}

int Metrics::run(osgViewer::ViewerBase& viewer)
{
    if (!viewer.isRealized())
    {
        viewer.realize();
    }

    // Report memory and fps every 60 frames.
    unsigned int reportEvery = 60;

    osgViewer::ViewerBase::Views views;
    viewer.getViews(views);

    while (!viewer.done())
    {
        {
            OE_PROFILING_ZONE_NAMED("advance");
            viewer.advance();
        }

        {
            OE_PROFILING_ZONE_NAMED("eventTraversal");
            viewer.eventTraversal();
        }

        {
            OE_PROFILING_ZONE_NAMED("updateTraversal");
            viewer.updateTraversal();
        }

        {
            OE_PROFILING_ZONE_NAMED("renderingTraversals");
            viewer.renderingTraversals();
        }

        // Report memory and fps periodically. periodically.
        if (views[0]->getFrameStamp()->getFrameNumber() % reportEvery == 0)
        {         
            OE_PROFILING_PLOT("WorkingSet", (float)(Memory::getProcessPhysicalUsage() / 1048576));
            OE_PROFILING_PLOT("PrivateBytes", (float)(Memory::getProcessPrivateUsage() / 1048576));
            OE_PROFILING_PLOT("PeakPrivateBytes", (float)(Memory::getProcessPeakPrivateUsage() / 1048576));                                                                                                 
        }

        frame();
    }
    return 0;
}
