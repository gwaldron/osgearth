/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2020 Pelican Mapping
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
#include "MonitorUI"
#include <osgEarth/Memory>
#include <osgEarth/GLUtils>
#include <osgEarth/Registry>

using namespace osgEarth::Monitor;
using namespace osgEarth;
using namespace osgEarth::Util;

namespace ui = osgEarth::Util::Controls;

#define LC "[MonitorUI] "

MonitorUI::MonitorUI()
{
    this->setHorizAlign(ALIGN_LEFT);
    this->setVertAlign(ALIGN_BOTTOM);

    int r=0;

    this->setControl(0, r, new ui::LabelControl("Physical RAM:"));
    _ws = this->setControl(1, r, new ui::LabelControl());
    ++r;

    this->setControl(0, r, new ui::LabelControl("Total RAM:"));
    _pb = this->setControl(1, r, new ui::LabelControl());
    ++r;

    this->setControl(0, r, new ui::LabelControl("Peak RAM:"));
    _ppb = this->setControl(1, r, new ui::LabelControl());
    ++r;

    this->setControl(0, r, new ui::LabelControl("Jobs:"));
    _jobdata = this->setControl(1, r, new ui::LabelControl());
    ++r;

    this->setControl(0, r, new ui::LabelControl("ICO Jobs:"));
    _ico = this->setControl(1, r, new ui::LabelControl());
    ++r;
}

void
MonitorUI::update(const osg::FrameStamp* fs)
{
    if (fs && fs->getFrameNumber() % 5 == 0)
    {
        _ws->setText(Stringify() << (Memory::getProcessPhysicalUsage() / 1048576) << " M");
        _pb->setText(Stringify() << (Memory::getProcessPrivateUsage() / 1048576) << " M");
        _ppb->setText(Stringify() << (Memory::getProcessPeakPrivateUsage() / 1048576) << " M");

        std::stringstream buf;
        const JobArena::Metrics& m = JobArena::metrics();
        int pending = 0, running = 0;
        for (int i = 0; i <= m.maxArenaIndex; ++i)
        {
            if (m.arena(i).active)
            {
                buf << m.arena(i).arenaName
                    << " (" << m.arena(i).concurrency << ") "
                    << m.arena(i).numJobsRunning
                    << " / "
                    << m.arena(i).numJobsPending
                    << " // "
                    << m.arena(i).numJobsCanceled
                    << "\n";
            }
        }
        _jobdata->setText(buf.str());

        _ico->setText(Stringify() << GLObjectsCompiler::totalJobs());
    }
}
