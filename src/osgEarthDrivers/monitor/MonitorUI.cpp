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
    //this->setHorizAlign(ALIGN_LEFT);
    //this->setVertAlign(ALIGN_BOTTOM);

    int r=0;

    this->setControl(0, r, new ui::LabelControl("RAM:"));
    _ram = this->setControl(1, r, new ui::LabelControl());
    ++r;

    this->setControl(0, r, new ui::LabelControl("Jobs:"));
    _jobs = this->setControl(1, r, new ui::LabelControl());
    ++r;
}

void
MonitorUI::update(const osg::FrameStamp* fs)
{
    if (fs && fs->getFrameNumber() % 5 == 0)
    {
        _ram->setText(Stringify() 
            << (Memory::getProcessPhysicalUsage() / 1048576) << " M / "
            << (Memory::getProcessPrivateUsage() / 1048576) << " M // "
            << (Memory::getProcessPeakPrivateUsage() / 1048576) << " M");

        std::stringstream buf;
        const JobArena::Metrics& m = JobArena::allMetrics();
        int pending = 0, running = 0;
        for (int i = 0; i <= m.maxArenaIndex; ++i)
        {
            if (m.arena(i))
            {
                buf << m.arena(i)->arenaName
                    << " (" << m.arena(i)->concurrency << ") "
                    << m.arena(i)->numJobsRunning
                    << " / "
                    << m.arena(i)->numJobsPending
                    << " // "
                    << m.arena(i)->numJobsCanceled
                    << "\n";
            }
        }
        buf << "ico: " << GLObjectsCompiler::totalJobs();

        _jobs->setText(buf.str());
    }
}
