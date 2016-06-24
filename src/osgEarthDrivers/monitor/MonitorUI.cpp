/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2015 Pelican Mapping
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

using namespace osgEarth::Monitor;
using namespace osgEarth;
using namespace osgEarth::Util;

namespace ui = osgEarth::Util::Controls;

#define LC "[MonitorUI] "

MonitorUI::MonitorUI()
{
    this->setHorizAlign(ALIGN_RIGHT);
    this->setVertAlign(ALIGN_BOTTOM);

    int r=0;
    this->setControl(0, r, new ui::LabelControl("Current Mem:"));
    _pb = new ui::LabelControl();
    _pb->setHorizAlign(ALIGN_RIGHT);
    this->setControl(1, r, _pb.get());

    ++r;
    this->setControl(0, r, new ui::LabelControl("Peak Mem:"));
    _peak = new ui::LabelControl();
    _peak->setHorizAlign(ALIGN_RIGHT);
    this->setControl(1, r, _peak.get());
}

void
MonitorUI::update(const osg::FrameStamp* fs)
{
    if (fs && fs->getFrameNumber() % 60 == 0)
    {
        unsigned bytes = Memory::getProcessUsage();
        _pb->setText(Stringify() << (bytes / 1048576) << " M");
        _peak->setText(Stringify() << (Memory::getProcessPeakUsage() / 1048576) << " M");
    }
}
