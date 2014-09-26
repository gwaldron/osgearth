/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2014 Pelican Mapping
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

#include <osgEarthUtil/ActivityMonitorTool>
#include <osgEarth/Registry>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;

//-----------------------------------------------------------------------

ActivityMonitorTool::ActivityMonitorTool(VBox* vbox) :
_vbox( vbox )
{
    //nop
}

bool
ActivityMonitorTool::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
    if (ea.getEventType() == ea.FRAME)
    {
        osg::ref_ptr<VBox> vbox;
        if ( _vbox.lock(vbox) )
        {
            std::set<std::string> activity;
            Registry::instance()->getActivities(activity);
            if ( activity != _prev )
            {            
                _vbox->clearControls();
                for(std::set<std::string>::const_iterator i = activity.begin(); i != activity.end(); ++i)
                {
                    _vbox->addControl( new LabelControl(*i) );
                }
                _prev = activity;
            }
        }        
    }

    return false;
}
