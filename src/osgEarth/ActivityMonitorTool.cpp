/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */

#include <osgEarth/ActivityMonitorTool>
#include <osgEarth/Registry>

using namespace osgEarth;
using namespace osgEarth::Contrib;
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
