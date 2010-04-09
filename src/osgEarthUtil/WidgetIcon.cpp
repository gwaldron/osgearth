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

#include <osgEarthUtil/WidgetIcon>
#include <osgWidget/Box>
#include <osgDB/ReadFile>
#include <osgAnimation/EaseMotion>
#include <osg/NodeVisitor>

using namespace osgEarthUtil;

struct AlphaSetterVisitor : public osg::NodeVisitor
{
    float _alpha;
    AlphaSetterVisitor( float alpha = 1.0):osg::NodeVisitor(TRAVERSE_ALL_CHILDREN) { _alpha = alpha;}

    void apply(osg::MatrixTransform& node)
    {
        osgWidget::Window* win = dynamic_cast<osgWidget::Window*>(&node);
        #if 0
        if (win) {

            for (osgWidget::Window::Iterator it = win->begin(); it != win->end(); it++)
            {
                osgWidget::Color color = it->get()->getColor();
                if (! dynamic_cast<osgWidget::Label*>((*it).get())) {
                    color[3] = color[3] *_alpha;
                    it->get()->setColor(color);
                } else {
                    color[3] = 0.0;
                    it->get()->setColor(color);
                }
            }
            {
                osgWidget::Color color = win->getBackground()->getColor();
                color[3] = color[3] *_alpha;
                win->getBackground()->setColor(color);
            }
        }
#endif
        traverse(node);
    }
};

struct ColorSetterVisitor : public osg::NodeVisitor
{
    osgWidget::Color _color;
    ColorSetterVisitor( const osgWidget::Color& color):osg::NodeVisitor(TRAVERSE_ALL_CHILDREN) { _color = color;}

    void apply(osg::MatrixTransform& node)
    {
        osgWidget::Window* win = dynamic_cast<osgWidget::Window*>(&node);

        if (win) {
            for (osgWidget::Window::Iterator it = win->begin(); it != win->end(); it++)
            {
                osgWidget::Color color = _color;
                if (! dynamic_cast<osgWidget::Label*>((*it).get())) {
                    it->get()->setColor(_color);
                }
            }
            {
                //win->getBackground()->setColor(osgWidget::Color(0,0,0,0));
                if (win->getName().find("FrameIcon") == std::string::npos)
                    win->getBackground()->setColor(_color);
            }
        }
        traverse(node);
    }
};


struct EventOK : public osgWidget::Callback, osg::NodeCallback
{
    typedef osgAnimation::OutCubicMotion WidgetMotion;
    WidgetMotion _motionOver;
    WidgetMotion _motionLeave;
    
    double _lastUpdate;
    osgWidget::Color _defaultColor;
    osgWidget::Color _overColor;
    bool _over;
    osg::ref_ptr<osgWidget::Frame> _frame;
    float _width;
    float _height;
    EventOK(osgWidget::Frame* frame) : osgWidget::Callback(osgWidget::EVENT_ALL), _frame(frame) 
    {
        _motionOver = WidgetMotion(0.0, 0.4);
        _motionLeave = WidgetMotion(0.0, 0.5);
        _defaultColor = _frame->getEmbeddedWindow()->getColor();
        _overColor = osgWidget::Color(229.0/255.0,
                                      103.0/255.0,
                                      17.0/255,
                                      _defaultColor[3]);
        _over  = false;
    }

    bool operator()(osgWidget::Event& ev)
    {
        if (ev.type == osgWidget::EVENT_MOUSE_ENTER)
        {
            _over = true;
            _width = _frame->getWidth();
            _height = _frame->getHeight();
            _motionOver.reset();
            return true;
        }
        else if (ev.type == osgWidget::EVENT_MOUSE_LEAVE) 
        {
            _over = false;
            _motionLeave.reset();
            return true;
        }
        return false;
    }

    void operator()(osg::Node* node, osg::NodeVisitor* nv)
    {
        if (nv->getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR)
        {
            const osg::FrameStamp* fs = nv->getFrameStamp();
            double dt = fs->getSimulationTime() - _lastUpdate;
            _lastUpdate = fs->getSimulationTime();

            if (_frame.valid())
            {
                float value;
                if (_over)
                {
                    _motionOver.update(dt);
                    value = _motionOver.getValue();
                }
                else
                {
                    _motionLeave.update(dt);
                    value = 1.0 - _motionLeave.getValue();
                }

                osgWidget::Color c = _defaultColor + ((_overColor - _defaultColor) * value);
                ColorSetterVisitor colorSetter(c);
                _frame->accept(colorSetter);
            }
        }
        node->traverse(*nv);
    }
};

bool WidgetIcon::create(osg::Image* icon)
{
    return createWithBorder(0, icon);
}

bool WidgetIcon::createWithBorder(osg::Image* themeMessage,
                                  osg::Image* icon)
{
    
    osg::ref_ptr<osgWidget::Widget> iconWidget;
    iconWidget = new osgWidget::Widget("Icon",48, 48);
    iconWidget->setImage(icon, true, true);
    iconWidget->setEventMask(osgWidget::EVENT_NONE);

    osg::ref_ptr<osgWidget::Box> box = new osgWidget::Box("BoxIcon");
    box->addWidget(iconWidget.get());
    box->resize();

    osg::ref_ptr<osgWidget::Frame> window;

    if (themeMessage)
    {
        box->setEventMask(osgWidget::EVENT_NONE);
        osgWidget::Frame* frame = osgWidget::Frame::createSimpleFrameFromTheme(
            "FrameIcon",
            themeMessage,
            300.0f,
            50.0f,
            osgWidget::Frame::FRAME_ALL
            );

        osgWidget::Color colorBack = frame->getEmbeddedWindow()->getColor();
        box->getBackground()->setColor(colorBack);
        frame->getBackground()->setColor(0.0f, 0.0f, 0.0f, 0.0f);

        frame->getCorner(osgWidget::Frame::CORNER_LOWER_LEFT)->setEventMask(osgWidget::EVENT_NONE);
        frame->getCorner(osgWidget::Frame::CORNER_LOWER_RIGHT)->setEventMask(osgWidget::EVENT_NONE);
        frame->getCorner(osgWidget::Frame::CORNER_UPPER_LEFT)->setEventMask(osgWidget::EVENT_NONE);
        frame->getCorner(osgWidget::Frame::CORNER_UPPER_RIGHT)->setEventMask(osgWidget::EVENT_NONE);
        frame->getBorder(osgWidget::Frame::BORDER_LEFT)->setEventMask(osgWidget::EVENT_NONE);
        frame->getBorder(osgWidget::Frame::BORDER_RIGHT)->setEventMask(osgWidget::EVENT_NONE);
        frame->getBorder(osgWidget::Frame::BORDER_TOP)->setEventMask(osgWidget::EVENT_NONE);
        frame->getBorder(osgWidget::Frame::BORDER_BOTTOM)->setEventMask(osgWidget::EVENT_NONE);

        frame->setWindow(box.get());
        window = frame;
        frame->resizeFrame(box->getWidth(), box->getHeight());

    } else {
        osgWidget::Frame* frame = osgWidget::Frame::createSimpleFrame(
            "FrameIcon",
            0.0f,
            0.0f,
            icon->s(),
            icon->t(),
            osgWidget::Frame::FRAME_RESIZE
            );

        frame->getBackground()->setColor(0.0f, 0.0f, 0.0f, 0.0f);
        osgWidget::Color adjustAlpha = frame->getEmbeddedWindow()->getColor();
        frame->getEmbeddedWindow()->setColor(osgWidget::Color(0,0,0,0.0));

        frame->getCorner(osgWidget::Frame::CORNER_LOWER_LEFT)->setEventMask(osgWidget::EVENT_NONE);
        frame->getCorner(osgWidget::Frame::CORNER_LOWER_RIGHT)->setEventMask(osgWidget::EVENT_NONE);
        frame->getCorner(osgWidget::Frame::CORNER_UPPER_LEFT)->setEventMask(osgWidget::EVENT_NONE);
        frame->getCorner(osgWidget::Frame::CORNER_UPPER_RIGHT)->setEventMask(osgWidget::EVENT_NONE);
        frame->getBorder(osgWidget::Frame::BORDER_LEFT)->setEventMask(osgWidget::EVENT_NONE);
        frame->getBorder(osgWidget::Frame::BORDER_RIGHT)->setEventMask(osgWidget::EVENT_NONE);
        frame->getBorder(osgWidget::Frame::BORDER_TOP)->setEventMask(osgWidget::EVENT_NONE);
        frame->getBorder(osgWidget::Frame::BORDER_BOTTOM)->setEventMask(osgWidget::EVENT_NONE);

        box->getBackground()->setColor(0,0,0,0);

        frame->setWindow(box.get());
        window = frame;
        frame->resizeFrame(box->getWidth(), box->getHeight());
    }
    _window = window;

    AlphaSetterVisitor alpha(.7f);
    getWindow()->accept(alpha);

    // EventOK* event = new EventOK(getWindow());
    // getWindow()->setUpdateCallback(event);
    // getWindow()->addCallback(event);

    return true;
}

osgWidget::Frame* WidgetIcon::getWindow() { return _window.get(); }
