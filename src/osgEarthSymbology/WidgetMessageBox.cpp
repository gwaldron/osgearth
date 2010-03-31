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

#include <osgEarthSymbology/WidgetMessageBox>
#include <osgWidget/Box>
#include <osgDB/ReadFile>
#include <osgAnimation/EaseMotion>
#include <osg/NodeVisitor>

using namespace osgEarth::Symbology;

struct AlphaSetterVisitor : public osg::NodeVisitor
{
    float _alpha;
    AlphaSetterVisitor( float alpha = 1.0):osg::NodeVisitor(TRAVERSE_ALL_CHILDREN) { _alpha = alpha;}

    void apply(osg::MatrixTransform& node)
    {
        osgWidget::Window* win = dynamic_cast<osgWidget::Window*>(&node);

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
                if (win->getName().find("PopUp") == std::string::npos)
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
            //std::cout << "enter" << std::endl;
            return true;
        }
        else if (ev.type == osgWidget::EVENT_MOUSE_LEAVE) 
        {
            _over = false;
            _motionLeave.reset();
            //std::cout << "leave" << std::endl;
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


bool WidgetMessageBox::create(osg::Image* themeMessage,
                              const std::string& titleText,
                              const std::string& messageText,
                              const std::string& buttonText,
                              osgText::Font* font,
                              int fontSize)
{

    osg::ref_ptr<osgWidget::Frame> frame = osgWidget::Frame::createSimpleFrameFromTheme(
        "PopUp",
        themeMessage,
        300.0f,
        50.0f,
        osgWidget::Frame::FRAME_ALL
        );
    frame->getBackground()->setColor(0.0f, 0.0f, 0.0f, 0.0f);
    osgWidget::Color adjustAlpha = frame->getEmbeddedWindow()->getColor();

    osgWidget::Label* labelText = createLabel(messageText, font, fontSize, osgWidget::Color(0,0,0,1), adjustAlpha);
    osgWidget::Label* labelTitle = createLabel(titleText, font, fontSize+5, osgWidget::Color(0.4,0,0,1), adjustAlpha);

    osgWidget::Box*   box   = new osgWidget::Box("VBOX", osgWidget::Box::VERTICAL);

    labelTitle->setPadBottom(30.0f);
    labelText->setPadBottom(30.0f);
    labelText->getText()->setMaximumWidth(500);
    labelText->setLabel(messageText);

    labelText->setWidth(labelText->getTextSize()[0]);
    labelText->setHeight(labelText->getTextSize()[1]);

    box->addWidget(labelText);
    box->addWidget(labelTitle);

    labelText->setEventMask(osgWidget::EVENT_NONE);
    labelTitle->setEventMask(osgWidget::EVENT_NONE);
    box->setEventMask(osgWidget::EVENT_NONE);

    frame->getCorner(osgWidget::Frame::CORNER_LOWER_LEFT)->setEventMask(osgWidget::EVENT_NONE);    
    frame->getCorner(osgWidget::Frame::CORNER_LOWER_RIGHT)->setEventMask(osgWidget::EVENT_NONE);
    frame->getCorner(osgWidget::Frame::CORNER_UPPER_LEFT)->setEventMask(osgWidget::EVENT_NONE);
    frame->getCorner(osgWidget::Frame::CORNER_UPPER_RIGHT)->setEventMask(osgWidget::EVENT_NONE);
    frame->getBorder(osgWidget::Frame::BORDER_LEFT)->setEventMask(osgWidget::EVENT_NONE);
    frame->getBorder(osgWidget::Frame::BORDER_RIGHT)->setEventMask(osgWidget::EVENT_NONE);
    frame->getBorder(osgWidget::Frame::BORDER_TOP)->setEventMask(osgWidget::EVENT_NONE);
    frame->getBorder(osgWidget::Frame::BORDER_BOTTOM)->setEventMask(osgWidget::EVENT_NONE);

    osgWidget::Color colorBack = frame->getEmbeddedWindow()->getColor();
    box->getBackground()->setColor(colorBack);

    frame->setWindow(box);

    box->resize();
    frame->resizeFrame(box->getWidth(), box->getHeight());

    _window = frame;
    return true;
}

osgWidget::Label* WidgetMessageBox::createLabel(const std::string& string, osgText::Font* font, int size, const osgWidget::Color& color, const osgWidget::Color& widgetColor)
{
    osgWidget::Label* label = new osgWidget::Label("label", "");
    label->getText()->setFont(font);
    label->setFontSize(size);
    label->setFontColor(color);
    label->setColor(widgetColor);
    label->setLabel(string);
    label->setCanFill(true);
    label->setDataVariance(osg::Object::DYNAMIC);
    return label;
}

#if 0
WidgetMessageBox WidgetMessageBox::popUp(const std::string& title, const std::string& text, const std::string exit)
{
    int fontSize = 16;

    WidgetMessageBox message;
    message.create(osgDB::readImageFile("../data/theme-8.png"),
                   title,
                   text,
                   exit,
                   osgText::readFontFile("/home/mornifle/dev/easy3d/easy3d-data/fonts/Vera.ttf"),
                   fontSize);

    AlphaSetterVisitor alpha(.7f);
    message.getWindow()->accept(alpha);

    EventOK* event = new EventOK(message.getWindow());
    message.getWindow()->setUpdateCallback(event);
    message.getWindow()->addCallback(event);

    return message;
}

#endif

WidgetMessageBox WidgetMessageBox::popUp(osg::Image* themeMessage, const std::string& title, const std::string& text, const std::string exit, osgText::Font* font, int fontSize )
{
    WidgetMessageBox message;
    message.create(themeMessage,
                   title,
                   text,
                   exit,
                   font,
                   fontSize);

    AlphaSetterVisitor alpha(.7f);
    message.getWindow()->accept(alpha);

    EventOK* event = new EventOK(message.getWindow());
    message.getWindow()->setUpdateCallback(event);
    message.getWindow()->addCallback(event);

    return message;
}

osgWidget::Frame* WidgetMessageBox::getButton() { return _button.get(); }
osgWidget::Frame* WidgetMessageBox::getWindow() { return _window.get(); }
