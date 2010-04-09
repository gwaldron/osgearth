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

#include <osgEarthUtil/WidgetMessageBox>
#include <osgWidget/Box>
#include <osgDB/ReadFile>
#include <osgAnimation/EaseMotion>
#include <osg/NodeVisitor>

using namespace osgEarthUtil;

struct SetDataVariance : public osg::NodeVisitor
{
    SetDataVariance() : osg::NodeVisitor(TRAVERSE_ALL_CHILDREN) {}
    void apply(osg::Node& node) { 
        node.setDataVariance(osg::Object::DYNAMIC); 
        traverse(node);
    }
};

class AlphaSetterVisitor : public osg::NodeVisitor
{
protected:
    float _alpha;
    bool _applyToText;

    ~AlphaSetterVisitor() {}

public:
    AlphaSetterVisitor( float alpha = 1.0, bool applyToText = false) : osg::NodeVisitor(TRAVERSE_ALL_CHILDREN), osg::Referenced(true) { _alpha = alpha; _applyToText = applyToText;}

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
                    if (_applyToText) {
                        osgWidget::Label* label = dynamic_cast<osgWidget::Label*>((*it).get());
                        osg::Vec4 c = label->getText()->getColor();
                        c[3] *= _alpha;
                        label->setFontColor(c);
                    }
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
    osgWidget::Color _titleColor;
    osgWidget::Color _textColor;
    ColorSetterVisitor( const osgWidget::Color& color,
                        const osgWidget::Color& titleColor,
                        const osgWidget::Color& textColor): _color(color), _titleColor(titleColor), _textColor(textColor),
        osg::NodeVisitor(TRAVERSE_ALL_CHILDREN) {;}

    void apply(osg::MatrixTransform& node)
    {
        osgWidget::Window* win = dynamic_cast<osgWidget::Window*>(&node);

        if (win) {
            for (osgWidget::Window::Iterator it = win->begin(); it != win->end(); it++)
            {
                osgWidget::Color color = _color;
                if (! dynamic_cast<osgWidget::Label*>((*it).get())) {
                    it->get()->setColor(_color);
                } else {
                    osgWidget::Label* label = dynamic_cast<osgWidget::Label*>((*it).get());
                    if (label && label->getName() == std::string("LabelText"))
                        label->setFontColor(_textColor);
                    else if (label && label->getName() == std::string("LabelTitle"))
                        label->setFontColor(_titleColor);
                }
            }
            {
                if (win->getName().find("PopUp") == std::string::npos)
                    win->getBackground()->setColor(_color);
            }
        }
        traverse(node);
    }
};




WidgetMessageBox::ColorAnimation::ColorAnimation(osgWidget::Frame* frame) : osgWidget::Callback(osgWidget::EVENT_ALL), _frame(frame)
{
    _motionOver = WidgetMotion(0.0, 0.4);
    _motionLeave = WidgetMotion(0.0, 0.5);
    _defaultColor = _frame->getEmbeddedWindow()->getColor();
    _overColor = osgWidget::Color(1.0, 1.0, 1.0, 1.0);
    _over  = false;
}
void WidgetMessageBox::ColorAnimation::setTitleColor(const osg::Vec4& color) { _titleColor = color; }
void WidgetMessageBox::ColorAnimation::setTextColor(const osg::Vec4& color) { _textColor = color; }
void WidgetMessageBox::ColorAnimation::setSrcColor(const osg::Vec4& color) { _defaultColor = color; }
void WidgetMessageBox::ColorAnimation::setDstColor(const osg::Vec4& color) { _overColor = color; }

bool WidgetMessageBox::ColorAnimation::operator()(osgWidget::Event& ev)
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

void WidgetMessageBox::ColorAnimation::operator()(osg::Node* node, osg::NodeVisitor* nv)
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
            ColorSetterVisitor colorSetter(c, _titleColor, _textColor);
            _frame->accept(colorSetter);
        }
    }
    traverse(node, nv);
}


class AlphaSetterVisitor2 : public osg::NodeVisitor
{
protected:
    float _alpha;
    bool _applyToText;

    ~AlphaSetterVisitor2() {}

public:
    AlphaSetterVisitor2( float alpha = 1.0, bool applyToText = false) : osg::NodeVisitor(TRAVERSE_ALL_CHILDREN), osg::Referenced(true) { _alpha = alpha; _applyToText = applyToText;}

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
                    if (_applyToText) {
                        osgWidget::Label* label = dynamic_cast<osgWidget::Label*>((*it).get());
                        osg::Vec4 c = label->getText()->getColor();
                        c[3] *= _alpha;
                        label->setFontColor(c);
                    }
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

void WidgetMessageBox::setColor(const osg::Vec4& color)
{
    _color = color;
    _colorAnimation->setSrcColor(_color);
}

void WidgetMessageBox::setFocusColor(const osg::Vec4& color)
{
    _focusColor = color;
    _colorAnimation->setDstColor(_focusColor);
}

void WidgetMessageBox::setTitleText(const std::string& text)
{
    if (_labelTitle.valid())
        _labelTitle->setLabel(text);
}

void WidgetMessageBox::setMessageText(const std::string& text)
{
    if (_labelText.valid())
        _labelText->setLabel(text);
}

bool WidgetMessageBox::create(osg::Image* themeMessage,
                              const std::string& titleText,
                              const std::string& messageText,
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
    frame->setDataVariance(osg::Object::DYNAMIC);

    osgWidget::Color adjustAlpha = frame->getEmbeddedWindow()->getColor();

    _titleColor = osgWidget::Color(0.4,0,0,1);
    _textColor = osgWidget::Color(0,0,0,1);

    _labelText = createLabel(messageText, font, fontSize, _textColor, adjustAlpha);
    _labelText->setName("LabelText");

    _labelTitle = createLabel(titleText, font, fontSize+5, _titleColor, adjustAlpha);
    _labelTitle->setName("LabelTitle");

    osgWidget::Box*   box   = new osgWidget::Box("VBOX", osgWidget::Box::VERTICAL);

    _labelTitle->setPadBottom(10.0f);
    _labelText->setPadBottom(0.0f);
    _labelText->setLabel(messageText);

    _labelText->setWidth(_labelText->getTextSize()[0]);
    _labelText->setHeight(_labelText->getTextSize()[1]);

    box->addWidget(_labelText.get());
    box->addWidget(_labelTitle.get());

    _labelText->setEventMask(osgWidget::EVENT_NONE);
    _labelTitle->setEventMask(osgWidget::EVENT_NONE);
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
    SetDataVariance setToDynamic;
    _window->accept(setToDynamic);

    float alpha = 0.8;
    _color = osg::Vec4(0.4, 0.8, 0.8, alpha);

    _colorAnimation = new ColorAnimation(getWindow());
    _appearAnimation = new ColorAnimationFadeInFadeOut(getWindow());

    _colorAnimation->setTitleColor(_titleColor);
    _colorAnimation->setTextColor(_textColor);
    getWindow()->addUpdateCallback(_colorAnimation);
    getWindow()->addUpdateCallback(_appearAnimation);
    getWindow()->addCallback(_colorAnimation);


    osg::ref_ptr<AlphaSetterVisitor2> alphaSetter = new AlphaSetterVisitor2(0, true);
    getWindow()->accept(*alphaSetter);
    alphaSetter = 0;
//    setAppear();

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

osgWidget::Frame* WidgetMessageBox::getWindow() { return _window.get(); }
WidgetMessageBox::WidgetMessageBox() : _colorAnimation(0) {}


WidgetMessageBox::ColorAnimationFadeInFadeOut::ColorAnimationFadeInFadeOut(osgWidget::Frame* frame) : _frame(frame)
{
    _motionOver = WidgetMotion(0.0, 0.4);
    _motionLeave = WidgetMotion(0.0, 0.5);
    _initialized = false;
    _srcAlpha = 0.0;
    _dstAlpha = 1.0;
    _lastUpdate = -1.0;
    _appear = true;
}

void WidgetMessageBox::ColorAnimationFadeInFadeOut::setSrcAlpha(float alpha) { _srcAlpha = alpha; }
void WidgetMessageBox::ColorAnimationFadeInFadeOut::setDstAlpha(float alpha) { _dstAlpha = alpha; }

void WidgetMessageBox::ColorAnimationFadeInFadeOut::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
    if (nv->getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR)
    {
        const osg::FrameStamp* fs = nv->getFrameStamp();
        float alpha = 0;
        if (_initialized)
        {
            if (_lastUpdate < 0)
                _lastUpdate = fs->getSimulationTime();

            double dt = fs->getSimulationTime() - _lastUpdate;
            _lastUpdate = fs->getSimulationTime();

            float value;
            if (_appear)
            {
                _motionOver.update(dt);
                value = _motionOver.getValue();
            }
            else
            {
                _motionLeave.update(dt);
                value = 1.0 - _motionLeave.getValue();
            }

            alpha = _srcAlpha + ((_dstAlpha - _srcAlpha) * value);
        }
        osg::ref_ptr<AlphaSetterVisitor2> alphaSetter = new AlphaSetterVisitor2(alpha, true);
        _frame->accept(*alphaSetter);
    }
    traverse(node, nv);
}

void WidgetMessageBox::ColorAnimationFadeInFadeOut::setAppear() { _appear = true; _motionLeave.reset(); _motionOver.reset(); _initialized = true;}
void WidgetMessageBox::ColorAnimationFadeInFadeOut::setDisappear() { _appear = false;  _motionLeave.reset(); _motionOver.reset();  _initialized = true;}

void WidgetMessageBox::setAppear() 
{
    if (_appearAnimation.valid())
        _appearAnimation->setAppear();
}
void WidgetMessageBox::setDisappear()
{
    if (_appearAnimation.valid())
        _appearAnimation->setDisappear();
}
