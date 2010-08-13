/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2010 Pelican Mapping
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
#include <osgEarthUtil/Popups>
#include <osgEarth/HTTPClient>
#include <osgText/Text>
#include <osg/Depth>
#include <osgDB/ReadFile>
#include <osgWidget/ViewerEventHandlers>
#include <OpenThreads/ScopedLock>

using namespace osgEarth;
using namespace osgEarthUtil;
using namespace osgEarth::Symbology;
using namespace OpenThreads;

#define COLOROUT(X) X.r() << "," << X.g() << "," << X.b() << "," << X.a()

// ---------------------------------------------------------------------------

static const PopupSymbol* s_defaultPopupSymbol = new PopupSymbol();

typedef std::map<std::string, osg::ref_ptr<osgText::Font> > FontCache;
static FontCache s_fontCache;
static OpenThreads::Mutex s_fontCacheLock;

static osgText::Font*
loadFont( const std::string& uri )
{
    osgText::Font* font = 0L;
    if ( !uri.empty() )
    {
        ScopedLock<Mutex> lock( s_fontCacheLock );
        FontCache::iterator i = s_fontCache.find( uri );
        if ( i == s_fontCache.end() )
        {
            font = osgText::readFontFile( uri );
            if ( font )
            {
                s_fontCache[ uri ] = font;
            }
        }
        else
        {
            font = i->second.get();
        }
    }
    return font;
}

// ---------------------------------------------------------------------------

// the encoded default theme image for popups. 
// generated with bin2c 
static const unsigned char defaultPopupThemePNG[]={
137, 80, 78, 71, 13, 10, 26, 10,  0,  0,  0, 13, 73, 72, 68, 82,  0,  0,  0, 48,
  0,  0,  0, 48,  8,  6,  0,  0,  0, 87,  2,249,135,  0,  0,  0,  1,115, 82, 71,
 66,  0,174,206, 28,233,  0,  0,  0,  6, 98, 75, 71, 68,  0,255,  0,255,  0,255,
160,189,167,147,  0,  0,  0,  9,112, 72, 89,115,  0,  0, 27,175,  0,  0, 27,175,
  1, 94, 26,145, 28,  0,  0,  0,  7,116, 73, 77, 69,  7,218,  4,  2, 15,  2, 24,
 10,254, 70, 75,  0,  0,  2, 30, 73, 68, 65, 84,104,222,237,154, 49,111,218, 64,
 24,134,191,163,157,106,123,177, 65, 74,  7,102, 75,216, 27, 67,100,  1, 75,116,
 24,150, 10,143,228, 79,192, 80, 10,248, 95,228,167,228,  7, 84,108,216, 82, 27,
162,176,  1,150, 97, 69, 66,222, 92,155,201, 95,134,134,196, 52, 13, 86, 20, 53,
182,171,123, 39, 75, 62,221,189,143,238,211,221,233,222,  3,248,223, 85,175,215,
 51, 61,246,199,164,  6, 97, 24, 26,178, 44, 95, 72,146,116,198,243,252, 39, 68,
252,167,166,  9, 33,224,251,126,224,121,222, 54, 12,195,  9,  0, 92,191,170,  3,
 77,211,  0,  0, 64, 85,213,111,227,241, 24,109,219, 70, 68,140,240,253, 21,217,
182,141,166,105,162,162, 40, 35,  0,128, 70,163,145,108,190,219,237,146,102,179,
121,227, 56,206,239, 94,162, 52,188,227,209,216,174,235, 98,187,221,190,237,245,
122,164, 86,171,157,134,208,117,253, 71, 16,  4, 81,148,166,243,231, 32,209,126,
191,143,116, 93,191, 57,105, 94, 85,213,193, 98,177,192, 44,153,143, 67, 44,151,
 75, 84, 20,101,240, 34,192,112, 56,196,172,203, 52,205,163, 85,228,  3,  0,  0,
165, 20, 68, 81,252,210,239,247, 47,203,229, 50, 16, 66, 50,185,164, 35, 34,112,
 28,  7,243,249,252,174, 82,169, 44,215,235,245,211, 79, 89,150,175, 82, 90,109,
 94, 93, 77,178, 44, 95, 29,124, 23, 14, 31,146, 36,125,  6,  0,146,131,189,149,
 60,120, 61,  6,224,121,158,203,203,233, 32,238,181, 16,175,175,188, 40,238,181,
144,247,179, 26,  3, 96,  0, 12,128,  1, 48,  0,  6,192,  0, 24,  0,  3, 96,  0,
 12,128,  1, 48,  0,  6,192,  0, 24, 64,202,  0, 89,189, 15,253,155,226, 94, 31,
  1,124,223,255,149, 23,128,184,215, 71,  0,207,243,182,  0,144,135,235, 57,124,
240,250,  4, 64, 41,  5, 65, 16,190, 79,167, 83,146,229, 43, 70, 68,  4,219,182,
137, 32,  8, 19, 74,233,243,  6,163,209, 40,119,  1,199,145, 20, 69, 25,172, 86,
171,204, 70, 76,142,227,160,170,170, 95, 79, 78, 83,150, 67,190, 86,171,245,243,
228, 62,160,105, 26,136,162,120,222,233,116,110, 93,215, 37,135,186, 75,179,230,
  1,  0, 54,155, 13, 49, 12,227,174, 84, 42,157, 39,198,172,127,  6,221,150,101,
165, 22,116, 91,150,149, 24,116, 39,238, 94,213,106,213,240,125,255,162, 88, 44,
158,113, 28,247, 46, 79, 13,130, 32,  8,118,187,221,150,231,249,201,108, 54,187,
126, 83,135, 89,127,236,145,123,221,  3,212,  7,119,182, 49,219,150,179,  0,  0,
  0,  0, 73, 69, 78, 68,174, 66, 96,130
};

static osg::ref_ptr<osg::Image> s_defaultPopupThemeImage;

static osg::Image*
getDefaultPopupThemeImage()
{
    if ( !s_defaultPopupThemeImage.valid() )
    {
        osgDB::ReaderWriter* r = osgDB::Registry::instance()->getReaderWriterForExtension( "png" );
        if ( r )
        {
            std::string input( (const char*)defaultPopupThemePNG, sizeof(defaultPopupThemePNG) );
            std::stringstream instr( input );
            osgDB::ReaderWriter::ReadResult result = r->readImage( instr );
            s_defaultPopupThemeImage = result.takeImage();
        }
    }
    return s_defaultPopupThemeImage.get();
}

// ---------------------------------------------------------------------------

PopupSymbol::PopupSymbol() :
_backgroundColor( osg::Vec4f( 0,0,0,0.6 ) ),
_focusBackgroundColor( osg::Vec4f(.5,.5,1,1 ) )
{
    fill()->color().set( .9,.9,.9, 1 );
}

// ---------------------------------------------------------------------------

class PopupNode : public osg::Node
{
public:
    PopupNode()
    {
        setDataVariance(osg::Object::DYNAMIC);  
        setCullingActive(false); 
        _hide = true;
    }

    void setWidget(Popup* widget) {
        _widget = widget;
    }

    const Popup* getWidget() const {
        return _widget.get();
    }

    Popup* getWidget() {
        return _widget.get();
    }

    void accept(osg::NodeVisitor& nv)
    {
        setNumChildrenRequiringUpdateTraversal(1);
        if (nv.validNodeMask(*this)) 
        {
            if (nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR) {
                osg::Vec2 size = osg::Vec2(_widget->getWindow()->getWidth(), _widget->getWindow()->getHeight());
                osg::Vec2 positionCenteredAnchor = _positionOnScreen - size/2;
                _widget->getWindow()->setX(osg::round(positionCenteredAnchor[0]));
                _widget->getWindow()->setY(osg::round(positionCenteredAnchor[1]));
                if (_hide)
                    _widget->getWindowManager()->removeChild(_widget->getWindow());
                else {
                    if (!_widget->getWindowManager()->containsNode(_widget->getWindow()))
                        _widget->getWindowManager()->addChild(_widget->getWindow());
                }

            } else if (nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR) {
                osg::CullStack* cull = dynamic_cast<osg::CullStack*>(&nv);
                if (cull) {

                    std::vector<osg::Vec3> vertices;
                    vertices.push_back(osg::Vec3(0,0,0));
                    if (cull->isCulled(vertices)) {
                        _hide = true;
                        return;
                    } else {
                        _hide = false;
                    }

                    // compute 2d on screen
                    osg::Matrix matrix = *(cull->getModelViewMatrix());
                    matrix *= *(cull->getProjectionMatrix());
                    osg::Vec3 point = matrix.getTrans();



                    point *= 1.0/matrix(3,3); // w
                    float x = cull->getViewport()->width()/2 * (1.0 + point[0]);
                    float y = cull->getViewport()->height()/2 * (1.0 + point[1]);

                    _positionOnScreen = osg::Vec2(x, y);
                }
            }
        }
    }

protected:
    osg::ref_ptr<Popup> _widget;
    osg::Vec2 _positionOnScreen;
    bool _hide;
};

// ---------------------------------------------------------------------------

void
Popup::attach(osg::Group* node)
{
    PopupNode* wn = new PopupNode;
    wn->setWidget(this);
    node->addChild(wn);
}

void
Popup::detach(osg::Group* parent)
{
    for( unsigned int i=0; i<parent->getNumChildren(); ++i )
    {
        PopupNode* p = dynamic_cast<PopupNode*>( parent->getChild(i) );
        if ( p )
        {
            p->setWidget(0L);
            parent->removeChild( i, 1 );
            break;
        }
    }
}

// ---------------------------------------------------------------------------

struct UpdatePositionItems : public osg::NodeCallback
{
    UpdatePositionItems() {}
    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv) {
        if (nv->getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR) {
            osgWidget::WindowManager* wm = dynamic_cast<osgWidget::WindowManager*>(node);
            if (wm)
                wm->resizeAllWindows();
        }
        traverse(node,nv);
    }
};

//PopupManager::PopupManager(osgViewer::Viewer& viewer) : _widgetNumInstance(0)
PopupManager::PopupManager( osgViewer::View* view ) : _widgetNumInstance(0)
{
    const unsigned int MASK_2D = 0xF0000000;
    osgWidget::WindowManager* wm = new osgWidget::WindowManager(
        view,
        view->getCamera()->getGraphicsContext()->getTraits()->width,
        view->getCamera()->getGraphicsContext()->getTraits()->height,
        MASK_2D,
        0 //osgWidget::WindowManager::WM_PICK_DEBUG
        );

    osg::Camera* camera = wm->createParentOrthoCamera();
    view->getSceneData()->asGroup()->addChild(camera);
    view->addEventHandler(new osgWidget::MouseHandler(wm));
    view->addEventHandler(new osgWidget::KeyboardHandler(wm));
    view->addEventHandler(new osgWidget::ResizeHandler(wm, camera));
    view->addEventHandler(new osgWidget::CameraSwitchHandler(wm, camera));
    _windowManager = wm;
    _windowManager->addUpdateCallback(new UpdatePositionItems);
}


TextPopup*
PopupManager::createTextPopup(const std::string& title,
                              const std::string& content,
                              const PopupSymbol* symbol_ )
{
    const PopupSymbol* symbol = symbol_ != 0L ? symbol_ : s_defaultPopupSymbol;

    TextPopup* popup = new TextPopup( symbol );
    //popup->setTitleText( title );
    //popup->setMessageText( content );

    // assigns the popup to a parent window manager.
    popup->setWindowManager(getWindowManager());

    osgWidget::point_type w = _windowManager->getWidth();
    osgWidget::point_type h = _windowManager->getHeight();
    osgWidget::point_type ww = popup->getWindow()->getWidth();
    osgWidget::point_type hw = popup->getWindow()->getHeight();
    osgWidget::point_type ox = (w - ww) / 2;
    osgWidget::point_type oy = (h - hw) / 2;
    
    popup->getWindow()->setPosition(osgWidget::Point(osg::round(ox), osg::round(oy), -(float)(_widgetNumInstance+1)) );
    _widgetNumInstance++;

    return popup;
}

// ---------------------------------------------------------------------------



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
                    if (label && label->getName() == std::string("LabelText")) {
                        label->setFontColor(_textColor);
                    }
                    else if (label && label->getName() == std::string("LabelTitle")) {
                        label->setFontColor(_titleColor);
                    }
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




TextPopup::ColorAnimation::ColorAnimation(osgWidget::Frame* frame) : osgWidget::Callback(osgWidget::EVENT_ALL), _frame(frame)
{
    _motionOver = WidgetMotion(0.0, 0.4);
    _motionLeave = WidgetMotion(0.0, 0.5);
    _defaultColor = _frame->getEmbeddedWindow()->getColor();
    _overColor = osgWidget::Color(1.0, 1.0, 1.0, 1.0);
    _over  = false;
}
void TextPopup::ColorAnimation::setTitleColor(const osg::Vec4& color) { _titleColor = color; }
void TextPopup::ColorAnimation::setTextColor(const osg::Vec4& color) { _textColor = color; }
void TextPopup::ColorAnimation::setSrcColor(const osg::Vec4& color) { _defaultColor = color; }
void TextPopup::ColorAnimation::setDstColor(const osg::Vec4& color) { _overColor = color; }

bool TextPopup::ColorAnimation::operator()(osgWidget::Event& ev)
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

void TextPopup::ColorAnimation::operator()(osg::Node* node, osg::NodeVisitor* nv)
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

//void TextPopup::setColor(const osg::Vec4& color)
//{
//    _color = color;
//    _colorAnimation->setSrcColor(_color);
//}
//
//void TextPopup::setFocusColor(const osg::Vec4& color)
//{
//    _focusColor = color;
//    _colorAnimation->setDstColor(_focusColor);
//}

void TextPopup::setTitleText(const std::string& text)
{
    if (_labelTitle.valid())
    {
        _labelTitle->setLabel(text);
    }
}

void TextPopup::setMessageText(const std::string& text)
{
    if (_labelText.valid())
    {
        _labelText->setLabel(text);
    }
}

TextPopup::TextPopup( const PopupSymbol* symbol )
{
    // load up the theme image, using the default if one is not set.
    osg::ref_ptr<osg::Image> themeImage;
    if ( symbol->theme().isSet() )
        HTTPClient::readImageFile( symbol->theme().value(), themeImage );
    else
        themeImage = getDefaultPopupThemeImage();

    // read the font (from the cache if possible)
    osg::ref_ptr<osgText::Font> font;
    if ( !symbol->font()->empty() )
        font = loadFont( symbol->font().value() );

    //if ( font.valid() )
    //    OE_INFO << "GOT FONT: " << font->getFileName() << std::endl;

    osg::ref_ptr<osgWidget::Frame> frame = osgWidget::Frame::createSimpleFrameFromTheme(
        "PopUp",
        themeImage.get(),
        300.0f,
        50.0f,
        osgWidget::Frame::FRAME_ALL );

    // temporarily disable rollover animation -gw
    frame->getBackground()->setColor(0, 0, 0, 0);
    frame->setEventMask( osgWidget::EVENT_NONE );

    osgWidget::Color adjustAlpha = frame->getEmbeddedWindow()->getColor();

    _titleColor = symbol->fill()->color();
    _textColor = symbol->fill()->color();
    float fontSize = symbol->size().value();

    _labelText = createLabel("Placeholder Message", font.get(), fontSize, _textColor, adjustAlpha);
    _labelText->setName("LabelText");

    _labelTitle = createLabel("Placeholder Title", font.get(), fontSize+5, _titleColor, adjustAlpha);
    _labelTitle->setName("LabelTitle");

    osgWidget::Box* box   = new osgWidget::Box("VBOX", osgWidget::Box::VERTICAL);

    //OE_NOTICE << "TItle color=" << COLOROUT(_titleColor) << std::endl;
    //OE_NOTICE << "adustALpha =" << COLOROUT(adjustAlpha) << std::endl;

    _labelTitle->setPadBottom(4.0f);
    _labelTitle->setWidth(_labelTitle->getTextSize()[0]);
    _labelTitle->setHeight(_labelTitle->getTextSize()[1]);
    _labelTitle->setColor( _titleColor );

    _labelText->setPadBottom(0.0f);
    _labelText->setWidth(_labelText->getTextSize()[0]);
    _labelText->setHeight(_labelText->getTextSize()[1]);
    _labelText->setColor( _textColor );

    box->addWidget( _labelText.get() );
    box->addWidget( _labelTitle.get() );
   
    _labelText->setEventMask(osgWidget::EVENT_NONE);
    _labelTitle->setEventMask(osgWidget::EVENT_NONE);
    
    box->setEventMask( osgWidget::EVENT_NONE );

    frame->setDataVariance(osg::Object::DYNAMIC);
    frame->getCorner(osgWidget::Frame::CORNER_LOWER_LEFT)->setEventMask(osgWidget::EVENT_NONE);    
    frame->getCorner(osgWidget::Frame::CORNER_LOWER_RIGHT)->setEventMask(osgWidget::EVENT_NONE);
    frame->getCorner(osgWidget::Frame::CORNER_UPPER_LEFT)->setEventMask(osgWidget::EVENT_NONE);
    frame->getCorner(osgWidget::Frame::CORNER_UPPER_RIGHT)->setEventMask(osgWidget::EVENT_NONE);
    frame->getBorder(osgWidget::Frame::BORDER_LEFT)->setEventMask(osgWidget::EVENT_NONE);
    frame->getBorder(osgWidget::Frame::BORDER_RIGHT)->setEventMask(osgWidget::EVENT_NONE);
    frame->getBorder(osgWidget::Frame::BORDER_TOP)->setEventMask(osgWidget::EVENT_NONE);
    frame->getBorder(osgWidget::Frame::BORDER_BOTTOM)->setEventMask(osgWidget::EVENT_NONE);

    // copy the frame bg to the box
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

    _colorAnimation->setTitleColor( _titleColor );
    _colorAnimation->setTextColor( _textColor );
    _colorAnimation->setSrcColor( symbol->backgroundColor().value() );
    _colorAnimation->setDstColor( symbol->focusBackgroundColor().value() );

    getWindow()->addUpdateCallback(_colorAnimation);
    getWindow()->addUpdateCallback(_appearAnimation);
    getWindow()->addCallback(_colorAnimation);
    osg::ref_ptr<AlphaSetterVisitor2> alphaSetter = new AlphaSetterVisitor2(0, true);
    getWindow()->accept(*alphaSetter);
    alphaSetter = 0;
}

osgWidget::Label* TextPopup::createLabel(const std::string& string, 
                                         osgText::Font* font, 
                                         int size, 
                                         const osgWidget::Color& color, 
                                         const osgWidget::Color& widgetColor)
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

osgWidget::Frame* TextPopup::getWindow()
{ 
    return _window.get();
}

TextPopup::ColorAnimationFadeInFadeOut::ColorAnimationFadeInFadeOut(osgWidget::Frame* frame) : _frame(frame)
{
    _motionOver = WidgetMotion(0.0, 0.4);
    _motionLeave = WidgetMotion(0.0, 0.5);
    _initialized = false;
    _srcAlpha = 0.0;
    _dstAlpha = 1.0;
    _lastUpdate = -1.0;
    _appear = true;
}

void TextPopup::ColorAnimationFadeInFadeOut::setSrcAlpha(float alpha) { _srcAlpha = alpha; }

void TextPopup::ColorAnimationFadeInFadeOut::setDstAlpha(float alpha) { _dstAlpha = alpha; }

void TextPopup::ColorAnimationFadeInFadeOut::operator()(osg::Node* node, osg::NodeVisitor* nv)
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

void TextPopup::ColorAnimationFadeInFadeOut::setAppear()
{ 
    _appear = true;
    _motionLeave.reset();
    _motionOver.reset(); 
    _initialized = true;
}

void TextPopup::ColorAnimationFadeInFadeOut::setDisappear()
{
    _appear = false;
    _motionLeave.reset();
    _motionOver.reset();
    _initialized = true;
}

void TextPopup::setAppear() 
{
    if (_appearAnimation.valid())
        _appearAnimation->setAppear();
}
void TextPopup::setDisappear()
{
    if (_appearAnimation.valid())
        _appearAnimation->setDisappear();
}
