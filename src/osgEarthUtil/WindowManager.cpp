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

#include <osgEarthUtil/WindowManager>
#include <osgEarthUtil/WidgetMessageBox>
#include <osgWidget/ViewerEventHandlers>
#include <osgViewer/Viewer>
#include <osgDB/ReadFile>

using namespace osgEarthUtil;

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

WindowManager::WindowManager(osgViewer::Viewer& viewer) : _widgetNumInstance(0)
{
    const unsigned int MASK_2D = 0xF0000000;
    osgWidget::WindowManager* wm = new osgWidget::WindowManager(
        &viewer,
        viewer.getCamera()->getGraphicsContext()->getTraits()->width,
        viewer.getCamera()->getGraphicsContext()->getTraits()->height,
        MASK_2D,
        0 //osgWidget::WindowManager::WM_PICK_DEBUG
        );
    osg::Camera* camera = wm->createParentOrthoCamera();
    viewer.getSceneData()->asGroup()->addChild(camera);
    viewer.addEventHandler(new osgWidget::MouseHandler(wm));
    viewer.addEventHandler(new osgWidget::KeyboardHandler(wm));
    viewer.addEventHandler(new osgWidget::ResizeHandler(wm, camera));
    viewer.addEventHandler(new osgWidget::CameraSwitchHandler(wm, camera));
    _windowManager = wm;
    _windowManager->addUpdateCallback(new UpdatePositionItems);
}


WidgetMessageBox* WindowManager::createWidgetMessageBox(const std::string& title, const std::string& content, const osgEarth::Symbology::TextSymbol* symbol)
{
    if (!symbol)
        return 0;
    osg::Image* image = 0;
    if (!symbol->theme()->empty())
        image = osgDB::readImageFile(symbol->theme().value());

    osgText::Font* font = 0;
    if (!symbol->font()->empty())
        font = osgText::readFontFile(symbol->font().value());

    float size = symbol->size().value();
    osg::Vec4 color = symbol->fill()->color();

    WidgetMessageBox* wmb = new WidgetMessageBox;
    wmb->create(image,
                title,
                content,
                font,
                size);
    wmb->setColor(color);
    color[3] = 1.0;
    wmb->setFocusColor(color);
    wmb->setWindowManager(getWindowManager());

    osgWidget::point_type w = _windowManager->getWidth();
    osgWidget::point_type h = _windowManager->getHeight();
    osgWidget::point_type ww = wmb->getWindow()->getWidth();
    osgWidget::point_type hw = wmb->getWindow()->getHeight();
    osgWidget::point_type ox = (w - ww) / 2;
    osgWidget::point_type oy = (h - hw) / 2;
    
    wmb->getWindow()->setPosition(osgWidget::Point(osg::round(ox), osg::round(oy), -(float)(_widgetNumInstance+1)) );
    _widgetNumInstance++;
    return wmb;
}
