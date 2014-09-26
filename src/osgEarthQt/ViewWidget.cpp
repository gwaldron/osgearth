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
#include <osgEarthQt/ViewWidget>
#include <osgGA/StateSetManipulator>
#include <osgQt/GraphicsWindowQt>
#include <osgViewer/View>

#include <QWidget>

#define LC "[ViewWidget] "

using namespace osgEarth;
using namespace osgEarth::QtGui;


ViewWidget::ViewWidget(osgViewer::View* view, osg::GraphicsContext* gc) :
_view( view )
{
    init( gc );
}


void
ViewWidget::init(osg::GraphicsContext* gc)
{
    gc = createOrShareGC( gc );

    osg::Camera* camera = _view->getCamera();
    if ( !camera )
    {
        camera = new osg::Camera();
        _view->setCamera( camera );
    }
    camera->setGraphicsContext( gc );
    camera->setViewport(new osg::Viewport(0, 0, gc->getTraits()->width, gc->getTraits()->height));
    camera->setProjectionMatrixAsPerspective( 30.0f, gc->getTraits()->width/gc->getTraits()->height, 1.0f, 10000.0f );
}



osg::GraphicsContext*
ViewWidget::createOrShareGC(osg::GraphicsContext* gc)
{
    if ( !gc )
        gc->createNewContextID();

    osg::DisplaySettings* ds = osg::DisplaySettings::instance().get();
    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits(ds);

    traits->readDISPLAY();
    if (traits->displayNum<0) traits->displayNum = 0;

    traits->windowDecoration = false;
    traits->x = 0; //x();
    traits->y = 0; //y();
    traits->width = 100; // width();
    traits->height = 100; //height();
    traits->doubleBuffer = true;
    traits->alpha = ds->getMinimumNumAlphaBits();
    traits->stencil = ds->getMinimumNumStencilBits();
    traits->sampleBuffers = ds->getMultiSamples();
    traits->samples = ds->getNumMultiSamples();
    traits->sharedContext = gc;
    traits->inheritedWindowData = new osgQt::GraphicsWindowQt::WindowData(this);

    return new osgQt::GraphicsWindowQt(traits.get());
}


ViewWidget::~ViewWidget()
{
    //nop
}


void
ViewWidget::paintEvent(QPaintEvent* e)
{
    //nop
}
