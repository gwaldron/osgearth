/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2016 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/StateSetManipulator>
#include <osgEarth/LineDrawable>
#include <osgEarth/CullingUtils>

#define LC "[drawables] "

using namespace osgEarth;

void addVerts(LineDrawable* line, double x, double y)
{
    line->pushVertex(osg::Vec3(x, 0, y));
    line->pushVertex(osg::Vec3(x + 5, 0, y));
    line->pushVertex(osg::Vec3(x + 10, 0, y));
    line->pushVertex(osg::Vec3(x + 10, 0, y + 5));
    line->pushVertex(osg::Vec3(x + 10, 0, y + 10));
    line->pushVertex(osg::Vec3(x + 5, 0, y + 10));
    line->pushVertex(osg::Vec3(x, 0, y + 10));
    line->pushVertex(osg::Vec3(x, 0, y + 5));
    line->dirty();
}

osg::Node* createLineDrawables()
{
    LineGroup* group = new LineGroup();

    group->addCullCallback(new InstallViewportSizeUniform());

    LineDrawable* strip = new LineDrawable(GL_LINE_STRIP);
    strip->setLineWidth(2);
    addVerts(strip, 10, 10);
    group->addChild(strip);

    LineDrawable* loop = new LineDrawable(GL_LINE_LOOP);
    loop->setLineWidth(25);
    addVerts(loop, 30, 10);
    group->addChild(loop);

    LineDrawable* stippled = new LineDrawable(GL_LINE_STRIP);
    stippled->setLineWidth(4);
    stippled->setStippling(1, 0xff00);
    addVerts(stippled, 50, 10);
    group->addChild(stippled);

    LineDrawable* segments = new LineDrawable(GL_LINES);
    segments->setLineWidth(3);
    addVerts(segments, 70, 10);
    group->addChild(segments);

    return group;
}

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    osgViewer::Viewer viewer(arguments);

    viewer.setSceneData(createLineDrawables());
    
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    return viewer.run();
}
