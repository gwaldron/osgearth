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
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
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

void addLotsOfVerts(LineDrawable* line)
{
    for (int x = 0; x < 10; ++x)
    {
        for (int y = 0; y < 10; ++y)
        {
            line->pushVertex(osg::Vec3(x, 0, y));
        }
    }
    line->dirty();
}

osg::Node* makeGeometryForImport(double x, double y)
{
    osg::Geometry* geom = new osg::Geometry();
    osg::Vec3Array* verts = new osg::Vec3Array();
    verts->push_back(osg::Vec3(x, 0, y));
    verts->push_back(osg::Vec3(x + 5, 0, y));
    verts->push_back(osg::Vec3(x + 10, 0, y));
    verts->push_back(osg::Vec3(x + 10, 0, y + 5));
    verts->push_back(osg::Vec3(x + 10, 0, y + 10));
    verts->push_back(osg::Vec3(x + 5, 0, y + 10));
    verts->push_back(osg::Vec3(x, 0, y + 10));
    verts->push_back(osg::Vec3(x, 0, y + 5));
    geom->setVertexArray(verts);    
    osg::Vec4Array* colors = new osg::Vec4Array(1);
    (*colors)[0].set(1,1,1,1);
    geom->setColorArray(colors);
    geom->addPrimitiveSet(new osg::DrawArrays(GL_LINE_STRIP, 0, verts->size()));
    return geom;
}

struct TestFirstCount : public osg::NodeCallback
{
    void operator()(osg::Node* node, osg::NodeVisitor* nv)
    {
        if (nv->getFrameStamp()->getFrameNumber() % 20 == 0)
        {
            LineDrawable* line = (LineDrawable*)node;

            unsigned total = line->getNumVerts();
            unsigned first = line->getFirst();
        
            line->setFirst( (first+1) % total );
            line->setCount( 3 );
        }
    }
};

osg::Node* createLineDrawables()
{
    LineGroup* group = new LineGroup();

    group->addCullCallback(new InstallViewportSizeUniform());

    LineDrawable* strip = new LineDrawable(GL_LINE_STRIP);
    strip->setLineWidth(3);
    strip->setColor(osg::Vec4(1,1,1,1));
    addVerts(strip, 10, 10);
    group->addChild(strip);

    LineDrawable* loop = new LineDrawable(GL_LINE_LOOP);
    loop->setLineWidth(8);
    loop->setColor(osg::Vec4(1,1,0,1));
    addVerts(loop, 30, 10);
    group->addChild(loop);

    LineDrawable* stippled = new LineDrawable(GL_LINE_STRIP);
    stippled->setLineWidth(4);
    stippled->setStipplePattern(0xff00);
    stippled->setColor(osg::Vec4(0,1,0,1));
    addVerts(stippled, 50, 10);
    group->addChild(stippled);

    LineDrawable* segments = new LineDrawable(GL_LINES);
    segments->setLineWidth(3);
    segments->setColor(osg::Vec4(0,1,1,1));
    addVerts(segments, 70, 10);
    group->addChild(segments);

    LineDrawable* firstCount = new LineDrawable(GL_LINE_STRIP);
    firstCount->setLineWidth(5);
    firstCount->setColor(osg::Vec4(1,0,1,1));
    addVerts(firstCount, 90, 10);
    firstCount->addUpdateCallback(new TestFirstCount());
    group->addChild(firstCount);

    osg::ref_ptr<osg::Node> node = makeGeometryForImport(110, 10);
    group->import(node.get());

    return group;
}

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    osgViewer::Viewer viewer(arguments);

    osg::ref_ptr<osg::Node> node = createLineDrawables();

    if (arguments.read("--serialize"))
    {
        const char* fileName = "out.osgt";

        OE_NOTICE << "Writing to " << fileName << " ..." << std::endl;
        if (!osgDB::writeNodeFile(*node.get(), fileName))
        {
            OE_WARN << "serialize failed!\n";
            return -1;
        }

        OE_NOTICE << "Reading from " << fileName << " ..." << std::endl;
        node = osgDB::readNodeFile(fileName);
        if (!node.valid())
        {
            OE_WARN << "deserialize failed!\n";
            return -1;
        }
    }

    viewer.setSceneData(node.get());
    
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    return viewer.run();
}
