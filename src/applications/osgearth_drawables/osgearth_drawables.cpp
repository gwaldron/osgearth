/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2019 Pelican Mapping
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
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/LineStipple>
#include <osgGA/StateSetManipulator>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgEarth/LineDrawable>
#include <osgEarth/PointDrawable>
#include <osgEarth/CullingUtils>
#include <osgEarth/VirtualProgram>
#include <osgEarth/GLUtils>

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
    line->pushVertex(osg::Vec3(x + 5, 0, y + 5));
    line->finish();
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
    line->finish();
}

LineDrawable* makeStar(double x, double y, double r)
{
    LineDrawable* star = new LineDrawable(GL_LINES);
    for(float i=0.0f; i<osg::PI*2.0; i += osg::PI/16.0)
    {
        float c = cos(i), s = sin(i);
        star->pushVertex(osg::Vec3(x, 0, y));
        star->pushVertex(osg::Vec3(x+(r*c-r*s), 0, y+(r*c+r*s)));
    }
    star->finish();
    return star;
}

PointDrawable* makeGridOfPoints(double x, double y)
{
    PointDrawable* grid = new PointDrawable();
    for(float i=x; i<x+20; i+=5)
    {
        for(float j=y; j<y+20; j+=5)
        {
            grid->pushVertex(osg::Vec3(i, 0, j));
        }
    }
    grid->finish();
    return grid;
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
    verts->push_back(osg::Vec3(x + 5, 0, y + 5));
    geom->setVertexArray(verts);    
    osg::Vec4Array* colors = new osg::Vec4Array(1);
    (*colors)[0].set(1,1,1,1);
    geom->setColorArray(colors);
    geom->addPrimitiveSet(new osg::DrawArrays(GL_LINE_STRIP, 0, verts->size()));
    geom->getOrCreateStateSet()->setAttributeAndModes(new osg::LineWidth(3.0f));
    geom->getOrCreateStateSet()->setAttributeAndModes(new osg::LineStipple(1, 0xfff0));
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

osg::Node* createDrawables()
{
    // You need a viewport uniform for the lines to work.
    // MapNode installs one automatically, but we're not using MapNode
    // in this example.
    osg::Group* group = new osg::Group();
    group->addCullCallback(new InstallViewportSizeUniform());

    float x = 10;
    float y = 10;

    LineDrawable* strip = new LineDrawable(GL_LINE_STRIP);
    strip->setLineWidth(8);
    strip->setColor(osg::Vec4(1,1,1,1));
    addVerts(strip, x, y);
    group->addChild(strip);

    x += 20;
    LineDrawable* loop = new LineDrawable(GL_LINE_LOOP);
    loop->setLineWidth(1);
    loop->setColor(osg::Vec4(1,1,0,1));
    addVerts(loop, x, y);
    group->addChild(loop);

    x += 20;
    LineDrawable* stippled = new LineDrawable(GL_LINE_STRIP);
    stippled->setLineWidth(4);
    stippled->setStipplePattern(0xff00);
    stippled->setColor(osg::Vec4(0,1,0,1));
    addVerts(stippled, x, y);
    group->addChild(stippled);
    
    x += 20;
    LineDrawable* segments = new LineDrawable(GL_LINES);
    segments->setLineWidth(3);
    segments->setColor(osg::Vec4(0,1,1,1));
    addVerts(segments, x, y);
    group->addChild(segments);

    x += 20;
    LineDrawable* firstCount = new LineDrawable(GL_LINE_STRIP);
    firstCount->setLineWidth(5);
    firstCount->setColor(osg::Vec4(1,0,1,1));
    addVerts(firstCount, x, y);
    firstCount->addUpdateCallback(new TestFirstCount());
    group->addChild(firstCount);
    
    x += 20;
    osg::ref_ptr<osg::Node> node = makeGeometryForImport(x, y);
    LineGroup* lines = new LineGroup();
    lines->import(node.get());
    group->addChild(lines);

    x += 20;
    LineDrawable* points = new LineDrawable(GL_POINTS);
    addVerts(points, x, y);
    group->addChild(points);

    x = 20;
    y -= 20;
    for(unsigned i=0; i<10; ++i)
    {
        LineDrawable* across = new LineDrawable(GL_LINES);
        across->pushVertex(osg::Vec3(x, 0, y));
        across->pushVertex(osg::Vec3(x+100, 0, y));
        across->setLineWidth((float)(i+1));
        across->finish();
        group->addChild(across);
        y -= (i+2);
    }

    x = 20;
    y -= 20;
    LineDrawable* star = makeStar(x, y, 10);
    star->setColor(osg::Vec4(1,1,1,1));
    star->setLineWidth(1.0f);
    group->addChild(star);

    x += 40;
    LineDrawable* star2 = makeStar(x, y, 10);
    star2->setColor(osg::Vec4(1,.5,0,1));
    star2->setLineWidth(2.0f);
    group->addChild(star2);

    x += 40;
    LineDrawable* star3 = makeStar(x, y, 10);
    star3->setColor(osg::Vec4(1,1,0,1));
    star3->setLineWidth(3.0f);
    group->addChild(star3);

    y -= 40;
    x = 20;
    PointDrawable* grid = makeGridOfPoints(x, y);
    grid->setPointSize(3.0f);
    grid->setColor(osg::Vec4(0,1,1,1));
    group->addChild(grid);

    x += 50;
    PointDrawable* grid2 = makeGridOfPoints(x, y);
    grid2->setPointSize(20.0f);
    GLUtils::setPointSmooth(grid2->getOrCreateStateSet(), 1);
    group->addChild(grid2);

    return group;
}

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    osgViewer::Viewer viewer(arguments);

    osg::ref_ptr<osg::Node> node = createDrawables();

#ifdef OSG_GL3_AVAILABLE
    // Sets up the State for GL3 mode
    viewer.setRealizeOperation(new GL3RealizeOperation());
#endif

    if (arguments.read("--ortho"))
    {
        viewer.realize();
        double r = node->getBound().radius() * 1.2;
        double ar = viewer.getCamera()->getViewport()->width() / viewer.getCamera()->getViewport()->height();
        viewer.getCamera()->setProjectionMatrixAsOrtho(-r, +r, -r/ar, +r/ar, -r*2.0, +r*2.0);
    }

    if (arguments.read("--antialias") || arguments.read("--smooth"))
    {
        GLUtils::setLineSmooth(node->getOrCreateStateSet(), 1);
        GLUtils::setPointSmooth(node->getOrCreateStateSet(), 1);
        node->getOrCreateStateSet()->setMode(GL_BLEND, 1);
    }

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
        node = osgDB::readRefNodeFile(fileName);
        if (!node.valid())
        {
            OE_WARN << "deserialize failed!\n";
            return -1;
        }
    }

    // Sets up global default uniform values needed by osgEarth
    GLUtils::setGlobalDefaults(viewer.getCamera()->getOrCreateStateSet());

    viewer.setSceneData(node.get());
    
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    return viewer.run();
}
