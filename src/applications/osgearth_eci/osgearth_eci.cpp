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

#include <osgEarth/MapNode>
#include <osgEarth/DateTime>
#include <osgEarth/NodeUtils>
#include <osgEarth/PointDrawable>
#include <osgEarth/CullingUtils>
#include <osgEarth/LineDrawable>
#include <osgEarth/Lighting>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/Sky>
#include <osgViewer/Viewer>
#include <iostream>

#define LC "[eci] "

using namespace osgEarth;
using namespace osgEarth::Util;
namespace ui = osgEarth::Util::Controls;

int
usage(const char* name, const char* msg)
{
    OE_NOTICE 
        << "\nUsage: " << name << " [file.earth]\n"
        << msg << std::endl;

    return 0;
}

// Reference time for the J2000 ECI coordinate frame 
static DateTime J2000Epoch(2000, 1, 1, 12.00);

// Transform that takes us from a J2000 ECI reference frame 
// to an ECEF reference frame (i.e. MapNode)
class J2000ToECEFTransform : public osg::MatrixTransform
{
public:
    void setDateTime(const DateTime& dt)
    {
        // Earth's rotation rate: International Astronomical Union (IAU) GRS 67
        const double IAU_EARTH_ANGULAR_VELOCITY = 7292115.1467e-11; // (rad/sec)

        double secondsElapsed = (double)(dt.asTimeStamp() - J2000Epoch.asTimeStamp());
        const double rotation = IAU_EARTH_ANGULAR_VELOCITY * secondsElapsed;
        
        osg::Matrix matrix;
        matrix.makeRotate(rotation, 0, 0, 1);
        this->setMatrix(matrix);
    }
};

// If the "global" coordinate system is ECI, you can put this transform
// under the MapNode (in ECEF space) to "revert" to that global ECI frame.
// Useful if you want to put ECI-space data under the MapNode.
class ECIReferenceFrame : public osg::Group
{
public:
    void traverse(osg::NodeVisitor& nv)
    {
        osgUtil::CullVisitor* cv = Culling::asCullVisitor(nv);
        if (cv)
        {
            const osg::Camera* cam = cv->getRenderStage()->getCamera();
            cv->pushModelViewMatrix(new osg::RefMatrix(cam->getViewMatrix()), osg::Transform::ABSOLUTE_RF);
            osg::Group::traverse(nv);
            cv->popModelViewMatrix();
        }
        else osg::Group::traverse(nv);
    }
};

// Loads up some ECI data for display
class ECIDrawable : public PointDrawable
{
public:
    void setDateTime(const DateTime& dt)
    {
        osg::FloatArray* times = dynamic_cast<osg::FloatArray*>(getVertexAttribArray(6));
        unsigned i;
        for (i = 0; i < times->size(); ++i)
        {
            if (dt.asTimeStamp() < (*times)[i])
                break;
        }
        setCount(i);
    }

    DateTime load(const std::string& file)
    {
        osg::FloatArray* times = new osg::FloatArray();
        times->setBinding(osg::Array::BIND_PER_VERTEX);
        setVertexAttribArray(6, times);
        
        setPointSmooth(true);
        setPointSize(4.0f);

        std::ifstream in(file);
        while (!in.eof())
        {
            std::string val;
            in >> val;
            if (val == "PlatformData")
            {
                in >> val;
                if (val == "1")
                {
                    in >> val; // timestamp
                    osgEarth::replaceIn(val, "\"", "");
                    double t = as<double>(val, 0.0);
                    in >> val;
                    double x = as<double>(val, 0.0);
                    in >> val;
                    double y = as<double>(val, 0.0);
                    in >> val;
                    double z = as<double>(val, 0.0);

                    pushVertex(osg::Vec3(x, y, z));
                    pushVertexAttrib(times, t);
                }
            }
        }

        finish();
    
        if (!times->empty())
            return DateTime(times->front());
        else
            return DateTime();
    }
};

osg::Node* createECIAxes()
{
    const float R = 10e6;
    LineDrawable* d = new LineDrawable(GL_LINES);
    d->allocate(6);

    d->setVertex(0, osg::Vec3(0,0,0));
    d->setColor(0, osg::Vec4(1,0,0,1));
    d->setVertex(1, osg::Vec3(R,0,0));
    d->setColor(1, osg::Vec4(1,0,0,1));

    d->setVertex(2, osg::Vec3(0,0,0));
    d->setColor(2, osg::Vec4(0,1,0,1));
    d->setVertex(3, osg::Vec3(0,R,0));
    d->setColor(3, osg::Vec4(0,1,0,1));

    d->setVertex(4, osg::Vec3(0,0,0));
    d->setColor(4, osg::Vec4(0,0,1,1));
    d->setVertex(5, osg::Vec3(0,0,R));
    d->setColor(5, osg::Vec4(0,0,1,1));

    d->setLineWidth(10);
    Lighting::set(d->getOrCreateStateSet(), osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);
    return d;
}

struct App
{
    DateTime _start;
    HSliderControl* _time;
    LabelControl* _timeLabel;
    J2000ToECEFTransform* _ecef;
    osg::Group* _eci;
    ECIDrawable* _eciDrawable;

    App() 
    {
        _eciDrawable = 0L;
        _start = J2000Epoch;
    }

    void setTime()
    {
        DateTime newTime(_start.year(), _start.month(), _start.day(), _start.hours() + _time->getValue());

        if (_ecef)
            _ecef->setDateTime(newTime);

        if (_eciDrawable)
            _eciDrawable->setDateTime(newTime);

        _timeLabel->setText(newTime.asRFC1123());
    }
};

OE_UI_HANDLER(setTime);


int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    if ( arguments.read("--help") )
        return usage(argv[0], "Help");

    std::string eciFile;
    arguments.read("--eci", eciFile);

    osgViewer::Viewer viewer(arguments);
    viewer.setCameraManipulator( new EarthManipulator(arguments) );

    ui::VBox* container = new ui::VBox();

    // Load an earth file  
    osg::Node* earth = MapNodeHelper().load(arguments, &viewer, container);
    if ( earth )
    {
        App app;

        // UI control to modify the time of day.
        app._time = container->addControl(new HSliderControl(0.0, 23.99999, 0.0, new setTime(app)));
        app._time->setWidth(400);
        app._timeLabel = container->addControl(new LabelControl());

        // New scene graph root
        osg::Group* root = new osg::Group();
        
        // A special transform takes us from the ECI into an ECEF frame
        // based on the current date and time.
        // The earth (MapNode) lives here since it is ECEF.
        app._ecef = new J2000ToECEFTransform();
        app._ecef->addChild(earth);
        
        // This group holds data in the ECI frame.
        app._eci = new ECIReferenceFrame();
        app._eci->addChild(createECIAxes());
        MapNode::get(earth)->addChild(app._eci);

        // Load some ECI frame data
        if (!eciFile.empty())
        {
            app._eciDrawable = new ECIDrawable();
            app._start = app._eciDrawable->load(eciFile);
            app._eci->addChild(app._eciDrawable);
        }

        root->addChild(app._ecef);
        viewer.setSceneData(root);
        viewer.run();
    }
    else
    {
        return usage(argv[0], "Bad earth file");
    }

    return 0;
}