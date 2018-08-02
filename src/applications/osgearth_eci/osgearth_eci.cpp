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


class ECIToECEFTransform : public osg::MatrixTransform
{
public:
    void setDateTime(const DateTime& dt)
    {
        // (rad/sec) Earth's rotation rate: International Astronomical Union (IAU) GRS 67
        const double earthRotationRate = 7292115.1467e-11;
        osg::Matrix matrix;

        const double eciToEcefRadians = -earthRotationRate * (double)dt.asTimeStamp();
        
        matrix.makeRotate(eciToEcefRadians, 0, 0, 1);
        this->setMatrix(matrix);
    }
};


class ECIReferenceFrame : public osg::MatrixTransform
{
public:
    ECIReferenceFrame()
    {
        addCullCallback(new InstallViewportSizeUniform());
    }
};


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


struct App
{
    DateTime _start;
    HSliderControl* _time;
    ECIToECEFTransform* _ecefGroup;
    osg::Group* _eciGroup;
    ECIDrawable* _eciDrawable;

    App() 
    {
        _eciDrawable = 0L;
    }

    void setTime()
    {
        DateTime newTime(_start.year(), _start.month(), _start.day(), _start.hours() + _time->getValue());

        if (_ecefGroup)
            _ecefGroup->setDateTime(newTime);

        if (_eciDrawable)
            _eciDrawable->setDateTime(newTime);
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
        container->addControl(new LabelControl(app._time));

        // New scene graph root
        osg::Group* root = new osg::Group();
        
        // A special transform takes us from the ECI into an ECEF frame
        // based on the current date and time.
        // The earth (MapNode) lives here since it is ECEF.
        app._ecefGroup = new ECIToECEFTransform();
        app._ecefGroup->addChild(earth);
        root->addChild(app._ecefGroup);
        
        // This group holds data in the ECI frame.
        app._eciGroup = new ECIReferenceFrame();
        root->addChild(app._eciGroup);

        // Load some ECI frame data
        if (!eciFile.empty())
        {
            app._eciDrawable = new ECIDrawable();
            app._start = app._eciDrawable->load(eciFile);
            app._eciGroup->addChild(app._eciDrawable);
        }

        viewer.setSceneData(root);
        viewer.run();
    }
    else
    {
        return usage(argv[0], "Bad earth file");
    }

    return 0;
}