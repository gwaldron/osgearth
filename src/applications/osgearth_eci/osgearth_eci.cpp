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

/**
 * Experiment with using a J2000/ECI reference frame as the root of the scene,
 * with the MapNode under an ECI-to-ECEF transform.
 *
 * https://celestrak.com/columns/v02n01/
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
#include <osgEarthSymbology/Color>
#include <osgEarthAnnotation/LabelNode>
#include <osgViewer/Viewer>
#include <osg/AutoTransform>
#include <iostream>

#define LC "[eci] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Symbology;
using namespace osgEarth::Annotation;
namespace ui = osgEarth::Util::Controls;

int
usage(const char* name, const char* msg)
{
    OE_NOTICE 
        << "\nUsage: " << name << " [file.earth]\n"
        << "     --tle <filename>    : Load a Celestrak TLE file\n"
        << "     --maxpoints <num>   : Limit the track size to <num> points\n"
        << msg << std::endl;

    return 0;
}

// Code to read TLE track data files from https://celestrak.com/NORAD
struct ECILocation
{
    DateTime timestamp;
    Angle ra, incl;
    Distance alt;
    osg::Vec3d eci;
};
typedef std::vector<ECILocation> ECITrack;

class TLEReader
{
public:
    // https://celestrak.com/NORAD/documentation/tle-fmt.php
    bool read(const std::string& filename, ECITrack& track) const
    {
        std::ifstream fin(filename.c_str());
        while(!fin.eof())
        {
            std::string line1, line2;

            std::getline(fin, line1);
            std::getline(fin, line2);
            if (line1.empty() || line2.empty())
                break;

            track.push_back(ECILocation());
            ECILocation& loc = track.back();

            // read timestamp
            int year2digit = osgEarth::as<int>(line1.substr(18, 2), 99);
            int year = year2digit > 50? 1900+year2digit : 2000+year2digit;
            double dayOfYear = osgEarth::as<double>(line1.substr(20, 12), 0);
            loc.timestamp = DateTime(year, dayOfYear);

            // read ra/decl
            loc.incl.set(osgEarth::as<double>(line2.substr(8,8),0), Units::DEGREES);
            loc.ra.set(osgEarth::as<double>(line2.substr(17,8),0), Units::DEGREES);
            loc.alt.set(6371 + 715, Units::KILOMETERS);

            // convert to ECI
            double
                R = loc.alt.as(Units::METERS),
                ra = loc.ra.as(Units::RADIANS),
                incl = loc.incl.as(Units::RADIANS);

            loc.eci =
                osg::Quat(ra,   osg::Vec3d(0,0,1)) *
                osg::Quat(incl, osg::Vec3d(1,0,0)) *
                osg::Vec3d(R,0,0);
        }
        OE_INFO << "Read " << track.size() << " track points" << std::endl;
        return true;
    }
};

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
    ECIReferenceFrame()
    {
        Lighting::set(getOrCreateStateSet(), osg::StateAttribute::OFF);
    }

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

// Loads up an ECITrack for display as a series of points.
class ECIDrawable : public PointDrawable
{
public:
    ECIDrawable()
    {
        setPointSmooth(true);
        setPointSize(4.0f);
    }

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

    const osg::Vec3& getCurrentPoint() const
    {
        unsigned index = getFirst() + getCount() - 1u;
        return getVertex(index);
    }
    
    void load(const ECITrack& track)
    {
        osg::FloatArray* times = new osg::FloatArray();
        times->setBinding(osg::Array::BIND_PER_VERTEX);
        setVertexAttribArray(6, times);

        osg::Vec4f HSLA;
        Color color;

        for(unsigned i=0; i<track.size(); ++i)
        {
            const ECILocation& loc = track[i];
            pushVertex(loc.eci);
            pushVertexAttrib(times, (float)loc.timestamp.asTimeStamp());
            
            // simple color ramp
            HSLA.set((float)i/(float)(track.size()-1), 1.0f, 1.0f, 1.0f);
            color.fromHSL(HSLA);
            setColor(i, color);      
        }
        finish();
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
    return d;
}

// Application-wide data and control structure
struct App
{
    DateTime start, end;
    HSliderControl* time;
    LabelControl* timeLabel;
    SkyNode* sky;
    J2000ToECEFTransform* ecef;
    osg::Group* eci;
    ECIDrawable* eciDrawable;    
    ECITrack track;

    App() 
    {
        eciDrawable = 0L;
        start = J2000Epoch;
        end = start + 24.0;
    }

    void setTime()
    {
        DateTime newTime(time->getValue());

        if (sky)
            sky->setDateTime(newTime);

        if (ecef)
            ecef->setDateTime(newTime);

        if (eciDrawable)
            eciDrawable->setDateTime(newTime);

        timeLabel->setText(newTime.asRFC1123());
    }
};

OE_UI_HANDLER(setTime);


int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    if ( arguments.read("--help") )
        return usage(argv[0], "Help");

    App app;

    // Read in an optiona TLE track data file
    std::string tlefile;
    if (arguments.read("--tle", tlefile))
    {
        TLEReader().read(tlefile, app.track);
        if (!app.track.empty())
        {
            int maxPoints;
            if (arguments.read("--maxpoints", maxPoints) && app.track.size() > maxPoints)
            {
                app.track.resize(maxPoints);
            }
            app.start = app.track.front().timestamp;
            app.end   = app.track.back().timestamp;
        }
    }

    osgViewer::Viewer viewer(arguments);
    viewer.setCameraManipulator( new EarthManipulator(arguments) );

    ui::VBox* container = new ui::VBox();
    container->setChildSpacing(3);
    container->addControl(new ui::LabelControl("ECI COORDINATE SYSTEM EXAMPLE", Color::Yellow));

    // UI control to modify the time of day.
    ui::HBox* h = container->addControl(new ui::HBox());
    h->addControl(new ui::LabelControl("Time:"));
    app.time = h->addControl(new HSliderControl(
        app.start.asTimeStamp(), app.end.asTimeStamp(), app.start.asTimeStamp(),
        new setTime(app)));
    app.time->setWidth(500);
    app.timeLabel = container->addControl(new LabelControl());

    // Load an earth file  
    osg::Node* earth = MapNodeHelper().load(arguments, &viewer, container);
    if (earth)
    {
        // New scene graph root
        osg::Group* root = new osg::Group();

        // First create a Sky which we will place in the (default) ECI frame.
        SkyOptions skyOptions;
        skyOptions.coordinateSystem() = SkyOptions::COORDSYS_ECI;
        app.sky = SkyNode::create(MapNode::get(earth));
        app.sky->attach(&viewer);
        root->addChild(app.sky);
        
        // A special transform takes us from the ECI into an ECEF frame
        // based on the current date and time.
        // The earth (MapNode) lives here since it is ECEF.
        app.ecef = new J2000ToECEFTransform();
        app.sky->addChild(app.ecef);
        app.ecef->addChild(earth);
        
        // This group holds data in the ECI frame.
        app.eci = new ECIReferenceFrame();
        app.eci->addChild(createECIAxes());
        MapNode::get(earth)->addChild(app.eci);

        // Track data
        if (!app.track.empty())
        {
            app.eciDrawable = new ECIDrawable();
            app.eciDrawable->load(app.track);
            app.eci->addChild(app.eciDrawable);
        }

        viewer.realize();
        app.time->setWidth(viewer.getCamera()->getViewport()->width()-40);

        app.setTime();
        viewer.setSceneData(root);
        viewer.run();
    }
    else
    {
        return usage(argv[0], "Bad earth file");
    }

    return 0;
}