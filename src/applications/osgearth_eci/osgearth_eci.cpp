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

/**
 * Experiment with using a J2000/ECI reference frame as the root of the scene,
 * with the MapNode under an ECI-to-ECEF transform.
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
        << "     --tle <filename>    : Load a NORAD TLE file\n"
        << "     --maxpoints <num>   : Limit the track size to <num> points\n"
        << "     --ecef              : View the track in ECEF space instead of ECI\n"
        << "     --tessellate        : Add interpolated points to the track data\n"
        << "\nDownload NORAD TLE files from https://www.celestrak.com/NORAD/archives\n\n"
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
        osg::Matrix matrix = createMatrix(dt);
        setMatrix(matrix);
    }

    static osg::Matrix createMatrix(const DateTime& dt)
    {
        // Earth's rotation rate: International Astronomical Union (IAU) GRS 67
        const double IAU_EARTH_ANGULAR_VELOCITY = 7292115.1467e-11; // (rad/sec)

        double secondsElapsed = (double)(dt.asTimeStamp() - J2000Epoch.asTimeStamp());
        const double rotation = IAU_EARTH_ANGULAR_VELOCITY * secondsElapsed;
        
        osg::Matrix matrix;
        matrix.makeRotate(rotation, 0, 0, 1);
        return matrix;
    }
};

// Code to read TLE track data files from https://celestrak.com/NORAD
struct ECILocation
{
    DateTime timestamp;     // point time
    Angle incl;             // inclination
    Angle raan;             // right ascencion of ascending node
    Distance alt;           // altitude
    osg::Vec3d eci;         // ECI coordinate
    osg::Vec3d ecef;        // ECEF coordinate

    void computeECIAndECEF()
    {
        eci =
            osg::Quat(raan.as(Units::RADIANS), osg::Vec3d(0, 0, 1)) *
            osg::Quat(incl.as(Units::RADIANS), osg::Vec3d(1, 0, 0)) *
            osg::Vec3d(alt.as(Units::METERS), 0, 0);

        osg::Matrix eci2ecef = J2000ToECEFTransform::createMatrix(timestamp);
        ecef = eci * eci2ecef;
    }
};

struct ECITrack : public std::vector<ECILocation>
{
    // interpolate points for a smoother track
    void tessellate()
    {
        ECITrack newTrack;
        for(unsigned k=0; k<size()-1; ++k)
        {
            for(float t=0; t<1.0f; t+=0.1)
            {
                const ECILocation& p0 = at(k);
                const ECILocation& p1 = at(k+1);

                newTrack.push_back(ECILocation());
                ECILocation& loc = newTrack.back();
                
                loc.timestamp = DateTime(p0.timestamp.asTimeStamp() + (p1.timestamp.asTimeStamp()-p0.timestamp.asTimeStamp())*t);
                loc.raan.set(p0.raan.as(Units::RADIANS) + (p1.raan.as(Units::RADIANS)-p0.raan.as(Units::RADIANS))*t, Units::RADIANS);
                loc.incl.set(p0.incl.as(Units::RADIANS) + (p1.incl.as(Units::RADIANS)-p0.incl.as(Units::RADIANS))*t, Units::RADIANS);
                loc.alt = p0.alt;
                loc.computeECIAndECEF();
            }
        }
        swap(newTrack);
    }
};

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
            loc.raan.set(osgEarth::as<double>(line2.substr(17,8),0), Units::DEGREES);
            loc.alt.set(6371 + 715, Units::KILOMETERS);

            loc.computeECIAndECEF();
        }
        OE_INFO << "Read " << track.size() << " track points" << std::endl;
        return true;
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
class ECITrackDrawable : public LineDrawable //public PointDrawable
{
public:
    ECITrackDrawable() : LineDrawable(GL_LINE_STRIP)
    {
        Lighting::set(getOrCreateStateSet(), 0);
        //setPointSmooth(true);
        //setPointSize(4.0f);
    }

    void setDateTime(const DateTime& dt)
    {
        osg::FloatArray* times = dynamic_cast<osg::FloatArray*>(getVertexAttribArray(6));
        unsigned i;
        for (i = 0; i < getNumVerts(); ++i)
        {
            if (dt.asTimeStamp() < getVertexAttrib(times, i))
                break;
        }
        setCount(i);
    }
    
    void load(const ECITrack& track, bool drawECEF)
    {
        osg::FloatArray* times = new osg::FloatArray();
        times->setBinding(osg::Array::BIND_PER_VERTEX);
        setVertexAttribArray(6, times);

        osg::Vec4f HSLA;
        Color color;

        for(unsigned i=0; i<track.size(); ++i)
        {
            const ECILocation& loc = track[i];
            pushVertex(drawECEF? loc.ecef : loc.eci);
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
    ECITrackDrawable* trackDrawable;    
    ECITrack track;

    App() 
    {
        trackDrawable = 0L;
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

        if (trackDrawable)
            trackDrawable->setDateTime(newTime);

        timeLabel->setText(newTime.asRFC1123());
    }
};

OE_UI_HANDLER(setTime);


int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    if ( arguments.read("--help") )
        return usage(argv[0], "");

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
                app.track.resize(maxPoints);
            if (arguments.read("--tessellate"))
                app.track.tessellate();
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
        app.sky->getSunLight()->setAmbient(osg::Vec4(0.5,0.5,0.5,1.0));
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
            app.trackDrawable = new ECITrackDrawable();

            bool drawECEF = arguments.read("--ecef");
            if (drawECEF)
            {
                app.trackDrawable->load(app.track, true);
                MapNode::get(earth)->addChild(app.trackDrawable);
            }
            else
            {
                app.trackDrawable->load(app.track, false);
                app.eci->addChild(app.trackDrawable);
            }
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