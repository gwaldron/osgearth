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
#include <osgEarth/Notify>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/Controls>
#include <osgEarth/MapNode>
#include <osgEarth/ThreadingUtils>
#include <osgEarthAnnotation/CircleNode>
#include <osgEarthAnnotation/AnnotationLayer>
#include <osgEarthSymbology/Style>
#include <iostream>

#define LC "[scene_clamping] "

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Util;
namespace ui = osgEarth::Util::Controls;

int
usage(const char* name)
{
    OE_NOTICE 
        << "\nUsage: " << name << " file.earth" << std::endl
        << MapNodeHelper().usage() << std::endl;

    return 0;
}

#define LAT 34.534
#define LON -119.578

struct App
{
    LocalGeometryNode* _lgn;
    ui::CheckBoxControl* _clamp;
    ui::CheckBoxControl* _clampRelative;
    ui::CheckBoxControl* _clampPerVertex;
    ui::CheckBoxControl* _includeVertAlts;
    ui::HSliderControl*  _altitude;
    ui::HSliderControl*  _longitude;

    Layer* createAnnotations()
    {
        GeoPoint position(SpatialReference::get("wgs84"), LON, LAT);

        Style style;
        LineSymbol* line = style.getOrCreate<LineSymbol>();
        line->stroke()->color().set(1,1,0,1);
        line->stroke()->width() = 5;

        CircleNode* circle = new CircleNode();
        circle->set(
            position,
            Distance(5, Units::KILOMETERS),
            style);

        AnnotationLayer* layer = new AnnotationLayer();
        layer->getGroup()->addChild(circle);

        _lgn = circle;

        return layer;
    }

    void applyAll()
    {
        applyPosition();
        applyStyle();
    }

    void applyPosition()
    {
        bool clamping = _clamp->getValue() == true;
        bool clampRelativeToTerrain = clamping && _clampRelative->getValue() == true;

        GeoPoint p = _lgn->getPosition();
        p.altitudeMode() = clamping ? ALTMODE_RELATIVE : ALTMODE_ABSOLUTE;
        p.alt() = !clamping || clampRelativeToTerrain ? _altitude->getValue() : 0.0;
        p.x() = _longitude->getValue();
        _lgn->setPosition(p);
    }

    void applyStyle()
    {
        //bool clamping = _clamp->getValue() == true;
        bool clampPerVertex = _clampPerVertex->getValue() == true;
        bool includeVertAlts = _includeVertAlts->getValue() == true;

        Style style = _lgn->getStyle();
        style.remove<AltitudeSymbol>();
        style.remove<RenderSymbol>();

        if (clampPerVertex)
        {
            AltitudeSymbol* alt = style.getOrCreate<AltitudeSymbol>();
            alt->technique() = alt->TECHNIQUE_SCENE;
            alt->binding() = alt->BINDING_VERTEX;
            alt->clamping() = includeVertAlts? alt->CLAMP_RELATIVE_TO_TERRAIN : alt->CLAMP_TO_TERRAIN;

            RenderSymbol* render = style.getOrCreate<RenderSymbol>();
            render->depthOffset()->enabled() = true;
        }

        _lgn->setStyle(style);
    }
};

#define UI_HANDLER(X) \
    struct X : public ui::ControlEventHandler { \
        App& _app; X(App& app):_app(app) { } \
        void onValueChanged(ui::Control*) { _app. X (); } }

UI_HANDLER(applyAll);
UI_HANDLER(applyStyle);
UI_HANDLER(applyPosition);

void createUI(App& app, ui::Container* main)
{    
    ui::HBox* h;

    h = main->addControl(new ui::HBox());
    app._clamp = h->addControl(new ui::CheckBoxControl(true, new applyPosition(app)));
    h->addControl(new ui::LabelControl("Clamp"));

    h = main->addControl(new ui::HBox());
    app._clampRelative = h->addControl(new ui::CheckBoxControl(false, new applyPosition(app)));
    h->addControl(new ui::LabelControl("Clamp relative to terrain"));
 
    h = main->addControl(new ui::HBox());
    app._clampPerVertex = h->addControl(new ui::CheckBoxControl(false, new applyStyle(app)));
    h->addControl(new ui::LabelControl("Clamp per-vertex"));

    h = main->addControl(new ui::HBox());
    app._includeVertAlts = h->addControl(new ui::CheckBoxControl(false, new applyStyle(app)));
    h->addControl(new ui::LabelControl("Include vertex altitudes"));

    h = main->addControl(new ui::HBox());
    h->addControl(new ui::LabelControl("Altitude"));
    app._altitude = h->addControl(new ui::HSliderControl(0, 3000, 0, new applyPosition(app)));
    app._altitude->setWidth(200);
    h->addControl(new ui::LabelControl(app._altitude));

    h = main->addControl(new ui::HBox());
    h->addControl(new ui::LabelControl("Longitude"));
    app._longitude = h->addControl(new ui::HSliderControl(LON-0.25, LON+0.25, LON, new applyPosition(app)));
    app._longitude->setWidth(200);
    h->addControl(new ui::LabelControl(app._longitude));
}

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    // help?
    if ( arguments.read("--help") )
        return usage(argv[0]);
   
    // create a viewer:
    osgViewer::Viewer viewer(arguments);
    EarthManipulator* em = new EarthManipulator();
    viewer.setCameraManipulator(em);
    ui::Container* main = new ui::VBox();
    osg::Node* node = MapNodeHelper().load(arguments, &viewer, main);
    if ( node )
    {
        App app;
        createUI(app, main);
        MapNode::get(node)->getMap()->addLayer(app.createAnnotations());
        viewer.setSceneData( node );
        app.applyAll();
        em->setViewpoint(Viewpoint("Start", -119.575, 34.547, 700, 0, -28, 32500), 2.0);
        return viewer.run();
    }
    else
    {
        return usage(argv[0]);
    }
}
