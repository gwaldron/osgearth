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

#include <osg/Notify>
#include <osgGA/StateSetManipulator>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/GUIEventHandler>
#include <osgEarth/Map>
#include <osgEarth/MapNode>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/AutoClipPlaneHandler>
#include <osgEarth/Utils>
#include <osgEarthFeatures/GeometryUtils>

#include <osgEarthSymbology/Style>

#include <osgEarthDrivers/gdal/GDALOptions>
#include <osgEarthDrivers/feature_ogr/OGRFeatureOptions>
#include <osgEarthDrivers/agglite/AGGLiteOptions>
#include <osgEarthDrivers/model_feature_geom/FeatureGeomModelOptions>

#include <osgEarthUtil/Controls>

#include <osgEarthAnnotation/FeatureEditing>

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Drivers;
using namespace osgEarth::Symbology;
using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;
using namespace osgEarth::Annotation;

osg::Vec4
randomColor()
{
    float r = (float)rand() / (float)RAND_MAX;
    float g = (float)rand() / (float)RAND_MAX;
    float b = (float)rand() / (float)RAND_MAX;
    return osg::Vec4(r,g,b,1.0f);
}


static int s_fid = 0;

static osg::ref_ptr< AddPointHandler > s_addPointHandler;
static osg::ref_ptr< osg::Node > s_editor;
static osg::ref_ptr< FeatureNode > s_featureNode;
static osgViewer::Viewer* s_viewer;
static osg::ref_ptr< osg::Group > s_root;
static osg::ref_ptr< osg::Group > s_editorsRoot;
static osg::ref_ptr< MapNode > s_mapNode;

Grid* createToolBar()
{    
    Grid* toolbar = new Grid();
    toolbar->setBackColor(0,0,0,0.5);
    toolbar->setMargin( 10 );
    toolbar->setPadding( 10 );
    toolbar->setChildSpacing( 10 );
    toolbar->setChildVertAlign( Control::ALIGN_CENTER );
    toolbar->setAbsorbEvents( true );
    toolbar->setVertAlign( Control::ALIGN_TOP );    
    return toolbar;    
}

struct AddVertsModeHandler : public ControlEventHandler
{
    AddVertsModeHandler()        
    {
    }

    void onClick( Control* control, int mouseButtonMask ) {

        //remove the editor if it's valid
        if (s_editor.valid())
        {
            s_editorsRoot->removeChild( s_editor.get() );
            s_editor = NULL;

            // Unset the stipple on the line
            Style style = s_featureNode->getStyle();
            style.get<LineSymbol>()->stroke()->stipple().unset();
            s_featureNode->setStyle( style );            
        }

        //Add the new add point handler
        if (!s_addPointHandler.valid())
        {            
            s_addPointHandler = new AddPointHandler( s_featureNode.get() );
            s_addPointHandler->setIntersectionMask( 0x1 );
            s_viewer->addEventHandler( s_addPointHandler.get() );
        }        
    }
};

struct EditModeHandler : public ControlEventHandler
{
    EditModeHandler()        
    { 
    }

    void onClick( Control* control, int mouseButtonMask ) {
        
        //Remove the add point handler if it's valid
        if (s_addPointHandler.valid())
        {            
            osgEarth::removeEventHandler( s_viewer, s_addPointHandler.get() );
            s_addPointHandler = NULL;
        }        

        if (!s_editor.valid())
        {            
            Style style = s_featureNode->getStyle();
            style.getOrCreate<LineSymbol>()->stroke()->stipple() = 0x00FF;                        
            s_featureNode->setStyle( style );            
            s_editor = new FeatureEditor( s_featureNode );
            s_editorsRoot->addChild( s_editor.get() );            
        }
    }    
};

struct ChangeStyleHandler : public ControlEventHandler
{
    ChangeStyleHandler(const Style &style) 
        : _style( style )
    {
        //nop
    }

    void onClick( Control* control, int mouseButtonMask ) {
        s_featureNode->setStyle( _style );        
    }

    Style _style;  
};

Style buildStyle( const osg::Vec4 &color, float width )
{
    // Define a style for the feature data. Since we are going to render the
    // vectors as lines, configure the line symbolizer:
    Style style;

    LineSymbol* ls = style.getOrCreateSymbol<LineSymbol>();
    ls->stroke()->color() = color;
    ls->stroke()->width() = width;        
    
    AltitudeSymbol* as = style.getOrCreate<AltitudeSymbol>();
    as->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;
    as->technique() = AltitudeSymbol::TECHNIQUE_DRAPE;

    style.getOrCreate<PolygonSymbol>()->fill()->color() = Color::Red;

    RenderSymbol* rs = style.getOrCreateSymbol<RenderSymbol>();
    rs->depthOffset()->enabled() = true;
    rs->depthOffset()->minBias() = 1000;

    return style;    
}

//
// NOTE: run this sample from the repo/tests directory.
//
int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    osgViewer::Viewer viewer(arguments);
    s_viewer = &viewer;

    // Start by creating the map:
    s_mapNode = MapNode::load(arguments);
    if ( !s_mapNode )
    {
        Map* map = new Map();

        // Start with a basemap imagery layer; we'll be using the GDAL driver
        // to load a local GeoTIFF file:
        GDALOptions basemapOpt;
        basemapOpt.url() = "../data/world.tif";
        map->addImageLayer( new ImageLayer( ImageLayerOptions("basemap", basemapOpt) ) );

        // That's it, the map is ready; now create a MapNode to render the Map:
        MapNodeOptions mapNodeOptions;
        mapNodeOptions.enableLighting() = false;

        s_mapNode = new MapNode( map, mapNodeOptions );
    }
    s_mapNode->setNodeMask( 0x01 );    

        
    // Define a style for the feature data.
    Style style = buildStyle( Color::Yellow, 2.0f );    

    //LineString* line = new LineString();    
    Geometry* geom = GeometryUtils::geometryFromWKT("POLYGON((191.026667 87.63333,114.75 78,89.5 77.333336,81.833336 75.333336,70.683334 74.5,70.916664 73.666664,68.666664 73.666664,66.291664 71.505,57.65 71.166664,58 73.9,48.616665 73,49.198334 71.43,49.5 70.5,43.266666 68.666664,32.083332 71.5,32.083332 74,35 74,35 81,32 81,32 90,191.026667 87.63333))");
    OE_NOTICE << "Geometry " << GeometryUtils::geometryToWKT(geom) << std::endl;
    Feature* feature = new Feature(geom, s_mapNode->getMapSRS(), Style(), s_fid++);
    s_featureNode = new FeatureNode( s_mapNode, feature );    
    s_featureNode->setStyle( style );
    
    s_editorsRoot = new osg::Group;

    s_root = new osg::Group;
    s_root->addChild( s_mapNode.get() );
    s_root->addChild( s_featureNode.get() );
    s_root->addChild( s_editorsRoot.get() );


    //Setup the controls
    ControlCanvas* canvas = ControlCanvas::getOrCreate( &viewer );
    s_root->addChild( canvas );
    Grid *toolbar = createToolBar( );
    canvas->addControl( toolbar );
    canvas->setNodeMask( 0x1 << 1 );

    int col = 0;
    LabelControl* addVerts = new LabelControl("Add Verts");
    toolbar->setControl(col++, 0, addVerts );    
    addVerts->addEventHandler( new AddVertsModeHandler());
    
    LabelControl* edit = new LabelControl("Edit");
    toolbar->setControl(col++, 0, edit );    
    edit->addEventHandler(new EditModeHandler());

    unsigned int row = 0;
    Grid *styleBar = createToolBar( );
    styleBar->setPosition(0, 50);
    canvas->addControl( styleBar );
    
    //Make a list of styles
    styleBar->setControl(0, row++, new LabelControl("Styles") );    

    unsigned int numStyles = 8;
    for (unsigned int i = 0; i < numStyles; ++i)
    {
        float w = 50;
        osg::Vec4 color = randomColor();

        float widths[3] = {2, 4, 8};

        unsigned int r = row++;
        for (unsigned int j = 0; j < 3; j++) 
        {
            Control* l = new Control();            
            l->setBackColor( color );
            l->addEventHandler(new ChangeStyleHandler(buildStyle( color, widths[j] ) ));
            l->setSize(w,5 * widths[j]);
            styleBar->setControl(j, r, l);
        }
    }
   
    
    viewer.setSceneData( s_root.get() );
    viewer.setCameraManipulator( new EarthManipulator() );

    if ( s_mapNode )
        viewer.getCamera()->addCullCallback( new osgEarth::Util::AutoClipPlaneCullCallback(s_mapNode) );

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    return viewer.run();
}
