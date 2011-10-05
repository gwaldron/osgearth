/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2010 Pelican Mapping
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

#include <osgEarthSymbology/Style>
#include <osgEarthFeatures/FeatureModelGraph>
#include <osgEarthFeatures/FeatureListSource>
#include <osgEarthFeatures/GeometryCompiler>

#include <osgEarthDrivers/gdal/GDALOptions>
#include <osgEarthDrivers/feature_ogr/OGRFeatureOptions>
#include <osgEarthDrivers/agglite/AGGLiteOptions>
#include <osgEarthDrivers/model_feature_geom/FeatureGeomModelOptions>
#include <osgEarthDrivers/model_feature_stencil/FeatureStencilModelOptions>

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
static osg::ref_ptr< Feature > s_activeFeature;
static osgViewer::Viewer* s_viewer;
static osg::ref_ptr< osg::Group > s_root;
static osg::ref_ptr< FeatureListSource > s_source;
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
    AddVertsModeHandler( FeatureModelGraph* featureGraph)
        : _featureGraph( featureGraph )
    {
    }

    void onClick( Control* control, int mouseButtonMask ) {

        //remove the editor if it's valid
        if (s_editor.valid())
        {
            s_root->removeChild( s_editor.get() );
            s_editor = NULL;

            Style* style = _featureGraph->getStyles()->getDefaultStyle();
            if ( style )
            {            
                style->get<LineSymbol>()->stroke()->stipple().unset();
                _featureGraph->dirty();
            }
        }

        //Add the new add point handler
        if (!s_addPointHandler.valid() && s_activeFeature.valid())
        {
            s_addPointHandler = new AddPointHandler(s_activeFeature.get(), s_source.get(), s_mapNode->getMap()->getProfile()->getSRS());
            s_addPointHandler->setIntersectionMask( 0x1 );
            s_viewer->addEventHandler( s_addPointHandler.get() );
        }        
    }

    osg::ref_ptr< FeatureModelGraph > _featureGraph;
};

struct EditModeHandler : public ControlEventHandler
{
    EditModeHandler( FeatureModelGraph* featureGraph)
        : _featureGraph( featureGraph )
    { 
    }

    void onClick( Control* control, int mouseButtonMask ) {
        
        //Remove the add point handler if it's valid
        if (s_addPointHandler.valid())
        {            
            osgEarth::removeEventHandler( s_viewer, s_addPointHandler.get() );
            s_addPointHandler = NULL;
        }        

        if (!s_editor.valid() && s_activeFeature.valid())
        {
            Style* style = _featureGraph->getStyles()->getDefaultStyle();
            if ( style )
            {
                style->get<LineSymbol>()->stroke()->stipple() = 0x00FF;
                _featureGraph->dirty();
            }
            s_editor = new FeatureEditor(s_activeFeature.get(), s_source.get(), s_mapNode.get());
            s_root->addChild( s_editor.get() );
        }
    }

    osg::ref_ptr< FeatureModelGraph > _featureGraph;
};

struct ChangeStyleHandler : public ControlEventHandler
{
    ChangeStyleHandler( FeatureModelGraph* features, StyleSheet* styleSheet) 
        : _features( features), _styleSheet(styleSheet)
    {
        //nop
    }

    void onClick( Control* control, int mouseButtonMask ) {
        _features->setStyles( _styleSheet.get() );
    }

    osg::ref_ptr< FeatureModelGraph > _features;
    osg::ref_ptr< StyleSheet >        _styleSheet;
};

StyleSheet* buildStyleSheet( const osg::Vec4 &color, float width )
{
    // Define a style for the feature data. Since we are going to render the
    // vectors as lines, configure the line symbolizer:
    Style style;

    LineSymbol* ls = style.getOrCreateSymbol<LineSymbol>();
    ls->stroke()->color() = color;
    ls->stroke()->width() = width;

    //AltitudeSymbol* as = style.getOrCreate<AltitudeSymbol>();
    //as->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;

    StyleSheet* styleSheet = new StyleSheet();
    styleSheet->addStyle( style );
    return styleSheet;
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

        
    // Define a style for the feature data. Since we are going to render the
    // vectors as lines, configure the line symbolizer:
    StyleSheet* styleSheet = buildStyleSheet( Color::Yellow, 2.0f );

    s_source = new FeatureListSource();

    LineString* line = new LineString();
    line->push_back( osg::Vec3d(-60, 20, 0) );
    line->push_back( osg::Vec3d(-120, 20, 0) );
    line->push_back( osg::Vec3d(-120, 60, 0) );
    line->push_back( osg::Vec3d(-60, 60, 0) );
    Feature *feature = new Feature(s_fid++);
    feature->setGeometry( line );
    s_source->insertFeature( feature );
    s_activeFeature = feature;
  
    s_root = new osg::Group;
    s_root->addChild( s_mapNode.get() );

    Session* session = new Session(s_mapNode->getMap(), styleSheet);

    FeatureModelGraph* graph = new FeatureModelGraph( 
        s_source.get(), 
        FeatureModelSourceOptions(), 
        new GeomFeatureNodeFactory(),
        session );

    graph->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    graph->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);

    s_root->addChild( graph );

    //Setup the controls
    ControlCanvas* canvas = ControlCanvas::get( &viewer );
    s_root->addChild( canvas );
    Grid *toolbar = createToolBar( );
    canvas->addControl( toolbar );
    canvas->setNodeMask( 0x1 << 1 );



    int col = 0;
    LabelControl* addVerts = new LabelControl("Add Verts");
    toolbar->setControl(col++, 0, addVerts );    
    addVerts->addEventHandler( new AddVertsModeHandler( graph ));
    
    LabelControl* edit = new LabelControl("Edit");
    toolbar->setControl(col++, 0, edit );    
    edit->addEventHandler(new EditModeHandler( graph ));

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
            l->addEventHandler(new ChangeStyleHandler(graph, buildStyleSheet( color, widths[j] ) ));
            l->setSize(w,5 * widths[j]);
            styleBar->setControl(j, r, l);
        }
    }
   
    
    viewer.setSceneData( s_root.get() );
    viewer.setCameraManipulator( new EarthManipulator() );
    viewer.addEventHandler( new osgEarth::Util::AutoClipPlaneHandler );

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    return viewer.run();
}
