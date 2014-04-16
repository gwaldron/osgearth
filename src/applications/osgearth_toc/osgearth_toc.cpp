/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2013 Pelican Mapping
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
#include <osgEarth/Map>
#include <osgEarth/MapFrame>
#include <osgEarth/MapNode>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/Controls>
#include <osgEarthUtil/ExampleResources>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/StateSetManipulator>
#include <osgDB/ReadFile>

using namespace osgEarth;
using namespace osgEarth::Util::Controls;

void createControlPanel( osgViewer::View* );
void updateControlPanel();

static osg::ref_ptr<Map> s_activeMap;
static osg::ref_ptr<Map> s_inactiveMap;
static Grid* s_masterGrid;
static Grid* s_imageBox;
static Grid* s_elevationBox;
static Grid* s_modelBox;
static bool s_updateRequired = true;

//------------------------------------------------------------------------

struct MyMapListener : public MapCallback
{
    void onMapModelChanged( const MapModelChange& change ) {
        s_updateRequired = true;
    }
};

struct UpdateOperation : public osg::Operation
{
    UpdateOperation() : osg::Operation( "", true ) { }

    void operator()(osg::Object*)
    {
        if ( s_updateRequired )
        {
            updateControlPanel();
            s_updateRequired = false;
        }
    }
};

//------------------------------------------------------------------------

int
main( int argc, char** argv )
{
    osg::ArgumentParser arguments( &argc,argv );

    // configure the viewer.
    osgViewer::Viewer viewer( arguments );

    // install a motion model
    viewer.setCameraManipulator( new osgEarth::Util::EarthManipulator() );

    // Load an earth file 
    osg::Node* loaded = osgEarth::Util::MapNodeHelper().load(arguments, &viewer);
    osgEarth::MapNode* mapNode = osgEarth::MapNode::get(loaded);
    if ( !mapNode ) {
        OE_WARN << "No osgEarth MapNode found in the loaded file(s)." << std::endl;
        return -1;
    }

    // the displayed Map:
    s_activeMap = mapNode->getMap();
    s_activeMap->addMapCallback( new MyMapListener() );

    // a Map to hold inactive layers (layers that have been removed from the displayed Map)
    s_inactiveMap = new Map();
    s_inactiveMap->addMapCallback( new MyMapListener() );

    osg::Group* root = new osg::Group();

    // install the control panel
    createControlPanel( &viewer );
    root->addChild( loaded );

    // update the control panel with the two Maps:
    updateControlPanel();

    viewer.setSceneData( root );

    // install our control panel updater
    viewer.addUpdateOperation( new UpdateOperation() );

    viewer.run();
}

//------------------------------------------------------------------------

struct LayerVisibleHandler : public ControlEventHandler
{
    LayerVisibleHandler( TerrainLayer* layer ) : _layer(layer) { }
    void onValueChanged( Control* control, bool value )
    {
        _layer->setVisible( value );
    }
    TerrainLayer* _layer;
};

struct LayerOpacityHandler : public ControlEventHandler
{
    LayerOpacityHandler( ImageLayer* layer ) : _layer(layer) { }
    void onValueChanged( Control* control, float value )
    {
        _layer->setOpacity( value );
    }
    ImageLayer* _layer;
};

struct ModelLayerVisibleHandler : public ControlEventHandler
{
    ModelLayerVisibleHandler( ModelLayer* layer ) : _layer(layer) { }
    void onValueChanged( Control* control, bool value )
    {
        _layer->setVisible( value );
    }
    ModelLayer* _layer;
};

struct ModelLayerOpacityHandler : public ControlEventHandler
{
    ModelLayerOpacityHandler( ModelLayer* layer ) : _layer(layer) { }
    void onValueChanged( Control* control, float value )
    {
        _layer->setOpacity( value );
    }
    ModelLayer* _layer;
};

struct AddLayerHandler : public ControlEventHandler
{
    AddLayerHandler( TerrainLayer* layer ) : _layer(layer) { }
    void onClick( Control* control, int mouseButtonMask ) {

        ImageLayer* imageLayer = dynamic_cast< ImageLayer*>( _layer.get() );
        ElevationLayer* elevationLayer = dynamic_cast< ElevationLayer*>( _layer.get() );

        if (imageLayer)
        {
            s_inactiveMap->removeImageLayer( imageLayer );
            s_activeMap->addImageLayer( imageLayer );
        }
        else
        {
            s_inactiveMap->removeElevationLayer( elevationLayer );
            s_activeMap->addElevationLayer( elevationLayer );
        }
    }
    osg::ref_ptr<TerrainLayer> _layer;
};

struct RemoveLayerHandler : public ControlEventHandler
{
    RemoveLayerHandler( TerrainLayer* layer ) : _layer(layer) { }
    void onClick( Control* control, int mouseButtonMask ) {        
        ImageLayer* imageLayer = dynamic_cast< ImageLayer*>( _layer.get() );
        ElevationLayer* elevationLayer = dynamic_cast< ElevationLayer*>( _layer.get() );

        if (imageLayer)
        {
            s_inactiveMap->addImageLayer( imageLayer );
            s_activeMap->removeImageLayer( imageLayer );
        }
        else
        {
            s_inactiveMap->addElevationLayer( elevationLayer );
            s_activeMap->removeElevationLayer( elevationLayer );
        }
    }
    osg::ref_ptr<TerrainLayer> _layer;
};

struct MoveLayerHandler : public ControlEventHandler
{
    MoveLayerHandler( TerrainLayer* layer, int newIndex ) : _layer(layer), _newIndex(newIndex) { }
    void onClick( Control* control, int mouseButtonMask ) {
        ImageLayer* imageLayer = dynamic_cast< ImageLayer*>( _layer );
        ElevationLayer* elevationLayer = dynamic_cast< ElevationLayer*>( _layer );

        if (imageLayer)
        {
            s_activeMap->moveImageLayer( imageLayer, _newIndex );
        }
        else
        {
            s_activeMap->moveElevationLayer( elevationLayer, _newIndex );
        }
    }
    TerrainLayer* _layer;
    int _newIndex;
};

//------------------------------------------------------------------------


void
createControlPanel( osgViewer::View* view )
{
    ControlCanvas* canvas = ControlCanvas::get( view );

    s_masterGrid = new Grid();
    s_masterGrid->setBackColor(0,0,0,0.5);
    s_masterGrid->setMargin( 10 );
    s_masterGrid->setPadding( 10 );
    s_masterGrid->setChildSpacing( 10 );
    s_masterGrid->setChildVertAlign( Control::ALIGN_CENTER );
    s_masterGrid->setAbsorbEvents( true );
    s_masterGrid->setVertAlign( Control::ALIGN_BOTTOM );

    //The image layers
    s_imageBox = new Grid();
    s_imageBox->setBackColor(0,0,0,0.5);
    s_imageBox->setMargin( 10 );
    s_imageBox->setPadding( 10 );
    s_imageBox->setChildSpacing( 10 );
    s_imageBox->setChildVertAlign( Control::ALIGN_CENTER );
    s_imageBox->setAbsorbEvents( true );
    s_imageBox->setVertAlign( Control::ALIGN_BOTTOM );
    s_masterGrid->setControl( 0, 0, s_imageBox );

    //the elevation layers
    s_elevationBox = new Grid();
    s_elevationBox->setBackColor(0,0,0,0.5);
    s_elevationBox->setMargin( 10 );
    s_elevationBox->setPadding( 10 );
    s_elevationBox->setChildSpacing( 10 );
    s_elevationBox->setChildVertAlign( Control::ALIGN_CENTER );
    s_elevationBox->setAbsorbEvents( true );
    s_elevationBox->setVertAlign( Control::ALIGN_BOTTOM );
    s_masterGrid->setControl( 1, 0, s_elevationBox );

    //The image layers
    s_modelBox = new Grid();
    s_modelBox->setBackColor(0,0,0,0.5);
    s_modelBox->setMargin( 10 );
    s_modelBox->setPadding( 10 );
    s_modelBox->setChildSpacing( 10 );
    s_modelBox->setChildVertAlign( Control::ALIGN_CENTER );
    s_modelBox->setAbsorbEvents( true );
    s_modelBox->setVertAlign( Control::ALIGN_BOTTOM );
    s_masterGrid->setControl( 2, 0, s_modelBox );

    canvas->addControl( s_masterGrid );
}

void
createLayerItem( Grid* grid, int gridRow, int layerIndex, int numLayers, TerrainLayer* layer, bool isActive )
{
    // a checkbox to enable/disable the layer:
    CheckBoxControl* enabled = new CheckBoxControl( layer->getVisible() );
    enabled->addEventHandler( new LayerVisibleHandler(layer) );
    grid->setControl( 0, gridRow, enabled );

    // the layer name
    LabelControl* name = new LabelControl( layer->getName() );
    grid->setControl( 1, gridRow, name );

    ImageLayer* imageLayer = dynamic_cast< ImageLayer* > (layer );
    if (imageLayer)
    {
        // an opacity slider
        HSliderControl* opacity = new HSliderControl( 0.0f, 1.0f, imageLayer->getOpacity() );
        opacity->setWidth( 125 );
        opacity->setHeight( 12 );
        opacity->addEventHandler( new LayerOpacityHandler(imageLayer) );
        grid->setControl( 2, gridRow, opacity );
    }

    // move buttons
    if ( layerIndex < numLayers-1 && isActive )
    {
        LabelControl* upButton = new LabelControl( "UP", 14 );
        upButton->setBackColor( .4,.4,.4,1 );
        upButton->setActiveColor( .8,0,0,1 );
        upButton->addEventHandler( new MoveLayerHandler( layer, layerIndex+1 ) );
        grid->setControl( 3, gridRow, upButton );
    }
    if ( layerIndex > 0 && isActive)
    {
        LabelControl* upButton = new LabelControl( "DOWN", 14 );
        upButton->setBackColor( .4,.4,.4,1 );
        upButton->setActiveColor( .8,0,0,1 );
        upButton->addEventHandler( new MoveLayerHandler( layer, layerIndex-1 ) );
        grid->setControl( 4, gridRow, upButton );
    }

    // add/remove button:
    LabelControl* addRemove = new LabelControl( isActive? "REMOVE" : "ADD", 14 );
    addRemove->setHorizAlign( Control::ALIGN_CENTER );
    addRemove->setBackColor( .4,.4,.4,1 );
    addRemove->setActiveColor( .8,0,0,1 );
    if ( isActive )
        addRemove->addEventHandler( new RemoveLayerHandler(layer) );
    else
        addRemove->addEventHandler( new AddLayerHandler(layer) );
    grid->setControl( 5, gridRow, addRemove );
}

void
createModelLayerItem( Grid* grid, int gridRow, ModelLayer* layer, bool isActive )
{
    // a checkbox to enable/disable the layer:
    CheckBoxControl* enabled = new CheckBoxControl( layer->getVisible() );
    enabled->addEventHandler( new ModelLayerVisibleHandler(layer) );
    grid->setControl( 0, gridRow, enabled );

    // the layer name
    LabelControl* name = new LabelControl( layer->getName() );
    grid->setControl( 1, gridRow, name );

    // an opacity slider
    HSliderControl* opacity = new HSliderControl( 0.0f, 1.0f, layer->getOpacity() );
    opacity->setWidth( 125 );
    opacity->setHeight( 12 );
    opacity->addEventHandler( new ModelLayerOpacityHandler(layer) );
    grid->setControl( 2, gridRow, opacity );
}

void
updateControlPanel()
{
    // erase all child controls and just rebuild them b/c we're lazy.

    //Rebuild all the image layers    
    s_imageBox->clearControls();

    int row = 0;

    LabelControl* activeLabel = new LabelControl( "Image Layers", 20, osg::Vec4f(1,1,0,1) );
    s_imageBox->setControl( 1, row++, activeLabel );

    // the active map layers:
    MapFrame mapf( s_activeMap.get(), Map::ENTIRE_MODEL );
    int layerNum = mapf.imageLayers().size()-1;
    for( ImageLayerVector::const_reverse_iterator i = mapf.imageLayers().rbegin(); i != mapf.imageLayers().rend(); ++i )
        createLayerItem( s_imageBox, row++, layerNum--, mapf.imageLayers().size(), i->get(), true );

    MapFrame mapf2( s_inactiveMap.get() );
    if ( mapf2.imageLayers().size() > 0 )
    {
        LabelControl* inactiveLabel = new LabelControl( "Removed:", 18, osg::Vec4f(1,1,0,1) );
        s_imageBox->setControl( 1, row++, inactiveLabel );

        for( unsigned int i=0; i<mapf2.imageLayers().size(); ++i )
        {
            createLayerItem( s_imageBox, row++, -1, -1, mapf2.getImageLayerAt(i), false );
        }
    }




    //Rebuild the elevation layers
    s_elevationBox->clearControls();

    row = 0;

    activeLabel = new LabelControl( "Elevation Layers", 20, osg::Vec4f(1,1,0,1) );
    s_elevationBox->setControl( 1, row++, activeLabel );

    // the active map layers:
    layerNum = mapf.elevationLayers().size()-1;
    for( ElevationLayerVector::const_reverse_iterator i = mapf.elevationLayers().rbegin(); i != mapf.elevationLayers().rend(); ++i )
        createLayerItem( s_elevationBox, row++, layerNum--, mapf.elevationLayers().size(), i->get(), true );

    if ( mapf2.elevationLayers().size() > 0 )
    {
        LabelControl* inactiveLabel = new LabelControl( "Removed:", 18, osg::Vec4f(1,1,0,1) );
        s_elevationBox->setControl( 1, row++, inactiveLabel );

        for( unsigned int i=0; i<mapf2.elevationLayers().size(); ++i )
        {
            createLayerItem( s_elevationBox, row++, -1, -1, mapf2.getElevationLayerAt(i), false );
        }
    }



    //Rebuild the model layers
    s_modelBox->clearControls();

    row = 0;

    activeLabel = new LabelControl( "Model Layers", 20, osg::Vec4f(1,1,0,1) );
    s_modelBox->setControl( 1, row++, activeLabel );

    // the active map layers:
    for( ModelLayerVector::const_reverse_iterator i = mapf.modelLayers().rbegin(); i != mapf.modelLayers().rend(); ++i )
        createModelLayerItem( s_modelBox, row++, i->get(), true );
}
