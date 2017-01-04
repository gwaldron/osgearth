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
#include <osgEarth/Map>
#include <osgEarth/MapFrame>
#include <osgEarth/MapNode>
#include <osgEarth/MapModelChange>
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
static Grid* s_masterGrid;
static Grid* s_imageBox;
static Grid* s_elevationBox;
static Grid* s_modelBox;
static bool s_updateRequired = true;
static MapModelChange s_change;

enum LayerType {
    IMAGE_LAYER = 0,
    ELEVATION_LAYER = 1,
    MODEL_LAYER = 2
};
struct LayerConfiguration {
    ConfigOptions _options;
    LayerType _type;
};
typedef std::map<std::string, LayerConfiguration> InactiveLayers;
static InactiveLayers _inactive;

std::string layerTypeNames[3] = {
    "image",
    "elevation",
    "model"
};

//------------------------------------------------------------------------

void updateControlPanel();


struct MyMapListener : public MapCallback
{
    void onMapModelChanged( const MapModelChange& change ) {
        s_updateRequired = true;
        s_change = change;
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

            if (s_change.getElevationLayer())
            {
                OE_NOTICE << "Dirtying model layers.\n";
                dirtyModelLayers();
            }
        }
    }

    void dirtyModelLayers()
    {
        ModelLayerVector modelLayers;
        s_activeMap->getLayers(modelLayers);

        for(unsigned i=0; i<modelLayers.size(); ++i)
        {
            ModelSource* ms = modelLayers.at(i)->getModelSource();
            if ( ms )
            {
                ms->dirty();
            }
            else
            {
                OE_NOTICE << modelLayers.at(i)->getName()
                    << " has no model source.\n";
            }
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
    //s_inactiveMap = new Map();
    //s_inactiveMap->addMapCallback( new MyMapListener() );

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
    AddLayerHandler(const LayerConfiguration& lc) : _lc(lc) { }

    void onClick( Control* control, int mouseButtonMask ) {
        Layer* layer = 0L;
        if (_lc._type == IMAGE_LAYER) {
            layer = new ImageLayer(_lc._options);
        }
        else if (_lc._type == ELEVATION_LAYER) {
            layer = new ElevationLayer(_lc._options);
        }
        else {
            return;
        }
        s_activeMap->addLayer(layer);
        _inactive.erase(layer->getName());
    }

    LayerConfiguration _lc;
};

struct RemoveLayerHandler : public ControlEventHandler
{
    RemoveLayerHandler( TerrainLayer* layer ) : _layer(layer) { }

    void onClick( Control* control, int mouseButtonMask ) {
        LayerConfiguration& lc = _inactive[_layer->getName()];
        lc._type =
            dynamic_cast<ImageLayer*>(_layer.get())? IMAGE_LAYER :
            dynamic_cast<ElevationLayer*>(_layer.get()) ? ELEVATION_LAYER :
            MODEL_LAYER;
        lc._options = _layer->getTerrainLayerOptions();

        s_activeMap->removeLayer(_layer.get());
    }
    osg::ref_ptr<TerrainLayer> _layer;
};

struct MoveLayerHandler : public ControlEventHandler
{
    MoveLayerHandler( TerrainLayer* layer, int newIndex ) : _layer(layer), _newIndex(newIndex) { }
    void onClick( Control* control, int mouseButtonMask ) {
        s_activeMap->moveLayer(_layer, _newIndex);
    }
    TerrainLayer* _layer;
    int _newIndex;
};

//------------------------------------------------------------------------


void
createControlPanel( osgViewer::View* view )
{
    ControlCanvas* canvas = ControlCanvas::getOrCreate( view );

    s_masterGrid = new Grid();
    s_masterGrid->setBackColor(0,0,0,0.5);
    s_masterGrid->setMargin( 10 );
    s_masterGrid->setPadding( 10 );
    s_masterGrid->setChildSpacing( 10 );
    s_masterGrid->setChildVertAlign( Control::ALIGN_CENTER );
    s_masterGrid->setAbsorbEvents( true );
    s_masterGrid->setVertAlign( Control::ALIGN_TOP );

    //The image layers
    s_imageBox = new Grid();
    s_imageBox->setBackColor(0,0,0,0.5);
    s_imageBox->setMargin( 10 );
    s_imageBox->setPadding( 10 );
    s_imageBox->setChildSpacing( 10 );
    s_imageBox->setChildVertAlign( Control::ALIGN_CENTER );
    s_imageBox->setAbsorbEvents( true );
    s_imageBox->setVertAlign( Control::ALIGN_TOP );
    s_masterGrid->setControl( 0, 0, s_imageBox );

    //the elevation layers
    s_elevationBox = new Grid();
    s_elevationBox->setBackColor(0,0,0,0.5);
    s_elevationBox->setMargin( 10 );
    s_elevationBox->setPadding( 10 );
    s_elevationBox->setChildSpacing( 10 );
    s_elevationBox->setChildVertAlign( Control::ALIGN_CENTER );
    s_elevationBox->setAbsorbEvents( true );
    s_elevationBox->setVertAlign( Control::ALIGN_TOP );
    s_masterGrid->setControl( 1, 0, s_elevationBox );

    //The image layers
    s_modelBox = new Grid();
    s_modelBox->setBackColor(0,0,0,0.5);
    s_modelBox->setMargin( 10 );
    s_modelBox->setPadding( 10 );
    s_modelBox->setChildSpacing( 10 );
    s_modelBox->setChildVertAlign( Control::ALIGN_CENTER );
    s_modelBox->setAbsorbEvents( true );
    s_modelBox->setVertAlign( Control::ALIGN_TOP );
    s_masterGrid->setControl( 2, 0, s_modelBox );

    canvas->addControl( s_masterGrid );
}

void
createLayerItem( Grid* grid, int gridRow, int layerIndex, int numLayers, TerrainLayer* layer, bool isActive )
{
    int gridCol = 0;

    // layer type
    std::string typeName = dynamic_cast<ImageLayer*>(layer) ? "image" : dynamic_cast<ElevationLayer*>(layer) ? "elevation" : "other";
    LabelControl* typeLabel = new LabelControl(typeName, osg::Vec4(.5,.7,.5,1));
    grid->setControl( gridCol++, gridRow, typeLabel );

    // a checkbox to enable/disable the layer:
    CheckBoxControl* enabled = new CheckBoxControl( layer->getVisible() );
    enabled->addEventHandler( new LayerVisibleHandler(layer) );
    grid->setControl( gridCol++, gridRow, enabled );

    // the layer name
    LabelControl* name = new LabelControl( layer->getName() );
    grid->setControl( gridCol, gridRow, name );
    gridCol++;

    ImageLayer* imageLayer = dynamic_cast< ImageLayer* > (layer );
    if (imageLayer)
    {
        // an opacity slider
        HSliderControl* opacity = new HSliderControl( 0.0f, 1.0f, imageLayer->getOpacity() );
        opacity->setWidth( 125 );
        opacity->setHeight( 12 );
        opacity->addEventHandler( new LayerOpacityHandler(imageLayer) );
        grid->setControl( gridCol, gridRow, opacity );
    }
    gridCol++;

    // status indicator
    LabelControl* statusLabel = layer->getStatus().isOK()
        ? new LabelControl("[ok]", osg::Vec4(0,1,0,1))
        : new LabelControl("[error]", osg::Vec4(1,0,0,1));
    grid->setControl( gridCol, gridRow, statusLabel );
    gridCol++;

    // move buttons
    if ( layerIndex < numLayers-1 && isActive )
    {
        LabelControl* upButton = new LabelControl( "UP", 14 );
        upButton->setBackColor( .4,.4,.4,1 );
        upButton->setActiveColor( .8,0,0,1 );
        upButton->addEventHandler( new MoveLayerHandler( layer, layerIndex+1 ) );
        grid->setControl( gridCol, gridRow, upButton );
    }
    gridCol++;

    if ( layerIndex > 0 && isActive)
    {
        LabelControl* upButton = new LabelControl( "DOWN", 14 );
        upButton->setBackColor( .4,.4,.4,1 );
        upButton->setActiveColor( .8,0,0,1 );
        upButton->addEventHandler( new MoveLayerHandler( layer, layerIndex-1 ) );
        grid->setControl( gridCol, gridRow, upButton );
    }
    gridCol++;

    // add/remove button:
    LabelControl* addRemove = new LabelControl( isActive? "REMOVE" : "ADD", 14 );
    addRemove->setHorizAlign( Control::ALIGN_CENTER );
    addRemove->setBackColor( .4,.4,.4,1 );
    addRemove->setActiveColor( .8,0,0,1 );
    addRemove->addEventHandler( new RemoveLayerHandler(layer) );

    grid->setControl( gridCol, gridRow, addRemove );
    gridCol++;
}

void
createInactiveLayerItem( Grid* grid, int gridRow, const std::string& name, const LayerConfiguration& lc )
{
    int gridCol = 0;

    // the layer name
    LabelControl* nameLabel = new LabelControl( name );
    grid->setControl( gridCol, gridRow, nameLabel );
    gridCol++;
    
    LabelControl* addRemove = new LabelControl( "ADD", 14 );
    addRemove->setHorizAlign( Control::ALIGN_CENTER );
    addRemove->setBackColor( .4,.4,.4,1 );
    addRemove->setActiveColor( .8,0,0,1 );
    addRemove->addEventHandler( new AddLayerHandler(lc) );
    grid->setControl( gridCol, gridRow, addRemove );
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

    LabelControl* statusLabel = layer->getStatus().isOK()
        ? new LabelControl("[ok]", osg::Vec4(0, 1, 0, 1))
        : new LabelControl("[error]", osg::Vec4(1, 0, 0, 1));
    grid->setControl(2, gridRow, statusLabel);

    // an opacity slider
    HSliderControl* opacity = new HSliderControl( 0.0f, 1.0f, layer->getOpacity() );
    opacity->setWidth( 125 );
    opacity->setHeight( 12 );
    opacity->addEventHandler( new ModelLayerOpacityHandler(layer) );
    grid->setControl( 3, gridRow, opacity );
}

void
updateControlPanel()
{
    // erase all child controls and just rebuild them b/c we're lazy.

    //Rebuild all the image layers    
    s_imageBox->clearControls();

    int row = 0;

    LabelControl* activeLabel = new LabelControl( "Map Layers", 20, osg::Vec4f(1,1,0,1) );
    s_imageBox->setControl( 1, row++, activeLabel );

    // the active map layers:
    MapFrame mapf( s_activeMap.get() );

    const LayerVector& layers = mapf.layers();
    for (int i = layers.size()-1; i >= 0; --i)
    {
        Layer* layer = layers[i].get();
        if (dynamic_cast<ImageLayer*>(layer))
        {
            createLayerItem(s_imageBox, row++, i, layers.size(), dynamic_cast<ImageLayer*>(layer), true);
        }
        else if (dynamic_cast<ElevationLayer*>(layer))
        {
            createLayerItem( s_imageBox, row++, i, layers.size(), dynamic_cast<ElevationLayer*>(layer), true );
        }
        else if (dynamic_cast<ModelLayer*>(layer))
        {
            createModelLayerItem( s_imageBox, row++, dynamic_cast<ModelLayer*>(layer), true );
        }
    }

    // inactive layers:
    if (!_inactive.empty())
    {
        s_imageBox->setControl(0, row++, new LabelControl("Removed:", 18, osg::Vec4f(1,1,0,1)));
        for (InactiveLayers::const_iterator i = _inactive.begin(); i != _inactive.end(); ++i)
        {
            createInactiveLayerItem(s_imageBox, row++, i->first, i->second);
        }
    }

#if 0
    ImageLayerVector imageLayers;
    mapf.getLayers(imageLayers);
    int layerNum = imageLayers.size()-1;
    for( ImageLayerVector::const_reverse_iterator i = imageLayers.rbegin(); i != imageLayers.rend(); ++i )
        createLayerItem( s_imageBox, row++, layerNum--, imageLayers.size(), i->get(), true );

    // inactive layers:
    if (!_inactive.empty())
    {
        s_imageBox->setControl(0, row++, new LabelControl("Removed:", 18, osg::Vec4f(1,1,0,1)));
        for (InactiveLayers::const_iterator i = _inactive.begin(); i != _inactive.end(); ++i)
        {
            createInactiveLayerItem(s_imageBox, row++, i->first, i->second);
        }
    }

    //Rebuild the elevation layers
    s_elevationBox->clearControls();

    row = 0;

    activeLabel = new LabelControl( "Elevation Layers", 20, osg::Vec4f(1,1,0,1) );
    s_elevationBox->setControl( 1, row++, activeLabel );

    // the active map layers:
    ElevationLayerVector elevationLayers;
    mapf.getLayers(elevationLayers);

    layerNum = elevationLayers.size()-1;
    for( ElevationLayerVector::const_reverse_iterator i = elevationLayers.rbegin(); i != elevationLayers.rend(); ++i )
        createLayerItem( s_elevationBox, row++, layerNum--, elevationLayers.size(), i->get(), true );

    //Rebuild the model layers
    s_modelBox->clearControls();

    row = 0;

    activeLabel = new LabelControl( "Model Layers", 20, osg::Vec4f(1,1,0,1) );
    s_modelBox->setControl( 1, row++, activeLabel );

    // the active map layers:
    ModelLayerVector modelLayers;
    mapf.getLayers(modelLayers);
    for( ModelLayerVector::const_reverse_iterator i = modelLayers.rbegin(); i != modelLayers.rend(); ++i )
        createModelLayerItem( s_modelBox, row++, i->get(), true );
#endif
}
