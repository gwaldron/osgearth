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
#include <osgEarth/Map>
#include <osgEarth/MapNode>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/Controls>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/StateSetManipulator>
#include <osgDB/ReadFile>

using namespace osgEarth;
using namespace osgEarth::Util::Controls;

osg::Node* createControlPanel( osgViewer::View* );
void updateControlPanel();

static osg::ref_ptr<Map> s_activeMap;
static osg::ref_ptr<Map> s_inactiveMap;
static Grid* s_layerBox;
static bool s_updateRequired = false;

//------------------------------------------------------------------------

struct MyMapListener : public MapCallback
{
    void onMapModelChanged( const MapModelChange& change ) {
        s_updateRequired = true;
    }
};

//------------------------------------------------------------------------

int
main( int argc, char** argv )
{
    osg::ArgumentParser arguments( &argc,argv );

    // load a graph from the command line
    osg::Node* node = osgDB::readNodeFiles( arguments );

    // make sure we loaded a .earth file
    osgEarth::MapNode* mapNode = MapNode::findMapNode( node );
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
    

    // configure the viewer.
    osgViewer::Viewer viewer( arguments );

    osg::Group* root = new osg::Group();

    // install the control panel
    root->addChild( createControlPanel( &viewer ) );
    root->addChild( node );

    // update the control panel with the two Maps:
    updateControlPanel();
    
    viewer.addEventHandler( new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );
    viewer.addEventHandler( new osgViewer::StatsHandler() );

    viewer.setSceneData( root );

    // install a proper manipulator
    viewer.setCameraManipulator( new osgEarth::Util::EarthManipulator() );


    while( !viewer.done() )
    {
        viewer.frame();

        if ( s_updateRequired )
        {
            updateControlPanel();
            s_updateRequired = false;
        }
    }
}

//------------------------------------------------------------------------

struct LayerEnabledHandler : public ControlEventHandler
{
    LayerEnabledHandler( ImageLayer* layer ) : _layer(layer) { }
    void onValueChanged( Control* control, bool value ) {
        _layer->setEnabled( value );
    }
    ImageLayer* _layer;
};

struct LayerOpacityHandler : public ControlEventHandler
{
    LayerOpacityHandler( ImageLayer* layer ) : _layer(layer) { }
    void onValueChanged( Control* control, float value ) {
        _layer->setOpacity( value );
    }
    ImageLayer* _layer;
};

struct AddLayerHandler : public ControlEventHandler
{
    AddLayerHandler( ImageLayer* layer ) : _layer(layer) { }
    void onClick( Control* control, int mouseButtonMask ) {
        s_inactiveMap->removeImageLayer( _layer.get() );
        s_activeMap->addImageLayer( _layer.get() );
    }
    osg::ref_ptr<ImageLayer> _layer;
};

struct RemoveLayerHandler : public ControlEventHandler
{
    RemoveLayerHandler( ImageLayer* layer ) : _layer(layer) { }
    void onClick( Control* control, int mouseButtonMask ) {
        s_inactiveMap->addImageLayer( _layer.get() );
        s_activeMap->removeImageLayer( _layer.get() );
    }
    osg::ref_ptr<ImageLayer> _layer;
};

struct MoveLayerHandler : public ControlEventHandler
{
    MoveLayerHandler( ImageLayer* layer, int newIndex ) : _layer(layer), _newIndex(newIndex) { }
    void onClick( Control* control, int mouseButtonMask ) {
        s_activeMap->moveImageLayer( _layer, _newIndex );
    }
    ImageLayer* _layer;
    int _newIndex;
};

//------------------------------------------------------------------------


osg::Node*
createControlPanel( osgViewer::View* view )
{
    ControlCanvas* canvas = new ControlCanvas( view );

    // the outer container:
    s_layerBox = new Grid();
    s_layerBox->setBackColor(0,0,0,0.5);
    s_layerBox->setMargin( 10 );
    s_layerBox->setPadding( 10 );
    s_layerBox->setSpacing( 10 );
    s_layerBox->setChildVertAlign( Control::ALIGN_CENTER );
    s_layerBox->setAbsorbEvents( true );
    s_layerBox->setVertAlign( Control::ALIGN_BOTTOM );

    canvas->addControl( s_layerBox );
    return canvas;
}

void
createLayerItem( int gridRow, int layerIndex, int numLayers, ImageLayer* layer, bool isActive )
{
    // a checkbox to enable/disable the layer:
    CheckBoxControl* enabled = new CheckBoxControl( layer->getEnabled() );
    enabled->addEventHandler( new LayerEnabledHandler(layer) );
    s_layerBox->setControl( 0, gridRow, enabled );

    // the layer name
    LabelControl* name = new LabelControl( layer->getName() );
    s_layerBox->setControl( 1, gridRow, name );

    // an opacity slider
    HSliderControl* opacity = new HSliderControl( 0.0f, 1.0f, layer->getOpacity() );
    opacity->setWidth( 125 );
    opacity->setHeight( 12 );
    opacity->addEventHandler( new LayerOpacityHandler(layer) );
    s_layerBox->setControl( 2, gridRow, opacity );

    // move buttons
    if ( layerIndex < numLayers-1 && isActive )
    {
        LabelControl* upButton = new LabelControl( "UP", 14 );
        upButton->setBackColor( .4,.4,.4,1 );
        upButton->setActiveColor( .8,0,0,1 );
        upButton->addEventHandler( new MoveLayerHandler( layer, layerIndex+1 ) );
        s_layerBox->setControl( 3, gridRow, upButton );
    }
    if ( layerIndex > 0 && isActive)
    {
        LabelControl* upButton = new LabelControl( "DOWN", 14 );
        upButton->setBackColor( .4,.4,.4,1 );
        upButton->setActiveColor( .8,0,0,1 );
        upButton->addEventHandler( new MoveLayerHandler( layer, layerIndex-1 ) );
        s_layerBox->setControl( 4, gridRow, upButton );
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
    s_layerBox->setControl( 5, gridRow, addRemove );
}

void
updateControlPanel()
{
    // erase all child controls and just rebuild them b/c we're lazy.
    s_layerBox->clearControls();

    int row = 0;

    LabelControl* activeLabel = new LabelControl( "Map Layers", 20, osg::Vec4f(1,1,0,1) );
    s_layerBox->setControl( 1, row++, activeLabel );

    // the active map layers:
    MapFrame mapf( s_activeMap.get(), Map::IMAGE_LAYERS );
    int layerNum = mapf.imageLayers().size()-1;
    for( ImageLayerVector::const_reverse_iterator i = mapf.imageLayers().rbegin(); i != mapf.imageLayers().rend(); ++i )
        createLayerItem( row++, layerNum--, mapf.imageLayers().size(), i->get(), true );

    MapFrame mapf2( s_inactiveMap.get(), Map::IMAGE_LAYERS );
    if ( mapf2.imageLayers().size() > 0 )
    {
        LabelControl* inactiveLabel = new LabelControl( "Removed:", 18, osg::Vec4f(1,1,0,1) );
        s_layerBox->setControl( 1, row++, inactiveLabel );

        for( unsigned int i=0; i<mapf2.imageLayers().size(); ++i )
        {
            createLayerItem( row++, -1, -1, mapf2.getImageLayerAt(i), false );
        }
    }
}
