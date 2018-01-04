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
#include <osgEarth/ElevationPool>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/Controls>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/ViewFitter>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/StateSetManipulator>
#include <osgDB/ReadFile>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;

void createControlPanel( osgViewer::View* );
void updateControlPanel();

static osg::ref_ptr<Map> s_activeMap;
static Grid* s_masterGrid;
static Grid* s_activeBox;
static Grid* s_inactiveBox;
static bool s_updateRequired = true;
static MapModelChange s_change;
static EarthManipulator* s_manip;
static osgViewer::View* s_view;

typedef std::map<std::string, ConfigOptions> InactiveLayers;
static InactiveLayers _inactive;

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
            ModelSource* ms = modelLayers[i]->getModelSource();
            if ( ms )
            {
                ms->dirty();
            }
            else
            {
                OE_NOTICE << modelLayers[i]->getName()
                    << " has no model source.\n";
            }
        }
    }
};


struct DumpElevation : public osgGA::GUIEventHandler
{
    DumpElevation(MapNode* mapNode, char c) : _mapNode(mapNode), _c(c) { }
    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa, osg::Object*, osg::NodeVisitor*)
    {
        if (ea.getEventType() == ea.KEYDOWN && ea.getKey() == _c)
        {
            osg::Vec3d world;
            _mapNode->getTerrain()->getWorldCoordsUnderMouse(aa.asView(), ea.getX(), ea.getY(), world);
            GeoPoint coords;
            coords.fromWorld(s_activeMap->getSRS(), world);
            osg::ref_ptr<ElevationEnvelope> env = s_activeMap->getElevationPool()->createEnvelope(s_activeMap->getSRS(), 23u);
            float ep_elev = env->getElevation(coords.x(), coords.y());
            OE_NOTICE << "Elevations under mouse. EP=" << ep_elev << "\n";
        }
        return false;
    }
    char _c;
    MapNode* _mapNode;
};

//------------------------------------------------------------------------

int
main( int argc, char** argv )
{
    osg::ArgumentParser arguments( &argc,argv );

    // configure the viewer.
    osgViewer::Viewer viewer( arguments );
    s_view = &viewer;

    // install a motion model
    viewer.setCameraManipulator( s_manip = new osgEarth::Util::EarthManipulator() );

    // disable the small-feature culling (so text will work)
    viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);

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

    osg::Group* root = new osg::Group();

    // install the control panel
    createControlPanel( &viewer );
    root->addChild( loaded );

    // update the control panel with the two Maps:
    updateControlPanel();

    viewer.setSceneData( root );

    // install our control panel updater
    viewer.addUpdateOperation( new UpdateOperation() );

    viewer.addEventHandler(new DumpElevation(mapNode, 'E'));

    viewer.run();
}

//------------------------------------------------------------------------

struct ToggleLayerVisibility : public ControlEventHandler
{
    ToggleLayerVisibility( VisibleLayer* layer ) : _layer(layer) { }
    void onValueChanged( Control* control, bool value )
    {
        _layer->setVisible( value );
    }
    VisibleLayer* _layer;
};

struct LayerOpacityHandler : public ControlEventHandler
{
    LayerOpacityHandler( VisibleLayer* layer ) : _layer(layer) { }
    void onValueChanged( Control* control, float value )
    {
        _layer->setOpacity( value );
    }
    VisibleLayer* _layer;
};

struct AddLayerHandler : public ControlEventHandler
{
    AddLayerHandler(const ConfigOptions& lc) : _lc(lc) { }

    void onClick( Control* control, int mouseButtonMask )
    {
        Layer* layer = Layer::create(_lc);
        if (layer)
        {
            s_activeMap->addLayer(layer);
            _inactive.erase(layer->getName());
        }
    }

    ConfigOptions _lc;
};

struct RemoveLayerHandler : public ControlEventHandler
{
    RemoveLayerHandler( Layer* layer ) : _layer(layer) { }

    void onClick( Control* control, int mouseButtonMask )
    {
        _inactive[_layer->getName()] = _layer->getConfig(); // save it
        s_activeMap->removeLayer(_layer.get()); // and remove it
    }
    osg::ref_ptr<Layer> _layer;
};

struct MoveLayerHandler : public ControlEventHandler
{
    MoveLayerHandler( Layer* layer, int newIndex ) : _layer(layer), _newIndex(newIndex) { }
    void onClick( Control* control, int mouseButtonMask )
    {
        s_activeMap->moveLayer(_layer, _newIndex);
    }
    Layer* _layer;
    int _newIndex;
};

struct ZoomLayerHandler : public ControlEventHandler
{
    ZoomLayerHandler(Layer* layer) : _layer(layer) { }
    void onClick(Control* control)
    {
        const GeoExtent& extent = _layer->getExtent();
        if (extent.isValid())
        {
            ViewFitter fitter(s_activeMap->getSRS(), s_view->getCamera());
            std::vector<GeoPoint> points;
            points.push_back(GeoPoint(extent.getSRS(), extent.west(), extent.south()));
            points.push_back(GeoPoint(extent.getSRS(), extent.east(), extent.north()));
            Viewpoint vp;
            if (fitter.createViewpoint(points, vp))
            {
                s_manip->setViewpoint(vp, 2.0);
            }
        }
    }
    Layer* _layer;
};

//------------------------------------------------------------------------


void
createControlPanel( osgViewer::View* view )
{
    ControlCanvas* canvas = ControlCanvas::getOrCreate( view );

    s_masterGrid = new Grid();
    s_masterGrid->setMargin( 5 );
    s_masterGrid->setPadding( 5 );
    s_masterGrid->setChildSpacing( 10 );
    s_masterGrid->setChildVertAlign( Control::ALIGN_CENTER );
    s_masterGrid->setAbsorbEvents( true );
    s_masterGrid->setVertAlign( Control::ALIGN_TOP );

    //The Map layers
    s_activeBox = new Grid();
    s_activeBox->setBackColor(0,0,0,0.5);
    s_activeBox->setMargin( 10 );
    s_activeBox->setPadding( 10 );
    s_activeBox->setChildSpacing( 10 );
    s_activeBox->setChildVertAlign( Control::ALIGN_CENTER );
    s_activeBox->setAbsorbEvents( true );
    s_activeBox->setVertAlign( Control::ALIGN_TOP );
    s_masterGrid->setControl( 0, 0, s_activeBox );

    //the removed layers
    s_inactiveBox = new Grid();
    s_inactiveBox->setBackColor(0,0,0,0.5);
    s_inactiveBox->setMargin( 10 );
    s_inactiveBox->setPadding( 10 );
    s_inactiveBox->setChildSpacing( 10 );
    s_inactiveBox->setChildVertAlign( Control::ALIGN_CENTER );
    s_inactiveBox->setAbsorbEvents( true );
    s_inactiveBox->setVertAlign( Control::ALIGN_TOP );
    s_masterGrid->setControl( 0, 1, s_inactiveBox );

    canvas->addControl( s_masterGrid );
}

void
addLayerItem( Grid* grid, int layerIndex, int numLayers, Layer* layer, bool isActive )
{
    int gridCol = 0;
    int gridRow = grid->getNumRows();

    VisibleLayer* visibleLayer = dynamic_cast<VisibleLayer*>(layer);

    // only show layers that derive from VisibleLayer
    if (!visibleLayer)
        return;

    ImageLayer* imageLayer = dynamic_cast<ImageLayer*>(layer);

    // don't show hidden coverage layers
    if (imageLayer && imageLayer->isCoverage() && !imageLayer->getVisible())
        return;
    
    ElevationLayer* elevationLayer = dynamic_cast<ElevationLayer*>(layer);

    // a checkbox to enable/disable the layer:
    if (visibleLayer && layer->getEnabled() && !(imageLayer && imageLayer->isCoverage()))
    {
        CheckBoxControl* enabled = new CheckBoxControl( visibleLayer->getVisible() );
        enabled->addEventHandler( new ToggleLayerVisibility(visibleLayer) );
        grid->setControl( gridCol, gridRow, enabled );
    }
    gridCol++;

    // the layer name
    LabelControl* name = new LabelControl( layer->getName() );
    if (!layer->getEnabled())
        name->setForeColor(osg::Vec4f(1,1,1,0.35));
    grid->setControl( gridCol, gridRow, name );
    gridCol++;

    // layer type
    std::string typeName = typeid(*layer).name();
    typeName = typeName.substr(typeName.find_last_of(":")+1);
    LabelControl* typeLabel = new LabelControl(typeName, osg::Vec4(.5,.7,.5,1));
    grid->setControl( gridCol, gridRow, typeLabel );
    gridCol++;

    // status indicator
    LabelControl* statusLabel =
        layer->getStatus().isError() ? new LabelControl("[error]", osg::Vec4(1,0,0,1)) :
        !layer->getEnabled()?          new LabelControl("[disabled]", osg::Vec4(1,1,1,0.35)) :
                                       new LabelControl("[ok]", osg::Vec4(0,1,0,1)) ;
    grid->setControl( gridCol, gridRow, statusLabel );
    gridCol++;

    if (visibleLayer && !elevationLayer && visibleLayer->getEnabled())
    {
        // an opacity slider
        HSliderControl* opacity = new HSliderControl( 0.0f, 1.0f, visibleLayer->getOpacity() );
        opacity->setWidth( 125 );
        opacity->setHeight( 12 );
        opacity->addEventHandler( new LayerOpacityHandler(visibleLayer) );
        grid->setControl( gridCol, gridRow, opacity );
    }
    gridCol++;

    // zoom button
    if (layer->getExtent().isValid())
    {
        LabelControl* zoomButton = new LabelControl("GO", 14);
        zoomButton->setBackColor( .4,.4,.4,1 );
        zoomButton->setActiveColor( .8,0,0,1 );
        zoomButton->addEventHandler( new ZoomLayerHandler(layer) );
        grid->setControl( gridCol, gridRow, zoomButton );
    }
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

    if (layer->getStatus().isError())
    {
        grid->setControl(gridCol, gridRow, new LabelControl(layer->getStatus().message(), osg::Vec4(1,.2,.2,1)));
    }
}

void
createInactiveLayerItem( Grid* grid, int gridRow, const std::string& name, const ConfigOptions& lc )
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
updateControlPanel()
{
    // erase all child controls and just rebuild them b/c we're lazy.

    //Rebuild all the image layers    
    s_activeBox->clearControls();

    int row = 0;

    LabelControl* activeLabel = new LabelControl( "Map Layers", 20, osg::Vec4f(1,1,0,1) );
    s_activeBox->setControl( 1, row++, activeLabel );

    // the active map layers:
    MapFrame mapf( s_activeMap.get() );

    const LayerVector& layers = mapf.layers();
    for (int i = layers.size()-1; i >= 0; --i)
    {
        Layer* layer = layers[i].get();
        addLayerItem(s_activeBox, i, layers.size(), layer, true);

        if (layer->getStatus().isError())
        {
            OE_WARN << layer->getName() << " : " << layer->getStatus().toString() << "\n";
        }
    }

    // inactive layers:
    s_inactiveBox->clearControls();

    if (!_inactive.empty())
    {
        s_inactiveBox->setControl(0, row++, new LabelControl("Removed:", 18, osg::Vec4f(1,1,0,1)));
        for (InactiveLayers::const_iterator i = _inactive.begin(); i != _inactive.end(); ++i)
        {
            createInactiveLayerItem(s_inactiveBox, row++, i->first, i->second);
        }
    }

    s_inactiveBox->setVisible(!_inactive.empty());
}
