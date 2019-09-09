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
#include <osgEarth/Map>
#include <osgEarth/MapNode>
#include <osgEarth/MapModelChange>
#include <osgEarth/ElevationPool>
#include <osgEarth/XmlUtils>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/Controls>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/ViewFitter>
#include <osgEarthAnnotation/LabelNode>
#include <osgEarthAnnotation/AnnotationLayer>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/StateSetManipulator>
#include <osgDB/ReadFile>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;
using namespace osgEarth::Annotation;

void createControlPanel(Container*);
void updateControlPanel();

static osg::ref_ptr<Map> s_activeMap;
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

struct DumpLabel : public osgGA::GUIEventHandler
{
    DumpLabel(MapNode* mapNode, char c) : _mapNode(mapNode), _c(c), _layer(0L) { }

    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa, osg::Object*, osg::NodeVisitor*)
    {
        if (ea.getEventType() == ea.KEYDOWN && ea.getKey() == _c)
        {
            osg::Vec3d world;
            _mapNode->getTerrain()->getWorldCoordsUnderMouse(aa.asView(), ea.getX(), ea.getY(), world);

            GeoPoint coords;
            coords.fromWorld(s_activeMap->getSRS(), world);

            if (!_layer)
            {
                _layer = new AnnotationLayer();
                _layer->setName("User-created Labels");
                _mapNode->getMap()->addLayer(_layer);
            }

            LabelNode* label = new LabelNode();
            label->setText("Label");
            label->setPosition(coords);

            Style style;
            TextSymbol* symbol = style.getOrCreate<TextSymbol>();
            symbol->alignment() = symbol->ALIGN_CENTER_CENTER;
            label->setStyle(style);

            _layer->addChild(label);

            osg::ref_ptr<XmlDocument> xml = new XmlDocument(label->getConfig());
            xml->store(std::cout);
        }
        return false;
    }
    char _c;
    MapNode* _mapNode;
    AnnotationLayer* _layer;
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
    Container* uiRoot = new VBox();
    uiRoot->setAbsorbEvents(false);
    createControlPanel(uiRoot);

    osg::Node* loaded = osgEarth::Util::MapNodeHelper().load(arguments, &viewer, uiRoot);
    osgEarth::MapNode* mapNode = osgEarth::MapNode::get(loaded);
    if ( !mapNode ) {
        OE_WARN << "No osgEarth MapNode found in the loaded file(s)." << std::endl;
        return -1;
    }

    // the displayed Map:
    s_activeMap = mapNode->getMap();
    s_activeMap->addMapCallback( new MyMapListener() );

    //osg::Group* root = new osg::Group();
    //root->addChild( loaded );

    // update the control panel with the two Maps:
    updateControlPanel();

    viewer.setSceneData( loaded );

    // install our control panel updater
    viewer.addUpdateOperation( new UpdateOperation() );

    viewer.addEventHandler(new DumpElevation(mapNode, 'E'));

    viewer.addEventHandler(new DumpLabel(mapNode, 'L'));

    viewer.run();
}

//------------------------------------------------------------------------

struct EnableDisableHandler : public ControlEventHandler
{
    EnableDisableHandler( Layer* layer ) : _layer(layer) { }
    void onClick( Control* control )
    {
        _layer->setEnabled( !_layer->getEnabled() );
        updateControlPanel();
    }
    Layer* _layer;
};

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
            std::vector<GeoPoint> points;
            points.push_back(GeoPoint(extent.getSRS(), extent.west(), extent.south()));
            points.push_back(GeoPoint(extent.getSRS(), extent.east(), extent.north()));
            
            ViewFitter fitter(s_activeMap->getSRS(), s_view->getCamera());
            Viewpoint vp;
            if (fitter.createViewpoint(points, vp))
            {
                s_manip->setViewpoint(vp, 2.0);
            }
        }
        else if (_layer->getNode())
        {
            const osg::BoundingSphere& bs = _layer->getNode()->getBound();
            if (bs.valid())
            {
                osg::Vec3d c = bs.center();
                double r = bs.radius();
                const SpatialReference* mapSRS = s_activeMap->getSRS();

                std::vector<GeoPoint> points;
                GeoPoint p;
                p.fromWorld(mapSRS, osg::Vec3d(c.x()+r, c.y(), c.z())); points.push_back(p);
                p.fromWorld(mapSRS, osg::Vec3d(c.x()-r, c.y(), c.z())); points.push_back(p);
                p.fromWorld(mapSRS, osg::Vec3d(c.x(), c.y()+r, c.z())); points.push_back(p);
                p.fromWorld(mapSRS, osg::Vec3d(c.x(), c.y()-r, c.z())); points.push_back(p);
                p.fromWorld(mapSRS, osg::Vec3d(c.x(), c.y(), c.z()+r)); points.push_back(p);
                p.fromWorld(mapSRS, osg::Vec3d(c.x(), c.y(), c.z()-r)); points.push_back(p);

                ViewFitter fitter(s_activeMap->getSRS(), s_view->getCamera());
                Viewpoint vp;
                if (fitter.createViewpoint(points, vp))
                {
                    s_manip->setViewpoint(vp, 2.0);
                }
            }
        }
    }
    Layer* _layer;
};

//------------------------------------------------------------------------


void
createControlPanel(Container* container)
{
    //The Map layers
    s_activeBox = new Grid();
    s_activeBox->setBackColor(0,0,0,0.1);
    s_activeBox->setPadding( 10 );
    s_activeBox->setChildSpacing( 10 );
    s_activeBox->setChildVertAlign( Control::ALIGN_CENTER );
    s_activeBox->setAbsorbEvents( true );
    container->addControl(s_activeBox);

    //the removed layers
    s_inactiveBox = new Grid();
    s_inactiveBox->setBackColor(0,0,0,0.1);
    s_inactiveBox->setPadding( 10 );
    s_inactiveBox->setChildSpacing( 10 );
    s_inactiveBox->setChildVertAlign( Control::ALIGN_CENTER );
    s_inactiveBox->setAbsorbEvents( true );
    container->addControl(s_inactiveBox);
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

    // a checkbox to toggle the layer's visibility:
    if (visibleLayer && layer->getEnabled() && !(imageLayer && imageLayer->isCoverage()))
    {
        CheckBoxControl* visibility = new CheckBoxControl( visibleLayer->getVisible() );
        visibility->addEventHandler( new ToggleLayerVisibility(visibleLayer) );
        grid->setControl( gridCol, gridRow, visibility );
    }
    gridCol++;

    // the layer name
    if (layer->getEnabled() && (layer->getExtent().isValid() || layer->getNode()))
    {
        ButtonControl* name = new ButtonControl(layer->getName());
        name->clearBackColor();
        name->setPadding(4);
        name->addEventHandler( new ZoomLayerHandler(layer) );
        grid->setControl( gridCol, gridRow, name );
    }
    else
    {
        LabelControl* name = new LabelControl( layer->getName() );
        name->setPadding(4);
        if (!layer->getEnabled())
            name->setForeColor(osg::Vec4f(1,1,1,0.35));
        grid->setControl( gridCol, gridRow, name );
    }
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

    // enable/disable button
    LabelControl* enableDisable = new LabelControl(layer->getEnabled() ? "DISABLE" : "ENABLE", 14);
    enableDisable->setHorizAlign( Control::ALIGN_CENTER );
    enableDisable->setBackColor( .4,.4,.4,1 );
    enableDisable->setActiveColor( .8,0,0,1 );
    enableDisable->addEventHandler( new EnableDisableHandler(layer) );
    grid->setControl( gridCol, gridRow, enableDisable );
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

    LabelControl* activeLabel = new LabelControl( "Map Layers" );
    activeLabel->setForeColor(osg::Vec4f(1,1,0,1));
    s_activeBox->setControl( 1, row++, activeLabel );

    // the active map layers:
    LayerVector layers;
    s_activeMap->getLayers(layers);

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
