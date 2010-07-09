/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2009 Pelican Ventures, Inc.
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
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgEarth/Map>
#include <osgEarth/MapNode>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/Viewpoint>
#include <osgEarthUtil/AutoClipPlaneHandler>
#include <osgEarthUtil/Graticule>
#include <osgEarth/EarthFile>

#include <osgEarthSymbology/Style>
#include <osgEarthSymbology/GeometrySymbol>
#include <osgEarthDrivers/tms/TMSOptions>
#include <osgEarthDrivers/model_feature_geom/FeatureGeomModelOptions>
#include <osgEarthDrivers/model_feature_stencil/FeatureStencilModelOptions>

#include <osgEarthDrivers/feature_ogr/OGRFeatureOptions>
#include <osgEarthDrivers/agglite/AGGLiteOptions>

using namespace osgEarth::Drivers;
using namespace osgEarth::Symbology;
using namespace osgEarth::Features;

// some preset viewpoints.
static osgEarthUtil::Viewpoint VPs[] = {
    osgEarthUtil::Viewpoint( "Africa",        osg::Vec3d(    0.0,   0.0, 0.0 ), 0.0, -90.0, 10e6 ),
    osgEarthUtil::Viewpoint( "California",    osg::Vec3d( -121.0,  34.0, 0.0 ), 0.0, -90.0, 6e6 ),
    osgEarthUtil::Viewpoint( "Europe",        osg::Vec3d(    0.0,  45.0, 0.0 ), 0.0, -90.0, 4e6 ),
    osgEarthUtil::Viewpoint( "Washington DC", osg::Vec3d(  -77.0,  38.0, 0.0 ), 0.0, -90.0, 1e6 ),
    osgEarthUtil::Viewpoint( "Australia",     osg::Vec3d(  135.0, -20.0, 0.0 ), 0.0, -90.0, 2e6 )
};

// a simple handler that demonstrates the "viewpoint" functionality in 
// osgEarthUtil::EarthManipulator. Press a number key to fly to a viewpoint.
struct FlyToViewpointHandler : public osgGA::GUIEventHandler 
{
    FlyToViewpointHandler( osgEarthUtil::EarthManipulator* manip ) : _manip(manip) { }

    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
    {
        if ( ea.getEventType() == ea.KEYDOWN && ea.getKey() >= '1' && ea.getKey() <= '5' )
        {
            _manip->setViewpoint( VPs[ea.getKey()-'1'], 4.0 );
        }
        return false;
    }

    osg::observer_ptr<osgEarthUtil::EarthManipulator> _manip;
};

// a simple handler that toggles a node mask on/off
struct NodeToggleHandler : public osgGA::GUIEventHandler 
{
    NodeToggleHandler( osg::Node* node, char key ) : _node(node), _key(key) { }

    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
    {
        if ( ea.getEventType() == ea.KEYDOWN && ea.getKey() == _key )
        {
            _node->setNodeMask( _node->getNodeMask() == 0 ? ~0 : 0 );
        }
        return false;
    }

    osg::observer_ptr<osg::Node> _node;
    char _key;

};

// a simple handler that toggles a node mask on/off
struct ToggleModelLayerHandler : public osgGA::GUIEventHandler 
{
    ToggleModelLayerHandler( ModelLayer* modelLayer) : _modelLayer(modelLayer)
    {
    }

    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
    {
        if ( ea.getEventType() == ea.KEYDOWN && ea.getKey() == 'q' )
        {
            _modelLayer->setEnabled( !_modelLayer->getEnabled() );
        }
        return false;
    }

    osg::observer_ptr< ModelLayer > _modelLayer;
};


int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
       
    osg::DisplaySettings::instance()->setMinimumNumStencilBits( 8 );
    osgViewer::Viewer viewer(arguments);

    // install the programmable manipulator.
    osgEarthUtil::EarthManipulator* manip = new osgEarthUtil::EarthManipulator();

    osg::Group* root = new osg::Group();
    osg::Node* earthNode = osgDB::readNodeFiles( arguments );

    if (!earthNode)
    {
        // Create a "Map" dynamically if no nodes were loaded
        // The "Map" is the data model object that we will be visualizing. It will be
        // geocentric by default, but you can specify a projected map in the constructor.
        osgEarth::Map* map = new osgEarth::Map();

        // Add an image layer to the map.
        {
            osg::ref_ptr<TMSOptions> tms = new TMSOptions();
            tms->url() = "http://demo.pelicanmapping.com/rmweb/data/bluemarble-tms/tms.xml";
            map->addMapLayer( new ImageMapLayer( "NASA", tms.get() ) );
        }

        // Add a heightfield layer to the map. You can add any number of heightfields and
        // osgEarth will composite them automatically.
        {
            osg::ref_ptr<TMSOptions> tms = new TMSOptions();
            tms->url() = "http://demo.pelicanmapping.com/rmweb/data/srtm30_plus_tms/tms.xml";
            map->addMapLayer( new HeightFieldMapLayer( "SRTM", tms.get() ) );
        }

        //Add a shapefile to the map
        {
            //Configure the feature options
            OGRFeatureOptions* featureOpt = new OGRFeatureOptions();
            featureOpt->url() = "../data/world.shp";

            //FeatureGeomModelOptions* opt = new FeatureGeomModelOptions();
            //AGGLiteOptions* opt = new AGGLiteOptions();
            FeatureStencilModelOptions* opt = new FeatureStencilModelOptions();
            opt->featureOptions() = featureOpt;

            osgEarth::Symbology::Style* style = new osgEarth::Symbology::Style; 
            osgEarth::Symbology::LineSymbol* ls = new osgEarth::Symbology::LineSymbol;
            ls->stroke()->color() = osg::Vec4f( 1,1,0,1 );
            ls->stroke()->width() = 1;
            style->addSymbol(ls); 
            opt->styles()->addStyle(style);
            opt->geometryTypeOverride() = Geometry::TYPE_LINESTRING;

            ModelLayer* modelLayer = new osgEarth::ModelLayer("shapefile", opt);
            map->addModelLayer( modelLayer );               
            //map->addMapLayer( new ImageMapLayer("world", opt) );
            viewer.addEventHandler( new ToggleModelLayerHandler( modelLayer ) );

        }

        // The MapNode will render the Map object in the scene graph.
        osgEarth::MapNode* mapNode = new osgEarth::MapNode( map );     
        earthNode = mapNode;
    }    

    root->addChild( earthNode );
    
    osgEarthUtil::Graticule* graticule = 0L;

    osgEarth::MapNode* mapNode = osgEarth::MapNode::findMapNode( earthNode );
    if ( mapNode )
    {
        manip->setNode( mapNode );
        if ( mapNode->getMap()->isGeocentric() )
        {
            manip->setHomeViewpoint( 
                osgEarthUtil::Viewpoint( osg::Vec3d( -90, 0, 0 ), 0.0, -90.0, 5e7 ) );

            // add a handler that will automatically calculate good clipping planes
            // for a geocentric map:
            //viewer.addEventHandler( new osgEarthUtil::AutoClipPlaneHandler( mapNode ) );
        }

        // create a graticle, and start it in the OFF position
        graticule = new osgEarthUtil::Graticule( mapNode->getMap() );
        graticule->setNodeMask(0);
        root->addChild( graticule );
    }

    viewer.setSceneData( root );
    viewer.setCameraManipulator( manip );

    manip->getSettings()->bindMouseDoubleClick(
        osgEarthUtil::EarthManipulator::ACTION_GOTO,
        osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON );

    // add our fly-to handler
    viewer.addEventHandler(new FlyToViewpointHandler( manip ));

    // add a handler to toggle the graticle
    if ( graticule )
        viewer.addEventHandler(new NodeToggleHandler( graticule, 'g' ));

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgViewer::ThreadingHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    return viewer.run();
}
