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
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>
#include <osgGA/StateSetManipulator>
#include <osgViewer/ViewerEventHandlers>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/Controls>
#include <osgEarthDrivers/feature_ogr/OGRFeatureOptions>
#include <osgEarthFeatures/FeatureSource>

#define LC "[osgearth_labels] "

using namespace osgEarth::Util::Controls;
using namespace osgEarth::Drivers;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

osg::Group* createLabels( Map* );

std::string g_featureFile, g_labelAttr, g_priorityAttr;
bool g_removeDupes = true;

/**
 * Demonstrates the dynamic labeling engine in osgEarthUtil::Controls.
 */

int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    if ( arguments.read( "--help" ) || argc < 2 )
    {
        OE_NOTICE << LC << std::endl << std::endl

            << "osgearth_labels <earthfile>" << std::endl
            << "    --features <filename>            : name of shapefile containing feature labels" << std::endl
            << "    --label-attr <attribute>         : attribute containing label text" << std::endl
            << "    --priority-attr <attribute>      : attribute containing priority value" << std::endl
            << "    --show-duplicates                : draws duplicate labels (usually won't)" << std::endl;
        
        return 0;
    }

    if ( !arguments.read( "--features", g_featureFile ) )
        g_featureFile = "../data/world.shp";
    
    if ( !arguments.read( "--label-attr", g_labelAttr ) )
        g_labelAttr = "cntry_name";

    if ( !arguments.read( "--priority-attr", g_priorityAttr ) )
        g_priorityAttr = "cntry_pop";

    if ( arguments.read( "--show-duplicates" ) )
        g_removeDupes = false;

    osgViewer::Viewer viewer(arguments);

    osg::Group* root = new osg::Group();
    osg::Node* node = osgDB::readNodeFiles( arguments );
    if ( node )
        root->addChild( node );

    MapNode* mapNode = MapNode::findMapNode(node);
    if ( mapNode )
    {
        viewer.setSceneData( root );
        viewer.setCameraManipulator( new osgEarth::Util::EarthManipulator );

        //root->addChild( new ControlCanvas( &viewer ) );

        // load up some labels.
        root->addChild( createLabels(mapNode->getMap()) );
    }

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgViewer::ThreadingHandler());
    viewer.addEventHandler(new osgViewer::LODScaleHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
    viewer.addEventHandler(new osgViewer::HelpHandler(arguments.getApplicationUsage()));

    return viewer.run();
}

osg::Group*
createLabels( Map* map )
{
    osg::ref_ptr<osg::Group> labels = new osg::Group();

    // first, open up the source shapefile
    OGRFeatureOptions fo;
    fo.url() = g_featureFile;

    osg::ref_ptr<FeatureSource> features = FeatureSourceFactory::create( fo );
    if ( !features.valid() )
    {
        OE_WARN << LC << "Unable to load features!" << std::endl;
        return 0L;
    }

    features->initialize( "" );
    const FeatureProfile* featureProfile = features->getFeatureProfile();
    if ( !featureProfile || !featureProfile->getSRS() )
    {
        OE_WARN << LC << "Feature data has no spatial reference!" << std::endl;
        return 0L;
    }

    osg::ref_ptr<FeatureCursor> cursor = features->createFeatureCursor();
    if ( !cursor.valid() )
    {
        OE_WARN << LC << "Failed to query the feature source!" << std::endl;
        return 0L;
    }

    //SceneControlBin* priorityBin = canvas->getSceneControls();

    unsigned count = 0;

    std::set<std::string> antiDupeSet;

    while( cursor->hasMore() )
    {
        Feature* feature = cursor->nextFeature();
        Geometry* geom = feature->getGeometry();

        if ( !geom )
            continue;

        // we will display the country name:
        std::string text = feature->getAttr( g_labelAttr );
        if ( text.empty() )
            continue;

        // and use the population to prioritize labels:
        std::string populationStr = feature->getAttr( g_priorityAttr );
        float population = osgEarth::as<float>( populationStr, 0.0f );

        // remove duplicate labels:
        if ( g_removeDupes )
        {
            if ( antiDupeSet.find(text) != antiDupeSet.end() )
                continue;
            antiDupeSet.insert(text);
        }

        // calculate the world location of the label:
        osg::Vec3d centerPoint = geom->getBounds().center();

        osg::Vec3d mapPoint;
        if ( !map->toMapPoint( centerPoint, featureProfile->getSRS(), mapPoint ) )
            continue;

        osg::Vec3d worldPoint;
        if ( !map->mapPointToGeocentricPoint( mapPoint, worldPoint ) )
            continue;

        // create the label and place it:
        osg::MatrixTransform* xform = new osg::MatrixTransform( osg::Matrix::translate(worldPoint) );
        xform->addChild( new ControlNode(new LabelControl(text)) );
        labels->addChild( xform );

        //LabelControl* label = new LabelControl( text );
        //osg::MatrixTransform* xform = priorityBin->addControl( label, population );
        //xform->setMatrix( osg::Matrix::translate(worldPoint) );

        ++count;

        //OE_NOTICE << LC << "Added: " << text << std::endl;
    }

    OE_NOTICE << LC << "Found " << count << " features. " << std::endl;

    return labels.release();
}

