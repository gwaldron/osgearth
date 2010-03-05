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
#include <osgGA/TrackballManipulator>
#include <osgDB/FileNameUtils>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgEarthSymbology/GeometrySymbolizer>
#include <osgEarthSymbology/FeatureDataSetAdapter>
#include <osgEarthSymbology/Style>
#include <osgEarthSymbology/SymbolicNode>
#include <osgEarthDrivers/feature_ogr/OGRFeatureOptions>

using namespace osgEarth::Symbology;

osg::Group* createSymbologyScene(const std::string url)
{
    osg::Group* grp = new osg::Group;

    osg::ref_ptr<osgEarth::Drivers::OGRFeatureOptions> featureOpt = new osgEarth::Drivers::OGRFeatureOptions();
    featureOpt->url() = url;
    osg::ref_ptr<osgEarth::Features::FeatureSource> features = FeatureSourceFactory::create( featureOpt );
    features->initialize("");

    osg::ref_ptr<FeatureDataSetAdapter> dataset = new FeatureDataSetAdapter(features.get());


    osg::ref_ptr<osgEarth::Symbology::Style> style = new osgEarth::Symbology::Style;

    osg::ref_ptr<PointSymbol> pointSymbol = new PointSymbol;
    pointSymbol->fill()->color() = osg::Vec4(1,0,0,1);
    style->setPoint(pointSymbol.get());

    osg::ref_ptr<PolygonSymbol> polySymbol = new PolygonSymbol;
    polySymbol->fill()->color() = osg::Vec4(0,1,1,1);
    style->setPolygon(polySymbol.get());

    osg::ref_ptr<GeometrySymbolizer> symbolizer = new GeometrySymbolizer;
    osg::ref_ptr<SymbolicNode> node = new SymbolicNode;
    node->setSymbolizer(symbolizer.get());
    node->setStyle(style.get());
    node->setDataSet(dataset.get());

    grp->addChild(node.get());

    return grp;
}


int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    osgViewer::Viewer viewer(arguments);

    // add some stock OSG handlers:
    viewer.setCameraManipulator(new osgGA::TrackballManipulator());
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    const std::string url = "../data/world.shp";
    std::string real = osgDB::getRealPath(url);
    osg::Node* node = createSymbologyScene(real);
    viewer.setSceneData(node);

    return viewer.run();
}
