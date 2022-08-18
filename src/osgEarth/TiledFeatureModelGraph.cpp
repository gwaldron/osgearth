/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2020 Pelican Mapping
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
#include <osgEarth/TiledFeatureModelGraph>
#include <osgEarth/GeometryCompiler>
#include <osgEarth/FeatureModelSource>
#include <osgEarth/NetworkMonitor>
#include <osgEarth/Registry>
#include <osg/MatrixTransform>

using namespace osgEarth;

TiledFeatureModelGraph::TiledFeatureModelGraph(const osgEarth::Map* map,
                                               FeatureSource* features,
                                               StyleSheet* styleSheet,
                                               Session* session) :
    SimplePager(map, Registry::instance()->getGlobalGeodeticProfile()),
    _features(features),
    _styleSheet(styleSheet),
    _session(session)
{    
    _nodeGraph = std::make_shared<NodeGraph>();
    setMinLevel(0);
    setMaxLevel(14);

#if 0
    _nodeGraph->operations.push_back(std::make_shared< SphereOperation>());
    
    //auto loadNode = std::make_shared < LoadNodeOperation >();
    //loadNode->setFilename("cow.osg");
    //_nodeGraph->operations.push_back(loadNode);

    auto transform = std::make_shared< TransformOperation>();
    transform->setTransform(osg::Matrixd::scale(1.0f, 1.0f, 1.0f));
    _nodeGraph->operations.push_back(transform);
#else

    _nodeGraph->operations.push_back(std::make_shared < LoadNodeOperation >("D:/dev/private-data/splat/assets/trees/BlueSpruce/blue_spruce.osgb"));
    _nodeGraph->operations.push_back(std::make_shared < LoadNodeOperation >("D:/dev/private-data/splat/assets/trees/SugarMaple/sugar_maple.osgb"));
    _nodeGraph->operations.push_back(std::make_shared < LoadNodeOperation >("c:/Users/Jason/Downloads/big-boulder/source/boulder.obj"));

    auto getFeatures = std::make_shared < GetFeaturesOperation >();
    getFeatures->setLayerName("data:osm");
    _nodeGraph->operations.push_back(getFeatures);

    auto cube = std::make_shared < LoadNodeOperation >("cube.obj");
    _nodeGraph->operations.push_back(cube);

    _nodeGraph->operations.push_back(std::make_shared < LoadNodeOperation >("cessna.osg"));

    auto loadNode = std::make_shared < LoadNodeOperation >("cow.osg");
    loadNode->setFilename("cow.osg");
    _nodeGraph->operations.push_back(loadNode);

    auto floatValue = std::make_shared<FloatValue>();
    floatValue->setValue(123.0f);
    _nodeGraph->operations.push_back(floatValue);

    // Make a sphere operation
    auto sphere = std::make_shared< SphereOperation>();
    _nodeGraph->operations.push_back(sphere);
    // connect the float value to the sphere radius
    floatValue->connect("Value", sphere.get(), "Radius");

    auto transform = std::make_shared< TransformOperation>();
    transform->setTransform(osg::Matrixd::scale(1000.0f, 1000.0f, 1000.0f));    
    _nodeGraph->operations.push_back(transform);
    // connect the sphere as the output
    //sphere->connect("Node", transform.get(), "Node");
    // connect the cow as the output to the transform
    loadNode->connect("Node", transform.get(), "Node");

    // Make a final node output
    auto output = std::make_shared< NodeOutputOperation >();
    // Connect the sphere to the output    
    transform->connect("Node", output.get(), "Node");
    _nodeGraph->operations.push_back(output);

    //auto loadNode = std::make_shared < LoadNodeOperation >();
    //loadNode->setFilename("cow.osg");
    //_nodeGraph->operations.push_back(loadNode);

    //auto transform = std::make_shared< TransformOperation>();
    //transform->setTransform(osg::Matrixd::scale(1.0f, 1.0f, 1.0f));
    //_nodeGraph->operations.push_back(transform);

#endif
}

void
TiledFeatureModelGraph::setFilterChain(FeatureFilterChain* chain)
{
    _filterChain = chain;
}

void
TiledFeatureModelGraph::setOwnerName(const std::string& value)
{
    _ownerName = value;
}


FeatureCursor*
TiledFeatureModelGraph::createCursor(FeatureSource* fs, FilterContext& cx, const Query& query, ProgressCallback* progress) const
{
    NetworkMonitor::ScopedRequestLayer layerRequest(_ownerName);
    FeatureCursor* cursor = fs->createFeatureCursor(query, progress);
    if (cursor && _filterChain.valid())
    {
        cursor = new FilteredFeatureCursor(cursor, _filterChain.get(), &cx);
    }
    return cursor;
}

osg::ref_ptr<osg::Node>
TiledFeatureModelGraph::createNode(const TileKey& key, ProgressCallback* progress)
{
    osg::BoundingSphered bounds = getBounds(key);
#if 0
    osg::MatrixTransform* mt = new osg::MatrixTransform;
    osg::Matrixd l2w;
    key.getExtent().getCentroid().createLocalToWorld(l2w);
    mt->setMatrix(l2w);

    // Execute the graph and add it to the transform
    _nodeGraph->execute();
    mt->addChild(_nodeGraph->getOutput());
    return mt;
#else

    NodeGraphNode* mt = new NodeGraphNode;
    mt->_tileKey = key;
    mt->_map = _map.get();
    osg::Matrixd l2w;
    key.getExtent().getCentroid().createLocalToWorld(l2w);
    mt->setMatrix(l2w);
    mt->_nodeGraph = _nodeGraph;
    mt->build();
    registerNode(mt);
    return mt;
#endif    

    //return SimplePager::createNode(key, progress);
}