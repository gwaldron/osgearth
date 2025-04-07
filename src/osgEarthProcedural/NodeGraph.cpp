/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/
#include "NodeGraph"

using namespace osgEarth;
using namespace osgEarth::Procedural;

// Register all node graph operations
REGISTER_NODEGRAPH_OPERATION("Float", FloatValue);
REGISTER_NODEGRAPH_OPERATION("Color", ColorValue);
REGISTER_NODEGRAPH_OPERATION("RandomValuePerFeature", RandomValuePerFeature);
REGISTER_NODEGRAPH_OPERATION("RandomColorPerFeature", RandomColorPerFeature);
REGISTER_NODEGRAPH_OPERATION("OSMHighwaysColorOperation", OSMHighwaysColorOperation);
REGISTER_NODEGRAPH_OPERATION("Sphere", SphereOperation);
REGISTER_NODEGRAPH_OPERATION("LoadNode", LoadNodeOperation);
REGISTER_NODEGRAPH_OPERATION("LoadFeatures", LoadFeaturesOperation);
REGISTER_NODEGRAPH_OPERATION("Transform", TransformOperation);
REGISTER_NODEGRAPH_OPERATION("Simplify", SimplifyOperation);
REGISTER_NODEGRAPH_OPERATION("JoinNodes", JoinNodesOperation);
REGISTER_NODEGRAPH_OPERATION("RandomPoints", RandomPointsOperation);
REGISTER_NODEGRAPH_OPERATION("PointsOnEdge", PointsOnEdgeOperation);
REGISTER_NODEGRAPH_OPERATION("GriddedPoints", GriddedPointsOperation);
REGISTER_NODEGRAPH_OPERATION("PointsAlongGeometry", PointsAlongGeometryOperation);
REGISTER_NODEGRAPH_OPERATION("Extrude", ExtrudeOperation);
REGISTER_NODEGRAPH_OPERATION("GetFeatures", GetFeaturesOperation);
REGISTER_NODEGRAPH_OPERATION("FilterFeatures", FilterFeaturesOperation);
REGISTER_NODEGRAPH_OPERATION("SelectFeatures", SelectFeaturesOperation);
REGISTER_NODEGRAPH_OPERATION("PolygonToPoints", PolygonToPointsOperation);
REGISTER_NODEGRAPH_OPERATION("JoinFeatures", JoinFeaturesOperation);
REGISTER_NODEGRAPH_OPERATION("ImageFromLayer", ImageFromLayerOperation);
REGISTER_NODEGRAPH_OPERATION("LoadImage", LoadImageOperation);
REGISTER_NODEGRAPH_OPERATION("ImageMask", ImageMaskOperation);
REGISTER_NODEGRAPH_OPERATION("ClampFeatures", ClampFeaturesOperation);
REGISTER_NODEGRAPH_OPERATION("OffsetFeatures", OffsetFeaturesOperation);
REGISTER_NODEGRAPH_OPERATION("Buffer", BufferOperation);
REGISTER_NODEGRAPH_OPERATION("OffsetCurve", OffsetCurveOperation);
REGISTER_NODEGRAPH_OPERATION("CurrentLOD", CurrentLODOperation);
REGISTER_NODEGRAPH_OPERATION("Comparison", ComparisonOperator);
REGISTER_NODEGRAPH_OPERATION("PlaceNodes", PlaceNodesOperation);
REGISTER_NODEGRAPH_OPERATION("FeaturesToLines", FeaturesToLinesOperation);
REGISTER_NODEGRAPH_OPERATION("FeaturesToPolygons", FeaturesToPolygonsOperation);
REGISTER_NODEGRAPH_OPERATION("NodeOutput", NodeOutputOperation);
REGISTER_NODEGRAPH_OPERATION("FeatureOutput", FeatureOutputOperation);



Config
NodeGraph::getConfig() const
{
    Config conf("nodegraph");
    Config operationsConf;
    for (auto& op : operations)
    {
        operationsConf.add(op->getConfig());
    }
    conf.set("nodes", operationsConf);

    // Serialize the links
    Config linksConf;
    for (auto& op : operations)
    {
        for (auto& link : op->getLinks())
        {
            Config linkConf("link");
            linkConf.set("source", link._source->getId());
            linkConf.set("source_pin", link._sourcePin->getName());
            linkConf.set("destination", link._destination->getId());
            linkConf.set("destination_pin", link._destinationPin->getName());
            linksConf.add(linkConf);
        }
    }
    conf.set("links", linksConf);

    conf.set("user_config", userConfig);

    return conf;
}

std::shared_ptr<NodeGraph>
NodeGraph::fromConfig(const Config& conf)
{
    std::shared_ptr<NodeGraph> graph = std::make_shared<NodeGraph>();
    Config nodeGraphConf = conf.child("nodegraph");

    Config operationsConf = nodeGraphConf.child("nodes");

    for (auto& opConf : operationsConf.children())
    {
        std::string type = opConf.value("type");
        std::shared_ptr<NodeGraphOperation> op = NodeGraphOperationFactory::instance()->create(type);
        if (op)
        {
            op->fromConfig(opConf);
            graph->operations.push_back(std::shared_ptr<NodeGraphOperation>(op));
        }
        else
        {
            OE_WARN << "Failed to deserialize NodeGraph operation of type " << type << std::endl;
        }
    }

    for (auto& linkConf : nodeGraphConf.child("links").children("link"))
    {
        int sourceId = linkConf.value<int>("source", -1);
        std::string sourcePin = linkConf.value("source_pin");
        int destinationId = linkConf.value<int>("destination", -1);
        std::string destinationPin = linkConf.value("destination_pin");

        NodeGraphOperation* source = nullptr;
        NodeGraphOperation* destination = nullptr;
        for (auto& op : graph->operations)
        {
            if (op->getId() == sourceId)
            {
                source = op.get();
            }
            if (op->getId() == destinationId)
            {
                destination = op.get();
            }
        }

        if (source && destination)
        {
            source->connect(sourcePin, destination, destinationPin);
        }
    }

    graph->userConfig = nodeGraphConf.child("user_config");

    return graph;
}

//....................................................................

void
NodeGraphNode::build()
{
    removeChildren(0, getNumChildren());

    osg::ref_ptr< osg::Node > node;

    if (!node.valid())
    {
        NodeGraphContext context;
        context.tileKey = _tileKey;
        context.map = _map;
        NodeGraphResult* result = _nodeGraph->execute<NodeOutputOperation>(context);

        if (result && result->nodeValue)
        {
            node = result->nodeValue;
        }
    }

    if (node.valid())
    {
        addChild(node.get());
    }
}
