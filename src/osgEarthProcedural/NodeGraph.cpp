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

