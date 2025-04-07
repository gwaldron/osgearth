/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/

#include "VertexCollectionVisitor"
#include <osg/CoordinateSystemNode>
#include <osg/Drawable>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Transform>

VertexCollectionVisitor::VertexCollectionVisitor(bool geocentric, TraversalMode traversalMode):
  _geocentric(geocentric), osg::NodeVisitor(traversalMode)
{
  _vertices = new osg::Vec3dArray();
  _ellipsoidModel = new osg::EllipsoidModel();
}

void VertexCollectionVisitor::reset()
{
    _matrixStack.clear();
    _vertices->clear();
}

void VertexCollectionVisitor::apply(osg::Node& node)
{
    traverse(node);
}

void VertexCollectionVisitor::apply(osg::Transform& transform)
{
    osg::Matrix matrix;
    if (!_matrixStack.empty()) matrix = _matrixStack.back();

    transform.computeLocalToWorldMatrix(matrix,this);

    pushMatrix(matrix);

    traverse(transform);

    popMatrix();
}

void VertexCollectionVisitor::apply(osg::Drawable& drawable)
{
  osg::Geometry* geometry = drawable.asGeometry();
  if (geometry)
  {
    osg::Vec3Array* verts = dynamic_cast<osg::Vec3Array*>(geometry->getVertexArray());
    if (verts)
    {
      if (_matrixStack.empty())
      {
        for (osg::Vec3Array::iterator it=verts->begin(); it != verts->end(); ++it)
          addVertex(osg::Vec3d(*it));
      }
      else
      {
        osg::Matrix& matrix = _matrixStack.back();
        for (osg::Vec3Array::iterator it=verts->begin(); it != verts->end(); ++it)
          addVertex(osg::Vec3d(*it) * matrix);
      }
    }
  }
}

void VertexCollectionVisitor::addVertex(osg::Vec3d vertex)
{
  if (_geocentric)
  {
    double lat, lon, height;
    _ellipsoidModel->convertXYZToLatLongHeight(vertex.x(), vertex.y(), vertex.z(), lat, lon, height);
    _vertices->push_back(osg::Vec3d(osg::RadiansToDegrees(lon), osg::RadiansToDegrees(lat), height));
  }
  else
  {
    _vertices->push_back(vertex);
  }
}
