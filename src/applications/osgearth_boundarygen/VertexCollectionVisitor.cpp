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

void VertexCollectionVisitor::apply(osg::Geode& geode)
{
  for(unsigned int i=0; i<geode.getNumDrawables(); ++i)
    applyDrawable(geode.getDrawable(i));
}

void VertexCollectionVisitor::applyDrawable(osg::Drawable* drawable)
{
  osg::Geometry* geometry = drawable->asGeometry();
  if (geometry)
  {
    osg::Vec3Array* verts = dynamic_cast<osg::Vec3Array*>(geometry->getVertexArray());
    if (verts)
    {
      if (_matrixStack.empty())
      {
        for (osg::Vec3Array::iterator it=verts->begin(); it != verts->end(); ++it)
          addVertex(*it);
      }
      else
      {
        osg::Matrix& matrix = _matrixStack.back();
        for (osg::Vec3Array::iterator it=verts->begin(); it != verts->end(); ++it)
          addVertex((*it) * matrix);
      }
    }
  }
}

void VertexCollectionVisitor::addVertex(osg::Vec3 vertex)
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
