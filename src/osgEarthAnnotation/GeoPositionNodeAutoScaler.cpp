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

#include <osgEarthAnnotation/GeoPositionNodeAutoScaler>
#include <osgEarthAnnotation/GeoPositionNode>
#include <osgUtil/CullVisitor>

#define LC "[GeoPositionNodeAutoScaler] "

using namespace osgEarth;
using namespace osgEarth::Annotation;


GeoPositionNodeAutoScaler::GeoPositionNodeAutoScaler(const osg::Vec3d& baseScale, double minScale, double maxScale) :
_baseScale( baseScale ),
_minScale( minScale ),
_maxScale( maxScale )
{
    //nop
}

void
GeoPositionNodeAutoScaler::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
    GeoPositionNode* geo = static_cast<GeoPositionNode*>(node);
    osgUtil::CullVisitor* cs = static_cast<osgUtil::CullVisitor*>(nv);
    double size = 1.0/cs->pixelSize( node->getBound().center(), 0.5f );
	if (size < _minScale)
		size = _minScale;
	else if (size>_maxScale)
		size = _maxScale;
    geo->getPositionAttitudeTransform()->setScale( osg::componentMultiply(_baseScale, osg::Vec3d(size,size,size)) );
    traverse(node, nv);
}
