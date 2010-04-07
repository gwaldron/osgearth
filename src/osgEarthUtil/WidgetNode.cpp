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

#include <osgEarthUtil/WidgetNode>

using namespace osgEarthUtil;

WidgetNode::WidgetNode()
{
    setDataVariance(osg::Object::DYNAMIC);  
    setCullingActive(false); 
    _hide = true;
}

void WidgetNode::setWidget(Widget* widget) { _widget = widget; }

const Widget* WidgetNode::getWidget() const { return _widget; }
Widget* WidgetNode::getWidget() { return _widget; }

void WidgetNode::accept(osg::NodeVisitor& nv)
{
    setNumChildrenRequiringUpdateTraversal(1);
    if (nv.validNodeMask(*this)) 
    {
        if (nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR) {
            osg::Vec2 size = osg::Vec2(_widget->getWindow()->getWidth(), _widget->getWindow()->getHeight());
            osg::Vec2 positionCenteredAnchor = _positionOnScreen - size/2;
            _widget->getWindow()->setX(osg::round(positionCenteredAnchor[0]));
            _widget->getWindow()->setY(osg::round(positionCenteredAnchor[1]));
            if (_hide)
                _widget->getWindowManager()->removeChild(_widget->getWindow());
            else {
                if (!_widget->getWindowManager()->containsNode(_widget->getWindow()))
                    _widget->getWindowManager()->addChild(_widget->getWindow());
            }

        } else if (nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR) {
            osg::CullStack* cull = dynamic_cast<osg::CullStack*>(&nv);
            if (cull) {
                    
                std::vector<osg::Vec3> vertices;
                vertices.push_back(osg::Vec3(0,0,0));
                if (cull->isCulled(vertices)) {
                    _hide = true;
                    return;
                } else {
                    _hide = false;
                }

                // compute 2d on screen
                osg::Matrix matrix = *(cull->getModelViewMatrix());
                matrix *= *(cull->getProjectionMatrix());
                osg::Vec3 point = matrix.getTrans();

                        

                point *= 1.0/matrix(3,3); // w
                float x = cull->getViewport()->width()/2 * (1.0 + point[0]);
                float y = cull->getViewport()->height()/2 * (1.0 + point[1]);

                _positionOnScreen = osg::Vec2(x, y);
            }
        }
    }
}
