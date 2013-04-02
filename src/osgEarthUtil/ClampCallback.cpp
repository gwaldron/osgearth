/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2013 Pelican Mapping
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

#include <osgEarthUtil/ClampCallback>
#include <osgEarth/Notify>
#include <osgUtil/IntersectionVisitor>
#include <osgUtil/LineSegmentIntersector>
#include <osgEarth/NodeUtils>
#include <OpenThreads/ScopedLock>
#include <osg/MatrixTransform>

#include <osg/io_utils>

using namespace osg;
using namespace osgEarth;
using namespace osgEarth::Util;
using namespace OpenThreads;

ClampCallback::ClampCallback(osg::Node* terrainNode)
{
    _offset = 0;
    _lastCulledFrame = 0;
    setTerrainNode( terrainNode );
    _intersectionMask = 0xffffffff;
}

bool ClampCallback::clamp(const osg::Vec3d& pos, osg::Vec3d& out) const
{
    //Return if there is no terrain node assigned
    if (!_terrainNode.valid()) return false;    

    //Compute the up vector
    osg::Vec3d up = _csn.valid() ? _csn->computeLocalUpVector( pos ) : osg::Vec3d(0,0,1);
    up.normalize();

    double segOffset = 50000;

    osg::Vec3d start = pos + (up * segOffset);
    osg::Vec3d end = pos - (up * segOffset);
    
    osgUtil::LineSegmentIntersector* i = new osgUtil::LineSegmentIntersector( start, end );
    
    osgUtil::IntersectionVisitor iv;
    iv.setTraversalMask(_intersectionMask);
    iv.setIntersector( i );
    _terrainNode->accept( iv );

    osgUtil::LineSegmentIntersector::Intersections& results = i->getIntersections();
    if ( !results.empty() )
    {
        const osgUtil::LineSegmentIntersector::Intersection& result = *results.begin();
        out = result.getWorldIntersectPoint();
        out += (up * _offset);
        return true;
    }
    return false;    
}

bool ClampCallback::clampGeometry(osg::Geometry* geom, const osg::Matrixd& localToWorld, const osg::Matrixd& worldToLocal) const
{
    osg::Vec3Array* verts = static_cast<osg::Vec3Array*>(geom->getVertexArray());
    for (unsigned int i = 0; i < verts->size(); ++i)
    {
        osg::Vec3d pos = (*verts)[i];
        pos = pos * localToWorld;
        
        osg::Vec3d out;
        if (clamp(pos, out))
        {
            out = out * worldToLocal;
            (*verts)[i].set(out.x(), out.y(), out.z() );
        }
    }
    geom->dirtyBound();
    geom->dirtyDisplayList();

    return true;
}

void ClampCallback::setTerrainNode(osg::Node* terrainNode)
{
    _terrainNode = terrainNode;
    if (_terrainNode.valid())
    {
        _csn = findTopMostNodeOfType<osg::CoordinateSystemNode>(_terrainNode.get());
    }
}

void ClampCallback::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
    if (nv->getVisitorType() == NodeVisitor::UPDATE_VISITOR)
    {
        //Only clamp if this node has been culled in the last few frames
        if (nv->getFrameStamp()->getFrameNumber() - _lastCulledFrame < 2)
        {
            osg::MatrixTransform* mt = dynamic_cast<osg::MatrixTransform*>(node);
            if (mt)
            {
                osg::Matrixd matrix = mt->getMatrix();
                osg::Vec3d pos = matrix.getTrans();
                osg::Vec3d clamped;
                if (clamp(pos, clamped))
                {
                    //We need to translate the current matrix by an amount that would move the current position to the clamped position
                    osg::Vec3d trans = (clamped - pos );
                    matrix *= osg::Matrixd::translate( trans );
                    mt->setMatrix( matrix );
                }
            }
            else
            {
                osg::Geode* geode = dynamic_cast<osg::Geode*>(node);
                if (geode)
                {
                    osg::Matrixd localToWorld = osg::computeLocalToWorld( nv->getNodePath() );
                    osg::Matrixd worldToLocal = osg::computeWorldToLocal( nv->getNodePath() );

                    for (unsigned int i = 0; i < geode->getNumDrawables(); ++i)
                    {
                        osg::Geometry* geom = geode->getDrawable(i)->asGeometry();
                        if (geom)
                        {
                            clampGeometry( geom, localToWorld, worldToLocal);
                        }
                    }
                }
            }
        }
    }
    else if (nv->getVisitorType() == NodeVisitor::CULL_VISITOR)
    {			
        _lastCulledFrame = nv->getFrameStamp()->getFrameNumber();
    }
    //Continue traversal
    traverse(node, nv);
}

void
ClampCallback::setOffset(double offset)
{
    _offset = offset;
}
