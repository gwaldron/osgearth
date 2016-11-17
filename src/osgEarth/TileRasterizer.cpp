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
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */
#include <osgEarth/TileRasterizer>
#include <osgEarth/NodeUtils>
#include <osgDB/ReadFile>

#define LC "[TileRasterizer] "

using namespace osgEarth;

static osg::ref_ptr<osg::Node> cow;

TileRasterizer::TileRasterizer() :
osg::Camera()
{
    // active an update traversal.
    setNumChildrenRequiringUpdateTraversal(1);
    setCullingActive(false);

    // set up the RTT camera.
    setViewport(new osg::Viewport(0, 0, 256, 256));
    setRenderTargetImplementation(FRAME_BUFFER_OBJECT);
    setClearColor(osg::Vec4(1,0.5,0,0.5));
    setClearDepth(1.0);
    setClearMask( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    //setImplicitBufferAttachmentMask(0, 0);
    setRenderOrder(PRE_RENDER);
    setReferenceFrame(ABSOLUTE_RF);
    setComputeNearFarMode( osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR );
    setViewMatrix(osg::Matrix::identity());

    cow = osgDB::readNodeFile("H:/devel/osg/OpenSceneGraph-Data/cow.osg");
    if (!cow.valid())
        exit(-1);

    getOrCreateStateSet()->setMode(GL_BLEND, 1);
    getOrCreateStateSet()->setMode(GL_DEPTH_TEST, 0);
    getOrCreateStateSet()->setMode(GL_CULL_FACE, 0);

    addChild(cow.get());

    double r = getBound().radius();
    setProjectionMatrixAsOrtho(-r, r, -r, r, -1000, 1000);

    OE_INFO << LC << "Radius = " << r << std::endl;
}

TileRasterizer::~TileRasterizer()
{
    OE_DEBUG << LC << "~TileRasterizer\n";
}

void
TileRasterizer::push(osg::Node* node, osg::Texture* texture, const GeoExtent& extent)
{
    Threading::ScopedMutexLock lock(_mutex);

    _workQueue.push(Entry());
    Entry& entry = _workQueue.back();
    entry._frames = 0;
    entry._attached = false;
    entry._node = node;
    entry._texture = texture;
    entry._extent = extent;

    OE_INFO << LC << "Pushed texture " << entry._texture->getName() << std::endl;
}

void
TileRasterizer::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == nv.CULL_VISITOR)
    {
        //nop
    }

    else if (nv.getVisitorType() == nv.UPDATE_VISITOR)
    {
        Threading::ScopedMutexLock lock(_mutex);

        if (!_workQueue.empty())
        {
            //OE_INFO << LC << "Work queue size = " << _workQueue.size() << std::endl;

            if (_workQueue.front()._frames == 0)
            {
                Entry& next = _workQueue.front();
                OE_INFO << LC << "Attaching texture " << next._texture->getName() << std::endl;

                setViewport(0, 0, next._texture->getTextureWidth(), next._texture->getTextureHeight());
                setRenderingCache(NULL);
                detach(COLOR_BUFFER);
                attach(COLOR_BUFFER, next._texture.get(), 0u, 0u, false);
            }

            if (_workQueue.front()._frames == 1)
            {
                _workQueue.pop();
            }
            else
            {
                _workQueue.front()._frames++;
            }
        }

        //if (_workQueue.empty())
        //{
        //    setNumChildrenRequiringUpdateTraversal(0);
        //}
    }

    osg::Camera::traverse(nv);
}