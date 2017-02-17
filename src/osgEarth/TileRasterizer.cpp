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
#include <osgEarth/VirtualProgram>
#include <osgEarth/Registry>
#include <osg/MatrixTransform>
#include <osg/Texture2D>
#include <osgDB/ReadFile>

#define LC "[TileRasterizer] "

using namespace osgEarth;

TileRasterizer::TileRasterizer() :
osg::Camera()
{
    // active an update traversal.
    setNumChildrenRequiringUpdateTraversal(1);
    setCullingActive(false);

    // set up the RTT camera.
    setClearColor(osg::Vec4(0,0,0,0));
    setClearMask(GL_COLOR_BUFFER_BIT);
    setReferenceFrame(ABSOLUTE_RF);
    setComputeNearFarMode( osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR );
    setRenderOrder(PRE_RENDER);
    setRenderTargetImplementation(FRAME_BUFFER_OBJECT);
    setImplicitBufferAttachmentMask(0, 0);
    setSmallFeatureCullingPixelSize(0.0f);
    setViewMatrix(osg::Matrix::identity());

    osg::StateSet* ss = getOrCreateStateSet();
    //VirtualProgram::getOrCreate(ss)->setInheritShaders(false);
    ss->setAttribute(new osg::Program(), osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

    ss->setMode(GL_BLEND, 0);
    ss->setMode(GL_LIGHTING, 0);
    ss->setMode(GL_CULL_FACE, 0);
    
#if 0
    osg::Image* image = osgDB::readImageFile("H:/data/textures/road.jpg");
    osg::Texture2D* tex = new osg::Texture2D(image);
    tex->setWrap( osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE );
    tex->setWrap( osg::Texture::WRAP_T, osg::Texture::REPEAT );
    tex->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR );
    tex->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
    tex->setMaxAnisotropy( 4.0f );
    tex->setResizeNonPowerOfTwoHint( false );
    ss->setTextureAttribute(0, tex);
#endif
}

TileRasterizer::~TileRasterizer()
{
    OE_DEBUG << LC << "~TileRasterizer\n";
}

void
TileRasterizer::push(osg::Node* node, osg::Texture* texture, const GeoExtent& extent)
{
    Threading::ScopedMutexLock lock(_mutex);

    _jobs.push(Job());
    Job& job = _jobs.back();
    job._node = node;
    job._texture = texture;
    job._extent = extent;
}

void
TileRasterizer::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == nv.UPDATE_VISITOR)
    {
        Threading::ScopedMutexLock lock(_mutex);

        // Detach if we have no work and the buffer attachment isn't empty.
        if (!getBufferAttachmentMap().empty())
        {
            detach(osg::Camera::COLOR_BUFFER);
            dirtyAttachmentMap();
            removeChildren(0, 1);
        }

        if (!_jobs.empty())
        {
            Job& job = _jobs.front();

            // Get the next texture
            osg::Texture* texture = job._texture.get();

            // Setup the viewport and attach to the new texture
            setViewport(0, 0, job._texture->getTextureWidth(), job._texture->getTextureHeight());

            setProjectionMatrixAsOrtho(job._extent.xMin(), job._extent.xMax(), job._extent.yMin(), job._extent.yMax(), -100, 100);

            bool mipmap = false;
            //osg::Texture::FilterMode mode = job._texture->getFilter(osg::Texture::MIN_FILTER);
            //bool mipmap =
            //    (mode == osg::Texture::LINEAR_MIPMAP_LINEAR) || 
            //    (mode == osg::Texture::LINEAR_MIPMAP_NEAREST) ||
            //    (mode == osg::Texture::NEAREST_MIPMAP_LINEAR) ||
            //    (mode == osg::Texture::NEAREST_MIPMAP_NEAREST);

            attach(COLOR_BUFFER, job._texture.get(), 0u, 0u, mipmap);
            dirtyAttachmentMap();

            addChild(_jobs.front()._node.get());

            // Remove the texture from the queue.
            _jobs.pop();
        }
    }

    if (!getBufferAttachmentMap().empty())
    {
        osg::Camera::traverse(nv);
    }
}
