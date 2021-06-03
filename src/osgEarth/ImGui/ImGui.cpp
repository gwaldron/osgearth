/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2018 Pelican Mapping
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
#include <osgEarth/ImGui/ImGui>

#include <osgGA/TrackballManipulator>

using namespace osgEarth::GUI;


struct FBOPreDraw : public osg::Camera::DrawCallback
{
    osg::ref_ptr<osg::Texture2D> _tex;
    mutable osg::ref_ptr<osg::FrameBufferObject> _fbo;

    FBOPreDraw(osg::Texture2D* tex) : _tex(tex){ }

    void operator()(osg::RenderInfo& ri) const override
    {
        if (_fbo.valid() == false)
        {
            _fbo = new osg::FrameBufferObject();

            _fbo->setAttachment(
                osg::Camera::COLOR_BUFFER,
                osg::FrameBufferAttachment(_tex.get()));

            _fbo->setAttachment(
                osg::Camera::DEPTH_BUFFER,
                osg::FrameBufferAttachment(
                    new osg::RenderBuffer(
                        _tex->getTextureWidth(),
                        _tex->getTextureHeight(),
                        GL_DEPTH_COMPONENT24)));
        }

        _fbo->apply(*ri.getState());
    }
};

struct FBOPostDraw : public osg::Camera::DrawCallback
{
    FBOPostDraw() {}

    void operator()(osg::RenderInfo& ri) const override
    {
        // deactivate the FBO before rendering the ImGui controls:
        osg::GLExtensions* ext = ri.getState()->get<osg::GLExtensions>();
        ext->glBindFramebuffer(GL_FRAMEBUFFER_EXT, 0);
    }
};



EmbeddedViewer::EmbeddedViewer(osg::Node* node, osg::GraphicsContext* mainContext)
{
    _graphicsWindowEmbedded = setUpViewerAsEmbeddedInWindow(0, 0, _width, _height);
    _graphicsWindowEmbedded->setState(mainContext->getState());

    setSceneData(node);

    MapNode* mapNode = MapNode::findMapNode(node);
    if (mapNode)
    {
        setCameraManipulator(new EarthManipulator);
    }
    else
    {
        setCameraManipulator(new osgGA::TrackballManipulator());
    }

    getEventQueue()->getCurrentEventState()->setMouseYOrientation(osgGA::GUIEventAdapter::Y_INCREASING_UPWARDS);

    auto& boundingSphere = node->getBound();
    osg::Vec3d center(boundingSphere.center());
    double radius = boundingSphere.radius();

    double dist = 3.5 * radius;

    double left, right, bottom, top, zNear, zFar;
    getCamera()->getProjectionMatrixAsFrustum(left, right, bottom, top, zNear, zFar);
    double vertical2 = fabs(right - left) / zNear / 2.;
    double horizontal2 = fabs(top - bottom) / zNear / 2.;
    double dim = horizontal2 < vertical2 ? horizontal2 : vertical2;
    double viewAngle = atan2(dim, 1.);
    dist = radius / sin(viewAngle);

    // target texture:
    _colorTexture = new osg::Texture2D();
    _colorTexture->setTextureSize(_width, _height);
    _colorTexture->setSourceFormat(GL_RGB);
    _colorTexture->setSourceType(GL_UNSIGNED_BYTE);
    _colorTexture->setInternalFormat(GL_RGB8);

    // install FBO wrapper for camera:
    getCamera()->setPreDrawCallback(new FBOPreDraw(_colorTexture.get()));
    getCamera()->setPostDrawCallback(new FBOPostDraw());

    getCamera()->setViewMatrix(osg::Matrixd::lookAt(center + osg::Vec3d(0.0, -dist, 0.0f),
        center,
        osg::Vec3d(0, 0, 1)));

    realize();
}

void EmbeddedViewer::setSize(unsigned int x, unsigned int y)
{
    getEventQueue()->setMouseInputRange(0, 0, x, y);
    if (_width != x || _height != y)
    {
        _width = x;
        _height = y;
        getEventQueue()->windowResize(0, 0, _width, _height);
        _graphicsWindowEmbedded->resized(0, 0, _width, _height);
        getCamera()->resize(_width, _height);

        _colorTexture->setTextureSize(_width, _height);
        _colorTexture->dirtyTextureObject();

    }
}

bool EmbeddedViewer::screenToLocal(float screen_x, float screen_y, float& local_x, float &local_y)
{
    local_x = screen_x - origin.x();
    local_y = screen_y - origin.y(); 
    return true;
}    


static osg::ref_ptr< EmbeddedViewer > focused_view;

void osgEarth::GUI::setFocusedView(EmbeddedViewer* view)
{
    focused_view = view;
}

EmbeddedViewer* osgEarth::GUI::getFocusedView()
{
    return focused_view.get();
}        

static osg::RefNodePath s_selectedNodePath;

osg::Node* osgEarth::GUI::getSelectedNode()
{
    if (s_selectedNodePath.empty())
    {
        return nullptr;
    }

    return s_selectedNodePath.back().get();
}

const osg::RefNodePath& osgEarth::GUI::getSelectedNodePath()
{
    return s_selectedNodePath;
}

void osgEarth::GUI::setSelectedNodePath(const osg::NodePath& nodePath)
{
    s_selectedNodePath.clear();

    for (auto itr = nodePath.begin(); itr != nodePath.end(); ++itr)
    {
        s_selectedNodePath.push_back(*itr);
    }
}