/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
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
#include "CesiumTilesetNode"
#include "Context"
#include "Settings"

#include <osgEarth/Notify>
#include <osgUtil/CullVisitor>
#include <Cesium3DTilesSelection/BoundingVolume.h>

using namespace osgEarth::Cesium;

CesiumTilesetNode::CesiumTilesetNode(unsigned int assetID, const std::string& token, float maximumScreenSpaceError, std::vector<int> overlays)
{ 
    Cesium3DTilesSelection::TilesetExternals externals{
        Context::instance().assetAccessor, Context::instance().prepareRenderResources, Context::instance().asyncSystem, Context::instance().creditSystem, Context::instance().logger, nullptr
    };

    Cesium3DTilesSelection::TilesetOptions options;
    options.forbidHoles = true;
    options.enableOcclusionCulling = false;
    options.maximumScreenSpaceError = maximumScreenSpaceError;
    Cesium3DTilesSelection::Tileset* tileset = new Cesium3DTilesSelection::Tileset(externals, assetID, token, options);

    for (auto& overlay = overlays.begin(); overlay != overlays.end(); ++overlay)
    {
        Cesium3DTilesSelection::RasterOverlayOptions rasterOptions;
        const auto ionRasterOverlay = new Cesium3DTilesSelection::IonRasterOverlay("", *overlay, token, rasterOptions);
        tileset->getOverlays().add(ionRasterOverlay);
    }    
    _tileset = tileset;

    setCullingActive(false);    
}

CesiumTilesetNode::CesiumTilesetNode(const std::string& url, const std::string& token, float maximumScreenSpaceError, std::vector<int> overlays)
{
    Cesium3DTilesSelection::TilesetExternals externals{
        Context::instance().assetAccessor, Context::instance().prepareRenderResources, Context::instance().asyncSystem, Context::instance().creditSystem, Context::instance().logger, nullptr
    };

    Cesium3DTilesSelection::TilesetOptions options;
    options.forbidHoles = true;
    options.enableOcclusionCulling = false;
    options.maximumScreenSpaceError = maximumScreenSpaceError;
    Cesium3DTilesSelection::Tileset* tileset = new Cesium3DTilesSelection::Tileset(externals, url, options);
    for (auto& overlay = overlays.begin(); overlay != overlays.end(); ++overlay)
    {
        Cesium3DTilesSelection::RasterOverlayOptions rasterOptions;
        const auto ionRasterOverlay = new Cesium3DTilesSelection::IonRasterOverlay("", *overlay, token, rasterOptions);
        tileset->getOverlays().add(ionRasterOverlay);
    }
    _tileset = tileset;
    setCullingActive(false);
}


CesiumTilesetNode::~CesiumTilesetNode()
{
    Cesium3DTilesSelection::Tileset* tileset = (Cesium3DTilesSelection::Tileset*)_tileset;
    delete tileset;
}

float CesiumTilesetNode::getMaximumScreenSpaceError() const
{
    Cesium3DTilesSelection::Tileset* tileset = (Cesium3DTilesSelection::Tileset*)_tileset;
    return tileset->getOptions().maximumScreenSpaceError;
}

void CesiumTilesetNode::setMaximumScreenSpaceError(float maximumScreenSpaceError)
{
    Cesium3DTilesSelection::Tileset* tileset = (Cesium3DTilesSelection::Tileset*)_tileset;
    tileset->getOptions().maximumScreenSpaceError = maximumScreenSpaceError;
}

void
CesiumTilesetNode::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == nv.CULL_VISITOR)
    {
        const osgUtil::CullVisitor* cv = nv.asCullVisitor();
        osg::Vec3d osgEye, osgCenter, osgUp;
        cv->getModelViewMatrix()->getLookAt(osgEye, osgCenter, osgUp);
        osg::Vec3d osgDir = osgCenter - osgEye;
        osgDir.normalize();

        glm::dvec3 pos(osgEye.x(), osgEye.y(), osgEye.z());
        glm::dvec3 dir(osgDir.x(), osgDir.y(), osgDir.z());
        glm::dvec3 up(osgUp.x(), osgUp.y(), osgUp.z());
        glm::dvec2 viewportSize(cv->getViewport()->width(), cv->getViewport()->height());

        double vfov, ar, znear, zfar;
        cv->getProjectionMatrix()->getPerspective(vfov, ar, znear, zfar);
        vfov = osg::DegreesToRadians(vfov);
        double hfov = 2 * atan(tan(vfov / 2) * (ar));

        // TODO:  Multiple views
        std::vector<Cesium3DTilesSelection::ViewState> viewStates;
        Cesium3DTilesSelection::ViewState viewState = Cesium3DTilesSelection::ViewState::create(pos, dir, up, viewportSize, hfov, vfov);
        viewStates.push_back(viewState);
        Cesium3DTilesSelection::Tileset* tileset = (Cesium3DTilesSelection::Tileset*)_tileset;
        auto updates = tileset->updateView(viewStates);

        removeChildren(0, getNumChildren());
        for (auto tile : updates.tilesToRenderThisFrame)
        {
            if (tile->getContent().isRenderContent())
            {
                MainThreadResult* result = reinterpret_cast<MainThreadResult*>(tile->getContent().getRenderContent()->getRenderResources());
                if (result && result->node.valid()) {
                    addChild(result->node.get());
                }
            }
        }
    }
    else if (nv.getVisitorType() == nv.UPDATE_VISITOR)
    {        
        osg::Group::traverse(nv);
    }

    osg::Group::traverse(nv);
}

osg::BoundingSphere CesiumTilesetNode::computeBound() const
{
    if (_tileset)
    {
        Cesium3DTilesSelection::Tileset* tileset = (Cesium3DTilesSelection::Tileset*)_tileset;
        auto rootTile = tileset->getRootTile();
        if (rootTile)
        {
            auto bbox = Cesium3DTilesSelection::getOrientedBoundingBoxFromBoundingVolume(rootTile->getBoundingVolume());
            auto& center = bbox.getCenter();
            auto& lengths = bbox.getLengths();
            float radius = std::max(lengths.x, std::max(lengths.y, lengths.z));
            return osg::BoundingSphere(osg::Vec3(center.x, center.y, center.z), radius);
        }
    }
    return osg::BoundingSphere();
}