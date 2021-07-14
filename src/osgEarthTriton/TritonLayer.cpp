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
#include "TritonLayer"
#include "TritonContext"
#include "TritonDrawable"
#include "TritonHeightMap"
#include "TritonCallback"

#include <osgEarth/MapNode>
#include <osgEarth/ImageLayer>
#include <osgEarth/NodeUtils>
#include <osgEarth/ElevationLOD>
#include <osgEarth/TerrainEngineNode>

#include <osgUtil/CullVisitor>

#define LC "[TritonLayer] "

using namespace osgEarth::Triton;

namespace osgEarth { namespace Triton
{
    class TritonLayerNode : public osg::Group
    {
    public:
        TritonLayerNode(osgEarth::Triton::TritonLayer* layer,
                        LayerReference<osgEarth::ImageLayer>& mask) :
            _tritonLayer(layer),
            _maskLayer(mask),
            _callback(0L),
            _needsMapNode(true)
        {
            // To detect the map node:
            ADJUST_UPDATE_TRAV_COUNT(this, +1);

            // Disable bounds culling
            setCullingActive(false);
        }

        ~TritonLayerNode()
        {
            //nop
        }

        void setUserCallback(osgEarth::Triton::Callback* callback)
        {
            _callback = callback;
        }

        void dirty()
        {
            create();
        }

        /** MapNode to use; will be discovered automatically if not set here */
        void setMapNode(osgEarth::MapNode* mapNode)
        {
            if (!mapNode)
            {
                this->removeChildren(0, this->getNumChildren());
                _drawable = 0L;
                _TRITON = 0L;
                _needsMapNode = true;
            }
            else
            {
                _mapNode = mapNode;
                create();
            }
        }

        void create()
        {
            this->removeChildren(0, this->getNumChildren());
            _drawable = 0L;

            osg::ref_ptr<osgEarth::MapNode> mapNode;
            if (!_mapNode.lock(mapNode))
                return;

            const osgEarth::Map* map = mapNode->getMap();

            // create an object to house Triton data and resources.
            if (!_TRITON.valid())
                _TRITON = new TritonContext(_tritonLayer->options());

            if (map)
                _TRITON->setSRS(map->getSRS());

            if (_callback.valid())
                _TRITON->setCallback(_callback.get());

            TritonDrawable* drawable = new TritonDrawable(_TRITON.get());
            _drawable = drawable;
            _drawable->setNodeMask(TRITON_OCEAN_MASK);
            drawable->setMaskLayer(_maskLayer.getLayer());
            this->addChild(_drawable);

            // Place in the depth-sorted bin and set a rendering order.
            // We want Triton to render after the terrain.
            _drawable->getOrCreateStateSet()->setRenderBinDetails(
                _tritonLayer->getRenderBinNumber(), 
                "DepthSortedBin");

            // If the user requested a height map, install it now.
            // Configuration of the height map generator will take place later when
            // we have a valid graphics context.            
            if (_tritonLayer->getUseHeightMap() == true)
            {
                TritonHeightMap* heightMapGen = new TritonHeightMap();
                heightMapGen->setTerrain(mapNode->getTerrainEngine()->getNode());
                if (_maskLayer.getLayer())
                    heightMapGen->setMaskLayer(_maskLayer.getLayer());
                this->addChild(heightMapGen);
                drawable->setHeightMapGenerator(heightMapGen);
            }
        }

        void traverse(osg::NodeVisitor& nv)
        {
            if (nv.getVisitorType() == nv.UPDATE_VISITOR)
            {
                // Find a MapNode in the traversal path if necessary:
                if (_needsMapNode)
                {
                    osgEarth::MapNode* mapNode = osgEarth::findInNodePath<osgEarth::MapNode>(nv);
                    if (mapNode)
                    {
                        setMapNode(mapNode);
                        _needsMapNode = false;
                        ADJUST_UPDATE_TRAV_COUNT(this, -1);
                    }
                }
            }

            else if (nv.getVisitorType() == nv.CULL_VISITOR && _drawable && _TRITON.valid() && _TRITON->getOcean())
            {
                // Update any intersections.
                // For now this is running in the CULL traversal, which is not ideal.
                // However the Triton Ocean::GetHeight method requires a pointer to the Triton "camera"
                // and under our framework this is only available in CULL or DRAW.
                // Running the intersection in eithe CULL or DRAW will result in a frame
                // incoherency w.r.t the Triton update() call that updates the ocean state.
                ::Triton::Ocean* ocean = _TRITON->getOcean();

                // Find the TritonCam associated with this osg Cam
                osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(&nv);
                ::Triton::Camera* tritonCam = static_cast<TritonDrawable*>(_drawable)->getTritonCam(cv->getCurrentCamera());

                osg::Vec3d eye = osg::Vec3d(0,0,0) * cv->getCurrentCamera()->getInverseViewMatrix();

                for(std::vector<osg::ref_ptr<TritonIntersections> >::iterator i = _isect.begin();
                    i != _isect.end();
                    ++i)
                {
                    TritonIntersections* ir = i->get();

                    // allocate enough space for the output:
                    ir->_input.resize(ir->_input.size());
                    ir->_normals.resize(ir->_input.size());

                    osg::Matrix local2world;
                    ir->_anchor.createLocalToWorld(local2world);

                    // Make sure it's in range so as not to waste cycles:
                    osg::Vec3d anchor = osg::Vec3d(0,0,0) * local2world;
                    double m = ir->getMaxRange().as(Units::METERS);
                    if ((eye-anchor).length2() > (m*m))
                    {
                        continue;
                    }

                    osg::Matrix world2local;
                    world2local.invert(local2world);

                    for(unsigned i=0; i<ir->_input.size(); ++i)
                    {
                        const osg::Vec3d& local = ir->_input[i];

                        // transform the ray to world coordinates
                        osg::Vec3d start = local * local2world;
                        osg::Vec3d dir = osg::Matrix::transform3x3(local2world, osg::Vec3d(0,0,1));

                        // intersect the ocean
                        float& out_height = ir->_heights[i];
                        ::Triton::Vector3 out_normalT;
                            
                        bool ok = ocean->GetHeight(
                            ::Triton::Vector3(start.x(), start.y(), start.z()),
                            ::Triton::Vector3(dir.x(), dir.y(), dir.z()),
                            out_height,
                            out_normalT,
                            true,           // visualCorrelation
                            true,           // includeWakes
                            true,           // highResolution
                            true,           // threadSafe,
                            0L,             // intersectionPoint,
                            true,           // autoFlip
                            tritonCam);

                        if (ok)
                        {
                            // populate the output data in local coordinates
                            osg::Vec3d& normal = ir->_normals[i];
                            normal.set(out_normalT.x, out_normalT.y, out_normalT.z);
                            normal = osg::Matrix::transform3x3(normal, world2local);
                        }
                        else
                        {
                            // todo...what?
                            OE_WARN << "GetHeight returned false dude" << std::endl;
                        }
                    }
                }
            }

            osg::Group::traverse(nv);
        }

        osg::ref_ptr<TritonContext> _TRITON;
        osg::Drawable* _drawable;
        LayerReference<osgEarth::ImageLayer>& _maskLayer;
        osg::observer_ptr<osgEarth::MapNode> _mapNode;
        osg::observer_ptr<osgEarth::Triton::TritonLayer> _tritonLayer;
        osg::ref_ptr<Callback> _callback;
        bool _needsMapNode;
        std::vector<osg::ref_ptr<TritonIntersections> > _isect;

    };
} }

//........................................................................

void
TritonLayer::Options::fromConfig(const osgEarth::Config& conf)
{
    _useHeightMap.init(true);
    _heightMapSize.init(1024);
    _renderBinNumber.init(12);
    _maxAltitude.init(50000);

    conf.get("license_code", _licenseCode);
    conf.get("resource_path", _resourcePath);
    conf.get("use_height_map", _useHeightMap);
    conf.get("height_map_size", _heightMapSize);
    conf.get("render_bin_number", _renderBinNumber);
    conf.get("max_altitude", _maxAltitude);
    maskLayer().get(conf, "mask_layer");
}

osgEarth::Config
TritonLayer::Options::getConfig() const
{
    osgEarth::Config conf = osgEarth::VisibleLayer::Options::getConfig();
    conf.set("license_code", _licenseCode);
    conf.set("resource_path", _resourcePath);
    conf.set("use_height_map", _useHeightMap);
    conf.set("height_map_size", _heightMapSize);
    conf.set("render_bin_number", _renderBinNumber);
    conf.set("max_altitude", _maxAltitude);
    maskLayer().set(conf, "mask_layer");

    return conf;
}

//........................................................................

/** Register this layer so it can be used in an earth file */
namespace osgEarth { namespace Triton
{
    REGISTER_OSGEARTH_LAYER(triton, TritonLayer);
    REGISTER_OSGEARTH_LAYER(triton_ocean, TritonLayer);
} }

OE_LAYER_PROPERTY_IMPL(TritonLayer, std::string, UserName, user);
OE_LAYER_PROPERTY_IMPL(TritonLayer, std::string, LicenseCode, licenseCode);
OE_LAYER_PROPERTY_IMPL(TritonLayer, std::string, ResourcePath, resourcePath);
OE_LAYER_PROPERTY_IMPL(TritonLayer, bool, UseHeightMap, useHeightMap);
OE_LAYER_PROPERTY_IMPL(TritonLayer, unsigned, HeightMapSize, heightMapSize);
OE_LAYER_PROPERTY_IMPL(TritonLayer, int, RenderBinNumber, renderBinNumber);
OE_LAYER_PROPERTY_IMPL(TritonLayer, float, MaxAltitude, maxAltitude);

void
TritonLayer::init()
{
    OE_INFO << LC << "Creating TritonLayer\n";

    osgEarth::VisibleLayer::init();

    _seaLevel = 0.0f;

    // Trick to force the VisibleLayer to install its opacity shader, 
    // which a modified Triton user-functions.glsl shader needs in order to control
    // sea surface opacity.
    float opacity = getOpacity();
    setOpacity(0.0f);
    setOpacity(opacity);

    this->setName("Triton");
    setRenderType(RENDERTYPE_CUSTOM);

    ElevationLOD* lod = new ElevationLOD();
    _root = lod;
    if (options().maxAltitude().isSet())
    {
        OE_INFO << LC << "Setting max altitude = " << options().maxAltitude().get() << std::endl;
        lod->setMaxElevation(options().maxAltitude().get());
    }

    _tritonNode = new TritonLayerNode(this, options().maskLayer());
    _root->addChild(_tritonNode.get());
}

void
TritonLayer::setUserCallback(Callback* callback)
{
    static_cast<TritonLayerNode*>(_tritonNode.get())->setUserCallback(callback);
}

osg::Node*
TritonLayer::getNode() const
{
    return _root.get();
}

void
TritonLayer::setMaskLayer(osgEarth::ImageLayer* maskLayer)
{
    options().maskLayer().setLayer(maskLayer);
    static_cast<TritonLayerNode*>(_tritonNode.get())->dirty();
}

osgEarth::ImageLayer*
TritonLayer::getMaskLayer() const
{
    return options().maskLayer().getLayer();
}

void
TritonLayer::addedToMap(const osgEarth::Map* map)
{   
    VisibleLayer::addedToMap(map);
    options().maskLayer().addedToMap(map);
}

void
TritonLayer::removedFromMap(const osgEarth::Map* map)
{
    VisibleLayer::removedFromMap(map);
    options().maskLayer().removedFromMap(map);
    setMaskLayer(0L);
}

void
TritonLayer::addIntersections(TritonIntersections* value)
{
    TritonLayerNode* node = static_cast<TritonLayerNode*>(_tritonNode.get());
    node->_isect.push_back(value);
}

osgEarth::Config
TritonLayer::getConfig() const
{
    osgEarth::Config c = osgEarth::VisibleLayer::getConfig();
    return c;
}
