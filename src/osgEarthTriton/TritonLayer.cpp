/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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

#include <osgEarth/MapNode>
#include <osgEarth/ImageLayer>
#include <osgEarth/ResourceReleaser>
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
        TritonLayerNode(const TritonLayerOptions& options, Callback* callback, osgEarth::Triton::TritonLayer* layer) :
            _options(options),
            _tritonLayer(layer),
            _callback(callback),
            _needsMapNode(true)
        {
            // To detect the map node:
            ADJUST_UPDATE_TRAV_COUNT(this, +1);

            // Disable bounds culling
            setCullingActive(false);
        }

        ~TritonLayerNode()
        {
            // submit the TRITON context to the releaser so it can shut down Triton
            // objects in a valid graphics context.
            if (_TRITON.valid())
            {
                // TODO: 2.10+ trying to delete the TRITON object crashes.
#if 0
                osg::ref_ptr<osgEarth::ResourceReleaser> releaser;
                if (_releaser.lock(releaser))
                {
                    releaser->push(_TRITON.get());
                }
#else
                //_TRITON->releaseGLObjects(0L);
#endif
            }
        }

        /** Layer to use to mask the rendering of the ocean surface */
        void setMaskLayer(const osgEarth::ImageLayer* layer)
        {
            _maskLayer = layer;
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

            // Remember the resource releaser so we can properly destroy 
            // Triton objects in a graphics context.
            _releaser = mapNode->getResourceReleaser();

            // create an object to house Triton data and resources.
            if (!_TRITON.valid())
                _TRITON = new TritonContext(_options);

            if (map)
                _TRITON->setSRS(map->getSRS());

            if (_callback.valid())
                _TRITON->setCallback(_callback.get());

            TritonDrawable* drawable = new TritonDrawable(_TRITON.get());
            _drawable = drawable;
            _drawable->setNodeMask(TRITON_OCEAN_MASK);
            drawable->setMaskLayer(_maskLayer.get());
            this->addChild(_drawable);

            // Place in the depth-sorted bin and set a rendering order.
            // We want Triton to render after the terrain.
            _drawable->getOrCreateStateSet()->setRenderBinDetails(_options.renderBinNumber().get(), "DepthSortedBin");

            // If the user requested a height map, install it now.
            // Configuration of the height map generator will take place later when
            // we have a valid graphics context.
            if (_options.useHeightMap() == true)
            {
                TritonHeightMap* heightMapGen = new TritonHeightMap();
                heightMapGen->setTerrain(mapNode->getTerrainEngine());
                if (_maskLayer.valid())
                    heightMapGen->setMaskLayer(_maskLayer.get());
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
        TritonOptions               _options;
        osg::Drawable*              _drawable;
        osg::observer_ptr<osgEarth::ResourceReleaser> _releaser;
        osg::observer_ptr<const osgEarth::ImageLayer> _maskLayer;
        osg::observer_ptr<osgEarth::MapNode> _mapNode;
        osg::observer_ptr<osgEarth::Triton::TritonLayer> _tritonLayer;
        osg::ref_ptr<Callback> _callback;
        bool _needsMapNode;
        std::vector<osg::ref_ptr<TritonIntersections> > _isect;

    };
} }

//........................................................................

/** Register this layer so it can be used in an earth file */
namespace osgEarth { namespace Triton
{
    REGISTER_OSGEARTH_LAYER(triton, TritonLayer);
    REGISTER_OSGEARTH_LAYER(triton_ocean, TritonLayer);
} }


TritonLayer::TritonLayer(Callback* userCallback) :
osgEarth::VisibleLayer(&_optionsConcrete),
_options(&_optionsConcrete),
_callback(userCallback)
{
    init();
}

TritonLayer::TritonLayer(const TritonLayerOptions& options, Callback* userCallback) :
osgEarth::VisibleLayer(&_optionsConcrete),
_options(&_optionsConcrete),
_optionsConcrete(options),
_callback(userCallback)
{
    init();
}

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

    _tritonNode = new TritonLayerNode(options(), _callback.get(), this);
    _root->addChild(_tritonNode.get());
}


osg::Node*
TritonLayer::getNode() const
{
    return _root.get();
}

void
TritonLayer::setMaskLayer(const osgEarth::ImageLayer* maskLayer)
{
    static_cast<TritonLayerNode*>(_tritonNode.get())->setMaskLayer(maskLayer);
}

void
TritonLayer::addedToMap(const osgEarth::Map* map)
{    
    if (options().maskLayer().isSet())
    {
        // listen for the mask layer.
        _layerListener.listen(map, options().maskLayer().get(), this, &TritonLayer::setMaskLayer);
    }      
}

void
TritonLayer::removedFromMap(const osgEarth::Map* map)
{
    if (options().maskLayer().isSet())
    {
        _layerListener.clear();
        setMaskLayer(0L);
    }
}

void
TritonLayer::addIntersections(TritonIntersections* value)
{
    TritonLayerNode* node = static_cast<TritonLayerNode*>(_tritonNode.get());
    node->_isect.push_back(value);
}
