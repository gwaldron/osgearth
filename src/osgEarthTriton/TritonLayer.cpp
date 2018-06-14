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
#include "TritonLayer"
#include "TritonNode"
#include "TritonContext"
#include "TritonDrawable"
#include "TritonHeightMap"

#include <osgEarth/MapNode>
#include <osgEarth/ImageLayer>
#include <osgEarth/ResourceReleaser>
#include <osgEarth/NodeUtils>
#include <osgEarth/ElevationLOD>
#include <osgEarth/TerrainEngineNode>

#define LC "[TritonLayer] "

using namespace osgEarth::Triton;

namespace
{
    class TritonLayerNode : public osg::Group
    {
    public:
        TritonLayerNode(const TritonLayerOptions& options, Callback* callback) :
            _options(options),
            _callback(callback),
            _needsMapNode(true)
        {
            // Triton requires a constant update traversal.
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
                osg::ref_ptr<osgEarth::ResourceReleaser> releaser;
                if (_releaser.lock(releaser))
                {
                    releaser->push(_TRITON.get());
                }
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

            TritonDrawable* drawable = new TritonDrawable(mapNode.get(), _TRITON.get());
            _drawable = drawable;
            _alphaUniform = getOrCreateStateSet()->getOrCreateUniform("oe_ocean_alpha", osg::Uniform::FLOAT);
            //_alphaUniform->set(getAlpha());
            _alphaUniform->set(1.0f); // TODO
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
                    }
                }

                // Tick Triton each frame:
                if (_TRITON->ready())
                {
                    _TRITON->update(nv.getFrameStamp()->getSimulationTime());
                }
            }

            osg::Group::traverse(nv);
        }

        osg::ref_ptr<TritonContext> _TRITON;
        TritonOptions               _options;
        osg::Drawable*              _drawable;
        osg::ref_ptr<osg::Uniform>  _alphaUniform;
        osg::observer_ptr<osgEarth::ResourceReleaser> _releaser;
        osg::observer_ptr<const osgEarth::ImageLayer> _maskLayer;
        osg::observer_ptr<osgEarth::MapNode> _mapNode;
        osg::ref_ptr<Callback> _callback;
        bool _needsMapNode;
    };
}

//........................................................................

/** Register this layer so it can be used in an earth file */
namespace osgEarth { namespace Triton
{
    REGISTER_OSGEARTH_LAYER(triton, TritonLayer);
    REGISTER_OSGEARTH_LAYER(triton_ocean, TritonLayer);
} }


TritonLayer::TritonLayer() :
osgEarth::VisibleLayer(&_optionsConcrete),
_options(&_optionsConcrete)
{
    init();
}

TritonLayer::TritonLayer(const TritonLayerOptions& options) :
osgEarth::VisibleLayer(&_optionsConcrete),
_options(&_optionsConcrete),
_optionsConcrete(options)
{
    init();
}

void
TritonLayer::init()
{
    OE_INFO << LC << "Creating TritonLayer\n";

    osgEarth::VisibleLayer::init();

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

    _tritonNode = new TritonLayerNode(options(), 0L);
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

