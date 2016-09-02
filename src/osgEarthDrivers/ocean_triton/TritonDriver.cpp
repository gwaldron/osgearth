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
#include <osgDB/ReaderWriter>
#include <osgDB/FileNameUtils>
#include <osgDB/Registry>
#include <osgDB/FileUtils>
#include <osgEarth/MapNode>
#include <osgEarth/Registry>
#include <osgEarth/ThreadingUtils>
#include <osgEarthUtil/Ocean>

#include <osgEarthTriton/TritonOptions>
#include <osgEarthTriton/TritonNode>

#undef  LC
#define LC "[TritonDriver] "

//---------------------------------------------------------------------------

namespace osgEarth { namespace Triton
{
    using namespace osgEarth;
    using namespace osgEarth::Util;

    /**
     * Extension that lets you load a SimpleOcean from an earth file.
     */
    class TritonExtension : public Extension,
                            public ExtensionInterface<MapNode>,
                            public TritonOptions,
                            public OceanNodeFactory
    {
    public:
        META_Object(osgearth_ocean_triton, TritonExtension);

        TritonExtension() { }

        TritonExtension(const ConfigOptions& options) :
            TritonOptions(options) { }

    public: // Extension

        const ConfigOptions& getConfigOptions() const { return *this; }

    public: // ExtensionInterface<MapNode>

        bool connect(MapNode* mapNode)
        {
            _oceanNode = createOceanNode(mapNode);
            mapNode->addChild(_oceanNode.get());
            return true;
        }

        bool disconnect(MapNode* mapNode)
        {
            if (mapNode && _oceanNode.valid())
                mapNode->removeChild(_oceanNode.get());
            return true;
        }

    public: // OceanNodeFactory

        OceanNode* createOceanNode(MapNode* mapNode) {
            return new TritonNode(mapNode, *this);
        }

    protected:
        TritonExtension(const TritonExtension& rhs, const osg::CopyOp& op) { }

        virtual ~TritonExtension() { }

        osg::ref_ptr<OceanNode> _oceanNode;
    };

    REGISTER_OSGEARTH_EXTENSION(osgearth_ocean_triton, TritonExtension);

} } // namespace osgEarth::Drivers::Triton
