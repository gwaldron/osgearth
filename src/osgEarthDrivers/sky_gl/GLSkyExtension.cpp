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
#include "GLSkyOptions"
#include "GLSkyNode"
#include <osg/Camera>
#include <osg/View>
#include <osgDB/FileNameUtils>
#include <osgEarth/Map>
#include <osgEarth/MapNode>
#include <osgEarth/NodeUtils>
#include <osgEarth/Sky>
#include <osgEarth/ExampleResources>

#define LC "[GLSkyDriver] "

using namespace osgEarth;
using namespace osgEarth::Util;

namespace osgEarth { namespace GLSky
{
    class GLSkyExtension : 
        public Extension,
        public ExtensionInterface<MapNode>,
        public ExtensionInterface<osg::View>,
        public SkyNodeFactory,
        public GLSkyOptions
    {
    public:
        META_OE_Extension( osgEarth, GLSkyExtension, sky_gl );

        GLSkyExtension() { }
        GLSkyExtension(const GLSkyOptions& options);

        const ConfigOptions& getConfigOptions() const { return *this; }

    public: // ExtensionInterface<MapNode>

        bool connect( MapNode* );
        bool disconnect( MapNode* );

    public: // ExtensionInterface<osg::View>

        bool connect( osg::View* );
        bool disconnect( osg::View* ) { return true; }

    public: // SkyNodeFactory

        SkyNode* createSkyNode();

    protected:
        virtual ~GLSkyExtension() { }

        osg::ref_ptr<SkyNode> _skyNode;
    };

    REGISTER_OSGEARTH_EXTENSION( osgearth_sky_gl, GLSkyExtension );
    
} } // namespace


#undef  LC
#define LC "[GLSky] "

using namespace osgEarth::GLSky;


GLSkyExtension::GLSkyExtension(const GLSkyOptions& options) :
GLSkyOptions(options)
{
    //nop
}

bool
GLSkyExtension::connect(MapNode* mapNode)
{
    _skyNode = createSkyNode();

    // Projected map? Set up a reference point at the center of the map
    if (mapNode->getMapSRS()->isProjected())
    {
        GeoPoint refPoint = 
            mapNode->getMap()->getProfile()->getExtent().getCentroid();
        _skyNode->setReferencePoint(refPoint);
    }

    osgEarth::insertParent(_skyNode.get(), mapNode);
    return true;
}

bool
GLSkyExtension::disconnect(MapNode* mapNode)
{
    osgEarth::removeGroup(_skyNode.get());
    _skyNode = 0L;
    return true;
}

bool
GLSkyExtension::connect(osg::View* view)
{
    if ( view && _skyNode.valid() )
    {
        _skyNode->attach( view, 0 );
    }
    return true;
}

SkyNode*
GLSkyExtension::createSkyNode()
{
    GLSkyNode* sky = new GLSkyNode(*this);
    return sky;
}
