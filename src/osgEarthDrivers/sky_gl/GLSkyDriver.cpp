/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2015 Pelican Mapping
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
#include <osgEarthUtil/Sky>

#define LC "[GLSkyDriver] "

using namespace osgEarth;
using namespace osgEarth::Util;

namespace osgEarth { namespace Drivers { namespace GLSky
{
    class GLSkyExtension : public Extension,
                           public ExtensionInterface<MapNode>,
                           public ExtensionInterface<osg::View>,
                           public SkyNodeFactory,
                           public GLSkyOptions
    {
    public:
        META_Object( osgEarth, GLSkyExtension );

        GLSkyExtension() { }
        GLSkyExtension(const GLSkyOptions& options);

    public: // ExtensionInterface<MapNode>

        bool connect( MapNode* );
        bool disconnect( MapNode* );

    public: // ExtensionInterface<osg::View>

        bool connect( osg::View* );
        bool disconnect( osg::View* ) { return true; }

    public: // SkyNodeFactory

        SkyNode* getOrCreateSkyNode(const Profile* profile);

    protected:
        GLSkyExtension(const GLSkyExtension&, const osg::CopyOp&) { }
        virtual ~GLSkyExtension() { }

        osg::ref_ptr<SkyNode> _skyNode;
    };
    
} } } // namespace


#undef  LC
#define LC "[GLSky] "

using namespace osgEarth::Drivers::GLSky;

REGISTER_OSGEARTH_EXTENSION( osgearth_sky_gl, GLSkyExtension );


GLSkyExtension::GLSkyExtension(const GLSkyOptions& options) :
GLSkyOptions(options)
{
    //nop
}

bool
GLSkyExtension::connect(MapNode* mapNode)
{
    OE_INFO << LC << "Hello world.\n";
    
    // find the tip top of the tree that MapNode is in:
    osg::Node* top = mapNode;
    while( top->getNumParents() > 0 && top->getParent(0)->className() != "Camera")
        top = top->getParent(0);

    osg::Group* topParent = top->getNumParents() > 0 ? top->getParent(0) : 0L;

    // make the sky node
    if ( !_skyNode.valid() )
    {
        _skyNode = getOrCreateSkyNode( mapNode->getMap()->getProfile() );
    }
     
    _skyNode->addChild( top );

    // if we're not at the treetop, insert the sky node.
    if ( topParent )
    {
        topParent->addChild( _skyNode.get() );
        topParent->removeChild( top );
    }

    return true;
}

bool
GLSkyExtension::disconnect(MapNode* mapNode)
{
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
GLSkyExtension::getOrCreateSkyNode(const Profile* profile)
{
    return new GLSkyNode(profile, *this);
}


#if 0
    class GLSkyDriver : public SkyDriver
    {
    public:
        GLSkyDriver()
        {
            supportsExtension(
                "osgearth_sky_gl",
                "osgEarth GL Sky Plugin" );
        }

        const char* className()
        {
            return "osgEarth GL Sky Plugin";
        }

        ReadResult readNode(const std::string& file_name, const osgDB::Options* options) const
        {
            if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
                return ReadResult::FILE_NOT_HANDLED;

            MapNode* mapNode = getMapNode(options);
            const Profile* profile = mapNode ? mapNode->getMap()->getProfile() : 0L;
            return new GLSkyNode(profile, getSkyOptions(options));
        }

    protected:
        virtual ~GLSkyDriver() { }
    };

    REGISTER_OSGPLUGIN(osgearth_sky_gl, GLSkyDriver)
#endif
