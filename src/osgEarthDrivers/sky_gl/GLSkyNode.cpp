/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2013 Pelican Mapping
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

#include "GLSkyNode"
#include "GLSkyShaders"

#include <osgEarthUtil/StarData>

#include <osgEarth/VirtualProgram>
#include <osgEarth/NodeUtils>
#include <osgEarth/Map>
#include <osgEarth/Utils>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/CullingUtils>
#include <osgEarth/ShaderFactory>

#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osg/PointSprite>
#include <osg/BlendFunc>
#include <osg/FrontFace>
#include <osg/CullFace>
#include <osg/Program>
#include <osg/Camera>
#include <osg/Point>
#include <osg/Shape>
#include <osg/Depth>
#include <osg/Quat>

#include <sstream>
#include <time.h>

#define LC "[GLSkyNode] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Drivers::GLSky;

//---------------------------------------------------------------------------

GLSkyNode::GLSkyNode(const SpatialReference* srs) :
SkyNode()
{
    initialize(srs);
}

GLSkyNode::GLSkyNode(const SpatialReference* srs,
                     const GLSkyOptions&     options) :
SkyNode ( options )
{
    initialize(srs);
}

void
GLSkyNode::initialize(const SpatialReference* srs)
{
    _srs = srs;
    _light = new osg::Light(0);

    // installs the main uniforms and the shaders that will light the subgraph (terrain).
    osg::StateSet* stateset = this->getOrCreateStateSet();

    VirtualProgram* vp = VirtualProgram::getOrCreate( stateset );
    vp->setName( "GLSky Scene Lighting" );

    // simple lighting.
    vp->setFunction(
        "oe_sky_vertex_main",
        Phong_Vertex,
        ShaderComp::LOCATION_VERTEX_VIEW);

    vp->setFunction(
        "oe_sky_fragment_main", 
        Phong_Fragment,
        ShaderComp::LOCATION_FRAGMENT_LIGHTING);

    onSetDateTime();
}

void
GLSkyNode::onSetEphemeris()
{
    // trigger the date/time update.
    onSetDateTime();
}

void
GLSkyNode::onSetDateTime()
{
    if ( !getSunLight() || !_srs.valid() )
        return;

    const DateTime& dt = getDateTime();
    osg::Vec3d sunPos = getEphemeris()->getSunPositionECEF( dt );
    if ( _srs->isGeographic() )
    {
        sunPos.normalize();
        getSunLight()->setPosition( osg::Vec4(sunPos, 0.0) );
    }
    else
    {
        GeoPoint sunPosECEF( _srs->getECEF(), sunPos, ALTMODE_ABSOLUTE );
        GeoPoint sunPosMap;
        if ( sunPosECEF.transform(_srs.get(), sunPosMap) )
        {
            getSunLight()->setPosition( osg::Vec4(sunPosMap.vec3d(), 0.0) );
        }
    }
}

void
GLSkyNode::attach( osg::View* view, int lightNum )
{
    if ( !view ) return;

    _light->setLightNum( lightNum );
    view->setLight( _light.get() );
    view->setLightingMode( osg::View::SKY_LIGHT );

    onSetDateTime();
}
