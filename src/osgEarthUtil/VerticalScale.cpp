/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
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
#include <osgEarthUtil/VerticalScale>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/VirtualProgram>

#define LC "[VerticalScale] "

using namespace osgEarth;
using namespace osgEarth::Util;


namespace
{
    // In the vertex shader, we use a vertex attribute that's genreated by the
    // terrain engine. In this example it's called "oe_vertscale_attribs" but you 
    // can give it any name you want, as long as it's bound to the proper
    // attribute location (see code). 
    //
    // The attribute contains a vec4 which holds the "up vector", the length of
    // which is the elevation, in indexes[0,1,2]. The height value is in
    // index[3].
    //
    // Here, we use the vertical scale uniform to move the vertex up or down
    // along its extrusion vector.

    const char* vs =
        "attribute vec4 oe_terrain_attr; \n"
        "uniform float oe_vertscale_scale; \n"

        "void oe_vertscale_vertex(inout vec4 VertexMODEL) \n"
        "{ \n"
        "    vec3  upVector  = oe_terrain_attr.xyz; \n"
        "    float elev      = oe_terrain_attr.w; \n"
        "    vec3  offset    = upVector * elev * (oe_vertscale_scale-1.0); \n"
        "    VertexMODEL    += vec4(offset/VertexMODEL.w, 0.0); \n"
        "} \n";
}


VerticalScale::VerticalScale() :
_scale( 1.0f )
{
    _scaleUniform = new osg::Uniform(osg::Uniform::FLOAT, "oe_vertscale_scale");
    _scaleUniform->set( _scale );
}


VerticalScale::~VerticalScale()
{
    setTerrainNode(0L);
}


void
VerticalScale::setScale(float scale)
{
    _scale = scale;
    _scaleUniform->set( _scale );
}


void
VerticalScale::setTerrainNode(osg::Node* node)
{
    if ( node )
    {
        osg::StateSet* ss = node->getOrCreateStateSet();

        ss->addUniform( _scaleUniform.get() );

        VirtualProgram* vp = dynamic_cast<VirtualProgram*>(ss->getAttribute(VirtualProgram::SA_TYPE));
        if ( !vp )
        {
            vp = new VirtualProgram();
            ss->setAttributeAndModes( vp, 1 );
        }
        vp->setFunction( "oe_vertscale_vertex", vs, ShaderComp::LOCATION_VERTEX_MODEL );
        //vp->addBindAttribLocation( "oe_vertscale_attribs",  osg::Drawable::ATTRIBUTE_6 );
    }
    else
    {
        //todo - remove
        OE_WARN << LC << "Remove NYI!" << std::endl;
    }
}
