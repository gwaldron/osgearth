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
#include <osgEarthUtil/VerticalScale>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/VirtualProgram>
#include <osgEarth/TerrainEngineNode>

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
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"

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
TerrainEffect(),
_scale       ( 1.0f )
{
    init();
}

VerticalScale::VerticalScale(const Config& conf) :
TerrainEffect(),
_scale       ( 1.0f )
{
    mergeConfig(conf);
    init();
}


void
VerticalScale::init()
{
    _scaleUniform = new osg::Uniform(osg::Uniform::FLOAT, "oe_vertscale_scale");
    _scaleUniform->set( _scale.get() );
}


void
VerticalScale::setScale(float scale)
{
    if ( scale != _scale.get() )
    {
        _scale = scale;
        _scaleUniform->set( _scale.get() );
    }
}


void
VerticalScale::onInstall(TerrainEngineNode* engine)
{
    if ( engine )
    {
        osg::StateSet* stateset = engine->getOrCreateStateSet();

        stateset->addUniform( _scaleUniform.get() );

        VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);
        vp->setFunction( "oe_vertscale_vertex", vs, ShaderComp::LOCATION_VERTEX_MODEL, 0.5f);
    }
}


void
VerticalScale::onUninstall(TerrainEngineNode* engine)
{
    if ( engine )
    {
        osg::StateSet* stateset = engine->getStateSet();
        if ( stateset )
        {
            stateset->removeUniform( _scaleUniform.get() );

            VirtualProgram* vp = VirtualProgram::get(stateset);
            if ( vp )
            {
                vp->removeShader( "oe_vertscale_vertex" );
            }
        }
    }
}



//-------------------------------------------------------------


void
VerticalScale::mergeConfig(const Config& conf)
{
    conf.getIfSet( "scale", _scale );
}

Config
VerticalScale::getConfig() const
{
    Config conf("vertical_scale");
    conf.addIfSet( "scale", _scale );
    return conf;
}
