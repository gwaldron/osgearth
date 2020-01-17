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
#include <osgEarthUtil/LODBlending>
#include <osgEarth/Registry>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/MapNode>

#define LC "[LODBlending] "

using namespace osgEarth;
using namespace osgEarth::Util;

namespace
{
    // This shader will morph elevation from old heights to new heights as
    // installed in the terrain tile's vertex attributes. oe_terrain_attr[3] holds
    // the new value; oe_terrain_attr[3] holds the old one. 
    //
    // We use two methods: distance to vertex and time. The morph ratio is
    // a function of the distance from the camera to the vertex (taking into
    // consideration the tile range factor), but when we limit that based on
    // a timer. This prevents fast zooming from skipping the morph altogether.
    //
    // It will also transition between a parent texture and the current texture.
    //
    // Caveats: You can still fake out the morph by zooming around very quickly.
    // Also, it will only morph properly if you use odd-numbers post spacings
    // in your terrain tile. (See MapOptions::elevation_tile_size). Finally,
    // a large PAGEDLOD cache will negate the blending effect when zooming out
    // and then back in. See MPGeometry.

    const char* vs_imagery =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"

        "uniform float oe_min_tile_range_factor; \n"
        "uniform vec4 oe_tile_key; \n"
        "uniform float osg_FrameTime; \n"
        "uniform float oe_tile_birthtime; \n"
        "uniform float oe_lodblend_delay; \n"
        "uniform float oe_lodblend_duration; \n"
        "uniform mat4 oe_layer_parent_texmat; \n"

        "out vec4 oe_layer_texc; \n"
        "out vec4 oe_lodblend_texc; \n"
        "out float oe_lodblend_r; \n"

        "void oe_lodblend_imagery_vertex(inout vec4 VertexVIEW) \n"
        "{ \n"
        "    float radius     = oe_tile_key.w; \n"
        "    float near       = oe_min_tile_range_factor*radius; \n"
        "    float far        = near + radius*2.0; \n"
        "    float d          = length(VertexVIEW.xyz/VertexVIEW.w); \n"
        "    float r_dist     = clamp((d-near)/(far-near), 0.0, 1.0); \n"

        "    float r_time     = 1.0 - clamp(osg_FrameTime-(oe_tile_birthtime+oe_lodblend_delay), 0.0, oe_lodblend_duration)/oe_lodblend_duration; \n"
        "    float r          = max(r_dist, r_time); \n"

        "    oe_lodblend_texc = oe_layer_parent_texmat * oe_layer_texc; \n"
        "    oe_lodblend_r    = oe_layer_parent_texmat[0][0] > 0.0 ? r : 0.0; \n" // obe?
        "} \n";

    const char* vs_elevation =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"

        "in vec4 oe_terrain_attr; \n"
        "in vec4 oe_terrain_attr2; \n"
        
        "vec3 vp_Normal; \n"

        "uniform float oe_min_tile_range_factor; \n"
        "uniform vec4 oe_tile_key; \n"
        "uniform float osg_FrameTime; \n"
        "uniform float oe_tile_birthtime; \n"
        "uniform float oe_lodblend_delay; \n"
        "uniform float oe_lodblend_duration; \n"
        "uniform float oe_lodblend_vscale; \n"

        "void oe_lodblend_elevation_vertex(inout vec4 VertexMODEL) \n"
        "{ \n"
        "    float radius     = oe_tile_key.w; \n"
        "    float near       = oe_min_tile_range_factor*radius; \n"
        "    float far        = near + radius*2.0; \n"
        "    vec4  VertexVIEW = gl_ModelViewMatrix * VertexMODEL; \n"
        "    float d          = length(VertexVIEW.xyz/VertexVIEW.w); \n"
        "    float r_dist     = clamp((d-near)/(far-near), 0.0, 1.0); \n"

        "    float r_time     = 1.0 - clamp(osg_FrameTime-(oe_tile_birthtime+oe_lodblend_delay), 0.0, oe_lodblend_duration)/oe_lodblend_duration; \n"
        "    float r          = max(r_dist, r_time); \n"

        "    vec3  upVector   = oe_terrain_attr.xyz; \n"
        "    float elev       = oe_terrain_attr.w; \n"
        "    float elevOld    = oe_terrain_attr2.w; \n"

        "    vec3  vscaleOffset = upVector * elev * (oe_lodblend_vscale-1.0); \n"
        "    vec3  blendOffset  = upVector * r * oe_lodblend_vscale * (elevOld-elev); \n"
        "    VertexMODEL       += vec4( (vscaleOffset + blendOffset)*VertexMODEL.w, 0.0 ); \n"
        "} \n";

    const char* vs_combined =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"

        "in vec4 oe_terrain_attr; \n"
        "in vec4 oe_terrain_attr2; \n"

        "vec3 vp_Normal; \n"

        "uniform float oe_min_tile_range_factor; \n"
        "uniform vec4 oe_tile_key; \n"
        "uniform float osg_FrameTime; \n"
        "uniform float oe_tile_birthtime; \n"
        "uniform float oe_lodblend_delay; \n"
        "uniform float oe_lodblend_duration; \n"
        "uniform float oe_lodblend_vscale; \n"
        "uniform mat4 oe_layer_parent_texmat; \n"

        "out vec4 oe_layer_texc; \n"
        "out vec4 oe_lodblend_texc; \n"
        "out float oe_lodblend_r; \n"

        "void oe_lodblend_combined_vertex(inout vec4 VertexMODEL) \n"
        "{ \n"
        "    float radius     = oe_tile_key.w; \n"
        "    float near       = oe_min_tile_range_factor*radius; \n"
        "    float far        = near + radius*2.0; \n"
        "    vec4  VertexVIEW = gl_ModelViewMatrix * VertexMODEL; \n"
        "    float d          = length(VertexVIEW.xyz/VertexVIEW.w); \n"
        "    float r_dist     = clamp((d-near)/(far-near), 0.0, 1.0); \n"

        "    float r_time     = 1.0 - clamp(osg_FrameTime-(oe_tile_birthtime+oe_lodblend_delay), 0.0, oe_lodblend_duration)/oe_lodblend_duration; \n"
        "    float r          = max(r_dist, r_time); \n"

        "    vec3  upVector   = oe_terrain_attr.xyz; \n"
        "    float elev       = oe_terrain_attr.w; \n"
        "    float elevOld    = oe_terrain_attr2.w; \n"

        "    vec3  vscaleOffset = upVector * elev * (oe_lodblend_vscale-1.0); \n"
        "    vec3  blendOffset  = upVector * r * oe_lodblend_vscale * (elevOld-elev); \n"
        "    VertexMODEL       += vec4( (vscaleOffset + blendOffset)*VertexMODEL.w, 0.0 ); \n"

        "    oe_lodblend_texc = oe_layer_parent_texmat * oe_layer_texc; \n"
        "    oe_lodblend_r    = oe_layer_parent_texmat[0][0] > 0.0 ? r : 0.0; \n" // obe?

        "    vp_Normal = normalize(mix(normalize(vp_Normal), oe_terrain_attr2.xyz, r)); \n"
        "} \n";

    const char* fs_imagery =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"

        "uniform vec4 oe_tile_key; \n"
        "uniform int oe_layer_uid; \n"
        "uniform sampler2D oe_layer_tex_parent; \n"

        "in vec4 oe_lodblend_texc; \n"
        "in float oe_lodblend_r; \n"

        "void oe_lodblend_imagery_fragment(inout vec4 color) \n"
        "{ \n"
        "    if ( oe_layer_uid >= 0 ) \n"
        "    { \n"
        "        vec4 texel = texture(oe_layer_tex_parent, oe_lodblend_texc.st); \n"
        "        float enable = step(0.09, texel.a); \n"          // did we get a parent texel?
        "        texel.rgb = mix(color.rgb, texel.rgb, enable); \n" // if not, use the incoming color for the blend
        "        texel.a = mix(0.0, color.a, enable); \n"           // ...and blend from alpha=0 for a fade-in effect.
        "        color = mix(color, texel, oe_lodblend_r); \n"
        "    } \n"
        "} \n";
}

LODBlending::LODBlending()
{
    init();
}


LODBlending::LODBlending(const LODBlendingOptions& options) :
LODBlendingOptions(options)
{
    init();
}


void
LODBlending::init()
{
    _delayUniform = new osg::Uniform(osg::Uniform::FLOAT, "oe_lodblend_delay");
    _delayUniform->set( osg::clampAbove(delay().get(), 0.0f));

    _durationUniform = new osg::Uniform(osg::Uniform::FLOAT, "oe_lodblend_duration");
    _durationUniform->set( osg::clampAbove(duration().get(), 0.0f) );

    _vscaleUniform = new osg::Uniform(osg::Uniform::FLOAT, "oe_lodblend_vscale");
    _vscaleUniform->set(osg::clampAbove(verticalScale().get(), 0.0f));
}


void
LODBlending::onInstall(TerrainEngineNode* engine)
{
    if ( engine )
    {
        if (engine->getName() == "osgEarth.RexTerrainEngineNode")
        {
            OE_WARN << LC << "LODBlending extension will be disabled; terrain engine supports blending natively" << std::endl;
            return;
        }

        // need the parent textures for blending.
        engine->requireParentTextures();

        osg::StateSet* stateset = engine->getOrCreateStateSet();

        stateset->addUniform( _delayUniform.get() );
        stateset->addUniform( _durationUniform.get() );
        stateset->addUniform( _vscaleUniform.get() );

        VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);
        vp->setName( "osgEarth::Util::LODBlending" );

        if ( blendElevation() == true )
        {
            vp->setFunction("oe_lodblend_elevation_vertex", vs_elevation, ShaderComp::LOCATION_VERTEX_MODEL );
        }

        if ( blendImagery() == true )
        {
            vp->setFunction("oe_lodblend_imagery_vertex", vs_imagery, ShaderComp::LOCATION_VERTEX_VIEW);
            vp->setFunction("oe_lodblend_imagery_fragment", fs_imagery, ShaderComp::LOCATION_FRAGMENT_COLORING);
        }

        OE_INFO << LC << "On!\n";
    }
}


void
LODBlending::onUninstall(TerrainEngineNode* engine)
{
    if ( engine )
    {
        osg::StateSet* stateset = engine->getStateSet();
        if ( stateset )
        {
            stateset->removeUniform( _delayUniform.get() );
            stateset->removeUniform( _durationUniform.get() );
            stateset->removeUniform( _vscaleUniform.get() );

            VirtualProgram* vp = VirtualProgram::get(stateset);
            if ( vp )
            {
                vp->removeShader( "oe_lodblend_imagery_vertex" );
                vp->removeShader( "oe_lodblend_elevation_vertex" );
                vp->removeShader( "oe_lodblend_imagery_fragment" );
            }
        }
    }
}


//--------------------------------------------------------------


#undef  LC
#define LC "[LODBlendingExtension] "

/**
    * Extension for loading the graticule node on demand.
    */
class LODBlendingExtension : public Extension,
                             public ExtensionInterface<MapNode>,
                             public LODBlendingOptions
{
public:
    META_OE_Extension(osgEarth, LODBlendingExtension, lod_blending);

    // CTORs
    LODBlendingExtension() { }
    LODBlendingExtension(const ConfigOptions& options) : LODBlendingOptions(options) { }

    // DTOR
    virtual ~LODBlendingExtension() { }


public: // Extension

    virtual const ConfigOptions& getConfigOptions() const { return *this; }


public: // ExtensionInterface<MapNode>

    bool connect(MapNode* mapNode)
    {
        if (!_effect.valid())
        {
            _effect = new LODBlending(*this);
            mapNode->getTerrainEngine()->addEffect(_effect.get());
        }
        return true;
    }

    bool disconnect(MapNode* mapNode)
    {
        if (mapNode && _effect.valid())
        {
            mapNode->getTerrainEngine()->removeEffect(_effect.get());
            _effect = 0L;
        }
        return true;
    }

private:
    osg::ref_ptr<LODBlending> _effect;
};

REGISTER_OSGEARTH_EXTENSION( osgearth_lod_blending, LODBlendingExtension );
