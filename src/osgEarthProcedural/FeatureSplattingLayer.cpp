#include "FeatureSplattingLayer"
#include <osgEarth/Capabilities>
#include <osgEarth/Registry>
#include <osgEarth/TerrainEngineNode>

using namespace osgEarth;
using namespace osgEarth::Procedural;

REGISTER_OSGEARTH_LAYER(featuresplatting, osgEarth::Procedural::FeatureSplattingLayer);

#undef LC
#define LC "[FeatureSplattingLayer] "

namespace
{
    const char* vs = R"(
        vec4 oe_layer_tilec;
        out vec2 oe_feature_splatting_uv;
        vec2 oe_terrain_scaleCoordsToRefLOD(in vec2 tc, in float refLOD); // SDK

        void oe_feature_splatting_vs(inout vec4 vertex)
        {
            vec2 uv = oe_terrain_scaleCoordsToRefLOD(oe_layer_tilec.st, 22);
            oe_feature_splatting_uv = uv;
        }
    )";

    const char* fs = R"(
        #pragma import_defines(OE_HAS_ALBEDO_TEX, OE_HAS_NORMAL_TEX, OE_HAS_PBR_TEX)

        in vec2 oe_feature_splatting_uv;
        in float oe_layer_opacity;

        uniform sampler2D oe_albedo_tex;
        uniform sampler2D oe_normal_tex;
        uniform sampler2D oe_pbr_tex;

        uniform float oe_normal_boost = 1.0;
   
        mat3 oe_normalMapTBN;
        vec3 vp_Normal;
        struct OE_PBR { float displacement, roughness, ao, metal; } oe_pbr;

        // Andrey Mishkinis height+alpha blender.
        // https://www.gamedeveloper.com/programming/advanced-terrain-texture-splatting
        float heightAndEffectMix(in float h1, in float a1, in float h2, in float a2)
        {
            const float oe_displacement_depth = 0.1;
            float ma = max(h1 + a1, h2 + a2) - oe_displacement_depth;
            float b1 = max(h1 + a1 - ma, 0.0);
            float b2 = max(h2 + a2 - ma, 0.0);
            return b2 / (b1 + b2);
        }

        void oe_feature_splatting_fs(inout vec4 color)
        {
            vec2 uv = oe_feature_splatting_uv;
            float unquantized_blend = 1.0;
            float quantized_blend = 1.0;
            float displacement = 0.0;

          #ifdef OE_HAS_PBR_TEX
            vec4 pbr = texture(oe_pbr_tex, uv);
            displacement = pbr.r;
          #endif

          #ifdef OE_HAS_ALBEDO_TEX
            const float ground_height = 0.0;
            vec4 albedo = texture(oe_albedo_tex, uv);
            float albedo_blend = 1.0 - color.r;
            unquantized_blend = heightAndEffectMix(ground_height, 1.0, displacement, albedo_blend) * oe_layer_opacity;
            quantized_blend = smoothstep(0.5, 0.6, unquantized_blend);
            color = vec4(albedo.rgb, quantized_blend);
          #endif

          #ifdef OE_HAS_NORMAL_TEX
            vec4 n = texture(oe_normal_tex, uv);
            vec3 normal = n.xyz*2.0 - 1.0;
            normal.z *= 1.0 / oe_normal_boost;
            vp_Normal = normalize(mix(vp_Normal, oe_normalMapTBN * normal, quantized_blend));
          #endif

          #ifdef OE_HAS_PBR_TEX
            oe_pbr.displacement = mix(oe_pbr.displacement, displacement, quantized_blend);
            oe_pbr.roughness = mix(oe_pbr.roughness, pbr.g, quantized_blend);
            oe_pbr.ao = mix(0.0, pbr.b, unquantized_blend);
            oe_pbr.metal = mix(oe_pbr.metal, pbr.a, quantized_blend);
          #endif
        }
    )";
}



void
FeatureSplattingLayer::init()
{
    super::init();
    setRenderType(RENDERTYPE_TERRAIN_SURFACE);
}

void
FeatureSplattingLayer::prepareForRendering(TerrainEngine* engine)
{
    super::prepareForRendering(engine);

    if (Capabilities::get().supportsInt64() == false)
    {
        setStatus(Status::ResourceUnavailable, "GLSL int64 support required but not available");
        OE_WARN << LC << getStatus().message() << std::endl;
        return;
    }

    if (Capabilities::get().getGLSLVersion() < 4.6f || Capabilities::get().supportsNVGL() == false)
    {
        setStatus(Status::ResourceUnavailable, "GLSL 4.6+ required but not available");
        OE_WARN << LC << getStatus().message() << std::endl;
        return;
    }

    //TODO error check these.
    engine->getResources()->reserveTextureImageUnitForLayer(_res_albedo, this);
    engine->getResources()->reserveTextureImageUnitForLayer(_res_normal, this);
    engine->getResources()->reserveTextureImageUnitForLayer(_res_pbr, this);

    buildStateSet();
}

void
FeatureSplattingLayer::buildStateSet()
{
    // fire up the shaders:
    auto* ss = getOrCreateStateSet();
    auto vp = VirtualProgram::getOrCreate(ss);
    vp->setFunction("oe_feature_splatting_vs", vs, ShaderComp::LOCATION_VERTEX_VIEW);
    vp->setFunction("oe_feature_splatting_fs", fs, ShaderComp::LOCATION_FRAGMENT_COLORING);

    // load up the PBR material:
    osg::ref_ptr<PBRTexture> tex = new PBRTexture();
    auto load_status = tex->load(options().material().value(), getReadOptions());
    if (load_status.isError())
    {
        setStatus(load_status);
        return;
    }

    tex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
    tex->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);

    if (tex->albedo.valid())
    {
        ss->setTextureAttribute(_res_albedo.unit(), tex->albedo);
        ss->addUniform(new osg::Uniform("oe_albedo_tex", _res_albedo.unit()));
        ss->setDefine("OE_HAS_ALBEDO_TEX");
    }

    if (tex->normal.valid())
    {
        ss->setTextureAttribute(_res_normal.unit(), tex->normal);
        ss->addUniform(new osg::Uniform("oe_normal_tex", _res_normal.unit()));
        ss->setDefine("OE_HAS_NORMAL_TEX");
    }

    if (tex->pbr.valid())
    {
        ss->setTextureAttribute(_res_pbr.unit(), tex->pbr);
        ss->addUniform(new osg::Uniform("oe_pbr_tex", _res_pbr.unit()));
        ss->setDefine("OE_HAS_PBR_TEX");
    }
}
