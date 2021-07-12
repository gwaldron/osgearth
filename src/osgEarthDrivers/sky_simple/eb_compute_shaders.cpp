
clear_2d_cs = 
    constants + R"(
//#include <constants.glsl>

// ------------------------------------------------------------------
// INPUTS -----------------------------------------------------------
// ------------------------------------------------------------------

layout (local_size_x = LOCAL_SIZE, local_size_y = LOCAL_SIZE, local_size_z = LOCAL_SIZE) in;

// ------------------------------------------------------------------
// UNIFORMS ---------------------------------------------------------
// ------------------------------------------------------------------

layout (binding = 0, INTERNAL_FORMAT) uniform image2D targetImage;

// ------------------------------------------------------------------
// MAIN -------------------------------------------------------------
// ------------------------------------------------------------------

void main()
{
    ivec2 uv = ivec2(gl_GlobalInvocationID.xy);
    imageStore(targetImage, uv, vec4(0.0, 0.0, 0.0, 0.0));
}

// ------------------------------------------------------------------
)";

clear_3d_cs =
    constants + R"(
//#include <constants.glsl>

// ------------------------------------------------------------------
// INPUTS -----------------------------------------------------------
// ------------------------------------------------------------------

layout (local_size_x = LOCAL_SIZE, local_size_y = LOCAL_SIZE, local_size_z = LOCAL_SIZE) in;

// ------------------------------------------------------------------
// UNIFORMS ---------------------------------------------------------
// ------------------------------------------------------------------

layout (binding = 0, INTERNAL_FORMAT) uniform image3D targetImage;

// ------------------------------------------------------------------
// MAIN -------------------------------------------------------------
// ------------------------------------------------------------------

void main()
{
    ivec3 uv = ivec3(gl_GlobalInvocationID);
    imageStore(targetImage, uv, vec4(0.0, 0.0, 0.0, 0.0));
}

// ------------------------------------------------------------------
)";

compute_direct_irradiance_cs = 
    constants +
    uniforms +
    utility +
    transmittance_functions +
    scattering_functions +
    irradiance_functions + R"(
//#include <constants.glsl>
//#include <uniforms.glsl>
//#include <utility.glsl>
//#include <transmittance_functions.glsl>
//#include <scattering_functions.glsl>
//#include <irradiance_functions.glsl>

// ------------------------------------------------------------------
// INPUTS -----------------------------------------------------------
// ------------------------------------------------------------------

layout (local_size_x = LOCAL_SIZE, local_size_y = LOCAL_SIZE, local_size_z = 1) in;

// ------------------------------------------------------------------
// INPUT ------------------------------------------------------------
// ------------------------------------------------------------------

layout (binding = 0, INTERNAL_FORMAT) uniform image2D irradiance_read;
layout (binding = 1, INTERNAL_FORMAT) uniform image2D irradiance_write;
layout (binding = 2, INTERNAL_FORMAT) uniform image2D delta_irradiance;

// ------------------------------------------------------------------
// UNIFORMS ---------------------------------------------------------
// ------------------------------------------------------------------

uniform vec4 blend;

uniform sampler2D transmittance;

// ------------------------------------------------------------------
// MAIN -------------------------------------------------------------
// ------------------------------------------------------------------

void main()
{
    ivec2 coord = ivec2(gl_GlobalInvocationID.xy);
    vec2 frag_coord = coord + vec2(0.5, 0.5);
    
    imageStore(delta_irradiance, coord, vec4(ComputeDirectIrradianceTexture(transmittance, frag_coord), 1.0));
    imageStore(irradiance_write, coord, vec4(0.0, 0.0, 0.0, 1.0));

    if(blend[1] == 1)
      imageStore(irradiance_write, coord, imageLoad(irradiance_write, coord) + imageLoad(irradiance_read, coord));
}

// ------------------------------------------------------------------
)";

compute_indirect_irradiance_cs =
    constants +
    uniforms +
    utility +
    transmittance_functions +
    scattering_functions +
    irradiance_functions + R"(
//#include <constants.glsl>
//#include <uniforms.glsl>
//#include <utility.glsl>
//#include <transmittance_functions.glsl>
//#include <scattering_functions.glsl>
//#include <irradiance_functions.glsl>

// ------------------------------------------------------------------
// INPUTS -----------------------------------------------------------
// ------------------------------------------------------------------

layout (local_size_x = LOCAL_SIZE, local_size_y = LOCAL_SIZE, local_size_z = 1) in;

// ------------------------------------------------------------------
// IMAGES -----------------------------------------------------------
// ------------------------------------------------------------------

layout (binding = 0, INTERNAL_FORMAT) uniform image2D delta_irradiance;
layout (binding = 1, INTERNAL_FORMAT) uniform image2D irradiance_read;
layout (binding = 2, INTERNAL_FORMAT) uniform image2D irradiance_write;

// ------------------------------------------------------------------
// UNIFORMS ---------------------------------------------------------
// ------------------------------------------------------------------

uniform vec4 blend;
uniform int scattering_order;

uniform sampler3D single_rayleigh_scattering;
uniform sampler3D single_mie_scattering;
uniform sampler3D multiple_scattering;

// ------------------------------------------------------------------
// MAIN -------------------------------------------------------------
// ------------------------------------------------------------------

void main()
{
    ivec2 coord = ivec2(gl_GlobalInvocationID.xy);
    vec2 frag_coord = coord + vec2(0.5, 0.5);

    vec3 delta_irradiance_value = ComputeIndirectIrradianceTexture(single_rayleigh_scattering, single_mie_scattering, multiple_scattering, frag_coord, scattering_order);

    vec3 irradiance = RadianceToLuminance(delta_irradiance_value);

    imageStore(delta_irradiance, coord, vec4(delta_irradiance_value, 1.0));

    imageStore(irradiance_write, coord, vec4(irradiance, 1.0));

    if(blend[1] == 1)
        imageStore(irradiance_write, coord, imageLoad(irradiance_write, coord) + imageLoad(irradiance_read, coord));
}

// ------------------------------------------------------------------
)";

compute_multiple_scattering_cs =
    constants +
    uniforms +
    utility +
    transmittance_functions +
    scattering_functions +
    irradiance_functions + R"(
//#include <constants.glsl>
//#include <uniforms.glsl>
//#include <utility.glsl>
//#include <transmittance_functions.glsl>
//#include <scattering_functions.glsl>
//#include <irradiance_functions.glsl>

// ------------------------------------------------------------------
// INPUTS -----------------------------------------------------------
// ------------------------------------------------------------------

layout (local_size_x = LOCAL_SIZE, local_size_y = LOCAL_SIZE, local_size_z = 1) in;

// ------------------------------------------------------------------
// IMAGES -----------------------------------------------------------
// ------------------------------------------------------------------

layout (binding = 0, INTERNAL_FORMAT) uniform image3D delta_multiple_scattering;
layout (binding = 1, INTERNAL_FORMAT) uniform image3D scattering_read;
layout (binding = 2, INTERNAL_FORMAT) uniform image3D scattering_write;

// ------------------------------------------------------------------
// UNIFORMS ---------------------------------------------------------
// ------------------------------------------------------------------

uniform vec4 blend;
uniform int layer;

uniform sampler2D transmittance;
uniform sampler3D delta_scattering_density;

// ------------------------------------------------------------------
// MAIN -------------------------------------------------------------
// ------------------------------------------------------------------

void main()
{
    ivec3 coord = ivec3(gl_GlobalInvocationID);
    coord.z = layer;

    vec3 frag_coord = coord + vec3(0.5, 0.5, 0.5);

    float nu;
    vec3 delta_multiple_scattering_value = ComputeMultipleScatteringTexture(transmittance, delta_scattering_density, frag_coord, nu);

    imageStore(delta_multiple_scattering, coord, vec4(delta_multiple_scattering_value, 1.0));

    imageStore(scattering_write, coord, vec4(RadianceToLuminance(delta_multiple_scattering_value) / RayleighPhaseFunction(nu), 0.0));

    if(blend[1] == 1)
        imageStore(scattering_write, coord, imageLoad(scattering_write, coord) + imageLoad(scattering_read, coord));
}

// ------------------------------------------------------------------
)";

compute_scattering_density_cs =
    constants +
    uniforms +
    utility +
    transmittance_functions +
    scattering_functions +
    irradiance_functions + R"(
//#include <constants.glsl>
//#include <uniforms.glsl>
//#include <utility.glsl>
//#include <transmittance_functions.glsl>
//#include <scattering_functions.glsl>
//#include <irradiance_functions.glsl>

// ------------------------------------------------------------------
// INPUTS -----------------------------------------------------------
// ------------------------------------------------------------------

layout (local_size_x = LOCAL_SIZE, local_size_y = LOCAL_SIZE, local_size_z = 1) in;

// ------------------------------------------------------------------
// IMAGES -----------------------------------------------------------
// ------------------------------------------------------------------

layout (binding = 0, INTERNAL_FORMAT) uniform image3D delta_scattering_density;

// ------------------------------------------------------------------
// UNIFORMS ---------------------------------------------------------
// ------------------------------------------------------------------

uniform vec4 blend;
uniform int layer;
uniform int scattering_order;

uniform sampler2D transmittance;
uniform sampler3D single_rayleigh_scattering;
uniform sampler3D single_mie_scattering;
uniform sampler3D multiple_scattering;
uniform sampler2D irradiance;

// ------------------------------------------------------------------
// MAIN -------------------------------------------------------------
// ------------------------------------------------------------------

void main()
{
    ivec3 coord = ivec3(gl_GlobalInvocationID);
    coord.z = layer;
    vec3 frag_coord = coord + vec3(0.5,0.5,0.5);

    vec3 scattering_density = ComputeScatteringDensityTexture(transmittance, single_rayleigh_scattering, single_mie_scattering, multiple_scattering, irradiance, frag_coord, scattering_order);

    imageStore(delta_scattering_density, coord, vec4(scattering_density, 1.0));
}

// ------------------------------------------------------------------
)";

compute_single_scattering_cs =
    constants +
    uniforms +
    utility +
    transmittance_functions +
    scattering_functions +
    irradiance_functions + R"(
//#include <constants.glsl>
//#include <uniforms.glsl>
//#include <utility.glsl>
//#include <transmittance_functions.glsl>
//#include <scattering_functions.glsl>
//#include <irradiance_functions.glsl>

// ------------------------------------------------------------------
// INPUTS -----------------------------------------------------------
// ------------------------------------------------------------------

layout (local_size_x = LOCAL_SIZE, local_size_y = LOCAL_SIZE, local_size_z = 1) in;

// ------------------------------------------------------------------
// IMAGES -----------------------------------------------------------
// ------------------------------------------------------------------

layout (binding = 0, INTERNAL_FORMAT) uniform image3D delta_rayleigh_scattering;
layout (binding = 1, INTERNAL_FORMAT) uniform image3D delta_mie_scattering;
layout (binding = 2, INTERNAL_FORMAT) uniform image3D scattering_read;
layout (binding = 3, INTERNAL_FORMAT) uniform image3D scattering_write;
layout (binding = 4, INTERNAL_FORMAT) uniform image3D single_mie_scattering_read;
layout (binding = 5, INTERNAL_FORMAT) uniform image3D single_mie_scattering_write;

// ------------------------------------------------------------------
// UNIFORMS ---------------------------------------------------------
// ------------------------------------------------------------------

uniform vec4 blend;
uniform int layer;
uniform sampler2D transmittance;

// ------------------------------------------------------------------
// MAIN -------------------------------------------------------------
// ------------------------------------------------------------------

void main()
{
    ivec3 coord = ivec3(gl_GlobalInvocationID);
    coord.z = layer;
    vec3 frag_coord = coord + vec3(0.5,0.5,0.5);

    vec3 delta_rayleigh, delta_mie;
    ComputeSingleScatteringTexture(transmittance, frag_coord, delta_rayleigh, delta_mie);

    imageStore(delta_rayleigh_scattering, coord, vec4(delta_rayleigh, 1));

    imageStore(delta_mie_scattering, coord, vec4(delta_mie, 1));

    imageStore(scattering_write, coord, vec4(RadianceToLuminance(delta_rayleigh), RadianceToLuminance(delta_mie).r));

    imageStore(single_mie_scattering_write, coord, vec4(RadianceToLuminance(delta_mie), 1));

    if(blend[2] == 1)
        imageStore(scattering_write, coord, imageLoad(scattering_write, coord) + imageLoad(scattering_read, coord));

    if(blend[3] == 1)
        imageStore(single_mie_scattering_write, coord, imageLoad(single_mie_scattering_write, coord) + imageLoad(single_mie_scattering_read, coord));
}

// ------------------------------------------------------------------
)";

compute_transmittance_cs =
    constants +
    uniforms +
    utility +
    transmittance_functions + R"(
//#include <constants.glsl>
//#include <uniforms.glsl>
//#include <utility.glsl>
//#include <transmittance_functions.glsl>

// ------------------------------------------------------------------
// INPUTS -----------------------------------------------------------
// ------------------------------------------------------------------

layout (local_size_x = LOCAL_SIZE, local_size_y = LOCAL_SIZE, local_size_z = 1) in;

// ------------------------------------------------------------------
// UNIFORMS ---------------------------------------------------------
// ------------------------------------------------------------------

layout (binding = 0, INTERNAL_FORMAT) uniform image2D transmittance;

// ------------------------------------------------------------------
// MAIN -------------------------------------------------------------
// ------------------------------------------------------------------

void main()
{
    ivec2 coord = ivec2(gl_GlobalInvocationID.xy);
    vec2 frag_coord = coord + vec2(0.5, 0.5);
    imageStore(transmittance, coord, vec4(ComputeTransmittanceToTopAtmosphereBoundaryTexture(frag_coord), 1.0));
}

// ------------------------------------------------------------------
)";
