    constants = R"(
// ------------------------------------------------------------------
// DEFINITIONS ------------------------------------------------------
// ------------------------------------------------------------------

#define LOCAL_SIZE 8
)";


    uniforms = R"(
/**
 * Copyright (c) 2017 Eric Bruneton
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/* This GLSL file defines the physical types and constants which are used in the
 * main functions of our atmosphere model.
 *
 * Physical quantities
 *
 * The physical quantities we need for our atmosphere model are radiometric and photometric quantities.
 * 
 * https://en.wikipedia.org/wiki/Radiometry
 * https://en.wikipedia.org/wiki/Photometry_(optics)
 *
 * We start with six base quantities: length, wavelength, angle, solid angle,
 * power and luminous power (wavelength is also a length, but we distinguish the
 * two for increased clarity).
 *
 */

#define Length float
#define Wavelength float
#define Angle float
#define SolidAngle float
#define Power float
#define LuminousPower float

/*
 * From this we "derive" the irradiance, radiance, spectral irradiance,
 * spectral radiance, luminance, etc, as well pure numbers, area, volume.
 */

#define Number float
#define InverseLength float
#define Area float
#define Volume float
#define NumberDensity float
#define Irradiance float
#define Radiance float
#define SpectralPower float
#define SpectralIrradiance float
#define SpectralRadiance float
#define SpectralRadianceDensity float
#define ScatteringCoefficient float
#define InverseSolidAngle float
#define LuminousIntensity float
#define Luminance float
#define Illuminance float

/*
 * We  also need vectors of physical quantities, mostly to represent functions
 * depending on the wavelength. In this case the vector elements correspond to
 * values of a function at some predefined wavelengths.
 */

// A generic function from Wavelength to some other type.
#define AbstractSpectrum vec3
// A function from Wavelength to Number.
#define DimensionlessSpectrum vec3
// A function from Wavelength to SpectralPower.
#define PowerSpectrum vec3
// A function from Wavelength to SpectralIrradiance.
#define IrradianceSpectrum vec3
// A function from Wavelength to SpectralRadiance.
#define RadianceSpectrum vec3
// A function from Wavelength to SpectralRadianceDensity.
#define RadianceDensitySpectrum vec3
// A function from Wavelength to ScaterringCoefficient.
#define ScatteringSpectrum vec3

// A position in 3D (3 length values).
#define Position vec3
// A unit direction vector in 3D (3 unitless values).
#define Direction vec3
// A vector of 3 luminance values.
#define Luminance3 vec3
// A vector of 3 illuminance values.
#define Illuminance3 vec3

/*
 * Finally, we also need precomputed textures containing physical quantities in each texel.
 */

#define TransmittanceTexture sampler2D
#define AbstractScatteringTexture sampler3D
#define ReducedScatteringTexture sampler3D
#define ScatteringTexture sampler3D
#define ScatteringDensityTexture sampler3D
#define IrradianceTexture sampler2D

/*
 * Physical units
 *
 * We can then define the units for our six base physical quantities:
 * meter (m), nanometer (nm), radian (rad), steradian (sr), watt (watt) and lumen (lm):
 */

const Length m = 1.0;
const Wavelength nm = 1.0;
const Angle rad = 1.0;
const SolidAngle sr = 1.0;
const Power watt = 1.0;
const LuminousPower lm = 1.0;

/*
 * From which we can derive the units for some derived physical quantities,
 * as well as some derived units (kilometer km, kilocandela kcd, degree deg):
 */

const float PI = 3.14159265358979323846;

const Length km = 1000.0 * m;
const Area m2 = m * m;
const Volume m3 = m * m * m;
const Angle pi = PI * rad;
const Angle deg = pi / 180.0;
const Irradiance watt_per_square_meter = watt / m2;
const Radiance watt_per_square_meter_per_sr = watt / (m2 * sr);
const SpectralIrradiance watt_per_square_meter_per_nm = watt / (m2 * nm);
const SpectralRadiance watt_per_square_meter_per_sr_per_nm = watt / (m2 * sr * nm);
const SpectralRadianceDensity watt_per_cubic_meter_per_sr_per_nm = watt / (m3 * sr * nm);
const LuminousIntensity cd = lm / sr;
const LuminousIntensity kcd = 1000.0 * cd;
const Luminance cd_per_square_meter = cd / m2;
const Luminance kcd_per_square_meter = kcd / m2;

/*
 * Atmosphere parameters
 *
 * Using the above types, we can now define the parameters of our atmosphere
 * model. We start with the definition of density profiles, which are needed for
 * parameters that depend on the altitude:
 */

uniform int TRANSMITTANCE_TEXTURE_WIDTH; 
uniform int TRANSMITTANCE_TEXTURE_HEIGHT; 

uniform int SCATTERING_TEXTURE_R_SIZE; 
uniform int SCATTERING_TEXTURE_MU_SIZE; 
uniform int SCATTERING_TEXTURE_MU_S_SIZE; 
uniform int SCATTERING_TEXTURE_NU_SIZE; 

uniform int SCATTERING_TEXTURE_WIDTH; 
uniform int SCATTERING_TEXTURE_HEIGHT; 
uniform int SCATTERING_TEXTURE_DEPTH; 

uniform int IRRADIANCE_TEXTURE_WIDTH; 
uniform int IRRADIANCE_TEXTURE_HEIGHT; 

uniform vec3 SKY_SPECTRAL_RADIANCE_TO_LUMINANCE; 
uniform vec3 SUN_SPECTRAL_RADIANCE_TO_LUMINANCE; 

uniform mat4 luminanceFromRadiance; 

// The density profile of air molecules, i.e. a function from altitude to
// dimensionless values between 0 (null density) and 1 (maximum density).
uniform Length rayleigh_width;
uniform Number rayleigh_exp_term;
uniform InverseLength rayleigh_exp_scale;
uniform InverseLength rayleigh_linear_term;
uniform Number rayleigh_constant_term;

// The scattering coefficient of air molecules at the altitude where their
// density is maximum (usually the bottom of the atmosphere), as a function of
// wavelength. The scattering coefficient at altitude h is equal to
// 'rayleigh_scattering' times 'rayleigh_density' at this altitude.
uniform ScatteringSpectrum rayleigh_scattering; 

// The density profile of aerosols, i.e. a function from altitude to
// dimensionless values between 0 (null density) and 1 (maximum density).
uniform Length mie_width;
uniform Number mie_exp_term;
uniform InverseLength mie_exp_scale;
uniform InverseLength mie_linear_term;
uniform Number mie_constant_term;

// The scattering coefficient of aerosols at the altitude where their density
// is maximum (usually the bottom of the atmosphere), as a function of
// wavelength. The scattering coefficient at altitude h is equal to
// 'mie_scattering' times 'mie_density' at this altitude.
uniform ScatteringSpectrum mie_scattering; 

// The density profile of air molecules that absorb light (e.g. ozone), i.e.
// a function from altitude to dimensionless values between 0 (null density)
// and 1 (maximum density).
uniform Length absorption0_width;
uniform Number absorption0_exp_term;
uniform InverseLength absorption0_exp_scale;
uniform InverseLength absorption0_linear_term;
uniform Number absorption0_constant_term;

uniform Length absorption1_width;
uniform Number absorption1_exp_term;
uniform InverseLength absorption1_exp_scale;
uniform InverseLength absorption1_linear_term;
uniform Number absorption1_constant_term;

// The extinction coefficient of molecules that absorb light (e.g. ozone) at
// the altitude where their density is maximum, as a function of wavelength.
// The extinction coefficient at altitude h is equal to
// 'absorption_extinction' times 'absorption_density' at this altitude.
uniform ScatteringSpectrum absorption_extinction; 

// The solar irradiance at the top of the atmosphere.
uniform IrradianceSpectrum solar_irradiance; 

// The sun's angular radius. Warning: the implementation uses approximations
// that are valid only if this angle is smaller than 0.1 radians.
uniform Angle sun_angular_radius; 

// The distance between the planet center and the bottom of the atmosphere.
uniform Length bottom_radius; 

// The distance between the planet center and the top of the atmosphere.
uniform Length top_radius; 

// The extinction coefficient of aerosols at the altitude where their density
// is maximum (usually the bottom of the atmosphere), as a function of
// wavelength. The extinction coefficient at altitude h is equal to
// 'mie_extinction' times 'mie_density' at this altitude.
uniform ScatteringSpectrum mie_extinction; 

// The asymetry parameter for the Cornette-Shanks phase function for the aerosols.
uniform Number mie_phase_function_g; 

// The average albedo of the ground.
uniform DimensionlessSpectrum ground_albedo; 

// The cosine of the maximum Sun zenith angle for which atmospheric scattering
// must be precomputed (for maximum precision, use the smallest Sun zenith
// angle yielding negligible sky light radiance values. For instance, for the
// Earth case, 102 degrees is a good choice - yielding mu_s_min = -0.2).
uniform Number mu_s_min; 

// An atmosphere layer of width 'width', and whose density is defined as
// 'exp_term' * exp('exp_scale' * h) + 'linear_term' * h + 'constant_term',
// clamped to [0,1], and where h is the altitude.
struct DensityProfileLayer 
{
  Length width;
  Number exp_term;
  InverseLength exp_scale;
  InverseLength linear_term;
  Number constant_term;
};

// An atmosphere density profile made of several layers on top of each other
// (from bottom to top). The width of the last layer is ignored, i.e. it always
// extend to the top atmosphere boundary. The profile values vary between 0
// (null density) to 1 (maximum density).
struct DensityProfile 
{
   DensityProfileLayer layers[2];
};

DensityProfile RayleighDensity()
{
  DensityProfileLayer layer;

  layer.width = rayleigh_width;
  layer.exp_term = rayleigh_exp_term;
  layer.exp_scale = rayleigh_exp_scale;
  layer.linear_term = rayleigh_linear_term;
  layer.constant_term = rayleigh_constant_term;

  DensityProfile profile;
  profile.layers[0] = layer;
  profile.layers[1] = layer;

  return profile;
}

DensityProfile MieDensity()
{
  DensityProfileLayer layer;

  layer.width = mie_width;
  layer.exp_term = mie_exp_term;
  layer.exp_scale = mie_exp_scale;
  layer.linear_term = mie_linear_term;
  layer.constant_term = mie_constant_term;

  DensityProfile profile;
  profile.layers[0] = layer;
  profile.layers[1] = layer;

  return profile;
}

DensityProfile AbsorptionDensity()
{
  DensityProfileLayer layer0;

  layer0.width = absorption0_width;
  layer0.exp_term = absorption0_exp_term;
  layer0.exp_scale = absorption0_exp_scale;
  layer0.linear_term = absorption0_linear_term;
  layer0.constant_term = absorption0_constant_term;

  DensityProfileLayer layer1;

  layer1.width = absorption1_width;
  layer1.exp_term = absorption1_exp_term;
  layer1.exp_scale = absorption1_exp_scale;
  layer1.linear_term = absorption1_linear_term;
  layer1.constant_term = absorption1_constant_term;

  DensityProfile profile;
  profile.layers[0] = layer0;
  profile.layers[1] = layer1;

  return profile;
}
)";


    utility = R"(
/**
 * Copyright (c) 2017 Eric Bruneton
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Precomputed Atmospheric Scattering
 * Copyright (c) 2008 INRIA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */


/*
 * Utility functions to avoid NaNs due to floating point
 * values slightly outside their theoretical bounds:
 */

Number ClampCosine(Number mu) 
{
  return clamp(mu, Number(-1.0), Number(1.0));
}

Length ClampDistance(Length d) 
{
  return max(d, 0.0 * m);
}

Length ClampRadius(Length r) 
{
  return clamp(r, bottom_radius, top_radius);
}

Length SafeSqrt(Area a) 
{
  return sqrt(max(a, 0.0 * m2));
}

Number mod(Number x, Number y)
{
	return x - y * floor(x / y);
}

vec3 RadianceToLuminance(vec3 rad)
{
  return (luminanceFromRadiance * vec4(rad, 1.0)).rgb;
}

#define TEX2D(tex, uv) texture(tex, uv)
#define TEX3D(tex, uv) texture(tex, uv)
)";



irradiance_functions = R"(
/**
 * Copyright (c) 2017 Eric Bruneton
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Precomputed Atmospheric Scattering
 * Copyright (c) 2008 INRIA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */
)" R"(
/*
 * Ground irradiance
 * 
 * The ground irradiance is the Sun light received on the ground after n \ge 0
 * bounces (where a bounce is either a scattering event or a reflection on the
 * ground). We need this for two purposes:
 * 
 * while precomputing the n-th order of scattering, with n \ge 2, in order
 * to compute the contribution of light paths whose (n-1)-th bounce is on the
 * ground (which requires the ground irradiance after n-2 bounces - see the Multiple
 * scattering section),
 * at rendering time, to compute the contribution of light paths whose last
 * bounce is on the ground (these paths are excluded, by definition, from our
 * precomputed scattering textures)
 * 
 * In the first case we only need the ground irradiance for horizontal surfaces
 * at the bottom of the atmosphere (during precomputations we assume a perfectly
 * spherical ground with a uniform albedo). In the second case, however, we need
 * the ground irradiance for any altitude and any surface normal, and we want to
 * precompute it for efficiency. In fact, as described in our
 * <a href="https://hal.inria.fr/inria-00288758/en">paper</a> we precompute it only
 * for horizontal surfaces, at any altitude (which requires only 2D textures,
 * instead of 4D textures for the general case), and we use approximations for
 * non-horizontal surfaces.
 * 
 * The following sections describe how we compute the ground irradiance, how we
 * store it in a precomputed texture, and how we read it back.
 * 
 * Computation
 * 
 * The ground irradiance computation is different for the direct irradiance,
 * i.e. the light received directly from the Sun, without any intermediate bounce,
 * and for the indirect irradiance (at least one bounce). We start here with the
 * direct irradiance.
 * 
 * The irradiance is the integral over an hemisphere of the incident radiance,
 * times a cosine factor. For the direct ground irradiance, the incident radiance
 * is the Sun radiance at the top of the atmosphere, times the transmittance
 * through the atmosphere. And, since the Sun solid angle is small, we can
 * approximate the transmittance with a constant, i.e. we can move it outside the
 * irradiance integral, which can be performed over (the visible fraction of) the
 * Sun disc rather than the hemisphere. Then the integral becomes equivalent to the
 * ambient occlusion due to a sphere, also called a view factor, which is given in
 * <a href="http://webserver.dmt.upm.es/~isidoro/tc3/Radiation%20View%20factors.pdf
 * ">Radiative view factors</a> (page 10). For a small solid angle, these complex
 * equations can be simplified as follows:
 */

IrradianceSpectrum ComputeDirectIrradiance(
    TransmittanceTexture transmittance_texture,
    Length r, Number mu_s) 
{

  Number alpha_s = sun_angular_radius / rad;

  // Approximate average of the cosine factor mu_s over the visible fraction of
  // the Sun disc.
  Number average_cosine_factor =
    mu_s < -alpha_s ? 0.0 : (mu_s > alpha_s ? mu_s :
        (mu_s + alpha_s) * (mu_s + alpha_s) / (4.0 * alpha_s));

  return solar_irradiance * GetTransmittanceToTopAtmosphereBoundary(transmittance_texture, r, mu_s) * average_cosine_factor;

}

)" R"(

/*
 * For the indirect ground irradiance the integral over the hemisphere must be
 * computed numerically. More precisely we need to compute the integral over all
 * the directions w of the hemisphere, of the product of:
 * 
 * the radiance arriving from direction w after n bounces,
 * the cosine factor, i.e. omega_z
 * 
 * This leads to the following implementation (where
 * multiple_scattering_texture is supposed to contain the n-th
 * order of scattering, if n>1, and scattering_order is equal to n):
 */

IrradianceSpectrum ComputeIndirectIrradiance(
    ReducedScatteringTexture single_rayleigh_scattering_texture,
    ReducedScatteringTexture single_mie_scattering_texture,
    ScatteringTexture multiple_scattering_texture,
    Length r, Number mu_s, int scattering_order) 
{

  const int SAMPLE_COUNT = 32;
  const Angle dphi = pi / Number(SAMPLE_COUNT);
  const Angle dtheta = pi / Number(SAMPLE_COUNT);

  IrradianceSpectrum result = IrradianceSpectrum(0, 0, 0);

  vec3 omega_s = vec3(sqrt(1.0 - mu_s * mu_s), 0.0, mu_s);

  for (int j = 0; j < SAMPLE_COUNT / 2; ++j) 
  {
    Angle theta = (Number(j) + 0.5) * dtheta;

    for (int i = 0; i < 2 * SAMPLE_COUNT; ++i) 
    {
      Angle phi = (Number(i) + 0.5) * dphi;
      vec3 omega = vec3(cos(phi) * sin(theta), sin(phi) * sin(theta), cos(theta));
      SolidAngle domega = (dtheta / rad) * (dphi / rad) * sin(theta) * sr;
      Number nu = dot(omega, omega_s);

      result += GetScattering(single_rayleigh_scattering_texture,
                single_mie_scattering_texture, multiple_scattering_texture,
                r, omega.z, mu_s, nu, false,scattering_order) * omega.z * domega;
    }
  }

  return result;
}
)" R"(

/*
 * Irradiance Precomputation
 * 
 * In order to precompute the ground irradiance in a texture we need a mapping
 * from the ground irradiance parameters to texture coordinates. Since we
 * precompute the ground irradiance only for horizontal surfaces, this irradiance
 * depends only on r and mu_s, so we need a mapping from (r,mu_s) to
 * (u,v) texture coordinates. The simplest, affine mapping is sufficient here,
 * because the ground irradiance function is very smooth:
 */

vec2 GetIrradianceTextureUvFromRMuS(Length r, Number mu_s) 
{

  Number x_r = (r - bottom_radius) / (top_radius - bottom_radius);
  Number x_mu_s = mu_s * 0.5 + 0.5;
  return vec2(GetTextureCoordFromUnitRange(x_mu_s, IRRADIANCE_TEXTURE_WIDTH),
                GetTextureCoordFromUnitRange(x_r, IRRADIANCE_TEXTURE_HEIGHT));
}

/*
 * The inverse mapping follows immediately:
 */

void GetRMuSFromIrradianceTextureUv(vec2 uv, out Length r, out Number mu_s) 
{

  Number x_mu_s = GetUnitRangeFromTextureCoord(uv.x, IRRADIANCE_TEXTURE_WIDTH);
  Number x_r = GetUnitRangeFromTextureCoord(uv.y, IRRADIANCE_TEXTURE_HEIGHT);
  r = bottom_radius + x_r * (top_radius - bottom_radius);
  mu_s = ClampCosine(2.0 * x_mu_s - 1.0);
}

/*
 * It is now easy to define a fragment shader function to precompute a texel of
 * the ground irradiance texture, for the direct irradiance:
 */

IrradianceSpectrum ComputeDirectIrradianceTexture(
    TransmittanceTexture transmittance_texture,
    vec2 gl_frag_coord) 
{
  Length r;
  Number mu_s;

  const vec2 IRRADIANCE_TEXTURE_SIZE = vec2(IRRADIANCE_TEXTURE_WIDTH, IRRADIANCE_TEXTURE_HEIGHT);

  GetRMuSFromIrradianceTextureUv(gl_frag_coord / IRRADIANCE_TEXTURE_SIZE, r, mu_s);
  return ComputeDirectIrradiance(transmittance_texture, r, mu_s);
}

/*
 * and the indirect one:
 */

IrradianceSpectrum ComputeIndirectIrradianceTexture(
    ReducedScatteringTexture single_rayleigh_scattering_texture,
    ReducedScatteringTexture single_mie_scattering_texture,
    ScatteringTexture multiple_scattering_texture,
    vec2 gl_frag_coord, int scattering_order) 
{
  Length r;
  Number mu_s;

  const vec2 IRRADIANCE_TEXTURE_SIZE = vec2(IRRADIANCE_TEXTURE_WIDTH, IRRADIANCE_TEXTURE_HEIGHT);

  GetRMuSFromIrradianceTextureUv(gl_frag_coord / IRRADIANCE_TEXTURE_SIZE, r, mu_s);

  return ComputeIndirectIrradiance(single_rayleigh_scattering_texture, single_mie_scattering_texture,
                                   multiple_scattering_texture, r, mu_s, scattering_order);
}
)" R"(

/*
 * Irradiance Lookup
 * 
 * Thanks to these precomputed textures, we can now get the ground irradiance
 * with a single texture lookup:
 */

IrradianceSpectrum GetIrradiance(
    IrradianceTexture irradiance_texture,
    Length r, Number mu_s) 
{
  vec2 uv = GetIrradianceTextureUvFromRMuS(r, mu_s);
  return TEX2D(irradiance_texture, uv).rgb;
}
)";


transmittance_functions = R"(
/**
 * Copyright (c) 2017 Eric Bruneton
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Precomputed Atmospheric Scattering
 * Copyright (c) 2008 INRIA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

)" R"(

/*
 * Transmittance
 *
 * https://en.wikipedia.org/wiki/Transmittance
 *
 * As the light travels from a point p to a point q in the atmosphere,
 * it is partially absorbed and scattered out of its initial direction because of
 * the air molecules and the aerosol particles. Thus, the light arriving at q
 * is only a fraction of the light from p, and this fraction, which depends on
 * wavelength, is called the transmittance. The following sections describe how 
 * we compute it, how we store it in a precomputed
 * texture, and how we read it back.
 *
 * Computation
 *
 * For 3 aligned points p, q and r inside the atmosphere, in this
 * order, the transmittance between p and r is the product of the
 * transmittance between p and q and between q and r.
 * In particular, the transmittance between p and q is the transmittance
 * between p and the nearest intersection i of the half-line [p,q]
 * with the top or bottom atmosphere boundary, divided by the transmittance between
 * q and i (or 0 if the segment [p,q] intersects the ground).
 *
 * Also, the transmittance between p and q and between q and p
 * are the same. Thus, to compute the transmittance between arbitrary points, it
 * is sufficient to know the transmittance between a point p in the atmosphere,
 * and points i on the top atmosphere boundary. This transmittance depends on
 * only two parameters, which can be taken as the radius $r=\Vert\bo\bp\Vert$ and
 * the cosine of the "view zenith angle",
 * $\mu=\bo\bp\cdot\bp\bi/\Vert\bo\bp\Vert\Vert\bp\bi\Vert$. To compute it, we
 * first need to compute the length $\Vert\bp\bi\Vert$, and we need to know when
 * the segment [p,i] intersects the ground.
 *
 * Distance to the top atmosphere boundary
 *
 * A point at distance d from p along [p,i] has coordinates
 * $[d\sqrt{1-\mu^2}, r+d\mu]^\top$, whose squared norm is $d^2+2r\mu d+r^2$.
 * Thus, by definition of i, we have
 * $\Vert\bp\bi\Vert^2+2r\mu\Vert\bp\bi\Vert+r^2=r_{\mathrm{top}}^2$,
 * from which we deduce the length $\Vert\bp\bi\Vert$:
 */

Length DistanceToTopAtmosphereBoundary(Length r, Number mu) 
{
  Area discriminant = r * r * (mu * mu - 1.0) + top_radius * top_radius;
  return ClampDistance(-r * mu + SafeSqrt(discriminant));
}

)" R"(

/*
 * We will also need, in the other sections, the distance to the bottom
 * atmosphere boundary, which can be computed in a similar way (this code assumes
 * that [p,i] intersects the ground).
 */

Length DistanceToBottomAtmosphereBoundary(Length r, Number mu) 
{
  Area discriminant = r * r * (mu * mu - 1.0) + bottom_radius * bottom_radius;
  return ClampDistance(-r * mu - SafeSqrt(discriminant));
}

/*
 * Intersections with the ground
 * 
 * The segment [p,i] intersects the ground when
 * $d^2+2r\mu d+r^2=r_{\mathrm{bottom}}^2$ has a solution with $d \ge 0$. This
 * requires the discriminant $r^2(\mu^2-1)+r_{\mathrm{bottom}}^2$ to be positive,
 * from which we deduce the following function:
 */

bool RayIntersectsGround(Length r, Number mu) 
{
  return mu < 0.0 && r * r * (mu * mu - 1.0) + bottom_radius * bottom_radius >= 0.0 * m2;
}

)" R"(

/*
 * Transmittance to the top atmosphere boundary
 * 
 * We can now compute the transmittance between p and i. From its definition and the Beer-Lambert law,
 * this involves the integral of the number density of air molecules along the
 * segment [p,i], as well as the integral of the number density of aerosols
 * and the integral of the number density of air molecules that absorb light
 * (e.g. ozone) - along the same segment. These 3 integrals have the same form and,
 * when the segment [p,i] does not intersect the ground, they can be computed
 * numerically with the help of the following auxilliary function using the trapezoidal rule.
 *
 * https://en.wikipedia.org/wiki/Trapezoidal_rule
 * https://en.wikipedia.org/wiki/Beer-Lambert_law
 *
 */

Number GetLayerDensity(DensityProfileLayer layer, Length altitude) 
{
  Number density = layer.exp_term * exp(layer.exp_scale * altitude) + layer.linear_term * altitude + layer.constant_term;
  return clamp(density, Number(0.0), Number(1.0));
}

Number GetProfileDensity(DensityProfile profile, Length altitude) 
{
  return altitude < profile.layers[0].width ?
      GetLayerDensity(profile.layers[0], altitude) :
      GetLayerDensity(profile.layers[1], altitude);
}

Length ComputeOpticalLengthToTopAtmosphereBoundary(DensityProfile profile, Length r, Number mu) 
{
  // Number of intervals for the numerical integration.
  const int SAMPLE_COUNT = 500;
  // The integration step, i.e. the length of each integration interval.
  Length dx = DistanceToTopAtmosphereBoundary(r, mu) / Number(SAMPLE_COUNT);
  // Integration loop.
  Length result = 0.0 * m;
  for (int i = 0; i <= SAMPLE_COUNT; ++i) 
  {
    Length d_i = Number(i) * dx;
    // Distance between the current sample point and the planet center.
    Length r_i = sqrt(d_i * d_i + 2.0 * r * mu * d_i + r * r);
    // Number density at the current sample point (divided by the number density
    // at the bottom of the atmosphere, yielding a dimensionless number).
    Number y_i = GetProfileDensity(profile, r_i - bottom_radius);
    // Sample weight (from the trapezoidal rule).
    Number weight_i = i == 0 || i == SAMPLE_COUNT ? 0.5 : 1.0;
    result += y_i * weight_i * dx;
  }

  return result;
}

/*
 * With this function the transmittance between p and i is now easy to
 * compute (we continue to assume that the segment does not intersect the ground):
 */

DimensionlessSpectrum ComputeTransmittanceToTopAtmosphereBoundary(Length r, Number mu) 
{

  DimensionlessSpectrum density = DimensionlessSpectrum(0.0);
  density += rayleigh_scattering * ComputeOpticalLengthToTopAtmosphereBoundary(RayleighDensity(), r, mu);
  density += mie_extinction * ComputeOpticalLengthToTopAtmosphereBoundary(MieDensity(), r, mu);
  density += absorption_extinction * ComputeOpticalLengthToTopAtmosphereBoundary(AbsorptionDensity(), r, mu);

  return exp(-density);
}

)" R"(

/*
 * Precomputation
 *
 * The above function is quite costly to evaluate, and a lot of evaluations are
 * needed to compute single and multiple scattering. Fortunately this function
 * depends on only two parameters and is quite smooth, so we can precompute it in a
 * small 2D texture to optimize its evaluation.
 * 
 * For this we need a mapping between the function parameters (r,mu) and the
 * texture coordinates (u,v), and vice-versa, because these parameters do not
 * have the same units and range of values. And even if it was the case, storing a
 * function f from the [0,1] interval in a texture of size n would sample the
 * function at 0.5/n, 1.5/n, ... (n-0.5)/n, because texture samples are at
 * the center of texels. Therefore, this texture would only give us extrapolated
 * function values at the domain boundaries (0 and 1). To avoid this we need
 * to store f(0) at the center of texel 0 and f(1) at the center of texel
 * n-1. This can be done with the following mapping from values x in [0,1] to
 * texture coordinates u in [0.5/n,1-0.5/n] - and its inverse:
 */

Number GetTextureCoordFromUnitRange(Number x, int texture_size) {
  return 0.5 / Number(texture_size) + x * (1.0 - 1.0 / Number(texture_size));
}

Number GetUnitRangeFromTextureCoord(Number u, int texture_size) {
  return (u - 0.5 / Number(texture_size)) / (1.0 - 1.0 / Number(texture_size));
}

/*
 * Using these functions, we can now define a mapping between (r,mu) and the
 * texture coordinates (u,v), and its inverse, which avoid any extrapolation
 * during texture lookups. In the <a href=
 * "http://evasion.inrialpes.fr/~Eric.Bruneton/PrecomputedAtmosphericScattering2.zip"
 * >original implementation</a> this mapping was using some ad-hoc constants chosen
 * for the Earth atmosphere case. Here we use a generic mapping, working for any
 * atmosphere, but still providing an increased sampling rate near the horizon.
 * Our improved mapping is based on the parameterization described in our
 * <a href="https://hal.inria.fr/inria-00288758/en">paper</a> for the 4D textures:
 * we use the same mapping for r, and a slightly improved mapping for mu
 * (considering only the case where the view ray does not intersect the ground).
 * More precisely, we map mu to a value $x_{\mu}$ between 0 and 1 by considering
 * the distance d to the top atmosphere boundary, compared to its minimum and
 * maximum values $d_{\mathrm{min}}=r_{\mathrm{top}}-r$ and
 * $d_{\mathrm{max}}=\rho+H$
 *
 * With these definitions, the mapping from (r,\mu)$ to the texture coordinates
 * (u,v) can be implemented as follows:
 */

vec2 GetTransmittanceTextureUvFromRMu(Length r, Number mu) 
{
  // Distance to top atmosphere boundary for a horizontal ray at ground level.
  Length H = sqrt(top_radius * top_radius - bottom_radius * bottom_radius);
  // Distance to the horizon.
  Length rho = SafeSqrt(r * r - bottom_radius * bottom_radius);
  // Distance to the top atmosphere boundary for the ray (r,mu), and its minimum
  // and maximum values over all mu - obtained for (r,1) and (r,mu_horizon).
  Length d = DistanceToTopAtmosphereBoundary(r, mu);
  Length d_min = top_radius - r;
  Length d_max = rho + H;
  Number x_mu = (d - d_min) / (d_max - d_min);
  Number x_r = rho / H;
  return vec2(GetTextureCoordFromUnitRange(x_mu, TRANSMITTANCE_TEXTURE_WIDTH),
                GetTextureCoordFromUnitRange(x_r, TRANSMITTANCE_TEXTURE_HEIGHT));
}

/*
 * and the inverse mapping follows immediately:
 */

void GetRMuFromTransmittanceTextureUv(vec2 uv, out Length r, out Number mu) 
{

  Number x_mu = GetUnitRangeFromTextureCoord(uv.x, TRANSMITTANCE_TEXTURE_WIDTH);
  Number x_r = GetUnitRangeFromTextureCoord(uv.y, TRANSMITTANCE_TEXTURE_HEIGHT);
  // Distance to top atmosphere boundary for a horizontal ray at ground level.
  Length H = sqrt(top_radius * top_radius - bottom_radius * bottom_radius);
  // Distance to the horizon, from which we can compute r:
  Length rho = H * x_r;
  r = sqrt(rho * rho + bottom_radius * bottom_radius);
  // Distance to the top atmosphere boundary for the ray (r,mu), and its minimum
  // and maximum values over all mu - obtained for (r,1) and (r,mu_horizon) -
  // from which we can recover mu:
  Length d_min = top_radius - r;
  Length d_max = rho + H;
  Length d = d_min + x_mu * (d_max - d_min);
  mu = d == 0.0 * m ? Number(1.0) : (H * H - rho * rho - d * d) / (2.0 * r * d);
  mu = ClampCosine(mu);
}

)" R"(

/*
 * It is now easy to define a fragment shader function to precompute a texel of
 * the transmittance texture:
*/

DimensionlessSpectrum ComputeTransmittanceToTopAtmosphereBoundaryTexture(vec2 gl_frag_coord) 
{
  const vec2 TRANSMITTANCE_TEXTURE_SIZE = vec2(TRANSMITTANCE_TEXTURE_WIDTH, TRANSMITTANCE_TEXTURE_HEIGHT);
  Length r;
  Number mu;
  GetRMuFromTransmittanceTextureUv(gl_frag_coord / TRANSMITTANCE_TEXTURE_SIZE, r, mu);
  return ComputeTransmittanceToTopAtmosphereBoundary(r, mu);
}

/*
 * Lookup
 *
 * With the help of the above precomputed texture, we can now get the
 * transmittance between a point and the top atmosphere boundary with a single
 * texture lookup (assuming there is no intersection with the ground):
 */

DimensionlessSpectrum GetTransmittanceToTopAtmosphereBoundary(TransmittanceTexture transmittance_texture, Length r, Number mu) 
{
  vec2 uv = GetTransmittanceTextureUvFromRMu(r, mu);
  return TEX2D(transmittance_texture, uv).rgb;
}

/*
 * Also, with $r_d=\Vert\bo\bq\Vert=\sqrt{d^2+2r\mu d+r^2}$ and $\mu_d=
 * \bo\bq\cdot\bp\bi/\Vert\bo\bq\Vert\Vert\bp\bi\Vert=(r\mu+d)/r_d$ the values of
 * r and mu at q, we can get the transmittance between two arbitrary
 * points p and q inside the atmosphere with only two texture lookups
 * (recall that the transmittance between p and q is the transmittance
 * between p and the top atmosphere boundary, divided by the transmittance
 * between q and the top atmosphere boundary, or the reverse - we continue to
 * assume that the segment between the two points does not intersect the ground):
 */

DimensionlessSpectrum GetTransmittance(TransmittanceTexture transmittance_texture, Length r, Number mu, Length d, bool ray_r_mu_intersects_ground) 
{

  Length r_d = ClampRadius(sqrt(d * d + 2.0 * r * mu * d + r * r));
  Number mu_d = ClampCosine((r * mu + d) / r_d);

  if (ray_r_mu_intersects_ground) 
  {
    return min(
        GetTransmittanceToTopAtmosphereBoundary(transmittance_texture, r_d, -mu_d) / 
		GetTransmittanceToTopAtmosphereBoundary(transmittance_texture, r, -mu), 
		DimensionlessSpectrum(1,1,1));
  } 
  else 
  {
    return min(
        GetTransmittanceToTopAtmosphereBoundary(transmittance_texture, r, mu) /
        GetTransmittanceToTopAtmosphereBoundary( transmittance_texture, r_d, mu_d),
        DimensionlessSpectrum(1,1,1));
  }
}

)" R"(

/*
 * where ray_r_mu_intersects_ground should be true if the ray
 * defined by r and mu intersects the ground. We don't compute it here with
 * RayIntersectsGround because the result could be wrong for rays
 * very close to the horizon, due to the finite precision and rounding errors of
 * floating point operations. And also because the caller generally has more robust
 * ways to know whether a ray intersects the ground or not (see below).
 *
 * Finally, we will also need the transmittance between a point in the
 * atmosphere and the Sun. The Sun is not a point light source, so this is an
 * integral of the transmittance over the Sun disc. Here we consider that the
 * transmittance is constant over this disc, except below the horizon, where the
 * transmittance is 0. As a consequence, the transmittance to the Sun can be
 * computed with GetTransmittanceToTopAtmosphereBoundary, times the
 * fraction of the Sun disc which is above the horizon.
 * 
 * This fraction varies from 0 when the Sun zenith angle theta_s is larger
 * than the horizon zenith angle theta_h plus the Sun angular radius alpha_s,
 * to 1 when theta_s is smaller than theta_h-\alpha_s. Equivalently, it
 * varies from 0 when $\mu_s=\cos\theta_s$ is smaller than
 * $\cos(\theta_h+\alpha_s)\approx\cos\theta_h-\alpha_s\sin\theta_h$ to 1 when
 * $\mu_s$ is larger than
 * $\cos(\theta_h-\alpha_s)\approx\cos\theta_h+\alpha_s\sin\theta_h$. In between,
 * the visible Sun disc fraction varies approximately like a smoothstep (this can
 * be verified by plotting the area of...
*/

DimensionlessSpectrum GetTransmittanceToSun(TransmittanceTexture transmittance_texture, Length r, Number mu_s)
{
    Number sin_theta_h = bottom_radius / r;
    Number cos_theta_h = -sqrt(max(1.0 - sin_theta_h * sin_theta_h, 0.0));

    Number step = smoothstep(-sin_theta_h * sun_angular_radius / rad, sin_theta_h * sun_angular_radius / rad, mu_s - cos_theta_h);

    return GetTransmittanceToTopAtmosphereBoundary(transmittance_texture, r, mu_s) * step;

}
)";


scattering_functions = R"(
/**
 * Copyright (c) 2017 Eric Bruneton
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Precomputed Atmospheric Scattering
 * Copyright (c) 2008 INRIA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Single scattering
 * 
 * The single scattered radiance is the light arriving from the Sun at some
 * point after exactly one scattering event inside the atmosphere (which can be due
 * to air molecules or aerosol particles; we exclude reflections from the ground,
 * computed <a href="#irradiance">separately</a>). The following sections describe
 * how we compute it, how we store it in a precomputed texture, and how we read it
 * back.
 * 
 * Computation
 * 
 * Consider the Sun light scattered at a point q by air molecules before
 * arriving at another point p (for aerosols, replace "Rayleigh" with "Mie").
 * 
 * The radiance arriving at $\bp$ is the product of:
 * 
 * the solar irradiance at the top of the atmosphere,
 * the transmittance between the Sun and q (i.e. the fraction of the Sun
 * light at the top of the atmosphere that reaches q),
 * the Rayleigh scattering coefficient at q (i.e. the fraction of the
 * light arriving at q which is scattered, in any direction),
 * the Rayleigh phase function (i.e. the fraction of the scattered light at
 * q which is actually scattered towards p),
 * the transmittance between q and p (i.e. the fraction of the light
 * scattered at q towards p that reaches p).
 * 
 * Thus, by noting w_s the unit direction vector towards the Sun, and with
 * the following definitions:
 * 
 * <li>$r=\Vert\bo\bp\Vert$,</li>
 * <li>$d=\Vert\bp\bq\Vert$,</li>
 * <li>$\mu=(\bo\bp\cdot\bp\bq)/rd$,</li>
 * <li>$\mu_s=(\bo\bp\cdot\bw_s)/r$,</li>
 * <li>$\nu=(\bp\bq\cdot\bw_s)/d$</li>
 * 
 * the values of r and mu_s for q are
 * 
 * <li>$r_d=\Vert\bo\bq\Vert=\sqrt{d^2+2r\mu d +r^2}$,</li>
 * <li>$\mu_{s,d}=(\bo\bq\cdot\bw_s)/r_d=((\bo\bp+\bp\bq)\cdot\bw_s)/r_d=
 * (r\mu_s + d\nu)/r_d$</li>
 * 
 * and the Rayleigh and Mie single scattering components can be computed as follows
 * (note that we omit the solar irradiance and the phase function terms, as well as
 * the scattering coefficients at the bottom of the atmosphere - we add them later
 * on for efficiency reasons):
 */

void ComputeSingleScatteringIntegrand(TransmittanceTexture transmittance_texture, 
				      Length r, Number mu, Number mu_s, Number nu, Length d, 
				      bool ray_r_mu_intersects_ground, out DimensionlessSpectrum rayleigh, out DimensionlessSpectrum mie) 
{
  Length r_d = ClampRadius(sqrt(d * d + 2.0 * r * mu * d + r * r));
  Number mu_s_d = ClampCosine((r * mu_s + d * nu) / r_d);

  DimensionlessSpectrum transmittance =
      GetTransmittance(transmittance_texture, r, mu, d, ray_r_mu_intersects_ground) *
      GetTransmittanceToSun(transmittance_texture, r_d, mu_s_d);

  rayleigh = transmittance * GetProfileDensity(RayleighDensity(), r_d - bottom_radius);
  mie = transmittance * GetProfileDensity(MieDensity(), r_d - bottom_radius);

}

)" R"(

/*
 * Consider now the Sun light arriving at p from a given direction w,
 * after exactly one scattering event. The scattering event can occur at any point
 * q between p and the intersection i of the half-line [p,w) with
 * the nearest atmosphere boundary. Thus, the single scattered radiance at p
 * from direction w is the integral of the single scattered radiance from q
 * to p for all points q between p and i. To compute it, we first
 * need the length $\Vert\bp\bi\Vert$:
 */

Length DistanceToNearestAtmosphereBoundary(Length r, Number mu, bool ray_r_mu_intersects_ground) 
{
  if (ray_r_mu_intersects_ground)
    return DistanceToBottomAtmosphereBoundary(r, mu);
  else
    return DistanceToTopAtmosphereBoundary(r, mu);
  
}

/*
 * The single scattering integral can then be computed as follows (using
 * the <a href="https://en.wikipedia.org/wiki/Trapezoidal_rule">trapezoidal
 * rule</a>):
 */

void ComputeSingleScattering(
    TransmittanceTexture transmittance_texture,
    Length r, Number mu, Number mu_s, Number nu,
    bool ray_r_mu_intersects_ground,
    out IrradianceSpectrum rayleigh, out IrradianceSpectrum mie) 
{

  // Number of intervals for the numerical integration.
  const int SAMPLE_COUNT = 50;

  // The integration step, i.e. the length of each integration interval.
  Length dx = DistanceToNearestAtmosphereBoundary(r, mu, ray_r_mu_intersects_ground) / Number(SAMPLE_COUNT);

  // Integration loop.
  DimensionlessSpectrum rayleigh_sum = DimensionlessSpectrum(0,0,0);
  DimensionlessSpectrum mie_sum = DimensionlessSpectrum(0,0,0);

  for (int i = 0; i <= SAMPLE_COUNT; ++i) 
  {
    Length d_i = Number(i) * dx;
    // The Rayleigh and Mie single scattering at the current sample point.
    DimensionlessSpectrum rayleigh_i;
    DimensionlessSpectrum mie_i;
    ComputeSingleScatteringIntegrand(transmittance_texture, r, mu, mu_s, nu, d_i, ray_r_mu_intersects_ground, rayleigh_i, mie_i);

    // Sample weight (from the trapezoidal rule).
    Number weight_i = (i == 0 || i == SAMPLE_COUNT) ? 0.5 : 1.0;
    rayleigh_sum += rayleigh_i * weight_i;
    mie_sum += mie_i * weight_i;
  }

  rayleigh = rayleigh_sum * dx * solar_irradiance * rayleigh_scattering;

  mie = mie_sum * dx * solar_irradiance * mie_scattering;
}

/*
 * Note that we added the solar irradiance and the scattering coefficient terms
 * that we omitted in ComputeSingleScatteringIntegrand, but not the
 * phase function terms - they are added at <a href="#rendering">render time</a>
 * for better angular precision. We provide them here for completeness:
 */

InverseSolidAngle RayleighPhaseFunction(Number nu) 
{
  InverseSolidAngle k = 3.0 / (16.0 * PI * sr);
  return k * (1.0 + nu * nu);
}

InverseSolidAngle MiePhaseFunction(Number g, Number nu) 
{
  InverseSolidAngle k = 3.0 / (8.0 * PI * sr) * (1.0 - g * g) / (2.0 + g * g);
  return k * (1.0 + nu * nu) / pow(abs(1.0 + g * g - 2.0 * g * nu), 1.5);
}

)" R"(

/*
 * Precomputation
 * 
 * The ComputeSingleScattering function is quite costly to
 * evaluate, and a lot of evaluations are needed to compute multiple scattering.
 * We therefore want to precompute it in a texture, which requires a mapping from
 * the 4 function parameters to texture coordinates. Assuming for now that we have
 * 4D textures, we need to define a mapping from (r,mu,mu_s,nu) to texture
 * coordinates (u,v,w,z). The function below implements the mapping defined in
 * our <a href="https://hal.inria.fr/inria-00288758/en">paper</a>, with some small
 * improvements (refer to the paper and to the above figures for the notations):
 * 
 * the mapping for mu takes the minimal distance to the nearest atmosphere
 * boundary into account, to map mu to the full [0,1]$ interval (the original
 * mapping was not covering the full [0,1] interval).
 * the mapping for mu_s is more generic than in the paper (the original
 * mapping was using ad-hoc constants chosen for the Earth atmosphere case). It is
 * based on the distance to the top atmosphere boundary (for the sun rays), as for
 * the mu mapping, and uses only one ad-hoc (but configurable) parameter. Yet,
 * as the original definition, it provides an increased sampling rate near the
 * horizon.
 * 
 */

vec4 GetScatteringTextureUvwzFromRMuMuSNu(
    Length r, Number mu, Number mu_s, Number nu,
    bool ray_r_mu_intersects_ground) 
{

  // Distance to top atmosphere boundary for a horizontal ray at ground level.
  Length H = sqrt(top_radius * top_radius - bottom_radius * bottom_radius);

  // Distance to the horizon.
  Length rho = SafeSqrt(r * r - bottom_radius * bottom_radius);
  Number u_r = GetTextureCoordFromUnitRange(rho / H, SCATTERING_TEXTURE_R_SIZE);

  // Discriminant of the quadratic equation for the intersections of the ray
  // (r,mu) with the ground (see RayIntersectsGround).
  Length r_mu = r * mu;
  Area discriminant = r_mu * r_mu - r * r + bottom_radius * bottom_radius;
  Number u_mu;

  if (ray_r_mu_intersects_ground) 
  {
    // Distance to the ground for the ray (r,mu), and its minimum and maximum
    // values over all mu - obtained for (r,-1) and (r,mu_horizon).
    Length d = -r_mu - SafeSqrt(discriminant);
    Length d_min = r - bottom_radius;
    Length d_max = rho;
    u_mu = 0.5 - 0.5 * GetTextureCoordFromUnitRange(d_max == d_min ? 0.0 :
        (d - d_min) / (d_max - d_min), SCATTERING_TEXTURE_MU_SIZE / 2);
  } 
  else 
  {
    // Distance to the top atmosphere boundary for the ray (r,mu), and its
    // minimum and maximum values over all mu - obtained for (r,1) and
    // (r,mu_horizon).
    Length d = -r_mu + SafeSqrt(discriminant + H * H);
    Length d_min = top_radius - r;
    Length d_max = rho + H;
    u_mu = 0.5 + 0.5 * GetTextureCoordFromUnitRange(
        (d - d_min) / (d_max - d_min), SCATTERING_TEXTURE_MU_SIZE / 2);
  }

  Length d = DistanceToTopAtmosphereBoundary(bottom_radius, mu_s);
  Length d_min = top_radius - bottom_radius;
  Length d_max = H;
  Number a = (d - d_min) / (d_max - d_min);
  Number A = -2.0 * mu_s_min * bottom_radius / (d_max - d_min);

  Number u_mu_s = GetTextureCoordFromUnitRange(max(1.0 - a / A, 0.0) / (1.0 + a), SCATTERING_TEXTURE_MU_S_SIZE);

  Number u_nu = (nu + 1.0) / 2.0;
  return vec4(u_nu, u_mu_s, u_mu, u_r);
}

/*
 * The inverse mapping follows immediately:
*/

void GetRMuMuSNuFromScatteringTextureUvwz(
    vec4 uvwz, out Length r, out Number mu, out Number mu_s,
	out Number nu, out bool ray_r_mu_intersects_ground)
{

  // Distance to top atmosphere boundary for a horizontal ray at ground level.
  Length H = sqrt(top_radius * top_radius - bottom_radius * bottom_radius);
  // Distance to the horizon.
  Length rho = H * GetUnitRangeFromTextureCoord(uvwz.w, SCATTERING_TEXTURE_R_SIZE);
  r = sqrt(rho * rho + bottom_radius * bottom_radius);

  if (uvwz.z < 0.5) 
  {
    // Distance to the ground for the ray (r,mu), and its minimum and maximum
    // values over all mu - obtained for (r,-1) and (r,mu_horizon) - from which
    // we can recover mu:
    Length d_min = r - bottom_radius;
    Length d_max = rho;
    Length d = d_min + (d_max - d_min) * GetUnitRangeFromTextureCoord(1.0 - 2.0 * uvwz.z, SCATTERING_TEXTURE_MU_SIZE / 2);
    mu = d == 0.0 * m ? Number(-1.0) : ClampCosine(-(rho * rho + d * d) / (2.0 * r * d));
    ray_r_mu_intersects_ground = true;
  } 
  else 
  {
    // Distance to the top atmosphere boundary for the ray (r,mu), and its
    // minimum and maximum values over all mu - obtained for (r,1) and
    // (r,mu_horizon) - from which we can recover mu:
    Length d_min = top_radius - r;
    Length d_max = rho + H;
    Length d = d_min + (d_max - d_min) * GetUnitRangeFromTextureCoord(2.0 * uvwz.z - 1.0, SCATTERING_TEXTURE_MU_SIZE / 2);
    mu = d == 0.0 * m ? Number(1.0) : ClampCosine((H * H - rho * rho - d * d) / (2.0 * r * d));
    ray_r_mu_intersects_ground = false;
  }

  Number x_mu_s = GetUnitRangeFromTextureCoord(uvwz.y, SCATTERING_TEXTURE_MU_S_SIZE);
  Length d_min = top_radius - bottom_radius;
  Length d_max = H;
  Number A = -2.0 * mu_s_min * bottom_radius / (d_max - d_min);
  Number a = (A - x_mu_s * A) / (1.0 + x_mu_s * A);
  Length d = d_min + min(a, A) * (d_max - d_min);
  mu_s = d == 0.0 * m ? Number(1.0) : ClampCosine((H * H - d * d) / (2.0 * bottom_radius * d));

  nu = ClampCosine(uvwz.x * 2.0 - 1.0);
}

)" R"(

/*
 * We assumed above that we have 4D textures, which is not the case in practice.
 * We therefore need a further mapping, between 3D and 4D texture coordinates. The
 * function below expands a 3D texel coordinate into a 4D texture coordinate, and
 * then to (r,mu,mu_s,nu) parameters. It does so by "unpacking" two texel
 * coordinates from the x texel coordinate. Note also how we clamp the nu
 * parameter at the end. This is because nu is not a fully independent variable:
 * its range of values depends on mu and mu_s (this can be seen by computing
 * mu, mu_s and nu from the cartesian coordinates of the zenith, view and
 * sun unit direction vectors), and the previous functions implicitely assume this
 * (their assertions can break if this constraint is not respected).
 */

void GetRMuMuSNuFromScatteringTextureFragCoord(
    vec3 gl_frag_coord,
    out Length r, out Number mu, out Number mu_s, out Number nu,
    out bool ray_r_mu_intersects_ground) 
{
  const vec4 SCATTERING_TEXTURE_SIZE = vec4(
      SCATTERING_TEXTURE_NU_SIZE - 1,
      SCATTERING_TEXTURE_MU_S_SIZE,
      SCATTERING_TEXTURE_MU_SIZE,
      SCATTERING_TEXTURE_R_SIZE);

  Number frag_coord_nu = floor(gl_frag_coord.x / Number(SCATTERING_TEXTURE_MU_S_SIZE));
  Number frag_coord_mu_s = mod(gl_frag_coord.x, Number(SCATTERING_TEXTURE_MU_S_SIZE));

  vec4 uvwz = vec4(frag_coord_nu, frag_coord_mu_s, gl_frag_coord.y, gl_frag_coord.z) / SCATTERING_TEXTURE_SIZE;

  GetRMuMuSNuFromScatteringTextureUvwz(uvwz, r, mu, mu_s, nu, ray_r_mu_intersects_ground);

  // Clamp nu to its valid range of values, given mu and mu_s.
  nu = clamp(nu, mu * mu_s - sqrt((1.0 - mu * mu) * (1.0 - mu_s * mu_s)),
      mu * mu_s + sqrt((1.0 - mu * mu) * (1.0 - mu_s * mu_s)));
}

/*
 * With this mapping, we can finally write a function to precompute a texel of
 * the single scattering in a 3D texture:
 */

void ComputeSingleScatteringTexture(
    TransmittanceTexture transmittance_texture, vec3 gl_frag_coord,
    out IrradianceSpectrum rayleigh, out IrradianceSpectrum mie)
{
  Length r;
  Number mu;
  Number mu_s;
  Number nu;
  bool ray_r_mu_intersects_ground;
  GetRMuMuSNuFromScatteringTextureFragCoord(gl_frag_coord,
      r, mu, mu_s, nu, ray_r_mu_intersects_ground);
  ComputeSingleScattering(transmittance_texture,
      r, mu, mu_s, nu, ray_r_mu_intersects_ground, rayleigh, mie);
}

/*
 * Lookup
 * 
 * With the help of the above precomputed texture, we can now get the scattering
 * between a point and the nearest atmosphere boundary with two texture lookups (we
 * need two 3D texture lookups to emulate a single 4D texture lookup with
 * quadrilinear interpolation; the 3D texture coordinates are computed using the
 * inverse of the 3D-4D mapping defined in
 * GetRMuMuSNuFromScatteringTextureFragCoord):
 */

AbstractSpectrum GetScattering(
    AbstractScatteringTexture scattering_texture,
    Length r, Number mu, Number mu_s, Number nu,
    bool ray_r_mu_intersects_ground) 
{
  vec4 uvwz = GetScatteringTextureUvwzFromRMuMuSNu(r, mu, mu_s, nu, ray_r_mu_intersects_ground);

  Number tex_coord_x = uvwz.x * Number(SCATTERING_TEXTURE_NU_SIZE - 1);
  Number tex_x = floor(tex_coord_x);
  Number lerp = tex_coord_x - tex_x;

  vec3 uvw0 = vec3((tex_x + uvwz.y) / Number(SCATTERING_TEXTURE_NU_SIZE), uvwz.z, uvwz.w);

  vec3 uvw1 = vec3((tex_x + 1.0 + uvwz.y) / Number(SCATTERING_TEXTURE_NU_SIZE), uvwz.z, uvwz.w);

  return TEX3D(scattering_texture, uvw0).rgb * (1.0 - lerp) + TEX3D(scattering_texture, uvw1).rgb * lerp;
}

)" R"(

/*
 * Finally, we provide here a convenience lookup function which will be useful
 * in the next section. This function returns either the single scattering, with
 * the phase functions included, or the n-th order of scattering, with n>1. It
 * assumes that, if scattering_order is strictly greater than 1, then
 * multiple_scattering_texture corresponds to this scattering order,
 * with both Rayleigh and Mie included, as well as all the phase function terms.
 */

RadianceSpectrum GetScattering(
    ReducedScatteringTexture single_rayleigh_scattering_texture,
    ReducedScatteringTexture single_mie_scattering_texture,
    ScatteringTexture multiple_scattering_texture,
    Length r, Number mu, Number mu_s, Number nu,
    bool ray_r_mu_intersects_ground,
    int scattering_order) 
{
  if (scattering_order == 1) 
  {
    IrradianceSpectrum rayleigh = GetScattering(
        single_rayleigh_scattering_texture, r, mu, mu_s, nu,
        ray_r_mu_intersects_ground);

    IrradianceSpectrum mie = GetScattering(
        single_mie_scattering_texture, r, mu, mu_s, nu,
        ray_r_mu_intersects_ground);

    return rayleigh * RayleighPhaseFunction(nu) +
        mie * MiePhaseFunction(mie_phase_function_g, nu);
  } 
  else 
  {
    return GetScattering(
        multiple_scattering_texture, r, mu, mu_s, nu,
        ray_r_mu_intersects_ground);
  }
}

)" R"(

/*
 * Multiple scattering
 * 
 * The multiply scattered radiance is the light arriving from the Sun at some
 * point in the atmosphere after two or more bounces (where a bounce is
 * either a scattering event or a reflection from the ground). The following
 * sections describe how we compute it, how we store it in a precomputed texture,
 * and how we read it back.
 * 
 * Note that, as for single scattering, we exclude here the light paths whose
 * last bounce is a reflection on the ground. The contribution from these
 * paths is computed separately, at rendering time, in order to take the actual
 * ground albedo into account (for intermediate reflections on the ground, which
 * are precomputed, we use an average, uniform albedo).
 * 
 * Computation
 * 
 * Multiple scattering can be decomposed into the sum of double scattering,
 * triple scattering, etc, where each term corresponds to the light arriving from
 * the Sun at some point in the atmosphere after exactly 2, 3, etc bounces.
 * Moreover, each term can be computed from the previous one. Indeed, the light
 * arriving at some point p from direction w after n bounces is an
 * integral over all the possible points q for the last bounce, which involves
 * the light arriving at q from any direction, after n-1 bounces.
 * 
 * This description shows that each scattering order requires a triple integral
 * to be computed from the previous one (one integral over all the points q
 * on the segment from p to the nearest atmosphere boundary in direction w,
 * and a nested double integral over all directions at each point q).
 * Therefore, if we wanted to compute each order "from scratch", we would need a
 * triple integral for double scattering, a sextuple integral for triple
 * scattering, etc. This would be clearly inefficient, because of all the redundant
 * computations (the computations for order n would basically redo all the
 * computations for all previous orders, leading to quadratic complexity in the
 * total number of orders). Instead, it is much more efficient to proceed as
 * follows:
 * 
 * precompute single scattering in a texture (as described above), for n^2:
 * 
 * precompute the n-th scattering in a texture, with a triple integral whose
 * integrand uses lookups in the (n-1)-th scattering texture
 * 
 * This strategy avoids many redundant computations but does not eliminate all
 * of them. Consider for instance the points p and p' in the figure below,
 * and the computations which are necessary to compute the light arriving at these
 * two points from direction w after n bounces. These computations involve,
 * in particular, the evaluation of the radiance L which is scattered at q in
 * direction -w, and coming from all directions after n-1 bounces:
 * 
 * 
 * Therefore, if we computed the n-th scattering with a triple integral as
 * described above, we would compute L redundantly (in fact, for all points p
 * between q and the nearest atmosphere boundary in direction -w). To avoid
 * this, and thus increase the efficiency of the multiple scattering computations,
 * we refine the above algorithm as follows:
 * 
 * precompute single scattering in a texture (as described above),for n^2:
 * 
 * for each point q and direction w, precompute the light which is
 * scattered at q towards direction -w, coming from any direction after
 * n-1 bounces (this involves only a double integral, whose integrand uses
 * lookups in the (n-1)-th scattering texture),
 * for each point p and direction w, precompute the light coming from
 * direction w after n bounces (this involves only a single integral, whose
 * integrand uses lookups in the texture computed at the previous line)
 * 
 * To get a complete algorithm, we must now specify how we implement the two
 * steps in the above loop. This is what we do in the rest of this section.
 * 
 * First step
 * 
 * The first step computes the radiance which is scattered at some point q
 * inside the atmosphere, towards some direction -w. Furthermore, we assume
 * that this scattering event is the n-th bounce.
 * 
 * This radiance is the integral over all the possible incident directions
 * w_i, of the product of
 * 
 * the incident radiance L_i arriving at q from direction w_i after
 * n-1 bounces, which is the sum of:
 * 
 * a term given by the precomputed scattering texture for the (n-1)-th
 * order,
 * if the ray [q, w_i) intersects the ground at r, the contribution
 * from the light paths with n-1 bounces and whose last bounce is at r, i.e.
 * on the ground (these paths are excluded, by definition, from our precomputed
 * textures, but we must take them into account here since the bounce on the ground
 * is followed by a bounce at q). This contribution, in turn, is the product
 * of:
 * 
 * the transmittance between q and r,
 * the (average) ground albedo,
 * the <a href="https://www.cs.princeton.edu/~smr/cs348c-97/surveypaper.html"
 * >Lambertian BRDF</a> $1/\pi$,
 * the irradiance received on the ground after n-2 bounces. We explain in the
 * <a href="#irradiance">next section</a> how we precompute it in a texture. For
 * now, we assume that we can use the following function to retrieve this
 * irradiance from a precomputed texture:
 */

IrradianceSpectrum GetIrradiance(
    IrradianceTexture irradiance_texture,
    Length r, Number mu_s);

/*
 * 
 * the scattering coefficient at q,
 * the scattering phase function for the directions w and w_i
 * 
 * This leads to the following implementation (where
 * multiple_scattering_texture is supposed to contain the (n-1)-th
 * order of scattering, if n>2, irradiance_texture is the irradiance
 * received on the ground after n-2 bounces, and <scattering_order is
 * equal to n):
 */

RadianceDensitySpectrum ComputeScatteringDensity(
    TransmittanceTexture transmittance_texture,
    ReducedScatteringTexture single_rayleigh_scattering_texture,
    ReducedScatteringTexture single_mie_scattering_texture,
    ScatteringTexture multiple_scattering_texture,
    IrradianceTexture irradiance_texture,
    Length r, Number mu, Number mu_s, Number nu, int scattering_order) 
{

  // Compute unit direction vectors for the zenith, the view direction omega and
  // and the sun direction omega_s, such that the cosine of the view-zenith
  // angle is mu, the cosine of the sun-zenith angle is mu_s, and the cosine of
  // the view-sun angle is nu. The goal is to simplify computations below.
  vec3 zenith_direction = vec3(0.0, 0.0, 1.0);
  vec3 omega = vec3(sqrt(1.0 - mu * mu), 0.0, mu);
  Number sun_dir_x = omega.x == 0.0 ? 0.0 : (nu - mu * mu_s) / omega.x;
  Number sun_dir_y = sqrt(max(1.0 - sun_dir_x * sun_dir_x - mu_s * mu_s, 0.0));
  vec3 omega_s = vec3(sun_dir_x, sun_dir_y, mu_s);

  const int SAMPLE_COUNT = 16;
  const Angle dphi = pi / Number(SAMPLE_COUNT);
  const Angle dtheta = pi / Number(SAMPLE_COUNT);
  RadianceDensitySpectrum rayleigh_mie = RadianceDensitySpectrum(0,0,0);

  // Nested loops for the integral over all the incident directions omega_i.
  for (int l = 0; l < SAMPLE_COUNT; ++l) 
  {
    Angle theta = (Number(l) + 0.5) * dtheta;
    Number cos_theta = cos(theta);
    Number sin_theta = sin(theta);
    bool ray_r_theta_intersects_ground = RayIntersectsGround(r, cos_theta);

    // The distance and transmittance to the ground only depend on theta, so we
    // can compute them in the outer loop for efficiency.
    Length distance_to_ground = 0.0 * m;
    DimensionlessSpectrum transmittance_to_ground = DimensionlessSpectrum(0,0,0);
    DimensionlessSpectrum ground_albedo = DimensionlessSpectrum(0,0,0);
    if (ray_r_theta_intersects_ground) 
    {
      distance_to_ground = DistanceToBottomAtmosphereBoundary(r, cos_theta);
      transmittance_to_ground = GetTransmittance(transmittance_texture, r, cos_theta, distance_to_ground, true);
    }

    for (int m = 0; m < 2 * SAMPLE_COUNT; ++m) 
    {
      Angle phi = (Number(m) + 0.5) * dphi;
      vec3 omega_i = vec3(cos(phi) * sin_theta, sin(phi) * sin_theta, cos_theta);
      SolidAngle domega_i = (dtheta / rad) * (dphi / rad) * sin(theta) * sr;

      // The radiance L_i arriving from direction omega_i after n-1 bounces is
      // the sum of a term given by the precomputed scattering texture for the
      // (n-1)-th order:
      Number nu1 = dot(omega_s, omega_i);
      RadianceSpectrum incident_radiance = GetScattering(
          single_rayleigh_scattering_texture, single_mie_scattering_texture,
          multiple_scattering_texture, r, omega_i.z, mu_s, nu1,
          ray_r_theta_intersects_ground, scattering_order - 1);

      // and of the contribution from the light paths with n-1 bounces and whose
      // last bounce is on the ground. This contribution is the product of the
      // transmittance to the ground, the ground albedo, the ground BRDF, and
      // the irradiance received on the ground after n-2 bounces.
      vec3 ground_normal = normalize(zenith_direction * r + omega_i * distance_to_ground);
      IrradianceSpectrum ground_irradiance = GetIrradiance(irradiance_texture, bottom_radius, dot(ground_normal, omega_s));
      incident_radiance += transmittance_to_ground * ground_albedo * (1.0 / (PI * sr)) * ground_irradiance;

      // The radiance finally scattered from direction omega_i towards direction
      // -omega is the product of the incident radiance, the scattering
      // coefficient, and the phase function for directions omega and omega_i
      // (all this summed over all particle types, i.e. Rayleigh and Mie).
      Number nu2 = dot(omega, omega_i);

      Number rayleigh_density = GetProfileDensity(RayleighDensity(), r - bottom_radius);
      Number mie_density = GetProfileDensity(MieDensity(), r - bottom_radius);

      rayleigh_mie += incident_radiance * (
          rayleigh_scattering * rayleigh_density * RayleighPhaseFunction(nu2) +
          mie_scattering * mie_density * MiePhaseFunction(mie_phase_function_g, nu2)) *
          domega_i;
    }
  }

  return rayleigh_mie;
}

)" R"(

/*
 * Second step
 * 
 * The second step to compute the n-th order of scattering is to compute for
 * each point p and direction w, the radiance coming from direction w
 * after n bounces, using a texture precomputed with the previous function.
 * 
 * This radiance is the integral over all points q between p and the
 * nearest atmosphere boundary in direction w of the product of:
 * 
 * a term given by a texture precomputed with the previous function, namely
 * the radiance scattered at q towards p, coming from any direction after
 * n-1 bounces, the transmittance betweeen p and q
 * 
 * Note that this excludes the light paths with n bounces and whose last
 * bounce is on the ground, on purpose. Indeed, we chose to exclude these paths
 * from our precomputed textures so that we can compute them at render time
 * instead, using the actual ground albedo.
 * 
 * The implementation for this second step is straightforward:
*/

RadianceSpectrum ComputeMultipleScattering(
    TransmittanceTexture transmittance_texture,
    ScatteringDensityTexture scattering_density_texture,
    Length r, Number mu, Number mu_s, Number nu,
    bool ray_r_mu_intersects_ground) 
{

  // Number of intervals for the numerical integration.
  const int SAMPLE_COUNT = 50;
  // The integration step, i.e. the length of each integration interval.
  Length dx = DistanceToNearestAtmosphereBoundary(r, mu, ray_r_mu_intersects_ground) / Number(SAMPLE_COUNT);

  // Integration loop.
  RadianceSpectrum rayleigh_mie_sum = RadianceSpectrum(0,0,0);

  for (int i = 0; i <= SAMPLE_COUNT; ++i) 
  {
    Length d_i = Number(i) * dx;

    // The r, mu and mu_s parameters at the current integration point (see the
    // single scattering section for a detailed explanation).
    Length r_i = ClampRadius(sqrt(d_i * d_i + 2.0 * r * mu * d_i + r * r));
    Number mu_i = ClampCosine((r * mu + d_i) / r_i);
    Number mu_s_i = ClampCosine((r * mu_s + d_i * nu) / r_i);

    // The Rayleigh and Mie multiple scattering at the current sample point.
    RadianceSpectrum rayleigh_mie_i =
        GetScattering(scattering_density_texture, r_i, mu_i, mu_s_i, nu, ray_r_mu_intersects_ground) *
        GetTransmittance(transmittance_texture, r, mu, d_i, ray_r_mu_intersects_ground) * dx;

    // Sample weight (from the trapezoidal rule).
    Number weight_i = (i == 0 || i == SAMPLE_COUNT) ? 0.5 : 1.0;
    rayleigh_mie_sum += rayleigh_mie_i * weight_i;
  }

  return rayleigh_mie_sum;
}

)" R"(

/*
 * Precomputation
 * 
 * s explained in the overall algorithm to
 * compute multiple scattering, we need to precompute each order of scattering in a
 * texture to save computations while computing the next order. And, in order to
 * store a function in a texture, we need a mapping from the function parameters to
 * texture coordinates. Fortunately, all the orders of scattering depend on the
 * same (r,mu,mu_s,nu) parameters as single scattering, so we can simple reuse
 * the mappings defined for single scattering. This immediately leads to the
 * following simple functions to precompute a texel of the textures for the
 * first and second steps of each iteration
 * over the number of bounces:
*/

RadianceDensitySpectrum ComputeScatteringDensityTexture(
    TransmittanceTexture transmittance_texture,
    ReducedScatteringTexture single_rayleigh_scattering_texture,
    ReducedScatteringTexture single_mie_scattering_texture,
    ScatteringTexture multiple_scattering_texture,
    IrradianceTexture irradiance_texture,
    vec3 gl_frag_coord, int scattering_order) 
{
  Length r;
  Number mu;
  Number mu_s;
  Number nu;
  bool ray_r_mu_intersects_ground;
  GetRMuMuSNuFromScatteringTextureFragCoord(gl_frag_coord,
      r, mu, mu_s, nu, ray_r_mu_intersects_ground);

  return ComputeScatteringDensity(transmittance_texture,
      single_rayleigh_scattering_texture, single_mie_scattering_texture,
      multiple_scattering_texture, irradiance_texture, r, mu, mu_s, nu,
      scattering_order);
}

RadianceSpectrum ComputeMultipleScatteringTexture(
    TransmittanceTexture transmittance_texture,
    ScatteringDensityTexture scattering_density_texture,
    vec3 gl_frag_coord, out Number nu) 
{
  Length r;
  Number mu;
  Number mu_s;
  bool ray_r_mu_intersects_ground;
  GetRMuMuSNuFromScatteringTextureFragCoord(gl_frag_coord,
      r, mu, mu_s, nu, ray_r_mu_intersects_ground);

  return ComputeMultipleScattering(transmittance_texture,
      scattering_density_texture, r, mu, mu_s, nu,
      ray_r_mu_intersects_ground);
}
)";


rendering_functions = R"(
/**
 * Copyright (c) 2017 Eric Bruneton
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Precomputed Atmospheric Scattering
 * Copyright (c) 2008 INRIA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */


 /*
  * Rendering
  *
  * Here we assume that the transmittance, scattering and irradiance textures
  * have been precomputed, and we provide functions using them to compute the sky
  * color, the aerial perspective, and the ground radiance.
  *
  * More precisely, we assume that the single Rayleigh scattering, without its
  * phase function term, plus the multiple scattering terms (divided by the Rayleigh
  * phase function for dimensional homogeneity) are stored in a
  * scattering_texture. We also assume that the single Mie scattering
  * is stored, without its phase function term:
  *
  * either separately, in a single_mie_scattering_texture (this
  * option was not provided our <a href=
  * "http://evasion.inrialpes.fr/~Eric.Bruneton/PrecomputedAtmosphericScattering2.zip"
  * >original implementation</a>),
  * or, if the COMBINED_SCATTERING_TEXTURES preprocessor
  * macro is defined, in the scattering_texture. In this case, which is
  * only available with a GLSL compiler, Rayleigh and multiple scattering are stored
  * in the RGB channels, and the red component of the single Mie scattering is
  * stored in the alpha channel).
  *
  * In the second case, the green and blue components of the single Mie
  * scattering are extrapolated as described in our
  * <a href="https://hal.inria.fr/inria-00288758/en">paper</a>, with the following
  * function:
  */

#ifdef COMBINED_SCATTERING_TEXTURES
vec3 GetExtrapolatedSingleMieScattering(vec4 scattering)
{
	if (scattering.r <= 0.0)
		return vec3(0, 0, 0);

    // GW- This 'reconstruction' matches the paper, but it produces
    // some rendering artifacts on the horizon and in close-up trees..
    // so we cannot use combined textures until resolving this
	return scattering.rgb *
        (scattering.a / scattering.r) *
		(rayleigh_scattering.r / mie_scattering.r) *
		(mie_scattering / rayleigh_scattering);
}
#endif

)" R"(

/*
 * We can then retrieve all the scattering components (Rayleigh + multiple
 * scattering on one side, and single Mie scattering on the other side) with the
 * following function, based on GetScattering (we duplicate
 * some code here, instead of using two calls to GetScattering, to
 * make sure that the texture coordinates computation is shared between the lookups
 * in scattering_texture and single_mie_scattering_texture):
 */

IrradianceSpectrum GetCombinedScattering(
	ReducedScatteringTexture scattering_texture,
	ReducedScatteringTexture single_mie_scattering_texture,
	Length r, Number mu, Number mu_s, Number nu,
	bool ray_r_mu_intersects_ground,
	out IrradianceSpectrum single_mie_scattering)
{
	vec4 uvwz = GetScatteringTextureUvwzFromRMuMuSNu(r, mu, mu_s, nu, ray_r_mu_intersects_ground);

	Number tex_coord_x = uvwz.x * Number(SCATTERING_TEXTURE_NU_SIZE - 1.0);
	Number tex_x = floor(tex_coord_x);
	Number lerp = tex_coord_x - tex_x;

	vec3 uvw0 = vec3((tex_x + uvwz.y) / Number(SCATTERING_TEXTURE_NU_SIZE), uvwz.z, uvwz.w);
	vec3 uvw1 = vec3((tex_x + 1.0 + uvwz.y) / Number(SCATTERING_TEXTURE_NU_SIZE), uvwz.z, uvwz.w);

#ifdef COMBINED_SCATTERING_TEXTURES
	vec4 combined_scattering = TEX3D(scattering_texture, uvw0) * (1.0 - lerp) + TEX3D(scattering_texture, uvw1) * lerp;
	IrradianceSpectrum scattering = combined_scattering.xyz;
	single_mie_scattering = GetExtrapolatedSingleMieScattering(combined_scattering);
#else
	IrradianceSpectrum scattering = TEX3D(scattering_texture, uvw0).xyz * (1.0 - lerp) + TEX3D(scattering_texture, uvw1).xyz * lerp;
	single_mie_scattering = TEX3D(single_mie_scattering_texture, uvw0).xyz * (1.0 - lerp) + TEX3D(single_mie_scattering_texture, uvw1).xyz * lerp;
#endif

	return scattering;
}

)" R"(

/*
 * Rendering Sky
 *
 * To render the sky we simply need to display the sky radiance, which we can
 * get with a lookup in the precomputed scattering texture(s), multiplied by the
 * phase function terms that were omitted during precomputation. We can also return
 * the transmittance of the atmosphere (which we can get with a single lookup in
 * the precomputed transmittance texture), which is needed to correctly render the
 * objects in space (such as the Sun and the Moon). This leads to the following
 * function, where most of the computations are used to correctly handle the case
 * of viewers outside the atmosphere, and the case of light shafts:
 */

RadianceSpectrum GetSkyRadiance(
	TransmittanceTexture transmittance_texture,
	ReducedScatteringTexture scattering_texture,
	ReducedScatteringTexture single_mie_scattering_texture,
	Position camera, Direction view_ray, Length shadow_length,
	Direction sun_direction, out DimensionlessSpectrum transmittance)
{
	// Compute the distance to the top atmosphere boundary along the view ray,
	// assuming the viewer is in space (or NaN if the view ray does not intersect
	// the atmosphere).
	Length r = length(camera);
	Length rmu = dot(camera, view_ray);
	Length distance_to_top_atmosphere_boundary = -rmu - sqrt(rmu * rmu - r * r + top_radius * top_radius);

	// If the viewer is in space and the view ray intersects the atmosphere, move
	// the viewer to the top atmosphere boundary (along the view ray):
	if (distance_to_top_atmosphere_boundary > 0.0 * m)
	{
		camera = camera + view_ray * distance_to_top_atmosphere_boundary;
		r = top_radius;
		rmu += distance_to_top_atmosphere_boundary;
	}
	else if (r > top_radius)
	{
		// If the view ray does not intersect the atmosphere, simply return 0.
		transmittance = DimensionlessSpectrum(1, 1, 1);
		return RadianceSpectrum(0, 0, 0);
	}

	// Compute the r, mu, mu_s and nu parameters needed for the texture lookups.
	Number mu = rmu / r;
	Number mu_s = dot(camera, sun_direction) / r;
	Number nu = dot(view_ray, sun_direction);
	bool ray_r_mu_intersects_ground = RayIntersectsGround(r, mu);

	transmittance = ray_r_mu_intersects_ground ? DimensionlessSpectrum(0, 0, 0) :
		GetTransmittanceToTopAtmosphereBoundary(transmittance_texture, r, mu);

	IrradianceSpectrum single_mie_scattering;
	IrradianceSpectrum scattering;

	if (shadow_length == 0.0 * m)
	{
		scattering = GetCombinedScattering(
			scattering_texture, single_mie_scattering_texture,
			r, mu, mu_s, nu, ray_r_mu_intersects_ground,
			single_mie_scattering);
	}
	else
	{
		// Case of light shafts (shadow_length is the total length noted l in our
		// paper): we omit the scattering between the camera and the point at
		// distance l, by implementing Eq. (18) of the paper (shadow_transmittance
		// is the T(x,x_s) term, scattering is the S|x_s=x+lv term).
		Length d = shadow_length;
		Length r_p = ClampRadius(sqrt(d * d + 2.0 * r * mu * d + r * r));

		Number mu_p = (r * mu + d) / r_p;
		Number mu_s_p = (r * mu_s + d * nu) / r_p;

		scattering = GetCombinedScattering(
			scattering_texture, single_mie_scattering_texture,
			r_p, mu_p, mu_s_p, nu, ray_r_mu_intersects_ground,
			single_mie_scattering);

		DimensionlessSpectrum shadow_transmittance =
			GetTransmittance(transmittance_texture,
				r, mu, shadow_length, ray_r_mu_intersects_ground);

		scattering = scattering * shadow_transmittance;
		single_mie_scattering = single_mie_scattering * shadow_transmittance;
	}

	return scattering * RayleighPhaseFunction(nu) + single_mie_scattering *
		MiePhaseFunction(mie_phase_function_g, nu);
}

)" R"(

/*
 * Rendering Aerial perspective
 *
 * To render the aerial perspective we need the transmittance and the scattering
 * between two points (i.e. between the viewer and a point on the ground, which can
 * at an arbibrary altitude). We already have a function to compute the
 * transmittance between two points (using 2 lookups in a texture which only
 * contains the transmittance to the top of the atmosphere), but we don't have one
 * for the scattering between 2 points. Hopefully, the scattering between 2 points
 * can be computed from two lookups in a texture which contains the scattering to
 * the nearest atmosphere boundary, as for the transmittance (except that here the
 * two lookup results must be subtracted, instead of divided). This is what we
 * implement in the following function (the initial computations are used to
 * correctly handle the case of viewers outside the atmosphere):
 */

RadianceSpectrum GetSkyRadianceToPoint(
	TransmittanceTexture transmittance_texture,
	ReducedScatteringTexture scattering_texture,
	ReducedScatteringTexture single_mie_scattering_texture,
	Position camera, Position pos, Length shadow_length,
	Direction sun_direction, out DimensionlessSpectrum transmittance)
{
	// Compute the distance to the top atmosphere boundary along the view ray,
	// assuming the viewer is in space (or NaN if the view ray does not intersect
	// the atmosphere).
	Direction view_ray = normalize(pos - camera);
	Length r = length(camera);
	Length rmu = dot(camera, view_ray);
	Length distance_to_top_atmosphere_boundary = -rmu - sqrt(rmu * rmu - r * r + top_radius * top_radius);

	// If the viewer is in space and the view ray intersects the atmosphere, move
	// the viewer to the top atmosphere boundary (along the view ray):
	if (distance_to_top_atmosphere_boundary > 0.0 * m)
	{
		camera = camera + view_ray * distance_to_top_atmosphere_boundary;
		r = top_radius;
		rmu += distance_to_top_atmosphere_boundary;
	}

	// Compute the r, mu, mu_s and nu parameters for the first texture lookup.
	Number mu = rmu / r;
	Number mu_s = dot(camera, sun_direction) / r;
	Number nu = dot(view_ray, sun_direction);
	Length d = length(pos - camera);
	bool ray_r_mu_intersects_ground = RayIntersectsGround(r, mu);

  // https://github.com/ebruneton/precomputed_atmospheric_scattering/pull/32
  // Hack to avoid rendering artifacts near the horizon, due to finite
  // atmosphere texture resolution and finite floating point precision.
  if (!ray_r_mu_intersects_ground) {
    Number mu_horiz = -SafeSqrt(1.0 - (bottom_radius / r) * (bottom_radius / r));
    mu = max(mu, mu_horiz + 0.004); 
  }


	transmittance = GetTransmittance(transmittance_texture,
		r, mu, d, ray_r_mu_intersects_ground);

	IrradianceSpectrum single_mie_scattering;
	IrradianceSpectrum scattering = GetCombinedScattering(
		scattering_texture, single_mie_scattering_texture,
		r, mu, mu_s, nu, ray_r_mu_intersects_ground,
		single_mie_scattering);

	// Compute the r, mu, mu_s and nu parameters for the second texture lookup.
	// If shadow_length is not 0 (case of light shafts), we want to ignore the
	// scattering along the last shadow_length meters of the view ray, which we
	// do by subtracting shadow_length from d (this way scattering_p is equal to
	// the S|x_s=x_0-lv term in Eq. (17) of our paper).
	d = max(d - shadow_length, 0.0 * m);
	Length r_p = ClampRadius(sqrt(d * d + 2.0 * r * mu * d + r * r));
	Number mu_p = (r * mu + d) / r_p;
	Number mu_s_p = (r * mu_s + d * nu) / r_p;

	IrradianceSpectrum single_mie_scattering_p;
	IrradianceSpectrum scattering_p = GetCombinedScattering(
		scattering_texture, single_mie_scattering_texture,
		r_p, mu_p, mu_s_p, nu, ray_r_mu_intersects_ground,
		single_mie_scattering_p);

	// Combine the lookup results to get the scattering between camera and point.
	DimensionlessSpectrum shadow_transmittance = transmittance;
	if (shadow_length > 0.0 * m)
	{
		// This is the T(x,x_s) term in Eq. (17) of our paper, for light shafts.
		shadow_transmittance = GetTransmittance(transmittance_texture, r, mu, d, ray_r_mu_intersects_ground);
	}

	scattering = scattering - shadow_transmittance * scattering_p;
	single_mie_scattering = single_mie_scattering - shadow_transmittance * single_mie_scattering_p;

#ifdef COMBINED_SCATTERING_TEXTURES
	single_mie_scattering = GetExtrapolatedSingleMieScattering(
		vec4(scattering, single_mie_scattering.r));
#endif

	// Hack to avoid rendering artifacts when the sun is below the horizon.
	single_mie_scattering = single_mie_scattering *
		smoothstep(Number(0), Number(0.01), mu_s);

	return scattering * RayleighPhaseFunction(nu) + single_mie_scattering *
		MiePhaseFunction(mie_phase_function_g, nu);
}

)" R"(

/*
 * Rendering Ground
 *
 * To render the ground we need the irradiance received on the ground after 0 or
 * more bounce(s) in the atmosphere or on the ground. The direct irradiance can be
 * computed with a lookup in the transmittance texture,
 * via GetTransmittanceToSun, while the indirect irradiance is given
 * by a lookup in the precomputed irradiance texture (this texture only contains
 * the irradiance for horizontal surfaces; we use the approximation defined in our
 * <a href="https://hal.inria.fr/inria-00288758/en">paper</a> for the other cases).
 * The function below returns the direct and indirect irradiances separately:
 */

IrradianceSpectrum GetSunAndSkyIrradiance(
	TransmittanceTexture transmittance_texture,
	IrradianceTexture irradiance_texture,
	Position pos, Direction normal, Direction sun_direction,
	out IrradianceSpectrum sky_irradiance)
{
	Length r = length(pos);
	Number mu_s = dot(pos, sun_direction) / r;

	// Indirect irradiance (approximated if the surface is not horizontal).
	sky_irradiance = GetIrradiance(irradiance_texture, r, mu_s) * (1.0 + dot(normal, pos) / r) * 0.5;

	// Direct irradiance.
	return solar_irradiance * GetTransmittanceToSun(transmittance_texture, r, mu_s) * max(dot(normal, sun_direction), 0.0);
}

)";

radiance_api = R"(

uniform sampler2D transmittance_texture;
uniform sampler2D irradiance_texture;
uniform sampler3D scattering_texture;
uniform sampler3D single_mie_scattering_texture;

// ------------------------------------------------------------------

#ifdef RADIANCE_API_ENABLED

RadianceSpectrum GetSolarRadiance() 
{
	return solar_irradiance / (PI * sun_angular_radius * sun_angular_radius);
}

// ------------------------------------------------------------------

RadianceSpectrum GetSkyRadiance(
	Position camera, Direction view_ray, Length shadow_length,
	Direction sun_direction, out DimensionlessSpectrum transmittance) 
{
	return GetSkyRadiance(transmittance_texture,
		scattering_texture, single_mie_scattering_texture,
		camera, view_ray, shadow_length, sun_direction, transmittance);
}

// ------------------------------------------------------------------

RadianceSpectrum GetSkyRadianceToPoint(
	Position camera, Position _point, Length shadow_length,
	Direction sun_direction, out DimensionlessSpectrum transmittance) 
{
	return GetSkyRadianceToPoint(transmittance_texture,
		scattering_texture, single_mie_scattering_texture,
		camera, _point, shadow_length, sun_direction, transmittance);
}

// ------------------------------------------------------------------

IrradianceSpectrum GetSunAndSkyIrradiance(
	Position p, Direction normal, Direction sun_direction,
	out IrradianceSpectrum sky_irradiance) 
{
	return GetSunAndSkyIrradiance(transmittance_texture,
		irradiance_texture, p, normal, sun_direction, sky_irradiance);
}

// ------------------------------------------------------------------

#else

Luminance3 GetSolarRadiance()
{
	return solar_irradiance /
		(PI * sun_angular_radius * sun_angular_radius) *
		SUN_SPECTRAL_RADIANCE_TO_LUMINANCE;
}

// ------------------------------------------------------------------

Luminance3 GetSkyRadiance(
	Position camera, Direction view_ray, Length shadow_length,
	Direction sun_direction, out DimensionlessSpectrum transmittance) 
{
	return GetSkyRadiance(transmittance_texture,
		scattering_texture, single_mie_scattering_texture,
		camera, view_ray, shadow_length, sun_direction, transmittance) *
		SKY_SPECTRAL_RADIANCE_TO_LUMINANCE;
}

// ------------------------------------------------------------------

Luminance3 GetSkyRadianceToPoint(
	Position camera, Position _point, Length shadow_length,
	Direction sun_direction, out DimensionlessSpectrum transmittance) 
{
	return GetSkyRadianceToPoint(transmittance_texture,
		scattering_texture, single_mie_scattering_texture,
		camera, _point, shadow_length, sun_direction, transmittance) *
		SKY_SPECTRAL_RADIANCE_TO_LUMINANCE;
}

// ------------------------------------------------------------------

Illuminance3 GetSunAndSkyIrradiance(
	Position p, Direction normal, Direction sun_direction,
	out IrradianceSpectrum sky_irradiance) 
{
	IrradianceSpectrum sun_irradiance = GetSunAndSkyIrradiance(
		transmittance_texture, irradiance_texture, p, normal,
		sun_direction, sky_irradiance);
	sky_irradiance *= SKY_SPECTRAL_RADIANCE_TO_LUMINANCE;
	return sun_irradiance * SUN_SPECTRAL_RADIANCE_TO_LUMINANCE;
}

#endif
)";
