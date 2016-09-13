#version $GLSL_VERSION_STR

//
// Description : Array and textureless GLSL 2D/3D/4D simplex 
//               noise functions.
//      Author : Ian McEwan, Ashima Arts.
//  Maintainer : ijm
//     Lastmod : 20110822 (ijm)
//     License : Copyright (C) 2011 Ashima Arts. All rights reserved.
//               Distributed under the MIT License. See LICENSE file.
//               https://github.com/ashima/webgl-noise
// 

vec4 oe_noise_mod289(vec4 x) {
  return x - floor(x * (1.0 / 289.0)) * 289.0; }

float oe_noise_mod289(float x) {
  return x - floor(x * (1.0 / 289.0)) * 289.0; }

vec4 oe_noise_permute(vec4 x) {
     return oe_noise_mod289(((x*34.0)+1.0)*x);
}

float oe_noise_permute(float x) {
     return oe_noise_mod289(((x*34.0)+1.0)*x);
}

vec4 oe_noise_taylorInvSqrt(vec4 r)
{
  return 1.79284291400159 - 0.85373472095314 * r;
}

float oe_noise_taylorInvSqrt(float r)
{
  return 1.79284291400159 - 0.85373472095314 * r;
}

vec4 oe_noise_grad4(float j, vec4 ip)
  {
  const vec4 ones = vec4(1.0, 1.0, 1.0, -1.0);
  vec4 p,s;

  p.xyz = floor( fract (vec3(j) * ip.xyz) * 7.0) * ip.z - 1.0;
  p.w = 1.5 - dot(abs(p.xyz), ones.xyz);
  s = vec4(lessThan(p, vec4(0.0)));
  p.xyz = p.xyz + (s.xyz*2.0 - 1.0) * s.www; 

  return p;
  }
						
// (sqrt(5) - 1)/4 = F4, used once below
#define oe_noise_F4 0.309016994374947451

float oe_noise_snoise(vec4 v)
  {
  const vec4  C = vec4( 0.138196601125011,  // (5 - sqrt(5))/20  G4
                        0.276393202250021,  // 2 * G4
                        0.414589803375032,  // 3 * G4
                       -0.447213595499958); // -1 + 4 * G4

// First corner
  vec4 i  = floor(v + dot(v, vec4(oe_noise_F4)) );
  vec4 x0 = v -   i + dot(i, C.xxxx);

// Other corners

// Rank sorting originally contributed by Bill Licea-Kane, AMD (formerly ATI)
  vec4 i0;
  vec3 isX = step( x0.yzw, x0.xxx );
  vec3 isYZ = step( x0.zww, x0.yyz );
//  i0.x = dot( isX, vec3( 1.0 ) );
  i0.x = isX.x + isX.y + isX.z;
  i0.yzw = 1.0 - isX;
//  i0.y += dot( isYZ.xy, vec2( 1.0 ) );
  i0.y += isYZ.x + isYZ.y;
  i0.zw += 1.0 - isYZ.xy;
  i0.z += isYZ.z;
  i0.w += 1.0 - isYZ.z;

  // i0 now contains the unique values 0,1,2,3 in each channel
  vec4 i3 = clamp( i0, 0.0, 1.0 );
  vec4 i2 = clamp( i0-1.0, 0.0, 1.0 );
  vec4 i1 = clamp( i0-2.0, 0.0, 1.0 );

  //  x0 = x0 - 0.0 + 0.0 * C.xxxx
  //  x1 = x0 - i1  + 1.0 * C.xxxx
  //  x2 = x0 - i2  + 2.0 * C.xxxx
  //  x3 = x0 - i3  + 3.0 * C.xxxx
  //  x4 = x0 - 1.0 + 4.0 * C.xxxx
  vec4 x1 = x0 - i1 + C.xxxx;
  vec4 x2 = x0 - i2 + C.yyyy;
  vec4 x3 = x0 - i3 + C.zzzz;
  vec4 x4 = x0 + C.wwww;

// Permutations
  i = oe_noise_mod289(i); 
  float j0 = oe_noise_permute( oe_noise_permute( oe_noise_permute( oe_noise_permute(i.w) + i.z) + i.y) + i.x);
  vec4 j1 = oe_noise_permute( oe_noise_permute( oe_noise_permute( oe_noise_permute (
             i.w + vec4(i1.w, i2.w, i3.w, 1.0 ))
           + i.z + vec4(i1.z, i2.z, i3.z, 1.0 ))
           + i.y + vec4(i1.y, i2.y, i3.y, 1.0 ))
           + i.x + vec4(i1.x, i2.x, i3.x, 1.0 ));

// Gradients: 7x7x6 points over a cube, mapped onto a 4-cross polytope
// 7*7*6 = 294, which is close to the ring size 17*17 = 289.
  vec4 ip = vec4(1.0/294.0, 1.0/49.0, 1.0/7.0, 0.0) ;

  vec4 p0 = oe_noise_grad4(j0,   ip);
  vec4 p1 = oe_noise_grad4(j1.x, ip);
  vec4 p2 = oe_noise_grad4(j1.y, ip);
  vec4 p3 = oe_noise_grad4(j1.z, ip);
  vec4 p4 = oe_noise_grad4(j1.w, ip);

// Normalise gradients
  vec4 norm = oe_noise_taylorInvSqrt(vec4(dot(p0,p0), dot(p1,p1), dot(p2, p2), dot(p3,p3)));
  p0 *= norm.x;
  p1 *= norm.y;
  p2 *= norm.z;
  p3 *= norm.w;
  p4 *= oe_noise_taylorInvSqrt(dot(p4,p4));

// Mix contributions from the five corners
  vec3 m0 = max(0.6 - vec3(dot(x0,x0), dot(x1,x1), dot(x2,x2)), 0.0);
  vec2 m1 = max(0.6 - vec2(dot(x3,x3), dot(x4,x4)            ), 0.0);
  m0 = m0 * m0;
  m1 = m1 * m1;
  return 49.0 * ( dot(m0*m0, vec3( dot( p0, x0 ), dot( p1, x1 ), dot( p2, x2 )))
               + dot(m1*m1, vec2( dot( p3, x3 ), dot( p4, x4 ) ) ) ) ;
}

// Generates a tiled fractal simplex noise value and clamps the values to [0..1].
float oe_noise_fractal4D(in vec2 seed, in float frequency, in float persistence, in float lacunarity, in int octaves)
{
    const float TwoPI = 6.283185;
	float f = frequency;
	float amp = 1.0;
	float maxAmp = 0.0;
	float n = 0.0;
    
    vec4 seed4D;
    seed4D.xy = cos(seed*TwoPI)/TwoPI;
    seed4D.zw = sin(seed*TwoPI)/TwoPI;

	for(int i=0; i<octaves; ++i)
	{
		n += oe_noise_snoise(seed4D*f) * amp;
		maxAmp += amp;
		amp *= persistence;
		f *= lacunarity;
	}
	//return n / maxAmp;
    const float low = 0.0;
    const float high = 1.0;

    n /= maxAmp;
    n = n * (high-low)/2.0 + (high+low)/2.0;
    return clamp(n, 0.0, 1.0);
}
