//
//
//  Noise sandbox.
//  Run with
//  oe r.earth --uniform freq 64 16 --uniform amplitude 1 50 --uniform pers 0.5 0.9 --uniform lac 2.0 3.0 --uniform baseLOD 11 12 --uniform slopeAmp 0 1 --uniform curvatureAmp 0 1 --uniform threshold -1 1 --sky --coords --activity
//
//
#include <osgViewer/Viewer>
#include <osgEarth/Notify>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>
#include <osgEarth/VirtualProgram>
#include <osgEarth/MapNode>
#include <osgEarth/TerrainEngineNode>

const char* noiseShader = OE_MULTILINE(

vec4 mod289(vec4 x) {
  return x - floor(x * (1.0 / 289.0)) * 289.0; }

float mod289(float x) {
  return x - floor(x * (1.0 / 289.0)) * 289.0; }

vec4 permute(vec4 x) {
     return mod289(((x*34.0)+1.0)*x);
}

float permute(float x) {
     return mod289(((x*34.0)+1.0)*x);
}

vec4 taylorInvSqrt(vec4 r)
{
  return 1.79284291400159 - 0.85373472095314 * r;
}

float taylorInvSqrt(float r)
{
  return 1.79284291400159 - 0.85373472095314 * r;
}

vec4 grad4(float j, vec4 ip)
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
const float F4 = 0.309016994374947451;

float snoise(vec4 v)
  {
  const vec4  C = vec4( 0.138196601125011,  // (5 - sqrt(5))/20  G4
                        0.276393202250021,  // 2 * G4
                        0.414589803375032,  // 3 * G4
                       -0.447213595499958); // -1 + 4 * G4

// First corner
  vec4 i  = floor(v + dot(v, vec4(F4)) );
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
  i = mod289(i); 
  float j0 = permute( permute( permute( permute(i.w) + i.z) + i.y) + i.x);
  vec4 j1 = permute( permute( permute( permute (
             i.w + vec4(i1.w, i2.w, i3.w, 1.0 ))
           + i.z + vec4(i1.z, i2.z, i3.z, 1.0 ))
           + i.y + vec4(i1.y, i2.y, i3.y, 1.0 ))
           + i.x + vec4(i1.x, i2.x, i3.x, 1.0 ));

// Gradients: 7x7x6 points over a cube, mapped onto a 4-cross polytope
// 7*7*6 = 294, which is close to the ring size 17*17 = 289.
  vec4 ip = vec4(1.0/294.0, 1.0/49.0, 1.0/7.0, 0.0) ;

  vec4 p0 = grad4(j0,   ip);
  vec4 p1 = grad4(j1.x, ip);
  vec4 p2 = grad4(j1.y, ip);
  vec4 p3 = grad4(j1.z, ip);
  vec4 p4 = grad4(j1.w, ip);

// Normalise gradients
  vec4 norm = taylorInvSqrt(vec4(dot(p0,p0), dot(p1,p1), dot(p2, p2), dot(p3,p3)));
  p0 *= norm.x;
  p1 *= norm.y;
  p2 *= norm.z;
  p3 *= norm.w;
  p4 *= taylorInvSqrt(dot(p4,p4));

// Mix contributions from the five corners
  vec3 m0 = max(0.6 - vec3(dot(x0,x0), dot(x1,x1), dot(x2,x2)), 0.0);
  vec2 m1 = max(0.6 - vec2(dot(x3,x3), dot(x4,x4)            ), 0.0);
  m0 = m0 * m0;
  m1 = m1 * m1;
  return 49.0 * ( dot(m0*m0, vec3( dot( p0, x0 ), dot( p1, x1 ), dot( p2, x2 )))
               + dot(m1*m1, vec2( dot( p3, x3 ), dot( p4, x4 ) ) ) ) ;

  }

float oe_noise_fractal4(in vec4 seed, in float F, in float P, in float L, in int octaves)
{
    float amp = 1.0;
    float maxamp = 0.0;
    float n = 0.0;

    for(int i=0; i<octaves; ++i)
    {
        n += snoise(vec4(seed.x*F, seed.y*F, seed.z*F, seed.w*F)) * amp;
        maxamp += amp;
        amp *= P;
        F *= L;
    }
    return n;
}

);


const char* vertex = OE_MULTILINE(
// SDK functions:
float oe_terrain_getElevation(in vec2 uv);
vec2 oe_terrain_scaleCoordsToRefLOD(in vec2 tc, in float refLOD);
vec4 oe_terrain_getNormalAndCurvature(in vec2 tc);
float oe_noise_fractal4(in vec4 seed, in float freq, in float pers, in float lac, in int octaves);

vec3 oe_UpVectorView;
vec4 oe_layer_tilec;

uniform vec4 oe_tile_key;

uniform float freq;
uniform float pers;
uniform float lac;
uniform float slopeAmp;
uniform float curvatureAmp;
uniform float baseLOD;
uniform float amplitude;
uniform float threshold;
uniform float DD;

in vec2 oe_normalMapCoords;
in vec3 oe_normalMapBinormal;

out vec4 v;

void oe_temp_vertex(inout vec4 vertexView)
{
    float b = floor(baseLOD);
    vec2 a = oe_terrain_scaleCoordsToRefLOD(oe_layer_tilec.st, b);
    
    const float TwoPI = 6.28318530718;

    vec4 seed = vec4( cos(a.x*TwoPI)/TwoPI, 
                      cos(a.y*TwoPI)/TwoPI,
                      sin(a.x*TwoPI)/TwoPI,
                      sin(a.y*TwoPI)/TwoPI );

    
    vec4 encodedNormal = oe_terrain_getNormalAndCurvature(oe_normalMapCoords);
    vec3 normal = normalize(encodedNormal.xyz*2.0-1.0);

    float slope = 2.0*(1.0-dot(normal.xyz, vec3(0,0,1))); // 0=flat, 1=vertical

    int octaves = min(12.0, oe_tile_key.z-b);
    float n = oe_noise_fractal4(seed, freq, pers, lac, octaves);

    //n = n*2.0-1.0;
    //v = vertexView;

    //n = max(n, threshold);
    
    if (n > threshold) 
    {
    float curvature = 2.0*clamp(encodedNormal.w-0.5, 0.0, 1.0) * curvatureAmp;
    float slopeAndCurv = clamp(slope+curvature, 0.0, 1.0);
    
    vec3 tangent = normalize(cross(oe_normalMapBinormal, oe_UpVectorView));
    vec3 realNormal = normalize( mat3(tangent, oe_normalMapBinormal, oe_UpVectorView) * normal );

    float finalAmp = mix(amplitude, amplitude*slopeAndCurv, slopeAmp);

    //vertexView.xyz += oe_UpVectorView * (n*finalAmp);
    vertexView.xyz += realNormal * (n*finalAmp);
    }

    v = vertexView;
}

);

const char* fragment = OE_MULTILINE(

// SDK functions:
float oe_terrain_getElevation(in vec2 uv);
vec2 oe_terrain_scaleCoordsToRefLOD(in vec2 tc, in float refLOD);
vec4 oe_terrain_getNormalAndCurvature(in vec2 tc);
float oe_noise_fractal4(in vec4 seed, in float freq, in float pers, in float lac, in int octaves);

vec3 oe_UpVectorView;
vec4 oe_layer_tilec;
vec3 vp_Normal;

in float slope;

uniform vec4 oe_tile_key;

uniform float freq;
uniform float pers;
uniform float lac;
uniform float slopeAmp;
uniform float curvatureAmp;
uniform float baseLOD;
uniform float amplitude;
uniform float threshold;

in vec2 oe_normalMapCoords;
in vec3 oe_normalMapBinormal;

in vec4 v;

void oe_temp_fragment(inout vec4 color)
{
    float b = floor(baseLOD);
    vec2 a = oe_terrain_scaleCoordsToRefLOD(oe_layer_tilec.st, b);
    //
    const float TwoPI = 6.28318530718;

    vec4 seed = vec4( cos(a.x*TwoPI)/TwoPI, 
                      cos(a.y*TwoPI)/TwoPI,
                      sin(a.x*TwoPI)/TwoPI,
                      sin(a.y*TwoPI)/TwoPI );
    //
    vec4 encodedNormal = oe_terrain_getNormalAndCurvature(oe_normalMapCoords);
    vec3 normal = normalize(encodedNormal.xyz*2.0-1.0);
    
    vec3 vx = dFdx(v.xyz);
    vec3 vy = dFdy(v.xyz);
    vec3 vn = normalize(cross(vx, vy));
    //float slope = min(maxSlope, 1.0 - dot(vn, oe_UpVectorView));

    float slope = 2.0*(1.0-dot(normal.xyz, vec3(0,0,1))); // 0=flat, 1=vertical
    
    int octaves = min(12.0, oe_tile_key.z-b);
    float n = oe_noise_fractal4(seed, freq, pers, lac, octaves);
      
    //n = n*2.0-1.0;

    //n = max(n, threshold);
    
    if (n > threshold)
    {
    vec3 tangent = normalize(cross(oe_normalMapBinormal, oe_UpVectorView));
    vec3 realNormal = normalize( mat3(tangent, oe_normalMapBinormal, oe_UpVectorView) * normal );
    
    float curvature = 2.0*clamp(encodedNormal.w-0.5, 0.0, 1.0) * curvatureAmp;
    float slopeAndCurv = clamp(slope+curvature, 0.0, 1.0);
    float finalAmp = mix(amplitude, amplitude*slope, slopeAmp);

    //vec3 v2 = v.xyz + oe_UpVectorView * (n*finalAmp);
    vec3 v2 = v.xyz + realNormal * (n*finalAmp);

    vx = dFdx(v2.xyz);
    vy = dFdy(v2.xyz);
    vp_Normal = normalize(cross(vx,vy));
    }

    // visualize noise
    //color.rgb = vec3(n,n,n);
}
    );

#define LC "[viewer] "

using namespace osgEarth;
using namespace osgEarth::Util;

int
usage(const char* name)
{
    OE_NOTICE 
        << "\nUsage: " << name << " file.earth" << std::endl
        << MapNodeHelper().usage() << std::endl;

    return 0;
}

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    // help?
    if ( arguments.read("--help") )
        return usage(argv[0]);

    float vfov = -1.0f;
    arguments.read("--vfov", vfov);

    // create a viewer:
    osgViewer::Viewer viewer(arguments);

    // Tell the database pager to not modify the unref settings
    viewer.getDatabasePager()->setUnrefImageDataAfterApplyPolicy( true, false );

    // thread-safe initialization of the OSG wrapper manager. Calling this here
    // prevents the "unsupported wrapper" messages from OSG
    osgDB::Registry::instance()->getObjectWrapperManager()->findWrapper("osg::Image");

    // install our default manipulator (do this before calling load)
    viewer.setCameraManipulator( new EarthManipulator(arguments) );

    // disable the small-feature culling
    viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);

    // set a near/far ratio that is smaller than the default. This allows us to get
    // closer to the ground without near clipping. If you need more, use --logdepth
    viewer.getCamera()->setNearFarRatio(0.0001);

    if ( vfov > 0.0 )
    {
        double fov, ar, n, f;
        viewer.getCamera()->getProjectionMatrixAsPerspective(fov, ar, n, f);
        viewer.getCamera()->setProjectionMatrixAsPerspective(vfov, ar, n, f);
    }

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags    
    osg::Node* node = MapNodeHelper().load(arguments, &viewer);
    if ( node )
    {
        MapNode* m = MapNode::get(node);
        
        VirtualProgram* vp = VirtualProgram::getOrCreate(m->getTerrainEngine()->getOrCreateStateSet());
        
        osg::Shader* noiseVS = new osg::Shader(osg::Shader::VERTEX, noiseShader);
        vp->setShader("noiseVS", noiseVS);
        
        osg::Shader* noiseFS = new osg::Shader(osg::Shader::FRAGMENT, noiseShader);
        vp->setShader("noiseFS", noiseFS);

        vp->setFunction("oe_temp_vertex", vertex, ShaderComp::LOCATION_VERTEX_VIEW);

        vp->setFunction("oe_temp_fragment", fragment, ShaderComp::LOCATION_FRAGMENT_COLORING);

        viewer.setSceneData( node );
        viewer.run();
    }
    else
    {
        return usage(argv[0]);
    }
}