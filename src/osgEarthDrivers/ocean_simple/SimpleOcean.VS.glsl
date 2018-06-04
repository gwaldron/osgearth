#version $GLSL_VERSION_STR
$GLSL_DEFAULT_PRECISION_FLOAT

#pragma vp_name       SimpleOcean with Proxy VS
#pragma vp_entryPoint oe_ocean_vertex
#pragma vp_location   vertex_view

#pragma import_defines(OE_SIMPLE_OCEAN_USE_TEXTURE)
#pragma import_defines(OE_SIMPLE_OCEAN_USE_MASK)

// uniforms
uniform mat4 osg_ViewMatrixInverse;  
uniform float osg_FrameTime;  
uniform float ocean_seaLevel;                 // sea level offset

// outputs to fragment stage 
out float ocean_v_msl;                      // elevation (MSL) of camera
out float ocean_v_range;                    // distance from camera to current vertex

#ifdef OE_SIMPLE_OCEAN_USE_TEXTURE
out vec4 ocean_surface_tex_coord;       // tex coords for surface texture

// convert an ecef coordinate to lon/lat (low precision)
vec2 ocean_xyz_to_spherical(in vec3 xyz)  
{  
    float r = length(xyz);  
    float lat = acos(xyz.z/r);  
    float lon = atan(xyz.y, xyz.x);  
    return vec2(lon,lat);  
}  

#endif


// stage global
vec3 vp_Normal;


#ifdef OE_SIMPLE_OCEAN_USE_MASK
out vec4 ocean_mask_tex_coord;

// Ocean mask version:
void oe_ocean_vertex(inout vec4 VertexMODEL)  
{  
    // adjust our vert for the sea level - extrude along the normal vector 
    // (this must be done in modelview space to preserve precision)
    vec4 mvVertex = VertexMODEL;  
    vec3 mvNormal = gl_NormalMatrix * gl_Normal;  
    vec4 mvVertex2 = vec4(mvVertex.xyz + (mvNormal * ocean_seaLevel), mvVertex.w );  

    VertexMODEL = mvVertex2;  

    // ocean mask texture coordinate:
    ocean_mask_tex_coord = gl_MultiTexCoord0;  

    // send interpolated params to the fs:
    vec4 eye = osg_ViewMatrixInverse * vec4(0,0,0,1);  

    // height of camera above sea level:
    ocean_v_msl = length(eye.xyz/eye.w) - 6378137.0 + ocean_seaLevel;  

    // disatnce to camera:
    ocean_v_range = ocean_v_msl;  
    
#ifdef OE_SIMPLE_OCEAN_USE_TEXTURE
    // scale the texture mapping to something reasonable:
    vec4 worldVertex = osg_ViewMatrixInverse * mvVertex;  
    vec2 lonlat = ocean_xyz_to_spherical( worldVertex.xyz/worldVertex.w );  
    ocean_surface_tex_coord.xy = lonlat/0.0005;
    ocean_surface_tex_coord.zw = ocean_surface_tex_coord.xy;  
    ocean_surface_tex_coord.w -= mod(0.1*osg_FrameTime,25.0)/25.0; 
#endif
}

#else

uniform sampler2D oe_ocean_proxyTex; // heightfield encoded into 16 bit texture
uniform mat4 oe_ocean_proxyMat;      // texture matrix for elevation data
vec4 oe_layer_tilec;                // stage global tile coordinates
out float ocean_terrainHeight;      // terrain height at proxy Map vertex.

// Proxy layer version:
void oe_ocean_vertex(inout vec4 VertexVIEW)  
{  
    // adjust our vert for the sea level - extrude along the normal vector 
    // (this must be done in view space to preserve precision)
    vec4 mvVertex = VertexVIEW;  
    vec3 mvNormal = vp_Normal;  
    vec4 mvVertex2 = vec4(mvVertex.xyz + (mvNormal * ocean_seaLevel), mvVertex.w );  

    VertexVIEW = mvVertex2;  

    // read elevation data from proxy height texture
    ocean_terrainHeight = texture(oe_ocean_proxyTex, (oe_ocean_proxyMat*oe_layer_tilec).st).r;

    // send interpolated params to the fs:
    vec4 eye = osg_ViewMatrixInverse * vec4(0,0,0,1);  

    // height of camera above sea level:
    ocean_v_msl = length(eye.xyz/eye.w) - 6378137.0 + ocean_seaLevel;  

    // disatnce to camera:
    ocean_v_range = ocean_v_msl;  

#ifdef OE_SIMPLE_OCEAN_USE_TEXTURE
    // scale the texture mapping to something reasonable:
    vec4 worldVertex = osg_ViewMatrixInverse * mvVertex;  
    vec2 lonlat = ocean_xyz_to_spherical( worldVertex.xyz/worldVertex.w );  
    ocean_surface_tex_coord.xy = lonlat/0.0005;
    ocean_surface_tex_coord.zw = ocean_surface_tex_coord.xy;  
    ocean_surface_tex_coord.w -= mod(0.1*osg_FrameTime,25.0)/25.0; 
#endif
 }

#endif
