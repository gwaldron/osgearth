#version 400 compatibility

#pragma vp_entryPoint "oe_grass_geom"
#pragma vp_location   "geometry"
                
layout(triangles)        in;        // triangles from the TileDrawable
layout(triangle_strip)   out;       // output a triangle-strip billboard
layout(max_vertices = 4) out;       // four verts per billboard
                
// Internal helper functions:
void VP_LoadVertex(in int);
void VP_EmitViewVertex();

uniform float osg_FrameTime;
uniform vec4  oe_tile_key;
                
uniform float oe_grass_lod            = 21.0;
uniform float oe_grass_width          = 0.15;
uniform float oe_grass_height         = 0.25;
uniform float oe_grass_noise          = 1.0;
uniform float oe_grass_ao             = 0.0;   // ambient occlusion of ground verts (0=full)
uniform float oe_grass_colorVariation = 0.0;

uniform float oe_grass_windFactor  = 0.0;
uniform float oe_grass_maxDistance = 25.0;

uniform sampler2D oe_tile_elevationTex;
uniform mat4      oe_tile_elevationTexMatrix;

uniform sampler2D mask_tex;
uniform mat4      mask_texMatrix;

uniform sampler2D oe_noise_tex;

in vec4 oe_layer_tilec;

// Output grass texture coordinates to the fragment shader
out vec2 oe_grass_texCoord;

// Output a falloff metric to the fragment shader for distance blending
out float oe_grass_falloff;

// Output color variations:
out vec4 vp_Color;
out vec3 vp_Normal;

// Up vector for clamping.
in vec3 oe_UpVectorView;  


void
oe_grass_clamp(inout vec4 vert_view, in vec3 up, vec2 UV)
{
    // Sample the elevation texture and move the vertex accordingly.
    vec4 elevc = oe_tile_elevationTexMatrix * vec4(UV, 0.0, 1.0);
    float elev = texture(oe_tile_elevationTex, elevc.st).r;
    vert_view.xyz += up*elev;
}


// Generate a pseudo-random value in the specified range:
float
oe_grass_rangeRand(float minValue, float maxValue, vec2 co)
{
    float t = fract(sin(dot(co.xy ,vec2(12.9898,78.233))) * 43758.5453);
    return minValue + t*(maxValue-minValue);
}

void
oe_grass_jitterVertex(inout vec4 vertex, inout vec2 tileCoord, float delta, float tileCoordRes)
{
    vec2 pos0 = vertex.xy;
    
    vec2 dxy = vec2(
        oe_grass_rangeRand(-delta, delta, tileCoord),
        oe_grass_rangeRand(-delta, delta, vertex.xy) );
        
    vertex.xy += dxy;
    tileCoord = clamp(tileCoord + dxy*tileCoordRes, 0.0, 1.0);
}

float
oe_grass_applyWind(float time, float factor, float randOffset)
{
   return sin(time + randOffset) * factor;
}

vec3
oe_grass_getRandomBarycentricPoint(vec2 seed)
{
    // compute random barycentric coordinates.
    vec3 b;
    b[0] = oe_grass_rangeRand(0.0, 1.0, seed.xy);
    b[1] = oe_grass_rangeRand(0.0, 1.0, seed.yx);
    if (b[0]+b[1] >= 1.0)
    {
        b[0] = 1.0 - b[0];
        b[1] = 1.0 - b[1];
    }
    b[2] = 1.0 - b[0] - b[1];
    return b;
}
                                
void
oe_grass_geom()
{
    vec4 center = vec4(0,0,0,1);
    vec2 tileUV = vec2(0,0);
    
    // get a random point within the input triangle
    vec3 b = oe_grass_getRandomBarycentricPoint(gl_in[0].gl_Position.xy);
    
    // Load the triangle data and compute the new position and tile coords
    // using the barycentric coordinates.
    for(int i=0; i < 3; ++i)
    {
        VP_LoadVertex(i);      
        
        center.x += b[i] * gl_in[i].gl_Position.x;
        center.y += b[i] * gl_in[i].gl_Position.y;
        center.z += b[i] * gl_in[i].gl_Position.z;
        
        tileUV.x += b[i] * oe_layer_tilec.x;
        tileUV.y += b[i] * oe_layer_tilec.y;
    } 
    
    // Transform to view space.
    vec4 center_view = gl_ModelViewMatrix * center;
    vec3 up_view     = oe_UpVectorView;
    
    // Clamp the center point to the elevation.
    oe_grass_clamp(center_view, up_view, tileUV);
    
    // calculate the normalized camera range:
    float nRange = clamp(-center_view.z/oe_grass_maxDistance, 0.0, 1.0);
    
    // sample the noise texture.
    float n = texture(oe_noise_tex, tileUV*oe_grass_noise).r;

    float width = oe_grass_width;
	
    // push the falloff closer to the max distance.
    float falloff = 1.0-(nRange*nRange*nRange);
    
    // vary the height of each instance and shrink it as it disappears into the distance.
    float height = oe_grass_height;
    height *= abs(1.0+n);
    height *= falloff;
    
    // Tell the fragment shader to blend into the distance.
    oe_grass_falloff = nRange;

	// compute the grass vertices in view space.
    vec4 newVerts[4];
    
    const vec3 tangent_view = vec3(1,0,0); // assuming no roll.
    
    newVerts[0] = vec4(center_view.xyz - tangent_view*width*0.5, 1.0);
    newVerts[1] = vec4(center_view.xyz + tangent_view*width*0.5, 1.0);
    newVerts[2] = vec4(newVerts[0].xyz + up_view*height, 1.0);
    newVerts[3] = vec4(newVerts[1].xyz + up_view*height, 1.0);
                      
    // TODO: animate based on wind parameters.
    newVerts[2].xyz += tangent_view * oe_grass_applyWind(osg_FrameTime*(1+n), oe_grass_width*oe_grass_windFactor*n, newVerts[2].x);
    newVerts[3].xyz += tangent_view * oe_grass_applyWind(osg_FrameTime*(1-n), oe_grass_width*oe_grass_windFactor*n, tileUV.t);
    
    // Color variation
    float cv = clamp(n, 1.0-oe_grass_colorVariation, 1.0);

    vec3 normal = vec3(0,0,1);
    normal.xy += vec2(oe_grass_rangeRand(-0.25, 0.25, vec2(n)));
    vp_Normal = normalize(gl_NormalMatrix * normal);
    
    vp_Color = vec4(cv*oe_grass_ao, cv*oe_grass_ao, cv*oe_grass_ao, falloff);
    gl_Position = newVerts[0];
    oe_grass_texCoord = vec2(0,0);
    VP_EmitViewVertex();
    
    gl_Position = newVerts[1];
    oe_grass_texCoord = vec2(1,0);
    VP_EmitViewVertex();

    vp_Color = vec4(cv,cv,cv,falloff);      
    gl_Position = newVerts[2];
    oe_grass_texCoord = vec2(0,1);
    VP_EmitViewVertex();

    oe_grass_texCoord = vec2(1,1);
    gl_Position = newVerts[3];
    VP_EmitViewVertex();
                    
    EndPrimitive();
}
