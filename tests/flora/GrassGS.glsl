#version 400 compatibility

#pragma vp_entryPoint "oe_grass_geom"
#pragma vp_location   "geometry"
                
layout(triangles) in;
layout(triangle_strip) out;
layout(max_vertices = 4) out;
                
// Internal helper functions:
void VP_LoadVertex(in int);
void VP_EmitViewVertex();

uniform float osg_FrameTime;
uniform vec4  oe_tile_key;
                
uniform float oe_grass_lod         = 21.0;
uniform float oe_grass_width       = 0.15;
uniform float oe_grass_height      = 0.25;
uniform float oe_grass_noise       = 1.0;

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

// Output color variations:
out vec4 vp_Color;  
out vec3 vp_UpVector;       

// Output normals:         
out vec3 vp_Normal;


void oe_grass_clamp(inout vec4 vert_view, in vec3 up)
{
    // Sample the elevation texture and move the vertex accordingly.
    vec4 elevc = oe_tile_elevationTexMatrix * oe_layer_tilec;
    float elev = texture(oe_tile_elevationTex, elevc.st).r;
    vert_view.xyz += up*elev;
}


// Generate a pseudo-random value in the specified range:
float oe_grass_rangeRand(float minValue, float maxValue, vec2 co)
{
    float t = fract(sin(dot(co.xy ,vec2(12.9898,78.233))) * 43758.5453);
    return minValue + t*(maxValue-minValue);
}

float JitterHeight(float fHeight, float fDelta, vec2 uv)
{
	return fHeight + oe_grass_rangeRand(-fDelta, +fDelta, uv);
}

vec2 JitterPosition(vec2 position, vec2 vDelta, vec2 uv)
{    
    float x = oe_grass_rangeRand(-vDelta.x, vDelta.x, uv);
    float y = oe_grass_rangeRand(-vDelta.y, vDelta.y, position.xy);
	return position + vec2(x, y);
}

void
oe_grass_jitterVertex(inout vec4 vertex, float delta, vec2 seed)
{
    vec2 pos0 = vertex.xy;
    vertex.x += oe_grass_rangeRand(-delta, delta, seed);
    vertex.y += oe_grass_rangeRand(-delta, delta, pos0);
}

float oe_grass_applyWind(float time, float factor, float randOffset)
{
   return sin(time + randOffset) * factor;
}
                                
void oe_grass_geom()
{    
    float lod = floor(oe_grass_lod);
    
    // Bail out if we're not at the correct LOD.
    if ( oe_tile_key.z != lod )
        return;
    
    vec4 positions[3];
    vec3 normals[3];
	vec2 tileCoords[3];
	vec3 vMask;
    
    // Load the triangle data:
    for(int i=0; i < 3; ++i)
    {
        positions[i] = gl_in[i].gl_Position;
        VP_LoadVertex(i);
        
        // Sample the mask texture
		//vMask[i] = texture(mask_tex, (mask_texMatrix*oe_layer_tilec).st).a;
        
        normals[i] = vp_Normal;
		tileCoords[i] = oe_layer_tilec.xy;
    }

    // Check the mask and bail if we are outside:
    //if ( (vMask.r+vMask.g+vMask.b)*0.33333 < 0.2 )
		//return;
    
    //float maxOffset = oe_tile_key.w * 0.1;    
                    
    // Center of the triangle:
    vec4 center_model = (positions[0] + positions[1] + positions[2])*0.33333;
    
    float maxOffset = distance(center_model, positions[0]);

    // Randomly alter the position    
    oe_grass_jitterVertex(center_model, maxOffset, oe_layer_tilec.xy);
    
    // Transform to view space.
    vec4 center_view = gl_ModelViewMatrix * center_model;
    vec3 up_view     = gl_NormalMatrix * vp_UpVector;
    
    // Clamp the ceneter point to the elevation.
    oe_grass_clamp(center_view, up_view);
    
    float range  = -center_view.z;
    
    // cull points that aren't in view:
    if ( range > oe_grass_maxDistance || range < 0 )
        return;
        
    // calcluate the normalized camera range:
    float nRange = clamp(range/oe_grass_maxDistance, 0.0, 1.0);
    
    // Modulate the width based on distance.
    float width = oe_grass_width + nRange*(oe_grass_width*8 - oe_grass_width);
    
    // Modulate the height and width based on the noise function.
    float n = texture(oe_noise_tex, oe_layer_tilec.st*oe_grass_noise).r;
    float height = oe_grass_height * (1.0+n);
    width = max(width, width+width*n*2);
	
    float falloff = nRange*nRange*nRange;
    float alpha = 1.0-falloff; // push the falloff closer to the max distance.

	// compute the grass vertices in view space.
    vec4 newVerts[4];
    
    vec3 tangent_view = vec3(1,0,0); // assuming no roll.
    
    newVerts[0] = vec4(center_view.xyz - tangent_view*width*0.5, 1.0);
    newVerts[1] = vec4(center_view.xyz + tangent_view*width*0.5, 1.0);
    newVerts[2] = vec4(newVerts[0].xyz + up_view*height, 1.0);
    newVerts[3] = vec4(newVerts[1].xyz + up_view*height, 1.0);
                      
    //TODO: animate based on wind parameters.
    newVerts[2].xyz += tangent_view * oe_grass_applyWind(osg_FrameTime*(1+n), oe_grass_width*oe_grass_windFactor*n, newVerts[2].x);
    newVerts[3].xyz += tangent_view * oe_grass_applyWind(osg_FrameTime*(1+n), oe_grass_width*oe_grass_windFactor*n, oe_layer_tilec.y);

    vec3 normal = vec3(0,0,1);
    normal.xy += vec2(oe_grass_rangeRand(-0.25, 0.25, vec2(n)));
    vp_Normal = normalize(gl_NormalMatrix * normal);

    //vp_Normal = blade_facing_vector;
    
    vp_Color = vec4(0,0,0,alpha);
    gl_Position = newVerts[0];
    oe_grass_texCoord = vec2(0,0);
    VP_EmitViewVertex();
    
    gl_Position = newVerts[1];
    oe_grass_texCoord = vec2(1,0);
    VP_EmitViewVertex();

    vp_Color = vec4(1,1,1,alpha);      
    gl_Position = newVerts[2];
    oe_grass_texCoord = vec2(0,1);
    VP_EmitViewVertex();

    oe_grass_texCoord = vec2(1,1);
    gl_Position = newVerts[3];
    VP_EmitViewVertex();
                    
    EndPrimitive();
}