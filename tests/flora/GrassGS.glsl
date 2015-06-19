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
                                
void
oe_grass_geom()
{    
    vec4 positions[3];
	vec2 tileCoords[3];
    
    // Load the triangle data:
    for(int i=0; i < 3; ++i)
    {
        VP_LoadVertex(i);      
        positions[i] = gl_in[i].gl_Position;
		tileCoords[i] = oe_layer_tilec.st;
    }
                    
    // Center of the triangle:
    vec4 center_model = (positions[0]  + positions[1]  + positions[2]) *0.3333333;
    vec2 tileUV       = (tileCoords[0] + tileCoords[1] + tileCoords[2])*0.3333333;
    
    // Estimate the relative resolution of tile coordinates so we can jitter them
    // along with the vertex.
    float tileUVRes = distance(tileCoords[0], tileCoords[1]) / distance(positions[0], positions[1]);
    
    // Maximum distance to offset the instances from the original vertex location.
    float maxOffset = distance(center_model, positions[0]) * oe_grass_noise;

    // Randomly alter the position    
    oe_grass_jitterVertex(center_model, tileUV, maxOffset, tileUVRes);
    
    // Transform to view space.
    vec4 center_view = gl_ModelViewMatrix * center_model;
    vec3 up_view     = normalize(gl_NormalMatrix * vp_UpVector);
    
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

	// compute the grass vertices in view space.
    vec4 newVerts[4];
    
    vec3 tangent_view = vec3(1,0,0); // assuming no roll.
    
    newVerts[0] = vec4(center_view.xyz - tangent_view*width*0.5, 1.0);
    newVerts[1] = vec4(center_view.xyz + tangent_view*width*0.5, 1.0);
    newVerts[2] = vec4(newVerts[0].xyz + up_view*height, 1.0);
    newVerts[3] = vec4(newVerts[1].xyz + up_view*height, 1.0);
                      
    //TODO: animate based on wind parameters.
    newVerts[2].xyz += tangent_view * oe_grass_applyWind(osg_FrameTime*(1+n), oe_grass_width*oe_grass_windFactor*n, newVerts[2].x);
    newVerts[3].xyz += tangent_view * oe_grass_applyWind(osg_FrameTime*(1-n), oe_grass_width*oe_grass_windFactor*n, tileUV.t);

    vec3 normal = vec3(0,0,1);
    normal.xy += vec2(oe_grass_rangeRand(-0.25, 0.25, vec2(n)));
    vp_Normal = normalize(gl_NormalMatrix * normal);
    
    vp_Color = vec4(0,0,0,falloff);
    gl_Position = newVerts[0];
    oe_grass_texCoord = vec2(0,0);
    VP_EmitViewVertex();
    
    gl_Position = newVerts[1];
    oe_grass_texCoord = vec2(1,0);
    VP_EmitViewVertex();

    vp_Color = vec4(1,1,1,falloff);      
    gl_Position = newVerts[2];
    oe_grass_texCoord = vec2(0,1);
    VP_EmitViewVertex();

    oe_grass_texCoord = vec2(1,1);
    gl_Position = newVerts[3];
    VP_EmitViewVertex();
                    
    EndPrimitive();
}