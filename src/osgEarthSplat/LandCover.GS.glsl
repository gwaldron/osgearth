#version $GLSL_VERSION_STR
#pragma vp_name       LandCover geometry shader
#pragma vp_entryPoint oe_landcover_geom
#pragma vp_location   geometry
                
layout(triangles)        in;        // triangles from the TileDrawable
layout(triangle_strip)   out;       // output a triangle-strip billboard
layout(max_vertices = 8) out;       // four verts per billboard
                
// VP helper functions:
void VP_LoadVertex(in int);
void VP_EmitViewVertex();

uniform float osg_FrameTime;            // Frame time (seconds) used for wind animation
                
uniform float oe_landcover_width;           // width of each billboard
uniform float oe_landcover_height;          // height of each billboard
uniform float oe_landcover_ao;              // fake ambient occlusion of ground verts (0=full)

uniform float oe_landcover_fill;            // percentage of points that make it through, based on noise function
uniform float oe_landcover_windFactor;      // wind blowing the foliage
uniform float oe_landcover_maxDistance;     // distance at which flora disappears

uniform float oe_landcover_contrast;
uniform float oe_landcover_brightness;

uniform sampler2D oe_tile_elevationTex;
uniform mat4      oe_tile_elevationTexMatrix;
uniform float     oe_tile_elevationSize;

uniform bool oe_isShadowCamera;

// Noise texture:
uniform sampler2D oe_splat_noiseTex;

// different noise texture channels:
#define NOISE_SMOOTH   0
#define NOISE_RANDOM   1
#define NOISE_RANDOM_2 2
#define NOISE_CLUMPY   3

// Input tile coordinates [0..1]
in vec4 oe_layer_tilec;

// Output grass texture coordinates to the fragment shader
out vec2 oe_landcover_texCoord;

// Input from the TCS that 
//flat in int oe_landcover_biomeIndex;

// Output that selects the land cover texture from the texture array (non interpolated)
flat out float oe_landcover_arrayIndex;

struct oe_landcover_Biome {
    int firstBillboardIndex;
    int numBillboards;
    float density;
    float fill;
    vec2 maxWidthHeight;
};
void oe_landcover_getBiome(in int biomeIndex, out oe_landcover_Biome biome);

struct oe_landcover_Billboard {
    int arrayIndex;
    float width;
    float height;
};
void oe_landcover_getBillboard(in int billboardIndex, out oe_landcover_Billboard bb);


// Output colors/normals:
out vec4 vp_Color;
out vec3 vp_Normal;

// Up vector for clamping.
in vec3 oe_UpVectorView;

// SDK import
float oe_terrain_getElevation(in vec2);

// Generated in code
int oe_landcover_getBiomeIndex(in vec4);

uniform bool oe_landcover_useMask;
uniform sampler2D MASK_SAMPLER;
uniform mat4 MASK_TEXTURE;


// Sample the elevation texture and move the vertex accordingly.
void
oe_landcover_clamp(inout vec4 vert_view, in vec3 up, vec2 UV)
{
    float elev = oe_terrain_getElevation( UV );
    vert_view.xyz += up*elev;
}

// Generate a pseudo-random value in the specified range:
float
oe_landcover_rangeRand(float minValue, float maxValue, vec2 co)
{
    float t = fract(sin(dot(co.xy ,vec2(12.9898,78.233))) * 43758.5453);
    return minValue + t*(maxValue-minValue);
}

// Generate a wind-perturbation value
float
oe_landcover_applyWind(float time, float factor, float randOffset)
{
   return sin(time + randOffset) * factor;
}

// Generate a pseudo-random barycentric point inside a triangle.
vec3
oe_landcover_getRandomBarycentricPoint(vec2 seed)
{
    vec3 b;
    b[0] = oe_landcover_rangeRand(0.0, 1.0, seed.xy);
    b[1] = oe_landcover_rangeRand(0.0, 1.0, seed.yx);
    if (b[0]+b[1] >= 1.0)
    {
        b[0] = 1.0 - b[0];
        b[1] = 1.0 - b[1];
    }
    b[2] = 1.0 - b[0] - b[1];
    return b;
}

// MAIN ENTRY POINT  
void
oe_landcover_geom()
{    
    vec4 center = vec4(0,0,0,1);
    vec2 tileUV = vec2(0,0);
    
    // gen a random point within the input triangle
    vec3 b = oe_landcover_getRandomBarycentricPoint(gl_in[0].gl_Position.xy);
    
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
   
    // Look up the biome at this point:
    int biomeIndex = oe_landcover_getBiomeIndex(vec4(tileUV,0,1));
    if ( biomeIndex < 0 )
    {
        // No biome defined; bail out without emitting any geometry.
        return;
    }
    
    // If we're using a mask texture, sample it now:
    if ( oe_landcover_useMask )
    {
        float mask = texture(MASK_SAMPLER, (MASK_TEXTURE*vec4(tileUV,0,1)).st).a;
        if ( mask > 0.0 )
        {
            // Failed to pass the mask; no geometry emitted.
            return;
        }
    }
    
    // Transform to view space.
    vec4 center_view = gl_ModelViewMatrix * center;
    vec3 up_view     = oe_UpVectorView;
    
    // Clamp the center point to the elevation.
    oe_landcover_clamp(center_view, up_view, tileUV);

    // Calculate the normalized camera range:
    float nRange = clamp(-center_view.z/oe_landcover_maxDistance, 0.0, 1.0);

    // Distance culling:
    if ( nRange == 1.0 )
        return;

    // look up biome:
    oe_landcover_Biome biome;
    oe_landcover_getBiome(biomeIndex, biome);

    // sample the noise texture.
    vec4 noise = texture(oe_splat_noiseTex, tileUV);

    // a pseudo-random scale factor to the width and height of a billboard
    float sizeScale = abs(1.0 + noise[NOISE_RANDOM_2]);
    
    // Viewpoint culling:
    // TODO: remove hard-coded max width/height and replace with a vp_define or a uniform.
    // Note: this value must account for the height variation introduced by the noise function
    // later in this shader!
    vec4 cullPoint = center_view;
    //vec2 maxWidthHeight = biome.maxWidthHeight * sizeScale;
    cullPoint.xy -= sign(cullPoint.xy) * min(biome.maxWidthHeight*sizeScale, abs(cullPoint.xy));
    cullPoint = gl_ProjectionMatrix * cullPoint;
    float absw = abs(cullPoint.w);
    if ( abs(cullPoint.x) > absw || abs(cullPoint.y) > absw )// || abs(cullPoint.z) > absw )
        return;

    // discard instances based on noise value threshold (coverage). If it passes,
    // scale the noise value back up to [0..1]
    if ( noise[NOISE_SMOOTH] > oe_landcover_fill )
        return;
    else
        noise[NOISE_SMOOTH] /= oe_landcover_fill;

    // select a billboard seemingly at random. Need to scale n to account for the fill limit first though.
    int billboardIndex = biome.firstBillboardIndex + int( floor(noise[NOISE_RANDOM] * float(biome.numBillboards) ) );
    billboardIndex = min(billboardIndex, biome.firstBillboardIndex + biome.numBillboards - 1);

    oe_landcover_Billboard billboard;
    oe_landcover_getBillboard(billboardIndex, billboard);
    
    // pass the billboard's array index along to the fragment shader.
    oe_landcover_arrayIndex = float(billboard.arrayIndex);
    
	
    // push the falloff closer to the max distance.
    float falloff = 1.0-(nRange*nRange*nRange);

    // billboard width, which shrinks into the distance
    float width = billboard.width * falloff * sizeScale;
    
    float height = billboard.height * falloff * sizeScale;

    // vary the height of each instance and shrink it as it disappears into the distance.
    // TODO: consider parameterizing this so we can toggle the feature
    //height *= sizeScale;

    // shrink land cover as it dissappears into the distance:
    //height *= falloff;

	// compute the billboard corners in view space.
    vec4 LL, LR, UL, UR;
    
    if ( oe_isShadowCamera == false )
    {
        vec3 tangentVector = normalize(cross(vec3(0,0,-1), up_view));
        vec3 halfWidthTangentVector = tangentVector * 0.5 * width;
        vec3 heightVector = up_view*height;
        
        LL = vec4(center_view.xyz - halfWidthTangentVector, 1.0);
        LR = vec4(center_view.xyz + halfWidthTangentVector, 1.0);
        UL = vec4(LL.xyz + heightVector, 1.0);
        UR = vec4(LR.xyz + heightVector, 1.0);
                      
        // TODO: animate based on wind parameters.
        float nw = noise[NOISE_SMOOTH];
        float wind = width*oe_landcover_windFactor*nw;
        UL.x += oe_landcover_applyWind(osg_FrameTime*(1+nw), wind, UL.x);
        UR.x += oe_landcover_applyWind(osg_FrameTime*(1-nw), wind, tileUV.t);
    
        // Color variation, brightness, and contrast:
        vec3 color = vec3( noise[NOISE_RANDOM_2] );
        color = ( ((color - 0.5) * oe_landcover_contrast + 0.5) * oe_landcover_brightness);

        vp_Color = vec4(color*oe_landcover_ao, falloff);

        // calculates normals:
        vec3 faceNormalVector = normalize(cross(tangentVector, heightVector));
        float blend = 0.25 + (noise[NOISE_RANDOM_2]*0.25);
        vec3 Lnormal = mix(-tangentVector, faceNormalVector, blend);
        vec3 Rnormal = mix( tangentVector, faceNormalVector, blend);

        gl_Position = LL;
        oe_landcover_texCoord = vec2(0,0);
        vp_Normal = Lnormal;
        VP_EmitViewVertex();
    
        gl_Position = LR;
        oe_landcover_texCoord = vec2(1,0);
        vp_Normal = Rnormal;
        VP_EmitViewVertex();

        vp_Color = vec4(color, falloff);      

        gl_Position = UL;
        oe_landcover_texCoord = vec2(0,1);
        vp_Normal = Lnormal;
        VP_EmitViewVertex();

        oe_landcover_texCoord = vec2(1,1);
        vp_Normal = Rnormal;
        gl_Position = UR;
        VP_EmitViewVertex();
                    
        EndPrimitive();
    }
    else
    {
        // generating cross-hatch geometry (for shadowing)

        vec3 eastVector = gl_NormalMatrix * vec3(1,0,0);
        vec3 halfWidthTangentVector = cross(eastVector, up_view) * 0.5 * width;
        vec3 heightVector = up_view*height;

        vp_Color = vec4(1,1,1,falloff);

        for(int i=0; i<2; ++i)
        {
            LL = vec4(center_view.xyz - halfWidthTangentVector, 1.0);
            LR = vec4(center_view.xyz + halfWidthTangentVector, 1.0);
            UL = vec4(LL.xyz + heightVector, 1.0);
            UR = vec4(LR.xyz + heightVector, 1.0);
    
            gl_Position = LL;
            oe_landcover_texCoord = vec2(0,0);
            VP_EmitViewVertex();
    
            gl_Position = LR;
            oe_landcover_texCoord = vec2(1,0);
            VP_EmitViewVertex();    

            gl_Position = UL;
            oe_landcover_texCoord = vec2(0,1);
            VP_EmitViewVertex();

            oe_landcover_texCoord = vec2(1,1);
            gl_Position = UR;
            VP_EmitViewVertex();
                    
            EndPrimitive();

            halfWidthTangentVector = cross(halfWidthTangentVector, up_view);
        }
    }
}
