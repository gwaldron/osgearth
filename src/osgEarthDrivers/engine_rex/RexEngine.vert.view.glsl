#version 330 compatibility

#pragma vp_name       "REX Engine - Vertex/View"
#pragma vp_entryPoint "oe_rexEngine_applyElevation"
#pragma vp_location   "vertex_view"
#pragma vp_order      "0.4"

#define VP_REX_TILE_LEVEL_MORPHING 0

// Stage globals
out vec4 oe_layer_tilec;
out vec4 vp_Vertex;
out vec3 oe_UpVectorView;
out vec3 oe_TangentVectorView;
out vec4 flerp;
out vec4 oe_layer_texc;

uniform sampler2D oe_tile_elevationTex;
uniform mat4      oe_tile_elevationTexMatrix;
uniform vec4	  oe_tile_morph_constants;
uniform vec4	  oe_tile_grid_dimensions;
uniform vec4	  oe_tile_key;
uniform vec4	  oe_tile_extents;
uniform vec4	  oe_tile_camera_to_tilecenter;
uniform mat4	  oe_layer_texMatrix;

void MorphVertex( inout vec3 vPositionMorphed,  inout vec2 vUVMorphed
				, vec3 vPositionOriginal, vec2 vUVOriginal
				, float fMorphLerpK
				, vec2 vTileScale
				, vec3 vTangent
				, vec3 vBinormal)
{
   vec2 fFractionalPart = fract( vUVOriginal.xy * vec2(oe_tile_grid_dimensions.y, oe_tile_grid_dimensions.y) ) * vec2(oe_tile_grid_dimensions.z, oe_tile_grid_dimensions.z);
   vUVMorphed = vUVOriginal - (fFractionalPart * fMorphLerpK);

   vUVMorphed = clamp(vUVMorphed, 0, 1);

  // vPositionMorphed = vPositionOriginal.xy - (vTileScale*fFractionalPart * fMorphLerpK) ;
  vec2 dudv = vUVMorphed - vUVOriginal;

  vPositionMorphed.xyz = vPositionOriginal.xyz + normalize(vTangent)*dudv.x*vTileScale.x + normalize(vBinormal)*dudv.y*vTileScale.y;   
}

void oe_rexEngine_applyElevation(inout vec4 vertexView)
{
	// we use tangent space morphing only on higher res grids
	// the lod at and beyond which this tangent space morphing is
	// done is encoded in oe_tile_grid_dimensions.w
	#if 1
	if (oe_tile_key.z <= oe_tile_grid_dimensions.w)
	#endif
	{
		// Sample the elevation texture and move the vertex accordingly.
		vec4 elevc = oe_tile_elevationTexMatrix * oe_layer_tilec;
		float elev = texture(oe_tile_elevationTex, elevc.st).r;

		// assumption: vp_Normal is normalized
		vertexView.xyz += oe_UpVectorView*elev;

		oe_layer_texc = oe_layer_texMatrix * oe_layer_tilec;
	}
	#if 1
	else
	{
		vec4 elevc = oe_tile_elevationTexMatrix * oe_layer_tilec;
		float elev = textureLod(oe_tile_elevationTex, elevc.st,0).r;

		vec4 vertexViewElevated = vertexView;
		vertexViewElevated.xyz += oe_UpVectorView*elev;

#if VP_REX_TILE_LEVEL_MORPHING
		float fMorphLerpK  = 1.0f - clamp( oe_tile_morph_constants.z - oe_tile_camera_to_tilecenter.x * oe_tile_morph_constants.w, 0.0, 1.0 );
#else
		float fDistanceToEye = length(vertexViewElevated);
		float fMorphLerpK  = 1.0f - clamp( oe_tile_morph_constants.z - fDistanceToEye * oe_tile_morph_constants.w, 0.0, 1.0 );
#endif

		vec3 vPositionMorphed;
		vec2 vUVMorphed;

		MorphVertex(vPositionMorphed, vUVMorphed
				  , vertexView.xyz	, oe_layer_tilec.xy
				  , fMorphLerpK
				  , oe_tile_extents.xy
				  , oe_TangentVectorView, cross(oe_UpVectorView, oe_TangentVectorView) );

		vertexView.xyz = vPositionMorphed.xyz ;

		vec4 elevcMorphed = oe_tile_elevationTexMatrix * vec4(vUVMorphed,oe_layer_tilec.z,oe_layer_tilec.w);
		float elevMorphed = textureLod(oe_tile_elevationTex, elevcMorphed.st,0).r;

		vertexView.xyz += oe_UpVectorView*elevMorphed;

        // Update the tile coords:
        oe_layer_tilec.st = vUVMorphed;

        // And compute the texture coords
		oe_layer_texc = oe_layer_texMatrix * oe_layer_tilec; 

		flerp.xyz = vec3(fMorphLerpK);
	}
	#endif
}
