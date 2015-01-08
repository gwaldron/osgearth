setlocal
set OSGEARTH_SPLAT_EDIT=1
osgearth_viewer splat-test.earth ^
	--ico ^
	--logdepth ^
	--sky ^
	--uniform oe_splat_blending_range 200000 0 ^
	--uniform oe_splat_detail_range 100000 0 ^
	--uniform oe_splat_warp 0 0.01 ^
	--uniform oe_splat_blur 1 4 ^
	--uniform oe_splat_scaleOffset 0 7 ^
	--uniform oe_splat_freq 1 256 ^
	--uniform oe_splat_pers 0.25 2.0 ^
	--uniform oe_splat_lac  1.0 6.0 ^
	--uniform oe_splat_octaves 1 8 ^
	--uniform oe_splat_saturation 0 1 ^
	--uniform oe_splat_threshold 0 1 ^
	--uniform oe_splat_minSlope 0 1 ^
	--uniform oe_splat_contrast 1 4 ^
	--uniform oe_splat_brightness 1 4 ^
	--uniform oe_splat_snowMinElevation 8000 1 ^
	--uniform oe_splat_snowPatchiness 1 6 ^
	--uniform oe_bumpmap_intensity 0 2.0 ^
	--uniform oe_bumpmap_scale 1 20 ^
	%*
endlocal

	