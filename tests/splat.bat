osgearth_viewer splat-test.earth ^
	--ico ^
	--logdepth2 ^
	--sky ^
	--uniform oe_splat_blending_range 200000 0 ^
	--uniform oe_splat_detail_range 100000 0 ^
	--uniform oe_splat_warp 0 0.01 ^
	--uniform oe_splat_covlod 0 5 ^
	--uniform oe_splat_blur 1 32 ^
	--uniform oe_splat_scaleOffset 0 7 ^
	--uniform oe_nmap_intensity 0 2.0 ^
	--uniform oe_nmap_scale 1 20 ^
	%*
	