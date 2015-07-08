setlocal
set SPLAT_USE_COLOR_IMAGE=1
%OSG_EARTH_HOME%\bin\osgearth_viewer splat-imagery.earth ^
	--logdepth2 ^
	--sky ^
	--uniform oe_splat_color_ratio 0 1 ^
	--uniform oe_splat_color_start_dist 0 20000 ^
	--window 100 100 800 600 ^
	%*
endlocal

	