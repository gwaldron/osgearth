@echo off
setlocal
osgearth_viewer splat.earth ^
    --sky ^
    --logdepth ^
    --samples 4 ^
    --uniform oe_landcover_density 1 7 ^
    --uniform oe_landcover_fill 1 0 ^
    --uniform oe_landcover_brightness 1 3 ^
    --uniform oe_landcover_contrast 0 1 ^
    --uniform oe_landcover_maxDistance 7000 200 ^
    --uniform oe_landcover_windFactor 0 3 ^
    %*    
endlocal