@echo off
setlocal
osgearth_viewer splat.earth ^
    --sky --logdepth ^
    --uniform oe_landcover_density 1 5 ^
    --uniform oe_landcover_fill 1 0 ^
    --uniform oe_landcover_brightness 1 3 ^
    --uniform oe_landcover_contrast 0 1 ^
    --uniform oe_landcover_maxDistance 7000 200 ^
    %*    
endlocal