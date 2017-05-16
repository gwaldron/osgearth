@echo off
rem Batch file to play with the Ground Cover parameters
setlocal
osgearth_viewer splat.earth ^
    --sky ^
    --logdepth ^
    --samples 4 ^
    --uniform oe_GroundCover_density 1 7 ^
    --uniform oe_GroundCover_fill 1 0 ^
    --uniform oe_GroundCover_brightness 1 3 ^
    --uniform oe_GroundCover_contrast 0 1 ^
    --uniform oe_GroundCover_maxDistance 7000 200 ^
    --uniform oe_GroundCover_windFactor 0 3 ^
    %*    
endlocal