@echo off
setlocal
set OSG_NEAR_FAR_RATIO=0.00001
set OSG_NUM_DATABASE_THREADS=8
set OSG_NUM_HTTP_DATABASE_THREADS=4

osgearth_viewer splat.earth ^
    --sky ^
    --vfov 45 ^
    --manip-terrain-avoidance false ^
    --logdepth ^
    %*
    
endlocal

