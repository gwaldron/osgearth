@echo off
setlocal
set OSG_NEAR_FAR_RATIO=0.00001
set OSG_NUM_DATABASE_THREADS=6
set OSG_NUM_HTTP_DATABASE_THREADS=3

osgearth_viewer flora.earth ^
    --sky ^
    --vfov 45 ^
    --manip-terrain-avoidance false ^
    %*
    
endlocal

