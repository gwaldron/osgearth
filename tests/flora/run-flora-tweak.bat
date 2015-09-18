@echo off
setlocal
set OSG_NEAR_FAR_RATIO=0.00001
set OSG_NUM_DATABASE_THREADS=6
set OSG_NUM_HTTP_DATABASE_THREADS=3

osgearth_viewer flora.earth ^
    --sky ^
    --vfov 45 ^
    --manip-terrain-avoidance false ^
    --uniform oe_flora_windFactor 0 1.0 ^
    --uniform oe_flora_noise 1.0 0.0 ^
    --uniform oe_flora_exposure 1.0 3.0 ^
    --uniform oe_flora_width 0.5 1.0 ^
    --uniform oe_flora_height 0.25 1.5 ^
    --uniform oe_flora_density 1.0 3.0 ^
    %*
    
endlocal

