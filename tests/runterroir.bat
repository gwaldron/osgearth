@echo off
osgearth_toc terroir.earth ^
    --uniform density_power 1 2 ^
    --uniform moisture_power 1 2 ^
    --uniform temperature_power 1 2 ^
    --uniform rugged_power 1 2 ^
    --uniform depth 0.05 1.0 ^
    --uniform oe_sky_exposure 4.5 7.0 ^
    --uniform oe_sky_contrast 1 3 ^
    --uniform snow 0 1 ^
    --uniform oe_gc_sse 500 50 ^
    --sky ^
    --shadows ^
    --vfov 60 ^
    --samples 4 ^
    --uniform demo_wind 0 3 ^
    %*