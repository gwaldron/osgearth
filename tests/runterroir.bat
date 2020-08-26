@echo off
osgearth_toc terroir.earth ^
    --uniform density_power 1 2 ^
    --uniform moisture_power 1 2 ^
    --uniform temperature_power 1 2 ^
    --uniform rugged_power 1 2 ^
    --uniform depth 0.01 1.0 ^
    --uniform oe_sky_exposure 4.5 2.0 ^
    --uniform snow 0 1 ^
    --uniform oe_gc_sse 1000 50 ^
    --sky ^
    --vfov 60 ^
    --samples 4 ^
    %*