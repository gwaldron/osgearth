@echo off
osgearth_viewer grass.earth ^
    --uniform oe_grass_width 0.1 0.5 ^
    --uniform oe_grass_height 0.25 1 ^
    --uniform oe_grass_lod 22 21 ^
    --uniform oe_grass_windFactor 0 1 ^
    --uniform oe_grass_maxDistance 25 100 ^
    --uniform oe_grass_noise 0.1 1 ^
    --logdepth ^
    %*
