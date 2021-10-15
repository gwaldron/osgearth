#pragma once

#include <string>

namespace Bruneton
{
    struct Shaders
    {
        Shaders();

        // sdk:
        std::string
            constants,
            utility,
            uniforms,
            transmittance_functions,
            scattering_functions,
            irradiance_functions,
            rendering_functions,
            radiance_api;

        // rendering:
        std::string
            header,
            pbr,
            ground_init_frag,
            ground_best_vert,
            ground_best_frag,
            ground_fast_vert,
            ground_fast_frag,
            sky_vert,
            sky_frag;

        // compute:
        std::string
            clear_2d_cs,
            clear_3d_cs,
            compute_direct_irradiance_cs,
            compute_indirect_irradiance_cs,
            compute_multiple_scattering_cs,
            compute_scattering_density_cs,
            compute_single_scattering_cs,
            compute_transmittance_cs;
    };
}
