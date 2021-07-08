#pragma once

/*
https://github.com/diharaw/BrunetonSkyModel

Copyright (c) 2018 Dihara Wijetunga

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "eb_ogl.h"

namespace dw
{
    struct TextureBuffer
    {
        dw::Texture* m_delta_irradiance_texture;
        dw::Texture* m_delta_rayleigh_scattering_texture;
        dw::Texture* m_delta_mie_scattering_texture;
        dw::Texture* m_delta_scattering_density_texture;
        dw::Texture* m_delta_multiple_scattering_texture;
        dw::Texture* m_transmittance_array[2];
        dw::Texture* m_irradiance_array[2];
        dw::Texture* m_scattering_array[2];
        dw::Texture* m_optional_single_mie_scattering_array[2];

        TextureBuffer(bool half_precision);
        ~TextureBuffer();

        void clear(dw::Program* program_2d, dw::Program* program_3d);
        void new_texture_2d_array(dw::Texture** arr, int width, int height, bool half_precision);
        void new_texture_3d_array(dw::Texture** arr, int width, int height, int depth, bool half_precision);

        static dw::Texture* new_texture_2d(int width, int height, bool half_precision);
        static dw::Texture* new_texture_3d(int width, int height, int depth, bool half_precision);

    private:
        void clear_array(dw::Program* program, dw::Texture** arr);
        void clear_texture(dw::Program* program, dw::Texture* arr);
    };
}