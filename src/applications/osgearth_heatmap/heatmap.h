/* heatmap - High performance heatmap creation in C.
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013 Lucas Beyer
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef _HEATMAP_H
#define _HEATMAP_H

/* Necessary for size_t */
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Maybe make an opaque type out of this. But then again,
 * I'm assuming the users of this lib are not stupid here.
 * If you mess with the internals and things break, blame yourself.
 */
typedef struct {
    float* buf;    /* Contains the heat value of every heatmap pixel. */
    float max;     /* The highest heat in the whole map. Used for normalization. */
    unsigned w, h; /* Pixel-dimension of the heatmap. */
} heatmap_t;

/* A stamp is "stamped" (added) onto the heatmap for every datapoint which
 * is seen. This is usually something spheric, but there are no limits to your
 * artistic freedom!
 */
typedef struct {
    float* buf;    /* The stampdata which is added onto the heatmap. */
    unsigned w, h; /* The size (in pixel) of the stamp. */
} heatmap_stamp_t;

/* A colorscheme is used to transform the heatmap's heat values (floats)
 * into an actual colorful heatmap.
 * Maybe counterintuitively, the coldest color comes first (stored at index 0)
 * and the hottest color comes last (stored at (ncolors-1)*4).
 * Note that one color is made up of FOUR chars, since it is RGBA.
 * You probably want the very first color to be (0,0,0,0) such that the heatmap
 * is transparent where there was no data and you can overlay it onto
 * another image, like a world map.
 */
typedef struct {
    const unsigned char* colors;  /* Color values in RGBA. */
    size_t ncolors;               /* Amount of colors (not amount of bytes or array size). */
} heatmap_colorscheme_t;

/* Creates a new heatmap of given size. */
heatmap_t* heatmap_new(unsigned w, unsigned h);
/* Frees up all memory taken by the heatmap. */
void heatmap_free(heatmap_t* h);

/* Adds a single point to the heatmap using the default stamp. */
void heatmap_add_point(heatmap_t* h, unsigned x, unsigned y);
/* Adds a single point to the heatmap using a given stamp. */
void heatmap_add_point_with_stamp(heatmap_t* h, unsigned x, unsigned y, const heatmap_stamp_t* stamp);

/* Adds a single weighted point to the heatmap using the default stamp. */
void heatmap_add_weighted_point(heatmap_t* h, unsigned x, unsigned y, float w);
/* Adds a single weighted point to the heatmap using a given stamp. */
void heatmap_add_weighted_point_with_stamp(heatmap_t* h, unsigned x, unsigned y, float w, const heatmap_stamp_t* stamp);

/* Renders an image of the heatmap into the given colorbuf.
 *
 * colorbuf: A buffer large enough to hold 4*heatmap_width*heatmap_height
 *           unsigned chars. These chars are the RGBA values of the pixels.
 *
 *           If colorbuf is NULL, a new large enough buffer will be malloc'd.
 *
 * return: A pointer to the given colorbuf is returned. It is the caller's
 *         responsibility to free that buffer. If no colorbuf is given (NULL),
 *         a newly malloc'd buffer is returned. This buffer needs to be free'd
 *         by the caller whenever it is not used anymore.
 */
unsigned char* heatmap_render_default_to(const heatmap_t* h, unsigned char* colorbuf);

/* Renders an RGB image of the heatmap into the given colorbuf,
 * using a given colorscheme.
 *
 * colorscheme: See the description of heatmap_colorscheme_t for more details.
 *
 * For details on the colorbuf and the return value, refer to the documentation
 * of `heatmap_render_default_to`.
 */
unsigned char* heatmap_render_to(const heatmap_t* h, const heatmap_colorscheme_t* colorscheme, unsigned char* colorbuf);

/* Renders an RGB image of the heatmap into the given colorbuf,
 * using a given colorscheme.
 *
 * colorscheme: See the description of heatmap_colorscheme_t for more details.
 *
 * saturation: The heatmap will be truncated at the given heat value, meaning
 *             all spots hotter than `saturation` will be assigned the same
 *             color as the hottest color on the scale.
 *
 * For details on the colorbuf and the return value, refer to the documentation
 * of `heatmap_render_default_to`.
 */
unsigned char* heatmap_render_saturated_to(const heatmap_t* h, const heatmap_colorscheme_t* colorscheme, float saturation, unsigned char* colorbuf);

/* Creates a new stamp COPYING the given w*h floats in data.
 *
 * w, h: The width/height of the stamp, in pixels.
 * data: exactly w*h float values which will be added to the heatmap centered
 *       around every datapoint drawn onto the heatmap.
 *
 * For more information about stamps, read `heatmap_stamp_t`'s documentation.
 */
heatmap_stamp_t* heatmap_stamp_load(unsigned w, unsigned h, float* data);

/* Generates a default round stamp of a given radius. This means the stamp will
 * have a size of 2*radius+1 square. The default stamp is just a spherical
 * gradient around the center.
 *
 * For more information about stamps, read `heatmap_stamp_t`'s documentation.
 */
heatmap_stamp_t* heatmap_stamp_gen(unsigned radius);

/* Generates a stamp just like `heatmap_stamp_gen` but calls the given
 * `distshape` function in order to determine the value of every single pixel.
 *
 * distshape: Function which gets called for every pixel. The only argument
 *            given to that function is the distance from the centre, 0 being
 *            exactly on the centre and 1 being one-behind the radius.
 *            One minus the returned value, clamped to [0,1] will be the
 *            pixel's value.
 *
 * For more information about stamps, read `heatmap_stamp_t`'s documentation.
 */
heatmap_stamp_t* heatmap_stamp_gen_nonlinear(unsigned radius, float (*distshape)(float));

/* Frees up all memory taken by the stamp. */
void heatmap_stamp_free(heatmap_stamp_t* s);

/* Create a new colorscheme using a COPY of the given `ncolors` `colors`.
 *
 * colors: a buffer containing RGBA colors to use when rendering the heatmap.
 *
 * For more information about colorschemes, read `heatmap_colorscheme_t`'s
 * documentation.
 */
heatmap_colorscheme_t* heatmap_colorscheme_load(const unsigned char* colors, size_t ncolors);

/* Frees up all memory taken by the colorscheme. */
void heatmap_colorscheme_free(heatmap_colorscheme_t* cs);

extern const heatmap_colorscheme_t* heatmap_cs_default;

#ifdef __cplusplus
} /* extern "C" */
#endif

#ifdef __cplusplus
/* C++ wrapper API, but TODO: is this even necessary? */
namespace heatmap {
}
#endif

#endif /* _HEATMAP_H */

