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

#ifndef _HEATMAP_COLORSCHEMES_GRAY_H
#define _HEATMAP_COLORSCHEMES_GRAY_H

#ifdef __cplusplus
extern "C" {
#endif

/* Black to white gradient with 0 being transparent. */
extern const heatmap_colorscheme_t* heatmap_cs_b2w;
/* Black to white gradient with 0 being all black. */
extern const heatmap_colorscheme_t* heatmap_cs_b2w_opaque;
/* White to black gradient with 0 being transparent. */
extern const heatmap_colorscheme_t* heatmap_cs_w2b;
/* White to black gradient with 0 being all white. */
extern const heatmap_colorscheme_t* heatmap_cs_w2b_opaque;

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* _HEATMAP_COLORSCHEMES_GRAY_H */

