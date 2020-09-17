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

#ifndef _HEATMAP_COLORSCHEMES_RDPU_H
#define _HEATMAP_COLORSCHEMES_RDPU_H

#ifdef __cplusplus
extern "C" {
#endif

/* This one has only N discrete colors. */
extern const heatmap_colorscheme_t* heatmap_cs_RdPu_discrete;
/* This is a very soft gradient along abovementioned discrete colors. */
extern const heatmap_colorscheme_t* heatmap_cs_RdPu_soft;
/* This is a mix of the above two. Makes for a pretty result in many cases. */
extern const heatmap_colorscheme_t* heatmap_cs_RdPu_mixed;
/* An exponential version of the default mix of the above two. */
/* Use this if your maximum is very "spiked". */
extern const heatmap_colorscheme_t* heatmap_cs_RdPu_mixed_exp;

#ifdef __cplusplus
}
#endif

#endif /* _HEATMAP_COLORSCHEMES_RDPU_H */
