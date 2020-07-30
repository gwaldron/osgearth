// heatmap - High performance heatmap creation in C.
//
// The MIT License (MIT)
//
// Copyright (c) 2013 Lucas Beyer
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of
// this software and associated documentation files (the "Software"), to deal in
// the Software without restriction, including without limitation the rights to
// use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
// the Software, and to permit persons to whom the Software is furnished to do so,
// subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
// FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
// IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
// CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//

package main

import "fmt"
import "github.com/lucasb-eyer/go-colorful"
import "image"
import "image/draw"
import "image/png"
import "log"
import "math"
import "os"
import "text/template"
import "strings"

const LICENSE string =
`/* heatmap - High performance heatmap creation in C.
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

`

const HEADER_TEMPLATE string = LICENSE +
`#ifndef _HEATMAP_COLORSCHEMES_{{ToUpper .}}_H
#define _HEATMAP_COLORSCHEMES_{{ToUpper .}}_H

#ifdef __cplusplus
extern "C" {
#endif

/* This one has only N discrete colors. */
extern const heatmap_colorscheme_t* heatmap_cs_{{.}}_discrete;
/* This is a very soft gradient along abovementioned discrete colors. */
extern const heatmap_colorscheme_t* heatmap_cs_{{.}}_soft;
/* This is a mix of the above two. Makes for a pretty result in many cases. */
extern const heatmap_colorscheme_t* heatmap_cs_{{.}}_mixed;
/* An exponential version of the default mix of the above two. */
/* Use this if your maximum is very "spiked". */
extern const heatmap_colorscheme_t* heatmap_cs_{{.}}_mixed_exp;

#ifdef __cplusplus
}
#endif

#endif /* _HEATMAP_COLORSCHEMES_{{ToUpper .}}_H */
`

const SOURCE_TEMPLATE string = LICENSE +
`#ifdef __cplusplus
extern "C" {
#endif

#include "heatmap.h"
#include "colorschemes/{{.Name}}.h"

static const unsigned char discrete_data[] = {
    0, 0, 0, 0{{.Discrete}}
};
static const heatmap_colorscheme_t discrete = { discrete_data, sizeof(discrete_data)/sizeof(discrete_data[0]/4) };
const heatmap_colorscheme_t* heatmap_cs_{{.Name}}_discrete = &discrete;

static const unsigned char soft_data[] = {
    0, 0, 0, 0{{.Soft}}
};
static const heatmap_colorscheme_t soft = { soft_data, sizeof(soft_data)/sizeof(soft_data[0]/4) };
const heatmap_colorscheme_t* heatmap_cs_{{.Name}}_soft = &soft;

static const unsigned char mixed_data[] = {
    0, 0, 0, 0{{.Mixed}}
};
static const heatmap_colorscheme_t mixed = { mixed_data, sizeof(mixed_data)/sizeof(mixed_data[0]/4) };
const heatmap_colorscheme_t* heatmap_cs_{{.Name}}_mixed = &mixed;

static const unsigned char mixed_exp_data[] = {
    0, 0, 0, 0{{.MixedExp}}
};
static const heatmap_colorscheme_t mixed_exp = { mixed_exp_data, sizeof(mixed_exp_data)/sizeof(mixed_exp_data[0]/4) };
const heatmap_colorscheme_t* heatmap_cs_{{.Name}}_mixed_exp = &mixed_exp;

#ifdef __cplusplus
}
#endif
`

// This table contains the "keypoints" of the colorgradient you want to generate.
// The position of each keypoint has to live in the range [0,1]
type GradientTableEntry struct{
    Col colorful.Color
    Pos float64
}
type GradientTable []GradientTableEntry

// This is the meat of the gradient computation. It returns a HCL-blend between
// the two colors around `t`.
// Note: It relies heavily on the fact that the gradient keypoints are sorted.
func (self GradientTable) GetInterpolatedColorFor(t float64) colorful.Color {
    for i := 0 ; i < len(self) - 1 ; i++ {
        c1 := self[i]
        c2 := self[i+1]
        if c1.Pos <= t && t <= c2.Pos {
            // We are in between c1 and c2. Go blend them!
            t := (t - c1.Pos)/(c2.Pos - c1.Pos)
            return c1.Col.BlendHcl(c2.Col, t).Clamped()
        }
    }

    // Nothing found? Means we're at (or past) the last gradient keypoint.
    return self[len(self)-1].Col
}

// This returns the color of the closest gradient keypoint.
// Note: This too relies on the fact that the gradient keypoints are sorted.
func (self GradientTable) GetColorFor(t float64) colorful.Color {
    for i := 0 ; i < len(self) - 1 ; i++ {
        if t < (self[i].Pos + self[i+1].Pos)*0.5 {
            return self[i].Col
        }
    }

    return self[len(self)-1].Col
}

// This is a very nice thing Golang forces you to do!
// It is necessary so that we can write out the literal of the colortable below.
func MustParseHex(s string) colorful.Color {
    c, err := colorful.Hex(s)
    if err != nil {
        log.Fatalf("Invalid color: %v: %v\n", s, err.Error())
    }
    return c
}

func savepng(img image.Image, fname string) {
    toimg, err := os.Create(fname)
    if err != nil {
        log.Printf("Error saving png: %v\n", err)
        return
    }
    defer toimg.Close()

    png.Encode(toimg, img)
}

func CreateColorschemes(keypoints GradientTable, name string, w, h int) {
    c_file, err := os.Create(name + ".c")
    if err != nil { log.Fatal(err) }
    defer func() {
        if err := c_file.Close(); err != nil {
            log.Fatal(err)
        }
    }()

    h_file, err := os.Create(name + ".h")
    if err != nil { log.Fatal(err) }
    defer func() {
        if err := h_file.Close(); err != nil {
            log.Fatal(err)
        }
    }()

    // Might as well already create the header, we know all we need by now.
    tmpl, err := template.New("test").Funcs(template.FuncMap{"ToUpper": strings.ToUpper}).Parse(HEADER_TEMPLATE)
    if err != nil { log.Panic(err) }
    if err = tmpl.Execute(h_file, name); err != nil { log.Panic(err) }

    img_discrete  := image.NewRGBA(image.Rect(0,0,w,len(keypoints)))
    img_soft      := image.NewRGBA(image.Rect(0,0,w,h))
    img_mixed     := image.NewRGBA(image.Rect(0,0,w,h))
    img_mixed_exp := image.NewRGBA(image.Rect(0,0,w,h))
    discrete, soft, mixed, mixed_exp := "", "", "", ""

    // discrete
    for y, kp := range keypoints {
        r, g, b := kp.Col.RGB255()
        discrete += fmt.Sprintf(", %v, %v, %v, 255", r, g, b)
        draw.Draw(img_discrete, image.Rect(0, y, w, y+1), &image.Uniform{kp.Col}, image.ZP, draw.Src)
    }
    savepng(img_discrete, name + "_discrete.png")

    // soft
    for y := 0 ; y < h ; y++ {
        t := float64(y)/float64(h)
        a := uint8(math.Min(t*30.0, 1.0)*255.0)
        c := keypoints.GetInterpolatedColorFor(t)
        r, g, b := c.RGB255()
        soft += fmt.Sprintf(", %v, %v, %v, %v", r, g, b, a)
        draw.Draw(img_soft, image.Rect(0, y, w, y+1), &image.Uniform{c}, image.ZP, draw.Src)
    }
    savepng(img_soft, name + "_soft.png")

    // mixed
    for y := 0 ; y < h ; y++ {
        t := float64(y)/float64(h)
        a := uint8(math.Min(t*30.0, 1.0)*255.0)
        // Here, we actually want to create a gradient overlaid with a stairs-like gradient.
        // Thus c1 is the soft gradient color and c2 the closest keypoint's color.
        c1 := keypoints.GetInterpolatedColorFor(t)
        c2 := keypoints.GetColorFor(t)
        c := c1.BlendRgb(c2, 0.2)
        r, g, b := c.RGB255()
        mixed += fmt.Sprintf(", %v, %v, %v, %v", r, g, b, a)
        draw.Draw(img_mixed, image.Rect(0, y, w, y+1), &image.Uniform{c}, image.ZP, draw.Src)
    }
    savepng(img_mixed, name + "_mixed.png")

    // exp
    for y := 0 ; y < h ; y++ {
        t := float64(y)/float64(h)
        a := uint8(math.Min(t*100.0, 1.0)*255.0) // TODO: Change a
        t = 1.0 - t;
        t = 1.0 - t*t * t*t * t*t * t*t * t*t
        c1 := keypoints.GetInterpolatedColorFor(t)
        c2 := keypoints.GetColorFor(t)
        c := c1.BlendRgb(c2, 0.2)
        r, g, b := c.RGB255()
        mixed_exp += fmt.Sprintf(", %v, %v, %v, %v", r, g, b, a)
        draw.Draw(img_mixed_exp, image.Rect(0, y, w, y+1), &image.Uniform{c}, image.ZP, draw.Src)
    }
    savepng(img_mixed_exp, name + "_mixed_exp.png")

    // Write the C file now that we got all data.
    tmpl, err = template.New("test").Parse(SOURCE_TEMPLATE)
    if err != nil { log.Panic(err) }
    if err = tmpl.Execute(c_file, struct { Name, Discrete, Soft, Mixed, MixedExp string }{ name, discrete, soft, mixed, mixed_exp }); err != nil { log.Panic(err) }
}
