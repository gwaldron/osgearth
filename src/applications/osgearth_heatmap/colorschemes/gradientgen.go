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

import "flag"
import "log"
import "strconv"

// same thing as MustParseHex but for float parsing
func MustParseFloatZeroOne(s string) float64 {
    f, err := strconv.ParseFloat(s, 64)
    if err != nil {
        log.Fatalf("Invalid keypoint position: %v: %v\n", s, err.Error())
    }
    if f < 0.0 || 1.0 < f {
        log.Fatalf("Invalid keypoint position: %v: keypoints must lie within 0.0 and 1.0\n", s)
    }
    return f
}

func main() {
    pname := flag.String("name", "unnamed", "The name of the colorscheme.")
    pw := flag.Uint64("w", 40, "Width of the pictures of the colorscheme.")
    ph := flag.Uint64("h", 1024, "Height (number of levels) of the colorscheme.")
    flag.Parse()

    if flag.NArg() < 2*2 || flag.NArg() % 2 != 0 {
        flag.Usage()
        log.Fatal("Need at least two gradient keypoints!")
    }

    keypoints := GradientTable{}
    for i := 0 ; i < flag.NArg() ; i += 2 {
        keypoints = append(keypoints, GradientTableEntry{MustParseHex(flag.Arg(i)), MustParseFloatZeroOne(flag.Arg(i+1))})
    }

    CreateColorschemes(keypoints, *pname, int(*pw), int(*ph))
}
