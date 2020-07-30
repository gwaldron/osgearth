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

func main() {
    pw := flag.Uint64("w", 40, "Width of the pictures of the colorscheme.")
    ph := flag.Uint64("h", 1024, "Height (number of levels) of the colorscheme.")
    flag.Parse()

    // No, I tested it, only using 3 or 4 of the keypoints isn't enough!
    colorschemes := map[string]GradientTable{
        // Sequential

        // Multihue
        "BuGn": GradientTable{
            {MustParseHex("#00441B"), 0.000},
            {MustParseHex("#006D2C"), 0.125},
            {MustParseHex("#238B45"), 0.250},
            {MustParseHex("#41AE76"), 0.375},
            {MustParseHex("#66C2A4"), 0.500},
            {MustParseHex("#99D8C9"), 0.625},
            {MustParseHex("#CCECE6"), 0.750},
            {MustParseHex("#E5F5F9"), 0.875},
            {MustParseHex("#F7FCFD"), 1.000},
        },
       "BuPu": GradientTable{
            {MustParseHex("#4D004B"), 0.000},
            {MustParseHex("#810F7C"), 0.125},
            {MustParseHex("#88419D"), 0.250},
            {MustParseHex("#8C6BB1"), 0.375},
            {MustParseHex("#8C96C6"), 0.500},
            {MustParseHex("#9EBCDA"), 0.625},
            {MustParseHex("#BFD3E6"), 0.750},
            {MustParseHex("#E0ECF4"), 0.875},
            {MustParseHex("#F7FCFD"), 1.000},
        },
        "GnBu": GradientTable{
            {MustParseHex("#084081"), 0.000},
            {MustParseHex("#0868AC"), 0.125},
            {MustParseHex("#2B8CBE"), 0.250},
            {MustParseHex("#4EB3D3"), 0.375},
            {MustParseHex("#7BCCC4"), 0.500},
            {MustParseHex("#A8DDB5"), 0.625},
            {MustParseHex("#CCEBC5"), 0.750},
            {MustParseHex("#E0F3DB"), 0.875},
            {MustParseHex("#F7FCF0"), 1.000},
        },
        "OrRd": GradientTable{
            {MustParseHex("#7F0000"), 0.000},
            {MustParseHex("#B30000"), 0.125},
            {MustParseHex("#D7301F"), 0.250},
            {MustParseHex("#EF6548"), 0.375},
            {MustParseHex("#FC8D59"), 0.500},
            {MustParseHex("#FDBB84"), 0.625},
            {MustParseHex("#FDD49E"), 0.750},
            {MustParseHex("#FEE8C8"), 0.875},
            {MustParseHex("#FFF7EC"), 1.000},
        },
        "PuBu": GradientTable{
            {MustParseHex("#023858"), 0.000},
            {MustParseHex("#045A8D"), 0.125},
            {MustParseHex("#0570B0"), 0.250},
            {MustParseHex("#3690C0"), 0.375},
            {MustParseHex("#74A9CF"), 0.500},
            {MustParseHex("#A6BDDB"), 0.625},
            {MustParseHex("#D0D1E6"), 0.750},
            {MustParseHex("#ECE7F2"), 0.875},
            {MustParseHex("#FFF7FB"), 1.000},
        },
        "PuBuGn": GradientTable{
            {MustParseHex("#014636"), 0.000},
            {MustParseHex("#016C59"), 0.125},
            {MustParseHex("#02818A"), 0.250},
            {MustParseHex("#3690C0"), 0.375},
            {MustParseHex("#67A9CF"), 0.500},
            {MustParseHex("#A6BDDB"), 0.625},
            {MustParseHex("#D0D1E6"), 0.750},
            {MustParseHex("#ECE2F0"), 0.875},
            {MustParseHex("#FFF7FB"), 1.000},
        },
        "PuRd": GradientTable{
            {MustParseHex("#67001F"), 0.000},
            {MustParseHex("#980043"), 0.125},
            {MustParseHex("#CE1256"), 0.250},
            {MustParseHex("#E7298A"), 0.375},
            {MustParseHex("#DF65B0"), 0.500},
            {MustParseHex("#C994C7"), 0.625},
            {MustParseHex("#D4B9DA"), 0.750},
            {MustParseHex("#E7E1EF"), 0.875},
            {MustParseHex("#F7F4F9"), 1.000},
        },
        "RdPu": GradientTable{
            {MustParseHex("#49006A"), 0.000},
            {MustParseHex("#7A0177"), 0.125},
            {MustParseHex("#AE017E"), 0.250},
            {MustParseHex("#DD3497"), 0.375},
            {MustParseHex("#F768A1"), 0.500},
            {MustParseHex("#FA9FB5"), 0.625},
            {MustParseHex("#FCC5C0"), 0.750},
            {MustParseHex("#FDE0DD"), 0.875},
            {MustParseHex("#FFF7F3"), 1.000},
        },
        "YlGn": GradientTable{
            {MustParseHex("#004529"), 0.000},
            {MustParseHex("#006837"), 0.125},
            {MustParseHex("#238443"), 0.250},
            {MustParseHex("#41AB5D"), 0.375},
            {MustParseHex("#78C679"), 0.500},
            {MustParseHex("#ADDD8E"), 0.625},
            {MustParseHex("#D9F0A3"), 0.750},
            {MustParseHex("#F7FCB9"), 0.875},
            {MustParseHex("#FFFFE5"), 1.000},
        },
        "YlGnBu": GradientTable{
            {MustParseHex("#081D58"), 0.000},
            {MustParseHex("#253494"), 0.125},
            {MustParseHex("#225EA8"), 0.250},
            {MustParseHex("#1D91C0"), 0.375},
            {MustParseHex("#41B6C4"), 0.500},
            {MustParseHex("#7FCDBB"), 0.625},
            {MustParseHex("#C7E9B4"), 0.750},
            {MustParseHex("#EDF8B1"), 0.875},
            {MustParseHex("#FFFFD9"), 1.000},
        },
        "YlOrBr": GradientTable{
            {MustParseHex("#662506"), 0.000},
            {MustParseHex("#993404"), 0.125},
            {MustParseHex("#CC4C02"), 0.250},
            {MustParseHex("#EC7014"), 0.375},
            {MustParseHex("#FE9929"), 0.500},
            {MustParseHex("#FEC44F"), 0.625},
            {MustParseHex("#FEE391"), 0.750},
            {MustParseHex("#FFF7BC"), 0.875},
            {MustParseHex("#FFFFE5"), 1.000},
        },
        "YlOrRd": GradientTable{
            {MustParseHex("#800026"), 0.000},
            {MustParseHex("#BD0026"), 0.125},
            {MustParseHex("#E31A1C"), 0.250},
            {MustParseHex("#FC4E2A"), 0.375},
            {MustParseHex("#FD8D3C"), 0.500},
            {MustParseHex("#FEB24C"), 0.625},
            {MustParseHex("#FED976"), 0.750},
            {MustParseHex("#FFEDA0"), 0.875},
            {MustParseHex("#FFFFCC"), 1.000},
        },
        // Single Hue
        "Blues": GradientTable{
            {MustParseHex("#08306B"), 0.000},
            {MustParseHex("#08519C"), 0.125},
            {MustParseHex("#2171B5"), 0.250},
            {MustParseHex("#4292C6"), 0.375},
            {MustParseHex("#6BAED6"), 0.500},
            {MustParseHex("#9ECAE1"), 0.625},
            {MustParseHex("#C6DBEF"), 0.750},
            {MustParseHex("#DEEBF7"), 0.875},
            {MustParseHex("#F7FBFF"), 1.000},
        },
        "Greens": GradientTable{
            {MustParseHex("#00441B"), 0.000},
            {MustParseHex("#006D2C"), 0.125},
            {MustParseHex("#238B45"), 0.250},
            {MustParseHex("#41AB5D"), 0.375},
            {MustParseHex("#74C476"), 0.500},
            {MustParseHex("#A1D99B"), 0.625},
            {MustParseHex("#C7E9C0"), 0.750},
            {MustParseHex("#E5F5E0"), 0.875},
            {MustParseHex("#F7FCF5"), 1.000},
        },
        "Greys": GradientTable{
            {MustParseHex("#000000"), 0.000},
            {MustParseHex("#252525"), 0.125},
            {MustParseHex("#525252"), 0.250},
            {MustParseHex("#737373"), 0.375},
            {MustParseHex("#969696"), 0.500},
            {MustParseHex("#BDBDBD"), 0.625},
            {MustParseHex("#D9D9D9"), 0.750},
            {MustParseHex("#F0F0F0"), 0.875},
            {MustParseHex("#FFFFFF"), 1.000},
        },
        "Oranges": GradientTable{
            {MustParseHex("#7F2704"), 0.000},
            {MustParseHex("#A63603"), 0.125},
            {MustParseHex("#D94801"), 0.250},
            {MustParseHex("#F16913"), 0.375},
            {MustParseHex("#FD8D3C"), 0.500},
            {MustParseHex("#FDAE6B"), 0.625},
            {MustParseHex("#FDD0A2"), 0.750},
            {MustParseHex("#FEE6CE"), 0.875},
            {MustParseHex("#FFF5EB"), 1.000},
        },
        "Purples": GradientTable{
            {MustParseHex("#3F007D"), 0.000},
            {MustParseHex("#54278F"), 0.125},
            {MustParseHex("#6A51A3"), 0.250},
            {MustParseHex("#807DBA"), 0.375},
            {MustParseHex("#9E9AC8"), 0.500},
            {MustParseHex("#BCBDDC"), 0.625},
            {MustParseHex("#DADAEB"), 0.750},
            {MustParseHex("#EFEDF5"), 0.875},
            {MustParseHex("#FCFBFD"), 1.000},
        },
        "Reds": GradientTable{
            {MustParseHex("#67000D"), 0.000},
            {MustParseHex("#A50F15"), 0.125},
            {MustParseHex("#CB181D"), 0.250},
            {MustParseHex("#EF3B2C"), 0.375},
            {MustParseHex("#FB6A4A"), 0.500},
            {MustParseHex("#FC9272"), 0.625},
            {MustParseHex("#FCBBA1"), 0.750},
            {MustParseHex("#FEE0D2"), 0.875},
            {MustParseHex("#FFF5F0"), 1.000},
        },

        // Diverging
        "BrBG": GradientTable{
            {MustParseHex("#003C30"), 0.0},
            {MustParseHex("#01665E"), 0.1},
            {MustParseHex("#35978F"), 0.2},
            {MustParseHex("#80CDC1"), 0.3},
            {MustParseHex("#C7EAE5"), 0.4},
            {MustParseHex("#F5F5F5"), 0.5},
            {MustParseHex("#F6E8C3"), 0.6},
            {MustParseHex("#DFC27D"), 0.7},
            // The following contains a typo in green in colorbrewer2!
            {MustParseHex("#BFC22D"), 0.8},
            {MustParseHex("#8C510A"), 0.9},
            {MustParseHex("#543005"), 1.0},
        },
        "PiYG": GradientTable{
            {MustParseHex("#276419"), 0.0},
            {MustParseHex("#4D9221"), 0.1},
            {MustParseHex("#7FBC41"), 0.2},
            {MustParseHex("#B8E186"), 0.3},
            {MustParseHex("#E6F5D0"), 0.4},
            {MustParseHex("#F7F7F7"), 0.5},
            {MustParseHex("#FDE0EF"), 0.6},
            {MustParseHex("#F1B6DA"), 0.7},
            {MustParseHex("#DE77AE"), 0.8},
            {MustParseHex("#C51B7D"), 0.9},
            {MustParseHex("#8E0152"), 1.0},
        },
        "PRGn": GradientTable{
            {MustParseHex("#00441B"), 0.0},
            {MustParseHex("#1B7837"), 0.1},
            {MustParseHex("#5AAE61"), 0.2},
            {MustParseHex("#A6DBA0"), 0.3},
            {MustParseHex("#D9F0D3"), 0.4},
            {MustParseHex("#F7F7F7"), 0.5},
            {MustParseHex("#E7D4E8"), 0.6},
            {MustParseHex("#C2A5CF"), 0.7},
            {MustParseHex("#9970AB"), 0.8},
            {MustParseHex("#762A83"), 0.9},
            {MustParseHex("#40004B"), 1.0},
        },
        "PuOr": GradientTable{
            {MustParseHex("#2D004B"), 0.0},
            {MustParseHex("#542788"), 0.1},
            {MustParseHex("#8073AC"), 0.2},
            {MustParseHex("#B2ABD2"), 0.3},
            {MustParseHex("#D8DAEB"), 0.4},
            {MustParseHex("#F7F7F7"), 0.5},
            {MustParseHex("#FEE0B6"), 0.6},
            {MustParseHex("#FDB863"), 0.7},
            {MustParseHex("#E08214"), 0.8},
            {MustParseHex("#B35806"), 0.9},
            {MustParseHex("#7F3B08"), 1.0},
        },
        "RdBu": GradientTable{
            {MustParseHex("#053061"), 0.0},
            {MustParseHex("#2166AC"), 0.1},
            {MustParseHex("#4393C3"), 0.2},
            {MustParseHex("#92C5DE"), 0.3},
            {MustParseHex("#D1E5F0"), 0.4},
            {MustParseHex("#F7F7F7"), 0.5},
            {MustParseHex("#FDDBC7"), 0.6},
            {MustParseHex("#F4A582"), 0.7},
            {MustParseHex("#D6604D"), 0.8},
            {MustParseHex("#B2182B"), 0.9},
            {MustParseHex("#67001F"), 1.0},
        },
        "RdGy": GradientTable{
            {MustParseHex("#1A1A1A"), 0.0},
            {MustParseHex("#4D4D4D"), 0.1},
            {MustParseHex("#878787"), 0.2},
            {MustParseHex("#BABABA"), 0.3},
            {MustParseHex("#E0E0E0"), 0.4},
            {MustParseHex("#FFFFFF"), 0.5},
            {MustParseHex("#FDDBC7"), 0.6},
            {MustParseHex("#F4A582"), 0.7},
            {MustParseHex("#D6604D"), 0.8},
            {MustParseHex("#B2182B"), 0.9},
            {MustParseHex("#67001F"), 1.0},
        },
        "RdYlBu": GradientTable{
            {MustParseHex("#313695"), 0.0},
            {MustParseHex("#4575B4"), 0.1},
            {MustParseHex("#74ADD1"), 0.2},
            {MustParseHex("#ABD9E9"), 0.3},
            {MustParseHex("#E0F3F8"), 0.4},
            {MustParseHex("#FFFFBF"), 0.5},
            {MustParseHex("#FEE090"), 0.6},
            {MustParseHex("#FDAE61"), 0.7},
            {MustParseHex("#F46D43"), 0.8},
            {MustParseHex("#D73027"), 0.9},
            {MustParseHex("#A50026"), 1.0},
        },
        "RdYlGn": GradientTable{
            {MustParseHex("#006837"), 0.0},
            {MustParseHex("#1A9850"), 0.1},
            {MustParseHex("#66BD63"), 0.2},
            {MustParseHex("#A6D96A"), 0.3},
            {MustParseHex("#D9EF8B"), 0.4},
            // {MustParseHex("#FFFFBF"), 0.5},
            {MustParseHex("#FEE08B"), 0.6},
            {MustParseHex("#FDAE61"), 0.7},
            {MustParseHex("#F46D43"), 0.8},
            {MustParseHex("#D73027"), 0.9},
            {MustParseHex("#A50026"), 1.0},
        },
        "Spectral": GradientTable{
            {MustParseHex("#5e4fa2"), 0.0},
            {MustParseHex("#3288bd"), 0.1},
            {MustParseHex("#66c2a5"), 0.2},
            {MustParseHex("#abdda4"), 0.3},
            {MustParseHex("#e6f598"), 0.4},
            // {MustParseHex("#ffffbf"), 0.5},
            {MustParseHex("#f1f3a7"), 0.5},
            {MustParseHex("#fee090"), 0.6},
            {MustParseHex("#fdae61"), 0.7},
            {MustParseHex("#f46d43"), 0.8},
            {MustParseHex("#d53e4f"), 0.9},
            {MustParseHex("#9e0142"), 1.0},
        },
    }

    for name, keypoints := range colorschemes {
        CreateColorschemes(keypoints, name, int(*pw), int(*ph))
    }
}
