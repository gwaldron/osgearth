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

#if defined(_MSC_VER)
#define DW_ALIGNED(x) __declspec(align(x))
#else
#if defined(__GNUC__) || defined(__clang__)
#define DW_ALIGNED(x) __attribute__ ((aligned(x)))
#endif
#endif

#define DW_ZERO_MEMORY(x) memset(&x, 0, sizeof(x))

#define DW_SAFE_DELETE(x) if(x) { delete x; x = nullptr; }
#define DW_SAFE_DELETE_ARRAY(x) if(x) { delete[] x; x = nullptr; }