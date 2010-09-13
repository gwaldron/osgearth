//----------------------------------------------------------------------------
// Anti-Grain Geometry - Version 2.1 Lite 
// Copyright (C) 2002-2003 Maxim Shemanarev (McSeem)
//
// Permission to copy, use, modify, sell and distribute this software 
// is granted provided this copyright notice appears in all copies. 
// This software is provided "as is" without express or implied
// warranty, and with no claim as to its suitability for any purpose.
//
// The author gratefully acknowleges the support of David Turner, 
// Robert Wilhelm, and Werner Lemberg - the authors of the FreeType 
// libray - in producing this work. See http://www.freetype.org for details.
//
//----------------------------------------------------------------------------
// Contact: mcseem@antigrain.com
//          mcseemagg@yahoo.com
//          http://www.antigrain.com
//----------------------------------------------------------------------------
#ifndef AGGLITE_H
#define AGGLITE_H

#include <memory.h>

namespace agg
{
    //------------------------------------------------------------------------
    typedef signed char    int8;
    typedef unsigned char  int8u;
    typedef signed short   int16;
    typedef unsigned short int16u;
    typedef signed int     int32;
    typedef unsigned int   int32u;



    //========================================================================
    struct rgba8 
    {
        enum order { rgb, bgr };

        int8u r;
        int8u g;
        int8u b;
        int8u a;

        //--------------------------------------------------------------------
        rgba8() {}

        //--------------------------------------------------------------------
        rgba8(unsigned r_, unsigned g_, unsigned b_, unsigned a_=255) :
            r(int8u(r_)), g(int8u(g_)), b(int8u(b_)), a(int8u(a_)) {}

        //--------------------------------------------------------------------
        rgba8(unsigned packed, order o) : 
            r((o == rgb) ? ((packed >> 16) & 0xFF) : (packed & 0xFF)),
            g((packed >> 8)  & 0xFF),
            b((o == rgb) ? (packed & 0xFF) : ((packed >> 16) & 0xFF)),
            a(255) {}

        //--------------------------------------------------------------------
        void opacity(double a_)
        {
            if(a_ < 0.0) a_ = 0.0;
            if(a_ > 1.0) a_ = 1.0;
            a = int8u(a_ * 255.0);
        }

        //--------------------------------------------------------------------
        double opacity() const
        {
            return double(a) / 255.0;
        }

        //--------------------------------------------------------------------
        rgba8 gradient(rgba8 c, double k) const
        {
            rgba8 ret;
            int ik = int(k * 256);
            ret.r = int8u(int(r) + (((int(c.r) - int(r)) * ik) >> 8));
            ret.g = int8u(int(g) + (((int(c.g) - int(g)) * ik) >> 8));
            ret.b = int8u(int(b) + (((int(c.b) - int(b)) * ik) >> 8));
            ret.a = int8u(int(a) + (((int(c.a) - int(a)) * ik) >> 8));
            return ret;
        }

        rgba8 pre() const
        {
            return rgba8((r*a) >> 8, (g*a) >> 8, (b*a) >> 8, a);
        }
    };


    //========================================================================
    // Rendering buffer wrapper. This class does not know anything about 
    // memory organizations, all it does it keeps an array of pointers 
    // to each pixel row. The general rules of rendering are as follows.
    // 
    // 1. Allocate or create somehow a rendering buffer itself. Since 
    //    the library does not depend on any particular platform or
    //    architecture it was decided that it's your responsibility 
    //    to create and destroy rendering buffers properly. You can use
    //    any available mechanism to create it - you can use a system API 
    //    function, simple memory allocation, or even statically defined array. 
    //    You also should know the memory organization (or possible variants)
    //    in your system. For example, there's an R,G,B or B,G,R organizations 
    //    with one byte per component (three byter per pixel) is used very often. 
    //    So, if you intend to use class render_bgr24, for example, you should 
    //    allocate at least width*height*3 bytes of memory.
    //
    // 2. Create a rendering_buffer object and then call method attach(). It requires
    //    a pointer to the buffer itself, width and height of the buffer in 
    //    pixels, and the length of the row in bytes. All these values must 
    //    properly correspond to the memory organization. The argument stride
    //    is used because in reality the row length in bytes does not obligatory 
    //    correspond with the width of the image in pixels, i.e. it cannot be 
    //    simply calculated as width_in_pixels * bytes_per_pixel. For example, 
    //    it must be aligned to 4 bytes in Windows bitmaps. Besides, the value
    //    of stride can be negative - it depends on the order of displaying
    //    the rendering buffer - from top to bottom or from bottom to top. 
    //    In other words, if stride > 0 the pointers to each row will start 
    //    from the beginning of the buffer and increase. If it < 0, the pointers 
    //    start from the end of the buffer and decrease. It gives you an 
    //    additional degree of freedom.
    //    Method attach() can be called more than once. The execution time of it
    //    is very little, still it allocates memory of heigh * sizeof(char*) bytes
    //    and has a loop while(height--) {...}, so it's unreasonable to call it
    //    every time before drawing any single pixel :-)
    //
    // 3. Create an object (or a number of objects) of a rendering class, such as
    //    renderer_bgr24_solid, renderer_bgr24_image and so on. These classes
    //    require a pointer to the renderer_buffer object, but they do not perform
    //    any considerable operations except storing this pointer. So, rendering
    //    objects can be created on demand almost any time. These objects know 
    //    about concrete memory organization (this knowledge is hardcoded), so 
    //    actually, the memory you allocated or created in clause 1 should 
    //    actually be in correspondence to the needs of the rendering class.
    //  
    // 4. Rener your image using rendering classes, for example, rasterizer
    //  
    // 5. Display the result, or store it, or whatever. It's also your 
    //    responsibility and depends on the platform.
    //------------------------------------------------------------------------
    class rendering_buffer
    {
    public:
        ~rendering_buffer();

        //-----------------------------------------Initialization
        rendering_buffer(unsigned char* buf,
                         unsigned width, 
                         unsigned height,
                         int      stride);

        //-----------------------------------------Initialization
        void attach(unsigned char* buf,
                    unsigned width, 
                    unsigned height,
                    int      stride);

        //-----------------------------------------Acessors
        const unsigned char* buf()    const { return m_buf;    }
        unsigned             width()  const { return m_width;  }
        unsigned             height() const { return m_height; }
        int                  stride() const { return m_stride; }

        bool inbox(int x, int y) const
        {
            return x >= 0 && y >= 0 && x < int(m_width) && y < int(m_height);
        }
        
        unsigned abs_stride() const 
        { 
            return (m_stride < 0) ? unsigned(-m_stride) : unsigned(m_stride); 
        }

        unsigned char* row(unsigned y) { return m_rows[y];  }
        const unsigned char* row(unsigned y) const { return m_rows[y]; }
        
    private:
        rendering_buffer(const rendering_buffer&);
        const rendering_buffer& operator = (const rendering_buffer&);

    private:
        unsigned char*  m_buf;        // Pointer to renrdering buffer
        unsigned char** m_rows;       // Pointers to each row of the buffer
        unsigned        m_width;      // Width in pixels
        unsigned        m_height;     // Height in pixels
        int             m_stride;     // Number of bytes per row. Can be < 0
        unsigned        m_max_height; // Maximal current height
    };



    //========================================================================
    //
    // This class is used to transfer data from class outline (or a similar one)
    // to the rendering buffer. It's organized very simple. The class stores 
    // information of horizontal spans to render it into a pixel-map buffer. 
    // Each span has initial X, length, and an array of bytes that determine the 
    // alpha-values for each pixel. So, the restriction of using this class is 256 
    // levels of Anti-Aliasing, which is quite enough for any practical purpose.
    // Before using this class you should know the minimal and maximal pixel 
    // coordinates of your scanline. The protocol of using is:
    // 1. reset(min_x, max_x)
    // 2. add_cell() / add_span() - accumulate scanline. You pass Y-coordinate 
    //    into these functions in order to make scanline know the last Y. Before 
    //    calling add_cell() / add_span() you should check with method is_ready(y)
    //    if the last Y has changed. It also checks if the scanline is not empty. 
    //    When forming one scanline the next X coordinate must be always greater
    //    than the last stored one, i.e. it works only with ordered coordinates.
    // 3. If the current scanline is_ready() you should render it and then call 
    //    reset_spans() before adding new cells/spans.
    //    
    // 4. Rendering:
    // 
    // Scanline provides an iterator class that allows you to extract
    // the spans and the cover values for each pixel. Be aware that clipping
    // has not been done yet, so you should perform it yourself.
    // Use scanline::iterator to render spans:
    //-------------------------------------------------------------------------
    //
    // int base_x = sl.base_x();          // base X. Should be added to the span's X
    //                                    // "sl" is a const reference to the 
    //                                    // scanline passed in.
    //
    // int y = sl.y();                    // Y-coordinate of the scanline
    //
    // ************************************
    // ...Perform vertical clipping here...
    // ************************************
    //
    // scanline::iterator span(sl);
    // 
    // unsigned char* row = m_rbuf->row(y); // The the address of the beginning 
    //                                      // of the current row
    // 
    // unsigned num_spans = sl.num_spans(); // Number of spans. It's guaranteed that
    //                                      // num_spans is always greater than 0.
    //
    // do
    // {
    //     int x = span.next() + base_x;        // The beginning X of the span
    //
    //     const int8u covers* = span.covers(); // The array of the cover values
    //
    //     int num_pix = span.num_pix();        // Number of pixels of the span.
    //                                          // Always greater than 0, still we
    //                                          // shoud use "int" instead of 
    //                                          // "unsigned" because it's more
    //                                          // convenient for clipping
    //
    //     **************************************
    //     ...Perform horizontal clipping here...
    //     ...you have x, covers, and pix_count..
    //     **************************************
    //
    //     unsigned char* dst = row + x;  // Calculate the start address of the row.
    //                                    // In this case we assume a simple 
    //                                    // grayscale image 1-byte per pixel.
    //     do
    //     {
    //         *dst++ = *covers++;        // Hypotetical rendering. 
    //     }
    //     while(--num_pix);
    // } 
    // while(--num_spans);  // num_spans cannot be 0, so this loop is quite safe
    //------------------------------------------------------------------------
    //
    // The question is: why should we accumulate the whole scanline when we
    // could render just separate spans when they're ready?
    // That's because using the scaline is in general faster. When is consists 
    // of more than one span the conditions for the processor cash system
    // are better, because switching between two different areas of memory 
    // (that can be large ones) occures less frequently.
    //------------------------------------------------------------------------
    class scanline
    {
    public:
        enum { aa_shift = 8 };

        class iterator
        {
        public:
            iterator(const scanline& sl) :
                m_covers(sl.m_covers),
                m_cur_count(sl.m_counts),
                m_cur_start_ptr(sl.m_start_ptrs)
            {
            }

            int next()
            {
                ++m_cur_count;
                ++m_cur_start_ptr;
                return int(*m_cur_start_ptr - m_covers);
            }

            int num_pix() const { return int(*m_cur_count); }
            const int8u* covers() const { return *m_cur_start_ptr; }

        private:
            const int8u*        m_covers;
            const int16u*       m_cur_count;
            const int8u* const* m_cur_start_ptr;
        };

        friend class iterator;

        ~scanline();
        scanline();

        void     reset(int min_x, int max_x, int dx=0, int dy=0);

        void     reset_spans();
        void     add_cell(int x, int y, unsigned cover);
        void     add_span(int x, int y, unsigned len, unsigned cover);
        int      is_ready(int y) const;
        int      base_x()    const { return m_min_x + m_dx;  }
        int      y()         const { return m_last_y + m_dy; }
        unsigned num_spans() const { return m_num_spans;     }

    private:
        scanline(const scanline&);
        const scanline& operator = (const scanline&);

    private:
        int      m_min_x;
        unsigned m_max_len;
        int      m_dx;
        int      m_dy;
        int      m_last_x;
        int      m_last_y;
        int8u*   m_covers;
        int8u**  m_start_ptrs;
        int16u*  m_counts;
        unsigned m_num_spans;
        int8u**  m_cur_start_ptr;
        int16u*  m_cur_count;
    };


    //------------------------------------------------------------------------
    inline void scanline::reset_spans()
    {
        m_last_x        = 0x7FFF;
        m_last_y        = 0x7FFF;
        m_cur_count     = m_counts;
        m_cur_start_ptr = m_start_ptrs;
        m_num_spans     = 0;
    }


    //------------------------------------------------------------------------
    inline void scanline::add_cell(int x, int y, unsigned cover)
    {
        x -= m_min_x;
        m_covers[x] = (unsigned char)cover;
        if(x == m_last_x+1)
        {
            (*m_cur_count)++;
        }
        else
        {
            *++m_cur_count = 1;
            *++m_cur_start_ptr = m_covers + x;
            m_num_spans++;
        }
        m_last_x = x;
        m_last_y = y;
    }


    //------------------------------------------------------------------------
    inline int scanline::is_ready(int y) const
    {
        return m_num_spans && (y ^ m_last_y);
    }




    //========================================================================
    // This class template is used basically for rendering scanlines. 
    // The 'Span' argument is one of the span renderers, such as span_rgb24 
    // and others.
    // 
    // Usage:
    // 
    //     // Creation
    //     agg::rendering_buffer rbuf(ptr, w, h, stride);
    //     agg::renderer<agg::span_rgb24> ren(rbuf);
    //     agg::rasterizer ras;
    //
    //     // Clear the frame buffer
    //     ren.clear(agg::rgba8(0,0,0));
    //
    //     // Making polygon
    //     // ras.move_to(. . .);
    //     // ras.line_to(. . .);
    //     // . . .
    //
    //     // Rendering
    //     ras.render(ren, agg::rgba8(200, 100, 80));
    //  
    //------------------------------------------------------------------------
    template<class Span> class renderer
    {
    public:
        //--------------------------------------------------------------------
        renderer(rendering_buffer& rbuf) : m_rbuf(&rbuf)
        {
        }
        
        //--------------------------------------------------------------------
        void clear(const rgba8& c)
        {
            unsigned y;
            for(y = 0; y < m_rbuf->height(); y++)
            {
                m_span.hline(m_rbuf->row(y), 0, m_rbuf->width(), c);
            }
        }

        //--------------------------------------------------------------------
        void pixel(int x, int y, const rgba8& c)
        {
            if(m_rbuf->inbox(x, y))
            {
                m_span.hline(m_rbuf->row(y), x, 1, c);
            }
        }

        //--------------------------------------------------------------------
        rgba8 pixel(int x, int y) const
        {
            if(m_rbuf->inbox(x, y))
            {
                return m_span.get(m_rbuf->row(y), x);
            }
            return rgba8(0,0,0);
        }

        //--------------------------------------------------------------------
        void render(const scanline& sl, const rgba8& c)
        {
            if(sl.y() < 0 || sl.y() >= int(m_rbuf->height()))
            {
                return;
            }

            unsigned num_spans = sl.num_spans();
            int base_x = sl.base_x();
            unsigned char* row = m_rbuf->row(sl.y());
            scanline::iterator span(sl);

            do
            {
                int x = span.next() + base_x;
                const int8u* covers = span.covers();
                int num_pix = span.num_pix();
                if(x < 0)
                {
                    num_pix += x;
                    if(num_pix <= 0) continue;
                    covers -= x;
                    x = 0;
                }
                if(x + num_pix >= int(m_rbuf->width()))
                {
                    num_pix = m_rbuf->width() - x;
                    if(num_pix <= 0) continue;
                }
                m_span.render(row, x, num_pix, covers, c);
            }
            while(--num_spans);
        }

        //--------------------------------------------------------------------
        rendering_buffer& rbuf() { return *m_rbuf; }

    private:
        rendering_buffer* m_rbuf;
        Span              m_span;
    };


    //------------------------------------------------------------------------
    // These constants determine the subpixel accuracy, to be more precise, 
    // the number of bits of the fractional part of the coordinates. 
    // The possible coordinate capacity in bits can be calculated by formula:
    // sizeof(int) * 8 - poly_base_shift * 2, i.e, for 32-bit integers and
    // 8-bits fractional part the capacity is 16 bits or [-32768...32767].
    enum
    {
        poly_base_shift = 8,
        poly_base_size  = 1 << poly_base_shift,
        poly_base_mask  = poly_base_size - 1
    };
    
    //------------------------------------------------------------------------
    inline int poly_coord(double c)
    {
        return int(c * poly_base_size);
    }

    //------------------------------------------------------------------------
    // A pixel cell. There're no constructors defined and it was done 
    // intentionally in order to avoid extra overhead when allocating an 
    // array of cells.
    struct cell
    {
        int16 x;
        int16 y;
        int   packed_coord;
        int   cover;
        int   area;

        void set(int x, int y, int c, int a);
        void set_coord(int x, int y);
        void set_cover(int c, int a);
        void add_cover(int c, int a);
    };


    //------------------------------------------------------------------------
    // An internal class that implements the main rasterization algorithm.
    // Used in the rasterizer. Should not be used direcly.
    class outline
    {
        enum
        {
            cell_block_shift = 12,
            cell_block_size  = 1 << cell_block_shift,
            cell_block_mask  = cell_block_size - 1,
            cell_block_pool  = 256,
            cell_block_limit = 1024
        };

    public:

        ~outline();
        outline();

        void reset();

        void move_to(int x, int y);
        void line_to(int x, int y);

        int min_x() const { return m_min_x; }
        int min_y() const { return m_min_y; }
        int max_x() const { return m_max_x; }
        int max_y() const { return m_max_y; }

        unsigned num_cells() const {return m_num_cells; }
        const cell* const* cells();

    private:
        outline(const outline&);
        const outline& operator = (const outline&);

        void set_cur_cell(int x, int y);
        void add_cur_cell();
        void sort_cells();
        void render_scanline(int ey, int x1, int y1, int x2, int y2);
        void render_line(int x1, int y1, int x2, int y2);
        void allocate_block();
        
        static void qsort_cells(cell** start, unsigned num);

    private:
        unsigned  m_num_blocks;
        unsigned  m_max_blocks;
        unsigned  m_cur_block;
        unsigned  m_num_cells;
        cell**    m_cells;
        cell*     m_cur_cell_ptr;
        cell**    m_sorted_cells;
        unsigned  m_sorted_size;
        cell      m_cur_cell;
        int       m_cur_x;
        int       m_cur_y;
        int       m_close_x;
        int       m_close_y;
        int       m_min_x;
        int       m_min_y;
        int       m_max_x;
        int       m_max_y;
        unsigned  m_flags;
    };


    //------------------------------------------------------------------------
    enum filling_rule_e
    {
        fill_non_zero,
        fill_even_odd
    };
    

    //========================================================================
    // Polygon rasterizer that is used to render filled polygons with 
    // high-quality Anti-Aliasing. Internally, by default, the class uses 
    // integer coordinates in format 24.8, i.e. 24 bits for integer part 
    // and 8 bits for fractional - see poly_base_shift. This class can be 
    // used in the following  way:
    //
    // 1. filling_rule(filling_rule_e ft) - optional.
    //
    // 2. gamma() - optional.
    //
    // 3. reset()
    //
    // 4. move_to(x, y) / line_to(x, y) - make the polygon. One can create 
    //    more than one contour, but each contour must consist of at least 3
    //    vertices, i.e. move_to(x1, y1); line_to(x2, y2); line_to(x3, y3);
    //    is the absolute minimum of vertices that define a triangle.
    //    The algorithm does not check either the number of vertices nor
    //    coincidence of their coordinates, but in the worst case it just 
    //    won't draw anything.
    //    The orger of the vertices (clockwise or counterclockwise) 
    //    is important when using the non-zero filling rule (fill_non_zero).
    //    In this case the vertex order of all the contours must be the same
    //    if you want your intersecting polygons to be without "holes".
    //    You actually can use different vertices order. If the contours do not 
    //    intersect each other the order is not important anyway. If they do, 
    //    contours with the same vertex order will be rendered without "holes" 
    //    while the intersecting contours with different orders will have "holes".
    //
    // filling_rule() and gamma() can be called anytime before "sweeping".
    //------------------------------------------------------------------------
    class rasterizer
    {
    public:
        enum
        {
            aa_shift = scanline::aa_shift,
            aa_num   = 1 << aa_shift,
            aa_mask  = aa_num - 1,
            aa_2num  = aa_num * 2,
            aa_2mask = aa_2num - 1
        };

        rasterizer() :
            m_filling_rule(fill_non_zero)
        {
            memcpy(m_gamma, s_default_gamma, sizeof(m_gamma));
        }

        //--------------------------------------------------------------------
        void reset() { m_outline.reset(); }

        //--------------------------------------------------------------------
        void filling_rule(filling_rule_e filling_rule) 
        { 
            m_filling_rule = filling_rule; 
        }

        //--------------------------------------------------------------------
        void gamma(double g);
        void gamma(const int8u* g);

        //--------------------------------------------------------------------
        void move_to(int x, int y) { m_outline.move_to(x, y); }
        void line_to(int x, int y) { m_outline.line_to(x, y); }

        //--------------------------------------------------------------------
        void move_to_d(double x, double y) { m_outline.move_to(poly_coord(x), 
                                                               poly_coord(y)); }
        void line_to_d(double x, double y) { m_outline.line_to(poly_coord(x), 
                                                               poly_coord(y)); }

        //--------------------------------------------------------------------
        int min_x() const { return m_outline.min_x(); }
        int min_y() const { return m_outline.min_y(); }
        int max_x() const { return m_outline.max_x(); }
        int max_y() const { return m_outline.max_y(); }

        //--------------------------------------------------------------------
        unsigned calculate_alpha(int area) const
        {
            int cover = area >> (poly_base_shift*2 + 1 - aa_shift);

            if(cover < 0) cover = -cover;
            if(m_filling_rule == fill_even_odd)
            {
                cover &= aa_2mask;
                if(cover > aa_num)
                {
                    cover = aa_2num - cover;
                }
            }
            if(cover > aa_mask) cover = aa_mask;
            return cover;
        }

        //--------------------------------------------------------------------
        template<class Renderer> void render(Renderer& r, 
                                             const rgba8& c, 
                                             int dx=0, 
                                             int dy=0)
        {
            const cell* const* cells = m_outline.cells();
            if(m_outline.num_cells() == 0) return;

            int x, y;
            int cover;
            int alpha;
            int area;

            m_scanline.reset(m_outline.min_x(), m_outline.max_x(), dx, dy);

            cover = 0;
            const cell* cur_cell = *cells++;
            for(;;)
            {
                const cell* start_cell = cur_cell;

                int coord  = cur_cell->packed_coord;
                x = cur_cell->x;
                y = cur_cell->y;

                area   = start_cell->area;
                cover += start_cell->cover;

                //accumulate all start cells
                while((cur_cell = *cells++) != 0)
                {
                    if(cur_cell->packed_coord != coord) break;
                    area  += cur_cell->area;
                    cover += cur_cell->cover;
                }

                if(area)
                {
                    alpha = calculate_alpha((cover << (poly_base_shift + 1)) - area);
                    if(alpha)
                    {
                        if(m_scanline.is_ready(y))
                        {
                            r.render(m_scanline, c);
                            m_scanline.reset_spans();
                        }
                        m_scanline.add_cell(x, y, m_gamma[alpha]);
                    }
                    x++;
                }

                if(!cur_cell) break;

                if(cur_cell->x > x)
                {
                    alpha = calculate_alpha(cover << (poly_base_shift + 1));
                    if(alpha)
                    {
                        if(m_scanline.is_ready(y))
                        {
                            r.render(m_scanline, c);
                            m_scanline.reset_spans();
                        }
                        m_scanline.add_span(x, y, 
                                            cur_cell->x - x, 
                                            m_gamma[alpha]);
                    }
                }
            } 
        
            if(m_scanline.num_spans())
            {
                r.render(m_scanline, c);
            }
        }


        //--------------------------------------------------------------------
        bool hit_test(int tx, int ty);

    private:
        rasterizer(const rasterizer&);
        const rasterizer& operator = (const rasterizer&);

    private:
        outline        m_outline;
        scanline       m_scanline;
        filling_rule_e m_filling_rule;
        int8u          m_gamma[256];
        static const int8u s_default_gamma[256];     
    };


    //========================================================================
    struct span_mono8
    {
        //--------------------------------------------------------------------
        static unsigned mono8(unsigned r, unsigned g, unsigned b)
        {
            return (r * 77 + g * 150 + b * 29) >> 8;
        }

        //--------------------------------------------------------------------
        static void render(unsigned char* ptr, 
                           int x,
                           unsigned count, 
                           const unsigned char* covers, 
                           const rgba8& c)
        {
            unsigned char* p = ptr + x;
            unsigned dst = mono8(c.r, c.g, c.b);
            do
            {
                int alpha = (*covers++) * c.a;
                unsigned src = *p;
                *p++ = (((dst - src) * alpha) + (src << 16)) >> 16;
            }
            while(--count);
        }

        //--------------------------------------------------------------------
        static void hline(unsigned char* ptr, 
                          int x,
                          unsigned count, 
                          const rgba8& c)
        {
            unsigned char* p = ptr + x;
            unsigned v = mono8(c.r, c.g, c.b);
            do { *p++ = v; } while(--count);
        }

        //--------------------------------------------------------------------
        static rgba8 get(unsigned char* ptr, int x)
        {
            unsigned rgb = ptr[x];
            rgba8 c;
            c.r = rgb; 
            c.g = rgb; 
            c.b = rgb;
            c.a = 255;
            return c;
        }
    };



    //========================================================================
    struct span_rgb555
    {
        //--------------------------------------------------------------------
        static int16u rgb555(unsigned r, unsigned g, unsigned b)
        {
            return ((r & 0xF8) << 7) | ((g & 0xF8) << 2) | (b >> 3);
        }

        //--------------------------------------------------------------------
        static void render(unsigned char* ptr, 
                           int x,
                           unsigned count, 
                           const unsigned char* covers, 
                           const rgba8& c)
        {
            int16u* p = ((int16u*)ptr) + x;
            do
            {
                int16 rgb = *p;
                int alpha = (*covers++) * c.a;

                int r = (rgb >> 7) & 0xF8;
                int g = (rgb >> 2) & 0xF8;
                int b = (rgb << 3) & 0xF8;

                *p++ = (((((c.r - r) * alpha) + (r << 16)) >> 9) & 0x7C00) |
                       (((((c.g - g) * alpha) + (g << 16)) >> 14) & 0x3E0) |
                        ((((c.b - b) * alpha) + (b << 16)) >> 19);
            }
            while(--count);
        }

        //--------------------------------------------------------------------
        static void hline(unsigned char* ptr, 
                          int x,
                          unsigned count, 
                          const rgba8& c)
        {
            int16u* p = ((int16u*)ptr) + x;
            int16u  v = rgb555(c.r, c.g, c.b);
            do { *p++ = v; } while(--count);
        }

        //--------------------------------------------------------------------
        static rgba8 get(unsigned char* ptr, int x)
        {
            int16u rgb = ((int16u*)ptr)[x];
            rgba8 c;
            c.r = (rgb >> 7) & 0xF8; 
            c.g = (rgb >> 2) & 0xF8; 
            c.b = (rgb << 3) & 0xF8;
            c.a = 255;
            return c;
        }
    };




    //========================================================================
    struct span_rgb565
    {
        //--------------------------------------------------------------------
        static int16u rgb565(unsigned r, unsigned g, unsigned b)
        {
            return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
        }

        //--------------------------------------------------------------------
        static void render(unsigned char* ptr, 
                           int x,
                           unsigned count, 
                           const unsigned char* covers, 
                           const rgba8& c)
        {
            int16u* p = ((int16u*)ptr) + x;
            do
            {
                int16 rgb = *p;
                int alpha = (*covers++) * c.a;

                int r = (rgb >> 8) & 0xF8;
                int g = (rgb >> 3) & 0xFC;
                int b = (rgb << 3) & 0xF8;

                *p++ = (((((c.r - r) * alpha) + (r << 16)) >> 8) & 0xF800) |
                       (((((c.g - g) * alpha) + (g << 16)) >> 13) & 0x7E0) |
                        ((((c.b - b) * alpha) + (b << 16)) >> 19);
            }
            while(--count);
        }

        //--------------------------------------------------------------------
        static void hline(unsigned char* ptr, 
                          int x,
                          unsigned count, 
                          const rgba8& c)
        {
            int16u* p = ((int16u*)ptr) + x;
            int16u  v = rgb565(c.r, c.g, c.b);
            do { *p++ = v; } while(--count);
        }

        //--------------------------------------------------------------------
        static rgba8 get(unsigned char* ptr, int x)
        {
            int16u rgb = ((int16u*)ptr)[x];
            rgba8 c;
            c.r = (rgb >> 8) & 0xF8; 
            c.g = (rgb >> 3) & 0xFC; 
            c.b = (rgb << 3) & 0xF8;
            c.a = 255;
            return c;
        }
    };



    //========================================================================
    struct span_bgr24
    {
        //--------------------------------------------------------------------
        static void render(unsigned char* ptr, 
                           int x,
                           unsigned count, 
                           const unsigned char* covers, 
                           const rgba8& c)
        {
            unsigned char* p = ptr + x + x + x;
            do
            {
                int alpha = (*covers++) * c.a;
                int b = p[0];
                int g = p[1];
                int r = p[2];
                *p++ = (((c.b - b) * alpha) + (b << 16)) >> 16;
                *p++ = (((c.g - g) * alpha) + (g << 16)) >> 16;
                *p++ = (((c.r - r) * alpha) + (r << 16)) >> 16;
            }
            while(--count);
        }

        //--------------------------------------------------------------------
        static void hline(unsigned char* ptr, 
                          int x,
                          unsigned count, 
                          const rgba8& c)
        {
            unsigned char* p = ptr + x + x + x;
            do { *p++ = c.b; *p++ = c.g; *p++ = c.r; } while(--count);
        }

        //--------------------------------------------------------------------
        static rgba8 get(unsigned char* ptr, int x)
        {
            unsigned char* p = ptr + x + x + x;
            rgba8 c;
            c.b = *p++; 
            c.g = *p++; 
            c.r = *p++;
            c.a = 255;
            return c;
        }
    };



    //========================================================================
    struct span_rgb24
    {
        //--------------------------------------------------------------------
        static void render(unsigned char* ptr, 
                           int x,
                           unsigned count, 
                           const unsigned char* covers, 
                           const rgba8& c)
        {
            unsigned char* p = ptr + x + x + x;
            do
            {
                int alpha = (*covers++) * c.a;
                int r = p[0];
                int g = p[1];
                int b = p[2];
                *p++ = (((c.r - r) * alpha) + (r << 16)) >> 16;
                *p++ = (((c.g - g) * alpha) + (g << 16)) >> 16;
                *p++ = (((c.b - b) * alpha) + (b << 16)) >> 16;
            }
            while(--count);
        }

        //--------------------------------------------------------------------
        static void hline(unsigned char* ptr, 
                          int x,
                          unsigned count, 
                          const rgba8& c)
        {
            unsigned char* p = ptr + x + x + x;
            do { *p++ = c.r; *p++ = c.g; *p++ = c.b; } while(--count);
        }

        //--------------------------------------------------------------------
        static rgba8 get(unsigned char* ptr, int x)
        {
            unsigned char* p = ptr + x + x + x;
            rgba8 c;
            c.r = *p++; 
            c.g = *p++; 
            c.b = *p++;
            c.a = 255;
            return c;
        }
    };



    //========================================================================
    struct span_abgr32
    {
        //--------------------------------------------------------------------
        static void render(unsigned char* ptr, 
                           int x,
                           unsigned count, 
                           const unsigned char* covers, 
                           const rgba8& c)
        {
            unsigned char* p = ptr + (x << 2);
            do
            {
                int alpha = (*covers++) * c.a;
                int a = p[0];
                int b = p[1];
                int g = p[2];
                int r = p[3];
                *p++ = (((c.a - a) * alpha) + (a << 16)) >> 16;
                *p++ = (((c.b - b) * alpha) + (b << 16)) >> 16;
                *p++ = (((c.g - g) * alpha) + (g << 16)) >> 16;
                *p++ = (((c.r - r) * alpha) + (r << 16)) >> 16;
            }
            while(--count);
        }

        //--------------------------------------------------------------------
        static void hline(unsigned char* ptr, 
                          int x,
                          unsigned count, 
                          const rgba8& c)
        {
            unsigned char* p = ptr + (x << 2);
            do { *p++ = c.a; *p++ = c.b; *p++ = c.g; *p++ = c.r; } while(--count);
        }

        //--------------------------------------------------------------------
        static rgba8 get(unsigned char* ptr, int x)
        {
            unsigned char* p = ptr + (x << 2);
            rgba8 c;
            c.a = *p++; 
            c.b = *p++; 
            c.g = *p++;
            c.r = *p;
            return c;
        }
    };




    //========================================================================
    struct span_argb32
    {
        //--------------------------------------------------------------------
        static void render(unsigned char* ptr, 
                           int x,
                           unsigned count, 
                           const unsigned char* covers, 
                           const rgba8& c)
        {
            unsigned char* p = ptr + (x << 2);
            do
            {
                int alpha = (*covers++) * c.a;
                int a = p[0];
                int r = p[1];
                int g = p[2];
                int b = p[3];
                *p++ = (((c.a - a) * alpha) + (a << 16)) >> 16;
                *p++ = (((c.r - r) * alpha) + (r << 16)) >> 16;
                *p++ = (((c.g - g) * alpha) + (g << 16)) >> 16;
                *p++ = (((c.b - b) * alpha) + (b << 16)) >> 16;
            }
            while(--count);
        }

        //--------------------------------------------------------------------
        static void hline(unsigned char* ptr, 
                          int x,
                          unsigned count, 
                          const rgba8& c)
        {
            unsigned char* p = ptr + (x << 2);
            do { *p++ = c.a; *p++ = c.r; *p++ = c.g; *p++ = c.b; } while(--count);
        }

        //--------------------------------------------------------------------
        static rgba8 get(unsigned char* ptr, int x)
        {
            unsigned char* p = ptr + (x << 2);
            rgba8 c;
            c.a = *p++; 
            c.r = *p++; 
            c.g = *p++;
            c.b = *p;
            return c;
        }
    };



    //========================================================================
    struct span_bgra32
    {
        //--------------------------------------------------------------------
        static void render(unsigned char* ptr, 
                           int x,
                           unsigned count, 
                           const unsigned char* covers, 
                           const rgba8& c)
        {
            unsigned char* p = ptr + (x << 2);
            do
            {
                int alpha = (*covers++) * c.a;
                int b = p[0];
                int g = p[1];
                int r = p[2];
                int a = p[3];
                *p++ = (((c.b - b) * alpha) + (b << 16)) >> 16;
                *p++ = (((c.g - g) * alpha) + (g << 16)) >> 16;
                *p++ = (((c.r - r) * alpha) + (r << 16)) >> 16;
                *p++ = (((c.a - a) * alpha) + (a << 16)) >> 16;
            }
            while(--count);
        }

        //--------------------------------------------------------------------
        static void hline(unsigned char* ptr, 
                          int x,
                          unsigned count, 
                          const rgba8& c)
        {
            unsigned char* p = ptr + (x << 2);
            do { *p++ = c.b; *p++ = c.g; *p++ = c.r; *p++ = c.a; } while(--count);
        }

        //--------------------------------------------------------------------
        static rgba8 get(unsigned char* ptr, int x)
        {
            unsigned char* p = ptr + (x << 2);
            rgba8 c;
            c.b = *p++; 
            c.g = *p++; 
            c.r = *p++;
            c.a = *p;
            return c;
        }
    };




    //========================================================================
    struct span_rgba32
    {
        //--------------------------------------------------------------------
        static void render(unsigned char* ptr, 
                           int x,
                           unsigned count, 
                           const unsigned char* covers, 
                           const rgba8& c)
        {
            unsigned char* p = ptr + (x << 2);
            do
            {
                int alpha = (*covers++) * c.a;
                int r = p[0];
                int g = p[1];
                int b = p[2];
                int a = p[3];
                *p++ = (((c.r - r) * alpha) + (r << 16)) >> 16;
                *p++ = (((c.g - g) * alpha) + (g << 16)) >> 16;
                *p++ = (((c.b - b) * alpha) + (b << 16)) >> 16;
                *p++ = (((c.a - a) * alpha) + (a << 16)) >> 16;
            }
            while(--count);
        }

        //--------------------------------------------------------------------
        static void hline(unsigned char* ptr, 
                          int x,
                          unsigned count, 
                          const rgba8& c)
        {
            unsigned char* p = ptr + (x << 2);
            do { *p++ = c.r; *p++ = c.g; *p++ = c.b; *p++ = c.a; } while(--count);
        }

        //--------------------------------------------------------------------
        static rgba8 get(unsigned char* ptr, int x)
        {
            unsigned char* p = ptr + (x << 2);
            rgba8 c;
            c.r = *p++; 
            c.g = *p++; 
            c.b = *p++;
            c.a = *p;
            return c;
        }
    };
}

// ===== agg.cpp ===== //

namespace agg
{
    //========================================================================

    //------------------------------------------------------------------------
    rendering_buffer::~rendering_buffer()
    {
        delete [] m_rows;
    }


    //------------------------------------------------------------------------
    rendering_buffer::rendering_buffer(unsigned char* buf,
                                       unsigned width, 
                                       unsigned height,
                                       int      stride) :
        m_buf(0),
        m_rows(0),
        m_width(0),
        m_height(0),
        m_stride(0),
        m_max_height(0)
    {
        attach(buf, width, height, stride);
    }


    //------------------------------------------------------------------------
    void rendering_buffer::attach(unsigned char* buf,
                                  unsigned width, 
                                  unsigned height,
                                  int      stride)
    {
        m_buf = buf;
        m_width = width;
        m_height = height;
        m_stride = stride;
        if(height > m_max_height)
        {
            delete [] m_rows;
            m_rows = new unsigned char* [m_max_height = height];
        }

        unsigned char* row_ptr = m_buf;

        if(stride < 0)
        {
            row_ptr = m_buf - int(height - 1) * stride;
        }

        unsigned char** rows = m_rows;

        while(height--)
        {
            *rows++ = row_ptr;
            row_ptr += stride;
        }
    }


    //========================================================================

    //------------------------------------------------------------------------
    scanline::~scanline()
    {
        if ( m_counts ) delete [] m_counts;
        if ( m_start_ptrs ) delete [] m_start_ptrs;
        if ( m_covers ) delete [] m_covers;
    }


    //------------------------------------------------------------------------
    scanline::scanline()
        : m_min_x(0),
          m_max_len(0),
          m_dx(0),
          m_dy(0),
          m_last_x(0x7FFF),
          m_last_y(0x7FFF),
          m_covers(0),
          m_start_ptrs(0),
          m_counts(0),
          m_num_spans(0),
          m_cur_start_ptr(0),
          m_cur_count(0)
    {
    }


    //------------------------------------------------------------------------
    void scanline::reset(int min_x, int max_x, int dx, int dy)
    {
        unsigned max_len = max_x - min_x + 2;
        if(max_len > m_max_len)
        {
            if ( m_counts ) delete [] m_counts;
            if ( m_start_ptrs ) delete [] m_start_ptrs;
            if ( m_covers ) delete [] m_covers;
            m_covers     = new unsigned char  [max_len];
            m_start_ptrs = new unsigned char* [max_len];
            m_counts     = new int16u[max_len];
            m_max_len    = max_len;
        }
        m_dx            = dx;
        m_dy            = dy;
        m_last_x        = 0x7FFF;
        m_last_y        = 0x7FFF;
        m_min_x         = min_x;
        m_cur_count     = m_counts;
        m_cur_start_ptr = m_start_ptrs;
        m_num_spans     = 0;
    }


    //------------------------------------------------------------------------
    void scanline::add_span(int x, int y, unsigned num, unsigned cover)
    {
        x -= m_min_x;

        memset(m_covers + x, cover, num);
        if(x == m_last_x+1)
        {
            (*m_cur_count) += (int16u)num;
        }
        else
        {
            *++m_cur_count = (int16u)num;
            *++m_cur_start_ptr = m_covers + x;
            m_num_spans++;
        }
        m_last_x = x + num - 1;
        m_last_y = y;
    }



    //========================================================================

    //------------------------------------------------------------------------
    const int8u rasterizer::s_default_gamma[] = 
    {
          0,  0,  1,  1,  2,  2,  3,  4,  4,  5,  5,  6,  7,  7,  8,  8,
          9, 10, 10, 11, 11, 12, 13, 13, 14, 14, 15, 16, 16, 17, 18, 18,
         19, 19, 20, 21, 21, 22, 22, 23, 24, 24, 25, 25, 26, 27, 27, 28,
         29, 29, 30, 30, 31, 32, 32, 33, 34, 34, 35, 36, 36, 37, 37, 38,
         39, 39, 40, 41, 41, 42, 43, 43, 44, 45, 45, 46, 47, 47, 48, 49,
         49, 50, 51, 51, 52, 53, 53, 54, 55, 55, 56, 57, 57, 58, 59, 60,
         60, 61, 62, 62, 63, 64, 65, 65, 66, 67, 68, 68, 69, 70, 71, 71,
         72, 73, 74, 74, 75, 76, 77, 78, 78, 79, 80, 81, 82, 83, 83, 84,
         85, 86, 87, 88, 89, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99,
        100,101,101,102,103,104,105,106,107,108,109,110,111,112,114,115,
        116,117,118,119,120,121,122,123,124,126,127,128,129,130,131,132,
        134,135,136,137,139,140,141,142,144,145,146,147,149,150,151,153,
        154,155,157,158,159,161,162,164,165,166,168,169,171,172,174,175,
        177,178,180,181,183,184,186,188,189,191,192,194,195,197,199,200,
        202,204,205,207,209,210,212,214,215,217,219,220,222,224,225,227,
        229,230,232,234,236,237,239,241,242,244,246,248,249,251,253,255
    };





    //========================================================================
    enum
    {
        not_closed    = 1,
        sort_required = 2
    };

    //------------------------------------------------------------------------
    inline void cell::set_cover(int c, int a)
    {
        cover = c;
        area = a;
    }

    //------------------------------------------------------------------------
    inline void cell::add_cover(int c, int a)
    {
        cover += c;
        area += a;
    }

    //------------------------------------------------------------------------
    inline void cell::set_coord(int cx, int cy)
    {
        x = int16(cx);
        y = int16(cy);
        packed_coord = (cy << 16) + cx;
    }

    //------------------------------------------------------------------------
    inline void cell::set(int cx, int cy, int c, int a)
    {
        x = int16(cx);
        y = int16(cy);
        packed_coord = (cy << 16) + cx;
        cover = c;
        area = a;
    }

    //------------------------------------------------------------------------
    outline::~outline()
    {
        delete [] m_sorted_cells;
        if(m_num_blocks)
        {
            cell** ptr = m_cells + m_num_blocks - 1;
            while(m_num_blocks--)
            {
                delete [] *ptr;
                ptr--;
            }
            delete [] m_cells;
        }
    }


    //------------------------------------------------------------------------
    outline::outline() :
        m_num_blocks(0),
        m_max_blocks(0),
        m_cur_block(0),
        m_num_cells(0),
        m_cells(0),
        m_cur_cell_ptr(0),
        m_sorted_cells(0),
        m_sorted_size(0),
        m_cur_x(0),
        m_cur_y(0),
        m_close_x(0),
        m_close_y(0),
        m_min_x(0x7FFFFFFF),
        m_min_y(0x7FFFFFFF),
        m_max_x(-0x7FFFFFFF),
        m_max_y(-0x7FFFFFFF),
        m_flags(sort_required)
    {
        m_cur_cell.set(0x7FFF, 0x7FFF, 0, 0);
    }


    //------------------------------------------------------------------------
    void outline::reset()
    { 
        m_num_cells = 0; 
        m_cur_block = 0;
        m_cur_cell.set(0x7FFF, 0x7FFF, 0, 0);
        m_flags |= sort_required;
        m_flags &= ~not_closed;
        m_min_x =  0x7FFFFFFF;
        m_min_y =  0x7FFFFFFF;
        m_max_x = -0x7FFFFFFF;
        m_max_y = -0x7FFFFFFF;
    }



    //------------------------------------------------------------------------
    void outline::allocate_block()
    {
        if(m_cur_block >= m_num_blocks)
        {
            if(m_num_blocks >= m_max_blocks)
            {
                cell** new_cells = new cell* [m_max_blocks + cell_block_pool];
                if(m_cells)
                {
                    memcpy(new_cells, m_cells, m_max_blocks * sizeof(cell*));
                    delete [] m_cells;
                }
                m_cells = new_cells;
                m_max_blocks += cell_block_pool;
            }
            m_cells[m_num_blocks++] = new cell [unsigned(cell_block_size)];
        }
        m_cur_cell_ptr = m_cells[m_cur_block++];
    }


    //------------------------------------------------------------------------
    inline void outline::add_cur_cell()
    {
        if(m_cur_cell.area | m_cur_cell.cover)
        {
            if((m_num_cells & cell_block_mask) == 0)
            {
                if(m_num_blocks >= cell_block_limit) return;
                allocate_block();
            }
            *m_cur_cell_ptr++ = m_cur_cell;
            m_num_cells++;
        }
    }



    //------------------------------------------------------------------------
    inline void outline::set_cur_cell(int x, int y)
    {
        if(m_cur_cell.packed_coord != (y << 16) + x)
        {
            add_cur_cell();
            m_cur_cell.set(x, y, 0, 0);
        }
    }



    //------------------------------------------------------------------------
    inline void outline::render_scanline(int ey, int x1, int y1, int x2, int y2)
    {
        int ex1 = x1 >> poly_base_shift;
        int ex2 = x2 >> poly_base_shift;
        int fx1 = x1 & poly_base_mask;
        int fx2 = x2 & poly_base_mask;

        int delta, p, first, dx;
        int incr, lift, mod, rem;

        //trivial case. Happens often
        if(y1 == y2)
        {
            set_cur_cell(ex2, ey);
            return;
        }

        //everything is located in a single cell.  That is easy!
        if(ex1 == ex2)
        {
            delta = y2 - y1;
            m_cur_cell.add_cover(delta, (fx1 + fx2) * delta);
            return;
        }

        //ok, we'll have to render a run of adjacent cells on the same
        //scanline...
        p     = (poly_base_size - fx1) * (y2 - y1);
        first = poly_base_size;
        incr  = 1;

        dx = x2 - x1;

        if(dx < 0)
        {
            p     = fx1 * (y2 - y1);
            first = 0;
            incr  = -1;
            dx    = -dx;
        }

        delta = p / dx;
        mod   = p % dx;

        if(mod < 0)
        {
            delta--;
            mod += dx;
        }

        m_cur_cell.add_cover(delta, (fx1 + first) * delta);

        ex1 += incr;
        set_cur_cell(ex1, ey);
        y1  += delta;

        if(ex1 != ex2)
        {
            p     = poly_base_size * (y2 - y1 + delta);
            lift  = p / dx;
            rem   = p % dx;

            if (rem < 0)
            {
                lift--;
                rem += dx;
            }

            mod -= dx;

            while (ex1 != ex2)
            {
                delta = lift;
                mod  += rem;
                if(mod >= 0)
                {
                    mod -= dx;
                    delta++;
                }

                m_cur_cell.add_cover(delta, (poly_base_size) * delta);
                y1  += delta;
                ex1 += incr;
                set_cur_cell(ex1, ey);
            }
        }
        delta = y2 - y1;
        m_cur_cell.add_cover(delta, (fx2 + poly_base_size - first) * delta);
    }






    //------------------------------------------------------------------------
    void outline::render_line(int x1, int y1, int x2, int y2)
    {
        int ey1 = y1 >> poly_base_shift;
        int ey2 = y2 >> poly_base_shift;
        int fy1 = y1 & poly_base_mask;
        int fy2 = y2 & poly_base_mask;

        int dx, dy, x_from, x_to;
        int p, rem, mod, lift, delta, first, incr;

        if(ey1   < m_min_y) m_min_y = ey1;
        if(ey1+1 > m_max_y) m_max_y = ey1+1;
        if(ey2   < m_min_y) m_min_y = ey2;
        if(ey2+1 > m_max_y) m_max_y = ey2+1;

        dx = x2 - x1;
        dy = y2 - y1;

        //everything is on a single scanline
        if(ey1 == ey2)
        {
            render_scanline(ey1, x1, fy1, x2, fy2);
            return;
        }

        //Vertical line - we have to calculate start and end cells,
        //and then - the common values of the area and coverage for
        //all cells of the line. We know exactly there's only one 
        //cell, so, we don't have to call render_scanline().
        incr  = 1;
        if(dx == 0)
        {
            int ex = x1 >> poly_base_shift;
            int two_fx = (x1 - (ex << poly_base_shift)) << 1;
            int area;

            first = poly_base_size;
            if(dy < 0)
            {
                first = 0;
                incr  = -1;
            }

            x_from = x1;

            //render_scanline(ey1, x_from, fy1, x_from, first);
            delta = first - fy1;
            m_cur_cell.add_cover(delta, two_fx * delta);

            ey1 += incr;
            set_cur_cell(ex, ey1);

            delta = first + first - poly_base_size;
            area = two_fx * delta;
            while(ey1 != ey2)
            {
                //render_scanline(ey1, x_from, poly_base_size - first, x_from, first);
                m_cur_cell.set_cover(delta, area);
                ey1 += incr;
                set_cur_cell(ex, ey1);
            }
            //render_scanline(ey1, x_from, poly_base_size - first, x_from, fy2);
            delta = fy2 - poly_base_size + first;
            m_cur_cell.add_cover(delta, two_fx * delta);
            return;
        }

        //ok, we have to render several scanlines
        p     = (poly_base_size - fy1) * dx;
        first = poly_base_size;

        if(dy < 0)
        {
            p     = fy1 * dx;
            first = 0;
            incr  = -1;
            dy    = -dy;
        }

        delta = p / dy;
        mod   = p % dy;

        if(mod < 0)
        {
            delta--;
            mod += dy;
        }

        x_from = x1 + delta;
        render_scanline(ey1, x1, fy1, x_from, first);

        ey1 += incr;
        set_cur_cell(x_from >> poly_base_shift, ey1);

        if(ey1 != ey2)
        {
            p     = poly_base_size * dx;
            lift  = p / dy;
            rem   = p % dy;

            if(rem < 0)
            {
                lift--;
                rem += dy;
            }
            mod -= dy;

            while(ey1 != ey2)
            {
                delta = lift;
                mod  += rem;
                if (mod >= 0)
                {
                    mod -= dy;
                    delta++;
                }

                x_to = x_from + delta;
                render_scanline(ey1, x_from, poly_base_size - first, x_to, first);
                x_from = x_to;

                ey1 += incr;
                set_cur_cell(x_from >> poly_base_shift, ey1);
            }
        }
        render_scanline(ey1, x_from, poly_base_size - first, x2, fy2);
    }


    //------------------------------------------------------------------------
    void outline::move_to(int x, int y)
    {
        if((m_flags & sort_required) == 0) reset();
        if(m_flags & not_closed) line_to(m_close_x, m_close_y);
        set_cur_cell(x >> poly_base_shift, y >> poly_base_shift);
        m_close_x = m_cur_x = x;
        m_close_y = m_cur_y = y;
    }



    //------------------------------------------------------------------------
    void outline::line_to(int x, int y)
    {
        if((m_flags & sort_required) && ((m_cur_x ^ x) | (m_cur_y ^ y)))
        {
            int c;

            c = m_cur_x >> poly_base_shift;
            if(c < m_min_x) m_min_x = c;
            ++c;
            if(c > m_max_x) m_max_x = c;

            c = x >> poly_base_shift;
            if(c < m_min_x) m_min_x = c;
            ++c;
            if(c > m_max_x) m_max_x = c;

            render_line(m_cur_x, m_cur_y, x, y);
            m_cur_x = x;
            m_cur_y = y;
            m_flags |= not_closed;
        }
    }


    enum
    {
        qsort_threshold = 9
    };


    //------------------------------------------------------------------------
    template <class T> static inline void swap_cells(T* a, T* b)
    {
        T temp = *a;
        *a = *b;
        *b = temp;
    }

    //------------------------------------------------------------------------
    template <class T> static inline bool less_than(T* a, T* b)
    {
        return (*a)->packed_coord < (*b)->packed_coord;
    }



    //------------------------------------------------------------------------
    void outline::qsort_cells(cell** start, unsigned num)
    {
        cell**  stack[80];
        cell*** top; 
        cell**  limit;
        cell**  base;

        limit = start + num;
        base  = start;
        top   = stack;

        for (;;)
        {
            int len = int(limit - base);

            cell** i;
            cell** j;
            cell** pivot;

            if(len > qsort_threshold)
            {
                // we use base + len/2 as the pivot
                pivot = base + len / 2;
                swap_cells(base, pivot);

                i = base + 1;
                j = limit - 1;

                // now ensure that *i <= *base <= *j 
                if(less_than(j, i))
                {
                    swap_cells(i, j);
                }

                if(less_than(base, i))
                {
                    swap_cells(base, i);
                }

                if(less_than(j, base))
                {
                    swap_cells(base, j);
                }

                for(;;)
                {
                    do i++; while( less_than(i, base) );
                    do j--; while( less_than(base, j) );

                    if ( i > j )
                    {
                        break;
                    }

                    swap_cells(i, j);
                }

                swap_cells(base, j);

                // now, push the largest sub-array
                if(j - base > limit - i)
                {
                    top[0] = base;
                    top[1] = j;
                    base   = i;
                }
                else
                {
                    top[0] = i;
                    top[1] = limit;
                    limit  = j;
                }
                top += 2;
            }
            else
            {
                // the sub-array is small, perform insertion sort
                j = base;
                i = j + 1;

                for(; i < limit; j = i, i++)
                {
                    for(; less_than(j + 1, j); j--)
                    {
                        swap_cells(j + 1, j);
                        if (j == base)
                        {
                            break;
                        }
                    }
                }
                if(top > stack)
                {
                    top  -= 2;
                    base  = top[0];
                    limit = top[1];
                }
                else
                {
                    break;
                }
            }
        }
    }





    //------------------------------------------------------------------------
    void outline::sort_cells()
    {
        if(m_num_cells == 0) return;
        if(m_num_cells > m_sorted_size)
        {
            delete [] m_sorted_cells;
            m_sorted_size = m_num_cells;
            m_sorted_cells = new cell* [m_num_cells + 1];
        }

        cell** sorted_ptr = m_sorted_cells;
        cell** block_ptr = m_cells;
        cell*  cell_ptr;

        unsigned nb = m_num_cells >> cell_block_shift;
        unsigned i;

        while(nb--)
        {
            cell_ptr = *block_ptr++;
            i = cell_block_size;
            while(i--) 
            {
                *sorted_ptr++ = cell_ptr++;
            }
        }

        cell_ptr = *block_ptr++;
        i = m_num_cells & cell_block_mask;
        while(i--) 
        {
            *sorted_ptr++ = cell_ptr++;
        }
        m_sorted_cells[m_num_cells] = 0;
        qsort_cells(m_sorted_cells, m_num_cells);
    }




    //------------------------------------------------------------------------
    const cell* const* outline::cells()
    {
        if(m_flags & not_closed)
        {
            line_to(m_close_x, m_close_y);
            m_flags &= ~not_closed;
        }

        //Perform sort only the first time.
        if(m_flags & sort_required)
        {
            add_cur_cell();
            if(m_num_cells == 0) return 0;
            sort_cells();
            m_flags &= ~sort_required;
        }
        return m_sorted_cells;
    }



    //------------------------------------------------------------------------
    void rasterizer::gamma(double g)
    {
        unsigned i;
        for(i = 0; i < 256; i++)
        {
            m_gamma[i] = (unsigned char)(pow(double(i) / 255.0, g) * 255.0);
        }
    }


    //------------------------------------------------------------------------
    void rasterizer::gamma(const int8u* g)
    {
        memcpy(m_gamma, g, sizeof(m_gamma));
    }


    //------------------------------------------------------------------------
    bool rasterizer::hit_test(int tx, int ty)
    {
        const cell* const* cells = m_outline.cells();
        if(m_outline.num_cells() == 0) return false;

        int x, y;
        int cover;
        int alpha;
        int area;

        cover = 0;
        const cell* cur_cell = *cells++;
        for(;;)
        {
            const cell* start_cell = cur_cell;

            int coord  = cur_cell->packed_coord;
            x = cur_cell->x;
            y = cur_cell->y;

            if(y > ty) return false;

            area   = start_cell->area;
            cover += start_cell->cover;

            while((cur_cell = *cells++) != 0)
            {
                if(cur_cell->packed_coord != coord) break;
                area  += cur_cell->area;
                cover += cur_cell->cover;
            }

            if(area)
            {
                alpha = calculate_alpha((cover << (poly_base_shift + 1)) - area);
                if(alpha)
                {
                    if(tx == x && ty == y) return true;
                }
                x++;
            }

            if(!cur_cell) break;

            if(cur_cell->x > x)
            {
                alpha = calculate_alpha(cover << (poly_base_shift + 1));
                if(alpha)
                {
                    if(ty == y && tx >= x && tx <= cur_cell->x) return true;
                }
            }
        }
        return false;
    }
}

#endif // AGGLITE_H
