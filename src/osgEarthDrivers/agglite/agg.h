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
#ifndef AGG_INCLUDED
#define AGG_INCLUDED

#include <string.h>

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



#endif

