#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "agg.h"


enum
{
    width  = 400,
    height = 300
};


double random(double min, double max)
{
    int r = (rand() << 15) | rand();
    return ((r & 0xFFFFFFF) / double(0xFFFFFFF + 1)) * (max - min) + min;
}


void draw_ellipse(agg::rasterizer& ras,
                  double x,  double y,
                  double rx, double ry)
{
    int i;
    ras.move_to_d(x + rx, y);

    // Here we have a fixed number of approximation steps, namely 360
    // while in reality it's supposed to be smarter.
    for(i = 1; i < 360; i++)
    {
        double a = double(i) * 3.1415926 / 180.0;
        ras.line_to_d(x + cos(a) * rx, y + sin(a) * ry);
    }
}


void draw_line(agg::rasterizer& ras,
               double x1, double y1, 
               double x2, double y2,
               double width)
{

    double dx = x2 - x1;
    double dy = y2 - y1;
    double d = sqrt(dx*dx + dy*dy);
    
    dx = width * (y2 - y1) / d;
    dy = width * (x2 - x1) / d;

    ras.move_to_d(x1 - dx,  y1 + dy);
    ras.line_to_d(x2 - dx,  y2 + dy);
    ras.line_to_d(x2 + dx,  y2 - dy);
    ras.line_to_d(x1 + dx,  y1 - dy);
}


int main()
{
    // Allocate the framebuffer
    unsigned char* buf = new unsigned char[width * height * 3];

    // Create the rendering buffer 
    agg::rendering_buffer rbuf(buf, width, height, width * 3);

    // Create the renderer and the rasterizer
    agg::renderer<agg::span_rgb24> ren(rbuf);
    agg::rasterizer ras;

    // Setup the rasterizer
    ras.gamma(1.3);
    ras.filling_rule(agg::fill_even_odd);

    ren.clear(agg::rgba8(255, 255, 255));

    int i;

    // Draw random polygons
    for(i = 0; i < 10; i++)
    {
        int n = rand() % 6 + 3;

        // Make the polygon. One can call move_to() more than once. 
        // In this case the rasterizer behaves like Win32 API PolyPolygon().
        ras.move_to_d(random(-30, rbuf.width() + 30), 
                      random(-30, rbuf.height() + 30));

        int j;
        for(j = 1; j < n; j++)
        {
            ras.line_to_d(random(-30, rbuf.width() + 30), 
                          random(-30, rbuf.height() + 30));
        }

        // Render
        ras.render(ren, agg::rgba8(rand() & 0xFF, 
                                   rand() & 0xFF, 
                                   rand() & 0xFF, 
                                   rand() & 0xFF));
    }

    // Draw random ellipses
    for(i = 0; i < 50; i++)
    {
        draw_ellipse(ras, 
                     random(-30, rbuf.width()  + 30), 
                     random(-30, rbuf.height() + 30),
                     random(3, 50), 
                     random(3, 50));
        ras.render(ren, agg::rgba8(rand() & 0x7F, 
                                   rand() & 0x7F, 
                                   rand() & 0x7F,
                                  (rand() & 0x7F) + 100));
    }

    // Draw random straight lines
    for(i = 0; i < 20; i++)
    {
        draw_line(ras, 
                  random(-30, rbuf.width()  + 30), 
                  random(-30, rbuf.height() + 30),
                  random(-30, rbuf.width()  + 30), 
                  random(-30, rbuf.height() + 30),
                  random(0.1, 10));

        ras.render(ren, agg::rgba8(rand() & 0x7F, 
                                   rand() & 0x7F, 
                                   rand() & 0x7F));
    }

    // Write a .ppm file
    FILE* fd = fopen("agg_test.ppm", "wb");
    fprintf(fd, "P6\n%d %d\n255\n", rbuf.width(), rbuf.height());
    fwrite(buf, 1, rbuf.width() * rbuf.height() * 3, fd);
    fclose(fd);

    delete [] buf;
    return 0;
}

