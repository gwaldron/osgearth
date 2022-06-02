/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2020 Pelican Mapping
 * http://osgearth.org
 *
 * osgEarth is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */
#include "SDF"
#include "Math"
#include "Metrics"
#include "FeatureSource"
#include "FeatureRasterizer"
#include "Session"

using namespace osgEarth;
using namespace osgEarth::Util;

namespace
{
    inline bool isPositivePowerOfTwo(unsigned x) {
        return (x & (x - 1)) == 0;
    }

    // https://www.comp.nus.edu.sg/~tants/jfa/i3d06.pdf
    const char* jfa_cs = R"(
    #version 430
    layout(local_size_x=1, local_size_y=1, local_size_z=1) in;

    // output image binding
    layout(binding=0, rg16f) uniform image2D buf;

    uniform int L;

    #define NODATA 32767.0

    float squared_distance_2d(in vec4 a, in vec4 b)
    {
        vec2 c = b.xy-a.xy;
        return dot(c, c);
    }

    void main()
    {
        vec2 pixel_uv = vec2(
            float(gl_WorkGroupID.x) / float(gl_NumWorkGroups.x-1),
            float(gl_WorkGroupID.y) / float(gl_NumWorkGroups.y-1));

        int s = int(gl_WorkGroupID.x);
        int t = int(gl_WorkGroupID.y);

        vec4 pixel_points_to = imageLoad(buf, ivec2(gl_WorkGroupID));
        if (pixel_points_to.x == NODATA)
            return;

        vec4 remote;
        vec4 remote_points_to;

        for(int rs = s - L; rs <= s + L; rs += L)
        {
            if (rs < 0 || rs >= gl_NumWorkGroups.x)
                continue;

            remote.x = float(rs);

            for(int rt = t - L; rt <= t + L; rt += L)
            {
                if (rt < 0 || rt >= gl_NumWorkGroups.y)
                    continue;

                if (rs == s && rt == t)
                    continue;

                remote.y = float(rt);

                remote_points_to = imageLoad(buf, ivec2(rs,rt));
                if (remote_points_to.x == NODATA)
                {
                    imageStore(buf, ivec2(rs,rt), pixel_points_to);
                }
                else
                {
                    // compare the distances and pick the closest.
                    float d_existing = squared_distance_2d(remote, remote_points_to);
                    float d_possible = squared_distance_2d(remote, pixel_points_to);

                    if (d_possible < d_existing)
                    {
                        imageStore(buf, ivec2(rs,rt), pixel_points_to);
                    }
                }
            }
        }
    }
    )";
}

SDFGenerator::SDFGenerator() :
    _useGPU(false)
{
    //nop
}

void
SDFGenerator::setUseGPU(bool value)
{
    _useGPU = value;

    //if (_useGPU && !_program.valid())
    //{
    //    _program = new osg::Program();
    //    _program->addShader(new osg::Shader(osg::Shader::COMPUTE, jfa_cs));
    //}
}

GeoImage
SDFGenerator::allocateSDF(
    unsigned size,
    const GeoExtent& extent,
    GLenum pixelFormat) const
{
    osg::ref_ptr<osg::Image> sdf = new osg::Image();
    sdf->allocateImage(size, size, 1, pixelFormat, GL_UNSIGNED_BYTE);

    //todo: replace with a complete ImageUtils function
    sdf->setInternalTextureFormat(
        pixelFormat == GL_RED ? GL_R8 :
        pixelFormat == GL_RG ? GL_RG8 :
        pixelFormat == GL_RGB ? GL_RGB8 :
        pixelFormat == GL_RGBA ? GL_RGBA8 :
        pixelFormat);

    ImageUtils::PixelWriter write(sdf.get());
    write.assign(Color(1, 1, 1, 1));
    return GeoImage(sdf.release(), extent);
}

bool
SDFGenerator::createNearestNeighborField(
    const FeatureList& features,
    unsigned nnfieldSize,
    const GeoExtent& extent,
    bool inverted,
    GeoImage& nnfield,
    Cancelable* progress) const
{
    if (features.empty())
        return false;

    OE_SOFT_ASSERT_AND_RETURN(extent.isValid(), false);
    OE_SOFT_ASSERT_AND_RETURN(isPositivePowerOfTwo(nnfieldSize), false);

    // Render features to a temporary image
    Style style;
    if (features.front()->getGeometry()->isLinear())
        style.getOrCreate<LineSymbol>()->stroke()->color() = Color::Black;
    else
        style.getOrCreate<PolygonSymbol>()->fill()->color() = Color::Black;

    FeatureRasterizer rasterizer(nnfieldSize, nnfieldSize, extent, Color(1, 1, 1, 0));
    rasterizer.render(features, style);
    GeoImage source = rasterizer.finalize();

    return createNearestNeighborField(
        source,
        inverted,
        nnfield,
        progress);
}

bool
SDFGenerator::createNearestNeighborField(
    const GeoImage& inputRaster,
    bool inverted,
    GeoImage& nnfield,
    Cancelable* progress) const
{
    OE_PROFILING_ZONE;

    // Convert pixels to local coordinates relative to the lower-left corner of extent
    if (!nnfield.valid())
    {
        osg::Image* image = new osg::Image();
        image->allocateImage(inputRaster.getImage()->s(), inputRaster.getImage()->t(), 1, GL_RG, GL_FLOAT);
        image->setInternalTextureFormat(GL_RG16F);
        nnfield = std::move(GeoImage(image, inputRaster.getExtent()));
    }

    // actually need to write to the GeoImage, and that's OK.
    osg::Image* nnimage = const_cast<osg::Image*>(nnfield.getImage());

    ImageUtils::PixelReader read_raster(inputRaster.getImage());
    ImageUtils::PixelWriter write_nnf(nnimage);

    constexpr float NODATA = 32767.0f;
    osg::Vec4f nodata(NODATA, NODATA, NODATA, NODATA);
    osg::Vec4f pixel, coord;
    GeoImageIterator iter(inputRaster);
    iter.forEachPixel([&]()
        {
            read_raster(pixel, iter.s(), iter.t());
            if ((!inverted && pixel.a() >= 0.5f) || (inverted && pixel.a() <= 0.5f))
                coord.set((float)iter.s(), (float)iter.t(), 0.0f, 0.0f);
            else
                coord = nodata;
            
            write_nnf(coord, iter.s(), iter.t());
        }
    );

    //if (_useGPU)
    //{
    //    // not good, too many barriers
    //    compute_nnf_on_gpu(nnimage);
    //}
    //else
    {
        compute_nnf_on_cpu(nnimage);
    }

    return true;
}

void
SDFGenerator::createDistanceField(
    const GeoImage& nnfield,
    GeoImage& sdf,
    float span,
    float lo,
    float hi,
    Cancelable* progress) const
{
    OE_SOFT_ASSERT_AND_RETURN(nnfield.valid(), void());
    OE_SOFT_ASSERT_AND_RETURN(sdf.valid(), void());

    // That's OK.
    osg::Image* sdfimage = const_cast<osg::Image*>(sdf.getImage());

    ImageUtils::PixelReader read_sdf(sdfimage);
    ImageUtils::PixelWriter write_sdf(sdfimage);

    ImageUtils::PixelReader read_nnf(nnfield.getImage());
    read_nnf.setBilinear(false);
    
    GeoImageIterator i(sdf);

    osg::Vec4f me, closest, pixel;
    int c = 0; // clamp((int)(channel - GL_RED), 0, 3);

    osg::Vec2f bias(
        (sdf.getExtent().xMin() - nnfield.getExtent().xMin()) / nnfield.getExtent().width(),
        (sdf.getExtent().yMin() - nnfield.getExtent().yMin()) / nnfield.getExtent().height());

    osg::Vec2f scale(
        sdf.getExtent().width() / nnfield.getExtent().width(),
        sdf.getExtent().height() / nnfield.getExtent().height());

    //int bias_s = (nnfield.getImage()->s() - sdf.getImage()->s()) / 2;
    //int bias_t = (nnfield.getImage()->t() - sdf.getImage()->t()) / 2;

    float nnf_u, nnf_v;

    float cellSize = 1.0f / (float)(nnfield.getImage()->s() - 1);

    i.forEachPixel([&]()
        {
            read_sdf(pixel, i.s(), i.t());

#if 0
            // this code path assumes perfect centering

            int nnfs = i.s() + bias_s;
            int nnft = i.t() + bias_t;

            me.set(nnfs, nnft, 0, 0);
            read_nnf(closest, nnfs, nnft);
#else
            // convert ndc coords to the NNF domain
            nnf_u = clamp(i.u() * scale.x() + bias.x(), 0.0, 1.0);
            nnf_v = clamp(i.v() * scale.y() + bias.y(), 0.0, 1.0);

            me.set(
                floor(nnf_u * nnfield.getImage()->s()),
                floor(nnf_v * nnfield.getImage()->t()),
                0, 0);

            read_nnf(closest, nnf_u, nnf_v);
#endif

            float d = distance2D(me, closest);
            d = unitremap(d * cellSize * span, lo, hi);
            if (d < pixel[c])
            {
                pixel[c] = d;
                write_sdf(pixel, i.s(), i.t());
            }
        });
}


#if 0
void
SDFGenerator::compute_nnf_on_gpu(osg::Image* image) const
{
    NNFSession& session = _compute.get(_program.get());
    session.setImage(image);
    session.execute();
}

void
SDFGenerator::NNFSession::renderImplementation(osg::State* state)
{
    if (_L_uniform < 0)
    {
        const osg::Program::PerContextProgram* pcp = state->getLastAppliedProgramObject();
        _L_uniform = pcp->getUniformLocation(osg::Uniform::getNameID("L"));
    }

    osg::GLExtensions* ext = state->get<osg::GLExtensions>();

    // https://www.comp.nus.edu.sg/~tants/jfa/i3d06.pdf
    for (int L = _image->s() / 2; L >= 1; L /= 2)
    {
        ext->glUniform1i(_L_uniform, L);
        ext->glDispatchCompute(_image->s(), _image->t(), 1);
        ext->glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
    }
}
#endif

/*
inline void readRGFloatPixel(ImageUtils::PixelReader& reader, osg::Vec4& pixel, unsigned int s, unsigned int t, unsigned int r=0)
{
    const float* data = (const float*)(reader.data(s, t, r));
    pixel.x() = float(*data++);
    pixel.y() = float(*data++);
}

inline void writeRGFloatPixel(ImageUtils::PixelWriter& writer, osg::Vec4& pixel, unsigned int s, unsigned int t, unsigned int r = 0)
{
    float* data = (float*)(writer.data(s, t, r));
    *data++ = pixel.x();
    *data++ = pixel.y();
}
*/

inline void readRGFloatPixel(float* grid, unsigned int w, unsigned int h, osg::Vec4& pixel, unsigned int s, unsigned int t)
{
    float *data = &grid[(t * w + s) * 2];
    pixel.x() = *data++;
    pixel.y() = *data++;
}

inline void writeRGFloatPixel(float* grid, unsigned int w, unsigned int h, osg::Vec4& pixel, unsigned int s, unsigned int t)
{
    float* data = &grid[(t * w + s) * 2];
    *data++ = pixel.x();
    *data++ = pixel.y();
}

void
SDFGenerator::compute_nnf_on_cpu(osg::Image* buf) const
{
    OE_PROFILING_ZONE;

    // Jump-Flood algorithm for computing discrete voronoi
    // https://www.comp.nus.edu.sg/~tants/jfa/i3d06.pdf
    osg::Vec4f pixel_points_to;
    osg::Vec4f remote;
    osg::Vec4f remote_points_to;

    // There are many read/write accesses in a tight loop in this algorithm so it is much faster to access
    // the raw image data directly by pointer using a known format (GL_RG float) rather than use the PixelReader functions.
    
    //ImageUtils::PixelReader readBuf(buf);
    //ImageUtils::PixelWriter writeBuf(buf);
    int n = buf->s();
    constexpr float NODATA = 32767;

    unsigned int imageWidth = buf->s();
    unsigned int imageHeight = buf->t();
    float* imageData = (float*)(buf->data());

    for (int L = n / 2; L >= 1; L /= 2)
    {
        for (unsigned int iterT = 0; iterT < buf->t(); ++iterT)
        {
            for (unsigned int iterS = 0; iterS < buf->s(); ++iterS)
            {
                readRGFloatPixel(imageData, imageWidth, imageHeight, pixel_points_to, iterS, iterT);

                // no data at this pixel yet? skip it; there is nothing to propagate.
                if (pixel_points_to.x() != NODATA)
                {
                    for (int s = iterS - L; s <= iterS + L; s += L)
                    {
                        if (s < 0 || s >= buf->s())
                            continue;

                        remote[0] = (float)s;

                        for (int t = iterT - L; t <= iterT + L; t += L)
                        {
                            if (t < 0 || t >= buf->t())
                                continue;
                            if (s == iterS && t == iterT)
                                continue;

                            remote[1] = (float)t;

                            readRGFloatPixel(imageData, imageWidth, imageHeight, remote_points_to, s, t);

                            if (remote_points_to.x() == NODATA) // remote is unset? Just copy
                            {
                                writeRGFloatPixel(imageData, imageWidth, imageHeight, pixel_points_to, s, t);
                            }
                            else
                            {
                                // compare the distances and pick the closest.
                                float d_existing = distanceSquared2D(remote, remote_points_to);
                                float d_possible = distanceSquared2D(remote, pixel_points_to);

                                if (d_possible < d_existing)
                                {
                                    writeRGFloatPixel(imageData, imageWidth, imageHeight, pixel_points_to, s, t);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

#define INF 1E20

//! Compute the 1d distance transform
//! @param f Array of values to compute the distance transform function
//! @param d Temporary work array of size at least n
//! @param v Temporary work array of size at least n
//! @param z Temporary work array of size at least n + 1
//! @param n The size of f
static void edt1d(const float* f, float* d, int* v, float* z, unsigned int n) {
    int k = 0;
    v[0] = 0;
    z[0] = -INF;
    z[1] = INF;
    for (int q = 1; q <= n - 1; ++q) {
        float s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k]);
        while (s <= z[k]) {
            k--;
            s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k]);
        }
        k++;
        v[k] = q;
        z[k] = s;
        z[k + 1] = INF;
    }

    k = 0;
    for (int q = 0; q <= n - 1; ++q) {
        while (z[k + 1] < q)
            k++;
        int r = v[k];
        d[q] = (q - r) * (q - r) + f[r];
    }
}

//! Compute the 2d distance transform of a grid of floats
//! @param grid A 2d grid of floats
//! @param width The width of the grid
//! @param height The height of the grid
void edt2d(float* grid, unsigned int width, unsigned int height)
{
    unsigned int maxLength = std::max(width, height);
    float* f = new float[maxLength];
    float* d = new float[maxLength];
    int* v = new int[maxLength];
    float* z = new float[maxLength + 1u];

    // process columns
    for (int x = 0; x < width; ++x) {
        for (int y = 0; y < height; ++y) {
            f[y] = grid[width * y + x];
        }
        // Do the distance transform.
        edt1d(f, d, v, z, height);
        // Copy d back into the grid
        for (int y = 0; y < height; ++y) {
            grid[width * y + x] = d[y];
        }        
    }

    // process rows
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            f[x] = grid[width * y + x];
        }

        // Do the distance transform
        edt1d(f, d, v, z, width);

        // Copy d back into the grid
        for (int x = 0; x < width; ++x) {
            grid[width * y + x] = d[x];
        }
    }

    delete[] f;
    delete[] d;
    delete[] v;
    delete[] z;
}

osg::Image* SDFGenerator::createDistanceField(const osg::Image* image, float minPixels, float maxPixels) const
{
    OE_PROFILING_ZONE;

    ImageUtils::PixelReader read(image);

    unsigned int width = image->s();
    unsigned int height = image->t();

    // Initialize the grid to INF
    std::vector<float> grid(width * height, INF);

    // Mark pixels with alpha > 0 as having a distance of 0
    for (unsigned int y = 0; y < height; ++y) {
        for (unsigned int x = 0; x < width; ++x) {
            osg::Vec4 pixel;
            read(pixel, x, y);
            float a = pixel.a();
            if (a > 0.0f) {
                grid[y * width + x] = 0;
            }
        }
    }

    // Compute the distance transform
    edt2d(grid.data(), width, height);

    // Copy the distance transform back into the image
    osg::ref_ptr<osg::Image> sdf = new osg::Image();
    sdf->allocateImage(width, height, 1, GL_RED, GL_UNSIGNED_BYTE);
    sdf->setInternalTextureFormat(GL_R8);

    osg::Vec4 p;

    ImageUtils::PixelWriter write(sdf.get());
    write.assign(Color(1, 1, 1, 1));
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            // The distance computed is the square distance, so take the square root here to get the actual distance
            float d = sqrt(grid[width * y + x]);
            // Remap the value between 0 and 1
            float value = unitremap(d, minPixels, maxPixels);
            p.set(value, 1.0, 1.0, 1.0);
            write(p, x, y);
        }
    }
    return sdf.release();
}