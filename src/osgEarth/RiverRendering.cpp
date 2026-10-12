/* osgEarth
 * Copyright 2026 Pelican Mapping
 * MIT License
 */
#include "RiverRendering.h"
#include <osgEarth/GLUtils>
#include <osgEarth/Notify>
#include <osg/GLExtensions>
#include <osg/FrameBufferObject>
#include <osg/Texture>
#include <osg/State>
#include <complex>
#include <random>
#include <algorithm>
#include <cmath>
#include <atomic>

#define LC "[RiverLayer] "

using namespace osgEarth;
using namespace osgEarth::RiverDetail;

#ifndef GL_FRAMEBUFFER_ATTACHMENT_OBJECT_TYPE
#define GL_FRAMEBUFFER_ATTACHMENT_OBJECT_TYPE 0x8CD0
#endif
#ifndef GL_FRAMEBUFFER_ATTACHMENT_COLOR_ENCODING
#define GL_FRAMEBUFFER_ATTACHMENT_COLOR_ENCODING 0x8210
#endif
#ifndef GL_FRAMEBUFFER_ATTACHMENT_COMPONENT_TYPE
#define GL_FRAMEBUFFER_ATTACHMENT_COMPONENT_TYPE 0x8211
#endif
#ifndef GL_FRAMEBUFFER_ATTACHMENT_RED_SIZE
#define GL_FRAMEBUFFER_ATTACHMENT_RED_SIZE 0x8212
#define GL_FRAMEBUFFER_ATTACHMENT_GREEN_SIZE 0x8213
#define GL_FRAMEBUFFER_ATTACHMENT_BLUE_SIZE 0x8214
#define GL_FRAMEBUFFER_ATTACHMENT_ALPHA_SIZE 0x8215
#define GL_FRAMEBUFFER_ATTACHMENT_DEPTH_SIZE 0x8216
#define GL_FRAMEBUFFER_ATTACHMENT_STENCIL_SIZE 0x8217
#endif
#ifndef GL_UNSIGNED_NORMALIZED
#define GL_UNSIGNED_NORMALIZED 0x8C17
#endif
#ifndef GL_SRGB
#define GL_SRGB 0x8C40
#endif
#ifndef GL_SRGB8
#define GL_SRGB8 0x8C41
#endif
#ifndef GL_SRGB8_ALPHA8
#define GL_SRGB8_ALPHA8 0x8C43
#endif
#ifndef GL_R11F_G11F_B10F
#define GL_R11F_G11F_B10F 0x8C3A
#endif
#ifndef GL_RGB10_A2
#define GL_RGB10_A2 0x8059
#endif
#ifndef GL_DEPTH_STENCIL_ATTACHMENT
#define GL_DEPTH_STENCIL_ATTACHMENT 0x821A
#endif
#ifndef GL_DEPTH
#define GL_DEPTH 0x1801
#endif
#ifndef GL_STENCIL
#define GL_STENCIL 0x1802
#endif
#ifndef GL_BACK_LEFT
#define GL_BACK_LEFT 0x0402
#endif
#ifndef GL_FRONT_LEFT
#define GL_FRONT_LEFT 0x0400
#endif
#ifndef GL_READ_BUFFER
#define GL_READ_BUFFER 0x0C02
#endif
#ifndef GL_DRAW_FRAMEBUFFER_BINDING
#define GL_DRAW_FRAMEBUFFER_BINDING 0x8CA6
#endif
#ifndef GL_READ_FRAMEBUFFER_BINDING
#define GL_READ_FRAMEBUFFER_BINDING 0x8CAA
#endif
#ifndef GL_DEPTH32F_STENCIL8
#define GL_DEPTH32F_STENCIL8 0x8CAD
#endif
#ifndef GL_FLOAT_32_UNSIGNED_INT_24_8_REV
#define GL_FLOAT_32_UNSIGNED_INT_24_8_REV 0x8DAD
#endif
#ifndef GL_TEXTURE_MAX_LEVEL
#define GL_TEXTURE_MAX_LEVEL 0x813D
#endif

namespace
{
    using cpx = std::complex<double>;

    //! In-place radix-2 FFT of a power-of-two length sequence.
    void fft1(std::vector<cpx>& a, bool inverse)
    {
        const std::size_t n = a.size();
        for (std::size_t i = 1, j = 0; i < n; ++i)
        {
            std::size_t bit = n >> 1;
            for (; j & bit; bit >>= 1) j ^= bit;
            j ^= bit;
            if (i < j) std::swap(a[i], a[j]);
        }
        for (std::size_t len = 2; len <= n; len <<= 1)
        {
            double ang = 2.0 * osg::PI / (double)len * (inverse ? 1.0 : -1.0);
            cpx wl(std::cos(ang), std::sin(ang));
            for (std::size_t i = 0; i < n; i += len)
            {
                cpx w(1.0, 0.0);
                for (std::size_t j = 0; j < len / 2; ++j)
                {
                    cpx u = a[i + j], v = a[i + j + len / 2] * w;
                    a[i + j] = u + v;
                    a[i + j + len / 2] = u - v;
                    w *= wl;
                }
            }
        }
        if (inverse)
            for (auto& x : a) x /= (double)n;
    }

    //! In-place 2D FFT of an n x n row-major grid.
    void fft2(std::vector<cpx>& data, unsigned n, bool inverse)
    {
        std::vector<cpx> line(n);
        for (unsigned y = 0; y < n; ++y)
        {
            std::copy(data.begin() + y * n, data.begin() + (y + 1) * n, line.begin());
            fft1(line, inverse);
            std::copy(line.begin(), line.end(), data.begin() + y * n);
        }
        for (unsigned x = 0; x < n; ++x)
        {
            for (unsigned y = 0; y < n; ++y) line[y] = data[y * n + x];
            fft1(line, inverse);
            for (unsigned y = 0; y < n; ++y) data[y * n + x] = line[y];
        }
    }

    //! Tiling random field (n x n, values normalized to [-1, 1]) whose spectrum falls off as
    //! |k|^-exponent between kmin and kmax cycles per tile, with soft band edges.
    std::vector<float> spectralField(unsigned n, double kmin, double kmax, double exponent, unsigned seed)
    {
        std::mt19937 rng(seed);
        std::normal_distribution<double> gauss(0.0, 1.0);
        std::vector<cpx> spectrum((std::size_t)n * n);
        for (unsigned y = 0; y < n; ++y)
        {
            double ky = y <= n / 2 ? (double)y : (double)y - n;
            for (unsigned x = 0; x < n; ++x)
            {
                double kx = x <= n / 2 ? (double)x : (double)x - n;
                double k = std::sqrt(kx * kx + ky * ky);
                double a = gauss(rng), b = gauss(rng);
                if (k < 0.5) continue;
                double band = std::min(1.0, std::max(0.0, (k - kmin * 0.7) / (kmin * 0.3 + 1e-9))) *
                    std::min(1.0, std::max(0.0, (kmax * 1.3 - k) / (kmax * 0.3 + 1e-9)));
                spectrum[(std::size_t)y * n + x] = cpx(a, b) * (band * std::pow(k, -exponent));
            }
        }
        fft2(spectrum, n, true);
        std::vector<float> field((std::size_t)n * n);
        double mean = 0.0;
        for (std::size_t i = 0; i < field.size(); ++i) mean += spectrum[i].real();
        mean /= (double)field.size();
        double peak = 1e-12;
        for (std::size_t i = 0; i < field.size(); ++i)
            peak = std::max(peak, std::abs(spectrum[i].real() - mean));
        for (std::size_t i = 0; i < field.size(); ++i)
            field[i] = (float)((spectrum[i].real() - mean) / peak);
        return field;
    }

    //! Tiling Worley noise over an n x n image with "cells" jittered feature points per side.
    //! Returns the distances to the nearest (f1) and second nearest (f2) points in cell units,
    //! and optionally the index of the nearest cell.
    void worley(unsigned n, unsigned cells, unsigned seed, std::vector<float>& f1, std::vector<float>& f2,
        std::vector<unsigned>* nearest = nullptr)
    {
        std::mt19937 rng(seed);
        std::uniform_real_distribution<float> uni(0.0f, 1.0f);
        std::vector<osg::Vec2f> pts((std::size_t)cells * cells);
        for (auto& p : pts) p.set(uni(rng), uni(rng));
        f1.assign((std::size_t)n * n, 0.0f);
        f2.assign((std::size_t)n * n, 0.0f);
        if (nearest) nearest->assign((std::size_t)n * n, 0u);
        const float scale = (float)cells / (float)n;
        for (unsigned y = 0; y < n; ++y)
        {
            for (unsigned x = 0; x < n; ++x)
            {
                float px = (x + 0.5f) * scale, py = (y + 0.5f) * scale;
                int cx = (int)px, cy = (int)py;
                float d1 = 1e9f, d2 = 1e9f;
                unsigned id = 0u;
                for (int oy = -1; oy <= 1; ++oy)
                {
                    for (int ox = -1; ox <= 1; ++ox)
                    {
                        int gx = cx + ox, gy = cy + oy;
                        int wx = (gx % (int)cells + (int)cells) % (int)cells;
                        int wy = (gy % (int)cells + (int)cells) % (int)cells;
                        const osg::Vec2f& j = pts[(std::size_t)wy * cells + wx];
                        float dx = gx + j.x() - px, dy = gy + j.y() - py;
                        float d = std::sqrt(dx * dx + dy * dy);
                        if (d < d1) d2 = d1, d1 = d, id = (unsigned)(wy * (int)cells + wx);
                        else if (d < d2) d2 = d;
                    }
                }
                f1[(std::size_t)y * n + x] = d1;
                f2[(std::size_t)y * n + x] = d2;
                if (nearest) (*nearest)[(std::size_t)y * n + x] = id;
            }
        }
    }

    //! GLSL-style smoothstep for the texture generators.
    inline float smoothstepf(float e0, float e1, float x)
    {
        float t = std::min(std::max((x - e0) / (e1 - e0), 0.0f), 1.0f);
        return t * t * (3.0f - 2.0f * t);
    }

    //! Clamps a 0..1 value and quantizes it to 8 bits.
    inline unsigned char toByte(float v)
    {
        return (unsigned char)std::lround(std::min(std::max(v, 0.0f), 1.0f) * 255.0f);
    }
}

osg::ref_ptr<osg::Image>
RiverDetail::createWaveImage(unsigned n)
{
    // Broadband ripples for the surface normals.
    std::vector<float> h = spectralField(n, 3.0, n / 5.0, 1.55, 1u);

    // Normals from wrapped central differences, scaled to a natural RMS slope.
    std::vector<float> gx((std::size_t)n * n), gy((std::size_t)n * n);
    double rms = 0.0;
    for (unsigned y = 0; y < n; ++y)
    {
        for (unsigned x = 0; x < n; ++x)
        {
            float dx = 0.5f * (h[y * n + (x + 1) % n] - h[y * n + (x + n - 1) % n]);
            float dy = 0.5f * (h[((y + 1) % n) * n + x] - h[((y + n - 1) % n) * n + x]);
            gx[y * n + x] = dx, gy[y * n + x] = dy;
            rms += dx * dx + dy * dy;
        }
    }
    rms = std::sqrt(rms / (double)(n * n));
    const float slopeScale = (float)(0.42 / std::max(rms, 1e-9));

    // Caustics: refract parallel light through a smooth swell and splat where it lands.
    std::vector<float> swell = spectralField(n, 2.0, 14.0, 1.9, 7u);
    std::vector<float> light((std::size_t)n * n, 0.0f);
    {
        const float focus = 0.055f * (float)n; // lens strength in texels per unit slope
        for (unsigned y = 0; y < n; ++y)
        {
            for (unsigned x = 0; x < n; ++x)
            {
                float dx = 0.5f * (swell[y * n + (x + 1) % n] - swell[y * n + (x + n - 1) % n]);
                float dy = 0.5f * (swell[((y + 1) % n) * n + x] - swell[((y + n - 1) % n) * n + x]);
                float tx = x - dx * focus * (float)n * 0.25f;
                float ty = y - dy * focus * (float)n * 0.25f;
                int ix = (int)std::floor(tx), iy = (int)std::floor(ty);
                float fx = tx - ix, fy = ty - iy;
                auto put = [&](int px, int py, float w)
                {
                    px = (px % (int)n + (int)n) % (int)n;
                    py = (py % (int)n + (int)n) % (int)n;
                    light[(std::size_t)py * n + px] += w;
                };
                put(ix, iy, (1 - fx) * (1 - fy));
                put(ix + 1, iy, fx * (1 - fy));
                put(ix, iy + 1, (1 - fx) * fy);
                put(ix + 1, iy + 1, fx * fy);
            }
        }
        // soften the splat noise
        std::vector<float> blurred(light.size());
        for (unsigned y = 0; y < n; ++y)
        {
            for (unsigned x = 0; x < n; ++x)
            {
                float sum = 0.0f;
                for (int oy = -1; oy <= 1; ++oy)
                {
                    for (int ox = -1; ox <= 1; ++ox)
                    {
                        float weight = (ox == 0 && oy == 0) ? 4.0f : (ox == 0 || oy == 0) ? 2.0f : 1.0f;
                        sum += light[((y + n + oy) % n) * n + (x + n + ox) % n] * weight;
                    }
                }
                blurred[(std::size_t)y * n + x] = sum / 16.0f;
            }
        }
        light.swap(blurred);
    }

    osg::ref_ptr<osg::Image> image = new osg::Image();
    image->allocateImage(n, n, 1, GL_RGBA, GL_UNSIGNED_BYTE);
    image->setInternalTextureFormat(GL_RGBA8);
    unsigned char* data = image->data();
    for (std::size_t i = 0; i < (std::size_t)n * n; ++i)
    {
        osg::Vec3f nrm(-gx[i] * slopeScale, -gy[i] * slopeScale, 1.0f);
        nrm.normalize();
        data[i * 4 + 0] = toByte(nrm.x() * 0.5f + 0.5f);
        data[i * 4 + 1] = toByte(nrm.y() * 0.5f + 0.5f);
        data[i * 4 + 2] = toByte(h[i] * 0.5f + 0.5f);
        data[i * 4 + 3] = toByte(smoothstepf(0.75f, 3.2f, light[i]));
    }
    return image;
}

osg::ref_ptr<osg::Image>
RiverDetail::createFoamImage(unsigned n)
{
    std::vector<float> a1, a2, b1, b2, c1, c2;
    worley(n, 9u, 11u, a1, a2);   // coarse foam cells
    worley(n, 22u, 12u, b1, b2);  // finer cells
    worley(n, 56u, 13u, c1, c2);  // bubbles
    std::vector<float> low = spectralField(n, 1.0, 4.0, 1.0, 21u);
    std::vector<float> mid = spectralField(n, 4.0, 18.0, 1.0, 22u);

    osg::ref_ptr<osg::Image> image = new osg::Image();
    image->allocateImage(n, n, 1, GL_RGBA, GL_UNSIGNED_BYTE);
    image->setInternalTextureFormat(GL_RGBA8);
    unsigned char* data = image->data();
    for (std::size_t i = 0; i < (std::size_t)n * n; ++i)
    {
        // Lace: bright along cell borders, with bubble holes punched into it.
        float borderA = 1.0f - smoothstepf(0.0f, 0.22f, a2[i] - a1[i]);
        float borderB = 1.0f - smoothstepf(0.0f, 0.16f, b2[i] - b1[i]);
        float holes = smoothstepf(0.18f, 0.45f, c1[i]);
        float coarse = std::min(1.0f, borderA * 0.85f + borderB * 0.55f) * (0.55f + 0.45f * holes);
        float fine = (1.0f - smoothstepf(0.0f, 0.2f, c2[i] - c1[i])) * 0.8f + (1.0f - smoothstepf(0.05f, 0.4f, c1[i])) * 0.35f;
        data[i * 4 + 0] = toByte(coarse);
        data[i * 4 + 1] = toByte(fine);
        data[i * 4 + 2] = toByte(low[i] * 0.5f + 0.5f);
        data[i * 4 + 3] = toByte(mid[i] * 0.5f + 0.5f);
    }
    return image;
}

osg::ref_ptr<osg::Image>
RiverDetail::createBedImage(unsigned n)
{
    // Round stones at three scales scattered over sand and silt: each Worley feature point
    // becomes a stone (or is left out) with its own radius, color and tone.
    struct Layer { unsigned cells; unsigned seed; float keep; float rmin, rmax; };
    static const Layer layers[3] = {
        { 9u, 51u, 0.42f, 0.30f, 0.46f },   // boulders
        { 23u, 52u, 0.62f, 0.28f, 0.45f },  // cobbles
        { 57u, 53u, 0.78f, 0.25f, 0.42f },  // pebbles
    };
    std::vector<float> f1[3], f2[3];
    std::vector<unsigned> id[3];
    for (int l = 0; l < 3; ++l)
        worley(n, layers[l].cells, layers[l].seed, f1[l], f2[l], &id[l]);
    std::vector<float> film = spectralField(n, 1.0, 6.0, 1.0, 54u);
    std::vector<float> grain = spectralField(n, 20.0, 120.0, 0.6, 55u);
    std::vector<float> ripple = spectralField(n, 3.0, 14.0, 1.4, 56u);

    // Stone palette (linear): river-washed granite, schist, sandstone, quartzite, basalt.
    static const float palette[7][3] = {
        { 0.40f, 0.39f, 0.37f }, { 0.47f, 0.41f, 0.33f }, { 0.24f, 0.24f, 0.23f },
        { 0.53f, 0.45f, 0.32f }, { 0.34f, 0.36f, 0.37f }, { 0.60f, 0.57f, 0.52f },
        { 0.36f, 0.30f, 0.25f } };
    auto hash = [](unsigned v) {
        v ^= v >> 16; v *= 0x7feb352dU; v ^= v >> 15; v *= 0x846ca68bU; v ^= v >> 16;
        return v;
    };

    osg::ref_ptr<osg::Image> image = new osg::Image();
    image->allocateImage(n, n, 1, GL_RGBA, GL_UNSIGNED_BYTE);
    image->setInternalTextureFormat(GL_RGBA8);
    unsigned char* data = image->data();
    for (std::size_t i = 0; i < (std::size_t)n * n; ++i)
    {
        // Sand with faint current ripples, darkening to silt in the low spots.
        float sandTone = 0.85f + 0.3f * (grain[i] * 0.5f + 0.5f) + 0.12f * ripple[i];
        float rgb[3] = { 0.30f * sandTone, 0.27f * sandTone, 0.21f * sandTone };
        float height = 0.15f + 0.1f * ripple[i];

        for (int l = 0; l < 3; ++l)
        {
            unsigned h = hash(id[l][i] * 2654435761u + 977u * (unsigned)l + 13u);
            if ((h & 1023u) / 1023.0f > layers[l].keep)
                continue;
            float r = layers[l].rmin + (layers[l].rmax - layers[l].rmin) * ((h >> 10) & 255u) / 255.0f;
            float d = f1[l][i];
            if (d > r) continue;
            float edge = smoothstepf(r, r * 0.82f, d);           // soft rim
            float dome = std::sqrt(std::max(0.0f, 1.0f - (d / r) * (d / r)));
            const float* c = palette[(h >> 18) % 7u];
            float tone = (0.8f + 0.4f * ((h >> 4) & 63u) / 63.0f) * (0.6f + 0.4f * dome);
            for (int k = 0; k < 3; ++k)
                rgb[k] = rgb[k] + (c[k] * tone - rgb[k]) * edge;
            height = height + (0.3f + 0.7f * dome - height) * edge;
        }

        // Slimy algae film in patches.
        float algae = smoothstepf(0.1f, 0.85f, film[i] * 0.5f + 0.5f) * 0.6f;
        static const float algaeTint[3] = { 0.66f, 0.80f, 0.45f };
        for (int k = 0; k < 3; ++k)
            rgb[k] *= 1.0f + (algaeTint[k] - 1.0f) * algae;

        data[i * 4 + 0] = toByte(std::pow(rgb[0], 1.0f / 2.2f));
        data[i * 4 + 1] = toByte(std::pow(rgb[1], 1.0f / 2.2f));
        data[i * 4 + 2] = toByte(std::pow(rgb[2], 1.0f / 2.2f));
        data[i * 4 + 3] = toByte(height);
    }
    return image;
}

osg::Texture2D*
RiverDetail::createDataTexture(osg::Image* image)
{
    osg::Texture2D* tex = new osg::Texture2D(image);
    tex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
    tex->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
    tex->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
    tex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
    tex->setMaxAnisotropy(8.0f);
    tex->setUseHardwareMipMapGeneration(true);
    tex->setResizeNonPowerOfTwoHint(false);
    tex->setInternalFormat(GL_RGBA8);
    return tex;
}

//...................................................................

namespace
{
    //! A texture or framebuffer owned by osgEarth's GL object pool, which deletes it on
    //! its own context once nothing else references it.
    class CopyObject : public GLObject
    {
    public:
        using Ptr = std::shared_ptr<CopyObject>;

        //! Creates an unallocated texture name; call with the context current.
        static Ptr texture(osg::State& state, std::size_t bytes)
        {
            Ptr object(new CopyObject(GL_TEXTURE, state, bytes));
            glGenTextures(1, &object->_name);
            GLObjectPool::get(state)->watch(object);
            return object;
        }

        //! Creates a framebuffer name; call with the context current.
        static Ptr framebuffer(osg::State& state)
        {
            Ptr object(new CopyObject(GL_FRAMEBUFFER_EXT, state, 0));
            object->ext()->glGenFramebuffers(1, &object->_name);
            GLObjectPool::get(state)->watch(object);
            return object;
        }

        //! Called by the pool with the owning context current.
        void release() override
        {
            if (_name == 0) return;
            if (ns() == GL_FRAMEBUFFER_EXT) ext()->glDeleteFramebuffers(1, &_name);
            else glDeleteTextures(1, &_name);
            _name = 0;
        }

        GLsizei size() const override { return _bytes; }

    private:
        CopyObject(GLenum ns, osg::State& state, std::size_t bytes) :
            GLObject(ns, state),
            _bytes((GLsizei)std::min<std::size_t>(bytes, (std::size_t)INT_MAX)) { }
        GLsizei _bytes;
    };

    using GetAttachmentParam = void (GL_APIENTRY*)(GLenum, GLenum, GLenum, GLint*);
}

//! Per camera and context copy targets. Only that context's draw thread touches the GL
//! side; "valid" is read by cull threads.
struct SceneCopyDrawable::Target
{
    unsigned contextID = 0u;
    const osg::Camera* key = nullptr;    // identity only
    osg::observer_ptr<const osg::Camera> camera;
    unsigned lastFrame = 0u;
    GLint source = -1;                   // framebuffer copied from
    GLint width = 0, height = 0;
    GLenum colorFormat = 0, depthFormat = 0;
    CopyObject::Ptr color, depth, fbo;
    GetAttachmentParam getParam = nullptr;
    bool failed = false;
    bool validated = false;
    std::atomic<bool> valid{ false };
};

namespace
{
    //! Internal format that exactly matches a framebuffer attachment (blits that resolve
    //! multisampling require identical formats); 0 if unknown.
    GLenum matchingColorFormat(GetAttachmentParam get, GLenum attachment)
    {
        GLint type = GL_NONE, r = 0, g = 0, b = 0, a = 0, component = GL_NONE, encoding = GL_NONE;
        get(GL_READ_FRAMEBUFFER_EXT, attachment, GL_FRAMEBUFFER_ATTACHMENT_OBJECT_TYPE, &type);
        if (type == GL_NONE)
            return 0;
        get(GL_READ_FRAMEBUFFER_EXT, attachment, GL_FRAMEBUFFER_ATTACHMENT_RED_SIZE, &r);
        get(GL_READ_FRAMEBUFFER_EXT, attachment, GL_FRAMEBUFFER_ATTACHMENT_GREEN_SIZE, &g);
        get(GL_READ_FRAMEBUFFER_EXT, attachment, GL_FRAMEBUFFER_ATTACHMENT_BLUE_SIZE, &b);
        get(GL_READ_FRAMEBUFFER_EXT, attachment, GL_FRAMEBUFFER_ATTACHMENT_ALPHA_SIZE, &a);
        get(GL_READ_FRAMEBUFFER_EXT, attachment, GL_FRAMEBUFFER_ATTACHMENT_COMPONENT_TYPE, &component);
        get(GL_READ_FRAMEBUFFER_EXT, attachment, GL_FRAMEBUFFER_ATTACHMENT_COLOR_ENCODING, &encoding);
        if (component == GL_FLOAT)
        {
            if (r == 11 && b == 10) return GL_R11F_G11F_B10F;
            if (r == 16) return a ? GL_RGBA16F_ARB : GL_RGB16F_ARB;
            if (r == 32) return a ? GL_RGBA32F_ARB : GL_RGB32F_ARB;
            return 0;
        }
        if (encoding == GL_SRGB && r == 8)
            return a ? GL_SRGB8_ALPHA8 : GL_SRGB8;
        if (r == 8 && g == 8 && b == 8)
            return a ? GL_RGBA8 : GL_RGB8;
        if (r == 10 && g == 10 && b == 10)
            return GL_RGB10_A2;
        if (r == 16)
            return a ? GL_RGBA16 : GL_RGB16;
        return 0;
    }

    //! Depth internal format matching a framebuffer's depth attachment; 0 if unknown.
    GLenum matchingDepthFormat(GetAttachmentParam get, bool defaultFramebuffer)
    {
        const GLenum depthAttachment = defaultFramebuffer ? GL_DEPTH : GL_DEPTH_ATTACHMENT_EXT;
        const GLenum stencilAttachment = defaultFramebuffer ? GL_STENCIL : GL_STENCIL_ATTACHMENT_EXT;
        GLint type = GL_NONE, bits = 0, component = GL_NONE, stencilType = GL_NONE, stencil = 0;
        get(GL_READ_FRAMEBUFFER_EXT, depthAttachment, GL_FRAMEBUFFER_ATTACHMENT_OBJECT_TYPE, &type);
        if (type == GL_NONE)
            return 0;
        get(GL_READ_FRAMEBUFFER_EXT, depthAttachment, GL_FRAMEBUFFER_ATTACHMENT_DEPTH_SIZE, &bits);
        get(GL_READ_FRAMEBUFFER_EXT, depthAttachment, GL_FRAMEBUFFER_ATTACHMENT_COMPONENT_TYPE, &component);
        get(GL_READ_FRAMEBUFFER_EXT, stencilAttachment, GL_FRAMEBUFFER_ATTACHMENT_OBJECT_TYPE, &stencilType);
        if (stencilType != GL_NONE)
            get(GL_READ_FRAMEBUFFER_EXT, stencilAttachment, GL_FRAMEBUFFER_ATTACHMENT_STENCIL_SIZE, &stencil);
        if (component == GL_FLOAT && bits == 32) return stencil ? GL_DEPTH32F_STENCIL8 : GL_DEPTH_COMPONENT32F;
        if (bits == 24) return stencil ? GL_DEPTH24_STENCIL8_EXT : GL_DEPTH_COMPONENT24;
        if (stencil == 0 && bits == 16) return GL_DEPTH_COMPONENT16;
        if (stencil == 0 && bits == 32) return GL_DEPTH_COMPONENT32;
        return 0;
    }

    //! Allocates immutable-size storage for a texture bound to GL_TEXTURE_2D.
    void allocateStorage(osg::GLExtensions* ext, GLenum format, GLint levels, GLint width, GLint height)
    {
        if (ext->glTexStorage2D)
        {
            ext->glTexStorage2D(GL_TEXTURE_2D, levels, format, width, height);
            return;
        }
        GLenum pixelFormat = GL_RGBA, type = GL_UNSIGNED_BYTE;
        switch (format)
        {
        case GL_DEPTH24_STENCIL8_EXT: pixelFormat = GL_DEPTH_STENCIL_EXT, type = GL_UNSIGNED_INT_24_8_EXT; break;
        case GL_DEPTH32F_STENCIL8: pixelFormat = GL_DEPTH_STENCIL_EXT, type = GL_FLOAT_32_UNSIGNED_INT_24_8_REV; break;
        case GL_DEPTH_COMPONENT32F: pixelFormat = GL_DEPTH_COMPONENT, type = GL_FLOAT; break;
        case GL_DEPTH_COMPONENT16:
        case GL_DEPTH_COMPONENT24:
        case GL_DEPTH_COMPONENT32: pixelFormat = GL_DEPTH_COMPONENT, type = GL_UNSIGNED_INT; break;
        default: pixelFormat = GL_RGBA, type = GL_FLOAT; break;
        }
        for (GLint level = 0; level < levels; ++level)
        {
            glTexImage2D(GL_TEXTURE_2D, level, format, std::max(1, width >> level), std::max(1, height >> level), 0,
                pixelFormat, type, nullptr);
        }
    }
}

SceneCopyDrawable::SceneCopyDrawable(int colorUnit, int depthUnit) :
    _colorUnit(colorUnit),
    _depthUnit(depthUnit)
{
    setName("River scene copy");
    setUseDisplayList(false);
    setUseVertexBufferObjects(false);
    setCullingActive(false);
    setDataVariance(osg::Object::DYNAMIC);
}

bool
SceneCopyDrawable::isValidFor(const osg::Camera* camera) const
{
    std::lock_guard<std::mutex> lock(_mutex);
    for (auto& t : _targets)
        if (t->key == camera && t->valid)
            return true;
    return false;
}

void
SceneCopyDrawable::releaseGLObjects(osg::State* state) const
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (!state)
    {
        _targets.clear();
        return;
    }
    unsigned id = state->getContextID();
    _targets.erase(
        std::remove_if(_targets.begin(), _targets.end(), [id](const std::shared_ptr<Target>& t) { return t->contextID == id; }),
        _targets.end());
}

void
SceneCopyDrawable::drawImplementation(osg::RenderInfo& ri) const
{
    osg::State& state = *ri.getState();
    const osg::Camera* camera = ri.getCurrentCamera();
    const osg::Viewport* viewport = state.getCurrentViewport();
    if (!camera || !viewport)
        return;

    const GLint x = (GLint)viewport->x(), y = (GLint)viewport->y();
    const GLint width = (GLint)viewport->width(), height = (GLint)viewport->height();
    if (width < 2 || height < 2)
        return;

    osg::GLExtensions* ext = state.get<osg::GLExtensions>();
    if (!ext || !ext->glBlitFramebuffer || !ext->glBindFramebuffer || !ext->glGenFramebuffers ||
        !ext->glFramebufferTexture2D || !ext->glCheckFramebufferStatus)
        return;

    const unsigned contextID = state.getContextID();
    const unsigned frame = state.getFrameStamp() ? state.getFrameStamp()->getFrameNumber() : 0u;

    std::shared_ptr<Target> target;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        for (auto i = _targets.begin(); i != _targets.end(); )
        {
            const Target& t = **i;
            if (t.contextID == contextID && (!t.camera.valid() || (t.key != camera && frame - t.lastFrame > 600u)))
                i = _targets.erase(i);
            else
                ++i;
        }
        for (auto& t : _targets)
            if (t->contextID == contextID && t->key == camera)
                target = t;
        if (!target)
        {
            target = std::make_shared<Target>();
            target->contextID = contextID;
            target->key = camera;
            target->camera = camera;
            _targets.push_back(target);
        }
    }

    Target& t = *target;
    t.lastFrame = frame;
    if (t.failed)
        return;

    if (!t.getParam)
    {
        osg::setGLExtensionFuncPtr(t.getParam,
            "glGetFramebufferAttachmentParameteriv", "glGetFramebufferAttachmentParameterivEXT");
        if (!t.getParam)
        {
            t.failed = true;
            return;
        }
    }

    GLint drawFBO = 0, readFBO = 0;
    glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &drawFBO);
    glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &readFBO);

    if (drawFBO != t.source || width != t.width || height != t.height || !t.color)
    {
        // Match the source's formats exactly; resolving a multisampled buffer requires it.
        ext->glBindFramebuffer(GL_READ_FRAMEBUFFER_EXT, drawFBO);
        GLint readBuffer = GL_BACK;
        glGetIntegerv(GL_READ_BUFFER, &readBuffer);
        GLenum colorAttachment = (GLenum)readBuffer;
        if (drawFBO == 0)
            colorAttachment = readBuffer == GL_FRONT ? GL_FRONT_LEFT : readBuffer == GL_BACK ? GL_BACK_LEFT : (GLenum)readBuffer;
        GLenum colorFormat = matchingColorFormat(t.getParam, colorAttachment);
        GLenum depthFormat = matchingDepthFormat(t.getParam, drawFBO == 0);
        ext->glBindFramebuffer(GL_READ_FRAMEBUFFER_EXT, readFBO);

        if (colorFormat == 0 || depthFormat == 0)
        {
            t.failed = true;
            t.valid = false;
            OE_INFO << LC << "Camera \"" << camera->getName() << "\" framebuffer cannot be copied; "
                "water refraction and reflections are simplified" << std::endl;
            return;
        }

        t.source = drawFBO;
        t.width = width;
        t.height = height;
        t.colorFormat = colorFormat;
        t.depthFormat = depthFormat;
        t.validated = false;

        GLint levels = 1;
        while ((std::max(width, height) >> levels) > 0 && levels < 7) ++levels;

        state.setActiveTextureUnit(_colorUnit);
        t.color = CopyObject::texture(state, (std::size_t)width * height * 4u * 4u / 3u);
        glBindTexture(GL_TEXTURE_2D, t.color->name());
        allocateStorage(ext, colorFormat, levels, width, height);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, levels - 1);

        state.setActiveTextureUnit(_depthUnit);
        t.depth = CopyObject::texture(state, (std::size_t)width * height * 4u);
        glBindTexture(GL_TEXTURE_2D, t.depth->name());
        allocateStorage(ext, depthFormat, 1, width, height);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);

        t.fbo = CopyObject::framebuffer(state);
        ext->glBindFramebuffer(GL_DRAW_FRAMEBUFFER_EXT, t.fbo->name());
        ext->glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, t.color->name(), 0);
        const bool packed = depthFormat == GL_DEPTH24_STENCIL8_EXT || depthFormat == GL_DEPTH32F_STENCIL8;
        ext->glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER_EXT, packed ? GL_DEPTH_STENCIL_ATTACHMENT : GL_DEPTH_ATTACHMENT_EXT,
            GL_TEXTURE_2D, t.depth->name(), 0);
        GLenum status = ext->glCheckFramebufferStatus(GL_DRAW_FRAMEBUFFER_EXT);
        ext->glBindFramebuffer(GL_DRAW_FRAMEBUFFER_EXT, drawFBO);
        if (status != GL_FRAMEBUFFER_COMPLETE_EXT)
        {
            t.failed = true;
            t.valid = false;
            t.color = t.depth = t.fbo = nullptr;
            OE_WARN << LC << "Scene copy framebuffer incomplete (0x" << std::hex << status << std::dec
                << "); water refraction and reflections are simplified" << std::endl;
            return;
        }
    }

    // Errors raised earlier by other code are not ours; drain them before checking the copy.
    if (!t.validated)
        for (int i = 0; i < 16 && glGetError() != GL_NO_ERROR; ++i) { }

    // Only the scissor test applies to blits.
    const GLboolean scissor = glIsEnabled(GL_SCISSOR_TEST);
    if (scissor) glDisable(GL_SCISSOR_TEST);
    ext->glBindFramebuffer(GL_READ_FRAMEBUFFER_EXT, drawFBO);
    ext->glBindFramebuffer(GL_DRAW_FRAMEBUFFER_EXT, t.fbo->name());
    ext->glBlitFramebuffer(x, y, x + width, y + height, 0, 0, width, height,
        GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT, GL_NEAREST);
    ext->glBindFramebuffer(GL_READ_FRAMEBUFFER_EXT, readFBO);
    ext->glBindFramebuffer(GL_DRAW_FRAMEBUFFER_EXT, drawFBO);
    if (scissor) glEnable(GL_SCISSOR_TEST);

    if (!t.validated)
    {
        GLenum error = glGetError();
        if (error != GL_NO_ERROR)
        {
            t.failed = true;
            t.valid = false;
            t.color = t.depth = t.fbo = nullptr;
            OE_WARN << LC << "Scene copy failed (GL error 0x" << std::hex << error << std::dec
                << "); water refraction and reflections are simplified" << std::endl;
            return;
        }
        t.validated = true;
    }

    // Bind for the water, which draws next. These units are reserved for the river layer.
    state.setActiveTextureUnit(_colorUnit);
    glBindTexture(GL_TEXTURE_2D, t.color->name());
    if (ext->glGenerateMipmap)
        ext->glGenerateMipmap(GL_TEXTURE_2D);
    state.setActiveTextureUnit(_depthUnit);
    glBindTexture(GL_TEXTURE_2D, t.depth->name());
    t.valid = true;
}
