/* osgEarth
 * Copyright 2026 Pelican Mapping
 * MIT License
 */
#pragma once

// Internal header: procedural water textures and the scene copy pass used by
// RiverLayer. Not part of the public API.

#include <osgEarth/Common>
#include <osg/Drawable>
#include <osg/Texture2D>
#include <osg/Camera>
#include <osg/observer_ptr>
#include <memory>
#include <mutex>
#include <vector>

namespace osgEarth { namespace RiverDetail
{
    //! Generates the tiling wave texture: RG = surface normal (x, y) of a broadband
    //! ripple height field, B = the height itself, A = caustic intensity computed by
    //! refracting light through a smoother swell onto a flat bed. Deterministic.
    osg::ref_ptr<osg::Image> createWaveImage(unsigned size);

    //! Generates the tiling foam/noise texture: R = coarse foam lace, G = fine bubbles,
    //! B = low-frequency noise, A = mid-frequency noise. Deterministic.
    osg::ref_ptr<osg::Image> createFoamImage(unsigned size);

    //! Generates the tiling river bed texture: RGB = sRGB albedo of rounded cobbles over
    //! gravel and silt with a patchy algae film, A = relative stone height. Deterministic.
    osg::ref_ptr<osg::Image> createBedImage(unsigned size);

    //! Wraps an image in a repeating, mipmapped, anisotropically filtered texture.
    osg::Texture2D* createDataTexture(osg::Image* image);

    /**
     * Copies the opaque scene (color and depth) of whatever camera is drawing into
     * textures bound on two reserved texture units, so the water shader can refract,
     * reflect and measure water depth against it. Place it in a render bin that draws
     * after the opaque scene and before the water. Copies are kept per camera and per
     * graphics context and resized with the viewport.
     */
    class SceneCopyDrawable : public osg::Drawable
    {
    public:
        SceneCopyDrawable(int colorUnit, int depthUnit);

        //! Whether the most recent copy for this camera succeeded (false if the camera's
        //! framebuffer cannot be copied). Safe from the cull thread.
        bool isValidFor(const osg::Camera* camera) const;

        void drawImplementation(osg::RenderInfo& renderInfo) const override;

        void releaseGLObjects(osg::State* state) const override;

        osg::BoundingBox computeBoundingBox() const override { return osg::BoundingBox(); }

        const char* libraryName() const override { return "osgEarth"; }
        const char* className() const override { return "RiverSceneCopy"; }
        osg::Object* cloneType() const override { return nullptr; }
        osg::Object* clone(const osg::CopyOp&) const override { return nullptr; }

    public:
        struct Target;

    private:
        int _colorUnit, _depthUnit;
        mutable std::mutex _mutex;
        mutable std::vector<std::shared_ptr<Target>> _targets;
    };
} }
