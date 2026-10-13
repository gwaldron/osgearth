/* osgEarth
 * Copyright 2026 Pelican Mapping
 * MIT License
 */
#include <osgEarthPrestige/SurfaceSplattingLayer>
#include <osgEarthPrestige/PrestigeShaders>
#include <osgEarth/Map>
#include <osgEarth/MaterialTexturePreparation>
#include <osgEarth/PolygonSymbol>
#include <osgEarth/Progress>
#include <osgEarth/Shaders>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/TerrainResources>
#include <osgEarth/VirtualProgram>
#include <osgUtil/CullVisitor>
#include <array>
#include <cmath>

using namespace osgEarth;

namespace osgEarthPrestige
{
REGISTER_OSGEARTH_LAYER_FACTORY("prestige:surface_splatting", SurfaceSplattingLayer);

namespace
{
    //! Preserve exact coverage bytes in a private R8 image; legacy RGBA caches contribute alpha.
    //! Empty/unsupported inputs fail closed, and cancellation never publishes a partial tile.
    GeoImage redCoverage(const GeoImage& source, ProgressCallback* progress)
    {
        if (!source.valid() || (progress && progress->isCanceled())) return GeoImage::INVALID;
        const auto* input = source.getImage();
        const auto format = input->getPixelFormat();
        if (input->isCompressed() || input->getDataType() != GL_UNSIGNED_BYTE || input->r() != 1)
            return GeoImage(Status(Status::ConfigurationError, "Surface coverage requires uncompressed 8-bit pixels"));
        if (format == GL_RED && input->getInternalTextureFormat() == GL_R8) return source;
        const unsigned channels = format == GL_RGBA || format == GL_BGRA ? 4u :
            format == GL_RED || format == GL_LUMINANCE ? 1u : 0u;
        if (channels == 0u)
            return GeoImage(Status(Status::ConfigurationError, "Unsupported surface coverage cache encoding"));
        osg::ref_ptr<osg::Image> output = new osg::Image;
        output->allocateImage(input->s(), input->t(), 1, GL_RED, GL_UNSIGNED_BYTE, 1);
        output->setInternalTextureFormat(GL_R8);
        output->setOrigin(input->getOrigin());
        output->setName(input->getName());
        for (int y = 0; y < input->t(); ++y)
        {
            if (progress && progress->isCanceled()) return GeoImage::INVALID;
            const auto* read = input->data(0, y);
            auto* write = output->data(0, y);
            for (int x = 0; x < input->s(); ++x) write[x] = read[x*channels + channels-1u];
        }
        GeoImage result(output.get(), source.getExtent());
        result.setTrackingToken(source.getTrackingToken());
        return result;
    }

    //! Construct a one-texel fallback; an absent file differs from a requested file that failed to load.
    osg::Image* constantImage(const osg::Vec4& value)
    {
        auto* image = new osg::Image;
        image->allocateImage(1, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE);
        image->setColor(value, 0, 0);
        return image;
    }

    //! Recognize hardware sRGB decoding, including precompressed material images.
    bool decodesSRGB(GLenum format)
    {
        return format == GL_SRGB || format == GL_SRGB_ALPHA || format == GL_SRGB8 || format == GL_SRGB8_ALPHA8 ||
            format == 0x8C4C || format == 0x8C4D || format == 0x8C4E || format == 0x8C4F || format == 0x8E8D;
    }

    //! Own a regional frame independently of the layer; uniforms are private to each cull traversal.
    struct SurfaceCullCallback : Layer::TraversalCallback
    {
        osg::Matrixd worldToRegion;
        double period = 4.0;

        //! Cancel ECEF translation in doubles, then reduce only translation to preserve UV derivatives.
        void operator()(osg::Node* node, osg::NodeVisitor* nv) const override
        {
            auto* cv = dynamic_cast<osgUtil::CullVisitor*>(nv);
            osg::Matrixd inverseView;
            if (!cv || !inverseView.invert(*cv->getModelViewMatrix()))
            {
                traverse(node, nv);
                return;
            }
            osg::Matrixd matrix = inverseView * worldToRegion * osg::Matrixd::scale(1.0/period, 1.0/period, 1.0/period);
            for (unsigned axis = 0; axis < 3; ++axis) matrix(3, axis) -= std::floor(matrix(3, axis));
            osg::ref_ptr<osg::StateSet> state = new osg::StateSet;
            state->addUniform(new osg::Uniform("oe_surface_viewToTexture", osg::Matrixf(matrix)));
            cv->pushStateSet(state);
            traverse(node, nv);
            cv->popStateSet();
        }
    };
}

struct SurfaceSplattingLayer::Impl
{
    osg::ref_ptr<PBRTexture> texture;
    osg::ref_ptr<SurfaceCullCallback> callback;
    std::array<TextureImageUnitReservation, 4> units;
    bool normal = false;
    bool albedoLinear = false;

    //! Roll back every unit, including a partially successful allocation.
    void releaseUnits(osg::StateSet* state)
    {
        for (auto& unit : units)
        {
            if (state && unit.valid()) state->removeTextureAttribute(unit.unit(), osg::StateAttribute::TEXTURE);
            unit.release();
        }
        if (state)
        {
            state->removeDefine("OE_TERRAIN_LAYER_MATERIAL");
            state->removeDefine("OE_TERRAIN_LAYER_COVERAGE_RED");
            PrestigeShaders shaders;
            osgEarth::Util::Shaders core;
            shaders.add("PBRMaterial.glsl", core.context().at(core.PBRMaterial));
            shaders.unload(VirtualProgram::get(state), "SurfaceSplattingLayer.glsl");
        }
    }
};

Config SurfaceSplattingLayer::Options::getConfig() const
{
    Config conf = FeatureImageLayer::Options::getConfig();
    conf.set("material", material());
    conf.set("anchor", anchor());
    conf.set("texture_period", texturePeriod());
    conf.set("normal_strength", normalStrength());
    conf.set("coverage_outline", coverageOutline());
    // Distinguish native R8 tiles from former RGBA tiles in automatic persistent-cache identities.
    conf.set("coverage_encoding", "r8");
    return conf;
}

void SurfaceSplattingLayer::Options::fromConfig(const Config& conf)
{
    conf.get("material", material());
    conf.get("anchor", anchor());
    conf.get("texture_period", texturePeriod());
    conf.get("normal_strength", normalStrength());
    conf.get("coverage_outline", coverageOutline());
}

void SurfaceSplattingLayer::init()
{
    FeatureImageLayer::init();
    _impl.reset(new Impl);
    // Coverage is filtered alpha, not categorical land-cover data. Fill all supplied polygons.
    Style style;
    style.getOrCreate<PolygonSymbol>()->fill()->color() = Color::White;
    osg::ref_ptr<StyleSheet> styles = new StyleSheet;
    styles->addStyle(style);
    setStyleSheet(styles);
    options().backgroundColor() = Color::Transparent;
    options().coverage() = false;
    // An empty child must hide its parent's roads, using the same channel as populated tiles.
    _emptyImage = new osg::Image;
    _emptyImage->allocateImage(1, 1, 1, GL_RED, GL_UNSIGNED_BYTE, 1);
    _emptyImage->setInternalTextureFormat(GL_R8);
    *_emptyImage->data() = 0;
}

SurfaceSplattingLayer::~SurfaceSplattingLayer() = default;

Status SurfaceSplattingLayer::openImplementation()
{
    const auto& anchor = options().anchor().get();
    const auto period = options().texturePeriod().get();
    const auto strength = options().normalStrength().get();
    if (!std::isfinite(period) || period <= 0.0 || !std::isfinite(strength) || strength < 0.0f ||
        !std::isfinite(anchor.x()) || !std::isfinite(anchor.y()) || !std::isfinite(anchor.z()) ||
        std::abs(anchor.x()) > 180.0 || std::abs(anchor.y()) > 90.0)
        return Status(Status::ConfigurationError, "Invalid surface anchor, texture period, or normal strength");

    // Resolve immutable raster settings at open, including options configured after construction.
    Style coverageStyle;
    auto* polygon = coverageStyle.getOrCreate<PolygonSymbol>();
    polygon->fill()->color() = Color::White;
    polygon->outline() = options().coverageOutline().get();
    osg::ref_ptr<StyleSheet> coverageStyles = new StyleSheet;
    coverageStyles->addStyle(coverageStyle);
    setStyleSheet(coverageStyles);
    // Coverage is linear fractional data. Keep filtering, but prevent global imagery compression
    // settings from changing its exact 8-bit values or expanding it to RGBA blocks.
    options().textureCompression() = "none";
    auto status = FeatureImageLayer::openImplementation();
    if (status.isError()) return status;
    auto material = options().material().get();
    _impl->normal = material.normalImage.valid() || material.normal().isSet();
    if (!material.colorImage && !material.color().isSet())
        material.colorImage = constantImage(osg::Vec4(0.22f, 0.23f, 0.24f, 1.0f));
    if (!material.roughnessImage && !material.roughness().isSet() &&
        !material.packedImage && !material.packed().isSet() && material.layout() == PBRMaterial::DRAM)
        material.roughnessImage = constantImage(osg::Vec4(0.85f, 0.85f, 0.85f, 1.0f));
    osg::ref_ptr<PBRTexture> loaded = new PBRTexture;
    status = loaded->load(material, getReadOptions());
    if (status.isError()) return status;
    // Generate albedo mips in linear light. The shader re-encodes hardware-decoded samples for terrain lighting.
    if (!loaded->albedo->getImage(0)->isCompressed()) loaded->albedo->setInternalFormat(GL_SRGB8_ALPHA8);
    // OSG 3.6's ordinary Texture2D upload rewrites compressed formats when the
    // base dimensions are not block aligned, including 1x1 constant fallbacks.
    // Preserve those uncompressed inputs; the shared preparer also serves a
    // direct-upload arena which does not have this restriction.
    osg::ref_ptr<PBRTexture> compatible = new PBRTexture(*loaded, osg::CopyOp::SHALLOW_COPY);
    for (auto* component : { &compatible->albedo, &compatible->normal, &compatible->pbr, &compatible->occlusion })
    {
        if (!*component) continue;
        const auto* image = (*component)->getImage(0);
        if (image && (image->s()%4 != 0 || image->t()%4 != 0))
        {
            if (image->isCompressed())
                return Status(Status::ConfigurationError, "Precompressed surface textures need block-aligned base dimensions");
            *component = nullptr;
        }
    }
    MaterialTexturePreparation preparation(0);
    _impl->texture = preparation.prepare(compatible);
    if (!_impl->texture->albedo) _impl->texture->albedo = loaded->albedo;
    if (!_impl->texture->normal) _impl->texture->normal = loaded->normal;
    if (!_impl->texture->pbr) _impl->texture->pbr = loaded->pbr;
    if (!_impl->texture->occlusion) _impl->texture->occlusion = loaded->occlusion;
    // Older OSG upload code does not size sRGB S3TC formats correctly. Keep the
    // prepared sRGB bytes/mips in ordinary BC storage and let terrain lighting decode.
    auto* albedoImage = _impl->texture->albedo->getImage(0);
    if (albedoImage->isCompressed() && !decodesSRGB(albedoImage->getPixelFormat()))
        _impl->texture->albedo->setInternalFormat(albedoImage->getPixelFormat());
    _impl->albedoLinear = decodesSRGB(_impl->texture->albedo->getInternalFormat()) ||
        decodesSRGB(_impl->texture->albedo->getImage(0)->getInternalTextureFormat());
    _impl->texture->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
    _impl->texture->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
    return Status::OK();
}

GeoImage SurfaceSplattingLayer::createImageImplementation(const TileKey& key, ProgressCallback* progress) const
{
    return redCoverage(FeatureImageLayer::createImageImplementation(key, progress), progress);
}

void SurfaceSplattingLayer::postCreateImageImplementation(
    GeoImage& image, const TileKey& key, ProgressCallback* progress) const
{
    image = redCoverage(image, progress);
}

void SurfaceSplattingLayer::addedToMap(const Map* map)
{
    FeatureImageLayer::addedToMap(map);
    if (!map->getSRS()->isGeographic())
    {
        setStatus(Status(Status::ConfigurationError, "Surface splatting currently requires a geocentric map"));
        return;
    }
    const auto& anchor = options().anchor().get();
    _impl->callback = new SurfaceCullCallback;
    GeoPoint(SpatialReference::get("wgs84"), anchor.x(), anchor.y(), anchor.z(), ALTMODE_ABSOLUTE)
        .createWorldToLocal(_impl->callback->worldToRegion);
    _impl->callback->period = options().texturePeriod().get();
    setCullCallback(_impl->callback);
}

void SurfaceSplattingLayer::removedFromMap(const Map* map)
{
    setCullCallback(nullptr);
    _impl->callback = nullptr;
    _impl->releaseUnits(getStateSet());
    FeatureImageLayer::removedFromMap(map);
}

Status SurfaceSplattingLayer::closeImplementation()
{
    setCullCallback(nullptr);
    _impl->callback = nullptr;
    _impl->releaseUnits(getStateSet());
    _impl->texture = nullptr;
    return FeatureImageLayer::closeImplementation();
}

void SurfaceSplattingLayer::prepareForRendering(TerrainEngine* engine)
{
    FeatureImageLayer::prepareForRendering(engine);
    if (!_impl->texture || getStatus().isError()) return;
    auto* state = getOrCreateStateSet();
    _impl->releaseUnits(getStateSet());
    const char* names[] = { "oe_surface_albedo", "oe_surface_normal", "oe_surface_pbr", "oe_surface_ao" };
    const auto& material = _impl->texture;
    osg::Texture* textures[] = { material->albedo.get(), material->normal.get(),
        material->pbr.get(), material->occlusion.get() };
    for (unsigned i = 0; i < 4; ++i)
    {
        if (!textures[i]) continue;
        if (!engine->getResources()->reserveTextureImageUnitForLayer(_impl->units[i], this, "Prestige surface material"))
        {
            _impl->releaseUnits(getStateSet());
            setStatus(Status(Status::ResourceUnavailable, "Insufficient texture units for surface material"));
            return;
        }
    }
    for (unsigned i = 0; i < 4; ++i)
    {
        if (!textures[i]) continue;
        state->setTextureAttributeAndModes(_impl->units[i].unit(), textures[i], osg::StateAttribute::ON);
        state->addUniform(new osg::Uniform(names[i], _impl->units[i].unit()));
    }
    state->addUniform(new osg::Uniform("oe_surface_factors", material->layoutAndFactors));
    state->addUniform(new osg::Uniform("oe_surface_flags", int(_impl->normal) |
        (material->pbr ? 2 : 0) | (material->occlusion ? 4 : 0) | (_impl->albedoLinear ? 8 : 0)));
    state->addUniform(new osg::Uniform("oe_surface_normalStrength", options().normalStrength().get()));
    state->setDefine("OE_TERRAIN_LAYER_MATERIAL");
    state->setDefine("OE_TERRAIN_LAYER_COVERAGE_RED");
    PrestigeShaders shaders;
    osgEarth::Util::Shaders core;
    shaders.add("PBRMaterial.glsl", core.context().at(core.PBRMaterial));
    shaders.load(VirtualProgram::getOrCreate(state), "SurfaceSplattingLayer.glsl");
}
}
