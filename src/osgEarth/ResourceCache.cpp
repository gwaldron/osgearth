/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/ResourceCache>
#include <osg/Texture2D>

using namespace osgEarth;

#define LC "[ResourceCache] "

bool
ResourceCache::getOrCreateLineTexture(const URI& uri, osg::ref_ptr<osg::Texture>& output, const osgDB::Options* readOptions)
{
    auto result = _texCache.get_or_insert(
        uri.full(),
        [&](auto& new_value) {
            osg::ref_ptr<osg::Image> image = uri.getImage(readOptions);
            if (image.valid())
            {
                osg::Texture2D* tex = new osg::Texture2D(image);
                tex->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
                tex->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
                tex->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
                tex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
                tex->setMaxAnisotropy(4.0f);
                tex->setResizeNonPowerOfTwoHint(false);
                tex->setUnRefImageDataAfterApply(false);
                new_value = tex;
            }
        });

    output = result.has_value() ? result.value() : nullptr;
    return output.valid();
}

bool
ResourceCache::getOrCreateStateSet(SkinResource*                skin,
                                   osg::ref_ptr<osg::StateSet>& output,
                                   const osgDB::Options*        readOptions)
{
    // Note: we use the imageURI as the basis for the caching key since 
    // it's the only property used by Skin->createStateSet(). If that
    // changes, we need to address it here. It might be better it SkinResource
    // were to provide a unique key.

    auto result = _skinCache.get_or_insert(
        skin->getUniqueID(),
        [&](auto& new_value) {
            osg::ref_ptr<osg::StateSet> ss = skin->createStateSet(readOptions);
            if (ss.valid()) new_value = ss;
        });

    output = result.has_value() ? result.value() : nullptr;
    return output.valid();
}

bool
ResourceCache::getOrCreateInstanceNode(InstanceResource*        res,
                                       osg::ref_ptr<osg::Node>& output,
                                       const osgDB::Options*    readOptions)
{
    std::string key = res->getConfig().toJSON(false);

    auto result = _instanceCache.get_or_insert(
        key,
        [&](auto& new_value) {
            osg::ref_ptr<osg::Node> node = res->createNode(readOptions);
            if (node.valid()) new_value = node;
        });

    output = result.has_value() ? result.value() : nullptr;
    return output.valid();
}

bool
ResourceCache::cloneOrCreateInstanceNode(InstanceResource*        res,
                                         osg::ref_ptr<osg::Node>& output,
                                         const osgDB::Options*    readOptions)
{
    std::string key = res->getConfig().toJSON(false);

    // Deep copy everything except for images.  Some models may share imagery so we only want one copy of it at a time.
    osg::CopyOp copyOp = osg::CopyOp::DEEP_COPY_ALL & ~osg::CopyOp::DEEP_COPY_IMAGES & ~osg::CopyOp::DEEP_COPY_TEXTURES;

    auto result = _instanceCache.get_or_insert(
        key,
        [&](auto& new_value) {
            osg::ref_ptr<osg::Node> node = res->createNode(readOptions);
            if (node.valid()) new_value = node;
        });

    if (result.has_value() && result.value().valid())
    {
        output = osg::clone(result.value().get(), copyOp);
    }

    return output.valid();
}
