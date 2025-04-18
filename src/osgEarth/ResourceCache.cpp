/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/ResourceCache>
#include <osg/Texture2D>

using namespace osgEarth;

#define LC "[ResourceCache] "

// internal thread-safety not required since we mutex it in this object.
ResourceCache::ResourceCache() :
    _skinCache(false),
    _instanceCache(false),
    _texCache(false)
{
    //nop
}

bool
ResourceCache::getOrCreateLineTexture(const URI& uri, osg::ref_ptr<osg::Texture>& output, const osgDB::Options* readOptions)
{
    std::lock_guard<std::mutex> lock(_texMutex);
    TextureCache::Record rec;
    if (_texCache.get(uri.full(), rec) && rec.value().valid())
    {
        output = rec.value().get();
    }
    else
    {
        osg::ref_ptr<osg::Image> image = uri.getImage(readOptions);
        if (image.valid())
        {
            osg::Texture2D* tex = new osg::Texture2D(image);
            tex->setWrap( osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE );
            tex->setWrap( osg::Texture::WRAP_T, osg::Texture::REPEAT );
            tex->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR );
            tex->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
            tex->setMaxAnisotropy( 4.0f );
            tex->setResizeNonPowerOfTwoHint( false );
            tex->setUnRefImageDataAfterApply(false);
            output = tex;
            _texCache.insert(uri.full(), output.get());
        }
        else
        {
            OE_WARN << LC << "Unable to load image from " << uri.full() << std::endl;
        }
    }

    return output.valid();
}

bool
ResourceCache::getOrCreateStateSet(SkinResource*                skin,
                                   osg::ref_ptr<osg::StateSet>& output,
                                   const osgDB::Options*        readOptions)
{
    output = 0L;
    //std::string key = skin->getConfig().toJSON(false);

    // Note: we use the imageURI as the basis for the caching key since 
    // it's the only property used by Skin->createStateSet(). If that
    // changes, we need to address it here. It might be better it SkinResource
    // were to provide a unique key.
    std::string key = skin->getUniqueID();

    // exclusive lock (since it's an LRU)
    {
        std::lock_guard<std::mutex> exclusive( _skinMutex );
            
        // double check to avoid race condition
        SkinCache::Record rec;       
        if ( _skinCache.get(key, rec) && rec.value().valid() )
        {
            output = rec.value().get();
        }
        else
        {
            // still not there, make it.
            output = skin->createStateSet(readOptions);
            if ( output.valid() )
            {
                _skinCache.insert( key, output.get() );
            }
        }
    }

    return output.valid();
}


bool
ResourceCache::getOrCreateInstanceNode(InstanceResource*        res,
                                       osg::ref_ptr<osg::Node>& output,
                                       const osgDB::Options*    readOptions)
{
    output = 0L;
    std::string key = res->getConfig().toJSON(false);

    // exclusive lock (since it's an LRU)
    {
        std::lock_guard<std::mutex> exclusive( _instanceMutex );

        // double check to avoid race condition
        InstanceCache::Record rec;
        if ( _instanceCache.get(key, rec) && rec.value().valid() )
        {
            output = rec.value().get();
        }
        else
        {
            // still not there, make it.
            output = res->createNode(readOptions);
            if ( output.valid() )
            {
                _instanceCache.insert( key, output.get() );
            }
        }
    }

    return output.valid();
}

bool
ResourceCache::cloneOrCreateInstanceNode(InstanceResource*        res,
                                         osg::ref_ptr<osg::Node>& output,
                                         const osgDB::Options*    readOptions)
{
    output = 0L;
    std::string key = res->getConfig().toJSON(false);

    // exclusive lock (since it's an LRU)
    {
        std::lock_guard<std::mutex> exclusive( _instanceMutex );

        // Deep copy everything except for images.  Some models may share imagery so we only want one copy of it at a time.
        osg::CopyOp copyOp = osg::CopyOp::DEEP_COPY_ALL & ~osg::CopyOp::DEEP_COPY_IMAGES & ~osg::CopyOp::DEEP_COPY_TEXTURES;

        // double check to avoid race condition
        InstanceCache::Record rec;
        if ( _instanceCache.get(key, rec) && rec.value().valid() )
        {
            output = osg::clone(rec.value().get(), copyOp);
        }
        else
        {
            // still not there, make it.
            output = res->createNode(readOptions);
            if ( output.valid() )
            {
                _instanceCache.insert( key, output.get() );
                output = osg::clone(output.get(), copyOp);
            }
        }
    }

    return output.valid();
}
