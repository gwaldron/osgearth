/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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
#include <osgEarthSymbology/ResourceCache>
#include <osg/Texture2D>

using namespace osgEarth;
using namespace osgEarth::Symbology;


// internal thread-safety not required since we mutex it in this object.
ResourceCache::ResourceCache() : // const osgDB::Options* dbOptions ) :
//_dbOptions    ( dbOptions ),
_skinCache    ( false ),
_instanceCache( false ),
_resourceLibraryCache( false )
{
    //nop
}

bool
ResourceCache::getOrCreateLineTexture(const URI& uri, osg::ref_ptr<osg::Texture>& output, const osgDB::Options* readOptions)
{
    Threading::ScopedMutexLock lock(_texMutex);
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
            output = tex;
            _texCache.insert(uri.full(), output.get());
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
        Threading::ScopedMutexLock exclusive( _skinMutex );
            
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
        Threading::ScopedMutexLock exclusive( _instanceMutex );

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
        Threading::ScopedMutexLock exclusive( _instanceMutex );

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
