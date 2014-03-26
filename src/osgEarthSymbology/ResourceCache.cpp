/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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

using namespace osgEarth;
using namespace osgEarth::Symbology;


// internal thread-safety not required since we mutex it in this object.
ResourceCache::ResourceCache(const osgDB::Options* dbOptions ) :
_dbOptions    ( dbOptions ),
_skinCache    ( false ),
_instanceCache( false ),
_skinTextureArrayCache( false ),
_resourceLibraryCache( false )
{
    //nop
}

bool
ResourceCache::getOrCreateStateSet(SkinResource*                skin,
                                   osg::ref_ptr<osg::StateSet>& output)
{
    output = 0L;
    std::string key = skin->getConfig().toJSON(false);

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
            output = skin->createStateSet( _dbOptions.get() );
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
                                       osg::ref_ptr<osg::Node>& output)
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
            output = res->createNode( _dbOptions.get() );
            if ( output.valid() )
            {
                _instanceCache.insert( key, output.get() );
            }
        }
    }

    return output.valid();
}

 bool ResourceCache::getOrCreateSkinTextureArray(ResourceLibrary* library, osg::ref_ptr< SkinTextureArray >& output )
 {
    output = 0L;
    std::string key = library->getConfig().toJSON(false);

    // exclusive lock (since it's an LRU)
    {
        Threading::ScopedMutexLock exclusive( _skinTextureArrayMutex );
            
        // double check to avoid race condition
        SkinTextureArrayCache::Record rec;
        if ( _skinTextureArrayCache.get(key, rec) && rec.value().valid() )
        {
            output = rec.value().get();
        }
        else
        {
            SkinResourceVector skins;
            library->getSkins( skins );
            output = new SkinTextureArray();
            output->build( skins, _dbOptions.get() );
            _skinTextureArrayCache.insert( key, output.get() );            
        }
    }

    return output.valid();
 }

 bool ResourceCache::getOrCreateStateSet( ResourceLibrary* library,  osg::ref_ptr<osg::StateSet>& output )
 {
    output = 0L;
    std::string key = library->getConfig().toJSON(false);

    // exclusive lock (since it's an LRU)
    {
        Threading::ScopedMutexLock exclusive( _resourceLibraryMutex );
            
        // double check to avoid race condition
        ResourceLibraryCache::Record rec;
        if ( _resourceLibraryCache.get(key, rec) && rec.value().valid() )
        {
            output = rec.value().get();
        }
        else
        {
            osg::ref_ptr< SkinTextureArray > skinTextureArray;
            getOrCreateSkinTextureArray( library, skinTextureArray );

            output = new osg::StateSet();
            output->setTextureAttribute(0, skinTextureArray->getTexture(), osg::StateAttribute::ON);

            _resourceLibraryCache.insert( key, output.get() );            
        }
    }

    return output.valid();
 }


bool
ResourceCache::cloneOrCreateInstanceNode(InstanceResource*        res,
                                         osg::ref_ptr<osg::Node>& output)
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
            output = osg::clone(rec.value().get(), osg::CopyOp::DEEP_COPY_ALL);
        }
        else
        {
            // still not there, make it.
            output = res->createNode( _dbOptions.get() );
            if ( output.valid() )
            {
                _instanceCache.insert( key, output.get() );
                output = osg::clone(output.get(), osg::CopyOp::DEEP_COPY_ALL);
            }
        }
    }

    return output.valid();
}
