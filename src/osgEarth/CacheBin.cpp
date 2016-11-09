/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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
#include <osgEarth/CacheBin>
#include <osgEarth/ImageUtils>
#include <osgEarth/ThreadingUtils>
#include <osgEarth/Registry>
#include <osgEarth/Cache>

#include <osgDB/ReaderWriter>
#include <osgDB/FileNameUtils>
#include <osgDB/Registry>
#include <osg/NodeVisitor>
#include <osg/Texture>
#include <osg/Image>
#include <osg/TextureBuffer>

using namespace osgEarth;

namespace
{
#undef  LC
#define LC "[PrepareForCaching] "
    
    /**
     * Visitor that preps a scene graph for writing to the cache. 
     *
     * There are various things that need to happen:
     *
     * - Remove any user data containers, since these will not serialize and
     *   will cause the OSG serializer to fail.
     *
     * - Replace texture image filenames to point at objects in the cache.
     *   Before doing this, however, you need to run the WriteImagesToCache
     *   visitor.
     */
    struct PrepareForCaching : public osg::NodeVisitor
    {
        unsigned _textures;         // profiling 
        unsigned _userDataClears;   // profiling

        PrepareForCaching() : osg::NodeVisitor()
        {
            setTraversalMode(TRAVERSE_ALL_CHILDREN);
            setNodeMaskOverride(~0);
            _textures = 0;
            _userDataClears = 0;
        }

        void apply(osg::Node& node)
        {
            apply(node.getStateSet());
            applyUserData(node);
            traverse(node);
        }

        void apply(osg::Geode& geode)
        {
            for (unsigned i = 0; i < geode.getNumDrawables(); ++i)
            {
                apply(geode.getDrawable(i));
            }
            apply(static_cast<osg::Node&>(geode));
        }

        void apply(osg::Drawable* drawable)
        {
            if (!drawable) return;
            apply(drawable->getStateSet());
            applyUserData(*drawable);
            
            osg::Geometry* geom = drawable->asGeometry();
            if (geom)
                apply(geom);
        }

        void apply(osg::Geometry* geom)
        {
            // This detects any NULL vertex attribute arrays and then populates them.
            // Do this because a NULL VAA will crash the OSG serialization reader (osg 3.4.0)
            osg::Geometry::ArrayList& arrays = geom->getVertexAttribArrayList();
            for (osg::Geometry::ArrayList::iterator i = arrays.begin(); i != arrays.end(); ++i)
            {
                if (i->get() == 0L)
                {
                    *i = new osg::FloatArray();
                    i->get()->setBinding( osg::Array::BIND_OFF );
                }
            }
        }

        void apply(osg::StateSet* ss)
        {
            if (!ss) return;

            osg::StateSet::AttributeList& a0 = ss->getAttributeList();
            for (osg::StateSet::AttributeList::iterator i = a0.begin(); i != a0.end(); ++i)
            {
                osg::StateAttribute* sa = i->second.first.get();
                applyUserData(*sa);
            }

            // Disable the texture image-unref feature so we can share the resource 
            // across cached tiles.
            osg::StateSet::TextureAttributeList& a = ss->getTextureAttributeList();
            for (osg::StateSet::TextureAttributeList::iterator i = a.begin(); i != a.end(); ++i)
            {       
                osg::StateSet::AttributeList& b = *i;
                for (osg::StateSet::AttributeList::iterator j = b.begin(); j != b.end(); ++j)
                {
                    osg::StateAttribute* sa = j->second.first.get();
                    if (sa)
                    {
                        osg::Texture* tex = dynamic_cast<osg::Texture*>(sa);
                        if (tex)
                        {              
                            tex->setUnRefImageDataAfterApply(false);               

                            // OSG's DatabasePager attaches "marker objects" to Textures' UserData when it runs a
                            // FindCompileableGLObjectsVisitor. This operation is not thread-safe; it doesn't
                            // account for the possibility that the texture may already be in use elsewhere.
                            //
                            // To prevent a threading violation, and the ensuing crash that reliably occurs
                            // in Release mode (but not Debug for whatever reason) we are forced to make a
                            // shallow clone of the Texture object and use that for serialization instead of
                            // the original, since the original may change in the middle of the process.
                            // We then replace the original with our close locally and serialize it safely.
                            //
                            // This "hack" prevents a crash in OSG 3.4.0 when trying to modify and then write
                            // serialize the scene graph containing these shared texture objects.
                            // Kudos to Jason B for figuring this one out.

                            osg::Texture* texClone = osg::clone(tex, osg::CopyOp::SHALLOW_COPY);
                            if ( texClone )
                            {
                                for (unsigned k = 0; k < texClone->getNumImages(); ++k)
                                {
                                    osg::Image* image = texClone->getImage(k);
                                    if ( image )
                                    {
                                        applyUserData(*image);
                                    }
                                }

                                applyUserData(*texClone);

                                j->second.first = texClone;
                            }
                            else
                            {
                                OE_WARN << LC << "Texture clone failed.\n";
                            }
                        }
                        else
                        {
                            applyUserData(*sa);
                        }
                    }
                }
            }

            applyUserData(*ss);
        }

        void applyUserData(osg::Object& object)
        {
            if (object.getUserData())
            {
                _userDataClears++;
            }
            object.setUserDataContainer(0L);
        }
    };

    
#undef  LC
#define LC "[WriteImagesToCache] "
        
    /**
     * Traverses a graph, located externally referenced images, and writes
     * them to the cache using a unique cache key. Then this will change the
     * image's FileName to point at the cached image instead of the original
     * source. The caches image key includes the .osgearth_cachebin extension,
     * which will invoke a pseudoloader that redirects the read to the cache bin.
     *
     * When you later go to read from the cache, the CacheBin must
     * be in the osgDB::Options used to invoke the read.
     */
    struct WriteExternalReferencesToCache : public osgEarth::TextureAndImageVisitor
    {
        CacheBin*               _bin;
        const osgDB::Options*   _writeOptions;
        static Threading::Mutex _globalMutex;

        // constructor
        WriteExternalReferencesToCache(CacheBin* bin, const osgDB::Options* writeOptions)
            : TextureAndImageVisitor(), _bin(bin), _writeOptions(writeOptions)
        {
            setTraversalMode( TRAVERSE_ALL_CHILDREN );
            setNodeMaskOverride( ~0L );
        }

        void apply(osg::Texture& tex)
        {
            if (dynamic_cast<osg::TextureBuffer*>(&tex) != 0L)
            {
                // skip texture buffers, they need no prep and 
                // will be inlined as long as they have a write hint
                // set to STORE_INLINE.
            }
            else
            {
                osgEarth::TextureAndImageVisitor::apply(tex);
            }
        }

        void apply(osg::Image& image)
        {
            std::string path = image.getFileName();
            if (path.empty())
            {
                OE_WARN << LC << "ERROR image with blank filename.\n";
            }

            if (!osgEarth::endsWith(path, ".osgearth_cachebin"))
            {
                // take a plugin-global mutex to avoid two threads altering the image
                // at the same time
                Threading::ScopedMutexLock lock(_globalMutex);

                if (!osgEarth::endsWith(path, ".osgearth_cachebin"))
                {
                    // get the hashed key that the cache bin will use to actually write the image,
                    // and replace the image filename with it.
                    std::string cacheKey = path;
                    std::string hashKey = _bin->getHashedKey(cacheKey);

                    // Append the pseudoloader suffix so our PL can locate the image in the cache.
                    image.setFileName(cacheKey + ".osgearth_cachebin");
                    image.setWriteHint(osg::Image::EXTERNAL_FILE);

                    // If an object with the same key is already cached, skip it.
                    CacheBin::RecordStatus rs = _bin->getRecordStatus(cacheKey);
                    if (rs != CacheBin::STATUS_OK)
                    {
                        // The OSGB serializer won't actually write the image data without this:
                        osg::ref_ptr<osgDB::Options> dbo = Registry::cloneOrCreateOptions(_writeOptions);
                        dbo->setPluginStringData("WriteImageHint", "IncludeData");

                        OE_INFO << LC << "Writing image \"" << image.getFileName() << "\" to the cache\n";

                        if (!_bin->write(cacheKey, &image, dbo.get()))
                        {
                            OE_WARN << LC << "...error, write failed!\n";
                        }
                    }
                    else
                    {
                        //OE_INFO << LC << "..Image \"" << path << "\" already cached\n";
                    }
                }
            }
        }
    };

    
    Threading::Mutex WriteExternalReferencesToCache::_globalMutex;
}


bool
CacheBin::writeNode(const std::string&    key,
                    osg::Node*            node,
                    const Config&         metadata,
                    const osgDB::Options* writeOptions)
{
    // Preparation step - removes things like UserDataContainers
    PrepareForCaching prep;
    node->accept( prep );

    // Write external refs (like texture images) to the cache bin
    WriteExternalReferencesToCache writeRefs(this, writeOptions);
    node->accept( writeRefs );

    // finally, write the graph to the bin:
    write(key, node, metadata, writeOptions);

    return true;
}


#undef  LC
#define LC "[ReadImageFromCachePseudoLoader] "

namespace
{
    /**
     * Pseudoloader that looks for anything with an "osgearth_cachebin" extension
     * and tries to load it from a CacheBin stored in the Options. This is useful
     * when caching nodes that reference external texture images that are also
     * stored in the cache bin.
     *
     * For this to work, you must change the image filenames in your graph so that
     * they are in the form "cachekey.osgearth_cachebin". Then the pseudoloader will
     * intercept the load and load them from the cache. Obviously this requires that
     * you write both the images and the graph to the same cachebin during the
     * same operation.
     */
    struct osgEarthReadImageFromCachePseudoLoader : public osgDB::ReaderWriter
    {
        osgEarthReadImageFromCachePseudoLoader()
        {
            this->supportsExtension("osgearth_cachebin", "osgEarth CacheBin Pseudoloader");
        }

        ReadResult readObject(const std::string& url, const osgDB::Options* readOptions) const
        {
            if (osgDB::getLowerCaseFileExtension(url) != "osgearth_cachebin")
                return ReadResult::FILE_NOT_HANDLED;

            CacheSettings* cacheSettings = CacheSettings::get(readOptions);
            if (!cacheSettings || !cacheSettings->isCacheEnabled() || !cacheSettings->getCacheBin())
            {
                return ReadResult::FILE_NOT_FOUND;
            }

            std::string key = osgDB::getNameLessExtension(url);
            
            OE_DEBUG << LC << "Reading \"" << key << "\"\n";

            osgEarth::ReadResult rr = cacheSettings->getCacheBin()->readObject(key, readOptions);
            
            return rr.succeeded() ?
                ReadResult(rr.getObject()) :
                ReadResult::FILE_NOT_FOUND;
        }

        ReadResult readImage(const std::string& url, const osgDB::Options* readOptions) const
        {
            if (osgDB::getLowerCaseFileExtension(url) != "osgearth_cachebin")
                return ReadResult::FILE_NOT_HANDLED;
            
            CacheSettings* cacheSettings = CacheSettings::get(readOptions);
            if (!cacheSettings || !cacheSettings->isCacheEnabled() || !cacheSettings->getCacheBin())
            {
                OE_DEBUG << LC << "Cache not enabled...!\n";
                return ReadResult::FILE_NOT_FOUND;
            }

            std::string key = osgDB::getNameLessExtension(url);
            
            OE_DEBUG << LC << "Reading \"" << key << "\"\n";

            osgEarth::ReadResult rr = cacheSettings->getCacheBin()->readImage(key, readOptions);
            
            return rr.succeeded() ?
                ReadResult(rr.getImage()) :
                ReadResult::FILE_NOT_FOUND;
        }
    };

    REGISTER_OSGPLUGIN(osgearth_cachebin, osgEarthReadImageFromCachePseudoLoader);

}
