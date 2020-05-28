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
#include "TextureArena"
#include <osg/State>

#ifndef GL_TEXTURE_SPARSE_ARB
#define GL_TEXTURE_SPARSE_ARB 0x91A6
#endif

#ifndef GL_VIRTUAL_PAGE_SIZE_INDEX_ARB
#define GL_VIRTUAL_PAGE_SIZE_INDEX_ARB 0x91A7
#endif

#define GL_NUM_VIRTUAL_PAGE_SIZES_ARB 0x91A8
#define GL_VIRTUAL_PAGE_SIZE_X_ARB 0x9195
#define GL_VIRTUAL_PAGE_SIZE_Y_ARB 0x9196 
#define GL_VIRTUAL_PAGE_SIZE_Z_ARB 0x9197

using namespace osgEarth;


TextureArena::TextureArena() :
    _compiled(false)
{
    //todo
    setDataVariance(DYNAMIC);
}

TextureArena::~TextureArena()
{
    //todo
}

void
TextureArena::add(Texture* tex)
{
    if (!tex) return;

    if (tex->_image.valid() == false)
    {
        //TODO support read options for caching
        tex->_image = tex->_uri->getImage(NULL);
    }

    if (tex->_image.valid())
    {   
        _toAdd.push_back(tex);
    }

    //TODO: consider issues like multiple GCs and "unref after apply"
}

void
TextureArena::activate(Texture* tex)
{
    if (!tex) return;
    _toActivate.push_back(tex);

    //TODO: consider issues like multiple GCs and "unref after apply"
}

void
TextureArena::deactivate(Texture* tex)
{
    if (!tex) return;
    _toDeactivate.push_back(tex);

    //TODO: consider issues like multiple GCs and "unref after apply"
}

#if 0
TextureArena::Locator
TextureArena::add(osg::Image* image)
{
    //TODO: pick a set based on the image's size, format, and data type
    Layout layout;
    layout._s = image->s();
    layout._t = image->t();
    layout._format = image->getPixelFormat();
    layout._type = image->getDataType();

    TextureSet* set = NULL;

    for(auto i = _layouts.begin(); i != _layouts.end(); ++i)
    {
        if (*i == layout)
        {
            set = _sets[&(*i)].get();
            break;
        }
    }

    if (set == NULL)
    {
        _layouts.push_back(layout);
        osg::ref_ptr<TextureSet>& newSet = _sets[&_layouts.back()];
        newSet = new TextureSet(layout);
        set = newSet.get();
    }

    return set->add(image);
}

void
TextureArena::remove(const Locator& id)
{
    if (id._valid == false)
        return;

    //TODO - proper set.
    TextureSet* set = _sets.begin()->second.get();

    set->remove(id);
}
#endif

void
TextureArena::allocate(Texture* tex, osg::State& state) const
{
    osg::GLExtensions* ext = state.get<osg::GLExtensions>();

    //TODO: move all this to GLTexture
    glGenTextures(1, &tex->_gltexture.mutable_value());

    //TODO: deal with potentially large numbers of these things
    //state.getGraphicsContext()->add(new GLTextureReleaser(tex->_object.get()));

    // Blit our image to the GPU
    glBindTexture(
        GL_TEXTURE_2D,
        tex->_gltexture.get());

    ext->glTexStorage2D(
        GL_TEXTURE_2D,
        tex->_image->getNumMipmapLevels(),
        GL_RGBA8, //tex->_image->getInternalTextureFormat(),
        tex->_image->s(),
        tex->_image->t());

    // Create the bindless handle
    tex->_glhandle = ext->glGetTextureHandle(tex->_gltexture.get());

    // At this point, if we go with SPARSE textures, don't actually
    // copy the image down until activation.

    glTexSubImage2D(
        GL_TEXTURE_2D,
        0, // mip level
        0, 0, // xofffset, yoffset
        tex->_image->s(), tex->_image->t(), // width, height
        tex->_image->getPixelFormat(),
        tex->_image->getDataType(),
        tex->_image->data());
}

void
TextureArena::apply(osg::State& state) const
{
    //TODO: support multiple contexts
    osg::GLExtensions* ext = state.get<osg::GLExtensions>();

    // remove pending objects by swapping them out of memory
    for(auto i = _toDeactivate.begin(); i != _toDeactivate.end(); ++i)
    {
        Texture* tex = i->get();
        if (tex->_glhandle.isSet())
        {
            ext->glMakeTextureHandleNonResident(tex->_glhandle.get());
            tex->_resident = false;

            //TODO: remove a sparse tex from memory forcably by calling
            // i.e. call glTexPageCommitment(...,GL_FALSE) here...? Consider it
        }
    }
    _toDeactivate.clear();

    // add pending textures by swapping them in to memory
    for(auto i = _toActivate.begin(); i != _toActivate.end(); ++i)
    {
        Texture* tex = i->get();
        if (tex->_glhandle.isSet())
        {
            ext->glMakeTextureHandleResident(tex->_glhandle.get());
            tex->_resident = true;
        }

        //TODO: Consider making the texture SPARSE as well so we can
        // control the actual texture residency along with the handle residency.
        // i.e. call glTexPageCommitment(...,GL_TRUE) here
    }
    _toActivate.clear();

    // TODO: update the SSBO (?) with handle information...or don't, maybe
    // all the handles are always just there whether they are in use or not.
}

void
TextureArena::compileGLObjects(osg::State& state) const
{
    // allocate textures and resident handles
    for(auto i = _toAdd.begin(); i != _toAdd.end(); ++i)
    {
        Texture* tex = i->get();
        if (tex->_gltexture.isSet() == false)
        {
            allocate(tex, state);
        }

        //TODO: Consider making the texture SPARSE as well so we can
        // control the actual texture residency along with the handle residency.
        // i.e. call glTexPageCommitment(...,GL_TRUE) here
    }
    _toAdd.clear();
    _compiled = true;

    apply(state);
}

void
TextureArena::resizeGLObjectBuffers(unsigned maxSize)
{
    // TODO
}

void
TextureArena::releaseGLObjects(osg::State* state) const
{
    // TODO
}

#if 0
TextureArena::TextureSet::TextureSet(const Layout& layout) :
    _layout(layout),
    _target(GL_TEXTURE_2D_ARRAY),
    _internalFormat(GL_RGBA8),
    _miplevels(1),
    _capacity(1),
    _maxSize(0u)
{
    //nop
}

TextureArena::Locator
TextureArena::TextureSet::add(osg::Image* image)
{
    ToAdd toAdd;
    toAdd._image = image;

    allocate(toAdd._locator);
    _toAdds.push_back(toAdd);

    return toAdd._locator;
}

void
TextureArena::TextureSet::remove(const Locator& locator)
{
    _freelist.push(locator);
    _toRemoves.push_back(locator);
}

void
TextureArena::TextureSet::allocate(Locator& locator)
{
    if (_freelist.size() > 0)
    {
        locator = _freelist.front();
        _freelist.pop();
    }
    else
    {
        locator._width = _layout._s;
        locator._height = _layout._t;
        locator._depth = 1;
        locator._zoffset = _maxSize++;
    }
    locator._valid = true;
}

void
TextureArena::TextureSet::apply(osg::State& state) const
{
    osg::GLExtensions* ext = state.get<osg::GLExtensions>();
    ext->glActiveTexture(GL_TEXTURE0);

    osg::ref_ptr<GCData>& cd = _cxdata[state.getContextID()];
    if (!cd.valid())
    {
        // allocate a new sparse texture array
        cd = new GCData();

        glGenTextures(1, &cd->_tex->_handle);
        glBindTexture(_target, cd->_tex->_handle);
        glTexParameteri(_target, GL_TEXTURE_SPARSE_ARB, GL_TRUE);

        state.getGraphicsContext()->add(new GLTextureReleaser(cd->_tex.get()));

#if 0
        // TODO: This could be done once per internal format. For now, just do it every time.
        GLint indexCount = 0, xSize = 0, ySize = 0, zSize = 0;
        GLint bestIndex = -1, bestXSize = 0, bestYSize = 0;

        void (GL_APIENTRY * glGetInternalformativ)(GLenum, GLenum, GLenum, GLsizei, GLint*);
        osg::setGLExtensionFuncPtr(glGetInternalformativ, "glGetInternalformativ", "glGetInternalformativARB");

        glGetInternalformativ(_target, _internalFormat, GL_NUM_VIRTUAL_PAGE_SIZES_ARB, 1, &indexCount);

        for (GLint i = 0; i < indexCount; ++i) {
            glTexParameteri(_target, GL_VIRTUAL_PAGE_SIZE_INDEX_ARB, i);
            glGetInternalformativ(_target, _internalFormat, GL_VIRTUAL_PAGE_SIZE_X_ARB, 1, &xSize);
            glGetInternalformativ(_target, _internalFormat, GL_VIRTUAL_PAGE_SIZE_Y_ARB, 1, &ySize);
            glGetInternalformativ(_target, _internalFormat, GL_VIRTUAL_PAGE_SIZE_Z_ARB, 1, &zSize);
            
            // For our purposes, the "best" format is the one that winds up with Z=1 and the largest x and y sizes.
            if (zSize == 1) {
                if (xSize >= bestXSize && ySize >= bestYSize) {
                    bestIndex = i;
                    bestXSize = xSize;
                    bestYSize = ySize;
                }
            }
        }
        glTexParameteri(_target, GL_VIRTUAL_PAGE_SIZE_INDEX_ARB, bestIndex);
#endif

        ext->glTexStorage3D(_target, _miplevels, _internalFormat, _layout._s, _layout._t, _capacity);
    }
    else
    {
        glBindTexture(_target, cd->_tex->_handle);
    }

    for(auto i = _toRemoves.begin(); i != _toRemoves.end(); ++i)
    {
        // remove stuff and add to freelist
        const Locator& locator = *i;

        ext->glTexPageCommitment(
            _target, 
            0, // miplevel
            locator._xoffset, locator._yoffset, locator._zoffset,
            locator._width, locator._height, locator._depth,
            GL_FALSE);
    }

    //TODO: replace this
    _toRemoves.clear();

    // Iterator over the "add queue" and make pending images resident:
    for(auto i = _toAdds.begin(); i != _toAdds.end(); ++i)
    {
        const ToAdd& toAdd = *i;
        const Locator& locator = toAdd._locator;

        // commit backing store to GPU:
        ext->glTexPageCommitment(
            _target, 
            0, // miplevel
            locator._xoffset, locator._yoffset, locator._zoffset,
            locator._width, locator._height, locator._depth,
            GL_TRUE);

        // blit image data to GPU at newly committed location:
        ext->glTexSubImage3D(
            _target,
            0, // miplevel,
            locator._xoffset, locator._yoffset, locator._zoffset,
            locator._width, locator._height, locator._depth,
            _layout._format,
            _layout._type,
            toAdd._image->data());
    }

    //TODO: replace this
    _toAdds.clear();
}

void
TextureArena::TextureSet::resizeGLObjectBuffers(unsigned maxsize)
{
    _cxdata.resize(maxsize);
}

void
TextureArena::TextureSet::releaseGLObjects(osg::State* state) const
{
#if 1
    // need this? can just use the deleter?
    if (state)
    {
        osg::ref_ptr<GCData>& cd = _cxdata[state->getContextID()];
        if (cd.valid())
        {
            cd->_tex->_handle = (GLuint)~0;
        }
    }
    else
    {
        //??
    }
#endif
}

TextureArena::GCData::GCData() :
    _tex(new GLTexture())
{
    //nop
}

TextureArena::Locator::Locator() :
    _valid(false),
    _set(0),
    _xoffset(0),
    _yoffset(0),
    _zoffset(0),
    _width(0),
    _height(0),
    _depth(1)
{
    //nop
}

bool
TextureArena::Layout::operator==(const TextureArena::Layout& rhs) const
{
    return 
        _s == rhs._s &&
        _t == rhs._t &&
        _format == rhs._format &&
        _type == rhs._type;
}
#endif