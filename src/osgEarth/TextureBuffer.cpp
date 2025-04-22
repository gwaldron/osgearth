/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/

#include <osgEarth/TextureBuffer>
#include <osgDB/ObjectWrapper>

using namespace osgEarth;

TextureBuffer::TextureBuffer() :
    osg::TextureBuffer()
{
}

TextureBuffer::TextureBuffer(const osgEarth::TextureBuffer& rhs, const osg::CopyOp& copyop) :
    osg::TextureBuffer(rhs, copyop)
{
}

REGISTER_OBJECT_WRAPPER(TextureBuffer,
    new osgEarth::TextureBuffer,
    osgEarth::TextureBuffer,
    // Don't include osg::TextureBuffer in the associates list b/c it's serializer doesn't properly work
    "osg::Object osg::StateAttribute osg::Texture osgEarth::TextureBuffer")
{
    ADD_IMAGE_SERIALIZER(Image, osg::Image, NULL);  // _image
    ADD_INT_SERIALIZER(TextureWidth, 0);  // _textureWidth
}

