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
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#include <osgEarth/TextureBuffer>
#include <osgDB/ObjectWrapper>
#include <osgDB/InputStream>
#include <osgDB/OutputStream>

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

