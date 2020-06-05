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
#include <osgEarth/GLUtils>
#include <osgEarth/Lighting>

#include <osg/LineStipple>
#include <osg/GraphicsContext>
#include <osg/ContextData>
#include <osgViewer/GraphicsWindow>

#ifdef OSG_GL_FIXED_FUNCTION_AVAILABLE
#include <osg/LineWidth>
#include <osg/Point>
#endif

using namespace osgEarth;

#define LC "[GLUtils] "

#define OE_DEVEL OE_DEBUG

#ifndef GL_LINE_SMOOTH
#define GL_LINE_SMOOTH 0x0B20
#endif

#ifndef GL_POINT_SIZE
#define GL_POINT_SIZE 0x0B11
#endif

#ifndef GL_NORMALIZE
#define GL_NORMALIZE 0x0BA1
#endif

void
GLUtils::setGlobalDefaults(osg::StateSet* stateSet)
{
    // anything that uses a uniform.
    setLineWidth(stateSet, 1.0f, osg::StateAttribute::ON);
    setLineStipple(stateSet, 1, 0xffff, osg::StateAttribute::ON);
    setPointSize(stateSet, 1, osg::StateAttribute::ON);
}

void
GLUtils::setLighting(osg::StateSet* stateSet, osg::StateAttribute::OverrideValue ov)
{
#ifdef OSG_GL_FIXED_FUNCTION_AVAILABLE
    stateSet->setMode(GL_LIGHTING, ov);
#endif

    stateSet->setDefine(OE_LIGHTING_DEFINE, ov);
}

void
GLUtils::setLineWidth(osg::StateSet* stateSet, float value, osg::StateAttribute::OverrideValue ov)
{
#ifdef OSG_GL_FIXED_FUNCTION_AVAILABLE
    stateSet->setAttributeAndModes(new osg::LineWidth(value), ov);
#endif

    stateSet->addUniform(new osg::Uniform("oe_GL_LineWidth", value), ov);
}

void
GLUtils::setLineStipple(osg::StateSet* stateSet, int factor, unsigned short pattern, osg::StateAttribute::OverrideValue ov)
{
#ifdef OSG_GL_FIXED_FUNCTION_AVAILABLE
    stateSet->setAttributeAndModes(new osg::LineStipple(factor, pattern), ov);
#endif

    stateSet->addUniform(new osg::Uniform("oe_GL_LineStippleFactor", (int)factor), ov);
    stateSet->addUniform(new osg::Uniform("oe_GL_LineStipplePattern", (int)pattern), ov);
}

void
GLUtils::setLineSmooth(osg::StateSet* stateSet, osg::StateAttribute::OverrideValue ov)
{
#ifdef OSG_GL_FIXED_FUNCTION_AVAILABLE
    stateSet->setMode(GL_LINE_SMOOTH, ov);
#endif

    stateSet->setDefine("OE_LINE_SMOOTH", ov);
}

void
GLUtils::setPointSize(osg::StateSet* stateSet, float value, osg::StateAttribute::OverrideValue ov)
{
#ifdef OSG_GL_FIXED_FUNCTION_AVAILABLE
    stateSet->setAttributeAndModes(new osg::Point(value), ov);
#endif

    stateSet->addUniform(new osg::Uniform("oe_GL_PointSize", value), ov);
}

void
GLUtils::setPointSmooth(osg::StateSet* stateSet, osg::StateAttribute::OverrideValue ov)
{
#ifdef OSG_GL_FIXED_FUNCTION_AVAILABLE
    stateSet->setMode(GL_POINT_SMOOTH, ov);
#endif

    stateSet->setDefine("OE_POINT_SMOOTH", ov);
}

void
GLUtils::remove(osg::StateSet* stateSet, GLenum cap)
{
    if (!stateSet)
        return;

#ifdef OSG_GL_FIXED_FUNCTION_AVAILABLE
    switch(cap)
    {
    case GL_LIGHTING:
        stateSet->removeMode(GL_LIGHTING);
        break;

    case GL_LINE_WIDTH:
        stateSet->removeAttribute(osg::StateAttribute::LINEWIDTH);
        break;

    case GL_LINE_STIPPLE:
        stateSet->removeAttribute(osg::StateAttribute::LINESTIPPLE);
        break;

    case GL_LINE_SMOOTH:
        stateSet->removeMode(GL_LINE_SMOOTH);
        break;

    case GL_POINT_SIZE:
        stateSet->removeAttribute(osg::StateAttribute::POINT);
        break;

    default:
        stateSet->removeMode(cap);
        break;
    }
#endif

    switch(cap)
    {   
    case GL_LIGHTING:
        stateSet->removeDefine(OE_LIGHTING_DEFINE);
        break;

    case GL_LINE_WIDTH:
        stateSet->removeUniform("oe_GL_LineWidth");
        break;

    case GL_LINE_STIPPLE:
        stateSet->removeUniform("oe_GL_LineStippleFactor");
        stateSet->removeUniform("oe_GL_LineStipplePattern");
        break;

    case GL_LINE_SMOOTH:
        stateSet->removeDefine("OE_LINE_SMOOTH");
        break;

    case GL_POINT_SIZE:
        stateSet->removeUniform("oe_GL_PointSize");
        break;
    }
}


void
GLUtils::deleteGLBuffer(unsigned contextID, GLuint handle)
{
    osg::GLBufferObjectManager* bufferObjectManager = osg::get<osg::GLBufferObjectManager>(contextID);
    if (!bufferObjectManager)
    {
        OE_WARN << "Unable to delete GL object " << handle << " in context " << contextID << std::endl;
        return;
    }

    // create a temporary object for OSG to delete
    osg::ref_ptr<osg::GLBufferObject> glBufferObject = new osg::GLBufferObject(contextID, 0, handle);

    osg::GLBufferObjectSet* bufferObjectSet = bufferObjectManager->getGLBufferObjectSet(glBufferObject->getProfile());
    if (!bufferObjectSet)
    {
        OE_WARN << "Unable to delete GL object " << handle << " in context " << contextID << std::endl;
        return;
    }

    // queue the handle up for eventual (?) deletion by OSG.
    bufferObjectSet->orphan(glBufferObject.get());
}

void
CustomRealizeOperation::setSyncToVBlank(bool value)
{
    _vsync = value;
}

void
CustomRealizeOperation::operator()(osg::Object* object)
{
    if (_vsync.isSet())
    {
        osgViewer::GraphicsWindow* win = dynamic_cast<osgViewer::GraphicsWindow*>(object);
        if (win)
        {
            win->setSyncToVBlank(_vsync.get());
        }
    }
}

void
GL3RealizeOperation::operator()(osg::Object* object)
{
    osg::GraphicsContext* gc = dynamic_cast<osg::GraphicsContext*>(object);
    if (gc)
    {
        osg::State* state = gc->getState();

        // force NVIDIA-style vertex attribute aliasing, since osgEarth
        // makes use of some specific attribute registers. Later we can
        // perhaps create a reservation system for this.
        state->resetVertexAttributeAlias(false);

#ifdef OSG_GL3_AVAILABLE
        state->setUseModelViewAndProjectionUniforms(true);
        state->setUseVertexAttributeAliasing(true);
#endif

#ifndef OSG_GL_FIXED_FUNCTION_AVAILABLE
        state->setModeValidity(GL_LIGHTING, false);
        state->setModeValidity(GL_NORMALIZE, false);
        state->setModeValidity(GL_RESCALE_NORMAL, false);
        state->setModeValidity(GL_LINE_STIPPLE, false);
        state->setModeValidity(GL_LINE_SMOOTH, false);
#endif
    }

    CustomRealizeOperation::operator()(object);
}

#undef LC
#define LC "[GLObjectReleaser] "

GLObject::GLObject(osg::State& state, const std::string& label) :
    _label(label),
    _ext(state.get<osg::GLExtensions>())
{
    //nop
}

GLBuffer::GLBuffer(GLenum target, osg::State& state, const std::string& label) :
    GLObject(state, label),
    _target(target),
    _name(~0U)
{
    ext()->glGenBuffers(1, &_name);
    if (_name != ~0U)
    {
        bind();
        ext()->debugObjectLabel(GL_BUFFER, _name, label);
        osg::get<GLObjectReleaser>(ext()->contextID)->watch(this);
    }
}

void
GLBuffer::bind()
{
    ext()->glBindBuffer(_target, _name);
}

void
GLBuffer::bind(GLenum otherTarget)
{
    ext()->glBindBuffer(otherTarget, _name);
}

void
GLBuffer::release()
{
    if (_name != ~0U)
    {
        OE_DEVEL << "Releasing buffer " << _name << "(" << _label << ")" << std::endl;
        ext()->glDeleteBuffers(1, &_name);
        _name = ~0U;
    }
}

GLTexture::GLTexture(GLenum target, osg::State& state, const std::string& label) :
    GLObject(state, label),
    _target(target),
    _name(~0U),
    _handle(~0ULL),
    _isResident(false)
{
    glGenTextures(1, &_name);
    if (_name != ~0U)
    {
        bind();
        ext()->debugObjectLabel(GL_TEXTURE, _name, label);
        osg::get<GLObjectReleaser>(ext()->contextID)->watch(this);
        // cannot call glGetTextureHandle until all state it set.
    }
}

void
GLTexture::bind()
{
    glBindTexture(_target, _name);
}

GLuint64
GLTexture::handle()
{
    if (_handle == ~0ULL)
    {
        bind();
        _handle = ext()->glGetTextureHandle(_name);
    }
    return _handle;
}

void
GLTexture::makeResident(bool toggle)
{
    if (_isResident != toggle)
        ext()->glMakeTextureHandleResident(_handle);
    else
        ext()->glMakeTextureHandleNonResident(_handle);

    _isResident = toggle;
}

void
GLTexture::release()
{
    if (_handle != ~0ULL)
    {
        ext()->glMakeTextureHandleNonResident(_handle);
        _handle = ~0ULL;
    }
    if (_name != ~0U)
    {
        OE_DEVEL << "Releasing texture " << _name << "(" << _label << ")" << std::endl;
        glDeleteTextures(1, &_name);
        _name = ~0U;
    }
}


SSBO::SSBO() :
    _allocatedSize(0),
    _bindingIndex(-1),
    _glBufferStorage(NULL),
    _glClearBufferSubData(NULL)
{
    //nop
}

void
SSBO::initGL()
{
    if (_glBufferStorage == NULL)
    {
        // polyfill for pre-OSG 3.6 support
        osg::setGLExtensionFuncPtr(_glBufferStorage, "glBufferStorage", "glBufferStorageARB");
        osg::setGLExtensionFuncPtr(_glClearBufferSubData, "glClearBufferSubData", "glClearBufferSubDataARB");
    }
}

void
SSBO::release() const
{
    _buffer = NULL; // triggers the releaser
    _allocatedSize = 0u;
}

void
SSBO::bindLayout() const
{
    if (_buffer.valid() && _bindingIndex >= 0)
    {
        _buffer->ext()->glBindBufferBase(GL_SHADER_STORAGE_BUFFER, _bindingIndex, _buffer->name());
    }
}

#undef LC
#define LC "[GLObjectReleaser] "

GLObjectReleaser::GLObjectReleaser(unsigned contextID) :
    osg::GraphicsObjectManager("OE GLObjectReleaser", contextID)
{
    //nop
}

void
GLObjectReleaser::watch(GLObject* obj)
{
    _objects.insert(obj);
}

void 
GLObjectReleaser::flushDeletedGLObjects(double currentTime, double& availableTime)
{
    flushAllDeletedGLObjects();
}

void 
GLObjectReleaser::flushAllDeletedGLObjects()
{
    // keep all non-released objects in the temp container
    // so we can retain them for next time
    _temp.clear();
    for(auto& object : _objects)
    {
        if (object->referenceCount() == 1)
            object->release();
        else
            _temp.insert(object);
    }

    _objects.swap(_temp);
}

void 
GLObjectReleaser::deleteAllGLObjects()
{
    // not really sure what this is supposed to do TBH
    flushAllDeletedGLObjects();
}

void 
GLObjectReleaser::discardAllGLObjects()
{
    // no graphics context available..just empty the bucket
    _objects.clear();
}


#ifndef OE_HAVE_BINDIMAGETEXTURE
using namespace osg;

int BindImageTexture::compare(const osg::StateAttribute &sa) const
{
    COMPARE_StateAttribute_Types(BindImageTexture,sa)
        // Compare each parameter in turn against the rhs.
        COMPARE_StateAttribute_Parameter(_target)
        COMPARE_StateAttribute_Parameter(_imageunit)
        COMPARE_StateAttribute_Parameter(_access)
        COMPARE_StateAttribute_Parameter(_format)
        COMPARE_StateAttribute_Parameter(_layered)
        COMPARE_StateAttribute_Parameter(_level)
        return 0;
}

void BindImageTexture::apply(osg::State& state) const
{
    if(_target.valid())
    {
        unsigned int contextID = state.getContextID();
        osg::Texture::TextureObject *to = _target->getTextureObject( contextID );
        if( !to ) //|| _target->isDirty( contextID ))
        {
            // _target never been applied yet or is dirty
            state.applyTextureAttribute( state.getActiveTextureUnit(), _target.get());
            to = _target->getTextureObject( contextID );
        }

        state.get<osg::GLExtensions>()->glBindImageTexture(_imageunit, to->id(), _level, _layered, _layer, _access, _format);
    }
}

#endif

