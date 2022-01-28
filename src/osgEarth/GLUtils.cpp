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
#include <osgEarth/StringUtils>
#include <osgEarth/Math>
#include <osgEarth/Utils>
#include <osgEarth/Capabilities>
#include <osgEarth/Registry>

#include <osg/LineStipple>
#include <osg/GraphicsContext>
#include <osgUtil/IncrementalCompileOperation>
#include <osgViewer/GraphicsWindow>
#include <osg/Texture2D>
#include <osg/BindImageTexture>

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

#ifndef GL_BUFFER_GPU_ADDRESS_NV
#define GL_BUFFER_GPU_ADDRESS_NV 0x8F1D
#endif

namespace
{
    struct
    {
        typedef void (GL_APIENTRY* DebugProc)(GLenum, GLenum, GLuint, GLenum, GLsizei, const GLchar*, const void*);
        
        void (GL_APIENTRY * DebugMessageCallback)(DebugProc, const void*);
        void (GL_APIENTRY * DebugMessageControl)(GLenum, GLenum, GLenum, GLsizei, const GLuint*, bool);
        void (GL_APIENTRY * PushDebugGroup)(GLenum, GLuint, GLsizei, const char*);
        void (GL_APIENTRY * PopDebugGroup)(void);

        // NV_shader_buffer_load
        // https://developer.download.nvidia.com/opengl/specs/GL_NV_shader_buffer_load.txt
        void (GL_APIENTRY * MakeNamedBufferResidentNV)(GLuint name, GLenum access);
        void (GL_APIENTRY * MakeNamedBufferNonResidentNV)(GLuint name);
        void (GL_APIENTRY * GetNamedBufferParameterui64vNV)(GLenum name, GLenum pname, GLuint64* params);

        void (GL_APIENTRY * MakeBufferResidentNV)(GLuint name, GLenum access);
        void (GL_APIENTRY * MakeBufferNonResidentNV)(GLuint name);
        void (GL_APIENTRY * GetBufferParameterui64vNV)(GLenum target, GLenum pname, GLuint64* params);

        void (GL_APIENTRY * NamedBufferData)(GLuint name, GLsizeiptr size, const void* data, GLenum usage);
        void (GL_APIENTRY * NamedBufferSubData)(GLuint name, GLintptr offset ,GLsizeiptr size, const void* data);
        void*(GL_APIENTRY * MapNamedBuffer)(GLuint name, GLbitfield access);
        void*(GL_APIENTRY * MapNamedBufferRange)(GLuint name, GLintptr offset, GLsizeiptr length, GLbitfield access);
        void (GL_APIENTRY * UnmapNamedBuffer)(GLuint name);

        inline void init()
        {
            if (DebugMessageCallback == nullptr)
            {
                //Threading::setThreadName("GL");

                float version = osg::getGLVersionNumber();

                osg::setGLExtensionFuncPtr(DebugMessageCallback, "glDebugMessageCallback", "glDebugMessageCallbackKHR");
                osg::setGLExtensionFuncPtr(DebugMessageControl, "glDebugMessageControl", "glDebugMessageControlKHR");
                osg::setGLExtensionFuncPtr(PushDebugGroup, "glPushDebugGroup", "glPushDebugGroupKHR");
                osg::setGLExtensionFuncPtr(PopDebugGroup, "glPopDebugGroup", "glPopDebugGroupKHR");

                osg::setGLExtensionFuncPtr(MakeNamedBufferResidentNV, "glMakeNamedBufferResidentNV");
                osg::setGLExtensionFuncPtr(MakeNamedBufferNonResidentNV, "glMakeNamedBufferNonResidentNV");
                osg::setGLExtensionFuncPtr(GetNamedBufferParameterui64vNV, "glGetNamedBufferParameterui64vNV");

                osg::setGLExtensionFuncPtr(MakeBufferResidentNV, "glMakeBufferResidentNV");
                osg::setGLExtensionFuncPtr(MakeBufferNonResidentNV, "glMakeBufferNonResidentNV");
                osg::setGLExtensionFuncPtr(GetBufferParameterui64vNV, "glGetBufferParameterui64vNV");

                if (version >= 4.5f)
                {
                    osg::setGLExtensionFuncPtr(NamedBufferData, "glNamedBufferData");
                    osg::setGLExtensionFuncPtr(NamedBufferSubData, "glNamedBufferSubData");
                    osg::setGLExtensionFuncPtr(MapNamedBuffer, "glMapNamedBuffer");
                    osg::setGLExtensionFuncPtr(MapNamedBufferRange, "glMapNamedBufferRange");
                    osg::setGLExtensionFuncPtr(UnmapNamedBuffer, "glUnmapNamedBuffer");
                }
            }
        }
    } gl;
}

// static
bool GLUtils::_gldebugging = false;

void
GLUtils::enableGLDebugging()
{
    _gldebugging = true;
}

void
GLUtils::pushDebugGroup(const char* name)
{
    gl.init();
    gl.PushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 0, -1, name);
}

void
GLUtils::popDebugGroup()
{
    gl.init();
    gl.PopDebugGroup();
}

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
CustomRealizeOperation::setSyncToVBlank(bool value)
{
    _vsync = value;
}

namespace
{
#ifndef GL_DEBUG_OUTPUT_SYNCHRONOUS
#define GL_DEBUG_OUTPUT_SYNCHRONOUS       0x8242
#define GL_DEBUG_NEXT_LOGGED_MESSAGE_LENGTH 0x8243
#define GL_DEBUG_CALLBACK_FUNCTION        0x8244
#define GL_DEBUG_CALLBACK_USER_PARAM      0x8245
#define GL_DEBUG_SOURCE_API               0x8246
#define GL_DEBUG_SOURCE_WINDOW_SYSTEM     0x8247
#define GL_DEBUG_SOURCE_SHADER_COMPILER   0x8248
#define GL_DEBUG_SOURCE_THIRD_PARTY       0x8249
#define GL_DEBUG_SOURCE_APPLICATION       0x824A
#define GL_DEBUG_SOURCE_OTHER             0x824B
#define GL_DEBUG_TYPE_ERROR               0x824C
#define GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR 0x824D
#define GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR  0x824E
#define GL_DEBUG_TYPE_PORTABILITY         0x824F
#define GL_DEBUG_TYPE_PERFORMANCE         0x8250
#define GL_DEBUG_TYPE_OTHER               0x8251
#define GL_MAX_DEBUG_MESSAGE_LENGTH       0x9143
#define GL_MAX_DEBUG_LOGGED_MESSAGES      0x9144
#define GL_DEBUG_LOGGED_MESSAGES          0x9145
#define GL_DEBUG_SEVERITY_HIGH            0x9146
#define GL_DEBUG_SEVERITY_MEDIUM          0x9147
#define GL_DEBUG_SEVERITY_LOW             0x9148
#define GL_DEBUG_TYPE_MARKER              0x8268
#define GL_DEBUG_TYPE_PUSH_GROUP          0x8269
#define GL_DEBUG_TYPE_POP_GROUP           0x826A
#define GL_DEBUG_SEVERITY_NOTIFICATION    0x826B
#define GL_MAX_DEBUG_GROUP_STACK_DEPTH    0x826C
#define GL_DEBUG_GROUP_STACK_DEPTH        0x826D
#endif

    void s_oe_gldebugproc(
        GLenum source,
        GLenum type,
        GLuint id,
        GLenum severity,
        GLsizei length,
        const GLchar* message,
        const void* userParam)
    {
        const std::string severities[3] = { "HIGH", "MEDIUM", "LOW" };

        if (severity != GL_DEBUG_SEVERITY_NOTIFICATION &&
            severity != GL_DEBUG_SEVERITY_LOW)
        {
            const std::string& s = severities[severity - GL_DEBUG_SEVERITY_HIGH];
            OE_WARN << "GL (" << s << ", " << source << ") -- " << message << std::endl;
        }
    }

    template<typename T>
    T getSSBOAlignment()
    {
        static GLsizei _ssboAlignment = -1;
        if (_ssboAlignment < 0)
            glGetIntegerv(GL_SHADER_STORAGE_BUFFER_OFFSET_ALIGNMENT, &_ssboAlignment);
        return T(_ssboAlignment);
    }
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

    if (GLUtils::isGLDebuggingEnabled())
    {
        gl.init();

        //osg::GraphicsContext* gc = static_cast<osg::GraphicsContext*>(object);
        //OE_HARD_ASSERT(gc != nullptr);

        if (gl.DebugMessageCallback)
        {
            OE_INFO << "ENABLING DEBUG GL MESSAGES" << std::endl;

            glEnable(GL_DEBUG_OUTPUT);
            glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);

            gl.DebugMessageCallback(s_oe_gldebugproc, nullptr);
            gl.DebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DEBUG_SEVERITY_MEDIUM, 0, nullptr, GL_TRUE);
            gl.DebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DEBUG_SEVERITY_HIGH, 0, nullptr, GL_TRUE);
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
#define LC "[GLObjectPool] "

GLObjectPool*
GLObjectPool::get(osg::State& state)
{
    return osg::get<GLObjectPool>(state.getContextID());
}

GLObjectPool::GLObjectPool(unsigned cxid) :
    osg::GraphicsObjectManager("osgEarth::GLObjectPool", cxid),
    _hits(0),
    _misses(0),
    _totalBytes(0),
    _avarice(0.5f)
{
    //nop
}

void
GLObjectPool::watch(GLObject::Ptr object)
{
    ScopedMutexLock lock(_mutex);
    _objects.insert(object);
}

void
GLObjectPool::releaseAll()
{
    ScopedMutexLock lock(_mutex);
    for (auto& object : _objects)
        object->release();
    _objects.clear();
}

GLsizeiptr
GLObjectPool::totalBytes() const
{
    return _totalBytes;
}

GLObjectPool::Collection
GLObjectPool::objects() const
{
    return _objects;
}

void
GLObjectPool::flushDeletedGLObjects(double now, double& avail)
{
    ScopedMutexLock lock(_mutex);

    GLsizeiptr bytes = 0;
    std::unordered_set<GLObject::Ptr> keep;
    unsigned maxNumToRelease = std::max(1u, _avarice);
    unsigned numReleased = 0u;

    for (auto& object : _objects)
    {
        if (object.use_count() == 1 && numReleased < maxNumToRelease)
        {
            object->release();
            ++numReleased;
        }
        else
        {
            keep.insert(object);
            bytes += object->size();
        }
    }
    _objects.swap(keep);
    _totalBytes = bytes;
}

void
GLObjectPool::flushAllDeletedGLObjects()
{
    deleteAllGLObjects();
}

void
GLObjectPool::deleteAllGLObjects()
{
    ScopedMutexLock lock(_mutex);
    for (auto& object : _objects)
        object->release();
    _objects.clear();
    _totalBytes = 0;
}

void
GLObjectPool::discardAllGLObjects()
{
    ScopedMutexLock lock(_mutex);
    _objects.clear();
    _totalBytes = 0;
}



GLObject::GLObject(osg::State& state, Type type, const std::string& label) :
    _label(label),
    _type(type),
    _recyclable(false),
    _ext(state.get<osg::GLExtensions>())
{
    gl.init();
}

GLVAO::GLVAO(osg::State& state, const std::string& label) :
    GLObject(state, VAO, label.empty() ? "Unlabaled VAO" : label),
    _name(0U)
{
    ext()->glGenVertexArrays(1, &_name);
}

GLVAO::Ptr
GLVAO::create(osg::State& state, const std::string& label)
{
    Ptr result(new GLVAO(state, label));
    GLObjectPool::get(state)->watch(result);
    return result;
}

void
GLVAO::release()
{
    if (_name != 0U)
        ext()->glDeleteVertexArrays(1, &_name);
    _name = 0U;
}

void
GLVAO::bind()
{
    OE_SOFT_ASSERT_AND_RETURN(_name != 0U, void());
    ext()->glBindVertexArray(_name);
}

void
GLVAO::unbind()
{
    ext()->glBindVertexArray(0);
}

GLBuffer::GLBuffer(GLenum target, osg::State& state, const std::string& label) :
    GLObject(state, BUFFER, label.empty() ? "Unlabeled buffer" : label),
    _target(target),
    _name(~0U),
    _size(0),
    _immutable(false),
    _address(0ULL),
    _isResident(false)
{
    ext()->glGenBuffers(1, &_name);
    reset(target, label);
}

void
GLBuffer::reset(GLenum target, const std::string& label)
{
    _target = target;
    _label = label;

    if (_name != ~0U)
    {
        bind();
        ext()->debugObjectLabel(GL_BUFFER, _name, label);
    }
}

GLBuffer::Ptr
GLBuffer::create(GLenum target, osg::State& state, const std::string& label)
{
    Ptr object(new GLBuffer(target, state, label));
    GLObjectPool::get(state)->watch(object);
    OE_DEVEL << LC << "GLBuffer::create, name=" << object->name() << std::endl;
    return object;
}

GLBuffer::Ptr
GLBuffer::create(GLenum target, osg::State& state, GLsizei sizeHint, const std::string& label)
{
    const GLObject::Compatible comp = [sizeHint](GLObject* obj) {
        return
            obj->type() == BUFFER &&
            obj->recyclable() &&
            obj->size() == sizeHint;
    };

    Ptr object = GLObjectPool::get(state)->recycle<GLBuffer>(comp);
    if (object)
    {
        object->reset(target, label);
        return object;
    }
    else
    {
        object = create(target, state, label);
        object->_recyclable = true;
    }
    return object;
}

void
GLBuffer::bind() const
{
    //OE_DEVEL << LC << "GLBuffer::bind, name=" << name() << std::endl;
    OE_SOFT_ASSERT_AND_RETURN(_name != ~0U, void(), "bind() called on invalid/deleted name: " + label() << );
    ext()->glBindBuffer(_target, _name);
}

void
GLBuffer::bind(GLenum otherTarget) const
{
    //OE_DEVEL << LC << "GLBuffer::bind, name=" << name() << std::endl;
    OE_SOFT_ASSERT_AND_RETURN(_name != ~0U, void(), "bind() called on invalid/deleted name: " + label() << );
    ext()->glBindBuffer(otherTarget, _name);
}

void
GLBuffer::unbind() const
{
    OE_SOFT_ASSERT_AND_RETURN(_name != ~0U, void(), "unbind() called on invalid/deleted name: " + label() << );
    ext()->glBindBuffer(_target, 0);
}

void
GLBuffer::uploadData(GLsizei datasize, const GLvoid* data, GLbitfield flags) const
{
    OE_SOFT_ASSERT_AND_RETURN(_immutable == false || datasize <= size(), void());

    if (!gl.NamedBufferData)
        bind(target());

    if (datasize > size())
        bufferData(datasize, data, flags);
    else if (data != nullptr)
    {
#if 1
        bufferSubData(0, datasize, data);
#else
        // CRASHES. Why?
        bind();
        void* ptr = ext()->glMapBuffer(_target, GL_WRITE_ONLY_ARB);
        OE_HARD_ASSERT(ptr != nullptr);
        ::memcpy(ptr, data, datasize);
        OE_HARD_ASSERT(ext()->glUnmapBuffer(_target) == GL_TRUE);
        unbind();
#endif
    }
}

void
GLBuffer::uploadData(GLenum otherTarget, GLsizei datasize, const GLvoid* data, GLbitfield flags) const
{
    OE_SOFT_ASSERT_AND_RETURN(_immutable == false || datasize <= size(), void());

    bind(otherTarget);

    if (datasize > size())
        bufferData(datasize, data, flags);
    else if (data != nullptr)
        bufferSubData(0, datasize, data);
}

void
GLBuffer::bufferData(GLsizei size, const GLvoid* data, GLbitfield flags) const
{
    if (_target == GL_SHADER_STORAGE_BUFFER)
        size = align(size, getSSBOAlignment<GLsizei>());
    if (gl.NamedBufferData)
        gl.NamedBufferData(name(), size, data, flags);
    else
        ext()->glBufferData(_target, size, data, flags);
    _size = size;
    _immutable = false;
}

void
GLBuffer::bufferSubData(GLintptr offset, GLsizei datasize, const GLvoid* data) const
{
    OE_SOFT_ASSERT_AND_RETURN(offset + datasize <= size(), void());
    if (gl.NamedBufferSubData)
        gl.NamedBufferSubData(name(), offset, datasize, data);
    else
        ext()->glBufferSubData(target(), offset, datasize, data);
}

void
GLBuffer::bufferStorage(GLsizei size, const GLvoid* data, GLbitfield flags) const
{
    if (_target == GL_SHADER_STORAGE_BUFFER)
        size = align(size, getSSBOAlignment<GLsizei>());

    if (recyclable() && size == _size)
    {
        ext()->glBufferSubData(_target, 0, size, data);
    }
    else
    {
        if (recyclable())
            flags |= GL_DYNAMIC_STORAGE_BIT;

        ext()->glBufferStorage(_target, size, data, flags);
    }
    _size = size;
    _immutable = true;
}

void*
GLBuffer::map(GLbitfield access) const
{
    if (gl.MapNamedBuffer)
        return gl.MapNamedBuffer(_name, access);
    else
    {
        bind();
        return ext()->glMapBuffer(_target, access);
    }
}

void
GLBuffer::unmap() const
{
    if (gl.UnmapNamedBuffer)
        gl.UnmapNamedBuffer(_name);
    else
    {
        bind();
        ext()->glUnmapBuffer(_target);
    }
}

void
GLBuffer::bindBufferBase(GLuint index) const
{
    if (target() == GL_SHADER_STORAGE_BUFFER ||
        target() == GL_UNIFORM_BUFFER ||
        target() == GL_ATOMIC_COUNTER_BUFFER)
    {
        ext()->glBindBufferBase(target(), index, name());
    }
    else
    {
        ext()->glBindBufferBase(GL_SHADER_STORAGE_BUFFER, index, name());
    }
}

void
GLBuffer::release()
{
    if (_name != ~0U)
    {
        OE_DEVEL << LC << "GLBuffer::release, name=" << name() << std::endl;

        makeNonResident();
        //OE_DEVEL << "Releasing buffer " << _name << "(" << _label << ")" << std::endl;
        ext()->glDeleteBuffers(1, &_name);
        _name = ~0U;
        _size = 0;
    }
}

GLuint64
GLBuffer::address()
{
    if (_address == 0ULL)
    {
        gl.init();
        OE_HARD_ASSERT(gl.GetNamedBufferParameterui64vNV);
        gl.GetNamedBufferParameterui64vNV(name(), GL_BUFFER_GPU_ADDRESS_NV, &_address);
    }
    return _address;
}

void
GLBuffer::makeResident()
{
    if (address() != 0ULL && !_isResident)
    {
        OE_HARD_ASSERT(gl.MakeNamedBufferResidentNV);
        //Currently only GL_READ_ONLY is supported according to the spec
        gl.MakeNamedBufferResidentNV(name(), GL_READ_ONLY_ARB);
        _isResident = true;
    }
}

void
GLBuffer::makeNonResident()
{
    if (_address != 0ULL && _isResident)
    {
        OE_HARD_ASSERT(gl.MakeNamedBufferNonResidentNV);
        gl.MakeNamedBufferNonResidentNV(name());
        _isResident = false;
        // address can be invalidated, so zero it out
        _address = 0ULL;
    }
}


GLTexture::Profile::Profile(GLenum target) :
    osg::Texture::TextureProfile(target),
    _minFilter(GL_LINEAR),
    _magFilter(GL_LINEAR),
    _wrapS(GL_CLAMP_TO_EDGE),
    _wrapT(GL_CLAMP_TO_EDGE),
    _wrapR(GL_CLAMP_TO_EDGE),
    _maxAnisotropy(1.0f)
{
    //nop
}

GLTexture::Profile::Profile(
    GLenum    target,
    GLint     numMipmapLevels,
    GLenum    internalFormat,
    GLsizei   width,
    GLsizei   height,
    GLsizei   depth,
    GLint     border,
    GLint     minFilter,
    GLint     magFilter,
    GLint     wrapS,
    GLint     wrapT,
    GLint     wrapR,
    GLfloat   maxAnisotropy) :
    osg::Texture::TextureProfile(
        target,
        numMipmapLevels,
        internalFormat,
        width, height, depth,
        border),
    _minFilter(minFilter),
    _magFilter(magFilter),
    _wrapS(wrapS),
    _wrapT(wrapT),
    _wrapR(wrapR),
    _maxAnisotropy(maxAnisotropy)
{
    //nop
}

bool
GLTexture::Profile::operator==(const GLTexture::Profile& rhs) const
{
    return
        osg::Texture::TextureProfile::operator==(rhs) &&
        _minFilter == rhs._minFilter &&
        _magFilter == rhs._magFilter &&
        _wrapS == rhs._wrapS &&
        _wrapT == rhs._wrapT &&
        _wrapR == rhs._wrapR &&
        _maxAnisotropy == rhs._maxAnisotropy;
}

GLTexture::GLTexture(GLenum target, osg::State& state, const std::string& label) :
    GLObject(state, TEXTURE, label.empty() ? "Unlabeled texture" : label),
    _target(target),
    _name(~0U),
    _handle(~0ULL),
    _isResident(false),
    _profile(target),
    _size(0)
{
    glGenTextures(1, &_name);
    reset(target, label, state);
}

void
GLTexture::reset(GLenum target, const std::string& label, osg::State& state)
{
    _target = target;
    _label = label;

    if (_name != ~0U)
    {
        bind(state);
        ext()->debugObjectLabel(GL_TEXTURE, _name, label);
    }
}

GLTexture::Ptr
GLTexture::create(GLenum target, osg::State& state, const std::string& label)
{
    Ptr obj(new GLTexture(target, state, label));
    GLObjectPool::get(state)->watch(obj);
    OE_DEVEL << LC << "GLTexture::create, name=" << obj->name() << std::endl;
    return obj;
}

GLTexture::Ptr
GLTexture::create(
    GLenum target, 
    osg::State& state, 
    const Profile& profileHint,
    const std::string& label)
{
    const GLObject::Compatible comp = [profileHint](GLObject* obj) {
        return
            obj->type() == TEXTURE &&
            obj->recyclable() &&
            static_cast<GLTexture*>(obj)->_profile == profileHint;
    };

    Ptr object = GLObjectPool::get(state)->recycle<GLTexture>(comp);
    if (object)
    {
        object->reset(target, label, state);
        return object;
    }
    else
    {
        object = create(target, state, label);
        object->_recyclable = true;
    }
    return object;
}

void
GLTexture::bind(osg::State& state)
{
    OE_DEVEL << LC << "GLTexture::bind, name=" << name() << std::endl;
    OE_SOFT_ASSERT_AND_RETURN(_name != ~0U, void(), "bind() called on invalid/deleted name: " + label() << );

    glBindTexture(_target, _name);

    // must be called with a compatible state
    // (same context under which the texture was created)
    OE_SOFT_ASSERT(state.getContextID() == _ext->contextID);

    // Inform OSG of the state change
    state.haveAppliedTextureAttribute(
        state.getActiveTextureUnit(), osg::StateAttribute::TEXTURE);
}

GLuint64
GLTexture::handle(osg::State& state)
{
    if (_handle == ~0ULL)
    {
        bind(state);
        _handle = ext()->glGetTextureHandle(_name);
    }

    OE_SOFT_ASSERT(_handle != ~0ULL, "glGetTextureHandle failed");
    return _handle;
}

void
GLTexture::makeResident(bool toggle)
{
    //TODO: does this stall??
    //if (toggle != ext()->glIsTextureHandleResident(_handle))
    if (_isResident != toggle)
    {
        OE_SOFT_ASSERT_AND_RETURN(_handle != ~0ULL, void(), "makeResident() called on invalid handle: " + label() << );

        if (toggle == true)
            ext()->glMakeTextureHandleResident(_handle);
        else
            ext()->glMakeTextureHandleNonResident(_handle);

        OE_DEVEL << "'" << id() << "' name=" << name() <<" resident=" << (toggle ? "yes" : "no") << std::endl;

        _isResident = toggle;
    }
}

void
GLTexture::release()
{
    OE_DEVEL << LC << "GLTexture::release, name=" << name() << std::endl;
    if (_handle != ~0ULL)
    {
        makeResident(false);
        _handle = ~0ULL;
    }
    if (_name != ~0U)
    {
        OE_DEVEL << "Releasing texture " << _name << "(" << _label << ")" << std::endl;
        glDeleteTextures(1, &_name);
        _name = ~0U;
    }
}

void
GLTexture::storage2D(const Profile& profile)
{
    if (size() == 0)
    {
        _profile = profile;

        ext()->glTexStorage2D(
            _target,
            profile._numMipmapLevels,
            profile._internalFormat,
            profile._width,
            profile._height);
        
        glTexParameteri(_target, GL_TEXTURE_MIN_FILTER, profile._minFilter);
        glTexParameteri(_target, GL_TEXTURE_MAG_FILTER, profile._magFilter);
        glTexParameteri(_target, GL_TEXTURE_WRAP_S, profile._wrapS);
        glTexParameteri(_target, GL_TEXTURE_WRAP_T, profile._wrapT);
        glTexParameterf(_target, GL_TEXTURE_MAX_ANISOTROPY_EXT, profile._maxAnisotropy);

        _size = _profile._size;
    }
}

void
GLTexture::storage3D(const Profile& profile)
{
    if (size() == 0)
    {
        _profile = profile; // Profile(_target, mipLevels, internalFormat, s, t, r, 0);

        ext()->glTexStorage3D(
            _target, 
            profile._numMipmapLevels, 
            profile._internalFormat,
            profile._width,
            profile._height,
            profile._depth);

        glTexParameteri(_target, GL_TEXTURE_MIN_FILTER, profile._minFilter);
        glTexParameteri(_target, GL_TEXTURE_MAG_FILTER, profile._magFilter);
        glTexParameteri(_target, GL_TEXTURE_WRAP_S, profile._wrapS);
        glTexParameteri(_target, GL_TEXTURE_WRAP_T, profile._wrapT);
        glTexParameterf(_target, GL_TEXTURE_MAX_ANISOTROPY_EXT, profile._maxAnisotropy);

        _size = _profile._size;
    }
}

void
GLTexture::subImage2D(GLint level, GLint xoff, GLint yoff, GLsizei width, GLsizei height, GLenum format, GLenum type, const void* pixels) const
{
    glTexSubImage2D(_target, level, xoff, yoff, width, height, format, type, pixels);
}

void
GLTexture::subImage3D(GLint level, GLint xoff, GLint yoff, GLint zoff, GLsizei width, GLsizei height, GLsizei depth, GLenum format, GLenum type, const void* pixels) const
{
    ext()->glTexSubImage3D(_target, level, xoff, yoff, zoff, width, height, depth, format, type, pixels);
}

void
GLTexture::compressedSubImage2D(GLint level, GLint xoff, GLint yoff, GLsizei width, GLsizei height, GLenum format, GLsizei imageSize, const void* data) const
{
    ext()->glCompressedTexSubImage2D(_target, level, xoff, yoff, width, height, format, imageSize, data);
}

void
GLTexture::compressedSubImage3D(GLint level, GLint xoff, GLint yoff, GLint zoff, GLsizei width, GLsizei height, GLsizei depth, GLenum format, GLsizei imageSize, const void* data) const
{
    ext()->glCompressedTexSubImage3D(_target, level, xoff, yoff, zoff, width, height, depth, format, imageSize, data);
}


GLFBO::Ptr
GLFBO::create(osg::State& state, const std::string& label)
{
    Ptr object(new GLFBO(state, label));
    GLObjectPool::get(state)->watch(object);
    return object;
}

GLFBO::GLFBO(osg::State& state, const std::string& label) :
    GLObject(state, FBO, label),
    _name(0)
{
    ext()->glGenFramebuffers(1, &_name);
}

void
GLFBO::release()
{
    if (_name != 0)
    {
        ext()->glDeleteFramebuffers(1, &_name);
        _name = 0;
    }
}

bool
GLFBO::valid() const
{
    return _name != 0;
}

GLsizei
GLFBO::size() const
{
    //todo
    return 0;
}

GLTexture::Ptr
GLFBO::renderToTexture(
    GLsizei width,
    GLsizei height,
    DrawFunction draw, 
    osg::State& state)
{
    // http://www.opengl-tutorial.org/intermediate-tutorials/tutorial-14-render-to-texture/

    OE_SOFT_ASSERT_AND_RETURN(width > 0 && height > 0, nullptr);
    OE_SOFT_ASSERT_AND_RETURN(draw != nullptr, nullptr);

    GLTexture::Profile profile(
        GL_TEXTURE_2D,    // target
        1,                // mip levels
        GL_RGBA8,         // internal format
        width,            // width
        height,           // height
        1,                // depth
        0,                // border,
        GL_NEAREST,       // minification filter
        GL_NEAREST,       // magnification filter
        GL_CLAMP_TO_EDGE, // wrap S
        GL_CLAMP_TO_EDGE, // wrap T
        GL_CLAMP_TO_EDGE, // wrap R
        4.0f);            // max anisotropy

    // create out RTT texture:
    GLTexture::Ptr texture = GLTexture::create(
        GL_TEXTURE_2D,
        state,
        profile,
        "RTT");

    // allocate the storage.
    // TODO: use glTexImage2D instead so we can change the 
    // mipmapping filters later?
    texture->storage2D(profile);

    // set up a depth buffer:
    GLuint depth_rb;
    ext()->glGenRenderbuffers(1, &depth_rb);
    ext()->glBindRenderbuffer(GL_RENDERBUFFER_EXT, depth_rb);
    ext()->glRenderbufferStorage(
        GL_RENDERBUFFER_EXT,
        GL_DEPTH_COMPONENT,
        width, height);

    // attach the texture
    ext()->glFramebufferTexture(
        GL_FRAMEBUFFER_EXT,
        GL_COLOR_ATTACHMENT0_EXT,
        texture->name(),
        0);

    // set the list of draw buffers.
    GLenum draw_buffers[1] = { GL_COLOR_ATTACHMENT0_EXT };
    ext()->glDrawBuffers(1, draw_buffers);

    // check for completeness:
    bool complete = ext()->glCheckFramebufferStatus(GL_FRAMEBUFFER_EXT) != GL_FRAMEBUFFER_COMPLETE_EXT;
    OE_SOFT_ASSERT(complete);

    if (complete)
    {
        ext()->glBindFramebuffer(GL_FRAMEBUFFER_EXT, _name);

        // save and reconfigure viewport:
        const osg::Viewport* viewport = state.getCurrentViewport();
        glViewport(0, 0, width, height);

        // Render to texture
        draw(state);

        // restore viewport
        viewport->apply(state);

        ext()->glBindFramebuffer(GL_FRAMEBUFFER_EXT, 0);

        // Re-apply previous OSG state
        //state.dirtyAllAttributes();
        //state.dirtyAllModes();
        //state.apply();
    }

    ext()->glDeleteRenderbuffers(1, &depth_rb);

    return texture;
}




#undef LC
#define LC "[GPUJobArena] "

//static defs
osg::ref_ptr<GPUJobArena> GPUJobArena::_arena_pool;
Mutex GPUJobArena::_arena_pool_mutex("GPUJobArena(OE)");

GPUJobArena&
GPUJobArena::arena()
{
    ScopedMutexLock lock(_arena_pool_mutex);

    if (_arena_pool.valid() == false)
    {
        _arena_pool = new GPUJobArena();
    }

    return *_arena_pool.get();
}

GPUJobArena::GPUJobArena() :
    osg::GraphicsOperation("oe.GPUJobArena", true),
    _timeSlice(0), // default time slice (milliseconds)
    _done(false),
    _queue_mutex("GPUJobArena(OE).queue")
{
    const char* value = ::getenv("OSGEARTH_GPU_TIME_SLICE_MS");
    if (value)
    {
        _timeSlice = std::chrono::milliseconds(clamp(atoi(value), 1, 1000));
    }
}

GPUJobArena::~GPUJobArena()
{
    setGraphicsContext(nullptr);
}

void
GPUJobArena::setGraphicsContext(osg::GraphicsContext* gc)
{
    if (gc != _gc.get() || gc == nullptr)
    {
        osg::ref_ptr<osg::GraphicsContext> old_gc(_gc);
        if (old_gc.valid())
        {
            old_gc->remove(this);
        }

        _gc = nullptr;

        if (gc)
        {
            _gc = gc;
            gc->add(this);
            OE_DEVEL << LC << getName() << " attached to GC " << std::hex << gc << std::dec << std::endl;
        }
    }
}

osg::ref_ptr<osg::GraphicsContext>
GPUJobArena::getGraphicsContext()
{
    return osg::ref_ptr<osg::GraphicsContext>(_gc);
}

void
GPUJobArena::setTimeSlice(const std::chrono::milliseconds& value)
{
    _timeSlice = value;
}

const std::chrono::milliseconds&
GPUJobArena::getTimeSlice() const
{
    return _timeSlice;
}

std::size_t
GPUJobArena::size() const
{
    std::lock_guard<Mutex> lock(_queue_mutex);
    return _queue.size();
}

void
GPUJobArena::dispatch(Delegate& del)
{
    std::lock_guard<Mutex> lock(_queue_mutex);
    _queue.emplace_back(del);
}

void
GPUJobArena::operator()(osg::GraphicsContext* gc)
{
    using Clock = std::chrono::high_resolution_clock;
    using Tick = std::chrono::time_point<Clock>;
    using ms = std::chrono::milliseconds;

    // always run at least one job.
    Clock::time_point start = Clock::now();
    while (!_done)
    {
        Delegate next;

        _queue_mutex.lock();
        if (!_queue.empty() && !_done)
        {
            next = std::move(_queue.front());
            _queue.pop_front();
        }
        _queue_mutex.unlock();

        if (next != nullptr)
        {
            // run the job
            next(gc->getState());

            auto timeElapsed = std::chrono::duration_cast<ms>(Clock::now() - start);
            if (timeElapsed >= _timeSlice)
            {
                break;
            }
        }
        else
        {
            break;
        }
    }
}

GPUJobArenaConnector::GPUJobArenaConnector() :
    osg::Drawable()
{
    // ensure it doesn't get culled out
    setCullingActive(false);

    // ensure the draw runs synchronously:
    setDataVariance(DYNAMIC);

    // force the draw to run every frame:
    setUseDisplayList(false);
}

GPUJobArenaConnector::~GPUJobArenaConnector()
{
    GPUJobArena& arena = GPUJobArena::arena();
    arena.setGraphicsContext(nullptr);
}

void
GPUJobArenaConnector::drawImplementation(osg::RenderInfo& ri) const
{
    GPUJobArena& arena = GPUJobArena::arena();

    if (arena.getGraphicsContext().valid() == false)
    {
        arena.setGraphicsContext(ri.getState()->getGraphicsContext());
    }
}

//........................................................................

ComputeImageSession::ComputeImageSession() :
    _stateSet(nullptr),
    _pbo(INT_MAX)
{
    _stateSet = new osg::StateSet();
    _tex = new osg::Texture2D();
    _tex->setFilter(_tex->MIN_FILTER, _tex->NEAREST);
    _tex->setFilter(_tex->MAG_FILTER, _tex->NEAREST);
    _stateSet->setTextureAttribute(0, _tex, 1);
    _stateSet->addUniform(new osg::Uniform("buf", 0));
}

void
ComputeImageSession::setProgram(osg::Program* program)
{
    _stateSet->setAttribute(program, 1);
}

void
ComputeImageSession::setImage(osg::Image* image)
{
    _image = image;
    _tex->setImage(image);
    _stateSet->setAttribute(new osg::BindImageTexture(
        0, _tex, osg::BindImageTexture::READ_WRITE, 
        image->getInternalTextureFormat(), 0, GL_TRUE));
    image->dirty();
}

void
ComputeImageSession::execute()
{
    auto render_job = GPUJob<bool>().dispatch(
        [this](osg::State* state, Cancelable* progress)
        {
            render(state);
            return true;
        }
    );

    auto readback_job = GPUJob<bool>().dispatch(
        [this](osg::State* state, Cancelable* progress)
        {
            readback(state);
            return true;
        }
    );

    render_job.join();
    readback_job.join();
}

void
ComputeImageSession::render(osg::State* state)
{
    OE_SOFT_ASSERT_AND_RETURN(_image.valid(), void());

    osg::GLExtensions* ext = state->get<osg::GLExtensions>();

    if (_stateSet.valid())
    {
        state->apply(_stateSet.get());
    }

    // BUG: need to resize if the new image is larger! (use GLBuffer)
    if (_pbo == INT_MAX)
    {
        int size = _image->getTotalSizeInBytes();
        ext->glGenBuffers(1, &_pbo);
        ext->glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, _pbo);
        ext->glBufferData(GL_PIXEL_PACK_BUFFER_ARB, size, 0, GL_STREAM_READ);
    }

    renderImplementation(state);

    // Post an async readback to the GL queue
    ext->glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, _pbo);
    glGetTexImage(GL_TEXTURE_2D, 0, _image->getPixelFormat(), _image->getDataType(), 0);
}

void
ComputeImageSession::readback(osg::State* state)
{
    osg::GLExtensions* ext = state->get<osg::GLExtensions>();

    ext->glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, _pbo);
    GLubyte* src = (GLubyte*)ext->glMapBuffer(GL_PIXEL_PACK_BUFFER_ARB, GL_READ_ONLY_ARB);
    if (src)
    {
        ::memcpy(_image->data(), src, _image->getTotalSizeInBytes());
        ext->glUnmapBuffer(GL_PIXEL_PACK_BUFFER_ARB);
    }
    ext->glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, 0);
}

//........................................................................

namespace
{
    using ICO = osgUtil::IncrementalCompileOperation;

    struct ICOCallback : public ICO::CompileCompletedCallback
    {
        Promise<osg::ref_ptr<osg::Node>> _promise;
        osg::ref_ptr<osg::Node> _node;
        std::atomic_int& _jobsActive;

        ICOCallback(const osg::ref_ptr<osg::Node>& node, std::atomic_int& jobsActive) :
            _node(node),
            _jobsActive(jobsActive) { }

        virtual ~ICOCallback() {
            _jobsActive--;
        }

        bool compileCompleted(ICO::CompileSet* compileSet) override
        {
            _promise.resolve(_node);
            return true;
        }
    };

    static osg::ref_ptr<osg::DummyObject> s_icoMarker = new osg::DummyObject();
}

std::atomic_int GLObjectsCompiler::_jobsActive;

osg::ref_ptr<osgUtil::StateToCompile>
GLObjectsCompiler::collectState(osg::Node* node) const
{
    // (note: COMPILE_DISPLAY_LISTS actually compiles Drawables)
    osg::ref_ptr<osgUtil::StateToCompile> state = new osgUtil::StateToCompile(
        osgUtil::GLObjectsVisitor::COMPILE_STATE_ATTRIBUTES |
        osgUtil::GLObjectsVisitor::COMPILE_DISPLAY_LISTS,
        s_icoMarker.get());

    if (node)
    {
        node->accept(*state.get());
    }

    return state;
}

Future<osg::ref_ptr<osg::Node>>
GLObjectsCompiler::compileAsync(
    const osg::ref_ptr<osg::Node>& node,
    const osg::Object* host,
    Cancelable* progress) const
{
    Future<osg::ref_ptr<osg::Node>> result;

    if (node.valid())
    {
        bool compileScheduled = false;

        osg::ref_ptr<ICO> ico;
        if (ObjectStorage::get(host, ico))
        {
            // (note: COMPILE_DISPLAY_LISTS actually compiles Drawables)
            osg::ref_ptr<osgUtil::StateToCompile> state = collectState(node.get());

            if (state->empty() == false)
            {
                auto compileSet = new osgUtil::IncrementalCompileOperation::CompileSet();
                compileSet->buildCompileMap(ico->getContextSet(), *state.get());
                ICOCallback* callback = new ICOCallback(node, _jobsActive);
                result = callback->_promise.getFuture();
                compileSet->_compileCompletedCallback = callback;
                _jobsActive++;
                ico->add(compileSet, false);
                compileScheduled = true;
            }
        }

        if (!compileScheduled)
        {
            // no ICO available - just resolve the future immediately
            Promise<osg::ref_ptr<osg::Node>> promise;
            result = promise.getFuture();
            promise.resolve(node);
        }
    }

    return result;
}

Future<osg::ref_ptr<osg::Node>>
GLObjectsCompiler::compileAsync(
    const osg::ref_ptr<osg::Node>& node,
    osgUtil::StateToCompile* state,
    const osg::Object* host,
    Cancelable* progress) const
{
    Future<osg::ref_ptr<osg::Node>> result;

    OE_SOFT_ASSERT_AND_RETURN(node.valid(), result);

    // if there is an ICO available, schedule the GPU compilation
    bool compileScheduled = false;
    if (state != nullptr && !state->empty())
    {
        osg::ref_ptr<ICO> ico;
        if (ObjectStorage::get(host, ico) && ico->isActive())
        {
            auto compileSet = new osgUtil::IncrementalCompileOperation::CompileSet();
            compileSet->buildCompileMap(ico->getContextSet(), *state);
            ICOCallback* callback = new ICOCallback(node, _jobsActive);
            result = callback->_promise.getFuture();
            compileSet->_compileCompletedCallback = callback;
            _jobsActive++;
            ico->add(compileSet, false);
            compileScheduled = true;
        }
    }

    if (!compileScheduled)
    {
        // no ICO available - just resolve the future immediately
        Promise<osg::ref_ptr<osg::Node>> promise;
        result = promise.getFuture();
        promise.resolve(node);
    }

    return result;
}


void
GLObjectsCompiler::compileNow(
    const osg::ref_ptr<osg::Node>& node,
    const osg::Object* host,
    Cancelable* progress) const
{
    if (node)
    {
        Future<osg::ref_ptr<osg::Node>> result = compileAsync(node, host, progress);
        result.join(progress);
    }
}
