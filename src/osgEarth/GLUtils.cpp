/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/
#include <osgEarth/GLUtils>
#include <osgEarth/Lighting>
#include <osgEarth/StringUtils>
#include <osgEarth/Math>
#include <osgEarth/Utils>
#include <osgEarth/Capabilities>
#include <osgEarth/Registry>
#include <osgEarth/Notify>

#include <osg/GraphicsContext>
#include <osgUtil/IncrementalCompileOperation>
#include <osgViewer/GraphicsWindow>
#include <osg/Texture2D>
#include <osg/BindImageTexture>
#include <osg/LineStipple>

#ifdef OSG_GL_FIXED_FUNCTION_AVAILABLE
#include <osg/LineWidth>
#include <osg/Point>
#endif

using namespace osgEarth;

#define LC "[GLUtils] "

#define USE_RECYCLING

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

#ifndef GL_FRAMEBUFFER
#define GL_FRAMEBUFFER 0x8D40
#endif

#ifndef GL_RENDERBUFFER
#define GL_RENDERBUFFER 0x8D41
#endif

#ifndef GL_BUFFER_GPU_ADDRESS_NV
#define GL_BUFFER_GPU_ADDRESS_NV 0x8F1D
#endif

#ifndef GL_TEXTURE_SWIZZLE_A
#define GL_TEXTURE_SWIZZLE_A 0x8E45
#endif

namespace
{
    struct
    {
        typedef void (GL_APIENTRY* DebugProc)(GLenum, GLenum, GLuint, GLenum, GLsizei, const GLchar*, const void*);

        void (GL_APIENTRY* DebugMessageCallback)(DebugProc, const void*);
        void (GL_APIENTRY* DebugMessageControl)(GLenum, GLenum, GLenum, GLsizei, const GLuint*, bool);
        void (GL_APIENTRY* PushDebugGroup)(GLenum, GLuint, GLsizei, const char*);
        void (GL_APIENTRY* PopDebugGroup)(void);

        // NV_shader_buffer_load
        // https://developer.download.nvidia.com/opengl/specs/GL_NV_shader_buffer_load.txt
        void (GL_APIENTRY* MakeNamedBufferResidentNV)(GLuint name, GLenum access);
        void (GL_APIENTRY* MakeNamedBufferNonResidentNV)(GLuint name);
        void (GL_APIENTRY* GetNamedBufferParameterui64vNV)(GLenum name, GLenum pname, GLuint64* params);

        void (GL_APIENTRY* MakeBufferResidentNV)(GLuint name, GLenum access);
        void (GL_APIENTRY* MakeBufferNonResidentNV)(GLuint name);
        void (GL_APIENTRY* GetBufferParameterui64vNV)(GLenum target, GLenum pname, GLuint64* params);

        void (GL_APIENTRY* NamedBufferData)(GLuint name, GLsizeiptr size, const void* data, GLenum usage);
        void (GL_APIENTRY* NamedBufferSubData)(GLuint name, GLintptr offset, GLsizeiptr size, const void* data);
        void* (GL_APIENTRY* MapNamedBuffer)(GLuint name, GLbitfield access);
        void* (GL_APIENTRY* MapNamedBufferRange)(GLuint name, GLintptr offset, GLsizeiptr length, GLbitfield access);
        void (GL_APIENTRY* UnmapNamedBuffer)(GLuint name);

        void (GL_APIENTRY* CopyBufferSubData)(GLenum readTarget, GLenum writeTarget, GLintptr readOffset, GLintptr writeOffset, GLsizei size);
        void (GL_APIENTRY* CopyNamedBufferSubData)(GLuint readName, GLuint writeName, GLintptr readOffset, GLintptr writeOffset, GLsizei size);
        void (GL_APIENTRY* GetNamedBufferSubData)(GLuint name, GLintptr offset, GLsizei size, void*);

        bool useNamedBuffers;

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

                osg::setGLExtensionFuncPtr(CopyBufferSubData, "glCopyBufferSubData");

#if 1
                if (version >= 4.5f)
                {
                    osg::setGLExtensionFuncPtr(NamedBufferData, "glNamedBufferData");
                    osg::setGLExtensionFuncPtr(NamedBufferSubData, "glNamedBufferSubData");
                    osg::setGLExtensionFuncPtr(MapNamedBuffer, "glMapNamedBuffer");
                    osg::setGLExtensionFuncPtr(MapNamedBufferRange, "glMapNamedBufferRange");
                    osg::setGLExtensionFuncPtr(UnmapNamedBuffer, "glUnmapNamedBuffer");
                    osg::setGLExtensionFuncPtr(CopyNamedBufferSubData, "glCopyNamedBufferSubData");
                    osg::setGLExtensionFuncPtr(GetNamedBufferSubData, "glGetNamedBufferSubData");
                }
#endif
                useNamedBuffers =
                    NamedBufferData &&
                    NamedBufferSubData;
            }
        }
    } gl;
}

// static
bool GLUtils::_gldebugging = false;
bool GLUtils::_useNVGL = false;

void
GLUtils::useNVGL(bool value)
{
    _useNVGL = value;

    if (_useNVGL)
    {
        OE_INFO << LC << "Using NVIDIA GL4 extensions" << std::endl;
    }
    else
    {
        OE_INFO << LC << "Disabling NVIDIA GL4 extensions" << std::endl;
    }
}

bool
GLUtils::useNVGL()
{
    return _useNVGL && Registry::capabilities().supportsNVGL();
}

namespace
{
    struct Mapping {
        const osg::State* _ptr = nullptr;
    };
    static Mapping s_mappings[4096];
}

unsigned
GLUtils::getUniqueStateID(const osg::State& state)
{
    // in theory this should never need a mutex..
    for (int i = 0; i < 4096; ++i)
    {
        if (s_mappings[i]._ptr == &state)
        {
            return i;
        }
        else if (s_mappings[i]._ptr == nullptr)
        {
            s_mappings[i]._ptr = &state;
            return i;
        }
    }
    return 0;
}

unsigned
GLUtils::getSharedContextID(const osg::State& state)
{
#ifdef OSGEARTH_SINGLE_GL_CONTEXT
    return 0;
#else
    return state.getContextID();
#endif
}

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
    switch (cap)
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

    switch (cap)
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

    void GL_APIENTRY s_oe_gldebugproc(
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

            std::stringstream buf;
            CallStack stack;
            for(unsigned i=1; i<stack.symbols.size(); ++i)
            {
                if (stack.symbols[i].find("s_oe_gldebugproc") == std::string::npos)
                    buf << "\n - " << stack.symbols[i];
                if (stack.symbols[i] == "main") break;
            }
            OE_WARN << "Call stack:" << buf.str() << std::endl;
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

    template<typename T>
    T getUBOAlignment()
    {
        static GLsizei _uboAlignment = -1;
        if (_uboAlignment < 0)
            glGetIntegerv(GL_UNIFORM_BUFFER_OFFSET_ALIGNMENT, &_uboAlignment);
        return T(_uboAlignment);
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

    for(auto& op : _moreOperations)
    {
        if (op.valid())
            (*op)(object);
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

        // We always want to use osg modelview and projection uniforms and vertex attribute aliasing.
        // Since we use modern opengl throughout even if OSG isn't explicitly built with GL3.
        state->setUseModelViewAndProjectionUniforms(true);
        state->setUseVertexAttributeAliasing(true);

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

// static decl
namespace
{
    std::mutex s_pools_mutex;
    std::vector<GLObjectPool*> s_pools;
}
bool GLObjectPool::_enableRecycling = true;
unsigned GLObjectPool::_bytes_to_delete_per_frame = 100000u;
unsigned GLObjectPool::_frames_to_delay_deletion = 30u;


GLObjectPool*
GLObjectPool::get(osg::State& state)
{
#ifdef OSGEARTH_SINGLE_GL_CONTEXT
    GLObjectPool* pool = osg::get<GLObjectPool>(0);
#else
    GLObjectPool* pool = osg::get<GLObjectPool>(state.getContextID());
#endif

    pool->track(state.getGraphicsContext());
    return pool;
}

std::unordered_map<int, GLObjectPool*>
GLObjectPool::getAll()
{
    std::unordered_map<int, GLObjectPool*> result;
    std::lock_guard<std::mutex> lock(s_pools_mutex);
    for (auto& pool : s_pools)
        result[pool->getContextID()] = pool;
    return result;
}

GLObjectPool::GLObjectPool(unsigned cxid) :
    osg::GraphicsObjectManager("osgEarth::GLObjectPool", cxid)
{
    std::lock_guard<std::mutex> lock(s_pools_mutex);
    s_pools.emplace_back(this);

    char* value = ::getenv("OSGEARTH_GL_OBJECT_POOL_DELAY");
    if (value)
        _frames_to_delay_deletion = as<unsigned>(value, _frames_to_delay_deletion);
}

namespace
{
    struct GCServicingOperation : public osg::GraphicsOperation
    {
        osg::observer_ptr<GLObjectPool> _pool;

        GCServicingOperation(GLObjectPool* pool) :
            osg::GraphicsOperation("GLObjectPool", true),
            _pool(pool) { }

        void operator()(osg::GraphicsContext* gc) override
        {
            if (_pool.valid())
                _pool->releaseOrphans(gc);
            else
                setKeep(false);
        }
    };
}

void
GLObjectPool::track(osg::GraphicsContext* gc)
{
    // if we already know about this GC, we are good
    for (auto& known_gc : _gcs)
    {
        if (known_gc == gc)
            return;
    }

    // A new GC has appeared. Create a servicing operation that will run
    // each frame and check for orphaned data.
    gc->add(new GCServicingOperation(this));

    _gcs.emplace_back(gc);
}

void
GLObjectPool::watch(GLObject::Ptr object)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _objects.emplace_back(object);
}

void
GLObjectPool::releaseGLObjects(osg::State* state)
{
    // ONLY do mass-release here; not state-based. Leave that to
    // the entities that own the GL objects themselves.
    if (state == nullptr)
    {
        for(auto pool : s_pools)
        {
            pool->releaseAll(nullptr);
        }
    }
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
    //nop
}

void
GLObjectPool::flushAllDeletedGLObjects()
{
    //nop
}

void
GLObjectPool::deleteAllGLObjects()
{
    //nop
}

void
GLObjectPool::discardAllGLObjects()
{
    //nop
}

void
GLObjectPool::releaseAll(const osg::GraphicsContext* gc)
{
    std::lock_guard<std::mutex> lock(_mutex);

    GLsizeiptr bytes = 0;
    GLObjectPool::Collection keepers;

    for (auto& object : _objects)
    {
        // if an object 'owned' by this GC is not shared, release its GL objects
        // and drop it from the tracking pool.
        if (object->gc() == gc && object->shared() == false)
        {
            object->release();
        }
        else
        {
            bytes += object->size();
            keepers.emplace_back(std::move(object));
        }
    }

    _objects.swap(keepers);
    _totalBytes = bytes;
}

void
GLObjectPool::releaseOrphans(const osg::GraphicsContext* gc)
{
    std::lock_guard<std::mutex> lock(_mutex);

    int bytes_remaining = (int)_bytes_to_delete_per_frame;

    // then check for objects to add to the orphan list:
    GLsizeiptr bytes = 0;
    Collection keepers;
    keepers.reserve(_objects.size());
    for (unsigned i = 0; i < _objects.size(); ++i)
    {
        auto& object = _objects[i];
        
        if (!object->valid())
        {
            // object has already been released; it cannot be recycled,
            // so drop it from the tracker pool.
            continue;
        }

        // the object is not referenced anywhere else but here,
        // and we've satisfied the byte-slice and time-slice requirements,
        // release its memory and drop it from the tracker pool.
        if (object.use_count() == 1 &&
            bytes_remaining > 0 &&
            object->_orphan_frames++ >= _frames_to_delay_deletion)
        {
            object->release();
            bytes_remaining -= object->size();
            continue;
        }

        // keep the object around.
        bytes += object->size();
        keepers.emplace_back(std::move(object));
    }

    _objects.swap(keepers);
    _totalBytes = bytes;
}


GLObject::GLObject(GLenum ns, osg::State& state) :
    _ns(ns),
    _ext(osg::GLExtensions::Get(state.getContextID(), true)),
    _gc(state.getGraphicsContext())
{
    gl.init();
}

void
GLObject::debugLabel(const std::string& category, const std::string& uniqueid)
{
    _category = category;
    _uid = uniqueid;

    OE_SOFT_ASSERT_AND_RETURN(valid(), void());
    ext()->debugObjectLabel(ns(), name(), label());
}

GLQuery::GLQuery(GLenum target, osg::State& state) :
    GLObject(GL_QUERY, state),
    _target(target),
    _active(false)
{
    ext()->glGenQueries(1, &_name);
}

GLQuery::Ptr
GLQuery::create(GLenum target, osg::State& state)
{
    Ptr result(new GLQuery(target, state));
    GLObjectPool::get(state)->watch(result);
    return result;
}

void
GLQuery::begin()
{
    // end an already-active query
    if (_active)
        ext()->glEndQuery(_target);

    ext()->glBeginQuery(_target, _name);
    _active = true;
}

bool
GLQuery::isReady() const
{
    OE_HARD_ASSERT(_active);
    GLuint available = GL_TRUE;
    ext()->glGetQueryObjectuiv(_name, GL_QUERY_RESULT_AVAILABLE, &available);
    return available == GL_TRUE;
}

void
GLQuery::getResult(GLuint* results)
{
    OE_HARD_ASSERT(_active);
    ext()->glGetQueryObjectuiv(_name, GL_QUERY_RESULT, results);
}

void
GLQuery::end()
{
    if (_active)
        ext()->glEndQuery(_target);
}

void
GLQuery::release()
{
    if (_name != 0)
        ext()->glDeleteQueries(1, &_name);
    _name = 0;
}


GLVAO::GLVAO(osg::State& state) :
    GLObject(GL_VERTEX_ARRAY, state)
{
    ext()->glGenVertexArrays(1, &_name);
}

GLVAO::Ptr
GLVAO::create(osg::State& state)
{
    Ptr result(new GLVAO(state));
    GLObjectPool::get(state)->watch(result);
    return result;
}

void
GLVAO::release()
{
    if (_name != 0)
        ext()->glDeleteVertexArrays(1, &_name);
    _name = 0;
}

void
GLVAO::bind()
{
    OE_SOFT_ASSERT_AND_RETURN(_name != 0, void());
    ext()->glBindVertexArray(_name);
}

void
GLVAO::unbind()
{
    ext()->glBindVertexArray(0);
}

GLBuffer::GLBuffer(GLenum target, osg::State& state) :
    GLObject(GL_BUFFER, state),
    _target(target)
{
    ext()->glGenBuffers(1, &_name);
    if (name() == 0)
    {
        GLenum e = glGetError();
        OE_FATAL << "OpenGL error " << e << std::endl;
        OE_HARD_ASSERT(name() != 0);
    }
}

GLBuffer::Ptr
GLBuffer::create(GLenum target, osg::State& state)
{
    Ptr object(new GLBuffer(target, state));
    GLObjectPool::get(state)->watch(object);
    OE_DEVEL << LC << "GLBuffer::create, name=" << object->name() << std::endl;
    return object;
}

GLBuffer::Ptr
GLBuffer::create_shared(GLenum target, osg::State& state)
{
    auto ptr = create(target, state);
    if (ptr) ptr->_shared = true;
    return ptr;
}

#define NEXT_MULTIPLE(X, Y) (((X+Y-1)/Y)*Y)

GLBuffer::Ptr
GLBuffer::create(GLenum target, osg::State& state, GLsizei sizeHint, GLsizei chunkSize)
{
#ifdef USE_RECYCLING
    GLsizei alignedSizeHint = NEXT_MULTIPLE(sizeHint, chunkSize);

    const GLObject::Compatible comp = [alignedSizeHint](GLObject* obj) {
        return
            obj->ns() == GL_BUFFER &&
            obj->recyclable() &&
            obj->size() == alignedSizeHint;
    };

    Ptr object = GLObjectPool::get(state)->recycle<GLBuffer>(comp);
    if (object)
    {
        object->_target = target;
        return object;
    }
    else
    {
        object = create(target, state);
        object->setChunkSize(chunkSize);
        object->_recyclable = true;
    }
    return object;
#else
    auto result = create(target, state);
    result->setChunkSize(chunkSize);
    return result;
#endif
}

GLBuffer::Ptr
GLBuffer::create_shared(GLenum target, osg::State& state, GLsizei sizeHint, GLsizei chunkSize)
{
    auto ptr = create(target, state, sizeHint, chunkSize);
    if (ptr) ptr->_shared = true;
    return ptr;
}

void
GLBuffer::bind() const
{
    //OE_DEVEL << LC << "GLBuffer::bind, name=" << name() << std::endl;
    if (_name == 0) {
        OE_WARN << "Catch" << std::endl;
    }
    OE_SOFT_ASSERT_AND_RETURN(_name != 0, void(), "bind() called on invalid/deleted name: " + _name << );
    ext()->glBindBuffer(_target, _name);
}

void
GLBuffer::bind(GLenum otherTarget) const
{
    if (_name == 0) {
        OE_WARN << "Catch" << std::endl;
    }
    //OE_DEVEL << LC << "GLBuffer::bind, name=" << name() << std::endl;
    OE_SOFT_ASSERT_AND_RETURN(_name != 0, void(), "bind() called on invalid/deleted name: " + label() << );
    ext()->glBindBuffer(otherTarget, _name);
}

void
GLBuffer::unbind() const
{
    OE_SOFT_ASSERT_AND_RETURN(_name != 0, void(), "unbind() called on invalid/deleted name: " + label() << );
    ext()->glBindBuffer(_target, 0);
}

void
GLBuffer::uploadData(GLsizei datasize, const GLvoid* data, GLbitfield flags) const
{
    OE_SOFT_ASSERT_AND_RETURN(_immutable == false || datasize <= size(), void());

    if (!gl.useNamedBuffers)
        bind();

    if (datasize > size())
    {
        bufferData(datasize, data, flags);
    }
    else if (data != nullptr)
    {
        bufferSubData(0, datasize, data);
    }

    if (!gl.useNamedBuffers)
        unbind();
}

void
GLBuffer::bufferData(GLsizei datasize, const GLvoid* data, GLbitfield flags) const
{
    GLsizei alloc_size = NEXT_MULTIPLE(datasize, _chunk_size);

    if (_target == GL_SHADER_STORAGE_BUFFER)
    {
        alloc_size = ::align(datasize, getSSBOAlignment<GLsizei>());
    }

    if (gl.NamedBufferData)
    {
        if (alloc_size > _alloc_size)
            gl.NamedBufferData(name(), alloc_size, nullptr, flags);

        gl.NamedBufferSubData(name(), 0, datasize, data);
    }
    else
    {
        if (alloc_size > _alloc_size)
            ext()->glBufferData(_target, alloc_size, nullptr, flags);

        ext()->glBufferSubData(_target, 0, datasize, data);
    }

    _alloc_size = alloc_size;
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
GLBuffer::bufferStorage(GLsizei datasize, const GLvoid* data, GLbitfield flags) const
{
    if (_target == GL_SHADER_STORAGE_BUFFER)
    {
        OE_SOFT_ASSERT(datasize == ::align(datasize, getSSBOAlignment<GLsizei>()));
    }

    if (recyclable() && datasize <= _alloc_size)
    {
        ext()->glBufferSubData(_target, 0, datasize, data);
    }
    else
    {
        if (recyclable())
            flags |= GL_DYNAMIC_STORAGE_BIT;

        ext()->glBufferStorage(_target, datasize, data, flags);
        _alloc_size = datasize;
    }

    _immutable = true;
}

void
GLBuffer::bufferStorage(GLsizei buffersize, GLsizei datasize, const GLvoid* data, GLbitfield flags) const
{
    OE_SOFT_ASSERT_AND_RETURN(datasize <= buffersize, void());

    if (_target == GL_SHADER_STORAGE_BUFFER)
    {
        buffersize = ::align(buffersize, getSSBOAlignment<GLsizei>());
    }

    if (recyclable() && datasize <= _alloc_size)
    {
        ext()->glBufferSubData(_target, 0, datasize, data);
    }
    else
    {
        if (recyclable())
            flags |= GL_DYNAMIC_STORAGE_BIT;

        ext()->glBufferStorage(_target, buffersize, nullptr, flags);
        _alloc_size = buffersize;

        ext()->glBufferSubData(_target, 0, datasize, data);
    }

    _immutable = true;
}

void
GLBuffer::copyBufferSubData(GLBuffer::Ptr dst, GLintptr readOffset, GLintptr writeOffset, GLsizei size) const
{
    if (gl.CopyNamedBufferSubData)
        gl.CopyNamedBufferSubData(name(), dst->name(), readOffset, writeOffset, size);
    else
        gl.CopyBufferSubData(target(), dst->target(), readOffset, writeOffset, size);
}

void
GLBuffer::getBufferSubData(GLintptr offset, GLsizei size, void* data) const
{
    if (gl.GetNamedBufferSubData)
        gl.GetNamedBufferSubData(name(), offset, size, data);
    else
        ext()->glGetBufferSubData(_target, offset, size, data);
}

void*
GLBuffer::map(GLbitfield access) const
{
    if (gl.MapNamedBuffer)
        return gl.MapNamedBuffer(_name, access);
    else
        return ext()->glMapBuffer(_target, access);
}

void*
GLBuffer::mapRange(GLintptr offset, GLsizei length, GLbitfield access) const
{
    if (gl.MapNamedBufferRange)
        return gl.MapNamedBufferRange(_name, offset, length, access);
    else
        return ext()->glMapBufferRange(_target, offset, length, access);
}

void
GLBuffer::unmap() const
{
    if (gl.UnmapNamedBuffer)
        gl.UnmapNamedBuffer(_name);
    else
        ext()->glUnmapBuffer(_target);
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
    if (_name != 0)
    {
        OE_DEVEL << LC << "GLBuffer::release, name=" << name() << std::endl;

#ifdef OSGEARTH_SINGLE_GL_CONTEXT
        if (_isResident == true)
        {
            OE_HARD_ASSERT(gl.MakeNamedBufferNonResidentNV);
            gl.MakeNamedBufferNonResidentNV(name());
            _address = 0;
            _isResident.value = false;
        }
#else
        for (auto& i : _isResident)
            i.second.value = false;
#endif

        //OE_DEVEL << "Releasing buffer " << _name << "(" << _label << ")" << std::endl;
        ext()->glDeleteBuffers(1, &_name);
        _name = 0;
        _alloc_size = 0;
    }
}

GLuint64
GLBuffer::address()
{
    if (_address == 0)
    {
        gl.init();
        OE_HARD_ASSERT(gl.GetNamedBufferParameterui64vNV);
        gl.GetNamedBufferParameterui64vNV(name(), GL_BUFFER_GPU_ADDRESS_NV, &_address);
    }
    return _address;
}

void
GLBuffer::makeResident(osg::State& state)
{
#ifdef OSGEARTH_SINGLE_GL_CONTEXT
    Resident& resident = _isResident;
#else
    Resident& resident = _isResident[state.getGraphicsContext()];
#endif

    OE_SOFT_ASSERT(address() != 0, "makeResident() called on buffer with no address");

    // Note: it's legal to call this function if the object is already resident.

    if (address() != 0 && resident == false)
    {
        OE_HARD_ASSERT(gl.MakeNamedBufferResidentNV);
        //Currently only GL_READ_ONLY is supported according to the spec
        gl.MakeNamedBufferResidentNV(name(), GL_READ_ONLY_ARB);
        resident.value = true;
    }
}

void
GLBuffer::makeNonResident(osg::State& state)
{
#ifdef OSGEARTH_SINGLE_GL_CONTEXT
    Resident& resident = _isResident;
#else
    Resident& resident = _isResident[state.getGraphicsContext()];
#endif

    OE_SOFT_ASSERT(address() != 0, "makeNonResident() called on buffer with no address");

    // Note: it's legal to call this function if the object is already non-resident.

    if (address() != 0 && resident == true)
    {
        OE_HARD_ASSERT(gl.MakeNamedBufferNonResidentNV);
        gl.MakeNamedBufferNonResidentNV(name());
        // address can be invalidated, so zero it out
        _address = 0;
        resident.value = false;
    }
}

size_t
GLBuffer::align(size_t val)
{
    if (_target == GL_SHADER_STORAGE_BUFFER)
        return ::align(val, getSSBOAlignment<size_t>());
    else
        return ::align(val, getUBOAlignment<size_t>());
}

void
GLBuffer::setChunkSize(GLsizei value)
{
    if (value > 0)
    {
        _chunk_size = align(value);
    }
}

GLTexture::Profile::Profile(GLenum target) :
    osg::Texture::TextureProfile(target)
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

GLTexture::GLTexture(GLenum target, osg::State& state) :
    GLObject(GL_TEXTURE, state),
    _target(target),
    _profile(target)
{
    glGenTextures(1, &_name);
    _shared = true;
}

GLTexture::Ptr
GLTexture::create(GLenum target, osg::State& state)
{
    Ptr object(new GLTexture(target, state));
    GLObjectPool::get(state)->watch(object);
    OE_DEVEL << LC << "GLTexture::create, name=" << object->name() << std::endl;
    return object;
}

GLTexture::Ptr
GLTexture::create(GLenum target, osg::State& state, const Profile& profileHint)
{
#ifdef USE_RECYCLING
    const GLObject::Compatible comp = [profileHint](GLObject* obj) {
        return
            obj->ns() == GL_TEXTURE &&
            obj->recyclable() &&
            static_cast<GLTexture*>(obj)->_profile == profileHint;
    };

    Ptr object = GLObjectPool::get(state)->recycle<GLTexture>(comp);
    if (object)
    {
        return object;
    }
    else
    {
        object = create(target, state);
        object->_recyclable = true;
    }
    return object;
#else
    return create(target, state);
#endif
}

void
GLTexture::bind(osg::State& state)
{
    OE_DEVEL << LC << "GLTexture::bind, name=" << name() << std::endl;
    OE_SOFT_ASSERT_AND_RETURN(_name != 0, void(), "bind() called on invalid/deleted name: " + label() << );

    glBindTexture(_target, _name);

    // Inform OSG of the state change
    state.haveAppliedTextureAttribute(state.getActiveTextureUnit(), osg::StateAttribute::TEXTURE);

    // account for the FFP version of the mode
    GLenum fixed_function_target = _target;
    if (_target == GL_TEXTURE_2D_ARRAY)
    {
        fixed_function_target = GL_TEXTURE_2D;
    }

    state.haveAppliedTextureMode(state.getActiveTextureUnit(), fixed_function_target);
}

GLuint64
GLTexture::handle(osg::State& state)
{
    if (_handle == 0)
    {
        bind(state);
        _handle = ext()->glGetTextureHandle(_name);
    }

    OE_SOFT_ASSERT(_handle != 0, "glGetTextureHandle failed");
    return _handle;
}

void
GLTexture::makeResident(const osg::State& state, bool toggle)
{
#ifdef OSGEARTH_SINGLE_GL_CONTEXT
    Resident& resident = _isResident;
#else
    Resident& resident = _isResident[state.getGraphicsContext()];
#endif

    // Note: it's legal to call this function if the object is already resident.
    
    if (resident != toggle)
    {
        OE_SOFT_ASSERT_AND_RETURN(_handle != 0, void(), "makeResident() called on invalid handle: " + label() << );

        if (toggle == true)
            ext()->glMakeTextureHandleResident(_handle);
        else
            ext()->glMakeTextureHandleNonResident(_handle);

        OE_DEVEL << "'" << id() << "' name=" << name() << " resident=" << (toggle ? "yes" : "no") << std::endl;

        resident.value = toggle;
    }
}

bool
GLTexture::isResident(const osg::State& state) const
{
#ifdef OSGEARTH_SINGLE_GL_CONTEXT
    Resident& resident = _isResident;
#else
    Resident& resident = _isResident[state.getGraphicsContext()];
#endif

    return resident == true;
}

void
GLTexture::release()
{
    OE_DEVEL << LC << "GLTexture::release, name=" << name() << std::endl;
    if (_handle != 0)
    {        
#ifdef OSGEARTH_SINGLE_GL_CONTEXT
        if (_isResident == true)
            ext()->glMakeTextureHandleNonResident(_handle);

        _isResident.value = false;
#else
        for (auto& i : _isResident)
            i.second.value = false;
#endif
        _handle = 0;
    }
    if (_name != 0)
    {
        OE_DEVEL << "Releasing texture " << _name << "(" << label() << ")" << std::endl;
        glDeleteTextures(1, &_name);
        _name = 0;
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

        // special trick: signed RGTC1 textures are compressed normals, so
        // swizzle the A component to be a zero so we can detect them.
        if (profile._internalFormat == GL_COMPRESSED_SIGNED_RED_RGTC1_EXT ||
            profile._internalFormat == GL_COMPRESSED_SIGNED_RED_GREEN_RGTC2_EXT ||
            profile._internalFormat == GL_COMPRESSED_RED_RGTC1_EXT ||
            profile._internalFormat == GL_COMPRESSED_RED_GREEN_RGTC2_EXT)
        {
            glTexParameteri(_target, GL_TEXTURE_SWIZZLE_A, GL_ZERO);
        }


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
GLFBO::create(osg::State& state)
{
    Ptr object(new GLFBO(state));
    GLObjectPool::get(state)->watch(object);
    return object;
}

GLFBO::GLFBO(osg::State& state) :
    GLObject(GL_FRAMEBUFFER, state)
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

GLsizei
GLFBO::size() const
{
    //todo
    return 0;
}

GLTexture::Ptr
GLFBO::renderToTexture(GLsizei width, GLsizei height, DrawFunction draw, osg::State& state)
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
        profile);

    texture->debugLabel("GLFBO");

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
#define LC "[GLPipeline]"

GLPipeline::Dispatcher::Dispatcher(GLPipeline::Ptr pipeline) :
    osg::GraphicsOperation("GLPipelineDispatcher", true),
    _pipeline_ref(pipeline),
    _myGC(pipeline->_gc)
{
    //nop
}

void
GLPipeline::Dispatcher::push(osg::Operation* op)
{
    _queue_mutex.lock();
    _thisQ.push(op);
    _queue_mutex.unlock();
}

void
GLPipeline::Dispatcher::operator()(osg::GraphicsContext* gc)
{
    osg::ref_ptr<osg::FrameStamp> fs = new osg::FrameStamp();

    if (_pipeline_ref.expired())
    {
        setKeep(false);
        return;
    }

    OpQ keptOps;

    const double MAX_TIME_MS = 1.5;
    double elapsedTimeMS = 0.0;    

    osg::Timer_t startTick = osg::Timer::instance()->tick();

    while (!_thisQ.empty())
    {
        osg::ref_ptr<osg::Operation> next;

        // check for new job:
        _queue_mutex.lock();
        {
            if (!_thisQ.empty())
            {
                next = _thisQ.front();
                _thisQ.pop();
            }
        }
        _queue_mutex.unlock();

        if (next.valid())
        {
            // run it
            next->operator()(gc);

            // if keep==true, that means the delegate wishes to run
            // again for another invocation. In this case we defer it to
            // the next "frame" as deliniated by the frame-sync. The whole
            // point of a multi-invocation operation is to let the GPU have
            // time to complete its async calls between invocations!
            if (next->getKeep())
            {
                _thisQ.push(next);
            }
        }

        elapsedTimeMS += osg::Timer::instance()->delta_m(startTick, osg::Timer::instance()->tick());
        if (elapsedTimeMS >= MAX_TIME_MS)
        {
            break;
        }
    }

    // Add the kept operations back to the queue
    while (!keptOps.empty())
    {
        osg::ref_ptr<osg::Operation> i = keptOps.front();
        keptOps.pop();
        _thisQ.push(i);
    }
}

//static defs
Mutex GLPipeline::_mutex;
std::unordered_map<osg::State*, GLPipeline::Ptr> GLPipeline::_lut;


GLPipeline::Ptr
GLPipeline::get(osg::State& state)
{
    std::unique_lock<std::mutex> lock(GLPipeline::_mutex);
    GLPipeline::Ptr& p = _lut[&state];

    if (p == nullptr)
    {
        p = std::make_shared<GLPipeline>();
        p->_gc = state.getGraphicsContext();
        p->_dispatcher = new Dispatcher(p);
        state.getGraphicsContext()->add(p->_dispatcher.get());
    }

    OE_HARD_ASSERT(p != nullptr, "Cannot find a GC :(");
    return p;
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
ComputeImageSession::execute(osg::State& state)
{
    auto task = [this](osg::State& state, jobs::promise<bool>& promise, int invocation)
        {
            if (invocation == 0)
            {
                OE_GL_ZONE_NAMED("CIS/render");
                render(&state);
                return true; // request another invocation
            }
            else
            {
                OE_GL_ZONE_NAMED("CIS/readback");
                readback(&state);
                return false; // all done.
            }
        };

    auto job = GLPipeline::get(state)->dispatch<bool>(task);
    job.join();
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
        ext->glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, 0);
    }

    renderImplementation(state);

    // Post an async readback to the GL queue
    ext->glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, _pbo);
    glGetTexImage(GL_TEXTURE_2D, 0, _image->getPixelFormat(), _image->getDataType(), 0);
    ext->glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, 0);
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

#include <iostream>
namespace
{
    using ICO = osgUtil::IncrementalCompileOperation;

    struct ICOCallback : public ICO::CompileCompletedCallback
    {
        jobs::promise<osg::ref_ptr<osg::Node>> _promise;
        osg::ref_ptr<osg::Node> _node;
        std::atomic_int& _jobsActive;

        ICOCallback(const osg::ref_ptr<osg::Node>& node, std::atomic_int& jobsActive) :
            _node(node),
            _jobsActive(jobsActive) { }

        virtual ~ICOCallback()
        {
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

namespace
{
    struct StateToCompileEx : public osgUtil::StateToCompile
    {
        void apply(osg::StateSet& stateSet) override
        {
        }
    };
}

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

jobs::future<osg::ref_ptr<osg::Node>>
GLObjectsCompiler::compileAsync(
    const osg::ref_ptr<osg::Node>& node,
    const osg::Object* host,
    Cancelable* progress) const
{
    jobs::future<osg::ref_ptr<osg::Node>> result;

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
                result = callback->_promise;
                compileSet->_compileCompletedCallback = callback;
                _jobsActive++;
                ico->add(compileSet, false);
                compileScheduled = true;
            }
        }

        if (!compileScheduled)
        {
            // no ICO available - just resolve the future immediately
            result.resolve(node);
        }
    }

    return result;
}

jobs::future<osg::ref_ptr<osg::Node>>
GLObjectsCompiler::compileAsync(
    const osg::ref_ptr<osg::Node>& node,
    osgUtil::StateToCompile* state,
    const osg::Object* host,
    Cancelable* progress) const
{
    jobs::future<osg::ref_ptr<osg::Node>> result;

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
            result = callback->_promise;
            compileSet->_compileCompletedCallback = callback;
            _jobsActive++;
            ico->add(compileSet, false);
            compileScheduled = true;
        }
    }

    if (!compileScheduled)
    {
        // no ICO available - just resolve the future immediately
        result.resolve(node);
    }

    return result;
}

void
GLObjectsCompiler::requestIncrementalCompile(
    const osg::ref_ptr<osg::Node>& node,
    osgUtil::StateToCompile* state,
    osg::observer_ptr<const osg::Object> host_weak,
    jobs::promise<osg::ref_ptr<osg::Node>> promise) const
{
    if (!node.valid())
    {
        promise.resolve();
        return;
    }

    // if there is an ICO available, schedule the GPU compilation
    bool compileScheduled = false;

    if (state != nullptr && !state->empty())
    {
        osg::ref_ptr<const osg::Object> host;
        osg::ref_ptr<ICO> ico;
        if (host_weak.lock(host) && ObjectStorage::get(host.get(), ico) && ico->isActive())
        {
            auto compileSet = new osgUtil::IncrementalCompileOperation::CompileSet();
            compileSet->buildCompileMap(ico->getContextSet(), *state);
            ICOCallback* callback = new ICOCallback(node, _jobsActive);
            callback->_promise = promise;
            compileSet->_compileCompletedCallback = callback;
            _jobsActive++;
            ico->add(compileSet, false);
            compileScheduled = true;
        }
    }

    if (!compileScheduled)
    {
        // no ICO available - just resolve the future immediately
        promise.resolve(node);
    }
}

void
GLObjectsCompiler::compileNow(
    const osg::ref_ptr<osg::Node>& node,
    const osg::Object* host,
    Cancelable* progress) const
{
    if (node)
    {
        auto result = compileAsync(node, host, progress);
        result.join(progress);
    }
}
