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

#include <osg/LineStipple>
#include <osg/GraphicsContext>
#include <osgUtil/IncrementalCompileOperation>
#include <osgViewer/GraphicsWindow>
#include <osg/Texture2D>
#include <osg/BindImageTexture>

#include <osg/ContextData>

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

GLsizei
GLUtils::getSSBOAlignment(osg::State& state)
{
    static GLsizei _ssboAlignment = -1;
    if (_ssboAlignment < 0)
        glGetIntegerv(GL_SHADER_STORAGE_BUFFER_OFFSET_ALIGNMENT, &_ssboAlignment);
    return _ssboAlignment;
}

void
CustomRealizeOperation::setSyncToVBlank(bool value)
{
    _vsync = value;
}

void
CustomRealizeOperation::setEnableGLDebugging(bool value)
{
    _gldebug = value;
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

        if (severity != GL_DEBUG_SEVERITY_NOTIFICATION)
        {
            const std::string& s = severities[severity - GL_DEBUG_SEVERITY_HIGH];
            OE_WARN << "GL (" << s << ", " << source << ") -- " << message << std::endl;
        }
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

    if (_gldebug.isSetTo(true))
    {
        osg::GraphicsContext* gc = static_cast<osg::GraphicsContext*>(object);
        OE_HARD_ASSERT(gc != nullptr);

        OE_INFO << "ENABLING DEBUG GL MESSAGES" << std::endl;
        glEnable(GL_DEBUG_OUTPUT);
        glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
        GLFunctions& gl = GLFunctions::get(*gc->getState());
        gl.glDebugMessageCallback(s_oe_gldebugproc, nullptr);
        gl.glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DEBUG_SEVERITY_MEDIUM, 0, nullptr, GL_TRUE);
        gl.glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DEBUG_SEVERITY_HIGH, 0, nullptr, GL_TRUE);
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
    }
}

GLBuffer::Ptr
GLBuffer::create(GLenum target, osg::State& state, const std::string& label)
{
    Ptr obj(new GLBuffer(target, state, label));
    GLObjectReleaser::watch(obj, state);
    OE_DEVEL << LC << "GLBuffer::create, name=" << obj->name() << std::endl;
    return obj;
}

void
GLBuffer::bind() const
{
    OE_DEVEL << LC << "GLBuffer::bind, name=" << name() << std::endl;
    OE_SOFT_ASSERT_AND_RETURN(_name != ~0U, void(), "bind() called on invalid/deleted name");
    ext()->glBindBuffer(_target, _name);
}

void
GLBuffer::bind(GLenum otherTarget) const
{
    OE_DEVEL << LC << "GLBuffer::bind, name=" << name() << std::endl;
    OE_SOFT_ASSERT_AND_RETURN(_name != ~0U, void(), "bind() called on invalid/deleted name");
    ext()->glBindBuffer(otherTarget, _name);
}

void
GLBuffer::storage(GLintptr size, GLvoid* data, GLbitfield flags) const
{
    ext()->glBufferStorage(_target, size, data, flags);
}

void
GLBuffer::subData(GLintptr offset, GLsizeiptr size, GLvoid* data) const
{
    ext()->glBufferSubData(_target, offset, size, data);
}

void
GLBuffer::release()
{
    if (_name != ~0U)
    {
        OE_DEVEL << LC << "GLBuffer::release, name=" << name() << std::endl;
        //OE_DEVEL << "Releasing buffer " << _name << "(" << _label << ")" << std::endl;
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
        bind(state);
        ext()->debugObjectLabel(GL_TEXTURE, _name, label);
    }
}

GLTexture::Ptr
GLTexture::create(GLenum target, osg::State& state, const std::string& label)
{
    Ptr obj(new GLTexture(target, state, label));
    GLObjectReleaser::watch(obj, state);
    OE_DEVEL << LC << "GLTexture::release, name=" << obj->name() << std::endl;
    return obj;
}

void
GLTexture::bind(osg::State& state)
{
    OE_DEVEL << LC << "GLTexture::bind, name=" << name() << std::endl;
    OE_SOFT_ASSERT_AND_RETURN(_name != ~0U, void(), "bind() called on invalid/deleted name");

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
    if (_isResident != toggle)
    {
        OE_SOFT_ASSERT_AND_RETURN(_handle != ~0ULL, void(), "makeResident() called on invalid handle");

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


SSBO::SSBO() :
    _allocatedSize(0),
    _bindingIndex(-1)
{
    //nop
}

void
SSBO::release() const
{
    _buffer = nullptr; // triggers the releaser
    _allocatedSize = 0u;
}

void
SSBO::bindLayout() const
{
    if (_buffer != nullptr && _bindingIndex >= 0)
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
GLObjectReleaser::watch(GLObject::Ptr object, osg::State& state_unused)
{
    if (object && object->ext())
    {
        GLObjectReleaser* rel = osg::get<GLObjectReleaser>(object->ext()->contextID);
        if (rel)
        {
            ScopedMutexLock lock(rel->_mutex);
            rel->_objects.insert(object);
            OE_DEVEL << LC << "Added \"" << object->label() << "\"" << std::endl;
        }
    }
}

void
GLObjectReleaser::releaseAll(osg::State& state)
{
    GLObjectReleaser* rel = osg::get<GLObjectReleaser>(state.getContextID());
    if (rel)
    {
        ScopedMutexLock lock(rel->_mutex);

        for (auto& object : rel->_objects)
        {
            OE_DEVEL << LC << "Releasing \"" << object->label() << "\"" << std::endl;
            object->release();
        }

        rel->_objects.clear();
    }
}

void
GLObjectReleaser::flushDeletedGLObjects(double currentTime, double& availableTime)
{
    // OSG calls this method periodically
    flushAllDeletedGLObjects();
}

void
GLObjectReleaser::flushAllDeletedGLObjects()
{
    ScopedMutexLock lock(_mutex);

    // OSG calls this method periodically
    std::unordered_set<std::shared_ptr<GLObject>> temp;

    for (auto& object : _objects)
    {
        if (object.use_count() == 1)
        {
            OE_DEVEL << LC << "Releasing \"" << object->label() << "\"" << std::endl;
            object->release();
        }
        else
        {
            temp.insert(object);
        }
    }

    _objects.swap(temp);
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
    ScopedMutexLock lock(_mutex);
    // no graphics context available..just empty the bucket
    _objects.clear();
}

#undef LC
#define LC "[GPUJobArena] "

#if 0
GPUJobArena<bool>::Result
GPUJobArena::dispatchOnAllContexts(
    GPUJobArena::Function& function)
{
    if (_gcs.size() == 1)
    {
        return dispatch(function);
    }

    else
    {
        Job<bool>::dispatch(
            [...](Cancelable* progress)
            {
                std::queue<Future<bool>> futures;

                for (auto& gc : _gcs)
                {
                    Future<bool> f = arena(gc).dispatch(...);
                    futures.emplace_back(f);
                }

                while (futures.empty() == false)
                {
                    futures.front().get(progress);
                    futures.pop_front();
                }
            }
        );
    }
}
#endif


//static defs
osg::ref_ptr<GPUJobArena> GPUJobArena::_arena_pool;
Mutex GPUJobArena::_arena_pool_mutex;

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
    _done(false)
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

GLFunctions::GLFunctions() :
    glBufferStorage(NULL)
{
    //nop
}
GLFunctions GLFunctions::_buf[256];

GLFunctions&
GLFunctions::get(unsigned contextID)
{
    GLFunctions& f = _buf[contextID];
    if (f.glBufferStorage == NULL)
    {
        osg::setGLExtensionFuncPtr(f.glBufferStorage, "glBufferStorage", "glBufferStorageARB");
        osg::setGLExtensionFuncPtr(f.glClearBufferSubData, "glClearBufferSubData", "glClearBufferSubDataARB");
        osg::setGLExtensionFuncPtr(f.glMultiDrawElementsIndirect, "glMultiDrawElementsIndirect", "glMultiDrawElementsIndirectARB");
        osg::setGLExtensionFuncPtr(f.glDispatchComputeIndirect, "glDispatchComputeIndirect", "glDispatchComputeIndirectARB");
        osg::setGLExtensionFuncPtr(f.glTexStorage3D, "glTexStorage3D", "glTexStorage3DARB");
        osg::setGLExtensionFuncPtr(f.glDebugMessageCallback, "glDebugMessageCallback", "glDebugMessageCallbackKHR");
        osg::setGLExtensionFuncPtr(f.glDebugMessageControl, "glDebugMessageControl", "glDebugMessageControlKHR");
    }
    return f;
}