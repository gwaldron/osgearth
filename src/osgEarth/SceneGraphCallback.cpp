/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/
#include <osgEarth/SceneGraphCallback>

using namespace osgEarth; 
using namespace osgEarth::Util;

//...................................................................

SceneGraphCallbacks::SceneGraphCallbacks(osg::Object* sender) :
    _sender(sender)
{
    //nop
}

void
SceneGraphCallbacks::add(SceneGraphCallback* cb)
{
    if (cb)
    {
        Threading::ScopedRecursiveMutexLock lock(_mutex);
        _callbacks.push_back(cb);
    }
}

void
SceneGraphCallbacks::remove(SceneGraphCallback* cb)
{
    if (cb)
    {
        Threading::ScopedRecursiveMutexLock lock(_mutex);
        for (SceneGraphCallbackVector::iterator i = _callbacks.begin(); i != _callbacks.end(); ++i)
        {
            if (i->get() == cb)
            {
                _callbacks.erase(i);
                break;
            }
        }
    }
}

void
SceneGraphCallbacks::firePreMergeNode(osg::Node* node)
{
    SceneGraphCallbackVector copy;
    {
        Threading::ScopedRecursiveMutexLock lock(_mutex);
        copy = _callbacks;
    }

    osg::ref_ptr<osg::Object> sender;
    _sender.lock(sender);
    for (SceneGraphCallbackVector::iterator i = copy.begin(); i != copy.end(); ++i)
        i->get()->onPreMergeNode(node, sender.get());
}

void
SceneGraphCallbacks::firePostMergeNode(osg::Node* node)
{
    SceneGraphCallbackVector copy;
    {
        Threading::ScopedRecursiveMutexLock lock(_mutex);
        copy = _callbacks;
    }

    osg::ref_ptr<osg::Object> sender;
    _sender.lock(sender);
    for (SceneGraphCallbackVector::iterator i = copy.begin(); i != copy.end(); ++i)
        i->get()->onPostMergeNode(node, sender.get());
}

void
SceneGraphCallbacks::fireRemoveNode(osg::Node* node)
{
    SceneGraphCallbackVector copy;
    {
        Threading::ScopedRecursiveMutexLock lock(_mutex);
        copy = _callbacks;
    }

    osg::ref_ptr<osg::Object> sender;
    _sender.lock(sender);
    for (SceneGraphCallbackVector::iterator i = copy.begin(); i != copy.end(); ++i)
        i->get()->onRemoveNode(node, sender.get());
}

//...................................................................

PagedLODWithSceneGraphCallbacks::PagedLODWithSceneGraphCallbacks(SceneGraphCallbacks* host) :
_host(host)
{
    //nop
}

SceneGraphCallbacks*
PagedLODWithSceneGraphCallbacks::getSceneGraphCallbacks() const
{
    return _host.get();
}

void
PagedLODWithSceneGraphCallbacks::setSceneGraphCallbacks(SceneGraphCallbacks* host)
{
    _host = host;
}

bool
PagedLODWithSceneGraphCallbacks::addChild(osg::Node* child)
{
    bool ok = false;
    if (child)
    {
        ok = osg::PagedLOD::addChild(child);
        osg::ref_ptr<SceneGraphCallbacks> host;
        if (_host.lock(host))
            host->firePostMergeNode(child);
    }
    return ok;
}

bool
PagedLODWithSceneGraphCallbacks::insertChild(unsigned index, osg::Node* child)
{
    bool ok = false;
    if (child)
    {
        ok = osg::PagedLOD::insertChild(index, child);
        osg::ref_ptr<SceneGraphCallbacks> host;
        if (_host.lock(host))
            host->firePostMergeNode(child);
    }
    return ok;
}

bool
PagedLODWithSceneGraphCallbacks::replaceChild(osg::Node* oldChild, osg::Node* newChild)
{
    bool ok = false;
    if (oldChild && newChild)
    {
        ok = osg::PagedLOD::replaceChild(oldChild, newChild);
        osg::ref_ptr<SceneGraphCallbacks> host;
        if (_host.lock(host))
            host->firePostMergeNode(newChild);
    }
    return ok;
}

void
PagedLODWithSceneGraphCallbacks::childRemoved(unsigned pos, unsigned num)
{
    osg::ref_ptr<SceneGraphCallbacks> host;
    if (_host.lock(host))
    {
        for (unsigned i = pos; i < pos + num; ++i)
        {
            host->fireRemoveNode(getChild(i));
        }
    }
}
