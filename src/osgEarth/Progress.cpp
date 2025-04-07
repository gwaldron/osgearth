/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */

#include <osgEarth/Progress>

using namespace osgEarth;

ProgressCallback::ProgressCallback() :
    _canceled(false),
    _retryDelay_s(0.0f),
    _cancelable(nullptr)
{
    //NOP
}

ProgressCallback::ProgressCallback(Cancelable* cancelable) :
    _canceled(false),
    _retryDelay_s(0.0f),
    _cancelable(cancelable)
{
    //NOP
}

ProgressCallback::ProgressCallback(Cancelable* cancelable, std::function<bool()> predicate) :
    _canceled(false),
    _retryDelay_s(0.0f),
    _cancelable(cancelable),
    _cancelPredicate(predicate)
{
    //NOP
}

void
ProgressCallback::cancel()
{
    _canceled = true;
}

void
ProgressCallback::reset()
{
    _canceled = false;
}

bool
ProgressCallback::canceled() const
{
    if (!_canceled)
    {
        if ((shouldCancel()) ||
            (_cancelable && _cancelable->canceled()) ||
            (_cancelPredicate && _cancelPredicate()))
        {
            _canceled = true;
        }
    }
    return _canceled;
}

void ProgressCallback::reportError(const std::string& msg)
{
    _message = msg;
}

bool ProgressCallback::reportProgress(double             current,
                                      double             total,
                                      unsigned           stage,
                                      unsigned           numStages,
                                      const std::string& msg )
{
    return false;
}

