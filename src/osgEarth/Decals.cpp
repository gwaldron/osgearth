/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include "Decals"
#include "CullingUtils"
#include "Shaders"

#include <osg/ComputeBoundsVisitor>
#include <osg/ShapeDrawable>
#include <osg/PolygonMode>

using namespace osgEarth;

void
DecalNode::traverse(osg::NodeVisitor& nv)
{
    if (!_decals.empty())
    {
        if (nv.getVisitorType() == nv.CULL_VISITOR)
        {
            osgUtil::CullVisitor* cv = Culling::asCullVisitor(nv);
            if (cv->getCurrentRenderBin()->getName() != "OE_EMPTY_RENDER_BIN")
            {
                std::shared_ptr<detail::DecalDrawList> drawList;
                if (ObjectStorage::get(&nv, drawList))
                {
                    auto& leaves = drawList->_perCamera.get(cv->getState()).leaves;
                    auto invview = cv->getCurrentCamera()->getInverseViewMatrix();
                    auto& mvm = *cv->getModelViewMatrix();
                    osg::Matrix localToWorld = mvm * invview;
                    for (auto& decal : _decals)
                    {
                        Decal leaf = decal;
                        leaf.matrix = localToWorld * decal.matrix;
                        leaves.emplace_back(std::move(leaf));
                    }
                }

                if (_debug.valid())
                {
                    _debug->accept(nv);
                }
            }
        }
    }
    osg::Node::traverse(nv);
}

void
DecalNode::debug(bool value)
{
    _debug = nullptr;
    if (value)
    {
        _debug = new osg::Group();

        for (auto& decal : _decals) {
            auto mt = new osg::MatrixTransform(decal.matrix);
            mt->addChild(new osg::ShapeDrawable(new osg::Box(osg::Vec3(0, 0, 0), decal.size.x(), decal.size.y(), decal.size.z())));
            _debug->addChild(mt);
        }
        _debug->getOrCreateStateSet()->setAttribute(new osg::PolygonMode(
            osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE));
    }
}

osg::BoundingSphere
DecalNode::computeBound() const
{
    osg::BoundingSphere bs;
    for (const auto& decal : _decals) 
    {
        bs.expandBy(osg::BoundingSphere(
            osg::Vec3(0, 0, 0) * decal.matrix,
            std::max(decal.size.x(), decal.size.y()) * 0.7071f));
    }
    return bs;
}



void
DecalRTTNode::traverse(osg::NodeVisitor& nv)
{
    if (getNumChildren() > 0)
    {
        if (nv.getVisitorType() == nv.CULL_VISITOR)
        {
            osgUtil::CullVisitor* cv = Culling::asCullVisitor(nv);
            if (cv->getCurrentRenderBin()->getName() != "OE_EMPTY_RENDER_BIN")
            {
                if (_needsRTT)
                {
                    std::lock_guard<std::mutex> lock(_rttMutex);

                    if (!_rtt.valid())
                    {
                        _tex = new osg::Texture2D();
                        _tex->setTextureSize(_texWidth, _texHeight);
                        _tex->setSourceFormat(GL_RGBA);
                        _tex->setInternalFormat(GL_RGBA8);
                        _tex->setSourceType(GL_UNSIGNED_BYTE);

                        // create RTT camera
                        _rtt = new osg::Camera();
                        _rtt->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
                        _rtt->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
                        _rtt->setRenderOrder(osg::Camera::PRE_RENDER);
                        _rtt->setClearColor(osg::Vec4(0, 0, 0, 0));
                        _rtt->setClearMask(GL_COLOR_BUFFER_BIT); // | GL_DEPTH_BUFFER_BIT);
                        _rtt->setImplicitBufferAttachmentMask(0, 0);
                        _rtt->setViewport(0, 0, _texWidth, _texHeight);
                        _rtt->setViewMatrix(osg::Matrix::identity());
                        _rtt->setSmallFeatureCullingPixelSize(0.0f);
                        _rtt->attach(osg::Camera::COLOR_BUFFER0, _tex.get());   
                    }

                    if (_rtt->getNumChildren() == 0)
                    {
                        for (unsigned i = 0; i < getNumChildren(); ++i)
                            _rtt->addChild(getChild(i));
                    }

                    osg::ComputeBoundsVisitor cbv;
                    _rtt->accept(cbv);
                    auto& bbox = cbv.getBoundingBox();

                    _rtt->setProjectionMatrixAsOrtho2D(bbox.xMin(), bbox.xMax(), bbox.yMin(), bbox.yMax());

                    _rtt->accept(nv);

                    if (_autoSize)
                    {
                        _decal.size.x() = bbox.xMax() - bbox.xMin();
                        _decal.size.y() = bbox.yMax() - bbox.yMin();
                        _decal.textureSize->set(_decal.size.x(), _decal.size.y());
                    }
                    //_decal.size.z() = std::max(_decal.size.x(), _decal.size.y());
                    //_decal.size.x() = _decal.size.y() * (bbox.xMax() - bbox.xMin()) / (bbox.yMax() - bbox.yMin());

                    _needsRTT = getDynamic();

                    this->dirtyBound();
                }

                // wait until the texture gets compiled, and then wrap it to make it bindless
                if (_decal.texture == nullptr && _tex->areAllTextureObjectsLoaded())
                {
                    _decal.texture = Texture::create(_tex.get());

                    if (!getDynamic())
                        _rtt = nullptr; // release the RTT camera
                }

                std::shared_ptr<detail::DecalDrawList> drawList;
                if (ObjectStorage::get(&nv, drawList))
                {
                    auto& leaves = drawList->_perCamera.get(cv->getState()).leaves;
                    auto& mm = *cv->getModelViewMatrix() * cv->getCurrentCamera()->getInverseViewMatrix();
                    Decal leaf = _decal;
                    leaf.matrix = mm * _decal.matrix;
                    leaves.emplace_back(std::move(leaf));
                }
            }
        }
    }
    osg::Node::traverse(nv); // correct; do NOT call osg::Group::traverse!
}


void
DecalRTTNode::dirty()
{
    OE_SOFT_ASSERT_AND_RETURN(getDynamic(), void(), "DecalRTTNode::dirty() called on non-dynamic node");

    std::lock_guard<std::mutex> lock(_rttMutex);
    
    if (_rtt.valid())
    {
        _rtt->removeChildren(0, _rtt->getNumChildren());
    }
    _needsRTT = true;
}


osg::BoundingSphere
DecalRTTNode::computeBound() const
{
    _needsRTT = true;
    osg::BoundingSphere bs;
    bs.expandBy(osg::BoundingSphere(
        osg::Vec3(0, 0, 0) * _decal.matrix,
        std::max(_decal.size.x(), _decal.size.y()) * 0.7071f));
    return bs;
}


DecalDecorator*
DecalDecorator::getOrCreate(osg::StateSet* stateSet)
{
    OE_HARD_ASSERT(stateSet);
    auto dec = dynamic_cast<DecalDecorator*>(stateSet->getAttribute((osg::StateAttribute::Type)1001001001));
    if (!dec)
    {
        dec = new DecalDecorator();
        stateSet->setAttribute(dec);
        auto* vp = VirtualProgram::getOrCreate(stateSet);
        Shaders package;
        package.load(vp, package.Decals);
        //stateSet->setDefine("OE_DECALS_BUF_BINDING", std::to_string(dec->_bufferBinding), ~0);
        //stateSet->setDefine("OE_DECALS_TEX_BINDING", std::to_string(dec->_texturesBinding), ~0);
    }
    return dec;
}

void
DecalDecorator::remove(osg::StateSet* stateSet)
{
    OE_HARD_ASSERT(stateSet);
    stateSet->removeAttribute(getType());
    auto* vp = VirtualProgram::get(stateSet);
    if (vp) vp->removeShader("oe_applyDecals");
}

void
DecalDecorator::apply(osg::State& state) const
{
    auto& leaves = _drawList->_perCamera.get(&state).leaves;

    //std::lock_guard<std::mutex> lock(_mutex);

    auto& gc = GLObjects::get(_globjects, state);
    if (!gc._ssbo) {
        gc._ssbo = GLBuffer::create_shared(GL_SHADER_STORAGE_BUFFER, state);
        gc._ssbo->bind();
        gc._ssbo->debugLabel("Decals", "Decal instance buffer");
        gc._ssbo->unbind();
    }

    if (leaves.empty() && gc._buffer.size() > 1) // there's always at least 1
        return;

    gc._buffer.resize(1);
    gc._buffer[0].count = (std::uint32_t)(leaves.size());

    auto& mvm = state.getModelViewMatrix();

    // update the buffer
    for (auto& leaf : leaves)
    {
        gc._buffer.emplace_back();
        GPUDecalInstance& instance = gc._buffer.back();
        instance.projMatrixVS = osg::Matrix::inverse(leaf.matrix * mvm);
        instance.halfX = 0.5f * leaf.size.x();
        instance.halfY = 0.5f * leaf.size.y();
        instance.halfZ = 0.5f * leaf.size.z();
        instance.textureIndex = leaf.texture ? (std::int32_t)_textures->add(leaf.texture) : -1;
    }

    // send it to the GPU
    gc._ssbo->uploadData(gc._buffer);

    // bind it for shader access
    gc._ssbo->bindBufferBase(_bufferBinding);

    // activate the decal texture arena
    _textures->apply(state);

    // reset the draw list. note: this is not multi-GC friendly yet.
    leaves.clear();
}

void
DecalDecorator::releaseGLObjects(osg::State* state) const
{
    if (state) {
        auto& gc = GLObjects::get(_globjects, *state);
        gc._ssbo = nullptr;
    }
    else {
        _globjects.clear();
    }
}


DecalGroup::DecalGroup(DecalDecorator* decorator)
{
    OE_SOFT_ASSERT(decorator);
    if (decorator)
        _drawList = decorator->_drawList;
}
