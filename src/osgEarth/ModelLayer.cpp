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
#include <osgEarth/ModelLayer>
#include <osgEarth/GLUtils>
#include <osgEarth/GeoTransform>
#include <osgEarth/Registry>
#include <osgEarth/URI>

#include <osg/CullStack>
#include <osg/Depth>
#include <osg/PositionAttitudeTransform>
#include <osg/PagedLOD>
#include <osg/ProxyNode>

#define LC "[ModelLayer] " << getName() << " : "

using namespace osgEarth;

//------------------------------------------------------------------------

Config
ModelLayer::Options::getConfig() const
{
    Config conf = VisibleLayer::Options::getConfig();

    conf.set("url", _url);
    conf.set("lod_scale", _lodScale);
    conf.set("location", _location);
    conf.set("orientation", _orientation);
    conf.set("loading_priority_scale", _loadingPriorityScale);
    conf.set("loading_priority_offset", _loadingPriorityOffset);
    conf.set("paged", _paged);

    conf.set("shader_policy", "disable", _shaderPolicy, SHADERPOLICY_DISABLE);
    conf.set("shader_policy", "inherit", _shaderPolicy, SHADERPOLICY_INHERIT);
    conf.set("shader_policy", "generate", _shaderPolicy, SHADERPOLICY_GENERATE);

    conf.set( "lighting",       _lightingEnabled );
    conf.set( "mask_min_level", _maskMinLevel );

    if ( driver().isSet() )
        conf.merge(driver()->getConfig());

    return conf;
}

void
ModelLayer::Options::fromConfig( const Config& conf )
{
    _lightingEnabled.init(true);
    _maskMinLevel.init(0);
    _lodScale.init(1.0f);
    _shaderPolicy.init(SHADERPOLICY_GENERATE);
    _loadingPriorityScale.init(1.0f);
    _loadingPriorityOffset.init(0.0f);
    _paged.init(false);

    conf.get("url", _url);
    conf.get("lod_scale", _lodScale);
    conf.get("location", _location);
    conf.get("position", _location);
    conf.get("orientation", _orientation);
    conf.get("loading_priority_scale", _loadingPriorityScale);
    conf.get("loading_priority_offset", _loadingPriorityOffset);
    conf.get("paged", _paged);

    conf.get("shader_policy", "disable", _shaderPolicy, SHADERPOLICY_DISABLE);
    conf.get("shader_policy", "inherit", _shaderPolicy, SHADERPOLICY_INHERIT);
    conf.get("shader_policy", "generate", _shaderPolicy, SHADERPOLICY_GENERATE);

    conf.get( "lighting",       _lightingEnabled );
    conf.get( "mask_min_level", _maskMinLevel );

    if ( conf.hasValue("driver") )
        driver() = ModelSourceOptions(conf);
}

//------------------------------------------------------------------------

namespace
{
    class LODScaleOverrideNode : public osg::Group
    {
    public:
        LODScaleOverrideNode() : m_lodScale(1.0f) {}
        virtual ~LODScaleOverrideNode() {}
    public:
        void setLODScale(float scale) { m_lodScale = scale; }
        float getLODScale() const { return m_lodScale; }

        virtual void traverse(osg::NodeVisitor& nv)
        {
            if(nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR)
            {
                osg::CullStack* cullStack = dynamic_cast<osg::CullStack*>(&nv);
                if(cullStack)
                {
                    float oldLODScale = cullStack->getLODScale();
                    cullStack->setLODScale(oldLODScale * m_lodScale);
                    osg::Group::traverse(nv);
                    cullStack->setLODScale(oldLODScale);
                }
                else
                    osg::Group::traverse(nv);
            }
            else
                osg::Group::traverse(nv);
        }

    private:
        float m_lodScale;
    };

    class SetLoadPriorityVisitor : public osg::NodeVisitor
    {
    public:
        SetLoadPriorityVisitor(float scale=1.0f, float offset=0.0f)
            : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
            , m_scale(scale)
            , m_offset(offset)
        {
            setNodeMaskOverride( ~0 );
        }

        virtual void apply(osg::PagedLOD& node)
        {
            for(unsigned n = 0; n < node.getNumFileNames(); n++)
            {
                float old;
                old = node.getPriorityScale(n);
                node.setPriorityScale(n, old * m_scale);
                old = node.getPriorityOffset(n);
                node.setPriorityOffset(n, old + m_offset);
            }
            traverse(node);
        }

    private:
        float m_scale;
        float m_offset;
    };

    /**
     * Visitor that sets the DBOptions on deferred-loading nodes.
     */
    class SetDBOptionsVisitor : public osg::NodeVisitor
    {
    private:
        osg::ref_ptr<osgDB::Options> _dbOptions;

    public:
        SetDBOptionsVisitor(const osgDB::Options* dbOptions)
        {
            setTraversalMode( TRAVERSE_ALL_CHILDREN );
            setNodeMaskOverride( ~0 );
            _dbOptions = Registry::cloneOrCreateOptions( dbOptions );                
        }

    public: // osg::NodeVisitor

        void apply(osg::PagedLOD& node)
        {
            node.setDatabaseOptions( _dbOptions.get() );
            traverse(node);
        }

        void apply(osg::ProxyNode& node)
        {
            node.setDatabaseOptions( _dbOptions.get() );
            traverse(node);
        }
    };
}

//........................................................................

REGISTER_OSGEARTH_LAYER(model, ModelLayer);

OE_LAYER_PROPERTY_IMPL(ModelLayer, URI, URL, url);
OE_LAYER_PROPERTY_IMPL(ModelLayer, float, LODScale, lodScale);
OE_LAYER_PROPERTY_IMPL(ModelLayer, bool, Paged, paged);
OE_LAYER_PROPERTY_IMPL(ModelLayer, GeoPoint, Location, location);
OE_LAYER_PROPERTY_IMPL(ModelLayer, osg::Vec3, Orientation, orientation);
OE_LAYER_PROPERTY_IMPL(ModelLayer, unsigned, MaskMinLevel, maskMinLevel);
OE_LAYER_PROPERTY_IMPL(ModelLayer, ShaderPolicy, ShaderPolicy, shaderPolicy);

ModelLayer::~ModelLayer()
{
    //nop
}

void
ModelLayer::init()
{
    VisibleLayer::init();
    installDefaultOpacityShader();
    _root = new osg::Group();
    _root->setName(getName());
}

Status
ModelLayer::openImplementation()
{
    Status parentStatus = VisibleLayer::openImplementation();
    if (parentStatus.isError())
        return parentStatus;

    // Do we need to load a model source?
    if (!_modelSource.valid() && options().driver().isSet())
    {
        std::string driverName = options().driver()->getDriver();

        OE_INFO << LC << "Opening; driver=\"" << driverName << "\"" << std::endl;
        
        Status status;

        // Try to create the model source:
        _modelSource = ModelSourceFactory::create( options().driver().get() );
        if ( _modelSource.valid() )
        {
            _modelSource->setName( this->getName() );
            const Status& modelStatus = _modelSource->open(getReadOptions());
            return modelStatus;
        }
        else
        {
            return Status(Status::ServiceUnavailable, Stringify() << "Failed to create driver \"" << driverName << "\"");
        }
    }

    // Do we have a model URL to load?
    else if (!_modelSource.valid() && options().url().isSet())
    {
        osg::ref_ptr<osgDB::Options> localReadOptions =
            Registry::instance()->cloneOrCreateOptions(getReadOptions());
        
        // Add the URL to the file search path for relative-path paging support.
        localReadOptions->getDatabasePathList().push_back(
            osgDB::getFilePath(options().url()->full()) );
            
        // Only support paging if user has enabled it and provided a min/max range
        bool paged = 
            (options().paged() == true) &&
            (options().minVisibleRange().isSet() || options().maxVisibleRange().isSet());

        osg::ref_ptr<osg::Node> result;
        osg::ref_ptr<osg::Node> modelNode;
        osg::ref_ptr<osg::Group> modelNodeParent;

        // If we're not paging, just load the node now:
        if (!paged)
        {
            ReadResult rr = options().url()->readNode(localReadOptions.get());
            if (rr.failed())
            {
                return Status(Status::ResourceUnavailable,
                    Stringify() << "Failed to load model from URL ("<<rr.errorDetail()<<")");
            }
            modelNode = rr.getNode();
        }

        // Apply the location and orientation, if available:
        GeoTransform* geo = 0L;
        osg::PositionAttitudeTransform* pat = 0L;

        if (options().orientation().isSet())
        {
            pat = new osg::PositionAttitudeTransform();
            osg::Matrix rot_mat;
            rot_mat.makeRotate( 
                osg::DegreesToRadians(options().orientation()->y()), osg::Vec3(1,0,0),
                osg::DegreesToRadians(options().orientation()->x()), osg::Vec3(0,0,1),
                osg::DegreesToRadians(options().orientation()->z()), osg::Vec3(0,1,0) );
            pat->setAttitude(rot_mat.getRotate());
            modelNodeParent = pat;
        }

        if (options().location().isSet())
        {
            geo = new GeoTransform();
            geo->setPosition(options().location().get());
            if (pat)
                geo->addChild(pat);

            modelNodeParent = geo;
        }

        result = modelNodeParent.get();
        
        if ( options().minVisibleRange().isSet() || options().maxVisibleRange().isSet() )
        {                
            float minRange = options().minVisibleRange().getOrUse(0.0f);
            float maxRange = options().maxVisibleRange().getOrUse(FLT_MAX);

            osg::Group* group = nullptr;

            if (!paged)
            {
                // Just use a regular LOD
                osg::LOD* lod = new osg::LOD();
                lod->addChild(modelNode.release());
                lod->setRange(0, minRange, maxRange);
                group = lod;
            }
            else
            {
                PagedNode2* plod = new PagedNode2();

                URI uri = options().url().get();

                plod->setLoadFunction([uri, localReadOptions](Cancelable*) {
                    osg::ref_ptr<osg::Node> node = uri.getNode(localReadOptions.get());
                    ShaderGenerator gen;
                    node->accept(gen);
                    return node;
                });

                plod->setMinRange(minRange);
                plod->setMaxRange(maxRange);

                osg::Vec3d center;
                if (options().location().isSet())
                {
                    options().location()->toWorld(center);
                }
                else
                {
                    center = result->getBound().center();
                }
                plod->setCenter(center);

                if (result.valid())
                {
                    plod->setRadius(std::max(result->getBound().radius(), maxRange));
                }
                else
                {
                    plod->setRadius(maxRange);
                }

                group = plod;
            }

            modelNodeParent->addChild(group);
        }
        else
        {
            // Simply add the node to the matrix transform
            if (modelNode.valid() && modelNodeParent.valid())
            {            
                modelNodeParent->addChild(modelNode.get());
            }
        }

        if (!result.valid())
        {
            result = modelNode.get();
        }

        if (result.valid())
        {
            if(options().loadingPriorityScale().isSet() || options().loadingPriorityOffset().isSet())
            {
                SetLoadPriorityVisitor slpv(options().loadingPriorityScale().value(), options().loadingPriorityOffset().value());
                result->accept(slpv);
            }
    
            if(options().lodScale().isSet())
            {
                LODScaleOverrideNode * node = new LODScaleOverrideNode;
                node->setLODScale(options().lodScale().value());
                node->addChild(result.release());
                result = node;
            }

            if ( options().shaderPolicy() == SHADERPOLICY_GENERATE )
            {
                osg::ref_ptr<StateSetCache> cache = new StateSetCache();

                Registry::shaderGenerator().run(
                    result.get(),
                    options().url()->base(),
                    cache.get() );
            }
            else if ( options().shaderPolicy() == SHADERPOLICY_DISABLE )
            {
                result->getOrCreateStateSet()->setAttributeAndModes(
                    new osg::Program(),
                    osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE );
            }

            // apply the DB options if there are any, so that deferred nodes like PagedLOD et al
            // will inherit the loading options.
            if ( localReadOptions.valid() )
            {
                SetDBOptionsVisitor setDBO( localReadOptions.get() );
                result->accept( setDBO );
            }
        }

        if (result.valid())
        {
            setNode(result.release());
        }
    }

    return Status::NoError;
}

std::string
ModelLayer::getCacheID() const
{
    // Customized from the base class to use the driver() config instead of full config:
    std::string binID;
    if (options().cacheId().isSet() && !options().cacheId()->empty())
    {
        binID = options().cacheId().get();
    }
    else
    {
        Config conf = options().driver()->getConfig();
        binID = hashToString(conf.toJSON(false));
    }
    return binID;
}

void
ModelLayer::addedToMap(const Map* map)
{
    VisibleLayer::addedToMap(map);

    if (getStatus().isError())
        return;

    // Using a model source?
    if ( _modelSource.valid() )
    {
        // reset the scene graph.
        while (_root->getNumChildren() > 0)
        {
            getSceneGraphCallbacks()->fireRemoveNode(_root->getChild(0));
            _root->removeChildren(0, 1);
        }

        // Share the scene graph callbacks with the model source:
        _modelSource->setSceneGraphCallbacks(getSceneGraphCallbacks());

        // Create the scene graph from the mode source:
        osg::ref_ptr<osg::Node> node = _modelSource->createNode(map, 0L);
        if (node.valid())
        {
            if ( options().lightingEnabled().isSet() )
            {
                setLightingEnabled( options().lightingEnabled().get() );
            }

            _modelSource->sync( _modelSourceRev );

            // Handle disabling depth testing
            if ( _modelSource->getOptions().depthTestEnabled() == false )
            {
                osg::StateSet* ss = node->getOrCreateStateSet();
                ss->setAttributeAndModes( new osg::Depth( osg::Depth::ALWAYS ) );              
                ss->setRenderBinDetails( 99999, "RenderBin" ); //TODO: configure this bin ...
            }

            // enfore a rendering bin if necessary:
            if (_modelSource->getOptions().renderOrder().isSet())
            {
                osg::StateSet* ss = node->getOrCreateStateSet();
                ss->setRenderBinDetails(
                    _modelSource->getOptions().renderOrder().value(),
                    ss->getBinName().empty() ? "DepthSortedBin" : ss->getBinName());
            }

            if (_modelSource->getOptions().renderBin().isSet())
            {
                osg::StateSet* ss = node->getOrCreateStateSet();
                ss->setRenderBinDetails(
                    ss->getBinNumber(),
                    _modelSource->getOptions().renderBin().get());
            }

            _root->addChild(node.get());
        }
    }
}

void
ModelLayer::removedFromMap(const Map* map)
{
    // dispose of the scene graph.
    while (_root->getNumChildren() > 0)
    {
        getSceneGraphCallbacks()->fireRemoveNode(_root->getChild(0));
        _root->removeChildren(0, 1);
    }
}

void
ModelLayer::setNode(osg::Node* node)
{
    _root->removeChildren(0, _root->getNumChildren());
    if (node)
    {
        _root->addChild(node);
        setStatus(Status::OK());
    }
}

osg::Node*
ModelLayer::getNode() const
{
    return _root.get();
}

void
ModelLayer::setLightingEnabled( bool value )
{
    options().lightingEnabled() = value;

    GLUtils::setLighting(
        _root->getOrCreateStateSet(),
        value ? osg::StateAttribute::ON : 
        (osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED) );
}

bool
ModelLayer::isLightingEnabled() const
{
    return options().lightingEnabled().get();
}

void
ModelLayer::fireCallback(ModelLayerCallback::MethodPtr method)
{
    for(CallbackVector::const_iterator i = _callbacks.begin(); i != _callbacks.end(); ++i )
    {
        ModelLayerCallback* cb = dynamic_cast<ModelLayerCallback*>(i->get());
        if (cb) (cb->*method)( this );
    }
}
