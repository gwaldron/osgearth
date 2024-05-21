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

#include <osg/PagedLOD>
#include <osg/ProxyNode>
#include <osg/PositionAttitudeTransform>

#define LC "[ModelLayer] " << getName() << " : "

using namespace osgEarth;

//------------------------------------------------------------------------

Config
ModelLayer::Options::getConfig() const
{
    Config conf = VisibleLayer::Options::getConfig();

    conf.set("url", url());
    conf.set("position", position());
    conf.set("orientation", orientation());
    conf.set("min_pixels", minPixels());

    conf.set("shader_policy", "disable", shaderPolicy(), SHADERPOLICY_DISABLE);
    conf.set("shader_policy", "inherit", shaderPolicy(), SHADERPOLICY_INHERIT);
    conf.set("shader_policy", "generate", shaderPolicy(), SHADERPOLICY_GENERATE);

    conf.set( "lighting", lightingEnabled() );

    return conf;
}

void
ModelLayer::Options::fromConfig( const Config& conf )
{
    conf.get("url", url());
    conf.get("position", position());
    conf.get("location", position());
    conf.get("orientation", orientation());
    conf.get("min_pixels", minPixels());

    conf.get("shader_policy", "disable", shaderPolicy(), SHADERPOLICY_DISABLE);
    conf.get("shader_policy", "inherit", shaderPolicy(), SHADERPOLICY_INHERIT);
    conf.get("shader_policy", "generate", shaderPolicy(), SHADERPOLICY_GENERATE);

    conf.get( "lighting", lightingEnabled());
}

namespace
{
    class SetDBOptionsVisitor : public osg::NodeVisitor
    {
    private:
        osg::ref_ptr<osgDB::Options> _dbOptions;

    public:
        SetDBOptionsVisitor(const osgDB::Options* dbOptions)
        {
            setTraversalMode(TRAVERSE_ALL_CHILDREN);
            setNodeMaskOverride(~0);
            _dbOptions = Registry::cloneOrCreateOptions(dbOptions);
        }

    public: // osg::NodeVisitor

        void apply(osg::PagedLOD& node)
        {
            node.setDatabaseOptions(_dbOptions.get());
            traverse(node);
        }

        void apply(osg::ProxyNode& node)
        {
            node.setDatabaseOptions(_dbOptions.get());
            traverse(node);
        }
    };
}

//........................................................................

REGISTER_OSGEARTH_LAYER(model, ModelLayer);

void
ModelLayer::setURL(const URI& value)
{
    options().url() = value;
}

const URI&
ModelLayer::getURL() const
{
    return options().url().get();
}

void
ModelLayer::setShaderPolicy(const ShaderPolicy& value)
{
    options().shaderPolicy() = value;
}

const
ShaderPolicy& ModelLayer::getShaderPolicy() const
{
    return options().shaderPolicy().get();
}

void
ModelLayer::setPosition(const GeoPoint& value)
{
    options().position() = value;

    auto xform = findNode<GeoTransform>(getNode());
    if (xform)
    {
        xform->setPosition(value);
    }
}

const GeoPoint&
ModelLayer::getPosition() const
{
    return options().position().value();
}

void
ModelLayer::setOrientationHPR(const osg::Vec3& hpr)
{
    options().orientation() = hpr;

    auto pat = findNode<osg::PositionAttitudeTransform>(getNode());
    if (pat)
    {
        osg::Matrix rot_mat;
        rot_mat.makeRotate(
            osg::DegreesToRadians(options().orientation()->y()), osg::Vec3(1, 0, 0),
            osg::DegreesToRadians(options().orientation()->x()), osg::Vec3(0, 0, 1),
            osg::DegreesToRadians(options().orientation()->z()), osg::Vec3(0, 1, 0));
        pat->setAttitude(rot_mat.getRotate());
    }
}

const osg::Vec3&
ModelLayer::getOrientationHPR() const
{
    return options().orientation().value();
}



void
ModelLayer::init()
{
    super::init();
    _root = new osg::Group();
    _root->setName(getName());
}

Status
ModelLayer::openImplementation()
{
    Status parentStatus = super::openImplementation();
    if (parentStatus.isError())
        return parentStatus;

    // Do we have a model URL to load?
    if (options().url().isSet())
    {
        osg::ref_ptr<osgDB::Options> localReadOptions =
            Registry::instance()->cloneOrCreateOptions(getReadOptions());

        // Add the URL to the file search path for relative-path paging support.
        localReadOptions->getDatabasePathList().push_back(
            osgDB::getFilePath(options().url()->full()));

        osg::ref_ptr<osg::Group> modelParent;

        // Apply the location and orientation, if available:
        GeoTransform* geo = nullptr;
        osg::PositionAttitudeTransform* pat = nullptr;

        if (options().orientation().isSet())
        {
            pat = new osg::PositionAttitudeTransform();
            osg::Matrix rot_mat;
            rot_mat.makeRotate(
                osg::DegreesToRadians(options().orientation()->y()), osg::Vec3(1, 0, 0),
                osg::DegreesToRadians(options().orientation()->x()), osg::Vec3(0, 0, 1),
                osg::DegreesToRadians(options().orientation()->z()), osg::Vec3(0, 1, 0));
            pat->setAttitude(rot_mat.getRotate());
            modelParent = pat;
        }

        if (options().position().isSet())
        {
            geo = new GeoTransform();
            geo->setPosition(options().position().get());
            if (pat)
                geo->addChild(pat);
            else
                modelParent = geo;
        }

        float minRange = options().minVisibleRange().getOrUse(0.0f);
        float maxRange = options().maxVisibleRange().getOrUse(FLT_MAX);

        PagedNode2* plod = new PagedNode2();

        URI uri = options().url().get();
        auto localOptions = options();

        auto loader = [localOptions, localReadOptions](Cancelable*)
            {
                osg::ref_ptr<osg::Node> node = localOptions.url()->getNode(localReadOptions.get());
                if (node.valid())
                {
                    if (localOptions.shaderPolicy() == SHADERPOLICY_GENERATE)
                    {
                        osg::ref_ptr<StateSetCache> cache = new StateSetCache();
                        Registry::shaderGenerator().run(node, localOptions.url()->base(), cache.get());
                    }
                    else if (localOptions.shaderPolicy() == SHADERPOLICY_DISABLE)
                    {
                        node->getOrCreateStateSet()->setAttributeAndModes(
                            new osg::Program(),
                            osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
                    }

                    // apply the DB options if there are any, so that deferred nodes like PagedLOD et al
                    // will inherit the loading options.
                    if (localReadOptions.valid())
                    {
                        SetDBOptionsVisitor setDBO(localReadOptions.get());
                        node->accept(setDBO);
                    }
                }

                return node;
            };

        plod->setLoadFunction(loader);

        if (options().minPixels().isSet())
        {
            plod->setMinPixels(options().minPixels().value());
        }
        else
        {
            plod->setMinRange(minRange);
            plod->setMaxRange(maxRange);
        }

        // Only supports GeoPoint ALTMODE_ABSOLUTE.
        osg::Vec3d center;
        if (options().position().isSet())
            options().position()->toWorld(center);

        plod->setCenter(center);
        plod->setRadius(maxRange);

        if (modelParent.valid())
            modelParent->addChild(plod);

        auto result = modelParent.valid() ? modelParent : plod;

        _root->addChild(result);
    }

    return Status::NoError;
}

osg::Node*
ModelLayer::getNode() const
{
    return _root.get();
}
