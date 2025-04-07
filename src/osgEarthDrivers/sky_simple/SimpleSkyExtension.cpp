/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include "SimpleSkyOptions"
#include "SimpleSkyNode"
#include <osgDB/FileNameUtils>
#include <osgEarth/Map>
#include <osgEarth/MapNode>
#include <osgEarth/NodeUtils>
#include <osgEarth/ExampleResources>

#define LC "[SimpleSkyDriver] "

using namespace osgEarth::Util;

namespace osgEarth { namespace SimpleSky
{
    class SimpleSkyExtension : public Extension,
        public ExtensionInterface<MapNode>,
        public ExtensionInterface<osg::View>,
        public SimpleSkyOptions,
        public SkyNodeFactory
    {
    public:
        META_OE_Extension(osgEarth, SimpleSkyExtension, simple_sky);

        // CTORs
        SimpleSkyExtension() { }

        SimpleSkyExtension(const ConfigOptions& options) :
            SimpleSkyOptions(options) { }

    public: // Extension

        const ConfigOptions& getConfigOptions() const { return *this; }


    public: // ExtensionInterface<MapNode>

        bool connect(MapNode* mapNode) override
        {
            _skynode = createSkyNode();
            if (mapNode->getMapSRS()->isProjected())
            {
                GeoPoint refPoint = 
                    mapNode->getMap()->getProfile()->getExtent().getCentroid();
                _skynode->setReferencePoint(refPoint);
            }                
            osgEarth::insertParent(_skynode.get(), mapNode);
            return true;
        }

        bool disconnect(MapNode* mapNode) override
        {
            osgEarth::removeGroup(_skynode.get());
            _skynode = 0L;
            return true;
        }

    public: // ExtensionInterface<osg::View>

        bool connect(osg::View* view) override
        {
            if (view && _skynode.valid())
            {
                _skynode->attach(view, 0);
            }
            return true;
        }

        bool disconnect(osg::View* view) override
        {
            //todo
            return true;
        }

    public: // SkyNodeFactory

        SkyNode* createSkyNode() override {
            return new SimpleSkyNode(*this);
        }


    protected: // Object

        // DTOR
        virtual ~SimpleSkyExtension() { }


    private:
        osg::ref_ptr<SkyNode> _skynode;
    };

    REGISTER_OSGEARTH_EXTENSION(osgearth_sky_simple, SimpleSkyExtension)

} } // namespace osgEarth::SimpleSky
