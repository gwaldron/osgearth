/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
* MIT License
*/
#include "GrassLayer"
#include "ProceduralShaders"

#define LC "[GrassLayer] " << getName() << ": "

// If we're not using the GS, we have the option of using instancing or not.
// OFF benchmarks faster on older cards. On newer cards (RTX 2070 e.g.)
// it's slightly faster than using a giant DrawElementsUInt.
#define USE_INSTANCING_IN_VERTEX_SHADER

using namespace osgEarth;
using namespace osgEarth::Procedural;

REGISTER_OSGEARTH_LAYER(grass, GrassLayer);

Config
GrassLayer::Options::getConfig() const
{
    Config conf = GroundCoverLayer::Options::getConfig();
    return conf;
}

void
GrassLayer::Options::fromConfig(const Config& conf)
{
    //nop
}

void
GrassLayer::init()
{
    GroundCoverLayer::init();

    // custom alpha discard for grass
    options().maxAlpha().setDefault(0.75f);

    // custom LOD for grass
    options().lod() = 19u;

    // no shadow casting
    options().castShadows().setDefault(false);

    // small default spacing
    options().spacing().setDefault(Distance(0.55, Units::METERS)); // m

    // smooth picking for grouped instances of grass (3=clumpy)
    //getOrCreateStateSet()->setDefine("OE_GROUNDCOVER_PICK_NOISE_TYPE", "3");

    getOrCreateStateSet()->setMode(GL_BLEND, 0);
}

void
GrassLayer::loadRenderingShaders(VirtualProgram* vp, const osgDB::Options* options) const
{
    GroundCoverShaders s;
    s.load(vp, s.Grass, options);
}

osg::Node*
GrassLayer::createParametricGeometry(
    std::vector<osg::Texture*>& textures) const
{
    constexpr unsigned vertsPerInstance = 16;
    constexpr unsigned indiciesPerInstance = 54;

    osg::Geometry* out_geom = new osg::Geometry();
    out_geom->setUseVertexBufferObjects(true);
    out_geom->setUseDisplayList(false);

    static const GLushort indices[indiciesPerInstance] = {
        0,1,4, 4,1,5, 1,2,5, 5,2,6, 2,3,6, 6,3,7,
        4,5,8, 8,5,9, 5,6,9, 9,6,10, 6,7,10, 10,7,11,
        8,9,12, 12,9,13, 9,10,13, 13,10,14, 10,11,14, 14,11,15
    };

    out_geom->addPrimitiveSet(new osg::DrawElementsUShort(GL_TRIANGLES, indiciesPerInstance, &indices[0]));

    out_geom->setVertexArray(new osg::Vec3Array(osg::Array::BIND_PER_VERTEX, vertsPerInstance));

    osg::Vec3Array* normals = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX, vertsPerInstance);
    normals->assign(vertsPerInstance, osg::Vec3(0, 1, 0));
    out_geom->setNormalArray(normals);

    osg::StateSet* ss = out_geom->getOrCreateStateSet();
    if (textures.size() > 0)
        ss->setTextureAttribute(0, textures[0], 1);
    if (textures.size() > 1)
        ss->setTextureAttribute(1, textures[1], 1);

    //osg::ShortArray* handles = new osg::ShortArray(vertsPerInstance);
    //handles->setBinding(osg::Array::BIND_PER_VERTEX);
    //handles->assign(vertsPerInstance, tex_index_0);
    //out_geom->setVertexAttribArray(6, handles);

    return out_geom;
}