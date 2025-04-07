/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/
#include <osgEarth/PhongLightingEffect>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/Shaders>
#include <osgEarth/Lighting>

using namespace osgEarth;
using namespace osgEarth::Util;


PhongLightingEffect::PhongLightingEffect()
{
    init();
}

PhongLightingEffect::PhongLightingEffect(osg::StateSet* stateset)
{
    init();
    attach( stateset );
}

void
PhongLightingEffect::init()
{
    _supported = Registry::capabilities().supportsGLSL();
}

PhongLightingEffect::~PhongLightingEffect()
{
    detach();
}

void
PhongLightingEffect::attach(osg::StateSet* stateset)
{
    if ( stateset && _supported )
    {
        _statesets.push_back(stateset);
        VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);
        vp->setName(typeid(*this).name());
        
        Shaders shaders;
        shaders.load(vp, shaders.PhongLighting);

        stateset->setDefine(OE_LIGHTING_DEFINE, osg::StateAttribute::ON);
        stateset->setDefine("OE_NUM_LIGHTS", "1");
    }
}

void
PhongLightingEffect::detach()
{
    if ( _supported )
    {
        for (StateSetList::iterator it = _statesets.begin(); it != _statesets.end(); ++it)
        {
            osg::ref_ptr<osg::StateSet> stateset;
            if ( (*it).lock(stateset) )
            {
                detach(stateset.get());
                (*it) = 0L;
            }
        }

        _statesets.clear();
    }
}

void
PhongLightingEffect::detach(osg::StateSet* stateset)
{
    if ( stateset && _supported )
    {
        //if ( _lightingUniform.valid() )
        //    stateset->removeUniform( _lightingUniform.get() );

        stateset->removeDefine(OE_LIGHTING_DEFINE);

        VirtualProgram* vp = VirtualProgram::get( stateset );
        if ( vp )
        {
            Shaders shaders;
            shaders.unload(vp, shaders.PhongLighting);
        }
    }
}



PhongLightingGroup::PhongLightingGroup()
{
    _effect = new PhongLightingEffect(getOrCreateStateSet());
    Lighting::installDefaultMaterial(getOrCreateStateSet());
}
