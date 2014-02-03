/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2013 Pelican Mapping
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
#include <osgEarthUtil/ShadowUtils>

#include <osgEarth/MapNode>
#include <osgEarth/NodeUtils>
#include <osgEarth/Notify>
#include <osgEarth/VirtualProgram>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/TextureCompositor>
#include <osgEarth/Registry>
#include <osgEarth/ShaderFactory>
#include <osgEarth/Capabilities>

#include <osg/StateSet>
#include <osgShadow/StandardShadowMap>
#include <osgShadow/ViewDependentShadowMap>

#include <sstream>

#define LC "[ShadowUtils] "

using namespace osgEarth;
using namespace osgEarth::Util;

namespace
{
    bool setShadowUnit(osgShadow::ShadowedScene* sscene, int unit)
    {
        osgShadow::ShadowTechnique* st = sscene->getShadowTechnique();
        if (st)
        {
            osgShadow::StandardShadowMap* ssm
                = dynamic_cast<osgShadow::StandardShadowMap*>(st);
            if (ssm)
            {
                ssm->setShadowTextureUnit( unit );
                ssm->setShadowTextureCoordIndex( unit );
                return true;
            }
            else
            {
                osgShadow::ViewDependentShadowMap* vdsm = dynamic_cast< osgShadow::ViewDependentShadowMap*>( st );
                if (vdsm)
                {
                    sscene->getShadowSettings()
                        ->setBaseShadowTextureUnit( unit );
                    return true;
                }
            }
        }

        return false;
    }
}


bool
ShadowUtils::setUpShadows(osgShadow::ShadowedScene* sscene, osg::Group* root)
{
    osg::StateSet* ssStateSet = sscene->getOrCreateStateSet();

    MapNode* mapNode = MapNode::findMapNode(root);
    TerrainEngineNode* engine = mapNode->getTerrainEngine();
    if (!engine)
        return false;

    TextureCompositor* compositor = engine->getTextureCompositor();
    int su = -1;
    if (!compositor->reserveTextureImageUnit(su))
        return false;

    OE_INFO << LC << "Reserved texture unit " << su << " for shadowing" << std::endl;

    osgShadow::ViewDependentShadowMap* vdsm =  dynamic_cast< osgShadow::ViewDependentShadowMap*>(sscene->getShadowTechnique());

    int su1 = -1;
    if (vdsm && sscene->getShadowSettings()->getNumShadowMapsPerLight() == 2)
    {
        if (!compositor->reserveTextureImageUnit(su1) || su1 != su + 1)
        {
            OE_FATAL << LC << "couldn't get contiguous shadows for split vdsm\n";
            sscene->getShadowSettings()->setNumShadowMapsPerLight(1);
            if (su1 != -1)
                compositor->releaseTextureImageUnit(su1);
            su1 = -1;
        }
        else
        {
            OE_INFO << LC << "Reserved texture unit " << su1 << " for shadowing" << std::endl;
        }
    }

    // create a virtual program to attach to the shadowed scene.
    VirtualProgram* vp = new VirtualProgram();
    vp->setName( "shadow:terrain" );
    Registry::shaderFactory()->installLightingShaders(vp);

    ssStateSet->setAttributeAndModes( vp, 1 );

    std::stringstream buf;
    buf <<
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"
        "varying float oe_shadow_ambient; \n"
        "varying vec4 oe_shadow_coord; \n"
        "void oe_shadow_vertex(inout vec4 VertexVIEW) \n"
        "{ \n"
        "    vec4 position4 = VertexVIEW; \n"
        "    oe_shadow_coord.s = dot( position4, gl_EyePlaneS[" << su <<"]); \n"
        "    oe_shadow_coord.t = dot( position4, gl_EyePlaneT[" << su <<"]); \n"
        "    oe_shadow_coord.p = dot( position4, gl_EyePlaneR[" << su <<"]); \n"
        "    oe_shadow_coord.q = dot( position4, gl_EyePlaneQ[" << su <<"]); \n"
        "    oe_shadow_ambient = 0.5; \n" //gl_FrontLightProduct[0].ambient; \n"
        "} \n";
    std::string vertex = buf.str();

    const char* fragment =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"
        "uniform sampler2DShadow oe_shadow_map; \n"
        "varying vec4 oe_shadow_coord; \n"
        "varying float oe_shadow_ambient; \n"
        "void oe_shadow_fragment( inout vec4 color )\n"
        "{\n"
        "    float alpha = color.a; \n"
        "    float shadowFac = shadow2DProj(oe_shadow_map, oe_shadow_coord).r; \n"
        "    vec4 colorInFullShadow = color * oe_shadow_ambient; \n"
        "    color = mix(colorInFullShadow, color, shadowFac); \n"
        "    color.a = alpha;\n"
        "}\n";

    vp->setFunction(
        "oe_shadow_vertex", 
        vertex, 
        ShaderComp::LOCATION_VERTEX_VIEW );

    vp->setFunction(
        "oe_shadow_fragment",
        fragment,
        ShaderComp::LOCATION_FRAGMENT_LIGHTING, 10.0f );

    setShadowUnit(sscene, su);

    // VDSM uses a different sampler name, shadowTexture0.
    ssStateSet
        ->getOrCreateUniform("oe_shadow_map", osg::Uniform::SAMPLER_2D_SHADOW)
        ->set(su);

    //ssStateSet
    //    ->getOrCreateUniform("shadowTexture", osg::Uniform::SAMPLER_2D_SHADOW)
    //    ->set(su);

    return true;
}
