/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
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
#include <osgEarth/ShaderComposition>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/TextureCompositor>
#include <osgEarth/Registry>
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
    MapNode* findMapNode(osg::Group* node)
    {
        return findTopMostNodeOfType<MapNode>(node);
    }

    osgShadow::ViewDependentShadowMap*
        getTechniqueAsVdsm(osgShadow::ShadowedScene* sscene)
    {
        osgShadow::ShadowTechnique* st = sscene->getShadowTechnique();
        return dynamic_cast<osgShadow::ViewDependentShadowMap*>(st);
    }

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
                osgShadow::ViewDependentShadowMap* vdsm
                    = getTechniqueAsVdsm(sscene);
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

    MapNode* mapNode = findMapNode(root);
    TerrainEngineNode* engine = mapNode->getTerrainEngine();
    if (!engine)
        return false;

    TextureCompositor* compositor = engine->getTextureCompositor();
    int su = -1;
    if (!compositor->reserveTextureImageUnit(su))
        return false;

    OE_INFO << LC << "Reserved texture unit " << su << " for shadowing" << std::endl;

    osgShadow::ViewDependentShadowMap* vdsm = getTechniqueAsVdsm(sscene);
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

    // create a virtual program to attach to the shadows scene.
    VirtualProgram* vp = new VirtualProgram();
    vp->setName( "shadow:terrain" );
    vp->installDefaultColoringAndLightingShaders();

    ssStateSet->setAttributeAndModes( vp, 1 );


    std::stringstream buf;
    buf << "#version " << GLSL_VERSION_STR << "\n";
    buf << "varying vec4 colorAmbientEmissive;\n";
    buf << "varying vec4 shadow_TexCoord0;\n";
    if ( su1 >= 0 )
        buf << "varying vec4 shadow_TexCoord1;\n";


    buf << "void osgearth_vert_setupShadowCoords()\n";
    buf << "{\n";
    buf << "    vec4 position4 = gl_ModelViewMatrix * gl_Vertex;\n";
    buf << "    shadow_TexCoord0.s = dot( position4, gl_EyePlaneS[" << su <<"]);\n";
    buf << "    shadow_TexCoord0.t = dot( position4, gl_EyePlaneT[" << su <<"]);\n";
    buf << "    shadow_TexCoord0.p = dot( position4, gl_EyePlaneR[" << su <<"]);\n";
    buf << "    shadow_TexCoord0.q = dot( position4, gl_EyePlaneQ[" << su <<"]);\n";
    if (su1 >= 0)
    {
        buf << "    shadow_TexCoord1.s = dot( position4, gl_EyePlaneS[" << su1 <<"]);\n";
        buf << "    shadow_TexCoord1.t = dot( position4, gl_EyePlaneT[" << su1 <<"]);\n";
        buf << "    shadow_TexCoord1.p = dot( position4, gl_EyePlaneR[" << su1 <<"]);\n";
        buf << "    shadow_TexCoord1.q = dot( position4, gl_EyePlaneQ[" << su1 <<"]);\n";
    }
    buf << "    colorAmbientEmissive = gl_FrontLightModelProduct.sceneColor\n";
    buf << "                         + gl_FrontLightProduct[0].ambient;\n";
    //buf << "    colorAmbientEmissive = gl_LightModel.ambient + gl_FrontLightProduct[0].ambient; \n";
    //buf << "    colorAmbientEmissive = gl_FrontLightProduct[0].ambient; \n";
    buf << "}\n";

    std::string setupShadowCoords;
    setupShadowCoords = buf.str();

    vp->setFunction(
        "osgearth_vert_setupShadowCoords", 
        setupShadowCoords, 
        ShaderComp::LOCATION_VERTEX_POST_LIGHTING,
        -1.0 );

    std::stringstream buf2;
    buf2 <<
        "#version " << GLSL_VERSION_STR << "\n"
        "uniform sampler2DShadow shadowTexture;\n"
        "varying vec4 shadow_TexCoord0;\n";

    if (su1 >= 0)
    {
        // bound by vdsm
        buf2 << "uniform sampler2DShadow shadowTexture1;\n";
        buf2 << "varying vec4 shadow_TexCoord1;\n";
    }
    buf2 <<
        "varying vec4 colorAmbientEmissive;\n"
        "varying vec4 osg_FrontColor;\n"
        "varying vec4 osg_FrontSecondaryColor;\n"
        "void osgearth_frag_applyLighting( inout vec4 color )\n"
        "{\n"
        "    float alpha = color.a;\n"
        "    float shadowFac = shadow2DProj( shadowTexture, shadow_TexCoord0).r;\n";
    if (su1 > 0)
    {
        buf2 << "    shadowFac *= shadow2DProj( shadowTexture1, shadow_TexCoord1).r;\n";
    }
    buf2 <<
        "    vec4 diffuseLight = mix(colorAmbientEmissive, osg_FrontColor, shadowFac);\n"
        "    color = color * diffuseLight + osg_FrontSecondaryColor * shadowFac;\n"
        "    color.a = alpha;\n"
        "}\n";

    std::string fragApplyLighting;
    fragApplyLighting = buf2.str();

    // override the terrain engine's default lighting shader:
    vp->setShader("osgearth_frag_applyLighting",
        new osg::Shader(osg::Shader::FRAGMENT, fragApplyLighting),
        osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE );

    setShadowUnit(sscene, su);

    // VDSM uses a different sampler name, shadowTexture0.
    ssStateSet
        ->getOrCreateUniform("shadowTexture", osg::Uniform::SAMPLER_2D_SHADOW)
        ->set(su);

    return true;
}
