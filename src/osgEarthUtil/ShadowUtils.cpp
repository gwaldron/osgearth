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
    //vp->installDefaultColoringAndLightingShaders();

    ssStateSet->setAttributeAndModes( vp, 1 );


    std::stringstream buf;
    buf << "#version " << GLSL_VERSION_STR << "\n";
#ifdef OSG_GLES2_AVAILABLE
    buf << "precision mediump float;\n";
#endif
    buf << "varying vec4 oe_shadow_ambient;\n";
    buf << "varying vec4 oe_shadow_TexCoord0;\n";
    if ( su1 >= 0 )
        buf << "varying vec4 oe_shadow_TexCoord1;\n";


    buf << "void oe_shadow_setupShadowCoords(inout vec4 VertexVIEW)\n";
    buf << "{\n";
    buf << "    vec4 position4 = VertexVIEW;\n";
    buf << "    oe_shadow_TexCoord0.s = dot( position4, gl_EyePlaneS[" << su <<"]);\n";
    buf << "    oe_shadow_TexCoord0.t = dot( position4, gl_EyePlaneT[" << su <<"]);\n";
    buf << "    oe_shadow_TexCoord0.p = dot( position4, gl_EyePlaneR[" << su <<"]);\n";
    buf << "    oe_shadow_TexCoord0.q = dot( position4, gl_EyePlaneQ[" << su <<"]);\n";
    if (su1 >= 0)
    {
        buf << "    oe_shadow_TexCoord1.s = dot( position4, gl_EyePlaneS[" << su1 <<"]);\n";
        buf << "    oe_shadow_TexCoord1.t = dot( position4, gl_EyePlaneT[" << su1 <<"]);\n";
        buf << "    oe_shadow_TexCoord1.p = dot( position4, gl_EyePlaneR[" << su1 <<"]);\n";
        buf << "    oe_shadow_TexCoord1.q = dot( position4, gl_EyePlaneQ[" << su1 <<"]);\n";
    }

    // the ambient lighting will control the intensity of the shadow.
    buf << "    oe_shadow_ambient = gl_FrontLightProduct[0].ambient; \n"
        << "}\n";

    std::string setupShadowCoords;
    setupShadowCoords = buf.str();

    vp->setFunction(
        "oe_shadow_setupShadowCoords", 
        setupShadowCoords, 
        ShaderComp::LOCATION_VERTEX_VIEW,
        -1.0 );

    std::stringstream buf2;
    buf2 <<
        "#version " << GLSL_VERSION_STR << "\n"
#ifdef OSG_GLES2_AVAILABLE
        "precision mediump float;\n"
#endif
        "uniform sampler2DShadow shadowTexture;\n"
        "varying vec4 oe_shadow_TexCoord0;\n";

    if (su1 >= 0)
    {
        // bound by vdsm
        buf2 << "uniform sampler2DShadow shadowTexture1;\n";
        buf2 << "varying vec4 oe_shadow_TexCoord1;\n";
    }
    buf2 <<
        "varying vec4 oe_shadow_ambient;\n"

        "void oe_shadow_applyLighting( inout vec4 color )\n"
        "{\n"
        "    float alpha = color.a;\n"
        "    float shadowFac = shadow2DProj( shadowTexture, oe_shadow_TexCoord0).r;\n";
    if (su1 > 0)
    {
        buf2 << "    shadowFac *= shadow2DProj( shadowTexture1, oe_shadow_TexCoord1).r;\n";
    }

    // calculate the shadowed color and mix if with the lit color based on the
    // ambient lighting. The 0.5 is a multiplier that darkens the shadow in
    // proportion to ambient light. It should probably be a uniform.
    buf2 <<
        "    vec4 colorInFullShadow = color * oe_shadow_ambient; \n"
        "    color = mix(colorInFullShadow, color, shadowFac); \n"
        "    color.a = alpha;\n"
        "}\n";

    std::string fragApplyLighting;
    fragApplyLighting = buf2.str();

    vp->setFunction(
        "oe_shadow_applyLighting",
        fragApplyLighting,
        osgEarth::ShaderComp::LOCATION_FRAGMENT_LIGHTING );

    setShadowUnit(sscene, su);

    // VDSM uses a different sampler name, shadowTexture0.
    ssStateSet
        ->getOrCreateUniform("shadowTexture", osg::Uniform::SAMPLER_2D_SHADOW)
        ->set(su);

    return true;
}
