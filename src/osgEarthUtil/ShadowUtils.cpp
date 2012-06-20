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

#define LC "[ShadowUtils]"

#include <sstream>

#include <osg/StateSet>
#include <osgShadow/StandardShadowMap>
#include <osgShadow/ViewDependentShadowMap>

#include <osgEarthUtil/ShadowUtils>

#include <osgEarth/MapNode>
#include <osgEarth/NodeUtils>
#include <osgEarth/Notify>
#include <osgEarth/ShaderComposition>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/TextureCompositor>

namespace osgEarth
{
namespace Util
{

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

bool setUpShadows(osgShadow::ShadowedScene* sscene, osg::Group* root)
{
    MapNode* mapNode = findMapNode(root);
    TerrainEngineNode* engine = mapNode->getTerrainEngine();
    if (!engine)
        return false;
    TextureCompositor* compositor = engine->getTextureCompositor();
    int su = -1;
    if (!compositor->reserveTextureImageUnit(su))
        return false;
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
    }
    std::stringstream buf;
    buf << "varying vec4 colorAmbientEmissive;\n";
    buf << "void osgearth_setupShadowCoords()\n";
    buf << "{\n";
    buf << "    vec4 position4 = gl_ModelViewMatrix * gl_Vertex;\n";
    buf << "    gl_TexCoord[" << su << "].s = dot( position4, gl_EyePlaneS[" << su <<"]);\n";
    buf << "    gl_TexCoord[" << su << "].t = dot( position4, gl_EyePlaneT[" << su <<"]);\n";
    buf << "    gl_TexCoord[" << su << "].p = dot( position4, gl_EyePlaneR[" << su <<"]);\n";
    buf << "    gl_TexCoord[" << su << "].q = dot( position4, gl_EyePlaneQ[" << su <<"]);\n";
    if (su1 >= 0)
    {
        buf << "    gl_TexCoord[" << su1 << "].s = dot( position4, gl_EyePlaneS[" << su1 <<"]);\n";
        buf << "    gl_TexCoord[" << su1 << "].t = dot( position4, gl_EyePlaneT[" << su1 <<"]);\n";
        buf << "    gl_TexCoord[" << su1 << "].p = dot( position4, gl_EyePlaneR[" << su1 <<"]);\n";
        buf << "    gl_TexCoord[" << su1 << "].q = dot( position4, gl_EyePlaneQ[" << su1 <<"]);\n";
    }
    buf << "    colorAmbientEmissive = gl_FrontLightModelProduct.sceneColor\n";
    buf << "                    + gl_FrontLightProduct[0].ambient;\n";
    buf << "}\n";
    osg::StateSet* engineSS = engine->getStateSet();
    if (!engineSS)
        return false;
    VirtualProgram* vp
        = dynamic_cast<VirtualProgram*>(engineSS->getAttribute(osg::StateAttribute::PROGRAM));
    if (!vp)
        return false;
    vp->setFunction("osgearth_setupShadowCoords", buf.str(),
                    ShaderComp::LOCATION_VERTEX_POST_LIGHTING);
    std::stringstream buf2;
    buf2 <<
        "#version 110 \n"
        "uniform sampler2DShadow shadowTexture;\n";
    if (su1 >= 0)
    {
        // bound by vdsm
        buf2 << "uniform sampler2DShadow shadowTexture1;\n";
    }
    buf2 <<
        "varying vec4 colorAmbientEmissive;\n\n"
        "void osgearth_frag_applyLighting( inout vec4 color )\n"
        "{\n"
        "    float alpha = color.a;\n"
        "    float shadowFac = shadow2DProj( shadowTexture, gl_TexCoord["
            << su << "]).r;\n";
    if (su1 > 0)
    {
        buf2 << "    shadowFac *= shadow2DProj( shadowTexture1,"
            " gl_TexCoord[" << su1 << "]).r;\n";
    }
    buf2 <<
        "    vec4 diffuseLight = mix(colorAmbientEmissive, gl_Color, shadowFac);\n"
        "    color = color * diffuseLight + gl_SecondaryColor * shadowFac;\n"
        "    color.a = alpha;\n"
        "}\n";
    vp->setShader("osgearth_frag_applyLighting",
                  new osg::Shader(osg::Shader::FRAGMENT, buf2.str()));
    setShadowUnit(sscene, su);
    // VDSM uses a different sampler name, shadowTexture0.
    root->getOrCreateStateSet()
        ->getOrCreateUniform("shadowTexture", osg::Uniform::SAMPLER_2D)
        ->set(su);
    return true;
}
}
}
