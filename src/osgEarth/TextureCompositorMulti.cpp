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
#include <osgEarth/TextureCompositorMulti>
#include <osgEarth/ImageUtils>
#include <osgEarth/Registry>
#include <osgEarth/ShaderComposition>
#include <osgEarth/ShaderUtils>
#include <osgEarth/TileKey>
#include <osgEarth/StringUtils>
#include <osgEarth/Capabilities>
#include <osg/Texture2D>
#include <osg/TexEnv>
#include <osg/TexEnvCombine>
#include <vector>

using namespace osgEarth;

#define LC "[TextureCompositorMulti] "

//------------------------------------------------------------------------

namespace
{
    static std::string makeSamplerName(int slot)
    {
        return Stringify() << "tex" << slot;
    }

    static osg::Shader*
    s_createTextureVertexShader( const TextureLayout& layout, bool blending )
    {
        std::stringstream buf;
       
        const TextureLayout::TextureSlotVector& slots = layout.getTextureSlots();

        buf << "#version 110 \n";

        if ( blending )
        {
            buf << "uniform mat4 osgearth_TexBlendMatrix[" << slots.size() << "];\n";
        }

        buf << "void osgearth_vert_setupTexturing() \n"
            << "{ \n";

        // Set up the texture coordinates for each active slot (primary and secondary).
        // Primary slots are the actual image layer's texture image unit. A Secondary
        // slot is what an image layer uses for LOD blending, and is set on a per-layer basis.
        for( int slot = 0; slot < (int)slots.size(); ++slot )
        {
            if ( slots[slot] >= 0 )
            {
                UID uid = slots[slot];
                int primarySlot = layout.getSlot(uid, 0);

                if ( slot == primarySlot )
                {
                    // normal unit:
                    buf << "    gl_TexCoord["<< slot <<"] = gl_MultiTexCoord" << slot << ";\n";
                }
                else
                {
                    // secondary (blending) unit:
                    buf << "    gl_TexCoord["<< slot <<"] = osgearth_TexBlendMatrix["<< primarySlot << "] * gl_MultiTexCoord" << primarySlot << ";\n";
                }
            }
        }
            
        buf << "} \n";

        std::string str;
        str = buf.str();
        return new osg::Shader( osg::Shader::VERTEX, str );
    }

    static osg::Shader*
    s_createTextureFragShaderFunction( const TextureLayout& layout, int maxSlots, bool blending, float fadeInDuration )
    {
        const TextureLayout::RenderOrderVector& order = layout.getRenderOrder();

        std::stringstream buf;

        buf << "#version 110 \n";

        if ( blending )
        {
            buf << "#extension GL_ARB_shader_texture_lod : enable \n"
                << "uniform float osgearth_SlotStamp[" << maxSlots << "]; \n"
                << "uniform float osg_FrameTime; \n"
                << "uniform float osgearth_LODRangeFactor; \n";
        }

        buf << "uniform float osgearth_ImageLayerOpacity[" << maxSlots << "]; \n"
            << "uniform vec3 osgearth_ImageLayerHSL[" << maxSlots << "]; \n"
            << "uniform vec3 osgearth_ImageLayerBCG[" << maxSlots << "]; \n"
            //The enabled array is a fixed size.  Make sure this corresponds EXCATLY to the size definition in TerrainEngineNode.cpp
            << "uniform bool  osgearth_ImageLayerVisible[" << MAX_IMAGE_LAYERS << "]; \n"
            << "uniform float osgearth_ImageLayerRange[" << 2 * maxSlots << "]; \n"
            << "uniform float osgearth_ImageLayerAttenuation; \n"
            << "uniform float osgearth_CameraElevation; \n"
            << "varying float osgearth_CameraRange; \n";

        const TextureLayout::TextureSlotVector& slots = layout.getTextureSlots();

        for( int i = 0; i < maxSlots && i < (int)slots.size(); ++i )
        {
            if ( slots[i] >= 0 )
            {
                buf << "uniform sampler2D " << makeSamplerName(i) << ";\n";
            }
        }


        buf << "void RGB_2_HSL(in float r, in float g, in float b, out float h, out float s, out float l)\n"
        << "{            \n"
        << "        float var_Min = min( r, min(g, b) );    //Min. value of RGB\n"
        << "        float var_Max = max( r, max(g, b) );    //Max. value of RGB\n"
        << "        float del_Max = var_Max - var_Min;      //Delta RGB value\n"
        << "\n"
        << "        l = ( var_Max + var_Min ) / 2.0;\n"
        << "\n"
        << "        if ( del_Max == 0.0 )                     //This is a gray, no chroma...\n"
        << "        {\n"
        << "            h = 0.0;                                //HSL results from 0 to 1\n"
        << "            s = 0.0;\n"
        << "        }\n"
        << "        else                                    //Chromatic data...\n"
        << "        {\n"
        << "            if ( l < 0.5 ) s = del_Max / ( var_Max + var_Min );\n"
        << "            else           s = del_Max / ( 2.0 - var_Max - var_Min );\n"
        << "\n"
        << "            float del_R = ( ( ( var_Max - r ) / 6.0 ) + ( del_Max / 2.0 ) ) / del_Max;\n"
        << "            float del_G = ( ( ( var_Max - g ) / 6.0 ) + ( del_Max / 2.0 ) ) / del_Max;\n"
        << "            float del_B = ( ( ( var_Max - b ) / 6.0 ) + ( del_Max / 2.0 ) ) / del_Max;\n"
        << "            if      ( r == var_Max ) h = del_B - del_G;\n"
        << "            else if ( g == var_Max ) h = ( 1.0 / 3.0 ) + del_R - del_B;\n"
        << "            else if ( b == var_Max ) h = ( 2.0 / 3.0 ) + del_G - del_R;\n"
        << "            if ( h < 0.0 ) h += 1.0;\n"
        << "            if ( h > 1.0 ) h -= 1.0;\n"
        << "        }\n"
        << "}\n"
        << "\n";


        buf << "float Hue_2_RGB(float v1, float v2, float vH )\n"
            << "{\n"
            << "float ret;\n"
            << "if ( vH < 0.0 )\n"
            << "vH += 1.0;\n"
            << "if ( vH > 1.0 )\n"
            << "vH -= 1.0;\n"
            << "if ( ( 6.0 * vH ) < 1.0 )\n"
            << "ret = ( v1 + ( v2 - v1 ) * 6.0 * vH );\n"
            << "else if ( ( 2.0 * vH ) < 1.0 )\n"
            << "ret = ( v2 );\n"
            << "else if ( ( 3.0 * vH ) < 2.0 )\n"
            << "ret = ( v1 + ( v2 - v1 ) * ( ( 2.0 / 3.0 ) - vH ) * 6.0 );\n"
            << "else\n"
            << "ret = v1;\n"
            << "return ret;\n"
            << "}\n";

        buf << "void HSL_2_RGB(in float h, in float s, in float l, out float r, out float g, out float b)\n"
            << "{\n"
            << "  float var_2, var_1;\n"
            << "  if (s == 0.0)\n"
            << "  {\n"
            << "    r = l;\n"
            << "    g = l;\n"
            << "    b = l;\n"
            << "  }\n"
            << "  else\n"
            << "  {\n"
            << "    if ( l < 0.5 )\n"
            << "    {\n"
            << "      var_2 = l * ( 1.0 + s );\n"
            << "    }\n"
            << "    else\n"
            << "    {\n"
            << "      var_2 = ( l + s ) - ( s * l );\n"
            << "    }\n"

            << "    var_1 = 2.0 * l - var_2;\n"

            << "    r = Hue_2_RGB( var_1, var_2, h + ( 1.0 / 3.0 ) );\n"
            << "    g = Hue_2_RGB( var_1, var_2, h );\n"
            << "    b = Hue_2_RGB( var_1, var_2, h - ( 1.0 / 3.0 ) );\n"
            << "  }\n"
            << "}\n";
               

        buf << "void osgearth_frag_applyTexturing( inout vec4 color ) \n"
            << "{ \n"
            << "    vec3 color3 = color.rgb; \n"
            << "    vec4 texel; \n"
            << "    float maxOpacity = 0.0; \n"
            << "    float dmin, dmax, atten_min, atten_max, age; \n";           

        for( unsigned int i=0; i < order.size(); ++i )
        {
            int slot = order[i];
            int q = 2 * i;

            // if this UID has a secondyar slot, LOD blending ON.
            int secondarySlot = layout.getSlot( slots[slot], 1, maxSlots );

            buf << "    if (osgearth_ImageLayerVisible["<< i << "]) { \n"
                << "        dmin = osgearth_CameraElevation - osgearth_ImageLayerRange["<< q << "]; \n"
                << "        dmax = osgearth_CameraElevation - osgearth_ImageLayerRange["<< q+1 <<"]; \n"

                << "        if (dmin >= 0.0 && dmax <= 0.0) { \n"
                << "            atten_max = -clamp( dmax, -osgearth_ImageLayerAttenuation, 0.0 ) / osgearth_ImageLayerAttenuation; \n"
                << "            atten_min =  clamp( dmin, 0.0, osgearth_ImageLayerAttenuation ) / osgearth_ImageLayerAttenuation; \n";

            if ( secondarySlot >= 0 ) // LOD blending enabled for this layer
            {
                float invFadeInDuration = 1.0f/fadeInDuration;

                buf << std::fixed
                    << std::setprecision(1)
                    << "            age = "<< invFadeInDuration << " * min( "<< fadeInDuration << ", osg_FrameTime - osgearth_SlotStamp[" << slot << "] ); \n"
                    << "            age = clamp(age, 0.0, 1.0); \n"
                    << "            vec4 texel0 = texture2D(" << makeSamplerName(slot) << ", gl_TexCoord["<< slot << "].st);\n"
                    << "            vec4 texel1 = texture2D(" << makeSamplerName(secondarySlot) << ", gl_TexCoord["<< secondarySlot << "].st);\n"
                    << "            float mixval = age * osgearth_LODRangeFactor;\n"

                    // pre-multiply alpha before mixing:
                    << "            texel0.rgb *= texel0.a; \n"
                    << "            texel1.rgb *= texel1.a; \n"
                    << "            texel = mix(texel1, texel0, mixval); \n"

                    // revert to non-pre-multiplies alpha (assumes openGL state uses non-pre-mult alpha)
                    << "            if (texel.a > 0.0) { \n"
                    << "                texel.rgb /= texel.a; \n"
                    << "            } \n";
            }
            else
            {
                buf << "            texel = texture2D(" << makeSamplerName(slot) << ", gl_TexCoord["<< slot <<"].st); \n";
            }
            
            buf << "            float opacity =  texel.a * osgearth_ImageLayerOpacity[" << i << "];\n"                
                << "            vec3 layerColor = texel.rgb;\n"                
                << "            vec3 hsl = osgearth_ImageLayerHSL[" << i <<  "];\n"
                << "            if (hsl.x != 0.0 || hsl.y != 0.0 || hsl.z != 0.0) {\n"                
                << "                float h, s, l;\n"
                << "                RGB_2_HSL( layerColor.r, layerColor.g, layerColor.b, h, s, l);\n"
                << "                h += hsl.x;\n"
                << "                s += hsl.y;\n"
                << "                l += hsl.z;\n"

                // clamp H,S and L to [0..1]

                << "                h = clamp(h, 0.0, 1.0);\n"
                << "                s = clamp(s, 0.0, 1.0);\n"
                << "                l = clamp(l, 0.0, 1.0);\n"


                << "                float r, g, b;\n"
                << "                HSL_2_RGB( h, s, l, r, g, b);\n"
                << "                layerColor.r = r;\n"
                << "                layerColor.g = g;\n"
                << "                layerColor.b = b;\n"
                << "            }\n"
                << "            vec3 bcg = osgearth_ImageLayerBCG[" << i <<  "];\n"
				<< "            if (bcg.x != 0.0 || bcg.y != 0.0 || bcg.z != 0.0) {\n"
                // apply brightness
                << "                layerColor.rgb += bcg.x;\n"
                // apply contrast
				<< "                layerColor.rgb = ((layerColor.rgb - 0.5) * ((bcg.y * 50.0) + 50.0) / 50.0) + 0.5;\n"
                // apply gamma
                << "                layerColor.r = pow(layerColor.r, (50.0 / ((bcg.z * 50.0) + 50.0)));\n"
                << "                layerColor.g = pow(layerColor.g, (50.0 / ((bcg.z * 50.0) + 50.0)));\n"
                << "                layerColor.b = pow(layerColor.b, (50.0 / ((bcg.z * 50.0) + 50.0)));\n"
                // make sure values are within range
                << "                layerColor.r = clamp(layerColor.r, 0.0, 1.0);\n"
                << "                layerColor.g = clamp(layerColor.g, 0.0, 1.0);\n"
                << "                layerColor.b = clamp(layerColor.b, 0.0, 1.0);\n"
                << "            }\n"
                << "            color3 = mix(color3, layerColor, opacity * atten_max * atten_min); \n"               
                << "            if (opacity > maxOpacity) {\n"
                << "              maxOpacity = opacity;\n"
                << "            }\n"                
                << "        } \n"
                << "    } \n";
        }
        
            buf << "    color = vec4(color3, maxOpacity);\n"
                << "} \n";



        std::string str;
        str = buf.str();
        //OE_INFO << std::endl << str;
        return new osg::Shader( osg::Shader::FRAGMENT, str );
    }
}

//------------------------------------------------------------------------

namespace
{    
    static osg::Texture2D*
        s_getTexture( osg::StateSet* stateSet, UID layerUID, const TextureLayout& layout, osg::StateSet* parentStateSet, osg::Texture::FilterMode minFilter, osg::Texture::FilterMode magFilter)
    {
        int slot = layout.getSlot( layerUID, 0 );
        if ( slot < 0 )
            return 0L;

        osg::Texture2D* tex = static_cast<osg::Texture2D*>(
            stateSet->getTextureAttribute( slot, osg::StateAttribute::TEXTURE ) );

        if ( !tex )
        {
            tex = new osg::Texture2D();

            // configure the mipmapping

            tex->setMaxAnisotropy( 16.0f );

            tex->setResizeNonPowerOfTwoHint(false);
            tex->setFilter( osg::Texture::MAG_FILTER, magFilter );
            tex->setFilter( osg::Texture::MIN_FILTER, minFilter );

            // configure the wrapping
            tex->setWrap( osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE );
            tex->setWrap( osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE );

            stateSet->setTextureAttributeAndModes( slot, tex, osg::StateAttribute::ON );
            
            // install the slot attribute
            std::string name = makeSamplerName(slot);
            stateSet->getOrCreateUniform( name.c_str(), osg::Uniform::SAMPLER_2D )->set( slot );
        }

        // see if we need an LOD blending secondary texture:
        int secondarySlot = layout.getSlot( layerUID, 1 );
        if ( secondarySlot >= 0 )
        {
            osg::Texture2D* parentTex = 0;

            //int parentSlot = slot + layout.getRenderOrder().size();
            std::string parentSampler = makeSamplerName( secondarySlot );
            if (parentStateSet)
            {
                parentTex = static_cast<osg::Texture2D*>(
                    parentStateSet->getTextureAttribute( slot, osg::StateAttribute::TEXTURE ) );

                if (parentTex)
                {
                    stateSet->setTextureAttributeAndModes(secondarySlot, parentTex, osg::StateAttribute::ON );
                    stateSet->getOrCreateUniform(parentSampler.c_str(),
                                                 osg::Uniform::SAMPLER_2D )->set( secondarySlot );
                }
            }

            if ( !parentTex )
            {
                // Bind the main texture as the secondary texture and
                // set the scaling factors appropriately.
                stateSet->getOrCreateUniform(
                    parentSampler.c_str(), osg::Uniform::SAMPLER_2D)->set(slot);
            }

        }
        return tex;
    }
}

//------------------------------------------------------------------------

bool
TextureCompositorMultiTexture::isSupported( bool useGPU )
{
    const Capabilities& caps = osgEarth::Registry::instance()->getCapabilities();
    if ( useGPU )
        return caps.supportsGLSL( 1.10f ) && caps.supportsMultiTexture();
    else
        return caps.supportsMultiTexture();
}

TextureCompositorMultiTexture::TextureCompositorMultiTexture( bool useGPU, const TerrainOptions& options ) :
_lodTransitionTime( *options.lodTransitionTime() ),
_enableMipmapping( *options.enableMipmapping() ),
_minFilter( *options.minFilter() ),
_magFilter( *options.magFilter() ),
_useGPU( useGPU )
{
    _enableMipmappingOnUpdatedTextures = Registry::instance()->getCapabilities().supportsMipmappedTextureUpdates();
}

void
TextureCompositorMultiTexture::applyLayerUpdate(osg::StateSet*       stateSet,
                                                UID                  layerUID,
                                                const GeoImage&      preparedImage,
                                                const TileKey&       tileKey,
                                                const TextureLayout& layout,
                                                osg::StateSet*       parentStateSet) const
{
    osg::Texture2D* tex = s_getTexture( stateSet, layerUID, layout, parentStateSet, _minFilter, _magFilter);
    if ( tex )
    {
        osg::Image* image = preparedImage.getImage();
        image->dirty(); // required for ensure the texture recognizes the image as new data
        tex->setImage( image );

        // set up proper mipmapping filters:
        if (_enableMipmapping &&
            _enableMipmappingOnUpdatedTextures && 
            ImageUtils::isPowerOfTwo( image ) && 
            !(!image->isMipmap() && ImageUtils::isCompressed(image)) )
        {
            if ( tex->getFilter(osg::Texture::MIN_FILTER) != _minFilter )
                tex->setFilter( osg::Texture::MIN_FILTER, _minFilter );
        }
        else if ( tex->getFilter(osg::Texture::MIN_FILTER) != osg::Texture::LINEAR )
        {
            tex->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR );
        }

        bool lodBlending = layout.getSlot(layerUID, 1) >= 0;

        if (_enableMipmapping &&
            _enableMipmappingOnUpdatedTextures && 
            lodBlending )
        {
            int slot = layout.getSlot(layerUID, 0);

            // update the timestamp on the image layer to support blending.
            float now = (float)osg::Timer::instance()->delta_s( osg::Timer::instance()->getStartTick(), osg::Timer::instance()->tick() );
            ArrayUniform stampUniform( "osgearth_SlotStamp", osg::Uniform::FLOAT, stateSet, layout.getMaxUsedSlot() + 1 );
            stampUniform.setElement( slot, now );            

            // set the texture matrix to properly position the blend (parent) texture
            osg::Matrix mat;
            if ( parentStateSet != 0L )
            {
                unsigned tileX, tileY;
                tileKey.getTileXY(tileX, tileY);

                mat(0,0) = 0.5f;
                mat(1,1) = 0.5f;
                mat(3,0) = (float)(tileX % 2) * 0.5f;
                mat(3,1) = (float)(1 - tileY % 2) * 0.5f;
            }

            ArrayUniform texMatUniform( "osgearth_TexBlendMatrix", osg::Uniform::FLOAT_MAT4, stateSet, layout.getMaxUsedSlot() + 1 );
            texMatUniform.setElement( slot, mat );
        }
    }
}

void 
TextureCompositorMultiTexture::updateMasterStateSet(osg::StateSet*       stateSet,
                                                    const TextureLayout& layout    ) const
{
    int numSlots = layout.getMaxUsedSlot() + 1;
    int maxUnits = numSlots;

    if ( _useGPU )
    {
        // Validate against the max number of GPU texture units:
        if ( maxUnits > Registry::instance()->getCapabilities().getMaxGPUTextureUnits() )
        {
            maxUnits = Registry::instance()->getCapabilities().getMaxGPUTextureUnits();

            OE_WARN << LC
                << "Warning! You have exceeded the number of texture units available on your GPU ("
                << maxUnits << "). Consider using another compositing mode."
                << std::endl;
        }

        VirtualProgram* vp = static_cast<VirtualProgram*>( stateSet->getAttribute(osg::StateAttribute::PROGRAM) );
        if ( maxUnits > 0 )
        {
            // see if we have any blended layers:
            bool hasBlending = layout.containsSecondarySlots( maxUnits ); 

            vp->setShader( 
                "osgearth_vert_setupTexturing", 
                s_createTextureVertexShader(layout, hasBlending) );

            vp->setShader( 
                "osgearth_frag_applyTexturing",
                s_createTextureFragShaderFunction(layout, maxUnits, hasBlending, _lodTransitionTime ) );
        }
        else
        {
            vp->removeShader( "osgearth_frag_applyTexturing", osg::Shader::FRAGMENT );
            vp->removeShader( "osgearth_vert_setupTexturing", osg::Shader::VERTEX );
        }
    }

    else
    {
        // Forcably disable shaders
        stateSet->setAttributeAndModes( new osg::Program(), osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED );

        // Validate against the maximum number of textures available in FFP mode.
        if ( maxUnits > Registry::instance()->getCapabilities().getMaxFFPTextureUnits() )
        {
            maxUnits = Registry::instance()->getCapabilities().getMaxFFPTextureUnits();
            OE_WARN << LC << 
                "Warning! You have exceeded the number of texture units available in fixed-function pipeline "
                "mode on your graphics hardware (" << maxUnits << "). Consider using another "
                "compositing mode." << std::endl;
        }

        // FFP multitexturing requires that we set up a series of TexCombine attributes:
        if (maxUnits == 1)
        {
            osg::TexEnv* texenv = new osg::TexEnv(osg::TexEnv::MODULATE);
            stateSet->setTextureAttributeAndModes(0, texenv, osg::StateAttribute::ON);
        }
        else if (maxUnits >= 2)
        {
            //Blend together the colors and accumulate the alpha values of textures 0 and 1 on unit 0
            {
                osg::TexEnvCombine* texenv = new osg::TexEnvCombine;
                texenv->setCombine_RGB(osg::TexEnvCombine::INTERPOLATE);
                texenv->setCombine_Alpha(osg::TexEnvCombine::ADD);

                texenv->setSource0_RGB(osg::TexEnvCombine::TEXTURE0+1);
                texenv->setOperand0_RGB(osg::TexEnvCombine::SRC_COLOR);
                texenv->setSource0_Alpha(osg::TexEnvCombine::TEXTURE0+1);
                texenv->setOperand0_Alpha(osg::TexEnvCombine::SRC_ALPHA);

                texenv->setSource1_RGB(osg::TexEnvCombine::TEXTURE0+0);
                texenv->setOperand1_RGB(osg::TexEnvCombine::SRC_COLOR);
                texenv->setSource1_Alpha(osg::TexEnvCombine::TEXTURE0+0);
                texenv->setOperand1_Alpha(osg::TexEnvCombine::SRC_ALPHA);

                texenv->setSource2_RGB(osg::TexEnvCombine::TEXTURE0+1);
                texenv->setOperand2_RGB(osg::TexEnvCombine::SRC_ALPHA);

                stateSet->setTextureAttributeAndModes(0, texenv, osg::StateAttribute::ON);
            }


            //For textures 2 and beyond, blend them together with the previous
            //Add the alpha values of this unit and the previous unit
            for (int unit = 1; unit < maxUnits-1; ++unit)
            {
                osg::TexEnvCombine* texenv = new osg::TexEnvCombine;
                texenv->setCombine_RGB(osg::TexEnvCombine::INTERPOLATE);
                texenv->setCombine_Alpha(osg::TexEnvCombine::ADD);

                texenv->setSource0_RGB(osg::TexEnvCombine::TEXTURE0+unit+1);
                texenv->setOperand0_RGB(osg::TexEnvCombine::SRC_COLOR);
                texenv->setSource0_Alpha(osg::TexEnvCombine::TEXTURE0+unit+1);
                texenv->setOperand0_Alpha(osg::TexEnvCombine::SRC_ALPHA);

                texenv->setSource1_RGB(osg::TexEnvCombine::PREVIOUS);
                texenv->setOperand1_RGB(osg::TexEnvCombine::SRC_COLOR);
                texenv->setSource1_Alpha(osg::TexEnvCombine::PREVIOUS);
                texenv->setOperand1_Alpha(osg::TexEnvCombine::SRC_ALPHA);

                texenv->setSource2_RGB(osg::TexEnvCombine::TEXTURE0+unit+1);
                texenv->setOperand2_RGB(osg::TexEnvCombine::SRC_ALPHA);

                stateSet->setTextureAttributeAndModes(unit, texenv, osg::StateAttribute::ON);
            }

            //Modulate the colors to get proper lighting on the last unit
            //Keep the alpha results from the previous stage
            {
                osg::TexEnvCombine* texenv = new osg::TexEnvCombine;
                texenv->setCombine_RGB(osg::TexEnvCombine::MODULATE);
                texenv->setCombine_Alpha(osg::TexEnvCombine::REPLACE);

                texenv->setSource0_RGB(osg::TexEnvCombine::PREVIOUS);
                texenv->setOperand0_RGB(osg::TexEnvCombine::SRC_COLOR);
                texenv->setSource0_Alpha(osg::TexEnvCombine::PREVIOUS);
                texenv->setOperand0_Alpha(osg::TexEnvCombine::SRC_ALPHA);

                texenv->setSource1_RGB(osg::TexEnvCombine::PRIMARY_COLOR);
                texenv->setOperand1_RGB(osg::TexEnvCombine::SRC_COLOR);
                stateSet->setTextureAttributeAndModes(maxUnits-1, texenv, osg::StateAttribute::ON);
            }
        }
    }
}

osg::Shader*
TextureCompositorMultiTexture::createSamplerFunction(UID layerUID,
                                                     const std::string& functionName,
                                                     osg::Shader::Type type,
                                                     const TextureLayout& layout ) const
{
    osg::Shader* result = 0L;

    int slot = layout.getSlot( layerUID );
    if ( slot >= 0 )
    {
        std::stringstream buf;

        buf << "uniform sampler2D tex"<< slot << "; \n"
            << "vec4 " << functionName << "() \n"
            << "{ \n";

        if ( type == osg::Shader::VERTEX )
            buf << "    return texture2D(tex"<< slot << ", gl_MultiTexCoord"<< slot <<".st); \n";
        else
            buf << "    return texture2D(tex"<< slot << ", gl_TexCoord["<< slot << "].st); \n";

        buf << "} \n";

        std::string str;
        str = buf.str();
        result = new osg::Shader( type, str );
    }
    return result;
}
