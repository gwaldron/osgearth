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
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/
#include "LandCoverTerrainEffect"
#include "SplatOptions"

#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/VirtualProgram>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/URI>
#include <osgEarth/ShaderLoader>
#include <osgEarth/ImageUtils>

#include <osg/Texture2D>
#include <osgUtil/Optimizer>

#include "SplatShaders"

#define LC "[LandCoverTerrainEffect] "

using namespace osgEarth;
using namespace osgEarth::Splat;

LandCoverTerrainEffect::LandCoverTerrainEffect()
{
    //nop
}

void
LandCoverTerrainEffect::setDBOptions(const osgDB::Options* dbo)
{
    _dbo = dbo;
}

void
LandCoverTerrainEffect::onInstall(TerrainEngineNode* engine)
{
    if ( !_landCover.valid() )
        return;

    if ( engine )
    {
        for(LandCoverLayers::const_iterator i = _landCover->getLayers().begin();
            i != _landCover->getLayers().end();
            ++i)
        {
            const LandCoverLayer* layer = i->get();
            if ( layer )
            {
                if ( !layer->getBiomes().empty() )
                {
                    // For now, one biome per layer:
                    const LandCoverBiome* biome = layer->getBiomes().front().get();

                    if ( !biome->getBillboards().empty() )
                    {
                        OE_INFO << LC << "Adding land cover group: " << layer->getName() << " at LOD " << layer->getLOD() << "\n";

                        osg::StateSet* stateset = engine->addLandCoverGroup( layer->getName(), layer->getLOD() );
                        if ( stateset )
                        {
                            // Install the land cover shaders on the state set
                            VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);
                            LandCoverShaders shaders;
                            shaders.loadAll( vp, _dbo.get() );

                            // Generate the coverage acceptor
                            osg::Shader* covTest = biome->createPredicateShader( getCoverage(), osg::Shader::TESSCONTROL );
                            vp->setShader( covTest );

                            // Install the uniforms
                            stateset->addUniform( new osg::Uniform("oe_landcover_windFactor", layer->getWind()) );
                            stateset->addUniform( new osg::Uniform("oe_landcover_noise", 1.0f) );
                            stateset->addUniform( new osg::Uniform("oe_landcover_ao", 0.5f) );
                            stateset->addUniform( new osg::Uniform("oe_landcover_colorVariation", 0.3f) );
                            stateset->addUniform( new osg::Uniform("oe_landcover_exposure", 1.0f) );
                    
                            stateset->addUniform( new osg::Uniform("oe_landcover_density",     layer->getDensity()) );
                            stateset->addUniform( new osg::Uniform("oe_landcover_maxDistance", layer->getMaxDistance()) );

                            int unit;
                            if ( engine->getResources()->reserveTextureImageUnit(unit, "LandCover") )
                            {
                                osg::Texture* tex = 0L;
                                if ( true ) //biome->getBillboards().size() > 1 )
                                {
                                    int s=-1, t=-1;
                                    osg::Texture2DArray* tex = new osg::Texture2DArray();
                                    for(int i=0; i<biome->getBillboards().size(); ++i)
                                    {
                                        const LandCoverBillboard& bb = biome->getBillboards().at(i);

                                        osg::ref_ptr<osg::Image> im;

                                        if ( s < 0 )
                                        {
                                            s  = bb._image->s();
                                            t  = bb._image->t();
                                            im = bb._image.get();
                                            tex->setTextureSize(s, t, biome->getBillboards().size());         

                                            // Billboard parameters:
                                            // TODO: per-billboard settings
                                            stateset->addUniform( new osg::Uniform("oe_landcover_width",  bb._width) );
                                            stateset->addUniform( new osg::Uniform("oe_landcover_height", bb._height) );                                   
                                        }
                                        else
                                        {
                                            if ( bb._image->s() != s || bb._image->t() != t )
                                            {
                                                ImageUtils::resizeImage( bb._image.get(), s, t, im );
                                            }
                                            else
                                            {
                                                im = bb._image.get();
                                            }
                                        }

                                        tex->setImage( i, im.get() );
                                    }
                                    
                                    tex->setFilter(tex->MIN_FILTER, tex->NEAREST_MIPMAP_LINEAR);
                                    tex->setFilter(tex->MAG_FILTER, tex->LINEAR);
                                    tex->setWrap  (tex->WRAP_S, tex->CLAMP_TO_EDGE);
                                    tex->setWrap  (tex->WRAP_T, tex->CLAMP_TO_EDGE);
                                    tex->setUnRefImageDataAfterApply( true );
                                    tex->setMaxAnisotropy( 4.0 );
                                    tex->setResizeNonPowerOfTwoHint( false );

                                    stateset->setTextureAttribute(unit, tex);
                                    stateset->addUniform(new osg::Uniform("oe_landcover_texArray", unit) );
                                    stateset->addUniform(new osg::Uniform("oe_landcover_arraySize", (float)tex->getNumImages()));
                                }
                                else
                                {
                                    // For now, one texture per biome.
                                    const LandCoverBillboard& bb = biome->getBillboards().front();
                                    if ( bb._image.valid() )
                                    {
                                        osg::Texture2D* tex = new osg::Texture2D( bb._image.get() );

                                        tex->setFilter(tex->MIN_FILTER, tex->NEAREST_MIPMAP_LINEAR);
                                        tex->setFilter(tex->MAG_FILTER, tex->LINEAR);
                                        tex->setWrap  (tex->WRAP_S, tex->CLAMP_TO_EDGE);
                                        tex->setWrap  (tex->WRAP_T, tex->CLAMP_TO_EDGE);
                                        tex->setUnRefImageDataAfterApply( true );
                                        tex->setMaxAnisotropy( 4.0 );
                                        tex->setResizeNonPowerOfTwoHint( false );

                                        stateset->setTextureAttribute(unit, tex);
                                        stateset->addUniform(new osg::Uniform("oe_landcover_tex", unit) );

                                        // Billboard parameters:
                                        stateset->addUniform( new osg::Uniform("oe_landcover_width",  bb._width) );
                                        stateset->addUniform( new osg::Uniform("oe_landcover_height", bb._height) );
                                    }
                                    else
                                    {
                                        OE_WARN << LC << "ILLEGAL: null image for a land cover biome billboard\n";
                                    }
                                }
                            }
                            else
                            {
                                OE_WARN << LC << "Terrain engine failed to allocate a texture image unit for a land cover group\n";
                            }
                        }
                        else
                        {
                            OE_WARN << LC << "Terrain engine failed to return a stateset for a land cover group\n";
                        }
                    }
                    else
                    {
                        OE_WARN << LC << "ILLEGAL: land cover biome with nothing to render\n";
                    }
                }
                else
                {
                    OE_WARN << LC << "ILLEGAL: land cover layer with no biomes defined\n";
                }
            }
            else
            {
                OE_WARN << LC << "ILLEGAL: empty layer found in land cover layer list\n";
            }
        }
    }
}



void
LandCoverTerrainEffect::onUninstall(TerrainEngineNode* engine)
{
    if ( engine )
    {
        //TODO
    }
}
