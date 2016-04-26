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
#include "DetailTerrainEffect"
#include "DetailShaders"

#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/ImageUtils>
#include <osgEarthUtil/SimplexNoise>

#define LC "[Detail] "

#define SAMPLER_NAME "oe_detail_tex"

using namespace osgEarth;
using namespace osgEarth::Detail;

DetailTerrainEffect::DetailTerrainEffect(const DetailOptions& options) :
_options     ( options ),
_texImageUnit( -1 )
{
}

void
DetailTerrainEffect::onInstall(TerrainEngineNode* engine)
{
    if ( engine )
    {
        // Load the image
        osg::ref_ptr<osg::Image> image = _options.imageURI()->getImage();
        if ( !image.valid() )
        {
            OE_WARN << LC << "Failed; unable to load detail map image from "
                << _options.imageURI()->full() << "\n";
            return;
        }

        // Create the texture
        _tex = new osg::Texture2D(image.get());
        _tex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
        _tex->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
        _tex->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
        _tex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
        _tex->setMaxAnisotropy(1.0f);
        _tex->setUnRefImageDataAfterApply(true);
        _tex->setResizeNonPowerOfTwoHint(false);

        engine->getResources()->reserveTextureImageUnit(_texImageUnit, "Detail");
        if ( _texImageUnit >= 0 )
        {   
            // Create the uniform for the sampler.
            osg::StateSet* stateset = engine->getOrCreateStateSet();
            stateset->setTextureAttribute( _texImageUnit, _tex.get() );
            stateset->addUniform( new osg::Uniform(SAMPLER_NAME, _texImageUnit) );
            stateset->addUniform( new osg::Uniform("oe_detail_lod", (float)_options.lod().get()));
            stateset->addUniform( new osg::Uniform("oe_detail_alpha", _options.alpha().get()));
            stateset->addUniform( new osg::Uniform("oe_detail_maxRange", _options.maxRange().get()));
            stateset->addUniform( new osg::Uniform("oe_detail_attenDist", _options.attenDist().get()));

            // configure shaders
            VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);

            Shaders package;            

            package.load( vp, package.VertexView );
            package.load( vp, package.Fragment );           
            
            OE_INFO << LC << "Detail texture installed!\n";
        }
        else
        {
            OE_WARN << LC << "No texture image units available; detail disabled.\n";
        }
    }
}


void
DetailTerrainEffect::onUninstall(TerrainEngineNode* engine)
{
    if ( engine && _texImageUnit >= 0 )
    {
        osg::StateSet* stateset = engine->getStateSet();
        if ( stateset )
        {
            stateset->removeUniform( SAMPLER_NAME );
            stateset->removeTextureAttribute( _texImageUnit, _tex.get() );
        }

        engine->getResources()->releaseTextureImageUnit( _texImageUnit );
        _texImageUnit = -1;
    }
}