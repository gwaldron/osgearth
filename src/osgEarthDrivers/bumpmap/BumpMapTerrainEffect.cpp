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
#include "BumpMapTerrainEffect"
#include "BumpMapOptions"

#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/VirtualProgram>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/TerrainTileNode>

#include "BumpMapShaders"

#define LC "[BumpMap] "

#define BUMP_SAMPLER   "oe_bumpmap_tex"

using namespace osgEarth;
using namespace osgEarth::BumpMap;


BumpMapTerrainEffect::BumpMapTerrainEffect(const osgDB::Options* dbOptions) :
_ok(true),
_bumpMapUnit(-1)
{
    BumpMapOptions defaults;
    _octaves = defaults.octaves().get();
    _maxRange = defaults.maxRange().get();
    _baseLOD  = defaults.baseLOD().get();

    _scaleUniform     = new osg::Uniform("oe_bumpmap_scale", defaults.scale().get());
    _intensityUniform = new osg::Uniform("oe_bumpmap_intensity", defaults.intensity().get());
}

void
BumpMapTerrainEffect::setBumpMapImage(osg::Image* image)
{
    if ( image )
    {
        _bumpMapTex = new osg::Texture2D(image);
        _bumpMapTex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
        _bumpMapTex->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
        _bumpMapTex->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR);
        _bumpMapTex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
        _bumpMapTex->setMaxAnisotropy(1.0f);
        _bumpMapTex->setUnRefImageDataAfterApply(true);
        _bumpMapTex->setResizeNonPowerOfTwoHint(false);
    }
    else
    {
        OE_WARN << LC << "Failed to load the bump map texture\n";
    }
}

void
BumpMapTerrainEffect::onInstall(TerrainEngineNode* engine)
{
    if ( engine && _bumpMapTex.valid() )
    {
        osg::StateSet* stateset = engine->getSurfaceStateSet();

        // install the NormalMap texture array:
        if ( engine->getResources()->reserveTextureImageUnit(_bumpMapUnit, "BumpMap") )
        {
            // NormalMap sampler
            _bumpMapTexUniform = stateset->getOrCreateUniform(BUMP_SAMPLER, osg::Uniform::SAMPLER_2D);
            _bumpMapTexUniform->set( _bumpMapUnit );
            stateset->setTextureAttribute( _bumpMapUnit, _bumpMapTex.get() );

            // configure shaders
            VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);

            Shaders package;            
            package.define( "OE_USE_NORMAL_MAP", false ); //engine->normalTexturesRequired() );

            //package.load( vp, package.VertexModel );
            package.load( vp, package.VertexView );
            package.load( vp, _octaves <= 1? package.FragmentSimple : package.FragmentProgressive );

            if ( _octaves > 1 )
                stateset->addUniform(new osg::Uniform("oe_bumpmap_octaves", _octaves));

            stateset->addUniform(new osg::Uniform("oe_bumpmap_maxRange", _maxRange));
            stateset->addUniform(new osg::Uniform("oe_bumpmap_slopeFactor", 1.0f));
            stateset->addUniform(new osg::Uniform("oe_bumpmap_baseLOD", (float)_baseLOD));

            stateset->addUniform( _scaleUniform.get() );
            stateset->addUniform( _intensityUniform.get() );
        }
        else
        {
            OE_WARN << LC << "Failed to allocation a texture image unit!\n";
        }
    }
}


void
BumpMapTerrainEffect::onUninstall(TerrainEngineNode* engine)
{
    osg::StateSet* stateset = engine->getStateSet();
    if ( stateset )
    {
        if ( _bumpMapTex.valid() )
        {
            stateset->removeUniform("oe_bumpmap_maxRange");
            stateset->removeUniform("oe_bumpmap_octaves");
            stateset->removeUniform( _scaleUniform.get() );
            stateset->removeUniform( _intensityUniform.get() );
            stateset->removeUniform( _bumpMapTexUniform.get() );
            stateset->removeTextureAttribute( _bumpMapUnit, osg::StateAttribute::TEXTURE );
        }

        VirtualProgram* vp = VirtualProgram::get(stateset);
        if ( vp )
        {
            Shaders pkg;
            pkg.unloadAll( vp );
        }
    }
    
    if ( _bumpMapUnit >= 0 )
    {
        engine->getResources()->releaseTextureImageUnit( _bumpMapUnit );
        _bumpMapUnit = -1;
    }
}
