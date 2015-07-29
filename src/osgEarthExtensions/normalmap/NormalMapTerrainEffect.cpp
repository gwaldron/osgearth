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
#include "NormalMapTerrainEffect"

#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/VirtualProgram>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/TerrainTileNode>
#include <osgEarth/ShaderLoader>

#include "NormalMapShaders"

#define LC "[NormalMap] "

#define NORMAL_SAMPLER "oe_nmap_normalTex"
#define NORMAL_MATRIX  "oe_nmap_normalTexMatrix"

using namespace osgEarth;
using namespace osgEarth::NormalMap;

namespace
{
    class NormalTexInstaller : public TerrainTileNodeCallback
    {
    public:
        NormalTexInstaller(NormalMapTerrainEffect* effect, int unit)
            : _effect(effect), _unit(unit) { }
        
    public: // TileNodeCallback
        void operator()(const TileKey& key, osg::Node* node)
        {
            TerrainTileNode* tile = osgEarth::findTopMostNodeOfType<TerrainTileNode>(node);
            if ( !tile )
                return;
            
            osg::StateSet* ss = node->getOrCreateStateSet();
            osg::Texture* tex = tile->getNormalTexture();
            if ( tex )
            {
                ss->setTextureAttribute(_unit, tex);
            }

            osg::RefMatrixf* mat = tile->getNormalTextureMatrix();
            osg::Matrixf fmat;
            if ( mat )
            {
                fmat = osg::Matrixf(*mat);
            }
            else
            {
                // special marker indicating that there's no valid normal texture.
                fmat(0,0) = 0.0f;
            }

            ss->addUniform(new osg::Uniform(NORMAL_MATRIX, fmat) );
        }

    private:
        osg::observer_ptr<NormalMapTerrainEffect> _effect;
        int _unit;
    };
}


NormalMapTerrainEffect::NormalMapTerrainEffect(const osgDB::Options* dbOptions) :
_normalMapUnit( -1 )
{
    //nop
}

void
NormalMapTerrainEffect::onInstall(TerrainEngineNode* engine)
{
    if ( engine )
    {
        engine->requireNormalTextures();

        engine->getResources()->reserveTextureImageUnit(_normalMapUnit, "NormalMap");
        engine->addTileNodeCallback( new NormalTexInstaller(this, _normalMapUnit) );
        
        // shader components
        osg::StateSet* stateset = engine->getTerrainStateSet();
        VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);

        // configure shaders
        Shaders package;
        package.load( vp, package.Vertex );
        package.load( vp, package.Fragment );
        
        stateset->addUniform( new osg::Uniform(NORMAL_SAMPLER, _normalMapUnit) );
    }
}


void
NormalMapTerrainEffect::onUninstall(TerrainEngineNode* engine)
{
    osg::StateSet* stateset = engine->getStateSet();
    if ( stateset )
    {
        VirtualProgram* vp = VirtualProgram::get(stateset);
        if ( vp )
        {
            Shaders package;
            package.unload( vp, package.Vertex );
            package.unload( vp, package.Fragment );
        }
        stateset->removeUniform( NORMAL_SAMPLER );
    }
    
    if ( _normalMapUnit >= 0 )
    {
        engine->getResources()->releaseTextureImageUnit( _normalMapUnit );
        _normalMapUnit = -1;
    }
}
