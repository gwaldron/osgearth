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
#include "GraticuleTerrainEffect"

#include <osgEarth/VirtualProgram>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/ShaderLoader>

#include "Shaders"

#define LC "[Graticule] "

using namespace osgEarth;
using namespace osgEarth::Util;


GraticuleTerrainEffect::GraticuleTerrainEffect(const GraticuleOptions& options,
                                               const osgDB::Options*   dbOptions) :
_options( options )
{
    //nop
}

void
GraticuleTerrainEffect::onInstall(TerrainEngineNode* engine)
{
    if ( engine )
    {
        // shader components
        osg::StateSet* stateset = engine->getSurfaceStateSet();
        VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);

        // configure shaders
        Shaders package;
        package.load( vp, package.Graticule_Vertex );
        package.load( vp, package.Graticule_Fragment);

        stateset->addUniform( new osg::Uniform(
            GraticuleOptions::resolutionUniformName(), 10.0/180.0) );

        stateset->addUniform( new osg::Uniform(
            GraticuleOptions::colorUniformName(), _options.color().get()) );

        stateset->addUniform( new osg::Uniform(
            GraticuleOptions::lineWidthUniformName(), _options.lineWidth().get()) );
    }
}


void
GraticuleTerrainEffect::onUninstall(TerrainEngineNode* engine)
{
    osg::StateSet* stateset = engine->getSurfaceStateSet();
    if ( stateset )
    {
        VirtualProgram* vp = VirtualProgram::get(stateset);
        if ( vp )
        {
            Shaders package;
            package.unload( vp, package.Graticule_Vertex );
            package.unload( vp, package.Graticule_Fragment );

            stateset->removeUniform( GraticuleOptions::resolutionUniformName() );
            stateset->removeUniform( GraticuleOptions::colorUniformName() );
            stateset->removeUniform( GraticuleOptions::lineWidthUniformName() );
        }
    }
}
