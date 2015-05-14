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
#include "GraticuleTerrainEffect"

#include <osgEarth/VirtualProgram>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/ShaderLoader>

#include "GraticuleShaders"

#define LC "[Graticule] "

using namespace osgEarth;
using namespace osgEarth::Graticule;


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
        osg::StateSet* stateset = engine->getTerrainStateSet();
        VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);

        // configure shaders
        Shaders package;
        package.loadFunction( vp, package.Vertex );
        package.loadFunction( vp, package.Fragment );

        stateset->addUniform( new osg::Uniform(
            GraticuleOptions::resolutionUniformName(), _options.maxResolution().get()) );

        stateset->addUniform( new osg::Uniform(
            GraticuleOptions::colorUniformName(), _options.color().get()) );

        stateset->addUniform( new osg::Uniform(
            GraticuleOptions::lineWidthUniformName(), _options.lineWidth().get()) );
    }
}


void
GraticuleTerrainEffect::onUninstall(TerrainEngineNode* engine)
{
    osg::StateSet* stateset = engine->getTerrainStateSet();
    if ( stateset )
    {
        VirtualProgram* vp = VirtualProgram::get(stateset);
        if ( vp )
        {
            Shaders package;
            package.unloadFunction( vp, package.Vertex );
            package.unloadFunction( vp, package.Fragment );

            stateset->removeUniform( GraticuleOptions::resolutionUniformName() );
            stateset->removeUniform( GraticuleOptions::colorUniformName() );
            stateset->removeUniform( GraticuleOptions::lineWidthUniformName() );
        }
    }
}
