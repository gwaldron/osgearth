/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2016 Pelican Mapping
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
#include "BYOTerrainEngineNode"
#include "BYOTerrainEngineOptions"
#include <osgEarth/ShaderGenerator>
#include <osgEarth/Registry>

#define LC "[BYOTerrainEngineNode] "

using namespace osgEarth_engine_byo;
using namespace osgEarth;
using namespace osgEarth::Drivers;

//------------------------------------------------------------------------

BYOTerrainEngineNode::BYOTerrainEngineNode() :
TerrainEngineNode( )
{
    _uid = Registry::instance()->createUID();
}

BYOTerrainEngineNode::~BYOTerrainEngineNode()
{
    //nop
}

void
BYOTerrainEngineNode::preInitialize( const Map* map, const TerrainOptions& options )
{
    TerrainEngineNode::preInitialize( map, options );
    BYOTerrainEngineOptions myoptions(options);
    if ( myoptions.getNode() )
    {
        this->addChild( myoptions.getNode() );
    }
    else if ( myoptions.url().isSet() )
    {
        OE_INFO << LC << "Loading terrain from " << myoptions.url()->full() << std::endl;

        osg::Node* node = myoptions.url()->getNode();
        if ( node )
        {
            if ( myoptions.shaderPolicy() == SHADERPOLICY_GENERATE )
            {
                osg::ref_ptr<StateSetCache> cache = new StateSetCache();
                
                Registry::shaderGenerator().run(
                    node,
                    "osgEarth.BYOTerrainEngine",
                    cache.get() );
            }
            else if ( myoptions.shaderPolicy() == SHADERPOLICY_DISABLE )
            {
                node->getOrCreateStateSet()->setAttributeAndModes(
                    new osg::Program(),
                    osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE );
            }

            this->addChild( node );
        }
    }
}
