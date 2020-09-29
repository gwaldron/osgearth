/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
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
#include "TextureSplattingLayer"
#include "ProceduralShaders"
#include "NoiseTextureFactory"
#include <osgEarth/VirtualProgram>
#include <osgUtil/CullVisitor>
#include <osg/BlendFunc>
#include <osg/Drawable>
#include <cstdlib> // getenv

#define LC "[TextureSplattingLayer] " << getName() << ": "

using namespace osgEarth::Procedural;

REGISTER_OSGEARTH_LAYER(splat, TextureSplattingLayer);
REGISTER_OSGEARTH_LAYER(splatimage, TextureSplattingLayer);
REGISTER_OSGEARTH_LAYER(splat_imagery, TextureSplattingLayer);

//........................................................................

Config
TextureSplattingLayer::Options::getConfig() const
{
    Config conf = VisibleLayer::Options::getConfig();
    return conf;
}

void
TextureSplattingLayer::Options::fromConfig(const Config& conf)
{
}

//........................................................................

void
TextureSplattingLayer::init()
{
    VisibleLayer::init();

    setRenderType(osgEarth::Layer::RENDERTYPE_TERRAIN_SURFACE);

    osg::StateSet* ss = this->getOrCreateStateSet();

    // Arena to hold all the splatting textures
    _arena = new TextureArena();
    ss->setAttribute(_arena);

    // Install the texture splatting shader
    VirtualProgram* vp = VirtualProgram::getOrCreate(ss);
    TerrainShaders shaders;
    shaders.load(vp, shaders.TextureSplatting2, getReadOptions());
}

void
TextureSplattingLayer::addedToMap(const Map* map)
{
    VisibleLayer::addedToMap(map);    
    buildStateSets();
}

void
TextureSplattingLayer::removedFromMap(const Map* map)
{
    VisibleLayer::removedFromMap(map);
}

void
TextureSplattingLayer::setTerrainResources(TerrainResources* res)
{
    VisibleLayer::setTerrainResources(res);

    if (res)
    {
    }
}

void
TextureSplattingLayer::buildStateSets()
{
    //todo
}

void
TextureSplattingLayer::resizeGLObjectBuffers(unsigned maxSize)
{
    VisibleLayer::resizeGLObjectBuffers(maxSize);
}

void
TextureSplattingLayer::releaseGLObjects(osg::State* state) const
{
    VisibleLayer::releaseGLObjects(state);
}

Config
TextureSplattingLayer::getConfig() const
{
    Config c = VisibleLayer::getConfig();
    return c;
}
