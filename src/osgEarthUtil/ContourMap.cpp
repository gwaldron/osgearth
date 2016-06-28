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
#include <osgEarthUtil/ContourMap>
#include <osgEarthUtil/Shaders>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/VirtualProgram>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/MapNode>

#define LC "[ContourMap] "

using namespace osgEarth;
using namespace osgEarth::Util;


ContourMap::ContourMap() :
TerrainEffect()
{
    init();
}

ContourMap::ContourMap(const ConfigOptions& co) :
TerrainEffect(),
ContourMapOptions(co)
{
    init();
}

void
ContourMap::init()
{
    // negative means unset:
    _unit = -1;

    // uniforms we'll need:
    _xferMin     = new osg::Uniform(osg::Uniform::FLOAT,      "oe_contour_min" );
    _xferRange   = new osg::Uniform(osg::Uniform::FLOAT,      "oe_contour_range" );
    _xferSampler = new osg::Uniform(osg::Uniform::SAMPLER_1D, "oe_contour_xfer" );
    _opacityUniform = new osg::Uniform(osg::Uniform::FLOAT,   "oe_contour_opacity" );

    // Create a 1D texture from the transfer function's image.
    _xferTexture = new osg::Texture1D();
    _xferTexture->setResizeNonPowerOfTwoHint( false );
    _xferTexture->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR );
    _xferTexture->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
    _xferTexture->setWrap( osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE );

#if 0
    // build a default transfer function.
    // TODO: think about scale/bias controls.
    osg::TransferFunction1D* xfer = new osg::TransferFunction1D();
    float s = 2500.0f;

    if ( grayscale() == true )
    {
        xfer->setColor( -1.0000 * s, osg::Vec4f(.125,.125,.125, 1), false);
        xfer->setColor( -0.2500 * s, osg::Vec4f(.25,.25,.25, 1), false);
        xfer->setColor(  0.0000 * s, osg::Vec4f(.375,.375,.375, 1), false);
        xfer->setColor(  0.0062 * s, osg::Vec4f(.5,.5,.5,1), false);
        xfer->setColor(  0.1250 * s, osg::Vec4f(.625,.625,.625,1), false);
        xfer->setColor(  0.3250 * s, osg::Vec4f(.75,.75,.75,1), false);
        xfer->setColor(  0.7500 * s, osg::Vec4f(.875,.875,.875,1), false);
        xfer->setColor(  1.0000 * s, osg::Vec4f(1,1,1,1), false);
    }
    else
    {
        xfer->setColor( -1.0000 * s, osg::Vec4f(0, 0, 0.5, 1), false);
        xfer->setColor( -0.2500 * s, osg::Vec4f(0, 0, 1, 1), false);
        xfer->setColor(  0.0000 * s, osg::Vec4f(0, .5, 1, 1), false);
        xfer->setColor(  0.0062 * s, osg::Vec4f(.84,.84,.25,1), false);
        xfer->setColor(  0.1250 * s, osg::Vec4f(.125,.62,0,1), false);
        xfer->setColor(  0.3250 * s, osg::Vec4f(.80,.70,.47,1), false);
        xfer->setColor(  0.7500 * s, osg::Vec4f(.5,.5,.5,1), false);
        xfer->setColor(  1.0000 * s, osg::Vec4f(1,1,1,1), false);
    }
    xfer->updateImage();
    this->setTransferFunction( xfer );
    _opacityUniform->set( opacity().getOrUse(1.0f) );
#endif

    dirty();
}

void
ContourMap::dirty()
{
    _opacityUniform->set(opacity().getOrUse(1.0f));
    
    // build a transfer function.
    osg::TransferFunction1D* xfer = new osg::TransferFunction1D();
    float s = 2500.0f;

    if ( grayscale() == true )
    {
        xfer->setColor( -1.0000 * s, osg::Vec4f(.125,.125,.125, 1), false);
        xfer->setColor( -0.2500 * s, osg::Vec4f(.25,.25,.25, 1), false);
        xfer->setColor(  0.0000 * s, osg::Vec4f(.375,.375,.375, 1), false);
        xfer->setColor(  0.0062 * s, osg::Vec4f(.5,.5,.5,1), false);
        xfer->setColor(  0.1250 * s, osg::Vec4f(.625,.625,.625,1), false);
        xfer->setColor(  0.3250 * s, osg::Vec4f(.75,.75,.75,1), false);
        xfer->setColor(  0.7500 * s, osg::Vec4f(.875,.875,.875,1), false);
        xfer->setColor(  1.0000 * s, osg::Vec4f(1,1,1,1), false);
    }
    else
    {
        xfer->setColor( -1.0000 * s, osg::Vec4f(0, 0, 0.5, 1), false);
        xfer->setColor( -0.2500 * s, osg::Vec4f(0, 0, 1, 1), false);
        xfer->setColor(  0.0000 * s, osg::Vec4f(0, .5, 1, 1), false);
        xfer->setColor(  0.0062 * s, osg::Vec4f(.84,.84,.25,1), false);
        xfer->setColor(  0.1250 * s, osg::Vec4f(.125,.62,0,1), false);
        xfer->setColor(  0.3250 * s, osg::Vec4f(.80,.70,.47,1), false);
        xfer->setColor(  0.7500 * s, osg::Vec4f(.5,.5,.5,1), false);
        xfer->setColor(  1.0000 * s, osg::Vec4f(1,1,1,1), false);
    }
    xfer->updateImage();
    this->setTransferFunction( xfer );
}

ContourMap::~ContourMap()
{
    //nop
}

void
ContourMap::setTransferFunction(osg::TransferFunction1D* xfer)
{
    _xfer = xfer;

    _xferTexture->setImage( _xfer->getImage() );
    _xferMin->set( _xfer->getMinimum() );
    _xferRange->set( _xfer->getMaximum() - _xfer->getMinimum() );
}

void
ContourMap::onInstall(TerrainEngineNode* engine)
{
    if ( engine )
    {
        if ( !engine->getResources()->reserveTextureImageUnit(_unit, "ContourMap") )
        {
            OE_WARN << LC << "Failed to reserve a texture image unit; disabled." << std::endl;
            return;
        }

        osg::StateSet* stateset = engine->getOrCreateStateSet();

        // Install the texture and its sampler uniform:
        stateset->setTextureAttributeAndModes( _unit, _xferTexture.get(), osg::StateAttribute::ON );
        stateset->addUniform( _xferSampler.get() );
        _xferSampler->set( _unit );

        // (By the way: if you want to draw image layers on top of the contoured terrain,
        // set the "priority" parameter to setFunction() to a negative number so that it draws
        // before the terrain's layers.)
        VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);

        Shaders pkg;
        //pkg.load(vp, pkg.ContourMap_Vertex);
        pkg.load(vp, pkg.ContourMap_Fragment);

        // Install some uniforms that tell the shader the height range of the color map.
        stateset->addUniform( _xferMin.get() );
        _xferMin->set( _xfer->getMinimum() );

        stateset->addUniform( _xferRange.get() );
        _xferRange->set( _xfer->getMaximum() - _xfer->getMinimum() );

        stateset->addUniform( _opacityUniform.get() );
    }
}


void
ContourMap::onUninstall(TerrainEngineNode* engine)
{
    if ( engine )
    {
        osg::StateSet* stateset = engine->getStateSet();
        if ( stateset )
        {
            stateset->removeUniform( _xferMin.get() );
            stateset->removeUniform( _xferRange.get() );
            stateset->removeUniform( _xferSampler.get() );
            stateset->removeUniform( _opacityUniform.get() );

            stateset->removeTextureAttribute( _unit, osg::StateAttribute::TEXTURE );

            VirtualProgram* vp = VirtualProgram::get(stateset);
            if ( vp )
            {
                Shaders pkg;
                pkg.unload(vp, pkg.ContourMap_Vertex);
                pkg.unload(vp, pkg.ContourMap_Fragment);
            }
        }

        if ( _unit >= 0 )
        {
            engine->getResources()->releaseTextureImageUnit( _unit );
            _unit = -1;
        }
    }
}

//-------------------------------------------------------------

REGISTER_OSGEARTH_EXTENSION(osgearth_contourmap,  ContourMapExtension);
REGISTER_OSGEARTH_EXTENSION(osgearth_contour_map, ContourMapExtension);

bool
ContourMapExtension::connect(MapNode* mapNode)
{
    if ( !_effect.valid() )
        _effect = new ContourMap(*this);

    mapNode->getTerrainEngine()->addEffect( _effect.get() );
    return true;
}

bool
ContourMapExtension::disconnect(MapNode* mapNode)
{
    if ( mapNode )
        mapNode->getTerrainEngine()->removeEffect( _effect.get() );
    return true;
}
