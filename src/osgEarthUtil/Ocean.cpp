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
#include <osgEarthUtil/Ocean>
#include <osgEarth/Registry>
#include <osgEarth/CullingUtils>
#include <osgEarth/MapNode>
#include <osgDB/ReadFile>

using namespace osgEarth;
using namespace osgEarth::Util;

#undef  LC
#define LC "[Ocean] "

//------------------------------------------------------------------------

OceanOptions::OceanOptions(const ConfigOptions& options) :
DriverConfigOptions( options ),
_maxAltitude       ( 20000.0 )
{
    fromConfig(_conf);
}

void
OceanOptions::fromConfig( const Config& conf )
{
    conf.getIfSet( "max_altitude", _maxAltitude );
}

void
OceanOptions::mergeConfig( const Config& conf )
{
    DriverConfigOptions::mergeConfig( conf );
    fromConfig( conf );
}

Config
OceanOptions::getConfig() const
{
    Config conf = DriverConfigOptions::getConfig();
    conf.addIfSet( "max_altitude", _maxAltitude );
    return conf;
}

//------------------------------------------------------------------------

#undef  LC
#define LC "[OceanNode] "

OceanNode::OceanNode(const OceanOptions& options) :
_options ( options ),
_seaLevel( 0.0f ),
_alpha( 1.0f )
{
    //NOP
}

OceanNode::~OceanNode()
{
    //nop
}

void
OceanNode::setSeaLevel(float value)
{
    _seaLevel = value;
    onSetSeaLevel();
}

void
OceanNode::setAlpha(float value)
{
    _alpha = value;
    onSetAlpha();
}

void
OceanNode::traverse(osg::NodeVisitor& nv)
{
    if ( nv.getVisitorType() == nv.CULL_VISITOR && _srs.valid() )
    {
        osgUtil::CullVisitor* cv = Culling::asCullVisitor(nv);
        if ( cv->getCurrentCamera() )
        {
            // find the current altitude:
            osg::Vec3d eye = osg::Vec3d(0,0,0) * cv->getCurrentCamera()->getInverseViewMatrix();
            osg::Vec3d local;
            double altitude;
            _srs->transformFromWorld(eye, local, &altitude);

            // check against max altitude:
            if ( altitude > _options.maxAltitude().get() )
                return;
            
            // Set the near clip plane to account for an ocean sphere.
            // First, adjust for the sea level offset:
            altitude -= (double)getSeaLevel();

            // clamp the absolute value so it will work above or below sea level
            // and so we don't attempt to set the near clip below 1:
            altitude = std::max( ::fabs(altitude), 1.0 );

            // we don't want the ocean participating in the N/F calculation:
            osg::CullSettings::ComputeNearFarMode mode = cv->getComputeNearFarMode();
            cv->setComputeNearFarMode( osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR );

            // visit the ocean:
            osg::Group::traverse( nv );

            cv->setComputeNearFarMode( mode );

            // just use the height above (or below) the ocean as the near clip
            // plane distance. Close enough and errs on the safe side.
            double oldNear = cv->getCalculatedNearPlane();

            double newNear = std::min( oldNear, altitude );
            if ( newNear < oldNear )
            {
                cv->setCalculatedNearPlane( newNear );
            }

            return;
        }
    }
    osg::Group::traverse( nv );
}

//------------------------------------------------------------------------

#define MAPNODE_TAG "__osgEarth::MapNode"
#define OPTIONS_TAG "__osgEarth::Util::OceanOptions"

OceanNode*
OceanNode::create(const OceanOptions& options, MapNode* mapNode)
{
    if ( !mapNode ) {
        OE_WARN << LC << "Internal error; null map node passed to OceanNode::Create\n";
        return 0L;
    }

    std::string driverName = options.getDriver();
    if ( driverName.empty() )
        driverName = "simple";

    std::string extensionName = std::string("ocean_") + driverName;

    osg::ref_ptr<Extension> extension = Extension::create( extensionName, options );
    if ( !extension.valid() ) {
        OE_WARN << LC << "Failed to load extension for sky driver \"" << driverName << "\"\n";
        return 0L;
    }

    OceanNodeFactory* factory = extension->as<OceanNodeFactory>();
    if ( !factory ) {
        OE_WARN << LC << "Internal error; extension \"" << extensionName << "\" does not implement OceanNodeFactory\n";
        return 0L;
    }

    osg::ref_ptr<OceanNode> result = factory->createOceanNode(mapNode);

    return result.release();
}

OceanNode*
OceanNode::create(MapNode* mapNode)
{
    OceanOptions options;
    return create(options, mapNode);
}

//------------------------------------------------------------------------

const OceanOptions&
OceanDriver::getOceanOptions(const osgDB::Options* options) const
{
    static OceanOptions s_default;
    const void* data = options->getPluginData(OPTIONS_TAG);
    return data ? *static_cast<const OceanOptions*>(data) : s_default;
}


MapNode*
OceanDriver::getMapNode(const osgDB::Options* options) const
{
    return const_cast<MapNode*>(
        static_cast<const MapNode*>(
            options->getPluginData(MAPNODE_TAG) ) );
}
