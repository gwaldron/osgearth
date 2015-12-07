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
#include "LandCoverTilePatchCallback"
#include <osgEarth/TraversalData>
#include <osgEarth/Shadowing>
#include <osgUtil/CullVisitor>

using namespace osgEarth;
using namespace osgEarth::Splat;

#define LC "[LandCoverTilePatchCallback] "

void
LandCoverTilePatchCallback::cull(osgUtil::CullVisitor* cv,
                                 const TileKey&        key,
                                 osg::StateSet*        tileStateSet,
                                 osg::Node*            tilePatch)
{
    RefUID* zoneIndex = VisitorData::fetch<RefUID>(*cv, "oe.LandCover.zoneIndex");
    if ( zoneIndex && (*zoneIndex) < _zones.size() )
    {
        // Figure out what type of camera this is:
        unsigned clearMask = cv->getCurrentCamera()->getClearMask();
        bool isDepthCamera = ((clearMask & GL_COLOR_BUFFER_BIT) == 0u) && ((clearMask & GL_DEPTH_BUFFER_BIT) != 0u);
        bool isShadowCamera = osgEarth::Shadowing::isShadowCamera(cv->getCurrentCamera());

        // only consider land cover if we are capturing color OR shadow map.
        if ( isShadowCamera || !isDepthCamera )
        {
            bool pushedTileStateSet = false;

            //ZoneData& zone = _zones[*zoneIndex];
            Zone* zone = _zones[*zoneIndex].get();

            LandCoverLayers& layers = zone->getLandCover()->getLayers();
            for(int i=0; i<layers.size(); ++i)
            {
                //LayerData& layerData = zone._layers[i];
                LandCoverLayer* layer = layers[i].get();

                if ( layer->getLOD() == key.getLOD() &&
                    (!isShadowCamera || layer->getCastShadows()) )
                {
                    if ( !pushedTileStateSet )
                    {
                        cv->pushStateSet( tileStateSet );
                        pushedTileStateSet = true;
                    }

                    cv->pushStateSet( layer->getStateSet() );

                    tilePatch->accept( *cv );

                    cv->popStateSet();
                }
            }

            if ( pushedTileStateSet )
            {
                cv->popStateSet();
            }
        }
    }
}

void
LandCoverTilePatchCallback::release(const TileKey& key)
{
    // nop - implementation is stateless.
}
