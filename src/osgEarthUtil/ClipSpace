/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2019 Pelican Mapping
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

#ifndef OSGEARTHUTIL_CLIPSPACE_H
#define OSGEARTHUTIL_CLIPSPACE_H

#include <osgEarthUtil/Common>
#include <osgEarth/GeoData>

namespace osgEarth { namespace Util
{	

    /**
    * Utility class for perform operations in clip space
    */
    class OSGEARTHUTIL_EXPORT ClipSpace
    {
    public:
        osg::Matrix _worldToClip, _clipToWorld;

        ClipSpace(const osg::Matrix& MVP, const osg::Matrix& MVPinv);

        // Moves the input point to the bottom edge of the viewport.
        void clampToBottom(GeoPoint& p);

        // Moves the input point to left edge of the viewport.
        void clampToLeft(GeoPoint& p);
    };

} } // namespace osgEarth::Util

#endif //OSGEARTHUTIL_CLIPSPACE_H
