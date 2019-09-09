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
#ifndef OSGEARTH_ANNOTATION_SETTINGS_H
#define OSGEARTH_ANNOTATION_SETTINGS_H 1

#include <osgEarthAnnotation/Common>
#include <osg/NodeVisitor>

namespace osgEarth { namespace Annotation
{	
    /**
     * Global default settings for controlling annotation behavior
     */
    class OSGEARTHANNO_EXPORT AnnotationSettings
    {
    public:
        /**
         * Gets or sets the max camera altitude at which the occlusion culling callback is applied.
         * DEFAULT: 200000.0 meters
         */
        static void setOcclusionCullingMaxAltitude( double value ) { _occlusionCullingMaxAltitude = value; }
        static double getOcclusionCullingMaxAltitude() { return _occlusionCullingMaxAltitude; }


    private:
        static double _occlusionCullingMaxAltitude;
    };

} } // namespace osgEarth::Annotation

#endif //OSGEARTH_ANNOTATION_SETTINGS_H
