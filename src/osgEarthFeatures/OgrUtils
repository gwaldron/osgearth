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
#ifndef OSGEARTHFEATURES_FEATURE_OGR_UTILS
#define OSGEARTHFEATURES_FEATURE_OGR_UTILS 1

#include <osgEarthFeatures/Common>
#include <osgEarthFeatures/Feature>
#include <osgEarthSymbology/Geometry>
#include <osgEarth/StringUtils>
#include <osg/Notify>
#include <ogr_api.h>

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

struct OSGEARTHFEATURES_EXPORT OgrUtils
{
    static void populate( OGRGeometryH geomHandle, Symbology::Geometry* target, int numPoints );
    
    static Symbology::Polygon* createPolygon( OGRGeometryH geomHandle );
       
    static Symbology::Geometry* createGeometry( OGRGeometryH geomHandle );

    static OGRGeometryH encodePart( const Geometry* geometry, OGRwkbGeometryType part_type );

    static OGRGeometryH encodeShape( const Geometry* geometry, OGRwkbGeometryType shape_type, OGRwkbGeometryType part_type );    

    static OGRGeometryH createOgrGeometry(const Geometry* geometry, OGRwkbGeometryType requestedType = wkbUnknown);

    static Feature* createFeature( OGRFeatureH handle, const FeatureProfile* profile );
    
    static AttributeType getAttributeType( OGRFieldType type );

    static OGRwkbGeometryType getOGRGeometryType(const Geometry::Type& type);
    
    static OGRwkbGeometryType getOGRGeometryType(const Geometry* geometry);

private:
    
    static Feature* createFeature( OGRFeatureH handle, const SpatialReference* srs );
};


#endif // OSGEARTHFEATURES_FEATURE_OGR_GEOM_UTILS

