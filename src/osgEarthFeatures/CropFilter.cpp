/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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
#include <osgEarthFeatures/CropFilter>

#define LC "[CropFilter] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

CropFilter::CropFilter( CropFilter::Method method ) :
_method( method )
{
    //nop
}

FilterContext
CropFilter::push( FeatureList& input, FilterContext& context )
{
    if ( !context.extent().isSet() )
    {
        OE_WARN << LC << "Extent is not set (and is required)" << std::endl;
        return context;
    }

    const GeoExtent& extent = *context.extent();

    GeoExtent newExtent( extent.getSRS() );

    if ( _method == METHOD_CENTROID )
    {
        for( FeatureList::iterator i = input.begin(); i != input.end();  )
        {
            bool keepFeature = false;

            Feature* feature = i->get();
            Geometry* featureGeom = feature->getGeometry();

            if ( featureGeom && featureGeom->isValid() )
            {
                Bounds bounds = featureGeom->getBounds();
                if ( bounds.isValid() )
                {
                    osg::Vec3d centroid = bounds.center();
                    if ( extent.contains( centroid.x(), centroid.y() ) )
                    {
                        keepFeature = true;
                        newExtent.expandToInclude( bounds.xMin(), bounds.yMin() );
                    }
                }
            }

            if ( keepFeature )
                ++i;
            else
                i = input.erase( i );
        }
    }

    else // METHOD_CROPPING (requires GEOS)
    {
#ifdef OSGEARTH_HAVE_GEOS

        // create the intersection polygon:
        osg::ref_ptr<Symbology::Polygon> poly;
        
        for( FeatureList::iterator i = input.begin(); i != input.end();  )
        {
            bool keepFeature = false;

            Feature* feature = i->get();

            Symbology::Geometry* featureGeom = feature->getGeometry();
            if ( featureGeom && featureGeom->isValid() )
            {
                // test for trivial acceptance:
                const Bounds bounds = featureGeom->getBounds();
                if ( !bounds.isValid() )
                {
                    //nop
                }

                else if ( extent.contains( bounds ) )
                {
                    keepFeature = true;
                    newExtent.expandToInclude( bounds );
                }

                // then move on to the cropping operation:
                else
                {
                    if ( !poly.valid() )
                    {
                        poly = new Symbology::Polygon();
                        poly->push_back( osg::Vec3d( extent.xMin(), extent.yMin(), 0 ));
                        poly->push_back( osg::Vec3d( extent.xMax(), extent.yMin(), 0 ));
                        poly->push_back( osg::Vec3d( extent.xMax(), extent.yMax(), 0 ));
                        poly->push_back( osg::Vec3d( extent.xMin(), extent.yMax(), 0 ));
                    }

                    osg::ref_ptr<Geometry> croppedGeometry;
                    if ( featureGeom->crop( poly.get(), croppedGeometry ) )
                    {
                        if ( croppedGeometry->isValid() )
                        {
                            feature->setGeometry( croppedGeometry.get() );
                            keepFeature = true;
                            newExtent.expandToInclude( croppedGeometry->getBounds() );
                        }
                    }
                }
            }

            if ( keepFeature )
                ++i;
            else
                i = input.erase( i );
        }  

#else // OSGEARTH_HAVE_GEOS

        OE_WARN << "CropFilter - METHOD_CROPPING not available - please compile osgEarth with GEOS" << std::endl;
        return context;

#endif
    }

    FilterContext newContext = context;
    newContext.extent() = newExtent;

    return newContext;
}
