/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2020 Pelican Mapping
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
#include <osgEarth/CropFilter>

#define LC "[CropFilter] "

using namespace osgEarth;

CropFilter::CropFilter(CropFilter::Method method) :
    _method(method)
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
        FeatureList output;
        output.reserve(input.size());

        for(auto& feature : input)
        {
            if (feature.valid())
            {
                bool keepFeature = false;
                Geometry* featureGeom = feature->getGeometry();

                if (featureGeom && featureGeom->isValid())
                {
                    Bounds bounds = featureGeom->getBounds();
                    if (bounds.valid())
                    {
                        osg::Vec3d centroid = bounds.center();
                        if (extent.contains(centroid.x(), centroid.y()))
                        {
                            keepFeature = true;
                            newExtent.expandToInclude(bounds.xMin(), bounds.yMin());
                            newExtent.expandToInclude(bounds.xMax(), bounds.yMax());
                        }
                    }
                }

                if (keepFeature)
                {
                    output.emplace_back(feature);
                }
            }
        }

        input.swap(output);
    }
    
    else if (_method == METHOD_CROP_TO_EXTENT) //(requires GEOS)
    {
#ifdef OSGEARTH_HAVE_GEOS
        // create the intersection polygon:
        osg::ref_ptr<Polygon> poly;
        FeatureList output;
        output.reserve(input.size());
        
        for(auto& feature : input)
        {
            bool keepFeature = false;
            if (feature.valid())
            {
                Geometry* featureGeom = feature->getGeometry();
                if (featureGeom && featureGeom->isValid())
                {
                    // test for trivial acceptance:
                    GeoExtent featureExtent = feature->getExtent();
                    if (featureExtent.isInvalid())
                    {
                        //nop
                    }

                    else if (extent.contains(featureExtent))
                    {
                        keepFeature = true;
                        newExtent.expandToInclude(featureExtent);
                    }

                    // then move on to the cropping operation:
                    else
                    {
                        if (!poly.valid())
                        {
                            poly = new Polygon();
                            poly->push_back(osg::Vec3d(extent.xMin(), extent.yMin(), 0));
                            poly->push_back(osg::Vec3d(extent.xMax(), extent.yMin(), 0));
                            poly->push_back(osg::Vec3d(extent.xMax(), extent.yMax(), 0));
                            poly->push_back(osg::Vec3d(extent.xMin(), extent.yMax(), 0));
                        }

                        osg::ref_ptr<Geometry> croppedGeometry;
                        if (featureGeom->crop(poly.get(), croppedGeometry))
                        {
                            if (croppedGeometry->isValid())
                            {
                                feature->setGeometry(croppedGeometry.get());
                                keepFeature = true;
                                newExtent.expandToInclude(GeoExtent(newExtent.getSRS(), croppedGeometry->getBounds()));
                            }
                        }
                    }
                }
            }

            if (keepFeature)
            {
                output.emplace_back(feature);
            }
        }  

        output.swap(input);

#else // OSGEARTH_HAVE_GEOS

        OE_WARN << "CropFilter - METHOD_CROPPING not available - please compile osgEarth with GEOS" << std::endl;
        return context;

#endif
    }
    
    else // CROP_BBOX
    {
        FeatureList output;
        output.reserve(input.size());

        for (auto& feature : input)
        {
            if (feature.valid())
            {
                GeoExtent featureExtent = feature->getExtent();
                if (extent.intersects(featureExtent))
                {
                    output.emplace_back(feature);
                }
            }
        }

        output.swap(input);
    }

    FilterContext newContext = context;
    newContext.extent() = newExtent;

    return newContext;
}
