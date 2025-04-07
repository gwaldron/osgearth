/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/CropFilter>

#define LC "[CropFilter] "

using namespace osgEarth;

//OSGEARTH_REGISTER_SIMPLE_FEATUREFILTER(crop, CropFilter);

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
        Polygon poly;
        poly.reserve(4);
        poly.push_back(osg::Vec3d(extent.xMin(), extent.yMin(), 0));
        poly.push_back(osg::Vec3d(extent.xMax(), extent.yMin(), 0));
        poly.push_back(osg::Vec3d(extent.xMax(), extent.yMax(), 0));
        poly.push_back(osg::Vec3d(extent.xMin(), extent.yMax(), 0));

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
                        osg::ref_ptr<Geometry> cropped = featureGeom->crop(&poly);
                        if (cropped.valid() && cropped->isValid())
                        {
                            feature->setGeometry(cropped);
                            keepFeature = true;
                            newExtent.expandToInclude(GeoExtent(newExtent.getSRS(), cropped->getBounds()));
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
