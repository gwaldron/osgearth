/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/CentroidFilter>
#include <osgEarth/FilterContext>

#define LC "[CentroidFilter] "

using namespace osgEarth;

//------------------------------------------------------------------------

CentroidFilter::CentroidFilter()
{
    //NOP
}

FilterContext
CentroidFilter::push(FeatureList& features, FilterContext& context )
{
    for( FeatureList::iterator i = features.begin(); i != features.end(); ++i )
    {
        Feature* f = i->get();
        
        Geometry* geom = f->getGeometry();
        if ( !geom )
            continue;

        PointSet* newGeom = new PointSet();
        newGeom->push_back( geom->getBounds().center() );

        f->setGeometry( newGeom );
    }

    return context;
}
