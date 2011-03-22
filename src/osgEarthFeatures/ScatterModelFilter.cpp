/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2010 Pelican Mapping
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
#include <osgEarthFeatures/ScatterModelFilter>
#include <stdlib.h>

#define LC "[ScatterModelFilter] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

//------------------------------------------------------------------------

ScatterModelFilter::ScatterModelFilter( const Style* style ) :
SubstituteModelFilter( style ),
_density( 10.0f )
{
    //NOP
}

bool
ScatterModelFilter::scatterGeometry(FeatureList&         features,
                                    const FilterContext& context)
{
    // seed the random number generator so the randomness is the same each time
    // todo: control this seeding based on the feature source name, perhaps?
    ::srand( 0 );

    for( FeatureList::iterator i = features.begin(); i != features.end(); ++i )
    {
        Feature* f = i->get();
        
        // must contain an aereal geometry:
        Ring* ring = dynamic_cast<Ring*>( f->getGeometry() );
        if ( !ring )
            continue;

        double areaSqKm = 0.001 * ::fabs( ring->getSignedArea2D() );
        unsigned numInstances = areaSqKm * (double) osg::clampAbove( 0.1f, _density );

        if ( numInstances > 0 )
        {
            double zMin = 0.0; //TODO: find the minimum Z, I guess

            Bounds bounds = ring->getBounds();
            PointSet* points = new PointSet();

            for( unsigned j=0; j<numInstances; ++j )
            {
                double rx = ((double)::rand())/(double)RAND_MAX;
                double ry = ((double)::rand())/(double)RAND_MAX;

                double x = bounds.xMin() + rx * bounds.width();
                double y = bounds.yMin() + ry * bounds.height();

                points->push_back( osg::Vec3d( x, y, zMin ) );
            }

            // replace the poly with the scattered points.
            f->setGeometry( points );
        }
    }

    return true;
}

FilterContext
ScatterModelFilter::push(FeatureList&         features, 
                         const FilterContext& context )
{
    if ( !isSupported() ) {
        OE_WARN << LC << "support for this filter is not enabled" << std::endl;
        return context;
    }

    if ( !_style.valid() ) {
        OE_WARN << LC << "No style supplied; cannot process features" << std::endl;
        return context;
    }

    const ModelSymbol* symbol = _style->getSymbol<ModelSymbol>();
    if ( !symbol ) {
        OE_WARN << LC << "No ModelSymbol found in style; cannot process feautres" << std::endl;
        return context;
    }

    FilterContext newContext( context );

    // assemble the data for this pass
    Data data;
    data._model = newContext.getSession()->getModel( *symbol->url() );
    if ( !data._model.valid() )
    {
        OE_WARN << LC << "Unable to load model from \"" << *symbol->url() << "\"" << std::endl;
        return newContext;
    }

    // group to which to attach the resulting geode(s)
    osg::Group* group = new osg::Group();

    bool ok = true;

    // replace the features' original geometry with scattered points.
    ok = scatterGeometry( features, newContext );

    if ( _cluster )
    {
        ok = cluster( features, data, group, newContext );
    }

    else
    {
        for( FeatureList::iterator i = features.begin(); i != features.end(); ++i )
            if ( !pushFeature( i->get(), data, group, newContext ) )
                ok = false;
    }

    _result = group;

    return newContext;
}
