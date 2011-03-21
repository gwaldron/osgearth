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
#include <osgEarthFeatures/SubstituteModelFilter>
#include <osgEarth/HTTPClient>
#include <osg/Drawable>
#include <osg/Geode>
#include <osg/MatrixTransform>
#include <osg/NodeVisitor>
#include <osgUtil/SmoothingVisitor>
#include <list>
#include <deque>

#define LC "[SubstituteModelFilter] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

//------------------------------------------------------------------------

namespace
{
    static osg::Node* s_defaultModel =0L;
}

//------------------------------------------------------------------------

SubstituteModelFilter::SubstituteModelFilter( const Style* style ) :
_style( style ),
_cluster( false )
{
    //NOP
}

bool
SubstituteModelFilter::push(Feature*                     input,
                            SubstituteModelFilter::Data& data,
                            osg::Group*                  attachPoint,
                            FilterContext&               context )
{
    GeometryIterator gi( input->getGeometry() );
    gi.traversePolygonHoles() = false;

    while( gi.hasMore() )
    {
        Geometry* geom = gi.next();

        //TODO: this is the simple, inefficient way of doing it.
        //TODO: add vertex shifting and clustering support.
        for( unsigned i=0; i<geom->size(); ++i )
        {
            const osg::Vec3d& point = (*geom)[i];
            osg::MatrixTransform* xform = new osg::MatrixTransform();
            xform->setMatrix( _modelMatrix * osg::Matrixd::translate( point ) );
            xform->addChild( data._model.get() );
            attachPoint->addChild( xform );
        }
    }

    return true;
}


//clustering:
//  troll the external model for geodes. for each geode, create a geode in the target
//  model. then, for each geometry in that geode, replicate it once for each instance of
//  the model in the feature batch and transform the actual verts to a local offset
//  relative to the tile centroid. Finally, reassemble all the geodes and optimize. 
//  hopefully stateset sharing etc will work out. we may need to strip out LODs too.
bool
SubstituteModelFilter::cluster(const FeatureList&           features,
                               SubstituteModelFilter::Data& data,
                               osg::Group*                  attachPoint,
                               FilterContext&               cx )
{
    struct ClusterVisitor : public osg::NodeVisitor
    {
        ClusterVisitor( const FeatureList& features, const osg::Matrixd& modelMatrix, FilterContext& cx )
            : _features( features ),
              _modelMatrix( modelMatrix ),
              _cx( cx ),
              osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN )
        {
            //nop
        }

        void apply( osg::Geode& geode )
        {
            // save the geode's drawables..
            osg::Geode::DrawableList old_drawables = geode.getDrawableList();

            // ..and clear out the drawables list.
            geode.removeDrawables( 0, geode.getNumDrawables() );

            // foreach each drawable that was originally in the geode...
            for( osg::Geode::DrawableList::iterator i = old_drawables.begin(); i != old_drawables.end(); i++ )
            {
                osg::Drawable* old_d = i->get();

                // go through the list of input features...
                for( FeatureList::const_iterator j = _features.begin(); j != _features.end(); j++ )
                {
                    const Feature* feature = j->get();

                    // ...and clone the source drawable once for each input feature.
                    osg::ref_ptr<osg::Drawable> new_d = dynamic_cast<osg::Drawable*>( 
                        old_d->clone( osg::CopyOp::DEEP_COPY_ARRAYS | osg::CopyOp::DEEP_COPY_PRIMITIVES ) );

                    if ( dynamic_cast<osg::Geometry*>( new_d.get() ) )
                    {
                        osg::Geometry* geom = static_cast<osg::Geometry*>( new_d.get() );
                        osg::Vec3Array* verts = dynamic_cast<osg::Vec3Array*>( geom->getVertexArray() );
                        if ( verts )
                        {
                            // now, forceably offset the new cloned drawable by the input feature's first point.
                            // grab the centroid just in case:
                            const Geometry* fg = feature->getGeometry();
                            if ( fg == 0L || fg->getTotalPointCount() == 0 )
                                continue;

                            // get the first point in the geometry to use as an offset.
                            osg::Vec3d c;
                            ConstGeometryIterator gi( feature->getGeometry() );
                            while ( gi.hasMore() ) {
                                const Geometry* fg = gi.next();
                                if ( fg->size() > 0 ) {
                                    c = (*fg)[0];
                                    break;
                                }
                            }

                            osg::Matrix translate_mx = osg::Matrix::translate( c );

                            // get the scaler if there is one:
                            osg::Matrix scale_mx;
#if 0
                            if ( filter->getModelScaleScript() )
                            {
                                ScriptResult r = env->getScriptEngine()->run( filter->getModelScaleScript(), j->get(), env );
                                if ( r.isValid() )
                                    scale_mx = osg::Matrix::scale( r.asVec3() );
                                else
                                    env->getReport()->error( r.asString() );
                                //scaler = r.asVec3();
                            }
#endif

                            // and the heading id there is one:
                            osg::Matrix heading_mx;
#if 0
                            if ( filter->getModelHeadingScript() )
                            {
                                ScriptResult r = env->getScriptEngine()->run( filter->getModelHeadingScript(), j->get(), env );
                                if ( r.isValid() )
                                    heading_mx = osg::Matrix::rotate( osg::DegreesToRadians( r.asDouble(0) ), 0, 0, -1 );
                                else
                                    env->getReport()->error( r.asString() );
                            }
#endif

                            osg::Matrix mx = _modelMatrix * translate_mx; //heading_mx * scale_mx * translate_mx;

                            // transform all the verts.
                            for( osg::Vec3Array::iterator k = verts->begin(); k != verts->end(); ++k )
                                (*k).set( *k * mx );
                        }

                        // add the new cloned, translated drawable back to the geode.
                        geode.addDrawable( geom );
                    }
                }
            }

            geode.dirtyBound();

            // merge the geometry...
            osgUtil::Optimizer opt;
            opt.optimize( &geode, osgUtil::Optimizer::MERGE_GEOMETRY );

#if 0

            // automatically generate normals.
            // TODO: maybe this should be an option.
            osgUtil::SmoothingVisitor smoother;
            geode.accept( smoother );
#endif
            
            osg::NodeVisitor::apply( geode );
        }

    private:
        const FeatureList&   _features;
        FilterContext        _cx;
        osg::Matrixd         _modelMatrix;
    };


    // make a copy of the model:
	osg::Node* clone = dynamic_cast<osg::Node*>( data._model->clone( osg::CopyOp::DEEP_COPY_ALL ) );

    // ..and apply the clustering to the copy.
	ClusterVisitor cv( features, _modelMatrix, cx );
	clone->accept( cv );

    attachPoint->addChild( clone );
    return true;
}

FilterContext
SubstituteModelFilter::push(FeatureList&         features, 
                            const FilterContext& context )
{
    if ( !isSupported() ) {
        OE_WARN << "SubstituteModelFilter support not enabled" << std::endl;
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

    osg::Group* group = new osg::Group();

    bool ok = true;

    if ( _cluster )
    {
        ok = cluster( features, data, group, newContext );
    }

    else
    {
        for( FeatureList::iterator i = features.begin(); i != features.end(); ++i )
            if ( !push( i->get(), data, group, newContext ) )
                ok = false;
    }

    _result = group;

    return newContext;
}
