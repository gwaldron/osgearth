/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2014 Pelican Mapping
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
#include "BillboardExtension"
#include "BillboardShaders"

#include <osg/Point>
#include <osg/Texture2D>

#include <osgEarth/ElevationQuery>
#include <osgEarth/MapNode>
#include <osgEarth/Registry>
#include <osgEarth/ShaderGenerator>
#include <osgEarth/VirtualProgram>
#include <osgEarthFeatures/TransformFilter>

using namespace osgEarth;
using namespace osgEarth::Billboard;
using namespace osgEarth::Features;

#define LC "[BillboardExtension] "


BillboardExtension::BillboardExtension()
{
    //nop
}

BillboardExtension::BillboardExtension(const BillboardOptions& options) :
_options( options )
{
    //nop
}

BillboardExtension::~BillboardExtension()
{
    //nop
}

void
BillboardExtension::setDBOptions(const osgDB::Options* dbOptions)
{
    _dbOptions = dbOptions;
}

bool
BillboardExtension::connect(MapNode* mapNode)
{
    if ( !mapNode )
    {
        OE_WARN << LC << "Illegal: MapNode cannot be null." << std::endl;
        return false;
    }

    OE_INFO << LC << "Connecting to MapNode.\n";

    if ( !_options.imageURI().isSet() )
    {
        OE_WARN << LC << "Illegal: image URI is required" << std::endl;
        return false;
    }

    if ( !_options.featureOptions().isSet() )
    {
        OE_WARN << LC << "Illegal: feature source is required" << std::endl;
        return false;
    }
    
    _features = FeatureSourceFactory::create( _options.featureOptions().value() );
    if ( !_features.valid() )
    {
        OE_WARN << LC << "Illegal: no valid feature source provided" << std::endl;
        return false;
    }

    //if ( _features->getGeometryType() != osgEarth::Symbology::Geometry::TYPE_POINTSET )
    //{
    //    OE_WARN << LC << "Illegal: only points currently supported" << std::endl;
    //    return false;
    //}

    _features->initialize( _dbOptions );

    osg::Vec3dArray* verts;
    if ( _features->getFeatureProfile() )
    {
        verts = new osg::Vec3dArray();

        osg::ref_ptr<FeatureCursor> cursor = _features->createFeatureCursor();
        while ( cursor.valid() && cursor->hasMore() )
        {
            Feature* f = cursor->nextFeature();
            if ( f && f->getGeometry() )
            {
                // Init a filter to tranform feature in desired SRS 
                if (!mapNode->getMapSRS()->isEquivalentTo(_features->getFeatureProfile()->getSRS()))
                {
                    FilterContext cx;
                    cx.setProfile( new FeatureProfile(_features->getFeatureProfile()->getExtent()) );

                    TransformFilter xform( mapNode->getMapSRS() );
                    FeatureList featureList;
                    featureList.push_back(f);
                    cx = xform.push(featureList, cx);
                }

                osg::ref_ptr<osg::Vec3dArray> fVerts = f->getGeometry()->toVec3dArray();
                verts->insert(verts->end(), fVerts->begin(), fVerts->end());
            }
        }
    }
    else
    {
        OE_WARN << LC << "Illegal: feature source has no SRS" << std::endl;
        return false;
    }


    if ( verts && verts->size() > 0 )
    {
        GeoPoint centroid;
        _features->getFeatureProfile()->getExtent().getCentroid(centroid);

        osg::Matrixd l2w;
        centroid.createLocalToWorld(l2w);
        //mapNode->getMapSRS()->createLocalToWorld(centroid, l2w);

        osg::ref_ptr<const osgEarth::SpatialReference> tangentSRS = mapNode->getMapSRS()->createTangentPlaneSRS(centroid.vec3d());

        osg::Matrixd ltp_l2w;
        tangentSRS->createLocalToWorld(osg::Vec3d(0.0, 0.0, 0.0), ltp_l2w);
        l2w.preMult(ltp_l2w);

        osg::MatrixTransform* mt = new osg::MatrixTransform;
        mt->setMatrix(l2w);

        osgEarth::ElevationQuery eq(mapNode->getMap());
        eq.getElevations(verts->asVector(), mapNode->getMapSRS());

        for (int i=0; i < verts->size(); i++)
        {
            GeoPoint vert(mapNode->getMapSRS(), (*verts)[i].x(), (*verts)[i].y(), (*verts)[i].z(), osgEarth::ALTMODE_ABSOLUTE);
            GeoPoint vert_ltp = vert.transform(tangentSRS);
            (*verts)[i].set(vert_ltp.vec3d());
        }

        osg::Geometry* geometry = new osg::Geometry();
        geometry->setVertexArray( verts );

        osg::Vec4Array* colors = new osg::Vec4Array;
        colors->push_back(osg::Vec4(1.0f,1.0f,1.0f,1.0f));
        geometry->setColorArray(colors);
        geometry->setColorBinding(osg::Geometry::BIND_OVERALL);

        geometry->addPrimitiveSet( new osg::DrawArrays( GL_POINTS, 0, verts->size() ) );
        geometry->getOrCreateStateSet()->setAttribute( new osg::Point( 1.0f ), osg::StateAttribute::ON ); 

        //create image and texture to render to
        osg::Texture2D* tex = new osg::Texture2D(_options.imageURI()->getImage(_dbOptions));
        tex->setResizeNonPowerOfTwoHint(false);
        tex->setFilter( osg::Texture::MIN_FILTER, osg::Texture::NEAREST );
        tex->setFilter( osg::Texture::MAG_FILTER, osg::Texture::NEAREST );
        tex->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
        tex->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);

        geometry->setName("BillboardPoints");

        osg::Geode* geode = new osg::Geode;
        geode->addDrawable(geometry);

        osg::ref_ptr<StateSetCache> cache = new StateSetCache();
        Registry::shaderGenerator().run(geode, cache.get());

        //VirtualProgram* vp = VirtualProgram::getOrCreate( geode->getOrCreateStateSet() );
        //vp->setFunction("oe_modelsplat_vert_model", vs_model, ShaderComp::LOCATION_VERTEX_MODEL);
        //vp->setFunction("make_it_yellow", billboardFragmentShader, ShaderComp::LOCATION_FRAGMENT_COLORING, 2.0f);

        //osg::Shader* main = new osg::Shader(osg::Shader::GEOMETRY, billboardGeomShader);
        //vp->setShader("billboard_make_quad", main);

        //set the texture related uniforms
        osg::StateSet* geode_ss = geode->getOrCreateStateSet();
        geode_ss->setTextureAttributeAndModes( 2, tex, 1 );
        geode_ss->getOrCreateUniform("billboard_tex", osg::Uniform::SAMPLER_2D)->set( 2 );

        float bbWidth = (float)tex->getImage()->s() / 2.0f;
        float bbHeight = (float)tex->getImage()->t();
        float aspect = (float)tex->getImage()->s() / (float)tex->getImage()->t();
        if (_options.height().isSet())
        {
            bbHeight = _options.height().get();
            if (!_options.width().isSet())
            {
                bbWidth = bbHeight * aspect / 2.0f;
            }
        }
        if (_options.width().isSet())
        {
            bbWidth = _options.width().get() / 2.0f;
            if (!_options.height().isSet())
            {
                bbHeight = _options.width().get() / aspect;
            }
        }

        geode_ss->getOrCreateUniform("billboard_width", osg::Uniform::FLOAT)->set( bbWidth );
        geode_ss->getOrCreateUniform("billboard_height", osg::Uniform::FLOAT)->set( bbHeight );
        geode_ss->setMode(GL_BLEND, osg::StateAttribute::ON);

        osg::Program* pgm = new osg::Program;
        pgm->setName("billboard_program");
        pgm->addShader( new osg::Shader( osg::Shader::VERTEX, billboardVertShader ) );
        pgm->addShader( new osg::Shader( osg::Shader::GEOMETRY, billboardGeomShader ) );
        pgm->addShader( new osg::Shader( osg::Shader::FRAGMENT, billboardFragmentShader ) );
        pgm->setParameter( GL_GEOMETRY_VERTICES_OUT_EXT, 4 );
        pgm->setParameter( GL_GEOMETRY_INPUT_TYPE_EXT, GL_POINTS );
        pgm->setParameter( GL_GEOMETRY_OUTPUT_TYPE_EXT, GL_TRIANGLE_STRIP );
        geode_ss->setAttribute(pgm);

        geode_ss->setMode( GL_CULL_FACE, osg::StateAttribute::OFF );
        geode->setCullingActive(false);

        mt->addChild(geode);
        mapNode->getModelLayerGroup()->addChild(mt);

        return true;
    }

    return false;
}

bool
BillboardExtension::disconnect(MapNode* mapNode)
{
    if ( mapNode )
    {
        //TODO
    }

    return true;
}

