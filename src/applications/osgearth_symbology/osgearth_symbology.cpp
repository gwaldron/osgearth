/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2009 Pelican Ventures, Inc.
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


#include <osg/Notify>
#include <osgGA/StateSetManipulator>
#include <osgGA/GUIEventHandler>
#include <osgGA/TrackballManipulator>
#include <osgDB/FileNameUtils>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgEarthSymbology/GeometrySymbolizer>
#include <osgEarthSymbology/FeatureDataSetAdapter>
#include <osgEarthSymbology/Style>
#include <osgEarthSymbology/SymbolicNode>
#include <osgEarthDrivers/feature_ogr/OGRFeatureOptions>
#include <osg/MatrixTransform>
#include <osg/Geometry>
#include <osgUtil/Tessellator>
#include <osg/LineWidth>
#include <osg/Point>
#include <osg/Material>

using namespace osgEarth::Symbology;

struct PolygonPointSizeSymbol : public PolygonSymbol
{
    PolygonPointSizeSymbol() : _size (1.0)
    {
    }

    float& size() { return _size; }
    const float& size() const { return _size; }

protected:
    float _size;
};


struct GeometryPointSymbolizer : public GeometrySymbolizer
{
    bool update(FeatureDataSet* dataSet,
                const osgEarth::Symbology::Style* style,
                osg::Group* attachPoint,
                SymbolizerContext* context )
    {
        if (!dataSet || !attachPoint || !style)
            return false;

        osg::ref_ptr<osgEarth::Features::FeatureCursor> cursor = dataSet->createCursor();
        if (!cursor)
            return false;

        osg::ref_ptr<osg::Group> newSymbolized = new osg::Group;
        osg::ref_ptr<osg::Geode> geode = new osg::Geode;
        newSymbolized->addChild(geode.get());

        osgEarth::Features::Feature* feature = 0;
        while( cursor->hasMore() ) 
        {
            feature = cursor->nextFeature();
            if (!feature)
                continue;

            Geometry* geometry = feature->getGeometry();

            osg::ref_ptr<osg::Geometry> osgGeom = new osg::Geometry;
            osg::PrimitiveSet::Mode primMode = osg::PrimitiveSet::POINTS;

            osg::Vec4 color = osg::Vec4(1.0, 0.0, 1.0, 1.);

            switch( geometry->getType())
            {
            case Geometry::TYPE_POINTSET:
                primMode = osg::PrimitiveSet::POINTS;
                if (style->getPoint()) 
                {
                    color = style->getPoint()->fill()->color();

                    float size = style->getPoint()->size().value();
                    osgGeom->getOrCreateStateSet()->setAttributeAndModes( new osg::Point(size) );
                }
                break;

            case Geometry::TYPE_LINESTRING:
                primMode = osg::PrimitiveSet::LINE_STRIP;
                if (style->getLine()) 
                {
                    color = style->getLine()->stroke()->color();
                    float size = style->getLine()->stroke()->width().value();
                    osgGeom->getOrCreateStateSet()->setAttributeAndModes( new osg::LineWidth(size));
                }
                break;

            case Geometry::TYPE_RING:
                primMode = osg::PrimitiveSet::LINE_LOOP;
                if (style->getLine())
                {
                    color = style->getLine()->stroke()->color();
                    float size = style->getLine()->stroke()->width().value();
                    osgGeom->getOrCreateStateSet()->setAttributeAndModes( new osg::LineWidth(size));
                }
                break;

            case Geometry::TYPE_POLYGON:
                // use polygon as point for this specific symbolizer
                // it would be simpler to use the symbol style->getPoint but here
                // we want to dmonstrate how to customize Symbol and Symbolizer
                primMode = osg::PrimitiveSet::POINTS;
                if (style->getPolygon())
                {
                    const PolygonPointSizeSymbol* poly = dynamic_cast<const PolygonPointSizeSymbol*>(style->getPolygon());
                    if (poly) 
                    {
                        color = style->getPolygon()->fill()->color();

                        float size = poly->size();
                        osgGeom->getOrCreateStateSet()->setAttributeAndModes( new osg::Point(size) );
                    }
                }
                break;
            }

            osg::Material* material = new osg::Material;
            material->setDiffuse(osg::Material::FRONT_AND_BACK, color);

            osgGeom->setVertexArray( geometry->toVec3Array() );
            osgGeom->addPrimitiveSet( new osg::DrawArrays( primMode, 0, geometry->size() ) );

            osgGeom->getOrCreateStateSet()->setAttributeAndModes(material);
            osgGeom->getOrCreateStateSet()->setMode(GL_LIGHTING, false);
            geode->addDrawable(osgGeom);

        }

        if (geode->getNumDrawables()) 
        {
            attachPoint->addChild(newSymbolized.get());
            return true;
        }

        return false;
    }
};










struct GeometryVolumeSymbolizer : public GeometrySymbolizer
{

    GeometryVolumeSymbolizer(double offset, double height) : _offset(offset), _height(height)
    {}


    void tessellate( osg::Geometry* geom )
    {
        osgUtil::Tessellator tess;
        tess.setTessellationType( osgUtil::Tessellator::TESS_TYPE_GEOMETRY );
        tess.setWindingType( osgUtil::Tessellator::TESS_WINDING_ODD );
//    tess.setWindingType( osgUtil::Tessellator::TESS_WINDING_POSITIVE );
        tess.retessellatePolygons( *geom );
    }

    osg::Geode* createVolume(Geometry* geom, double offset, double height,SymbolizerContext* context )
    {
        if ( !geom ) return 0L;

        int numRings = 0;

        // start by offsetting the input data.
        {
            GeometryIterator i( geom );
            i.traverseMultiGeometry() = true;
            i.traversePolygonHoles() = true;
            while( i.hasMore() )
            {
                Geometry* part = i.next();
                if (offset != 0.0)
                {
                    for( osg::Vec3dArray::iterator j = part->begin(); j != part->end(); j++ )
                    {
#if 0
                        // not context yet
                        if ( context.isGeocentric() )
                        {
                            osg::Vec3d world = context.toWorld( *j );
                            // TODO: get the proper up vector; this is spherical.. or does it really matter for
                            // stencil volumes?
                            osg::Vec3d offset_vec = world;
                            offset_vec.normalize();
                            *j = context.toLocal( world + offset_vec * offset ); //(*j) += offset_vec * offset;
                        }
                        else
#endif
                        {
                            (*j).z() += offset;
                        }
                    }
                }

                // in the meantime, count the # of closed geoms. We will need to know this in 
                // order to pre-allocate the proper # of verts.
                if ( part->getType() == Geometry::TYPE_POLYGON || part->getType() == Geometry::TYPE_RING )
                {
                    numRings++;
                }
            }
        }

        // now, go thru and remove any coplanar segments from the geometry. The tesselator will
        // not work include a vert connecting two colinear segments in the tesselation, and this
        // will break the stenciling logic.
#define PARALLEL_EPSILON 0.01
        GeometryIterator i( geom );
        i.traverseMultiGeometry() = true;
        i.traversePolygonHoles() = true;
        while( i.hasMore() )
        {
            Geometry* part = i.next();
            if ( part->size() >= 3 )
            {
                osg::Vec3d prevVec = part->front() - part->back();
                prevVec.normalize();

                for( osg::Vec3dArray::iterator j = part->begin(); part->size() >= 3 && j != part->end(); )
                {
                    osg::Vec3d& p0 = *j;
                    osg::Vec3d& p1 = j+1 != part->end() ? *(j+1) : part->front();
                    osg::Vec3d vec = p1-p0; vec.normalize();

                    // if the vectors are essentially parallel, remove the extraneous vertex.
                    if ( (prevVec ^ vec).length() < PARALLEL_EPSILON )
                    {
                        j = part->erase( j );
                        //OE_NOTICE << "removed colinear segment" << std::endl;
                    }
                    else
                    {
                        ++j;
                        prevVec = vec;
                    }
                }
            }
        }


        bool made_geom = true;
#if 0
        // no context yet
        const SpatialReference* srs = context.profile()->getSRS();
#else
        const SpatialReference* srs = 0;
#endif

        // total up all the points so we can pre-allocate the vertex arrays.
        int num_cap_verts = geom->getTotalPointCount();
        int num_wall_verts = 2 * (num_cap_verts + numRings); // add in numRings b/c we need to close each wall

        osg::Geometry* walls = new osg::Geometry();
        osg::Vec3Array* verts = new osg::Vec3Array( num_wall_verts );
        walls->setVertexArray( verts );

        osg::Geometry* top_cap = new osg::Geometry();
        osg::Vec3Array* top_verts = new osg::Vec3Array( num_cap_verts );
        top_cap->setVertexArray( top_verts );

        osg::Geometry* bottom_cap = new osg::Geometry();
        osg::Vec3Array* bottom_verts = new osg::Vec3Array( num_cap_verts );
        bottom_cap->setVertexArray( bottom_verts );

        int wall_vert_ptr = 0;
        int top_vert_ptr = 0;
        int bottom_vert_ptr = 0;

        //double target_len = height;

        // now generate the extruded geometry.
        GeometryIterator k( geom );
        while( k.hasMore() )
        {
            Geometry* part = k.next();

            unsigned int wall_part_ptr = wall_vert_ptr;
            unsigned int top_part_ptr = top_vert_ptr;
            unsigned int bottom_part_ptr = bottom_vert_ptr;
            double part_len = 0.0;

            GLenum prim_type = part->getType() == Geometry::TYPE_POINTSET ? GL_LINES : GL_TRIANGLE_STRIP;

            for( osg::Vec3dArray::const_iterator m = part->begin(); m != part->end(); ++m )
            {
                osg::Vec3d extrude_vec;

#if 0
                // no context yet
                if ( srs )
                {
                    osg::Vec3d m_world = context.toWorld( *m ); //*m * context.inverseReferenceFrame();
                    if ( context.isGeocentric() )
                    {
                        osg::Vec3d p_vec = m_world; // todo: not exactly right; spherical

                        osg::Vec3d unit_vec = p_vec; 
                        unit_vec.normalize();
                        p_vec = p_vec + unit_vec*height;

                        extrude_vec = context.toLocal( p_vec ); //p_vec * context.referenceFrame();
                    }
                    else
                    {
                        extrude_vec.set( m_world.x(), m_world.y(), height );
                        extrude_vec = context.toLocal( extrude_vec ); //extrude_vec * context.referenceFrame();
                    }
                }
                else
#endif
                {
                    extrude_vec.set( m->x(), m->y(), height );
                }

                (*top_verts)[top_vert_ptr++] = extrude_vec;
                (*bottom_verts)[bottom_vert_ptr++] = *m;
             
                part_len += wall_vert_ptr > wall_part_ptr?
                    (extrude_vec - (*verts)[wall_vert_ptr-2]).length() :
                    0.0;

                int p;

                p = wall_vert_ptr++;
                (*verts)[p] = extrude_vec;

                p = wall_vert_ptr++;
                (*verts)[p] = *m;
            }

            // close the wall if it's a ring/poly:
            if ( part->getType() == Geometry::TYPE_RING || part->getType() == Geometry::TYPE_POLYGON )
            {
                part_len += wall_vert_ptr > wall_part_ptr?
                    ((*verts)[wall_part_ptr] - (*verts)[wall_vert_ptr-2]).length() :
                    0.0;

                int p;

                p = wall_vert_ptr++;
                (*verts)[p] = (*verts)[wall_part_ptr];

                p = wall_vert_ptr++;
                (*verts)[p] = (*verts)[wall_part_ptr+1];
            }

            walls->addPrimitiveSet( new osg::DrawArrays(
                                        prim_type,
                                        wall_part_ptr, wall_vert_ptr - wall_part_ptr ) );

            top_cap->addPrimitiveSet( new osg::DrawArrays(
                                          osg::PrimitiveSet::LINE_LOOP,
                                          top_part_ptr, top_vert_ptr - top_part_ptr ) );

            // reverse the bottom verts so the front face is down:
            std::reverse( bottom_verts->begin()+bottom_part_ptr, bottom_verts->begin()+bottom_vert_ptr );

            bottom_cap->addPrimitiveSet( new osg::DrawArrays(
                                             osg::PrimitiveSet::LINE_LOOP,
                                             bottom_part_ptr, bottom_vert_ptr - bottom_part_ptr ) );
        }

        // build solid surfaces for the caps:
        tessellate( top_cap );
        tessellate( bottom_cap );

        osg::Geode* geode = new osg::Geode();
        geode->addDrawable( walls );
        geode->addDrawable( top_cap );
        geode->addDrawable( bottom_cap );

        return geode;
    }

    bool update(FeatureDataSet* dataSet,
                const osgEarth::Symbology::Style* style,
                osg::Group* attachPoint,
                SymbolizerContext* context )
    {
        if (!dataSet || !attachPoint || !style)
            return false;

        osg::ref_ptr<osgEarth::Features::FeatureCursor> cursor = dataSet->createCursor();
        if (!cursor)
            return false;

        osg::ref_ptr<osg::Group> newSymbolized = new osg::Group;
        osg::ref_ptr<osg::Geode> geode = new osg::Geode;
        newSymbolized->addChild(geode.get());

        osgEarth::Features::Feature* feature = 0;
        while( cursor->hasMore() )
        {
            feature = cursor->nextFeature();
            if (!feature)
                continue;

            Geometry* geometry = feature->getGeometry();
            if (geometry) 
            {
                geode = createVolume(geometry, _offset, _height, context);
                if (geode)
                    newSymbolized->addChild(geode);
            }
        }

        if (newSymbolized->getNumChildren())
        {
            attachPoint->addChild(newSymbolized.get());
            return true;
        }

        return false;
    }


protected:
    double _offset;
    double _height;
};











osg::Group* createSymbologyScene(const std::string url)
{
    osg::Group* grp = new osg::Group;

    osg::ref_ptr<osgEarth::Drivers::OGRFeatureOptions> featureOpt = new osgEarth::Drivers::OGRFeatureOptions();
    featureOpt->url() = url;
    osg::ref_ptr<osgEarth::Features::FeatureSource> features = FeatureSourceFactory::create( featureOpt );
    features->initialize("");

    osg::ref_ptr<FeatureDataSetAdapter> dataset = new FeatureDataSetAdapter(features.get());

    {
        osg::ref_ptr<osgEarth::Symbology::Style> style = new osgEarth::Symbology::Style;

        osg::ref_ptr<PolygonSymbol> polySymbol = new PolygonSymbol;
        polySymbol->fill()->color() = osg::Vec4(0,1,1,1);
        style->setPolygon(polySymbol.get());

        osg::ref_ptr<GeometrySymbolizer> symbolizer = new GeometrySymbolizer;
        osg::ref_ptr<SymbolicNode> node = new SymbolicNode;
        node->setSymbolizer(symbolizer.get());
        node->setStyle(style.get());
        node->setDataSet(dataset.get());
        osg::MatrixTransform* tr = new osg::MatrixTransform;
        tr->setMatrix(osg::Matrix::translate(0, 0 , 0));
        tr->addChild(node.get());
        grp->addChild(tr);
    }


    {
        // use the same feature but with a custom Polygon as Point with a specific Symbolizer
        // not very useful, but demonstrate how to customize
        osg::ref_ptr<osgEarth::Symbology::Style> style = new osgEarth::Symbology::Style;

        osg::ref_ptr<PolygonPointSizeSymbol> polySymbol = new PolygonPointSizeSymbol;
        polySymbol->fill()->color() = osg::Vec4(1,0,0,1);
        polySymbol->size() = 2.0;
        style->setPolygon(polySymbol.get());

        osg::ref_ptr<GeometrySymbolizer> symbolizer = new GeometryPointSymbolizer;
        osg::ref_ptr<SymbolicNode> node = new SymbolicNode;
        node->setSymbolizer(symbolizer.get());
        node->setStyle(style.get());
        node->setDataSet(dataset.get());
        osg::MatrixTransform* tr = new osg::MatrixTransform;
        tr->addChild(node.get());
        tr->setMatrix(osg::Matrix::translate(0, 0 , 1));
        grp->addChild(tr);
    }


    {
        // use the same feature but with a custom Polygon as Point with a specific Symbolizer
        // not very useful, but demonstrate how to customize
        osg::ref_ptr<osgEarth::Symbology::Style> style = new osgEarth::Symbology::Style;

        osg::ref_ptr<GeometryVolumeSymbolizer> symbolizer = new GeometryVolumeSymbolizer(1, 10);
        osg::ref_ptr<SymbolicNode> node = new SymbolicNode;
        node->setSymbolizer(symbolizer.get());
        node->setStyle(style.get());
        node->setDataSet(dataset.get());
        osg::MatrixTransform* tr = new osg::MatrixTransform;
        tr->addChild(node.get());
        tr->setMatrix(osg::Matrix::translate(0, -200 , 0));
        grp->addChild(tr);
    }


    return grp;
}


int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    osgViewer::Viewer viewer(arguments);

    // add some stock OSG handlers:
    viewer.setCameraManipulator(new osgGA::TrackballManipulator());
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    const std::string url = "../data/world.shp";
    std::string real = osgDB::getRealPath(url);
    osg::Node* node = createSymbologyScene(real);
    viewer.setSceneData(node);

    return viewer.run();
}
