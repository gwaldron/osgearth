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
