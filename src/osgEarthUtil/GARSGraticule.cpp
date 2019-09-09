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
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */
#include <osgEarthUtil/GARSGraticule>
#include <osgEarthAnnotation/FeatureNode>
#include <osgEarthFeatures/TextSymbolizer>
#include <osgEarth/PagedNode>
#include <osgEarth/GLUtils>
#include <osgEarth/Text>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Annotation;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

namespace
{
    osg::BoundingSphere getBounds(const GeoExtent& extent)
    {
        int samples = 6;

        double xSample = extent.width() / (double)samples;
        double ySample = extent.height() / (double)samples;

        osg::BoundingSphere bs;
        for (int c = 0; c < samples+1; c++)
        {
            double x = extent.xMin() + (double)c * xSample;
            for (int r = 0; r < samples+1; r++)
            {
                double y = extent.yMin() + (double)r * ySample;
                osg::Vec3d world;

                GeoPoint samplePoint(extent.getSRS(), x, y, 0, ALTMODE_ABSOLUTE);

                GeoPoint wgs84 = samplePoint.transform(osgEarth::SpatialReference::create("epsg:4326"));
                wgs84.toWorld(world);
                bs.expandBy(world);
            }
        }
        return bs;
    }

    enum GARSLevel
    {
        GARS_30,
        GARS_15,
        GARS_5
    };

    std::string getGARSLabel(double lon, double lat, GARSLevel level)
    {
        int lonCell = floor((lon - -180.0) / 0.5);
        int latCell = floor((lat - -90.0) / 0.5);
     
        // Format the lon cell
        std::stringstream buf;
        if (lonCell < 9)
        {
            buf << "00";
        }
        else if (lonCell < 99)
        {
            buf << "0";
        }
        buf << (lonCell + 1);

        // Format the lat cell
        std::string latIndices = "ABCDEFGHJKLMNPQRSTUVWXYZ";
        int latPrimaryIndex = latCell / latIndices.size();
        int latSecondaryIndex = latCell - (latPrimaryIndex * latIndices.size());
        buf << latIndices[latPrimaryIndex] << latIndices[latSecondaryIndex];

        if (level == GARS_15 || level == GARS_5)
        {
            // Figure out the quadrant of the 15 minute cell within the parent 30 minute cell.
            int x15Cell = floor(fmod(lon - -180.0, 0.5) / 0.25);
            int y15Cell = floor(fmod(lat - -90.0, 0.5) / 0.25);
            int y15CellInverted = 2 - y15Cell - 1;
            buf << x15Cell + y15CellInverted * 2 + 1;

            if (level == GARS_5)
            {
                int x5Cell = floor((lon - -180.0 - (lonCell * 0.5 + x15Cell * 0.25)) / 0.08333333333);
                int y5Cell = floor((lat - -90.0 - (latCell * 0.5 + y15Cell * 0.25)) / 0.08333333333);
                int y5CellInverted = 3 - y5Cell - 1;
                buf << x5Cell + y5CellInverted * 3 + 1;

            }
        }
        return buf.str();
    }

    class GridNode : public PagedNode
    {
    public:
        GridNode(GARSGraticule* graticule, const GeoExtent& extent, GARSLevel level);

        virtual osg::Node* loadChild();

        virtual void build();

        virtual osg::BoundingSphere getChildBound() const;

        virtual bool hasChild() const;

        GeoExtent _extent;
        GARSGraticule* _graticule;
        GARSLevel _level;
    };

    GridNode::GridNode(GARSGraticule* graticule, const GeoExtent& extent, GARSLevel level):
    _graticule(graticule),
    _extent(extent),
    _level(level),
    PagedNode()
    {
        build();
        setupPaging();
    }

    osg::Node* GridNode::loadChild()
    {
        GARSLevel childLevel;
        unsigned dim = 2;
        if (_level == GARS_30)
        {
            childLevel = GARS_15;
            dim = 2;
        }
        else if (_level == GARS_15)
        {
            childLevel = GARS_5;
            dim = 3;
        }

        double width = _extent.width() / (double)dim;
        double height = _extent.height() / (double)dim;

        osg::Group* group = new osg::Group;
        for (unsigned int c = 0; c < dim; c++)
        {
            for (unsigned int r = 0; r < dim; r++)
            {
                double west = _extent.west() + (double)c * width;
                double south= _extent.south() + (double)r * height;
                double east = west + width;
                double north = south + height;

                group->addChild(new GridNode(_graticule, GeoExtent(_extent.getSRS(), west, south, east, north), childLevel));

            }
        }
        return group;    
    }

    void GridNode::build()
    { 
        Feature* feature = new Feature(new LineString(5), SpatialReference::create("wgs84"));
        feature->getGeometry()->push_back(_extent.west(), _extent.south(), 0.0);
        feature->getGeometry()->push_back(_extent.east(), _extent.south(), 0.0);
        feature->getGeometry()->push_back(_extent.east(), _extent.north(), 0.0);
        feature->getGeometry()->push_back(_extent.west(), _extent.north(), 0.0);
        feature->getGeometry()->push_back(_extent.west(), _extent.south(), 0.0);    
        FeatureList features;
        features.push_back(feature);

        Style style = _graticule->options().style().get();

        double lon, lat;
        _extent.getCentroid(lon, lat);
        std::string label = getGARSLabel(lon, lat, _level);
    
        FeatureNode* featureNode = new FeatureNode(features, style);
        // Add the node to the attachpoint.
        _attachPoint->addChild(featureNode);
       
        GeoPoint centroid(_extent.getSRS(), lon, lat, 0.0f);
        GeoPoint ll(_extent.getSRS(), _extent.west(), _extent.south(), 0.0f);

        const TextSymbol* textSymPrototype = style.get<TextSymbol>();
        osg::ref_ptr<TextSymbol> textSym = textSymPrototype ? new TextSymbol(*textSymPrototype) : new TextSymbol();

        if (textSym->size().isSet() == false)
            textSym->size() = 32.0f;

        if (textSym->alignment().isSet() == false)
            textSym->alignment() = textSym->ALIGN_LEFT_BASE_LINE;

        TextSymbolizer symbolizer(textSym.get());                

        osgText::Text* text = new osgEarth::Text(label);
        symbolizer.apply(text);
        text->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
        //text->getOrCreateStateSet()->setRenderBinToInherit();

        //osg::Geode* textGeode = new osg::Geode;
        //textGeode->addDrawable(text);

        osg::MatrixTransform* mt = new osg::MatrixTransform;
        mt->addChild(text);

        // Position the label at the bottom left of the grid cell.
        osg::Matrixd local2World;
        ll.createLocalToWorld(local2World);
        mt->setMatrix(local2World);

       _attachPoint->addChild(mt);

       //Registry::shaderGenerator().run(this, Registry::stateSetCache());
    }

    osg::BoundingSphere GridNode::getChildBound() const
    {
        return getBounds(_extent);
    }

    bool GridNode::hasChild() const
    {
        return _level != GARS_5;
    }


    /*******/
    class IndexNode : public PagedNode
    {
    public:
        IndexNode(GARSGraticule* graticule, const GeoExtent& extent);

        virtual osg::Node* loadChild();

        virtual osg::BoundingSphere getChildBound() const;

        virtual bool hasChild() const;

        GeoExtent _extent;
        GARSGraticule* _graticule;
    };

    IndexNode::IndexNode(GARSGraticule* graticule, const GeoExtent& extent):
    _graticule(graticule),
    _extent(extent),
    PagedNode()
    {
        setupPaging();
    }

    osg::Node* IndexNode::loadChild()
    {
        // Load the 30 minute cells.
        osg::Group* group = new osg::Group;

        int numCols = ceil(_extent.width() / 0.5);
        int numRows = ceil(_extent.height() / 0.5);

        for (int c = 0; c < numCols; c++)
        {    
            for (int r = 0; r < numRows; r++)
            {
                double west = _extent.xMin() + 0.5 * (double)c;
                double south = _extent.yMin() + 0.5 * r;
                group->addChild(new GridNode(_graticule, GeoExtent(_extent.getSRS(), west, south, west + 0.5, south + 0.5), GARS_30));         
            }
        }        
        return group;
    }

    osg::BoundingSphere IndexNode::getChildBound() const
    {
        return getBounds(_extent);
    }

    bool IndexNode::hasChild() const
    {
        return true;
    }
}

/*******/

REGISTER_OSGEARTH_LAYER(gars_graticule, GARSGraticule);


GARSGraticule::GARSGraticule() :
VisibleLayer(&_optionsConcrete),
_options(&_optionsConcrete)
{
    init();
}

GARSGraticule::GARSGraticule(const GARSGraticuleOptions& options) :
VisibleLayer(&_optionsConcrete),
_options(&_optionsConcrete),
_optionsConcrete(options)
{
    init();
}

void
GARSGraticule::dirty()
{
    rebuild();
}

void
GARSGraticule::init()
{
    VisibleLayer::init();

    osg::StateSet* ss = this->getOrCreateStateSet();
    ss->setMode( GL_DEPTH_TEST, 0 );
    GLUtils::setLighting(ss, 0);
    ss->setMode( GL_BLEND, 1 );


    // force it to render after the terrain.
    ss->setRenderBinDetails(1, "RenderBin");
    
    if (options().style().isSet() == false)
    {
        options().style()->getOrCreateSymbol<LineSymbol>()->stroke()->color() = Color::Blue;
        options().style()->getOrCreateSymbol<LineSymbol>()->tessellation() = 10;
    }

    // Always use draping.
    // Note: since we use draping we do NOT need to activate a horizon clip plane!
    options().style()->getOrCreateSymbol<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;
    options().style()->getOrCreateSymbol<AltitudeSymbol>()->technique() = AltitudeSymbol::TECHNIQUE_DRAPE;

    _root = new osg::Group();
    //_root->getOrCreateStateSet()->setAttribute(new osg::Program(), osg::StateAttribute::OFF);

    if (getEnabled() == true)
    {
        rebuild();
    }
}

void
GARSGraticule::addedToMap(const Map* map)
{
    rebuild();
}

void
GARSGraticule::removedFromMap(const Map* map)
{
    //nop
}

osg::Node*
GARSGraticule::getNode() const
{
    return _root.get();
}

void
GARSGraticule::rebuild()
{
    if (_root.valid() == false)
        return;

    _root->removeChildren(0, _root->getNumChildren());
    build30MinCells();
}


void GARSGraticule::build30MinCells()
{
    double size = 3.0;
    unsigned numCols = ceil(360.0 / size);
    unsigned numRows = ceil(180.0 / size);

    for (unsigned c = 0; c < numCols; c++)
    {
        for (unsigned r = 0; r < numRows; r++)
        {
            double west = -180.0 + (double)c * size;
            double south = -90.0 + (double)r * size;
            _root->addChild(new IndexNode(this, GeoExtent(SpatialReference::create("wgs84"), west, south, west + size, south + size)));
        }
    }
}
