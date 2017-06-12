/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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
#include <osgEarthAnnotation/GARSGraticule>
#include <osgEarthAnnotation/FeatureNode>
#include <osgDB/Registry>
#include <osgEarth/Utils>

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Symbology;

class PagedNode : public osg::Group
{
public:
    PagedNode();

    virtual osg::Node* loadChildren();

    virtual void build();

    virtual osg::BoundingSphere getKeyBound() const;

    virtual bool hasChildren() const;

    void setupPaging();

    osg::Group* _attachPoint;
    osg::PagedLOD* _plod;
    std::string _key;
};

namespace
{    
    /**
    * A pseudo-loader for paged feature tiles.
    */
    struct PagedNodePseudoLoader : public osgDB::ReaderWriter
    {
        PagedNodePseudoLoader()
        {
            supportsExtension( "osgearth_pseudo_pagednode", "" );
        }

        const char* className() const
        { // override
            return "PagedNodePseudoLoader";
        }

        ReadResult readNode(const std::string& uri, const Options* options) const
        {
            if ( !acceptsExtension( osgDB::getLowerCaseFileExtension(uri) ) )
                return ReadResult::FILE_NOT_HANDLED;

            osg::ref_ptr<PagedNode> node;
            if (!OptionsData<PagedNode>::lock(options, "osgEarth.PagedNode", node))
            {
                OE_WARN << "Internal error - no PagedNode object in OptionsData\n";
                return ReadResult::ERROR_IN_READING_FILE;
            }

            return node->loadChildren();
        }
    };

    REGISTER_OSGPLUGIN(osgearth_pseudo_pagednode, PagedNodePseudoLoader);
}

PagedNode::PagedNode()
{
    _plod = new osg::PagedLOD;
    addChild(_plod);

    _attachPoint = new osg::Group;

    _plod->addChild( _attachPoint );     
}

void PagedNode::build()
{
    // This is your chance to add something to the attachpoint.
}

void PagedNode::setupPaging()
{
    osg::BoundingSphere bs = getKeyBound();

    _plod->setCenter( bs.center() ); 
    _plod->setRadius( bs.radius() );

    if ( hasChildren() )
    {    

        // Now setup a filename on the PagedLOD that will load all of the children of this node.
        _plod->setFileName(1, "dummy.osgearth_pseudo_pagednode");
      
        // assemble data to pass to the pseudoloader
        osgDB::Options* options = new osgDB::Options();
        OptionsData<PagedNode>::set(options, "osgEarth.PagedNode", this);
        _plod->setDatabaseOptions( options );

        double rangeFactor = 6.0;

        // Setup the min and max ranges.
        float minRange = (float)(bs.radius() * rangeFactor);
        bool additive = false;

        if (!additive)
        {
            // Replace mode, the parent is replaced by its children.
            _plod->setRange( 0, minRange, FLT_MAX );
            _plod->setRange( 1, 0, minRange );
        }
        else
        {
            // Additive, the parent remains and new data is added
            _plod->setRange( 0, 0, FLT_MAX );
            _plod->setRange( 1, 0, minRange );
        }
    }
    else
    {
        // no children, so max out the visibility range.
        _plod->setRange( 0, 0, FLT_MAX );
    }   
}

osg::BoundingSphere PagedNode::getKeyBound() const
{
    return osg::BoundingSphere();
}

bool PagedNode::hasChildren() const
{
    return true;
}

osg::Node* PagedNode::loadChildren()
{
    return 0;
}

enum GARSLevel
{
    GARS_30,
    GARS_15,
    GARS_5
};


class GridNode : public PagedNode
{
public:
    GridNode(MapNode* mapNode, const GeoExtent& extent, GARSLevel level);

    virtual osg::Node* loadChildren();

    virtual void build();

    virtual osg::BoundingSphere getKeyBound() const;

    virtual bool hasChildren() const;

    GeoExtent _extent;
    osg::observer_ptr<MapNode> _mapNode;
    GARSLevel _level;
};

GridNode::GridNode(MapNode* mapNode, const GeoExtent& extent, GARSLevel level):
_mapNode(mapNode),
_extent(extent),
_level(level),
PagedNode()
{
    build();
    setupPaging();
}

osg::Node* GridNode::loadChildren()
{
    GARSLevel childLevel;
    int dim = 2;
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

            group->addChild(new GridNode(_mapNode.get(), GeoExtent(_extent.getSRS(), west, south, east, north), childLevel));

        }
    }
    return group;
    /*
    OE_NOTICE << "Load children" << std::endl;
    osg::Group* group = new osg::Group;
    double centerX = _extent.west() + _extent.width() / 2.0;
    double centerY = _extent.south() + _extent.height() / 2.0;

    group->addChild(new GridNode(_mapNode.get(), GeoExtent(_extent.getSRS(), _extent.west(), _extent.south(), centerX, centerY)));
    group->addChild(new GridNode(_mapNode.get(), GeoExtent(_extent.getSRS(), centerX, _extent.south(), _extent.east(), centerY)));
    group->addChild(new GridNode(_mapNode.get(), GeoExtent(_extent.getSRS(), _extent.west(), centerY, centerX, _extent.north())));
    group->addChild(new GridNode(_mapNode.get(), GeoExtent(_extent.getSRS(), centerX, centerY, _extent.east(), _extent.north())));
    return group;
    */

    

    /*
    OE_NOTICE << "Load children" << std::endl;
    osg::Group* group = new osg::Group;
    double centerX = _extent.west() + _extent.width() / 2.0;
    double centerY = _extent.south() + _extent.height() / 2.0;

    group->addChild(new GridNode(_mapNode.get(), GeoExtent(_extent.getSRS(), _extent.west(), _extent.south(), centerX, centerY)));
    group->addChild(new GridNode(_mapNode.get(), GeoExtent(_extent.getSRS(), centerX, _extent.south(), _extent.east(), centerY)));
    group->addChild(new GridNode(_mapNode.get(), GeoExtent(_extent.getSRS(), _extent.west(), centerY, centerX, _extent.north())));
    group->addChild(new GridNode(_mapNode.get(), GeoExtent(_extent.getSRS(), centerX, centerY, _extent.east(), _extent.north())));
    return group;
    */
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

    Style style;
    style.getOrCreateSymbol<LineSymbol>()->stroke()->color() = Color::Blue;
    style.getOrCreateSymbol<LineSymbol>()->tessellation() = 20;
    style.getOrCreateSymbol<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;
    style.getOrCreateSymbol<AltitudeSymbol>()->technique() = AltitudeSymbol::TECHNIQUE_DRAPE;

    if (_level == GARS_30)
    {
        style.getOrCreateSymbol<TextSymbol>()->content()->setLiteral("30");
    }
    else if (_level == GARS_15)
    {
        style.getOrCreateSymbol<TextSymbol>()->content()->setLiteral("15");
    }
    else if (_level == GARS_5)
    {    
        style.getOrCreateSymbol<TextSymbol>()->content()->setLiteral("5");
    }

    style.getOrCreateSymbol<TextSymbol>()->size() = 40.0;
    style.getOrCreateSymbol<TextSymbol>()->alignment() = TextSymbol::ALIGN_CENTER_CENTER;

    
    FeatureNode* featureNode = new FeatureNode(_mapNode.get(), features, style);
    // Add the node to the attachpoint.
    _attachPoint->addChild(featureNode);
}



osg::BoundingSphere GridNode::getKeyBound() const
{
    int samples = 6;    

    double xSample = _extent.width() / (double)samples;
    double ySample = _extent.height() / (double)samples;

    osg::BoundingSphere bs;
    for (int c = 0; c < samples+1; c++)
    {
        double x = _extent.xMin() + (double)c * xSample;
        for (int r = 0; r < samples+1; r++)
        {
            double y = _extent.yMin() + (double)r * ySample;
            osg::Vec3d world;

            GeoPoint samplePoint(_extent.getSRS(), x, y, 0, ALTMODE_ABSOLUTE);

            GeoPoint wgs84 = samplePoint.transform(osgEarth::SpatialReference::create("epsg:4326"));
            wgs84.toWorld(world);
            bs.expandBy(world);
        }
    }
    return bs;
}

bool GridNode::hasChildren() const
{
    return _level != GARS_5;
}


/*******/
class IndexNode : public PagedNode
{
public:
    IndexNode(MapNode* mapNode, const GeoExtent& extent);

    virtual osg::Node* loadChildren();

    virtual void build();

    virtual osg::BoundingSphere getKeyBound() const;

    virtual bool hasChildren() const;

    GeoExtent _extent;
    osg::observer_ptr<MapNode> _mapNode;
};

IndexNode::IndexNode(MapNode* mapNode, const GeoExtent& extent):
_mapNode(mapNode),
_extent(extent),
PagedNode()
{
    build();
    setupPaging();
}

osg::Node* IndexNode::loadChildren()
{
    // Load the 30 minute cells.
    OE_NOTICE << "Load children" << std::endl;
    osg::Group* group = new osg::Group;

    int numCols = ceil(_extent.width() / 0.5);
    int numRows = ceil(_extent.height() / 0.5);

    for (int c = 0; c < numCols; c++)
    {    
        for (int r = 0; r < numRows; r++)
        {
            Feature* feature = new Feature(new LineString(5), SpatialReference::create("wgs84"));
            double west = _extent.xMin() + 0.5 * (double)c;
            double south = _extent.yMin() + 0.5 * r;
            group->addChild(new GridNode(_mapNode.get(), GeoExtent(_extent.getSRS(), west, south, west + 0.5, south + 0.5), GARS_30));         
        }
    }        
    return group;
}

void IndexNode::build()
{   
    /*
    OE_NOTICE << "Building grid node" << std::endl;
    Feature* feature = new Feature(new LineString(5), SpatialReference::create("wgs84"));
    feature->getGeometry()->push_back(_extent.west(), _extent.south(), 0.0);
    feature->getGeometry()->push_back(_extent.east(), _extent.south(), 0.0);
    feature->getGeometry()->push_back(_extent.east(), _extent.north(), 0.0);
    feature->getGeometry()->push_back(_extent.west(), _extent.north(), 0.0);
    feature->getGeometry()->push_back(_extent.west(), _extent.south(), 0.0);    
    FeatureList features;
    features.push_back(feature);

    Style style;
    style.getOrCreateSymbol<LineSymbol>()->stroke()->color() = Color::Blue;
    style.getOrCreateSymbol<LineSymbol>()->tessellation() = 50;
    style.getOrCreateSymbol<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;
    style.getOrCreateSymbol<AltitudeSymbol>()->technique() = AltitudeSymbol::TECHNIQUE_DRAPE;

    
    FeatureNode* featureNode = new FeatureNode(_mapNode.get(), features, style);
    // Add the node to the attachpoint.
    _attachPoint->addChild(featureNode);
    */
}

osg::BoundingSphere IndexNode::getKeyBound() const
{
    int samples = 6;    

    double xSample = _extent.width() / (double)samples;
    double ySample = _extent.height() / (double)samples;

    osg::BoundingSphere bs;
    for (int c = 0; c < samples+1; c++)
    {
        double x = _extent.xMin() + (double)c * xSample;
        for (int r = 0; r < samples+1; r++)
        {
            double y = _extent.yMin() + (double)r * ySample;
            osg::Vec3d world;

            GeoPoint samplePoint(_extent.getSRS(), x, y, 0, ALTMODE_ABSOLUTE);

            GeoPoint wgs84 = samplePoint.transform(osgEarth::SpatialReference::create("epsg:4326"));
            wgs84.toWorld(world);
            bs.expandBy(world);
        }
    }
    return bs;
}

bool IndexNode::hasChildren() const
{
    return true;
}

/*******/




GARSGraticule::GARSGraticule(MapNode* mapNode):
_mapNode(mapNode)
{
    build30MinCells();
}

void GARSGraticule::build30MinCells()
{
    /*
    int numCols = 720;
    int numRows = 360;

    FeatureList features;

    for (int c = 0; c < numCols-1; c++)
    {    
        for (int r = 0; r < numRows-1; r++)
        {
            Feature* feature = new Feature(new LineString(5), SpatialReference::create("wgs84"));
            double west = -180.0 + 0.5 * (double)c;
            double south = -90.0 + 0.5 * r;
            feature->getGeometry()->push_back(west, south, 0.0);
            feature->getGeometry()->push_back(west + 0.5, south, 0.0);
            feature->getGeometry()->push_back(west + 0.5, south + 0.5, 0.0);
            feature->getGeometry()->push_back(west, south + 0.5, 0.0);
            feature->getGeometry()->push_back(west, south, 0.0);
            features.push_back(feature);
        }
    }

    Style style;
    style.getOrCreateSymbol<LineSymbol>()->stroke()->color() = Color::Blue;
    style.getOrCreateSymbol<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;
    style.getOrCreateSymbol<AltitudeSymbol>()->technique() = AltitudeSymbol::TECHNIQUE_DRAPE;
    FeatureNode* featureNode = new FeatureNode(_mapNode.get(), features, style);
    addChild(featureNode);
    */

    /*
    addChild(new GridNode(_mapNode.get(), GeoExtent(SpatialReference::create("wgs84"), -180.0, -90.0, 0.0, 90.0)));
    addChild(new GridNode(_mapNode.get(), GeoExtent(SpatialReference::create("wgs84"), 0.0, -90.0, 180.0, 90.0)));
    */

    double size = 5.0;
    int numCols = ceil(360.0 / size);
    int numRows = ceil(180.0 / size);

    for (unsigned int c = 0; c < numCols; c++)
    {
        for (unsigned int r = 0; r < numRows; r++)
        {
            double west = -180.0 + (double)c * size;
            double south = -90.0 + (double)r * size;
            addChild(new IndexNode(_mapNode.get(), GeoExtent(SpatialReference::create("wgs84"), west, south, west + size, south + size)));
        }
    }
}




