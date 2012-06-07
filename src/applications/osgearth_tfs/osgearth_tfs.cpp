/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
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
#include <osgEarth/Registry>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgEarth/XmlUtils>

#include <osgEarthDrivers/feature_ogr/OGRFeatureOptions>

using namespace osgEarth::Features;
using namespace osgEarth::Drivers;
using namespace osgEarth::Symbology;

#include <osgEarthFeatures/GeometryUtils>


typedef std::pair< FeatureID, double> FeatureValue;

typedef std::list< FeatureValue > FeatureValueList;



struct TFSLayer
{
    std::string _title;
    std::string _abstract;
    osgEarth::GeoExtent _extent;
    unsigned int _maxLevel;
    unsigned int _firstLevel;
};

class TFSReaderWriter
{
public:
    static bool read(const URI& uri, const osgDB::ReaderWriter::Options* options, TFSLayer &layer);
    static bool read( std::istream &in, TFSLayer &layer);

    static void write(const TFSLayer &layer, const std::string& location);
    static void write(const TFSLayer &layer, std::ostream& output);

};

bool
TFSReaderWriter::read(const URI& uri, const osgDB::ReaderWriter::Options *options, TFSLayer &layer)
{
    osgEarth::ReadResult result = uri.readString();
    if (result.succeeded())
    {
        std::string str = result.getString();
        std::stringstream in( str.c_str()  );
        return read( in, layer);
    }    
    return false;
}

bool
TFSReaderWriter::read( std::istream &in, TFSLayer &layer)
{
    osg::ref_ptr< XmlDocument > doc = XmlDocument::load( in );
    if (!doc.valid()) return false;

    osg::ref_ptr<XmlElement> e_layer = doc->getSubElement( "layer" );
    if (!e_layer.valid()) return false;

    layer._title      = e_layer->getSubElementText("title");
    layer._abstract   = e_layer->getSubElementText("abstract");    
    layer._firstLevel   = as<unsigned int>(e_layer->getSubElementText("firstlevel"), 0);
    layer._maxLevel = as<unsigned int>(e_layer->getSubElementText("maxlevel"), 0);


     //Read the bounding box
    osg::ref_ptr<XmlElement> e_bounding_box = e_layer->getSubElement("boundingbox");
    if (e_bounding_box.valid())
    {
        double minX = as<double>(e_bounding_box->getAttr( "minx" ), 0.0);
        double minY = as<double>(e_bounding_box->getAttr( "miny" ), 0.0);
        double maxX = as<double>(e_bounding_box->getAttr( "maxx" ), 0.0);
        double maxY = as<double>(e_bounding_box->getAttr( "maxy" ), 0.0);
        layer._extent = GeoExtent(SpatialReference::create( "epsg:4326" ), minX, minY, maxX, maxY);
    }


    return true;
}


static XmlDocument*
tfsToXmlDocument(const TFSLayer &layer)
{
    //Create the root XML document
    osg::ref_ptr<XmlDocument> doc = new XmlDocument();
    doc->setName( "Layer" );
        
    doc->addSubElement( "Title", layer._title );
    doc->addSubElement( "Abstract", layer._abstract );
    doc->addSubElement( "MaxLevel", toString<unsigned int>(layer._maxLevel ));
    doc->addSubElement( "FirstLevel", toString<unsigned int>(layer._firstLevel ));
    
    osg::ref_ptr<XmlElement> e_bounding_box = new XmlElement( "BoundingBox" );
    e_bounding_box->getAttrs()["minx"] = toString(layer._extent.xMin());
    e_bounding_box->getAttrs()["miny"] = toString(layer._extent.yMin());
    e_bounding_box->getAttrs()["maxx"] = toString(layer._extent.xMax());
    e_bounding_box->getAttrs()["maxy"] = toString(layer._extent.yMax());
    doc->getChildren().push_back(e_bounding_box.get() );
    
    return doc.release();
}

void
TFSReaderWriter::write(const TFSLayer &layer, const std::string &location)
{
    std::string path = osgDB::getFilePath(location);
    if (!osgDB::fileExists(path) && !osgDB::makeDirectory(path))
    {
        OE_WARN << "Couldn't create path " << std::endl;
    }
    std::ofstream out(location.c_str());
    write(layer, out);
}

void
TFSReaderWriter::write(const TFSLayer &layer, std::ostream &output)
{
    osg::ref_ptr<XmlDocument> doc = tfsToXmlDocument(layer);    
    doc->store(output);
}












bool compare_featurevalue (FeatureValue &a, FeatureValue& b)
{
    if (a.second < b.second) return true;
    if (a.second > b.second) return false;
    if (a.first < b.first) return true;  
    return false;
}

class Index
{
public:
    void insert(FeatureID id, double value)
    {
        values.push_back( FeatureValue( id, value ) );
    }   

    void finalize()
    {
        values.sort(compare_featurevalue);
    }

    FeatureValueList values;    
};

class FeatureTileVisitor;
class FeatureTile;

class FeatureTile : public osg::Referenced
{
public:
    FeatureTile( TileKey& key ):
      _key( key ),
      _isSplit( false )
    {        
    }

    const GeoExtent& getExtent() const
    {
        return _key.getExtent();
    }

    const TileKey& getKey() const
    {
        return _key;
    }

    bool getIsSplit() const { return _isSplit;}

    bool containsCentroid(Feature* feature)
    {        
        Geometry* featureGeom = feature->getGeometry();
        if ( featureGeom && featureGeom->isValid() )
        {
            Bounds bounds = featureGeom->getBounds();
            if ( bounds.isValid() )
            {
                osg::Vec3d centroid = bounds.center();
                if ( _key.getExtent().contains( centroid.x(), centroid.y() ) )
                {
                    return true;
                }
                /*else
                {   
                    OE_NOTICE << "Key = " << _key.str() << " Centroid " << centroid.x() << ", " << centroid.y() << " " << std::endl;
                    OE_NOTICE << "Extents " << _key.getExtent().toString() << std::endl;

                }*/
            }
            else
            {
                OE_NOTICE << "Invalid feature bounds" << std::endl;
            }
        }
        else
        {
            OE_NOTICE << "Bad geometry" << std::endl;
        }
        return false;
    }

    void split()
    {
        if (!_isSplit)
        {
            for (unsigned int i = 0; i < 4; ++i)
            {                
                _children[i] = new FeatureTile(_key.createChildKey( i ) );
            }
            _isSplit = true;
        }
    }

    void accept( FeatureTileVisitor* v);
    
    void traverse( FeatureTileVisitor* v )
    {
        if (_isSplit)
        {
            for (unsigned int i = 0; i < 4; ++i)
            {
                _children[i]->accept( v );
            }
        }
    }

    void commit()
    {
        //Maybe not necessary since we have the feature list
    }

    FeatureList& getFeatures()
    {
        return _features;
    }

private:
    FeatureList _features;
    TileKey _key;   
    osg::ref_ptr<FeatureTile> _children[4];
    bool _isSplit;
};

class FeatureTileVisitor : public osg::Referenced
{
public:
    virtual void traverse(FeatureTile* tile)
    {
        OE_NOTICE << "Busted..." << std::endl;
        tile->traverse( this );
    }
};



void FeatureTile::accept(FeatureTileVisitor* v)
{    
    v->traverse( this );
}

class AddFeatureVisitor : public FeatureTileVisitor
{
public:
    AddFeatureVisitor( Feature* feature, int maxFeatures, int firstLevel, int maxLevel):
      _feature( feature ),
      _maxFeatures( maxFeatures ),
      _maxLevel( maxLevel ),
      _firstLevel( firstLevel ),
      _added(false),
      _levelAdded(-1)
    {

    }

    virtual void traverse( FeatureTile* tile)
    {
        //If the node contains the feature, and it doesn't contain the max number of features add it.  If it's already full then 
        //split it.
        //TODO:  Does just doing a centroid check actually make sense here?
        if (tile->containsCentroid( _feature ) )
        {
            if (tile->getKey().getLevelOfDetail() >= _firstLevel && (tile->getFeatures().size() < _maxFeatures || tile->getKey().getLevelOfDetail() == _maxLevel))
            {
                tile->getFeatures().push_back( _feature.get() );
                //OE_NOTICE << "Adding feature to tile " << tile->getKey().str() << " which now has " << tile->getFeatures().size() << " features" << std::endl;
                //TODO:  Commit if we're full
                _added = true;
                _levelAdded = tile->getKey().getLevelOfDetail();
            }
            else
            {   
                //OE_NOTICE << "Splitting" << std::endl;
                tile->split();
                tile->traverse( this );
            }
        }
    }

    int _levelAdded;

    bool _added;
    int _maxFeatures;
    int _firstLevel;
    int _maxLevel;

    osg::ref_ptr< Feature > _feature;
};



std::string featureToJSON( Feature * feature )
{
    std::stringstream buf;

    buf << "{\"type\" : \"Feature\", " 
        << "\"id\": " << feature->getFID() << ","
        << "\"geometry\": " << GeometryUtils::geometryToGeoJSON( feature->getGeometry() ) << ",";
    
    //Write out all the properties
    buf << "\"properties\": {";
    if (feature->getAttrs().size() > 0)
    {
        AttributeTable::const_iterator last_attr = feature->getAttrs().end();
        last_attr--;

        for (AttributeTable::const_iterator itr = feature->getAttrs().begin(); itr != feature->getAttrs().end(); ++itr)
        {
            buf << "\"" << itr->first << "\": \"" << itr->second.getString() << "\"";
            if (itr != last_attr)
            {
                buf << ",";
            }
        }
    }    
    buf << "}"; //End of properties
    buf << "}";

    return buf.str();
}

std::string featuresToJson( FeatureList& features)
{
    std::stringstream buf;

    buf << "{\"type\": \"FeatureCollection\", \"features\": [";

    FeatureList::iterator last = features.end();
    last--;

    for (FeatureList::iterator i = features.begin(); i != features.end(); i++)
    {
        buf << featureToJSON( i->get() );
        if (i != last)
        {
            buf << ",";
        }
    }

    buf << "]}";

    return buf.str();

}

void printAllFeatures(FeatureSource* features)
{
    osg::ref_ptr< FeatureCursor > cursor = features->createFeatureCursor();
    while (cursor.valid() && cursor->hasMore())
    {
        osg::ref_ptr< Feature > feature = cursor->nextFeature();
        //printFeature( feature.get() );
        OE_NOTICE << featureToJSON( feature.get() ) << std::endl;
    }
}

void buildIndex(FeatureSource* features)
{
    Index index;
    osg::ref_ptr< FeatureCursor > cursor = features->createFeatureCursor();
    OE_NOTICE << "Building index..." << std::endl;
    int count = 0;    
    while (cursor.valid() && cursor->hasMore())
    {
        count++;
        osg::ref_ptr< Feature > feature = cursor->nextFeature();
        double value = feature->getDouble("hgt");
        index.insert( feature->getFID(), value );
        //OE_NOTICE << "Inserted " << feature->getFID() << " val=" << value << std::endl;
    }
    OE_NOTICE << "Inserted " << count << " items, finalizing" << std::endl;
    index.finalize();
    OE_NOTICE << "Done building index..." << std::endl;

    for (FeatureValueList::iterator i = index.values.begin(); i != index.values.end(); i++)
    {
        osg::ref_ptr< Feature > feature = features->getFeature( i->first );
        if (feature)
        {
        OE_NOTICE << "Got feature " << feature->getFID() << " with value " << i->second << std::endl;
        OE_NOTICE << featureToJSON( feature ) << std::endl;
        }
        else
        {
            OE_NOTICE << "Error, couldn't get feature " << i->first << std::endl;
        }
    }
}

class TFSMeta
{
    std::string _title;
};

class WriteFeaturesVisitor : public FeatureTileVisitor
{
public:
    WriteFeaturesVisitor(const std::string& dest, const std::string &layer):
      _dest( dest ),
      _layer( layer )
    {

    }

    virtual void traverse( FeatureTile* tile)
    {
        //if (tile->getFeatures().size() > 0)
        {   
            std::string contents = featuresToJson( tile->getFeatures() );
            std::stringstream buf;
            int x =  tile->getKey().getTileX();
            unsigned int numRows, numCols;
            tile->getKey().getProfile()->getNumTiles(tile->getKey().getLevelOfDetail(), numCols, numRows);
            int y  = numRows - tile->getKey().getTileY() - 1;

            buf << osgDB::concatPaths( _dest, _layer ) << "/" << tile->getKey().getLevelOfDetail() << "/" << x << "/" << y << ".json";
            std::string filename = buf.str();
            //OE_NOTICE << "Writing " << tile->getFeatures().size() << " to " << filename << std::endl;
            
            if ( !osgDB::fileExists( osgDB::getFilePath(filename) ) )
                osgDB::makeDirectoryForFile( filename );


            std::fstream output( filename.c_str(), std::ios_base::out );
            if ( output.is_open() )
            {
                output << contents;
                output.flush();
                output.close();                
            }            
        }
        tile->traverse( this );        
    }

    std::string _dest;
    std::string _layer;
};

void buildTFS(FeatureSource* features, int firstLevel, int maxLevel, int maxFeatures, 
                                       const std::string& dest, const std::string& layername,
                                       const std::string& description)
{        
    const GeoExtent &extent = features->getFeatureProfile()->getExtent();
    osg::ref_ptr< const osgEarth::Profile > profile = osgEarth::Profile::create(extent.getSRS(), extent.xMin(), extent.yMin(), extent.xMax(), extent.yMax(), 1, 1);
    

    TileKey rootKey = TileKey(0, 0, 0, profile );


    osg::ref_ptr< FeatureTile > root = new FeatureTile( rootKey );
    //Loop through all the features and try to insert them into the quadtree
    osg::ref_ptr< FeatureCursor > cursor = features->createFeatureCursor();
    int added = 0;
    int failed = 0;
    int highestLevel = 0;
    
    while (cursor.valid() && cursor->hasMore())
    {        
        osg::ref_ptr< Feature > feature = cursor->nextFeature();
        double x = feature->getGeometry()->getBounds().center().x();
        double y = feature->getGeometry()->getBounds().center().y();

       
        AddFeatureVisitor v(feature.get(), maxFeatures, firstLevel, maxLevel);
        root->accept( &v );
        if (!v._added)
        {
            OE_NOTICE << std::endl;
            failed++;
        }
        else
        {
            if (highestLevel < v._levelAdded)
            {
                highestLevel = v._levelAdded;
            }
            added++;
        }        
    }   
    OE_NOTICE << "Added " << added << " Failed " << failed << std::endl;

    WriteFeaturesVisitor write(dest, layername);
    root->accept( &write );

    //Write out the meta doc
    TFSLayer layer;
    layer._title = layername;
    layer._abstract = description;
    layer._firstLevel = firstLevel;
    layer._maxLevel = highestLevel;
    layer._extent = profile->getExtent();
    TFSReaderWriter::write( layer, osgDB::concatPaths(dest, layername) + "/tfs.xml");
}

int
usage( const std::string& msg )
{
    if ( !msg.empty() )
    {
        std::cout << msg << std::endl;
    }

    std::cout
        << std::endl
        << "USAGE: osgearth_tfs [options] filename" << std::endl
        << std::endl
        << "    --firstlevel                      ; The first level where features will be added to the quadtree" << std::endl
        << "    --maxlevel                        ; The maximum level of the feature quadtree" << std::endl
        << "    --maxfeatures                     ; The maximum number of features per tile" << std::endl
        << "    --destination                     ; The destination directory" << std::endl
        << "    --layer                           ; The name of the layer" << std::endl
        << "    --description                     ; The abstract/description of the layer" << std::endl
        << std::endl;

    return -1;
}



int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    if (argc < 2)
    {
        return usage("");
    }
    
    //The first level
    unsigned int firstLevel = 0;
    while (arguments.read("--firstlevel", firstLevel));

    //The max level
    unsigned int maxLevel = 8;
    while (arguments.read("--maxlevel", maxLevel));

    unsigned int maxFeatures = 500;
    while (arguments.read("--maxfeatures", maxFeatures));    

    //The destination directory
    std::string destination = "out";
    while (arguments.read("--destination", destination));

    //The name of the layer
    std::string layer = "layer";
    while (arguments.read("--layer", layer));

    //The description of the layer
    std::string description = "";
    while (arguments.read("--description", description));

    std::string filename;

    //Get the first argument that is not an option
    for(int pos=1;pos<arguments.argc();++pos)
    {
        if (!arguments.isOption(pos))
        {
            filename  = arguments[ pos ];
        }
    }

    if (filename.empty())
    {
        return usage( "Please provide a filename" );
    }


    



    //Open the feature source
    OGRFeatureOptions featureOpt;
    featureOpt.url() = filename;

    osg::ref_ptr< FeatureSource > features = FeatureSourceFactory::create( featureOpt );
    if (!features.valid())
    {
        OE_NOTICE << "Failed to open " << filename << std::endl;
        return 1;
    }

    features->initialize();
    const FeatureProfile* profile = features->getFeatureProfile();
    if (!profile)
    {
        OE_NOTICE << "Failed to create a valid profile for " << filename << std::endl;
        return 1;
    }

    OE_NOTICE << "Processing " << filename << std::endl
              << "  FirstLevel=" << firstLevel << std::endl
              << "  MaxLevel=" << maxLevel << std::endl
              << "  MaxFeatures=" << maxFeatures << std::endl
              << "  Destination=" << destination << std::endl
              << "  Layer=" << layer << std::endl
              << "  Description=" << description << std::endl
              << std::endl;



    osg::Timer_t startTime = osg::Timer::instance()->tick();
    buildTFS( features.get(), firstLevel, maxLevel, maxFeatures, destination, layer, description);
    osg::Timer_t endTime = osg::Timer::instance()->tick();
    OE_NOTICE << "Completed in " << osg::Timer::instance()->delta_s( startTime, endTime ) << std::endl;

    return 0;
}
