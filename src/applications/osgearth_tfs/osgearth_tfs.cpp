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
#include <osgEarthUtil/TFS>

#include <osgEarthDrivers/feature_ogr/OGRFeatureOptions>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Features;
using namespace osgEarth::Drivers;
using namespace osgEarth::Symbology;

#include <osgEarthFeatures/GeometryUtils>


typedef std::pair< FeatureID, double> FeatureValue;

typedef std::list< FeatureValue > FeatureValueList;

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
            }
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
        if (tile->containsCentroid( _feature ) )
        {
            if (tile->getKey().getLevelOfDetail() >= _firstLevel && 
               (tile->getFeatures().size() < _maxFeatures || tile->getKey().getLevelOfDetail() == _maxLevel))
            {
                tile->getFeatures().push_back( _feature.get() );
                //OE_NOTICE << "Adding feature to tile " << tile->getKey().str() << " which now has " << tile->getFeatures().size() << " features" << std::endl;
                //TODO:  Go ahead and write the tile to disk if it's full
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

void printAllFeatures(FeatureSource* features, const Query& query)
{
    osg::ref_ptr< FeatureCursor > cursor = features->createFeatureCursor(query);
    while (cursor.valid() && cursor->hasMore())
    {
        osg::ref_ptr< Feature > feature = cursor->nextFeature();        
        OE_NOTICE << feature.get()->getGeoJSON() << std::endl;
    }
}

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
        if (tile->getFeatures().size() > 0)
        {   
            std::string contents = Feature::featuresToGeoJSON( tile->getFeatures() );
            std::stringstream buf;
            int x =  tile->getKey().getTileX();
            unsigned int numRows, numCols;
            tile->getKey().getProfile()->getNumTiles(tile->getKey().getLevelOfDetail(), numCols, numRows);
            int y  = numRows - tile->getKey().getTileY() - 1;

            buf << osgDB::concatPaths( _dest, _layer ) << "/" << tile->getKey().getLevelOfDetail() << "/" << x << "/" << y << ".json";
            std::string filename = buf.str();
            OE_NOTICE << "Writing " << tile->getFeatures().size() << " features to " << filename << std::endl;
            
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
                                       const std::string& description,
                                       const Query& query )
{        
    const GeoExtent &extent = features->getFeatureProfile()->getExtent();
    osg::ref_ptr< const osgEarth::Profile > profile = osgEarth::Profile::create(extent.getSRS(), extent.xMin(), extent.yMin(), extent.xMax(), extent.yMax(), 1, 1);
    

    TileKey rootKey = TileKey(0, 0, 0, profile );


    osg::ref_ptr< FeatureTile > root = new FeatureTile( rootKey );
    //Loop through all the features and try to insert them into the quadtree
    osg::ref_ptr< FeatureCursor > cursor = features->createFeatureCursor( query );
    int added = 0;
    int failed = 0;
    int skipped = 0;
    int highestLevel = 0;
    
    while (cursor.valid() && cursor->hasMore())
    {        
        osg::ref_ptr< Feature > feature = cursor->nextFeature();

        if (feature->getGeometry() && feature->getGeometry()->getBounds().valid() && feature->getGeometry()->size() > 0)
        {

            AddFeatureVisitor v(feature.get(), maxFeatures, firstLevel, maxLevel);
            root->accept( &v );
            if (!v._added)
            {
                OE_NOTICE << "Failed to add feature " << feature->getFID() << std::endl;
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
        else
        {
            OE_NOTICE << "Skipping feature " << feature->getFID() << " with null or invalid geometry" << std::endl;
            skipped++;
        }
    }   
    OE_NOTICE << "Added=" << added << "Skipped=" << skipped << " Failed=" << failed << std::endl;

    WriteFeaturesVisitor write(dest, layername);
    root->accept( &write );

    //Write out the meta doc
    TFSLayer layer;
    layer.setTitle( layername );
    layer.setAbstract( description );
    layer.setFirstLevel( firstLevel );
    layer.setMaxLevel( highestLevel );
    layer.setExtent( profile->getExtent() );
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
        << "    --expression                      ; The expression to run on the feature source, specific to the feature source" << std::endl
        << "    --orderby                         ; Sort the features, if not already included in the expression" << std::endl
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
    unsigned int maxLevel = 6;
    while (arguments.read("--maxlevel", maxLevel));

    unsigned int maxFeatures = 300;
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

    std::string queryExpression = "";
    while (arguments.read("--expression", queryExpression));

    std::string queryOrderBy = "";
    while (arguments.read("--orderby", queryOrderBy));
    
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
              << "  Expression=" << queryExpression << std::endl
              << "  OrderBy=" << queryOrderBy << std::endl
              << std::endl;


    Query query;
    if (!queryExpression.empty())
    {
        query.expression() = queryExpression;
    }

    if (!queryOrderBy.empty())
    {
        query.orderby() = queryOrderBy;
    }

    osg::Timer_t startTime = osg::Timer::instance()->tick();
    buildTFS( features.get(), firstLevel, maxLevel, maxFeatures, destination, layer, description, query);
    //printAllFeatures( features.get(), query );
    osg::Timer_t endTime = osg::Timer::instance()->tick();
    OE_NOTICE << "Completed in " << osg::Timer::instance()->delta_s( startTime, endTime ) << " s " << std::endl;

    return 0;
}
