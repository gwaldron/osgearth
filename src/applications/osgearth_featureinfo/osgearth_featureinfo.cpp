/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2013 Pelican Mapping
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

#include <osgEarthDrivers/feature_ogr/OGRFeatureOptions>

using namespace osgEarth::Features;
using namespace osgEarth::Drivers;
using namespace osgEarth::Symbology;

#include <osgEarthFeatures/GeometryUtils>

std::string attributeTypeToString( AttributeType type )
{
    switch (type)
    {
    case ATTRTYPE_BOOL: return "Boolean";
    case ATTRTYPE_DOUBLE: return "Double";
    case ATTRTYPE_INT: return "Integer";
    case ATTRTYPE_STRING: return "String";
    default:  return "Unspecified";
    }
}

std::string indent = "    ";

void printStats(FeatureSource* features)
{
    std::cout << "Feature Count:  " << features->getFeatureCount() << std::endl;
    std::cout << "Geometry Type:  " << osgEarth::Symbology::Geometry::toString( features->getGeometryType() ) << std::endl;

    //Print the schema
    const FeatureSchema schema = features->getSchema();
    std::cout << "Schema:" << std::endl;
    for (FeatureSchema::const_iterator itr = schema.begin(); itr != schema.end(); ++itr)
    {
        std::cout << indent << itr->first << ": " << attributeTypeToString(itr->second) << std::endl;
    }
    std::cout << std::endl;
}

void printFeature( Feature* feature )
{
    std::cout << "FID: " << feature->getFID() << std::endl;
    for (AttributeTable::const_iterator itr = feature->getAttrs().begin(); itr != feature->getAttrs().end(); ++itr)
    {
        std::cout 
            << indent 
            << itr->first << "=" << itr->second.getString() << " ("
            << (itr->second.first == ATTRTYPE_INT?    "integer" :
                itr->second.first == ATTRTYPE_DOUBLE? "double" :
                itr->second.first == ATTRTYPE_BOOL?   "bool" :
                "string")
            << ")" << std::endl;
    }

    //Print out the geometry
    Geometry* geom = feature->getGeometry();
    if (geom)
    {
        std::cout << indent << GeometryUtils::geometryToWKT( geom ) << std::endl;
    }
    std::cout << std::endl;
}

void printAllFeatures(FeatureSource* features)
{
    osg::ref_ptr< FeatureCursor > cursor = features->createFeatureCursor();
    while (cursor.valid() && cursor->hasMore())
    {
        osg::ref_ptr< Feature > feature = cursor->nextFeature();
        printFeature( feature.get() );
    }
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
        << "USAGE: osgearth_featureinfo [options] filename" << std::endl
        << std::endl
        << "    --printfeatures                   ; Prints all features in the source" << std::endl
        << "    --delete fid                      ; Deletes the given FID from the source." << std::endl
        << "    --fid fid                         ; Displays the given FID." << std::endl
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


    std::vector< FeatureID > toDelete;
    int fid;
    while (arguments.read("--delete", fid))
    {
        toDelete.push_back( fid );
    }

    std::vector< FeatureID > fids;
    while (arguments.read("--fid", fid))
    {
        fids.push_back( fid );
    }
    
    bool printFeatures = false;
    if (arguments.read("--printfeatures" )) printFeatures = true;

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

    bool write = toDelete.size() > 0;



    //Open the feature source
    OGRFeatureOptions featureOpt;
    featureOpt.url() = filename;
    featureOpt.openWrite() = write;

    osg::ref_ptr< FeatureSource > features = FeatureSourceFactory::create( featureOpt );
    features->initialize();
    features->getFeatureProfile();

    //Delete any features if requested
    if (toDelete.size() > 0)
    {
        for (unsigned int i = 0; i < toDelete.size(); ++i)
        {
            FeatureID fid = toDelete[i];
            std::cout << "Deleting Feature " << fid << std::endl;
            features->deleteFeature( fid );
        }
    }
    else if (fids.size() > 0)
    {
        //Print out any specific FIDs
        for (unsigned int i = 0; i < fids.size(); ++i)
        {
            FeatureID fid = fids[i];
            osg::ref_ptr< Feature > feature = features->getFeature( fid );
            if (feature.valid())
            {
                printFeature( feature.get() );
            }
            else
            {
                std::cout << "Couldn't get feature " << fid << std::endl;
            }
        }
    }
    else
    {
        //Print out feature info
        printStats( features.get() );

        if (printFeatures)
        {
            printAllFeatures( features.get() );
        }
    }

    return 0;
}
