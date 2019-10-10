/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2016 Pelican Mapping
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
#include "BuildingFactory"
#include "BuildingSymbol"
#include "BuildingVisitor"
#include "BuildContext"
#include "Parapet"

#include <osgEarthFeatures/Session>
#include <osgEarthFeatures/AltitudeFilter>
#include <osgEarthSymbology/Geometry>
#include <osgEarthSymbology/ResourceLibrary>
#include <osgEarthSymbology/StyleSheet>

using namespace osgEarth;
using namespace osgEarth::Buildings;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

#define LC "[BuildingFactory] "

BuildingFactory::BuildingFactory()
{
    setSession( new Session(0L) );
}

void
BuildingFactory::setSession(Session* session)
{
    _session = session;
}

bool
BuildingFactory::cropToCentroid(const Feature* feature, const GeoExtent& extent) const
{
    if ( !extent.isValid() )
        return true;

    // make sure the centroid is in the crop-to extent: 
    GeoPoint centroid( feature->getSRS(), feature->getGeometry()->getBounds().center() );
    return extent.contains(centroid);
}

bool
BuildingFactory::create(Feature*               feature,
                        const GeoExtent&       cropTo,
                        ElevationEnvelope*     terrain,
                        const Style*           style,
                        BuildingVector&        output,
                        const osgDB::Options*  readOptions,
                        ProgressCallback*      progress)
{
    if ( !feature || !feature->getGeometry() )
        return false;

    double xformTime=0.0, clampTime=0.0, symbolTime=0.0, createTime=0.0;
    OE_START_TIMER(total);

    // TODO: for the single-feature version here, consider a token or other way
    // to compute all this common stuff up front. This was not necessary for the
    // FeatureCursor variation. -gw

    bool needToClamp = 
        terrain &&
        style &&
        style->has<AltitudeSymbol>() &&
        style->get<AltitudeSymbol>()->clamping() != AltitudeSymbol::CLAMP_NONE;

    // Find the building symbol if there is one; this will tell us how to 
    // resolve building heights, among other things.
    const BuildingSymbol* buildingSymbol =
        style ? style->get<BuildingSymbol>() :
        _session->styles() ? _session->styles()->getDefaultStyle()->get<BuildingSymbol>() :
        0L;
        
  
    // Pull a resource library if one is defined.
    ResourceLibrary* reslib = 0L;
    if (buildingSymbol && buildingSymbol->library().isSet())
    {
        optional<StringExpression>  librayExpr = buildingSymbol->library();
        std::string library = feature->eval(librayExpr.mutable_value(), _session.get());
        reslib = _session->styles()->getResourceLibrary(library);
    }
    if ( !reslib )
    {
        reslib = _session->styles()->getDefaultResourceLibrary();
    }

    // Construct a context to use during the build process.
    BuildContext context;
    context.setDBOptions( readOptions );
    context.setResourceLibrary( reslib );

    // URI context for external models
    URIContext uriContext( readOptions );

    // Set up mutable expression instances:
    optional<StringExpression>  modelExpr;
    optional<NumericExpression> heightExpr;
    optional<StringExpression>  tagsExpr;

    if ( buildingSymbol )
    {
        modelExpr  = buildingSymbol->modelURI();
        heightExpr = buildingSymbol->height();
        tagsExpr   = buildingSymbol->tags();
    }

    if ( progress && progress->isCanceled() )
    {
        progress->message() = "in BuildingFactory::create";
        return false;
    }

    // resolve selection values from the symbology:
    optional<URI> externalModelURI;
    float         height    = 0.0f;
    unsigned      numFloors = 0u;
    TagVector     tags;

    if ( buildingSymbol )
    {
        OE_START_TIMER(symbol);

        // see if we are referencing an external model.
        if ( modelExpr.isSet() )
        {
            std::string modelStr = feature->eval(modelExpr.mutable_value(), _session.get());
            //VRV_PATCH
            //Sometimes the ductape engine returns undefined for the modelStr.
            // lets set the modelStr to "" and ignore those cases as well
            if (modelStr == "undefined")
            {
               modelStr = "";
            }
            //END VRV_PATCH
            if (!modelStr.empty())
            {
                externalModelURI = URI(modelStr, uriContext);
            }
        }

        // calculate height from expression. We do this first because
        // a height of zero will cause us to skip the feature altogether.
        if ( !externalModelURI.isSet() && heightExpr.isSet() )
        {
            height = (float)feature->eval(heightExpr.mutable_value(), _session.get());
        
            if ( height > 0.0f )
            {
                // calculate tags from expression:
                if ( tagsExpr.isSet() )
                {
                    std::string tagString = trim(feature->eval(tagsExpr.mutable_value(), _session.get()));
                    if ( !tagString.empty() )
                        StringTokenizer(tagString, tags, " ", "\"", false);
                }

                else
                {
                    // If there's no tag expression, the default is "building".
                    tags.push_back("building");
                }
            }
        }

        symbolTime = OE_GET_TIMER(symbol);
    }

    if ( height > 0.0f || externalModelURI.isSet() )
    {
        OE_START_TIMER(xform);

        // Removing co-linear points will help produce a more "true"
        // longest-edge for rotation and roof rectangle calcuations.
        feature->getGeometry()->removeColinearPoints();

        // Transform the feature into the output SRS
        if ( _outSRS.valid() )
        {
            feature->transform( _outSRS.get() );
        }

        // this ensures that the feature's centroid is in our bounding
        // extent, so that a feature doesn't end up in multiple extents
        if ( !cropToCentroid(feature, cropTo) )
        {
            return true;
        }

        xformTime = OE_GET_TIMER(xform);


        // Prepare for terrain clamping by finding the minimum and 
        // maximum elevations under the feature:
        OE_START_TIMER(clamp);
                
        float min = FLT_MAX, max = -FLT_MAX;
        bool terrainMinMaxValid =
            needToClamp &&
            terrain &&
            feature->getGeometry() != 0L &&
            terrain->getElevationExtrema(feature->getGeometry()->asVector(), min, max);
                
        context.setTerrainMinMax(
            terrainMinMaxValid ? min : 0.0f,
            terrainMinMaxValid ? max : 0.0f );

        clampTime = OE_GET_TIMER(clamp);


        OE_START_TIMER(create);

        // If this is an external model, set up a building referencing the model
        if ( externalModelURI.isSet() )
        {
            Building* building = createExternalModelBuilding( feature, externalModelURI.get(), context );
            if ( building )
            {
                output.push_back( building );
            }
        }

        // Otherwise, we are creating a parametric building:
        else
        {
            // If using a catalog, ask it to create one or more buildings for this feature:
            if ( _catalog.valid() )
            {   
                float minHeight = terrainMinMaxValid ? max-min+3.0f : 3.0f;
                height = std::max( height, minHeight );
                _catalog->createBuildings(feature, tags, height, context, output, progress);
            }

            // Otherwise, create a simple one by hand:
            else
            {
                Building* building = createBuilding(feature, progress);
                if ( building )
                {
                    output.push_back( building );
                }
            }
        }

        createTime = OE_GET_TIMER(create);
    }

    double totalTime = OE_GET_TIMER(total);

    if ( progress && progress->collectStats() )
    {
        progress->stats("factory.xform")  += xformTime;
        progress->stats("factory.clamp")  += clampTime;
        progress->stats("factory.symbol") += symbolTime;
        progress->stats("factory.create") += createTime;
    }

    return true;
}

Building*
BuildingFactory::createExternalModelBuilding(Feature*      feature,
                                             const URI&    modelURI,
                                             BuildContext& context)
{
    if ( !feature || modelURI.empty() )
        return 0L;

    Geometry* geometry = feature->getGeometry();
    if ( !geometry || !geometry->isValid() )
        return 0L;

    osg::ref_ptr<Building> building = new Building();
    building->setExternalModelURI( modelURI );

    // Calculate a local reference frame for this building.
    // The frame clamps the building by including the terrain elevation that was passed in.
    osg::Vec2d center2d = geometry->getBounds().center2d();
    GeoPoint centerPoint( feature->getSRS(), center2d.x(), center2d.y(), context.getTerrainMin(), ALTMODE_ABSOLUTE );
    osg::Matrix local2world;
    centerPoint.createLocalToWorld( local2world );
    building->setReferenceFrame( local2world );

    return building.release();
}

Building*
BuildingFactory::createBuilding(Feature* feature, ProgressCallback* progress)
{
    if ( feature == 0L )
        return 0L;

    osg::ref_ptr<Building> building;

    Geometry* geometry = feature->getGeometry();

    if ( geometry && geometry->getComponentType() == Geometry::TYPE_POLYGON && geometry->isValid() )
    {
        // Calculate a local reference frame for this building:
        osg::Vec2d center2d = geometry->getBounds().center2d();
        GeoPoint centerPoint( feature->getSRS(), center2d.x(), center2d.y(), 0.0, ALTMODE_ABSOLUTE );
        osg::Matrix local2world, world2local;
        centerPoint.createLocalToWorld( local2world );
        world2local.invert( local2world );

        // Transform feature geometry into the local frame. This way we can do all our
        // building creation in cartesian, single-precision space.
        GeometryIterator iter(geometry, true);
        while(iter.hasMore())
        {
            Geometry* part = iter.next();
            for(Geometry::iterator i = part->begin(); i != part->end(); ++i)
            {
                osg::Vec3d world;
                feature->getSRS()->transformToWorld( *i, world );
                (*i) = world * world2local;
            }
        }

        BuildContext context;
        context.setSeed( feature->getFID() );

        // Next, iterate over the polygons and set up the Building object.
        GeometryIterator iter2( geometry, false );
        while(iter2.hasMore())
        {
            Polygon* polygon = dynamic_cast<Polygon*>(iter2.next());
            if ( polygon && polygon->isValid() )
            {
                // A footprint is the minumum info required to make a building.
                building = createSampleBuilding( feature );

                // Install the reference frame of the footprint geometry:
                building->setReferenceFrame( local2world );

                // Do initial cleaning of the footprint and install is:
                cleanPolygon( polygon );

                // Finally, build the internal structure from the footprint.
                building->build( polygon, context );
            }
            else
            {
                OE_WARN << LC << "Feature " << feature->getFID() << " is not a polygon. Skipping..\n";
            }
        }
    }

    return building.release();
}

void
BuildingFactory::cleanPolygon(Polygon* polygon)
{
    polygon->open();

    polygon->removeDuplicates();

    polygon->rewind( Polygon::ORIENTATION_CCW );

    // TODO: remove colinear points? for skeleton?
}

Building*
BuildingFactory::createSampleBuilding(const Feature* feature)
{
    Building* building = new Building();
    building->setUID( feature->getFID() );

    // figure out the building's height and number of floors.
    // single-elevation building.
    float height       = 15.0f;
    unsigned numFloors = 1u;

    // Add a single elevation.
    Elevation* elevation = new Elevation();
    building->getElevations().push_back(elevation);
    
    Roof* roof = new Roof();
    roof->setType( Roof::TYPE_FLAT );
    elevation->setRoof( roof );
    
    SkinResource* wallSkin = 0L;
    SkinResource* roofSkin = 0L;

    if ( _session.valid() )
    {
        ResourceLibrary* reslib = _session->styles()->getDefaultResourceLibrary();
        if ( reslib )
        {
            wallSkin = reslib->getSkin( "facade.commercial.1" );
            elevation->setSkinResource( wallSkin );

            roofSkin = reslib->getSkin( "roof.commercial.1" );
            roof->setSkinResource( roofSkin );
        }
        else
        {
            //OE_WARN << LC << "No resource library\n";
        }

        const BuildingSymbol* sym = _session->styles()->getDefaultStyle()->get<BuildingSymbol>();
        if ( sym )
        {
            if ( feature )
            {
                NumericExpression heightExpr = sym->height().get();
                height = feature->eval( heightExpr, _session.get() );
            }

            // calculate the number of floors
            if ( wallSkin )
            {
                numFloors = (unsigned)std::max(1.0f, osg::round(height / wallSkin->imageHeight().get()));
            }
            else
            {
                numFloors = (unsigned)std::max(1.0f, osg::round(height / sym->floorHeight().get()));
            }
        }
    }

    elevation->setHeight( height );
    elevation->setNumFloors( numFloors );

    Parapet* parapet = new Parapet();
    parapet->setParent( elevation );
    parapet->setWidth( 2.0f );
    parapet->setHeight( 2.0f );
    parapet->setNumFloors( 1u );

    parapet->setColor( Color::Gray.brightness(1.3f) );
    parapet->setRoof( new Roof() );
    parapet->getRoof()->setSkinResource( roofSkin );
    parapet->getRoof()->setColor( Color::Gray.brightness(1.2f) );

    elevation->getElevations().push_back( parapet );

    return building;
}
