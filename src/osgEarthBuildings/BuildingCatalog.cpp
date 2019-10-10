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
#include "BuildingCatalog"
#include "Parapet"
#include "Roof"
#include "BuildingSymbol"
#include "BuildingVisitor"
#include "BuildContext"

#include <osgEarth/XmlUtils>
#include <osgEarth/Containers>
#include <osgEarthSymbology/Style>

using namespace osgEarth;
using namespace osgEarth::Symbology;
using namespace osgEarth::Buildings;

#define LC "[BuildingCatalog] "


BuildingCatalog::BuildingCatalog()
{
    //nop
}

bool
BuildingCatalog::createBuildings(Feature*              feature,
                                 const TagVector&      tags,
                                 float                 height,
                                 BuildContext&         context,
                                 BuildingVector&       output,
                                 ProgressCallback*     progress) const
{
    if ( !feature )
        return false;

    Geometry* geometry = feature->getGeometry();

    if ( geometry && geometry->getComponentType() == Geometry::TYPE_POLYGON && geometry->isValid() )
    { 
        // Calculate a local reference frame for this building:
        osg::Vec2d center2d = geometry->getBounds().center2d();
        GeoPoint centerPoint( feature->getSRS(), center2d.x(), center2d.y(), context.getTerrainMin(), ALTMODE_ABSOLUTE );
        osg::Matrix local2world, world2local;
        centerPoint.createLocalToWorld( local2world );
        world2local.invert( local2world );

        // Transform feature geometry into the local frame. This way we can do all our
        // building creation in cartesian space.
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

        // Next, iterate over the polygons and set up the Building object.
        GeometryIterator iter2( geometry, false );
        while(iter2.hasMore())
        {
            Polygon* polygon = dynamic_cast<Polygon*>(iter2.next());
            if ( polygon && polygon->isValid() )
            {
                float area = polygon->getBounds().area2d();

                // A footprint is the minumum info required to make a building.
                osg::ref_ptr<Building> building = cloneBuildingTemplate(feature, tags, height, area);

                if ( building )
                {
                    // Install the reference frame of the footprint geometry:
                    building->setReferenceFrame( local2world );

                    // Do initial cleaning of the footprint and install is:
                    cleanPolygon( polygon );

                    // Apply the height:
                    building->setHeight( height );

                    // Seed the random number generator with the local-space area:
                    context.setSeed( (unsigned)area );

                    // Build the internal structures:
                    if ( building->build(polygon, context) )
                    {
                        output.push_back( building.get() );
                    }
                    else
                    {
                        OE_WARN << "building::build() failed for some reason\n";
                    }
                }
            }
            else
            {
                OE_WARN << LC << "Feature " << feature->getFID() << " is not a polygon. Skipping..\n";
            }
        }
    }

    return true;
}

void
BuildingCatalog::cleanPolygon(Polygon* fp) const
{
    if ( fp && fp->isValid() )
    {
        fp->open();

        fp->removeDuplicates();

        fp->rewind( Polygon::ORIENTATION_CCW );
    }
    // TODO: remove colinear points? for skeleton?
}

Building*
BuildingCatalog::cloneBuildingTemplate(Feature*           feature,
                                       const TagVector&   tags,
                                       float              height,
                                       float              area) const
{
    if ( !_buildingsTemplates.empty() )
    {
        std::vector<unsigned> candidates;

        for(unsigned i=0; i<_buildingsTemplates.size(); ++i)
        {
            const Building* bt = _buildingsTemplates.at(i).get();

            bool heightOK = (height >= bt->getMinHeight() && height <= bt->getMaxHeight());
            if ( !heightOK )
                continue;

            bool areaOK = (area == 0.0f) || (area >= bt->getMinArea() && area <= bt->getMaxArea());
            if ( !areaOK )
                continue;

            bool tagsOK = tags.empty() || (bt->containsTags(tags));
            if ( !tagsOK )
                continue;

            candidates.push_back(i);
        }

        if ( !candidates.empty() )
        {
            UID uid = feature->getFID() + 1u;
            unsigned index = Random((unsigned)area).next(candidates.size());
            Building* copy = osg::clone( _buildingsTemplates.at( candidates[index] ).get() );
            copy->setUID( uid );
            return copy;
        }
    }

    return 0L;
}

bool
BuildingCatalog::load(const URI& uri, const osgDB::Options* dbo, ProgressCallback* progress)
{
    osg::ref_ptr<XmlDocument> xml = XmlDocument::load(uri, dbo);
    if ( !xml.valid() )
    {
        if ( progress ) progress->reportError("File not found");
        return false;
    }

    Config conf = xml->getConfig();
    const Config* root = conf.find("buildings", true);

    if ( root )
        return parseBuildings( *root, progress );
    else
        return false;
}

bool
BuildingCatalog::parseBuildings(const Config& conf, ProgressCallback* progress)
{
    for(ConfigSet::const_iterator b = conf.children().begin(); b != conf.children().end(); ++b)
    {
        if ( b->empty() )
            continue;

        Building* building = new Building();

        if ( b->hasValue("tags") )
        {
            building->addTags( b->value("tags") );
        }

        building->setMinHeight( b->value("min_height", 0.0f) );
        building->setMaxHeight( b->value("max_height", FLT_MAX) );

        building->setMinArea( b->value("min_area", 0.0f) );
        building->setMaxArea( b->value("max_area", FLT_MAX) );

        if ( b->value("instanced", false) == true )
        {
            ModelSymbol* ms = new ModelSymbol();
            ms->addTags( "instanced" );
//            ms->addTags( "building instanced" );
            if ( b->hasValue("tags") )
                ms->addTags( b->value("tags") );
            building->setInstancedModelSymbol( ms );
        }

        const Config* elevations = b->child_ptr("elevations");
        if ( elevations )
        {
            parseElevations( *elevations, building, 0L, building->getElevations(), 0L, progress );
        }

        _buildingsTemplates.push_back( building );
    }

    OE_INFO << LC << "Read " << _buildingsTemplates.size() << " building templates\n";

    return true;
}

bool
BuildingCatalog::parseElevations(const Config&     conf, 
                                 Building*         building,
                                 Elevation*        parent, 
                                 ElevationVector&  output,
                                 SkinSymbol*       parentSkinSymbol,
                                 ProgressCallback* progress)
{            
    for(ConfigSet::const_iterator e = conf.children().begin();
        e != conf.children().end();
        ++e)
    {
        Elevation* elevation = 0L;        

        if ( e->value("type") == "parapet" )
        {
            Parapet* parapet = new Parapet();
            parapet->setWidth( e->value("width", parapet->getWidth()) );
            elevation = parapet;
        }
        else
        {
            elevation = new Elevation();
        }

        if ( parent )
        {
            elevation->setParent( parent );
        }
        
        // resolve the skin symbol for this Elevation.
        SkinSymbol* skinSymbol = parseSkinSymbol( &(*e) );
        if ( skinSymbol )
        {
            // set and use as new parent
            elevation->setSkinSymbol( skinSymbol );
        }
        else
        {
            // use this parent as new parent for sub-elevations
            skinSymbol = parentSkinSymbol;
            if ( parent == 0L )
                elevation->setSkinSymbol( parentSkinSymbol );
        }

        if (e->hasValue("tag"))
            elevation->setTag(e->value("tag"));
        else if (e->hasValue("tags"))
            elevation->setTag(e->value("tags"));
        else if (parent)
            elevation->setTag(parent->getTag());            

        // resolve the height properties:
        optional<float> hp;
        if ( e->get( "height_percentage", hp) )
            elevation->setHeightPercentage( hp.get()*0.01f );

        if ( e->hasValue( "height" ) )
            elevation->setAbsoluteHeight( e->value("height", 15.0f) );

        if ( e->hasValue( "bottom" ) )
            elevation->setBottom( e->value("bottom", 0.0f) );

        elevation->setInset( e->value("inset", 0.0f) );
        elevation->setXOffset( e->value("xoffset", 0.0f) );
        elevation->setYOffset( e->value("yoffset", 0.0f) );

        // color:
        if ( e->hasValue("color") )
            elevation->setColor( Color(e->value("color")) );

        if ( e->value("simplify", false) == true )
            elevation->setRenderAsBox( true );

        if ( e->hasChild("roof") )
        {
            Roof* roof = parseRoof( e->child_ptr("roof"), progress );
            if ( roof )
            {
                elevation->setRoof( roof );
                if (roof->getTag().empty())
                {
                    roof->setTag(elevation->getTag());
                }
            }
        }

        output.push_back( elevation );

        const Config* children = e->child_ptr("elevations");
        if ( children )
        {
            parseElevations( *children, building, elevation, elevation->getElevations(), skinSymbol, progress );
        }
    }

    return true;
}

Roof*
BuildingCatalog::parseRoof(const Config* r, ProgressCallback* progress) const
{
    Roof* roof = new Roof();

    if ( r->value("type") == "gable" )
        roof->setType( Roof::TYPE_GABLE );
    else if ( r->value("type") == "instanced" )
        roof->setType( Roof::TYPE_INSTANCED );
    else
        roof->setType( Roof::TYPE_FLAT );

    SkinSymbol* skinSymbol = parseSkinSymbol( r );
    if ( skinSymbol )
    {
        roof->setSkinSymbol( skinSymbol );
    }

    if ( r->hasValue("color") )
        roof->setColor( Color(r->value("color")) );

    ModelSymbol* modelSymbol = parseModelSymbol( r );
    if ( modelSymbol )
    {
        roof->setModelSymbol( modelSymbol );
    }

    if (r->hasValue("tag"))
        roof->setTag(r->value("tag"));
    else if (r->hasValue("tags"))
        roof->setTag(r->value("tags"));

    return roof;
}

SkinSymbol*
BuildingCatalog::parseSkinSymbol(const Config* c) const
{
    SkinSymbol* skinSymbol = 0L;

    if ( c->hasValue("skin_name") )
    {
        skinSymbol = new SkinSymbol();
        skinSymbol->name() = c->value("skin_name");
    }
    else if ( c->hasValue("skin_tags") )
    {
        skinSymbol = new SkinSymbol();
        skinSymbol->addTags( c->value("skin_tags") );
    }

    return skinSymbol;
}

ModelSymbol*
BuildingCatalog::parseModelSymbol(const Config* c) const
{
    ModelSymbol* symbol = 0L;

    if ( c->hasValue("model_name") )
    {
        symbol = new ModelSymbol();
        symbol->name() = c->value("model_name");
    }
    else if ( c->hasValue("model_tags") )
    {
        symbol = new ModelSymbol();
        symbol->addTags( c->value("model_tags") );
    }

    return symbol;
}