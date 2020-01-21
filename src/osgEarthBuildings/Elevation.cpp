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
#include "Elevation"
#include "BuildContext"

#define LC "[Elevation] "

using namespace osgEarth;
using namespace osgEarth;
using namespace osgEarth::Buildings;

Elevation::Elevation() :
_height            ( 50.0f ),
_heightPercentage  ( 1.0f ),
_numFloors         ( _height.get()/3.5f ),
_bottom            ( 0.0f ),
_inset             ( 0.0f ),
_xoffset           ( 0.0f ),
_yoffset           ( 0.0f ),
_color             ( Color::White ),
_cosR              ( 1.0f ),
_sinR              ( 0.0f ),
_parent            ( 0L ),
_renderAABB        ( false )
{
    //nop
}

Elevation::Elevation(const Elevation& rhs) :
_height          ( rhs._height ),
_heightPercentage( rhs._heightPercentage ),
_numFloors       ( rhs._numFloors ),
_bottom          ( rhs._bottom ),
_inset           ( rhs._inset ),
_xoffset         ( rhs._xoffset ),
_yoffset         ( rhs._yoffset ),
_skinSymbol      ( rhs._skinSymbol.get() ),
_skinResource    ( rhs._skinResource.get() ),
_color           ( rhs._color ),
_cosR            ( rhs._cosR ),
_sinR            ( rhs._sinR ),
_aabb            ( rhs._aabb ),
_parent          ( rhs._parent ),
_renderAABB      ( rhs._renderAABB ),
_tag             ( rhs._tag ),
_longEdgeMidpoint( rhs._longEdgeMidpoint ),
_longEdgeInsideNormal( rhs._longEdgeInsideNormal )
{
    if ( rhs.getRoof() )
    {
        setRoof( new Roof(*rhs.getRoof()) );
    }

    for(ElevationVector::const_iterator e = rhs.getElevations().begin(); e != rhs.getElevations().end(); ++e) 
    {
        Elevation* copy = e->get()->clone();
        copy->setParent( this );
        _elevations.push_back( copy );
    }
}

Elevation*
Elevation::clone() const
{
    return new Elevation( *this );
}

void
Elevation::setRoof(Roof* roof)
{
    _roof = roof;
    if ( roof )
        roof->setParent( this );
}

void
Elevation::setHeight(float height)
{
    // if the height was already set expressly, skip this.
    if ( !_height.isSet() )
    {
        float newHeight = height;
        if ( _heightPercentage.isSet() )
        {
            float hp = osg::clampBetween(_heightPercentage.get(), 0.01f, 1.0f);
            newHeight = height * hp;
        }
        _height.init( newHeight );
    }

    else if ( _height.get() < 0.0f )
    {
        // negative height means total height - value:
        _height.init( height + _height.get() );
    }

    for(ElevationVector::iterator e = _elevations.begin(); e != _elevations.end(); ++e)
    {
        e->get()->setHeight( height );
    }
}

void
Elevation::setAbsoluteHeight(float height)
{
    _height = height;
}

float
Elevation::getBottom() const
{
    return
        _bottom.isSet() ? _bottom.get() :
        _parent         ? _parent->getTop() :
        0.0f;
}

float
Elevation::getTop() const
{
    return getBottom() + getHeight();
}

bool
Elevation::build(const Polygon* in_footprint, BuildContext& bc)
{
    if ( !in_footprint || !in_footprint->isValid() )
        return false;

    const Polygon* footprint = in_footprint;

    // For simplification we replace the footprint with its rotated bounding box:
    osg::ref_ptr<Polygon> box;
    if ( getRenderAsBox() )
    {
        calculateRotations( in_footprint );
        if ( _aabb.valid() )
        {
            box = new Polygon();
            osg::Vec3d p;
            p.set( _aabb.xMin(), _aabb.yMin(), 0.0f ); unrotate(p); box->push_back(p);
            p.set( _aabb.xMax(), _aabb.yMin(), 0.0f ); unrotate(p); box->push_back(p);
            p.set( _aabb.xMax(), _aabb.yMax(), 0.0f ); unrotate(p); box->push_back(p);
            p.set( _aabb.xMin(), _aabb.yMax(), 0.0f ); unrotate(p); box->push_back(p);
            footprint = box.get();
        }
    }

    // Buffer the footprint if necessary to apply an inset:
    if ( getInset() != 0.0f )
    {
        osg::ref_ptr<Geometry> inset;
        BufferParameters bp( BufferParameters::CAP_DEFAULT, BufferParameters::JOIN_MITRE );
        if ( footprint->buffer(-getInset(), inset, bp) )
        {
            return buildImpl( dynamic_cast<Polygon*>(inset.get()), bc );
        }
    }

    return buildImpl( footprint, bc );
}

bool
Elevation::buildImpl(const Polygon* footprint, BuildContext& bc)
{
    if ( !footprint || !footprint->isValid() )
    {
        OE_DEBUG << LC << "Discarding invalid footprint.\n";
        return false;
    }
    
    _walls.clear();

    /** calculates the rotation based on the footprint */
    calculateRotations( footprint );

#if 0
    // offsets: shift the coordinates relative to the dominant rotation angle:
    if ( getXOffset() != 0.0f || getYOffset() != 0.0f )
    {
        if ( !copy.valid() )
            copy = footprint->clone();

        float dx = getXOffset(), dy = getYOffset();
        rotate( dx, dy );
        
        GeometryIterator gi( copy.get() );
        while( gi.hasMore() ) {
            Geometry* part = gi.next();
            for(Geometry::iterator i = part->begin(); i != part->end(); ++i) {
                i->x() += dx;
                i->y() += dy;
            }
        }
        footprint = dynamic_cast<Polygon*>(copy.get());
    }
#endif

    // Build the roof first, since we potentially need the roof skin in order
    // to generate roof texture coordinates.
    if ( getRoof() )
    {
        getRoof()->build( footprint, bc );
    }

    resolveSkin( bc );

    // prep for wall texture coordinate generation.
    float texWidthM  = _skinResource.valid() ? _skinResource->imageWidth().get()  : 0.0f;
    float texHeightM = _skinResource.valid() ? _skinResource->imageHeight().get() : 1.0f;
    
    bool hasTexture = true; // TODO

    // calcluate the bounds and the dominant rotation of the shape
    // based on the longest side.
    Bounds bounds = footprint->getBounds();

    float aabbWidth = _aabb.xMax() - _aabb.xMin();
    float aabbHeight = _aabb.yMax() - _aabb.yMin();

    // roof data:
    osg::Vec2f roofTexSpan;
    SkinResource* roofSkin = _roof.valid() ? _roof->getSkinResource() : 0L;
    if ( roofSkin )
    {
        if ( roofSkin->isTiled() == true )
        {
            roofTexSpan.x() = roofSkin->imageWidth().isSet() ? *roofSkin->imageWidth() : roofSkin->imageHeight().isSet() ? *roofSkin->imageHeight() : 10.0;
            if ( roofTexSpan.x() <= 0.0 )
                roofTexSpan.x() = 10.0;

            roofTexSpan.y() = roofSkin->imageHeight().isSet() ? *roofSkin->imageHeight() : roofSkin->imageWidth().isSet() ? *roofSkin->imageWidth() : 10.0;
            if ( roofTexSpan.y() <= 0.0 )
                roofTexSpan.y() = 10.0;
        }
        else
        {
            roofTexSpan.x() = aabbWidth;
            roofTexSpan.y() = aabbHeight;
        }
    }

    ConstGeometryIterator iter( footprint );
    while( iter.hasMore() )
    {
        const Geometry* part = iter.next();

        // skip a part that's too small (invalid)
        if (part->size() < 2)
            continue;

        // add a new wall.
        _walls.push_back( Wall() );
        Wall& wall = _walls.back();

        // Step 1 - Create the real corners and transform them into our target SRS.
        Corners corners;
        for(Geometry::const_iterator m = part->begin(); m != part->end(); ++m)
        {
            Corners::iterator corner = corners.insert(corners.end(), Corner());
            
            // mark as "from source", as opposed to being inserted by the algorithm.
            corner->isFromSource = true;
            corner->lower.set( m->x(), m->y(), getBottom() );

            // extrude:
            corner->upper.set( corner->lower.x(), corner->lower.y(), getTop() );

            // resolve UV coordinates based on dominant rotation:
            if ( roofSkin )
            {
                if ( roofSkin->isTiled() == true )
                {
                    float xr = corner->upper.x() - bounds.xMin();
                    float yr = corner->upper.y() - bounds.yMin();
                    rotate(xr, yr);
                    corner->roofUV.set( xr/roofTexSpan.x(), yr/roofTexSpan.y() );
                }
                else
                {
                    float xr = corner->upper.x(), yr = corner->upper.y();
                    rotate(xr, yr);
                    xr -= _aabb.xMin();
                    yr -= _aabb.yMin();
                    corner->roofUV.set( xr/aabbWidth, yr/aabbHeight );
                }
            }

            // cache the length for later use.
            corner->height = (corner->upper - corner->lower).length();
        }

        // Step 2 - Insert intermediate Corners as needed to satisfy texturing
        // requirements (if necessary) and record each corner offset (horizontal distance
        // from the beginning of the part geometry to the corner.)
        float cornerOffset    = 0.0;
        float nextTexBoundary = texWidthM;

        for(Corners::iterator c = corners.begin(); c != corners.end(); ++c)
        {
            Corners::iterator this_corner = c;

            Corners::iterator next_corner = c;
			bool isLastEdge = false;
			if ( ++next_corner == corners.end() )
			{
				isLastEdge = true;
				next_corner = corners.begin();
			}

            osg::Vec3f base_vec = next_corner->lower - this_corner->lower;
            float span = base_vec.length();

            this_corner->offsetX = cornerOffset;

            if ( hasTexture )
            {
                base_vec /= span; // normalize
                osg::Vec3f roof_vec = next_corner->upper - this_corner->upper;
                roof_vec.normalize();

                while(texWidthM > 0.0 && nextTexBoundary < cornerOffset+span)
                {
                    // insert a new fake corner.
					Corners::iterator new_corner;

                    if ( isLastEdge )
                    {
						corners.push_back(Corner());
						new_corner = c;
						new_corner++;
                    }
                    else
                    {
						new_corner = corners.insert(next_corner, Corner());
					}

                    new_corner->isFromSource = false;
                    float advance = nextTexBoundary-cornerOffset;
                    new_corner->lower = this_corner->lower + base_vec*advance;
                    new_corner->upper = this_corner->upper + roof_vec*advance;
                    new_corner->height = (new_corner->upper - new_corner->lower).length();
                    new_corner->offsetX = cornerOffset + advance;
                    nextTexBoundary += texWidthM;

                    // advance the main iterator
                    c = new_corner;
                }
            }

            cornerOffset += span;
        }

        // Step 3 - Calculate the angle of each corner.
        osg::Vec3f prev_vec;
        for(Corners::iterator c = corners.begin(); c != corners.end(); ++c)
        {
            Corners::const_iterator this_corner = c;

            Corners::const_iterator next_corner = c;
            if ( ++next_corner == corners.end() )
                next_corner = corners.begin();

            if ( this_corner == corners.begin() )
            {
                Corners::const_iterator prev_corner = corners.end();
                --prev_corner;
                prev_vec = this_corner->upper - prev_corner->upper;
                prev_vec.normalize();
            }

            osg::Vec3f this_vec = next_corner->upper - this_corner->upper;
            this_vec.normalize();
            if ( c != corners.begin() )
            {
                c->cosAngle = prev_vec * this_vec;
            }
        }

        // Step 4 - Create faces connecting each pair of corner posts.
        Faces& faces = wall.faces;
        for(Corners::const_iterator c = corners.begin(); c != corners.end(); ++c)
        {
            Corners::const_iterator this_corner = c;

            Corners::const_iterator next_corner = c;
            if ( ++next_corner == corners.end() )
                next_corner = corners.begin();

            faces.push_back(Face());
            Face& face = faces.back();
            face.left  = *this_corner;
            face.right = *next_corner;

            // recalculate the final offset on the last face
            if ( next_corner == corners.begin() )
            {
                osg::Vec3f vec = next_corner->upper - this_corner->upper;
                face.right.offsetX = face.left.offsetX + vec.length();
            }

            face.widthM = next_corner->offsetX - this_corner->offsetX;
        }
    }

    for(ElevationVector::iterator e = _elevations.begin(); e != _elevations.end(); ++e)
    {
        e->get()->build( footprint, bc );
    }

    return true;
}

void
Elevation::resolveSkin(BuildContext& bc)
{
    if ( getSkinSymbol() )
    {
        SkinResourceVector candidates;
        bc.getResourceLibrary()->getSkins( getSkinSymbol(), candidates, bc.getDBOptions() );                
        if ( !candidates.empty() )
        {
            unsigned index = Random(bc.getSeed()).next( candidates.size() );
            SkinResource* skin = candidates.at(index).get();
            setSkinResource( skin );
                    
            unsigned numFloors = (unsigned)std::max(1.0f, osg::round(getHeight() / skin->imageHeight().get()));
            setNumFloors( numFloors );
        }
    }
    else if ( getParent() )
    {
        SkinResource* skin = getParent()->getSkinResource();
        if ( skin )
        {
            setSkinResource( skin );                    
            unsigned numFloors = (unsigned)std::max(1.0f, osg::round(getHeight() / skin->imageHeight().get()));
            setNumFloors( numFloors );
        }
    }
}

void
Elevation::calculateRotations(const Polygon* footprint)
{
    if ( footprint )
    {
        // calculate the rotation data.
        // looks for the longest segment in the footprint and
        // returns the angle of that segment relative to north.
        Segment n;
        double  maxLen2 = 0.0;
        ConstSegmentIterator i( footprint, true );
        while( i.hasMore() )
        {
            Segment s = i.next();
            double len2 = (s.second - s.first).length2();
            if ( len2 > maxLen2 ) 
            {
                maxLen2 = len2;
                n = s;
            }
        }

        // swap coords if neceesary, so that p1 is always on the left.
        const osg::Vec3d& p1 = n.first.x() < n.second.x() ? n.first : n.second;
        const osg::Vec3d& p2 = n.first.x() < n.second.x() ? n.second : n.first;

        // compute a rotation that will transform the long segment to be
        // parallel to the Y axis.
        float r = atan2( p2.x()-p1.x(), p2.y()-p1.y() );
        _sinR = sinf( r );
        _cosR = cosf( r );

        // cache the midpoint of the longest segment, and the vector that
        // points towards the inside of the polygon.
        _longEdgeMidpoint = (p1+p2)*0.5;
        _longEdgeInsideNormal = (n.second-n.first)^osg::Vec3d(0,0,-1);
        _longEdgeInsideNormal.normalize();
        
        // compute the axis-aligned bbox
        _aabb.init();
        for(Polygon::const_iterator i = footprint->begin(); i != footprint->end(); ++i)
        {
            osg::Vec3f v(i->x(), i->y(), getTop());
            rotate( v );
            _aabb.expandBy( v );
        }
    }
}

float
Elevation::getUppermostZ() const
{
    if ( !_walls.empty() )
        if ( !_walls.front().faces.empty() )
            return _walls.front().faces.front().left.upper.z();

    return getTop();
}

Config
Elevation::getConfig() const
{
    Config conf;

    conf.set("inset", getInset());
    conf.set("height_percentage", _heightPercentage);
    conf.set("height", _height);
    
    if ( getRoof() )
        conf.set("roof", getRoof()->getConfig());

    if ( !getElevations().empty() )
    {
        Config evec("elevations");
        for(ElevationVector::const_iterator sub = getElevations().begin(); sub != getElevations().end(); ++sub)
            evec.add("elevation", sub->get()->getConfig());
        conf.set(evec);
    }
    return conf;
}
