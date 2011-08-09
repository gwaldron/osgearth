/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2010 Pelican Mapping
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

#include <osgEarthFeatures/OgrUtils>
#include <osgEarthFeatures/GeometryUtils>
#include <osgEarthFeatures/FeatureSourceNode>

using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

std::string osgEarth::Features::geometryToWkt( Geometry* geometry )
{
    OGRGeometryH g = OgrUtils::createOgrGeometry( geometry );
    std::string result;
    if (g)
    {
        char* buf;   
        if (OGR_G_ExportToWkt( g, &buf ) == OGRERR_NONE)
        {
            result = std::string(buf);
            OGRFree( buf );
        }
        OGR_G_DestroyGeometry( g );
    }
    return result;
}

std::string osgEarth::Features::geometryToJson( Geometry* geometry )
{
    OGRGeometryH g = OgrUtils::createOgrGeometry( geometry );
    std::string result;
    if (g)
    {
        char* buf;   
        buf = OGR_G_ExportToJson( g );
        if (buf)
        {
            result = std::string(buf);
            OGRFree( buf );
        }
        OGR_G_DestroyGeometry( g );
    }
    return result;
}

std::string osgEarth::Features::geometryToKml( Geometry* geometry )
{
    OGRGeometryH g = OgrUtils::createOgrGeometry( geometry );
    std::string result;
    if (g)
    {
        char* buf;   
        buf = OGR_G_ExportToKML( g, 0);
        if (buf)
        {
            result = std::string(buf);
            OGRFree( buf );
        }
        OGR_G_DestroyGeometry( g );
    }
    return result;
}

std::string osgEarth::Features::geometryToGml( Geometry* geometry )
{
    OGRGeometryH g = OgrUtils::createOgrGeometry( geometry );
    std::string result;
    if (g)
    {
        char* buf;   
        buf = OGR_G_ExportToGML( g );
        if (buf)
        {
            result = std::string(buf);
            OGRFree( buf );
        }
        OGR_G_DestroyGeometry( g );
    }
    return result;
}

void
FeatureSourceMeshConsolidator::run( osg::Geode& geode, FeatureSourceMultiNode * featureNode )
{
	unsigned numVerts = 0;
	unsigned numColors = 0;
	unsigned numNormals = 0;
	unsigned numTexCoordArrays = 0;
	unsigned numVertAttribArrays = 0;

	osg::Geometry::AttributeBinding newColorsBinding;
	osg::Geometry::AttributeBinding newNormalsBinding;

	// first, triangulate all the geometries and count all the components:
	for( unsigned i=0; i<geode.getNumDrawables(); ++i )
	{
		osg::Geometry* geom = geode.getDrawable(i)->asGeometry();
		if ( geom )
		{
			// optimize it into triangles first:
			MeshConsolidator::run( *geom );

			osg::Array* verts = geom->getVertexArray();
			if ( verts )
				numVerts += verts->getNumElements();

			osg::Array* colors = geom->getColorArray();
			if ( colors )
				numColors += colors->getNumElements();

			osg::Array* normals = geom->getNormalArray();
			if ( normals )
				numNormals += normals->getNumElements();

			numTexCoordArrays += geom->getNumTexCoordArrays();
			numVertAttribArrays += geom->getNumVertexAttribArrays();
		}
	}

	// bail if there are unsupported items in there.
	if (geode.getNumDrawables() < 2 ||
		numTexCoordArrays       > 0 ||
		numVertAttribArrays     > 0 )
	{
		return;
	}


	osg::Vec3Array* newVerts = new osg::Vec3Array();
	newVerts->reserve( numVerts );

	osg::Vec4Array* newColors =0L;
	if ( numColors > 0 )
	{
		newColors = new osg::Vec4Array();
		newColors->reserve( numColors==numVerts? numColors : 1 );
		newColorsBinding = numColors==numVerts? osg::Geometry::BIND_PER_VERTEX : osg::Geometry::BIND_OVERALL;
	}
	else
		newColorsBinding = osg::Geometry::BIND_OVERALL;

	osg::Vec3Array* newNormals =0L;
	if ( numNormals > 0 )
	{
		newNormals = new osg::Vec3Array();
		newNormals->reserve( numNormals==numVerts? numNormals : 1 );
		newNormalsBinding = numNormals==numVerts? osg::Geometry::BIND_PER_VERTEX : osg::Geometry::BIND_OVERALL;
	}
	else
		newNormalsBinding = osg::Geometry::BIND_OVERALL;

	unsigned offset = 0;
	osg::Geometry::PrimitiveSetList newPrimSets;

	for( unsigned i=0; i<geode.getNumDrawables(); ++i )
	{
		osg::Geometry* geom = geode.getDrawable(i)->asGeometry();
		if ( geom )
		{
			FeatureID fid;
			if(featureNode)
			{
				fid = featureNode->getFID(geom);
				featureNode->removeDrawable(geom);
			}
			else
				fid = FeatureID(-1);
			// copy over the verts:
			osg::Vec3Array* geomVerts = dynamic_cast<osg::Vec3Array*>( geom->getVertexArray() );
			if ( geomVerts )
			{
				std::copy( geomVerts->begin(), geomVerts->end(), std::back_inserter(*newVerts) );

				if ( newColors )
				{
					osg::Vec4Array* colors = dynamic_cast<osg::Vec4Array*>( geom->getColorArray() );
					if ( colors )
					{
						if ( newColorsBinding == osg::Geometry::BIND_PER_VERTEX )
						{
							std::copy( colors->begin(), colors->end(), std::back_inserter(*newColors) );
						}
						else if ( i == 0 ) // overall
						{
							newColors->push_back( (*colors)[0] );
						}
					}
				}

				if ( newNormals )
				{
					osg::Vec3Array* normals = dynamic_cast<osg::Vec3Array*>( geom->getNormalArray() );
					if ( normals )
					{
						if ( newNormalsBinding == osg::Geometry::BIND_PER_VERTEX )
						{
							std::copy( normals->begin(), normals->end(), std::back_inserter(*newNormals) );
						}
						else if ( i == 0 ) // overall
						{
							newNormals->push_back( (*normals)[0] );
						}
					}
				}

				for( unsigned j=0; j < geom->getNumPrimitiveSets(); ++j )
				{
					osg::PrimitiveSet* pset = geom->getPrimitiveSet(j);
					osg::PrimitiveSet* newpset = 0L;

					if ( dynamic_cast<osg::DrawElementsUByte*>(pset) )
						newpset = remake( static_cast<osg::DrawElementsUByte*>(pset), numVerts, offset );
					else if ( dynamic_cast<osg::DrawElementsUShort*>(pset) )
						newpset = remake( static_cast<osg::DrawElementsUShort*>(pset), numVerts, offset );
					else if ( dynamic_cast<osg::DrawElementsUInt*>(pset) )
						newpset = remake( static_cast<osg::DrawElementsUInt*>(pset), numVerts, offset );
					else if ( dynamic_cast<osg::DrawArrays*>(pset) )
						newpset = new osg::DrawArrays( pset->getMode(), offset, geomVerts->size() );

					if ( newpset )
					{
						newPrimSets.push_back( newpset );
						if(featureNode)
							featureNode->addPrimitiveSet(newpset, fid);
					}
				}

				offset += geomVerts->size();
			}
		}
	}

	// assemble the new geometry.
	osg::Geometry* newGeom = new osg::Geometry();

	newGeom->setVertexArray( newVerts );

	if ( newColors )
	{
		newGeom->setColorArray( newColors );
		newGeom->setColorBinding( newColorsBinding );
	}

	if ( newNormals )
	{
		newGeom->setNormalArray( newNormals );
		newGeom->setNormalBinding( newNormalsBinding );
	}

	newGeom->setPrimitiveSetList( newPrimSets );

	// replace the geode's drawables
	geode.removeDrawables( 0, geode.getNumDrawables() );
	geode.addDrawable( newGeom );
}
