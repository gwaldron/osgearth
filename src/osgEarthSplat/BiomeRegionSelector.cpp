/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2015 Pelican Mapping
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
#include "BiomeRegionSelector"

#include <osgEarth/CullingUtils>
#include <osgEarth/VirtualProgram>
#include <osgEarth/GeoData>

using namespace osgEarth;
using namespace osgEarth::Splat;

#define LC "[BiomeSelector] "

BiomeRegionSelector::BiomeRegionSelector(const BiomeRegionVector&     biomeRegions,
                                         const SplatTextureDefVector& textureDefs,
                                         osg::StateSet*               stateSet,
                                         int                          textureImageUnit) :
_biomeRegions( biomeRegions )
{
    for(unsigned i=0; i<_biomeRegions.size(); ++i)
    {
        BiomeRegion& biomeRegion = _biomeRegions[i];

        // pre-calculate optimized values for each biome region.
        std::vector<BiomeRegion::Region>& regions = biomeRegion.getRegions();
        for(unsigned r=0; r<regions.size(); ++r)
        {
            BiomeRegion::Region& region = regions[r];
            region.extent.createPolytope( region.tope );
            region.zmin2 = region.zmin > -DBL_MAX ? region.zmin*region.zmin : region.zmin;
            region.zmax2 = region.zmax < DBL_MAX ? region.zmax*region.zmax : region.zmax;

            // this only needs to be very approximate.
            region.meanRadius2 = region.extent.getSRS()->isGeographic() ?
                region.extent.getSRS()->getEllipsoid()->getRadiusEquator() : 0.0;
            region.meanRadius2 *= region.meanRadius2;
        }

        // next, set up a stateset with the approprate texture and program
        // for this biome.
        const SplatTextureDef& textureDef = textureDefs[i];

        // shallow-copy the stateset in prepration for customization:
        osg::StateSet* biomeSS = 
            i == 0 ? stateSet :
            osg::clone(stateSet, osg::CopyOp::SHALLOW_COPY);

        // install his biome's texture set:
        biomeSS->setTextureAttribute(textureImageUnit, textureDef._texture.get());

        // install this biome's sampling function. Use cloneOrCreate since each
        // stateset needs a different shader set in its VP.
        VirtualProgram* vp = VirtualProgram::cloneOrCreate( biomeSS );
        osg::Shader* shader = new osg::Shader(osg::Shader::FRAGMENT, textureDef._samplingFunction);
        vp->setShader( "oe_splat_getRenderInfo", shader );

        // put it on the list for fast access in the cull traversal.
        _stateSets.push_back( biomeSS );
    }
}

void
BiomeRegionSelector::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
    osgUtil::CullVisitor* cv = Culling::asCullVisitor(nv);

    osg::Vec3d vp = cv->getViewPoint();
    double z2 = vp.length2();
    
    osg::StateSet* stateSet = 0L;

    for(unsigned b=0; b<_biomeRegions.size() && stateSet == 0L; ++b)
    {
        const BiomeRegion& biomeRegion = _biomeRegions[b];        
        bool match = false;
            
        if ( biomeRegion.getRegions().size() == 0 )
        {
            // empty biome is a match.
            //OE_INFO << "matched " <<biomeRegion.name().get() << " b/c biome has no regions\n";
            match = true;
        }
        else
        {
            // check each region of the biome:
            for(unsigned r=0; r<biomeRegion.getRegions().size() && !match; ++r)
            {
                const BiomeRegion::Region& region = biomeRegion.getRegions()[r];

                // empty extent/tope is a match:
                if ( region.tope.empty() )
                {
                    //OE_INFO << "matched " << biomeRegion.name().get() << " b/c tope is empty\n";
                    match = true;
                }

                // otherwise, check for intersection:
                else if ( region.tope.contains(vp) )
                {
                    double hat2 = z2 - region.meanRadius2;
                    if ( hat2 >= region.zmin2 && hat2 <= region.zmax2 )
                    {
                        //OE_INFO << "matched " << biomeRegion.name().get() << " b/c eyepoint intersects\n";
                        match = true;
                    }
                }
            }
        }

        if ( match )
        {
            stateSet = _stateSets[b].get();
        }
    }

    if ( stateSet )
    {
        cv->pushStateSet( stateSet );
    }

    traverse(node, nv);

    if ( stateSet )
    {
        cv->popStateSet();
    }
}
