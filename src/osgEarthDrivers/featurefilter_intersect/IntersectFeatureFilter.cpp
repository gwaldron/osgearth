/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2014 Pelican Mapping
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
#include "IntersectFeatureFilterOptions"

#include <osgEarthFeatures/Filter>

#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgEarth/Registry>
#include <osgEarth/ImageUtils>
#include <osgEarthFeatures/FeatureSource>
#include <osgEarthSymbology/Geometry>

#define LC "[Intersect FeatureFilter] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Drivers;
using namespace osgEarth::Symbology;



class IntersectFeatureFilter : public FeatureFilter, public IntersectFeatureFilterOptions
{
private:
    Threading::Mutex _transformMutex;
    bool _transformed;

    osg::ref_ptr<const SpatialReference> _boundarySRS;
    std::vector< osg::ref_ptr<Ring> >    _boundaries;
    std::vector< Bounds >                _bboxes;
    Bounds                               _overallbbox;

public:
    IntersectFeatureFilter(const ConfigOptions& options)
        : FeatureFilter(), IntersectFeatureFilterOptions(options)
    {
        _transformed = false;
    }

    void transform(const SpatialReference* srs)
    {
        if ( !_boundarySRS || !srs ) return;

        for(unsigned i=0; i<_boundaries.size(); ++i)
        {
            Ring& ring = *_boundaries[i].get();
            _boundarySRS->transform( ring.asVector(), srs );
            _bboxes.push_back( ring.getBounds() );
            _overallbbox.expandBy( _bboxes.back() );
        }
    }

public: // FeatureFilter

    void initialize(const osgDB::Options* dbo)
    {
        // Load the feature source containing the intersection geometry.
        osg::ref_ptr<FeatureSource> fs = FeatureSourceFactory::create( features().get() );
        if ( !fs.valid() )
        {
            OE_WARN << LC << "Failed to load the intersect feature source.\n";
            return;
        }

        fs->initialize( dbo );

        if ( !fs->getFeatureProfile() )
        {
            OE_WARN << LC << "Failed to establish the feature profile.\n";
            return;
        }

        _boundarySRS = fs->getFeatureProfile()->getSRS();

        // Read in the testing features:
        osg::ref_ptr<FeatureCursor> cursor = fs->createFeatureCursor();
        if ( cursor.valid() )
        {
            while( cursor->hasMore() )
            {
                Feature* next = cursor->nextFeature();
                if ( next && next->getGeometry() )
                {
                    Ring* ring = dynamic_cast<Ring*>(next->getGeometry());
                    if ( ring )
                        _boundaries.push_back( ring );
                }
            }
        }

        OE_INFO << LC << this << " : " "Read " << _boundaries.size() << " boundary geometries.\n";
    }

    FilterContext push(FeatureList& input, FilterContext& context)
    {
        if ( !_boundaries.empty() )
        {
            // First time, transform the bounary set into the FeatureProfile SRS.
            if ( !_transformed )
            {
                Threading::ScopedMutexLock lock(_transformMutex);
                if ( !_transformed )
                {
                    transform(context.profile()->getSRS());
                    _transformed = true;
                }
            }

            FeatureList output;
            for(FeatureList::const_iterator f = input.begin(); f != input.end(); ++f)
            {
                Feature* feature = f->get();
                if ( feature && feature->getGeometry() )
                {
                    osg::Vec2d c = feature->getGeometry()->getBounds().center2d();

                    if ( contains() == true )
                    {
                        // coarsest:
                        if ( _overallbbox.contains(c.x(), c.y()) )
                        {
                            for(unsigned b=0; b<_boundaries.size(); ++b)
                            {
                                // finer:
                                if ( _bboxes[b].contains(c.x(), c.y()) )
                                {
                                    // finest:
                                    if ( exact()==false || _boundaries[b]->contains2D(c.x(), c.y()) )
                                    {
                                        output.push_back( feature );
                                        break;
                                    }
                                }
                            }
                        }
                    }

                    else
                    {    
                        bool contained = false;

                        // coarsest:
                        if ( _overallbbox.contains(c.x(), c.y()) )
                        {
                            // finer:
                            for(unsigned b=0; b<_boundaries.size() && !contained; ++b)
                            {
                                if ( _bboxes[b].contains(c.x(), c.y()) )
                                {
                                    // finest:
                                    if ( exact()==false || _boundaries[b]->contains2D(c.x(), c.y()) )
                                    {
                                        contained = true;
                                        break;
                                    }
                                }
                            }
                        }
                        if ( !contained )
                        {
                            output.push_back( feature );
                        }
                    }
                }
            }

            OE_DEBUG << LC << "Allowed " << output.size() << " out of " << input.size() << " features\n";
        
            input = output;
        }
        else
        {
            OE_INFO << LC << this << " : ""No boundaries; all pass\n";
        }

        return context;
    }
};


class IntersectFeatureFilterPlugin : public FeatureFilterDriver
{
public:
    IntersectFeatureFilterPlugin() : FeatureFilterDriver()
    {
        this->supportsExtension("osgearth_featurefilter_intersect", className() );
    }
    
    const char* className()
    {
        return "IntersectFeatureFilterPlugin";
    }

    ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        return new IntersectFeatureFilter( getConfigOptions(options) );
    }
};

REGISTER_OSGPLUGIN(osgearth_featurefilter_intersect, IntersectFeatureFilterPlugin);

//OSGEARTH_REGISTER_SIMPLE_FEATUREFILTER(intersect, IntersectFeatureFilter);

