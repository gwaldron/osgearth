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
#include <osgEarthFeatures/LabelSource>
#include <osgEarthAnnotation/LabelNode>
#include <osgEarthAnnotation/Decluttering>
#include <osgDB/FileNameUtils>
#include <osgUtil/Optimizer>

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Features;

class AnnotationLabelSource : public LabelSource
{
public:
    AnnotationLabelSource( const LabelSourceOptions& options )
        : LabelSource( options )
    {
        //nop
    }

    /**
     * Creates a simple label. The caller is responsible for placing it in the scene.
     */
    osg::Node* createNode(
        const std::string& text,
        const TextSymbol*  symbol )
    {
        return 0L; // no support
    }

    /**
     * Creates a complete set of positioned label nodes from a feature list.
     */
    osg::Node* createNode(
        const FeatureList&   input,
        const TextSymbol*    symbol,
        const FilterContext& context )
    {
        osg::Group* group = new osg::Group();
        Decluttering::setEnabled( group->getOrCreateStateSet(), true );

        std::set<std::string> used; // to prevent dupes
        bool skipDupes = (symbol->removeDuplicateLabels() == true);

        StringExpression  contentExpr ( *symbol->content() );
        NumericExpression priorityExpr( *symbol->priority() );

        for( FeatureList::const_iterator i = input.begin(); i != input.end(); ++i )
        {
            const Feature* feature = i->get();
            if ( !feature )
                continue;

            const Geometry* geom = feature->getGeometry();
            if ( !geom )
                continue;

            const std::string& value = feature->eval( contentExpr );

            if ( !value.empty() && (!skipDupes || used.find(value) == used.end()) )
            {
                LabelNode* labelNode = new LabelNode(
                    context.getSession()->getMapInfo().getProfile()->getSRS(),
                    geom->getBounds().center(),
                    value,
                    symbol );

                if ( symbol->priority().isSet() )
                {
                    AnnotationData* data = new AnnotationData();
                    data->setPriority( feature->eval(priorityExpr) );
                    labelNode->setAnnotationData( data );
                }
                
                group->addChild( labelNode );

                if ( skipDupes )
                {
                    used.insert(value);
                }
            }
        }

        return group;
    }
};

//------------------------------------------------------------------------

class AnnotationLabelSourceDriver : public LabelSourceDriver
{
public:
    AnnotationLabelSourceDriver()
    {
        supportsExtension( "osgearth_label_annotation", "osgEarth annotation label plugin" );
    }

    virtual const char* className()
    {
        return "osgEarth Annotation Label Plugin";
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        return new AnnotationLabelSource( getLabelSourceOptions(options) );
    }
};

REGISTER_OSGPLUGIN(osgearth_label_annotation, AnnotationLabelSourceDriver)