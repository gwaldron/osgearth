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
#include "Analyzer"

#include <osgEarth/Progress>
#include <osgEarth/TileKey>
#include <osgEarth/ImageUtils>
#include <osg/BlendFunc>
#include <osg/BlendColor>
#include <osgDB/Registry>
#include <osgDB/WriteFile>
#include <osgUtil/Statistics>
#include <iostream>
#include <set>

#define LC "[Analyzer] "

using namespace osgEarth;
using namespace osgEarth::Buildings;

namespace
{
    struct FindTextures : public osgEarth::TextureAndImageVisitor
    {
        std::set<osg::Texture*> _textures;

        void apply(osg::Texture& texture) {
            if (_textures.find(&texture) == _textures.end()) {
                _textures.insert(&texture);
            }
        }

        void print(std::ostream& out) {
            out << "Textures (" << _textures.size() << ") : \n";
            for (std::set<osg::Texture*>::const_iterator i = _textures.begin(); i != _textures.end(); ++i) {
                out << "    " << std::hex << (uintptr_t)(*i) << " : " << std::dec << (*i)->getImage(0)->getFileName() << "\n";
            }
        }
    };

    struct CompareStateSets {
        bool operator()(const osg::ref_ptr<osg::StateSet>& lhs, const osg::ref_ptr<osg::StateSet>& rhs) const {
            return lhs->compare(*rhs.get(), true) < 0;
        }
    };
    typedef std::set< osg::ref_ptr<osg::StateSet>, CompareStateSets> StateSetSet;

    void printStateSet(osg::StateSet* ss, std::ostream& os)
    {
        os << "  # attrs = " << ss->getAttributeList().size() << "\n";

        const osg::StateSet::AttributeList& alist = ss->getAttributeList();
        for (osg::StateSet::AttributeList::const_iterator i = alist.begin(); i != alist.end(); ++i) {
            os << "     " << std::hex << (uintptr_t)i->second.first.get() << std::dec << " : " << i->second.first.get()->className() << "\n";
        }

        os << "  # texattrs = \n";

        const osg::StateSet::TextureAttributeList& talist = ss->getTextureAttributeList();
        for (osg::StateSet::TextureAttributeList::const_iterator i = talist.begin(); i != talist.end(); ++i) {
            for (osg::StateSet::AttributeList::const_iterator j = i->begin(); j != i->end(); ++j) {
                os << "     " << std::hex << (uintptr_t)j->second.first.get() << std::dec << " : " << j->second.first.get()->className() << "\n";
            }
        }
    }

    struct FindStateSets : public osg::NodeVisitor
    {
        FindStateSets() {
            setTraversalMode(TRAVERSE_ALL_CHILDREN);
            setNodeMaskOverride(~0);
        }
        void apply(osg::Node& node) {
            apply(node.getStateSet());
            traverse(node);
        }
        void apply(osg::Geode& geode) {
            apply(geode.getStateSet());
            for (unsigned i = 0; i < geode.getNumDrawables(); ++i) {
                apply(geode.getDrawable(i)->getStateSet());
            }
        }
        void apply(osg::StateSet* ss) {
            if (ss) {
                _statesets.insert(ss);
            }
        }

        StateSetSet _statesets;

        void print(std::ostream& out) {
            out << "Statesets: (" << _statesets.size() << ")\n";
            osgDB::ReaderWriter* rw = osgDB::Registry::instance()->getReaderWriterForExtension("osgt");
            for (StateSetSet::iterator i = _statesets.begin(); i != _statesets.end(); ++i) {
                osg::StateSet* ss = i->get();
                out << std::hex << (uintptr_t)ss << std::dec << std::endl;
                printStateSet(ss, out);
            }
        }
    };

    struct ExStatsVisitor : public osgUtil::StatsVisitor
    {
        ExStatsVisitor() : osgUtil::StatsVisitor() {
            setTraversalMode(TRAVERSE_ALL_CHILDREN);
            setNodeMaskOverride(~0);
        }
        void apply(osg::Drawable& d) {
            osgUtil::StatsVisitor::apply(d);
            osg::Geometry* g = d.asGeometry();
            if (g && g->getVertexArray()) {
                osg::Object* vbo = g->getVertexArray()->getVertexBufferObject();
                _vbos.insert(vbo);                
            }
        }
        std::set<osg::Object*> _vbos;
    };
}

void
Analyzer::analyze(osg::Node* node, ProgressCallback* progress, unsigned numFeatures, double totalTime, const TileKey& tileKey)
{
    if (!node) return;
    if (!progress) return;

    static Threading::Mutex s_analyzeMutex;
    Threading::ScopedMutexLock lock(s_analyzeMutex);
    
    std::cout
        << "...............................................................................\n"
        << "Key = " << tileKey.str()
        << " : Features = " << numFeatures
        << ", Time = " << (int)(1000.0*totalTime)
        << " ms, Avg = " << std::setprecision(3) << (1000.0*(totalTime / (double)numFeatures)) << " ms"
        << std::endl;

    // collect statistics about the resulting scene graph:
    if (progress->collectStats())
    {
        ExStatsVisitor stats;
        node->accept(stats);
        progress->stats("# unique stateSets") = stats._statesetSet.size();
        progress->stats("# stateSet refs") = stats._numInstancedStateSet;
        progress->stats("# drawables") = stats._drawableSet.size();

        // Prints out the stats details
        stats.print(std::cout);

        // Uncomment to see the number of primitivesets (i.e. number of calls to DrawElements) in each drawable
        unsigned primsets = 0;
        for (osgUtil::StatsVisitor::DrawableSet::const_iterator i = stats._drawableSet.begin(); i != stats._drawableSet.end(); ++i) {
            primsets += (*i)->asGeometry()->getNumPrimitiveSets();
        }
        progress->stats("# primsets") = primsets;
        progress->stats("# vbos") = stats._vbos.size();

        FindTextures ft;
        node->accept(ft);
        ft.print(std::cout);

        FindStateSets fss;
        node->accept(fss);
        fss.print(std::cout);

        // Uncomment to dump out a file per tile.
        //osgDB::writeNodeFile(*node, "out.osgt");
    }

    std::stringstream buf;
    buf << "Stats:\n";

    for (ProgressCallback::Stats::const_iterator i = progress->stats().begin(); i != progress->stats().end(); ++i)
    {
        if (i->first.at(0) == '#')
        {
            buf
                << "    "
                << std::setw(15) << i->first
                << std::setw(10) << i->second
                << std::endl;
        }
        else
        {
            buf
                << "    "
                << std::setw(15) << i->first
                << std::setw(6) << (int)(1000.0*i->second) << " ms"
                << std::setw(6) << (int)(100.0*i->second / totalTime) << "%"
                << std::endl;
        }
    }

    std::cout << buf.str() << std::endl;

    // clear them when we are done.
    progress->stats().clear();

}