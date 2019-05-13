/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2018 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/
#include <osgEarth/Callouts>
#include <osgEarth/LineDrawable>
#include <osg/Depth>
#include <osgUtil/CullVisitor>

using namespace osgEarth;
using namespace osgEarth::Contrib;

//...................................................................

Callout::Callout(CalloutManager* cm) :
osgEarth::Text(),
_cm(cm)
{
    setDataVariance(osg::Object::DYNAMIC);
    setEnableDepthWrites(false);
}

void
Callout::accept(osg::NodeVisitor& nv)
{
    if (nv.validNodeMask(*this))
    {
        nv.pushOntoNodePath(this);
        if (nv.getVisitorType() == nv.CULL_VISITOR)
        {
            osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(&nv);
            _cm->push(this, *cv);
        }
        else
        {
            nv.apply(*this);
        }
        nv.popFromNodePath();
    }
}

//...................................................................

CalloutManager::CalloutManager() :
osg::Drawable()
{
    setCullingActive(false);
    setDataVariance(osg::Object::DYNAMIC);

    getOrCreateStateSet()->setRenderBinDetails(90210, "DepthSortedBin");
    // Doesn't work...why?
    //getOrCreateStateSet()->setMode(GL_DEPTH_TEST, 0);
    // DOesn't work...why?
    getOrCreateStateSet()->setAttributeAndModes(
        new osg::Depth(osg::Depth::ALWAYS, 0, 1, false),
        osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);

    // Leader lines.
    _leaders = new LineDrawable(GL_LINES);
    _leaders->setCullingActive(false);
    _leaders->setDataVariance(osg::Object::DYNAMIC);
    _leaders->setColor(osg::Vec4f(1, 1, 0, 1));
    _leaders->setLineWidth(1.0f);
    _leadersDirty = false;
    _leaderLen = 40;

    _walker = _callouts.end();

    this->addCullCallback(new SortCallback(this));

    // amount labels are allowed to overlap [0..1]
    _maxOverlap = 0.0;
}


// cull traversal calls this to render a Callout
void
CalloutManager::push(Callout* node, osgUtil::CullVisitor& nv)
{
    static CalloutRecord prototype;
    std::pair<Callouts::iterator, bool> result = _callouts.insert(CalloutTuple(node->getUID(), prototype));
    CalloutRecord& rec = result.first->second;
    if (result.second) // new insertion
    {
        rec._node = node;
        rec.realign();
    }
    rec._textBB = node->getBoundingBox();
    rec._matrix = nv.getModelViewMatrix();
    rec._frame = nv.getFrameStamp()->getFrameNumber();

    const osg::Viewport* vp = nv.getCurrentCamera()->getViewport();

    // transform the label position into viewport space:
    osg::Vec3d anchor =
        osg::Vec3d(0, 0, 0) *
        (*nv.getModelViewMatrix()) *
        (*nv.getProjectionMatrix()) *
        osg::Matrix::translate(1, 1, 1) *
        osg::Matrix::scale(0.5*vp->width(), 0.5*vp->height(), 0.5);

    if (rec._leaderLineIndex == INT_MAX)
    {
        rec._leaderLineIndex = _leaders->getNumVerts();
        _leaders->pushVertex(osg::Vec3f());
        _leaders->pushVertex(osg::Vec3f());
        _leadersDirty = true;
    }

    osg::Vec3d labelpos = anchor + rec._offsetVector*_leaderLen;
    rec._vpBB.LL.set(labelpos.x() + rec._textBB.LL.x(), labelpos.y() + rec._textBB.LL.y());
    rec._vpBB.UR.set(labelpos.x() + rec._textBB.UR.x(), labelpos.y() + rec._textBB.UR.y());

    rec._matrix->makeTranslate(labelpos);

    if (anchor != _leaders->getVertex(rec._leaderLineIndex))
    {
        _leaders->setVertex(rec._leaderLineIndex, anchor);
    }

    if (labelpos != _leaders->getVertex(rec._leaderLineIndex + 1))
    {
        _leaders->setVertex(rec._leaderLineIndex + 1, labelpos);
    }

    rec._leaderBB.LL.set(osg::minimum(anchor.x(), labelpos.x()), osg::minimum(anchor.y(), labelpos.y()));
    rec._leaderBB.UR.set(osg::maximum(anchor.x(), labelpos.x()), osg::maximum(anchor.y(), labelpos.y()));

    if (!rec._conflicted)
    {
        _leaders->setColor(rec._leaderLineIndex, osg::Vec4f(1, 1, 1, 0.2));
        _leaders->setColor(rec._leaderLineIndex + 1, osg::Vec4f(1, 1, 1, 0.8));
    }
}

void
CalloutManager::sort(osg::NodeVisitor& nv)
{
    if (_callouts.empty())
        return;

#ifdef USE_RTREE
    _index.RemoveAll();

    float a_min[2], a_max[2];
    float b_min[2], b_max[2];

    std::vector<CalloutRecord*> hits;

    int frame = nv.getFrameStamp()->getFrameNumber();

    for (Callouts::reverse_iterator i = _callouts.rbegin();
        i != _callouts.rend();
        ++i)
    {
        hits.clear();

        CalloutRecord& rec = i->second;

        if (frame - rec._frame <= 1)
        {
            a_min[0] = rec._vpBB.LL.x(), a_min[1] = rec._vpBB.LL.y();
            a_max[0] = rec._vpBB.UR.x(), a_max[1] = rec._vpBB.UR.y();

            b_min[0] = rec._leaderBB.LL.x(), b_min[1] = rec._leaderBB.LL.y();
            b_max[0] = rec._leaderBB.UR.x(), b_max[1] = rec._leaderBB.UR.y();

            // does the label conflict with anything?
            if (_index.Search(a_min, a_max, &hits, 1) > 0)
            {
                CalloutRecord* rhs = (*hits.begin());
                if ((rhs == NULL) || (rec._vpBB.overlap(rhs->_vpBB) > _maxOverlap))
                {
                    rec._conflicted = true;
                }
            }
            else
            {
                // does the leader line conflict with anything?
                if (_index.Search(b_min, b_max, NULL, 1) > 0)
                {
                    rec._conflicted = true;
                }
                else
                {
                    rec._conflicted = false;
                }
            }

            if (rec._conflicted == false)
            {
                _index.Insert(a_min, a_max, &rec);
                _index.Insert(b_min, b_max, NULL);
            }
        }
    }

#else
    _index.reserve(_callouts.size());
    _index.clear();

    for (Callouts::reverse_iterator i = _callouts.rbegin();
        i != _callouts.rend();
        ++i)
    {
        CalloutRecord& dec = i->second;
        if (nv.getFrameStamp()->getFrameNumber() - dec._frame <= 1)
        {
            for (SpatialIndex::iterator s = _index.begin(); s != _index.end(); ++s)
            {
                dec._conflicted = dec._vpBB.overlaps(*s);
                if (dec._conflicted)
                    break;
            }

            if (!dec._conflicted)
            {
                _index.push_back(dec._vpBB);
                _index.push_back(dec._leaderBB);
            }
        }
    }
#endif

    // Next, declutter incrementally

    if (_walker == _callouts.end())
        _walker = _callouts.begin();

    const int triesPerFrame = 10;
    for (int tries = 0; tries < triesPerFrame && _walker != _callouts.end(); )
    {
        CalloutRecord& rec = _walker->second;

        if (frame - rec._frame <= 2)
        {
            if (rec._conflicted == true)
            {
                rec.move(1);
                ++tries;
            }
            ++_walker;
        }

        else if (frame - rec._frame > 5 * 60 * 60) // expire after 5 minutes
        {
            Callouts::iterator temp = _walker;
            ++_walker;
            _callouts.erase(temp);
        }

        else
        {
            ++_walker;
        }
    }
}

void
CalloutManager::drawImplementation(osg::RenderInfo& ri) const
{
    const osg::Viewport* vp = ri.getCurrentCamera()->getViewport();
    osg::Matrix ortho;
    ortho.makeOrtho(vp->x(), vp->x() + vp->width() - 1, vp->y(), vp->y() + vp->height() - 1, -1000, 1000);
    osg::ref_ptr<osg::RefMatrix> proj = new osg::RefMatrix(ortho);
    ri.getState()->applyProjectionMatrix(proj);

#ifdef USE_RTREE
    bool appliedFirstState = false;

#if 0
    // render all:
    for (Callouts::const_iterator i = _Callouts.begin(); i != _Callouts.end(); ++i)
    {
        const Callout* dec = &i->second;
        if (dec->_frame < ri.getState()->getFrameStamp()->getFrameNumber())
            continue;
#else

    // render no-conflict
    SpatialIndex::Iterator i;
    for (_index.GetFirst(i); i.IsNotNull(); _index.GetNext(i))
    {
        const CalloutRecord* rec = (*i);
#endif
        if (rec)
        {
            if (!appliedFirstState)
            {
                ri.getState()->apply(rec->_node->getStateSet());
                glDepthFunc(GL_ALWAYS);
                glDepthMask(GL_FALSE);
                appliedFirstState = true;
            }
            ri.getState()->applyModelViewMatrix(rec->_matrix);
            rec->_node->draw(ri);
        }
    }

#else // non-Rtree-version

    // draw the labels
    unsigned numDrawn = 0L;
    for (Callouts::const_iterator i = _callouts.begin();
        i != _callouts.end();
        ++i)
    {
        const CalloutRecord& d = i->second;
        if (ri.getState()->getFrameStamp()->getFrameNumber() == d._frame &&
            !d._conflicted)
        {
            ri.getState()->apply(d._node->getStateSet());
            glDepthFunc(GL_ALWAYS);
            glDepthMask(GL_FALSE);
            ri.getState()->applyModelViewMatrix(d._matrix);
            d._node->draw(ri);
            numDrawn++;
        }
    }
#endif

    // the leader lines
    if (_leadersDirty)
    {
        _leaders->dirty();
        _leadersDirty = false;
    }

    ri.getState()->applyModelViewMatrix(osg::Matrix::identity());

    ri.getState()->apply(_leaders->getStateSet());
    ri.getState()->apply(_leaders->getGPUStateSet());
    glDepthFunc(GL_ALWAYS);
    glDepthMask(GL_FALSE);
    glEnable(GL_BLEND);

    if (ri.getState()->getUseModelViewAndProjectionUniforms())
        ri.getState()->applyModelViewAndProjectionUniformsIfRequired();

    _leaders->draw(ri);

    // reset for visibility control
    _leaders->setColor(osg::Vec4f(0, 0, 0, 0));
}


CalloutManager::BBox::BBox(const osg::BoundingBox& bbox) :
LL(bbox.xMin(), bbox.yMin()),
UR(bbox.xMax(), bbox.yMax())
{
    //nop
}

bool
CalloutManager::BBox::overlaps(const BBox& rhs) const
{
    return overlap(rhs) > 0.0;
}

double
CalloutManager::BBox::overlap(const BBox& rhs) const
{
    double xmin = osg::maximum(LL.x(), rhs.LL.x());
    double xmax = osg::minimum(UR.x(), rhs.UR.x());
    if (xmin >= xmax) return 0.0;

    double ymin = osg::maximum(LL.y(), rhs.LL.y());
    double ymax = osg::minimum(UR.y(), rhs.UR.y());
    if (ymin >= ymax) return 0.0;

    double area = (UR.x() - LL.x())*(UR.y() - LL.y());
    double overlapArea = (xmax - xmin)*(ymax - ymin);

    return overlapArea / area;
}

CalloutManager::CalloutRecord::CalloutRecord() :
    _node(NULL),
    _frame(NULL),
    _leaderLineIndex(INT_MAX),
    _conflicted(false),
    _offsetVector(0,1,0)
{
    //nop
}

bool 
CalloutManager::CalloutRecord::operator < (const CalloutRecord& rhs) const
{
    if ((intptr_t)_node < (intptr_t)rhs._node) return true;
    return false;
}

void 
CalloutManager::CalloutRecord::move(float dir)
{
    // rotate little more than 1/4 turn:
    const double rotation = 1.7; //osg::PI / 32; //1.6;
    const osg::Quat q(dir*rotation, osg::Vec3d(0, 0, 1));
    _offsetVector = q * _offsetVector;
    realign();
}

void 
CalloutManager::CalloutRecord::realign()
{
    if (_offsetVector.x() >= 0.5)
    {
        if (_offsetVector.y() >= 0.5)
            _node->setAlignment(osgEarth::Text::LEFT_BOTTOM);
        else if (_offsetVector.y() <= -0.5)
            _node->setAlignment(osgEarth::Text::LEFT_TOP);
        else
            _node->setAlignment(osgEarth::Text::LEFT_CENTER);
    }
    else if (_offsetVector.x() <= -0.5)
    {
        if (_offsetVector.y() >= 0.5)
            _node->setAlignment(osgEarth::Text::RIGHT_BOTTOM);
        else if (_offsetVector.y() <= -0.5)
            _node->setAlignment(osgEarth::Text::RIGHT_TOP);
        else
            _node->setAlignment(osgEarth::Text::RIGHT_CENTER);
    }
    else if (_offsetVector.y() >= 0.0)
    {
        _node->setAlignment(osgEarth::Text::CENTER_BOTTOM);
    }
    else if (_offsetVector.y() <= 0.0)
    {
        _node->setAlignment(osgEarth::Text::CENTER_TOP);
    }
}

CalloutManager::SortCallback::SortCallback(CalloutManager* cm) :
    _cm(cm)
{
    //nop
}

void
CalloutManager::SortCallback::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
    _cm->sort(*nv);
    traverse(node, nv);
}
