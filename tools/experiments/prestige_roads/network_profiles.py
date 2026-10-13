"""Shared-node road profiles and sparse crossing constraints for bounded multilayer road experiments."""

from collections import defaultdict
import heapq
import numpy as np
import shapely
from scipy import sparse
from shapely.geometry import Point
from shapely.ops import substring


class NetworkProfiles:
    """Own one height variable per shared source node, plus private interval samples; never join XY crossings."""

    def __init__(self, roads, spacing=10):
        """Densify complete source paths without losing source-node identity or making layer numbers into heights."""
        self.roads = roads
        self.xy, self.node_index, self.indices, self.stations = [], {}, [], []
        self.incident = defaultdict(set)
        for ri, road in enumerate(roads):
            coordinates = np.asarray(road['line'].coords)[:, :2]
            indices = []
            for segment, (a,b) in enumerate(zip(coordinates, coordinates[1:])):
                node = road['refs'][segment]
                if node not in self.node_index:
                    self.node_index[node] = len(self.xy)
                    self.xy.append(a)
                index = self.node_index[node]
                self.incident[index].add(ri)
                indices.append(index)
                count = max(1, int(np.ceil(np.linalg.norm(b-a)/spacing)))
                for step in range(1, count):
                    indices.append(len(self.xy))
                    self.incident[len(self.xy)].add(ri)
                    self.xy.append(a+(b-a)*step/count)
            node = road['refs'][-1]
            if node not in self.node_index:
                self.node_index[node] = len(self.xy)
                self.xy.append(coordinates[-1])
            index = self.node_index[node]
            self.incident[index].add(ri)
            indices.append(index)
            indices = np.array(indices)
            coords = np.array(self.xy)[indices]
            lengths = np.linalg.norm(np.diff(coords, axis=0), axis=1)
            if np.any(lengths<1e-6):
                raise ValueError('Duplicate consecutive source positions in way '+str(road['id']))
            self.indices.append(indices)
            self.stations.append(np.r_[0, np.cumsum(lengths)])
        self.xy = np.asarray(self.xy)
        self.neighbors = defaultdict(set)
        for indices in self.indices:
            for a,b in zip(indices,indices[1:]):
                self.neighbors[a].add(b);self.neighbors[b].add(a)
        self.heights = None

    def basis(self, road, xy):
        """Return the two height variables and linear weights at each nearest source-centerline station."""
        points = np.asarray(xy).reshape(-1,2)
        station = shapely.line_locate_point(self.roads[road]['line'], shapely.points(points))
        segment = np.clip(np.searchsorted(self.stations[road],station,side='right')-1,0,len(self.indices[road])-2)
        fraction = np.clip((station-self.stations[road][segment])/
            (self.stations[road][segment+1]-self.stations[road][segment]),0,1)
        return self.indices[road][segment], self.indices[road][segment+1], 1-fraction, fraction

    def height(self, road, x, y):
        """Evaluate an immutable solved road profile for terrain export or its own structure receiver."""
        x,y = np.broadcast_arrays(x,y)
        a,b,wa,wb = self.basis(road,np.column_stack([x.ravel(),y.ravel()]))
        return (self.heights[a]*wa+self.heights[b]*wb).reshape(x.shape), np.zeros(x.shape)

    def crossings(self, region):
        """Use mapped layer order on overlapping pavement; retain ambiguous or connected overlaps separately."""
        tree = shapely.STRtree([r['surface'] for r in self.roads])
        node_roads = defaultdict(set)
        source_edges = defaultdict(list)
        for index, road in enumerate(self.roads):
            for node in road['refs']:
                node_roads[node].add(index)
            for a,b in zip(road['refs'],road['refs'][1:]):
                length=np.linalg.norm(self.xy[self.node_index[a]]-self.xy[self.node_index[b]])
                source_edges[a].append((b,float(length),index))
                source_edges[b].append((a,float(length),index))
        self.connection_exemptions = []
        self.indirect_connections = []
        ordered, ambiguous, connected = [], [], []
        for i,a in enumerate(self.roads):
            for j in tree.query(a['surface'],predicate='intersects'):
                j=int(j)
                if j<=i:
                    continue
                b=self.roads[j]
                overlap=a['surface'].intersection(b['surface']).intersection(region)
                if overlap.area<.01:
                    continue
                if a['bridge']!=b['bridge'] or a['tags'].get('layer','0')!=b['tags'].get('layer','0'):
                    targets=set(b['refs'])
                    queue=[(0.,node,(node,),()) for node in a['refs']]
                    heapq.heapify(queue)
                    visited=set()
                    while queue:
                        distance,node,path,via=heapq.heappop(queue)
                        if node in visited:
                            continue
                        visited.add(node)
                        if node in targets and len(path)>1:
                            xy=[self.xy[self.node_index[value]] for value in path]
                            from shapely.geometry import LineString
                            patch=LineString(xy).buffer(max(a['width'],b['width']))
                            # Preserve the actual centerline crossing and its full local width, even
                            # when a separate landing connection exists elsewhere on the same pair.
                            crossing=a['line'].intersection(b['line'])
                            patch=patch.difference(crossing.buffer(max(a['width'],b['width'])))
                            joint=overlap.intersection(patch)
                            if joint.area>.01:
                                self.connection_exemptions.append({'ways':[a['id'],b['id']],
                                    'via_ways':[self.roads[k]['id'] for k in via], 'via_nodes':list(path),
                                    'path_length_m':distance,'overlap_area_m2':joint.area,
                                    'basis':'Bounded mapped landing path; actual centerline crossing protected'})
                                self.indirect_connections.append((i,j,joint))
                                overlap=overlap.difference(patch)
                            break
                        for neighbor,length,via_road in source_edges[node]:
                            if via_road not in (i,j) and neighbor not in visited and \
                                distance+length<=4*max(a['width'],b['width']):
                                heapq.heappush(queue,(distance+length,neighbor,path+(neighbor,),via+(via_road,)))
                    if overlap.area<.01:
                        continue
                shared=set(a['refs'])&set(b['refs'])
                # Only the neighborhood of a real source connection is exempt from crossing clearance.
                for node in shared:
                    patch=Point(self.xy[self.node_index[node]]).buffer(max(a['width'],b['width'])*4)
                    joint=overlap.intersection(patch)
                    if joint.area>.01:
                        connected.append((i,j,node,joint))
                    overlap=overlap.difference(patch)
                if overlap.area<.01:
                    continue
                # Tag changes can split a bridge landing a meter before its junction. The adjacent
                # bank road then overlaps the inferred bridge width without crossing underneath it.
                # Require an actual short source-connected path; proximity alone never creates a join.
                if a['line'].intersection(b['line']).is_empty:
                    near_a = set().union(*(node_roads[node] for node in a['refs']))
                    near_b = set().union(*(node_roads[node] for node in b['refs']))
                    for via in sorted((near_a & near_b)-{i,j}):
                        link = self.roads[via]
                        for na in set(a['refs']) & set(link['refs']):
                            for nb in set(b['refs']) & set(link['refs']):
                                sa = link['line'].project(Point(self.xy[self.node_index[na]]))
                                sb = link['line'].project(Point(self.xy[self.node_index[nb]]))
                                if abs(sa-sb)>4*max(a['width'],b['width']):
                                    continue
                                patch = substring(link['line'],sa,sb).buffer(max(a['width'],b['width']))
                                joint = overlap.intersection(patch)
                                if joint.area>.01:
                                    self.connection_exemptions.append({'ways':[a['id'],b['id']],
                                        'via_way':link['id'],'via_nodes':[na,nb],'path_length_m':abs(sa-sb),
                                        'overlap_area_m2':joint.area,
                                        'basis':'Short mapped connection at a split approach; no centerline crossing'})
                                    self.indirect_connections.append((i,j,joint))
                                    overlap = overlap.difference(patch)
                    if overlap.area<.01:
                        continue
                try:
                    la=float(a['tags'].get('layer','0'));lb=float(b['tags'].get('layer','0'))
                    if not np.isfinite([la,lb]).all():
                        raise ValueError('Nonfinite layer')
                except ValueError:
                    la=lb=0
                if la==lb:
                    # A crossing already consumed by a source connection cannot make a remote edge sliver ambiguous.
                    unconnected_crossing=a['line'].intersection(b['line']).intersection(overlap)
                    if not unconnected_crossing.is_empty or a['bridge']!=b['bridge']:
                        ambiguous.append({'ways':[a['id'],b['id']],'area_m2':overlap.area})
                    continue
                upper,lower=(i,j) if la>lb else (j,i)
                ordered.append((upper,lower,overlap))
        return ordered,ambiguous,connected

    def solve(self, terrain, ordered, connected, clearance=5.25, thickness=.8, grade=.08,
        join_mode='all', grade_caps=None, interpolation_guard=.03):
        """Solve all road heights together; enforce full-width clearance and shared nodes, rejecting infeasible inputs."""
        import osqp
        n=len(self.xy)
        rows,cols,values,lower,upper = [],[],[],[],[]
        curves, curve_cols, curve_values = [],[],[]
        curve_row=0
        labels=[]
        parents=np.arange(n)

        def root(index):
            """Find a shared merge-plane variable, compressing only compiler-owned equivalence links."""
            while parents[index]!=index:
                parents[index]=parents[parents[index]]
                index=parents[index]
            return index

        def add(indices,weights,lo,hi,label):
            """Append one sparse linear constraint, summing repeated shared variables during matrix construction."""
            row=len(lower)
            rows.extend([row]*len(indices));cols.extend(indices);values.extend(weights)
            lower.append(lo);upper.append(hi)
            labels.append(label)

        for road_index,(indices,stations) in enumerate(zip(self.indices,self.stations)):
            ds=np.diff(stations)
            limit=(grade_caps or {}).get(self.roads[road_index]['tags']['highway'],grade)
            for a,b,length in zip(indices,indices[1:],ds):
                add([a,b],[-1/length,1/length],-limit,limit,('grade',self.roads[road_index]['id'],int(a),int(b)))
            for k in range(1,len(indices)-1):
                weights=[1/ds[k-1],-1/ds[k-1]-1/ds[k],1/ds[k]]
                curves.extend([curve_row]*3);curve_cols.extend(indices[k-1:k+2]);curve_values.extend(weights)
                curve_row+=1
        # Bridge-only stations receive a modest DEM obstacle floor, not layer-based meters of lift.
        bridge_only=np.array([all(self.roads[r]['bridge'] for r in self.incident[i]) for i in range(n)])
        weight=np.where(bridge_only,.08,1.)
        for i in range(n):
            # Untagged embankment approaches can climb to upper decks; bridge tagging is not an elevation limit.
            add([i],[1],terrain[i]+(.85 if bridge_only[i] else -4),terrain[i]+80,
                ('terrain',i,[self.roads[r]['id'] for r in self.incident[i]]))
        crossing_evidence=[]
        for hi,lo,polygon in ordered:
            boundary=shapely.get_coordinates(shapely.segmentize(polygon,2))
            xmin,ymin,xmax,ymax=polygon.bounds
            xx,yy=np.meshgrid(np.arange(xmin,xmax,3),np.arange(ymin,ymax,3))
            grid=np.column_stack([xx.ravel(),yy.ravel()])
            grid=grid[shapely.intersects_xy(polygon,grid[:,0],grid[:,1])]
            points=np.unique(np.r_[boundary,grid],axis=0)
            ha,hb,hwa,hwb=self.basis(hi,points);la,lb,lwa,lwb=self.basis(lo,points)
            for k in range(len(points)):
                add([ha[k],hb[k],la[k],lb[k]],[hwa[k],hwb[k],-lwa[k],-lwb[k]],
                    clearance+thickness+interpolation_guard,np.inf,
                    ('crossing',self.roads[hi]['id'],self.roads[lo]['id'],points[k].tolist()))
            crossing_evidence.append({'upper_way':self.roads[hi]['id'],'lower_way':self.roads[lo]['id'],
                'samples':len(points),'overlap_area_m2':polygon.area,'clearance_m':clearance,
                'interpolation_guard_m':interpolation_guard,
                'ordering_basis':'Relative OSM layer order','clearance_basis':'Inferred prototype target'})
        # A branch overlap uses one shared height so separate ramp surfaces cannot disagree where they merge.
        join_rows=0
        for a,b,node,polygon in connected:
            # Dense urban ground junctions must not transitively become one city-wide horizontal pad.
            # Terrain receivers can blend ground profiles; separate deck receivers require exact joins.
            if join_mode=='structures' and not any(self.roads[i]['bridge'] or
                self.roads[i]['tags'].get('layer','0')!='0' for i in (a,b)):
                continue
            indices=set()
            boundary=shapely.get_coordinates(shapely.segmentize(polygon,1))
            for road in (a,b):
                before,after,_,_=self.basis(road,boundary)
                positions={int(index):position for position,index in enumerate(self.indices[road])}
                lo=min(positions[int(i)] for i in np.r_[before,after])
                hi=max(positions[int(i)] for i in np.r_[before,after])
                indices.update(self.indices[road][lo:hi+1])
            center=self.node_index[node]
            # Only actual branching connections need a plane; degree-two continuations retain their slope.
            if len(self.neighbors[center])<3:
                continue
            for i in indices:
                if i!=center:
                    parents[root(i)]=root(center)
                    add([i,center],[1,-1],0,0,('join',node,int(i),int(center)));join_rows+=1
        curvature=sparse.coo_matrix((curve_values,(curves,curve_cols)),shape=(curve_row,n)).tocsc()
        p=2*(sparse.diags(weight)+2000*(curvature.T@curvature))
        constraints=sparse.coo_matrix((values,(rows,cols)),shape=(len(lower),n)).tocsc()
        # Eliminate identical junction heights algebraically instead of asking the optimizer to satisfy
        # thousands of redundant equalities. Shared pavement then agrees exactly, including on export.
        _,group=np.unique([root(i) for i in range(n)],return_inverse=True)
        transform=sparse.coo_matrix((np.ones(n),(np.arange(n),group)),shape=(n,max(group)+1)).tocsc()
        reduced=(constraints@transform).tocsr();reduced.eliminate_zeros()
        keep=np.flatnonzero(reduced.getnnz(axis=1))
        dropped=np.setdiff1d(np.arange(len(lower)),keep)
        contradictory=dropped[(np.array(lower)[dropped]>0)|(np.array(upper)[dropped]<0)]
        if len(contradictory):
            error=ValueError('Network profile constraints are infeasible at a shared junction')
            error.diagnostic={'status':'contradictory shared junction constraints',
                'constraints':[labels[i] for i in contradictory[:30]]}
            raise error
        solver=osqp.OSQP()
        solver.setup(P=(transform.T@p@transform).tocsc(),q=transform.T@(-2*weight*terrain),
            A=reduced[keep].tocsc(),l=np.array(lower)[keep],u=np.array(upper)[keep],
            eps_abs=1e-6,eps_rel=1e-7,max_iter=100000,polishing=True,verbose=False)
        result=solver.solve(raise_error=False)
        if result.info.status_val not in (1,2):
            error=ValueError('Network profile solve failed: '+result.info.status)
            certificate=result.prim_inf_cert if result.info.status_val in (3,4) else None
            ranked=np.argsort(abs(certificate))[-30:][::-1] if certificate is not None else []
            error.diagnostic={'status':result.info.status,'certificate_rows':[
                {'constraint':labels[keep[i]],'certificate':float(certificate[i]),
                 'lower':lower[keep[i]],'upper':upper[keep[i]]}
                for i in ranked]}
            raise error
        heights=transform@result.x
        if not np.isfinite(heights).all():
            raise ValueError('Network profile solve returned nonfinite heights')
        evaluated=constraints@heights
        violation=max(np.max(np.array(lower)-evaluated),np.max(evaluated-np.array(upper)))
        if violation>1e-4:
            raise ValueError('Network profile constraints exceed tolerance: '+str(violation))
        self.heights=heights
        return {'variables':n,'constraints':len(lower),'branch_equalities':join_rows,
            'solver_variables':transform.shape[1],
            'solver_status':result.info.status,'solver_primal_residual':float(result.info.prim_res),
            'solver_dual_residual':float(result.info.dual_res),
            'crossings':crossing_evidence,'max_constraint_error':float(violation),
            'grade_cap_percent':grade*100,'deck_thickness_m':thickness,'clearance_m':clearance,
            'join_mode':join_mode,
            'class_grade_caps_percent':{k:100*v for k,v in (grade_caps or {}).items()},
            'max_lift_m':float(np.max(heights-terrain)),'min_cut_m':float(np.min(heights-terrain)),
            'height_status':'inferred from ReadyMap and relative OSM crossing order; not surveyed'}
