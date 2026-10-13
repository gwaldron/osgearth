"""Compile bounded, source-connected road regions with explicit stacked-road height constraints."""

import argparse
from collections import Counter
from datetime import datetime, timezone
import gzip
import hashlib
import json
from pathlib import Path
import pickle
import shutil
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
import shapely
from shapely.geometry import GeometryCollection, LineString, Point, Polygon, box, mapping
from shapely.ops import substring

import compile as compiler
from fetch_source import download_region
from network_profiles import NetworkProfiles
from structure_vectors import separate_structure


ROOT = Path(__file__).resolve().parent
OUTPUT = ROOT/'output/springfield-interchange'
SOURCE = ROOT/'source/springfield-interchange.osm.gz'
ORIGIN = (-77.176, 38.791)
REGION = box(-1500, -1500, 1500, 1500)
NAME = 'springfield-interchange'
TITLE = 'Springfield interchange'
SIZE = (3000, 3000)
IMAGERY = None
DATASETS = {
    'springfield-interchange': {'title': 'Springfield interchange', 'origin': ORIGIN, 'size': SIZE},
    'sion-city': {'title': 'Sion city', 'origin': (7.365, 46.231), 'size': (4000, 3000),
        'locale': 'sion', 'imagery': 'https://readymap.org/readymap/tiles/1.0.0/22/',
        'views': ('overview','rhone','interchange','street'), 'join_mode':'structures',
        'interpolation_guard':.08, 'bridge_terrain':'source/sion-readymap',
        'grade_caps':{'primary':.12,'primary_link':.12,'secondary':.12,'secondary_link':.12,
            'tertiary':.12,'tertiary_link':.12,'residential':.20,'unclassified':.20,
            'living_street':.20,'service':.25,'cycleway':.25}}
}


def configure(name=None):
    """Select a separate metric scene without changing any of the three frozen city fixtures."""
    global NAME, TITLE, ORIGIN, REGION, OUTPUT, SOURCE, SIZE, IMAGERY
    NAME = name or NAME
    settings = DATASETS[NAME]
    TITLE, ORIGIN, SIZE = settings['title'], settings['origin'], settings['size']
    IMAGERY = settings.get('imagery')
    REGION = box(-SIZE[0]/2, -SIZE[1]/2, SIZE[0]/2, SIZE[1]/2)
    OUTPUT, SOURCE = ROOT/'output'/NAME, ROOT/'source'/(NAME+'.osm.gz')
    locale = compiler.SCENES.get(settings.get('locale'), {})
    compiler.SCENES[NAME] = {
        'bbox': (-77.20, 38.773, -77.152, 38.809),
        'driving_side': 'right', 'center_line': 'double_yellow', **locale, 'origin': ORIGIN}
    compiler.configure_scene(NAME)
    compiler.SCENES[NAME]['bbox'] = shapely.transform(REGION, compiler.TO_GEO.transform, interleaved=False).bounds
    compiler.BBOX = compiler.SCENES[NAME]['bbox']
    if settings.get('locale'):
        compiler.SCENES[NAME]['additional_classes'] = ['service', 'tertiary_link', 'living_street']
    compiler.REGION = REGION.buffer(350, join_style=2)
    OUTPUT.mkdir(parents=True, exist_ok=True)


def digest(path):
    """Identify inputs without modifying them; Python code hashes use canonical LF line endings."""
    data=path.read_bytes()
    if path.suffix=='.py':
        data=data.replace(b'\r\n',b'\n')
    return hashlib.sha256(data).hexdigest()


def fetch():
    """Retain one bounded original OSM snapshot and provenance, stripping contributor account identifiers."""
    if SOURCE.exists():
        print('Retaining existing', TITLE, 'source snapshot', flush=True)
        return
    geographic = shapely.transform(REGION.buffer(500, join_style=2), compiler.TO_GEO.transform, interleaved=False)
    root, requests = download_region(geographic.bounds, 2)
    for element in root:
        for key in ('user', 'uid', 'changeset'):
            element.attrib.pop(key, None)
    SOURCE.write_bytes(gzip.compress(ET.tostring(root, encoding='utf-8', xml_declaration=True), mtime=0))
    compiler.write_json(SOURCE.with_suffix('.json'), {
        'requests': requests, 'source_sha256': digest(SOURCE), 'origin': ORIGIN,
        'render_size_m': SIZE, 'context_m': 500,
        'retrieved_utc': datetime.now(timezone.utc).isoformat(),
        'source_policy': 'Complete returned ways; newest object versions merged, not an atomic snapshot',
        'attribution': 'OpenStreetMap contributors', 'license': 'https://www.openstreetmap.org/copyright'})
    print('Retained source snapshot', SOURCE.name, flush=True)


def inventory():
    """Record source road, structure, level, and connection evidence before inferring any elevations."""
    roads, xy, flags, relations = compiler.read_source(SOURCE)
    from structure_vectors import source_topology, attributes, separate_structure
    edges, nodes, _ = source_topology(roads, xy, compiler.TO_GEO)
    for name, data in [('topology-edges', edges), ('topology-nodes', nodes)]:
        compiler.write_json(OUTPUT/(name+'.geojson'), {'type': 'FeatureCollection', 'features': data})
    result = {'source_sha256': digest(SOURCE), 'roads': len(roads), 'bridge_ways': sum(r['bridge'] for r in roads),
        'classes': dict(Counter(r['tags']['highway'] for r in roads)),
        'layers': dict(Counter(r['tags'].get('layer', 'absent') for r in roads)),
        'structures': [attributes(r) for r in roads if separate_structure(r)],
        'width_flags': flags, 'relations': relations,
        'policy': 'Layer is relative ordering evidence, not a meter height; source connectivity is shared node identity'}
    compiler.write_json(OUTPUT/'source-inventory.json', result)
    print({k:v for k,v in result.items() if k in ('roads','bridge_ways','classes','layers')}, flush=True)
    return roads, xy


def prepare():
    """Build complete source profiles and crossing constraints before sampling the unchanged elevation stack."""
    roads,xy=inventory()
    roads=[r for r in roads if not r['tunnel']]
    compiler.TERRAIN=lambda x,y:np.zeros(np.broadcast_arrays(x,y)[0].shape)
    compiler.solve_profiles(roads,xy)
    network=NetworkProfiles(roads)
    ordered,ambiguous,connected=network.crossings(compiler.REGION)
    by_id={r['id']:r for r in roads}
    for record in ambiguous:
        a,b=[by_id[i] for i in record['ways']]
        record['inside_render_m2']=a['surface'].intersection(b['surface']).intersection(REGION).area
        record['required_for_render']=record['inside_render_m2']>.01
    compiler.write_json(OUTPUT/'ambiguous-crossings.json',ambiguous)
    compiler.write_json(OUTPUT/'split-approach-connections.json',network.connection_exemptions)
    with (OUTPUT/'network-input.pkl').open('wb') as stream:
        pickle.dump((network,ordered,connected),stream)
    lon,lat=compiler.TO_GEO.transform(*network.xy.T)
    np.savetxt(OUTPUT/'terrain-query.csv',np.column_stack([np.arange(len(lon)),lon,lat]),delimiter=',',
        header='id,longitude,latitude',comments='',fmt='%.16g')
    (OUTPUT/'base.earth').write_text('''<map name="Road network base elevation">
    <options><terrain color="#bcc0b7" max_lod="22" tile_size="17" /></options>
    <TMSElevation name="ReadyMap elevation"><url>https://readymap.org/readymap/tiles/1.0.0/116/</url>
    <vdatum>egm96</vdatum></TMSElevation></map>''',encoding='utf-8')
    compiler.write_json(OUTPUT/'inputs.json',{'source_sha256':digest(SOURCE),
        'query_sha256':digest(OUTPUT/'terrain-query.csv'),'network_sha256':digest(OUTPUT/'network-input.pkl'),
        'origin':ORIGIN,'render_size_m':SIZE,'roads':len(roads),'variables':len(network.xy),
        'ordered_crossing_pairs':len(ordered),'connected_overlap_pairs':len(connected),
        'unresolved_ordering':sum(r['required_for_render'] for r in ambiguous),
        'context_only_unresolved_ordering':sum(not r['required_for_render'] for r in ambiguous)})
    print('Prepared',len(network.xy),'height variables,',len(ordered),'ordered crossing pairs,',
        len(ambiguous),'ambiguous pairs',flush=True)


def solve():
    """Reject changed input snapshots, then solve the full graph and record source evidence apart from inference."""
    record=json.loads((OUTPUT/'inputs.json').read_text())
    if record['unresolved_ordering']:
        raise ValueError('Resolve ambiguous source crossing order before solving structure heights')
    for file,key in ((SOURCE,'source_sha256'),(OUTPUT/'terrain-query.csv','query_sha256'),
        (OUTPUT/'network-input.pkl','network_sha256')):
        if digest(file)!=record[key]:
            raise ValueError('Input changed since prepare: '+str(file))
    samples=np.loadtxt(OUTPUT/'terrain-samples.csv',delimiter=',',skiprows=1)
    query=np.loadtxt(OUTPUT/'terrain-query.csv',delimiter=',',skiprows=1)
    if samples.shape!=(len(query),5) or not np.isfinite(samples).all() or np.max(abs(samples[:,:3]-query))>1e-10:
        raise ValueError('Terrain samples do not match source query')
    with (OUTPUT/'network-input.pkl').open('rb') as stream:
        network,ordered,connected=pickle.load(stream)
    try:
        result=network.solve(samples[:,4],ordered,connected,join_mode=DATASETS[NAME].get('join_mode','all'),
            grade_caps=DATASETS[NAME].get('grade_caps'),
            interpolation_guard=DATASETS[NAME].get('interpolation_guard',.03))
    except ValueError as error:
        compiler.write_json(OUTPUT/'solve-failure.json',getattr(error,'diagnostic',{'error':str(error)}))
        raise
    result['source_sha256']=digest(SOURCE)
    result['terrain_sha256']=digest(OUTPUT/'terrain-samples.csv')
    np.save(OUTPUT/'solved-heights.npy',network.heights)
    result['solver_sha256']=digest(ROOT/'network_profiles.py')
    result['network_sha256']=digest(OUTPUT/'network-input.pkl')
    result['heights_sha256']=digest(OUTPUT/'solved-heights.npy')
    compiler.write_json(OUTPUT/'profile-solve.json',result)
    failure=OUTPUT/'solve-failure.json'
    if failure.exists():
        failure.unlink()
    print({k:v for k,v in result.items() if k!='crossings'},flush=True)


def feature(geometry,properties,identifier):
    """Export local metric geometry as WGS84, preserving any explicitly supplied ellipsoid Z coordinates."""
    def geographic_part(part):
        """Keep mixed line-Z/polygon-XY collections legal GeoJSON instead of introducing NaN polygon heights."""
        if part.geom_type=='GeometryCollection':
            return GeometryCollection([geographic_part(child) for child in part.geoms])
        if not part.has_z:
            result=shapely.transform(part,compiler.TO_GEO.transform,interleaved=False)
            # Projection can collapse sub-nanometer overlay slivers; publish legal polygons without quantization.
            if part.geom_type in ('Polygon','MultiPolygon') and not result.is_valid:
                result=polygonal(result)
            return result
        def project(x,y,z):
            """Project XY without changing compiler-owned ellipsoid heights."""
            lon,lat=compiler.TO_GEO.transform(x,y)
            return lon,lat,z
        return shapely.transform(part,project,interleaved=False,include_z=True)
    geographic=geographic_part(geometry)
    return {'type':'Feature','id':identifier,'properties':properties,'geometry':mapping(geographic)}


def polygonal(geometry):
    """Discard only zero-area overlay debris; mixed-dimensional collections are not pavement or paint."""
    polygons=list(compiler.parts(shapely.make_valid(geometry)))
    return shapely.union_all(polygons) if polygons else Polygon()


def build():
    """Publish terrain receivers and separate multilayer deck geometry from the accepted shared-node solve."""
    from bridge_slice import DeckGLTF,export_deck
    from deck_assembly import structure_joins,separate_inferred_edges,assemble
    from network_receivers import raised_approaches,protect_lower_receivers
    from urban import street_paint
    marker=OUTPUT/'build-in-progress.json'
    compiler.write_json(marker,{'status':'incomplete'})
    solved=json.loads((OUTPUT/'profile-solve.json').read_text())
    for path,key in ((ROOT/'network_profiles.py','solver_sha256'),(OUTPUT/'network-input.pkl','network_sha256'),
        (OUTPUT/'solved-heights.npy','heights_sha256'),(SOURCE,'source_sha256')):
        if digest(path)!=solved.get(key):
            raise ValueError('Solve is stale or incomplete: '+str(path))
    with (OUTPUT/'network-input.pkl').open('rb') as stream:
        network,ordered,connected=pickle.load(stream)
    network.heights=np.load(OUTPUT/'solved-heights.npy')
    profile_fairing = None
    if DATASETS[NAME].get('bridge_terrain'):
        from bridge_profiles import fair_cable_spans
        profile_fairing=fair_cable_spans(network,ordered,SOURCE,compiler.TO_LOCAL,REGION,
            DATASETS[NAME].get('grade_caps'),DATASETS[NAME].get('interpolation_guard',.03))
        compiler.write_json(OUTPUT/'bridge-profile-fairing.json',profile_fairing)
    roads=network.roads
    mapped_structures=[separate_structure(r) for r in roads]
    promotions=raised_approaches(network,ordered,mapped_structures,REGION)
    structures=[mapped or i in promotions for i,mapped in enumerate(mapped_structures)]
    compiler.write_json(OUTPUT/'receiver-inferences.json',{
        'policy':'Separate geometry for raised approaches within 6m of a lower receiver, with >2m height step',
        'source_tags_unchanged':True,
        'raised_approaches':{str(roads[i]['id']):evidence for i,evidence in promotions.items()}})
    materials={m:[] for m in ('asphalt','white','yellow')}
    grading,profile_features,structure_features=[],[],[]
    adjacent=[[] for _ in roads]
    owned=[r['surface'].intersection(REGION) for r in roads]
    # Assign duplicate pavement only among roads sharing a real connection; stacked crossings keep both receivers.
    joins=[(a,b,overlap) for a,b,node,overlap in connected]
    joins.extend(getattr(network,'indirect_connections',[]))
    joins,join_evidence=structure_joins(network,structures,REGION,ordered,joins)
    for a,b,overlap in joins:
        adjacent[a].append((b,overlap));adjacent[b].append((a,overlap))
        priority=lambda i:(structures[i],roads[i]['lanes'],-roads[i]['id'])
        loser=b if priority(a)>priority(b) else a
        owned[loser]=owned[loser].difference(overlap)
    edge_inferences=separate_inferred_edges(network,structures,owned,ordered,joins)
    base_materials=ROOT/'output/bridge-boston-congress/materials'
    shutil.copytree(base_materials,OUTPUT/'materials',dirs_exist_ok=True)
    frame=SimpleNamespace(to_geo=compiler.TO_GEO,to_ecef=compiler.TO_ECEF,
        anchor=compiler.ANCHOR,enu=compiler.ENU)
    writer=DeckGLTF(frame,OUTPUT)
    counts,structure_audits,jobs=Counter(),[],[]
    for i,road in enumerate(roads):
        pavement=polygonal(owned[i])
        if pavement.is_empty:
            continue
        identifier=str(road['id'])
        line=LineString(np.column_stack([network.xy[network.indices[i]],network.heights[network.indices[i]]]))
        props={'road_id':identifier,'surface_id':'way/'+identifier,'source_tags':road['tags'],
            'width_m':road['width'],'width_basis':road['width_reason'],'lanes':road['lanes'],
            'height_status':'inferred','elevation_datum':'WGS84 ellipsoid meters',
            'receiver':'structure' if structures[i] else 'terrain',
            'receiver_basis':'inferred raised approach' if i in promotions else 'source structure tags' if
                mapped_structures[i] else 'source ground road',
            'profile_offset_m':0.,'profile_search_width_m':120}
        profile_features.append(feature(line,props,identifier))
        branch_patches=[p for j,p in adjacent[i] if p.area>.01]
        exclusion=shapely.union_all(branch_patches) if branch_patches else Polygon()
        locale=compiler.SCENES[NAME]
        white,yellow=street_paint([dict(road,bridge=False)],exclusion,road['surface'],
            locale['driving_side'],locale['center_line'])
        if road['tags']['highway'] in ('motorway','motorway_link'):
            neighbors=shapely.union_all([roads[j]['surface'].intersection(p.buffer(2)) for j,p in adjacent[i]])
            # Source one-way orientation determines left-yellow/right-white highway edge paint.
            left=road['left_edge'].buffer(.075,cap_style=2).difference(neighbors)
            right=road['right_edge'].buffer(.075,cap_style=2).difference(neighbors)
            if road['tags'].get('oneway')=='-1':
                left,right=right,left
            if locale['center_line']=='double_yellow':
                yellow=yellow.union(left);white=white.union(right)
            else:
                white=white.union(left).union(right)
        try:
            paint={'asphalt':pavement,'white':polygonal(white).intersection(pavement),
                'yellow':polygonal(yellow).intersection(pavement)}
        except shapely.errors.GEOSException:
            for name,geometry in [('pavement',pavement),('white',white),('yellow',yellow)]:
                (OUTPUT/('failed-'+identifier+'-'+name+'.wkb')).write_bytes(shapely.to_wkb(geometry))
            raise
        if not structures[i]:
            for material,geometry in paint.items():
                if not geometry.is_empty:
                    materials[material].append(feature(geometry,dict(props,material=material),identifier+'/'+material))
            support=road['surface'].intersection(REGION)
            deck_overlap=shapely.union_all([p for j,p in adjacent[i] if structures[j]])
            for offset,mask in ((0.,support.difference(deck_overlap)),(-.03,support.intersection(deck_overlap))):
                if not mask.is_empty:
                    grading.append(feature(GeometryCollection([line,mask]),dict(props,profile_offset_m=offset),
                        identifier+'/grading/'+str(offset)))
            continue
        for material,geometry in paint.items():
            if not geometry.is_empty:
                structure_features.append(feature(geometry,dict(props,material=material),identifier+'/'+material))
        if i in promotions:
            # A source way can include ordinary cut/fill road beyond its narrow raised edge. Keep that support
            # recessed beneath the same profile, but exclude lower corridors that caused receiver promotion.
            support=protect_lower_receivers(network,i,road['surface'].intersection(REGION),structures)
            if not support.is_empty:
                grading.append(feature(GeometryCollection([line,support]),dict(props,profile_offset_m=-.03,
                    mask_basis='Inferred raised-approach support; lower road corridors excluded'),identifier+'/support'))
        ground_ends=[]
        for station,node in ((0.,road['refs'][0]),(road['line'].length,road['refs'][-1])):
            if not any(not structures[j] for j in network.incident[network.node_index[node]]):
                continue
            ground_ends.append(station)
            apron=substring(road['line'],max(0,station-8),min(road['line'].length,station+8))
            apron=apron.buffer(road['width']/2+6,cap_style='flat').intersection(REGION)
            # A landing support may not overwrite a lower road's receiver at a nearby crossing.
            for upper,lower,overlap in ordered:
                if upper==i:
                    apron=apron.difference(roads[lower]['surface'].buffer(6))
            apron=protect_lower_receivers(network,i,apron,structures)
            if not apron.is_empty:
                grading.append(feature(GeometryCollection([line,apron]),dict(props,profile_offset_m=-.03,
                    mask_basis='Inferred short landing support; lower road corridors excluded'),
                    identifier+'/apron/'+str(station)))

        def height(x,y,road_index=i):
            """Read only this receiver's profile; nearby stacked decks never contribute to its elevation."""
            return network.height(road_index,x,y)

        def thickness(xy,alignment=road['line'],ends=tuple(ground_ends)):
            """Use a constant illustrative slab except for four-meter tapers at actual ground connections."""
            station=shapely.line_locate_point(alignment,shapely.points(xy))
            distance=np.minimum.reduce([abs(station-end) for end in ends]) if ends else np.full(len(xy),4.)
            u=np.clip(distance/4,0,1)
            return .8*u*u*(3-2*u)

        bridge=SimpleNamespace(pavement=pavement,materials=paint,height=height,thickness=thickness,road_index=i)
        jobs.append(bridge)
    network.raised_receivers=set(promotions)
    decks,deferred_joins=assemble(jobs,network,joins,structures,REGION,ordered)
    assembly_ids={str(roads[i]['id']):deck.surface_id for deck in decks for i in deck.indices}
    for f in structure_features:
        f['properties']['assembly_id']=assembly_ids[f['properties']['road_id']]
    for name,features in [('ground-'+m,f) for m,f in materials.items()]+[
        ('grading-profiles',grading),('solved-profiles',profile_features),('structure-surfaces',structure_features)]:
        compiler.write_json(OUTPUT/(name+'.geojson'),{'type':'FeatureCollection','features':features})
    print('Vector receivers complete; assembling',len(jobs),'source surfaces into',len(decks),'deck sheets',flush=True)
    railings,assembly_features,transition_features,transition_evidence=[],[],[],[]
    family_plan, prepared_decks = None, {}
    if DATASETS[NAME].get('bridge_terrain'):
        from bridge_slice import prepare_deck_surface
        from bridge_families import plan_families
        from terrain import ExactSurface
        for deck in decks:
            prepared_decks[deck.surface_id] = prepare_deck_surface(deck, max_edge=2)
            deck.exported_surface = ExactSurface(prepared_decks[deck.surface_id])
        family_plan = plan_families(SOURCE,network,decks,REGION,
            ROOT/DATASETS[NAME]['bridge_terrain'],compiler.TO_LOCAL,compiler.TO_GEO)
    for bridge in decks:
        identifier=bridge.surface_id
        for k,transition in enumerate(bridge.transitions):
            props={key:value for key,value in transition.items() if key!='geometry'}
            props['assembly_id']=identifier
            transition_evidence.append(props)
            if 'geometry' in transition:
                transition_features.append(feature(transition['geometry'],props,identifier+'/transition/'+str(k)))
        first_mesh=len(writer.doc['meshes'])
        triangles,areas,error=export_deck(bridge,OUTPUT,writer=writer,max_edge=2,
            prepared_deck=prepared_decks.pop(identifier,None))
        for material,geometry in dict(bridge.materials,asphalt=bridge.pavement).items():
            if not geometry.is_empty:
                assembly_features.append(feature(geometry,{'surface_id':identifier,'material':material,
                    'source_ways':bridge.source_ways,'receiver':'structure','height_status':'inferred',
                    'source_layers':sorted({roads[i]['tags'].get('layer','0') for i in bridge.indices}),
                    'source_evidence':'structure-surfaces.geojson and deck-assemblies.json'},identifier+'/'+material))
        counts.update(triangles)
        structure_audits.append({'assembly_id':identifier,'ways':bridge.source_ways,
            'first_mesh':first_mesh,'mesh_count':len(writer.doc['meshes'])-first_mesh,
            'triangles':triangles,'areas_m2':areas,'partition_error_m2':error,
            'max_join_height_adjustment_m':bridge.field.max_adjustment})
        for k,line in enumerate(bridge.railings):
            railings.append(feature(LineString(line),{'assembly_id':identifier,'source_ways':bridge.source_ways,
                'geometry_status':'inferred','basis':'Exposed assembled deck edge; mapped joins and landings open',
                'height_m':1.15,'post_spacing_max_m':2.,'edge_inset_m':.20,
                'material':'galvanized steel','dimensions_basis':'Illustrative prototype, not surveyed'},
                identifier+'/railing/'+str(k)))
        print('Deck',identifier,'triangles',sum(triangles.values()),flush=True)
    family_report = None
    if DATASETS[NAME].get('bridge_terrain'):
        from bridge_families import export_families
        family_report,family_features,family_counts=export_families(family_plan,writer,feature)
        compiler.write_json(OUTPUT/'bridge-families.json',family_report)
        compiler.write_json(OUTPUT/'bridge-members.geojson',{'type':'FeatureCollection','features':family_features})
        counts.update({'family_'+k:v for k,v in family_counts.items()})
    writer.save()
    compiler.write_json(OUTPUT/'bridge-railings.geojson',{'type':'FeatureCollection','features':railings})
    compiler.write_json(OUTPUT/'deck-surfaces.geojson',{'type':'FeatureCollection','features':assembly_features})
    compiler.write_json(OUTPUT/'deck-transitions.geojson',{'type':'FeatureCollection','features':transition_features})
    compiler.write_json(OUTPUT/'deck-assemblies.json',{'source_surfaces':len(jobs),'deck_sheets':len(decks),
        'source_connections':join_evidence,'kept_separate':deferred_joins,'edge_inferences':edge_inferences,
        'width_transitions':transition_evidence,
        'assemblies':structure_audits,'railing_runs':len(railings),
        'policy':'Union only source-connected, single-valued deck sheets; preserve stacked crossings',
        'join_profile':'Four-meter continuous blend of adjoining solved profiles; source profiles retained'})
    for view in DATASETS[NAME].get('views',('overview','stack','ramp','south')):
        (OUTPUT/(view+'.earth')).write_text(scene(network,ordered,view),encoding='utf-8')
    compiler.write_json(OUTPUT/'mesh-audit.json',structure_audits)
    compiler.write_json(OUTPUT/'manifest.json',{'source_sha256':digest(SOURCE),'profile_solve':
        json.loads((OUTPUT/'profile-solve.json').read_text()),'triangles':dict(counts),
        'generator_sha256':{name:digest(ROOT/name) for name in
            ('interchange.py','network_profiles.py','network_receivers.py','bridge_slice.py','surfaces.py',
             'deck_assembly.py','deck_transitions.py','bridge_railings.py','bridge_evidence.py','bridge_families.py',
             'bridge_profiles.py')},
        'structure_meshes':len(structure_audits),'structure_source_surfaces':len(jobs),'railing_runs':len(railings),
        'ground_material_features':{m:len(f) for m,f in materials.items()},
        'inferred_geometry_approaches':[roads[i]['id'] for i in promotions],
        'grading_features':len(grading),'profile_features':len(profile_features),'native_grading_lod':16,
        'scope':TITLE+' road network; tunnels excluded','render_size_m':SIZE,'origin':ORIGIN,
        'imagery':IMAGERY, 'bridge_members':family_report['member_count'] if family_report else 0,
        'bridge_profile_fairing':profile_fairing,
        'limitations':['Inferred heights and widths; no surveyed structure dimensions',
            'Horizontal merge patches preserve shared pavement; full lane-to-lane interchange layout unresolved',
            'Basic inferred bridge templates; mapped support association, signs, railway/water clearance solver, '
            'and runtime revision coordination remain unresolved']})
    marker.unlink()
    print('Published',TITLE,dict(counts),flush=True)


def scene(network,ordered,view):
    """Combine ordinary graded terrain, R8 asphalt/paint coverage, and separately owned deck geometry."""
    ET.register_namespace('prestige','osgEarthPrestige')
    root=ET.Element('map',name=TITLE+' / inferred multilayer road profiles')
    options=ET.SubElement(root,'options')
    ET.SubElement(options,'terrain',color='#bcc0b7',tile_size='17',max_lod='22',
        min_tile_range_factor='16' if NAME=='sion-city' and view=='overview' else '7',
        screen_space_error='8',morph_terrain='true')
    if IMAGERY:
        imagery=ET.SubElement(root,'TMSImage',name='ReadyMap base imagery')
        ET.SubElement(imagery,'url').text=IMAGERY
    elevation=ET.SubElement(root,'TMSElevation',name='ReadyMap base elevation')
    ET.SubElement(elevation,'url').text='https://readymap.org/readymap/tiles/1.0.0/116/'
    ET.SubElement(elevation,'vdatum').text='egm96'
    source=ET.SubElement(root,'OGRFeatures',name='Shared ground profiles')
    ET.SubElement(source,'url').text='grading-profiles.geojson'
    grading=ET.SubElement(root,'{osgEarthPrestige}grading',name='Shared network grading',features='Shared ground profiles')
    for key,value in {'road_footprints':'true','profile_elevation':'absolute','profile_offset':'[profile_offset_m]',
        'line_width':120,'max_line_width':120,'buffer_width':5,'max_buffer_width':5,
        'min_level':16,'max_data_level':16,'tile_size':257}.items():
        ET.SubElement(grading,key).text=str(value)
    for material in ('asphalt','yellow','white'):
        source=ET.SubElement(root,'OGRFeatures',name='Ground '+material)
        ET.SubElement(source,'url').text='ground-'+material+'.geojson'
        ET.SubElement(source,'attribution').text='OpenStreetMap contributors / ODbL; inferred road dimensions'
        layer=ET.SubElement(root,'{osgEarthPrestige}surface_splatting',name='Prestige '+material,features='Ground '+material)
        for key,value in {'anchor':','.join(map(str,(*ORIGIN,0))),'texture_period':1,'coverage_outline':'false',
            'min_level':15,'max_data_level':19 if material=='asphalt' else 21,'tile_size':512,'buffer_width':'2m'}.items():
            ET.SubElement(layer,key).text=str(value)
        ET.SubElement(layer,'material',color='materials/'+material+'-color.png',
            packed='materials/'+material+'-orm.png',layout='orm')
        ET.SubElement(layer,'cache_policy',usage='no_cache')
    model=ET.SubElement(root,'Model',name='Stacked roads and ramps / inferred profiles',lighting='true')
    ET.SubElement(model,'url').text='roads.gltf'
    ET.SubElement(model,'location',long=str(ORIGIN[0]),lat=str(ORIGIN[1]),alt='0',mode='absolute')
    ET.SubElement(root,'{osgEarthPrestige}sky',preset='flat',hours='16',ambient='.3',
        sun_intensity='3',environment_intensity='0')
    candidates=[crossing for crossing in ordered if REGION.buffer(-150).covers(crossing[2].centroid)]
    highest=max(candidates,key=lambda x:(float(network.roads[x[0]]['tags'].get('layer',0)),
        -x[2].centroid.distance(Point(0,0))))
    poses={'overview':(np.array([0,0]),3300,-50,15),
        'stack':(np.array(highest[2].centroid.coords)[0],450,-25,40),
        'ramp':(np.array(highest[2].centroid.coords)[0],95,-12,100),
        'south':(np.array([-370,-1340]),500,-30,15)}
    if NAME=='sion-city':
        poses={'overview':(np.array([0,0]),5000,-58,0),
            'rhone':(np.array(compiler.TO_LOCAL.transform(7.36474,46.224289)),650,-30,20),
            'interchange':(np.array(highest[2].centroid.coords)[0],500,-35,45),
            'street':(np.array(compiler.TO_LOCAL.transform(7.357309667148276,46.23205663374532)),140,-60,0)}
    views=ET.SubElement(root,'viewpoints',home='0')
    names=[view]+[name for name in poses if name!=view] if NAME=='sion-city' else [view]
    for name in names:
        xy,distance,pitch,heading=poses[name]
        lon,lat=compiler.TO_GEO.transform(*xy)
        i=highest[0]
        if NAME=='sion-city':
            i=min(range(len(network.roads)),key=lambda index:network.roads[index]['line'].distance(Point(xy)))
        h=float(network.height(i,*xy)[0])
        ET.SubElement(views,'viewpoint',name=name,long=str(lon),lat=str(lat),height=str(h),
            range=str(distance),pitch=str(pitch),heading=str(heading))
    return ET.tostring(root,encoding='unicode')


def scenes():
    """Refresh diagnostic cameras and layer configuration without rebuilding unchanged compiler geometry."""
    with (OUTPUT/'network-input.pkl').open('rb') as stream:
        network,ordered,_=pickle.load(stream)
    network.heights=np.load(OUTPUT/'solved-heights.npy')
    for view in DATASETS[NAME].get('views',('overview','stack','ramp','south')):
        (OUTPUT/(view+'.earth')).write_text(scene(network,ordered,view),encoding='utf-8')


def main():
    """Fetch or inventory the bounded interchange without modifying retained city outputs."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('fetch', 'inventory', 'prepare', 'solve', 'build', 'scenes'))
    parser.add_argument('--scene', choices=DATASETS, default='springfield-interchange')
    args = parser.parse_args()
    configure(args.scene)
    if args.action == 'fetch':
        fetch()
        inventory()
    else:
        globals()[args.action]()


if __name__ == '__main__':
    main()
