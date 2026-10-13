"""Add the existing urban junction, crossing, stop-bar and curb compiler to a solved regional road package."""

import argparse
from collections import defaultdict
import json
import pickle
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import shapely
from shapely.geometry import GeometryCollection, LineString, Polygon, shape

import interchange as regional
import raster_roads as planar
from vector_patch import fresh_planar_compile


def configure(name):
    """Use the same metric frame, source and locale in both the urban and multilayer compilers."""
    regional.configure(name)
    planar.ORIGIN, planar.LOCAL = regional.ORIGIN, regional.compiler.LOCAL
    planar.TO_LOCAL, planar.TO_GEO = regional.compiler.TO_LOCAL, regional.compiler.TO_GEO
    planar.REGION, planar.SOURCE = regional.REGION, regional.SOURCE
    planar.GEO_REGION = shapely.transform(regional.REGION, planar.TO_GEO.transform, interleaved=False)
    planar.SCENE_NAME = planar.SCENE_TEMPLATE = name
    planar.PAINT_NETWORK = planar.MATERIAL_CONTEXT = None


def vectors():
    """Regenerate all urban vectors and source control evidence, retaining a separate local checkpoint."""
    output = regional.OUTPUT/'urban'
    output.mkdir(exist_ok=True)
    regional.compiler.TERRAIN = lambda x,y: np.zeros(np.broadcast_arrays(x,y)[0].shape)
    layers, roads, layouts, cells = fresh_planar_compile(regional.REGION, output)
    with (output/'materials.pkl').open('wb') as stream:
        pickle.dump(layers, stream, protocol=5)
    regional.compiler.write_json(output/'complete.json', {
        'source_sha256': regional.digest(regional.SOURCE), 'layouts': len(layouts),
        'lane_connections': sum(len(j['connectors']) for j in layouts), 'paint_cells': cells,
        'materials': {m: len(polygons) for m,polygons in layers.items()},
        'source_control_evidence': [f'marking-evidence-{i:02d}.json' for i in cells]})
    print('Urban vectors complete', len(layouts), 'layouts', flush=True)


def compose():
    """Combine urban material polygons with separate structures and bind new rounded fill to solved ground profiles."""
    root=regional.OUTPUT
    if (root/'build-in-progress.json').exists():
        raise ValueError('Finish the structure build before composing city materials')
    info=json.loads((root/'urban/complete.json').read_text())
    if info['source_sha256']!=regional.digest(regional.SOURCE):
        raise ValueError('Urban source snapshot changed')
    with (root/'urban/materials.pkl').open('rb') as stream:
        layers=pickle.load(stream)
    with (root/'network-input.pkl').open('rb') as stream:
        network,ordered,connected=pickle.load(stream)
    network.heights=np.load(root/'solved-heights.npy')
    manifest=json.loads((root/'manifest.json').read_text())
    if manifest['profile_solve']['heights_sha256']!=regional.digest(root/'solved-heights.npy'):
        raise ValueError('Structure geometry does not match the current road heights')
    elevated=json.loads((root/'structure-surfaces.geojson').read_text())['features']
    promoted=set(manifest['inferred_geometry_approaches'])
    # Only promoted ground ways belong to the urban compiler; mapped bridges were already excluded upstream.
    exclude=shapely.union_all([shapely.transform(shape(f['geometry']),planar.TO_LOCAL.transform,interleaved=False)
        for f in elevated if int(f['properties']['road_id']) in promoted and f['properties']['material']=='asphalt'])
    structure_ids={int(f['properties']['road_id']) for f in elevated}
    joins=[(a,b,overlap) for a,b,node,overlap in connected]
    joins.extend(getattr(network,'indirect_connections',[]))
    landing_overlap=[overlap for a,b,overlap in joins
        if (network.roads[a]['id'] in structure_ids)!=(network.roads[b]['id'] in structure_ids)]
    exclude=exclude.union(shapely.union_all(landing_overlap))
    pavement=regional.polygonal(shapely.union_all(layers['asphalt']).difference(exclude))
    for material,polygons in layers.items():
        geometry=regional.polygonal(shapely.union_all(polygons).difference(exclude))
        if material in ('white','yellow','gutter'):
            geometry=geometry.intersection(pavement)
        features=[regional.feature(part,{'material':material,'receiver':'terrain',
            'source_evidence':'urban source and marking evidence retained separately'},str(i))
            for i,part in enumerate(regional.compiler.parts(geometry))]
        regional.compiler.write_json(root/('urban-'+material+'.geojson'),
            {'type':'FeatureCollection','features':features})
    grading=json.loads((root/'grading-profiles.geojson').read_text())
    local_masks=[]
    for f in grading['features']:
        for part in f['geometry']['geometries'][1:]:
            local_masks.append(shapely.transform(shape(part),planar.TO_LOCAL.transform,interleaved=False))
    residual=regional.polygonal(pavement.difference(shapely.union_all(local_masks)))
    terrain_indices=[i for i,r in enumerate(network.roads) if not r['bridge'] and r['id'] not in promoted
        and not r['tunnel'] and r['tags'].get('layer','0')=='0']
    tree=shapely.STRtree([network.roads[i]['line'] for i in terrain_indices])
    assigned=defaultdict(list)
    lower_protection=[(lower,network.roads[lower]['surface'].buffer(6).intersection(patch.buffer(8)))
        for upper,lower,patch in ordered if network.roads[lower]['id'] not in structure_ids]
    fill_count=0
    for mask in regional.compiler.parts(residual):
        if mask.area<1e-8:
            continue
        index=terrain_indices[int(tree.nearest(mask.representative_point()))]
        assigned[index].append(mask)
        fill_count+=1
    # One footprint per assigned profile avoids publishing thousands of duplicate XYZ centerlines
    # for disconnected pieces and harmless projection slivers along the same road.
    protected_area=0.
    fill_profiles=0
    for index,masks in sorted(assigned.items()):
        road=network.roads[index]
        mask=regional.polygonal(shapely.union_all(masks))
        original_area=mask.area
        # New rounded pavement may not broaden another profile's shoulder over an authoritative
        # lower-road crossing receiver. Its own lower profile can still fill its pavement boundary.
        for lower,protection in lower_protection:
            if index!=lower and mask.intersects(protection):
                mask=regional.polygonal(mask.difference(protection))
        protected_area+=original_area-mask.area
        if mask.is_empty:
            continue
        line=LineString(np.column_stack([network.xy[network.indices[index]],network.heights[network.indices[index]]]))
        grading['features'].append(regional.feature(GeometryCollection([line,mask]),{
            'road_id':str(road['id']),'profile_offset_m':0.,
            'mask_basis':'Inferred nearest solved ground centerline for rounded urban pavement fill'},
            'urban-fill/'+str(road['id'])))
        fill_profiles+=1
    regional.compiler.write_json(root/'urban-grading-profiles.geojson',grading)
    for path in root.glob('*.earth'):
        if path.name=='base.earth':
            continue
        earth=ET.fromstring(path.read_text())
        if regional.NAME=='sion-city' and path.name=='overview.earth':
            earth.find('options/terrain').set('min_tile_range_factor','16')
        for child in list(earth):
            if child.attrib.get('name') in ('Urban gutter','Urban curb','Prestige gutter','Prestige curb'):
                earth.remove(child)
        for source in earth.findall('OGRFeatures'):
            url=source.find('url')
            if url.text.startswith('runtime-vectors/'):
                url.text=Path(url.text).stem+'.geojson'
            if url.text=='grading-profiles.geojson':
                url.text='urban-grading-profiles.geojson'
            elif url.text.startswith('ground-'):
                url.text=url.text.replace('ground-','urban-')
        for material in ('gutter','curb'):
            if not json.loads((root/('urban-'+material+'.geojson')).read_text())['features']:
                continue
            source=ET.SubElement(earth,'OGRFeatures',name='Urban '+material)
            ET.SubElement(source,'url').text='urban-'+material+'.geojson'
            layer=ET.SubElement(earth,'{osgEarthPrestige}surface_splatting',name='Prestige '+material,
                features='Urban '+material)
            for key,value in {'anchor':','.join(map(str,(*regional.ORIGIN,0))), 'texture_period':1,
                'coverage_outline':'false','min_level':15,'max_data_level':21,'tile_size':512}.items():
                ET.SubElement(layer,key).text=str(value)
            ET.SubElement(layer,'material',color='materials/concrete-color.png',
                packed='materials/concrete-orm.png',layout='orm')
        ET.register_namespace('prestige','osgEarthPrestige')
        path.write_text(ET.tostring(earth,encoding='unicode'),encoding='utf-8')
    info.update({'rounded_fill_features':fill_profiles,'rounded_fill_parts':fill_count,
        'rounded_fill_area_m2':residual.area-protected_area,'lower_receiver_protection_m2':protected_area,
        'solved_heights_sha256':regional.digest(root/'solved-heights.npy'),
        'generator_sha256':regional.digest(regional.ROOT/'city_network.py'),
        'height_association':'Shared source-connected solve; nearest ground profile for extra urban fill',
        'curbs':'Terrain material coverage; vertical curb geometry deferred'})
    regional.compiler.write_json(root/'urban-composition.json',info)
    # Preserve inspectable GeoJSON and publish a validated, indexed runtime companion package.
    from runtime_vectors import package, activate
    activate(root, package(root))
    print('Urban materials composed with',len(assigned),'inferred rounded-fill profile associations',flush=True)


def main():
    """Generate or compose the selected city without rebuilding the other retained regional datasets."""
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('vectors','compose'))
    parser.add_argument('--scene',choices=regional.DATASETS,default='sion-city')
    args=parser.parse_args()
    configure(args.scene)
    globals()[args.action]()


if __name__=='__main__':
    main()
