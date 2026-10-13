"""Package inspection GeoJSON as indexed, bounded runtime vectors without changing the road design."""

import argparse
import ctypes
import ctypes.util
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import shapely
import numpy as np
from shapely.geometry import MultiPolygon, box, mapping, shape


GRID_LOD = 16
SPLIT_VERTICES = 2048
MATERIALS = ('asphalt', 'white', 'yellow', 'curb', 'gutter')


def digest(path):
    """Hash a file without retaining its contents in memory."""
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def polygons(geometry):
    """Yield positive-area polygon parts, discarding only zero-area clipping artifacts."""
    if geometry.geom_type == 'Polygon':
        if not geometry.is_empty and geometry.area > 0:
            yield geometry
    elif hasattr(geometry, 'geoms'):
        for part in geometry.geoms:
            yield from polygons(part)


def fragments(geometry):
    """Split expensive polygons on exact global-geodetic grid lines, preserving doubles and holes.

    LOD-16 boundaries coincide with whole pixels of 512-square coverage tiles at every supported
    LOD (15 and above). This avoids inserting antialiased seams inside a coverage pixel.
    Small features stay untouched; grading profiles never pass through this function.
    """
    if shapely.get_num_coordinates(geometry) <= SPLIT_VERTICES:
        yield geometry
        return
    step = 180.0 / (1 << GRID_LOD)
    left, bottom, right, top = geometry.bounds
    for y in range(math.floor((bottom + 90) / step), math.floor((top + 90) / step) + 1):
        for x in range(math.floor((left + 180) / step), math.floor((right + 180) / step) + 1):
            clip = box(-180 + x * step, -90 + y * step, -180 + (x + 1) * step, -90 + (y + 1) * step)
            parts = list(polygons(geometry.intersection(clip)))
            # Keep disconnected pieces of the same source in one fill operation. Separate SRC_OVER
            # draws would darken fractional coverage where two pieces share a coarse raster pixel.
            if parts:
                yield parts[0] if len(parts) == 1 else MultiPolygon(parts)


def gdal_converter(explicit=None):
    """Find the existing GDAL CLI; fail without downloading or changing the engine environment."""
    if explicit:
        candidate = Path(explicit)
    else:
        repo = Path(__file__).resolve().parents[3]
        candidate = repo / 'build/vcpkg_installed/x64-windows-release/tools/gdal/ogr2ogr.exe'
        if not candidate.is_file():
            candidate = Path(shutil.which('ogr2ogr') or 'ogr2ogr')
    if not candidate.is_file():
        raise FileNotFoundError('Provide --ogr2ogr pointing to the existing GDAL converter')
    return candidate.resolve()


def convert(converter, source, destination, output_format):
    """Convert with checked exit status and retained index/coordinate precision settings."""
    env = os.environ.copy()
    data = converter.parents[2] / 'share/gdal'
    if data.is_dir():
        env['GDAL_DATA'] = str(data)
    proj = converter.parents[2] / 'share/proj'
    if (proj / 'proj.db').is_file():
        env['PROJ_DATA'] = str(proj)
        env['PROJ_LIB'] = str(proj)
    options = ['-lco', 'SPATIAL_INDEX=YES'] if output_format == 'FlatGeobuf' else [
        '-lco', 'COORDINATE_PRECISION=17', '-lco', 'SIGNIFICANT_FIGURES=17']
    result = subprocess.run([str(converter), '-f', output_format, *options, str(destination), str(source)],
                            env=env, capture_output=True, text=True)
    if result.returncode:
        raise RuntimeError(f'GDAL conversion failed for {source.name}: {result.stderr}')


def binary_geometries(converter, path):
    """Read validation geometries as lossless WKB, avoiding GeoJSON writer rounding of tiny slivers."""
    libraries = [converter.parent / 'gdal.dll', converter.parents[2] / 'bin/gdal.dll']
    library = next((str(item) for item in libraries if item.is_file()), None) or ctypes.util.find_library('gdal')
    if not library:
        raise RuntimeError('Cannot locate the GDAL library beside the converter')
    directory = os.add_dll_directory(str(Path(library).parent)) if os.name == 'nt' else None
    gdal = ctypes.CDLL(library)

    def function(name, result, *arguments):
        """Bind the public GDAL C ABI with explicit pointer and argument types."""
        method = getattr(gdal, name)
        method.restype, method.argtypes = result, arguments
        return method

    pointer, integer, string = ctypes.c_void_p, ctypes.c_int, ctypes.c_char_p
    proj = converter.parents[2] / 'share/proj'
    if (proj / 'proj.db').is_file():
        paths = (string * 2)(str(proj).encode('utf-8'), None)
        function('OSRSetPROJSearchPaths', None, ctypes.POINTER(string))(paths)
    function('GDALAllRegister', None)()
    open_dataset = function('GDALOpenEx', pointer, string, integer, pointer, pointer, pointer)
    close = function('GDALClose', None, pointer)
    layer = function('GDALDatasetGetLayer', pointer, pointer, integer)
    spatial_reference = function('OGR_L_GetSpatialRef', pointer, pointer)
    geographic = function('OSRIsGeographic', integer, pointer)
    authority = function('OSRGetAuthorityCode', string, pointer, string)
    next_feature = function('OGR_L_GetNextFeature', pointer, pointer)
    geometry = function('OGR_F_GetGeometryRef', pointer, pointer)
    size = function('OGR_G_WkbSize', integer, pointer)
    export = function('OGR_G_ExportToWkb', integer, pointer, integer, pointer)
    field = function('OGR_F_GetFieldAsString', string, pointer, integer)
    field_index = function('OGR_F_GetFieldIndex', integer, pointer, string)
    field_count = function('OGR_F_GetFieldCount', integer, pointer)
    field_definition = function('OGR_F_GetFieldDefnRef', pointer, pointer, integer)
    field_type = function('OGR_Fld_GetType', integer, pointer)
    field_name = function('OGR_Fld_GetNameRef', string, pointer)
    field_double = function('OGR_F_GetFieldAsDouble', ctypes.c_double, pointer, integer)
    field_set = function('OGR_F_IsFieldSetAndNotNull', integer, pointer, integer)
    destroy = function('OGR_F_Destroy', None, pointer)
    dataset = open_dataset(str(path).encode('utf-8'), 4, None, None, None)
    if not dataset:
        raise ValueError(f'Cannot reopen {path}')
    try:
        source = layer(dataset, 0)
        srs = spatial_reference(source)
        # GDAL promotes XYZ profile collections to WGS84 3D; both preserve longitude/latitude and ellipsoid Z.
        if not srs or not geographic(srs) or authority(srs, None) not in (b'4326', b'4979'):
            raise ValueError(f'Runtime vectors lost their WGS84 coordinate reference: {path}')
        while True:
            feature = next_feature(source)
            if not feature:
                break
            try:
                key = field(feature, field_index(feature, b'runtime_source_index')).decode()
                part = int(field(feature, field_index(feature, b'runtime_part')))
                handle = geometry(feature)
                data = ctypes.create_string_buffer(size(handle))
                if export(handle, 1, data) != 0:
                    raise ValueError('Cannot export validation geometry')
                numbers = {}
                for index in range(field_count(feature)):
                    definition = field_definition(feature, index)
                    if field_type(definition) == 2 and field_set(feature, index):
                        numbers[field_name(definition).decode()] = field_double(feature, index)
                yield key, part, shapely.from_wkb(data.raw), numbers
            finally:
                destroy(feature)
    finally:
        close(dataset)
        if directory:
            directory.close()


def package(root, converter=None):
    """Write immutable indexed runtime files; validate decoded footprints before publishing a manifest.

    GeoJSON remains authoritative and unmodified. Grading GeometryCollections retain their complete
    XYZ centerlines, masks, and attributes together; only material polygons are subdivided.
    """
    root = Path(root).resolve()
    converter = gdal_converter(converter)
    sources = [root / ('urban-' + name + '.geojson') for name in (*MATERIALS, 'grading-profiles')]
    inputs = {path.name: digest(path) for path in sources}
    identity = hashlib.sha256(json.dumps({'inputs': inputs, 'generator': digest(__file__)},
                                        sort_keys=True).encode()).hexdigest()[:20]
    target = root / 'runtime-vectors' / identity
    manifest_path = target / 'manifest.json'
    if manifest_path.is_file():
        manifest = json.loads(manifest_path.read_text())
        if all(digest(target / item['file']) == item['sha256'] for item in manifest['layers']):
            return manifest_path
        raise ValueError('An immutable runtime package was modified; preserve it for investigation')
    target.mkdir(parents=True, exist_ok=True)
    records = []
    with tempfile.TemporaryDirectory(prefix='prestige-runtime-vectors-') as scratch:
        scratch = Path(scratch)
        for source in sources:
            collection = json.loads(source.read_text(encoding='utf-8'))
            features = collection['features']
            grading = source.stem.endswith('grading-profiles')
            output, expected, empty = [], {}, []
            max_source, max_output = 0, 0
            for number, feature in enumerate(features):
                original = shape(feature['geometry'])
                if original.is_empty:
                    empty.append(number)
                    continue
                if not original.is_valid:
                    raise ValueError(f'Invalid authoritative geometry: {source.name}, feature {number}')
                max_source = max(max_source, int(shapely.get_num_coordinates(original)))
                expected[str(number)] = original
                pieces = [original] if grading else list(fragments(original))
                for part, geometry in enumerate(pieces):
                    max_output = max(max_output, int(shapely.get_num_coordinates(geometry)))
                    properties = dict(feature['properties'])
                    properties.update(runtime_source_index=str(number), runtime_part=part,
                                      runtime_source_id=str(feature.get('id', number)))
                    # Keeping the raw grading geometry also preserves mixed 2D masks/3D profile components.
                    output.append({'type': 'Feature', 'properties': properties,
                                   'geometry': feature['geometry'] if grading else mapping(geometry)})
            intermediate = scratch / source.name
            intermediate.write_text(json.dumps({'type': 'FeatureCollection', 'features': output},
                                               separators=(',', ':')), encoding='utf-8')
            destination = target / (source.stem + '.fgb')
            if destination.exists():
                destination.unlink()  # This exact, unmanifested file belongs to this interrupted package.
            convert(converter, intermediate, destination, 'FlatGeobuf')
            restored = scratch / (source.stem + '-restored.geojson')
            convert(converter, destination, restored, 'GeoJSON')
            decoded = json.loads(restored.read_text(encoding='utf-8'))['features']
            binary = {(key, part): (geometry, numbers)
                      for key, part, geometry, numbers in binary_geometries(converter, destination)}
            groups = {}
            for item in decoded:
                properties = item['properties']
                key = properties['runtime_source_index']
                geometry, numbers = binary[(key, properties['runtime_part'])]
                properties.update(numbers)
                reference = output_feature_properties(features[int(key)]['properties'])
                if any(properties.get(k) != v for k, v in reference.items()):
                    raise ValueError(f'Attribute changed in {source.name}: {key}')
                if not geometry.is_valid:
                    raise ValueError(f'Invalid runtime geometry in {source.name}: {key}')
                groups.setdefault(key, []).append(geometry)
            if set(groups) != set(expected) or len(decoded) != len(output):
                raise ValueError(f'Missing runtime features in {source.name}')
            maximum_error = 0.0
            for key, parts in groups.items():
                original = expected[key]
                if grading:
                    # Z-aware equality is checked separately; planar equality alone can hide lost elevations.
                    if len(parts) != 1 or not shapely.equals_exact(original, parts[0], tolerance=1e-14):
                        raise ValueError(f'Grading geometry changed: {key}')
                    before = shapely.get_coordinates(original, include_z=True)
                    after = shapely.get_coordinates(parts[0], include_z=True)
                    # GDAL promotes 2D mask coordinates to Z=0 in a 3D collection. Profile Z must stay exact.
                    known = np.isfinite(before)
                    if before.shape != after.shape or not np.array_equal(before[known], after[known]):
                        raise ValueError(f'Grading coordinates changed: {key}')
                else:
                    error = original.symmetric_difference(shapely.union_all(parts)).area
                    maximum_error = max(maximum_error, error)
                    if error > max(1e-18, original.area * 1e-10):
                        raise ValueError(f'Runtime footprint changed: {source.name}, {key}, {error}')
            records.append({'source': source.name, 'source_sha256': inputs[source.name],
                            'source_bytes': source.stat().st_size, 'file': destination.name,
                            'sha256': digest(destination), 'bytes': destination.stat().st_size,
                            'source_features': len(features), 'runtime_features': len(output),
                            'empty_source_features_omitted': empty,
                            'maximum_source_vertices': max_source, 'maximum_runtime_vertices': max_output,
                            'maximum_footprint_error_degrees2': maximum_error,
                            'grading_profiles_unsplit': grading})
            print(source.name, '->', destination.name, len(output), 'features;', max_source, '->', max_output,
                  'maximum vertices', flush=True)
    manifest = {'schema': 1, 'grid': 'global-geodetic', 'split_lod': GRID_LOD,
                'split_threshold_vertices': SPLIT_VERTICES, 'minimum_coverage_lod': 15,
                'coverage_tile_size': 512, 'layers': records, 'geojson_preserved': True,
                'generator_sha256': digest(__file__)}
    manifest_path.write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
    return manifest_path


def output_feature_properties(properties):
    """Exclude no-data attributes that GDAL omits when round-tripping JSON nulls."""
    return {key: value for key, value in properties.items() if value is not None}


def activate(root, manifest_path):
    """Switch packaged URLs and use one query per polygon tile; preserve imagery/material/cache settings."""
    root, manifest_path = Path(root).resolve(), Path(manifest_path).resolve()
    manifest = json.loads(manifest_path.read_text())
    replacements = {}
    for item in manifest['layers']:
        source, output = root / item['source'], manifest_path.parent / item['file']
        if digest(source) != item['source_sha256'] or digest(output) != item['sha256']:
            raise ValueError('Source/runtime package changed after validation')
        replacements[Path(item['source']).stem] = output.relative_to(root).as_posix()
    ET.register_namespace('prestige', 'osgEarthPrestige')
    for path in root.glob('*.earth'):
        earth = ET.fromstring(path.read_text(encoding='utf-8'))
        changed = False
        selected = set()
        for source in earth.findall('OGRFeatures'):
            url = source.find('url')
            if url is not None and Path(url.text).stem in replacements:
                url.text = replacements[Path(url.text).stem]
                changed = True
                selected.add(source.get('name'))
        if changed:
            for layer in earth.findall('{osgEarthPrestige}surface_splatting'):
                if layer.get('features') not in selected:
                    continue
                if (layer.findtext('tile_size') != '512' or int(layer.findtext('min_level', '0')) < 15
                        or layer.findtext('coverage_outline') != 'false'):
                    raise ValueError('Indexed road surfaces require 512-square, LOD-15+, exact-fill coverage')
                # These are complete filled polygons without an expanding stroke. A query halo is
                # unnecessary and the generic neighbor-query path redraws shared features repeatedly.
                buffer = layer.find('buffer_width')
                if buffer is None:
                    buffer = ET.SubElement(layer, 'buffer_width')
                buffer.text = '0m'
            temporary = path.with_suffix('.earth.tmp')
            temporary.write_text(ET.tostring(earth, encoding='unicode'), encoding='utf-8')
            os.replace(temporary, path)
    (root / 'runtime-vectors.json').write_text(json.dumps({
        'manifest': manifest_path.relative_to(root).as_posix()}, indent=2) + '\n', encoding='utf-8')


def main():
    """Package an existing city, optionally switching its earth files after export checks pass."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    parser.add_argument('--ogr2ogr')
    parser.add_argument('--activate', action='store_true')
    args = parser.parse_args()
    manifest = package(args.root, args.ogr2ogr)
    if args.activate:
        activate(args.root, manifest)
    print(manifest, flush=True)


if __name__ == '__main__':
    main()
