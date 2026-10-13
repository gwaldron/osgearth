"""Map variants for comparing natural ReadyMap terrain with compiler-owned road grading."""

from terrain import URL


def write_real_earth(output, mode, sse, viewpoint, extra_views, scene, origin, altitude):
    """Write a live ReadyMap scene with absolute road heights and an optional sparse grading layer."""
    if mode not in ('raw', 'graded'):
        raise ValueError('Real terrain requires raw or graded mode')
    overlay = '''<GDALElevation name="INFERRED road-owned grading, WGS84 ellipsoid">
        <url>grading-overlay.tif</url><interpolation>bilinear</interpolation>
    </GDALElevation>''' if mode == 'graded' else ''
    views = {'overview': (*origin, -18, -65, 950, altitude),
             'junction': (*origin, 0, -65, 150, altitude),
             'far': (*origin, -20, -70, 7000, altitude)}
    views.update(extra_views or {})
    order = [viewpoint]+[name for name in views if name != viewpoint]
    view_xml = []
    for name in order:
        lon, lat, heading, pitch, distance, height = views[name]
        view_xml.append(f'<viewpoint name="{name}" long="{lon}" lat="{lat}" height="{height}" '
                        f'heading="{heading}" pitch="{pitch}" range="{distance}"/>')
    viewpoints = '\n        '.join(view_xml)
    document = f'''<map name="Prestige Roads / {scene.title()} / ReadyMap / {mode} / SSE {sse}">
    <options><terrain tile_size="33" screen_space_error="{sse}" morph_terrain="true"
        gpu_tessellation="false" skirt_ratio="0.005"/></options>
    <sky driver="simple" hours="17" ambient="0.35"/>
    <TMSImage name="ReadyMap imagery"><url>https://readymap.org/readymap/tiles/1.0.0/7/</url></TMSImage>
    <TMSElevation name="ReadyMap 90m source, EGM96"><url>{URL}</url><vdatum>egm96</vdatum></TMSElevation>
    {overlay}
    <Model name="Compiled OSM roads; inferred profiles and decks" lighting="true">
        <url>roads.gltf</url><location long="{origin[0]}" lat="{origin[1]}" alt="0" mode="absolute"/>
    </Model>
    <viewpoints home="0" time="3">
        {viewpoints}
    </viewpoints>
</map>
'''
    (output/f'{mode}-sse{sse}-{viewpoint}.earth').write_text(document, encoding='utf-8')
