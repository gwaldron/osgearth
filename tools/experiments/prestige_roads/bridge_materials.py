"""Generate reproducible, periodic material maps shared by the bridge and terrain prototype."""

import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.ndimage import gaussian_filter


def create_materials(directory):
    """Write original procedural albedo/ORM maps; periodic filtering avoids tile-border seams and baked lighting."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    size, seed = 1024, 20261012
    rng = np.random.default_rng(seed)
    grain = gaussian_filter(rng.normal(size=(size, size)), 0.7, mode='wrap')
    grain /= grain.std()
    medium = gaussian_filter(rng.normal(size=(size, size)), 6, mode='wrap')
    medium /= medium.std()
    coarse = gaussian_filter(rng.normal(size=(size, size)), 70, mode='wrap')
    coarse /= coarse.std()
    settings = [('asphalt', (63, 65, 66), 6.0, .86), ('concrete', (147, 145, 138), 3.5, .90),
                ('white', (229, 227, 216), 1.3, .60), ('yellow', (231, 188, 46), 1.3, .60)]
    record = {'generator': 'bridge_materials.py', 'seed': seed, 'period_m': 1.0, 'size_px': size,
              'source': 'Original numerical synthesis; no external imagery',
              'albedo_encoding': 'sRGB', 'packed_encoding': 'linear ORM: R=1, G=roughness, B=0',
              'policy': 'No lighting, layout, displacement, normal map, or flaking baked into materials',
              'materials': {}}
    for name, color, amplitude, roughness in settings:
        variation = amplitude*(grain*.65+medium*.3+coarse*.12)
        albedo = np.clip(np.asarray(color)[None, None, :]+variation[:, :, None], 0, 255).astype(np.uint8)
        rough = np.clip(roughness+.018*grain+.012*medium, .35, .98)
        packed = np.stack([np.full_like(rough, 255), rough*255, np.zeros_like(rough)], axis=-1).astype(np.uint8)
        paths = {'color': directory/(name+'-color.png'), 'packed': directory/(name+'-orm.png')}
        Image.fromarray(albedo).save(paths['color'])
        Image.fromarray(packed).save(paths['packed'])
        record['materials'][name] = {'mean_roughness': float(rough.mean()),
            'files': {key: {'file': path.name, 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
                      for key, path in paths.items()}}
    (directory/'materials.json').write_text(json.dumps(record, indent=2)+'\n', encoding='utf-8')
    return record
