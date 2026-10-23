"""Deterministic wire/schema fingerprint, excluding episode IDs and wall time."""
import hashlib
import json
from pathlib import Path

import h5py
import numpy as np


def fingerprint(episode):
    episode = Path(episode)
    digest = hashlib.sha256()
    names = []
    with h5py.File(episode / 'episode.h5') as file:
        file.visititems(lambda name, obj: names.append(name) if isinstance(obj, h5py.Dataset) else None)
        digest.update(json.dumps(dict(file.attrs), sort_keys=True).encode())
        for name in names:
            dataset = file[name]
            description = [name, dataset.dtype.str, dataset.shape, dict(dataset.attrs)]
            digest.update(json.dumps(description, sort_keys=True).encode())
            values = dataset[:]
            if values.dtype.kind == 'O':
                for value in values:
                    data = value.tobytes() if isinstance(value, np.ndarray) else value
                    digest.update(len(data).to_bytes(8, 'little'))
                    digest.update(data)
            else:
                digest.update(values.tobytes())
    metadata = json.loads((episode / 'metadata.json').read_text())
    config = metadata['config']
    for key in ('episode_id', 'start_unix'):
        metadata.pop(key, None)
    config.pop('output_root', None)
    config.pop('config_sha256', None)
    # Layout migration changes only this label; preserve the historical hash oracle.
    if config.get('camera_config') == 'config/cameras/realsense.json':
        config['camera_config'] = 'realsense/config/cameras.json'
    digest.update(json.dumps(metadata, sort_keys=True, allow_nan=False).encode())
    return {'datasets': len(names), 'sha256': digest.hexdigest()}
