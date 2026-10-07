"""Explicit trinary occupancy and pixel-centre/map transformations."""
from dataclasses import dataclass
from pathlib import Path
import hashlib
import math

import numpy as np
from PIL import Image
import yaml

UNKNOWN, FREE, OCCUPIED = -1, 0, 1


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


@dataclass(frozen=True)
class PixelResult:
    pixel: tuple
    cell: object
    inside_map: bool
    reason: object = None


class MapRaster:
    def __init__(self, gray, config, alpha=None):
        if not isinstance(config, dict):
            raise ValueError('map YAML must be a mapping')
        self.config = dict(config)
        if config.get('mode', 'trinary') != 'trinary':
            raise ValueError('unsupported occupancy mode')
        self.resolution = self._number(config.get('resolution'), 'resolution')
        if self.resolution <= 0:
            raise ValueError('resolution must be positive')
        origin = config.get('origin')
        if not isinstance(origin, (list, tuple)) or len(origin) != 3:
            raise ValueError('origin must contain x/y/yaw')
        self.origin = np.array([self._number(x, 'origin') for x in origin])
        negate = config.get('negate')
        if isinstance(negate, bool) or not isinstance(negate, int) or negate not in (0, 1):
            raise ValueError('negate must be integer 0 or 1')
        free = self._number(config.get('free_thresh'), 'free_thresh')
        occupied = self._number(config.get('occupied_thresh'), 'occupied_thresh')
        if not 0 <= free < occupied <= 1:
            raise ValueError('invalid occupancy thresholds')
        gray = np.asarray(gray)
        if gray.dtype != np.uint8 or gray.ndim != 2 or min(gray.shape) < 1:
            raise ValueError('expected nonempty 8-bit grayscale image')
        self.gray = gray.copy()
        self.height, self.width = gray.shape
        theta = self.origin[2]
        self.rotation = np.array([[math.cos(theta), -math.sin(theta)],
                                  [math.sin(theta), math.cos(theta)]])
        probability = gray.astype(float) / 255
        if not negate:
            probability = 1 - probability
        self.occupancy = np.full(gray.shape, UNKNOWN, dtype=np.int8)
        self.occupancy[probability < free] = FREE
        self.occupancy[probability > occupied] = OCCUPIED
        if alpha is not None:
            alpha = np.asarray(alpha)
            if alpha.shape != gray.shape:
                raise ValueError('alpha shape mismatch')
            self.occupancy[alpha < 255] = UNKNOWN
        self.config.update(mode='trinary')
        self.yaml_path = self.image_path = None
        self.source_hashes = {}

    @staticmethod
    def _number(value, name):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(name + ' must be finite numeric')
        return float(value)

    def pixel_to_map(self, c, v):
        c, v = np.broadcast_arrays(np.asarray(c, float), np.asarray(v, float))
        if not np.isfinite(c).all() or not np.isfinite(v).all():
            raise ValueError('nonfinite_input')
        q = np.stack(((c + .5) * self.resolution,
                      (self.height - v - .5) * self.resolution), axis=-1)
        return q @ self.rotation.T + self.origin[:2]

    def local_to_map(self, q):
        return np.asarray(q) @ self.rotation.T + self.origin[:2]

    def map_to_local(self, xy):
        return (np.asarray(xy, float) - self.origin[:2]) @ self.rotation

    def map_to_pixel(self, xy):
        xy = np.asarray(xy, float)
        if xy.shape != (2,) or not np.isfinite(xy).all():
            return PixelResult((None, None), None, False, 'nonfinite_input')
        q = self.map_to_local(xy) / self.resolution
        pixel = (float(q[0] - .5), float(self.height - q[1] - .5))
        inside = bool(0 <= q[0] < self.width and 0 <= q[1] < self.height)
        cell = (int(np.floor(q[0])), self.height - 1 - int(np.floor(q[1]))) if inside else None
        return PixelResult(pixel, cell, inside, None if inside else 'outside_map')

    def classify(self, xy):
        result = self.map_to_pixel(xy)
        if not result.inside_map:
            return result.reason
        c, v = result.cell
        return {FREE: 'free', UNKNOWN: 'unknown', OCCUPIED: 'occupied'}[int(self.occupancy[v, c])]


def load_map(yaml_path):
    path = Path(yaml_path).resolve()
    config = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(config, dict) or not isinstance(config.get('image'), str) or not config['image']:
        raise ValueError('map YAML requires an image path')
    image_path = (path.parent / config['image']).resolve()
    with Image.open(image_path) as image:
        if image.mode == 'L':
            gray, alpha = np.array(image), None
        elif image.mode in ('RGB', 'RGBA', 'LA'):
            data = np.array(image)
            if image.mode == 'LA':
                gray, alpha = data[:, :, 0], data[:, :, 1]
            else:
                if not np.array_equal(data[:, :, 0], data[:, :, 1]) or not np.array_equal(data[:, :, 0], data[:, :, 2]):
                    raise ValueError('non-grayscale RGB image is unsupported')
                gray = data[:, :, 0]
                alpha = data[:, :, 3] if image.mode == 'RGBA' else None
        else:
            raise ValueError('unsupported image mode: ' + image.mode)
    raster = MapRaster(gray, config, alpha)
    raster.yaml_path, raster.image_path = path, image_path
    raster.source_hashes = {'yaml': sha256(path), 'image': sha256(image_path)}
    return raster
