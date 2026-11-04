"""Hulpklassen voor de keepout-toggler: zone-detectie en footprint-diagnose.

Dit bestand gebruikt numpy, OpenCV en PyYAML. Ontbreekt een van de drie, dan
meldt ``TOOLS_AVAILABLE`` False en werkt de toggler zonder guard en diagnose.
"""
import math
import os

try:
    import cv2
    import numpy as np
    import yaml
    TOOLS_AVAILABLE = True
except ImportError:  # pragma: no cover
    TOOLS_AVAILABLE = False

LETHAL = 99  # 99 (inscribed) en 100 (lethal) in een OccupancyGrid


class MaskGrid:
    """Maskerbeeld uit een map_server-yaml, omgezet naar waarden 0..100 (-1 onbekend).

    De omzetting volgt nav2_map_server (map_io.cpp) voor 'trinary' en 'scale'.
    """

    def __init__(self, yaml_path):
        with open(yaml_path) as f:
            cfg = yaml.safe_load(f)

        image_path = cfg['image']
        if not os.path.isabs(image_path):
            image_path = os.path.join(os.path.dirname(yaml_path), image_path)

        image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise RuntimeError(f'Kan maskerbeeld niet lezen: {image_path}')

        shade = image.astype(np.float32) / 255.0
        occ = shade if cfg.get('negate', 0) else 1.0 - shade
        occ_th = float(cfg['occupied_thresh'])
        free_th = float(cfg['free_thresh'])
        mode = cfg.get('mode', 'trinary')

        if mode == 'scale' and occ_th > free_th:
            middle = np.rint((occ - free_th) / (occ_th - free_th) * 100.0)
        else:
            middle = np.full(occ.shape, -1.0, dtype=np.float32)

        values = np.where(occ > occ_th, 100.0,
                          np.where(occ < free_th, 0.0, middle))
        self.values = values.astype(np.int16)
        self.height, self.width = self.values.shape
        self.resolution = float(cfg['resolution'])
        self.origin_x = float(cfg['origin'][0])
        self.origin_y = float(cfg['origin'][1])

    def world_to_cell(self, x, y):
        """Geeft (rij, kolom) in het beeld terug; arrays of scalars."""
        col = np.floor((np.asarray(x) - self.origin_x) / self.resolution).astype(int)
        y_index = np.floor((np.asarray(y) - self.origin_y) / self.resolution).astype(int)
        return self.height - 1 - y_index, col

    def same_grid(self, other):
        return (
            self.values.shape == other.values.shape
            and math.isclose(self.resolution, other.resolution)
            and math.isclose(self.origin_x, other.origin_x)
            and math.isclose(self.origin_y, other.origin_y)
        )


class ZoneMap:
    """Cellen die enkel in het zone-masker lethal zijn (de verboden zones)."""

    def __init__(self, full_yaml, empty_yaml):
        self.full = MaskGrid(full_yaml)
        empty = MaskGrid(empty_yaml)
        if not self.full.same_grid(empty):
            raise RuntimeError('Zone- en service-masker hebben een verschillend raster.')
        self.zone = (self.full.values >= LETHAL) & ~(empty.values >= LETHAL)
        self.grid = self.full

    def zone_at(self, x, y):
        """True per punt dat in een zone-cel ligt (buiten het raster = False)."""
        rows, cols = self.grid.world_to_cell(x, y)
        valid = (
            (rows >= 0) & (rows < self.grid.height)
            & (cols >= 0) & (cols < self.grid.width)
        )
        hits = np.zeros(rows.shape, dtype=bool)
        hits[valid] = self.zone[rows[valid], cols[valid]]
        return hits

    def zone_within(self, x, y, radius):
        """True als er een zone-cel binnen ``radius`` meter van (x, y) ligt."""
        rows, cols = self.grid.world_to_cell(x, y)
        reach = int(math.ceil(radius / self.grid.resolution))
        r0, r1 = max(int(rows) - reach, 0), min(int(rows) + reach + 1, self.grid.height)
        c0, c1 = max(int(cols) - reach, 0), min(int(cols) + reach + 1, self.grid.width)
        if r0 >= r1 or c0 >= c1:
            return False

        window = self.zone[r0:r1, c0:c1]
        if not window.any():
            return False

        row_idx, col_idx = np.ogrid[r0:r1, c0:c1]
        within = (
            (row_idx - int(rows)) ** 2 + (col_idx - int(cols)) ** 2
            <= (radius / self.grid.resolution) ** 2
        )
        return bool((window & within).any())


def mask_statistics(data):
    """Telt de waarden van een OccupancyGrid-masker; geeft een dict terug."""
    values = _as_int8(data)
    return {
        'totaal': int(values.size),
        'onbekend': int((values < 0).sum()),
        'vrij': int((values == 0).sum()),
        'zacht': int(((values > 0) & (values < LETHAL)).sum()),
        'lethal': int((values >= LETHAL).sum()),
        'verschillende_waarden': int(len(np.unique(values))),
    }


def footprint_report(msg, pose, half_x, half_y, zones):
    """Analyseert de robotvoetafdruk in een costmap (OccupancyGrid).

    ``pose`` is (x, y, yaw) in het frame van de costmap.
    """
    width = msg.info.width
    height = msg.info.height
    resolution = msg.info.resolution
    origin_x = msg.info.origin.position.x
    origin_y = msg.info.origin.position.y
    grid = _as_int8(msg.data).reshape(height, width)

    x, y, yaw = pose
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    us = np.arange(-half_x, half_x + resolution, resolution)
    vs = np.arange(-half_y, half_y + resolution, resolution)
    u_grid, v_grid = np.meshgrid(us, vs)
    px = (x + cos_yaw * u_grid - sin_yaw * v_grid).ravel()
    py = (y + sin_yaw * u_grid + cos_yaw * v_grid).ravel()

    cols = np.floor((px - origin_x) / resolution).astype(int)
    rows = np.floor((py - origin_y) / resolution).astype(int)
    inside = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)

    center_col = int(math.floor((x - origin_x) / resolution))
    center_row = int(math.floor((y - origin_y) / resolution))
    if not (0 <= center_col < width and 0 <= center_row < height):
        return {'toestand': 'BUITEN_COSTMAP', 'midden': None, 'lethal': 0,
                'lethal_in_zone': 0, 'midden_in_zone': False}

    values = grid[rows[inside], cols[inside]]
    center_value = int(grid[center_row, center_col])
    lethal_points = values >= 100

    lethal_in_zone = 0
    center_in_zone = False
    if zones is not None:
        zone_hits = zones.zone_at(px[inside], py[inside])
        lethal_in_zone = int((lethal_points & zone_hits).sum())
        center_in_zone = bool(zones.zone_at(np.array([x]), np.array([y]))[0])

    if center_value >= 100:
        state = 'MIDDEN_LETHAL'
    elif lethal_points.any():
        state = 'VOETAFDRUK_LETHAL'
    elif center_value >= LETHAL:
        state = 'MIDDEN_INSCRIBED'
    else:
        state = 'OK'

    return {
        'toestand': state,
        'midden': center_value,
        'lethal': int(lethal_points.sum()),
        'lethal_in_zone': lethal_in_zone,
        'midden_in_zone': center_in_zone,
    }


def _as_int8(data):
    try:
        return np.frombuffer(data, dtype=np.int8)
    except (TypeError, ValueError):
        return np.array(data, dtype=np.int8)
