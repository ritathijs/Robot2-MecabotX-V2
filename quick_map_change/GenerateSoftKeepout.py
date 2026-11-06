"""Genereert een zacht keepout-masker (map_server mode 'scale') uit de bestaande
WORKING- en SERVICE-maskers.

Het bestaande WORKING-masker is binair: elke verboden zone is meteen 100%
lethal. Een robot die iets te ver in de zone komt, staat dan in lethal terrein
en kan niet meer plannen of draaien. Dit script maakt een masker waarbij:

  * de rand binnen de zone een helling heeft (niet lethal, wel duur),
    zodat de robot er altijd uit kan rijden;
  * de ruimte net buiten de zone een kostenring heeft die de robot weghoudt
    van de rand (de voetafdruk van 1 m x 1 m draait met hoeken van ~0,7 m);
  * de diepe kern van de zone en alle muren/onbekend gebied lethal blijven.

Een zone-cel is: geblokkeerd in WORKING, maar vrij in SERVICE. Alles wat in
beide geblokkeerd is (muren, onbekend) blijft 100.

Gebruik (vanuit deze map):
    python3 GenerateSoftKeepout.py
    python3 GenerateSoftKeepout.py --inner 0.3 --outer 0.8
"""
import argparse
import os

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage

DEFAULT_MAP_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "wheeltec_robot_nav2", "map")


def load_trinary(yaml_path):
    """Leest een trinary masker en geeft (cfg, geblokkeerd, vrij) terug."""
    with open(yaml_path) as f:
        cfg = yaml.safe_load(f)

    image_path = cfg["image"]
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(yaml_path), image_path)

    gray = np.array(Image.open(image_path).convert("L"), dtype=np.float32)
    shade = gray / 255.0
    occ = shade if cfg.get("negate", 0) else 1.0 - shade

    blocked = occ > float(cfg["occupied_thresh"])
    free = occ < float(cfg["free_thresh"])
    return cfg, blocked, free


def build_soft_mask(blocked_work, blocked_service, free_work, resolution,
                    inner_m, outer_m, edge_value, outer_value):
    """Geeft occupancy-waarden 0..100 terug (float) voor het zachte masker."""
    zone = blocked_work & ~blocked_service
    values = np.zeros(blocked_work.shape, dtype=np.float32)

    # Muren en onbekend gebied: ongewijzigd lethal.
    values[blocked_work & ~zone] = 100.0

    # Afstand (m) van elke cel tot de dichtstbijzijnde vrije cel buiten de
    # zone; muren tellen niet mee als uitweg.
    exit_cells = free_work & ~zone
    dist_to_exit = ndimage.distance_transform_edt(~exit_cells) * resolution

    # Binnen de zone: helling van edge_value (rand) tot 99 op inner_m diepte,
    # daarna lethal. Zone-cellen zonder uitweg in de buurt blijven lethal.
    depth = np.maximum(dist_to_exit - resolution, 0.0)
    ramp = edge_value + (99.0 - edge_value) * np.clip(depth / inner_m, 0.0, 1.0)
    in_ramp = zone & (depth < inner_m)
    values[zone] = 100.0
    values[in_ramp] = ramp[in_ramp]

    # Buiten de zone: kostenring die lineair afneemt tot 0 op outer_m.
    dist_to_zone = ndimage.distance_transform_edt(~zone) * resolution
    ring_distance = np.maximum(dist_to_zone - resolution, 0.0)
    ring = outer_value * (1.0 - np.clip(ring_distance / outer_m, 0.0, 1.0))
    in_ring = exit_cells & (ring_distance < outer_m)
    values[in_ring] = ring[in_ring]

    return values, zone


def write_outputs(values, cfg, out_dir, name):
    # map_server 'scale' met free_thresh=0 en occupied_thresh=1 rekent
    # occ = (255 - pixel) / 255 om naar waarde = occ * 100.
    pixels = np.rint(255.0 * (1.0 - values / 100.0)).astype(np.uint8)
    pgm_path = os.path.join(out_dir, f"{name}.pgm")
    Image.fromarray(pixels).save(pgm_path)

    yaml_path = os.path.join(out_dir, f"{name}.yaml")
    origin = ", ".join(str(v) for v in cfg["origin"])
    with open(yaml_path, "w") as f:
        f.write(
            f"image: {name}.pgm\n"
            "mode: scale\n"
            f"resolution: {cfg['resolution']}\n"
            f"origin: [{origin}]\n"
            "negate: 0\n"
            "occupied_thresh: 1.0\n"
            "free_thresh: 0.0\n"
        )
    return pgm_path, yaml_path


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--map-dir", default=DEFAULT_MAP_DIR)
    parser.add_argument("--working", default="WHEELTEC_KEEPOUT_WORKING.yaml")
    parser.add_argument("--service", default="WHEELTEC_KEEPOUT_SERVICE.yaml")
    parser.add_argument("--name", default="WHEELTEC_KEEPOUT_WORKING_SOFT")
    parser.add_argument("--inner", type=float, default=0.30,
                        help="breedte (m) van de helling binnen de zone")
    parser.add_argument("--outer", type=float, default=0.80,
                        help="breedte (m) van de kostenring buiten de zone")
    parser.add_argument("--edge-value", type=float, default=70.0,
                        help="waarde (0-99) op de zonerand, binnenkant")
    parser.add_argument("--outer-value", type=float, default=50.0,
                        help="waarde (0-99) van de ring vlak tegen de zone")
    args = parser.parse_args()

    map_dir = os.path.abspath(args.map_dir)
    cfg, blocked_work, free_work = load_trinary(os.path.join(map_dir, args.working))
    cfg_service, blocked_service, _ = load_trinary(os.path.join(map_dir, args.service))

    if blocked_work.shape != blocked_service.shape:
        raise SystemExit("WORKING en SERVICE hebben verschillende afmetingen.")
    for key in ("resolution", "origin"):
        if cfg[key] != cfg_service[key]:
            raise SystemExit(f"WORKING en SERVICE verschillen in '{key}'.")

    values, zone = build_soft_mask(
        blocked_work, blocked_service, free_work, float(cfg["resolution"]),
        args.inner, args.outer, args.edge_value, args.outer_value)

    pgm_path, yaml_path = write_outputs(values, cfg, map_dir, args.name)

    res = float(cfg["resolution"])
    print(f"Zone-cellen: {int(zone.sum())} ({zone.sum() * res * res:.1f} m2)")
    print(f"Lethal (100): {int((values >= 99.5).sum())}  "
          f"zacht (1-99): {int(((values > 0) & (values < 99.5)).sum())}  "
          f"vrij (0): {int((values == 0).sum())}")
    print(f"Geschreven: {pgm_path}\n            {yaml_path}")


if __name__ == "__main__":
    main()
