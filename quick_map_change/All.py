import yaml
import numpy as np
import math
from PIL import Image
from geometry_msgs.msg import Pose

# --- CONFIGURATIE ---
AFBEELDING_PAD = 'WHEELTEC.png'
BASIS_NAAM = "WHEELTEC"
REF_AFSTAND = 1.2
YAML_EXPORT_POSES = 'locaties.yaml'

COLOR_LEGEND = {
    "Laadstation":          (255, 0, 0), 
    "Laadstation_Richting": (255, 150, 150),
    "Werkpunt":             (0, 255, 0), 
    "Werkpunt_Richting":    (150, 255, 150),
    "Quizlocatie":          (0, 0, 255), 
    "Quizlocatie_Richting": (150, 150, 255),
    "Referentie_1":         (255, 0, 255), 
    "Referentie_2":         (255, 150, 255)
}

class MapProcessor:
    def __init__(self):
        self.pixel_coords = {}
        self.resolutie = 0.0
        self.poses = {"Laadstation": None, "Werkpunt": None, "Quizlocatie": None}
        self.img_shape = (0, 0) # (breedte, hoogte)

    def proces_volledige_flow(self):
        img = Image.open(AFBEELDING_PAD).convert('RGB')
        self.img_shape = img.size
        img_array = np.array(img)

        # 1. Pixel locaties
        for naam, kleur in COLOR_LEGEND.items():
            matches = np.all(img_array == kleur, axis=-1)
            y, x = np.where(matches)
            self.pixel_coords[naam] = (int(np.mean(x)), int(np.mean(y))) if len(x) > 0 else None

        # 2. Resolutie & Poses
        self._bereken_resolutie()
        self._bereken_poses()
        self._exporteer_poses()

        # 3. Kaart conversie
        self._converteer_naar_pgm(img_array)
        self._genereer_yaml_bestanden()

    def _bereken_resolutie(self):
        ref1, ref2 = self.pixel_coords["Referentie_1"], self.pixel_coords["Referentie_2"]
        pixel_dist = math.sqrt((ref2[0]-ref1[0])**2 + (ref2[1]-ref1[1])**2)
        self.resolutie = REF_AFSTAND / pixel_dist

    def _bereken_poses(self):
        for basis in self.poses.keys():
            if self.pixel_coords.get(basis):
                x_b, y_b = self.pixel_coords[basis]
                pose = Pose()
                # Coördinaten t.o.v. laadstation (0,0)
                ls_x, ls_y = self.pixel_coords["Laadstation"]
                pose.position.x = (x_b - ls_x) * self.resolutie
                pose.position.y = (ls_y - y_b) * self.resolutie # Y-as omgedraaid voor ROS
                
                richting = self.pixel_coords.get(f"{basis}_Richting")
                if richting:
                    dx = (richting[0] - x_b) * self.resolutie
                    dy = (y_b - richting[1]) * self.resolutie
                    yaw = math.atan2(dy, dx)
                    pose.orientation.z = math.sin(yaw / 2.0)
                    pose.orientation.w = math.cos(yaw / 2.0)
                self.poses[basis] = pose

    def _exporteer_poses(self):
        data = {n: {"pose": {"position": {"x": p.position.x, "y": p.position.y, "z": 0.0},
                             "orientation": {"x": 0.0, "y": 0.0, "z": p.orientation.z, "w": p.orientation.w}}} 
                for n, p in self.poses.items() if p}
        with open(YAML_EXPORT_POSES, 'w') as f:
            yaml.dump(data, f, sort_keys=False)

    def _converteer_naar_pgm(self, img_array):
        img_array = img_array.astype(float)
        mask = np.any([np.sqrt(np.sum((img_array - np.array(k))**2, axis=-1)) < 40 for k in COLOR_LEGEND.values()], axis=0)
        for _ in range(5):
            for y in range(1, img_array.shape[0]-1):
                for x in range(1, img_array.shape[1]-1):
                    if mask[y, x]:
                        valid = img_array[y-1:y+2, x-1:x+2][~mask[y-1:y+2, x-1:x+2]]
                        if valid.size > 0:
                            img_array[y, x] = np.mean(valid, axis=0)
                            mask[y, x] = False
        Image.fromarray(np.clip(img_array, 0, 255).astype(np.uint8)).convert('L').save(f"{BASIS_NAAM}.pgm")

    def _genereer_yaml_bestanden(self):
        # Nulpunt op Laadstation (ls_x, ls_y)
        ls_x, ls_y = self.pixel_coords["Laadstation"]
        # Origin = -(offset in meters). De Y-as in YAML/ROS gaat omhoog, in pixels omlaag.
        origin_x = -(ls_x * self.resolutie)
        origin_y = -((self.img_shape[1] - ls_y) * self.resolutie)
        
        config = [("default", 0.70, 0.70), ("KEEPOUT_WORKING", 0.15, 0.10), ("KEEPOUT_SERVICE", 0.35, 0.30)]
        for suffix, occ, free in config:
            naam = f"{BASIS_NAAM}_{suffix}.yaml" if suffix != "default" else f"{BASIS_NAAM}.yaml"
            with open(naam, 'w') as f:
                f.write(f"image: {BASIS_NAAM}.pgm\nmode: trinary\nresolution: {self.resolutie:.6f}\n"
                        f"origin: [{origin_x:.4f}, {origin_y:.4f}, 0]\nnegate: 0\n"
                        f"occupied_thresh: {occ}\nfree_thresh: {free}\n")

if __name__ == "__main__":
    processor = MapProcessor()
    processor.proces_volledige_flow()