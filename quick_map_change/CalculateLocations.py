import yaml
import numpy as np
import math
from PIL import Image
from geometry_msgs.msg import Pose

COLOR_LEGEND = {
    # Puur rood & lichtrood
    "Laadstation":          (255, 0, 0),
    "Laadstation_Richting": (255, 150, 150),
    # Puur groen & lichtgroen
    "Werkpunt":             (0, 255, 0),
    "Werkpunt_Richting":    (150, 255, 150),
    # Puur blauw & lichtblauw
    "Quizlocatie":          (0, 0, 255),
    "Quizlocatie_Richting": (150, 150, 255),
    # Puur magenta (paars) & licht magenta
    "Referentie_1":         (255, 0, 255),
    "Referentie_2":         (255, 150, 255)
}

class CalculatePositions:
    def __init__(self, afbeelding_pad: str, ref_afstand: float):
        self.afbeelding_pad = afbeelding_pad
        self.ref_afstand = ref_afstand

        self.pixel_coords = {}
        self.resolutie = 0.0

        # Sla de uiteindelijke ROS Pose-objecten op in een overzichtelijke dictionary
        self.poses = {
            "Laadstation": None,
            "Werkpunt": None,
            "Quizlocatie": None
        }

    def calculate(self):
        """Voert de volledige berekeningsflow uit."""
        self.pixel_coords = self._bereken_pixel_locaties()
        self.resolutie = self._bereken_resolutie()
        self._bereken_poses()
        self.exporteer()
        
    def _bereken_pixel_locaties(self) -> dict:
        """Zoekt de middens van de kleurvlakken op de afbeelding."""
        img = Image.open(self.afbeelding_pad).convert('RGB')
        img_array = np.array(img)
        
        gevonden_coords = {}

        for naam, kleur in COLOR_LEGEND.items():
            matches = np.all(img_array == kleur, axis=-1)
            y_coords, x_coords = np.where(matches)
            
            if len(x_coords) > 0:
                middel_x = int(np.round(np.mean(x_coords)))
                middel_y = int(np.round(np.mean(y_coords)))          
                gevonden_coords[naam] = (middel_x, middel_y)
            else:
                # Alleen loggen in plaats van direct crashen; handig als bepaalde punten optioneel zijn
                print(f"[WAARSCHUWING] Kleur voor '{naam}' niet gevonden op de afbeelding.")
                gevonden_coords[naam] = None

        return gevonden_coords

    def _bereken_resolutie(self) -> float:
        """Berekent de verhouding tussen meters en pixels."""
        ref1 = self.pixel_coords.get("Referentie_1")
        ref2 = self.pixel_coords.get("Referentie_2")

        if not ref1 or not ref2:
            raise ValueError("Beide referentiepixels moeten aanwezig zijn om de resolutie te berekenen.")

        dx = ref2[0] - ref1[0]
        dy = ref2[1] - ref1[1]
        pixel_afstand = math.sqrt(dx**2 + dy**2)

        if pixel_afstand == 0:
            raise ValueError("De twee referentiepixels bevinden zich op exact dezelfde locatie.")
        
        return self.ref_afstand / pixel_afstand

    def _bereken_poses(self):
        """Genereert geometry_msgs.Pose objecten voor elk geldig basispunt."""
        for basis in self.poses.keys():
            if self.pixel_coords.get(basis):
                x_base, y_base = self.pixel_coords[basis]
                
                pose = Pose()
                pose.position.x = float(x_base * self.resolutie)
                pose.position.y = float(y_base * self.resolutie)
                pose.position.z = 0.0
                
                # Check of er een corresponderend richtingspunt is
                richting_key = f"{basis}_Richting"
                if self.pixel_coords.get(richting_key):
                    x_dir, y_dir = self.pixel_coords[richting_key]
                    
                    abs_x_dir = float(x_dir * self.resolutie)
                    abs_y_dir = float(y_dir * self.resolutie)
                    
                    dx = abs_x_dir - pose.position.x
                    dy = abs_y_dir - pose.position.y
                    
                    yaw = math.atan2(dy, dx)
                    self._set_quaternion_from_yaw(pose, yaw)
                else:
                    self._set_quaternion_from_yaw(pose, 0.0)
                
                self.poses[basis] = pose

    def _set_quaternion_from_yaw(self, pose: Pose, yaw: float):
        """Hulpfunctie om gierhoek (yaw) om te zetten in een quaternion."""
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)

    def exporteer(self, bestandspad: str = 'locaties.yaml'):
        """Exporteert de berekende locaties naar een leesbaar YAML-bestand."""
        data = {}
        
        # Converteer ROS Pose objecten expliciet naar dictionaries voor propere YAML output
        for naam, pose in self.poses.items():
            key_naam = naam.lower()
            if pose:
                data[key_naam] = {
                    "pose": {
                        "position": {"x": pose.position.x, "y": pose.position.y, "z": pose.position.z},
                        "orientation": {"x": pose.orientation.x, "y": pose.orientation.y, "z": pose.orientation.z, "w": pose.orientation.w}
                    }
                }
            else:
                data[key_naam] = None
        
        try:
            with open(bestandspad, 'w') as file:
                yaml.dump(data, file, default_flow_style=False, sort_keys=False)
            print(f"[INFO] Locaties succesvol geëxporteerd naar '{bestandspad}'")
        except Exception as e:
            print(f"[FOUT] Kon bestand niet exporteren: {e}")

if __name__ == "__main__":
    calculator = CalculatePositions('WHEELTEC.png', 1.2)
    calculator.calculate()