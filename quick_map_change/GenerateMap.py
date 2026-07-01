import os
import numpy as np
from PIL import Image

# Kleurenlegende
COLOR_LEGEND = {
    "Laadstation": (255, 0, 0), "Laadstation_Richting": (255, 150, 150),
    "Werkpunt": (0, 255, 0), "Werkpunt_Richting": (150, 255, 150),
    "Quizlocatie": (0, 0, 255), "Quizlocatie_Richting": (150, 150, 255),
    "Referentie_1": (255, 0, 255), "Referentie_2": (255, 150, 255)
}

class MapConverter:
    def __init__(self, afbeelding_pad: str, basis_naam: str = "WHEELTEC"):
        self.afbeelding_pad = afbeelding_pad
        self.basis_naam = basis_naam

    def converteer_afbeelding(self):
        img = Image.open(self.afbeelding_pad).convert('RGB')
        img_array = np.array(img).astype(float)
        
        # Masker maken met tolerantie
        mask = np.zeros(img_array.shape[:2], dtype=bool)
        for kleur in COLOR_LEGEND.values():
            dist = np.sqrt(np.sum((img_array - np.array(kleur))**2, axis=-1))
            mask |= (dist < 40)

        # Inpainting (meerdere passes voor grotere stippen)
        for _ in range(5): 
            for y in range(1, img_array.shape[0]-1):
                for x in range(1, img_array.shape[1]-1):
                    if mask[y, x]:
                        patch = img_array[y-1:y+2, x-1:x+2]
                        patch_mask = mask[y-1:y+2, x-1:x+2]
                        valid_pixels = patch[~patch_mask]
                        if valid_pixels.size > 0:
                            img_array[y, x] = np.mean(valid_pixels, axis=0)
                            mask[y, x] = False

        # Opslaan als PGM
        opgeschoonde_img = Image.fromarray(np.clip(img_array, 0, 255).astype(np.uint8)).convert('L')
        opgeschoonde_img.save(f"{self.basis_naam}.pgm")
        print(f"[INFO] PGM opgeslagen als '{self.basis_naam}.pgm'")

    def _schrijf_yaml(self, bestandsnaam, occ, free):
        """Hulpfunctie om YAML bestanden te genereren."""
        yaml_inhoud = (
            f"image: {self.basis_naam}.pgm\n"
            "mode: trinary\n"
            f"resolution: 0.05\n"
            "origin: [-43.6, -37.2, 0]\n"
            "negate: 0\n"
            f"occupied_thresh: {occ}\n"
            f"free_thresh: {free}\n"
        )
        with open(bestandsnaam, 'w') as f:
            f.write(yaml_inhoud)
        print(f"[INFO] YAML aangemaakt: {bestandsnaam}")

    def genereer_alle_yamls(self):
        # Hoofdkaart
        self._schrijf_yaml(f"{self.basis_naam}.yaml", 0.70, 0.70)
        # Keepout Working
        self._schrijf_yaml(f"{self.basis_naam}_KEEPOUT_WORKING.yaml", 0.15, 0.10)
        # Keepout Service
        self._schrijf_yaml(f"{self.basis_naam}_KEEPOUT_SERVICE.yaml", 0.35, 0.30)

if __name__ == "__main__":
    converter = MapConverter("WHEELTEC.png")
    converter.converteer_afbeelding()
    converter.genereer_alle_yamls()