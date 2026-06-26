from PIL import Image
import numpy as np
import math

def calculatePixelCoordinates(imagePath):
    locations = {
        "Laadstation": (255, 0, 0),
        "Werkpunt": (0, 255, 0),
        "Quizlocatie": (0, 0, 255),
        "Referentie": (255, 0, 255)
    }

    img = Image.open(imagePath).convert('RGB')
    img_array = np.array(img)

    pixelCoordinates = {}

    for name, color in locations.items():
        matches = np.all(img_array == color, axis=-1)
        y_coords, x_coords = np.where(matches)
        pixelCoordinates[name] = list(zip(x_coords, y_coords))

    return pixelCoordinates

def calculateResolution(pixelCoordinates, refDistance):
    refPixels = pixelCoordinates.get("Referentie", [])
    
    if len(refPixels) != 2:
        raise ValueError(f"Er moeten exact 2 referentiepixels gevonden zijn. Er zijn er {len(refPixels)} gevonden.")
        
    ref1, ref2 = refPixels[0], refPixels[1]
    
    dx = ref2[0] - ref1[0]
    dy = ref2[1] - ref1[1]
    
    pixelDistance = math.sqrt(dx**2 + dy**2)

    if pixelDistance == 0:
        raise ValueError("De twee referentiepixels bevinden zich op exact dezelfde locatie.")

    resolution = refDistance / pixelDistance
    return resolution

def convertPixelToReal(pixelCoordinates, resolution):       

    realCoordinates = {}
    
    for location, coordinatesList in pixelCoordinates.items():
        realCoordinates[location] = []
        
        for x, y in coordinatesList:
            abs_x = x * resolution
            abs_y = y * resolution
            realCoordinates[location].append((abs_x, abs_y))
            
    return realCoordinates

def generateMapYaml(realCoordinates, resolution, imagePath="WHEELTEC.pgm", savePath="wheeltec.yaml"):
    yaml_tekst = f"image: {imagePath}\n"
    yaml_tekst += "mode: trinary\n"
    yaml_tekst += f"resolution: {resolution:.4f}\n"
    yaml_tekst += "origin: [0.0, 0.0, 0]\n"
    yaml_tekst += "negate: 0\n"
    yaml_tekst += "occupied_thresh: 0.70\n"
    yaml_tekst += "free_thresh: 0.70\n\n"
            
    writeresult(savePath, yaml_tekst)
    print(f"-> YAML succesvol opgeslagen in '{savePath}'")


def writeresult(path, tekst):
    with open(path, 'w') as file:
        file.write(tekst)
    file.close()

if __name__ == "__main__":
    print("Geef de naam van de afbeelding op:")
    bronafbeelding = f"{input()}.png"
    doelbestand = f"{input()}.txt"

    # Stap 1: Bereken de pixelcoördinaten van de locaties
    pixelCoordinates = calculatePixelCoordinates(bronafbeelding)

    # Stap 2: Vraag de gebruiker om de afstand tussen de referentiepunten
    print("Geef de afstand tussen de referentiepunten op (in meters):")
    refDistance = float(input())

    # Stap 3: Bereken de resolutie
    resolution = calculateResolution(pixelCoordinates, refDistance)

    # Stap 4: Converteer pixelcoördinaten naar echte coördinaten
    realCoordinates = convertPixelToReal(pixelCoordinates, resolution)



    

