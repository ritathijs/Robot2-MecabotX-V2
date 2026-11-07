# Robot2-MecabotX

## RT-versie: bezoekersdetectie

De RT-opstart start de people tracker niet. De behavior tree rijdt niet naar
bezoekers en voert geen zoekrotatie voor bezoekers uit. Patrouille, quiz,
AprilTag-poseherstel, camera en camerascan blijven in gebruik. `/tracking_enable`
blijft `false`, ook als een tracker afzonderlijk wordt gestart.

De eerdere versie is terug te vinden in commit
`4a91bb5a13f50d318ba832c456b8837ea3da0b50` op `mecabot_rt`. Herstel de
opstart, behavior tree, BT-registraties en trackingstatus samen als de functie
later terugkomt; controleer dan ook de veiligheid van de bewegingscommando's.

De gerichte controles kunnen zonder ROS worden uitgevoerd met
`python3 -m unittest discover -s mecabot_integration/test`.

Deze wijziging lost een Nav2-crash of het ontbreken van een onafhankelijke
bewegingswatchdog niet op. Test de robot onder toezicht met de noodstop bereikbaar.

## RT-versie: opstartlokalisatie met AprilTag

Bij een niet-gedockte opstart wordt de AprilTag-pose na detectie opnieuw naar
`/initialpose` gestuurd totdat een latere `/amcl_pose` in `map` overeenkomt:
maximaal 0,5 m positieverschil, 0,5 rad hoekverschil en covarianties onder 0,1.
Zonder bevestiging binnen 30 seconden blijft de start geblokkeerd en wordt
een fout gelogd. De bestaande opstartlogica probeert na een mislukking opnieuw;
bij een latere geslaagde werkzonestart kan de robot automatisch actief worden.
De overeenkomende pose is een controle, geen expliciete ontvangstbevestiging
van AMCL of garantie dat de fysieke positie correct is.

Dezelfde AMCL-controle geldt bij opstart aan het laadstation en na opnieuw
aanmeren, op basis van de vaste laadstationpose. Tijdens deze controle blijft
de robot inactief; na bevestiging is opnieuw activeren nodig om te vertrekken.
Als de pose ontbreekt, de dockstatus wegvalt of AMCL niet bevestigt, blijft de
start geblokkeerd. Ook hier volgt na een mislukking een nieuwe poging.
Test onder toezicht, eerst met de wielen vrij van de grond.

## RT-versie: slaapscherm bij laden

Als de robot inactief is en laadstroom detecteert, wordt een achtergebleven
`robot-startup`-scherm vervangen door `robot-charging` ("Hugoo slaapt").
Tijdens admin of login wordt het scherm onthouden en bij het sluiten hersteld.
Andere schermen, waaronder terugkeer- en foutschermen, worden niet vervangen.
Dit verandert de docking- of bewegingslogica niet.

## RT-versie: batterijspanningsgrenzen

De batterijcontrole leest `config/spanningsniveaus.json` uit het geïnstalleerde
`autocharge_batcheck`-package, niet meer uit de legacy-map `mecabot_bt`. De
standaardgrenzen blijven 21,0 V (laag) en 23,0 V (OK). Pas de waarden zo nodig
aan in `autocharge_batcheck/config/spanningsniveaus.json` en bouw het package
opnieuw; bij een gewone installatie wordt het configuratiebestand gekopieerd.
Vergelijk eerst de oude waarden op de robot als die lokaal aangepast zijn.

Na het ophalen van de wijzigingen:

```bash
cd ~/wheeltec_ros2_rt
colcon build --packages-select autocharge_batcheck
source install/setup.bash
```

De RT-opstart vereist dit package uit de RT-workspace. Controleer in het
batterijlog dat de gewenste grenzen geladen zijn. Het pad voor `batstatus.txt`
is hiermee niet gewijzigd.
