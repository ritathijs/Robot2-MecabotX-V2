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
aanmeren, op basis van `docked_robot_pose` in
`mecabot_integration/config/robot_locations.yaml`. Omdat AMCL tijdens de slaap
kan driften, wordt bij een activeringsverzoek aan het laadstation deze pose
opnieuw verstuurd. De robot blijft inactief tot AMCL een nieuwe, overeenkomende
pose bevestigt en wordt dan automatisch actief. Een tweede druk op de
activeringsknop of een inactief-commando annuleert het wachtende verzoek.

Als de pose ontbreekt, de dockstatus wegvalt of AMCL niet tijdig bevestigt,
blijft de start geblokkeerd en vervalt het activeringsverzoek. De lokalisatie
wordt opnieuw geprobeerd, maar na herstel is een nieuw activeringsverzoek nodig.
De vaste pose moet overeenkomen met de werkelijke positie en richting van de
robot aan het station; de AMCL-controle bewijst dat niet. Test onder toezicht,
eerst met de wielen vrij van de grond en de noodstop bereikbaar.

Na het ophalen van de wijzigingen bouw je op de robot het RT-package opnieuw:

```bash
cd ~/wheeltec_ros2_rt
colcon build --packages-select mecabot_integration
source install/setup.bash
```

Herstart daarna de RT-nodes zodat ze de geïnstalleerde code gebruiken.

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

## RT-versie: status opvragen van de robot_state_manager

- Kerngegevens: `ros2 service call /get_robot_state wheeltec_robot_msg/srv/GetRobotState "{}"`
- Volledige interne status (o.a. `startup_location_resolved`, `startup_location_failed`, `docked`, `charging`, `dock_wake_pending`, `dock_pose_rejected`, `docking_attempts`, `last_screen`): `ros2 topic echo /robot_state_debug --once`
- Een geblokkeerde Behavior Tree (startlocatie niet bevestigd) wordt om de 10 s als waarschuwing gelogd.
- Bij activeren met `docked=True` wordt de laadpaalpose opnieuw aangevraagd, ook zonder laadstroom en bij een onbekende startlocatie. De robot blijft inactief tot AMCL die pose bevestigt. IR alleen is geen reden om de laadpaalpose bij activeren te gebruiken.
- Wordt de robot handmatig tegen het laadstation geduwd terwijl het dockingscherm staat, dan schakelt het scherm bij laadstroom naar het slaapscherm (`robot-charging`), zolang de robot inactief is.
- Bevestigt AMCL de laadpaalpose niet binnen de timeout (bijv. geduwde robot of AMCL-drift), dan wordt `dock_pose_rejected` gezet en volgt AprilTag-lokalisatie in plaats van eindeloos dezelfde controle te herhalen. Een nieuwe laadgebeurtenis (`ROBOT-CHARGING`) geeft de laadpaalpose weer een nieuwe kans.
