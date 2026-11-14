# Mecabot RT - technische documentatie van de wijzigingen

## Reikwijdte

Dit document vat de belangrijkste wijzigingen op branch `mecabot_rt` samen
vanaf 1 september 2026 tot en met 13 november 2026. De eerste commits in deze
periode dateren van 9 september. In die periode zijn 137 commits gemaakt; de
samenvatting groepeert ze per technisch onderwerp en is geen volledige
commit-voor-commit changelog.

De RT-versie gebruikt vooral `mecabot_integration` en de RT-opstart- en
configuratiebestanden. Legacy-code staat nog in de repository, maar hoort niet
bij de normale RT-opstart. De aanwezigheid van een bestand of package in de
repository betekent op zichzelf niet dat het door de RT-stack wordt gestart.

## Samenvatting

De belangrijkste verandering is de overgang van losse, oudere robotprocessen
naar een RT-stack met een centrale robotstatusmanager, een RT Behavior Tree,
een RT-robotcontroller en eigen navigatie-, docking- en opstartscripts.
Daarnaast zijn de opstartlokalisatie en herstelpaden aangescherpt, is
handmatige bediening bij een onbekende locatie ondersteund en zijn Nav2,
keepout en camera-/lidarsensorconfiguraties doorontwikkeld.

## Architectuur en verantwoordelijkheden

| Onderdeel | Verantwoordelijkheid |
|---|---|
| `mecabot_integration` | RT Behavior Tree, C++-robotcontroller, robotstatusbeheer, quizverbinding, AprilTag-herstel en navigatiecoördinatie. |
| `rt_auto_recharger.py` | RT-docking- en laadproces, inclusief herstel en diagnose van laadstatussen. |
| `drive_mux` | Selecteert en begrenst de rijcommando's van autonome, handmatige en veiligheidsbronnen. |
| `wheeltec_nav2` / Nav2 | Padplanning, volgen van doelen, costmaps, sensorlayers en keepout-filter. |
| `startup_mecabot_rt.sh` | Controleert de RT-omgeving en start de RT-processen in afzonderlijke terminals. |
| `stop_mecabot_rt.sh` en `restart_mecabot_rt.sh` | Stoppen respectievelijk herstarten van de RT-processen. |
| `rt_simulation.launch.py` en `sim_robot.py` | Simuleren van RT-scenario's, waaronder bediening en laadgedrag. |

De integratielaunch start onder meer de robotcontroller, BT-node,
quiz-Socket.IO-bridge, AprilTag-node en RT-doelcoördinator. De bridge start
ook de robotstatusmanager. De RT-auto-recharger en de overige
robotinfrastructuur worden apart door het startupscript gestart.

## RT-logging en logfilter

De RT-startup schrijft de terminaluitvoer van de gelogde componenten naar
`/home/wheeltec/wheeltec_ros2_rt/logs`, bijvoorbeeld
`mecabot_integration.2026-11-14.log`. Elke regel krijgt de lokale systeemdatum
en -tijd met milliseconden: `[2026-11-14 10:30:00.123]`. De terminaluitvoer
zelf blijft ongewijzigd. Controleer de systeemklok en tijdzone op de robot.
Mux en bumper blijven uitgesloten omdat zij hun scherm continu herschrijven;
`ENABLE_LOGS=false` schakelt deze logging uit.

Er worden maximaal acht kalenderdagen bewaard, inclusief vandaag. Verouderde
dagbestanden van de RT-componenten worden bij startup en tijdens het draaien
opgeruimd; zonder draaiende RT-logging gebeurt de opruiming bij de volgende
startup. Bestaande ongedateerde logs blijven ongemoeid en vallen niet onder
deze bewaartermijn. Ook bijvoorbeeld `restart_mecabot_rt.log` en ROS-eigen
logs vallen buiten deze RT-componentlogging. Kopieer bij uitrol zowel het
startupscript als `rt_log_writer.py` naar dezelfde map.

Met het filter kan een basisnaam of een specifiek dagbestand worden gekozen:

```bash
python3 systemdservicesAndBash/filter_rt_log.py \
  /home/wheeltec/wheeltec_ros2_rt/logs/mecabot_integration.log \
  --datum 2026-11-14 --van 10:00 --tot 11:30

python3 systemdservicesAndBash/filter_rt_log.py \
  /home/wheeltec/wheeltec_ros2_rt/logs/mecabot_integration.log \
  --datum 2026-11-14 --van 10:00:00 --tot 11:30:59 \
  --zoek fout --zoek camera --uitvoer selectie.log

# Een oud, ongedateerd bestand expliciet selecteren:
python3 systemdservicesAndBash/filter_rt_log.py \
  /home/wheeltec/wheeltec_ros2_rt/logs/mecabot_integration.log --oud \
  --datum 2026-11-13 --van 13:45 --tot 14:00 --uitvoer oud-selectie.log
```

De grenzen zijn inclusief de opgegeven minuut of seconde en liggen op
dezelfde dag. Zonder zoekwoorden worden alle regels in het tijdvenster getoond;
met meerdere zoekwoorden volstaat één overeenkomst in het volledige bericht,
ongeacht hoofdletters. Een bericht bestaat uit een tijdgestempelde regel en
de daaropvolgende regels zonder tijdstempel. `--uitvoer`
maakt een nieuw bestand en overschrijft geen bestaand bestand. Met `--oud`
wordt het opgegeven bestand gelezen, ook als een nieuw dagbestand bestaat.
Het filter herkent ook ROS-regels zoals `[INFO] [1794573922.153623067]`
en toont daarbij een leesbaar tijdstempel vóór de oorspronkelijke regel.
De omzetting gebruikt de lokale tijdzone van de computer waarop het filter
draait en veronderstelt dat de ROS-tijd een Unix-systeemtijd was, geen
simulatietijd. Regels zonder herkenbaar tijdstempel worden meegenomen als de
dichtstbijzijnde tijdstempel ervoor of erna in het tijdvenster valt. Hun exacte
tijd is onbekend; bij door elkaar lopende processen is de koppeling aan een
bericht een benadering. Als het hele bestand geen herkenbare tijdstempels
bevat, worden de regels getoond en meldt het filter dat de tijdselectie niet
kon worden toegepast. De oorspronkelijke logbestanden worden niet gewijzigd.

## Functionele wijzigingen

### 1. RT-opstart en veiligheidscontroles

- Een aparte RT-startup, stop- en herstartprocedure is toegevoegd. De RT-start
  controleert onder meer de ROS-omgeving, vereiste packages, RT-installatie,
  camera, controller, laadconfiguratie en of er al ROS-nodes actief zijn.
- De startup wacht op vereiste topics en een geldige gemiddelde
  batterijmeting voordat hij de integratie als gestart beschouwt.
- De standaardstart is niet-interactief; een interactieve start kan met
  `--interactive`. De startup activeert de robot niet en laat de noodstop
  ongemoeid.
- Na een herstart start de robot inactief. **Inactief betekent niet
  noodzakelijk dat de robot fysiek stilstaat**: de BT kan bijvoorbeeld nog
  een laadactie uitvoeren. Houd de hardware-noodstop bereikbaar en test
  bewegingen onder toezicht.
- De oude algemene startupbestanden zijn uit de RT-opstart gehaald. De RT
  startup beëindigt niet stilzwijgend bestaande ROS-processen; die moeten
  eerst gericht gestopt worden.

### 2. Robotstatus, startlokalisatie en AprilTag-herstel

- De robotstatusmanager is uitgebreid met expliciete startstatus, dockstatus,
  laadstatus, schermstatus en diagnostiek.
- De startup wordt pas gereed wanneer de startlocatie is vastgesteld. AMCL
  moet na het publiceren van een beginpose een verse, overeenkomende
  lokalisatie publiceren; een oude AMCL-pose geldt niet als bevestiging.
- Bij een onbekende of mislukte startlocatie blijft autonome start
  geblokkeerd. AprilTag wordt gebruikt om lokalisatie opnieuw te proberen;
  bij mislukking wordt de robot in een veilige fout-/wachttoestand gehouden.
- Bij starten of activeren aan het laadstation kan de bekende laadpaalpose
  opnieuw naar AMCL worden gestuurd. Als die pose niet wordt bevestigd,
  schakelt herstel over op AprilTag in plaats van onbeperkt op dezelfde
  onbevestigde pose te vertrouwen.
- Na handmatig rijden wordt lokalisatie opnieuw aangevraagd. De robot wordt
  niet enkel op basis van een eerder bekende pose als correct gelokaliseerd
  beschouwd.
- De huidige diagnostiek is onder meer beschikbaar via `/robot_state_debug`
  en de service `/get_robot_state`; zie ook de RT-notities in
  [`README.md`](../README.md).

### 3. Behavior Tree en missielogica

De RT-BT beheert de hoofdlus van de robot:

1. Wacht tot de robotstatus en startlocatie gereed zijn.
2. Houd beweging tegen zolang de AprilTag-/lokalisatiestatus een
   hersteltoestand aangeeft.
3. Behandel een inactieve, ladende robot apart.
4. Controleer voor de autonome missie de actieve status en batterijstatus.
5. Rijd zo nodig eerst van het laadstation weg en naar de werkzone.
6. Geef een quizverzoek voorrang op de patrouille.
7. Voer de patrouille langs punten C, D en E uit; punt D bevat een
   AprilTag-controle en na punten C en E wordt gewacht.
8. Ga bij inactiviteit of onvoldoende batterij naar de laadprocedure.

Aanvullende wijzigingen voorkomen onder meer verouderde Nav2-resultaten,
dubbel verstuurde of direct herhaalde doelen, races rond patrouilleresets en
het herhaald insturen van een laadstationdoel wanneer de robot al gedockt is.
De RT-versie start bezoekersvolgen niet; de patrouille- en quizfunctionaliteit
blijven wel onderdeel van de BT.

### 4. Docking en laden

- `rt_auto_recharger.py` is toegevoegd als RT-eigen laad- en dockingproces.
- Dockstatus, laadstroom, IR-docking en AMCL-pose worden samen gebruikt om
  bepalen of de robot daadwerkelijk gedockt en geladen is.
- Er zijn time-outs, statusdiagnostiek en herstel toegevoegd voor onder meer
  ontbrekende services, late sensorgegevens en proces-/terminalproblemen.
- Tijdens IR-docking worden conflicterende Nav2-doelen onderdrukt; docking
  heeft een begrensde looptijd en het BT beheert herhaalde pogingen.
- Schermstatussen voor docking, laden en inactief zijn aangepast; tijdens
  admin-/loginweergaven wordt het eerder getoonde scherm onthouden en daarna
  hersteld.
- Paden naar laadpaal- en RT-runtimegegevens zijn naar de RT-workspace
  verplaatst. De configuratie staat onder meer in
  [`robot_locations.yaml`](../mecabot_integration/config/robot_locations.yaml).

### 5. Handmatige bediening en rijcommando's

- Handmatige bediening via het adminpaneel is aan de RT-integratie gekoppeld.
- In de toestanden voor een onbekende startlocatie of rij-fout kan handmatig
  rijden beschikbaar blijven, terwijl autonome rijbronnen door de mux worden
  geblokkeerd.
- De mux gebruikt de autonomieblokkering als aanvullende status; als die
  status verouderd raakt, wordt autonoom rijden geblokkeerd. Bij wegvallen
  van handmatige commando's wordt een nulcommando gepubliceerd.
- Handmatig rijden heft de hardware-noodstop en bestaande bumperprioriteit
  niet op. Dit is geen vervanging voor fysieke veiligheidsmaatregelen.

### 6. Navigatie, costmaps en keepout

- De lokale Nav2-costmap is ingericht voor lidar-obstakels en
  camerawaarnemingen. De camera-invoer is een PointCloud2-topic; de
  cameralaag is een `VoxelLayer`.
- Er zijn MPPI/controllerinstellingen afgestemd op vloeiender rijden en
  lokale obstacle-/inflationconfiguratie aangepast.
- Keepoutconfiguratie is uitgebreid met zachte maskers, waaronder PGM-kaarten,
  en de laagvolgorde is aangepast. Er is een herstelmechanisme dat de
  keepoutzone tijdelijk kan uitschakelen als de robot in een lethale zone
  vastzit. Diagnose-uitvoer voor keepout is standaard beperkt.
- Nav2 lifecycle-bond-time-outs zijn verhoogd en automatische reconnect na
  bond-uitval is uitgezet om ongewenste automatische herstarts te voorkomen.
- De RT-configuratie en de algemene flagship-configuratie wijzen niet
  noodzakelijk naar dezelfde navigatie-Behavior Tree. De startup geeft geen
  `params`-argument mee aan `wheeltec_nav2.launch.py`; die launch gebruikt
  standaard `param_flagship_mec_dl.yaml`. De aparte
  `RT_param_flagship_mec_dl.yaml` wordt alleen gebruikt als die expliciet
  wordt geselecteerd.
- Dit onderscheid is operationeel relevant: de RT-parameterfile gebruikt
  `navigate_with_replanning.xml`, terwijl de algemene parameterfile sinds
  november naar `navigate_to_pose_no_recovery.xml` verwijst. Ondanks de
  bestandsnaam bevat die laatste een `RecoveryNode` die lokale en globale
  costmaps kan wissen. Controleer bij uitrol altijd welke parameters en BT
  daadwerkelijk geladen zijn.
- Het handmatige script `clear_costmaps_rt.sh` vraagt expliciete bevestiging
  en waarschuwt dat wissen ook dynamische obstakels en keepoutfilters raakt.
  Een succesvolle serviceaanroep bewijst niet dat de omgeving of filters
  daarna veilig zijn; inspecteer de costmaps voordat de robot weer rijdt.

### 7. Lokalisatie, camera en lidar

- EKF- en TF-instellingen zijn tijdens tests aangepast, waaronder
  transformtijd-offset en de odometrie-/IMU-configuratie. De IMU-yaw-aanpassing
  is na waargenomen gyrodrift teruggedraaid; beschouw dit dus als
  testresultaten, niet als bewijs dat sensortiming volledig is opgelost.
- Astra-cameratransforms zijn als statische transforms gepubliceerd.
- De lidar-driver controleert pakketlengtes en laat ongeldige pakketten vallen
  om onveilige bufferverwerking te voorkomen.
- Sensorberichten kunnen nog worden verworpen als hun tijdstempel niet
  overeenkomt met de beschikbare TF-data. De RT-configuratie neemt die
  waarschuwingen niet weg; controleer sensortimestamps, TF en hostklokken
  tijdens hardwaretests.

### 8. Batterij en communicatie

- Batterijdrempels worden uit packageconfiguratie gelezen in plaats van uit
  de legacy BT-map. De standaardgrenzen zijn 21,0 V (laag) en 23,0 V (OK);
  controleer robotlokale aanpassingen voor uitrol.
- Batterijmetingen en statuslogs zijn begrensd/afgeremd om onnodige
  logbelasting te beperken.
- De quiz-Socket.IO-bridge probeert opnieuw verbinding te maken en stuurt
  statuswijzigingen alleen wanneer nodig om dubbele berichten en belasting
  op de Pi/server te verminderen.

## Configuratie en uitrol

Belangrijke RT-bestanden:

- [`startup_mecabot_rt.sh`](../systemdservicesAndBash/startup_mecabot_rt.sh)
- [`stop_mecabot_rt.sh`](../systemdservicesAndBash/stop_mecabot_rt.sh)
- [`restart_mecabot_rt.sh`](../systemdservicesAndBash/restart_mecabot_rt.sh)
- [`mecabot_integration.launch.py`](../mecabot_integration/launch/mecabot_integration.launch.py)
- [`behavior_tree.xml`](../mecabot_integration/config/behavior_tree.xml)
- [`robot_locations.yaml`](../mecabot_integration/config/robot_locations.yaml)
- [`RT_param_flagship_mec_dl.yaml`](../wheeltec_robot_nav2/param/wheeltec_params/RT_param_flagship_mec_dl.yaml)

De opstart verwacht op de Pi de workspace `/home/wheeltec/wheeltec_ros2_rt`,
ROS 2 Humble, de geïnstalleerde RT-packages en runtimebestanden in de
RT-workspace. Het startupscript controleert dat de RT-eigen packages uit die
workspace geladen worden. Pas bij uitrol daarom niet alleen de bronbestanden
aan: bouw de betrokken packages opnieuw, source de RT-installatie en herstart
de RT-stack.

Voor RT-specifieke wijzigingen is de gerichte testset:

```bash
python3 -m unittest discover -s mecabot_integration/test
```

Aanvullende batterijconfiguratietests staan in
`autocharge_batcheck/test`. Deze tests controleren softwarelogica, maar
vervangen geen fysieke tests van lokalisatie, docking, noodstop,
bumperprioriteit, lidar/camera en costmaps. Voer bewegingstests uit onder
toezicht met de noodstop bereikbaar.

## Belangrijkste aandachtspunten

1. Verifieer vóór hardwaregebruik de werkelijk geladen Nav2-parameterfile en
   Behavior Tree; de naam `RT_param_flagship_mec_dl.yaml` betekent niet dat
   die automatisch gekozen wordt.
2. Een robot die als inactief wordt gerapporteerd, kan nog BT- of
   dockingbeweging uitvoeren. Inactief is geen veiligheidsstop.
3. Verifieer laadpaalpose, werkzone-/patrouilleposes en AprilTag-ankers op de
   fysieke robot; AMCL-bevestiging alleen bewijst niet dat de opgegeven pose
   fysiek juist is.
4. De RT-branch is actief in ontwikkeling. De commitgeschiedenis documenteert
   iteraties en herstelacties, niet een formele veiligheidsvalidatie of
   vrijgave voor autonoom gebruik.

