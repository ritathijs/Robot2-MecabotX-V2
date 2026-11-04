Dit bestand werdt aangemaakt door Matthijs Mondelaers op 11/02/2025

Zacht keepout-masker, guard en diagnose
---------------------------------------
- Masker genereren: quick_map_change/GenerateSoftKeepout.py (schrijft
  wheeltec_robot_nav2/map/WHEELTEC_KEEPOUT_WORKING_SOFT.png/.yaml, mode 'scale').
  Opnieuw draaien na elke wijziging aan WORKING/SERVICE; --inner en --outer
  regelen de breedte van de helling binnen en de ring buiten de zone.
- De node laadt het SOFT-masker als het bestaat, en valt terug op het binaire
  WORKING-masker als het ontbreekt of niet laadt.
- Zone AAN wordt uitgesteld zolang er een lethale zone-cel binnen guard_radius
  (0.49 m) van de robot ligt. Zone UIT is altijd direct.
- Diagnose in de log: zoek op "[Keepout-diagnose]" (masker zacht/binair, en
  meldingen als de voetafdruk in lethale cellen staat). Uitzetten kan met de
  parameters diagnostics_enabled en guard_enabled.
 
