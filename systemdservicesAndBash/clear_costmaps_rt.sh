#!/usr/bin/env bash

# Handmatig uitvoeren: ./src/systemdservicesAndBash/clear_costmaps_rt.sh
# Wis de costmaps alleen wanneer de robot veilig stilstaat.

echo "LET OP: dit wist ook echte dynamische obstakels en reset de keepoutfilters."
echo "Zorg dat de robot stilstaat, de noodstop ingedrukt is en er geen navigatieopdracht loopt."
read -r -p "Typ JA om beide costmaps te wissen: " confirmation || {
    echo "Geen bevestiging ontvangen; er is niets gewist." >&2
    exit 1
}
if [[ "$confirmation" != "JA" ]]; then
    echo "Niet bevestigd; er is niets gewist."
    exit 1
fi

if ! source /opt/ros/humble/setup.bash; then
    echo "FOUT: ROS Humble kon niet worden geladen." >&2
    exit 1
fi
if ! source /home/wheeltec/wheeltec_ros2_rt/install/setup.bash; then
    echo "FOUT: de RT-workspace kon niet worden geladen." >&2
    exit 1
fi

clear_costmap() {
    local name="$1"
    local service="$2"
    echo "$name costmap wissen..."
    if timeout 15s ros2 service call "$service" nav2_msgs/srv/ClearEntireCostmap "{}"; then
        echo "$name clear-service heeft geantwoord."
    else
        local status=$?
        echo "FOUT: $name clear mislukt of timeout (exit $status)." >&2
        echo "Niet blind herhalen: de aanvraag kan nog bezig zijn." >&2
        exit 1
    fi
}

clear_costmap "Globale" /global_costmap/clear_entirely_global_costmap
clear_costmap "Lokale" /local_costmap/clear_entirely_local_costmap

echo "Beide clear-services hebben geantwoord; dat bevestigt niet dat de filters hersteld zijn."
echo "Controleer vóór vertrek beide costmaps en het keepoutmasker in RViz en nav2.log."
echo "Obstakels buiten sensorzicht komen mogelijk niet terug; controleer de omgeving."
