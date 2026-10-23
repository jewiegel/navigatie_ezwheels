# Echte robot (EZ-Wheel SWD)

Ziekenhuisnavigatie starten op de robot. Wat de nodes doen en hoe je routes maakt,
staat in [StartenSImulatie.md](StartenSImulatie.md); dit bestand gaat over wat op de
robot anders is.

> **Veiligheid:** begin met korte routes in een vrije ruimte, met iemand bij de
> noodstop. Bij een blokkade geeft de robot nooit op; alleen `cancel_mission` stopt hem.

## In het kort

```bash
docker exec -it ez-way-v2-ros2-iron-amr bash
export CYCLONEDDS_URI=file:///home/ezway/.config/ez-way/ez_way_bringup/cyclonedds_config.xml
export ROS_LOG_DIR=/tmp
cd <workspace> && source install/setup.bash

ros2 launch hospital_robot_bringup robot.launch.py \
  map:=/maps/<locatie>/map.yaml routes_file:=/maps/<locatie>/routes.yaml
```

Daarna: beginpositie zetten → route maken of kiezen → `start_route`.

## 1. Voorwaarden

| Onderdeel | Door | |
|---|---|---|
| Wieldriver, `/odom`, TF `odom → base_footprint → base_link` | EZ-Way-software | **moet draaien** |
| Lidar op `/laser_1/scan_filtered` | EZ-Way-software | **moet draaien** |
| EZ-Way-navigatie ("automatic mode") | EZ-Way-software | **moet uit** (zelfde Nav2-nodenamen) |
| MQTT-broker van de HMI | robot | draait al |

Gebruik in **elke** terminal dezelfde `CYCLONEDDS_URI` (en `ROS_DOMAIN_ID`) als de
EZ-Way-software. Controleren:

```bash
ros2 topic hz /laser_1/scan_filtered
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_link
```

## 2. Bouwen (na elke code-update)

```bash
cd <workspace>
colcon build --packages-select my_bot_interfaces hospital_robot_bt_nodes \
  hospital_robot_navigation hospital_robot_bringup my_bot
source install/setup.bash
```

- Neem `colcon_defaults.yaml` mee naar de workspace, of geef `--base-paths` op: de
  `hospital_robot_*`-pakketten en `my_bot_interfaces` staan in de map van `my_bot`.
- Bestanden verwijderd of hernoemd: schoon bouwen (`rm -rf build install log`).
- Foxglove vraagt `ros-iron-foxglove-bridge`; zonder: `foxglove:=false`.

## 3. Kaart en routes per locatie

Routes horen bij één kaart. Bewaar ze samen, met **absolute paden** en **buiten
`install/`**, op een map die een herstart van de container overleeft:

```
/maps/<locatie>/map.yaml   map.pgm   routes.yaml
```

- `image: map.pgm` in `map.yaml` (alleen de bestandsnaam).
- `routes.yaml` mag ontbreken: hij wordt aangemaakt bij de eerste `route_draft/save`.
- Een relatief pad hangt af van de map waar je start, en dan lees en schrijf je
  steeds een ander bestand. Het gebruikte pad staat in de log: `Routes uit /maps/...`.
- Nieuwe kaart: met SLAM, zie [StartenSImulatie.md](StartenSImulatie.md) stap 7 (zonder
  Gazebo en `use_sim_time`), en opslaan met
  `ros2 run nav2_map_server map_saver_cli -f /maps/<locatie>/map`. Een bestaande
  EZ-Way-kaart kan ook.

## 4. Starten

| Argument | Standaard | |
|---|---|---|
| `map` | — | **verplicht**, kaart van de locatie |
| `routes_file` | — | **verplicht**, absoluut pad |
| `timing_file` | `share/hospital_robot_bringup/config/timing.yaml` | blokkadetijden |
| `foxglove` | `true` | foxglove_bridge op poort 8765 |
| `hmi` | `true` | HMI-bridge (MQTT) |
| `hmi_broker` | `localhost` | MQTT-broker; anders bv. `10.1.0.2` |
| `hmi_start_route` | leeg | route voor de startknop op de HMI; leeg = knop doet niets |

Start AMCL, Nav2 met de ziekenhuisbomen (`config/nav2_params_robot.yaml`),
route_manager, mission_manager, turn_detector, signal_manager, hmi_bridge en
foxglove_bridge. Geen drivers, lidar of robot_state_publisher. Klaar bij
`lifecycle_manager_navigation: Managed nodes are active`; de hmi_bridge meldt
`MQTT verbonden met ...`.

### Beginpositie (altijd na het starten)

AMCL begint op (0, 0) van de kaart. Staat de robot ergens anders: Foxglove 3D-panel →
**Publish → Pose estimate** (`/initialpose`): klik waar hij staat en sleep in de
kijkrichting. Controleer dat de scan op de muren van de kaart valt.

## 5. Foxglove

Verbinden: **Foxglove WebSocket → `ws://<ip-van-de-robot>:8765`**. Handige layout:

| Panel | Instelling |
|---|---|
| **3D** | frame `map`; `/map`, `/laser_1/scan_filtered`, `/global_costmap/costmap`, `/plan`, `/route_draft/poses`, `/route_preview`, `/turn_markers`. Publish **Pose** op `/route_draft/add_pose` |
| **Service Call** | `/route_draft/save` `{"name": "gang_a"}`, `/route_draft/undo` `{}`, `/start_route` `{"route_name": "gang_a"}`, `/cancel_mission` `{}` |
| **Raw Messages** | `/mission_status`, `/indicators` |
| **Indicator** | `/navigation/blocked` (`.data`) |

## 6. Rijden

```bash
ros2 service call /start_route my_bot_interfaces/srv/StartRoute "{route_name: gang_a}"
ros2 service call /cancel_mission std_srvs/srv/Trigger
```

| `/mission_status` | Volgende stap |
|---|---|
| `IDLE` | `start_route` |
| `DRIVING <route>` | — |
| `ARRIVED <route>` | direct een nieuwe `start_route` |
| `WAITING_OPERATOR <route>` | Nav2 brak het doel af: kijken, dan `cancel_mission` |

- Bij een blokkade blijft `/mission_status` op `DRIVING` (de MissionManager luistert
  naar `bt_state`, dat niemand publiceert). Volg blokkades via
  `ros2 topic echo navigation/blocked --qos-durability transient_local`.
- Losse doelen: Foxglove **Publish → Pose** op `/goal_pose` (ander topic dan voor routes).
- Stoppen: **Ctrl+C**. Een `KeyboardInterrupt`-traceback bij het afsluiten is onschuldig.

## 7. Knipperlichten, buzzer en HMI

De signal_manager en hmi_bridge sturen de lampen en buzzer van de HMI via MQTT
(protocol: [StartenSImulatie.md](StartenSImulatie.md) stap 5). De bridge laat de
lampen zelf knipperen en speelt de buzzerpatronen af. Meekijken op de robot:

```bash
mosquitto_sub -v -t 'robot/#'
ros2 topic echo /hmi/mqtt_connected --qos-durability transient_local
```

## 8. Instellingen

- `hospital_robot_bringup/config/timing.yaml`: `progress_timeout_s` (10 s zonder 0,5 m
  voortgang → blokkade) en `blockade_wait_s` (10 s tussen twee checks).
- `config/nav2_params_robot.yaml`: o.a. `max_vel_x` (0,5 m/s), `xy_goal_tolerance` /
  `yaw_goal_tolerance` (0,25), `inflation_radius` (0,80 m), detectieafstand
  (`obstacle_max_range`, grootte local costmap).
- Na een wijziging: bouwen en de launch herstarten.

## Problemen oplossen

| Probleem | Oorzaak / oplossing |
|---|---|
| Log herhaalt `Timed out waiting for transform from base_link to odom` | Driver draait niet, of andere `CYCLONEDDS_URI`/`ROS_DOMAIN_ID` |
| Costmap leeg, robot rijdt tegen dingen | Geen `/laser_1/scan_filtered` |
| Dubbele nodes / vreemde Nav2-fouten | EZ-Way automatic mode draait nog |
| `missing required argument 'map'` of `'routes_file'` | Argument vergeten |
| Scan valt naast de muren | Beginpositie opnieuw zetten |
| `start_route`: `Onbekende route` | Naam staat niet in het `routes_file` van deze locatie |
| Opgeslagen routes weg na een build of herstart | Relatief `routes_file`, of bestand niet op een blijvende map; kijk in de log welk pad hij gebruikt |
| Robot blijft "geblokkeerd" bij één waypoint | Waypoint te dicht bij een muur: `cancel_mission`, waypoint verplaatsen |
| HMI-lampen reageren niet | `MQTT verbonden` niet in de log van de hmi_bridge: andere `hmi_broker` opgeven |
| Foxglove verbindt niet | `foxglove:=false`, bridge niet geïnstalleerd, of verkeerd IP |
