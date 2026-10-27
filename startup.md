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

Met het webdashboard (stap 6) erbij, in een tweede terminal in dezelfde container:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml   # ws://<robot>:9090
```

en op je laptop `cd dashboard && npm run dev`, dan
`http://localhost:5173/?ros=ws://<ip-van-de-robot>:9090` openen.

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

## 6. Webdashboard

Een website ([dashboard/](dashboard/)) voor de kaart, routes, de huidige missie en de
signalen. Hij praat via **rosbridge** (poort 9090) met de robot, naast Foxglove.

### Eenmalig

- **rosbridge op de robot.** De EZ-Way-image heeft het niet. Installeer
  `ros-iron-rosbridge-suite` in de container; een losse `apt install` is na een herstart
  van de container weg, dus laat het in de image opnemen of herhaal het:
  ```bash
  apt-get update && apt-get install -y ros-iron-rosbridge-suite
  ```
- **Node.js (≥ 20) op je laptop**, dan in `dashboard/`: `npm install`.
- **Bouwen met de nieuwe services** (`list_routes`, `set_route`, `delete_route`): de
  bouwopdracht van stap 2 neemt `my_bot_interfaces` en `my_bot` al mee.

### Starten (elke keer)

1. **Robot:** `robot.launch.py` zoals in stap 4. route_manager en mission_manager moeten
   draaien; zonder krijg je in het dashboard "Service … niet beschikbaar".
2. **rosbridge**, in een tweede terminal in de container (zelfde `CYCLONEDDS_URI`, en
   eerst `source install/setup.bash`, anders kent rosbridge de eigen msg/srv-types niet):
   ```bash
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml
   ```
   Klaar bij `Rosbridge WebSocket server started on port 9090`.
3. **Dashboard**, op je laptop:
   ```bash
   cd dashboard && npm run dev
   ```
   Open `http://localhost:5173/?ros=ws://<ip-van-de-robot>:9090`. Rechtsboven moet
   **Verbonden** staan, daarna verschijnt de kaart.

   Zonder Node op de laptop: bouw één keer (`npm run build`), kopieer `dashboard/dist/`
   naar de robot en serveer het daar met `python3 -m http.server 8080 --directory dist`;
   open dan `http://<ip-van-de-robot>:8080` (de pagina verbindt vanzelf met poort 9090
   van dezelfde host).

### Gebruik

| Paneel | Wat |
|---|---|
| **Kaart** | slepen = pannen, scrollen = zoomen; klik = waypoint, sleep = verplaatsen, rechtsklik = verwijderen |
| **Opgeslagen routes** | kiezen (laadt op de kaart), opslaan, verwijderen, **Start route** / **Stop missie** via mission_manager |
| **Huidige missie** | status, voortgang (`x van n waypoints`), blokkade, afstand en rijtijd |
| **Signalen** | knipperlichten, buzzer, komende bocht, HMI-verbinding |
| **Status** | verbinding, snelheid, batterij, diagnostics |

- Start routes met **Start route** in "Opgeslagen routes". De knop **Verstuur naar
  Nav2** in de route-editor stuurt direct naar Nav2, buiten mission_manager om: lampen,
  HMI en `cancel_mission` weten daar niets van, en een lopende route wordt afgebroken.
- rosbridge heeft **geen login**: iedereen die poort 9090 bereikt kan de robot
  besturen. Alleen op een afgeschermd netwerk gebruiken.

### Simulatie

Zelfde stappen in de `ros2_iron`-container (rosbridge staat in `~/ros2_docker/Dockerfile`;
na `docker build -t mijn_ros2_iron ~/ros2_docker` zit het vast in de image):
`sim.launch.py` in plaats van `robot.launch.py`, en op de host `npm run dev` met
gewoon `http://localhost:5173` (de container draait met `--net=host`).

## 7. Rijden

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

## 8. Knipperlichten, buzzer en HMI

De signal_manager en hmi_bridge sturen de lampen en buzzer van de HMI via MQTT
(protocol: [StartenSImulatie.md](StartenSImulatie.md) stap 5). De bridge laat de
lampen zelf knipperen en speelt de buzzerpatronen af. Meekijken op de robot:

```bash
mosquitto_sub -v -t 'robot/#'
ros2 topic echo /hmi/mqtt_connected --qos-durability transient_local
```

## 9. Instellingen

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
| Dashboard blijft op "Verbinden…" | rosbridge draait niet, verkeerd IP in `?ros=`, of poort 9090 dicht |
| `package 'rosbridge_server' not found` | rosbridge niet geïnstalleerd in deze container (stap 6, Eenmalig), of shell van vóór de installatie: opnieuw `source /opt/ros/iron/setup.bash` |
| Dashboard verbonden, maar geen kaart | map_server nog niet actief; wacht of herlaad de pagina |
| "Service /list_routes niet beschikbaar" | route_manager draait niet, of is van vóór de laatste build: launch herstarten |
| Fout over onbekend type `my_bot_interfaces/...` | rosbridge gestart zonder `source install/setup.bash` |
