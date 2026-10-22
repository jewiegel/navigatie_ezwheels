# Opstarten op de echte robot

Hoe je de ziekenhuisnavigatie (Nav2 met de eigen behavior trees, route_manager en
MissionManagerNode) start op de EZ-Wheel SWD-robot, en wat je daarna allemaal kunt
doen. Voor de simulatie: zie [StartenSImulatie.md](StartenSImulatie.md).

> **Veiligheid:** test eerst met korte routes, in een vrije ruimte, met iemand bij de
> noodstop. De robot geeft bij een blokkade nooit op; hij blijft wachten tot de weg
> vrij is of tot je de missie annuleert.

## In het kort

```bash
# In de robot-container
export CYCLONEDDS_URI=file:///home/ezway/.config/ez-way/ez_way_bringup/cyclonedds_config.xml
export ROS_LOG_DIR=/tmp
source <workspace>/install/setup.bash

ros2 launch hospital_robot_bringup robot.launch.py \
  map:=/maps/ziekenhuis_a/map.yaml \
  routes_file:=/maps/ziekenhuis_a/routes.yaml
```

Daarna: beginpositie zetten (stap 5) → route maken of kiezen (stap 7) → `start_route`.

---

## 1. Wat er moet draaien en wat niet

| Onderdeel | Wie start het | Status |
|---|---|---|
| Wieldriver, `/odom`, TF `odom → base_footprint → base_link` | EZ-Wheel/EZ-Way-software | **moet draaien** |
| Lidar op `/laser_1/scan_filtered` | EZ-Wheel/EZ-Way-software | **moet draaien** |
| Navigatie van EZ-Way ("automatic mode": eigen Nav2/AMCL) | EZ-Way-software | **moet uit** |
| AMCL, map_server, Nav2 met de ziekenhuisbomen | `robot.launch.py` | |
| route_manager, MissionManagerNode | `robot.launch.py` | |
| turn_detector (bochten op `turn`) | `robot.launch.py` | |
| foxglove_bridge (poort 8765) | `robot.launch.py` | optioneel |

De navigatie van EZ-Way moet uit, omdat die Nav2-nodes met dezelfde namen start. Die
kunnen de eigen BT-plugins niet laden. De hardware-kant (driver, lidar, odometrie)
moet wel blijven draaien: `robot.launch.py` start geen drivers, geen lidar en geen
robot_state_publisher.

## 2. In de container komen

```bash
docker exec -it ez-way-v2-ros2-iron-amr bash
export CYCLONEDDS_URI=file:///home/ezway/.config/ez-way/ez_way_bringup/cyclonedds_config.xml
export ROS_LOG_DIR=/tmp
```

Gebruik in **elke** terminal dezelfde `CYCLONEDDS_URI` (en dezelfde `ROS_DOMAIN_ID`
als de EZ-Way-software). Anders zien de nodes de topics van de robot niet.

Controleer dat de robotsoftware de hardware levert:

```bash
ros2 topic hz /laser_1/scan_filtered      # lidar
ros2 topic hz /odom                       # odometrie
ros2 run tf2_ros tf2_echo odom base_link  # TF van de driver
```

## 3. Bouwen (na elke code-update)

In de workspace op de robot (`<workspace>` = de map met `src/`):

```bash
cd <workspace>
colcon build --packages-select my_bot_interfaces hospital_robot_bt_nodes \
  hospital_robot_navigation hospital_robot_bringup my_bot
source install/setup.bash
```

- Voor de Foxglove-koppeling moet `foxglove_bridge` geïnstalleerd zijn
  (`ros-iron-foxglove-bridge`). Zonder bridge: start met `foxglove:=false`.
- Op de ontwikkel-pc staan de pakketpaden in `colcon_defaults.yaml`. Neem dat
  bestand mee naar de robot, of geef `--base-paths` op (de `hospital_robot_*`-pakketten
  en `my_bot_interfaces` staan in de map van `my_bot`).

## 4. Kaarten en routes per locatie

Routes horen bij een kaart: coördinaten van gebouw A zijn in gebouw B zinloos.
Bewaar daarom per locatie de kaart en de routes samen, **buiten `install/`**:

```
/maps/
├── ziekenhuis_a/
│   ├── map.yaml
│   ├── map.pgm
│   └── routes.yaml
└── verpleeghuis_b/
    ├── map.yaml
    ├── map.pgm
    └── routes.yaml
```

- Gebruik in `map.yaml` alleen de bestandsnaam bij `image:` (`image: map.pgm`), dan
  blijft het werken als je de map verplaatst.
- `routes.yaml` mag bij een nieuwe locatie ontbreken: de route_manager begint dan
  met nul routes en maakt het bestand aan bij de eerste `route_draft/save`.

### Nieuwe kaart maken

Eén keer per locatie, met SLAM Toolbox in de mapping-modus. Rij de robot rustig
door alle gangen en kamers en sluit af bij het startpunt. Daarna opslaan:

```bash
ros2 run nav2_map_server map_saver_cli -f /maps/ziekenhuis_a/map
```

Zie [StartenSImulatie.md](StartenSImulatie.md), stap 5, voor de SLAM-instellingen.
Een bestaande kaart van EZ-Way kan ook.

## 5. Starten

```bash
ros2 launch hospital_robot_bringup robot.launch.py \
  map:=/maps/ziekenhuis_a/map.yaml \
  routes_file:=/maps/ziekenhuis_a/routes.yaml
```

### Argumenten

| Argument | Standaard | Betekenis |
|---|---|---|
| `map` | — **verplicht** | Kaart van de locatie (`.yaml`) |
| `routes_file` | — **verplicht** | Routes van de locatie, **absoluut pad** (bv. `/maps/ziekenhuis_a/routes.yaml`). Een relatief pad hangt af van de map waar je start, dus dan lees en schrijf je steeds een ander bestand. Het volledige pad staat bij het starten in de log: `Routes uit /maps/...` |
| `timing_file` | `share/hospital_robot_bringup/config/timing.yaml` | Tijden (zie stap 9) |
| `foxglove` | `true` | `false` = geen foxglove_bridge |

Klaar als de log `lifecycle_manager_navigation: Managed nodes are active` toont.

### Beginpositie zetten (altijd na het starten)

AMCL begint op positie (0, 0) van de kaart. Staat de robot ergens anders, zet dan
de beginpositie:

- **Foxglove:** 3D-panel → **Publish → Pose estimate** (topic `/initialpose`).
  Klik waar de robot staat en sleep in de kijkrichting.
- **Terminal** (als je de coördinaten weet):
  ```bash
  ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
    "{header: {frame_id: map}, pose: {pose: {position: {x: 1.0, y: 2.0}, orientation: {w: 1.0}},
      covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.07]}}"
  ```

Controleer daarna in Foxglove dat de lidar-punten (`/laser_1/scan_filtered`) op de
muren van de kaart vallen. Zo niet: beginpositie opnieuw zetten.

## 6. Foxglove

Verbinden: **Open connection → Foxglove WebSocket → `ws://<ip-van-de-robot>:8765`**.

Handige layout (sla hem op als layout, dan hoef je dit maar één keer te doen):

| Panel | Instelling |
|---|---|
| **3D** | Display frame `map`. Topics: `/map`, `/laser_1/scan_filtered`, `/global_costmap/costmap`, `/plan`, `/route_draft/poses` (concept-route), `/route_preview` (gekozen route). Publish: **Pose** op `/route_draft/add_pose` |
| **Service Call** | `/route_draft/save` — `{"name": "gang_a", "overwrite": false}` |
| **Service Call** | `/route_draft/undo` — `{}` |
| **Service Call** | `/start_route` — `{"route_name": "gang_a"}` |
| **Service Call** | `/cancel_mission` — `{}` |
| **Service Call** | `/get_route` — `{"name": "gang_a"}` |
| **Raw Messages** | `/mission_status` |
| **Indicator** | `/navigation/blocked`, veld `.data` |

## 7. Routes

### Een route maken door te klikken

1. Foxglove 3D-panel, **Publish → Pose** op `/route_draft/add_pose`: klik per
   waypoint de plek en sleep de rijrichting. De oranje pijlen (`/route_draft/poses`)
   tonen de concept-route.
2. Verkeerd geklikt: `/route_draft/undo` (laatste weg) of `/route_draft/clear` (alles weg).
3. Opslaan: `/route_draft/save` met `{"name": "gang_a"}`. Bestaat de naam al, gebruik
   dan een andere naam of `"overwrite": true`.

Vanuit de terminal:

```bash
ros2 service call /route_draft/undo  std_srvs/srv/Trigger
ros2 service call /route_draft/clear std_srvs/srv/Trigger
ros2 service call /route_draft/save  my_bot_interfaces/srv/SaveRoute "{name: gang_a}"
```

Tips:

- Leg waypoints minstens ~0,6 m van muren (robotstraal 0,22 m + inflation 0,55 m).
  Kan Nav2 een waypoint niet bereiken, dan blijft de robot op "geblokkeerd" staan.
- Laat de pijl wijzen naar het volgende waypoint, anders draait de robot daar eerst
  om zijn as.
- Een routenaam bestaat uit letters, cijfers, `_` en `-`. De waypoints heten
  `<naam>_1`, `<naam>_2`, …
- De concept-route staat alleen in het geheugen. Herstart je voor het opslaan, dan
  ben je hem kwijt.

### Een route bekijken

```bash
ros2 service call /get_route my_bot_interfaces/srv/GetRoute "{name: gang_a}"
```

Het antwoord bevat de waypoints. De route verschijnt ook als blauwe pijlen op
`/route_preview`. Een onbekende naam geeft een lijst van alle routes.

### Routes met de hand bewerken

In `routes.yaml` (wordt bij elke aanvraag opnieuw gelezen, dus een herstart is niet nodig):

```yaml
routes:
  gang_a:
    map_id: ziekenhuis_a
    waypoints:
    - {name: gang_a_1, x: 3.19, y: 0.21, yaw: 0.0}    # m, m, rad in het map-frame
    - {name: gang_a_2, x: 5.48, y: -0.6, yaw: -0.8}
```

Een fout in het bestand (bv. ontbrekende `yaw`) geeft bij `start_route` een
melding met route- en waypointnummer.

## 8. Rijden

### Een route starten en stoppen

```bash
ros2 service call /start_route my_bot_interfaces/srv/StartRoute "{route_name: gang_a}"
ros2 service call /cancel_mission std_srvs/srv/Trigger     # stoppen / resetten
ros2 topic echo /mission_status
```

| `/mission_status` | Betekenis | Volgende stap |
|---|---|---|
| `IDLE` | Wacht op een route | `start_route` |
| `DRIVING <route>` | Rijdt | — |
| `ARRIVED <route>` | Route klaar | Direct een nieuwe `start_route` |
| `WAITING_OPERATOR <route>` | Nav2 heeft het doel afgebroken | Kijken wat er is, dan `cancel_mission` |

> Tijdens een blokkade blijft `/mission_status` op `DRIVING` staan. De BT meldt de
> blokkade op `navigation/blocked`, maar de MissionManager luistert (nog) naar
> `bt_state`, en dat topic publiceert niemand. Volg blokkades dus via
> `navigation/blocked`.

De route wordt **waypoint voor waypoint** gereden. Elk waypoint wordt echt bereikt
(binnen 0,25 m en 0,25 rad), daarna rijdt de robot door naar het volgende.

### Blokkade

Kan de robot niet verder (geen pad, of 10 s geen voortgang), dan:

1. `navigation/blocked` → `true` (één keer)
2. Elke `blockade_wait_s` seconden: oude obstakels op afstand vergeten, opnieuw
   inmeten en proberen te plannen
3. Is de weg vrij: `navigation/blocked` → `false` (één keer) en de robot rijdt
   verder vanaf hetzelfde waypoint

De robot **geeft nooit op**. Alleen `cancel_mission` (of een nieuw doel) stopt hem.

```bash
ros2 topic echo navigation/blocked --qos-durability transient_local
```

### Losse doelen (zonder route)

Ook die gebruiken het ziekenhuisgedrag:

- **Foxglove:** **Publish → Pose** met topic `/goal_pose`. Let op: dat is een ander
  topic dan voor het klikken van routes.
- **Terminal:**
  ```bash
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: 3.0, y: 0.5}, orientation: {w: 1.0}}}}"
  ```

## 9. Instellingen

`hospital_robot_bringup/config/timing.yaml` (na wijzigen: opnieuw bouwen, of met
`timing_file:=` een eigen bestand meegeven):

```yaml
navigation:
  progress_timeout_s: 10.0   # zo lang geen 0,5 m voortgang → blokkade
  blockade_wait_s: 10.0      # tijd tussen twee checks of de weg weer vrij is
```

Nav2-instellingen van de robot staan in `config/nav2_params_robot.yaml`, o.a.:

- `max_vel_x` (0,5 m/s)
- `xy_goal_tolerance` en `yaw_goal_tolerance` (0,25), hoe precies een waypoint
  bereikt moet worden
- `inflation_radius` (0,55 m)

## 10. Stoppen

**Ctrl+C** in de terminal van `robot.launch.py`. Een `KeyboardInterrupt`-traceback
van de route_manager of de mission_manager bij het afsluiten is onschuldig. Zet
daarna eventueel de EZ-Way-software terug in automatic mode.

## 11. Problemen oplossen

| Probleem | Oorzaak / oplossing |
|---|---|
| Log herhaalt `Timed out waiting for transform from base_link to odom` | Driver/odometrie van de robot draait niet, of andere `CYCLONEDDS_URI`/`ROS_DOMAIN_ID` |
| Costmap blijft leeg, robot rijdt tegen dingen | Lidar-topic `/laser_1/scan_filtered` ontbreekt (`ros2 topic hz`) |
| Nodes starten dubbel / vreemde Nav2-fouten | EZ-Way automatic mode draait nog |
| `Included launch description missing required argument 'map'` | `map:=` vergeten |
| Robot staat verkeerd op de kaart, scan valt naast de muren | Beginpositie zetten (stap 5) |
| `start_route`: `Service /get_route niet beschikbaar` | route_manager draait niet (log van de launch bekijken) |
| `start_route`: `Onbekende route` | Naam staat niet in het `routes_file` van deze locatie |
| `start_route`: `Kan niet starten in WAITING_OPERATOR` | Eerst `cancel_mission` |
| Opgeslagen routes weg na een build of herstart | Relatief `routes_file` (zie waarschuwing in de log van de route_manager), of het bestand staat niet op een volume dat de container overleeft. Kijk in de log welk pad hij gebruikt: `Routes uit ...` en `opgeslagen ... in ...` |
| Geklikte pose komt niet in de concept-route | Display frame niet `map`, of verkeerd topic (`/route_draft/add_pose`) |
| `navigation/blocked = true` zonder obstakel | Robot schoot 10 s geen 0,5 m op (bv. vast op een drempel); controleer de log van `controller_server` |
| Robot blijft "geblokkeerd" bij één waypoint | Waypoint te dicht bij een muur of in een obstakel → `cancel_mission`, waypoint verplaatsen |
| Foxglove kan niet verbinden | `foxglove:=false` gestart, `foxglove_bridge` niet geïnstalleerd, of verkeerd IP/poort 8765 |
