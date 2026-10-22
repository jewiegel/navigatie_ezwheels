# Simulatie starten en testen

Stappenplan om de ziekenhuisrobot in Gazebo te draaien met Nav2 en de eigen
behavior tree (`hospital_robot_*`-pakketten), een kaart te maken en de eigen
nodes te testen. Alles draait in de **ROS 2 Iron Docker container**.

## Snelstart

In een container-terminal (zie stap 1), na het bouwen (stap 2):

```bash
ros2 launch hospital_robot_bringup sim.launch.py
```

Dit start Gazebo (ziekenhuiswereld), RViz, AMCL op `/ros2_ws/maps/ziekenhuis.yaml`
en Nav2 met de behavior tree van de ziekenhuisrobot. Rijden: zie stap 3.

| # | Terminal | Commando |
|---|---|---|
| 1 | Simulatie + Nav2 + RViz | `ros2 launch hospital_robot_bringup sim.launch.py` |
| 2 | Blokkade-melding volgen | `ros2 topic echo navigation/blocked --qos-durability transient_local` |
| 3 | Doel sturen (optioneel) | `ros2 action send_goal /navigate_through_poses ...` |
| 4 | Besturing (optioneel) | controller of toetsenbord |

---

## 1. In de Docker container komen

Het image en het startscript staan in `~/ros2_docker/`:

- `Dockerfile` — image `mijn_ros2_iron` (Iron + Gazebo, Nav2, SLAM Toolbox,
  joy/teleop, twist_mux, paho-mqtt)
- `start.sh` — start de container, of opent een extra shell als hij al draait

### Eenmalig: image bouwen

Opnieuw doen na elke wijziging in de `Dockerfile`:

```bash
cd ~/ros2_docker
docker build -t mijn_ros2_iron .
```

> Software die je los met `apt install` in de container zet, is weg zodra de
> container stopt (`--rm`). Zet vaste packages altijd in de `Dockerfile`.

### Container starten / extra terminal openen

```bash
~/ros2_docker/start.sh
```

- Eerste keer: start de container `ros2_iron`.
- Daarna: elke nieuwe host-terminal waarin je `start.sh` draait, is een extra
  shell in dezelfde container.

In de container:

- `~/ros2_ws` van de host is beschikbaar als **`/ros2_ws`**
- `/opt/ros/iron/setup.bash` en `/ros2_ws/install/setup.bash` worden automatisch
  gesourced (via `~/.bashrc`). Na een build in een al open terminal: zelf
  `source /ros2_ws/install/setup.bash`.
- `/dev/input` is doorgegeven, dus de controller werkt in de container

Gewijzigde opties in `start.sh` (bv. devices) werken pas na een herstart van de
container: sluit alle shells of doe `docker stop ros2_iron`, en start opnieuw.

### Eenmalig per container: ros2 daemon resetten

De container gebruikt `--net=host`. Draait er op de host ook een ros2 daemon
(bv. van Humble), dan krijg je bij `ros2 topic ...` de fout
`RuntimeError: !rclpy.ok()` of hangt het commando. Oplossing:

```bash
ros2 daemon stop        # in de container én op de host
```

Of gebruik `--no-daemon`, bv. `ros2 topic list --no-daemon`.

---

## 2. Bouwen

In de container:

```bash
cd /ros2_ws
colcon build
source install/setup.bash
```

De pakketten:

| Pakket | Inhoud |
|---|---|
| `my_bot` | robotmodel, Gazebo-wereld, Nav2-config (`config/nav2_params.yaml`), launch-files, eigen Python-nodes |
| `my_bot_interfaces` | messages/services voor MissionManagerNode en route_manager (`StartRoute`, `GetRoute`, `SaveRoute`) |
| `hospital_robot_bt_nodes` | eigen BT-plugins in C++ (`PublishBlocked`, `IsPathBlocked`, `GetNextWaypoint`, `PopWaypoint`) |
| `hospital_robot_navigation` | de behavior tree-sjablonen (`hospital_navigate.xml.in`, `hospital_navigate_to_pose.xml.in`) |
| `hospital_robot_bringup` | launch-files en `config/timing.yaml` |

- De paden staan in `/ros2_ws/colcon_defaults.yaml`, die colcon automatisch
  leest als je vanuit `/ros2_ws` bouwt of test. De `hospital_robot_*`-pakketten en
  `my_bot_interfaces` staan ín de map van `my_bot`, en colcon zoekt niet verder in
  een map die zelf al een package is — daarom staan ze er apart in.
- De oude `src/my_bot/` valt buiten die paden, dus geen *Duplicate package names*.
- Eén package bouwen: `colcon build --packages-select hospital_robot_bringup`.
- Opnieuw bouwen is nodig na wijzigingen in launch-, config-, XML- of
  Python-bestanden (die worden naar `install/` gekopieerd).

> De container draait als root, dus `build/` en `install/` worden eigendom van
> root. Bouw daarom altijd in de container. Wil je toch op de host bouwen:
> `sudo chown -R $USER:$USER ~/ros2_ws/build ~/ros2_ws/install ~/ros2_ws/log`.

---

## 3. Simulatie met de ziekenhuis-behavior tree

> De simulatierobot gebruikt dezelfde topics als de echte robot: lidar op
> `/laser_1/scan_filtered`, odometrie op `/odom`, besturing via `/cmd_vel`, frames
> `odom → base_footprint → base_link`. Verschil tussen `config/nav2_params.yaml`
> (simulatie) en `config/nav2_params_robot.yaml`: alleen `inflation_radius`
> (0,55 / 0,80 m) en `use_sim_time`.

```bash
ros2 launch hospital_robot_bringup sim.launch.py
```

| Argument | Standaard |
|---|---|
| `map` | `/ros2_ws/maps/ziekenhuis.yaml` |
| `timing_file` | `share/hospital_robot_bringup/config/timing.yaml` |
| `foxglove` | `true` — foxglove_bridge op poort 8765 (`ws://localhost:8765`) |

Wat er gebeurt:

1. Gazebo met de ziekenhuiswereld, de robot en de bridge (`my_bot/launch_sim.launch.py`).
2. RViz met `my_bot/config/setup.rviz` (inclusief het Nav2-paneel).
3. Na 8 s (Gazebo moet eerst `/clock` en `/tf` publiceren):
   - AMCL op de ziekenhuiskaart. De beginpositie (0, 0) wordt automatisch gezet
     (`set_initial_pose` in `nav2_params.yaml`), dus **2D Pose Estimate is niet
     nodig** zolang de robot op zijn spawnplek staat.
   - Nav2 via `hospital_robot_bringup/navigation.launch.py`: die vult de
     sjablonen in met de tijden uit `timing.yaml` en zet ze als boom voor
     `NavigateThroughPoses` (`hospital_navigate.xml.in`) en `NavigateToPose`
     (`hospital_navigate_to_pose.xml.in`), voegt de eigen BT-plugins toe aan
     `plugin_lib_names` en zet de progress-timeout.

Klaar als in de log `lifecycle_manager_navigation: Managed nodes are active` staat
(± 15 s).

### Wat de behavior tree doet

- De route wordt **waypoint voor waypoint** gereden: plan naar het volgende
  waypoint, rijd tot het bereikt is (binnen `xy_goal_tolerance` 0,25 m en
  `yaw_goal_tolerance` 0,25 rad, zie `general_goal_checker` in
  `config/nav2_params.yaml`), draai naar de richting van het waypoint en ga door
  naar het volgende.
- Lukt plannen of rijden niet (geen pad, of `progress_timeout_s` lang geen 0,5 m
  voortgang): **blokkadeprocedure**: `navigation/blocked = true` (één keer), daarna steeds
  costmaps wissen behalve rond de robot → `blockade_wait_s` wachten → proberen te
  plannen naar het huidige waypoint. Pas als dat lukt: `navigation/blocked = false`
  (één keer) en verder rijden. De robot **geeft nooit op**; alleen een voltooide
  route of annuleren (Cancel) beëindigt het doel.

Tijden aanpassen in `hospital_robot_bringup/config/timing.yaml` (daarna opnieuw bouwen):

```yaml
navigation:
  progress_timeout_s: 10.0   # zo lang geen voortgang → FollowPath faalt
  blockade_wait_s: 10.0      # blokkade: wachttijd tussen twee checks of het pad vrij is
```

### Rijden vanuit RViz

Alle manieren gebruiken het ziekenhuisgedrag (blokkadeprocedure, nooit opgeven):

| RViz | Nav2-actie | Boom |
|---|---|---|
| **2D Goal Pose** (één doel) | `NavigateToPose` | `hospital_navigate_to_pose.xml.in` |
| Nav2-paneel → **Start Nav Through Poses** (route) | `NavigateThroughPoses` | `hospital_navigate.xml.in` |
| Nav2-paneel → **Start Waypoint Following** | per waypoint een `NavigateToPose` | `hospital_navigate_to_pose.xml.in` |

Een route met het **Navigation 2**-paneel:

1. Klik in het paneel op **Waypoint / Nav Through Poses Mode**.
2. Kies **Nav2 Goal** in de werkbalk en klik de waypoints op de kaart
   (klik = plek, slepen = richting). Laat de pijl wijzen naar het volgende
   waypoint, anders draait de robot daar onnodig.
3. Klik **Start Nav Through Poses**.

Met **Cancel** in het paneel stop je het doel.

### Een route rijden vanaf de commandoregel

```bash
ros2 action send_goal /navigate_through_poses nav2_msgs/action/NavigateThroughPoses \
  "{poses: [
    {header: {frame_id: map}, pose: {position: {x: 3.19, y: 0.21}, orientation: {w: 1.0}}},
    {header: {frame_id: map}, pose: {position: {x: 5.48, y: -0.6}, orientation: {w: 1.0}}}
  ]}" --feedback
```

In de feedback telt `number_of_poses_remaining` af per bereikt waypoint.

### Blokkade testen

```bash
ros2 topic echo navigation/blocked --qos-durability transient_local
```

Zet in Gazebo een obstakel zó dat er **geen pad meer omheen** is (bv. een gang
helemaal dicht). Kan de planner eromheen, dan rijdt de robot om en is er geen
blokkade. Verwacht: één keer `data: true`, dat blijft zo zolang het obstakel er
staat (elke `blockade_wait_s` een nieuwe check, geen tussentijdse `false`). Haal je
het obstakel weg, dan komt binnen ± `blockade_wait_s` één keer `data: false` en
rijdt de robot zijn route verder af.

### Alleen de simulatie, of met de standaardboom van Nav2

```bash
ros2 launch my_bot launch_sim.launch.py              # alleen Gazebo + robot + bridge
ros2 launch my_bot sim_nav.launch.py                 # Gazebo + RViz + AMCL + Nav2 met standaardbomen
ros2 launch my_bot sim_nav.launch.py slam:=true      # idem, met SLAM i.p.v. AMCL
```

Controleren of de simulatie draait:

```bash
ros2 topic list                           # /laser_1/scan_filtered, /odom, /clock, /tf moeten er staan
ros2 topic hz /laser_1/scan_filtered      # lidar publiceert?
```

---

## 4. Rijden met controller of toetsenbord

Beide publiceren op `/cmd_vel`; de bridge stuurt dat door naar Gazebo. Niet
tegelijk met een Nav2-doel gebruiken.

### Xbox controller

```bash
ls /dev/input/js0          # controller zichtbaar in de container?
ros2 launch teleop_twist_joy teleop-launch.py joy_config:=xbox
```

- **X ingedrukt houden** (knop-index 2) = rijden (dodemansknop)
- **Linkerstick** = vooruit/achteruit en draaien
- **RB** (index 5) erbij = turbo

Reageert de robot niet, kijk dan welke knop welke index heeft:

```bash
ros2 topic echo /joy       # druk op knoppen en kijk welke index verandert
ros2 topic echo /cmd_vel   # komen er snelheden binnen?
```

Andere dodemansknop kiezen (bv. LB = 9):

```bash
ros2 launch teleop_twist_joy teleop-launch.py joy_config:=xbox --ros-args -p enable_button:=9
```

### Toetsenbord

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

| Toets | Actie |
|---|---|
| `i` / `,` | vooruit / achteruit |
| `j` / `l` | links / rechts draaien |
| `k` | stoppen |
| `q` / `z` | sneller / langzamer |

Het terminalvenster moet focus hebben, anders komen de toetsen niet aan.

> Gebruik niet `cmd_vel_keyboard` van `twist_mux`: de uitgang daarvan
> (`/diff_cont/cmd_vel_unstamped`) wordt in de simulatie door niets gelezen.

---

## 5. Kaart maken met SLAM

Alleen nodig voor een nieuwe kaart; `ziekenhuis.yaml` bestaat al.

Zet eerst in `config/mapper_params_online_async.yaml` de modus op **mapping**
en bouw opnieuw:

```yaml
    mode: mapping #localization of mapping
```

```bash
ros2 launch my_bot launch_sim.launch.py                         # terminal 1
ros2 launch my_bot online_async_launch.py use_sim_time:=true    # terminal 2
rviz2 -d /ros2_ws/install/my_bot/share/my_bot/config/setup.rviz # terminal 3
```

Rij met controller of toetsenbord door alle gangen en kamers: rustig rijden
(± 0,2–0,3 m/s), langzaam draaien en eindigen bij het startpunt.

### Kaart opslaan

Terwijl SLAM nog draait:

```bash
mkdir -p /ros2_ws/maps
ros2 run nav2_map_server map_saver_cli -f /ros2_ws/maps/ziekenhuis --ros-args -p use_sim_time:=true
```

Resultaat: `ziekenhuis.pgm` + `ziekenhuis.yaml` in `~/ros2_ws/maps/` (blijft
bewaard, want die map is gemount). Zet de SLAM-modus daarna weer terug.

---

## 6. De ziekenhuis-behavior tree aanpassen

- **Bomen:** `hospital_robot_navigation/behavior_trees/hospital_navigate.xml.in`
  (route, NavigateThroughPoses) en `hospital_navigate_to_pose.xml.in` (één doel,
  NavigateToPose). Houd de blokkadeprocedure in beide gelijk.
  Placeholders `${naam}` worden bij het opstarten ingevuld uit de sectie
  `navigation` van `timing.yaml`. Ontbreekt een waarde, of staat er na het
  invullen nog een `${`, dan stopt de launch met een duidelijke fout.
- **Nieuwe placeholder:** zet hem in de XML én in `timing.yaml`.
- **Eigen C++ BT-node:** in `hospital_robot_bt_nodes`, één gedeelde bibliotheek
  per node met `BT_REGISTER_NODES` (zie de bestaande nodes en de README daar).
  Voeg de bibliotheeknaam toe aan `HOSPITAL_BT_PLUGINS` in
  `hospital_robot_bringup/launch/navigation.launch.py` én in
  `hospital_robot_bringup/test/test_bt_template.py`.
  In Iron vervangt `plugin_lib_names` de standaardlijst van Nav2; de launch
  zet de eigen plugins daarom áchter de volledige lijst uit `nav2_params.yaml`.
- **Testen zonder simulatie:** `colcon test --packages-select hospital_robot_bringup`
  vult de sjabloon in en controleert alle node- en poortnamen tegen de echte
  plugins (via `check_bt_xml`).
- Nav2 laadt plugins en de boom alleen bij het opstarten: na een wijziging
  opnieuw bouwen en `sim.launch.py` herstarten.

### Debuggen

- Fouten bij het laden van de boom staan in de log van `bt_navigator`
  (terminal 1, of `/root/.ros/log/bt_navigator_*.log`).
- `ros2 topic echo /behavior_tree_log` laat zien welke node van status
  verandert (`RUNNING` → `SUCCESS` / `FAILURE`).
- Welke boom Nav2 gebruikt:
  `ros2 param get /bt_navigator default_nav_through_poses_bt_xml` (en
  `default_nav_to_pose_bt_xml`).

---

## 7. Eigen nodes

### Routes rijden: route_manager + MissionManagerNode

- **route_manager** (`RouteManagerServer.py`) levert routes op naam via de service
  `get_route`. Routes staan in `config/routes.yaml`.
- **MissionManagerNode** haalt bij `start_route` de route op bij de route_manager,
  stuurt hem als `NavigateThroughPoses` naar Nav2 (dus met de ziekenhuisboom) en
  houdt de missietoestand bij op `/mission_status`
  (`IDLE` → `DRIVING` ⇄ `BLOCKED` → `ARRIVED`, of `WAITING_OPERATOR` als Nav2 het
  doel afbreekt).

`sim.launch.py` start beide al mee. De route_manager gebruikt daar
`/ros2_ws/src/navigatie_ezwheels/config/routes.yaml` (argument `routes_file`), dus
opgeslagen routes blijven bewaard en komen in git; bouwen is niet nodig. Zonder
`sim.launch.py` (bv. op de robot) start je ze los:

```bash
ros2 run my_bot RouteManagerServer.py --ros-args -p use_sim_time:=true \
  -p routes_file:=/ros2_ws/src/navigatie_ezwheels/config/routes.yaml
ros2 run my_bot MissionManagerNode.py --ros-args -p use_sim_time:=true
```

Bedienen:

```bash
ros2 service call /start_route my_bot_interfaces/srv/StartRoute "{route_name: gang_a}"
ros2 service call /cancel_mission std_srvs/srv/Trigger     # stoppen / resetten
ros2 topic echo /mission_status
```

Na `ARRIVED` kan direct een nieuwe route; na `WAITING_OPERATOR` eerst `cancel_mission`.

### Een route maken door te klikken

De route_manager houdt een **concept-route** bij. Elke pose op
`route_draft/add_pose` (frame `map`) wordt een waypoint: plek = waar je klikt,
richting = waar je naartoe sleept. De concept-route staat op `route_draft/poses`
(in RViz de oranje pijlen van het display **RouteDraft**).

1. **Klikken**
   - **Foxglove:** 3D-panel, Display frame `map`, **Publish → Pose** met topic
     `/route_draft/add_pose`. Klik en sleep per waypoint. Voeg `/route_draft/poses`
     toe in het 3D-panel om ze te zien.
   - **RViz:** zet bij **Tool Properties → 2D Goal Pose → Topic**
     tijdelijk `/route_draft/add_pose` en klik zoals bij een doel. Zet hem daarna
     terug op `/goal_pose`, anders rijdt de robot niet meer op **2D Goal Pose**.
2. **Corrigeren**
   ```bash
   ros2 service call /route_draft/undo  std_srvs/srv/Trigger   # laatste waypoint weg
   ros2 service call /route_draft/clear std_srvs/srv/Trigger   # opnieuw beginnen
   ```
3. **Opslaan** (daarna is de concept-route weer leeg)
   ```bash
   ros2 service call /route_draft/save my_bot_interfaces/srv/SaveRoute "{name: gang_b}"
   ros2 service call /route_draft/save my_bot_interfaces/srv/SaveRoute "{name: gang_b, overwrite: true}"  # vervangen
   ```
   Naam: letters, cijfers, `_` en `-`. De waypoints heten `gang_b_1`, `gang_b_2`, …
4. **Rijden**
   ```bash
   ros2 service call /start_route my_bot_interfaces/srv/StartRoute "{route_name: gang_b}"
   ```

In Foxglove kan stap 2–4 met het **Service Call**-panel (zelfde service en type).

### Een opgeslagen route bekijken

```bash
ros2 service call /get_route my_bot_interfaces/srv/GetRoute "{name: gang_a}"
```

Het antwoord bevat alle waypoints (naam, x, y, yaw). Daarnaast publiceert de
route_manager elke opgevraagde route op `route_preview` (PoseArray, frame `map`):
in RViz de blauwe pijlen van het display **RoutePreview**, in Foxglove het topic
`/route_preview` in het 3D-panel. Ook een route die met `start_route` gestart
wordt, verschijnt daar, dus je ziet altijd welke route de robot rijdt.

Tips:

- Klik waypoints minstens ~0,6 m van muren (robotstraal 0,22 m + inflation
  0,55 m). Ligt een waypoint te dicht bij een muur, dan kan Nav2 er niet heen
  plannen en blijft de robot (die nooit opgeeft) op `blocked = true` staan.
- Sleep de richting naar het volgende waypoint; anders draait de robot daar
  eerst om zijn as.
- Een pose in een ander frame dan `map` wordt genegeerd (met een waarschuwing in
  de log van de route_manager).
- Handmatig bewerken van `config/routes.yaml` kan ook; het bestand wordt bij elke
  aanvraag opnieuw gelezen:

```yaml
routes:
  gang_a:
    map_id: ziekenhuis
    waypoints:
    - {name: gang_a_1, x: 3.19, y: 0.21, yaw: 0.0}    # m, m, rad in het map-frame
    - {name: gang_a_2, x: 5.48, y: -0.6, yaw: -0.8}
```

  Een fout in het bestand (bv. ontbrekende `yaw`) geeft bij `start_route` een
  melding met route en waypointnummer.

### turn_detector: bochten op tijd herkennen

`TurnDetectorNode.py` (start mee met `sim.launch.py` en `robot.launch.py`) publiceert
op `turn` (`my_bot_interfaces/msg/Turn`, 5 Hz), bedoeld voor een signal_manager
(knipperlichten):

| Veld | Betekenis |
|---|---|
| `direction` | komende bocht volgens het pad: `NONE` (0), `LEFT` (1), `RIGHT` (2) |
| `distance` | m tot de bocht begint |
| `angle` | rad, hoeveel de bocht draait (+ = links) |
| `turning` | draait de robot nu (gemeten via `/odom`): `NONE`, `LEFT`, `RIGHT` |

- Kijkt `lookahead_m` (3 m) vooruit in `/plan`; een richtingsverandering van
  ≥ `turn_angle_deg` (30°) telt als bocht.
- Omdat Nav2 per waypoint plant, plakt hij de resterende waypoints achter het pad:
  zo ziet hij ook de bocht **bij** een waypoint op tijd. Die lijst publiceert de
  BT-node `GetNextWaypoint` op `remaining_waypoints` (huidig doel eerst, leeg als de
  route klaar is), dus het werkt hoe de route ook gestart is: `start_route`, het
  RViz-paneel (**Start Nav Through Poses**) of `ros2 action send_goal`. Niet met
  **Start Waypoint Following**: dat gebruikt de `NavigateToPose`-boom per waypoint.
- De bocht wordt gemeten ten opzichte van de rijrichting van de robot (TF
  `map → base_link`), dus ook vlak voor het waypoint blijft hij gemeld tot de robot
  draait (`turning`). Zonder TF valt hij terug op het eerste stuk pad en mist hij
  een bocht in de laatste ~0,5 m.
- Bij elke verandering logt hij een regel, bv. `bocht RECHTS over 2.8 m (-31°)`.

```bash
ros2 topic echo /turn
```

Zien wat hij herkent: `turn_markers` (MarkerArray). In RViz staat het display
**TurnMarkers** al aan; in Foxglove zet je `/turn_markers` aan in het 3D-panel.

- Oranje knipperend bolletje links- of rechtsvoor op de robot: komende bocht of
  draaien naar die kant.
- Oranje bol met tekst (bv. `RECHTS 2.8 m`) op het pad: waar de bocht begint.

### signal_manager: knipperlichten en buzzer

`SignalManagerNode.py` (start mee met `sim.launch.py` en `robot.launch.py`):
sources → `RobotFacts` → actuators → uitgangen (`my_bot/signal_manager/`).

| Source | Topic | Feit |
|---|---|---|
| `TurnSource` | `/turn` | `turn`: komende bocht, anders draaien. Niet vers na `turn_timeout_s` (1 s) |
| `RobotStateSource` | `/mission_status` | `mode`: `IDLE`, `DRIVING`, `BLOCKED`, `WAITING_OPERATOR`, `ARRIVED`, `ERROR` |
| `CollisionStateSource` | `navigation/blocked` | `collision`: `NONE` / `BLOCKED` |

| Actuator | Gedrag | Uitgang |
|---|---|---|
| `Blinkers` | `gevaar` bij blokkade, `WAITING_OPERATOR`/`ERROR`, of tijdens `DRIVING` een source die niet vers is; anders `links`/`rechts` bij een bocht (brandt `hold_s` = 1 s na); anders `uit` | `/indicators` (bij verandering) |
| `Horn` | `elke_5s` zolang de route geblokkeerd is, daarna `uit`; anders `enkel` bij vertrek en aankomst, `dubbel` bij `WAITING_OPERATOR`/`ERROR` | `/buzzer_pattern` (bij verandering) |

Buzzerpatronen op `/buzzer_pattern` (`std_msgs/String`); de ontvanger speelt ze af:

| Patroon | Betekenis |
|---|---|
| `enkel` | één piep |
| `dubbel` | twee piepjes |
| `elke_5s` | elke 5 s een piep, tot een ander patroon komt |
| `uit` | herhalend patroon stoppen |

`mqtt_hmi_bridge` zet `/indicators` om naar MQTT (`robot/outputs/lamp_links`,
`lamp_rechts`). `/buzzer_pattern` gaat nog niet naar MQTT: de bridge kent alleen het
oude `/buzzer` (`Bool`, aan/uit). De actuators bepalen alleen hun toestand; het
versturen zit in de node, zodat dat later direct via MQTT kan.

```bash
ros2 topic echo /indicators
ros2 topic echo /buzzer_pattern std_msgs/msg/String
```

Vertrek- en aankomstpiep komen van `/mission_status`, dus alleen bij `start_route`
(MissionManager), niet bij een route vanuit het RViz-paneel.

> Start `indicator_node.py` niet tegelijk: die publiceert op dezelfde topics.

### hmi_bridge: koppeling met de HMI via MQTT

`HMIBridgeNode.py` (start mee met `robot.launch.py`, niet in de simulatie) stuurt de
signalen naar de HMI en ontvangt de knoppen (`my_bot/hmi_bridge/`). JSON, versie 1:

| Richting | MQTT-topic | Payload |
|---|---|---|
| robot → HMI | `robot/hmi/command` | `{"v": 1, "seq": 42, "blinkers": "links", "buzzer": "enkel", "buzzer_id": 3, "mission": "DRIVING gang_a"}` |
| HMI → robot | `robot/hmi/status` | `{"v": 1, "ack": 42, "start": false, "stop": false}` |

- **command** gaat bij elke verandering en elke `heartbeat_s` (1 s), met een oplopend
  `seq`. `blinkers` en `buzzer` komen van de signal_manager (`/indicators`,
  `/buzzer_pattern`), `mission` van `/mission_status`.
- **Buzzer:** speel het patroon alleen af als `buzzer_id` verandert. Anders herhaalt de
  hartslag elke seconde een `enkel`.
- **status:** `ack` = laatst ontvangen `seq`. `start`/`stop` = knop ingedrukt; de bridge
  reageert alleen op indrukken (`false` → `true`): `start` → `start_route` met
  `start_route_name`, `stop` → `cancel_mission`. Ongeldige berichten worden genegeerd
  (waarschuwing in de log).
- **Verbinding:** stuurt de HMI langer dan `hmi_timeout_s` (3 s) geen status, dan wordt
  `/hmi/connected` `false` (en weer `true` bij het volgende bericht). Laat de HMI dus
  ook zonder knopdruk regelmatig (bv. elke seconde) een status sturen.

Testen zonder HMI, met een broker (bv. `mosquitto`) en de `mosquitto_*`-clients:

```bash
ros2 run my_bot HMIBridgeNode.py --ros-args -p start_route_name:=gang_a
mosquitto_sub -t robot/hmi/command                                   # commands zien
mosquitto_pub -t robot/hmi/status -m '{"v": 1, "ack": 1, "start": true}'
ros2 topic echo /hmi/connected --qos-durability transient_local
```

Zonder broker: `-p backend:=memory` (verstuurt niets, alleen voor tests).

### indicator_node, mqtt_hmi_bridge, environment_speed_node

Deze nodes luisteren nog naar de topics van de oude `patrol_node`
(`/patrol_state`, `/start_patrol`, `/stop_patrol`) en zijn nog niet aangesloten
op de MissionManagerNode / ziekenhuisboom. Ze starten wel:

```bash
ros2 run my_bot indicator_node.py --ros-args -p use_sim_time:=true
ros2 run my_bot environment_speed_node.py --ros-args -p use_sim_time:=true
ros2 run my_bot mqtt_hmi_bridge.py --ros-args -p broker_host:=localhost   # MQTT-broker nodig
```

---

## 8. Op de echte robot

```bash
ros2 launch hospital_robot_bringup robot.launch.py map:=<kaart van het gebouw>.yaml \
  routes_file:=<schrijfbare map>/routes.yaml
```

Start hetzelfde als `sim.launch.py`, maar zonder Gazebo en RViz en met
`config/nav2_params_robot.yaml` (`use_sim_time` uit):

- AMCL + map_server op `map` (verplicht argument: de kaart van het echte gebouw,
  niet de simulatiekaart)
- Nav2 met beide ziekenhuisbomen (`hospital_robot_bringup/navigation.launch.py`)
- route_manager en MissionManagerNode
- foxglove_bridge op poort 8765 (uit met `foxglove:=false`)

| Argument | Standaard |
|---|---|
| `map` | — (verplicht) |
| `routes_file` | — (verplicht) — absoluut pad buiten `install/`, bv. `/maps/<locatie>/routes.yaml` |
| `timing_file` | `share/hospital_robot_bringup/config/timing.yaml` |
| `foxglove` | `true` |

Voorwaarden op de robot:

- **De EZ-Wheel/EZ-Way-software levert de hardware**: lidar op
  `/laser_1/scan_filtered`, `/odom`, TF `odom → base_footprint → base_link` en een
  driver die naar `/cmd_vel` luistert. Deze launch start géén drivers,
  robot_state_publisher of lidar.
- **De navigatie van EZ-Way ("automatic mode") mag niet tegelijk draaien**: die
  heeft eigen Nav2-nodes met dezelfde namen en kan de ziekenhuis-BT-plugins niet
  laden.
- Zelfde DDS-instellingen als de EZ-Way-software, bv.
  `export CYCLONEDDS_URI=file:///home/ezway/.config/ez-way/ez_way_bringup/cyclonedds_config.xml`
  (zie `README.md`).
- Alle pakketten gebouwd in de robot-container (`colcon build`), inclusief
  `foxglove_bridge` als je die gebruikt.

Na het starten:

1. **Beginpositie zetten.** AMCL start op (0, 0) van de kaart. Staat de robot
   ergens anders: Foxglove 3D-panel → **Publish → Pose estimate** (topic
   `/initialpose`), of RViz **2D Pose Estimate** vanaf een laptop.
2. Controleer in Foxglove dat de scan (`/laser_1/scan_filtered`) op de muren van de
   kaart valt.
3. Routes maken en rijden zoals in stap 7 (concept-route klikken, `route_draft/save`,
   `start_route`). Begin met korte routes en een hand bij de noodstop.

Komt Nav2 niet op `Managed nodes are active` maar blijft de log
`Timed out waiting for transform from base_link to odom` herhalen, dan levert de
robotsoftware de odometrie/TF niet (driver uit of ander DDS-domein).

---

## 9. Tests (zonder simulatie)

```bash
cd /ros2_ws
colcon test --packages-select hospital_robot_bt_nodes hospital_robot_navigation hospital_robot_bringup
colcon test-result --verbose

cd /ros2_ws/src/navigatie_ezwheels
python3 -m unittest discover -s test -p "test_mission*" -v   # MissionStateMachine
python3 -m unittest discover -s test -p "test_route*" -v     # route-opslag (routes.yaml)
```

`test/test_patrol_controller.py` en `test/fakes.py` horen bij de verwijderde
`patrol_node` en falen daarom.

---

## Problemen oplossen

| Probleem | Oplossing |
|---|---|
| `RuntimeError: !rclpy.ok()` of `ros2 ...` hangt | `ros2 daemon stop` (container én host), of `--no-daemon` |
| `Duplicate package names` bij bouwen | Bouwen vanuit `/ros2_ws` (daar staat `colcon_defaults.yaml`) |
| `Package 'hospital_robot_bringup' not found` | Bouwen en `source /ros2_ws/install/setup.bash` in die terminal |
| `colcon test` vindt een pakket niet | Draaien vanuit `/ros2_ws` |
| `Operation not permitted` bij bouwen | `build/`/`install/` zijn van root → bouw in de container of `chown` |
| `start_route` weigert: `Service /get_route niet beschikbaar` | route_manager starten (stap 7) |
| `route_draft/save`: `bestaat al` | Andere naam, of `overwrite: true` |
| Geklikte pose komt niet in de concept-route | Display frame niet `map` (zie log route_manager), of verkeerd topic (`/route_draft/add_pose`) |
| `start_route` weigert: `Onbekende route` | Naam staat niet in `routes.yaml` (de melding noemt de beschikbare routes) |
| `start_route` weigert: `Kan niet starten in WAITING_OPERATOR` | Eerst `ros2 service call /cancel_mission std_srvs/srv/Trigger` |
| Robot rijdt met de standaardboom i.p.v. de ziekenhuisboom | Gestart met `my_bot sim_nav.launch.py` i.p.v. `hospital_robot_bringup sim.launch.py` |
| `navigation/blocked = true` zonder obstakel | Log van `controller_server`: `Failed to make progress` → robot schoot `progress_timeout_s` lang geen 0,5 m op |
| bt_navigator start niet: `Could not load library` | Plugin staat in `plugin_lib_names` maar is niet gebouwd; bouwen of uit `nav2_params.yaml` halen |
| `Node not recognized` in de boom | Plugin ontbreekt in `HOSPITAL_BT_PLUGINS` of in `plugin_lib_names` |
| Nav2: TF- of tijdfouten | `use_sim_time:=true` vergeten |
| Robot staat verkeerd op de kaart | **2D Pose Estimate** in RViz |
| `/dev/input/js0` bestaat niet in de container | Container herstarten met de nieuwe `start.sh` |
| Robot rijdt niet met controller | X ingedrukt houden; check `ros2 topic echo /joy` en `/cmd_vel` |
| Package niet gevonden (nav2, slam_toolbox, joy) | In de `Dockerfile` zetten en image opnieuw bouwen |
