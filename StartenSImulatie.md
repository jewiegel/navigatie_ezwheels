# Simulatie

De ziekenhuisrobot in Gazebo met Nav2, de eigen behavior tree en de eigen nodes.
Alles draait in de **ROS 2 Iron Docker container**. Voor de echte robot: zie
[startup.md](startup.md).

## 1. Container en bouwen

```bash
~/ros2_docker/start.sh          # start de container, of opent een extra shell erin
cd /ros2_ws && colcon build && source install/setup.bash
```

- `~/ros2_ws` van de host is `/ros2_ws` in de container. Bouw altijd in de container
  (`build/` en `install/` zijn van root).
- Image opnieuw bouwen na een wijziging in `~/ros2_docker/Dockerfile`:
  `docker build -t mijn_ros2_iron ~/ros2_docker`. Losse `apt install`s zijn weg na
  een herstart van de container.
- colcon leest de pakketpaden uit `/ros2_ws/colcon_defaults.yaml`; bouw en test dus
  vanuit `/ros2_ws`.
- Na het verwijderen of hernoemen van bestanden: schoon bouwen
  (`rm -rf build install log && colcon build`). colcon haalt niets weg uit `install/`.

| Pakket | Inhoud |
|---|---|
| `my_bot` | robotmodel, wereld, Nav2-config, eigen Python-nodes |
| `my_bot_interfaces` | `Turn`, `StartRoute`, `GetRoute`, `SaveRoute` |
| `hospital_robot_bt_nodes` | BT-plugins in C++ (`GetNextWaypoint`, `PopWaypoint`, `PublishBlocked`, `IsPathBlocked`) |
| `hospital_robot_navigation` | behavior tree-sjablonen |
| `hospital_robot_bringup` | `sim.launch.py`, `robot.launch.py`, `config/timing.yaml` |

## 2. Simulatie starten

```bash
ros2 launch hospital_robot_bringup sim.launch.py
```

Start Gazebo, RViz, foxglove_bridge (`ws://localhost:8765`), AMCL, Nav2 met de
ziekenhuisbomen en de eigen nodes. Klaar als de log
`lifecycle_manager_navigation: Managed nodes are active` toont (± 15 s). De robot
staat op (0, 0); dat is ook de beginpositie van AMCL.

| Argument | Standaard |
|---|---|
| `map` | `/ros2_ws/maps/ziekenhuis.yaml` |
| `routes_file` | `/ros2_ws/src/navigatie_ezwheels/config/routes.yaml` (in git, geen build nodig) |
| `timing_file` | `share/hospital_robot_bringup/config/timing.yaml` |
| `foxglove` | `true` |

De simulatie gebruikt dezelfde topics als de robot (`/laser_1/scan_filtered`, `/odom`,
`/cmd_vel`). `config/nav2_params.yaml` (simulatie) en `nav2_params_robot.yaml`
verschillen alleen in `inflation_radius` (0,55 / 0,80 m), `base_frame_id` en
`use_sim_time`.

## 3. Rijden

**Route op naam** (MissionManager):

```bash
ros2 service call /start_route my_bot_interfaces/srv/StartRoute "{route_name: gang_b}"
ros2 service call /cancel_mission std_srvs/srv/Trigger
ros2 topic echo /mission_status       # IDLE, DRIVING, ARRIVED, WAITING_OPERATOR (+ route)
```

Na `ARRIVED` kan direct een nieuwe route; na `WAITING_OPERATOR` eerst `cancel_mission`.

**Vanuit RViz:** **2D Goal Pose** (één doel), of het Nav2-paneel → **Waypoint / Nav
Through Poses Mode** → waypoints klikken → **Start Nav Through Poses**. Gebruik niet
**Start Waypoint Following**: dan ziet de turn_detector bochten bij waypoints niet.

**Met de hand:** `ros2 run teleop_twist_keyboard teleop_twist_keyboard`, of een
Xbox-controller (X ingedrukt houden):
`ros2 launch teleop_twist_joy teleop-launch.py joy_config:=xbox`. Niet tegelijk met
een Nav2-doel.

### Wat de behavior tree doet

- Rijdt **waypoint voor waypoint**: plannen naar het volgende waypoint, rijden tot het
  bereikt is (0,25 m en 0,25 rad), naar de yaw van het waypoint draaien, door.
- **Blokkade** (geen pad, of `progress_timeout_s` lang geen 0,5 m voortgang):
  `navigation/blocked` → `true`, dan steeds costmaps wissen (behalve rond de robot),
  `blockade_wait_s` wachten en opnieuw plannen. Lukt dat: `navigation/blocked` →
  `false` en verder. De robot **geeft nooit op**; alleen `cancel_mission` stopt hem.
- Tijden in `hospital_robot_bringup/config/timing.yaml` (beide standaard 10 s).

## 4. Routes maken

Routes staan in het `routes_file` en worden bij elke aanvraag opnieuw gelezen.

1. **Klikken:** elke pose op `/route_draft/add_pose` (frame `map`) wordt een waypoint:
   klik = plek, slepen = rijrichting. Foxglove: 3D-panel → **Publish → Pose** op
   `/route_draft/add_pose`. RViz: **Tool Properties → 2D Goal Pose → Topic** tijdelijk
   op `/route_draft/add_pose` (daarna terug op `/goal_pose`). De concept-route staat op
   `/route_draft/poses`.
2. **Corrigeren en opslaan:**
   ```bash
   ros2 service call /route_draft/undo  std_srvs/srv/Trigger
   ros2 service call /route_draft/clear std_srvs/srv/Trigger
   ros2 service call /route_draft/save  my_bot_interfaces/srv/SaveRoute "{name: gang_c}"   # overwrite: true om te vervangen
   ```
3. **Bekijken:** `ros2 service call /get_route my_bot_interfaces/srv/GetRoute "{name: gang_c}"`
   (ook als pijlen op `/route_preview`).

- Waypoints minstens ~0,6 m van muren; anders blijft de robot op "geblokkeerd" staan.
- **Laat de pijl naar het volgende waypoint wijzen.** Anders draait de robot op het
  waypoint eerst om zijn as, en knippert hij even naar die kant.
- Met de hand bewerken kan ook:
  ```yaml
  routes:
    gang_a:
      map_id: ziekenhuis
      waypoints:
      - {name: gang_a_1, x: 3.19, y: 0.21, yaw: 0.0}    # m, m, rad in het map-frame
  ```

## 5. Eigen nodes

Alle nodes starten mee met `sim.launch.py`, behalve de hmi_bridge (alleen `robot.launch.py`).

| Node | Doet | Belangrijkste topics |
|---|---|---|
| `route_manager` | routes laden/opslaan, concept-route | `get_route`, `route_draft/*`, `route_preview` |
| `mission_manager` | route naar Nav2, missietoestand | `start_route`, `cancel_mission`, `mission_status` |
| `turn_detector` | komende bocht en draaien | `turn`, `turn_markers` (RViz/Foxglove) |
| `signal_manager` | knipperlichten en buzzer | `indicators`, `buzzer_pattern` |
| `hmi_bridge` | MQTT naar de HMI | zie hieronder |

**turn_detector:** kijkt `lookahead_m` (3 m) vooruit in `/plan`; een
richtingsverandering van ≥ 30° is een bocht. Omdat Nav2 per waypoint plant, plakt hij
de resterende waypoints (`/remaining_waypoints`, van de BT) achter het pad, zodat ook
een bocht **bij** een waypoint op tijd gemeld wordt. Logt elke verandering, bv.
`bocht LINKS over 2.8 m (+90°); draait nu: geen`.

**signal_manager** (`my_bot/signal_manager/`: sources → `RobotFacts` → actuators):

| Uitgang | Waarde | Wanneer |
|---|---|---|
| `/indicators` | `gevaar` | blokkade, `WAITING_OPERATOR`/`ERROR`, of tijdens het rijden een bron die wegvalt |
| | `links` / `rechts` | onderweg: de komende bocht; binnen `at_turn_m` (0,5 m): de echte draairichting. Brandt `hold_s` (1 s) na |
| | `uit` | anders |
| `/buzzer_pattern` | `elke_5s` → `uit` | zolang de route geblokkeerd is |
| | `enkel` | vertrek en aankomst (alleen bij `start_route`) |
| | `dubbel` | `WAITING_OPERATOR` / `ERROR` |

**hmi_bridge** (`my_bot/hmi_bridge/`) gebruikt dezelfde MQTT-topics als de oude bridge
die op de HMI werkte:

| Richting | MQTT-topic | Payload |
|---|---|---|
| robot → HMI | `robot/outputs/lamp_links`, `lamp_rechts` | `1`/`0`, knipperend met `blink_period_s` (1 s); `gevaar` = beide |
| robot → HMI | `robot/outputs/buzzer` | `1`/`0`, de bridge speelt het patroon af |
| robot → HMI | `robot/patrol_state` | `idle`, `rijdend`, `wachten`, `voltooid`, `fout` |
| HMI → robot | `robot/start_patrol` = `1` | `start_route` met `start_route_name` (leeg = niets) |
| HMI → robot | `robot/stop_patrol` = `1` | `cancel_mission` |

Alles gaat bij een verandering en elke `heartbeat_s` (1 s). `/hmi/mqtt_connected` is
`false` als de broker langer dan 3 s weg is.

## 6. Testen in de simulatie

**Signalen volgen:**

```bash
ros2 topic echo /turn
ros2 topic echo /indicators --field data
ros2 topic echo /buzzer_pattern std_msgs/msg/String --field data
```

**Blokkade:** zet in Gazebo een obstakel dat de gang helemaal afsluit (kan de planner
eromheen, dan is er geen blokkade). Of doe alsof:

```bash
ros2 topic echo navigation/blocked --qos-durability transient_local
ros2 topic pub -t 3 --qos-durability transient_local navigation/blocked std_msgs/msg/Bool "{data: true}"
```

**HMI zonder HMI** (broker op de host, bridge in de container):

```bash
docker run --rm -d --name mqtt_test --network host eclipse-mosquitto:2.0.18 mosquitto -c /mosquitto-no-auth.conf
ros2 run my_bot HMIBridgeNode.py --ros-args -p use_sim_time:=true -p start_route_name:=gang_b
docker exec mqtt_test mosquitto_sub -v -t 'robot/#'              # alles zien
docker exec mqtt_test mosquitto_pub -t robot/start_patrol -m 1   # startknop
docker stop mqtt_test
```

## 7. Kaart maken met SLAM

Alleen voor een nieuwe kaart. Zet `mode: mapping` in
`config/mapper_params_online_async.yaml` en bouw opnieuw. Dan:

```bash
ros2 launch my_bot launch_sim.launch.py                                   # Gazebo + robot
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true \
  slam_params_file:=/ros2_ws/install/my_bot/share/my_bot/config/mapper_params_online_async.yaml
rviz2 -d /ros2_ws/install/my_bot/share/my_bot/config/setup.rviz
```

Rij rustig (± 0,2–0,3 m/s) door alle gangen en eindig bij het startpunt. Opslaan
terwijl SLAM nog draait, daarna `mode` weer op `localization`:

```bash
ros2 run nav2_map_server map_saver_cli -f /ros2_ws/maps/ziekenhuis --ros-args -p use_sim_time:=true
```

## 8. Behavior tree aanpassen

- Bomen: `hospital_robot_navigation/behavior_trees/hospital_navigate.xml.in` (route)
  en `hospital_navigate_to_pose.xml.in` (één doel). Houd de blokkadeprocedure gelijk.
  `${naam}` wordt bij het starten ingevuld uit `timing.yaml` (sectie `navigation`).
- Nieuwe C++ BT-node: in `hospital_robot_bt_nodes` (één bibliotheek per node), en de
  naam toevoegen aan `HOSPITAL_BT_PLUGINS` in `hospital_robot_bringup/launch/navigation.launch.py`
  én `hospital_robot_bringup/test/test_bt_template.py`.
- Nav2 laadt bomen en plugins alleen bij het starten: bouwen en launch herstarten.
- Debuggen: log van `bt_navigator`, `ros2 topic echo /behavior_tree_log`.

## 9. Tests

```bash
cd /ros2_ws
colcon test --packages-select hospital_robot_bt_nodes hospital_robot_navigation hospital_robot_bringup
colcon test-result --verbose

cd /ros2_ws/src/navigatie_ezwheels
for t in mission route turn signal hmi; do python3 -m unittest discover -s test -p "test_${t}*"; done
```

`colcon test` op `my_bot` meldt stijlfouten (copyright-koppen, docstrings); die zeggen
niets over de werking.

## Problemen oplossen

| Probleem | Oplossing |
|---|---|
| `ros2 ...` hangt of `RuntimeError: !rclpy.ok()` | `ros2 daemon stop` (container én host), of `--no-daemon` |
| `Duplicate package names` / pakket niet gevonden bij bouwen of testen | Vanuit `/ros2_ws` draaien (`colcon_defaults.yaml`) |
| `Package '...' not found` bij starten | `source /ros2_ws/install/setup.bash` in die terminal |
| Tijd springt terug, TF-fouten | Twee simulaties tegelijk (twee `/clock`); stop er één |
| Verwijderd bestand doet nog mee | Oude kopie in `install/`: schoon bouwen |
| `start_route`: `Onbekende route` / `WAITING_OPERATOR` | Naam staat niet in het `routes_file` / eerst `cancel_mission` |
| Geklikte pose komt niet in de concept-route | Frame niet `map`, of verkeerd topic |
| `navigation/blocked = true` zonder obstakel | Waypoint te dicht bij een muur, of `progress_timeout_s` lang geen voortgang (log `controller_server`) |
| bt_navigator: `Could not load library` / `Node not recognized` | Plugin niet gebouwd, of ontbreekt in `HOSPITAL_BT_PLUGINS` |
| Knipperlicht ziet bocht bij waypoint pas bij het draaien | Route via **Waypoint Following** gestart, of `hospital_robot_bt_nodes` niet opnieuw gebouwd |
