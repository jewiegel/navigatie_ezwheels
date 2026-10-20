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
| `my_bot_interfaces` | messages/services voor de MissionManagerNode (`StartRoute`, `GetRoute`) |
| `hospital_robot_bt_nodes` | eigen BT-plugins in C++ (`PublishBlocked`, `IsPathBlocked`, `GetNextWaypoint`, `PopWaypoint`) |
| `hospital_robot_navigation` | de behavior tree-sjabloon `hospital_navigate.xml.in` |
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

```bash
ros2 launch hospital_robot_bringup sim.launch.py
```

| Argument | Standaard |
|---|---|
| `map` | `/ros2_ws/maps/ziekenhuis.yaml` |
| `timing_file` | `share/hospital_robot_bringup/config/timing.yaml` |

Wat er gebeurt:

1. Gazebo met de ziekenhuiswereld, de robot en de bridge (`my_bot/launch_sim.launch.py`).
2. RViz met `my_bot/config/setup.rviz` (inclusief het Nav2-paneel).
3. Na 8 s (Gazebo moet eerst `/clock` en `/tf` publiceren):
   - AMCL op de ziekenhuiskaart. De beginpositie (0, 0) wordt automatisch gezet
     (`set_initial_pose` in `nav2_params.yaml`), dus **2D Pose Estimate is niet
     nodig** zolang de robot op zijn spawnplek staat.
   - Nav2 via `hospital_robot_bringup/navigation.launch.py`: die vult
     `hospital_navigate.xml.in` in met de tijden uit `timing.yaml`, zet hem als
     boom voor `NavigateThroughPoses`, voegt de eigen BT-plugins toe aan
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

### Een route rijden vanuit RViz

De ziekenhuisboom hoort bij **NavigateThroughPoses**. Gebruik daarom het
**Navigation 2**-paneel:

1. Klik in het paneel op **Waypoint / Nav Through Poses Mode**.
2. Kies **Nav2 Goal** in de werkbalk en klik de waypoints op de kaart
   (klik = plek, slepen = richting). Laat de pijl wijzen naar het volgende
   waypoint, anders draait de robot daar onnodig.
3. Klik **Start Nav Through Poses**.

Niet gebruiken voor deze boom:

- **Start Waypoint Following** — stuurt elk punt apart als `NavigateToPose` →
  standaardboom van Nav2.
- **2D Goal Pose** — publiceert in deze RViz-config op `/goal_pose_bt` voor
  `nav2_test_node.py` (stap 7); zonder die node gebeurt er niets.

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
ros2 topic list            # /scan, /odom, /clock, /tf moeten er staan
ros2 topic hz /scan        # lidar publiceert?
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

- **Boom:** `hospital_robot_navigation/behavior_trees/hospital_navigate.xml.in`.
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

### Losse boom proberen met nav2_test_node

Voor experimenten met een losse XML in `behavior_trees/` van `my_bot`
(`NavigateToPose`):

```bash
ros2 run my_bot nav2_test_node.py --ros-args \
  -p behavior_tree:=test1.xml -p use_sim_time:=true
```

Klik daarna een doel met **2D Goal Pose** (publiceert op `/goal_pose_bt`). De
node stuurt elk geklikt doel met die boom naar Nav2.

### Debuggen

- Fouten bij het laden van de boom staan in de log van `bt_navigator`
  (terminal 1, of `/root/.ros/log/bt_navigator_*.log`).
- `ros2 topic echo /behavior_tree_log` laat zien welke node van status
  verandert (`RUNNING` → `SUCCESS` / `FAILURE`).
- Welke boom Nav2 gebruikt:
  `ros2 param get /bt_navigator default_nav_through_poses_bt_xml`.

---

## 7. Eigen nodes

### MissionManagerNode

Lifecycle-node die routes start via Nav2 (`NavigateThroughPoses`, dus met de
ziekenhuisboom) en de missietoestand bijhoudt.

```bash
ros2 run my_bot MissionManagerNode.py --ros-args -p use_sim_time:=true

ros2 service call /start_route my_bot_interfaces/srv/StartRoute "{route_name: gang_a}"
ros2 service call /cancel_mission std_srvs/srv/Trigger
ros2 topic echo /mission_status
```

> Heeft de service `get_route` (route_manager) nodig om een route op naam op te
> halen. Die bestaat nog niet: zonder route_manager weigert `start_route` elke
> route.

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

## 8. Tests (zonder simulatie)

```bash
cd /ros2_ws
colcon test --packages-select hospital_robot_bt_nodes hospital_robot_navigation hospital_robot_bringup
colcon test-result --verbose

cd /ros2_ws/src/navigatie_ezwheels
python3 -m unittest discover -s test -p "test_mission*" -v   # MissionStateMachine
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
| Robot rijdt niet bij **2D Goal Pose** | Die knop is voor `nav2_test_node.py`; gebruik het Nav2-paneel (stap 3) |
| Robot rijdt met de standaardboom i.p.v. de ziekenhuisboom | Gestart met **Start Waypoint Following** of `navigate_to_pose`; gebruik **Start Nav Through Poses** |
| `navigation/blocked = true` zonder obstakel | Log van `controller_server`: `Failed to make progress` → robot schoot `progress_timeout_s` lang geen 0,5 m op |
| bt_navigator start niet: `Could not load library` | Plugin staat in `plugin_lib_names` maar is niet gebouwd; bouwen of uit `nav2_params.yaml` halen |
| `Node not recognized` in de boom | Plugin ontbreekt in `HOSPITAL_BT_PLUGINS` of in `plugin_lib_names` |
| Nav2: TF- of tijdfouten | `use_sim_time:=true` vergeten |
| Robot staat verkeerd op de kaart | **2D Pose Estimate** in RViz |
| `/dev/input/js0` bestaat niet in de container | Container herstarten met de nieuwe `start.sh` |
| Robot rijdt niet met controller | X ingedrukt houden; check `ros2 topic echo /joy` en `/cmd_vel` |
| Package niet gevonden (nav2, slam_toolbox, joy) | In de `Dockerfile` zetten en image opnieuw bouwen |
