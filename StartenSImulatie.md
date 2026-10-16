# Simulatie starten en testen

Stappenplan om de robot in Gazebo te draaien, met controller of toetsenbord een
kaart te maken, Nav2 te testen en de eigen nodes (patrouille, knipperlichten,
snelheid) uit te proberen. Alles draait in de **ROS 2 Iron Docker container**.

Overzicht van de terminals die je uiteindelijk open hebt:

| # | Terminal | Commando |
|---|---|---|
| 1 | Simulatie (Gazebo) | `ros2 launch my_bot launch_sim.launch.py` |
| 2 | SLAM **of** AMCL | `online_async_launch.py` / `localization_launch.py` |
| 3 | Nav2 | `navigation_launch.py` |
| 4 | RViz | `rviz2` |
| 5 | Besturing | controller of toetsenbord |
| 6+ | Eigen nodes | `patrol_node.py`, `indicator_node.py`, … |

### Snel: terminal 1–4 in één keer

```bash
ros2 launch my_bot sim_nav.launch.py                 # AMCL op /ros2_ws/maps/ziekenhuis.yaml
ros2 launch my_bot sim_nav.launch.py map:=/ros2_ws/maps/andere.yaml
ros2 launch my_bot sim_nav.launch.py slam:=true      # SLAM i.p.v. AMCL
```

Start Gazebo en RViz, en na 8 s AMCL (of SLAM) en Nav2, allemaal met
`use_sim_time:=true`. Bij AMCL zet je daarna nog een **2D Pose Estimate** in RViz.

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
  shell in dezelfde container. Doe dit voor elke terminal uit de tabel hierboven.

In de container:

- `~/ros2_ws` van de host is beschikbaar als **`/ros2_ws`**
- `/opt/ros/iron/setup.bash` en `/ros2_ws/install/setup.bash` worden automatisch
  gesourced (via `~/.bashrc`)
- `/dev/input` is doorgegeven, dus de controller werkt in de container

Gewijzigde opties in `start.sh` (bv. devices) werken pas na een herstart van de
container: sluit alle shells of doe `docker stop ros2_iron`, en start opnieuw.

### Eenmalig per container: ros2 daemon resetten

De container gebruikt `--net=host`. Draait er op de host ook een ros2 daemon
(bv. van Humble), dan krijg je bij `ros2 topic ...` de fout
`RuntimeError: !rclpy.ok()`. Oplossing:

```bash
ros2 daemon stop        # in de container én op de host
```

Of gebruik `--no-daemon`, bv. `ros2 topic list --no-daemon`.

---

## 2. Bouwen

In de container:

```bash
cd /ros2_ws
colcon build --base-paths src/navigatie_ezwheels
source install/setup.bash
```

- `--base-paths` is nodig omdat in `src/` twee packages met de naam `my_bot`
  staan (`my_bot/` en `navigatie_ezwheels/`). Zonder deze optie stopt colcon met
  *Duplicate package names*. Alternatief: `touch /ros2_ws/src/my_bot/COLCON_IGNORE`.
- Opnieuw bouwen is nodig na wijzigingen in launch-, config- of Python-bestanden
  (die worden naar `install/` gekopieerd).

> De container draait als root, dus `build/` en `install/` worden eigendom van
> root. Bouw daarom altijd in de container. Wil je toch op de host bouwen:
> `sudo chown -R $USER:$USER ~/ros2_ws/build ~/ros2_ws/install ~/ros2_ws/log`.

---

## 3. Simulatie starten (terminal 1)

```bash
ros2 launch my_bot launch_sim.launch.py
```

Start Gazebo met de wereld `worlds/ziekenhuis_merged.sdf`, spawnt de robot en
start de bridge (`/clock`, `/cmd_vel`, `/scan`, `/odom`, `/tf`, `/joint_states`).

Controleren:

```bash
ros2 topic list            # /scan, /odom, /clock, /tf moeten er staan
ros2 topic hz /scan        # lidar publiceert?
```

---

## 4. Rijden met controller of toetsenbord (terminal 5)

Beide publiceren op `/cmd_vel`; de bridge stuurt dat door naar Gazebo.

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

## 5. Kaart maken met SLAM (terminal 2)

Zet eerst in `config/mapper_params_online_async.yaml` de modus op **mapping**
(staat standaard op `localization` met een kaart van een andere pc) en bouw
opnieuw:

```yaml
    mode: mapping #localization of mapping
```

Zet hem na het opslaan van de kaart weer terug als je op de SLAM-kaart wilt
lokaliseren.

```bash
ros2 launch my_bot online_async_launch.py use_sim_time:=true
```

Open RViz (terminal 4):

```bash
rviz2 -d /ros2_ws/src/navigatie_ezwheels/config/setup.rviz
```

- **Fixed Frame** = `map`
- Voeg een **Map**-display toe op topic `/map` als die er nog niet staat

Rij de robot met controller of toetsenbord door alle gangen en kamers:

- rustig rijden (± 0,2–0,3 m/s) en langzaam draaien geeft een nettere kaart
- eindig bij het startpunt, zodat SLAM de lus kan sluiten

### Kaart opslaan

Terwijl SLAM nog draait:

```bash
mkdir -p /ros2_ws/maps
ros2 run nav2_map_server map_saver_cli -f /ros2_ws/maps/ziekenhuis --ros-args -p use_sim_time:=true
```

Resultaat: `ziekenhuis.pgm` + `ziekenhuis.yaml` in `~/ros2_ws/maps/` (blijft
bewaard, want die map is gemount).

---

## 6. Navigatie testen met Nav2 (terminal 2 + 3)

Kies **A** (op de live SLAM-kaart) of **B** (op een opgeslagen kaart, zoals op
de echte robot).

**A. Met SLAM** — laat terminal 2 uit stap 5 draaien.

**B. Met AMCL op een opgeslagen kaart** — vervang SLAM in terminal 2 door:

```bash
ros2 launch my_bot localization_launch.py map:=/ros2_ws/maps/ziekenhuis.yaml use_sim_time:=true
```

Start daarna Nav2 (terminal 3):

```bash
ros2 launch my_bot navigation_launch.py use_sim_time:=true
```

> `use_sim_time:=true` niet vergeten — de standaard is `false` en dan krijg je
> TF-/tijdfouten.

Testen in RViz:

1. Alleen bij AMCL: **2D Pose Estimate** → klik waar de robot staat en in welke
   richting hij kijkt.
2. **2D Goal Pose** → klik een doel. De robot plant een pad en rijdt erheen.
3. Zet in Gazebo een obstakel op het pad om te zien of hij eromheen plant.

---

## 7. Eigen behavior tree toevoegen

Nav2 gebruikt standaard `navigate_to_pose_w_replanning_and_recovery.xml`. Een
eigen boom zet je in `behavior_trees/` (voorbeeld: `behavior_trees/test1.xml`).

### 1. XML-bestand maken

Maak bv. `behavior_trees/mijn_bt.xml`:

```xml
<root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <PipelineSequence name="PlannenEnVolgen">
            <RateController hz="1.0">
                <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
            </RateController>
            <FollowPath path="{path}" controller_id="FollowPath"/>
        </PipelineSequence>
    </BehaviorTree>
</root>
```

- `planner_id` / `controller_id` moeten overeenkomen met de namen in
  `config/nav2_params.yaml` (`GridBased`, `FollowPath`).
- Elke node die je gebruikt (`Spin`, `BackUp`, `ClearEntireCostmap`, …) moet
  een plugin hebben in `bt_navigator` → `plugin_lib_names` in
  `config/nav2_params.yaml`. Ontbreekt die, dan weigert Nav2 het doel met
  *"Error loading XML"* / *"node not recognized"* in de log van terminal 3.
- Herstel-acties gebruiken de behaviors uit `behavior_server` →
  `behavior_plugins` (`spin`, `backup`, `wait`, …).

### 2. Bouwen

De map `behavior_trees/` wordt al geïnstalleerd (`CMakeLists.txt`), dus nieuwe
bestanden komen vanzelf mee. Na elke wijziging in de XML: opnieuw bouwen.

```bash
cd /ros2_ws
colcon build --base-paths src/navigatie_ezwheels
source install/setup.bash
```

### 3a. Per doel gebruiken (testen)

`nav2_test_node.py` blijft draaien en stuurt elk doel dat je in RViz klikt naar
Nav2, met de gekozen boom. Een relatieve naam wordt gezocht in
`share/my_bot/behavior_trees/`:

```bash
ros2 run my_bot nav2_test_node.py --ros-args \
  -p behavior_tree:=mijn_bt.xml -p use_sim_time:=true
```

- **2D Goal Pose** in RViz publiceert (met `config/setup.rviz`) op
  `/goal_pose_bt`; de node maakt daar een Nav2-doel van. Een nieuw doel
  vervangt het lopende doel.
- **2D Pose Estimate** gaat zoals altijd rechtstreeks naar AMCL (`/initialpose`).
- Draait de node niet, dan doet **2D Goal Pose** niets: Nav2 zelf luistert op
  `/goal_pose`. Zet in RViz het topic van de tool terug op `/goal_pose` om
  zonder de node met de standaardboom te rijden.
- Zonder `behavior_tree` gebruikt de node de standaardboom. `patrol_node`
  gebruikt altijd de standaardboom.

### 3b. Als standaardboom instellen

Wil je dat álle doelen (ook RViz en patrouille) je boom gebruiken, zet dan in
`config/nav2_params.yaml` onder `bt_navigator` → `ros__parameters`:

```yaml
    default_nav_to_pose_bt_xml: /ros2_ws/install/my_bot/share/my_bot/behavior_trees/mijn_bt.xml
```

Opnieuw bouwen en Nav2 herstarten.

### Debuggen

- Log van Nav2 (terminal 3) toont fouten bij het laden van de XML.
- `ros2 topic echo /behavior_tree_log` laat zien welke node van status
  verandert (`RUNNING` → `SUCCESS` / `FAILURE`).

---

## 8. Eigen nodes starten en testen (terminal 6+)

Nav2 (stap 6) moet draaien. Geef elke node `use_sim_time:=true` mee.

### patrol_node — patrouille

Routes staan in `config/patrol_routes.yaml`, tijden en afstanden in
`config/patrol_params.yaml` (na wijzigen: opnieuw bouwen).

```bash
ros2 run my_bot patrol_node.py --ros-args \
  --params-file /ros2_ws/src/navigatie_ezwheels/config/patrol_params.yaml \
  -p use_sim_time:=true
```

Waypoints zijn `[x, y, yaw]` in het `map`-frame. Met SLAM is de oorsprong de
spawnplek van de robot; controleer in RViz met **Publish Point**
(`ros2 topic echo /clicked_point`) of de waypoints op een vrije plek liggen.

Route starten / stoppen en de status volgen:

```bash
ros2 topic pub --once /start_patrol  std_msgs/msg/Bool "{data: true}"   # route 1
ros2 topic pub --once /start_patrol2 std_msgs/msg/Bool "{data: true}"   # route 2
ros2 topic pub --once /stop_patrol   std_msgs/msg/Bool "{data: true}"   # stoppen

ros2 topic echo /patrol_state
```

**Blokkade testen:** zet tijdens het rijden in Gazebo een doos op het pad.
Verwacht verloop van `/patrol_state`: `rijdend` → `wachten` → alternatief /
terugkeren → `fout` als het niet lukt. In de simulatie is `wait_seconds` 5 s
(op de robot 3 minuten).

### indicator_node — knipperlichten en buzzer

```bash
ros2 run my_bot indicator_node.py --ros-args -p use_sim_time:=true

ros2 topic echo /indicators    # links / rechts / gevaar / uit
ros2 topic echo /buzzer
```

Rij een route met bochten; bij een bocht moet `links`/`rechts` verschijnen, bij
stilstand `gevaar`.

### environment_speed_node — snelheid vóór bochten

```bash
ros2 run my_bot environment_speed_node.py --ros-args -p use_sim_time:=true

ros2 topic echo /speed_limit   # 0.5 → 0.2 m/s vóór scherpe bochten
```

### mqtt_hmi_bridge — HMI (optioneel)

Heeft een MQTT-broker nodig. In de simulatie is er geen HMI; alleen nodig als je
de MQTT-kant wilt testen met een lokale broker (standaard `localhost:1883`):

```bash
ros2 run my_bot mqtt_hmi_bridge.py --ros-args -p broker_host:=localhost
```

---

## 9. Unittests (zonder simulatie)

De patrouille-logica heeft tests die zonder ROS, Nav2 of robot draaien:

```bash
cd /ros2_ws/src/navigatie_ezwheels
python3 -m unittest discover -s test -v
```

---

## Problemen oplossen

| Probleem | Oplossing |
|---|---|
| `RuntimeError: !rclpy.ok()` bij `ros2 topic ...` | `ros2 daemon stop` (container én host), of `--no-daemon` |
| `Duplicate package names` bij bouwen | `colcon build --base-paths src/navigatie_ezwheels` |
| `file ... was not found in the share directory` | Opnieuw bouwen en `source /ros2_ws/install/setup.bash` |
| `Operation not permitted` bij bouwen | `build/`/`install/` zijn van root → bouw in de container of `chown` |
| `/dev/input/js0` bestaat niet in de container | Container herstarten met de nieuwe `start.sh` |
| Robot rijdt niet met controller | X ingedrukt houden; check `ros2 topic echo /joy` en `/cmd_vel` |
| Nav2: TF- of tijdfouten | `use_sim_time:=true` vergeten |
| Robot rijdt niet met Nav2 | `ros2 topic echo /cmd_vel` — komt er iets uit Nav2? Draait SLAM/AMCL (frame `map`)? |
| Package niet gevonden (nav2, slam_toolbox, joy) | In de `Dockerfile` zetten en image opnieuw bouwen |
