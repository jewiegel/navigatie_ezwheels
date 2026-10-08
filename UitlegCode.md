# Uitleg van de code

Dit document legt uit hoe de navigatiemodule van de zorgrobot in elkaar zit:
welke onderdelen er zijn, waar het begint en hoe één patrouille van begin tot
eind verloopt. Voor de praktische commando's: zie
[StartenSImulatie.md](StartenSImulatie.md) (simulatie) en
[README.md](README.md) (echte robot).

---

## 1. Het grote plaatje

De robot (EZ-Wheel SWD basis) rijdt autonoom een vaste route van waypoints door
een zorginstelling. Hij moet **voorspelbaar** zijn: hij geeft richting aan met
knipperlichten, piept bij problemen en volgt een vaste escalatie als de weg
geblokkeerd is.

De software bestaat uit drie lagen:

```
┌──────────────────────────────────────────────────────────────────────┐
│  HMI (FT2J touchscreen)                                              │
│      knop start/stop            ▲ status, lampen, buzzer             │
└──────────│──────── MQTT ────────│──────────────────────────────────────┘
           ▼                      │
┌──────────────────────────────────────────────────────────────────────┐
│  Eigen nodes (deze repo, map my_bot/)                                │
│                                                                      │
│   mqtt_hmi_bridge ──/start_patrol──► patrol_node ──/patrol_state──┐  │
│        ▲                               │   ▲                      │  │
│        │ /indicators /buzzer           │   │ resultaat/feedback   ▼  │
│        └──────────── indicator_node ◄──┼───┼── /plan /cmd_vel /odom  │
│                      environment_speed_node ──/speed_limit──┐        │
└────────────────────────────────────────│───│────────────────│────────┘
                    NavigateThroughPoses │   │                │
                    NavigateToPose       ▼   │                ▼
┌──────────────────────────────────────────────────────────────────────┐
│  Nav2 (planner, controller, behavior tree) + AMCL/SLAM + kaart       │
│      /cmd_vel ▼                     ▲ /scan /odom /tf                │
└──────────────────────────────────────────────────────────────────────┘
           ▼                          │
┌──────────────────────────────────────────────────────────────────────┐
│  Robot: EZ-Wheel wielen + lidar     óf    Gazebo-simulatie           │
└──────────────────────────────────────────────────────────────────────┘
```

- **Onderste laag** — de robot zelf of Gazebo: rijdt op `/cmd_vel`, levert
  `/scan` (lidar), `/odom` (odometrie) en `/tf`.
- **Middelste laag** — Nav2: weet waar de robot is (AMCL of SLAM op een kaart),
  plant een pad naar een doel en stuurt de wielen aan.
- **Bovenste laag** — onze eigen nodes: bepalen *welke* doelen Nav2 krijgt
  (patrouille), wat er gebeurt bij een blokkade, en vertalen het gedrag naar
  lampen, buzzer en HMI.

---

## 2. Mapstructuur

| Map / bestand | Inhoud |
|---|---|
| `my_bot/` | Python-nodes (de eigen code) |
| `my_bot/patrol/` | Logica van de patrouille (ROS-vrij, testbaar) |
| `my_bot/patrol/navigation_states/` | De toestanden van de state machine |
| `my_bot/patrol/ros/` | Adapters die de logica aan ROS/Nav2 koppelen |
| `launch/` | Launchfiles voor simulatie, Nav2, SLAM/AMCL en de robot |
| `config/` | Parameters: Nav2, SLAM, routes, patrouille, RViz, twist_mux |
| `description/` | Robotmodel (URDF/xacro) incl. Gazebo-plugins |
| `worlds/` | Gazebo-werelden (ziekenhuis, obstakels) |
| `test/` | Unittests voor de patrouille-logica |
| `merge_boxes.py` | Hulpscript: voegt losse dozen in een SDF samen (sneller in Gazebo) |
| `CMakeLists.txt` | Installeert scripts, `patrol/` en de mappen `config launch description worlds` |

Het ROS-package heet **`my_bot`** (zie `package.xml`), ook al heet de map
`navigatie_ezwheels`.

---

## 3. Waar het begint: de launchfiles

Er zijn twee manieren om alles te starten: in de **simulatie** of op de
**echte robot**.

### 3a. Simulatie

In de simulatie start je de onderdelen los (zie StartenSImulatie.md):

```
launch_sim.launch.py          → Gazebo + robotmodel + bridge
online_async_launch.py        → SLAM Toolbox        ┐ één van
localization_launch.py        → map_server + AMCL   ┘ beide
navigation_launch.py          → Nav2
ros2 run my_bot <node>.py     → eigen nodes
```

**`launch/launch_sim.launch.py`** doet:

1. `rsp.launch.py` — zet het robotmodel (`description/robot.urdf.xacro`) om naar
   `robot_description` en start de **robot_state_publisher** (publiceert de
   vaste TF's tussen `base_link`, wielen en `laser_link`).
2. **Gazebo** (`ros_gz_sim`) met de wereld `worlds/ziekenhuis_merged.sdf`.
3. **create** — spawnt de robot in Gazebo vanuit `robot_description`.
4. **parameter_bridge** — koppelt Gazebo-topics aan ROS:
   `/clock`, `/cmd_vel`, `/scan`, `/odom`, `/tf`, `/joint_states`.
5. **twist_mux** en **joint_state_publisher** (zie aandachtspunten, §11).

### 3b. Echte robot

Op de robot draait alles in de Docker container van EZ-Way. Daar zijn twee
varianten:

- **`launch/my_bot_launch.py`** — alleen de eigen nodes (`patrol_node`,
  `indicator_node`, `mqtt_hmi_bridge`, `foxglove_bridge`). Nav2 en AMCL worden
  dan gestart door de EZ-Way software ("automatic mode"). Als één node stopt,
  stopt alles (`on_exit=Shutdown()`).
- **`launch/launch_robot.launch.py`** — de hele stack: robot_state_publisher,
  lidar (IDEC SE2L via `urg_node2`), twist_mux, AMCL + map_server en Nav2 (uit
  `nav2_bringup`, met `config/nav2_params_robot.yaml`), plus alle eigen nodes.

---

## 4. Robotmodel en simulatie

### Robotmodel (`description/`)

- `robot.urdf.xacro` — voegt de drie delen samen.
- `robotcore.xacro` — `base_footprint` → `base_link` (doos 0,5 × 0,4 m), twee
  aangedreven wielen (straal 0,1 m, 0,44 m uit elkaar) en een zwenkwiel achter.
- `lidar.xacro` — `laser_link` voorop, met een Gazebo `gpu_lidar` die op
  `/scan` publiceert.
- `gazebo_control.xacro` — Gazebo-plugins:
  - **DiffDrive**: luistert op `/cmd_vel`, publiceert `/odom` en de TF
    `odom → base_footprint`.
  - **JointStatePublisher**: wielstanden op `/joint_states`.

Op de echte robot doet de EZ-Wheel-driver wat DiffDrive in Gazebo doet.

### Werelden (`worlds/`)

- `ziekenhuis.sdf` — ziekenhuisplattegrond opgebouwd uit veel losse dozen.
- `ziekenhuis_merged.sdf` — dezelfde wereld, gemaakt met `merge_boxes.py`:
  alle dozen zijn één model met veel links, zodat Gazebo veel sneller rekent.
  **Deze wordt gebruikt.**
- `obstakels1.sdf`, `empty.world` — testwerelden.

### TF-boom

```
map ──(AMCL of SLAM)──► odom ──(DiffDrive / EZ-Wheel)──► base_footprint
                                     ──(robot_state_publisher)──► base_link ──► laser_link, wielen
```

---

## 5. Kaart en lokalisatie

Nav2 moet weten waar de robot op de kaart staat (`map → odom`). Dat kan op
twee manieren:

| Manier | Launchfile | Config | Wanneer |
|---|---|---|---|
| **SLAM Toolbox** | `online_async_launch.py` | `config/mapper_params_online_async.yaml` | Kaart maken (`mode: mapping`) of lokaliseren op een SLAM-kaart (`mode: localization`) |
| **AMCL** + map_server | `localization_launch.py` | `config/nav2_params.yaml` (sectie `amcl`) | Navigeren op een opgeslagen `.yaml` + `.pgm` kaart (zo werkt de robot) |

Een kaart maken = robot met controller rondrijden met SLAM in `mapping`-modus,
daarna opslaan met `map_saver_cli`.

---

## 6. Nav2

**`launch/navigation_launch.py`** is een kopie van de standaard Nav2-launchfile
en start: `controller_server`, `smoother_server`, `planner_server`,
`behavior_server`, `bt_navigator`, `waypoint_follower`, `velocity_smoother` en
een `lifecycle_manager` die ze activeert.

Belangrijke instellingen in `config/nav2_params.yaml` (simulatie) en
`config/nav2_params_robot.yaml` (robot; verschil is vooral
`use_sim_time: False`):

| Onderdeel | Instelling | Betekenis |
|---|---|---|
| `bt_navigator` | standaard behavior trees, plus `nav2_planner_selector_bt_node` | Hiermee kan de planner gekozen worden via `/planner_selector` |
| `planner_server` | `GridBased` = `NavfnPlanner` | Plant een pad om obstakels heen op de costmap |
| `controller_server` | `FollowPath` = `DWBLocalPlanner`, `max_vel_x: 0.5` | Volgt het pad en publiceert snelheden |
| `controller_server` | `speed_limit_topic: /speed_limit` | Hier kan `environment_speed_node` de snelheid begrenzen |
| costmaps | lidar `/scan`, `robot_radius: 0.22`, inflation | Obstakels en veiligheidsmarge |
| `behavior_server` | Spin, BackUp, DriveOnHeading, Wait | Ingebouwde herstelgedragingen van Nav2 |

Nav2 publiceert onderweg het geplande pad op **`/plan`** en de snelheden op
**`/cmd_vel`**. Die gebruiken onze nodes weer.

De EZ-Way software heeft zelf een planner `Waypoint` die in een rechte lijn
rijdt. `patrol_node` kiest daarom bij elke start expliciet **`GridBased`** via
`/planner_selector` (QoS transient_local, zodat Nav2 het ook ontvangt als het
later opstart).

---

## 7. patrol_node — het hart van het systeem

`patrol_node` bepaalt **wanneer** de robot rijdt, **waarheen** en **wat er
gebeurt als het misgaat**.

### 7.1 Opbouw: logica los van ROS ("ports & adapters")

```
patrol_node.py                     ← ROS-schil: maakt alles aan, koppelt topics
   │
   ├── patrol/ros/                 ← ADAPTERS (gebruiken rclpy / Nav2)
   │     ros_config.py             ROS-parameters + routebestand → PatrolConfig
   │     ros_scheduler.py          timers via de ROS-klok
   │     nav2_navigator.py         Nav2 action clients, cancel, controller-reset
   │     robot_signals.py          /cmd_vel, /buzzer, /indicators, /patrol_state
   │
   └── patrol/                     ← LOGICA (geen ROS, volledig testbaar)
         ports.py                  interfaces: Navigator, Signals, Scheduler, Logger
         config.py                 PatrolConfig + routes inlezen
         controller.py             PatrolController (de state machine)
         navigation_states/        de toestanden
         blockage_ladder.py        escalatie bij blokkades
         listeners.py              reacties op toestandswissels
         safety_monitor.py         detectie van EZ-Wheel veiligheidsstop
```

De controller en de toestanden praten alleen met de **interfaces** uit
`ports.py`. Op de robot vullen de adapters uit `patrol/ros/` die in; in de
unittests doen `test/fakes.py` dat. Zo kun je de hele patrouille-logica testen
zonder ROS, Nav2 of robot.

### 7.2 Opstarten van de node

`PatrolNode.__init__` (in `patrol_node.py`) doet in volgorde:

1. **`load_config(self)`** (`ros_config.py`) — elk veld van `PatrolConfig` wordt
   een ROS-parameter (waarden uit `config/patrol_params.yaml`, anders de
   standaard uit `config.py`). Routes komen uit `config/patrol_routes.yaml` in
   de share-map; ontbreekt dat bestand, dan de ingebouwde `DEFAULT_ROUTES`.
2. Adapters maken: `RosScheduler`, `RobotSignals`, `Nav2Navigator`.
3. **`PatrolController`** maken en drie listeners koppelen:
   `TransitionLogger`, `HmiStatePublisher`, `IndicatorPolicy`.
4. Per route een subscriber op zijn `start_topic` (bv. `/start_patrol`,
   `/start_patrol2`) → `controller.start(route_id)`. Plus `/stop_patrol` →
   `controller.stop()`.
5. Optioneel de `SafetyMonitor` (alleen als `safety_monitor_enabled: true`).
6. Planner `GridBased` selecteren en **`controller.begin()`** → toestand `idle`.

### 7.3 De routes

`config/patrol_routes.yaml`:

```yaml
routes:
  - id: 1
    start_topic: /start_patrol
    waypoints:
      - [x, y, yaw]     # meter / radialen in het map-frame
```

Een route toevoegen = een blok bijzetten en opnieuw bouwen; geen code nodig.

### 7.4 De state machine

Elke toestand is een klasse in `navigation_states/`. Bij elke wissel wordt een
**nieuwe** toestand aangemaakt. Een toestand is eigenaar van zijn eigen timers
en callbacks: bij het verlaten worden zijn timers geannuleerd en worden late
callbacks (bv. een laat Nav2-resultaat) genegeerd via `guard()`
(`navigation_state.py`).

| Toestand | Op `/patrol_state` | Wat hij doet | Start? | Stop? |
|---|---|---|---|---|
| `IdleState` | `idle` | Niets, wacht op start | ✔ | |
| `StartingState` | `planning` | Oude "zombie"-doelen annuleren, `controller_server` resetten, ~8 s wachten | | ✔ |
| `DrivingState` | `rijdend` (of `wachten` bij veiligheidsstop) | Alle resterende waypoints als één `NavigateThroughPoses`-doel; houdt voortgang en timeout bij | | ✔ |
| `WaitingState` | `wachten` | Stilstaan voor X seconden, daarna een vervolgactie | | ✔ |
| `BackingUpState` | `wachten` | 0,5 m achteruit met 0,1 m/s, daarna opnieuw rijden | | ✔ |
| `ReturningState` | `rijdend` | `NavigateToPose` naar het laatst gehaalde waypoint; eindigt altijd in `fout` | | ✔ |
| `CompletedState` | `voltooid` | Route klaar, stilstaan | ✔ | |
| `StoppedState` | `gestopt` | Doel annuleren, wielen stoppen | ✔ | |
| `ErrorState` | `fout` | Doel annuleren, wielen stoppen; operator moet ingrijpen | ✔ | |

```
            start                 na ~8 s              Nav2: SUCCEEDED
  idle ───────────► planning ───────────► rijdend ────────────────────► voltooid
   ▲                                       │  ▲
   │                         blokkade      │  │ opnieuw rijden
   │                (Nav2 ABORTED, timeout,▼  │
   │                 doel geweigerd)   escalatieladder (§7.5)
   │                                       │
   │                                       ▼ ladder op
   └──── start mag weer vanuit ───────── fout
         voltooid / gestopt / fout

  stop (vanuit planning/rijdend/wachten/terugkeren) ──► gestopt
```

### 7.5 Blokkade-afhandeling (escalatieladder)

Een **blokkade** is: Nav2 meldt dat de route mislukt (ABORTED), Nav2 weigert
het doel (ook na 1 retry), of er is `nav_timeout_sec` lang geen waypoint
gepasseerd.

Bij elke blokkade roept de toestand `controller.on_blocked()` aan, die de
**volgende stap** van de ladder uitvoert (`blockage_ladder.py`):

| Stap | Klasse | Actie | Daarna |
|---|---|---|---|
| 1 | `WaitStep` | Stilstaan `wait_seconds` (sim: 5 s) zodat mensen opzij kunnen | direct stap 2 |
| 2 | `BackupStep` | Piep + `backup_distance` achteruit | opnieuw rijden (nieuw pad) |
| 3 | `AlternativeWaypointStep` | Huidig waypoint overslaan, via het volgende verder (geen volgend → terugkeren) | korte pauze (`renav_pause`), dan rijden |
| 4 | `ReturnStep` | Terug naar het laatst gehaalde waypoint | `fout` |
| — | ladder op | | `fout` |

Belangrijk: zodra de robot een waypoint **passeert**, gaat de ladder terug naar
stap 1 (`waypoint_passed`). Een volgend obstakel verderop krijgt dus weer de
hele ladder.

De volgorde aanpassen of een stap weglaten = `DEFAULT_LADDER` wijzigen.

### 7.6 Hoe de voortgang bijgehouden wordt

`DrivingState` stuurt alle resterende waypoints in één keer naar Nav2
(`NavigateThroughPoses`). Nav2 stuurt als feedback `number_of_poses_remaining`.
Daalt dat getal, dan is een waypoint gepasseerd:

- `current_index` schuift op, `last_successful_idx` = het gepasseerde waypoint;
- de timeout per waypoint begint opnieuw;
- de ladder wordt gereset.

### 7.7 De Nav2-adapter (`nav2_navigator.py`)

- **Volgnummers per doel:** elk doel krijgt een `seq`. Na `cancel()` of een
  nieuw doel worden callbacks van het oude doel genegeerd. Zo kan een laat
  `CANCELED` nooit per ongeluk een escalatiestap starten.
- **`prepare()`** (bij `planning`): annuleert alle openstaande doelen op beide
  action servers ("zombie-doelen" van een vorige run) en deactiveert de
  `controller_server`, zodat zijn interne staat gewist wordt; de
  `lifecycle_manager` zet hem weer aan. Daarna wacht `StartingState`
  `controller_reset_delay` (8 s).
- **`shutdown()`**: bij Ctrl+C / SIGTERM wordt het actieve doel netjes
  geannuleerd, zodat de robot niet doorrijdt.

### 7.8 Listeners (reageren op elke toestandswissel)

| Listener | Doet |
|---|---|
| `TransitionLogger` | Logt `[STATE] oud → nieuw` |
| `HmiStatePublisher` | Publiceert de naam op `/patrol_state` |
| `IndicatorPolicy` | Zet `/indicators`: `gevaar` bij achteruit/terugkeren/voltooid/fout, `uit` bij idle/rijdend/gestopt |
| `SafetyMonitor` | Reset zijn eigen detectie bij een wissel |

### 7.9 SafetyMonitor (standaard uit)

De EZ-Wheel stopt zelf als er een persoon voor staat; Nav2 merkt dat niet
direct. De monitor kijkt naar `/odom`: stond de robot in `rijdend` stil voor
langer dan `safety_still_seconds` (15 s), dan meldt hij `wachten` aan de HMI.
Duurt het langer dan `person_wait_seconds` (120 s), dan escaleert hij naar de
ladder (en slaat de wachtstap over, want er is al gewacht). Aanzetten met
`safety_monitor_enabled: true`.

---

## 8. indicator_node — knipperlichten en buzzer

`my_bot/indicator_node.py` luistert op `/patrol_state`, `/plan`, `/cmd_vel` en
`/odom` en publiceert `/indicators`, `/buzzer` en `/indicator_status`.

| Situatie | Hoe gedetecteerd | Resultaat |
|---|---|---|
| Bocht vooruit | `/plan`: hoek tussen begin→midden en midden→eind van de eerste 60 % van het pad > 45° | `links` / `rechts` aan |
| Robot draait | `/cmd_vel`: rijdt (> 0,05 m/s) én draait > 0,20 rad/s | `links` / `rechts` aan |
| Bocht voorbij | `/cmd_vel`: draaisnelheid < 0,17 rad/s én lamp brandt ≥ 2 s | `uit` |
| Stilstand tijdens rijden/wachten | `/odom` < 0,05 m/s langer dan 1,2 s | `gevaar` knippert |
| `fout` of `voltooid` | `/patrol_state` | `gevaar` knippert continu |
| Naar `gestopt`, `wachten` of `fout` | `/patrol_state` | één piep (0,3 s) |

Knipperlichten werken alleen in de toestand `rijdend`. Elke 0,5 s wordt een
diagnoseregel op `/indicator_status` gezet (naar de HMI).

`my_bot/test_indicators.py` is een testnode die alle standen doorloopt, zodat
je op de HMI kunt zien of de lampen reageren.

---

## 9. environment_speed_node — afremmen voor bochten

`my_bot/environment_speed_node.py` kijkt naar de eerste **2 m** van `/plan`.
Zit daar een bocht > 45° in, dan publiceert hij op `/speed_limit` 0,2 m/s; is
het pad weer recht, dan 0,5 m/s. Bij `idle/gestopt/voltooid/fout` zet hij de
snelheid terug naar normaal. Nav2's `controller_server` past de limiet toe.

> In `my_bot_launch.py` staat deze node **uit** ("liet de robot kruipen"); in
> `launch_robot.launch.py` staat hij aan.

---

## 10. mqtt_hmi_bridge — koppeling met de HMI

`my_bot/mqtt_hmi_bridge.py` verbindt ROS met de FT2J HMI via een MQTT-broker
(Mosquitto, parameter `broker_host`/`broker_port`).

| Richting | MQTT-topic | ROS-topic |
|---|---|---|
| HMI → robot | `robot/start_patrol` (`"1"`) | `/start_patrol` |
| HMI → robot | `robot/stop_patrol` (`"1"`) | `/stop_patrol` |
| robot → HMI | `robot/patrol_state` | `/patrol_state` |
| robot → HMI | `robot/indicator_status` | `/indicator_status` |
| robot → HMI | `robot/outputs/buzzer` (`1`/`0`) | `/buzzer` |
| robot → HMI | `robot/outputs/lamp_links` / `lamp_rechts` (`1`/`0`) | `/indicators` (`gevaar` = beide) |

---

## 11. Het hele verloop van één patrouille

Een voorbeeld van begin tot eind, op de echte robot:

1. **Opstarten.** Nav2 + AMCL draaien met de kaart. De eigen nodes starten;
   `patrol_node` leest config en routes, kiest `GridBased` en publiceert
   `idle`. De HMI toont "idle".
2. **Knop op de HMI.** De HMI stuurt `robot/start_patrol = "1"`.
   `mqtt_hmi_bridge` maakt daar `/start_patrol = true` van.
3. **Start.** `PatrolController.start(1)`: voortgang op 0, wielen stop, planner
   opnieuw `GridBased`, toestand **`planning`**. `Nav2Navigator.prepare()`
   annuleert oude doelen en reset de `controller_server`; na 8 s →
   **`rijdend`**.
4. **Rijden.** `DrivingState` stuurt alle waypoints als één
   `NavigateThroughPoses`. Nav2 plant (`/plan`) en rijdt (`/cmd_vel`).
   - `indicator_node` ziet een bocht in `/plan` of `/cmd_vel` → `links`/`rechts`
     → bridge → lamp op de HMI.
   - `environment_speed_node` ziet een bocht in de komende 2 m → `/speed_limit`
     0,2 m/s.
   - Elke keer dat de feedback een waypoint minder meldt: index omhoog,
     timeout opnieuw, ladder gereset.
5. **Blokkade (bv. bed in de gang).** Nav2 meldt ABORTED of de timeout
   verloopt → `on_blocked()`:
   1. `wachten` 5 s (piep via `indicator_node`, gevarenlichten bij stilstand);
   2. piep + 0,5 m achteruit (`wachten`, `gevaar`), dan opnieuw `rijdend`;
   3. lukt het weer niet → waypoint overslaan, via het volgende;
   4. lukt dat ook niet → terug naar het laatst gehaalde waypoint → **`fout`**.
6. **Einde.** Nav2 meldt SUCCEEDED → **`voltooid`**: wielen stop,
   gevarenlichten knipperen, HMI toont "voltooid". Een nieuwe start mag.
7. **Stoppen** kan op elk moment tijdens de route: `/stop_patrol` →
   **`gestopt`** (doel geannuleerd, wielen stop).

---

## 12. Tests

`test/test_patrol_controller.py` test de patrouille-logica zonder ROS: start,
stop, voortgang, de hele escalatieladder, timeouts, late callbacks, routes
inlezen. `test/fakes.py` speelt Nav2, de klok en de signalen na (bv.
`rig.nav.accept()`, `rig.nav.feedback(2)`, tijd vooruitzetten).

```bash
cd navigatie_ezwheels
python3 -m unittest discover -s test -v
```

---

## 13. Aandachtspunten

Dingen die opvielen bij het doorlezen en die verwarrend kunnen zijn:

1. **Twee bronnen voor `/indicators` en `/buzzer`.** Zowel `patrol_node`
   (`IndicatorPolicy`, `BackupStep.beep()`) als `indicator_node` publiceren
   hierop. Ze kunnen elkaar dus overschrijven (bv. `IndicatorPolicy` zet `uit`
   bij `rijdend`, terwijl `indicator_node` net `links` zet).
2. **HMI start alleen route 1.** `mqtt_hmi_bridge` publiceert alleen
   `/start_patrol`; `/start_patrol2` (route 2) is niet via MQTT te starten.
3. **SLAM-config staat op `mode: localization`** met
   `map_file_name: /home/linuxkoenp/...` (pad van een andere pc). Voor een
   nieuwe kaart maken moet dit op `mode: mapping` staan.
4. **twist_mux in de simulatie werkt niet mee.** In `launch_sim.launch.py` is
   het params-pad relatief (`my_bot/config/twist_mux.yaml`) en gaat de uitvoer
   naar `/diff_cont/cmd_vel_unstamped`, die Gazebo niet leest. Nav2 en teleop
   rijden daarom direct via `/cmd_vel`.
5. **Wachttijd README vs. config.** De README noemt 3 minuten wachten bij een
   blokkade; `patrol_params.yaml` staat op 5 s.
6. **`launch_robot.launch.py` gebruikt de Nav2-launchfiles uit `nav2_bringup`**,
   niet de kopieën in `launch/`. De kopieën worden alleen in de simulatie
   gebruikt.
7. **`SafetyMonitor` staat uit** (`safety_monitor_enabled: False`, "tijdelijk
   uit voor debugging").
8. **Twee packages met de naam `my_bot`** in `~/ros2_ws/src` (`my_bot/` en
   `navigatie_ezwheels/`). Bouw met
   `colcon build --base-paths src/navigatie_ezwheels`.
