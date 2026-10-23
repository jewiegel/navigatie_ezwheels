# Webdashboard: plan

Dit plan is opgesteld door drie agents: ros-explorer (inventaris van de repo), architect (ontwerp) en critic (risico's en tegenspraak). Waar ze het oneens waren, staat de keuze bij [Besluiten](#besluiten-uit-het-overleg).

## Doel

Het dashboard moet vier dingen kunnen:

1. Een strak, modern design en layout hebben.
2. De map tonen (map_server, slam_toolbox, met AMCL-lokalisatie).
3. Een route maken: waypoints klikken, een lijn tussen de waypoints in volgorde tonen, en de route naar Nav2 sturen.
4. De robotstatus tonen.

## Uitgangspunten uit de repo

- **foxglove_bridge draait al.** `robot.launch.py` start hem op :8765 met de standaardinstellingen (alleen `port` wordt gezet, [robot.launch.py:104](../hospital_robot_bringup/launch/robot.launch.py#L104)). rosbridge is niet geïnstalleerd.
- **Bestaande route-API.** `route_manager` ([RouteManagerServer.py](../my_bot/RouteManagerServer.py)) heeft `route_draft/add_pose`, `route_draft/poses` (latched), `undo`, `clear`, `save`, `get_route` en `route_preview`. Routes staan in `routes.yaml`.
- **mission_manager is de enige Nav2-client.** Hij stuurt de hele route als één `NavigateThroughPoses`-goal ([Nav2ActionClient.py](../my_bot/mission_manager/ports/Nav2ActionClient.py)). De BT-blokkadelogica, signal_manager (lampen en buzzer) en de HMI hangen allemaal aan dit pad. Hij biedt `start_route` en `cancel_mission` aan.
- **Map en lokalisatie.** De runtime-lokalisatie is map_server + AMCL. slam_toolbox wordt door geen enkel launch-bestand gestart en heeft een hardcoded pad (`/home/linuxkoenp/...`) in `mapper_params_online_async.yaml`.
- **Frames.** `map`, `odom` en `base_link`. In de sim gebruikt AMCL `base_footprint`.
- **Platform.** De robot draait ROS 2 **Iron** in de vendor-container `ez-way-v2-ros2-iron-amr`. Iron is sinds 4 dec 2024 EOL. Losse `apt install`s verdwijnen bij een rebuild van de container.

## Tech stack

| Laag | Keuze | Waarom |
|---|---|---|
| Transport | **foxglove_bridge** (bestaand, Iron 0.8.1) met `@foxglove/ws-protocol`, `@foxglove/rosmsg` en `@foxglove/rosmsg2-serialization` | Er hoeft niets nieuws in de vendor-image. Binaire CDR kost ongeveer 1 byte per cel, tegen 3–4 bytes in JSON bij rosbridge. Abonneren, publiceren en services aanroepen werken. Actions zijn niet nodig. |
| Frontend | Vite + React + TypeScript + Tailwind | Klein en gangbaar. Geen component- of drag-and-drop-library: ↑/↓/✕-knoppen zijn genoeg. |
| Map-rendering | Canvas 2D (offscreen bitmap met `setTransform` voor pan en zoom) | Geen WebGL of kaartlibrary nodig voor een statische OccupancyGrid. |
| Serveren | `python3 -m http.server 8080 --bind <ip> --directory <share>/web` via ExecuteProcess | Alleen de standaardbibliotheek. De pagina verbindt met `ws://${location.hostname}:8765`. |
| Backend-aanvulling | 2 kleine nodes/services in `my_bot` (zie de ROS-interfaces hieronder) | Lost race conditions en TF-belasting op aan de robotkant. |

**Afgewezen:**
- **rosbridge + roslibjs.** Het moet in een EOL-image geïnstalleerd worden. Action-cancel-fixes (2.2.0) zijn nooit voor Iron uitgekomen. Het blijft de fallback: alleen `ros.ts` hoeft dan te veranderen.
- **Een eigen FastAPI-backend.** Meer code en een extra proces.
- **Direct een Nav2-action vanuit de browser** (FollowWaypoints of NavigateThroughPoses). Zie besluit 1.

## Mappenstructuur

```
hospital_robot_dashboard/            # nieuw ament_cmake-pakket, geen ROS-code
  package.xml
  CMakeLists.txt                     # installeert web/dist + launch; faalt duidelijk als dist ontbreekt
  launch/dashboard.launch.py         # http.server :8080
  web/
    package.json, vite.config.ts, index.html
    src/main.tsx, App.tsx, index.css
    src/ros.ts                       # foxglove-client: connect/reconnect, subscribe, publish, callService
    src/mapMath.ts                   # grid↔wereld, yaw-berekening
    src/mapMath.test.ts              # vitest: rondgang klik→map→scherm (ook θ≠0), y-flip, yaw
    src/MapView.tsx                  # canvas, pan/zoom, lagen: map, /plan, route, robot
    src/RoutePanel.tsx               # waypointlijst, herschikken/verwijderen, naam, Verstuur/Annuleer
    src/StatusPanel.tsx              # missie, blokkade, snelheid, lokalisatie, HMI, verbinding
my_bot/RobotPoseNode.py              # nieuw: map→base_link als PoseStamped @5 Hz
my_bot_interfaces/srv/SetRoute.srv   # nieuw: atomair route opslaan
```

**Layout:** het map-canvas vult het scherm. Rechts staat een sidebar met drie kaarten: Status, Route en Verbinding. Het palet is neutraal, met dark mode via `prefers-color-scheme`. Een "Verbinding verbroken"-banner schakelt alle bedieningsknoppen uit.

## ROS-interfaces

### Lezen (abonneren)

| Interface | Type | Gebruik | Let op |
|---|---|---|---|
| `/map_server/map` (service) | nav_msgs/srv/GetMap | Map ophalen bij laden en bij reconnect | Omzeilt de latched-race (risico 4) |
| `/map` | nav_msgs/OccupancyGrid | Map-updates | transient_local; bij slam_toolbox elke 5 s een volledige grid |
| `robot_pose` (nieuw) | geometry_msgs/PoseStamped | Robotpositie | 5 Hz, frame `map` |
| `/amcl_pose` | PoseWithCovarianceStamped | Alleen de lokalisatiekwaliteit (σxy groen/oranje/rood) | Publiceert alleen na een AMCL-update |
| `/plan` | nav_msgs/Path | Gepland pad tekenen | |
| `route_draft/poses`, `route_preview` | PoseArray | Concept- en voorbeeldroute | latched |
| `remaining_waypoints` | PoseArray | Gepasseerde waypoints grijs maken (voortgang) | latched |
| `mission_status` | std_msgs/String `"<STATE> <route>"` | Statusbadge (IDLE, DRIVING, BLOCKED, WAITING_OPERATOR, ARRIVED, ERROR) | Wordt transient_local (taak 7) |
| `navigation/blocked` | std_msgs/Bool | Rode banner | latched |
| `/odom` | nav_msgs/Odometry | Snelheid | Client-side throttlen |
| `hmi/mqtt_connected` | std_msgs/Bool | HMI-verbinding | latched |
| `indicators` | std_msgs/String | Knipperlichten | |

### Schrijven

| Interface | Type | Gebruik |
|---|---|---|
| `route_set` (nieuw) | my_bot_interfaces/srv/SetRoute: `string name, Waypoint[] waypoints, bool overwrite` → `bool success, string message` | Route in één call opslaan |
| `get_route` | my_bot_interfaces/srv/GetRoute | Bestaande route in de editor laden (op naam) |
| `start_route` | my_bot_interfaces/srv/StartRoute | Route starten via mission_manager, en daarmee Nav2 NavigateThroughPoses |
| `cancel_mission` | std_srvs/Trigger | Annuleren |

### Ontbreekt bewust in v1

- **Batterij.** Er is geen topic. Toon "n.v.t.", niet verbergen.
- **Voortgang uit NavigateThroughPoses-feedback.** mission_manager stuurt die niet door; `remaining_waypoints` volstaat.
- **Lijst van routes.** Er is geen service.
- **Initial pose zetten, costmaps en particle cloud.**
- **Een slam_toolbox-launchoptie.** Dat is aparte bringup-taak, geen dashboardwerk. De rendering werkt al op elke `/map`.

## Kernformules (`mapMath.ts`)

Invoer: `res`, origin `(ox, oy)`, `θ = 2·atan2(qz, qw)` van de origin en `H` = mapheight in cellen.

- **Grid naar bitmap.** Waarde -1 wordt grijs (205), 0 wit (254) en p (1..100) wordt `255 - round(p·2.55)`. Datarij j komt in beeldrij `H-1-j` (y-flip).
- **Klik naar map:**
  - `px=(sx-panX)/zoom`, `py=(sy-panY)/zoom`
  - `lx=px·res`, `ly=(H-py)·res`
  - `x=ox+cosθ·lx-sinθ·ly`, `y=oy+sinθ·lx+cosθ·ly`
- **Map naar scherm:**
  - `dx=x-ox`, `dy=y-oy`
  - `lx=cosθ·dx+sinθ·dy`, `ly=-sinθ·dx+cosθ·dy`
  - `px=lx/res`, `py=H-ly/res`
  - Voor een heading op het canvas: `angle=-(yaw-θ)`
- **Waypoint-yaw:**
  - Waypoint k krijgt `yaw_k = atan2(y[k+1]-y[k], x[k+1]-x[k])`.
  - Het laatste punt krijgt standaard de heading van het laatste segment. De gebruiker kan die heading draaien (sleep aan een pijl), want Nav2 controleert juist de eindoriëntatie.
  - Punten die (vrijwel) samenvallen met de vorige worden geweigerd, omdat `atan2(0,0)` stil 0 geeft.
  - Na elke add, delete of herschikking worden alle headings opnieuw berekend.

## Besluiten uit het overleg

1. **Nooit direct naar Nav2.** Dit wilden architect en critic allebei. Een tweede Nav2-client vecht met mission_manager. mission_status, de lampen en de HMI zouden dan IDLE tonen terwijl de robot rijdt, en `cancel_mission` zou dat goal niet stoppen. Daarom gaat het dashboard altijd via `start_route` en `cancel_mission`.

2. **Een atomaire `route_set`-service in plaats van clear + N×add_pose.**
   - De architect stelde voor om clear aan te roepen, N poses te publiceren, op de echo te wachten en dan op te slaan.
   - De critic toonde aan dat dit racy is:
     - De eerste publishes gaan verloren zolang de DDS-match er nog niet is.
     - Een burst van meer dan 10 poses overloopt de keep-last-diepte van 10.
     - `clear` wist een concept dat iemand anders in RViz of Foxglove aan het maken is.
   - Een service van ongeveer 15 regels in `RouteManagerServer.py` is kleiner dan de workaround aan de clientkant.
   - De route wordt opgeslagen onder de naam die de gebruiker invoert, niet altijd onder "dashboard". Zo komt er geen verborgen route in de HMI-lijst.

3. **De robotpose komt van een `robot_pose`-node, niet van TF in de browser.**
   - De architect wilde `/tf` in de browser samenstellen.
   - De critic: foxglove_bridge kan per client niet throttlen. Dan komt `/tf` op ongeveer 50 Hz binnen en wordt het op de main thread gedecodeerd, wat wankel is op ziekenhuis-wifi of een tablet. De framenamen verschillen bovendien tussen sim en robot.
   - Gekozen: een node van ongeveer 30 regels met een tf2-buffer, die `robot_pose` op 5 Hz publiceert. De parameter `base_frame` staat standaard op `base_link` en is in de sim `base_footprint`.

4. **De map wordt geladen via GetMap en daarna via `/map`.** Latched delivery hangt af van of de publisher al bestaat op het moment dat de client abonneert. Als de browser vóór map_server actief is verbindt, komt de map nooit. GetMap bij laden en bij reconnect is de zekerheid.

5. **`mission_status` wordt TRANSIENT_LOCAL met depth 1.** Een browser die later opent ziet zo direct de huidige status. Volatile subscribers (signal_manager en hmi_bridge) blijven compatibel.

6. **Beveiliging zit in v1, niet in "later".**
   - Port 8765 geeft nu al iedereen op het netwerk `cmd_vel`, lifecycle-services en Nav2-parameters, zonder authenticatie.
   - Minimaal:
     - `capabilities: [clientPublish, services, connectionGraph]` (geen `parameters`)
     - `client_topic_whitelist: []`, want na besluit 2 publiceert de client niets meer
     - `service_whitelist` en `topic_whitelist` beperkt tot de tabellen hierboven
     - `send_buffer_limit` expliciet hoog genoeg (zie risico 4)
   - Voor productie: de bridge en http.server binden aan localhost, met nginx (TLS + basic auth) of VPN ervoor.

7. **Iron EOL: niet migreren voor dit dashboard.** Iron is de basis van de vendor (EZ-Way drivers, CycloneDDS). foxglove_bridge 0.8.1 zit al in het launch-pad, en dat moet in de image ingebakken zijn. Het wordt een apart projectrisico voor de eigenaar, met Jazzy als vervolgstap. Pin daarom ook de `@foxglove/ws-protocol`-versie die met 0.8.1 praat.

8. **Het dashboard is een apart pakket**, `hospital_robot_dashboard`, volgens dezelfde naamgeving als de andere `hospital_robot_*`-pakketten. `web/dist` wordt buiten de robot gebouwd, want de container heeft geen Node. Committen of in CI bouwen is nog open (zie open vragen).

9. **Alleen map_server in v1, slam_toolbox niet.** De rendering werkt met elke `/map`, maar slam_toolbox wordt niet gelaunched en de config is kapot (hardcoded pad). Hem daarvoor ondersteunen is nu YAGNI.

## Risico's (gerangschikt)

| # | Risico | Mitigatie |
|---|---|---|
| 1 | Onbeveiligde bridge: iedereen op het LAN kan de robot besturen | Besluit 6: whitelists en capabilities in v1, plus een proxy met auth voor productie |
| 2 | Verkeerde of halve route gereden, of het concept van een collega gewist | Besluit 2: atomaire `route_set` |
| 3 | Betekenisloze eind-heading terwijl Nav2 die juist controleert | Eind-heading in te stellen, standaard de laatste segmentrichting |
| 4 | De map komt stil nooit aan: latched-race, of `send_buffer_limit` (vermoedelijk ~10 MB) wordt overschreden | GetMap bij laden. Meet de echte mapgrootte (`ziekenhuis.pgm`) en zet `send_buffer_limit` expliciet |
| 5 | De browser trekt `/tf` op volle rate niet | Besluit 3: `robot_pose` op 5 Hz |
| 6 | EOL-stack en packaging-drift (image, verouderde `dist`, protocolversie) | Pakket in de image bakken, versies pinnen, build-check |
| 7 | Meerdere bedieners (HMI, twee browsers) starten of annuleren tegelijk | mission_manager blijft de enige arbiter; het dashboard toont altijd de actuele `mission_status` |
| 8 | Reconnect: de websocket valt weg op wifi | `ros.ts` met een reconnect-loop die opnieuw abonneert; controls uit tijdens de disconnect |

## Taken

| # | Taak | Bestanden | Hangt af van |
|---|---|---|---|
| 0 | **Verifiëren op de robot** (een halve dag, vóór de bouw): <br>• foxglove_bridge-versie en standaard-capabilities<br>• `send_buffer_limit`<br>• latched `/map` bij twee clients<br>• grootte van `ziekenhuis.pgm`<br>• het werkelijke TF-framepad | alleen notities in dit document | |
| 1 | **Scaffold**: pakket, Vite-app, layout-shell, `dashboard:=true`-arg | `hospital_robot_dashboard/{package.xml,CMakeLists.txt,launch/dashboard.launch.py}`, `web/{package.json,vite.config.ts,index.html,src/main.tsx,src/App.tsx,src/index.css}`, `hospital_robot_bringup/launch/{robot,sim}.launch.py` | |
| 2 | **Transport**: connect, reconnect, schema-decode, subscribe, publish en callService; verbindingsbadge | `web/src/ros.ts`, `web/src/App.tsx` | 1 |
| 3 | **Map-wiskunde met test** | `web/src/mapMath.ts`, `web/src/mapMath.test.ts` | 1 |
| 4 | **MapView**: GetMap en `/map` naar bitmap, pan en zoom, klikken om toe te voegen, `/plan`-laag | `web/src/MapView.tsx` | 2, 3 |
| 5 | **Robotpose**: de node plus tekenen | `my_bot/RobotPoseNode.py`, `CMakeLists.txt` (install), `hospital_robot_bringup/launch/{robot,sim}.launch.py`, `web/src/MapView.tsx` | 4 |
| 6 | **Route-editor en verzenden**: lijst, herschikken, verwijderen, eind-heading, naam, Verstuur (`route_set` en daarna `start_route`), Annuleer, Laad (`get_route`), voortgang via `remaining_waypoints` | `my_bot_interfaces/srv/SetRoute.srv`, `my_bot_interfaces/CMakeLists.txt`, `my_bot/RouteManagerServer.py` (service plus test), `web/src/RoutePanel.tsx`, `web/src/MapView.tsx` | 4 |
| 7 | **Status**: alle statustopics, en `mission_status` latched maken | `web/src/StatusPanel.tsx`, `my_bot/mission_manager/ports/Ros2Notifier.py` | 2 |
| 8 | **Beveiliging en bridge-config**: capabilities, whitelists, `send_buffer_limit`, bind-adressen | `hospital_robot_bringup/launch/{robot,sim}.launch.py` | 0, 6, 7 |
| 9 | **Docs**: `npm run build`, URL `http://<robot>:8080`, de Iron-EOL-notitie | `startup.md`, `StartenSImulatie.md` | alles |

Taak 2 en 3 kunnen parallel lopen, en daarna ook 5, 6 en 7.

## Open vragen voor de eigenaar

- `web/dist` committen, of bouwen in CI of in een Dockerfile-stage? Committen is het simpelst, maar `dist` kan dan ongemerkt verouderen ten opzichte van de broncode.
- Moet foxglove_bridge in productie open blijven voor Foxglove Studio? De whitelists uit besluit 6 beperken dat ook.
- Biedt EZ-Way een batterijtopic aan? Dan is het een kleine toevoeging aan StatusPanel.
