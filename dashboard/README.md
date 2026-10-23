# EZ-Wheels webdashboard

Een website voor deze robot. Je kunt er de map bekijken (map_server, slam_toolbox of AMCL), een route met waypoints maken en naar Nav2 sturen, en de robotstatus zien. De site praat met ROS 2 via rosbridge (websocket, poort 9090).

## Op de robot: rosbridge starten

rosbridge zit niet in de vendor-image. Installeer het daarom in de Dockerfile; een losse `apt install` verdwijnt bij een rebuild.

```bash
sudo apt install ros-iron-rosbridge-suite
source ~/ros2_ws/install/setup.bash    # zodat de eigen msg/srv-types bekend zijn
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

## Ontwikkelen

```bash
cd dashboard
npm install
npm run dev        # http://localhost:5173  (verbindt met ws://<host>:9090)
npm test           # vitest
npm run build      # statische site in dist/
```

Gebruik `?ros=ws://<robot-ip>:9090` in de URL om met een andere robot te verbinden.

## Serveren

`dist/` is een statische site. Je kunt hem op twee manieren serveren:

- met een webserver naar keuze;
- zonder extra tooling: `python3 -m http.server 8080 --directory dist`.

## Gebruikte ROS-interfaces

| Interface | Type | Rol |
|---|---|---|
| `/map` | nav_msgs/OccupancyGrid | lezen |
| `/amcl_pose` | geometry_msgs/PoseWithCovarianceStamped | lezen |
| `/plan` | nav_msgs/Path | lezen |
| `/navigate_through_poses` | nav2_msgs/action/NavigateThroughPoses | action |
| `mission_status` | std_msgs/String | lezen |
| `navigation/blocked` | std_msgs/Bool | lezen |
| `/battery_state` | sensor_msgs/BatteryState | lezen; er is nog geen publisher |
| `/diagnostics` | diagnostic_msgs/DiagnosticArray | lezen |
| `/odom` | nav_msgs/Odometry | lezen |

## Bekende beperkingen

- **Twee partijen sturen naar Nav2.** Het dashboard stuurt zelf `NavigateThroughPoses`. Een lopende route van mission_manager wordt daardoor afgebroken (preempt), en mission_manager, de signal_manager-lampen en de HMI weten niets van de dashboardroute. Zie `docs/dashboard-plan.md`, besluit 1.
- **Geen authenticatie.** Iedereen die poort 9090 bereikt, kan de robot besturen. Gebruik het dashboard alleen op een afgeschermd netwerk.
- **De robotpositie kan stilstaan.** `/amcl_pose` publiceert alleen na een AMCL-update, dus bij een stilstaande robot ververst de positie niet.
- **slam_toolbox publiceert geen `/amcl_pose`.** Tijdens het mappen ontbreekt de robotpositie daarom.
- **`/map` kan later binnenkomen.** Latched `/map` werkt via herabonneren (elke 3 s tot het eerste bericht), omdat rosbridge 2.1 geen QoS-keuze per subscribe ondersteunt.
