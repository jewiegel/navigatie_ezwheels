# hospital_robot_navigation

## Inhoud

- `behavior_trees/hospital_navigate.xml.in` — NavigateThroughPoses-boom voor de
  ziekenhuisrobot, gebaseerd op `navigate_through_poses_w_replanning_and_recovery.xml`
  van Nav2 Iron (zelfde node- en poortnamen). Rijdt de route **waypoint voor
  waypoint**: plan alleen naar het volgende waypoint (`GetNextWaypoint` +
  `ComputePathToPose`, elke seconde opnieuw), volg tot de goal checker het waypoint
  bereikt meldt (positie én richting), haal het weg (`PopWaypoint`) en ga door.
  Eén pad door alle waypoints tegelijk plooit bij een keerpunt terug op zichzelf,
  waardoor DWB vóór het waypoint gaat pendelen. Faalt plannen of volgen, dan draait
  de blokkadeprocedure: `navigation/blocked = true` → herhaal {costmaps wissen
  behalve 3 m rond de robot → wachten → plannen naar het huidige waypoint} tot
  plannen lukt → `navigation/blocked = false` → verder rijden. Rond de robot wordt
  niet gewist omdat de lidar achter de robot een dode hoek heeft; anders zou een
  obstakel daar vergeten worden en de check onterecht "vrij" zeggen. De boom geeft
  nooit op (`RetryUntilSuccessful num_attempts="-1"`): alleen een voltooide route
  of annuleren beëindigt het doel.
  Hoe dicht "bereikt" is, bepaalt `general_goal_checker` (`xy_goal_tolerance`,
  `yaw_goal_tolerance`) in de Nav2-parameters.
- `check_bt_xml` — commandoregelprogramma (geen ROS-node) dat een BT-XML controleert
  tegen de nodetypes van opgegeven plugin-bibliotheken (`BT::VerifyXML`) en het
  nodemodel (poorten per type) op stdout zet. Zonder draaiende Nav2 te gebruiken.

## Parameters (placeholders)

| Placeholder | Bron (`hospital_robot_bringup/config/timing.yaml`) |
|---|---|
| `${blockade_wait_s}` | `navigation.blockade_wait_s` — `Wait.wait_duration`: tijd tussen twee checks tijdens een blokkade |

De sjabloon wordt bij het opstarten ingevuld door `hospital_robot_bringup`.
Gebruik het `.xml.in`-bestand nooit direct in Nav2.

## Testen

```bash
cd /ros2_ws
colcon build --packages-select hospital_robot_navigation
colcon test --packages-select hospital_robot_navigation   # ament_lint
# Handmatig een ingevulde boom checken:
install/hospital_robot_navigation/lib/hospital_robot_navigation/check_bt_xml tree.xml \
  nav2_compute_path_through_poses_action_bt_node ... hospital_robot_publish_blocked_bt_node
```

De inhoudelijke test van de sjabloon (invullen + check tegen alle plugins) staat in
`hospital_robot_bringup/test/test_bt_template.py`.
