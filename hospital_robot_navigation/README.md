# hospital_robot_navigation

## Inhoud

- `behavior_trees/hospital_navigate.xml.in` — NavigateThroughPoses-boom voor de
  ziekenhuisrobot, gebaseerd op `navigate_through_poses_w_replanning_and_recovery.xml`
  van Nav2 Iron (zelfde node- en poortnamen). Plant elke seconde opnieuw en volgt
  het pad. Faalt dat, dan draait de blokkadeprocedure:
  `navigation/blocked = true` → wachten → beide costmaps wissen →
  `navigation/blocked = false` → opnieuw proberen.
- `check_bt_xml` — commandoregelprogramma (geen ROS-node) dat een BT-XML controleert
  tegen de nodetypes van opgegeven plugin-bibliotheken (`BT::VerifyXML`) en het
  nodemodel (poorten per type) op stdout zet. Zonder draaiende Nav2 te gebruiken.

## Parameters (placeholders)

| Placeholder | Bron (`hospital_robot_bringup/config/timing.yaml`) |
|---|---|
| `${blockade_wait_s}` | `navigation.blockade_wait_s` — `Wait.wait_duration` |
| `${blockade_retries}` | `navigation.blockade_retries` — `RecoveryNode.number_of_retries` |

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
