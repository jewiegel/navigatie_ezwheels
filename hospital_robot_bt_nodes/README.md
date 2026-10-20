# hospital_robot_bt_nodes

Eigen BehaviorTree.CPP-plugins (v3, zoals Nav2 in ROS 2 Iron) voor de ziekenhuisrobot.

| Bibliotheek | Inhoud |
|---|---|
| `hospital_robot_path_block_check` | `PathBlockCheck`: pure logica (alleen berichttypen) — loopt `lookahead_m` meter langs een `nav_msgs/Path` en kijkt of een cel in een `nav2_msgs/Costmap` `>= lethal_threshold` is. Punten buiten de costmap blokkeren niet; leeg pad of lege costmap = niet geblokkeerd. |
| `hospital_robot_publish_blocked_bt_node` | BT-node `PublishBlocked` (SyncAction) |
| `hospital_robot_is_path_blocked_bt_node` | BT-node `IsPathBlocked` (Condition) |

Beide BT-nodes halen de ROS-node uit het blackboard onder `"node"` (zoals Nav2).
De plugins laad je via `bt_navigator.plugin_lib_names` met de namen uit de tabel.
**Iron:** die parameter vervangt de standaardlijst, dus zet ze áchter de volledige
Nav2-lijst (dat doet `hospital_robot_bringup` automatisch).

## Poorten

**`PublishBlocked`** — publiceert `std_msgs/Bool` (depth 1, reliable, transient_local), geeft altijd `SUCCESS`.

| Poort | Type | Standaard | |
|---|---|---|---|
| `blocked` | bool | — (verplicht) | ontbreekt of geen bool → `BT::RuntimeError` |
| `topic` | string | `navigation/blocked` | relatief, valt onder de namespace |

**`IsPathBlocked`** — `SUCCESS` als het pad geblokkeerd is, anders `FAILURE` (ook zonder pad of costmap).

| Poort | Type | Standaard | |
|---|---|---|---|
| `path` | `nav_msgs::msg::Path` | — | via blackboard, bv. `{path}` |
| `lookahead` | double | `1.5` | meter langs het pad |
| `costmap_topic` | string | `global_costmap/costmap_raw` | relatief |
| `lethal_threshold` | int | `253` | 0–255, anders `BT::RuntimeError` (int i.p.v. uint8: BT v3 kan geen uint8 parsen) |

`IsPathBlocked` zit nog niet in `hospital_navigate.xml.in`; hij is klaar om toe te voegen.

## Testen

```bash
cd /ros2_ws
colcon build --packages-select hospital_robot_bt_nodes
colcon test --packages-select hospital_robot_bt_nodes
colcon test-result --test-result-base build/hospital_robot_bt_nodes --verbose
```

gtests: `test_path_block_check` (vrij pad, lethal binnen/net voorbij lookahead,
pad deels buiten de costmap, leeg pad, lege costmap), `test_publish_blocked`
(juiste waarde op het topic, ontbrekende/ongeldige poort), `test_is_path_blocked`
(SUCCESS/FAILURE met een kunstmatige costmap, geen pad, geen costmap). Plus ament_lint.
