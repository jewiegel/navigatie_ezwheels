# hospital_robot_bringup

Start Nav2 met de behavior tree van de ziekenhuisrobot.

## Inhoud

- `config/timing.yaml` — tijden van de robot (sectie `navigation`).
- `launch/sim.launch.py` — Gazebo + RViz + AMCL + Nav2 (ziekenhuisbomen) + route_manager + MissionManagerNode.
- `launch/robot.launch.py` — echte robot: hetzelfde zonder Gazebo/RViz, met
  `nav2_params_robot.yaml`, verplicht argument `map:=`, optioneel `foxglove:=false`.
  Verwacht dat de robotsoftware lidar (`/laser_1/scan_filtered`), `/odom`, TF en
  de wieldriver levert. Zie `StartenSImulatie.md` §8.
- `launch/navigation.launch.py` — vult de boom-sjablonen in, past de
  Nav2-parameters aan en start `my_bot/launch/navigation_launch.py`.
- `hospital_robot_bringup/bt_template.py` — de functies daarvoor (ook gebruikt in de test).

## Wat de launch doet

1. Leest `navigation.*` uit `timing.yaml`; ontbreekt een sleutel → harde fout.
2. Vult `${blockade_wait_s}` in; staat er daarna nog een
   `${` in de XML → harde fout met regelnummer. Dat gebeurt voor beide sjablonen;
   resultaat in `/tmp/hospital_navigate_*.xml` en `/tmp/hospital_navigate_to_pose_*.xml`.
3. Maakt een kopie van het Nav2-parameterbestand met:
   - `bt_navigator.default_nav_through_poses_bt_xml` = ingevulde `hospital_navigate.xml.in`;
   - `bt_navigator.default_nav_to_pose_bt_xml` = ingevulde `hospital_navigate_to_pose.xml.in`;
   - `bt_navigator.plugin_lib_names` = de bestaande lijst **+** de
     hospital-plugins (in Iron vervangt deze parameter de standaardlijst);
   - `controller_server.progress_checker.movement_time_allowance` = `navigation.progress_timeout_s`.
4. Start Nav2 met die kopie. Er worden geen extra nodes gestart.

## Gebruik

```bash
ros2 launch hospital_robot_bringup navigation.launch.py use_sim_time:=true
ros2 topic echo navigation/blocked --qos-durability transient_local
```

| Launch-argument | Standaard |
|---|---|
| `timing_file` | `share/hospital_robot_bringup/config/timing.yaml` |
| `params_file` | `share/my_bot/config/nav2_params.yaml` |
| `use_sim_time` | `false` |

| `timing.yaml` | Betekenis |
|---|---|
| `navigation.progress_timeout_s` | progress checker: zoveel s zonder voortgang → FollowPath faalt |
| `navigation.blockade_wait_s` | tijd tussen twee checks of het pad weer vrij is tijdens een blokkade |

## Testen

```bash
cd /ros2_ws
colcon build --packages-select hospital_robot_bt_nodes hospital_robot_navigation hospital_robot_bringup
source install/setup.bash
colcon test --packages-select hospital_robot_bringup
colcon test-result --test-result-base build/hospital_robot_bringup --verbose
```

`test/test_bt_template.py`: invullen met testwaarden, geen `${` over, fout bij
ontbrekende sleutel / overgebleven `${`, Nav2-parameters correct aangepast, en de
ingevulde boom gecontroleerd met `check_bt_xml` tegen alle Nav2-plugins uit
`nav2_params.yaml` plus de hospital-plugins — inclusief poortnamen.
