# hospital_robot_bringup

Start Nav2 met de behavior tree van de ziekenhuisrobot.

## Inhoud

- `config/timing.yaml` — tijden van de robot (sectie `navigation`).
- `launch/navigation.launch.py` — vult `hospital_navigate.xml.in` in, past de
  Nav2-parameters aan en start `my_bot/launch/navigation_launch.py`.
- `hospital_robot_bringup/bt_template.py` — de functies daarvoor (ook gebruikt in de test).

## Wat de launch doet

1. Leest `navigation.*` uit `timing.yaml`; ontbreekt een sleutel → harde fout.
2. Vult `${blockade_wait_s}` en `${blockade_retries}` in; staat er daarna nog een
   `${` in de XML → harde fout met regelnummer. Resultaat in `/tmp/hospital_navigate_*.xml`.
3. Maakt een kopie van het Nav2-parameterbestand met:
   - `bt_navigator.default_nav_through_poses_bt_xml` = de ingevulde boom;
   - `bt_navigator.plugin_lib_names` = de bestaande lijst **+** de twee
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
| `navigation.blockade_wait_s` | wachttijd in de blokkadeprocedure |
| `navigation.blockade_retries` | aantal blokkadeprocedures voordat het doel faalt |

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
