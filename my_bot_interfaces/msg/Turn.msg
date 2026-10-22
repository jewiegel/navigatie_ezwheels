# Bocht die eraan komt en/of waarin de robot nu draait (turn_detector -> signal_manager).
uint8 NONE=0
uint8 LEFT=1
uint8 RIGHT=2

uint8 direction      # komende bocht volgens het geplande pad (NONE, LEFT, RIGHT)
float32 distance     # m langs het pad tot de bocht begint; 0 als direction NONE is
float32 angle        # rad, hoeveel de bocht draait (+ = links, - = rechts)
uint8 turning        # draait de robot nu, gemeten via odometrie (NONE, LEFT, RIGHT)
