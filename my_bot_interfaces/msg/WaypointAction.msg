# Actie op een waypoint (route_manager -> mission_manager).
string type           # naam in de TaskRegistry van de mission_manager, bv. wait of spin
string params_json    # parameters van de actie als JSON-object, bv. {"seconds": 3}
uint32 max_retries    # zo vaak opnieuw na mislukken; daarna gaat de route toch door
