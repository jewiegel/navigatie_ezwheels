"""Genereert model.sdf: driezijdige rolcontainer uit basisvormen.

Gebruik (na het aanpassen van maten of kleuren):  python3 maak_model.py > model.sdf
Andere stand van de zwenkwielen (graden, kar-frame):  python3 maak_model.py 0 0 0 0 > ...
"""
import math
import sys

# Maten van de echte kar: 84,5 cm breed, 70 cm lang, wielen 20,5 cm, zwenkwiel 233 mm hoog.
# De bodem staat op poten hoger dan bij de echte kar, zodat de gesimuleerde robot
# (0.275 m hoog, lidar op 0.25 m) eronder past.
W, D = 0.845, 0.70                # breedte (y), lengte (x); de robot rijdt erin langs x
CASTOR_TOP = 0.233                # bovenkant draaiplaat van het zwenkwiel
PLATE_Z0, PLATE_Z1 = 0.33, 0.36   # bodem: onderkant = vrije hoogte voor de robot
LEG_R = 0.02                      # poot van draaiplaat naar bodem
H = PLATE_Z1 + 1.50               # totale hoogte; gaashekken: aanname, niet gemeten
WHEEL_R, WHEEL_W = 0.1025, 0.05   # wiel: diameter 20,5 cm
CASTOR_IN = 0.06                  # draai-as zoveel binnen de rand van de bodem
TRAIL = 0.04                      # naloop: wielmidden zoveel achter de draai-as
# Stand van elk zwenkwiel (graden, kar-frame): verschillend, zoals na wat duwen.
CASTOR_YAW = [float(a) for a in sys.argv[1:5]] if len(sys.argv) > 4 else [20, 110, -45, 75]
TUBE_R, WIRE_R = 0.0125, 0.003    # frame-buis, gaasdraad
BOTTOM, TOP = PLATE_Z1 + 0.03, H - 0.03
BLUE = '<material><ambient>0.05 0.2 0.75 1</ambient><diffuse>0.05 0.25 0.85 1</diffuse><specular>0.3 0.3 0.3 1</specular></material>'
PLASTIC = '<material><ambient>0.05 0.18 0.6 1</ambient><diffuse>0.08 0.25 0.75 1</diffuse><specular>0.1 0.1 0.1 1</specular></material>'
WHITE = '<material><ambient>0.85 0.85 0.85 1</ambient><diffuse>0.95 0.95 0.95 1</diffuse></material>'
GREY = '<material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material>'
SLOT = '<material><ambient>0.02 0.06 0.25 1</ambient><diffuse>0.02 0.08 0.3 1</diffuse></material>'
out = []
n = 0


def visual(geom, pose, mat, collide=False):
    global n
    n += 1
    out.append(f'      <visual name="v{n}"><pose>{pose}</pose><geometry>{geom}</geometry>{mat}</visual>')
    if collide:
        out.append(f'      <collision name="c{n}"><pose>{pose}</pose><geometry>{geom}</geometry></collision>')


def box(sx, sy, sz, x, y, z, mat, collide=False, yaw=0.0):
    visual(f'<box><size>{sx} {sy} {sz}</size></box>', f'{x} {y} {z} 0 0 {yaw}', mat, collide)


def rod(r, length, x, y, z, axis, mat, collide=False, yaw=0.0):
    rpy = {'z': '0 0 0', 'x': '0 1.5708 0', 'y': f'1.5708 0 {yaw}'}[axis]
    visual(f'<cylinder><radius>{r}</radius><length>{length}</length></cylinder>', f'{x} {y} {z} {rpy}', mat, collide)


def gate(fixed, a0, a1, along, n_vertical, rows):
    """Gaashek in het vlak fixed (x of y vast), van a0 tot a1 langs 'along'."""
    def at(a, z):
        return (fixed, a, z) if along == 'y' else (a, fixed, z)
    mid, span = (a0 + a1) / 2, a1 - a0
    # Frame: twee staanders, boven- en onderregel
    for a in (a0, a1):
        rod(TUBE_R, TOP - BOTTOM, *at(a, (TOP + BOTTOM) / 2), 'z', BLUE, True)
    for z in (BOTTOM, TOP):
        rod(TUBE_R, span, *at(mid, z), along, BLUE, True)
    # Gaas: verticale en horizontale draden
    for i in range(1, n_vertical + 1):
        rod(WIRE_R, TOP - BOTTOM, *at(a0 + span * i / (n_vertical + 1), (TOP + BOTTOM) / 2), 'z', BLUE)
    for z in rows:
        rod(WIRE_R, span, *at(mid, z), along, BLUE)


# Bodem met opstaande rand en roostervakken
box(D, W, PLATE_Z1 - PLATE_Z0, 0, 0, (PLATE_Z0 + PLATE_Z1) / 2, PLASTIC, True)
for i in range(1, 9):   # donkere sleuven: het rooster
    box(0.012, W - 0.08, 0.002, -D / 2 + D * i / 9, 0, PLATE_Z1 + 0.001, SLOT)
for i in range(1, 8):
    box(D - 0.08, 0.012, 0.002, 0, -W / 2 + W * i / 8, PLATE_Z1 + 0.001, SLOT)
# Zwenkwielen in de hoeken (zelfde volgorde als cart_castors in docking): wiel en vork
# draaien om de draai-as; het wielmidden ligt TRAIL achter de as. Dan draaiplaat en poot.
for (sx, sy), yaw_deg in zip(((1, 1), (1, -1), (-1, 1), (-1, -1)), CASTOR_YAW):
    x, y = sx * (D / 2 - CASTOR_IN), sy * (W / 2 - CASTOR_IN)
    a = math.radians(yaw_deg)
    c, s = math.cos(a), math.sin(a)
    wx, wy = x - TRAIL * c, y - TRAIL * s
    rod(WHEEL_R, WHEEL_W, wx, wy, WHEEL_R, 'y', WHITE, True, a)
    rod(0.04, WHEEL_W, wx, wy, WHEEL_R, 'y', GREY, False, a)               # naaf
    for side in (-1, 1):                                                    # vorkpoten
        off = side * (WHEEL_W / 2 + 0.006)
        box(0.05, 0.006, CASTOR_TOP - 0.02 - WHEEL_R, wx - off * s, wy + off * c,
            (WHEEL_R + CASTOR_TOP - 0.02) / 2, GREY, True, a)
    box(TRAIL + 0.05, WHEEL_W + 0.024, 0.01, (x + wx) / 2, (y + wy) / 2,
        CASTOR_TOP - 0.025, GREY, False, a)                                 # vorkkop
    box(0.10, 0.08, 0.02, x, y, CASTOR_TOP - 0.01, GREY, True)             # draaiplaat
    rod(LEG_R, PLATE_Z0 - CASTOR_TOP, x, y, (PLATE_Z0 + CASTOR_TOP) / 2, 'z', BLUE, True)

rows = [BOTTOM + (TOP - BOTTOM) * k / 4 for k in (1, 2, 3)]
x0, x1 = -D / 2 + 0.02, D / 2 - 0.02
y0, y1 = -W / 2 + 0.02, W / 2 - 0.02
gate(x0, y0 + 0.03, y1 - 0.03, 'y', 6, rows)   # achterkant (-x); voorkant (+x) is open
for y in (y0, y1):                              # zijkanten
    gate(y, x0 + 0.03, x1, 'x', 7, rows)
    for z in (0.5, 1.45):                       # witte scharnierclips
        box(0.04, 0.035, 0.05, x0 + 0.01, y, z, WHITE)

mass, ixx = 35.0, 35.0 * (W ** 2 + H ** 2) / 12
body = '\n'.join(out)
sys.stdout.write(f'''<?xml version="1.0"?>
<!-- Driezijdige rolcontainer naar de echte kar: 0.845 m breed (y), 0.70 m lang (x), wielen
     0.205 m; bodem op poten met 0.33 m vrije hoogte, zodat de robot eronder past.
     Open voorkant langs +x.
     Gegenereerd uit basisvormen; oorsprong midden op de vloer. -->
<sdf version="1.9">
  <model name="rolcontainer">
    <static>true</static>
    <link name="body">
      <inertial>
        <pose>0 0 0.6 0 0 0</pose>
        <mass>{mass}</mass>
        <inertia><ixx>{ixx:.3f}</ixx><iyy>{35.0 * (D ** 2 + H ** 2) / 12:.3f}</iyy><izz>{35.0 * (W ** 2 + D ** 2) / 12:.3f}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
{body}
    </link>
  </model>
</sdf>
''')
