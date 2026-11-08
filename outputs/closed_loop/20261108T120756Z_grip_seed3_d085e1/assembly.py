"""Mesh assembly datums in mm, reconstructed from mating faces and hole patterns.

No STL is modified. Link lengths are pivot-to-pivot, not mesh bounding-box lengths.
Shoulder bracket axial seating/internal drivetrain remain subject to physical review.
"""
import math

BASE_BOTTOM = -38.41212844848633
BASE_TOP = 17.587871551513672
DRIVE_BOTTOM = 76.40288543701172
DRIVE_TOP = 90.40288543701172
SHOULDER_BASE = 106.20469665527344
SHOULDER_FIXED_PIVOT = 57.79711723327637
SHOULDER_MOVING_PIVOT = 77.87110137939453
SHOULDER_TOP = 126.27799224853516
LINK1_BOTTOM = 166.2779998779297
LINK1_TOP = 316.2779846191406
ELBOW_BOTTOM = 275.7779846191406
ELBOW_FIXED_PIVOT = 310.5779846191406
ELBOW_MOVING_PIVOT = 310.8153991699219
LINK2_TOP = 496.3280029296875

SHOULDER_HEIGHT_MM = (BASE_TOP - BASE_BOTTOM + DRIVE_TOP - DRIVE_BOTTOM
                      + SHOULDER_BASE - SHOULDER_FIXED_PIVOT)
UPPER_ARM_MM = (SHOULDER_TOP - SHOULDER_MOVING_PIVOT + LINK1_TOP - LINK1_BOTTOM
                + ELBOW_FIXED_PIVOT - ELBOW_BOTTOM)
FOREARM_MM = LINK2_TOP - ELBOW_MOVING_PIVOT

# (mesh name, translation in body frame mm, rotation radians). The stationary and
# moving halves belong to different bodies so the seam remains concentric when bent.
BASE_VISUALS = [
    ('base', (.838527, -5.870415, -BASE_BOTTOM), (0, 0, 0)),
    ('base_plate', (.838527, -5.870415, -BASE_BOTTOM), (0, 0, 0)),
]
PAN_VISUALS = [
    ('drive_plate', (-.838527, .129585, BASE_TOP - BASE_BOTTOM - DRIVE_BOTTOM), (0, 0, math.pi)),
    ('shoulder_1', (.838527, 67.461307, BASE_TOP - BASE_BOTTOM + DRIVE_TOP - DRIVE_BOTTOM + SHOULDER_BASE), (math.pi, 0, 0)),
]
UPPER_VISUALS = [
    ('shoulder_2', (.838527, -40, -SHOULDER_MOVING_PIVOT), (0, 0, 0)),
    ('link_1', (.838527, 0, SHOULDER_TOP - SHOULDER_MOVING_PIVOT - LINK1_BOTTOM), (0, 0, 0)),
    ('elbow_1', (.838527, 0, UPPER_ARM_MM - ELBOW_FIXED_PIVOT), (0, 0, 0)),
]
FOREARM_VISUALS = [
    ('elbow_2', (.838527, -30, -ELBOW_MOVING_PIVOT), (0, 0, 0)),
    ('link_2', (.838527, 0, -ELBOW_MOVING_PIVOT), (0, 0, 0)),
    ('wrist_gear_1', (.838527, 0, -ELBOW_MOVING_PIVOT), (0, 0, 0)),
]


def color(name):
    return '0.95 0.55 0.15 1' if name.startswith('link_') else '0.65 0.68 0.72 1'
