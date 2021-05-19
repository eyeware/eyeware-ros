#!/usr/bin/env python
# coding=utf-8

"""
Copyright (c) 2021 Eyeware Tech SA http://www.eyeware.tech
"""


# Run in this directory with:
#
# python3 generate_camera_stl.py > ../objects/camera.stl

import numpy as np

R = 1.0
origin = [0.0, 0.0, 0.0]
p1 = [-R, -R / 2, R * 2]
p2 = [R, -R / 2, R * 2]
p3 = [R, R / 2, R * 2]
p4 = [-R, R / 2, R * 2]

a1 = [-R * 0.5, (R / 2) + (R / 2) * 0.1, R * 2]
a2 = [0, 2 * (R / 2) + R * 0.1, R * 2]
a3 = [R * 0.5, (R / 2) + (R / 2) * 0.1, R * 2]

for a in [a1, a2, a3]:
    a[1] *= -1


def array2str(a):
    return " ".join(str(x) for x in a)


def print_double(p1, p2, p3, indent=2):
    print_facet(p1, p2, p3, indent=indent)
    print_facet(p1, p3, p2, indent=indent)


def print_facet(p1, p2, p3, indent=2):
    p1 = np.array(p1)
    p2 = np.array(p2)
    p3 = np.array(p3)
    normal = np.cross(p2 - p1, p3 - p1)
    print(f"{' '*indent}facet normal {array2str(normal)}")
    print(f"{' '*2*indent}outer loop")
    for p in [p1, p2, p3]:
        print(f"{' '*3*indent}vertex {array2str(p)}")
    print(f"{' '*2*indent}endloop")
    print(f"{' '*indent}endfacet")


print("solid camera")
# From origin to rectangle
print_facet(origin, p2, p1)
print_facet(origin, p3, p2)
print_facet(origin, p4, p3)
print_facet(origin, p1, p4)
# The rectangle
print_facet(p1, p2, p3)
print_facet(p1, p3, p4)
# The triangle
print_double(a1, a2, a3)
print("endsolid")
