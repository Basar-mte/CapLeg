# -*- coding: utf-8 -*-
"""
CapLeg - capstan-driven powered transfemoral prosthesis (research platform)
Fusion 360 builder script
===========================================================================

Builds the complete prosthesis as a jointed Fusion 360 assembly:

    CapLeg_Thigh  socket, pyramid receiver, knee head + male pyramid, knee capstan disc, axle  [grounded]
    CapLeg_Shank  knee frame + AK80-9 capstan drive, pylon, 6-axis load cell,
                  ankle frame + AK80-9 capstan drive, parallel-elastic (PEA) spring cartridge
    CapLeg_Talus  ankle hub, ankle capstan sector, subtalar (roll) lugs + axle
    CapLeg_Foot   printed keel with subtalar bearing boss, TPU sole, TPU roll springs, toe pin
    CapLeg_Toe    spring-loaded metatarsophalangeal (MTP) toe + TPU pad

Joints (revolute, with limits): knee (Y), ankle_pitch (Y), ankle_roll (X), toe (Y).
Fusion joint signs: knee + = flexion, ankle_pitch + = plantarflexion,
                    ankle_roll + = inversion (right foot), toe - = toe dorsiflexion.

Frame: millimetres, origin = knee axis centre, +X anterior, +Y medial (right leg), +Z up.
Zero pose = standing straight, all joints at 0.

Run in Fusion: UTILITIES > ADD-INS > Scripts and Add-Ins > Scripts (+) > pick this folder > Run.
Running it opens a NEW design document and builds everything (~20 s).
"""
import adsk.core
import adsk.fusion
import math
import os
import json
import traceback

# =================================================================== PARAMETERS (mm)
P = dict(
    shank_len=420.0,        # knee axis -> ankle pitch axis
    ankle_h=90.0,           # ankle pitch axis -> floor
    roll_dz=44.0,           # ankle pitch axis -> subtalar (roll) axis
    toe_x=120.0,            # MTP axis, anterior of ankle axis
    toe_dz=11.5,            # MTP axis height above floor
    thigh_len=430.0,        # knee -> hip centre (used by simulation rig only)
    plate_in=27.0,          # inner face of side plates (|y|)
    plate_t=8.0,            # side plate thickness
    cover_t=3.0,            # side cover thickness
    act_r=49.0,             # CubeMars AK80-9 envelope radius (D98 mm)
    act_w=38.5,             # AK80-9 axial length
    drum_r=11.0,            # capstan drum radius (D22)
    cable_r=1.25,           # D2.5 mm UHMWPE (Dyneema) cable
    cap_y0=12.5,            # capstan plane (medial bay between plates)
    cap_y1=26.0,
    knee_disc_r=57.5,       # knee capstan groove-root radius (flange 60)
    ankle_sector_r=62.5,    # ankle capstan groove-root radius
    knee_motor=(-8.0, -95.0),     # knee actuator axis (x, z)
    ankle_motor_ang=135.0,        # ankle actuator direction from ankle axis (deg, XZ)
    ankle_motor_d=95.336,         # ankle actuator centre distance
    pylon_r=15.0,           # D30 mm pylon tube
    pylon_wall=2.0,
)

COLORS = {                    # appearance name: (RGB, Fusion library base appearance)
    'Accent Red':     ((211, 44, 50), 'Plastic - Matte (Red)'),
    'Graphite':       ((40, 40, 42), 'Plastic - Matte (Black)'),
    'Smoke Clear':    ((120, 124, 130), 'Plastic - Translucent Glossy (Gray)'),
    'Aluminium':      ((196, 198, 202), 'Aluminum - Satin'),
    'Steel':          ((165, 168, 172), 'Steel - Satin'),
    'Actuator Black': ((22, 22, 24), 'Paint - Enamel Glossy (Black)'),
    'TPU Black':      ((17, 17, 17), 'Rubber - Soft'),
    'Cable White':    ((235, 235, 228), 'Plastic - Glossy (White)'),
    'PCB Green':      ((24, 70, 46), 'Plastic - Matte (Green)'),
}

MATERIALS = {
    'printed': 'Nylon 6/6', 'socket': 'PET Plastic', 'tpu': 'Rubber',
    'alu': 'Aluminum 6061', 'steel': 'Steel', 'pcb': 'Plastic',
}

THIGH, SHANK, TALUS, FOOT, TOE = 'CapLeg_Thigh', 'CapLeg_Shank', 'CapLeg_Talus', 'CapLeg_Foot', 'CapLeg_Toe'
LINKS = [THIGH, SHANK, TALUS, FOOT, TOE]

CM = 0.1   # Fusion API internal unit is cm


# =================================================================== small helpers
def _app():
    return adsk.core.Application.get()


def _des():
    return adsk.fusion.Design.cast(_app().activeProduct)


def p3(x, y, z):
    return adsk.core.Point3D.create(x * CM, y * CM, z * CM)


def v3(x, y, z):
    return adsk.core.Vector3D.create(x, y, z)


def vi(mm):
    return adsk.core.ValueInput.createByReal(mm * CM)


def _tb():
    return adsk.fusion.TemporaryBRepManager.get()


def geom():
    """Derived geometry (all mm)."""
    g = dict(P)
    g['AZ'] = -P['shank_len']
    g['FLOOR'] = g['AZ'] - P['ankle_h']
    g['ROLL_Z'] = g['AZ'] - P['roll_dz']
    g['TOE_Z'] = g['FLOOR'] + P['toe_dz']
    g['TOE_X'] = P['toe_x']
    g['PI'] = P['plate_in']
    g['PO'] = P['plate_in'] + P['plate_t']
    g['CO'] = g['PO'] + P['cover_t']
    a = math.radians(P['ankle_motor_ang'])
    g['AM'] = (P['ankle_motor_d'] * math.cos(a), g['AZ'] + P['ankle_motor_d'] * math.sin(a))
    g['KM'] = tuple(P['knee_motor'])
    g['KCLAMP'] = (-190.0, -152.0)
    AZ = g['AZ']
    g['ATOP'] = (AZ + 95.0, AZ + 107.0)
    g['LC'] = (AZ + 107.0, AZ + 127.0)
    g['ACLAMP'] = (AZ + 127.0, AZ + 157.0)
    g['PYL'] = (g['ACLAMP'][1] - 28.0, g['KCLAMP'][0] + 35.0)
    g['HIP_Z'] = P['thigh_len']
    return g


# ------------------------------------------------------------ temporary B-Rep primitives
def cyl(a, b, r, r2=None):
    return _tb().createCylinderOrCone(p3(*a), r * CM, p3(*b), (r if r2 is None else r2) * CM)


def cyl_y(x, z, y0, y1, r):
    return cyl((x, y0, z), (x, y1, z), r)


def cyl_x(y, z, x0, x1, r):
    return cyl((x0, y, z), (x1, y, z), r)


def cyl_z(x, y, z0, z1, r, r2=None):
    return cyl((x, y, z0), (x, y, z1), r, r2)


def box(x0, x1, y0, y1, z0, z1):
    obb = adsk.core.OrientedBoundingBox3D.create(
        p3((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), v3(1, 0, 0), v3(0, 1, 0),
        abs(x1 - x0) * CM, abs(y1 - y0) * CM, abs(z1 - z0) * CM)
    return _tb().createBox(obb)


def obox(c, ldir, wdir, L, W, H):
    obb = adsk.core.OrientedBoundingBox3D.create(p3(*c), v3(*ldir), v3(*wdir), L * CM, W * CM, H * CM)
    return _tb().createBox(obb)


def U(a, *bs):
    for b in bs:
        _tb().booleanOperation(a, b, adsk.fusion.BooleanTypes.UnionBooleanType)
    return a


def D(a, *bs):
    for b in bs:
        _tb().booleanOperation(a, b, adsk.fusion.BooleanTypes.DifferenceBooleanType)
    return a


def I(a, *bs):
    for b in bs:
        _tb().booleanOperation(a, b, adsk.fusion.BooleanTypes.IntersectionBooleanType)
    return a


def ring_y(x, z, y0, y1, ro, ri):
    return D(cyl_y(x, z, y0, y1, ro), cyl_y(x, z, y0 - 1, y1 + 1, ri))


def halfspace_xz(cx, cz, nx, nz, y0, y1, size=400.0):
    """Box occupying the side p.n >= 0 of the plane through (cx, cz) with normal (nx, nz) in XZ."""
    L = math.hypot(nx, nz)
    nx, nz = nx / L, nz / L
    c = (cx + nx * size / 2, (y0 + y1) / 2, cz + nz * size / 2)
    return obox(c, (nx, 0, nz), (0, 1, 0), size, abs(y1 - y0), size)


def pie_y(cx, cz, y0, y1, r, a0, a1):
    """Pie slice (span < 180 deg) between polar angles a0..a1 (deg, atan2(z, x))."""
    t0, t1 = math.radians(a0), math.radians(a1)
    body = cyl_y(cx, cz, y0, y1, r)
    I(body, halfspace_xz(cx, cz, -math.sin(t0), math.cos(t0), y0 - 1, y1 + 1))
    I(body, halfspace_xz(cx, cz, math.sin(t1), -math.cos(t1), y0 - 1, y1 + 1))
    return body


def frustum_sq(cx, cy, z0, z1, s0, s1):
    """Square frustum: side s0 at z0, s1 at z1 (s1 > s0 = inverted prosthetic pyramid)."""
    smax = max(s0, s1)
    body = box(cx - smax / 2, cx + smax / 2, cy - smax / 2, cy + smax / 2, z0, z1)
    k = (s1 - s0) / 2.0 / (z1 - z0)
    zm = (z0 + z1) / 2.0
    nl = math.hypot(1.0, k)
    nr, nz = 1.0 / nl, -k / nl
    for q in range(4):
        ang = math.radians(90 * q)
        ca, sa = math.cos(ang), math.sin(ang)
        n = (nr * ca, nr * sa, nz)
        rp = s0 / 2.0 + k * (zm - z0)
        pp = (cx + rp * ca, cy + rp * sa, zm)
        c = (pp[0] - n[0] * 100, pp[1] - n[1] * 100, pp[2] - n[2] * 100)
        I(body, obox(c, n, (-sa, ca, 0), 200, 200, 200))
    return body


def cross_spans(c1, R, c2, r):
    """Crossed (internal) tangent cable spans between a capstan of radius R at c1 and drum r at c2 (XZ)."""
    (x1, z1), (x2, z2) = c1, c2
    dx, dz = x2 - x1, z2 - z1
    d = math.hypot(dx, dz)
    th = math.atan2(dz, dx)
    al = math.acos((R + r) / d)
    out = []
    for s in (+1, -1):
        a = th + s * al
        nx, nz = math.cos(a), math.sin(a)
        out.append(((x1 + R * nx, z1 + R * nz), (x2 - r * nx, z2 - r * nz)))
    return out


# ------------------------------------------------------------ 2-D outline helpers
def circle_hull(circles):
    """Convex hull of circles [(u, v, r)] -> closed outline of lines/arcs, CCW in (u, v)."""
    pts = []
    N = 360
    for idx, (cx, cy, r) in enumerate(circles):
        for k in range(N):
            t = 2 * math.pi * k / N
            pts.append((cx + r * math.cos(t), cy + r * math.sin(t), idx))
    pts.sort(key=lambda p: (p[0], p[1]))

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    hull = lower[:-1] + upper[:-1]
    seq = []
    for p in hull:
        if not seq or seq[-1] != p[2]:
            seq.append(p[2])
    while len(seq) > 1 and seq[0] == seq[-1]:
        seq.pop()

    def tangent(ci, cj):
        (x1, y1, r1), (x2, y2, r2) = circles[ci], circles[cj]
        dx, dy = x2 - x1, y2 - y1
        L = math.hypot(dx, dy)
        ux, uy = dx / L, dy / L
        px, py = uy, -ux
        a = (r1 - r2) / L
        b = math.sqrt(max(0.0, 1 - a * a))
        nx, ny = a * ux + b * px, a * uy + b * py
        return (x1 + r1 * nx, y1 + r1 * ny), (x2 + r2 * nx, y2 + r2 * ny)

    n = len(seq)
    tps = [tangent(seq[i], seq[(i + 1) % n]) for i in range(n)]
    segs = []
    for i in range(n):
        ci = seq[(i + 1) % n]
        s = tps[i][1]
        e = tps[(i + 1) % n][0]
        cx, cy, r = circles[ci]
        a0 = math.atan2(s[1] - cy, s[0] - cx)
        a1 = math.atan2(e[1] - cy, e[0] - cx)
        while a1 <= a0:
            a1 += 2 * math.pi
        am = 0.5 * (a0 + a1)
        m = (cx + r * math.cos(am), cy + r * math.sin(am))
        segs.append(('L', tps[i][0], tps[i][1]))
        if a1 - a0 > 1e-6:
            segs.append(('A', s, m, e))
    return segs


def _to_model(which, u, v):
    if which == 'xz':
        return (u, 0.0, v)
    if which == 'xy':
        return (u, v, 0.0)
    return (0.0, u, v)          # 'yz'


def _axis(which):
    return {'xz': 1, 'xy': 2, 'yz': 0}[which]


def extrude_outline(comp_, which, segs, w0, w1, name, holes=()):
    """Sketch a closed outline on a base plane and extrude it between world offsets w0..w1."""
    plane = {'xy': comp_.xYConstructionPlane, 'xz': comp_.xZConstructionPlane,
             'yz': comp_.yZConstructionPlane}[which]
    sk = comp_.sketches.add(plane)
    sk.name = name + '_sk'
    sk.isComputeDeferred = True

    def sp(u, v):
        return sk.modelToSketchSpace(p3(*_to_model(which, u, v)))

    lines = sk.sketchCurves.sketchLines
    arcs = sk.sketchCurves.sketchArcs
    for s in segs:
        if s[0] == 'L':
            lines.addByTwoPoints(sp(*s[1]), sp(*s[2]))
        else:
            arcs.addByThreePoints(sp(*s[1]), sp(*s[2]), sp(*s[3]))
    for (u, v, r) in holes:
        sk.sketchCurves.sketchCircles.addByCenterRadius(sp(u, v), r * CM)
    sk.isComputeDeferred = False
    best, ba = None, -1.0
    for i in range(sk.profiles.count):
        pr = sk.profiles.item(i)
        a = pr.areaProperties(adsk.fusion.CalculationAccuracy.LowCalculationAccuracy).area
        if a > ba:
            best, ba = pr, a
    nrm = sk.xDirection.crossProduct(sk.yDirection)
    nw = [nrm.x, nrm.y, nrm.z][_axis(which)]
    s0, s1 = w0 * nw, w1 * nw
    st, ln = min(s0, s1), abs(w1 - w0)
    ext = comp_.features.extrudeFeatures
    ei = ext.createInput(best, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    if abs(st) > 1e-9:
        ei.startExtent = adsk.fusion.OffsetStartDefinition.create(vi(st))
    ei.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(vi(ln)),
                        adsk.fusion.ExtentDirections.PositiveExtentDirection)
    f = ext.add(ei)
    f.name = 'EX_' + name
    b = f.bodies.item(0)
    b.name = name
    sk.isVisible = False
    return b


def add_bodies(comp_, items, tag=None):
    """Add temporary B-Rep bodies [(name, tbody)] to a component (inside one base feature)."""
    names = []
    des = _des()
    if des.designType == adsk.fusion.DesignTypes.ParametricDesignType:
        bf = comp_.features.baseFeatures.add()
        bf.startEdit()
        for name, tbody in items:
            b = comp_.bRepBodies.add(tbody, bf)
            b.name = name
            names.append(name)
        bf.finishEdit()
        try:
            bf.name = 'BF_' + (tag or items[0][0])
        except Exception:
            pass
    else:
        for name, tbody in items:
            b = comp_.bRepBodies.add(tbody)
            b.name = name
            names.append(name)
    return [comp_.bRepBodies.itemByName(n) for n in names]


def combine(comp_, target, tools, op='cut'):
    t = comp_.bRepBodies.itemByName(target)
    oc = adsk.core.ObjectCollection.create()
    for n in tools:
        oc.add(comp_.bRepBodies.itemByName(n))
    ci = comp_.features.combineFeatures.createInput(t, oc)
    ci.operation = {'cut': adsk.fusion.FeatureOperations.CutFeatureOperation,
                    'join': adsk.fusion.FeatureOperations.JoinFeatureOperation,
                    'intersect': adsk.fusion.FeatureOperations.IntersectFeatureOperation}[op]
    ci.isKeepToolBodies = False
    comp_.features.combineFeatures.add(ci)
    return comp_.bRepBodies.itemByName(target)


def _fillet(comp_, edges, r):
    if edges.count == 0:
        return False
    for rr in (r, r * 0.6, r * 0.35):
        try:
            fi = comp_.features.filletFeatures.createInput()
            try:
                fi.edgeSetInputs.addConstantRadiusEdgeSet(edges, vi(rr), True)
            except Exception:
                fi.addConstantRadiusEdgeSet(edges, vi(rr), True)
            comp_.features.filletFeatures.add(fi)
            return True
        except Exception:
            continue
    return False


def fillet_outer_loop(comp_, body_name, axis, sign, r):
    """Fillet the outer loop of the extreme planar face whose normal is +/- world axis."""
    b = comp_.bRepBodies.itemByName(body_name)
    best, bv = None, None
    for f in b.faces:
        if f.geometry.surfaceType != adsk.core.SurfaceTypes.PlaneSurfaceType:
            continue
        ok, n = f.evaluator.getNormalAtPoint(f.pointOnFace)
        if [n.x, n.y, n.z][axis] * sign < 0.99:
            continue
        c = [f.pointOnFace.x, f.pointOnFace.y, f.pointOnFace.z][axis] * sign
        if bv is None or c > bv:
            best, bv = f, c
    if best is None:
        return False
    edges = adsk.core.ObjectCollection.create()
    for lp in best.loops:
        if lp.isOuter:
            for e in lp.edges:
                edges.add(e)
    return _fillet(comp_, edges, r)


def fillet_edges_where(comp_, body_name, pred, r):
    b = comp_.bRepBodies.itemByName(body_name)
    edges = adsk.core.ObjectCollection.create()
    for e in b.edges:
        try:
            if pred(e):
                edges.add(e)
        except Exception:
            pass
    return _fillet(comp_, edges, r)


def _bb(e):
    bb = e.boundingBox
    return (bb.minPoint.x / CM, bb.minPoint.y / CM, bb.minPoint.z / CM,
            bb.maxPoint.x / CM, bb.maxPoint.y / CM, bb.maxPoint.z / CM)


# =================================================================== document / components
def new_document():
    app = _app()
    prefs = app.preferences.generalPreferences
    old = prefs.defaultModelingOrientation
    prefs.defaultModelingOrientation = adsk.core.DefaultModelingOrientations.ZUpModelingOrientation
    try:
        app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    finally:
        prefs.defaultModelingOrientation = old
    des = _des()
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    des.fusionUnitsManager.distanceDisplayUnits = adsk.fusion.DistanceUnits.MillimeterDistanceUnits
    root = des.rootComponent
    for n in LINKS:
        o = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        o.component.name = n
    return des


def comp(name):
    return occ(name).component


def occ(name):
    root = _des().rootComponent
    for i in range(root.occurrences.count):
        o = root.occurrences.item(i)
        if o.component.name == name:
            return o
    raise RuntimeError('occurrence not found: ' + name)


# =================================================================== THIGH
def build_thigh():
    g = geom()
    c = comp(THIGH)
    y0c, y1c = g['cap_y0'], g['cap_y1']
    items = []

    # --- knee head with male pyramid (printed PA-CF for bench work; machine Ti/Al for clinical use)
    top_plate = box(-26, 26, -25, 25, 62, 68)
    for sx in (-1, 1):                                        # 45 deg chamfers: knee-flexion clearance
        D(top_plate, obox((sx * 26, 0, 68), (0.7071, 0, 0.7071), (0, 1, 0), 14, 60, 14))
    head = U(cyl_y(0, 0, -25, y0c, 28.0),
             box(-26, 26, -25, y0c, 0, 62),
             top_plate,
             cyl_z(0, 0, 68, 72.5, 21.0),
             frustum_sq(0, 0, 72.5, 83.0, 16.0, 21.0))
    D(head, cyl_y(0, 0, -40, 40, 6.0))                       # D12 axle bore
    for k in range(6):                                        # M5 bolts to capstan disc
        t = math.radians(60 * k + 30)
        D(head, cyl_y(18 * math.cos(t), 18 * math.sin(t), y0c - 16, y0c + 0.5, 2.6))
    D(head, box(-12, 12, -26, -15, 18, 50))                   # thigh IMU / wiring pocket (lateral)
    items.append(('P03_KneeHead', head))

    # --- knee capstan disc (full disc: 120 deg ROM + cable wrap needs ~260 deg of groove)
    disc = cyl_y(0, 0, y0c, y1c, 60.0)
    D(disc, ring_y(0, 0, 16.5, 22.0, 61.0, g['knee_disc_r']))   # cable groove (2 cable lanes)
    for k in range(6):
        t = math.radians(60 * k)
        D(disc, cyl_y(37 * math.cos(t), 37 * math.sin(t), y0c - 1, y1c + 1, 11.0))
    D(disc, cyl_y(0, 0, y0c - 1, y1c + 1, 6.0))
    for k in range(6):
        t = math.radians(60 * k + 30)
        D(disc, cyl_y(18 * math.cos(t), 18 * math.sin(t), y0c - 1, y1c + 1, 2.6))
    items.append(('P04_KneeCapstanDisc', disc))

    # --- female pyramid receiver (socket adapter)
    rec = cyl_z(0, 0, 72.5, 96.0, 25.0)
    D(rec, box(-11.5, 11.5, -11.5, 11.5, 72.0, 85.5))
    for (ux, uy) in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        D(rec, cyl((9 * ux, 9 * uy, 79.0), (27 * ux, 27 * uy, 79.0), 4.0))
    for k in range(4):
        t = math.radians(90 * k + 45)
        D(rec, cyl_z(19 * math.cos(t), 19 * math.sin(t), 86.0, 97.0, 3.2))
    items.append(('P02_PyramidReceiver', rec))

    # --- hardware: axle + end caps
    items.append(('H01_KneeAxle_D12', cyl_y(0, 0, -g['CO'], g['CO'], 6.0)))
    items.append(('H02_KneeCap_Lat', D(cyl_y(0, 0, -g['CO'] - 3, -g['CO'], 15.0),
                                       cyl_y(0, 0, -g['CO'] - 1.2, -g['CO'] + 1, 4.0))))
    items.append(('H02_KneeCap_Med', D(cyl_y(0, 0, g['CO'], g['CO'] + 3, 15.0),
                                       cyl_y(0, 0, g['CO'] - 1, g['CO'] + 1.2, 4.0))))
    add_bodies(c, items, 'thigh_parts')
    fillet_edges_where(c, 'P03_KneeHead',
                       lambda e: e.geometry.curveType == adsk.core.Curve3DTypes.Line3DCurveType
                       and abs(_bb(e)[2] - _bb(e)[5]) > 30 and abs(_bb(e)[0]) > 20, 6.0)
    build_socket(c, g)


def build_socket(c, g):
    planes = c.constructionPlanes
    secs = [(104.0, 38.0, 34.0), (200.0, 56.0, 52.0), (300.0, 68.0, 64.0), (392.0, 74.0, 70.0)]
    profs = []
    for (z, a, b) in secs:
        pi = planes.createInput()
        pi.setByOffset(c.xYConstructionPlane, vi(z))
        pl = planes.add(pi)
        pl.isLightBulbOn = False
        sk = c.sketches.add(pl)
        sk.name = 'socket_z%d' % int(z)
        cen = sk.modelToSketchSpace(p3(0, 0, z))
        ma = sk.modelToSketchSpace(p3(a, 0, z))
        mi = sk.modelToSketchSpace(p3(0, b, z))
        sk.sketchCurves.sketchEllipses.add(cen, ma, mi)
        sk.isVisible = False
        profs.append(sk.profiles.item(0))
    lofts = c.features.loftFeatures
    li = lofts.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    for pr in profs:
        li.loftSections.add(pr)
    li.isSolid = True
    lf = lofts.add(li)
    lf.name = 'LOFT_Socket'
    body = lf.bodies.item(0)
    body.name = 'P01_Socket'
    top = None
    for f in body.faces:
        if f.geometry.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType and abs(f.pointOnFace.z - 39.2) < 0.05:
            top = f
    oc = adsk.core.ObjectCollection.create()
    oc.add(top)
    si = c.features.shellFeatures.createInput(oc, False)
    si.insideThickness = vi(4.0)
    c.features.shellFeatures.add(si)
    # distal base + suction-valve boss (joined), trim line (medial lower), valve hole, bolt holes
    tilt = math.radians(15.0)
    n = (0.0, math.sin(tilt), math.cos(tilt))
    p0 = (0.0, 0.0, 374.0)
    trim = obox((p0[0] + n[0] * 150, p0[1] + n[1] * 150, p0[2] + n[2] * 150), n, (1, 0, 0), 300, 400, 400)
    base = cyl_z(0, 0, 96.0, 106.0, 32.0)
    boss = cyl_x(-8.0, 150.0, 43.0, 53.0, 10.0)
    add_bodies(c, [('tmp_socket_base', base), ('tmp_socket_boss', boss)], 'socket_add')
    combine(c, 'P01_Socket', ['tmp_socket_base', 'tmp_socket_boss'], 'join')
    cut_tools = [('tmp_trim', trim), ('tmp_valve', cyl_x(-8.0, 150.0, 30.0, 60.0, 5.0))]
    for k in range(4):
        t = math.radians(90 * k + 45)
        cut_tools.append(('tmp_sb%d' % k, cyl_z(19 * math.cos(t), 19 * math.sin(t), 95.0, 108.0, 3.2)))
    add_bodies(c, cut_tools, 'socket_cut')
    combine(c, 'P01_Socket', [n_ for n_, _ in cut_tools], 'cut')
    add_bodies(c, [('H03_SuctionValve', U(cyl_x(-8.0, 150.0, 53.0, 57.0, 8.0),
                                          cyl_x(-8.0, 150.0, 40.0, 53.0, 4.8)))], 'valve')


# =================================================================== SHANK (knee frame, pylon, ankle frame)
def build_shank():
    g = geom()
    c = comp(SHANK)
    KMx, KMz = g['KM']
    AMx, AMz = g['AM']
    AZ = g['AZ']
    PI_, PO, CO = g['PI'], g['PO'], g['CO']

    # ---------------- knee frame plates + covers (sketch based)
    # posterior spacer lobe sits outside the swept envelope of socket/receiver at 120 deg flexion
    k_out = circle_hull([(0, 0, 66.0), (KMx, KMz, 56.0), (-62, -110, 10.0), (-24, -182, 8.0), (24, -182, 8.0)])
    k_holes = [(0, 0, 14.0), (45, -72, 2.75), (-62, -110, 2.25),
               (-24, -160, 2.75), (24, -160, 2.75), (-24, -180, 2.75), (24, -180, 2.75)]
    extrude_outline(c, 'xz', k_out, -PO, -PI_, 'P05_KneePlate_Lat', k_holes + [(KMx, KMz, 20.0)])
    extrude_outline(c, 'xz', k_out, PI_, PO, 'P06_KneePlate_Med', k_holes + [(KMx, KMz, 13.0)])
    k_cov = circle_hull([(0, 0, 61.0), (KMx, KMz, 51.0), (-62, -110, 5.0), (-24, -182, 3.0), (24, -182, 3.0)])
    extrude_outline(c, 'xz', k_cov, -CO, -PO, 'P08_KneeCover_Lat', [(0, 0, 16.0)])
    extrude_outline(c, 'xz', k_cov, PO, CO, 'P09_KneeCover_Med', [(0, 0, 16.0)])

    # ---------------- ankle frame plates + covers
    a_out = circle_hull([(0, AZ, 40.0), (AMx, AMz, 56.0), (-14, AZ + 99, 8.0), (14, AZ + 99, 8.0)])
    a_holes = [(0, AZ, 14.0), (-14, AZ + 101, 2.75), (14, AZ + 101, 2.75)]
    extrude_outline(c, 'xz', a_out, -PO, -PI_, 'P11_AnklePlate_Lat', a_holes + [(AMx, AMz, 20.0)])
    extrude_outline(c, 'xz', a_out, PI_, PO, 'P12_AnklePlate_Med', a_holes + [(AMx, AMz, 13.0)])
    a_cov = circle_hull([(0, AZ, 35.0), (AMx, AMz, 51.0), (-14, AZ + 99, 3.0), (14, AZ + 99, 3.0)])
    extrude_outline(c, 'xz', a_cov, -CO, -PO, 'P14_AnkleCover_Lat', [(0, AZ, 16.0)])
    extrude_outline(c, 'xz', a_cov, PO, CO, 'P15_AnkleCover_Med', [(0, AZ, 16.0)])
    # flat seat for the load cell on top of the ankle frame (hull would otherwise slope over it)
    notch = [('tmp_notch_%d' % i, box(-33, 45, -60, 60, g['ATOP'][1], g['ATOP'][1] + 60)) for i in range(4)]
    add_bodies(c, notch, 'ankle_notch')
    for i, nm in enumerate(('P11_AnklePlate_Lat', 'P12_AnklePlate_Med', 'P14_AnkleCover_Lat', 'P15_AnkleCover_Med')):
        combine(c, nm, ['tmp_notch_%d' % i], 'cut')

    for nm, sg in (('P05_KneePlate_Lat', -1), ('P06_KneePlate_Med', 1), ('P11_AnklePlate_Lat', -1),
                   ('P12_AnklePlate_Med', 1), ('P08_KneeCover_Lat', -1), ('P09_KneeCover_Med', 1),
                   ('P14_AnkleCover_Lat', -1), ('P15_AnkleCover_Med', 1)):
        fillet_outer_loop(c, nm, 1, sg, 2.0)

    items = []
    # ---------------- knee clamp block (pylon receiver)
    clamp = box(-32, 32, -PI_, PI_, g['KCLAMP'][0], g['KCLAMP'][1])
    D(clamp, cyl_z(0, 0, g['KCLAMP'][0] - 1, g['KCLAMP'][0] + 35, 15.1),
      box(-33, 0, -1.0, 1.0, g['KCLAMP'][0] - 1, g['KCLAMP'][0] + 25))
    for (x, z) in ((-24, -160), (24, -160), (-24, -180), (24, -180)):
        D(clamp, cyl_y(x, z, -30, 30, 2.75))
    items.append(('P07_KneeClamp', clamp))

    # ---------------- knee drive: AK80-9 + capstan drum + bearings + spacers + cables
    items += actuator_items('Knee', KMx, KMz, g)
    items.append(('H10_Bearing6001_KneeLat', ring_y(0, 0, -PO, -PI_, 14.0, 6.0)))
    items.append(('H10_Bearing6001_KneeMed', ring_y(0, 0, PI_, PO, 14.0, 6.0)))
    for i, (x, z, ro, ri) in enumerate(((45, -72, 6.0, 2.75), (-62, -110, 4.5, 2.25))):
        items.append(('H11_Spacer_%d' % (i + 1), ring_y(x, z, -PI_, PI_, ro, ri)))
    items.append(('H12_ElectronicsStack', box(10, 50, 13.5, 25, -148, -112)))
    rK = g['knee_disc_r'] + g['cable_r']
    rD = g['drum_r'] + g['cable_r']
    for i, (a, b) in enumerate(cross_spans((0, 0), rK, (KMx, KMz), rD)):
        yy = 17.9 if i == 0 else 20.6
        items.append(('H13_KneeCable_%d' % (i + 1), cyl((a[0], yy, a[1]), (b[0], yy, b[1]), g['cable_r'])))

    # ---------------- pylon
    items.append(('H20_Pylon_D30', D(cyl_z(0, 0, g['PYL'][0], g['PYL'][1], 15.0),
                                     cyl_z(0, 0, g['PYL'][0] - 1, g['PYL'][1] + 1, 13.0))))

    # ---------------- ankle top block, load cell, pylon clamp
    top = box(-22, 22, -PI_, PI_, g['ATOP'][0], g['ATOP'][1])
    for x in (-14, 14):
        D(top, cyl_y(x, AZ + 101, -30, 30, 2.75))
    for k in range(4):
        t = math.radians(90 * k + 45)
        D(top, cyl_z(16 * math.cos(t), 16 * math.sin(t), g['ATOP'][0] - 1, g['ATOP'][1] + 1, 2.6))
    items.append(('P13_AnkleTopBlock', top))
    lc = U(cyl_z(0, 0, g['LC'][0], g['LC'][1], 30.0), box(26, 36, -6, 6, g['LC'][0] + 5, g['LC'][1] - 5))
    zm = 0.5 * (g['LC'][0] + g['LC'][1])
    D(lc, D(cyl_z(0, 0, zm - 0.7, zm + 0.7, 31.0), cyl_z(0, 0, zm - 1.0, zm + 1.0, 29.2)))
    items.append(('H21_LoadCell_6axis', lc))
    zc0, zc1 = g['ACLAMP']
    pc = U(cyl_z(0, 0, zc0, zc1, 24.0), box(-34, -16, -10, 10, zc1 - 22, zc1 - 2))
    D(pc, cyl_z(0, 0, zc1 - 28, zc1 + 1, 15.1), box(-35, 0, -1.0, 1.0, zc1 - 24, zc1 + 1),
      cyl_y(-27, zc1 - 12, -12, 12, 2.75))
    items.append(('P17_PylonClamp_Ankle', pc))

    # ---------------- ankle drive
    items += actuator_items('Ankle', AMx, AMz, g)
    items.append(('H10_Bearing6001_AnkleLat', ring_y(0, AZ, -PO, -PI_, 14.0, 6.0)))
    items.append(('H10_Bearing6001_AnkleMed', ring_y(0, AZ, PI_, PO, 14.0, 6.0)))
    rS = g['ankle_sector_r'] + g['cable_r']
    for i, (a, b) in enumerate(cross_spans((0, AZ), rS, (AMx, AMz), rD)):
        yy = 17.9 if i == 0 else 20.6
        items.append(('H14_AnkleCable_%d' % (i + 1), cyl((a[0], yy, a[1]), (b[0], yy, b[1]), g['cable_r'])))
    pea = U(cyl_y(0, AZ, -CO - 12, -CO, 24.0), cyl_y(0, AZ, -CO - 14, -CO - 12, 16.0))
    D(pea, ring_y(0, AZ, -CO - 7, -CO - 5, 25, 22.5), cyl_y(0, AZ, -CO - 20, -CO + 1, 6.0))
    items.append(('P16_PEA_SpringCartridge', pea))
    add_bodies(c, items, 'shank_parts')


def actuator_items(tag, x, z, g):
    PI_, PO = g['PI'], g['PO']
    y_out = -PI_ + g['act_w']                   # output face (11.5)
    act = cyl_y(x, z, -PI_, y_out, g['act_r'])
    D(act, ring_y(x, z, -PI_ + 11, -PI_ + 13, g['act_r'] + 1, g['act_r'] - 1.5),
      ring_y(x, z, -PI_ + 16, -PI_ + 18, g['act_r'] + 1, g['act_r'] - 1.5))
    flange = cyl_y(x, z, y_out, g['cap_y0'], 30.0)
    drum = U(cyl_y(x, z, g['cap_y0'], g['cap_y1'], g['drum_r']), cyl_y(x, z, g['cap_y1'], PO, 5.0))
    for k in range(5):
        y = g['cap_y0'] + 2.0 + 2.4 * k
        D(drum, ring_y(x, z, y, y + 1.2, g['drum_r'] + 1, g['drum_r'] - 0.8))
    return [('H%s_Actuator_AK80-9' % ('30' if tag == 'Knee' else '31'), act),
            ('H%s_OutputFlange' % ('32' if tag == 'Knee' else '33'), flange),
            ('P%s_CapstanDrum' % ('10' if tag == 'Knee' else '18'), drum),
            ('H10_Bearing6000_%sDrum' % tag, ring_y(x, z, PI_, PO, 13.0, 5.0))]


# =================================================================== TALUS
def build_talus():
    g = geom()
    c = comp(TALUS)
    AZ, RZ = g['AZ'], g['ROLL_Z']
    y0c, y1c = g['cap_y0'], g['cap_y1']
    tal = U(cyl_y(0, AZ, -25, y0c, 26.0),
            box(-40, 40, -12, 12, AZ - 26, AZ - 14),
            box(26, 40, -12, 12, AZ - 58, AZ - 14),
            box(-40, -26, -12, 12, AZ - 58, AZ - 14),
            box(-10, 14, -26, 26, AZ - 28, AZ - 20))
    D(tal, cyl_y(0, AZ, -40, 40, 6.0), cyl_x(0, RZ, -45, 45, 4.0))
    for k in range(6):
        t = math.radians(60 * k + 30)
        D(tal, cyl_y(14 * math.cos(t), AZ + 14 * math.sin(t), y0c - 14, y0c + 0.5, 2.2))
    sec = U(pie_y(0, AZ, y0c, y1c, 64.0, 50.0, 217.0), cyl_y(0, AZ, y0c, y1c, 19.0))
    D(sec, ring_y(0, AZ, 16.5, 22.0, 66.0, g['ankle_sector_r']))
    for a in (80.0, 133.0, 186.0):
        t = math.radians(a)
        D(sec, cyl_y(41 * math.cos(t), AZ + 41 * math.sin(t), y0c - 1, y1c + 1, 9.0))
    D(sec, cyl_y(0, AZ, y0c - 1, y1c + 1, 6.0))
    for k in range(6):
        t = math.radians(60 * k + 30)
        D(sec, cyl_y(14 * math.cos(t), AZ + 14 * math.sin(t), y0c - 1, y1c + 1, 2.2))
    items = [('P19_Talus', tal), ('P20_AnkleCapstanSector', sec),
             ('H40_AnkleAxle_D12', cyl_y(0, AZ, -g['CO'] - 12, g['CO'], 6.0)),
             ('H41_AnkleCap_Med', D(cyl_y(0, AZ, g['CO'], g['CO'] + 3, 15.0),
                                    cyl_y(0, AZ, g['CO'] - 1, g['CO'] + 1.2, 4.0))),
             ('H42_RollAxle_D8', cyl_x(0, RZ, -41, 41, 4.0))]
    add_bodies(c, items, 'talus_parts')


# =================================================================== FOOT
FOOT_PLAN = [('L', (-42, -28), (40, -38)), ('L', (40, -38), (116, -42)), ('L', (116, -42), (116, 46)),
             ('L', (116, 46), (20, 32)), ('L', (20, 32), (-42, 28)), ('A', (-42, 28), (-70, 0), (-42, -28))]
TOE_PLAN = [('L', (118, -42), (158, -38)), ('A', (158, -38), (192, 12), (168, 45)),
            ('L', (168, 45), (118, 46)), ('L', (118, 46), (118, -42))]


def build_foot():
    g = geom()
    c = comp(FOOT)
    F, AZ, RZ = g['FLOOR'], g['AZ'], g['ROLL_Z']
    TX, TZ = g['TOE_X'], g['TOE_Z']
    # keel = plan extrusion  INTERSECT  side-profile extrusion
    extrude_outline(c, 'xy', FOOT_PLAN, F + 6, F + 50, 'P21_FootKeel')
    prof = [(-72, 6), (118, 6), (118, 15), (70, 19), (40, 24), (-20, 24), (-52, 20), (-72, 20)]
    segs = [('L', (prof[i][0], F + prof[i][1]), (prof[(i + 1) % len(prof)][0], F + prof[(i + 1) % len(prof)][1]))
            for i in range(len(prof))]
    extrude_outline(c, 'xz', segs, -60, 60, 'tmp_keel_profile')
    combine(c, 'P21_FootKeel', ['tmp_keel_profile'], 'intersect')
    fillet_edges_where(c, 'P21_FootKeel', lambda e: _bb(e)[5] > F + 9.0, 3.0)
    # subtalar boss + web, toe knuckles (join);  bores + toe notch (cut)
    boss = U(cyl_x(0, RZ, -24, 24, 14.0), box(-22, 22, -8, 8, F + 20, RZ))
    kn = U(cyl_y(TX, TZ, 22, 40, 5.5), cyl_y(TX, TZ, -40, -22, 5.5),
           box(108, TX, 22, 40, F + 6, TZ + 5.5), box(108, TX, -40, -22, F + 6, TZ + 5.5))
    add_bodies(c, [('tmp_boss', boss), ('tmp_knuckles', kn)], 'foot_add')
    combine(c, 'P21_FootKeel', ['tmp_boss', 'tmp_knuckles'], 'join')
    cuts = [('tmp_bore', cyl_x(0, RZ, -30, 30, 4.2)),
            ('tmp_brg1', cyl_x(0, RZ, -25, -17, 11.0)), ('tmp_brg2', cyl_x(0, RZ, 17, 25, 11.0)),
            ('tmp_notch', box(106, 130, -21, 21, F - 1, F + 30)),
            ('tmp_pin', cyl_y(TX, TZ, -45, 45, 2.6))]
    add_bodies(c, cuts, 'foot_cut')
    combine(c, 'P21_FootKeel', [n for n, _ in cuts], 'cut')
    # TPU sole + roll springs + pin + subtalar bearings
    extrude_outline(c, 'xy', FOOT_PLAN, F, F + 6, 'P22_SolePad_TPU')
    items = [('P23_RollSpring_TPU_Med', box(-8, 12, 14, 23, F + 24, AZ - 28.5)),
             ('P23_RollSpring_TPU_Lat', box(-8, 12, -23, -14, F + 24, AZ - 28.5)),
             ('H50_ToePin_D5', cyl_y(TX, TZ, -40, 40, 2.5)),
             ('H51_Bearing608_Ant', D(cyl_x(0, RZ, 17, 24, 11.0), cyl_x(0, RZ, 16, 25, 4.0))),
             ('H51_Bearing608_Post', D(cyl_x(0, RZ, -24, -17, 11.0), cyl_x(0, RZ, -25, -16, 4.0)))]
    add_bodies(c, items, 'foot_parts')


# =================================================================== TOE
def build_toe():
    g = geom()
    c = comp(TOE)
    F = g['FLOOR']
    TX, TZ = g['TOE_X'], g['TOE_Z']
    extrude_outline(c, 'xy', TOE_PLAN, F + 6, F + 14, 'P24_Toe')
    add_bodies(c, [('tmp_tk', U(cyl_y(TX, TZ, -21, 21, 5.5), box(TX, 128, -21, 21, F + 6, TZ + 5.5)))], 'toe_add')
    combine(c, 'P24_Toe', ['tmp_tk'], 'join')
    cuts = [('tmp_c1', box(108, 127.5, 21.5, 60, F - 1, F + 30)), ('tmp_c2', box(108, 127.5, -60, -21.5, F - 1, F + 30)),
            ('tmp_pin', cyl_y(TX, TZ, -45, 45, 2.6))]
    add_bodies(c, cuts, 'toe_cut')
    combine(c, 'P24_Toe', [n for n, _ in cuts], 'cut')
    fillet_edges_where(c, 'P24_Toe', lambda e: _bb(e)[5] > F + 13.0 and _bb(e)[0] > 126.0, 2.5)
    extrude_outline(c, 'xy', TOE_PLAN, F, F + 6, 'P25_ToePad_TPU')
    add_bodies(c, [('tmp_tp', box(100, TX + 1.0, -60, 60, F - 1, F + 7))], 'toepad_cut')
    combine(c, 'P25_ToePad_TPU', ['tmp_tp'], 'cut')


# =================================================================== APPEARANCE / MATERIAL
def _appearance(name):
    des = _des()
    a = des.appearances.itemByName(name)
    if a:
        return a
    rgb, base_name = COLORS[name]
    lib = _app().materialLibraries.itemByName('Fusion Appearance Library')
    base = lib.appearances.itemByName(base_name)
    a = des.appearances.addByCopy(base, name)
    for key in ('Color', 'Base Color', 'Diffuse Color'):
        try:
            prop = a.appearanceProperties.itemByName(key)
            if prop:
                adsk.core.ColorProperty.cast(prop).value = adsk.core.Color.create(rgb[0], rgb[1], rgb[2], 255)
        except Exception:
            pass
    return a


def _material(key):
    lib = _app().materialLibraries.itemByName('Fusion Material Library')
    try:
        return lib.materials.itemByName(MATERIALS[key])
    except Exception:
        return None


def style_for(body_name):
    n = body_name
    if n.startswith('P01'):
        return 'Smoke Clear', 'socket'
    if 'Cover' in n or 'PEA' in n or 'RollSpring' in n:
        return 'Accent Red', ('tpu' if 'TPU' in n else 'printed')
    if 'SolePad' in n or 'ToePad' in n:
        return 'TPU Black', 'tpu'
    if 'Actuator' in n:
        return 'Actuator Black', 'alu'
    if 'Cable' in n:
        return 'Cable White', 'pcb'
    if 'Electronics' in n:
        return 'PCB Green', 'pcb'
    if any(k in n for k in ('Capstan', 'Drum', 'Pylon', 'LoadCell', 'Spacer', 'OutputFlange')):
        return 'Aluminium', 'alu'
    if any(k in n for k in ('Axle', 'Cap_', 'Bearing', 'Pin', 'Valve')):
        return 'Steel', 'steel'
    return 'Graphite', 'printed'


def apply_styles():
    for ln in LINKS:
        c = comp(ln)
        for b in c.bRepBodies:
            ap, mat = style_for(b.name)
            m = _material(mat)
            if m:
                try:
                    b.material = m
                except Exception:
                    pass
            b.appearance = _appearance(ap)


# =================================================================== JOINTS
JOINTS = [
    # name,          child, parent, point key, axis, lower, upper (deg, Fusion sign)
    ('knee',        SHANK, THIGH, 'K', 'y', -5.0, 120.0),
    ('ankle_pitch', TALUS, SHANK, 'A', 'y', -15.0, 20.0),
    ('ankle_roll',  FOOT,  TALUS, 'R', 'x', -10.0, 10.0),
    ('toe',         TOE,   FOOT,  'T', 'y', -40.0, 0.0),
]


def joint_points(g):
    return {'K': (0.0, 0.0, 0.0), 'A': (0.0, 0.0, g['AZ']), 'R': (0.0, 0.0, g['ROLL_Z']),
            'T': (g['TOE_X'], 0.0, g['TOE_Z'])}


def make_joints():
    g = geom()
    des = _des()
    root = des.rootComponent
    occ(THIGH).isGrounded = True
    pts = joint_points(g)
    dirs = {'x': adsk.fusion.JointDirections.XAxisJointDirection,
            'y': adsk.fusion.JointDirections.YAxisJointDirection,
            'z': adsk.fusion.JointDirections.ZAxisJointDirection}
    for (name, child, parent, key, ax, lo, hi) in JOINTS:
        pc = comp(parent)
        sk = pc.sketches.add(pc.xYConstructionPlane)
        sk.name = 'JP_' + name
        sp = sk.sketchPoints.add(sk.modelToSketchSpace(p3(*pts[key])))
        sk.isVisible = False
        geo = adsk.fusion.JointGeometry.createByPoint(sp.createForAssemblyContext(occ(parent)))
        ji = root.asBuiltJoints.createInput(occ(child), occ(parent), geo)
        ji.setAsRevoluteJointMotion(dirs[ax])
        j = root.asBuiltJoints.add(ji)
        j.name = name
        lim = j.jointMotion.rotationLimits
        lim.isMinimumValueEnabled = True
        lim.minimumValue = math.radians(lo)
        lim.isMaximumValueEnabled = True
        lim.maximumValue = math.radians(hi)
        lim.isRestValueEnabled = True
        lim.restValue = 0.0


def set_pose(**deg):
    root = _des().rootComponent
    for i in range(root.asBuiltJoints.count):
        j = root.asBuiltJoints.item(i)
        if j.name in deg:
            j.jointMotion.rotationValue = math.radians(deg[j.name])


def view(eye_dir=(1.0, -1.1, 0.35), target=(20.0, 0.0, -110.0), dist=2600.0, fit=True, fn=None, w=1200, h=1500):
    """Point the camera (Z up). eye_dir is from target towards the eye. Optionally save an image."""
    vp = _app().activeViewport
    cam = vp.camera
    L = math.sqrt(sum(c_ * c_ for c_ in eye_dir))
    e = [target[i] + eye_dir[i] / L * dist for i in range(3)]
    cam.isSmoothTransition = False
    cam.target = p3(*target)
    cam.eye = p3(*e)
    cam.upVector = v3(0, 0, 1)
    vp.camera = cam
    if fit:
        vp.fit()
    vp.refresh()
    if fn:
        vp.saveAsImageFile(fn, w, h)


def interference(poses):
    """Fusion interference analysis (all body pairs) for a list of joint poses (deg, Fusion sign)."""
    des = _des()
    root = des.rootComponent
    out = []
    for pose in poses:
        full = dict(knee=0.0, ankle_pitch=0.0, ankle_roll=0.0, toe=0.0)
        full.update(pose)
        set_pose(**full)
        oc = adsk.core.ObjectCollection.create()
        for i in range(root.occurrences.count):
            o = root.occurrences.item(i)
            for b in o.component.bRepBodies:
                if b.isSolid:
                    oc.add(b.createForAssemblyContext(o))
        ii = des.createInterferenceInput(oc)
        ii.areCoincidentFacesIncluded = False
        res = des.analyzeInterference(ii)
        hits = []
        for k in range(res.count):
            r = res.item(k)
            try:
                vol = r.interferenceBody.volume * 1000.0
            except Exception:
                vol = -1.0
            hits.append((r.entityOne.name, r.entityTwo.name, round(vol, 3)))
        out.append((pose, hits))
    set_pose(knee=0.0, ankle_pitch=0.0, ankle_roll=0.0, toe=0.0)
    return out


# =================================================================== EXPORT
def export_all(out_root):
    """Export per-body STL (zero pose, world frame, mm), STEP, F3D and a JSON of body/joint metadata."""
    des = _des()
    em = des.exportManager
    set_pose(knee=0.0, ankle_pitch=0.0, ankle_roll=0.0, toe=0.0)
    stl_dir = os.path.join(out_root, 'cad', 'stl_world')
    step_dir = os.path.join(out_root, 'cad', 'step')
    for d in (stl_dir, step_dir):
        os.makedirs(d, exist_ok=True)
    g = geom()
    meta = {'units': 'mm', 'frame': 'origin knee axis, +X anterior, +Y medial (right leg), +Z up',
            'params': P, 'geom': g,
            'joints': [dict(name=j[0], child=j[1], parent=j[2], point=joint_points(g)[j[3]], axis=j[4],
                            lower_deg_fusion=j[5], upper_deg_fusion=j[6]) for j in JOINTS],
            'bodies': []}
    for ln in LINKS:
        c = comp(ln)
        for b in c.bRepBodies:
            if not b.isSolid:
                continue
            fn = os.path.join(stl_dir, b.name + '.stl')
            o = em.createSTLExportOptions(b, fn)
            o.isBinaryFormat = True
            o.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
            try:
                o.unitType = adsk.fusion.DistanceUnits.MillimeterDistanceUnits
            except Exception:
                pass
            o.sendToPrintUtility = False
            em.execute(o)
            pp = b.physicalProperties
            com = pp.centerOfMass
            bb = b.boundingBox
            meta['bodies'].append(dict(
                name=b.name, link=ln, appearance=b.appearance.name if b.appearance else '',
                material=b.material.name if b.material else '',
                volume_mm3=pp.volume * 1000.0, area_mm2=pp.area * 100.0, fusion_mass_kg=pp.mass,
                com_mm=[com.x * 10, com.y * 10, com.z * 10],
                bbox_mm=[bb.minPoint.x * 10, bb.minPoint.y * 10, bb.minPoint.z * 10,
                         bb.maxPoint.x * 10, bb.maxPoint.y * 10, bb.maxPoint.z * 10]))
    so = em.createSTEPExportOptions(os.path.join(step_dir, 'CapLeg_assembly.step'), des.rootComponent)
    em.execute(so)
    try:
        fo = em.createFusionArchiveExportOptions(os.path.join(out_root, 'cad', 'CapLeg.f3d'), des.rootComponent)
        em.execute(fo)
    except Exception:
        pass
    with open(os.path.join(out_root, 'cad', 'fusion_export_meta.json'), 'w') as fh:
        json.dump(meta, fh, indent=1)
    return meta


# =================================================================== FULL BUILD
def build_all():
    new_document()
    build_thigh()
    build_shank()
    build_talus()
    build_foot()
    build_toe()
    apply_styles()
    make_joints()
    view()


def run(context):
    ui = None
    try:
        ui = _app().userInterface
        build_all()
        ui.messageBox('CapLeg built.\nUse ASSEMBLE > Drive Joints to move knee / ankle_pitch / ankle_roll / toe.')
    except Exception:
        if ui:
            ui.messageBox('CapLeg build failed:\n{}'.format(traceback.format_exc()))
