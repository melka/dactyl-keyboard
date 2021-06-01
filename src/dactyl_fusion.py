def union(shapes):
    print('union()')
    shape = None
    for item in shapes:
        if shape is None:
            shape = item
        else:
            shape = shape.union(item)
    return shape


def face_from_points(points):
    # print('face_from_points()')
    edges = []
    num_pnts = len(points)
    for i in range(len(points)):
        p1 = points[i]
        p2 = points[(i + 1) % num_pnts]
        edges.append(
            cq.Edge.makeLine(
                cq.Vector(p1[0], p1[1], p1[2]),
                cq.Vector(p2[0], p2[1], p2[2]),
            )
        )

    face = cq.Face.makeFromWires(cq.Wire.assembleEdges(edges))

    return face


def hull_from_points(points):
    print('hull_from_points()')
    hull_calc = sphull(points)
    n_faces = len(hull_calc.simplices)

    faces = []
    for i in range(n_faces):
        face_items = hull_calc.simplices[i]
        fpnts = []
        for item in face_items:
            fpnts.append(points[item])
        faces.append(face_from_points(fpnts))

    shape = cq.Solid.makeSolid(cq.Shell.makeShell(faces))
    shape = cq.Workplane('XY').union(shape)
    return shape


def hull_from_shapes(shapes, points=None):
    print('hull_from_shapes()')
    vertices = []
    for shape in shapes:
        verts = shape.vertices()
        for vert in verts.objects:
            vertices.append(np.array(vert.toTuple()))
    if points is not None:
        for point in points:
            vertices.append(np.array(point))

    shape = hull_from_points(vertices)
    return shape


def tess_hull(shapes, sl_tol=.5, sl_angTol=1):
    # print('hull_from_shapes()')
    vertices = []
    solids = []
    for wp in shapes:
        for item in wp.solids().objects:
            solids.append(item)

    for shape in solids:
        verts = shape.tessellate(sl_tol, sl_angTol)[0]
        for vert in verts:
            vertices.append(np.array(vert.toTuple()))

    shape = hull_from_points(vertices)
    return shape


def column_offset(column: int) -> list:
    # print('column_offset()')
    if column == 2:
        return [0, 2.82, -4.5]
    elif column >= 4:
        return [0, -12, 5.64]  # original [0 -5.8 5.64]
    else:
        return [0, 0, 0]


#########################
## Placement Functions ##
#########################


def rotate_around_x(position, angle):
    # print('rotate_around_x()')
    t_matrix = np.array(
        [
            [1, 0, 0],
            [0, np.cos(angle), -np.sin(angle)],
            [0, np.sin(angle), np.cos(angle)],
        ]
    )
    return np.matmul(t_matrix, position)


def rotate_around_y(position, angle):
    # print('rotate_around_y()')
    t_matrix = np.array(
        [
            [np.cos(angle), 0, np.sin(angle)],
            [0, 1, 0],
            [-np.sin(angle), 0, np.cos(angle)],
        ]
    )
    return np.matmul(t_matrix, position)




def add_translate(shape, xyz):
    print('add_translate()')
    vals = []
    for i in range(len(shape)):
        vals.append(shape[i] + xyz[i])
    return vals

def key_position(position, column, row):
    print('key_position()')
    return apply_key_geometry(
        position, add_translate, rotate_around_x, rotate_around_y, column, row
    )

############
## Thumbs ##
############


def thumborigin():
    # print('thumborigin()')
    origin = key_position([mount_width / 2, -(mount_height / 2), 0], 1, cornerrow)
    for i in range(len(origin)):
        origin[i] = origin[i] + thumb_offsets[i]
    return origin


def thumb_tr_place(shape):
    print('thumb_tr_place()')
    shape = rotate(shape, [10, -23, 10])
    shape = shape.translate(thumborigin())
    shape = shape.translate([-12, -16, 3])
    return shape


def thumb_tl_place(shape):
    print('thumb_tl_place()')
    shape = rotate(shape, [10, -23, 10])
    shape = shape.translate(thumborigin())
    shape = shape.translate([-32, -15, -2])
    return shape


def thumb_mr_place(shape):
    print('thumb_mr_place()')
    shape = rotate(shape, [-6, -34, 48])
    shape = shape.translate(thumborigin())
    shape = shape.translate([-29, -40, -13])
    return shape


def thumb_ml_place(shape):
    print('thumb_ml_place()')
    shape = rotate(shape, [6, -34, 40])
    shape = shape.translate(thumborigin())
    shape = shape.translate([-51, -25, -12])
    return shape


def thumb_br_place(shape):
    print('thumb_br_place()')
    shape = rotate(shape, [-16, -33, 54])
    shape = shape.translate(thumborigin())
    shape = shape.translate([-37.8, -55.3, -25.3])
    return shape


def thumb_bl_place(shape):
    print('thumb_bl_place()')
    shape = rotate(shape, [-4, -35, 52])
    shape = shape.translate(thumborigin())
    shape = shape.translate([-56.3, -43.3, -23.5])
    return shape


def thumb_1x_layout(shape, cap=False):
    print('thumb_1x_layout()')
    if cap:
        shapes = thumb_mr_place(shape)
        shapes = shapes.add(thumb_ml_place(shape))
        shapes = shapes.add(thumb_br_place(shape))
        shapes = shapes.add(thumb_bl_place(shape))
    else:
        shapes = union(
            [
                thumb_mr_place(shape),
                thumb_ml_place(shape),
                thumb_br_place(shape),
                thumb_bl_place(shape),
            ]
        )
    return shapes


def thumb_15x_layout(shape, cap=False):
    print('thumb_15x_layout()')
    if cap:
        shape = rotate(shape, (0, 0, 90))
        return thumb_tr_place(shape).add(thumb_tl_place(shape).solids().objects[0])
    else:
        return thumb_tr_place(shape).union(thumb_tl_place(shape))

sa_length = 18.25
sa_double_length = 37.5
web_thickness = 3.5 + .5
post_size = 0.1

def double_plate():
    print('double_plate()')
    plate_height = (sa_double_length - mount_height) / 3
    # plate_height = (2*sa_length-mount_height) / 3
    top_plate = cq.Workplane("XY").box(mount_width, plate_height, web_thickness)
    top_plate = translate(top_plate,
                          [0, (plate_height + mount_height) / 2, plate_thickness - (web_thickness / 2)]
                          )
    return union((top_plate, mirror(top_plate, 'XZ')))

def thumb(side="right"):
    print('thumb()')
    shape = thumb_1x_layout(rotate(single_plate(side=side), (0, 0, -90)))
    shape = shape.union(thumb_15x_layout(rotate(single_plate(side=side), (0, 0, -90))))
    shape = shape.union(thumb_15x_layout(double_plate()))
    return shape

def model_side(side="right"):
    print('model_right()')
    shape = cq.Workplane('XY').union(key_holes(side=side))
    # shape = shape.union(connectors())
    shape = shape.union(thumb(side=side))
    # shape = shape.union(thumb_connectors())
    # s2 = cq.Workplane('XY').union(case_walls())
    # s2 = union([s2, *screw_insert_outers])
    # # s2 = s2.union(teensy_holder())
    # s2 = s2.union(usb_holder())

    # s2 = s2.cut(rj9_space())
    # s2 = s2.cut(usb_holder_hole())
    # s2 = s2.cut(union(screw_insert_holes))

    # shape = shape.union(rj9_holder())
    # shape = shape.union(s2, tol=.01)
    # # shape = shape.union(wire_posts())
    # block = cq.Workplane("XY").box(350, 350, 40)
    # block = block.translate((0, 0, -20))
    # shape = shape.cut(block)

    # if show_caps:
    #     shape = shape.add(thumbcaps())
    #     shape = shape.add(caps())

    # if side == "left":
    #     shape = shape.mirror('YZ')

    return shape

mod_r = model_side(side="right")
cq.exporters.export(w=mod_r, fname=path.join(r"..", "step", r"right_clean.step"), exportType='STEP')