import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
from fx_load import *
from fx_utilities import *
import copy
from fx_lineset import *


def draw_events_xyt_matplotlib(events):
    """
    Function to draw events with matplotlib
    :param events: numpy array, [x, y, t, p]
    :return:
    """
    fig = plt.figure()
    ax = fig.add_subplot(projection="3d")

    for event in events:
        # For each set of style and range settings, plot n random points in the box
        # defined by x in [23, 32], y in [0, 100], z in [zlow, zhigh].

        ax.scatter(event[0], event[1], event[2], s=1.0)
        ax.set_xlabel("x axis")
        ax.set_ylabel("y axis")
        ax.set_zlabel("t axis")


def get_axis_xyt_open3d(x_size, y_size, t_size, origin=None):
    """
    Function to get a bounding box
    :param x_size: x length
    :param y_size: y length
    :param t_size: t length
    :return: Open3D line set
    """
    if origin is None:
        origin = [0, 0, 0]
    x_origin, y_origin, t_origin = origin
    points = [
        [0, 0, 0],
        [x_size, 0, 0],
        [0, y_size, 0],
        [x_size, y_size, 0],
        [0, 0, t_size],
        [x_size, 0, t_size],
        [0, y_size, t_size],
        [x_size, y_size, t_size],
    ]

    origins = [
        [x_origin, y_origin, t_origin],
        [x_origin, y_origin, t_origin],
        [x_origin, y_origin, t_origin],
        [x_origin, y_origin, t_origin],
        [x_origin, y_origin, t_origin],
        [x_origin, y_origin, t_origin],
        [x_origin, y_origin, t_origin],
        [x_origin, y_origin, t_origin]
    ]
    points = np.array(points) + np.array(origins)
    lines = [
        [0, 1],
        [0, 2],
        [1, 3],
        [2, 3],
        [4, 5],
        [4, 6],
        [5, 7],
        [6, 7],
        [0, 4],
        [1, 5],
        [2, 6],
        [3, 7],
    ]
    colors = [[0, 0, 0] for i in range(len(lines))]
    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points),
        lines=o3d.utility.Vector2iVector(lines),
    )
    line_set.colors = o3d.utility.Vector3dVector(colors)

    # return line_set

    line_mesh1 = LineMesh(points, lines, colors, radius=1.5)
    line_mesh1_geoms = line_mesh1.cylinder_segments

    return line_mesh1_geoms


def draw_two_pcd_open3d(pcd1, pcd2, x_size=240, y_size=180,
                        pcd1_origin=None,
                        pcd2_origin=None,
                        other_open3d_geometry=None):
    """
    draw pcd point cloud
    :param pcd2_origin:
    :param pcd1: point cloud
    :param x_size: x size
    :param y_size: y size
    :param other_open3d_geometry: list of other open3d geometry
    :return: none
    """
    if pcd2_origin is None:
        pcd2_origin = [0, 0, 0]
    if pcd1_origin is None:
        pcd1_origin = [0, 0, 0]
    x = np.asarray(pcd1.points)
    t1_size = np.max(np.asarray(pcd1.points)[:, 2])
    t2_size = np.max(np.asarray(pcd2.points)[:, 2])
    axis1 = get_axis_xyt_open3d(x_size, y_size, t1_size, pcd1_origin)
    axis2 = get_axis_xyt_open3d(x_size, y_size, t2_size, pcd2_origin)
    coord = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=80, origin=[0, 0, 0])
    pcd1 = copy.deepcopy(pcd1).translate(pcd1_origin)
    pcd2 = copy.deepcopy(pcd2).translate(pcd2_origin)
    # objects_to_draw = [pcd1, pcd2, *axis1, *axis2, coord]
    objects_to_draw = [pcd1, pcd2, *axis1, *axis2]

    # pcd_copy = copy.deepcopy(pcd).translate((1.3, 0, 0))
    # objects_to_draw.append(pcd_copy)

    if other_open3d_geometry:
        objects_to_draw.extend(other_open3d_geometry)
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=3840, height=2032)
    # draw objects
    for ob in objects_to_draw:
        vis.add_geometry(ob)

    # set view control
    ctr = vis.get_view_control()
    ctr.set_front([0, 0, -100])
    ctr.set_up([0, -100, 0])
    ctr.set_lookat([x_size / 2, y_size / 2, 100])
    ctr.set_zoom(0.1)

    # load view point
    # param = o3d.io.read_pinhole_camera_parameters("./config/viewpoint.json")
    param = o3d.io.read_pinhole_camera_parameters("./config/draw_viewpoint.json")
    ctr.convert_from_pinhole_camera_parameters(param)

    # load render config
    # vis.get_render_option().load_from_json("./config/graph_renderoption.json")
    vis.get_render_option().load_from_json("./config/graph_renderoption.json")
    # # vis.get_render_option().line_width = 10.0
    vis.run()

    # # save view point
    # param = vis.get_view_control().convert_to_pinhole_camera_parameters()
    # o3d.io.write_pinhole_camera_parameters("./config/draw_viewpoint.json", param)
    #
    # # save render config
    # vis.get_render_option().save_to_json("./config/graph_renderoption.json")

    vis.destroy_window()


def draw_pcd_open3d(pcd, x_size=240, y_size=180, other_open3d_geometry=None):
    """
    draw pcd point cloud
    :param pcd: point cloud
    :param x_size: x size
    :param y_size: y size
    :param other_open3d_geometry: list of other open3d geometry
    :return: none
    """
    x = np.asarray(pcd.points)
    t_size = np.max(np.asarray(pcd.points)[:, 2])
    axis = get_axis_xyt_open3d(x_size, y_size, t_size)
    coord = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=80, origin=[0, 0, 0])
    objects_to_draw = [pcd, *axis, coord]

    if other_open3d_geometry:
        objects_to_draw.extend(other_open3d_geometry)
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=3840, height=2032)
    # draw objects
    for ob in objects_to_draw:
        vis.add_geometry(ob)

    # set view control
    ctr = vis.get_view_control()
    ctr.set_front([0, 0, -100])
    ctr.set_up([0, -100, 0])
    ctr.set_lookat([x_size / 2, y_size / 2, 100])
    ctr.set_zoom(0.1)

    # load view point
    # param = o3d.io.read_pinhole_camera_parameters("./config/viewpoint.json")
    param = o3d.io.read_pinhole_camera_parameters("./config/draw_viewpoint.json")
    ctr.convert_from_pinhole_camera_parameters(param)

    # load render config
    # vis.get_render_option().load_from_json("./config/graph_renderoption.json")
    vis.get_render_option().load_from_json("./config/graph_renderoption.json")
    # # vis.get_render_option().line_width = 10.0
    vis.run()

    # # save view point
    # param = vis.get_view_control().convert_to_pinhole_camera_parameters()
    # o3d.io.write_pinhole_camera_parameters("./config/draw_viewpoint.json", param)
    #
    # # save render config
    # vis.get_render_option().save_to_json("./config/graph_renderoption.json")

    vis.destroy_window()


def draw_events_xyt_open3d(events, x_size=240, y_size=180, t_factor=1000, t_from_zero=False):
    """
    Function to draw event camera"s events using Open3D
    :param events: numpy array, [x, y, t, p]
    :param x_size: pixel number of x axis of image plane
    :param y_size: pixel number of y axis of image plane
    :param t_factor: factor when drawing t
    :param t_from_zero: t begin from zero
    :return:
    """
    idx_pos = (events[:, 3] == 1)
    idx_neg = (events[:, 3] == 0)

    # Note: numpy fancy index, return a copy, not a view
    events_pos = events[idx_pos]
    events_neg = events[idx_neg]

    if t_from_zero:
        events_pos[:, 2] -= events_pos[0, 2]
        events_neg[:, 2] -= events_neg[0, 2]

    events_pos[:, 2] *= t_factor
    events_neg[:, 2] *= t_factor

    pcd_pos = o3d.geometry.PointCloud()
    pcd_pos.points = o3d.utility.Vector3dVector(events_pos[:, :3])
    pcd_pos.paint_uniform_color([1, 0, 0])

    pcd_neg = o3d.geometry.PointCloud()
    pcd_neg.points = o3d.utility.Vector3dVector(events_neg[:, :3])
    pcd_neg.paint_uniform_color([0, 0, 1])

    # get axis and coordinate
    axis = get_axis_xyt_open3d(x_size, y_size, events_pos[-1, 2])

    coord = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=80, origin=[0, 0, 0])

    # objects_to_draw = [pcd_pos, pcd_neg, *axis, coord]
    objects_to_draw = [pcd_pos, pcd_neg, *axis, ]
    # objects_to_draw = [pcd_pos, pcd_neg, coord]
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=3840, height=2032)
    # draw objects
    for ob in objects_to_draw:
        vis.add_geometry(ob)

    # set view control
    ctr = vis.get_view_control()
    ctr.set_front([0, 0, -100])
    ctr.set_up([0, -100, 0])
    ctr.set_lookat([x_size / 2, y_size / 2, 100])
    ctr.set_zoom(0.1)

    # # load view point
    param = o3d.io.read_pinhole_camera_parameters("./config/viewpoint.json")
    ctr.convert_from_pinhole_camera_parameters(param)

    # load render config
    vis.get_render_option().load_from_json("./config/renderoption.json")

    vis.run()

    # save view point
    # param = vis.get_view_control().convert_to_pinhole_camera_parameters()
    # o3d.io.write_pinhole_camera_parameters("./config/viewpoint.json", param)

    # save render config
    # vis.get_render_option().save_to_json("./config/renderoption.json")
    vis.destroy_window()


def cv_get_iwe(argmax, xs, ys, ts, ps, img_size, begin_index=0, cnt_num=1200):
    """
    Warp events to a image (a 2D numpy array).
    Args:
        argmax: iwe warp parameter [v_x, v_y]
        xs: xs
        ys: ys
        ts: ts
        ps: ps
        img_size: image size [width, height]
        begin_index: iwe warp begin index
        cnt_num: warp events number

    Returns: a 2d numpy array the image of warped events

    """
    xs_copy, ys_copy = xs[begin_index:begin_index + cnt_num].copy(), ys[begin_index:begin_index + cnt_num].copy()
    img = np.zeros([img_size[1], img_size[0]])
    for i in range(begin_index, begin_index + cnt_num):
        xs_copy[i] = xs_copy[i] - argmax[0] * (ts[i] - ts[0])
        ys_copy[i] = ys_copy[i] - argmax[1] * (ts[i] - ts[0])
        if 0 < xs_copy[i] < img_size[0] and 0 < ys_copy[i] < img_size[1]:
            img[int(ys_copy[i])][int(xs_copy[i])] = 1.0
    return img


def get_event_image(pcd, timestamp, time_interval, img_size):
    """
    get event image from events point cloud
    :param pcd: events point cloud
    :param timestamp: time you choose
    :param time_interval: time interval before and after the timestamp you choose
    :param img_size: image size
    :return: event image
    """
    events = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)
    if not is_sorted(events[:, 2]):
        logging.error("Events are not sorted with timestamp!")
    begin_idx = np.searchsorted(events[:, 2], timestamp - time_interval)
    end_idx = np.searchsorted(events[:, 2], timestamp + time_interval)
    events_selected = events[begin_idx:end_idx]
    colors_selected = colors[begin_idx:end_idx]
    # blank image
    # img = np.zeros([img_size[1], img_size[0], 3])
    img = np.zeros((img_size[1], img_size[0], 4), dtype=np.uint8)  # RGBA
    delta_value = 40

    # img[:, :] = [1, 1, 1]
    for idx, event in enumerate(events_selected):
        if 0 <= round(event[0]) < img_size[0] and 0 <= round(event[1]) < img_size[1]:
            if (img[round(event[1]), round(event[0]), 0] == 1):
                img[round(event[1]), round(event[0]), 0] = 0
                img[round(event[1]), round(event[0]), 1] = 0
                img[round(event[1]), round(event[0]), 2] = 0
            if (colors_selected[idx, 0] > 0):
                img[round(event[1]), round(event[0]), 0] = 255
                img[round(event[1]), round(event[0]), 3] += delta_value
                if img[round(event[1]), round(event[0]), 3] >= 255:
                    img[round(event[1]), round(event[0]), 3] = 255
            else:
                img[round(event[1]), round(event[0]), 2] = 255
                img[round(event[1]), round(event[0]), 3] += delta_value
                if img[round(event[1]), round(event[0]), 3] >= 255:
                    img[round(event[1]), round(event[0]), 3] = 255

    return img


def cv_get_iwe(argmax, events, img_size, begin_index=0, cnt_num=1200):
    """
    Warp events to a image (a 2D numpy array).
    Args:
        argmax: iwe warp parameter [v_x, v_y]
        events: a numpy array [x, y, t, p]
        img_size: image size [width, height]
        begin_index: iwe warp begin index
        cnt_num: warp events number

    Returns: a 2d numpy array the image of warped events

    """
    events_copy = events[begin_index:begin_index + cnt_num].copy()
    img = np.zeros([img_size[1], img_size[0]])
    for i in range(cnt_num):
        events_copy[i, 0] = events_copy[i, 0] - argmax[0] * events_copy[i, 2]
        events_copy[i, 1] = events_copy[i, 1] - argmax[1] * events_copy[i, 2]
        if 0 < events_copy[i, 0] < img_size[0] and 0 < events_copy[i, 1] < img_size[1]:
            img[int(events_copy[i, 1])][int(events_copy[i, 0])] = 1.0
    return img


def draw_transformations_open3d(R1, R2, t1, t2, factor=10.0):
    t1 = t1 / np.linalg.norm(t1)
    t2 = t2 / np.linalg.norm(t2)

    trans = np.identity(4)
    trans[:3, :3] = R1
    trans[:3, 3] = t1 * factor
    trans_calib = np.identity(4)
    trans_calib[:3, :3] = R2
    trans_calib[:3, 3] = t2 * factor
    mesh_left = o3d.geometry.TriangleMesh.create_coordinate_frame()
    mesh_right = copy.deepcopy(mesh_left)
    mesh_right.transform(np.linalg.inv(trans))
    mesh_right_calib = copy.deepcopy(mesh_left)
    mesh_right_calib.transform(np.linalg.inv(trans_calib))
    # o3d.visualization.draw_geometries([mesh_left, mesh_right, mesh_right_calib])

    objects_to_draw = [mesh_left, mesh_right, mesh_right_calib]
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    # draw objects
    for ob in objects_to_draw:
        vis.add_geometry(ob)
    # set view control
    ctr = vis.get_view_control()
    # load view point
    param = o3d.io.read_pinhole_camera_parameters("./config/trans_viewpoint.json")
    ctr.convert_from_pinhole_camera_parameters(param)

    # load render config
    vis.get_render_option().load_from_json("./config/trans_renderoption.json")

    vis.run()

    # # save view point
    # param = vis.get_view_control().convert_to_pinhole_camera_parameters()
    # o3d.io.write_pinhole_camera_parameters("./config/trans_viewpoint.json", param)
    #
    # # save render config
    # vis.get_render_option().save_to_json("./config/trans_renderoption.json")

    vis.destroy_window()

# if __name__ == "__main__":
#     events = read_events_from_txt("/home/wlxing/Data/eth_davis/davis_240c/slider_depth/events.txt", max_events=100000)
#     print(events[-1, :])
#     draw_events_xyt_open3d(events, t_factor=1000)
#     print(events[-1, :])
