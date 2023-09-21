import numpy
from dv import AedatFile
import numpy as np
import open3d as o3d
import logging


def events_array_to_pcd(events_array, t_factor=1000, t_from_zero=True):
    """
    Function convert events numpy array to open3d pcd
    :param events_array: numpy array [ x, y, t, p ]
    :return: pcd
    """
    timestamp = events_array[:, 2].copy()
    polarity = events_array[:, 3].copy()
    space = np.zeros_like(timestamp)
    allones = np.ones_like(timestamp)

    if t_from_zero:
        events_array[:, 2] -= events_array[0, 2]
    events_array[:, 2] *= t_factor
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(events_array[:, :3])
    # pcd.colors = o3d.utility.Vector3dVector(np.array([timestamp, polarity, space]).transpose())
    pcd.colors = o3d.utility.Vector3dVector(np.array([polarity, space, allones - polarity]).transpose())
    return pcd


def read_events_from_txt(txt_path, dtype=float, comments="#", delimiter=None, skip_events=0, max_events=None):
    """
    Function to read events from text file
    :param txt_path: text file path
    :param dtype: data type
    :param comments: the character comments begin with
    :param delimiter: the delimiter character
    :param skip_events: the number of events need to be skipped at the beginning of the file
    :param max_events: the maximum number of events
    :return: a numpy array [x, y, t, p]
    """
    events = np.loadtxt(txt_path, dtype=dtype, comments=comments, delimiter=delimiter, skiprows=skip_events,
                        max_rows=max_events)
    return events[:, [1, 2, 0, 3]]


def read_poses_from_txt(txt_path, dtype=float, comments="#", delimiter=None, skip_poses=0, max_poses=None):
    """
    Read poses from text file
    Args:
        txt_path: text path
        dtype: data type
        comments: comment character
        delimiter: delimit character
        skip_poses: skip poses
        max_poses: max poses

    Returns: a numpy array [timestamp px py pz qx qy qz qw]

    """
    poses = np.loadtxt(txt_path, dtype=dtype, comments=comments, delimiter=delimiter, skiprows=skip_poses,
                       max_rows=max_poses)
    return poses


def load_aedatfile(file_path, get_frames=False, get_events=True):
    """
    Function to parse adeatfile
    :param get_frames: get frames or not
    :param get_events: get events or not
    :param file_path: file path
    :return: img_shape, frames, events_array
    """
    with AedatFile(file_path) as f:
        logging.info("Data location: " + file_path)
        # list all the names of streams in the file
        logging.info("Data content: " + str(f.names))

        # access dimensions of the event stream
        img_shape = f["events"].size[1], f["events"].size[0]

        # loop through the "frames" stream
        frames = []
        if get_frames:
            for frame in f["frames"]:
                frames.append([frame.image, frame.timestamp])

        logging.info("Event camera image shape: " + str(img_shape))

        if get_events:
            # events will be a named numpy array
            events = np.hstack([packet for packet in f["events"].numpy()])

            # Access information of all events by type
            timestamps, x, y, polarities = events["timestamp"], events["x"], events["y"], events["polarity"]
            events_array = np.array([x, y, timestamps / 1e6, polarities], dtype=float).transpose()
            # data time
            logging.info("Data begin timestamp: " + str(timestamps[0] / 1e6))
            logging.info("Data end timestamp: " + str(timestamps[-1] / 1e6))
            logging.info("Data duration: " + str((timestamps[-1] - timestamps[0]) / 1e6))
        else:
            events_array = np.zeros([1, 1])

    return img_shape, frames, events_array


if __name__ == "__main__":
    load_aedatfile("./data/dvSave-2021_11_05_20_28_33.aedat4")
