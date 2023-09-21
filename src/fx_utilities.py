import numpy as np
import logging
import matplotlib.pyplot as plt
# from cv2 import cv2
import cv2
import yaml


def is_sorted(a):
    """
    Function to judge numpy array a is sorted or not
    :param a: one dimension numpy array
    :return: True or False
    """
    return np.all(a[:-1] <= a[1:])


# def get_line_mpl(line_param, img_shape):
#     """
#     Function to get points pair to draw a line
#     :param line_param: numpy array [a, b, c]
#     :param img_shape: image shape numpy array [image width, image height]
#     :return: return [x0, x1], [y0, y1]
#     """
#     a = line_param[0]
#     b = line_param[1]
#     c = line_param[2]
#
#     x0 = 0
#     y0 = -c / b
#     x1 = img_shape[0] - 1
#     y1 = -(c + a * x1) / b
#     return np.array([x0, x1]), np.array([y0, y1])


def get_line_mpl(line_param, img_shape):
    """
    Function to get points pair to draw a line
    :param line_param: numpy array [a, b, c]
    :param img_shape: image shape numpy array [image width, image height]
    :return: return x_points, y_points
    """
    a, b, c = line_param
    img_w, img_h = img_shape

    # 生成1000个点
    x_points = np.linspace(0, img_w - 1, 10000)
    y_points = -(a * x_points + c) / b

    # 筛选在图像范围内的点
    valid_indices = (y_points >= 0) & (y_points <= img_h - 1)
    x_points = x_points[valid_indices]
    y_points = y_points[valid_indices]

    return x_points, y_points

def point_line_distance(line_param, point):
    """
    Function to calculate the distance between a point and a lien
    :param line_param: numpy array [a, b, c]
    :param point: numpy array [x, y]
    :return: the distance
    """
    a = line_param[0]
    b = line_param[1]
    c = line_param[2]
    x = point[0]
    y = point[1]
    d = np.abs(a * x + b * y + c) / np.sqrt(a * a + b * b)
    return d


def average_position(pcd, timestamp, time_interval):
    """
    Get average position of a slice of a point cloud
    :param pcd: point cloud
    :param timestamp: time you want
    :param time_interval: time interval before and after the time you want
    :return: position numpy array [x, y, t]
    """
    events = np.asarray(pcd.points)
    if not is_sorted(events[:, 2]):
        logging.error("Events are not sorted with timestamp!")
    begin_idx = np.searchsorted(events[:, 2], timestamp - time_interval)
    end_idx = np.searchsorted(events[:, 2], timestamp + time_interval)
    events_selected = events[begin_idx:end_idx]
    # logging.info("Num of events selected to average position: " + str(end_idx - begin_idx))
    return np.mean(events_selected, axis=0)


def average_position_density(pcd, timestamp, time_interval, density_limit=1e3, remove_margin_events=True,
                             margin_value=0):
    """
    Get average position of a slice of a point cloud
    :param margin_value:
    :param remove_margin_events:
    :param pcd: point cloud
    :param timestamp: time you want
    :param time_interval: time interval before and after the time you want
    :param density_limit: point limitation of the time interval
    :return: if density is beyond density_limit, position numpy array [x, y, t]
             if not, return numpy array [0, 0, -1]
    """
    events = np.asarray(pcd.points)
    if not is_sorted(events[:, 2]):
        logging.error("Events are not sorted with timestamp!")
    begin_idx = np.searchsorted(events[:, 2], timestamp - time_interval)
    end_idx = np.searchsorted(events[:, 2], timestamp + time_interval)
    events_selected = events[begin_idx:end_idx]
    # logging.info("Num of events selected to average position: " + str(end_idx - begin_idx))
    # if margin events need to be removed
    if remove_margin_events:
        margin_mask = (events_selected[:, 0] < margin_value) | (events_selected[:, 0] > 346 - margin_value) | \
                      (events_selected[:, 1] < margin_value) | (events_selected[:, 1] > 260 - margin_value)
        margin_num = np.sum(margin_mask)

        if events_selected.shape[0] != 0 and margin_num / events_selected.shape[0] > 0.05:
            # logging.info("margin events")
            return np.array([0, 0, -1])

    if events_selected.shape[0] > density_limit and begin_idx != 0 and end_idx != events.shape[0]:
        return np.mean(events_selected, axis=0)
    else:
        return np.array([0, 0, -1])


def get_cross_matrix(lp_mat, rp_mat):
    """
    Get a matrix in solving Fundamental matrix linear equation
    :param lp_mat: left points np.array([u, v, 1])
    :param rp_mat: right points np.array([u, v, 1])
    :return: cross matrix
    """
    result = np.zeros(shape=[lp_mat.shape[0], 9])
    for idx in range(result.shape[0]):
        u1 = lp_mat[idx, 0]
        v1 = lp_mat[idx, 1]
        u2 = rp_mat[idx, 0]
        v2 = rp_mat[idx, 1]
        result[idx] = np.array([u2 * u1, u2 * v1, u2, v2 * u1, v2 * v1, v2, u1, v1, 1])
    return result


def solve_fundamental_matrix_svd(left_pts_array, right_pts_array):
    """
    Use SVD to compute fundamental matrix
    :param left_pts_array: left points np.array([u, v])
    :param right_pts_array: left points np.array([u, v])
    :return:
    """
    cross_mat = get_cross_matrix(left_pts_array, right_pts_array)
    # singular value decomposition of cross mat
    U, S, V = np.linalg.svd(cross_mat)
    # F is the singular vector of minimum singular value
    F_svd = V[-1].reshape(3, 3)
    # and F should follow det(F)=0, so SVD again
    U, S, V = np.linalg.svd(F_svd)
    S[2] = 0
    F_svd = np.dot(U, np.dot(np.diag(S), V))
    # normalization
    F_svd = F_svd / np.linalg.norm(F_svd)
    return F_svd


def show_distortion(K, rad_dist):
    # test undistortion
    full_one_events = np.zeros((260 * 346 * 10, 4), dtype=float)
    # full_one_events = np.zeros_like(events[:260 * 346])
    for row in range(260):
        for col in range(346):
            full_one_events[row * 346 + col] = np.array([col, row, 0, 1])

    img = np.zeros([260, 346])
    for e in full_one_events:
        y = int(e[1])
        x = int(e[0])
        if 0 <= x <= 345 and 0 <= y <= 259:
            img[y, x] = 1
    img[0, 0] = 0
    plt.imshow(img)
    plt.show()

    full_one_events_undist = cv2.undistortPoints(full_one_events[:, :2], K,
                                                 np.array([rad_dist[0, 0], rad_dist[0, 1], 0, 0]),
                                                 None, None, K)
    full_one_events_undist = np.squeeze(full_one_events_undist)
    full_one_events[:, :2] = full_one_events_undist

    img = np.zeros([260, 346])
    for e in full_one_events:
        y = round(e[1])
        x = round(e[0])
        if 0 <= x <= 345 and 0 <= y <= 259:
            img[y, x] = 1
    plt.imshow(img)
    plt.show()


def load_config(config_file):
    """
    load yaml config file
    :param config_file:
    :return:
    """
    with open(config_file) as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
        logging.info("config file: " + str(config))
    return config
