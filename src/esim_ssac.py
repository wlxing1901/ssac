from fx_vis import *
from fx_load import *
from fx_utilities import *
import matplotlib.pyplot as plt
import numpy as np
import cv2
import logging
import scipy.io
import copy
import matlab.engine
from scipy.spatial.transform import Rotation

import matplotlib

matplotlib.use('TkAgg')
matplotlib.rcParams['font.family'] = 'Times New Roman'
matplotlib.rcParams['font.size'] = 20  # You can adjust this value to make the font size bigger or smaller

eng = matlab.engine.start_matlab()
logging.basicConfig(level=logging.INFO)


def esim_single_dvs_process(filename, begin_time, duration, filter_points, filter_radius, pos_time_interval, K,
                            rad_dist):
    """
    Process to aedat4 file
    :param filename: filename
    :param begin_time:
    :param duration:
    :param filter_points:
    :param filter_radius:
    :param pos_time_interval:
    :param K: camera intrinsic matrix
    :param rad_dist: distortion params
    :return:
    """
    # load events left
    events = read_events_from_txt(filename, skip_events=1)

    # # undistortion
    # events_undist = cv2.undistortPoints(events[:, :2], K, np.array([rad_dist[0, 0], rad_dist[0, 1], 0, 0]),
    #                                     None, None, K)
    # events_undist = np.squeeze(events_undist)
    # events[:, :2] = events_undist

    # select events in time slot
    begin_idx = np.searchsorted(events[:, 2], begin_time)
    end_idx = np.searchsorted(events[:, 2], begin_time + duration)
    logging.info("Index selection: " + str(begin_idx) + " to " + str(end_idx))
    events_pcd = events_array_to_pcd(events[begin_idx:end_idx],
                                     t_factor=t_factor,
                                     t_from_zero=True)
    draw_pcd_open3d(events_pcd, img_shape[0], img_shape[1])

    # remove noise left
    logging.info("Radius oulier removal")
    # cl, ind = events_pcd.remove_radius_outlier(nb_points=filter_points, radius=filter_radius)
    pcd_ball = events_pcd
    logging.info("Num of remained events: " + str(np.asarray(pcd_ball.points).shape[0]))
    draw_pcd_open3d(pcd_ball, img_shape[0], img_shape[1])

    # calculate trajectory
    ball_traj = np.zeros(shape=[traj_num, 3])
    for idx, t in enumerate(np.linspace(0.0, duration * t_factor, traj_num)):
        average_pos = average_position_density(pcd_ball, t, pos_time_interval * t_factor, density_limit=density_limit)
        if average_pos[-1] != -1:
            ball_traj[idx, :2] = average_pos[:2]
            ball_traj[idx, 2] = t

    ball_traj_pcd = o3d.geometry.PointCloud()
    ball_traj_pcd.points = o3d.utility.Vector3dVector(ball_traj)
    draw_pcd_open3d(ball_traj_pcd, img_shape[0], img_shape[1])

    return pcd_ball




def main():
    logging.info("Begin to process esim data")
    # Step: load fundamental matrix
    if (not import_params_from_yaml):
        stereo_params = scipy.io.loadmat(stereo_params_dir)
        F_calib = stereo_params["F"]
        E_calib = stereo_params["E"]
        R_calib = stereo_params["R"]
        t_calib = stereo_params["t"]
        t_calib = np.squeeze(t_calib)
        K1 = stereo_params["K1"]
        K2 = stereo_params["K2"]

        rad_dist1 = stereo_params["radial_distortion1"]
        rad_dist2 = stereo_params["radial_distortion2"]
    else:
        K1 = np.array(config["K1"])
        K2 = np.array(config["K2"])
        rad_dist1 = np.array([[0, 0]])
        rad_dist2 = np.array([[0, 0]])

        E_calib = np.array(config["E"])
        R_calib = np.array(config["R"])
        t_calib = np.array(config["t"])
        t_calib = np.squeeze(t_calib)
        F_calib = np.linalg.inv(K1).transpose().dot(E_calib).dot(np.linalg.inv(K2))

    left_pcd_ball = esim_single_dvs_process(left_data_dir,
                                            left_begin_time,
                                            left_duration,
                                            nb_points,
                                            radius,
                                            traj_time_interval,
                                            K1,
                                            rad_dist1)
    right_pcd_ball = esim_single_dvs_process(right_data_dir,
                                             right_begin_time,
                                             right_duration,
                                             nb_points,
                                             radius,
                                             traj_time_interval,
                                             K2,
                                             rad_dist2)

    # Step: search the best time diff
    if find_best_time == 1:
        # calculate time difference
        time_diff_list = []
        avg_dist_list = []
        avg_dist_list_calib = []
        for time_diff in np.linspace(time_search_begin, time_search_end, time_search_num):
            logging.info("*************************")
            logging.info("time offset t_d: %.3f", time_diff)

            time_diff_list.append(time_diff)
            dist_sum = 0
            dist_sum_calib = 0
            dist_fail_num = 0
            left_dist_fail_num = 0
            right_dist_fail_num = 0

            # compute F_ball for each time_diff
            left_array = []
            right_array = []
            for idx, t in enumerate(np.linspace(0.0, right_duration * t_factor, dist_cnt_num)):
                right_pos = average_position_density(right_pcd_ball,
                                                     t,
                                                     traj_time_interval * t_factor,
                                                     density_limit=density_limit)
                right_pt = right_pos[:2]

                left_pos = average_position_density(left_pcd_ball,
                                                    t + time_diff * t_factor,
                                                    traj_time_interval * t_factor,
                                                    density_limit=density_limit)
                left_pt = left_pos[:2]

                if left_pos[-1] != -1 and right_pos[-1] != -1:
                    left_array.append(left_pt)
                    right_array.append(right_pt)

            left_array = np.array(left_array)
            right_array = np.array(right_array)

            # call matlab
            left_matlab = matlab.double(left_array.tolist())
            right_matlab = matlab.double(right_array.tolist())
            fLMedS = eng.estimateFundamentalMatrix(left_matlab, right_matlab, 'NumTrials', 2000)
            fLMedS = np.asarray(fLMedS)
            F_ball = fLMedS

            for idx, t in enumerate(np.linspace(0.0, right_duration * t_factor, dist_cnt_num)):
                right_pos = average_position_density(right_pcd_ball,
                                                     t,
                                                     traj_time_interval * t_factor,
                                                     density_limit=density_limit)
                right_pt = right_pos[:2]

                lines_left = cv2.computeCorrespondEpilines(right_pt.reshape(-1, 1, 2), 2, F_ball)
                lines_left = lines_left.reshape(-1, 3)
                lines_left_calib = cv2.computeCorrespondEpilines(right_pt.reshape(-1, 1, 2), 2, F_calib)
                lines_left_calib = lines_left_calib.reshape(-1, 3)

                left_pos = average_position_density(left_pcd_ball,
                                                    t + time_diff * t_factor,
                                                    traj_time_interval * t_factor,
                                                    density_limit=density_limit)
                left_pt = left_pos[:2]

                if left_pos[2] == -1:
                    left_dist_fail_num += 1
                if right_pos[2] == -1:
                    right_dist_fail_num += 1

                # when event density is not enough
                if left_pos[2] == -1 or right_pos[2] == -1:
                    dist_fail_num += 1
                    continue

                dist_sum += point_line_distance(lines_left[0], left_pt[:2])
                dist_sum_calib += point_line_distance(lines_left_calib[0], left_pt[:2])

            logging.info("dist fail num: " + str(dist_fail_num))
            logging.info("left dist fail num: " + str(left_dist_fail_num))
            logging.info("right dist fail num: " + str(right_dist_fail_num))
            avg_dist_list.append(dist_sum / (dist_cnt_num - dist_fail_num))
            avg_dist_list_calib.append(dist_sum_calib / (dist_cnt_num - dist_fail_num))
            logging.info("d_avg estimated params: %.3f", dist_sum / (dist_cnt_num - dist_fail_num))
            logging.info("d_avg calibrated params: %.3f", dist_sum_calib / (dist_cnt_num - dist_fail_num))

        # fig, ax = plt.subplots()
        # fig.set_size_inches(14, 6)
        # ax.plot(time_diff_list, avg_dist_list)
        # ax.set_title("Average distance d_avg")
        # ax.set_xlabel("time offset t_d (s)")
        # ax.set_ylabel("distance d_avg (pixel)")
        # plt.savefig(output_dir + "fundamental_distance.png", dpi=500)
        # plt.show()
        numpy.savetxt(output_dir + "fundamental_distance.csv", np.vstack([time_diff_list, avg_dist_list]),
                      delimiter=",")

        fig, ax = plt.subplots()
        fig.set_size_inches(14, 6)
        ax.plot(time_diff_list, avg_dist_list, label="d_avg estimated F matrix")
        ax.plot(time_diff_list, avg_dist_list_calib, label="d_avg calibrated F matrix")
        ax.set_title("Average distance")
        ax.set_xlabel("time offset t_d (s)")
        ax.set_ylabel("distance d_avg (pixel)")
        plt.legend()
        plt.savefig(output_dir + "fundamental_distance.png", dpi=500)
        plt.show()
        numpy.savetxt(output_dir + "fundamental_distance_calib_params.csv",
                      np.vstack([time_diff_list, avg_dist_list_calib]), delimiter=",")

        best_time_diff = np.min(time_diff_list[np.argmin(avg_dist_list)])
        logging.info("optimal t_d: %.3f", best_time_diff)
        logging.info("minimum d_avg: %.3f", avg_dist_list[np.argmin(avg_dist_list)])

    else:
        # Step: just for test, use the real time diff
        best_time_diff = real_time_diff

    logging.info("*************************")
    logging.info("optimal t_d: %.3f", best_time_diff)

    # Step: compute fundamental matrix at best time diff
    left_array = []
    right_array = []
    for idx, t in enumerate(np.linspace(0.0, right_duration * t_factor, dist_cnt_num)):
        right_pos = average_position_density(right_pcd_ball,
                                             t,
                                             traj_time_interval * t_factor,
                                             density_limit=density_limit)
        right_pt = right_pos[:2]

        left_pos = average_position_density(left_pcd_ball,
                                            t + best_time_diff * t_factor,
                                            traj_time_interval * t_factor,
                                            density_limit=density_limit)
        left_pt = left_pos[:2]

        if left_pos[-1] != -1 and right_pos[-1] != -1:
            left_array.append(left_pt)
            right_array.append(right_pt)

    left_array = np.array(left_array)
    right_array = np.array(right_array)

    # save matched points
    scipy.io.savemat(output_dir + "matched_points.mat", {"left_array": left_array, "right_array": right_array})

    # call matlab function to compute F matrix from ball trajectory
    left_matlab = matlab.double(left_array.tolist())
    right_matlab = matlab.double(right_array.tolist())

    # LMedS
    fLMedS = eng.estimateFundamentalMatrix(left_matlab, right_matlab, 'NumTrials', 12000)
    fLMedS = np.asarray(fLMedS)

    # # Note: RANSAC
    # fRANSAC = eng.estimateFundamentalMatrix(left_matlab, right_matlab, 'Method', 'RANSAC', 'NumTrials', 100000,
    #                                         'DistanceThreshold', 1e-2)
    # fRANSAC = np.asarray(fRANSAC)

    # # Note: SVD
    # F_svd = solve_fundamental_matrix_svd(left_array, right_array)

    F_ball = fLMedS

    logging.info("F_ball: \n" + str(F_ball / np.linalg.norm(F_ball)))
    # logging.info("F_svd: \n" + str(F_svd / np.linalg.norm(F_svd)))
    logging.info("F_calib: \n" + str(F_calib / np.linalg.norm(F_calib)))

    # # Note: test opencv fundamental matrix
    # F_opencv_lmeds, mask = cv2.findFundamentalMat(left_array, right_array, cv2.LMEDS, 0.99)
    # F_ball = F_opencv_lmeds

    # # Note: test F svd fundamental matrix
    # F_ball = F_svd

    # Step: decompose matrix F to get R and t and visualize them
    # normalized pose
    left_array_normalized = cv2.undistortPoints(left_array, K1, distCoeffs=None, dst=None, R=None, P=None)
    right_array_normalized = cv2.undistortPoints(right_array, K2, distCoeffs=None, dst=None, R=None, P=None)

    # get Essential matrix and decompose it to R and t
    E = K2.transpose().dot(F_ball).dot(K1)

    # logging.info("E computed: " + E)
    # logging.info("E calibrated: " + E_calib)

    _, R, t, mask = cv2.recoverPose(E, right_array_normalized, left_array_normalized, focal=1.0, pp=[0.0, 0.0])

    # _, R, t, mask = cv2.recoverPose(E, left_array_normalized, right_array_normalized, focal=1.0, pp=[0.0, 0.0])
    t = np.squeeze(t)
    t_skew = np.array([[0, -t[2], t[1]],
                       [t[2], 0, -t[0]],
                       [-t[1], t[0], 0]])

    # calculate the angle between R_calib with R
    r_error = Rotation.from_matrix(np.linalg.inv(R).dot(R_calib))
    r_error_angle = np.linalg.norm(r_error.as_rotvec()) / np.pi * 180
    logging.info("The angle between R and R_calib: %.3f degrees", r_error_angle)

    # calculate the angle between t_calib with t
    t_normalized = t / np.linalg.norm(t)
    t_calib_normalized = t_calib / np.linalg.norm(t_calib)
    t_error_angle = np.arccos(np.dot(t_normalized, t_calib_normalized)) / np.pi * 180
    logging.info("Normalized t: " + str(t_normalized))
    logging.info("Normalized t calibration: " + str(t_calib_normalized))
    logging.info("The angle between t and t_calib: %.3f degrees", t_error_angle)

    # visualize R_calib with R, t_calib with t
    # draw_transformations_open3d(R, R_calib, t, t_calib)

    # Step: draw images
    # draw lines for each Fundamental matrix
    for idx, t in enumerate(np.linspace(0.0, right_duration * t_factor, img_num)):
        right_pos = average_position_density(right_pcd_ball,
                                             t,
                                             traj_time_interval * t_factor,
                                             density_limit=density_limit)
        right_pt = right_pos[:2]

        left_pos = average_position_density(left_pcd_ball,
                                            t + best_time_diff * t_factor,
                                            traj_time_interval * t_factor,
                                            density_limit=density_limit)
        left_pt = left_pos[:2]

        if left_pos[2] == -1 or right_pos[2] == -1:
            right_eimg = get_event_image(right_pcd_ball,
                                         t,
                                         traj_time_interval * t_factor,
                                         img_shape)
            left_eimg = get_event_image(left_pcd_ball,
                                        t + best_time_diff * t_factor,
                                        traj_time_interval * t_factor,
                                        img_shape)

            fig, ax = plt.subplots(1, 2)
            fig.set_size_inches(14, 6)
            ax[0].imshow(left_eimg)
            ax[0].plot([], [], label="estimated F matrix ")
            ax[0].plot([], [], "-.", label="ground-truth F matrix")
            ax[0].scatter([], [], c='lime', edgecolor="darkgreen", marker='o',
                          label="left events center")
            ax[0].set_title("Left event image")
            ax[0].legend(loc='upper right')
            ax[1].imshow(right_eimg)
            ax[1].scatter([], [], c='lime', edgecolor="darkgreen", marker='o',
                          label="right events center")
            ax[1].set_title("Right event image")

            ax[1].legend(loc='upper right')
            ax[0].set_ylabel('y (pixel)')
            ax[0].set_xlabel('x (pixel)')
            ax[1].set_xlabel('x (pixel)')
            ax[1].set_ylabel('y (pixel)')

            plt.tight_layout()
            plt.savefig(output_dir + "left_right_" + str(idx) + ".png", dpi=300)
            plt.show()
            continue

        lines_left = cv2.computeCorrespondEpilines(right_pt.reshape(-1, 1, 2), 2, F_ball)
        lines_left_calib = cv2.computeCorrespondEpilines(right_pt.reshape(-1, 1, 2), 2, F_calib)

        lines_left = lines_left.reshape(-1, 3)
        xs, ys = get_line_mpl(lines_left[0], img_shape)

        lines_left_calib = lines_left_calib.reshape(-1, 3)
        xs_calib, ys_calib = get_line_mpl(lines_left_calib[0], img_shape)

        right_eimg = get_event_image(right_pcd_ball,
                                     t,
                                     traj_time_interval * t_factor,
                                     img_shape)
        left_eimg = get_event_image(left_pcd_ball,
                                    t + best_time_diff * t_factor,
                                    traj_time_interval * t_factor,
                                    img_shape)
        fig, ax = plt.subplots(1, 2)
        fig.set_size_inches(14, 6)
        ax[0].imshow(left_eimg)
        ax[0].plot(xs, ys, label="estimated F matrix ")
        ax[0].plot(xs_calib, ys_calib, "-.", label="ground-truth F matrix")
        ax[0].scatter(left_pt[0], left_pt[1], c='lime', edgecolor="darkgreen", marker='o', label="left events center")
        ax[0].set_title("Left event image")
        ax[0].legend(loc='upper right')
        ax[1].imshow(right_eimg)
        ax[1].scatter(right_pt[0], right_pt[1], c='lime', edgecolor="darkgreen", marker='o',
                      label="right events center")
        ax[1].set_title("Right event image")

        ax[1].legend(loc='upper right')
        ax[0].set_ylabel('y (pixel)')
        ax[0].set_xlabel('x (pixel)')
        ax[1].set_xlabel('x (pixel)')
        ax[1].set_ylabel('y (pixel)')

        plt.tight_layout()
        plt.savefig(output_dir + "left_right_" + str(idx) + ".png", dpi=300)
        plt.show()


if __name__ == "__main__":
    # load parameters
    config_file = "/home/wlxing/Codes/ssac_github/config/test/test.yaml"
    config = load_config(config_file)

    t_factor = config["t_factor"]
    left_begin_time = config["left_begin_time"]
    left_duration = config["left_duration"]
    real_time_diff = config["real_time_diff"]
    right_begin_time = left_begin_time + real_time_diff
    right_duration = config["right_duration"]
    traj_num = config["traj_num"]
    traj_time_interval = config["traj_time_interval"]
    img_shape = config["img_shape"]
    nb_points = config["nb_points"]
    radius = config["radius"]
    time_search_num = config["time_search_num"]
    dist_cnt_num = config["dist_cnt_num"]
    density_limit = config["density_limit"]
    left_data_dir = config["left_data_dir"]
    right_data_dir = config["right_data_dir"]
    stereo_params_dir = config["stereo_params_dir"]
    output_dir = config["output_dir"]
    find_best_time = config["find_best_time"]
    time_search_begin = config["time_search_begin"]
    time_search_end = config["time_search_end"]
    img_num = config["img_num"]
    import_params_from_yaml = config["import_params_from_yaml"]
    main()
