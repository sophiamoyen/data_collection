# Author: Sophie Lueth
# Date: July 2024

import os
import time
import datetime
import numpy as np
import cv2 as cv
import pickle
import shutil
import matplotlib
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
import pyzed.sl as sl

import rospy

from std_msgs.msg import String

from tsid_tiago.tsid_tiago_left_ee_elbow import TsidTiagoDualOmni
from tsid_tiago.conf import tsid_tiago_dual_omni_left_ee_elbow_conf as conf

from msc_tiago.teleop import Teleop
from msc_tiago.tiago_omni_compliant import TiagoOmni
from msc_tiago.zed_wrapper import ZedWrapper

GRIPPER_OPENING = 0.355
ZED_SERIAL_NUMBERS = [
                      28401408,
                      26658469
                      ]
HEAD_2_JOINT = -0.3
START_POSE = np.array([0.48, -0.25, 1.53, 1.43, 1.7, -0.80, -1.54])

np.set_printoptions(precision=3)


def answer_yes_no_question(question):
    """
    Args:
        question (str): question to be answered with yes or no
    Returns:
        bool
    """
    while True:
        ans = input(question + ' (y/n)')
        if ans == 'y':\
            return True
        elif ans == 'n':
            return False
        else:
            print('Please only answer with "y" for yes or "n" for no!')


def get_omni_q_pin(tiago_handler):
    torso, _ = tiago_handler.get_torso_state()
    arm_left, _ = tiago_handler.get_arm_left_state()
    gripper_left, _ = tiago_handler.get_gripper_left_state()
    arm_right, _ = tiago_handler.get_arm_right_state()
    gripper_right, _ = tiago_handler.get_gripper_right_state()
    head, _ = tiago_handler.get_head_state()

    q_pin = np.concatenate((np.array([0, 0, 1, 0]),
                            torso,
                            arm_left,
                            gripper_left,
                            np.array([0, 0, 0, 0, 0]),
                            arm_right,
                            np.array([0, 0, 0, 0, 0, 0]),
                            head))

    return q_pin


class RecordLeft:
    """
    Class for Structure for Data Collection
    """
    def __init__(self, path_to_logs, collect=True):
        """
        Args:
            path_to_logs (str): path to log folder. Subfolders with timestamp of each espisode will be created to save data of each episode
            collect (bool): if True, will plot this and all EE_trajs before
        """
        
        self._collect = collect
        self._path_to_logs = path_to_logs
        
        if not os.path.isdir(self._path_to_logs):
            os.mkdir(self._path_to_logs)
        
        # Initialisation
        # self.f_policy = conf.f_tsid/5
        self.f_policy = 15 # Hz
        self.dt_policy = 1/self.f_policy
        self._teleop = Teleop(dt=self.dt_policy, right_controller=False)
        self.tiago = TiagoOmni(side='left', gripper='robotiq')
        rospy.sleep(0.1)

        input ('Press Enter to move to the Start Position.')
        self.tiago.move_to_pose(START_POSE)

        q_ref = get_omni_q_pin(self.tiago)
        
        arm_left_pos, arm_left_vel = self.tiago.get_arm_left_state()
        base_pos, base_vel = self.tiago.get_base_state()
        q = np.concatenate((base_pos, arm_left_pos))
        self.tsid = TsidTiagoDualOmni(conf, q, q_ref=q_ref)  # Whole-Body Controller
        
        # self.zeds = [ZedWrapper(sn, fps=15) for sn in ZED_SERIAL_NUMBERS]
        # self.zedwrites = [None] * len(ZED_SERIAL_NUMBERS)
        self.zed_pub = rospy.Publisher('/zed_trigger', String, queue_size=10)
        
        self._past_trajs_ee = [] # contains old ee poses over time
        self._past_trajs_e = []  # contains old elbow trajectories over time

        self.setup_camera()
        
        # plotting utils
        self.cmap = matplotlib.colormaps.get_cmap('plasma')
        self.max_length = 0
    
    def setup_camera(self):
        for sn in ZED_SERIAL_NUMBERS:
            see_again = True
            while see_again:
                self.zed_pub.publish(String(data=f"save_rgb {sn} /home/hydra/sophie/whole_body_manipulation/logs/current_image.png"))
                # self.zed_pub.publish(String(data=f"save_rgb {sn} /home/sophie/code/whole_body_manipulation/logs/current_image.png"))
                see_again = answer_yes_no_question('Would you like to adjust and see the current camera state again?')

        # for zed in self.zeds:
        #     see_again = True
        #     while see_again:
        #         zed_img = zed.get_rgb()
        #         plt.imsave('/home/hydra/sophie/whole_body_manipulation/logs/current_image.png', zed_img)
        #
        #         see_again = answer_yes_no_question('Would you like to adjust and see the current camera state again?')
            
    def run(self):
        new_episode = answer_yes_no_question('Would you like to continue to a new episode (y) or quit (n)?')
        # if new_episode:
            # input('Press Enter to switch to impedance control mode.')
            # self.tiago.activate_arm_compliance(side='left') # activate joint impedance control

        while new_episode:
            datetime_str = datetime.datetime.now().strftime('%Y-%m-%d_%H:%M:%S')
            os.mkdir(self._path_to_logs + '/' + datetime_str)
            for sn in ZED_SERIAL_NUMBERS:
                self.zed_pub.publish(String(data=f"start_video {sn} {self._path_to_logs}/{datetime_str}/zed_{sn}_rgb.mp4"))
                # self.zed_pub.publish(String(data=f"start_depth_recording {sn} {self._path_to_logs}/{datetime_str}/zed_{sn}_depth.npy"))
                # self.zed_pub.publish(String(data=f"start_video {sn} /home/sophie/code/whole_body_manipulation/logs/{datetime_str}_zed_{sn}_rgb.mp4"))
                # self.zed_pub.publish(String(data=f"start_depth_recording {sn} /home/sophie/code/whole_body_manipulation/logs/{datetime_str}_zed_{sn}_depth.npy"))

            # self.zedwrites = [cv.VideoWriter(self._path_to_logs + '/' + datetime_str + f'/zed_{sn}_rgb.mp4', \
            #                                  cv.VideoWriter_fourcc(*'mp4v'), 20.0, (1280, 720)) for sn in ZED_SERIAL_NUMBERS] # (shape[1], shape[0])

            input('Press Enter to continue to teleoperation.')
            
            data, traj = self.teleop()

            print('Exporting ZED RGBs as video...')
            for sn in ZED_SERIAL_NUMBERS:
                self.zed_pub.publish(String(data=f"release_video {sn}"))
                # self.zed_pub.publish(String(data=f"release_depth_recording {sn}"))
            # for zedwrite in self.zedwrites:
            #     zedwrite.release()
            # self.zedwrites = [None] * len(ZED_SERIAL_NUMBERS)

            ## PLOT
            if self._collect:
                self.plot_ee_trajs(data, self._path_to_logs + '/' + datetime_str)
            else:
                self.plot_q_trajs(data, self._path_to_logs + '/' + datetime_str)
            
            if answer_yes_no_question('Would you like to save this episode?'):
                if self._collect:
                    self._past_trajs_ee.append(traj[:6])
                    self._past_trajs_e.append(traj[6:])
                
                if_collect = 'collect' if self._collect else 'tune'
                np.save(self._path_to_logs + '/' + datetime_str + '/' + f'data_{if_collect}.npy', data)
            else:
                shutil.rmtree(self._path_to_logs + '/' + datetime_str)
            
            new_episode = answer_yes_no_question('Would you like to continue to a new episode (y) or quit (n)?')

            if new_episode:
                input('Press Enter to move the Start position.')
                self.tiago.move_to_pose(START_POSE)

        input('Press Enter to move to Home Position')
        self.tiago.move_to_home()

    def teleop(self):
        # state
        rgbs_zed = []           # RGBs from ZED list of 3 of (720 x 720)
        depths_zed = []         # depths from ZED list of 3 of (1280x720)
        qs = []                 # joint positions \in R^9 
        dqs = []                # joint velocities \in R^9 
        ps_ee = []              # end-effector poses \in R^6
        dps_ee = []             # end-effector twist \in R^6
        ps_e = []               # elbow poses \in R^6
        dps_e = []              # elbow twist \in R^6
        # action (from teleop)
        ee_twists = []          # end-effector twist from teleop
        elbow_vels = []         # elbow velocity from teleop
        ms = []                 # modes
        gs = []                 # policy gripper opening
        # impedance controller tuning
        actual_qs = []
        actual_dqs = []
        # logs
        times = []
        
        arm_left_pos, arm_left_vel = self.tiago.get_arm_left_state()
        base_pos, base_vel = self.tiago.get_base_state()
        q = np.concatenate((base_pos, arm_left_pos))
        v = np.zeros(self.tsid.pin_wrapper.model.nv)
        
        start_time = time.time()
        start_wbc = start_time
        while True:
            # before = time.time()
            active, lin_vel, rot_vel, g, m, stop = self._teleop.get_update()
            # print(f'query teleop: {time.time() - before}')
            if stop:
                break
            
            if active:
                if m:   # Trigger has been pressed, now define elbow velocity
                    ee_twist_trans = np.zeros(3)
                    ee_twist_rot = np.zeros(3)
                    elbow_twist_trans = lin_vel
                else:
                    ee_twist_trans = lin_vel
                    ee_twist_rot = rot_vel
                    elbow_twist_trans = np.zeros(3)
                    self.tiago.stop_base()

                # send gripper command
                if g > GRIPPER_OPENING:
                    target_gripper_pos = GRIPPER_OPENING
                else:
                    target_gripper_pos = g

                diff = time.time() - start_wbc
                if diff >= 2*self.dt_policy:
                    print(f'Warning: Step time {diff * 1000:.3f} ms in Teleop control loop.')
                # else:
                #     print(f'Step time {diff * 1000:.3f} ms in Teleop control loop.')

                start_wbc = time.time()
                after_wbc = time.time()
                while after_wbc - start_wbc < self.dt_policy - 0.001:   # setting outer teleop/policy frequency
                    t1 = time.time()
                    q, v = self.tsid.compute_step(q, v, ee_twist_trans, ee_twist_rot, elbow_twist_trans, m=m)
                    self.tiago.send_arm_cmd('left', [q[3:]], [v[3:]], [rospy.Duration.from_sec(conf.dt)])
                    self.tiago.send_gripper_cmd('left', [[target_gripper_pos]], [[0]], times_from_start=[rospy.Duration.from_sec(conf.dt)])
                    now = time.time()
                    print(f'wbc part: {now - t1}')

                    if now - after_wbc >= conf.dt:
                        print(f'Warning: Step time {(now - after_wbc) * 1000:.3f} ms in WBC control loop.')
                    while time.time() - after_wbc < conf.dt:        # setting inner wbc frequency
                        time.sleep(conf.dt/100)

                    after_wbc = time.time()
                
                if m:
                    self.tiago.send_base_cmd(v[:2], v[2])
                    base_pos, _ = self.tiago.get_base_state()
                    # print(-base_pos[2])
                    # self.tiago.send_head_cmd([np.array([-base_pos[2], HEAD_2_JOINT])], times_from_start=[rospy.Duration.from_sec(self.dt_policy)])
                    
                before_append = time.time()
                # state
                # rgbs_t_zed = [zed.get_rgb() for zed in self.zeds]
                # depths_t_zed = [zed.get_depth() for zed in self.zeds]
                
                # rgbs_zed.append(rgbs_t_zed)
                # depths_zed.append(np.array(depths_t_zed))
                for sn in ZED_SERIAL_NUMBERS:
                    self.zed_pub.publish(String(data=f"grab_frame {sn}"))
                    # self.zed_pub.publish(String(data=f"grab_depth_frame {sn}"))

                print(f'zed images: {(time.time() - before_append)*1000:.3f} ms')
                qs.append(q) # taking the output of the WBC here
                dqs.append(v)
                pose_ee = self.tsid.pin_wrapper.forward_kinematics(q)
                ps_ee.append(np.concatenate((pose_ee.translation, Rotation.from_matrix(pose_ee.rotation).as_euler('xyz'))))
                dps_ee.append(self.tsid.pin_wrapper.jacobian(q) @ v)
                pose_elbow = self.tsid.pin_wrapper.forward_kinematics(q, frame=conf.elbow_frame_name)
                ps_e.append(np.concatenate((pose_elbow.translation, Rotation.from_matrix(pose_elbow.rotation).as_euler('xyz'))))
                dps_e.append(self.tsid.pin_wrapper.jacobian(q, frame='arm_left_4_joint') @ v)
                
                # action
                ee_twists.append(np.concatenate((ee_twist_trans, ee_twist_rot)))
                elbow_vels.append(elbow_twist_trans)
                ms.append(m)
                # print(f'g: {g}')
                
                # impedance controller tuning
                actual_q, actual_dq = self.tiago.get_arm_left_state()
                
                actual_qs.append(actual_q)
                actual_dqs.append(actual_dq)

                times.append(after_wbc - start_time)

                self.max_length = max(self.max_length, len(times))

            else:
                self.tiago.stop_base()
        
        # print('Exporting ZED RGBs as video...')
        # for rgbs_t_zed in rgbs_zed:
        #     for zedwrite, rgb in zip(self.zedwrites, rgbs_t_zed):
        #         # zedwrite.write(rgb)
        #         breakpoint()
        #         zedwrite.write(cv.cvtColor(rgb, cv.COLOR_RGB2BGR))
                
        print('Done')
        if self._collect:
            state = {
                     # 'depths_zed': np.array(depths_zed),              # depths from ZED
                     'qs': np.array(qs),                              # joint positions \in R^9 
                     'dqs': np.array(dqs),                            # joint velocities \in R^9 
                     'ps_ee': np.array(ps_ee),                        # end-effector poses \in R^6
                     'dps_ee': np.array(dps_ee),                      # end-effector twist \in R^6
                     'ps_e': np.array(ps_e),                          # elbow poses \in R^6
                     'dps_e': np.array(dps_e)}                        # elbow twist \in R^6}
            action = {'ee_twists': np.array(ee_twists),               # end-effector twist from teleop
                      'elbow_vels': np.array(elbow_vels),             # elbow velocity from teleop
                      'ms': np.array(ms),                             # modes
                      'gs': np.array(gs)}                             # policy gripper opening,
            data = {'state': state,
                    'action': action,
                    'times': np.array(times)}
            
            traj = np.array(np.concatenate((ps_ee, ps_e)))
                        
        else:
            # impedance controller tuning
            actual_ps_ee = []
            for actual_q, q in zip(actual_qs, qs):
                actual_pose_ee = self.tsid.forward_kinematics(np.concatenate((q[:2], actual_q))) # disregard error from the base
                actual_ps_ee.append(np.concatenate((actual_pose_ee.translation, Rotation.from_matrix(actual_pose_ee.rotation).as_euler('xyz'))))
            data = {'qs': np.array(qs)[:, 2:],                        # desired joint positions \in R^7
                    'dqs': np.array(dqs)[:, 2:],                      # desired joint velocities \in R^7
                    'ps_ee': np.array(ps_ee),                         # desired end-effector poses \in R^6
                    'actual_qs': np.array(actual_qs),                 # actual joint positions \in R^7
                    'actual_dqs': np.array(actual_dqs),                # actual joint velocities \in R^7
                    'actual_ps_ee': np.array(actual_ps_ee),           # actual end-effector poses in \in R^6
                    'times': np.array(times)}
                
            traj = np.linalg.norm(np.array(ps_ee)[:, :3] - np.array(actual_ps_ee)[:, :3], axis=1)

        print(f"The recording should have length {len(qs) / 15.0} s")

        return data, traj
    
    def plot_ee_trajs(self, data, path_to_logs):
        ## PLOT TASK SPACE ERROR: EE
        poss_3d_des = data['state']['ps_ee']
        
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
    
        # plot most recent EE trajectory
        xyz = poss_3d_des[:, :3]
        uvw = np.array([Rotation.from_euler('xyz', pose[3:]).as_matrix()[:3,2] for pose in poss_3d_des])

        caxis = np.arange(len(xyz)) / self.max_length
        colors = self.cmap(caxis, 1)
        ax.quiver(xyz[:,0], xyz[:,1], xyz[:,2], uvw[:,0], uvw[:,1], uvw[:,2], length=0.01, color=colors)
        
        for traj in self._past_trajs_ee:
            xyz = traj[:,:3]
            uvw = np.array([Rotation.from_euler('xyz', pose[3:]).as_matrix()[:3,2] for pose in traj])

            caxis = np.arange(len(xyz)) / self.max_length
            colors = self.cmap(caxis, 0.4)
            
            ax.quiver(xyz[:,0], xyz[:,1], xyz[:,2], uvw[:,0], uvw[:,1], uvw[:,2], length=0.01, color=colors)
            
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        ax.set_title('Endeffector Trajectories')
        ax.grid()
        
        plt.savefig(path_to_logs + '/ee_collection.png')
        # load like this:
        # fig = pickle.load(open('/scr/sclueth/logs/ee_data.pkl','rb')); fig.show()
        pickle.dump(fig, open(path_to_logs + '/ee_collection.pkl', 'wb'))
        
        ## PLOT TASK SPACE ERROR: ELBOW
        poss_3d_des = data['state']['ps_e']
            
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
    
        # plot most recent EE trajectory
        xyz = poss_3d_des[:, :3]
        uvw = np.array([Rotation.from_euler('xyz', pose[3:]).as_matrix()[:3,2] for pose in poss_3d_des])
    
        caxis = np.arange(len(xyz)) / self.max_length
        colors = self.cmap(caxis, 1)
        ax.quiver(xyz[:,0], xyz[:,1], xyz[:,2], uvw[:,0], uvw[:,1], uvw[:,2], length=0.01, color=colors)
        
        for traj in self._past_trajs_e:
            xyz = traj[:,:3]
            uvw = np.array([Rotation.from_euler('xyz', pose[3:]).as_matrix()[:3,2] for pose in traj])
        
            caxis = np.arange(len(xyz)) / self.max_length
            colors = self.cmap(caxis, 0.4)
            
            ax.quiver(xyz[:,0], xyz[:,1], xyz[:,2], uvw[:,0], uvw[:,1], uvw[:,2], length=0.01, color=colors)
        
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        ax.set_title('Endeffector Trajectories')
        ax.grid()
        
        plt.savefig(path_to_logs+ '/elbow_collection.png')
        # load like this (adjust)
        # fig = pickle.load(open('/scr/sclueth/logs/ee_data.pkl','rb')); fig.show()
        pickle.dump(fig, open(path_to_logs +'/elbow_collection.pkl', 'wb'))
    
    def plot_q_trajs(self, data, path_to_logs):
        """
        Plot relevant information for tuning impedance parameters
        Args:
            data (dict): for structure see teleop method, if not self._collect
            path_to_logs (str): path to folder, already includes handle to this episdoe
        """
        ## PLOT JOINT SPACE ERROR
        pos_des = data['qs']
        pos = data['actual_qs']
        pos_dot_des = data['dqs']
        pos_dot = data['actual_dqs']
        time = data['times']
        
        fig, axs = plt.subplots(2, 7, figsize=(20, 7))
        fig.suptitle('accuracy of this episode')
        
        for i, (pos_i_des, pos_i) in enumerate(zip(pos_des.T, pos.T)):
            axs[0,i].plot(time, pos_i_des, label=f'des')
            axs[0,i].plot(time, pos_i, label=f'act')
        
            axs[0,i].set_title(f'Pos Joint {i}')
            axs[0,i].set_xlabel('time (s)')
            axs[0,i].set_ylabel('angle (rad)')
            axs[0,i].grid()
            axs[0,i].legend()
        
        for i, (pos_i_dot_des, pos_dot_i) in enumerate(zip(pos_dot_des.T, pos_dot.T)):
            axs[1,i].plot(time, pos_i_dot_des, label=f'des')
            axs[1,i].plot(time, pos_dot_i, label=f'act')
    
            axs[1,i].set_title(f'Vel Joint {i}')
            axs[1,i].set_xlabel('time (s)')
            axs[1,i].set_ylabel('angle vel (rad/s)')
            axs[1,i].grid()
            axs[1,i].legend()
        
        plt.savefig(path_to_logs + '/joint_accuracy.png')
        
        ## PLOT TASK SPACE ERROR
        poss_3d_des = data['ps_ee']
        poss_3d = data['actual_ps_ee']
        e_3d = np.linalg.norm(np.array([pos_3d_des[:3] - pos_3d[:3] for pos_3d_des, pos_3d in zip(poss_3d_des, poss_3d)]), axis=1)
        
        xyz_des = poss_3d_des[::10, :3]
        uvw_des = np.array([Rotation.from_euler('xyz', pose[3:]).as_matrix()[:3,2] for pose in poss_3d_des[::10]])
        xyz = poss_3d[::10, :3]
        uvw = np.array([Rotation.from_euler('xyz', pose[3:]).as_matrix()[:3,2] for pose in poss_3d[::10]])
        
        fig = plt.figure()
        ax = fig.add_subplot(121, projection='3d')
        ax.quiver(xyz_des[:,0], xyz_des[:,1], xyz_des[:,2], uvw_des[:,0], uvw_des[:,1], uvw_des[:,2], length=0.01, color='tab:blue', label='des')
        ax.quiver(xyz[:,0], xyz[:,1], xyz[:,2], uvw[:,0], uvw[:,1], uvw[:,2], length=0.01, color='tab:orange', label='act')
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        ax.set_title('Endeffector Trajectories')
        ax.grid()
        ax.legend()
        
        ax = fig.add_subplot(122)
        ax.plot(time, e_3d)
        ax.set_title('Translational Endeffector Error')
        ax.set_xlabel('time (s)')
        ax.set_ylabel('Error (m)')
        ax.grid()
        
        plt.savefig(path_to_logs + '/ee_accuracy.png')
        pickle.dump(fig, open(path_to_logs + '/ee_accuracy.pkl', 'wb'))
    
    def close(self):
        pass
        # for zed in self.zeds:
            # zed.close()
        
        # for zedwrite in self.zedwrites:
        #     if zedwrite is not None:
        #         zedwrite.release()
            
        # self.tiago.deactivate_arm_compliance(side='left')
        