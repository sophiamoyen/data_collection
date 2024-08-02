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
ZED_SERIAL_NUMBERS = [cam.serial_number for cam in sl.Camera.get_device_list()]
HEAD_2_JOINT = -0.3
START_POSE = np.array([0.48, -0.25, 1.53, 1.43, 1.7, -0.80, -1.54])

np.set_printoptions(precision=3)


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
    def __init__(self, path_to_logs):
        """
        Args:
            path_to_logs (str): path to log folder. Subfolders with timestamp of each espisode will be created to save data of each episode
        """
        
        self._path_to_logs = path_to_logs
        
        if not os.path.isdir(self._path_to_logs):
            os.mkdir(self._path_to_logs)
        
        # Initialisation
        self.f_policy = 15 # Hz
        self.dt_policy = 1/self.f_policy
        self._teleop = Teleop(dt=self.dt_policy, right_controller=False)
        self.tiago = TiagoOmni(side='left', gripper='robotiq')
        rospy.sleep(0.1)

        self.zed_pub = rospy.Publisher('/zed_trigger', String, queue_size=10)
        self.setup_camera()
        
        # plotting utils
        self.cmap = matplotlib.colormaps.get_cmap('plasma')
        self.max_length = 0

    def move_start_pose(self):
        self.tiago.move_to_pose(START_POSE)
        self.q_ref = get_omni_q_pin(self.tiago)

    
    def save_params_wbc(self):
        arm_left_pos, arm_left_vel = self.tiago.get_arm_left_state()
        base_pos, base_vel = self.tiago.get_base_state()
        q = np.concatenate((base_pos, arm_left_pos))
        self.tsid = TsidTiagoDualOmni(conf, q, q_ref=self.q_ref)  # Whole-Body Controller

    def save_old_ee_poses(self):
        self._past_trajs_ee = [] # contains old ee poses over time
        self._past_trajs_e = []  # contains old elbow trajectories over time
    
    def setup_camera(self):
        i = 1
        for sn in ZED_SERIAL_NUMBERS:
            self.zed_pub.publish(String(data=f"save_rgb {sn} /home/hydra/sophie/whole_body_manipulation/logs/current_image_{i}.png"))
            i += 1 # Update name of png file

    def run_teleop(self):
        self.data, self.traj = self.teleop()

    def start_recording(self):
        self.datetime_str = datetime.datetime.now().strftime('%Y-%m-%d_%H:%M:%S')
        os.mkdir(self._path_to_logs + '/' + self.datetime_str)
        for sn in ZED_SERIAL_NUMBERS:
            self.zed_pub.publish(String(data=f"start_video {sn} {self._path_to_logs}/{datetime_str}/zed_{sn}_rgb.mp4"))

    def stop_recording(self):
        print('Exporting ZED RGBs as video...')
        for sn in ZED_SERIAL_NUMBERS:
            self.zed_pub.publish(String(data=f"release_video {sn}"))

        self.plot_ee_trajs(self.data, self._path_to_logs + '/' + self.datetime_str)

    def save_demo(self):
        self._past_trajs_ee.append(self.traj[:6])
        self._past_trajs_e.append(self.traj[6:])
        np.save(self._path_to_logs + '/' + self.datetime_str + '/' + f'data_collect.npy', self.data)

    def delete_demo(self):
        shutil.rmtree(self._path_to_logs + '/' + self.datetime_str)

    def got_to_home_pose(self):
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

                    
                before_append = time.time()

                for sn in ZED_SERIAL_NUMBERS:
                    self.zed_pub.publish(String(data=f"grab_frame {sn}"))


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
        

                
        print('Done')
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
    
    def close(self):
        pass

        
