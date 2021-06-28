#!/usr/bin/env python3


import yaml

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from ament_index_python import get_package_share_directory
from tf2_ros import transform_broadcaster
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Transform
from std_msgs.msg import Int32

import xacro
from urdf_parser_py.urdf import URDF

import numpy as np
from transforms3d.euler import euler2mat
from transforms3d.quaternions import mat2quat
from transforms3d.axangles import axangle2mat, mat2axangle

from marine_presenter.srv import ButtonPress

import os
import sys
from copy import deepcopy

config_file = sys.argv[1]
if not config_file.endswith('.yaml'):
    print('Pass a configuration file')
    sys.exit(0)
    
with open(config_file) as f:
    config = yaml.safe_load(f)
    
def HomogeneousFrom(t, R):
    return np.matrix(np.vstack((np.hstack((R, t)), [0,0,0,1])))
    
def Homogeneous(pose):
    R = euler2mat(pose[3], pose[4], pose[5], 'rxyz')
    t = np.array(pose[:3]).reshape(3,1)
    M = HomogeneousFrom(t, R)
    for i in range(3):
        for j in range(3):
            for v in [-1,0,1]:
                if abs(M[i,j]-v) < 1e-3:
                    M[i,j] = v
    return M

def HomogeneousInverse(M):
    Minv = M.copy()
    Minv[:3,:3] = M[:3,:3].T
    Minv[:3,[3]] = -Minv[:3,:3]*M[:3,[3]]
    return Minv
    
class Camera:
    def __init__(self, M,gain):
        self.M = M.copy()
        self.gain = gain
        
    def move(self, Md, smooth = True):
        
        straight = False
        
        if not smooth or np.allclose(self.M, Md, atol=1e-3):
            self.M = Md.copy()
            return self.M
        
        if straight:
            err = HomogeneousInverse(self.M)*Md
        else:
            err = HomogeneousInverse(Md)*self.M
        u,theta = mat2axangle(err[:3,:3])
        err = HomogeneousFrom(self.gain*err[:3,[3]], axangle2mat(u,self.gain*theta))
        if straight:
            self.M *= err
        else:
            self.M *= HomogeneousInverse(err)
        return self.M
        
slides = config['slides']
if 'gain' not in config:
    config['gain'] = .1

def mat2tf(M, tr):
    tr.translation.x = M[0,3]
    tr.translation.y = M[1,3]
    tr.translation.z = M[2,3]
    q = mat2quat(M[:3,:3])
    tr.rotation.x = q[1]
    tr.rotation.y = q[2]
    tr.rotation.z = q[3]
    tr.rotation.w = q[0]
        
        
class Button:
    def __init__(self, node):
        self.pressed = None
        self.srv = node.create_service(ButtonPress, 'remote', self.button_cb)
        self.sub = node.create_subscription(Int32, "go_to", self.slide_cb, 1)
        self.slide = -1
        self.slide_prev = -1
        
    def button_cb(self, req, res):
        self.pressed = req.button
        self.slide = -1
        res = ButtonPress.Response()
        return res
    
    def slide_cb(self, msg):
        des_slide = msg.data-1
        
        if self.slide_prev == -1:
            self.slide_prev = des_slide
        
        if des_slide != self.slide_prev:   
            self.slide = des_slide
            self.slide_prev = des_slide
    
    def reset(self):
        self.pressed = None
        
poses = {}

class MovingObject:
    def __init__(self, name, params, node):
        
        self.has_odom = 'rx' in params
        self.has_joints = 'joints' in params
               
        # load urdf to get root link
        description = xacro.process(params['file'], mappings={'prefix': name})        
        urdf = URDF.from_xml_string(description)
                
        if self.has_odom:            
            self.link = urdf.get_root()
            self.rx = float(params['rx'])
            self.ry = float(params['ry'])
            self.w = 2*np.pi/float(params['t'])
            roll = params['roll'] if 'roll' in params else 0
            if roll:
                vmin = abs(self.w * min(self.rx, self.ry))
                vmax = abs(self.w * max(self.rx, self.ry))
                self.roll0 = -roll * vmin / (vmax - vmin)
                self.rollv = roll / (vmax - vmin)
            else:
                self.rollv = 0
            self.origin = Homogeneous(params['center'])
            
            # build fixed-to-ellipse plane matrix
            poses[self.link] = self.origin.copy()
                                    
        if self.has_joints:
            
            self.joint_pub = node.create_publisher(JointState, '/{}/joint_states'.format(name), 1)
            self.joints_T = params['joints']            
            self.joints = JointState()
            self.joints.name = [j.name for j in urdf.joints if j.joint_type != 'fixed']
            self.joints.position = [0. for _ in self.joints_T]
        
    def update(self, now):
        
        t = now.nanoseconds/1000000000.
        
        if self.has_odom:
            c = np.cos(self.w*t)
            s = np.sin(self.w*t)
            dx = -self.rx * self.w * s
            dy = self.ry * self.w * c
            # position in x-y plane
            roll = 0
            if self.rollv:
                roll = self.roll0 + self.rollv * np.sqrt(dx*dx + dy*dy)
            M = HomogeneousFrom(np.array([[self.rx * c, self.ry * s, 0]]).T, euler2mat(roll, 0, np.arctan2(dy, dx)))
            
            # to absolute frame
            poses[self.link] = self.origin * M
            
        if self.has_joints:
            self.joints.header.stamp = now.to_msg()
            for i,T in enumerate(self.joints_T):
                self.joints.position[i] = t*2*np.pi/T % 2*np.pi - np.pi
            
            self.joint_pub.publish(self.joints)
        
class Slide:

    pose_offset = Homogeneous([0.01,0,0,0,0,0])
    pose_offset[:3,3] *= config['scale']
    pose_offset_inv = HomogeneousInverse(pose_offset)
    
    def __init__(self, slide):

        self.link = 'presenter/slide_{}'.format(slide)        
        poses[self.link] = Homogeneous(config[slide if slide in config else 1]['pose'])
        self.front = False
                        
        cam_rel_pose = Homogeneous([2.8,0,0,0,0,0])
        if 'fit' in config and slide in config['fit']:            
            cam_rel_pose = Homogeneous([2.3,0,0,0,0,0])
        cam_rel_pose[:3,3] *= config['scale']        
        self.cam = poses[self.link] * Slide.pose_offset * cam_rel_pose
            
    def hide(self):
        if self.front:
            poses[self.link] *= Slide.pose_offset_inv
            self.front = False
            
    def show(self):
        if not self.front:
            poses[self.link] *= Slide.pose_offset
            self.front = True            
        
class RSPNode(Node):
    def __init__(self, **kwargs):
        super().__init__('robot_state_publisher',
                        allow_undeclared_parameters=True, 
                        automatically_declare_parameters_from_overrides=True,
                        namespace='presenter',
                        **kwargs)
                
        # set slides as local robot_description for Coral to parse
        slides_xacro = get_package_share_directory('marine_presenter') + '/objects/slides.xacro'
        mesh_path = os.path.abspath(os.path.dirname(config_file)) + '/mesh'
        description = xacro.process(slides_xacro, mappings={'slides':str(slides), 'mesh_path':mesh_path})        
        self.declare_parameter('robot_description', description)
        
        
class PresenterNode(Node):
    def __init__(self, name, **kwargs):
        super().__init__(name,
                        allow_undeclared_parameters=True, 
                        automatically_declare_parameters_from_overrides=True,
                        **kwargs)
        
        # init slides
        self.slides = [None] + [Slide(slide) for slide in range(1, slides+1)]
        
        # init moving objects
        self.objects = {}
        if 'objects' in config:
            self.objects = [MovingObject(name, params, self) for name, params in config['objects'].items()]
            
        # init cam view
        self.cam = Camera(self.slides[1].cam, config['gain'])
        poses['coral_cam_view'] = self.cam.M.copy()
        self.Md = self.cam.M.copy()
        
        self.br = transform_broadcaster.TransformBroadcaster(self)
        
        # init button service
        self.button = Button(self)
        
        # current state
        self.slide = 1
        self.in_video = False
        
        self.timer = self.create_timer(0.05, self.update)
        
    def show_slide(self):
        self.slides[self.slide].show()
        
    def hide_slide(self):
        self.slides[self.slide].hide()        
        
    def update(self):
                
        # update current slide
        if self.button.pressed:
            if self.button.pressed == ButtonPress.Request.BUTTON_ALT:
                #print('Slide {} ({}): \n  {}'.format(self.slide, self.in_video, config[self.slide]))
                print('Slide {} ({})'.format(self.slide, self.in_video))
                if self.slide in config and 'video' in config[self.slide] and not self.in_video:
                    print('Slide {} ({}): \n  {}'.format(self.slide, self.in_video, config[self.slide]))
                    os.system('vlc {} -f &'.format(config[self.slide]['video']))
                    self.in_video = True
                elif self.in_video:
                    os.system('killall vlc')
                    self.in_video = False
            else:
                self.hide_slide()
                if self.button.pressed == ButtonPress.Request.BUTTON_PREV:
                    self.slide = 1 + (self.slide-1) % slides 
                else:
                    self.slide = 1 + (self.slide+1) % slides
                self.show_slide()
                
                self.Md = self.slides[self.slide].cam
                
            self.button.reset()
            
        elif self.button.slide not in (-1, self.slide):
            self.hide_slide()
            self.slide = self.button.slide
            self.show_slide()
            
            self.Md = self.slides[self.slide].cam
            
        # update TFs
        now = self.get_clock().now()        
        for obj in self.objects:
            obj.update(now)            
        poses['coral_cam_view'] = self.cam.move(self.Md)
        
        # publish them
        transforms = []
        tr = TransformStamped()
        tr.header.stamp = now.to_msg()
        tr.header.frame_id = 'world'
        for link, pose in poses.items():
            tr.child_frame_id = link
            mat2tf(pose, tr.transform)
            transforms.append(deepcopy(tr))
        self.br.sendTransform(transforms)
        

   
        
rclpy.init()    

executor = SingleThreadedExecutor()

executor.add_node(PresenterNode('presenter'))
executor.add_node(RSPNode())

executor.spin()

#processes.stop()
#presenter.destroy_node()
#rclpy.shutdown()
   
