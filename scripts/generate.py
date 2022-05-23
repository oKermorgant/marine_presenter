#!/usr/bin/env python3

import sys
import os
import shutil
import cv2
from ament_index_python import get_package_share_directory
import yaml
import numpy as np
from subprocess import check_output, run
from copy import deepcopy
import xacro
from urdf_parser_py.urdf import URDF

from scipy.spatial.transform import Rotation
def euler2mat(r, p, y, seq):    
    return Rotation.from_euler('xyz',(r,p,y)).as_dcm()
def mat2euler(M):
    return Rotation.from_dcm(M[:3,:3]).as_euler('xyz').tolist()

marine_ppt = get_package_share_directory('marine_presenter')

def add_icon(img, x, y, w):
    im = cv2.imread(img)
    H,W = im.shape[:2]
    X = int(W*x/100.)
    Y = int(H*y/100.)
    w /= 100.
    icon = cv2.imread(marine_ppt + '/objects/video.png')
    scale = w*W/icon.shape[1]
    icon = cv2.resize(icon, None, fx=scale, fy=scale, interpolation=cv2.INTER_LANCZOS4)
    im[Y:Y+icon.shape[0], X-icon.shape[1]:X] = icon
    cv2.imwrite(img, im)
    
def add_borders(img, w = 20):
    im = cv2.imread(img)
    H,W = im.shape[:2]
    w = int(W*(1+w/100.))
    im2 = np.full((H,W+w,3), 255, dtype=np.uint8)
    im2[:,w//2:W+w//2] = im
    cv2.imwrite(img, im2)
    
# get initial pdf
filename = sys.argv[1]
if filename[-4:] != '.pdf':
    filename = (filename+'.pdf').replace('..','.')
    
if not os.path.exists(filename):
    print(filename + ' does not exist, exiting')
    sys.exit(0)
    
# check config file
if os.path.exists(filename.replace('.pdf', '.yaml')):
    config = yaml.safe_load(open(filename.replace('.pdf', '.yaml')))
else:
    config = {}
    
video_x = 99.5
video_y = 0.5
video_w = 3
if 'video' in config:
    video_x = config['video']['x']
    video_y = config['video']['y']
    video_w = config['video']['w']

if 'scale' not in config:
    config['scale'] = 3
    
scale = config['scale']

ext = 'png'
cam_pose = [1, 0, scale, 0, 0, 0]    # wrt current slide

def dict_replace(s, d):
    for key in d:
        s = s.replace(key, d[key])
    return s

def read_pose(pose, base_pose = (0,0,0,0,0,0), scaling=True):
    if type(pose) == dict:
        if 'pose' in pose:
            return read_pose(pose['pose'], base_pose, scaling)
        return base_pose
    
    if len(pose) >= 3 and scaling:
        pose[2] *= scale
    if len(pose) == 6:
        return pose
    # complete with base_pose
    return pose + base_pose[len(pose)-6:]

def Homogeneous(pose):
    R = euler2mat(pose[3], pose[4], pose[5], 'rxyz')
    t = np.array(pose[:3]).reshape(3,1)
    M = np.vstack((np.hstack((R, t)), [0,0,0,1]))
    for i in range(3):
        for j in range(3):
            for v in [-1,0,1]:
                if abs(M[i,j]-v) < 1e-3:
                    M[i,j] = v
    return np.matrix(M)

def rec_insert(d, keys, val):
    key = keys.pop(0)
    if not len(keys):
        d[key] = val
        return
    if key not in d:
        d[key] = {}
    rec_insert(d[key], keys, val)
    
def resolve(xacro_file):
    pkg, path = xacro_file[10:].split('/',1)
    pkg = get_package_share_directory(pkg)
    
    xacro_file = os.path.join(pkg,path)
    
    if os.path.exists(xacro_file):
        return xacro_file

    # path is a raw file, look for it
    for root, dirs, files in os.walk(pkg, topdown=False):
        if path in files:
            return os.path.join(root, path)
        
    return xacro_file

basename = os.path.abspath(filename[:-4])
mesh_path = basename + '/mesh'
img_path = mesh_path + '/img'
config_out = basename + '/config.yaml'
generate = '-f' in sys.argv

def slide_img(slide):
    return '{}/{}.{}'.format(img_path, slide, ext)

for path in (basename, mesh_path, img_path):
    if not os.path.exists(path):
        os.mkdir(path)
        generate = True
        
# get PDF pages
pages = check_output(['pdfinfo',filename]).decode('utf-8').splitlines()
pages = [line for line in pages if line.startswith('Pages:')][0]
pages = int(pages.replace('Pages:','').replace(' ',''))

# change config to page-numbered, check videos

titles = {}
for slide in range(1,pages+1):
    title = check_output(['pdftotext','-f', str(slide), '-l', str(slide),'-layout', '{}'.format(filename),'-']).decode('utf-8')
    title = title[:title.find('\n')]
    if [ord(c) for c in title[-2:]] == [8722,49]:
        title = title[:-5]
    if title not in titles:
        titles[title] = [slide]
    else:
        titles[title].append(slide)
        
for title, slides in titles.items():
    
    base_slide = slides[0]
    
    if title in config:
        config[base_slide] = config.pop(title)
    
    if base_slide in config:
        
        slide_videos = [key for key in config[base_slide] if key.startswith('video')]
        for video in slide_videos:
            video_path = config[base_slide].pop(video)
            print(f'  found video: {video_path}')
            if not video_path.startswith('/'):
                video_path = os.path.abspath(os.path.dirname(basename) + '/' + video_path)
            if video == 'video':            
                for slide in slides:
                    rec_insert(config, [slide, 'video'], video_path)
            else:
                offset = int(video[5:])-1
                rec_insert(config, [base_slide+offset, 'video'], video_path)
 
videos = sorted([key for key in config if type(key) == int and 'video' in config[key]])
print('Found {} slides'.format(pages))
        
pdf_change = True
video_change = True
scale_change = True
if os.path.exists(config_out):
    try:
        prev = yaml.safe_load(open(config_out))
        if 'pdf_time' in prev:
            pdf_change = prev['pdf_time'] != int(os.stat(filename).st_mtime)
        video_change = videos != sorted([key for key in prev if type(key) == int and 'video' in prev[key]])
        scale_change = scale != prev['scale']
    except:
        pass
config['pdf_time'] = int(os.stat(filename).st_mtime)
    
    
if pdf_change or video_change or generate:
    print('{} change: generating base images...'.format(pdf_change and 'PDF' or 'Videos'))
    os.system('pdftoppm {} -{} -scale-to 2048 {}/'.format(filename, ext if ext == 'png' else 'jpeg', img_path))
    
    for img in os.listdir(img_path):
        os.rename('{}/{}'.format(img_path, img), '{}/{}'.format(img_path, img.strip('-0')))
    
    # add video tag / borders
    for idx in videos:
        add_icon(slide_img(idx), video_x, video_y, video_w)

    if 'fit' in config:
        for slide in config['fit']:
            add_borders(slide_img(slide))

config['slides'] = pages
if 'cam_pose' not in config:
    config['cam_pose'] = cam_pose

# build meshs
print('Generating osg meshes...')

base_mesh = open(marine_ppt + '/templates/slide.osg').read()

pose = read_pose(config)
if 'pose' in config:
    config.pop('pose')
    
for slide in range(1, pages+1):
    im = cv2.imread(slide_img(slide))
    H,W = im.shape[:2]
    ratio = round(float(W)/H,2)
        
    dest = '{}/{}.osg'.format(mesh_path,str(slide))
    if scale_change or not os.path.exists(dest):
        open(dest, 'w').write(base_mesh.replace('<image>', 'img/{}.{}'.format(slide, ext)).replace('<x>', str(ratio*scale)).replace('<z>', str(scale)))
    
    # add pose information
    if slide not in config:
        config[slide] = {}
    if 'pose' in config[slide]:
        pose = read_pose(config[slide], pose)
    config[slide]['pose'] = pose        
    
    
# objects are found by the presenter node, just adapt pose / scale and find files

if 'objects' in config:
    print('Resolving object paths...')
    for name in config['objects']:
        data = config['objects'][name]        
        xacro_file = data['file'] if 'file' in data else name
        
        if xacro_file.startswith('package'):
            xacro_file = resolve(xacro_file)
            data['file'] = xacro_file
        
        if '/' not in xacro_file:
            marine_files = os.listdir(marine_ppt + '/objects/urdf')
            # find relative to marine_ppt
            l = len(xacro_file)
            found = False
            
            while l and not found:
                if xacro_file[:l] + '.xacro' in marine_files:
                    print('  Found object "{}", available in marine_presenter as {}.xacro'.format(name, xacro_file[:l]))
                    xacro_file = '{}/objects/urdf/{}.xacro'.format(marine_ppt, xacro_file[:l])
                    found = True                    
                l -= 1
            
            if not found:
                # look for local objects
                l = len(xacro_file)
                while l and not found:
                    for root, dirs, files in os.walk(os.path.dirname(basename), topdown=False):                        
                        for ext in ('.xacro','.urdf'):
                            candidate = xacro_file[:l] + ext
                            if candidate in files:
                                print('  Found object "{}", found locally as {}'.format(name, candidate))
                                xacro_file = os.path.join(root, candidate)
                                found = True
                                break
                        if found:
                            break
                    l -= 1
            
            if not found:
                print('  Cannot find object "{}" in marine_presenter or local path'.format(name))
            
            data['file'] = xacro_file
            
        else:
            # just check the file exists and trust the user
            if os.path.exists(xacro_file):
                print('  Found object "{}" at {}'.format(name, xacro_file))
            else:
                print('  Cannot find object "{}", should be at {}'.format(name, xacro_file))

        # find pose and scale
        pose = [0,0,0,0,0,0]
        if 'center' in data and 'slide' not in data:
            # absolute pose
            pose = read_pose(data['center'],pose, False)
        if 'slide' in data:
            if 'center' in data:
                # relative pose with scaling
                pose = read_pose(data['center'],pose)
            if type(data['slide']) == str:
                data['slide'] = titles[data['slide']][0]
            # pose is actually relative to slide
            M = Homogeneous(config[data['slide']]['pose']) * Homogeneous(pose)
            for i in range(3):
                pose[i] = float(M[i,3])
            pose[3:] = mat2euler(M)
        data['center'] = [v for v in pose]
        for v in ('rx','ry'):
            if v in data:
                data[v] *= scale
                
yaml.safe_dump(config, open(config_out, 'w'), default_flow_style=False)     

print('Generating launch file...', '\r')

manual = open(marine_ppt + '/templates/manual.yaml').read()
with open(basename + '/manual.yaml', 'w') as f:
    f.write(manual.replace('<slides>', str(pages)))

launch = open(marine_ppt + '/templates/presentation_launch.py').read()

objects = {}
fixed_tfs = {}

if 'objects' in config:
    for name, params in config['objects'].items():        
        objects[name] = params['file']        
        # check if static
        if 'rx' not in params:
            description = xacro.process(params['file'], mappings={'prefix': name})        
            link = URDF.from_xml_string(description).get_root()
            fixed_tfs[link] = [params['center'][i] for i in [0,1,2,5,4,3]]       
            
with open(basename + '/presentation_launch.py', 'w') as f:
    f.write(launch.replace('<objects>', str(objects)).replace('<fixed_tfs>', str(fixed_tfs)))

print('done.')

if '-p' in sys.argv:
    os.system('ros2 launch ' + basename + '/presentation_launch.py')
