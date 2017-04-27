
#import argparse
usage="""

Run in simulation with a translation and a rotation of fake data:
Rona: for us, --fake_data_seg=demo_files0_seg00
Sneha(4/7): ./scripts/do_task.py master.h5 --fake_data_segment=demo_files0_seg00 --execution=0 --animation=1  --fake_data_transform .1 .1 .1 .1 .1 .1

./do_task.py ~/Data/sampledata/overhand/overhand.h5 --fake_data_segment=overhand0_seg00 --execution=0  --animation=1 --select_manual --fake_data_transform .1 .1 .1 .1 .1 .1

Run in simulation choosing the closest demo, single threaded
./do_task.py ~/Data/all.h5 --fake_data_segment=demo1-seg00 --execution=0  --animation=1  --parallel=0 

Actually run on the robot without pausing or animating 
./do_task.py ~/Data/overhand2/all.h5 --execution=1 --animation=0

"""
"""
parser = argparse.ArgumentParser(usage=usage)
parser.add_argument("h5file", type=str)
parser.add_argument("--cloud_proc_func", default="extract_yellow")
parser.add_argument("--cloud_proc_mod", default="rapprentice.cloud_proc_funcs")
    
parser.add_argument("--execution", type=int, default=0)
parser.add_argument("--animation", type=int, default=0)
parser.add_argument("--parallel", type=int, default=1)

parser.add_argument("--prompt", action="store_true")
parser.add_argument("--show_neighbors", action="store_true")
parser.add_argument("--select_manual", action="store_true")
parser.add_argument("--log", action="store_true")

parser.add_argument("--fake_data_segment",type=str)
parser.add_argument("--fake_data_transform", type=float, nargs=6, metavar=("tx","ty","tz","rx","ry","rz"),
    default=[0,0,0,0,0,0], help="translation=(tx,ty,tz), axis-angle rotation=(rx,ry,rz)")

parser.add_argument("--interactive",action="store_true")

args = parser.parse_args()

if args.fake_data_segment is None: assert args.execution==1
"""

from rapprentice import registration, colorize, berkeley_pr2, \
     animate_traj, ros2rave, plotting_openrave, task_execution, \
     planning, tps, func_utils, resampling, clouds
from rapprentice import math_utils as mu
from rapprentice.yes_or_no import yes_or_no

try:
    from rapprentice import pr2_trajectories, PR2
    import rospy
except ImportError:
    print "Couldn't import ros stuff"

import cloudprocpy, trajoptpy, openravepy
import os, numpy as np, h5py, time
from numpy import asarray
import importlib
import matplotlib

def extract_yellow(bgr, depth, T_w_k):
    """
    extract yellow points and downsample
    """
    rgb = bgr[...,::-1]
    print rgb
    scaled_rgb = np.divide(rgb, 255.0)
    print scaled_rgb
    hsv = matplotlib.colors.rgb_to_hsv(scaled_rgb)
    #hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    hsv = np.multiply(hsv,255.0)
    h = hsv[:,:,0]
    s = hsv[:,:,1]
    v = hsv[:,:,2]
    
    h_mask = (h < 56) | (h > 32) 
    s_mask = (s > 100)
    v_mask = (v > 150)
    red_mask = h_mask & s_mask & v_mask
    
    valid_mask = depth > 0

    xyz_k = clouds.depth_to_xyz(depth, berkeley_pr2.f)
    xyz_w = xyz_k.dot(T_w_k[:3,:3].T) + T_w_k[:3,3][None,None,:]
    
    z = xyz_w[:,:,2]   
    z0 = xyz_k[:,:,2]

    height_mask = xyz_w[:,:,2] > .7 # TODO pass in parameter
    
    good_mask = red_mask & height_mask & valid_mask
    good_mask =   skim.remove_small_objects(good_mask,min_size=64)

    good_xyz = xyz_w[good_mask]
    print good_xyz.shape

    return clouds.downsample(good_xyz, .001)
cloud_proc_func = extract_yellow

#cloud_proc_mod = importlib.import_module("rapprentice.cloud_proc_funcs")

class Globals:
    robot = None
    env = None

    pr2 = None


    
def main():

    demofile = h5py.File("master.h5", 'r')
    
    trajoptpy.SetInteractive(False)
    Globals.env = openravepy.Environment()
    Globals.env.StopSimulation()
    Globals.env.Load("robots/pr2-beta-static.zae")    
    Globals.robot = Globals.env.GetRobots()[0]
    Globals.viewer = trajoptpy.GetViewer(Globals.env)
    # cloud_proc_func(None, None, None)
    while True:
    	time.sleep(1)


if __name__ == "__main__":
    main()