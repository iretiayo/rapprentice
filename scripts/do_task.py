#!/usr/bin/env python

import argparse
usage="""

Run in simulation with a translation and a rotation of fake data:
Rona: for us, --fake_data_seg=demo_files0_seg00
Sneha(4/7): ./scripts/do_task.py master.h5 --fake_data_segment=demo_files0_seg00 --execution=0 --animation=1  --fake_data_transform .1 .1 .1 .1 .1 .1

./do_task.py ~/Data/sampledata/overhand/overhand.h5 --fake_data_segment=overhand0_seg00 --execution=0  --animation=1 --select_manual --fake_data_transform .1 .1 .1 .1 .1 .1

Run in simulation choosing the closest demo, single threaded
./do_task.py ~/Data/all.h5 --fake_data_segment=demo1-seg00 --execution=0  --animation=1  --parallel=0 

Actually run on the robot without pausing or animating 
./do_task.py ~/Data/overhand2/all.h5 --execution=1 --animation=0
Sneha (4/19): ./scripts/do_task.py master.h5 --execution=1 --animation=1 --parallel=1

"""
parser = argparse.ArgumentParser(usage=usage)
parser.add_argument("h5file", type=str)
parser.add_argument("--cloud_proc_func", default="extract_yellow")
parser.add_argument("--cloud_proc_mod", default="rapprentice.cloud_proc_funcs")
    
parser.add_argument("--execution", type=int, default=0)
parser.add_argument("--animation", type=int, default=0)
parser.add_argument("--parallel", type=int, default=1)

parser.add_argument("--prompt", default=1)#action="store_true")
parser.add_argument("--show_neighbors", default=1)#action="store_true")
parser.add_argument("--select_manual", default=0) #action="store_true")
parser.add_argument("--log", action="store_true")

parser.add_argument("--fake_data_segment",type=str)
parser.add_argument("--fake_data_transform", type=float, nargs=6, metavar=("tx","ty","tz","rx","ry","rz"),
    default=[0,0,0,0,0,0], help="translation=(tx,ty,tz), axis-angle rotation=(rx,ry,rz)")

parser.add_argument("--interactive",action="store_true")

args = parser.parse_args()

if args.fake_data_segment is None: assert args.execution==1

###################


"""
Workflow:
1. Fake data + animation only
    --fake_data_segment=xxx --execution=0
2. Fake data + Gazebo. Set Gazebo to initial state of fake data segment so we'll execute the same thing.
    --fake_data_segment=xxx --execution=1
    This is just so we know the robot won't do something stupid that we didn't catch with openrave only mode.
3. Real data + Gazebo
    --execution=1 
    The problem is that the gazebo robot is in a different state from the real robot, in particular, the head tilt angle. TODO: write a script that       sets gazebo head to real robot head
4. Real data + Real execution.
    --execution=1

The question is, do you update the robot's head transform.
If you're using fake data, don't update it.

"""

import subprocess
import sys
# import cv_bridge
# import cv2
import message_filters
from sensor_msgs.msg import Image
from rapprentice import registration, colorize, berkeley_pr2, \
     animate_traj, ros2rave, plotting_openrave, task_execution, \
     planning, tps, func_utils, resampling, clouds
from rapprentice import math_utils as mu
from rapprentice.yes_or_no import yes_or_no
import skimage.morphology as skim

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

subs = []
rgb = None
depth = None
do_dynamic_scaling = None

# cloud_proc_mod = importlib.import_module(args.cloud_proc_mod)
# cloud_proc_func = getattr(cloud_proc_mod, args.cloud_proc_func) 


#BEGINNING OF PASTE FROM CLOUD_PROC_FUNCs
DEBUG_PLOTS=True
rect = None
selecting = False
rect_mask = None
def on_mouse(event, x, y, flags, params):
    '''
        Mouse callback that sets the rectangle
        Click and drag to create the rectangle
    '''
    global rect, selecting
    if event == cv2.EVENT_LBUTTONDOWN and not selecting:
        rect = [(x, y)]
        selecting = True
        print 'First point selected: ', rect
    elif event == cv2.EVENT_LBUTTONUP and selecting:
        rect.append((x, y))
        selecting = False
        print 'Second point selected: ', rect


def get_rectangle(img):
    '''
        Get the rectangle selection from user and set it
        as a global var
    '''
    wname = "CropImage"
    cv2.namedWindow(wname)
    cv2.setMouseCallback(wname, on_mouse) 
    cv2.imshow(wname, img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def build_rectangle(img):
    global rect_mask
    rect_mask = np.zeros((img.shape[0], img.shape[1]), dtype=bool)
    rect_mask[rect[0][1]:rect[1][1],rect[0][0]:rect[1][0]] = True

def extract_yellow(bgr, depth, T_w_k, rect_file="rect.txt"):
    """
    extract yellow points and downsample
    """
    global rect
    if not os.path.isfile(rect_file):
        get_rectangle(bgr)
        with open(rect_file, 'w') as f:
            f.write(str(rect[0]) + "\n")
            f.write(str(rect[1]) + "\n")
    else:
        with open(rect_file, 'r') as f:
            rect = []
            for line in f:
                x,y = line.replace("(", '').replace(')', '').replace('\n', '').split(",")
                rect.append((int(x),int(y)))
    build_rectangle(bgr)
    rgb = bgr[...,::-1]
    scaled_rgb = np.divide(rgb, 255.0)
    hsv = matplotlib.colors.rgb_to_hsv(scaled_rgb)
    #hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    #hsv = np.multiply(hsv,255.0)
    h = hsv[:,:,0]
    s = hsv[:,:,1]
    v = hsv[:,:,2]
    #blue 157-255 out of 360
    blue_lower = 0.0 / 360.0
    blue_upper = 65.0 / 360.0
    h_mask = ((h < blue_upper) & (h > blue_lower))

    #h_mask = (h < 56) | (h > 32) 
    s_mask = (s > (10/100.0))
    v_mask = (v >  (40/100.0))
    red_mask = h_mask & s_mask & v_mask
    
    valid_mask = depth > 0.7

    xyz_k = clouds.depth_to_xyz(depth, berkeley_pr2.f)
    xyz_w = xyz_k.dot(T_w_k[:3,:3].T) + T_w_k[:3,3][None,None,:]
    
    z = xyz_w[:,:,2]   
    z0 = xyz_k[:,:,2]

    height_mask = xyz_w[:,:,2] > .7 # TODO pass in parameter
    good_mask = red_mask & height_mask & valid_mask 
    good_mask = good_mask & rect_mask
    good_mask =   skim.remove_small_objects(good_mask,min_size=64)

    if DEBUG_PLOTS and args.execution:
        import cv2
        cv2.imshow("z0",z0/z0.max())
        cv2.imshow("z",z/z.max())
        cv2.imshow("hue", h_mask.astype('uint8')*255)
        cv2.imshow("sat", s_mask.astype('uint8')*255)
        cv2.imshow("val", v_mask.astype('uint8')*255)
        cv2.imshow("final",good_mask.astype('uint8')*255)
        cv2.imshow("rect", rect_mask.astype('uint8')*255)
        cv2.imshow("rgb", bgr)
        print "Press any key on cv2 images to continue"
        cv2.waitKey()
            
        
    

    good_xyz = xyz_w[good_mask]
    print good_xyz.shape
    
    # our_x = good_xyz[:,0]
    # our_y = good_xyz[:,1]
    # our_z = good_xyz[:,2] 
    # good_xyz[:,0] = our_z
    # good_xyz[:,1] = our_y
    # good_xyz[:,2] = our_x
    

    return clouds.downsample(good_xyz, .025)
    # return good_xyz

def extract_red(rgb, depth, T_w_k):
    """
    extract red points and downsample
    """
    import cv2    
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
    h = hsv[:,:,0]
    s = hsv[:,:,1]
    v = hsv[:,:,2]
    
    h_mask = (h<15) | (h>145)
    s_mask = (s > 30 )
    v_mask = (v > 100)
    red_mask = h_mask & s_mask & v_mask
    
    valid_mask = depth > 0
    
    xyz_k = clouds.depth_to_xyz(depth, berkeley_pr2.f)
    xyz_w = xyz_k.dot(T_w_k[:3,:3].T) + T_w_k[:3,3][None,None,:]
    
    z = xyz_w[:,:,2]   
    z0 = xyz_k[:,:,2]

    height_mask = xyz_w[:,:,2] > .7 # TODO pass in parameter
    
    good_mask = red_mask & height_mask & valid_mask
    good_mask =   skim.remove_small_objects(good_mask,min_size=64)

    if DEBUG_PLOTS:
        cv2.imshow("z0",z0/z0.max())
        cv2.imshow("z",z/z.max())
        cv2.imshow("hue", h_mask.astype('uint8')*255)
        cv2.imshow("sat", s_mask.astype('uint8')*255)
        cv2.imshow("val", v_mask.astype('uint8')*255)
        cv2.imshow("final",good_mask.astype('uint8')*255)
        cv2.imshow("rgb", rgb)
        cv2.waitKey()
            
        
    

    good_xyz = xyz_w[good_mask]
    

    return clouds.downsample(good_xyz, .025)

cloud_proc_func = extract_yellow    
# cloud_proc_func = extract_red
#END OF COPY OF CLOUD_PROC_FUNCS
def redprint(msg):
    print colorize.colorize(msg, "red", bold=True)

        

def split_trajectory_by_gripper(seg_info):
    rgrip = asarray(seg_info["r_gripper_joint"])
    lgrip = asarray(seg_info["l_gripper_joint"])

    thresh = .04 # open/close threshold

    n_steps = len(lgrip)


    # indices BEFORE transition occurs
    l_openings = np.flatnonzero((lgrip[1:] >= thresh) & (lgrip[:-1] < thresh))
    r_openings = np.flatnonzero((rgrip[1:] >= thresh) & (rgrip[:-1] < thresh))
    l_closings = np.flatnonzero((lgrip[1:] < thresh) & (lgrip[:-1] >= thresh))
    r_closings = np.flatnonzero((rgrip[1:] < thresh) & (rgrip[:-1] >= thresh))

    before_transitions = np.r_[l_openings, r_openings, l_closings, r_closings]
    after_transitions = before_transitions+1
    seg_starts = np.unique(np.r_[0, after_transitions])
    seg_ends = np.unique(np.r_[before_transitions, n_steps-1])

    return seg_starts, seg_ends

def binarize_gripper(angle):
    open_angle = .08
    closed_angle = 0    
    thresh = .04
    if angle > thresh: return open_angle
    else: return closed_angle


    

    
def set_gripper_maybesim(lr, value):
    if args.execution:
        print('249')
        gripper = {"l":Globals.pr2.lgrip, "r":Globals.pr2.rgrip}[lr]
        gripper.set_angle(value)
        Globals.pr2.join_all()
    else:
        Globals.robot.SetDOFValues([value*5], [Globals.robot.GetJoint("%s_gripper_l_finger_joint"%lr).GetDOFIndex()])
    return True
        
def exec_traj_maybesim(bodypart2traj):
    if args.animation:
        dof_inds = []
        trajs = []
        for (part_name, traj) in bodypart2traj.items():
            manip_name = {"larm":"leftarm","rarm":"rightarm"}[part_name]
            dof_inds.extend(Globals.robot.GetManipulator(manip_name).GetArmIndices())            
            trajs.append(traj)
        full_traj = np.concatenate(trajs, axis=1)
        Globals.robot.SetActiveDOFs(dof_inds)
        animate_traj.animate_traj(full_traj, Globals.robot, restore=False,pause=True)
    if args.execution:
        if not args.prompt or yes_or_no("execute?"):
            #print('270')
            #import IPython
            #IPython.embed()
            pr2_trajectories.follow_body_traj(Globals.pr2, bodypart2traj, speed_factor=0.25)
        else:
            return False

    return True


def find_closest_manual(demofile, _new_xyz):
    "for now, just prompt the user"
    seg_names = demofile.keys()
    print "choose from the following options (type an integer)"
    for (i, seg_name) in enumerate(seg_names):
        print "%i: %s"%(i,seg_name)
    choice_ind = task_execution.request_int_in_range(len(seg_names))
    chosen_seg = seg_names[choice_ind] 
    return chosen_seg

def registration_cost(xyz0, xyz1):
    print xyz0
    scaled_xyz0, _ = registration.unit_boxify(xyz0)
    print xyz1
    scaled_xyz1, _ = registration.unit_boxify(xyz1)
    f,g = registration.tps_rpm_bij(scaled_xyz0, scaled_xyz1, rot_reg=1e-3, n_iter=30)
    cost = registration.tps_reg_cost(f) + registration.tps_reg_cost(g)
    return cost

DS_SIZE = .025

@func_utils.once
def get_downsampled_clouds(demofile):
    return [clouds.downsample(seg["cloud_xyz"], DS_SIZE) for seg in demofile.values()]

def find_closest_auto(demofile, new_xyz):
    if args.parallel:
        from joblib import Parallel, delayed
    demo_clouds = [asarray(seg["cloud_xyz"]) for seg in demofile.values()]
    keys = demofile.keys()
    if args.parallel:
        costs = Parallel(n_jobs=3,verbose=100)(delayed(registration_cost)(demo_cloud, new_xyz) for demo_cloud in demo_clouds)
    else:
        costs = []
        for (i,ds_cloud) in enumerate(demo_clouds):
            costs.append(registration_cost(ds_cloud, new_xyz))
            print "completed %i/%i"%(i+1, len(demo_clouds))
    
    print "costs\n",costs
    if args.show_neighbors:
        nshow = min(5, len(keys))
        import cv2, rapprentice.cv_plot_utils as cpu
        sortinds = np.argsort(costs)[:nshow]
        near_rgbs = [asarray(demofile[keys[i]]["rgb"]) for i in sortinds]
        bigimg = cpu.tile_images(near_rgbs, 1, nshow)
        cv2.imshow("neighbors", bigimg)
        print "press any key to continue"
        cv2.waitKey()
        
    ibest = np.argmin(costs)
    return keys[ibest]
            
            
def arm_moved(joint_traj):    
    if len(joint_traj) < 2: return False
    return ((joint_traj[1:] - joint_traj[:-1]).ptp(axis=0) > .01).any()
        
def tpsrpm_plot_cb(x_nd, y_md, targ_Nd, corr_nm, wt_n, f):
    ypred_nd = f.transform_points(x_nd)
    handles = []
    # new rope in green
    handles.append(Globals.env.plot3(ypred_nd, 3, (0,1,0)))
    handles.extend(plotting_openrave.draw_grid(Globals.env, f.transform_points, x_nd.min(axis=0), x_nd.max(axis=0), xres = .1, yres = .1, zres = .04))
    #Globals.viewer.Step()


def unif_resample(traj, max_diff, wt = None):        

    #Resample a trajectory so steps have same length in joint space    

    import scipy.interpolate as si
    tol = .005
    if wt is not None: 
        wt = np.atleast_2d(wt)
        traj = traj*wt
        
        
    dl = mu.norms(traj[1:] - traj[:-1],1)
    l = np.cumsum(np.r_[0,dl])
    goodinds = np.r_[True, dl > 1e-8]
    deg = min(3, sum(goodinds) - 1)
    if deg < 1: return traj, np.arange(len(traj))
    
    nsteps = max(int(np.ceil(float(l[-1])/max_diff)),2)
    newl = np.linspace(0,l[-1],nsteps)

    ncols = traj.shape[1]
    colstep = 10
    traj_rs = np.empty((nsteps,ncols)) 
    for istart in xrange(0, traj.shape[1], colstep):
        (tck,_) = si.splprep(traj[goodinds, istart:istart+colstep].T,k=deg,s = tol**2*len(traj),u=l[goodinds])
        traj_rs[:,istart:istart+colstep] = np.array(si.splev(newl,tck)).T
    if wt is not None: traj_rs = traj_rs/wt

    newt = np.interp(newl, l, np.arange(len(traj)))

    return traj_rs, newt

def get_rgbd():
    # set_params_cmd = "rosrun image_view extract_images_sync _inputs:='[/kinect2/hd/image_color, /kinect2/hd/image_depth_rect]'"
    # set_params_handle = subprocess.Popen(set_params_cmd, shell=True)
    # time.sleep(1)
    global rgb, depth
    if args.execution:
        import cv2, cv_bridge
        do_dynamic_scaling = rospy.get_param(
            '~do_dynamic_scaling', False)
        img_topics = ['/kinect2/hd/image_color','/kinect2/hd/image_depth_rect']

        bridge = cv_bridge.CvBridge()

        for i, t in enumerate(img_topics):
            imgmsg = rospy.wait_for_message(t, Image)
            img = bridge.imgmsg_to_cv2(imgmsg)
            encoding_in = imgmsg.encoding
            img = cv_bridge.cvtColorForDisplay(
                img, encoding_in=encoding_in, encoding_out='',
                do_dynamic_scaling=do_dynamic_scaling)
            if i== 0:
                rgb = cv2.resize(img, (640,480))
            else:
                depth_large = np.fromstring(imgmsg.data, dtype=np.uint16).reshape(imgmsg.height, imgmsg.width)
                depth = cv2.resize(depth_large, (640,480))




###################


class Globals:
    robot = None
    env = None

    pr2 = None

def main():
    global rgb, depth
    demofile = h5py.File(args.h5file, 'r')
    
    trajoptpy.SetInteractive(args.interactive)


    if args.log:
        print "in logs"
        LOG_DIR = osp.join(osp.expanduser("~/Data/do_task_logs"), datetime.datetime.now().strftime("%Y%m%d-%H%M%S"))
        os.mkdir(LOG_DIR)
        LOG_COUNT = 0
                        

    if args.execution:
        print "in execution"
        rospy.init_node("exec_task",disable_signals=True)
        Globals.pr2 = PR2.PR2()
        Globals.env = Globals.pr2.env
        Globals.robot = Globals.pr2.robot
        
    else:
        Globals.env = openravepy.Environment()
        Globals.env.StopSimulation()
        Globals.env.Load("robots/pr2-beta-static.zae")    
        Globals.robot = Globals.env.GetRobots()[0]

    if not args.fake_data_segment:
        print "not fake data seg"
        # grabber = cloudprocpy.CloudGrabber()
        # grabber.startRGBD()
    redprint("line 282")
    Globals.viewer = trajoptpy.GetViewer(Globals.env)
    print "after viewer"
    
    #####################
    while True:
        
    
        redprint("Acquire point cloud")
        if args.fake_data_segment:
            fake_seg = demofile[args.fake_data_segment]
            new_xyz = np.squeeze(fake_seg["cloud_xyz"])
            hmat = openravepy.matrixFromAxisAngle(args.fake_data_transform[3:6])
            hmat[:3,3] = args.fake_data_transform[0:3]
            new_xyz = new_xyz.dot(hmat[:3,:3].T) + hmat[:3,3][None,:]
            r2r = ros2rave.RosToRave(Globals.robot, asarray(fake_seg["joint_states"]["name"]))
            r2r.set_values(Globals.robot, asarray(fake_seg["joint_states"]["position"][0]))
        else:
            #Globals.pr2.head.set_pan_tilt(0,1.2)
            Globals.pr2.head.set_pan_tilt(0,1.32)
            Globals.pr2.rarm.goto_posture('side')
            Globals.pr2.larm.goto_posture('side')            
            Globals.pr2.join_all()
            time.sleep(.5)

            
            Globals.pr2.update_rave()
            
            #rgb, depth = grabber.getRGBD()
            get_rgbd()
            T_w_k = berkeley_pr2.get_kinect_transform(Globals.robot)
            new_xyz = cloud_proc_func(rgb, depth, T_w_k)
    
            #grab_end(new_xyz)

    
        if False: #args.log:
            LOG_COUNT += 1
            import cv2
            cv2.imwrite(osp.join(LOG_DIR,"rgb%i.png"%LOG_COUNT), rgb)
            cv2.imwrite(osp.join(LOG_DIR,"depth%i.png"%LOG_COUNT), depth)
            np.save(osp.join(LOG_DIR,"xyz%i.npy"%LOG_COUNT), new_xyz)
        
        ################################    
        redprint("Finding closest demonstration")
        if args.select_manual:
            seg_name = find_closest_manual(demofile, new_xyz)
        else:
            seg_name = find_closest_auto(demofile, new_xyz)
        
        seg_info = demofile[seg_name]
        redprint("closest demo: %s"%(seg_name))
        if "done" in seg_name:
            redprint("DONE!")
            break
    
    
        if args.log:
            with open(osp.join(LOG_DIR,"neighbor%i.txt"%LOG_COUNT),"w") as fh: fh.write(seg_name) 
        ################################

        redprint("Generating end-effector trajectory")    

        handles = []
        old_xyz = np.squeeze(seg_info["cloud_xyz"])
        #Plot old data in red
        handles.append(Globals.env.plot3(old_xyz,5, (1,0,0)))
        # Plot new data in green
        handles.append(Globals.env.plot3(new_xyz,5, (0,0,1)))

        scaled_old_xyz, src_params = registration.unit_boxify(old_xyz)
        scaled_new_xyz, targ_params = registration.unit_boxify(new_xyz)

        f,_ = registration.tps_rpm_bij(scaled_old_xyz, scaled_new_xyz, plot_cb = tpsrpm_plot_cb,
                                       plotting=5 if args.animation else 0,rot_reg=np.r_[1e-4,1e-4,1e-1], n_iter=50, reg_init=10, reg_final=.1)
        f = registration.unscale_tps(f, src_params, targ_params)
        
        handles.extend(plotting_openrave.draw_grid(Globals.env, f.transform_points, old_xyz.min(axis=0)-np.r_[0,0,.1], old_xyz.max(axis=0)+np.r_[0,0,.1], xres = .1, yres = .1, zres = .04))        

        link2eetraj = {}
        for lr in 'lr':
            link_name = "%s_gripper_tool_frame"%lr
            old_ee_traj = asarray(seg_info[link_name]["hmat"])
            new_ee_traj = f.transform_hmats(old_ee_traj)
            link2eetraj[link_name] = new_ee_traj
            
            # old trajectory in red
            handles.append(Globals.env.drawlinestrip(old_ee_traj[:,:3,3], 2, (1,0,0,1)))
            # new trajextory in green
            handles.append(Globals.env.drawlinestrip(new_ee_traj[:,:3,3], 2, (0,1,0,1)))
    
        miniseg_starts, miniseg_ends = split_trajectory_by_gripper(seg_info)    
        success = True
        print colorize.colorize("mini segments:", "red"), miniseg_starts, miniseg_ends
        for (i_miniseg, (i_start, i_end)) in enumerate(zip(miniseg_starts, miniseg_ends)):
            
            if args.execution=="real": Globals.pr2.update_rave()


            ################################    
            redprint("Generating joint trajectory for segment %s, part %i"%(seg_name, i_miniseg))
            
            
            
            # figure out how we're gonna resample stuff
            lr2oldtraj = {}
            for lr in 'lr':
                manip_name = {"l":"leftarm", "r":"rightarm"}[lr]                 
                old_joint_traj = asarray(seg_info[manip_name][i_start:i_end+1])
                #print (old_joint_traj[1:] - old_joint_traj[:-1]).ptp(axis=0), i_start, i_end
                if arm_moved(old_joint_traj):       
                    lr2oldtraj[lr] = old_joint_traj   
            if len(lr2oldtraj) > 0:
                old_total_traj = np.concatenate(lr2oldtraj.values(), 1)
                JOINT_LENGTH_PER_STEP = .1
                _, timesteps_rs = unif_resample(old_total_traj, JOINT_LENGTH_PER_STEP)
            ####

        
            ### Generate fullbody traj
            bodypart2traj = {}            
            for (lr,old_joint_traj) in lr2oldtraj.items():

                manip_name = {"l":"leftarm", "r":"rightarm"}[lr]
                 
                old_joint_traj_rs = mu.interp2d(timesteps_rs, np.arange(len(old_joint_traj)), old_joint_traj)

                ee_link_name = "%s_gripper_tool_frame"%lr
                new_ee_traj = link2eetraj[ee_link_name][i_start:i_end+1]          
                new_ee_traj_rs = resampling.interp_hmats(timesteps_rs, np.arange(len(new_ee_traj)), new_ee_traj)
                if args.execution: Globals.pr2.update_rave()
                new_joint_traj = planning.plan_follow_traj(Globals.robot, manip_name,
                 Globals.robot.GetLink(ee_link_name), new_ee_traj_rs,old_joint_traj_rs)
                part_name = {"l":"larm", "r":"rarm"}[lr]
                bodypart2traj[part_name] = new_joint_traj

        

            ################################    
            redprint("Executing joint trajectory for segment %s, part %i using arms '%s'"%(seg_name, i_miniseg, bodypart2traj.keys()))

            for lr in 'lr':
                success &= set_gripper_maybesim(lr, binarize_gripper(seg_info["%s_gripper_joint"%lr][i_start]))
                # Doesn't actually check if grab occurred, unfortunately

        
            if not success: break
        
            if len(bodypart2traj) > 0:
                print('Length of bodypart2traj is greater than 0 [589]')
                success &= exec_traj_maybesim(bodypart2traj)

            if not success: break


        
        redprint("Segment %s result: %s"%(seg_name, success))
    
    
        #if args.fake_data_segment: break
        

if __name__ == "__main__":
    main()
