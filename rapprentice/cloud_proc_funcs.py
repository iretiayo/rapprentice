import cloudprocpy
from rapprentice import berkeley_pr2, clouds
import cv2, numpy as np
import skimage.morphology as skim
import matplotlib
import os.path

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

    if DEBUG_PLOTS:
        cv2.imshow("z0",z0/z0.max())
        cv2.imshow("z",z/z.max())
        cv2.imshow("hue", h_mask.astype('uint8')*255)
        cv2.imshow("sat", s_mask.astype('uint8')*255)
        cv2.imshow("val", v_mask.astype('uint8')*255)
        cv2.imshow("final",good_mask.astype('uint8')*255)
        cv2.imshow("rect", rect_mask.astype('uint8')*255)
        cv2.imshow("rgb", bgr)
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
    #return good_xyz

def extract_red(rgb, depth, T_w_k):
    """
    extract red points and downsample
    """
        
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
  
def grabcut(rgb, depth, T_w_k):
    xyz_k = clouds.depth_to_xyz(depth, berkeley_pr2.f)
    xyz_w = xyz_k.dot(T_w_k[:3,:3].T) + T_w_k[:3,3][None,None,:]

    valid_mask = depth > 0

    import interactive_roi as ir
    xys = ir.get_polyline(rgb, "rgb")
    xy_corner1 = np.clip(np.array(xys).min(axis=0), [0,0], [639,479])
    xy_corner2 = np.clip(np.array(xys).max(axis=0), [0,0], [639,479])
    polymask = ir.mask_from_poly(xys)
    #cv2.imshow("mask",mask)
        
    xy_tl = np.array([xy_corner1, xy_corner2]).min(axis=0)
    xy_br = np.array([xy_corner1, xy_corner2]).max(axis=0)

    xl, yl = xy_tl
    w, h = xy_br - xy_tl
    mask = np.zeros((h,w),dtype='uint8')    
    mask[polymask[yl:yl+h, xl:xl+w] > 0] = cv2.GC_PR_FGD
    print mask.shape
    #mask[h//4:3*h//4, w//4:3*w//4] = cv2.GC_PR_FGD

    tmp1 = np.zeros((1, 13 * 5))
    tmp2 = np.zeros((1, 13 * 5))    
    cv2.grabCut(rgb[yl:yl+h, xl:xl+w, :],mask,(0,0,0,0),tmp1, tmp2,10,mode=cv2.GC_INIT_WITH_MASK)

    mask = mask % 2
    #mask = ndi.binary_erosion(mask, utils_images.disk(args.erode)).astype('uint8')
    contours = cv2.findContours(mask,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)[0]
    cv2.drawContours(rgb[yl:yl+h, xl:xl+w, :],contours,-1,(0,255,0),thickness=2)
    
    cv2.imshow('rgb', rgb)
    print "press enter to continue"
    cv2.waitKey()

    zsel = xyz_w[yl:yl+h, xl:xl+w, 2]
    mask = (mask%2==1) & np.isfinite(zsel)# & (zsel - table_height > -1)
    mask &= valid_mask[yl:yl+h, xl:xl+w]
    
    xyz_sel = xyz_w[yl:yl+h, xl:xl+w,:][mask.astype('bool')]
    return clouds.downsample(xyz_sel, .01)
    #rgb_sel = rgb[yl:yl+h, xl:xl+w,:][mask.astype('bool')]
        


def extract_red_alphashape(cloud, robot):
    """
    extract red, get alpha shape, downsample
    """
    raise NotImplementedError
    
    # downsample cloud
    cloud_ds = cloudprocpy.downsampleCloud(cloud, .01)
    
    # transform into body frame
    xyz1_kinect = cloud_ds.to2dArray()
    xyz1_kinect[:,3] = 1
    T_w_k = berkeley_pr2.get_kinect_transform(robot)
    xyz1_robot = xyz1_kinect.dot(T_w_k.T)
    
    # compute 2D alpha shape
    xyz1_robot_flat = xyz1_robot.copy()
    xyz1_robot_flat[:,2] = 0 # set z coordinates to zero
    xyz1_robot_flatalphashape = cloudprocpy.computeAlphaShape(xyz1_robot_flat)
    
    # unfortunately pcl alpha shape func throws out the indices, so we have to use nearest neighbor search
    cloud_robot_flatalphashape = cloudprocpy.CloudXYZ()
    cloud_robot_flatalphashape.from2dArray(xyz1_robot_flatalphashape)
    cloud_robot_flat = cloudprocpy.CloudXYZ()
    cloud_robot_flat.from2dArray(xyz1_robot_flat)
    alpha_inds = cloudprocpy.getNearestNeighborIndices(xyz1_robot_flatalphashape, xyz1_robot_flat)

    xyz_robot_alphashape = xyz1_robot_flatalphashape[:,:3]
    
    # put back z coordinate
    xyz_robot_alphashape[:,2] = xyz1_robot[alpha_inds,2] 

    return xyz_robot_alphashape[:,:3]
