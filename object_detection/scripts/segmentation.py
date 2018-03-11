import numpy as np 
import time
from connected import *
from occupancy_grid import *
from numpy import linalg as LA
import argparse
from scan_conversion import *

'''
This function can be used to obtain segments from 3D point cloud data.
'''

def segmentation(cloud_points):
    # finding out the occupancy grid
    occ_grid,point_cloud = occupancy_grid(cloud_points)
    res_grid = np.zeros((occ_grid.shape[0], occ_grid.shape[1]))
    res_grid[occ_grid >= 0.15] = 1
    res_grid[occ_grid < 0.15] = 0
    # obtaining connected components
    conn = Connected(res_grid)
    res = conn.connected_components()
    # selecting components which have more than 10 elements in them
    selected_labels = conn.refining_comps()
    res_z_vals = conn.getting_max_z(selected_labels, occ_grid)
    
    bbox_3d_all = []
    obj_points_all = []
    # Applying eigen decomposition to each component
    for i in selected_labels:
        indexes = np.where(res == i)
        cck = np.zeros((2, len(indexes[0])))
        cck_old = np.zeros((2, len(indexes[0])))
        mod_cck = conn.number_in_comps[i]
        if mod_cck >= 20:
            cck_old[0,:] = indexes[0] 
            cck_old[1,:] = indexes[1]
            cck[0,:] = indexes[0] - np.mean(indexes[0])
            cck[1,:] = indexes[1] - np.mean(indexes[1])
            
            cck_trans = cck.transpose()
            # computing co-variance matrix
            cov_mat = np.matmul(cck , cck_trans)/mod_cck
            e_vals, e_vecs = LA.eigh(cov_mat)
            
            new_eigen_vecs = np.column_stack((e_vecs[:,0], e_vecs[:,1]))
        
            # calculating the new coordinates
            new_coords = np.matmul(new_eigen_vecs, cck_old)
            x_min = np.min(new_coords[0,:])
            x_max = np.max(new_coords[0,:])
            y_min = np.min(new_coords[1,:])
            y_max = np.max(new_coords[1,:])
            
            # dimenstions of the box enclosing the segments in the point clouds.
            width = (x_max - x_min)/2
            height = (y_max - y_min)/2
            
            # center of gravity of each component.
            x_bar = np.sum(new_coords[0,:])/mod_cck
            y_bar = np.sum(new_coords[1,:])/mod_cck
        
            z_value = res_z_vals[i]
            
            # Rotating from ego to grid coordinates
            twod_points = np.array([[x_bar-width, y_bar-height],[x_bar+width, y_bar-height],
                                    [x_bar+width, y_bar+height],[x_bar-width, y_bar+height]])
            
            new_points = np.matmul(new_eigen_vecs.transpose(), twod_points.transpose()) 
            points = np.concatenate((new_points.transpose(), np.zeros((4,1))), axis=1)
            points =  np.vstack((points, points))
            points[4:,2] = z_value
        
            # getting point cloud data from the occupancy grid values
            point_x1 =  ((np.min(points[:,0]) - 334) *15/100)
            point_x2 = ((np.max(points[:,0]) - 334) * 15/100)
            point_y1 = ((np.min(points[:,1]) - 334) * 15/100)
            point_y2 = ((np.max(points[:,1]) - 334) * 15/100)
            box_3d = point_cloud[(point_cloud[:,0]  >= point_x1) & 
                               (point_cloud[:,0]  <= point_x2) &
                               (point_cloud[:,1]  >= point_y1) &
                               (point_cloud[:,1]  <= point_y2)]
            raw_points = ((points[:4, :2] - 334) * 15)/100
            # refining the point cloud pertaining to a component.
            object_points = scan_conversion(raw_points, box_3d)
            bbox_3d_all.append(raw_points)
            obj_points_all.append(object_points)
    return bbox_3d_all, obj_points_all

def plotPoints(points, ax):
    scaling = 0.2
    points[4:8,1] = points[4:8,1] + (points[4:8,2]/scaling)
    points[4:8,0] = points[4:8,0] + (points[4:8,2]/scaling)
    points[4:8,:] = np.flipud(points[4:8,:])
    ax.plot(points[:,0],points[:,1],'r-')
    ax.plot([points[1,0],points[6,0]],[points[1,1],points[6,1]], 'r-')
    ax.plot([points[0,0],points[3,0]],[points[0,1],points[3,1]], 'r-')
    ax.plot([points[4,0],points[7,0]],[points[4,1],points[7,1]], 'r-')
    ax.plot([points[7,0],points[0,0]],[points[7,1],points[0,1]], 'r-')
    ax.plot([points[2,0],points[5,0]],[points[2,1],points[5,1]], 'r-')
    return(ax)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('source', nargs='+', help='list of frames to process')
    args = parser.parse_args()
    
    segmentation(args.source)
    return


if __name__ == '__main__':
    main()

