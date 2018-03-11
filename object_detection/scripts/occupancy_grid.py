import numpy as np 


def occupancy_grid(cloud_points):
    # reading the binary file to a numpy array
    orig_data = np.asarray(cloud_points)
    point_cloud = orig_data.reshape((-1, 4))

    # Creating an empty occupancy grid
    occupancy_grid = np.zeros((668,668))

    # Converting the m to cms and finding out the appropriate box to place  a particular x,y value
    conversion = lambda in_val: np.floor((in_val*100)/15)

    new_x = [conversion(x) for x in point_cloud[:,0]]
    new_y = [conversion(y) for y in point_cloud[:,1]]

    # Data structures to hold the new x and y values to feed into the occupancy grid
    temp = np.zeros((len(new_x), 2))
    temp[:,0] = new_x
    temp[:,1] = new_y    

    temp1 = np.zeros((len(new_x), 3))
    temp1[:,0] = new_x
    temp1[:,1] = new_y
    temp1[:,2] = point_cloud[:,2]


    # Finding unique x,y values
    unique_rows = np.unique(temp, axis=0)

    # filling the occupancy grid with the maximum absolute difference in z values
    for i in range(len(unique_rows)):
        collect_z = np.where((temp[:,0] == unique_rows[i,0]) & (temp[:,1] == unique_rows[i,1]))
        c = temp1[collect_z[0],2]
        if len(c) == 1:
            max_abs_dist = np.abs(c)
        else:
            max_abs_dist = np.abs(np.max(c) - np.min(c))
        translated_coord = unique_rows[i,:]+334
        translated_coord = translated_coord.astype(np.int64)
        if translated_coord[0] >= 0 and translated_coord[0] < 668 and translated_coord[1] >= 0 and translated_coord[1] < 668:
            occupancy_grid[translated_coord[0],translated_coord[1]] = max_abs_dist
        
    return occupancy_grid,point_cloud


