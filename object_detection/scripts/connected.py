import numpy as np
import time


class Connected:
    # Initializing input data structures 
    def __init__(self, input_matrix):
        self.input_matrix = input_matrix
        self.row = input_matrix.shape[0]
        self.col = input_matrix.shape[1]
        self.direction = np.zeros((self.row, self.col)) # matrix which holds the output 
        self.neighbors_queue = []
        self.label = 0
        self.number_in_comps = {}
        self.max_z = {}

    # traversing the 8 neighborhood of an element.
    # loca is a list consisting of the row and column of the location for which 8-neighborhood has to be calculated.
    def get_neighbors(self, loca):
        for i in range(-1,2):
            for j in range(-1,2):
                if (loca[0]+i >= 0) and (loca[0]+i < (self.row)) and (loca[1]+j < (self.col)) and (loca[1]+j >= 0):
                    if (self.input_matrix[(loca[0]+i), (loca[1]+j)] ==1) and (self.direction[(loca[0]+i), (loca[1]+j)] == 0): 
                        if [loca[0]+i, loca[1]+j] not in self.neighbors_queue:
                            self.neighbors_queue.append([loca[0]+i, loca[1]+j])    


    # Function to calculate the component locations and label them differently to distinguish them.
    def connected_components(self):
        for r in range(self.row):
            for c in range(self.col):
                if self.direction[r,c] == 0 and self.input_matrix[r, c] == 1: 
                    self.label = self.label+1 # generating new label when we come across a new component
                    self.direction[r,c] = self.label
                    self.number_in_comps[self.label] = 0
                    self.number_in_comps[self.label] = self.number_in_comps[self.label] + 1
                    self.get_neighbors([r,c])
                    while(len(self.neighbors_queue) > 0): # Looping to find out all the elements in a component.
                        loc_val = self.neighbors_queue.pop()
                        self.direction[loc_val[0], loc_val[1]] = self.label
                        self.number_in_comps[self.label] = self.number_in_comps[self.label] + 1
                        self.get_neighbors(loc_val)
        return self.direction

    # This function can be called from a connected object after 
    # connected_components function has already been called to get the number of components found
    def number_components(self):
        return self.label

    # This function can be called from a connected object after 
    # connected_components function has already been called to refine the number of components found
    def refining_comps(self):
        f = []
        for i in range(self.number_components()):
            if(self.number_in_comps[i+1] > 10):
                f.append(i+1)
#        print(len(f))
        return f
    
    # This function can be called from a connected object after 
    # connected_components function has already been called to find out 
    # the maximum value in the cells of the component
    def getting_max_z(self, label_data, grid_struct):
        for i in label_data:
            indexes = np.where(self.direction == i)
            temp = np.max(grid_struct[indexes[0], indexes[1]])
            self.max_z[i] = temp
        return self.max_z
    

# g = np.random.randint(0,2, size=(10,10))
# print(g)
# c = Connected(g)
# res = c.connected_components()
# print(res)