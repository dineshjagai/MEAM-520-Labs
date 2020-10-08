import numpy as np
import json

class OccupancyMap:

    def __init__(self, filepath, radius):
        """
        Initialize Occupancy map. This would create the grid map based on the dimension and resolutions
        defined in the Json file. It would then load the blocks and mark their corresponding grid as True
         as occupied.

        :param filepath: the directory of your world json file.
        """

        # read json file
        # Dimension in Json file are in meters
        filename = filepath
        with open(filename) as file:
            info = json.load(file)
        self.bounds = info['bounds']['extents']
        self.origin = self.bounds[0::2]
        self.resolution = info['resolution']
        self.info = info

        # According to the dimensions of the world, create 2D grid space with the specify resolutions.
        # Grid space would the occupancy map with boolean values. True = occupied. False == unoccupied.
        voxel_metric = []
        voxel_index = []
        for i in range(2):
            voxel_metric.append(np.array(abs(self.bounds[1+i*2]-self.bounds[i*2])))
            voxel_index.append(int(np.ceil(voxel_metric[i]/self.resolution[i])))  # Using ceil to include total world
        self.occ = np.zeros(voxel_index, dtype=bool)

        # Loading blocks into occupancy map.
        self.blocks = self.get_blocks()
        self.InflatedBlocks = self.BlockInflation(self.blocks, radius)

        for block in self.InflatedBlocks:

            # find the voxels for the block
            index_replace = []
            n = list(range(4))
            for i in n[0::2]:
                index_replace.append(int(np.floor(block[i]/self.resolution[int(0)])))
                index_replace.append(int(np.ceil(block[i+1]/self.resolution[int(1)])))
            # update the occupancy map: True to the voxels occupied by the retangular
            self.occ[min(index_replace[0:2]):max(index_replace[0:2])+1, min(index_replace[2:4]):max(index_replace[2:4])+1] = True

    def get_blocks(self):
        """
        Get the blocks dimensions in the Json file as ndarray.

        :return: (N, 4) ndarray
        """
        all_block = []
        for block in self.info['blocks']:
            all_block = all_block + block['extents']

        y_size = len(self.info['blocks'][0]['extents'])

        x_size = int(len(all_block)/y_size)
        all_block = np.array(all_block)

        all_block = all_block.reshape(x_size, y_size)

        return all_block


    def get_bounds(self):
        """
        Get the boundary of the world.

        :return: (4,) ndarray
        """
        return np.array(self.bounds)

    def metric_to_index(self, metric):
        """
        Returns the index of the voxel containing a metric point.
        Remember that this and index_to_metric and not inverses of each other!
        If the metric point lies on a voxel boundary along some coordinate,
        the returned index is the lesser index.
        """
        return np.floor((np.array(metric) - np.array(self.origin))/self.resolution).astype('int')

    def index_to_metric_negative_corner(self,ind):
        """
        Given the index, return the
        :param ind:
        :return:
        """
        return ind*np.array(self.resolution) + np.array(self.origin)

    def index_to_metric_center(self, ind):
        """

        :param ind:
        :return:
        """
        return self.index_to_metric_negative_corner(ind) + np.array(self.resolution)/2.0

    def is_occupied_index(self, ind):
        """
        Check if occupied is occupied.

        :param ind:
        :return:
        """
        return self.occ[tuple(ind)]

    def is_valid_index(self, ind):
        """
        Check if the index is valide or not.

        :param ind:
        :return:
        """
        if ind[0] >= 0 and ind[0] < self.occ.shape[0]:
            if ind[1] >= 0 and ind[1] < self.occ.shape[1]:
                return True
        return False

    def is_valid_metric(self, metric):
        """
        Check it the metric is inside the boundary or not.

        :param metric:
        :return:
        """
        for i in range(3):
            if metric[i] <= self.bounds[i*2] or metric[i] >= self.bounds[i*2+1]:
                return False
        return True

    def BlockInflation(self, block, radius):
        """
        Inflate the Block dimension with the radius of the robot.
        :param block:
        :param radius:
        :return:  ndarray
        """
        n = list(range(int(block.shape[1])))
        for i in n[0::2]:
            block[:, i] = block[:, i] - np.array(radius)
            block[:, i + 1] = block[:, i + 1] + np.array(radius)
        for i in range(len(block[:,1])):
            if block[i][0] < self.bounds[0]:
                block[i][0] = self.bounds[0]
            if block[i][1] > self.bounds[1]:
                block[i][1] = self.bounds[1]
            if block[i][2] < self.bounds[2]:
                block[i][2] = self.bounds[2]
            if block[i][3] > self.bounds[3]:
                block[i][3] = self.bounds[3]
        return block