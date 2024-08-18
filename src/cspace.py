
class CSpace(object):
    def __init__(self, resolution, origin_x, origin_y, map_shape):
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.rows = map_shape[0]
        self.cols = map_shape[1]
    
    def meter2pixel(self, config):
        if len(config) == 2:
            yaw = 0
        else:
            yaw = config[2]
        x = max(int((config[0] -self.origin_x)/self.resolution), 0)
        x = min(int((config[0] -self.origin_x)/self.resolution), self.cols-1)
        y= max(int((config[1] -self.origin_y)/self.resolution), 0)
        y= min(int((config[1] -self.origin_y)/self.resolution), self.rows-1)
        return [x, y, yaw]

    def pixel2meter(self, config):
        if len(config) == 2:
            yaw = 0
        else:
            yaw = config[2]
        return [(config[0]*self.resolution + self.origin_x), (config[1]*self.resolution + self.origin_y), yaw]

    def pathindex2pathmeter(self, path_idnex):
        path_meter = []
        for coords_index in path_idnex:
            coords_meter = self.pixel2meter(coords_index)
            path_meter.append(coords_meter)
        return path_meter
    
    def pathmeter2pathindex(self, path_meter):
        path_index = []
        for coords_meter in path_meter:
            coords_index = self.meter2pixel(coords_meter)
            path_index.append(coords_index)
        return path_index
