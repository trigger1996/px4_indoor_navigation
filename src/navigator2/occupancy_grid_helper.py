import math
from nav_msgs.msg import OccupancyGrid

# https://blog.csdn.net/yzxnuaa/article/details/79626405
# https://blog.csdn.net/qq826309057/article/details/76473960

def resize(map_in, target_resolution):
    fx = 1 / (target_resolution / map_in.info.resolution)
    fy = 1 / (target_resolution / map_in.info.resolution)

    map_out = OccupancyGrid()
    map_out.info.resolution = target_resolution
    map_out.info.width  = int(math.floor(map_in.info.width * fx))
    map_out.info.height = int(math.floor(map_in.info.height * fy))
    map_out.info.origin.position.x = map_in.info.origin.position.x
    map_out.info.origin.position.y = map_in.info.origin.position.y
    map_out.info.origin.position.z = map_in.info.origin.position.z

    for y in range(0, map_out.info.height):
        for x in range(0, map_out.info.width):
            map_out.data.append(-1)

    for y in range(0, map_out.info.height):
        srcy = int(math.floor(y / fy))
        srcy = int(min(srcy, map_in.info.height - 1))
        for x in range(0, map_out.info.width):
            srcx = int(math.floor(x / fx))
            srcx = int(min(srcx, map_in.info.width - 1))

            map_out.data[y * map_out.info.width + x] = map_in.data[srcy * map_in.info.width + srcx]
            if map_out.data[y * map_out.info.width + x] != -1:
                iiiii = 1

    return map_out




