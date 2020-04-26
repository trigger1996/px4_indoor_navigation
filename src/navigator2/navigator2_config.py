
navigator2_config = {
    'is_use_bresenham' :              True,         # The Accuracy of VSLAM need to be improved, or this won't be of good use indoor

    'is_remove_collinear_pts' :       True,

    'is_rotate_uav' :                 True,

    'pathpruning_obstacle_dst' :      4,            # in grids

    'pathpruning_resolution' :        0.2,          # in meters

    # for debugging
    'disable_waiting_time' :          False,

    'enable_publishing_test_map' :    True
}
