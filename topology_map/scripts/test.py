import tf
import rospy

if __name__ == '__main__':
    rospy.init_node('test', anonymous=False)
    listener = tf.TransformListener()
    listener.waitForTransform("/map", "/base_link", rospy.Time(), rospy.Duration(4.0))
    trans, rot = listener.lookupTransform("/map", "/base_link", rospy.Time(0))
    point_in_map_frame = (1, 0, 0)
    point_in_base_link_frame = tf.transformations.quaternion_transform(point_in_map_frame, trans, rot)
    point_in_base_link_frame = listener.transformPoint("/base_link", point_in_map_frame)
