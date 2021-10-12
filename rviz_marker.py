import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped,Point

rospy.init_node("marker_pub")

pub = rospy.Publisher("arrow_pub", Marker, queue_size = 10)
rate = rospy.Rate(25)

w=0
x = 0.0

marker_data = Marker()
marker_data.points = []
while not rospy.is_shutdown():

    marker_data.header.frame_id = "map"
    marker_data.header.stamp = rospy.Time.now()
    
    marker_data.ns = "basic_shapes"
    marker_data.id = 0

    marker_data.action = Marker.ADD

    x += 0.01
    marker_data.pose.position.x = x
    marker_data.pose.position.y = 0.0
    marker_data.pose.position.z = 0.0

    #add_point = Point()
    #add_point.x = x
    #add_point.y = 1.0
    #add_point.z = 1.0

    #marker_data.points.append(add_point)

    marker_data.pose.orientation.x=0.0
    marker_data.pose.orientation.y=0.0
    marker_data.pose.orientation.z=1.0
    marker_data.pose.orientation.w=0.0

    marker_data.color.r = 1.0
    marker_data.color.g = 1.0
    marker_data.color.b = 0.0
    marker_data.color.a = 1.0

    marker_data.scale.x = 0.1
    marker_data.scale.y = 0.1
    marker_data.scale.z = 0.1

    marker_data.lifetime = rospy.Duration()

    marker_data.type = 1

    pub.publish(marker_data)

    rate.sleep()
