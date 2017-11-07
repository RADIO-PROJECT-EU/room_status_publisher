#!/usr/bin/python
import json
import rospy
from std_msgs.msg import Bool
import paho.mqtt.client as mqtt
from geometry_msgs.msg import PoseStamped
from room_status_publisher.msg import RoomStatusMsg

stopic = ""
s2topic = ""
tv_pub = None
ros_pub = None
keys_goal_pub = None
chair_tv_location = ""
chair_object_name = ""

def on_connect(client, userdata, flags, rc):
    global stopic, s2topic
    print("Connected with result code " + str(rc))

    print client.subscribe(stopic, 0)
    print "Subscribing to " + stopic

    print client.subscribe(s2topic, 0)
    print "Subscribing to " + s2topic

def on_subscribe(mosq, obj, mid, granted_qos):
    print "Subscription successful!"

def tv_callback(client, userdata, msg):
    global ros_pub, tv_pub, chair_object_name, chair_tv_location
    j = json.loads(msg.payload)
    b = Bool()
    if j["object"].lower() == chair_object_name and j["location"].lower() == chair_tv_location:
        b.data = True
    else:
        b.data = False

    tv_pub.publish(b)

def keys_callback(client, userdata, msg):
    global keys_goal_pub, keys_location1, keys_location2
    global location1x, location1y, location1z, location1w
    global location2x, location2y, location2z, location2w
    j = json.loads(msg.payload)
    ps = PoseStamped()
    ps.header.stamp = rospy.Time.now()
    ps.header.frame_id = 'map'
    if j["object"].lower() == keys_object_name and j["location"].lower() == keys_location1:
        ps.pose.position.x = location1x
        ps.pose.position.y = location1y
        ps.pose.orientation.z = location1z
        ps.pose.orientation.w = location1w
    elif j["object"].lower() == keys_object_name and j["location"].lower() == keys_location2:
        ps.pose.position.x = location2x
        ps.pose.position.y = location2y
        ps.pose.orientation.z = location2z
        ps.pose.orientation.w = location2w
    keys_goal_pub.publish(ps)


def init():
    global ros_pub, stopic, tv_pub, chair_tv_location, chair_object_name
    global s2topic, keys_goal_pub
    global keys_object_name, keys_location1, keys_location2
    global location1x, location1y, location1z, location1w
    global location2x, location2y, location2z, location2w

    # ROS stuff
    rospy.init_node('room_status_publisher')
    stopic = rospy.get_param("~tv_topic","apps/localization/relative")
    s2topic = rospy.get_param("~keys_topic","apps/localization/keys")
    ptopic = rospy.get_param("~pub_topic","~room_status")
    tvtopic = rospy.get_param("~tv_pub_topic","~tv_chair")
    keystopic = rospy.get_param("~keys_pub_topic","~keys")
    chair_object_name = rospy.get_param("~chair_object_name", "chair")
    chair_tv_location = rospy.get_param("~chair_tv_location", "tv")
    keys_object_name = rospy.get_param("~keys_object_name", "keys")
    keys_location1 = rospy.get_param("~keys_location1", "living_room")
    keys_location2 = rospy.get_param("~keys_location2", "bedroom")
    chair_object_name = chair_object_name.lower()
    chair_tv_location = chair_tv_location.lower()
    keys_object_name = keys_object_name.lower()
    keys_location1 = keys_location1.lower()
    keys_location2 = keys_location2.lower()
    location1x = rospy.get_param("~location1x", 0.0)
    location1y = rospy.get_param("~location1y", 0.0)
    location1z = rospy.get_param("~location1z", 0.0)
    location1w = rospy.get_param("~location1w", 0.0)
    location2x = rospy.get_param("~location2x", 0.0)
    location2y = rospy.get_param("~location2y", 0.0)
    location2z = rospy.get_param("~location2z", 0.0)
    location2w = rospy.get_param("~location2w", 0.0)
    ip = rospy.get_param("~ip", "150.140.187.125")
    port = rospy.get_param("~port", 1883)
    timeout = rospy.get_param("~timeout", 60)
    username = rospy.get_param("~username", "application")
    password = rospy.get_param("~password", "@PpL3c@tI0n")

    #ros_pub = rospy.Publisher(ptopic, RoomStatusMsg, queue_size=1)

    tv_pub = rospy.Publisher(tvtopic, Bool, queue_size=1)
    keys_goal_pub = rospy.Publisher(keystopic, PoseStamped, queue_size=1)

    # MQTT stuff
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_subscribe = on_subscribe
    client.message_callback_add(stopic, tv_callback)
    client.message_callback_add(s2topic, keys_callback)
    #client.on_message = on_message

    client.username_pw_set(username, password=password)
    client.connect(ip, port, timeout)

    # Blocking call that processes network traffic, dispatches callbacks and
    # handles reconnecting.
    # Other loop*() functions are available that give a threaded interface and a
    # manual interface.
    while not rospy.is_shutdown():
        client.loop()


if __name__ == '__main__':
    init()