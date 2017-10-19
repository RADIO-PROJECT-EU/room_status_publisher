#!/usr/bin/python
import json
import rospy
import paho.mqtt.client as mqtt
from room_status_publisher.msg import RoomStatusMsg
from std_msgs.msg import Bool

stopic = ""
tv_pub = None
ros_pub = None
chair_tv_location = ""
chair_object_name = ""

def on_connect(client, userdata, flags, rc):
    global stopic
    print("Connected with result code " + str(rc))

    print client.subscribe(stopic, 0)
    print "Subscribing to " + stopic

def on_subscribe(mosq, obj, mid, granted_qos):
    print "Subscription successful!"

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global ros_pub, tv_pub, chair_object_name, chair_tv_location
    j = json.loads(msg.payload)
    b = Bool()
    if j["object"] == chair_object_name and j["location"] == chair_tv_location:
        b.data = True
    else:
        b.data = False

    tv_pub.publish(b)

    #m = RoomStatusMsg()
    #m.header.stamp = rospy.Time.now()
    #m.name = msg.name
    #m.things = mgs.things
    #ros_pub.publish(m)
    #print(msg.topic+" "+str(msg.payload))


def init():
    global ros_pub, stopic, tv_pub, chair_tv_location, chair_object_name
    # ROS stuff
    rospy.init_node('room_status_publisher')
    stopic = rospy.get_param("~sub_topic","apps/localization/relative")
    #ptopic = rospy.get_param("pub_topic","~room_status")
    tvtopic = rospy.get_param("~tv_topic","~tv_chair")
    chair_object_name = rospy.get_param("~chair_object_name","chair")
    chair_tv_location = rospy.get_param("~chair_tv_location","tv")

    #ros_pub = rospy.Publisher(ptopic, RoomStatusMsg, queue_size=1)

    tv_pub = rospy.Publisher(tvtopic, Bool, queue_size=1)

    # MQTT stuff
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_subscribe = on_subscribe
    client.on_message = on_message

    #client.username_pw_set("application",password="@PpL3c@tI0n")
    #client.connect("150.140.187.126", 1883, 60)
    client.connect("localhost", 1883, 60)

    # Blocking call that processes network traffic, dispatches callbacks and
    # handles reconnecting.
    # Other loop*() functions are available that give a threaded interface and a
    # manual interface.
    while not rospy.is_shutdown():
        client.loop()


if __name__ == '__main__':
    init()