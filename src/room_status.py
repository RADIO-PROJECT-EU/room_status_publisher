#!/usr/bin/python
import rospy
import paho.mqtt.client as mqtt
from room_status_publisher.msg import RoomStatusMsg

ros_pub = None
stopic = ""

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    global stopic
    print("Connected with result code "+str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe(stopic)

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global ros_pub
    print msg.payload

    #m = RoomStatusMsg()
    #m.header.stamp = rospy.Time.now()
    #m.name = msg.name
    #m.things = mgs.things
    #ros_pub.publish(m)
    #print(msg.topic+" "+str(msg.payload))




def init():
    global ros_pub, stopic
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    client.username_pw_set("application",password="@PpL3c@tI0n")

    client.connect("172.21.13.170", 1883, 60)
    rospy.init_node('room_status_publisher')
    stopic = rospy.get_param("sub_topic","twg/relative/room_status_mqtt")
    ptopic = rospy.get_param("pub_topic","room_status_publisher/room_status")

    ros_pub = rospy.Publisher(ptopic, RoomStatusMsg, queue_size=1)
    print "node init"

    # Blocking call that processes network traffic, dispatches callbacks and
    # handles reconnecting.
    # Other loop*() functions are available that give a threaded interface and a
    # manual interface.
    while not rospy.is_shutdown():
        client.loop()


if __name__ == '__main__':
    init()