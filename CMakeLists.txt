cmake_minimum_required(VERSION 2.8.3)
project(room_status_publisher)

find_package(catkin REQUIRED message_generation std_msgs)

add_message_files(
   FILES
   RoomStatusMsg.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package()

