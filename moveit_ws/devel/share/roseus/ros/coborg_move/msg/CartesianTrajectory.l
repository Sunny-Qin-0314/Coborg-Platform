;; Auto-generated. Do not edit!


(when (boundp 'coborg_move::CartesianTrajectory)
  (if (not (find-package "COBORG_MOVE"))
    (make-package "COBORG_MOVE"))
  (shadow 'CartesianTrajectory (find-package "COBORG_MOVE")))
(unless (find-package "COBORG_MOVE::CARTESIANTRAJECTORY")
  (make-package "COBORG_MOVE::CARTESIANTRAJECTORY"))

(in-package "ROS")
;;//! \htmlinclude CartesianTrajectory.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass coborg_move::CartesianTrajectory
  :super ros::object
  :slots (_header _origin _goal ))

(defmethod coborg_move::CartesianTrajectory
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:origin __origin) (instance geometry_msgs::Pose :init))
    ((:goal __goal) (instance geometry_msgs::Pose :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _origin __origin)
   (setq _goal __goal)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:origin
   (&rest __origin)
   (if (keywordp (car __origin))
       (send* _origin __origin)
     (progn
       (if __origin (setq _origin (car __origin)))
       _origin)))
  (:goal
   (&rest __goal)
   (if (keywordp (car __goal))
       (send* _goal __goal)
     (progn
       (if __goal (setq _goal (car __goal)))
       _goal)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Pose _origin
    (send _origin :serialization-length)
    ;; geometry_msgs/Pose _goal
    (send _goal :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Pose _origin
       (send _origin :serialize s)
     ;; geometry_msgs/Pose _goal
       (send _goal :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Pose _origin
     (send _origin :deserialize buf ptr-) (incf ptr- (send _origin :serialization-length))
   ;; geometry_msgs/Pose _goal
     (send _goal :deserialize buf ptr-) (incf ptr- (send _goal :serialization-length))
   ;;
   self)
  )

(setf (get coborg_move::CartesianTrajectory :md5sum-) "e4f7ea05e9c61043107bec8842a0fd90")
(setf (get coborg_move::CartesianTrajectory :datatype-) "coborg_move/CartesianTrajectory")
(setf (get coborg_move::CartesianTrajectory :definition-)
      "std_msgs/Header header
geometry_msgs/Pose origin
geometry_msgs/Pose goal
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :coborg_move/CartesianTrajectory "e4f7ea05e9c61043107bec8842a0fd90")


