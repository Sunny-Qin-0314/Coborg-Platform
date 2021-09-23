; Auto-generated. Do not edit!


(cl:in-package coborg_move-msg)


;//! \htmlinclude CartesianTrajectory.msg.html

(cl:defclass <CartesianTrajectory> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (origin
    :reader origin
    :initarg :origin
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (goal
    :reader goal
    :initarg :goal
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass CartesianTrajectory (<CartesianTrajectory>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CartesianTrajectory>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CartesianTrajectory)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name coborg_move-msg:<CartesianTrajectory> is deprecated: use coborg_move-msg:CartesianTrajectory instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <CartesianTrajectory>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coborg_move-msg:header-val is deprecated.  Use coborg_move-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'origin-val :lambda-list '(m))
(cl:defmethod origin-val ((m <CartesianTrajectory>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coborg_move-msg:origin-val is deprecated.  Use coborg_move-msg:origin instead.")
  (origin m))

(cl:ensure-generic-function 'goal-val :lambda-list '(m))
(cl:defmethod goal-val ((m <CartesianTrajectory>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coborg_move-msg:goal-val is deprecated.  Use coborg_move-msg:goal instead.")
  (goal m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CartesianTrajectory>) ostream)
  "Serializes a message object of type '<CartesianTrajectory>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'origin) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goal) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CartesianTrajectory>) istream)
  "Deserializes a message object of type '<CartesianTrajectory>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'origin) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goal) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CartesianTrajectory>)))
  "Returns string type for a message object of type '<CartesianTrajectory>"
  "coborg_move/CartesianTrajectory")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CartesianTrajectory)))
  "Returns string type for a message object of type 'CartesianTrajectory"
  "coborg_move/CartesianTrajectory")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CartesianTrajectory>)))
  "Returns md5sum for a message object of type '<CartesianTrajectory>"
  "e4f7ea05e9c61043107bec8842a0fd90")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CartesianTrajectory)))
  "Returns md5sum for a message object of type 'CartesianTrajectory"
  "e4f7ea05e9c61043107bec8842a0fd90")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CartesianTrajectory>)))
  "Returns full string definition for message of type '<CartesianTrajectory>"
  (cl:format cl:nil "std_msgs/Header header~%geometry_msgs/Pose origin~%geometry_msgs/Pose goal~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CartesianTrajectory)))
  "Returns full string definition for message of type 'CartesianTrajectory"
  (cl:format cl:nil "std_msgs/Header header~%geometry_msgs/Pose origin~%geometry_msgs/Pose goal~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CartesianTrajectory>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'origin))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goal))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CartesianTrajectory>))
  "Converts a ROS message object to a list"
  (cl:list 'CartesianTrajectory
    (cl:cons ':header (header msg))
    (cl:cons ':origin (origin msg))
    (cl:cons ':goal (goal msg))
))
