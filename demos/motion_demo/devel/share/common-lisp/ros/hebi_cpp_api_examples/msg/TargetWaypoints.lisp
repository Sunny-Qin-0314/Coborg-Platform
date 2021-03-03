; Auto-generated. Do not edit!


(cl:in-package hebi_cpp_api_examples-msg)


;//! \htmlinclude TargetWaypoints.msg.html

(cl:defclass <TargetWaypoints> (roslisp-msg-protocol:ros-message)
  ((waypoints_vector
    :reader waypoints_vector
    :initarg :waypoints_vector
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point))))
)

(cl:defclass TargetWaypoints (<TargetWaypoints>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TargetWaypoints>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TargetWaypoints)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hebi_cpp_api_examples-msg:<TargetWaypoints> is deprecated: use hebi_cpp_api_examples-msg:TargetWaypoints instead.")))

(cl:ensure-generic-function 'waypoints_vector-val :lambda-list '(m))
(cl:defmethod waypoints_vector-val ((m <TargetWaypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hebi_cpp_api_examples-msg:waypoints_vector-val is deprecated.  Use hebi_cpp_api_examples-msg:waypoints_vector instead.")
  (waypoints_vector m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TargetWaypoints>) ostream)
  "Serializes a message object of type '<TargetWaypoints>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'waypoints_vector))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'waypoints_vector))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TargetWaypoints>) istream)
  "Deserializes a message object of type '<TargetWaypoints>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'waypoints_vector) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'waypoints_vector)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TargetWaypoints>)))
  "Returns string type for a message object of type '<TargetWaypoints>"
  "hebi_cpp_api_examples/TargetWaypoints")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TargetWaypoints)))
  "Returns string type for a message object of type 'TargetWaypoints"
  "hebi_cpp_api_examples/TargetWaypoints")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TargetWaypoints>)))
  "Returns md5sum for a message object of type '<TargetWaypoints>"
  "aa0b6e1fb814653675dd4da6b4a51d95")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TargetWaypoints)))
  "Returns md5sum for a message object of type 'TargetWaypoints"
  "aa0b6e1fb814653675dd4da6b4a51d95")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TargetWaypoints>)))
  "Returns full string definition for message of type '<TargetWaypoints>"
  (cl:format cl:nil "geometry_msgs/Point[] waypoints_vector~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TargetWaypoints)))
  "Returns full string definition for message of type 'TargetWaypoints"
  (cl:format cl:nil "geometry_msgs/Point[] waypoints_vector~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TargetWaypoints>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'waypoints_vector) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TargetWaypoints>))
  "Converts a ROS message object to a list"
  (cl:list 'TargetWaypoints
    (cl:cons ':waypoints_vector (waypoints_vector msg))
))
