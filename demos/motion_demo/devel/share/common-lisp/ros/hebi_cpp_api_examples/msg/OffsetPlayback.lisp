; Auto-generated. Do not edit!


(cl:in-package hebi_cpp_api_examples-msg)


;//! \htmlinclude OffsetPlayback.msg.html

(cl:defclass <OffsetPlayback> (roslisp-msg-protocol:ros-message)
  ((offset
    :reader offset
    :initarg :offset
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass OffsetPlayback (<OffsetPlayback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OffsetPlayback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OffsetPlayback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hebi_cpp_api_examples-msg:<OffsetPlayback> is deprecated: use hebi_cpp_api_examples-msg:OffsetPlayback instead.")))

(cl:ensure-generic-function 'offset-val :lambda-list '(m))
(cl:defmethod offset-val ((m <OffsetPlayback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hebi_cpp_api_examples-msg:offset-val is deprecated.  Use hebi_cpp_api_examples-msg:offset instead.")
  (offset m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OffsetPlayback>) ostream)
  "Serializes a message object of type '<OffsetPlayback>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'offset) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OffsetPlayback>) istream)
  "Deserializes a message object of type '<OffsetPlayback>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'offset) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OffsetPlayback>)))
  "Returns string type for a message object of type '<OffsetPlayback>"
  "hebi_cpp_api_examples/OffsetPlayback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OffsetPlayback)))
  "Returns string type for a message object of type 'OffsetPlayback"
  "hebi_cpp_api_examples/OffsetPlayback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OffsetPlayback>)))
  "Returns md5sum for a message object of type '<OffsetPlayback>"
  "de19cca9344eb5bfedb7e55986a47f2e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OffsetPlayback)))
  "Returns md5sum for a message object of type 'OffsetPlayback"
  "de19cca9344eb5bfedb7e55986a47f2e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OffsetPlayback>)))
  "Returns full string definition for message of type '<OffsetPlayback>"
  (cl:format cl:nil "geometry_msgs/Point offset~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OffsetPlayback)))
  "Returns full string definition for message of type 'OffsetPlayback"
  (cl:format cl:nil "geometry_msgs/Point offset~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OffsetPlayback>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'offset))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OffsetPlayback>))
  "Converts a ROS message object to a list"
  (cl:list 'OffsetPlayback
    (cl:cons ':offset (offset msg))
))
