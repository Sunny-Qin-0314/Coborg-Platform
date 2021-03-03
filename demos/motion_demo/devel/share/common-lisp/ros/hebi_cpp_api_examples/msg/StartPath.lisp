; Auto-generated. Do not edit!


(cl:in-package hebi_cpp_api_examples-msg)


;//! \htmlinclude StartPath.msg.html

(cl:defclass <StartPath> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass StartPath (<StartPath>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartPath>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartPath)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hebi_cpp_api_examples-msg:<StartPath> is deprecated: use hebi_cpp_api_examples-msg:StartPath instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartPath>) ostream)
  "Serializes a message object of type '<StartPath>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartPath>) istream)
  "Deserializes a message object of type '<StartPath>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StartPath>)))
  "Returns string type for a message object of type '<StartPath>"
  "hebi_cpp_api_examples/StartPath")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartPath)))
  "Returns string type for a message object of type 'StartPath"
  "hebi_cpp_api_examples/StartPath")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StartPath>)))
  "Returns md5sum for a message object of type '<StartPath>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartPath)))
  "Returns md5sum for a message object of type 'StartPath"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartPath>)))
  "Returns full string definition for message of type '<StartPath>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartPath)))
  "Returns full string definition for message of type 'StartPath"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartPath>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartPath>))
  "Converts a ROS message object to a list"
  (cl:list 'StartPath
))
