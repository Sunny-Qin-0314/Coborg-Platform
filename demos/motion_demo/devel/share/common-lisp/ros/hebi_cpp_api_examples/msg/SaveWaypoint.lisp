; Auto-generated. Do not edit!


(cl:in-package hebi_cpp_api_examples-msg)


;//! \htmlinclude SaveWaypoint.msg.html

(cl:defclass <SaveWaypoint> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform ""))
)

(cl:defclass SaveWaypoint (<SaveWaypoint>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SaveWaypoint>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SaveWaypoint)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hebi_cpp_api_examples-msg:<SaveWaypoint> is deprecated: use hebi_cpp_api_examples-msg:SaveWaypoint instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <SaveWaypoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hebi_cpp_api_examples-msg:name-val is deprecated.  Use hebi_cpp_api_examples-msg:name instead.")
  (name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SaveWaypoint>) ostream)
  "Serializes a message object of type '<SaveWaypoint>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SaveWaypoint>) istream)
  "Deserializes a message object of type '<SaveWaypoint>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SaveWaypoint>)))
  "Returns string type for a message object of type '<SaveWaypoint>"
  "hebi_cpp_api_examples/SaveWaypoint")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SaveWaypoint)))
  "Returns string type for a message object of type 'SaveWaypoint"
  "hebi_cpp_api_examples/SaveWaypoint")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SaveWaypoint>)))
  "Returns md5sum for a message object of type '<SaveWaypoint>"
  "c1f3d28f1b044c871e6eff2e9fc3c667")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SaveWaypoint)))
  "Returns md5sum for a message object of type 'SaveWaypoint"
  "c1f3d28f1b044c871e6eff2e9fc3c667")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SaveWaypoint>)))
  "Returns full string definition for message of type '<SaveWaypoint>"
  (cl:format cl:nil "string name~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SaveWaypoint)))
  "Returns full string definition for message of type 'SaveWaypoint"
  (cl:format cl:nil "string name~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SaveWaypoint>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SaveWaypoint>))
  "Converts a ROS message object to a list"
  (cl:list 'SaveWaypoint
    (cl:cons ':name (name msg))
))
