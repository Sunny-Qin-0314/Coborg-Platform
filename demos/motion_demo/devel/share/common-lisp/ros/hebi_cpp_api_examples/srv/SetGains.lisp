; Auto-generated. Do not edit!


(cl:in-package hebi_cpp_api_examples-srv)


;//! \htmlinclude SetGains-request.msg.html

(cl:defclass <SetGains-request> (roslisp-msg-protocol:ros-message)
  ((gains_package
    :reader gains_package
    :initarg :gains_package
    :type cl:string
    :initform "")
   (gains_file
    :reader gains_file
    :initarg :gains_file
    :type cl:string
    :initform ""))
)

(cl:defclass SetGains-request (<SetGains-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetGains-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetGains-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hebi_cpp_api_examples-srv:<SetGains-request> is deprecated: use hebi_cpp_api_examples-srv:SetGains-request instead.")))

(cl:ensure-generic-function 'gains_package-val :lambda-list '(m))
(cl:defmethod gains_package-val ((m <SetGains-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hebi_cpp_api_examples-srv:gains_package-val is deprecated.  Use hebi_cpp_api_examples-srv:gains_package instead.")
  (gains_package m))

(cl:ensure-generic-function 'gains_file-val :lambda-list '(m))
(cl:defmethod gains_file-val ((m <SetGains-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hebi_cpp_api_examples-srv:gains_file-val is deprecated.  Use hebi_cpp_api_examples-srv:gains_file instead.")
  (gains_file m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetGains-request>) ostream)
  "Serializes a message object of type '<SetGains-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'gains_package))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'gains_package))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'gains_file))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'gains_file))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetGains-request>) istream)
  "Deserializes a message object of type '<SetGains-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'gains_package) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'gains_package) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'gains_file) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'gains_file) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetGains-request>)))
  "Returns string type for a service object of type '<SetGains-request>"
  "hebi_cpp_api_examples/SetGainsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetGains-request)))
  "Returns string type for a service object of type 'SetGains-request"
  "hebi_cpp_api_examples/SetGainsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetGains-request>)))
  "Returns md5sum for a message object of type '<SetGains-request>"
  "4fedf26e82cd5de1e91a4ab742d58b2c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetGains-request)))
  "Returns md5sum for a message object of type 'SetGains-request"
  "4fedf26e82cd5de1e91a4ab742d58b2c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetGains-request>)))
  "Returns full string definition for message of type '<SetGains-request>"
  (cl:format cl:nil "string gains_package~%string gains_file~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetGains-request)))
  "Returns full string definition for message of type 'SetGains-request"
  (cl:format cl:nil "string gains_package~%string gains_file~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetGains-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'gains_package))
     4 (cl:length (cl:slot-value msg 'gains_file))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetGains-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetGains-request
    (cl:cons ':gains_package (gains_package msg))
    (cl:cons ':gains_file (gains_file msg))
))
;//! \htmlinclude SetGains-response.msg.html

(cl:defclass <SetGains-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SetGains-response (<SetGains-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetGains-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetGains-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hebi_cpp_api_examples-srv:<SetGains-response> is deprecated: use hebi_cpp_api_examples-srv:SetGains-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetGains-response>) ostream)
  "Serializes a message object of type '<SetGains-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetGains-response>) istream)
  "Deserializes a message object of type '<SetGains-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetGains-response>)))
  "Returns string type for a service object of type '<SetGains-response>"
  "hebi_cpp_api_examples/SetGainsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetGains-response)))
  "Returns string type for a service object of type 'SetGains-response"
  "hebi_cpp_api_examples/SetGainsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetGains-response>)))
  "Returns md5sum for a message object of type '<SetGains-response>"
  "4fedf26e82cd5de1e91a4ab742d58b2c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetGains-response)))
  "Returns md5sum for a message object of type 'SetGains-response"
  "4fedf26e82cd5de1e91a4ab742d58b2c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetGains-response>)))
  "Returns full string definition for message of type '<SetGains-response>"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetGains-response)))
  "Returns full string definition for message of type 'SetGains-response"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetGains-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetGains-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetGains-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetGains)))
  'SetGains-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetGains)))
  'SetGains-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetGains)))
  "Returns string type for a service object of type '<SetGains>"
  "hebi_cpp_api_examples/SetGains")