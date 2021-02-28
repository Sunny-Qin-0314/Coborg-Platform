; Auto-generated. Do not edit!


(cl:in-package hebi_cpp_api_examples-srv)


;//! \htmlinclude SetIKSeed-request.msg.html

(cl:defclass <SetIKSeed-request> (roslisp-msg-protocol:ros-message)
  ((seed
    :reader seed
    :initarg :seed
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass SetIKSeed-request (<SetIKSeed-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetIKSeed-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetIKSeed-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hebi_cpp_api_examples-srv:<SetIKSeed-request> is deprecated: use hebi_cpp_api_examples-srv:SetIKSeed-request instead.")))

(cl:ensure-generic-function 'seed-val :lambda-list '(m))
(cl:defmethod seed-val ((m <SetIKSeed-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hebi_cpp_api_examples-srv:seed-val is deprecated.  Use hebi_cpp_api_examples-srv:seed instead.")
  (seed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetIKSeed-request>) ostream)
  "Serializes a message object of type '<SetIKSeed-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'seed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'seed))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetIKSeed-request>) istream)
  "Deserializes a message object of type '<SetIKSeed-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'seed) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'seed)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetIKSeed-request>)))
  "Returns string type for a service object of type '<SetIKSeed-request>"
  "hebi_cpp_api_examples/SetIKSeedRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetIKSeed-request)))
  "Returns string type for a service object of type 'SetIKSeed-request"
  "hebi_cpp_api_examples/SetIKSeedRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetIKSeed-request>)))
  "Returns md5sum for a message object of type '<SetIKSeed-request>"
  "45f70114af10b3f5f5e2b664d72ca331")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetIKSeed-request)))
  "Returns md5sum for a message object of type 'SetIKSeed-request"
  "45f70114af10b3f5f5e2b664d72ca331")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetIKSeed-request>)))
  "Returns full string definition for message of type '<SetIKSeed-request>"
  (cl:format cl:nil "float64[] seed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetIKSeed-request)))
  "Returns full string definition for message of type 'SetIKSeed-request"
  (cl:format cl:nil "float64[] seed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetIKSeed-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'seed) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetIKSeed-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetIKSeed-request
    (cl:cons ':seed (seed msg))
))
;//! \htmlinclude SetIKSeed-response.msg.html

(cl:defclass <SetIKSeed-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SetIKSeed-response (<SetIKSeed-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetIKSeed-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetIKSeed-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hebi_cpp_api_examples-srv:<SetIKSeed-response> is deprecated: use hebi_cpp_api_examples-srv:SetIKSeed-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetIKSeed-response>) ostream)
  "Serializes a message object of type '<SetIKSeed-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetIKSeed-response>) istream)
  "Deserializes a message object of type '<SetIKSeed-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetIKSeed-response>)))
  "Returns string type for a service object of type '<SetIKSeed-response>"
  "hebi_cpp_api_examples/SetIKSeedResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetIKSeed-response)))
  "Returns string type for a service object of type 'SetIKSeed-response"
  "hebi_cpp_api_examples/SetIKSeedResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetIKSeed-response>)))
  "Returns md5sum for a message object of type '<SetIKSeed-response>"
  "45f70114af10b3f5f5e2b664d72ca331")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetIKSeed-response)))
  "Returns md5sum for a message object of type 'SetIKSeed-response"
  "45f70114af10b3f5f5e2b664d72ca331")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetIKSeed-response>)))
  "Returns full string definition for message of type '<SetIKSeed-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetIKSeed-response)))
  "Returns full string definition for message of type 'SetIKSeed-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetIKSeed-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetIKSeed-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetIKSeed-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetIKSeed)))
  'SetIKSeed-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetIKSeed)))
  'SetIKSeed-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetIKSeed)))
  "Returns string type for a service object of type '<SetIKSeed>"
  "hebi_cpp_api_examples/SetIKSeed")