; Auto-generated. Do not edit!


(cl:in-package hebi_cpp_api_examples-srv)


;//! \htmlinclude SetCommandLifetime-request.msg.html

(cl:defclass <SetCommandLifetime-request> (roslisp-msg-protocol:ros-message)
  ((lifetime
    :reader lifetime
    :initarg :lifetime
    :type cl:real
    :initform 0))
)

(cl:defclass SetCommandLifetime-request (<SetCommandLifetime-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetCommandLifetime-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetCommandLifetime-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hebi_cpp_api_examples-srv:<SetCommandLifetime-request> is deprecated: use hebi_cpp_api_examples-srv:SetCommandLifetime-request instead.")))

(cl:ensure-generic-function 'lifetime-val :lambda-list '(m))
(cl:defmethod lifetime-val ((m <SetCommandLifetime-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hebi_cpp_api_examples-srv:lifetime-val is deprecated.  Use hebi_cpp_api_examples-srv:lifetime instead.")
  (lifetime m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetCommandLifetime-request>) ostream)
  "Serializes a message object of type '<SetCommandLifetime-request>"
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'lifetime)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'lifetime) (cl:floor (cl:slot-value msg 'lifetime)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetCommandLifetime-request>) istream)
  "Deserializes a message object of type '<SetCommandLifetime-request>"
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'lifetime) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetCommandLifetime-request>)))
  "Returns string type for a service object of type '<SetCommandLifetime-request>"
  "hebi_cpp_api_examples/SetCommandLifetimeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCommandLifetime-request)))
  "Returns string type for a service object of type 'SetCommandLifetime-request"
  "hebi_cpp_api_examples/SetCommandLifetimeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetCommandLifetime-request>)))
  "Returns md5sum for a message object of type '<SetCommandLifetime-request>"
  "0659a04e68b42bafc22572c92df4e2e3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetCommandLifetime-request)))
  "Returns md5sum for a message object of type 'SetCommandLifetime-request"
  "0659a04e68b42bafc22572c92df4e2e3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetCommandLifetime-request>)))
  "Returns full string definition for message of type '<SetCommandLifetime-request>"
  (cl:format cl:nil "duration lifetime~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetCommandLifetime-request)))
  "Returns full string definition for message of type 'SetCommandLifetime-request"
  (cl:format cl:nil "duration lifetime~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetCommandLifetime-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetCommandLifetime-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetCommandLifetime-request
    (cl:cons ':lifetime (lifetime msg))
))
;//! \htmlinclude SetCommandLifetime-response.msg.html

(cl:defclass <SetCommandLifetime-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SetCommandLifetime-response (<SetCommandLifetime-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetCommandLifetime-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetCommandLifetime-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hebi_cpp_api_examples-srv:<SetCommandLifetime-response> is deprecated: use hebi_cpp_api_examples-srv:SetCommandLifetime-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetCommandLifetime-response>) ostream)
  "Serializes a message object of type '<SetCommandLifetime-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetCommandLifetime-response>) istream)
  "Deserializes a message object of type '<SetCommandLifetime-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetCommandLifetime-response>)))
  "Returns string type for a service object of type '<SetCommandLifetime-response>"
  "hebi_cpp_api_examples/SetCommandLifetimeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCommandLifetime-response)))
  "Returns string type for a service object of type 'SetCommandLifetime-response"
  "hebi_cpp_api_examples/SetCommandLifetimeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetCommandLifetime-response>)))
  "Returns md5sum for a message object of type '<SetCommandLifetime-response>"
  "0659a04e68b42bafc22572c92df4e2e3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetCommandLifetime-response)))
  "Returns md5sum for a message object of type 'SetCommandLifetime-response"
  "0659a04e68b42bafc22572c92df4e2e3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetCommandLifetime-response>)))
  "Returns full string definition for message of type '<SetCommandLifetime-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetCommandLifetime-response)))
  "Returns full string definition for message of type 'SetCommandLifetime-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetCommandLifetime-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetCommandLifetime-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetCommandLifetime-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetCommandLifetime)))
  'SetCommandLifetime-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetCommandLifetime)))
  'SetCommandLifetime-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCommandLifetime)))
  "Returns string type for a service object of type '<SetCommandLifetime>"
  "hebi_cpp_api_examples/SetCommandLifetime")