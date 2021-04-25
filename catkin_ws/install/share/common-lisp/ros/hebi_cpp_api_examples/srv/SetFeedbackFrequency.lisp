; Auto-generated. Do not edit!


(cl:in-package hebi_cpp_api_examples-srv)


;//! \htmlinclude SetFeedbackFrequency-request.msg.html

(cl:defclass <SetFeedbackFrequency-request> (roslisp-msg-protocol:ros-message)
  ((frequency_hz
    :reader frequency_hz
    :initarg :frequency_hz
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetFeedbackFrequency-request (<SetFeedbackFrequency-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetFeedbackFrequency-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetFeedbackFrequency-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hebi_cpp_api_examples-srv:<SetFeedbackFrequency-request> is deprecated: use hebi_cpp_api_examples-srv:SetFeedbackFrequency-request instead.")))

(cl:ensure-generic-function 'frequency_hz-val :lambda-list '(m))
(cl:defmethod frequency_hz-val ((m <SetFeedbackFrequency-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hebi_cpp_api_examples-srv:frequency_hz-val is deprecated.  Use hebi_cpp_api_examples-srv:frequency_hz instead.")
  (frequency_hz m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetFeedbackFrequency-request>) ostream)
  "Serializes a message object of type '<SetFeedbackFrequency-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'frequency_hz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetFeedbackFrequency-request>) istream)
  "Deserializes a message object of type '<SetFeedbackFrequency-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'frequency_hz) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetFeedbackFrequency-request>)))
  "Returns string type for a service object of type '<SetFeedbackFrequency-request>"
  "hebi_cpp_api_examples/SetFeedbackFrequencyRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetFeedbackFrequency-request)))
  "Returns string type for a service object of type 'SetFeedbackFrequency-request"
  "hebi_cpp_api_examples/SetFeedbackFrequencyRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetFeedbackFrequency-request>)))
  "Returns md5sum for a message object of type '<SetFeedbackFrequency-request>"
  "b11146fd2143e78325a7496114ee3a9e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetFeedbackFrequency-request)))
  "Returns md5sum for a message object of type 'SetFeedbackFrequency-request"
  "b11146fd2143e78325a7496114ee3a9e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetFeedbackFrequency-request>)))
  "Returns full string definition for message of type '<SetFeedbackFrequency-request>"
  (cl:format cl:nil "float64 frequency_hz~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetFeedbackFrequency-request)))
  "Returns full string definition for message of type 'SetFeedbackFrequency-request"
  (cl:format cl:nil "float64 frequency_hz~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetFeedbackFrequency-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetFeedbackFrequency-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetFeedbackFrequency-request
    (cl:cons ':frequency_hz (frequency_hz msg))
))
;//! \htmlinclude SetFeedbackFrequency-response.msg.html

(cl:defclass <SetFeedbackFrequency-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SetFeedbackFrequency-response (<SetFeedbackFrequency-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetFeedbackFrequency-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetFeedbackFrequency-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hebi_cpp_api_examples-srv:<SetFeedbackFrequency-response> is deprecated: use hebi_cpp_api_examples-srv:SetFeedbackFrequency-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetFeedbackFrequency-response>) ostream)
  "Serializes a message object of type '<SetFeedbackFrequency-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetFeedbackFrequency-response>) istream)
  "Deserializes a message object of type '<SetFeedbackFrequency-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetFeedbackFrequency-response>)))
  "Returns string type for a service object of type '<SetFeedbackFrequency-response>"
  "hebi_cpp_api_examples/SetFeedbackFrequencyResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetFeedbackFrequency-response)))
  "Returns string type for a service object of type 'SetFeedbackFrequency-response"
  "hebi_cpp_api_examples/SetFeedbackFrequencyResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetFeedbackFrequency-response>)))
  "Returns md5sum for a message object of type '<SetFeedbackFrequency-response>"
  "b11146fd2143e78325a7496114ee3a9e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetFeedbackFrequency-response)))
  "Returns md5sum for a message object of type 'SetFeedbackFrequency-response"
  "b11146fd2143e78325a7496114ee3a9e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetFeedbackFrequency-response>)))
  "Returns full string definition for message of type '<SetFeedbackFrequency-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetFeedbackFrequency-response)))
  "Returns full string definition for message of type 'SetFeedbackFrequency-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetFeedbackFrequency-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetFeedbackFrequency-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetFeedbackFrequency-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetFeedbackFrequency)))
  'SetFeedbackFrequency-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetFeedbackFrequency)))
  'SetFeedbackFrequency-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetFeedbackFrequency)))
  "Returns string type for a service object of type '<SetFeedbackFrequency>"
  "hebi_cpp_api_examples/SetFeedbackFrequency")