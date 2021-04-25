; Auto-generated. Do not edit!


(cl:in-package hebi_cpp_api_examples-msg)


;//! \htmlinclude Playback.msg.html

(cl:defclass <Playback> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (index
    :reader index
    :initarg :index
    :type cl:fixnum
    :initform 0)
   (mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Playback (<Playback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Playback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Playback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hebi_cpp_api_examples-msg:<Playback> is deprecated: use hebi_cpp_api_examples-msg:Playback instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <Playback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hebi_cpp_api_examples-msg:name-val is deprecated.  Use hebi_cpp_api_examples-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'index-val :lambda-list '(m))
(cl:defmethod index-val ((m <Playback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hebi_cpp_api_examples-msg:index-val is deprecated.  Use hebi_cpp_api_examples-msg:index instead.")
  (index m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <Playback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hebi_cpp_api_examples-msg:mode-val is deprecated.  Use hebi_cpp_api_examples-msg:mode instead.")
  (mode m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Playback>)))
    "Constants for message type '<Playback>"
  '((:GO_TO_WAYPOINT . 0)
    (:GO_TO_PATH_START . 1)
    (:PLAY_PATH . 2)
    (:NUM_MODES . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Playback)))
    "Constants for message type 'Playback"
  '((:GO_TO_WAYPOINT . 0)
    (:GO_TO_PATH_START . 1)
    (:PLAY_PATH . 2)
    (:NUM_MODES . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Playback>) ostream)
  "Serializes a message object of type '<Playback>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let* ((signed (cl:slot-value msg 'index)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Playback>) istream)
  "Deserializes a message object of type '<Playback>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'index) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Playback>)))
  "Returns string type for a message object of type '<Playback>"
  "hebi_cpp_api_examples/Playback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Playback)))
  "Returns string type for a message object of type 'Playback"
  "hebi_cpp_api_examples/Playback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Playback>)))
  "Returns md5sum for a message object of type '<Playback>"
  "15d0ac08484c4ab841188f5d622febfa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Playback)))
  "Returns md5sum for a message object of type 'Playback"
  "15d0ac08484c4ab841188f5d622febfa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Playback>)))
  "Returns full string definition for message of type '<Playback>"
  (cl:format cl:nil "# The name of the waypoint or path~%string name~%# If string not given, can playback by index instead~%int16 index~%~%# Playback modes~%int16 GO_TO_WAYPOINT = 0 # Go to a named or numbered waypoint~%int16 GO_TO_PATH_START = 1 # Go to the start point of the path~%int16 PLAY_PATH = 2 # Play path at offset & reset offset~%int16 NUM_MODES = 3 # Total number of available modes; this is an invalid mode selection!~%# Select playback mode~%int16 mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Playback)))
  "Returns full string definition for message of type 'Playback"
  (cl:format cl:nil "# The name of the waypoint or path~%string name~%# If string not given, can playback by index instead~%int16 index~%~%# Playback modes~%int16 GO_TO_WAYPOINT = 0 # Go to a named or numbered waypoint~%int16 GO_TO_PATH_START = 1 # Go to the start point of the path~%int16 PLAY_PATH = 2 # Play path at offset & reset offset~%int16 NUM_MODES = 3 # Total number of available modes; this is an invalid mode selection!~%# Select playback mode~%int16 mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Playback>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Playback>))
  "Converts a ROS message object to a list"
  (cl:list 'Playback
    (cl:cons ':name (name msg))
    (cl:cons ':index (index msg))
    (cl:cons ':mode (mode msg))
))
