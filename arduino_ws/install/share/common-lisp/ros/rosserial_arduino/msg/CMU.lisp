; Auto-generated. Do not edit!


(cl:in-package rosserial_arduino-msg)


;//! \htmlinclude CMU.msg.html

(cl:defclass <CMU> (roslisp-msg-protocol:ros-message)
  ((Potentiometer
    :reader Potentiometer
    :initarg :Potentiometer
    :type cl:fixnum
    :initform 0)
   (Flex_Sensor
    :reader Flex_Sensor
    :initarg :Flex_Sensor
    :type cl:fixnum
    :initform 0)
   (IR_Sensor
    :reader IR_Sensor
    :initarg :IR_Sensor
    :type cl:fixnum
    :initform 0)
   (Ultrasonic_Sensor
    :reader Ultrasonic_Sensor
    :initarg :Ultrasonic_Sensor
    :type cl:fixnum
    :initform 0)
   (Button_State
    :reader Button_State
    :initarg :Button_State
    :type cl:fixnum
    :initform 0))
)

(cl:defclass CMU (<CMU>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CMU>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CMU)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rosserial_arduino-msg:<CMU> is deprecated: use rosserial_arduino-msg:CMU instead.")))

(cl:ensure-generic-function 'Potentiometer-val :lambda-list '(m))
(cl:defmethod Potentiometer-val ((m <CMU>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosserial_arduino-msg:Potentiometer-val is deprecated.  Use rosserial_arduino-msg:Potentiometer instead.")
  (Potentiometer m))

(cl:ensure-generic-function 'Flex_Sensor-val :lambda-list '(m))
(cl:defmethod Flex_Sensor-val ((m <CMU>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosserial_arduino-msg:Flex_Sensor-val is deprecated.  Use rosserial_arduino-msg:Flex_Sensor instead.")
  (Flex_Sensor m))

(cl:ensure-generic-function 'IR_Sensor-val :lambda-list '(m))
(cl:defmethod IR_Sensor-val ((m <CMU>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosserial_arduino-msg:IR_Sensor-val is deprecated.  Use rosserial_arduino-msg:IR_Sensor instead.")
  (IR_Sensor m))

(cl:ensure-generic-function 'Ultrasonic_Sensor-val :lambda-list '(m))
(cl:defmethod Ultrasonic_Sensor-val ((m <CMU>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosserial_arduino-msg:Ultrasonic_Sensor-val is deprecated.  Use rosserial_arduino-msg:Ultrasonic_Sensor instead.")
  (Ultrasonic_Sensor m))

(cl:ensure-generic-function 'Button_State-val :lambda-list '(m))
(cl:defmethod Button_State-val ((m <CMU>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosserial_arduino-msg:Button_State-val is deprecated.  Use rosserial_arduino-msg:Button_State instead.")
  (Button_State m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CMU>) ostream)
  "Serializes a message object of type '<CMU>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'Potentiometer)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'Potentiometer)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'Flex_Sensor)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'Flex_Sensor)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'IR_Sensor)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'IR_Sensor)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'Ultrasonic_Sensor)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'Ultrasonic_Sensor)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'Button_State)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'Button_State)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CMU>) istream)
  "Deserializes a message object of type '<CMU>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'Potentiometer)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'Potentiometer)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'Flex_Sensor)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'Flex_Sensor)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'IR_Sensor)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'IR_Sensor)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'Ultrasonic_Sensor)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'Ultrasonic_Sensor)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'Button_State)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'Button_State)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CMU>)))
  "Returns string type for a message object of type '<CMU>"
  "rosserial_arduino/CMU")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CMU)))
  "Returns string type for a message object of type 'CMU"
  "rosserial_arduino/CMU")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CMU>)))
  "Returns md5sum for a message object of type '<CMU>"
  "be9cdfeec1c60f4d26327be218060167")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CMU)))
  "Returns md5sum for a message object of type 'CMU"
  "be9cdfeec1c60f4d26327be218060167")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CMU>)))
  "Returns full string definition for message of type '<CMU>"
  (cl:format cl:nil "uint16 Potentiometer~%uint16 Flex_Sensor~%uint16 IR_Sensor~%uint16 Ultrasonic_Sensor~%uint16 Button_State~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CMU)))
  "Returns full string definition for message of type 'CMU"
  (cl:format cl:nil "uint16 Potentiometer~%uint16 Flex_Sensor~%uint16 IR_Sensor~%uint16 Ultrasonic_Sensor~%uint16 Button_State~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CMU>))
  (cl:+ 0
     2
     2
     2
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CMU>))
  "Converts a ROS message object to a list"
  (cl:list 'CMU
    (cl:cons ':Potentiometer (Potentiometer msg))
    (cl:cons ':Flex_Sensor (Flex_Sensor msg))
    (cl:cons ':IR_Sensor (IR_Sensor msg))
    (cl:cons ':Ultrasonic_Sensor (Ultrasonic_Sensor msg))
    (cl:cons ':Button_State (Button_State msg))
))
