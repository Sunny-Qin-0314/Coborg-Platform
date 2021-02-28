;; Auto-generated. Do not edit!


(when (boundp 'rosserial_arduino::CMU)
  (if (not (find-package "ROSSERIAL_ARDUINO"))
    (make-package "ROSSERIAL_ARDUINO"))
  (shadow 'CMU (find-package "ROSSERIAL_ARDUINO")))
(unless (find-package "ROSSERIAL_ARDUINO::CMU")
  (make-package "ROSSERIAL_ARDUINO::CMU"))

(in-package "ROS")
;;//! \htmlinclude CMU.msg.html


(defclass rosserial_arduino::CMU
  :super ros::object
  :slots (_Potentiometer _Flex_Sensor _IR_Sensor _Ultrasonic_Sensor _Button_State ))

(defmethod rosserial_arduino::CMU
  (:init
   (&key
    ((:Potentiometer __Potentiometer) 0)
    ((:Flex_Sensor __Flex_Sensor) 0)
    ((:IR_Sensor __IR_Sensor) 0)
    ((:Ultrasonic_Sensor __Ultrasonic_Sensor) 0)
    ((:Button_State __Button_State) 0)
    )
   (send-super :init)
   (setq _Potentiometer (round __Potentiometer))
   (setq _Flex_Sensor (round __Flex_Sensor))
   (setq _IR_Sensor (round __IR_Sensor))
   (setq _Ultrasonic_Sensor (round __Ultrasonic_Sensor))
   (setq _Button_State (round __Button_State))
   self)
  (:Potentiometer
   (&optional __Potentiometer)
   (if __Potentiometer (setq _Potentiometer __Potentiometer)) _Potentiometer)
  (:Flex_Sensor
   (&optional __Flex_Sensor)
   (if __Flex_Sensor (setq _Flex_Sensor __Flex_Sensor)) _Flex_Sensor)
  (:IR_Sensor
   (&optional __IR_Sensor)
   (if __IR_Sensor (setq _IR_Sensor __IR_Sensor)) _IR_Sensor)
  (:Ultrasonic_Sensor
   (&optional __Ultrasonic_Sensor)
   (if __Ultrasonic_Sensor (setq _Ultrasonic_Sensor __Ultrasonic_Sensor)) _Ultrasonic_Sensor)
  (:Button_State
   (&optional __Button_State)
   (if __Button_State (setq _Button_State __Button_State)) _Button_State)
  (:serialization-length
   ()
   (+
    ;; uint16 _Potentiometer
    2
    ;; uint16 _Flex_Sensor
    2
    ;; uint16 _IR_Sensor
    2
    ;; uint16 _Ultrasonic_Sensor
    2
    ;; uint16 _Button_State
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16 _Potentiometer
       (write-word _Potentiometer s)
     ;; uint16 _Flex_Sensor
       (write-word _Flex_Sensor s)
     ;; uint16 _IR_Sensor
       (write-word _IR_Sensor s)
     ;; uint16 _Ultrasonic_Sensor
       (write-word _Ultrasonic_Sensor s)
     ;; uint16 _Button_State
       (write-word _Button_State s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16 _Potentiometer
     (setq _Potentiometer (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _Flex_Sensor
     (setq _Flex_Sensor (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _IR_Sensor
     (setq _IR_Sensor (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _Ultrasonic_Sensor
     (setq _Ultrasonic_Sensor (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _Button_State
     (setq _Button_State (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get rosserial_arduino::CMU :md5sum-) "be9cdfeec1c60f4d26327be218060167")
(setf (get rosserial_arduino::CMU :datatype-) "rosserial_arduino/CMU")
(setf (get rosserial_arduino::CMU :definition-)
      "uint16 Potentiometer
uint16 Flex_Sensor
uint16 IR_Sensor
uint16 Ultrasonic_Sensor
uint16 Button_State
")



(provide :rosserial_arduino/CMU "be9cdfeec1c60f4d26327be218060167")


