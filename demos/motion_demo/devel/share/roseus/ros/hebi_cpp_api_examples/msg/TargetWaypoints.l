;; Auto-generated. Do not edit!


(when (boundp 'hebi_cpp_api_examples::TargetWaypoints)
  (if (not (find-package "HEBI_CPP_API_EXAMPLES"))
    (make-package "HEBI_CPP_API_EXAMPLES"))
  (shadow 'TargetWaypoints (find-package "HEBI_CPP_API_EXAMPLES")))
(unless (find-package "HEBI_CPP_API_EXAMPLES::TARGETWAYPOINTS")
  (make-package "HEBI_CPP_API_EXAMPLES::TARGETWAYPOINTS"))

(in-package "ROS")
;;//! \htmlinclude TargetWaypoints.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass hebi_cpp_api_examples::TargetWaypoints
  :super ros::object
  :slots (_waypoints_vector ))

(defmethod hebi_cpp_api_examples::TargetWaypoints
  (:init
   (&key
    ((:waypoints_vector __waypoints_vector) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Point :init) r)) r))
    )
   (send-super :init)
   (setq _waypoints_vector __waypoints_vector)
   self)
  (:waypoints_vector
   (&rest __waypoints_vector)
   (if (keywordp (car __waypoints_vector))
       (send* _waypoints_vector __waypoints_vector)
     (progn
       (if __waypoints_vector (setq _waypoints_vector (car __waypoints_vector)))
       _waypoints_vector)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point[] _waypoints_vector
    (apply #'+ (send-all _waypoints_vector :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point[] _waypoints_vector
     (write-long (length _waypoints_vector) s)
     (dolist (elem _waypoints_vector)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point[] _waypoints_vector
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _waypoints_vector (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _waypoints_vector)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get hebi_cpp_api_examples::TargetWaypoints :md5sum-) "aa0b6e1fb814653675dd4da6b4a51d95")
(setf (get hebi_cpp_api_examples::TargetWaypoints :datatype-) "hebi_cpp_api_examples/TargetWaypoints")
(setf (get hebi_cpp_api_examples::TargetWaypoints :definition-)
      "geometry_msgs/Point[] waypoints_vector


================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :hebi_cpp_api_examples/TargetWaypoints "aa0b6e1fb814653675dd4da6b4a51d95")


