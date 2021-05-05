;; Auto-generated. Do not edit!


(when (boundp 'hebi_cpp_api_examples::OffsetPlayback)
  (if (not (find-package "HEBI_CPP_API_EXAMPLES"))
    (make-package "HEBI_CPP_API_EXAMPLES"))
  (shadow 'OffsetPlayback (find-package "HEBI_CPP_API_EXAMPLES")))
(unless (find-package "HEBI_CPP_API_EXAMPLES::OFFSETPLAYBACK")
  (make-package "HEBI_CPP_API_EXAMPLES::OFFSETPLAYBACK"))

(in-package "ROS")
;;//! \htmlinclude OffsetPlayback.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass hebi_cpp_api_examples::OffsetPlayback
  :super ros::object
  :slots (_offset ))

(defmethod hebi_cpp_api_examples::OffsetPlayback
  (:init
   (&key
    ((:offset __offset) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _offset __offset)
   self)
  (:offset
   (&rest __offset)
   (if (keywordp (car __offset))
       (send* _offset __offset)
     (progn
       (if __offset (setq _offset (car __offset)))
       _offset)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _offset
    (send _offset :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _offset
       (send _offset :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _offset
     (send _offset :deserialize buf ptr-) (incf ptr- (send _offset :serialization-length))
   ;;
   self)
  )

(setf (get hebi_cpp_api_examples::OffsetPlayback :md5sum-) "de19cca9344eb5bfedb7e55986a47f2e")
(setf (get hebi_cpp_api_examples::OffsetPlayback :datatype-) "hebi_cpp_api_examples/OffsetPlayback")
(setf (get hebi_cpp_api_examples::OffsetPlayback :definition-)
      "geometry_msgs/Point offset


================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :hebi_cpp_api_examples/OffsetPlayback "de19cca9344eb5bfedb7e55986a47f2e")


