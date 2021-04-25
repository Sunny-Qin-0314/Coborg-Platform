;; Auto-generated. Do not edit!


(when (boundp 'gb_visual_detection_3d_msgs::BoundingBoxes3d)
  (if (not (find-package "GB_VISUAL_DETECTION_3D_MSGS"))
    (make-package "GB_VISUAL_DETECTION_3D_MSGS"))
  (shadow 'BoundingBoxes3d (find-package "GB_VISUAL_DETECTION_3D_MSGS")))
(unless (find-package "GB_VISUAL_DETECTION_3D_MSGS::BOUNDINGBOXES3D")
  (make-package "GB_VISUAL_DETECTION_3D_MSGS::BOUNDINGBOXES3D"))

(in-package "ROS")
;;//! \htmlinclude BoundingBoxes3d.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass gb_visual_detection_3d_msgs::BoundingBoxes3d
  :super ros::object
  :slots (_header _bounding_boxes _normal_x _normal_y _normal_z ))

(defmethod gb_visual_detection_3d_msgs::BoundingBoxes3d
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:bounding_boxes __bounding_boxes) (let (r) (dotimes (i 0) (push (instance gb_visual_detection_3d_msgs::BoundingBox3d :init) r)) r))
    ((:normal_x __normal_x) 0.0)
    ((:normal_y __normal_y) 0.0)
    ((:normal_z __normal_z) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _bounding_boxes __bounding_boxes)
   (setq _normal_x (float __normal_x))
   (setq _normal_y (float __normal_y))
   (setq _normal_z (float __normal_z))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:bounding_boxes
   (&rest __bounding_boxes)
   (if (keywordp (car __bounding_boxes))
       (send* _bounding_boxes __bounding_boxes)
     (progn
       (if __bounding_boxes (setq _bounding_boxes (car __bounding_boxes)))
       _bounding_boxes)))
  (:normal_x
   (&optional __normal_x)
   (if __normal_x (setq _normal_x __normal_x)) _normal_x)
  (:normal_y
   (&optional __normal_y)
   (if __normal_y (setq _normal_y __normal_y)) _normal_y)
  (:normal_z
   (&optional __normal_z)
   (if __normal_z (setq _normal_z __normal_z)) _normal_z)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; gb_visual_detection_3d_msgs/BoundingBox3d[] _bounding_boxes
    (apply #'+ (send-all _bounding_boxes :serialization-length)) 4
    ;; float64 _normal_x
    8
    ;; float64 _normal_y
    8
    ;; float64 _normal_z
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; gb_visual_detection_3d_msgs/BoundingBox3d[] _bounding_boxes
     (write-long (length _bounding_boxes) s)
     (dolist (elem _bounding_boxes)
       (send elem :serialize s)
       )
     ;; float64 _normal_x
       (sys::poke _normal_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _normal_y
       (sys::poke _normal_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _normal_z
       (sys::poke _normal_z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; gb_visual_detection_3d_msgs/BoundingBox3d[] _bounding_boxes
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _bounding_boxes (let (r) (dotimes (i n) (push (instance gb_visual_detection_3d_msgs::BoundingBox3d :init) r)) r))
     (dolist (elem- _bounding_boxes)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float64 _normal_x
     (setq _normal_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _normal_y
     (setq _normal_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _normal_z
     (setq _normal_z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get gb_visual_detection_3d_msgs::BoundingBoxes3d :md5sum-) "1fb42483703ab1406c7e101f990caf57")
(setf (get gb_visual_detection_3d_msgs::BoundingBoxes3d :datatype-) "gb_visual_detection_3d_msgs/BoundingBoxes3d")
(setf (get gb_visual_detection_3d_msgs::BoundingBoxes3d :definition-)
      "std_msgs/Header header
BoundingBox3d[] bounding_boxes
float64 normal_x
float64 normal_y
float64 normal_z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: gb_visual_detection_3d_msgs/BoundingBox3d
string Class
float64 probability
float64 xmin
float64 ymin
float64 xmax
float64 ymax
float64 zmin
float64 zmax


")



(provide :gb_visual_detection_3d_msgs/BoundingBoxes3d "1fb42483703ab1406c7e101f990caf57")


