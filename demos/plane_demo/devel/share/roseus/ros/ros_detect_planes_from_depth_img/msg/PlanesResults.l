;; Auto-generated. Do not edit!


(when (boundp 'ros_detect_planes_from_depth_img::PlanesResults)
  (if (not (find-package "ROS_DETECT_PLANES_FROM_DEPTH_IMG"))
    (make-package "ROS_DETECT_PLANES_FROM_DEPTH_IMG"))
  (shadow 'PlanesResults (find-package "ROS_DETECT_PLANES_FROM_DEPTH_IMG")))
(unless (find-package "ROS_DETECT_PLANES_FROM_DEPTH_IMG::PLANESRESULTS")
  (make-package "ROS_DETECT_PLANES_FROM_DEPTH_IMG::PLANESRESULTS"))

(in-package "ROS")
;;//! \htmlinclude PlanesResults.msg.html


(defclass ros_detect_planes_from_depth_img::PlanesResults
  :super ros::object
  :slots (_N _norms _center_3d _center_2d _mask_color ))

(defmethod ros_detect_planes_from_depth_img::PlanesResults
  (:init
   (&key
    ((:N __N) 0)
    ((:norms __norms) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:center_3d __center_3d) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:center_2d __center_2d) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:mask_color __mask_color) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _N (round __N))
   (setq _norms __norms)
   (setq _center_3d __center_3d)
   (setq _center_2d __center_2d)
   (setq _mask_color __mask_color)
   self)
  (:N
   (&optional __N)
   (if __N (setq _N __N)) _N)
  (:norms
   (&optional __norms)
   (if __norms (setq _norms __norms)) _norms)
  (:center_3d
   (&optional __center_3d)
   (if __center_3d (setq _center_3d __center_3d)) _center_3d)
  (:center_2d
   (&optional __center_2d)
   (if __center_2d (setq _center_2d __center_2d)) _center_2d)
  (:mask_color
   (&optional __mask_color)
   (if __mask_color (setq _mask_color __mask_color)) _mask_color)
  (:serialization-length
   ()
   (+
    ;; int32 _N
    4
    ;; float32[] _norms
    (* 4    (length _norms)) 4
    ;; float32[] _center_3d
    (* 4    (length _center_3d)) 4
    ;; float32[] _center_2d
    (* 4    (length _center_2d)) 4
    ;; uint16[] _mask_color
    (* 2    (length _mask_color)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _N
       (write-long _N s)
     ;; float32[] _norms
     (write-long (length _norms) s)
     (dotimes (i (length _norms))
       (sys::poke (elt _norms i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _center_3d
     (write-long (length _center_3d) s)
     (dotimes (i (length _center_3d))
       (sys::poke (elt _center_3d i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _center_2d
     (write-long (length _center_2d) s)
     (dotimes (i (length _center_2d))
       (sys::poke (elt _center_2d i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; uint16[] _mask_color
     (write-long (length _mask_color) s)
     (dotimes (i (length _mask_color))
       (write-word (elt _mask_color i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _N
     (setq _N (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32[] _norms
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _norms (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _norms i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _center_3d
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _center_3d (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _center_3d i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _center_2d
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _center_2d (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _center_2d i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; uint16[] _mask_color
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _mask_color (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _mask_color i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;;
   self)
  )

(setf (get ros_detect_planes_from_depth_img::PlanesResults :md5sum-) "d1490bcb974cae216e975f12f5d851b8")
(setf (get ros_detect_planes_from_depth_img::PlanesResults :datatype-) "ros_detect_planes_from_depth_img/PlanesResults")
(setf (get ros_detect_planes_from_depth_img::PlanesResults :definition-)
      "int32 N                 # Number of detected planes.

# In the following arrays, 
# the planes' parameters are concatinated one by one.

float32[] norms         # Nx3. Plane normal (nx, ny, nz).
float32[] center_3d     # Nx3. Plane 3D center (cx, cy, cz).
float32[] center_2d     # Nx2. Plane 2D center on the image (px, py),
                        #   which means {px}th column, and {py}th row.
uint16[] mask_color     # Nx3. Plane mask color (blue, green, red). 
                        # Each color's range is [0, 255]
")



(provide :ros_detect_planes_from_depth_img/PlanesResults "d1490bcb974cae216e975f12f5d851b8")


