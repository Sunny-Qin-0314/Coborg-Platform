;; Auto-generated. Do not edit!


(when (boundp 'hebi_cpp_api_examples::ArmMotionGoal)
  (if (not (find-package "HEBI_CPP_API_EXAMPLES"))
    (make-package "HEBI_CPP_API_EXAMPLES"))
  (shadow 'ArmMotionGoal (find-package "HEBI_CPP_API_EXAMPLES")))
(unless (find-package "HEBI_CPP_API_EXAMPLES::ARMMOTIONGOAL")
  (make-package "HEBI_CPP_API_EXAMPLES::ARMMOTIONGOAL"))

(in-package "ROS")
;;//! \htmlinclude ArmMotionGoal.msg.html


(defclass hebi_cpp_api_examples::ArmMotionGoal
  :super ros::object
  :slots (_x _y _z _tipx _tipy _tipz _set_color _r _g _b ))

(defmethod hebi_cpp_api_examples::ArmMotionGoal
  (:init
   (&key
    ((:x __x) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:y __y) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:z __z) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:tipx __tipx) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:tipy __tipy) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:tipz __tipz) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:set_color __set_color) nil)
    ((:r __r) 0)
    ((:g __g) 0)
    ((:b __b) 0)
    )
   (send-super :init)
   (setq _x __x)
   (setq _y __y)
   (setq _z __z)
   (setq _tipx __tipx)
   (setq _tipy __tipy)
   (setq _tipz __tipz)
   (setq _set_color __set_color)
   (setq _r (round __r))
   (setq _g (round __g))
   (setq _b (round __b))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:tipx
   (&optional __tipx)
   (if __tipx (setq _tipx __tipx)) _tipx)
  (:tipy
   (&optional __tipy)
   (if __tipy (setq _tipy __tipy)) _tipy)
  (:tipz
   (&optional __tipz)
   (if __tipz (setq _tipz __tipz)) _tipz)
  (:set_color
   (&optional __set_color)
   (if __set_color (setq _set_color __set_color)) _set_color)
  (:r
   (&optional __r)
   (if __r (setq _r __r)) _r)
  (:g
   (&optional __g)
   (if __g (setq _g __g)) _g)
  (:b
   (&optional __b)
   (if __b (setq _b __b)) _b)
  (:serialization-length
   ()
   (+
    ;; float64[] _x
    (* 8    (length _x)) 4
    ;; float64[] _y
    (* 8    (length _y)) 4
    ;; float64[] _z
    (* 8    (length _z)) 4
    ;; float64[] _tipx
    (* 8    (length _tipx)) 4
    ;; float64[] _tipy
    (* 8    (length _tipy)) 4
    ;; float64[] _tipz
    (* 8    (length _tipz)) 4
    ;; bool _set_color
    1
    ;; uint8 _r
    1
    ;; uint8 _g
    1
    ;; uint8 _b
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _x
     (write-long (length _x) s)
     (dotimes (i (length _x))
       (sys::poke (elt _x i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _y
     (write-long (length _y) s)
     (dotimes (i (length _y))
       (sys::poke (elt _y i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _z
     (write-long (length _z) s)
     (dotimes (i (length _z))
       (sys::poke (elt _z i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _tipx
     (write-long (length _tipx) s)
     (dotimes (i (length _tipx))
       (sys::poke (elt _tipx i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _tipy
     (write-long (length _tipy) s)
     (dotimes (i (length _tipy))
       (sys::poke (elt _tipy i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _tipz
     (write-long (length _tipz) s)
     (dotimes (i (length _tipz))
       (sys::poke (elt _tipz i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; bool _set_color
       (if _set_color (write-byte -1 s) (write-byte 0 s))
     ;; uint8 _r
       (write-byte _r s)
     ;; uint8 _g
       (write-byte _g s)
     ;; uint8 _b
       (write-byte _b s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _x
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _x (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _x i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _y
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _y (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _y i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _z
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _z (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _z i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _tipx
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tipx (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _tipx i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _tipy
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tipy (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _tipy i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _tipz
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tipz (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _tipz i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; bool _set_color
     (setq _set_color (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _r
     (setq _r (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _g
     (setq _g (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _b
     (setq _b (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get hebi_cpp_api_examples::ArmMotionGoal :md5sum-) "5182f7e080f47b29c479712b9f962cb9")
(setf (get hebi_cpp_api_examples::ArmMotionGoal :datatype-) "hebi_cpp_api_examples/ArmMotionGoal")
(setf (get hebi_cpp_api_examples::ArmMotionGoal :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
# Goal position:
float64[] x
float64[] y
float64[] z
float64[] tipx
float64[] tipy
float64[] tipz

# Optionally, set a color when doing the move; otherwise, clear the color.
bool set_color
uint8 r
uint8 g
uint8 b

")



(provide :hebi_cpp_api_examples/ArmMotionGoal "5182f7e080f47b29c479712b9f962cb9")


