;; Auto-generated. Do not edit!


(when (boundp 'goal_getter::goal_msg)
  (if (not (find-package "GOAL_GETTER"))
    (make-package "GOAL_GETTER"))
  (shadow 'goal_msg (find-package "GOAL_GETTER")))
(unless (find-package "GOAL_GETTER::GOAL_MSG")
  (make-package "GOAL_GETTER::GOAL_MSG"))

(in-package "ROS")
;;//! \htmlinclude goal_msg.msg.html


(defclass goal_getter::goal_msg
  :super ros::object
  :slots (_Class _x _y _z ))

(defmethod goal_getter::goal_msg
  (:init
   (&key
    ((:Class __Class) "")
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    )
   (send-super :init)
   (setq _Class (string __Class))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   self)
  (:Class
   (&optional __Class)
   (if __Class (setq _Class __Class)) _Class)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:serialization-length
   ()
   (+
    ;; string _Class
    4 (length _Class)
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _z
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _Class
       (write-long (length _Class) s) (princ _Class s)
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z
       (sys::poke _z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _Class
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _Class (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z
     (setq _z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get goal_getter::goal_msg :md5sum-) "1995ab0b6624278887ba1c25d7923c9c")
(setf (get goal_getter::goal_msg :datatype-) "goal_getter/goal_msg")
(setf (get goal_getter::goal_msg :definition-)
      "string Class
float64 x
float64 y
float64 z


")



(provide :goal_getter/goal_msg "1995ab0b6624278887ba1c25d7923c9c")


