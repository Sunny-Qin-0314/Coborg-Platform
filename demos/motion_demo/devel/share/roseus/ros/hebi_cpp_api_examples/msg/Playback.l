;; Auto-generated. Do not edit!


(when (boundp 'hebi_cpp_api_examples::Playback)
  (if (not (find-package "HEBI_CPP_API_EXAMPLES"))
    (make-package "HEBI_CPP_API_EXAMPLES"))
  (shadow 'Playback (find-package "HEBI_CPP_API_EXAMPLES")))
(unless (find-package "HEBI_CPP_API_EXAMPLES::PLAYBACK")
  (make-package "HEBI_CPP_API_EXAMPLES::PLAYBACK"))

(in-package "ROS")
;;//! \htmlinclude Playback.msg.html


(intern "*GO_TO_WAYPOINT*" (find-package "HEBI_CPP_API_EXAMPLES::PLAYBACK"))
(shadow '*GO_TO_WAYPOINT* (find-package "HEBI_CPP_API_EXAMPLES::PLAYBACK"))
(defconstant hebi_cpp_api_examples::Playback::*GO_TO_WAYPOINT* 0)
(intern "*GO_TO_PATH_START*" (find-package "HEBI_CPP_API_EXAMPLES::PLAYBACK"))
(shadow '*GO_TO_PATH_START* (find-package "HEBI_CPP_API_EXAMPLES::PLAYBACK"))
(defconstant hebi_cpp_api_examples::Playback::*GO_TO_PATH_START* 1)
(intern "*PLAY_PATH*" (find-package "HEBI_CPP_API_EXAMPLES::PLAYBACK"))
(shadow '*PLAY_PATH* (find-package "HEBI_CPP_API_EXAMPLES::PLAYBACK"))
(defconstant hebi_cpp_api_examples::Playback::*PLAY_PATH* 2)
(intern "*NUM_MODES*" (find-package "HEBI_CPP_API_EXAMPLES::PLAYBACK"))
(shadow '*NUM_MODES* (find-package "HEBI_CPP_API_EXAMPLES::PLAYBACK"))
(defconstant hebi_cpp_api_examples::Playback::*NUM_MODES* 3)
(defclass hebi_cpp_api_examples::Playback
  :super ros::object
  :slots (_name _index _mode ))

(defmethod hebi_cpp_api_examples::Playback
  (:init
   (&key
    ((:name __name) "")
    ((:index __index) 0)
    ((:mode __mode) 0)
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _index (round __index))
   (setq _mode (round __mode))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:index
   (&optional __index)
   (if __index (setq _index __index)) _index)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; int16 _index
    2
    ;; int16 _mode
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; int16 _index
       (write-word _index s)
     ;; int16 _mode
       (write-word _mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int16 _index
     (setq _index (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _mode
     (setq _mode (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get hebi_cpp_api_examples::Playback :md5sum-) "15d0ac08484c4ab841188f5d622febfa")
(setf (get hebi_cpp_api_examples::Playback :datatype-) "hebi_cpp_api_examples/Playback")
(setf (get hebi_cpp_api_examples::Playback :definition-)
      "# The name of the waypoint or path
string name
# If string not given, can playback by index instead
int16 index

# Playback modes
int16 GO_TO_WAYPOINT = 0 # Go to a named or numbered waypoint
int16 GO_TO_PATH_START = 1 # Go to the start point of the path
int16 PLAY_PATH = 2 # Play path at offset & reset offset
int16 NUM_MODES = 3 # Total number of available modes; this is an invalid mode selection!
# Select playback mode
int16 mode

")



(provide :hebi_cpp_api_examples/Playback "15d0ac08484c4ab841188f5d622febfa")


