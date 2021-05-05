;; Auto-generated. Do not edit!


(when (boundp 'hebi_cpp_api_examples::StartPath)
  (if (not (find-package "HEBI_CPP_API_EXAMPLES"))
    (make-package "HEBI_CPP_API_EXAMPLES"))
  (shadow 'StartPath (find-package "HEBI_CPP_API_EXAMPLES")))
(unless (find-package "HEBI_CPP_API_EXAMPLES::STARTPATH")
  (make-package "HEBI_CPP_API_EXAMPLES::STARTPATH"))

(in-package "ROS")
;;//! \htmlinclude StartPath.msg.html


(defclass hebi_cpp_api_examples::StartPath
  :super ros::object
  :slots ())

(defmethod hebi_cpp_api_examples::StartPath
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(setf (get hebi_cpp_api_examples::StartPath :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get hebi_cpp_api_examples::StartPath :datatype-) "hebi_cpp_api_examples/StartPath")
(setf (get hebi_cpp_api_examples::StartPath :definition-)
      "
")



(provide :hebi_cpp_api_examples/StartPath "d41d8cd98f00b204e9800998ecf8427e")


