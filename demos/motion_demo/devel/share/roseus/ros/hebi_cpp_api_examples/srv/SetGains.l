;; Auto-generated. Do not edit!


(when (boundp 'hebi_cpp_api_examples::SetGains)
  (if (not (find-package "HEBI_CPP_API_EXAMPLES"))
    (make-package "HEBI_CPP_API_EXAMPLES"))
  (shadow 'SetGains (find-package "HEBI_CPP_API_EXAMPLES")))
(unless (find-package "HEBI_CPP_API_EXAMPLES::SETGAINS")
  (make-package "HEBI_CPP_API_EXAMPLES::SETGAINS"))
(unless (find-package "HEBI_CPP_API_EXAMPLES::SETGAINSREQUEST")
  (make-package "HEBI_CPP_API_EXAMPLES::SETGAINSREQUEST"))
(unless (find-package "HEBI_CPP_API_EXAMPLES::SETGAINSRESPONSE")
  (make-package "HEBI_CPP_API_EXAMPLES::SETGAINSRESPONSE"))

(in-package "ROS")





(defclass hebi_cpp_api_examples::SetGainsRequest
  :super ros::object
  :slots (_gains_package _gains_file ))

(defmethod hebi_cpp_api_examples::SetGainsRequest
  (:init
   (&key
    ((:gains_package __gains_package) "")
    ((:gains_file __gains_file) "")
    )
   (send-super :init)
   (setq _gains_package (string __gains_package))
   (setq _gains_file (string __gains_file))
   self)
  (:gains_package
   (&optional __gains_package)
   (if __gains_package (setq _gains_package __gains_package)) _gains_package)
  (:gains_file
   (&optional __gains_file)
   (if __gains_file (setq _gains_file __gains_file)) _gains_file)
  (:serialization-length
   ()
   (+
    ;; string _gains_package
    4 (length _gains_package)
    ;; string _gains_file
    4 (length _gains_file)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _gains_package
       (write-long (length _gains_package) s) (princ _gains_package s)
     ;; string _gains_file
       (write-long (length _gains_file) s) (princ _gains_file s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _gains_package
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _gains_package (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _gains_file
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _gains_file (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass hebi_cpp_api_examples::SetGainsResponse
  :super ros::object
  :slots ())

(defmethod hebi_cpp_api_examples::SetGainsResponse
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

(defclass hebi_cpp_api_examples::SetGains
  :super ros::object
  :slots ())

(setf (get hebi_cpp_api_examples::SetGains :md5sum-) "4fedf26e82cd5de1e91a4ab742d58b2c")
(setf (get hebi_cpp_api_examples::SetGains :datatype-) "hebi_cpp_api_examples/SetGains")
(setf (get hebi_cpp_api_examples::SetGains :request) hebi_cpp_api_examples::SetGainsRequest)
(setf (get hebi_cpp_api_examples::SetGains :response) hebi_cpp_api_examples::SetGainsResponse)

(defmethod hebi_cpp_api_examples::SetGainsRequest
  (:response () (instance hebi_cpp_api_examples::SetGainsResponse :init)))

(setf (get hebi_cpp_api_examples::SetGainsRequest :md5sum-) "4fedf26e82cd5de1e91a4ab742d58b2c")
(setf (get hebi_cpp_api_examples::SetGainsRequest :datatype-) "hebi_cpp_api_examples/SetGainsRequest")
(setf (get hebi_cpp_api_examples::SetGainsRequest :definition-)
      "string gains_package
string gains_file
---


")

(setf (get hebi_cpp_api_examples::SetGainsResponse :md5sum-) "4fedf26e82cd5de1e91a4ab742d58b2c")
(setf (get hebi_cpp_api_examples::SetGainsResponse :datatype-) "hebi_cpp_api_examples/SetGainsResponse")
(setf (get hebi_cpp_api_examples::SetGainsResponse :definition-)
      "string gains_package
string gains_file
---


")



(provide :hebi_cpp_api_examples/SetGains "4fedf26e82cd5de1e91a4ab742d58b2c")


