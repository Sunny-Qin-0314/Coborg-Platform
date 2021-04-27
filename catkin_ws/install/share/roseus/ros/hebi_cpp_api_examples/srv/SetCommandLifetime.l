;; Auto-generated. Do not edit!


(when (boundp 'hebi_cpp_api_examples::SetCommandLifetime)
  (if (not (find-package "HEBI_CPP_API_EXAMPLES"))
    (make-package "HEBI_CPP_API_EXAMPLES"))
  (shadow 'SetCommandLifetime (find-package "HEBI_CPP_API_EXAMPLES")))
(unless (find-package "HEBI_CPP_API_EXAMPLES::SETCOMMANDLIFETIME")
  (make-package "HEBI_CPP_API_EXAMPLES::SETCOMMANDLIFETIME"))
(unless (find-package "HEBI_CPP_API_EXAMPLES::SETCOMMANDLIFETIMEREQUEST")
  (make-package "HEBI_CPP_API_EXAMPLES::SETCOMMANDLIFETIMEREQUEST"))
(unless (find-package "HEBI_CPP_API_EXAMPLES::SETCOMMANDLIFETIMERESPONSE")
  (make-package "HEBI_CPP_API_EXAMPLES::SETCOMMANDLIFETIMERESPONSE"))

(in-package "ROS")





(defclass hebi_cpp_api_examples::SetCommandLifetimeRequest
  :super ros::object
  :slots (_lifetime ))

(defmethod hebi_cpp_api_examples::SetCommandLifetimeRequest
  (:init
   (&key
    ((:lifetime __lifetime) (instance ros::time :init))
    )
   (send-super :init)
   (setq _lifetime __lifetime)
   self)
  (:lifetime
   (&optional __lifetime)
   (if __lifetime (setq _lifetime __lifetime)) _lifetime)
  (:serialization-length
   ()
   (+
    ;; duration _lifetime
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; duration _lifetime
       (write-long (send _lifetime :sec) s) (write-long (send _lifetime :nsec) s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; duration _lifetime
     (send _lifetime :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _lifetime :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass hebi_cpp_api_examples::SetCommandLifetimeResponse
  :super ros::object
  :slots ())

(defmethod hebi_cpp_api_examples::SetCommandLifetimeResponse
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

(defclass hebi_cpp_api_examples::SetCommandLifetime
  :super ros::object
  :slots ())

(setf (get hebi_cpp_api_examples::SetCommandLifetime :md5sum-) "0659a04e68b42bafc22572c92df4e2e3")
(setf (get hebi_cpp_api_examples::SetCommandLifetime :datatype-) "hebi_cpp_api_examples/SetCommandLifetime")
(setf (get hebi_cpp_api_examples::SetCommandLifetime :request) hebi_cpp_api_examples::SetCommandLifetimeRequest)
(setf (get hebi_cpp_api_examples::SetCommandLifetime :response) hebi_cpp_api_examples::SetCommandLifetimeResponse)

(defmethod hebi_cpp_api_examples::SetCommandLifetimeRequest
  (:response () (instance hebi_cpp_api_examples::SetCommandLifetimeResponse :init)))

(setf (get hebi_cpp_api_examples::SetCommandLifetimeRequest :md5sum-) "0659a04e68b42bafc22572c92df4e2e3")
(setf (get hebi_cpp_api_examples::SetCommandLifetimeRequest :datatype-) "hebi_cpp_api_examples/SetCommandLifetimeRequest")
(setf (get hebi_cpp_api_examples::SetCommandLifetimeRequest :definition-)
      "duration lifetime
---

")

(setf (get hebi_cpp_api_examples::SetCommandLifetimeResponse :md5sum-) "0659a04e68b42bafc22572c92df4e2e3")
(setf (get hebi_cpp_api_examples::SetCommandLifetimeResponse :datatype-) "hebi_cpp_api_examples/SetCommandLifetimeResponse")
(setf (get hebi_cpp_api_examples::SetCommandLifetimeResponse :definition-)
      "duration lifetime
---

")



(provide :hebi_cpp_api_examples/SetCommandLifetime "0659a04e68b42bafc22572c92df4e2e3")


