;; Auto-generated. Do not edit!


(when (boundp 'hebi_cpp_api_examples::SetIKSeed)
  (if (not (find-package "HEBI_CPP_API_EXAMPLES"))
    (make-package "HEBI_CPP_API_EXAMPLES"))
  (shadow 'SetIKSeed (find-package "HEBI_CPP_API_EXAMPLES")))
(unless (find-package "HEBI_CPP_API_EXAMPLES::SETIKSEED")
  (make-package "HEBI_CPP_API_EXAMPLES::SETIKSEED"))
(unless (find-package "HEBI_CPP_API_EXAMPLES::SETIKSEEDREQUEST")
  (make-package "HEBI_CPP_API_EXAMPLES::SETIKSEEDREQUEST"))
(unless (find-package "HEBI_CPP_API_EXAMPLES::SETIKSEEDRESPONSE")
  (make-package "HEBI_CPP_API_EXAMPLES::SETIKSEEDRESPONSE"))

(in-package "ROS")





(defclass hebi_cpp_api_examples::SetIKSeedRequest
  :super ros::object
  :slots (_seed ))

(defmethod hebi_cpp_api_examples::SetIKSeedRequest
  (:init
   (&key
    ((:seed __seed) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _seed __seed)
   self)
  (:seed
   (&optional __seed)
   (if __seed (setq _seed __seed)) _seed)
  (:serialization-length
   ()
   (+
    ;; float64[] _seed
    (* 8    (length _seed)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _seed
     (write-long (length _seed) s)
     (dotimes (i (length _seed))
       (sys::poke (elt _seed i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _seed
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _seed (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _seed i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(defclass hebi_cpp_api_examples::SetIKSeedResponse
  :super ros::object
  :slots ())

(defmethod hebi_cpp_api_examples::SetIKSeedResponse
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

(defclass hebi_cpp_api_examples::SetIKSeed
  :super ros::object
  :slots ())

(setf (get hebi_cpp_api_examples::SetIKSeed :md5sum-) "45f70114af10b3f5f5e2b664d72ca331")
(setf (get hebi_cpp_api_examples::SetIKSeed :datatype-) "hebi_cpp_api_examples/SetIKSeed")
(setf (get hebi_cpp_api_examples::SetIKSeed :request) hebi_cpp_api_examples::SetIKSeedRequest)
(setf (get hebi_cpp_api_examples::SetIKSeed :response) hebi_cpp_api_examples::SetIKSeedResponse)

(defmethod hebi_cpp_api_examples::SetIKSeedRequest
  (:response () (instance hebi_cpp_api_examples::SetIKSeedResponse :init)))

(setf (get hebi_cpp_api_examples::SetIKSeedRequest :md5sum-) "45f70114af10b3f5f5e2b664d72ca331")
(setf (get hebi_cpp_api_examples::SetIKSeedRequest :datatype-) "hebi_cpp_api_examples/SetIKSeedRequest")
(setf (get hebi_cpp_api_examples::SetIKSeedRequest :definition-)
      "float64[] seed
---

")

(setf (get hebi_cpp_api_examples::SetIKSeedResponse :md5sum-) "45f70114af10b3f5f5e2b664d72ca331")
(setf (get hebi_cpp_api_examples::SetIKSeedResponse :datatype-) "hebi_cpp_api_examples/SetIKSeedResponse")
(setf (get hebi_cpp_api_examples::SetIKSeedResponse :definition-)
      "float64[] seed
---

")



(provide :hebi_cpp_api_examples/SetIKSeed "45f70114af10b3f5f5e2b664d72ca331")


