;; Auto-generated. Do not edit!


(when (boundp 'hebi_cpp_api_examples::SetFeedbackFrequency)
  (if (not (find-package "HEBI_CPP_API_EXAMPLES"))
    (make-package "HEBI_CPP_API_EXAMPLES"))
  (shadow 'SetFeedbackFrequency (find-package "HEBI_CPP_API_EXAMPLES")))
(unless (find-package "HEBI_CPP_API_EXAMPLES::SETFEEDBACKFREQUENCY")
  (make-package "HEBI_CPP_API_EXAMPLES::SETFEEDBACKFREQUENCY"))
(unless (find-package "HEBI_CPP_API_EXAMPLES::SETFEEDBACKFREQUENCYREQUEST")
  (make-package "HEBI_CPP_API_EXAMPLES::SETFEEDBACKFREQUENCYREQUEST"))
(unless (find-package "HEBI_CPP_API_EXAMPLES::SETFEEDBACKFREQUENCYRESPONSE")
  (make-package "HEBI_CPP_API_EXAMPLES::SETFEEDBACKFREQUENCYRESPONSE"))

(in-package "ROS")





(defclass hebi_cpp_api_examples::SetFeedbackFrequencyRequest
  :super ros::object
  :slots (_frequency_hz ))

(defmethod hebi_cpp_api_examples::SetFeedbackFrequencyRequest
  (:init
   (&key
    ((:frequency_hz __frequency_hz) 0.0)
    )
   (send-super :init)
   (setq _frequency_hz (float __frequency_hz))
   self)
  (:frequency_hz
   (&optional __frequency_hz)
   (if __frequency_hz (setq _frequency_hz __frequency_hz)) _frequency_hz)
  (:serialization-length
   ()
   (+
    ;; float64 _frequency_hz
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _frequency_hz
       (sys::poke _frequency_hz (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _frequency_hz
     (setq _frequency_hz (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass hebi_cpp_api_examples::SetFeedbackFrequencyResponse
  :super ros::object
  :slots ())

(defmethod hebi_cpp_api_examples::SetFeedbackFrequencyResponse
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

(defclass hebi_cpp_api_examples::SetFeedbackFrequency
  :super ros::object
  :slots ())

(setf (get hebi_cpp_api_examples::SetFeedbackFrequency :md5sum-) "b11146fd2143e78325a7496114ee3a9e")
(setf (get hebi_cpp_api_examples::SetFeedbackFrequency :datatype-) "hebi_cpp_api_examples/SetFeedbackFrequency")
(setf (get hebi_cpp_api_examples::SetFeedbackFrequency :request) hebi_cpp_api_examples::SetFeedbackFrequencyRequest)
(setf (get hebi_cpp_api_examples::SetFeedbackFrequency :response) hebi_cpp_api_examples::SetFeedbackFrequencyResponse)

(defmethod hebi_cpp_api_examples::SetFeedbackFrequencyRequest
  (:response () (instance hebi_cpp_api_examples::SetFeedbackFrequencyResponse :init)))

(setf (get hebi_cpp_api_examples::SetFeedbackFrequencyRequest :md5sum-) "b11146fd2143e78325a7496114ee3a9e")
(setf (get hebi_cpp_api_examples::SetFeedbackFrequencyRequest :datatype-) "hebi_cpp_api_examples/SetFeedbackFrequencyRequest")
(setf (get hebi_cpp_api_examples::SetFeedbackFrequencyRequest :definition-)
      "float64 frequency_hz
---

")

(setf (get hebi_cpp_api_examples::SetFeedbackFrequencyResponse :md5sum-) "b11146fd2143e78325a7496114ee3a9e")
(setf (get hebi_cpp_api_examples::SetFeedbackFrequencyResponse :datatype-) "hebi_cpp_api_examples/SetFeedbackFrequencyResponse")
(setf (get hebi_cpp_api_examples::SetFeedbackFrequencyResponse :definition-)
      "float64 frequency_hz
---

")



(provide :hebi_cpp_api_examples/SetFeedbackFrequency "b11146fd2143e78325a7496114ee3a9e")


