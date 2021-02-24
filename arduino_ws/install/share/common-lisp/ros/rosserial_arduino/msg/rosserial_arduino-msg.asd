
(cl:in-package :asdf)

(defsystem "rosserial_arduino-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Adc" :depends-on ("_package_Adc"))
    (:file "_package_Adc" :depends-on ("_package"))
    (:file "CMU" :depends-on ("_package_CMU"))
    (:file "_package_CMU" :depends-on ("_package"))
  ))