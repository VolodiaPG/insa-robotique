
(cl:in-package :asdf)

(defsystem "ropigo-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SimpleWrite" :depends-on ("_package_SimpleWrite"))
    (:file "_package_SimpleWrite" :depends-on ("_package"))
  ))