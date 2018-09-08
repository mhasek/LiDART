;; Auto-generated. Do not edit!


(when (boundp 'lidart_gap_finding::drive_param)
  (if (not (find-package "LIDART_GAP_FINDING"))
    (make-package "LIDART_GAP_FINDING"))
  (shadow 'drive_param (find-package "LIDART_GAP_FINDING")))
(unless (find-package "LIDART_GAP_FINDING::DRIVE_PARAM")
  (make-package "LIDART_GAP_FINDING::DRIVE_PARAM"))

(in-package "ROS")
;;//! \htmlinclude drive_param.msg.html


(defclass lidart_gap_finding::drive_param
  :super ros::object
  :slots (_velocity _angle ))

(defmethod lidart_gap_finding::drive_param
  (:init
   (&key
    ((:velocity __velocity) 0.0)
    ((:angle __angle) 0.0)
    )
   (send-super :init)
   (setq _velocity (float __velocity))
   (setq _angle (float __angle))
   self)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:serialization-length
   ()
   (+
    ;; float32 _velocity
    4
    ;; float32 _angle
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _velocity
       (sys::poke _velocity (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _velocity
     (setq _velocity (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angle
     (setq _angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get lidart_gap_finding::drive_param :md5sum-) "23ee9ebc4f65684302501539608c3833")
(setf (get lidart_gap_finding::drive_param :datatype-) "lidart_gap_finding/drive_param")
(setf (get lidart_gap_finding::drive_param :definition-)
      "float32 velocity
float32 angle

")



(provide :lidart_gap_finding/drive_param "23ee9ebc4f65684302501539608c3833")


