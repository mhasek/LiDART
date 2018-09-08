;; Auto-generated. Do not edit!


(when (boundp 'lidart_gap_finding::gaps)
  (if (not (find-package "LIDART_GAP_FINDING"))
    (make-package "LIDART_GAP_FINDING"))
  (shadow 'gaps (find-package "LIDART_GAP_FINDING")))
(unless (find-package "LIDART_GAP_FINDING::GAPS")
  (make-package "LIDART_GAP_FINDING::GAPS"))

(in-package "ROS")
;;//! \htmlinclude gaps.msg.html


(defclass lidart_gap_finding::gaps
  :super ros::object
  :slots (_r1 _theta1 _r2 _theta2 _x1 _y1 _x2 _y2 _euc_length _delta_angle ))

(defmethod lidart_gap_finding::gaps
  (:init
   (&key
    ((:r1 __r1) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:theta1 __theta1) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:r2 __r2) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:theta2 __theta2) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:x1 __x1) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:y1 __y1) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:x2 __x2) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:y2 __y2) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:euc_length __euc_length) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:delta_angle __delta_angle) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _r1 __r1)
   (setq _theta1 __theta1)
   (setq _r2 __r2)
   (setq _theta2 __theta2)
   (setq _x1 __x1)
   (setq _y1 __y1)
   (setq _x2 __x2)
   (setq _y2 __y2)
   (setq _euc_length __euc_length)
   (setq _delta_angle __delta_angle)
   self)
  (:r1
   (&optional __r1)
   (if __r1 (setq _r1 __r1)) _r1)
  (:theta1
   (&optional __theta1)
   (if __theta1 (setq _theta1 __theta1)) _theta1)
  (:r2
   (&optional __r2)
   (if __r2 (setq _r2 __r2)) _r2)
  (:theta2
   (&optional __theta2)
   (if __theta2 (setq _theta2 __theta2)) _theta2)
  (:x1
   (&optional __x1)
   (if __x1 (setq _x1 __x1)) _x1)
  (:y1
   (&optional __y1)
   (if __y1 (setq _y1 __y1)) _y1)
  (:x2
   (&optional __x2)
   (if __x2 (setq _x2 __x2)) _x2)
  (:y2
   (&optional __y2)
   (if __y2 (setq _y2 __y2)) _y2)
  (:euc_length
   (&optional __euc_length)
   (if __euc_length (setq _euc_length __euc_length)) _euc_length)
  (:delta_angle
   (&optional __delta_angle)
   (if __delta_angle (setq _delta_angle __delta_angle)) _delta_angle)
  (:serialization-length
   ()
   (+
    ;; float32[] _r1
    (* 4    (length _r1)) 4
    ;; float32[] _theta1
    (* 4    (length _theta1)) 4
    ;; float32[] _r2
    (* 4    (length _r2)) 4
    ;; float32[] _theta2
    (* 4    (length _theta2)) 4
    ;; float32[] _x1
    (* 4    (length _x1)) 4
    ;; float32[] _y1
    (* 4    (length _y1)) 4
    ;; float32[] _x2
    (* 4    (length _x2)) 4
    ;; float32[] _y2
    (* 4    (length _y2)) 4
    ;; float32[] _euc_length
    (* 4    (length _euc_length)) 4
    ;; float32[] _delta_angle
    (* 4    (length _delta_angle)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _r1
     (write-long (length _r1) s)
     (dotimes (i (length _r1))
       (sys::poke (elt _r1 i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _theta1
     (write-long (length _theta1) s)
     (dotimes (i (length _theta1))
       (sys::poke (elt _theta1 i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _r2
     (write-long (length _r2) s)
     (dotimes (i (length _r2))
       (sys::poke (elt _r2 i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _theta2
     (write-long (length _theta2) s)
     (dotimes (i (length _theta2))
       (sys::poke (elt _theta2 i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _x1
     (write-long (length _x1) s)
     (dotimes (i (length _x1))
       (sys::poke (elt _x1 i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _y1
     (write-long (length _y1) s)
     (dotimes (i (length _y1))
       (sys::poke (elt _y1 i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _x2
     (write-long (length _x2) s)
     (dotimes (i (length _x2))
       (sys::poke (elt _x2 i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _y2
     (write-long (length _y2) s)
     (dotimes (i (length _y2))
       (sys::poke (elt _y2 i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _euc_length
     (write-long (length _euc_length) s)
     (dotimes (i (length _euc_length))
       (sys::poke (elt _euc_length i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _delta_angle
     (write-long (length _delta_angle) s)
     (dotimes (i (length _delta_angle))
       (sys::poke (elt _delta_angle i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _r1
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _r1 (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _r1 i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _theta1
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _theta1 (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _theta1 i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _r2
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _r2 (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _r2 i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _theta2
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _theta2 (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _theta2 i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _x1
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _x1 (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _x1 i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _y1
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _y1 (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _y1 i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _x2
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _x2 (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _x2 i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _y2
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _y2 (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _y2 i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _euc_length
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _euc_length (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _euc_length i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _delta_angle
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _delta_angle (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _delta_angle i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get lidart_gap_finding::gaps :md5sum-) "c57a067c5b046afa4cdb6b94f79d87d2")
(setf (get lidart_gap_finding::gaps :datatype-) "lidart_gap_finding/gaps")
(setf (get lidart_gap_finding::gaps :definition-)
      "# angles are angle from lidar scan
# ranges are distance from lidar
# lengths are lengths of gap

float32[] r1
float32[] theta1
float32[] r2
float32[] theta2

float32[] x1
float32[] y1
float32[] x2
float32[] y2

float32[] euc_length
float32[] delta_angle
")



(provide :lidart_gap_finding/gaps "c57a067c5b046afa4cdb6b94f79d87d2")


