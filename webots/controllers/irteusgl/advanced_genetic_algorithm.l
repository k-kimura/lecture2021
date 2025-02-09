;;
;; advanced_genetic_algorithm.l
;;
(load "webotslib.l")
(defconstant NUM_SENSORS 8) (defconstant NUM_WHEELS 2)
(defconstant GENOTYPE_SIZE (* NUM_SENSORS NUM_WHEELS))
(defconstant SPEED_UNIT 0.00628) (setq time_step 32)
(setq matrix (make-matrix NUM_SENSORS NUM_WHEELS))
(defun check_for_new_genes ()
  ;;(format t ";; check_for_new_genes-----------------xxxxxxxxxxxxxxxxxx------------~%")
  (when (> (setq rec_len (wb_receiver_get_queue_length receiver)) 0)
    (setq rec_size (wb_receiver_get_data_size receiver))
    (setq rec (cout-float-vector (wb_receiver_get_data receiver) GENOTYPE_SIZE))
    (format t ";; rec_len=~A rec=~A rec_size=~A~%" rec_len rec rec_size)
    (dotimes (i GENOTYPE_SIZE) (setf (elt (matrix . entity) i) (elt rec i)))
    (wb_receiver_next_packet receiver)))
(defun clip_value (value min_max)
  (cond ((> value min_max) (float min_max))
	((< value (- min_max)) (float (- min_max)))
	(t (float value))))
(defun sense_compute_and_actuate nil
  (let ((sensor_values (instantiate float-vector NUM_SENSORS))
	(wheel_speed (instantiate float-vector NUM_WHEELS)))
    (dotimes (i NUM_SENSORS)
      (setf (elt sensor_values i) (wb_distance_sensor_get_value (elt sensors i))))
    (dotimes (i NUM_WHEELS)
      (dotimes (j NUM_SENSORS)
	(incf (elt wheel_speed i)
	      (* SPEED_UNIT (aref matrix j i) (elt sensor_values j)))))
    (setf (elt wheel_speed 0) (clip_value (elt wheel_speed 0) 6.28))
    (setf (elt wheel_speed 1) (clip_value (elt wheel_speed 1) 6.28))
    ;;(format t ";; wheel_speed=~A, matrix=~A~%" wheel_speed matrix)
    (wb_motor_set_velocity left_motor (elt wheel_speed 0))
    (wb_motor_set_velocity right_motor (elt wheel_speed 1))))
(defun main nil
  (wb_robot_init)
  (setq time_step (wb_robot_get_basic_time_step))
  (format t ";; wb_robot_init time_step =~A~%" time_step)
  (setq sensors (instantiate vector NUM_SENSORS))
  (dotimes (i NUM_SENSORS)
    (setq name (format nil "ps~d" i)) ;;(format t ";; ps=<~A>~%" name)
    (setf (elt sensors i) (wb_robot_get_device name))
    (wb_distance_sensor_enable (elt sensors i) time_step))
  (format t ";; sensors =~A~%" sensors)
  (setq receiver (wb_robot_get_device "receiver"))
  (wb_receiver_enable receiver time_step)
  (setq left_motor (wb_robot_get_device "left wheel motor"))
  (setq right_motor (wb_robot_get_device "right wheel motor"))
  (wb_motor_set_position left_motor INFINITY)
  (wb_motor_set_position right_motor INFINITY)
  (wb_motor_set_velocity left_motor 0.0)
  (wb_motor_set_velocity right_motor 0.0)
  (wb_robot_step time_step)
  (while t
    (wb_robot_step time_step)
    (check_for_new_genes)
    (sense_compute_and_actuate))
  (wb_robot_cleanup)
  )
(main)
