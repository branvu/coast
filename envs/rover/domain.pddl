(define (domain rovers)
  (:requirements :strips :equality)
  (:constants
    rv1 - rover
    rv2 - rover
    c1 - camera
    c2 - camera
    o1 - objective
    o2 - objective
    ;o3 - objective
    ;o4 - objective
    lander1 - lander
    rgbd - mode
    s1 - store
    s2 - store
    r1 - rock
    r2 - rock
    ;r3 - rock
    ;r4 - rock
    t1 - timestep
    t2 - timestep
    t3 - timestep
    t4 - timestep
    t5 - timestep
    t6 - timestep
    t7 - timestep
    t8 - timestep
    t9 - timestep
    t10 - timestep
    t11 - timestep
    t12 - timestep
    t13 - timestep
    t14 - timestep
    t15 - timestep
    t16 - timestep
    t17 - timestep
    t18 - timestep
    t19 - timestep
    t20 - timestep
    t21 - timestep
    t22 - timestep
    t23 - timestep
    t24 - timestep
    t25 - timestep
    t26 - timestep
    t27 - timestep
    t28 - timestep
    t29 - timestep
    t30 - timestep
)
(:types
  abstract - type
	timestep - type
	objective - abstract
	rover - abstract
	camera - abstract
	lander - abstract
	mode - abstract
	store - abstract
	rock - abstract
  )
  (:predicates
    (logtake_image ?v - rover ?o - objective ?c - camera ?m - mode ?t1 - timestep ?t2 - timestep)
    (logcalibrate ?v - rover ?o - objective ?c - camera ?t1 - timestep ?t2 - timestep)
    (logsend_image ?v - rover ?l - lander ?o - objective ?m - mode ?t1 - timestep ?t2 - timestep)
    (logsample_rock ?v - rover ?r - rock ?s - store ?t1 - timestep ?t2 - timestep)
    (logsend_analysis ?v - rover ?l - lander ?r - rock ?t1 - timestep ?t2 - timestep)
    (logdrop_rock  ?v - rover ?s - store ?t1 - timestep ?t2 - timestep)
    
    (Failtake_image ?v - rover ?o - objective ?c - camera ?m - mode ?t1 - timestep ?t2 - timestep)
    (Failcalibrate ?v - rover ?o - objective ?c - camera ?t1 - timestep ?t2 - timestep)
    (Failsend_image ?v - rover ?l - lander ?o - objective ?m - mode ?t1 - timestep ?t2 - timestep)
    (Failsample_rock ?v - rover ?r - rock ?s - store ?t1 - timestep ?t2 - timestep)
    (Failsend_analysis ?v - rover ?l - lander ?r - rock ?t1 - timestep ?t2 - timestep)
    (Faildrop_rock  ?v - rover ?s - store ?t1 - timestep ?t2 - timestep)
    
    (OnBoard ?c - camera ?v - rover)
    (Supports ?c - camera ?m - mode)
    (Calibrated ?o - objective ?c - camera ?v - rover)
    (HaveImage ?v - rover ?o - objective ?m - mode)
    (ReceivedImage ?o - objective ?m - mode)
    (ReceivedAnalysis ?r - rock)
    (Analyzed ?v - rover ?r - rock)
    (Free ?v - rover ?s - store)
    (Full ?v - rover ?s - store)
    (AtTimestep ?t1 - timestep)
    (Next ?t1 - timestep ?t2 - timestep)
  )

  (:action take_image
    :parameters (?v - rover ?o - objective ?c - camera ?m - mode ?t1 - timestep ?t2 - timestep)
    :precondition (and (OnBoard ?c ?v) (Supports ?c ?m)
                       (Calibrated ?o ?c ?v)
			            (AtTimestep ?t1) (not (= ?t1 ?t2)) (Next ?t1 ?t2)
                       (not (Failtake_image ?v ?o ?c ?m ?t1 ?t2))
                  )
    :effect (and (HaveImage ?v ?o ?m)
		         (AtTimestep ?t2)
                 (not (AtTimestep ?t1))
                 (logtake_image ?v ?o ?c ?m ?t1 ?t2))
   )
  (:action calibrate
    :parameters (?v - rover ?o - objective ?c - camera ?t1 - timestep ?t2 - timestep)
    :precondition (and (OnBoard ?c ?v)
			(AtTimestep ?t1) (not (= ?t1 ?t2)) (Next ?t1 ?t2)
                       (not (Failcalibrate ?v ?o ?c ?t1 ?t2))
                  )
    :effect (and (Calibrated ?o ?c ?v) (AtTimestep ?t2) (not (AtTimestep ?t1)) (logcalibrate ?v ?o ?c ?t1 ?t2))
  )
  (:action send_image
    :parameters (?v - rover ?l - lander ?o - objective ?m - mode ?t1 - timestep ?t2 - timestep)
    :precondition (and (HaveImage ?v ?o ?m)
			(AtTimestep ?t1) (not (= ?t1 ?t2)) (Next ?t1 ?t2)
                       (not (Failsend_image ?v ?l ?o ?m ?t1 ?t2))
                  )
    :effect (and (ReceivedImage ?o ?m) (AtTimestep ?t2) (not (AtTimestep ?t1)) (logsend_image ?v ?l ?o ?m ?t1 ?t2))
  )

  (:action sample_rock
    :parameters (?v - rover ?r - rock ?s - store ?t1 - timestep ?t2 - timestep)
    :precondition (and (Free ?v ?s)
			(AtTimestep ?t1) (not (= ?t1 ?t2)) (Next ?t1 ?t2)
            (not (Failsample_rock ?v ?r ?s ?t1 ?t2))
    )
    :effect (and (Full ?v ?s) (Analyzed ?v ?r)
                 (not (Free ?v ?s))
		             (AtTimestep ?t2)
                 (not (AtTimestep ?t1))
                 (logsample_rock ?v ?r ?s ?t1 ?t2))
  )
  (:action send_analysis
    :parameters (?v - rover ?l - lander ?r - rock ?t1 - timestep ?t2 - timestep)
    :precondition (and (Analyzed ?v ?r)
			(AtTimestep ?t1) (not (= ?t1 ?t2)) (Next ?t1 ?t2)
                       (not (Failsend_analysis ?v ?l ?r ?t1 ?t2))
                  )
    :effect (and (ReceivedAnalysis ?r) (AtTimestep ?t2) (not (AtTimestep ?t1)) (logsend_analysis ?v ?l ?r ?t1 ?t2))
  )
  (:action drop_rock
    :parameters (?v - rover ?s - store ?t1 - timestep ?t2 - timestep)
    :precondition (and (Full ?v ?s)
			(AtTimestep ?t1) (not (= ?t1 ?t2)) (Next ?t1 ?t2) 
            (not (Faildrop_rock ?v ?s ?t1 ?t2))
        )
    :effect (and (Free ?v ?s)
                 (not (Full ?v ?s))
		(AtTimestep ?t2)
                 (not (AtTimestep ?t1))
        (logdrop_rock ?v ?s ?t1 ?t2))
  )
)
