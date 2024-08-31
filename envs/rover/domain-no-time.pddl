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
    
)
(:types
  abstract - type
	objective - abstract
	rover - abstract
	camera - abstract
	lander - abstract
	mode - abstract
	store - abstract
	rock - abstract
  )
  (:predicates
    (logtake_image ?v - rover ?o - objective ?c - camera ?m - mode)
    (logcalibrate ?v - rover ?o - objective ?c - camera)
    (logsend_image ?v - rover ?l - lander ?o - objective ?m - mode)
    (logsample_rock ?v - rover ?r - rock ?s - store)
    (logsend_analysis ?v - rover ?l - lander ?r - rock)
    (logdrop_rock  ?v - rover ?s - store)
    
    (Failtake_image ?v - rover ?o - objective ?c - camera ?m - mode)
    (Failcalibrate ?v - rover ?o - objective ?c - camera)
    (Failsend_image ?v - rover ?l - lander ?o - objective ?m - mode)
    (Failsample_rock ?v - rover ?r - rock ?s - store)
    (Failsend_analysis ?v - rover ?l - lander ?r - rock)
    (Faildrop_rock  ?v - rover ?s - store)
    
    (OnBoard ?c - camera ?v - rover)
    (Supports ?c - camera ?m - mode)
    (Calibrated ?o - objective ?c - camera ?v - rover)
    (HaveImage ?v - rover ?o - objective ?m - mode)
    (ReceivedImage ?o - objective ?m - mode)
    (ReceivedAnalysis ?r - rock)
    (Analyzed ?v - rover ?r - rock)
    (Free ?v - rover ?s - store)
    (Full ?v - rover ?s - store)
  )

  (:action take_image
    :parameters (?v - rover ?o - objective ?c - camera ?m - mode)
    :precondition (and (OnBoard ?c ?v) (Supports ?c ?m)
                       (Calibrated ?o ?c ?v)
                       (not (Failtake_image ?v ?o ?c ?m))
                  )
    :effect (and (HaveImage ?v ?o ?m)
                 (logtake_image ?v ?o ?c ?m))
   )
  (:action calibrate
    :parameters (?v - rover ?o - objective ?c - camera)
    :precondition (and (OnBoard ?c ?v)
                       (not (Failcalibrate ?v ?o ?c))
                  )
    :effect (and (Calibrated ?o ?c ?v) (logcalibrate ?v ?o ?c))
  )
  (:action send_image
    :parameters (?v - rover ?l - lander ?o - objective ?m - mode)
    :precondition (and (HaveImage ?v ?o ?m)
                       (not (Failsend_image ?v ?l ?o ?m))
                  )
    :effect (and (ReceivedImage ?o ?m) (logsend_image ?v ?l ?o ?m))
  )

  (:action sample_rock
    :parameters (?v - rover ?r - rock ?s - store)
    :precondition (and (Free ?v ?s)
            (not (Failsample_rock ?v ?r ?s))
    )
    :effect (and (Full ?v ?s) (Analyzed ?v ?r)
                 (not (Free ?v ?s))
                 (logsample_rock ?v ?r ?s))
  )
  (:action send_analysis
    :parameters (?v - rover ?l - lander ?r - rock)
    :precondition (and (Analyzed ?v ?r)
                       (not (Failsend_analysis ?v ?l ?r))
                  )
    :effect (and (ReceivedAnalysis ?r) (logsend_analysis ?v ?l ?r))
  )
  (:action drop_rock
    :parameters (?v - rover ?s - store)
    :precondition (and (Full ?v ?s)
            (not (Faildrop_rock ?v ?s))
        )
    :effect (and (Free ?v ?s)
                 (not (Full ?v ?s))
        (logdrop_rock ?v ?s))
  )
)
