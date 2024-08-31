(define (domain sorting)
    (:requirements :strips :typing :equality :negative-preconditions :conditional-effects)
    (:constants
        ;b0 - block
        b1 - block
        b2 - block
        b3 - block
        b4 - block
        b5 - block
        b6 - block
        b7 - block
        b8 - block
        b9 - block
        b10 - block
        b11 - block
        b12 - block
        b13 - block
        b14 - block
        b15 - block
        b16 - block
        b17 - block
        b18 - block
        b19 - block
        b20 - block
        ;b21 - block
        ;b22 - block
        ;b23 - block
        ;b24 - block
        ;b25 - block

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
        ;t26 - timestep
        ;t27 - timestep
        ;t28 - timestep
        ;t29 - timestep
        ;t30 - timestep
        ;t31 - timestep
        ;t32 - timestep
        ;t33 - timestep
        ;t44 - timestep
        ;t45 - timestep
        ;t46 - timestep
        ;t47 - timestep
        ;t48 - timestep
        ;t49 - timestep
        ;t50 - timestep
        btable - table
        ftable - table
        ltable - table
        rtable - table
        grasp1 - grasp
        grasp2 - grasp
        grasp3 - grasp
        grasp4 - grasp
    )
    (:types
        block - abstract
        table - abstract
        abstract - type
        timestep - type
        grasp - abstract
    )
    (:functions
        (MoveCost)
        (PickCost)
        (PlaceCost)
    )
    (:predicates
        (AtTimestep ?t - timestep)
        (Next ?t1 - timestep ?t2 - timestep)
        (ontable ?b - block ?t - table)
        (handempty)
        (movable)
        (holding ?b - block ?g - grasp)
        (Failpick ?b - block ?t - table ?g - grasp ?t1 - timestep ?t2 - timestep)
        (Failplace ?b - block ?t - table ?g - grasp ?t1 - timestep ?t2 - timestep)
        ;(Failmove_base ?t1 - timestep ?t2 - timestep)
        (logpick ?b - block ?t - table ?g - grasp ?t1 - timestep ?t2 - timestep)
        (logplace ?b - block ?t - table ?g - grasp ?t1 - timestep ?t2 - timestep)
        (collisionblock ?b - block ?wall - block ?g - grasp)
    )
      ;(:action move_base
      ;  :parameters (?t1 - timestep ?t2 - timestep)
      ;  :precondition (and 
      ;      (not (Failmove_base ?t1 ?t2))
      ;      (not (= ?t1 ?t2))
      ;      (AtTimestep ?t1)
      ;      (Next ?t1 ?t2)
      ;      (movable)
      ;  )
      ;  :effect (and
      ;              (increase (total-cost) (MoveCost ?t1 ?t2))
      ;              (not (AtTimestep ?t1))
      ;              (AtTimestep ?t2)
      ;              (logmove_base ?t1 ?t2)
      ;              (not (movable))
      ;      )
      ;)
    (:action pick
        :parameters (?b - block ?t - table ?g - grasp ?t1 - timestep ?t2 - timestep)
        :precondition (and
                        (ontable ?b ?t)
                        (handempty)
                        (not (Failpick ?b ?t ?g ?t1 ?t2))
                        (not (= ?t1 ?t2))
                        (AtTimestep ?t1)
                        (Next ?t1 ?t2)
                        (forall (?wall - block) (not (collisionblock ?b ?wall ?g)))
                    )
        :effect (and
                    (not (ontable ?b ?t))
                    (not (handempty))
                    (holding ?b ?g)
                    (not (AtTimestep ?t1))
                    (AtTimestep ?t2)
                    (logpick ?b ?t ?g ?t1 ?t2)
                    (forall (?b2 - block) (not (collisionblock ?b2 ?b ?g)))
                    (movable)
                (when (and
	(= ?b b4) (= ?t ftable) (= ?g grasp1) (= ?t1 t7) (= ?t2 t8)(logpick b1 btable grasp1 t1 t2)(logplace b1 rtable grasp1 t2 t3)(logpick b2 ftable grasp4 t3 t4)(logplace b2 rtable grasp4 t4 t5)(logpick b3 btable grasp1 t5 t6)(logplace b3 ltable grasp1 t6 t7)
	
)
 (collisionblock b4 b3 grasp1))(when (and
	(= ?b b1) (= ?t btable) (= ?g grasp1) (= ?t1 t1) (= ?t2 t2)
	
)
 (and (collisionblock b1 b2 grasp1) (collisionblock b1 b3 grasp1) (collisionblock b1 b4 grasp1) (collisionblock b1 b5 grasp1) (collisionblock b1 b6 grasp1) (collisionblock b1 b7 grasp1) (collisionblock b1 b8 grasp1) )))
    )
    (:action place
        :parameters (?b - block ?t - table ?g - grasp ?t1 - timestep ?t2 - timestep)
        :precondition (and
                        (not (Failplace ?b ?t ?g ?t1 ?t2))
                        (holding ?b ?g)
                        (AtTimestep ?t1)
                        (not (= ?t1 ?t2))
                        (Next ?t1 ?t2)
                    )
        :effect (and 
            (not (holding ?b ?g))
                    (ontable ?b ?t)
                    (handempty)
                    (not (AtTimestep ?t1))
                    (AtTimestep ?t2)
                    (logplace ?b ?t ?g ?t1 ?t2)
                    (movable)
                )
    )
)
