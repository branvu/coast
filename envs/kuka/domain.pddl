(define (domain pick-and-place)
    (:requirements :strips :typing :equality :negative-preconditions :conditional-effects)
    (:constants
        mytable - table
        egg - object
        bacon - object
        celery - object
        radish - object
        mystove - stove
        mysink - sink
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
        object - abstract
        phys - abstract
        timestep - abstract
        sink - phys
        stove - phys
        table - phys
    )
    (:predicates
        (AtTimestep ?t - timestep)
        (Failpick ?obj - object ?on - phys ?t1 - timestep ?t2 - timestep)
        (Failplace ?obj - object ?on - phys ?t1 - timestep ?t2 - timestep)
        (Failclean ?obj - object ?on - sink ?t1 - timestep ?t2 - timestep)
        (Failcook ?obj - object ?on - stove ?t1 - timestep ?t2 - timestep)
        (Holding ?obj - object)
        (Cleaned ?obj - object)
        (Cooked ?obj - object)
        (On ?obj - object ?on - phys)
        (Next ?t1 - timestep ?t2 - timestep)

        (logpick ?obj - object ?on - phys ?t1 - timestep ?t2 - timestep)
        (logplace ?obj - object ?on - phys ?t1 - timestep ?t2 - timestep)
        (logclean ?obj - object ?on - sink ?t1 - timestep ?t2 - timestep)
        (logcook ?obj - object ?on - stove ?t1 - timestep ?t2 - timestep)
    )
    (:action pick
        :parameters (?obj - object ?on - phys ?t1 - timestep ?t2 - timestep)
        :precondition (and
                        (On ?obj ?on)
                        (not (Failpick ?obj ?on ?t1 ?t2))
                        (forall (?o - object) (not (Holding ?o)))
                        (not (= ?t1 ?t2))
                        (AtTimestep ?t1)
                        (Next ?t1 ?t2)
                        )
        :effect (and
                    (Holding ?obj)
                    (forall (?on - phys) (not (On ?obj ?on)))
                    (not (AtTimestep ?t1))
                    (AtTimestep ?t2)
                    (logpick ?obj ?on ?t1 ?t2)
                )
    )
    (:action place
        :parameters (?obj - object ?on - phys ?t1 - timestep ?t2 - timestep)
        :precondition (and
                        (not (Failplace ?obj ?on ?t1 ?t2))
                        (Holding ?obj)
                        (AtTimestep ?t1)
                        (not (= ?t1 ?t2))
                        (Next ?t1 ?t2)
                    )
        :effect (and 
                    (not (Holding ?obj))
                    (On ?obj ?on)
                    (not (AtTimestep ?t1))
                    (AtTimestep ?t2)
                    (logplace ?obj ?on ?t1 ?t2)
                )
    )
    (:action clean
        :parameters (?obj - object ?on - sink ?t1 - timestep ?t2 - timestep)
        :precondition (and
                        (On ?obj ?on)
                        (not (Failclean ?obj ?on ?t1 ?t2))
                        (AtTimestep ?t1)
                        (not (= ?t1 ?t2))
                        (Next ?t1 ?t2)
                    )
        :effect (and 
                    (Cleaned ?obj)
                    (not (AtTimestep ?t1))
                    (AtTimestep ?t2)
                    (logclean ?obj ?on ?t1 ?t2)
                )
    )
    (:action cook
        :parameters (?obj - object ?on - stove ?t1 - timestep ?t2 - timestep)
        :precondition (and
                        (On ?obj ?on)
                        (not (Failcook ?obj ?on ?t1 ?t2))
                        (AtTimestep ?t1)
                        (not (= ?t1 ?t2))
                        (Next ?t1 ?t2)
                    )
        :effect (and 
                    (Cooked ?obj)
                    (not (AtTimestep ?t1))
                    (AtTimestep ?t2)
                    (logcook ?obj ?on ?t1 ?t2)
                )
    )
)
