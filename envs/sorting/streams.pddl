(define (domain pick-and-place)
    (:requirements :strips :typing :equality :negative-preconditions :conditional-effects)
    (:constants)
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
        (logcook ?obj - object ?on - sink ?t1 - timestep ?t2 - timestep)
    )
    (:action Failpick_pre
        :parameters (?obj - object ?on - phys ?t1 - timestep ?t2 - timestep)
        :precondition (and)
        :effect (not (Failpick ?obj ?on ?t1 ?t2))
    )
    (:action Failpick_post
        :parameters (?obj - object ?on - phys ?t1 - timestep ?t2 - timestep)
        :precondition (and)
        :effect (not (Failpick ?obj ?on ?t1 ?t2))
    )
    (:action Failplace_pre
        :parameters (?obj - object ?on - phys ?t1 - timestep ?t2 - timestep)
        :precondition (and)
        :effect (not (Failplace ?obj ?on ?t1 ?t2))
    )
    (:action Failplace_post
        :parameters (?obj - object ?on - phys ?t1 - timestep ?t2 - timestep)
        :precondition (and)
        :effect (not (Failplace ?obj ?on ?t1 ?t2))
    )
    (:action Failcook_pre
        :parameters (?obj - object ?on - stove ?t1 - timestep ?t2 - timestep)
        :precondition (and)
        :effect (not (Failcook ?obj ?on ?t1 ?t2))
    )
    (:action Failcook_post
        :parameters (?obj - object ?on - stove ?t1 - timestep ?t2 - timestep)
        :precondition (and)
        :effect (not (Failcook ?obj ?on ?t1 ?t2))
    )
    (:action Failclean_pre
        :parameters (?obj - object ?on - sink ?t1 - timestep ?t2 - timestep)
        :precondition (and)
        :effect (not (Failclean ?obj ?on ?t1 ?t2))
    )
    (:action Failclean_post
        :parameters (?obj - object ?on - sink ?t1 - timestep ?t2 - timestep)
        :precondition (and)
        :effect (not (Failclean ?obj ?on ?t1 ?t2))
    )
)
    

