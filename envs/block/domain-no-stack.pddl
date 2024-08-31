(define (domain blocks)
    (:requirements :strips :typing :equality :negative-preconditions :conditional-effects)
    (:constants
        b1 - block
        b2 - block
        b3 - block
        b4 - block
        b5 - block
        b6 - block
        b7 - block
        b8 - block
        b9 - block
        loc1 - location
        loc2 - location
        loc3 - location
        loc4 - location
        loc5 - location
        loc6 - location
        loc7 - location
        loc8 - location
        loc9 - location
    )
    (:types
        block - abstract
        location - abstract
        abstract - type
    )
    (:predicates
        (on ?b1 - block ?b2 - block)
        (ontable ?b - block ?loc - location)
        (clear ?a - abstract)
        (exist ?b - block)
        (handempty)
        (holding ?b - block)
        (Failpick ?b - block ?loc - location)
        (Failplace ?b - block ?loc - location)
        (logpick ?b - block ?loc - location)
        (logplace ?b - block ?loc - location)
    )

    (:action pick
        :parameters (?b - block ?loc - location)
        :precondition (and
                        (ontable ?b ?loc)
                        (handempty)
                        (not (Failpick ?b ?loc))
                    )
        :effect (and
                    (not (ontable ?b ?loc))
                    (not (handempty))
                    (holding ?b)
                    (clear ?loc)
                    ;(logpick ?b ?loc)
                )
    )
    (:action place
        :parameters (?b - block ?loc - location)
        :precondition (and
                        (not (Failplace ?b ?loc))
                        (holding ?b)
                        (clear ?loc)
                    )
        :effect (and 
                    (not (holding ?b))
                    (ontable ?b ?loc)
                    (handempty)
                    (not (clear ?loc))
                    ;(logplace ?b ?loc)
                )
    )
)
