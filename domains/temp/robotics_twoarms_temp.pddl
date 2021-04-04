(define (domain robotics)
  (:requirements :adl :equality :typing)
  (:types object location manip)
  (:predicates
	   (at ?obj - object ?loc - location)
           (robotat ?loc - location)
	   (ingripper ?obj - object ?gripper - manip)
	   (isaccesspointfor ?loc - location ?obj - object ?targetloc - location ?gripper - manip)
	   (obstructs ?loc - location ?obj - object ?obj - object)
	   (putdownobstructs ?loc - location ?obj - object ?targetloc - location)
	   (isgp ?loc - location ?obj -object ?gripper - manip)
	   (empty ?gripper - manip)
	   (clear ?loc - location)
  )


  (:action moveto
     :parameters(?l1 - location ?l2 - location)
     :precondition (and  (robotat ?l1))
     :effect (and (not (robotat ?l1)) (robotat ?l2))
  )

 (:action putdown
  :parameters(?o1 - object ?target - location ?robotloc - location ?gripper - manip)
  :precondition (and (ingripper ?o1 ?gripper)
                     (robotat ?robotloc) (isaccesspointfor ?robotloc ?o1 ?target ?gripper)
		     (clear ?target)
;		     (forall (?o - object) (not (putdownobstructs ?robotloc ?o ?target)))
		     )
  :effect (and (at ?o1 ?target)
               (not (ingripper ?o1 ?gripper))
	       (empty ?gripper)
;	       (not (clear ?target)))
	       )
 )

 (:action grasp
     :parameters (?o1 - object ?l1  - location ?gp - location ?gripper - manip)
     :precondition (and (at  ?o1 ?l1)
     		       (robotat ?gp) (isgp ?gp ?o1 ?gripper)
		       (empty ?gripper)
		       (forall (?o - object)
		       	       	    (not (obstructs ?gp ?o ?o1))))
     :effect (and (not (at ?o1 ?l1))
     	          (forall (?o - object) (forall (?l - location) (not (obstructs ?l ?o1 ?o))))
                  (InGripper ?o1 ?gripper)
		  (not (empty ?gripper))
;		  (forall (?pdp - location) (forall (?pdl - location) (forall (?g - manip) (when (isaccesspointfor ?pdp ?pdl ?g) (not (putdownobstructs ?pdp ?o1 ?pdl))))))
		  (clear ?l1))
 )
)