(define (domain robotics)
  (:requirements :strips :equality  :typing)
  (:types location movable manip pose)
  (:constants lgripper - manip rgripper - manip)
  (:predicates
	   (At ?obj - movable ?loc - location)
     (RobotAt ?p - pose)
	   (InGripper ?obj - movable ?gripper - manip)
	   (IsDrawer ?drawer - movable)
     (InDrawer ?obj - movable ?drawer - movable)
	   (Object ?obj - movable)
	   (Empty ?gripper)
     (TempArea ?l - location)
	   (IsGP ?p - pose ?obj - movable ?gripper - manip)
     (IsAccessPointFor ?p - pose ?o - object ?surface - location ?gripper - manip)
     (Closed ?drawer - movable)
     (Obstructs ?p - pose ?o1 - movable ?o2 - movable)
  )
  
 (:action putDown
  :parameters(?o1 - movable ?l1 - location  ?l2 - pose ?gripper - manip)
  :precondition (and (Object ?o1)
  		               (InGripper ?o1 ?gripper) (RobotAt ?l2) (TempArea ?l1)
		                 (IsAccessPointFor ?l2 ?o1 ?l1 ?gripper))
  :effect (and (not (InGripper ?o1 ?gripper)) (At ?o1 ?l1) (empty ?gripper)
  ))

 (:action moveto
     :parameters(?l1 - pose  ?l2 - pose)
     :precondition (and (RobotAt ?l1))
     :effect (and (not (RobotAt ?l1)) (RobotAt ?l2)
  ))

 (:action grasp
     :parameters(?o1 - movable ?l1 - location ?l2 - pose ?gripper - manip ?drawer - movable)
     :precondition (and (At ?o1 ?l1) (Object ?o1) (IsDrawer ?drawer)
                        (not (and (Closed ?drawer) (InDrawer ?o1 ?drawer)))                        
     		                (RobotAt ?l2) (IsGP ?l2 ?o1 ?gripper)
		                    (empty ?gripper)
                        (forall (?o - movable) (not (obstructs ?l2 ?o ?o1)))
		       )
     :effect (and (InGripper ?o1 ?gripper) (not (At ?o1 ?l1)) (not (empty ?gripper)) (not (InDrawer ?o1 ?drawer))
                  (forall (?o - movable) (forall (?p - pose) (not (obstructs ?p ?o1 ?o))))
  ))

 (:action openDrawer
  :parameters (?drawer - movable ?lrobot - pose ?gripper - manip)
  :precondition (and (RobotAt ?lrobot) (IsDrawer ?drawer) (empty ?gripper)
                     (IsGP ?lrobot ?drawer ?gripper) (closed ?drawer)
                     (forall (?o - movable) (not (obstructs ?lrobot ?o ?drawer)))
                )
  :effect (and (not (closed ?drawer))
               (forall (?o - movable) (forall (?p - pose) (not (obstructs ?p ?drawer ?o))))
  ))

 (:action closeDrawer
  :parameters (?drawer - movable ?lrobot - pose ?gripper - manip)
  :precondition (and (RobotAt ?lrobot) (IsDrawer ?drawer) (empty ?gripper)
                     (IsGP ?lrobot ?drawer ?gripper) (not (closed ?drawer))
                     (forall (?o - movable) (not (obstructs ?lrobot ?o ?drawer)))
                )
  :effect (and (closed ?drawer)
               (forall (?o - movable) (forall (?p - pose) (not (obstructs ?p ?drawer ?o))))
  ))
)