(define (domain robotics)
  (:requirements :strips :equality  :typing)
  (:types location movable manip pose)
  (:constants door1 door2 door3 door4 door5 door6 door7 door8 door9 door10 - pose
  	      lgripper - manip rgripper - manip)
  (:predicates
	   (At ?obj - movable ?loc - location)
           (RobotAt ?p - pose)
	   (InGripper ?obj - movable ?gripper - manip)
	   (IsAccessPointFor ?p - pose ?targetLocation - location)
	   (OnTray ?obj - movable )
	   (IsTray ?tray - movable )
	   (Topmost ?obj - movable  ?tray - movable )
	   (Object ?obj - movable)
	   (Smaller ?obj1 - movable  ?obj2 - movable )
	   (On ?obj1 - movable  ?obj2 - movable )
	   (TempArea ?loc - location)
	   (TrayLocation ?loc - location)
	   (empty ?gripper)
	   (InRoom1 ?p - pose)
	   (InRoom2 ?p - pose)
	   (isgp ?p - pose ?obj - movable ?gripper - manip)
	   (obstructs ?p - pose ?o1 - movable ?o2 - movable)
  )


 (:action placeOnTray
   :parameters(?lrobot - pose ?obj1 - movable ?obj2 - movable  ?tray - movable  ?trayloc - location  ?gripper - manip)
   :precondition(and (Object ?obj1)
   		     (RobotAt ?lrobot) (IsTray ?tray) 
		     (At ?tray ?trayloc)
   		     (IsAccessPointFor ?lrobot ?trayloc) (InGripper ?obj1 ?gripper) 
		     (Topmost ?obj2 ?tray) 
		     (smaller ?obj1 ?obj2)
		     )
   :effect(and (not (InGripper ?obj1 ?gripper)) (OnTray ?obj1) (on ?obj1 ?obj2)
   	       (not (Topmost ?obj2 ?tray)) (Topmost ?obj1 ?tray) 
	       (empty ?gripper) 
	      )
 )	      

 (:action pickTray
  :parameters(?lrobot  - pose ?tray - movable  ?trayloc - location )
  :precondition(and (RobotAt ?lrobot) (IsTray ?tray)
  		    (empty lgripper) (empty rgripper)
		    (IsAccessPointFor ?lrobot ?trayloc)
		    (At ?tray ?trayloc)
		    (forall (?o - movable) (not (obstructs ?lrobot ?o ?tray)))
		    )
  :effect(and (not (At ?tray ?trayloc))
	      (not (empty lgripper)) (not (empty rgripper))
	      (InGripper ?tray lgripper) (InGripper ?tray rgripper)
	      (forall (?o - movable) (forall (?p - pose) (not (obstructs ?p ?tray ?o))))
	 )
 )

 (:action pickFromTray
  :parameters(?lrobot - pose  ?tray ?obj - movable  ?obj2  - movable ?trayloc  - location ?gripper - manip)
  :precondition(and (IsTray ?tray) (Object ?obj)
  		  (RobotAt ?lrobot) (IsAccessPointFor ?lrobot ?trayloc)
		  (At ?tray ?trayloc)
		  (Topmost ?obj ?tray) (on ?obj ?obj2)
		  (empty ?gripper)
		  (forall (?o - movable) (not (obstructs ?lrobot ?o ?obj)))
		  )
  :effect(and (InGripper ?obj ?gripper) (not (ontray ?obj))
  	      (not (Topmost ?obj ?tray)) (not (on ?obj ?obj2))
	      (forall (?o - movable) (forall (?p - pose) (not (obstructs ?p ?obj ?o))))
	      (topmost ?obj2 ?tray) (not (empty ?gripper))
	      )
)

 (:action putDownTray
  :parameters(?lrobot  - pose ?tray  - movable ?ltarget - location )
  :precondition (and (IsTray ?tray) 
  		     (InGripper ?tray lgripper) (InGripper ?tray rgripper) (TrayLocation ?ltarget)
		     (RobotAt ?lrobot) (IsAccessPointFor ?lrobot ?ltarget) 
		     )
  :effect (and (not (InGripper ?tray lgripper)) (not (InGripper ?tray rgripper)) (At ?tray ?ltarget) 
  	        (empty lgripper) (empty rgripper) 
	      )
 )

 (:action putDown
  :parameters(?o1 - movable ?l1 - location  ?l2 - pose ?gripper - manip)
  :precondition (and  (Object ?o1)
  		     (InGripper ?o1 ?gripper) (RobotAt ?l2) (TempArea ?l1)
		     (IsAccessPointFor ?l2 ?l1 ))
  :effect (and (not (InGripper ?o1 ?gripper)) (At ?o1 ?l1) (empty ?gripper)
	      )
 )

  (:action moveto_1
     :parameters(?l1  - pose ?l2 - pose )
     :precondition (and (RobotAt ?l1)
     		   	(InRoom1 ?l1) (InRoom1 ?l2)
     		   	)
     :effect (and (not (RobotAt ?l1)) (RobotAt ?l2) 
  ))

  (:action moveto_2
     :parameters(?l1 - pose  ?l2 - pose )
     :precondition (and  (RobotAt ?l1)
     		   	(InRoom2 ?l1) (InRoom2 ?l2)
     		   	)
     :effect (and (not (RobotAt ?l1)) (RobotAt ?l2) 
  ))

  ; (:action moveto_12
  ;    :parameters(?l1 - pose  ?l2 - pose )
  ;    :precondition (and (RobotAt ?l1))
  ;    :effect (and (not (RobotAt ?l1)) (RobotAt ?l2)
  ;   	      (increase (total-cost) 10)
  ;    	     )
  ; )

; (:action moveToAcrossRooms01Tray
;    :parameters(?l1 - pose ?l2 - pose ?o - movable)
;    :precondition (and  (RobotAt ?l1) (IsTray ?o) (InGripper ?o lgripper)
;    		   	   	  (= ?l2 door1)
;    				         (InRoom1 ?l1)
; 							)
;    :effect (and (not (RobotAt ?l1)) (RobotAt ?l2)
;    	                 )
; )


(:action moveToAcrossRooms01
    :parameters(?l1 - pose ?l2 - pose)
    :precondition (and  (RobotAt ?l1) 
    		   	   	  (= ?l2 door1)
    				         (InRoom1 ?l1)
 							)
    :effect (and (not (RobotAt ?l1)) (RobotAt ?l2)
    	                 )
)


   (:action moveToAcrossRooms12
      :parameters(?l1  - pose ?l2 - pose)
      :precondition (and (= ?l1 door1) (= ?l2 door2) (RobotAt ?l1)
      		   	       )
      :effect (and (not (RobotAt ?l1)) (RobotAt ?l2)
      	                 )
   )

   (:action moveToAcrossRooms23
      :parameters(?l1  - pose ?l2 - pose)
      :precondition (and (= ?l1 door2) (= ?l2 door3) (RobotAt ?l1)
      		   	       )
      :effect (and (not (RobotAt ?l1)) (RobotAt ?l2)
      	                 )
   )
   (:action moveToAcrossRooms34
      :parameters(?l1  - pose ?l2 - pose)
      :precondition (and (= ?l1 door3) (= ?l2 door4) (RobotAt ?l1)
      		   	       )
      :effect (and (not (RobotAt ?l1)) (RobotAt ?l2))
   )

   (:action moveToAcrossRooms45
      :parameters(?l1  - pose ?l2 - pose)
      :precondition (and (= ?l1 door4) (= ?l2 door5) (RobotAt ?l1)
      		   	       )
      :effect (and (not (RobotAt ?l1)) (RobotAt ?l2))
   )
   (:action moveToAcrossRooms56
      :parameters(?l1  - pose ?l2 - pose)
      :precondition (and (= ?l1 door5) (= ?l2 door6) (RobotAt ?l1)
      		   	       )
      :effect (and (not (RobotAt ?l1)) (RobotAt ?l2))
   )
   (:action moveToAcrossRooms67
      :parameters(?l1  - pose ?l2 - pose)
      :precondition (and (= ?l1 door6) (= ?l2 door7) (RobotAt ?l1)
      		   	       )
      :effect (and (not (RobotAt ?l1)) (RobotAt ?l2))
   )
   (:action moveToAcrossRooms78
      :parameters(?l1  - pose ?l2 - pose)
      :precondition (and (= ?l1 door7) (= ?l2 door8) (RobotAt ?l1)
      		   	       )
      :effect (and (not (RobotAt ?l1)) (RobotAt ?l2))
   )
   (:action moveToAcrossRooms89
      :parameters(?l1  - pose ?l2 - pose)
      :precondition (and (= ?l1 door8) (= ?l2 door9) (RobotAt ?l1)
      		   	       )
      :effect (and (not (RobotAt ?l1)) (RobotAt ?l2))
   )

   (:action moveToAcrossRooms9T
      :parameters(?l1 - pose ?l2 - pose)
      :precondition (and (= ?l1 door9) (RobotAt ?l1)
      		   	       (InRoom2 ?l2)
   						)
      :effect (and (not (RobotAt ?l1)) (RobotAt ?l2))
   )

   (:action moveToAcrossRoomsT9
      :parameters(?l1  - pose ?l2 - pose)
      :precondition (and (= ?l2 door9)  (RobotAt ?l1)
      		   	       (InRoom2 ?l1)
   						)
      :effect (and (not (RobotAt ?l1)) (RobotAt ?l2))
   )
   (:action moveToAcrossRooms98
      :parameters(?l1  - pose ?l2 - pose)
      :precondition (and (= ?l1 door9) (= ?l2 door8) (RobotAt ?l1)
      		   	       )
      :effect (and (not (RobotAt ?l1)) (RobotAt ?l2))
   )

   (:action moveToAcrossRooms87
      :parameters(?l1  - pose ?l2 - pose)
      :precondition (and (= ?l1 door8) (= ?l2 door7) (RobotAt ?l1)
      		   	       )
      :effect (and (not (RobotAt ?l1)) (RobotAt ?l2))
   )
   (:action moveToAcrossRooms76
      :parameters(?l1  - pose ?l2 - pose)
      :precondition (and (= ?l1 door7) (= ?l2 door6) (RobotAt ?l1)
      		   	       )
      :effect (and (not (RobotAt ?l1)) (RobotAt ?l2))
   )

   (:action moveToAcrossRooms65
      :parameters(?l1  - pose ?l2 - pose)
      :precondition (and (= ?l1 door6) (= ?l2 door5) (RobotAt ?l1)
      		   	       )
      :effect (and (not (RobotAt ?l1)) (RobotAt ?l2))
   )
   (:action moveToAcrossRooms54
      :parameters(?l1  - pose ?l2 - pose)
      :precondition (and (= ?l1 door5) (= ?l2 door4) (RobotAt ?l1)
      		   	       )
      :effect (and (not (RobotAt ?l1)) (RobotAt ?l2))
   )

   (:action moveToAcrossRooms43
      :parameters(?l1  - pose ?l2 - pose)
      :precondition (and (= ?l1 door4) (= ?l2 door3) (RobotAt ?l1)
      		   	       )
      :effect (and (not (RobotAt ?l1)) (RobotAt ?l2)
      	                 )
   )

   (:action moveToAcrossRooms32
      :parameters(?l1  - pose  ?l2  - pose )
      :precondition (and (= ?l1 door3) (= ?l2 door2) (RobotAt ?l1)
      		   	       )
      :effect (and (not (RobotAt ?l1)) (RobotAt ?l2)
      	                 )
   )

   (:action moveToAcrossRooms21
      :parameters(?l1   - pose ?l2  - pose )
      :precondition (and (= ?l1 door2) (= ?l2 door1) (RobotAt ?l1))
      :effect (and (not (RobotAt ?l1)) (RobotAt ?l2)
      	                 )
   )

   (:action moveToAcrossRooms10
      :parameters(?l1   - pose ?l2  - pose )
      :precondition (and (= ?l1 door1)  (RobotAt ?l1)
      		   	       (InRoom1 ?l2)
   						)
      :effect (and (not (RobotAt ?l1)) (RobotAt ?l2)
      	                 )
   )

 (:action grasp
     :parameters(?o1 - movable ?l1 - location ?l2 - pose  ?gripper - manip)
     :precondition(and  (At  ?o1 ?l1) (Object ?o1)
     		       (RobotAt ?l2) (IsGP ?l2 ?o1 ?gripper)
		       (empty ?gripper)
		       (forall (?o - movable) (not (obstructs ?l2 ?o ?o1)))
		       )
     :effect (and (InGripper ?o1 ?gripper) (not (At ?o1 ?l1)) (not (empty ?gripper))
     	     	  (forall (?o - movable) (forall (?p - pose) (not (obstructs ?p ?o1 ?o))))
 	      )
  )
)
