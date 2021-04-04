(define (problem dinnerTime1) 
(:domain robotics)
(:objects
	object11 object12 object21 object22 object13   object23 tray None - movable
	
	loc_object11 loc_object21 loc_object12	loc_object22 loc_object13 loc_object23 
	rarm_gp_object11 rarm_gp_object21 rarm_gp_object12 rarm_gp_object22 rarm_gp_object13 rarm_gp_object23  - location

	targettable sourcetable	trayLoc1 trayLoc2 blf_tray blf_targettable blf_sourcetable blf_trayLoc1 blf_trayLoc2	
	table1 blf_table1 table2 blf_table2 table3 blf_table3 table4 blf_table4	table6 blf_table6 - location

	robotInitLoc

	dest_object11
	dest_object12
	dest_object21
	dest_object22
	dest_object13
	dest_object23

	blf_dest_object11
	blf_dest_object12
	blf_dest_object21
	blf_dest_object22
	blf_dest_object13
	blf_dest_object23 - location

)

(:init
(= (total-cost) 0)


(Object object11)
(Object object12)
(Object object21)
(Object object22)
(Object object13)
(Object object23)


(empty lgripper)
(empty rgripper)
(IsTray tray)

; (TempArea table6)
; (TempArea table1)
; (TempArea table2)
; (TempArea table3)
; (TempArea table4)


(TempArea dest_object11)
(TempArea dest_object12)
(TempArea dest_object21)
(TempArea dest_object22)
(TempArea dest_object13)
(TempArea dest_object23)



(TrayLocation trayLoc1)
(TrayLocation trayLoc2)



(InRoom1 loc_object11)
(InRoom1 loc_object21)
(InRoom1 loc_object12)
(InRoom1 loc_object22)
(InRoom1 loc_object13)
(InRoom1 loc_object23)

(InRoom1 rarm_gp_object11)
(InRoom1 rarm_gp_object21)
(InRoom1 rarm_gp_object12)
(InRoom1 rarm_gp_object22)
(InRoom1 rarm_gp_object13)
(InRoom1 rarm_gp_object23)

(InRoom1 trayLoc1)
(InRoom1 table6)
(InRoom1 robotInitLoc)
(InRoom1 blf_trayLoc1)
(InRoom1 blf_table6)

(InRoom2 table1)
(InRoom2 blf_table1)
(InRoom2 table2)
(InRoom2 blf_table2)
(InRoom2 table3)
(InRoom2 blf_table3)
(InRoom2 table4)
(InRoom2 blf_table4)
(InRoom2 trayLoc2)
(InRoom2 blf_trayLoc2)

(InRoom2 dest_object11)
(InRoom2 	dest_object12)
(InRoom2 	dest_object21)
(InRoom2 	dest_object22)
(InRoom2 	dest_object13)
(InRoom2 	dest_object23)

(InRoom2 blf_dest_object11)
(InRoom2 	blf_dest_object12)
(InRoom2 	blf_dest_object21)
(InRoom2 	blf_dest_object22)
(InRoom2 	blf_dest_object13)
(InRoom2 	blf_dest_object23)


( at object23  loc_object23)
( at tray  trayloc1)
( on object11  none)
( on object12  object11)
( on object13  object12)
( on object21  object13)
( on object22  object21)
( empty lgripper)
( nheavy tray)
( topmost object22  tray)
( empty rgripper)
( robotat blf_trayloc1)
;(not (smaller object23 object22))

(At object23 loc_object23)
(At tray trayLoc1)


(IsAccessPointFor blf_table1 table1)

(IsAccessPointFor blf_sourcetable sourcetable)
(IsAccessPointFor blf_targettable targettable)
(IsAccessPointFor blf_trayLoc1 trayLoc1)
(IsAccessPointFor blf_trayLoc2 trayLoc2)
(IsAccessPointFor blf_table6 table6)
(IsAccessPointFor blf_table1 table1)
(IsAccessPointFor blf_table2 table2)
(IsAccessPointFor blf_table3 table3)
(IsAccessPointFor blf_table4 table4)
(IsAccessPointFor blf_dest_object11 dest_object11)
(IsAccessPointFor blf_dest_object12 dest_object12)
(IsAccessPointFor blf_dest_object21 dest_object21)
(IsAccessPointFor blf_dest_object22 dest_object22)
(IsAccessPointFor blf_dest_object13 dest_object13)
(IsAccessPointFor blf_dest_object23 dest_object23)

(IsAccessPointForTray blf_tray tray)


(IsGP rarm_gp_object11 object11 rgripper)
(IsGP rarm_gp_object21 object21 rgripper)
(IsGP rarm_gp_object12 object12 rgripper)
(IsGP rarm_gp_object22 object22 rgripper)
(IsGP rarm_gp_object13 object13 rgripper)
(IsGP rarm_gp_object23 object23 rgripper)

(Smaller object11 object12)
(Smaller object11 object21)
(Smaller object11 object22)
(Smaller object11 object13)
(Smaller object11 object23)

(Smaller object12 object11)
(Smaller object12 object21)
(Smaller object12 object22)
(Smaller object12 object13)
(Smaller object12 object23)

(Smaller object21 object12)
(Smaller object21 object11)
(Smaller object21 object22)
(Smaller object21 object13)
(Smaller object21 object23)

(Smaller object22 object12)
(Smaller object22 object21)
(Smaller object22 object11)
(Smaller object22 object13)
(Smaller object22 object23)

(Smaller object13 object12)
(Smaller object13 object21)
(Smaller object13 object22)
(Smaller object13 object11)
(Smaller object13 object23)

;(Smaller object23 object12)
;(Smaller object23 object21)
;(Smaller object23 object22)
;(Smaller object23 object13)
;(Smaller object23 object11)

(Smaller object11 none)
(Smaller object21 none)
(Smaller  object13 none)
(Smaller  object12 none)
(Smaller  object22 none)
(Smaller  object23 none)


)

(:goal (and 
	     (At object11 dest_object11)
       	     (At object12 dest_object12)
	     (At object21 dest_object21)
	     (At object22 dest_object22)
	     (At object13 dest_object13)
	     (At object23 dest_object23)
	    ))

(:metric minimize (total-cost))

)

