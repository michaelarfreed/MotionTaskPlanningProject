(define (problem dinnerTime1) 
(:domain robotics)
(:objects
	object11 object12 object21 object22 object13   object23 tray None - movable
	
	loc_object11 loc_object21 loc_object12	loc_object22 loc_object13 loc_object23 
	rarm_gp_object11 rarm_gp_object21 rarm_gp_object12 rarm_gp_object22 rarm_gp_object13 rarm_gp_object23  
    	;larm_gp_object11 larm_gp_object21 larm_gp_object12 larm_gp_object22 larm_gp_object13 larm_gp_object23  
	- location


	trayLoc1 trayLoc2   rarm_blf_trayLoc1 rarm_blf_trayLoc2	

	robotInitLoc

	destobject11
	destobject12
	destobject21
	destobject22
	destobject13
	destobject23

	rarm_blf_destobject11
	rarm_blf_destobject12
	rarm_blf_destobject21
	rarm_blf_destobject22
	rarm_blf_destobject13
	rarm_blf_destobject23 - location

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

(TempArea destobject11)
(TempArea destobject12)
(TempArea destobject21)
(TempArea destobject22)
(TempArea destobject13)
(TempArea destobject23)

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

; (InRoom1 larm_gp_object11)
; (InRoom1 larm_gp_object21)
; (InRoom1 larm_gp_object12)
; (InRoom1 larm_gp_object22)
; (InRoom1 larm_gp_object13)
; (InRoom1 larm_gp_object23)

(InRoom1 trayLoc1)
(InRoom1 robotInitLoc)
(InRoom1 rarm_blf_trayLoc1)

(InRoom2 trayLoc2)
(InRoom2 rarm_blf_trayLoc2)

(InRoom2	destobject11)
(InRoom2 	destobject12)
(InRoom2 	destobject21)
(InRoom2 	destobject22)
(InRoom2 	destobject13)
(InRoom2 	destobject23)

(InRoom2 	rarm_blf_destobject11)
(InRoom2 	rarm_blf_destobject12)
(InRoom2 	rarm_blf_destobject21)
(InRoom2 	rarm_blf_destobject22)
(InRoom2 	rarm_blf_destobject13)
(InRoom2 	rarm_blf_destobject23)




(At object11 loc_object11)
(At object21 loc_object21)
(At object12 loc_object12)
(At object22 loc_object22)
(At object13 loc_object13)
(At object23 loc_object23)
(At tray trayLoc1)
(RobotAt robotInitLoc)


(IsAccessPointFor rarm_blf_trayLoc1 trayLoc1 )
(IsAccessPointFor rarm_blf_trayLoc2 trayLoc2 )

(IsAccessPointFor rarm_blf_destobject11 destobject11 )
(IsAccessPointFor rarm_blf_destobject12 destobject12 )
(IsAccessPointFor rarm_blf_destobject21 destobject21 )
(IsAccessPointFor rarm_blf_destobject22 destobject22 )
(IsAccessPointFor rarm_blf_destobject13 destobject13 )
(IsAccessPointFor rarm_blf_destobject23 destobject23 )

(IsGP rarm_gp_object11 object11 rgripper)
(IsGP rarm_gp_object21 object21 rgripper)
(IsGP rarm_gp_object12 object12 rgripper)
(IsGP rarm_gp_object22 object22 rgripper)
(IsGP rarm_gp_object13 object13 rgripper)
(IsGP rarm_gp_object23 object23 rgripper)

; (IsGP larm_gp_object11 object11 lgripper)
; (IsGP larm_gp_object21 object21 lgripper)
; (IsGP larm_gp_object12 object12 lgripper)
; (IsGP larm_gp_object22 object22 lgripper)
; (IsGP larm_gp_object13 object13 lgripper)
; (IsGP larm_gp_object23 object23 lgripper)

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

(Smaller object23 object12)
(Smaller object23 object21)
(Smaller object23 object22)
(Smaller object23 object13)
(Smaller object23 object11)

(Smaller object11 none)
(Smaller object21 none)
(Smaller  object13 none)
(Smaller  object12 none)
(Smaller  object22 none)
(Smaller  object23 none)

(Topmost None tray)
)

(:goal (and 
	      (At object11 destobject11)
       	      (At object12 destobject12)
	      (At object21 destobject21)
	      (At object22 destobject22)
	      (At object13 destobject13)
	      (At object23 destobject23)
	    ))

(:metric minimize (total-cost))

)

