(define (problem dinnerTime1) 
(:domain robotics)
(:objects
	object11 object12 object21 object22 object31   object32 tray None - movable
	
	loc_object11 loc_object21 loc_object12	loc_object22 loc_object31 loc_object32 
	rarm_gp_object11 rarm_gp_object21 rarm_gp_object12 rarm_gp_object22 rarm_gp_object31 rarm_gp_object32  - location

	targettable sourcetable	trayLoc1 trayLoc2 blf_tray blf_targettable blf_sourcetable blf_trayLoc1 blf_trayLoc2	
	table1 blf_table1 table2 blf_table2 table3 blf_table3 table4 blf_table4	table6 blf_table6 - location

	robotInitLoc

	dest_object11
	dest_object12
	dest_object21
	dest_object22
	dest_object31
	dest_object32

	blf_dest_object11
	blf_dest_object12
	blf_dest_object21
	blf_dest_object22
	blf_dest_object31
	blf_dest_object32 - location

)

(:init
(= (total-cost) 0)


(Object object11)
(Object object12)
(Object object21)
(Object object22)
(Object object31)
(Object object32)


(empty lgripper)
(empty rgripper)
(IsTray tray)

(TempArea table6)
(TempArea table1)
(TempArea table2)
(TempArea table3)
(TempArea table4)


(TempArea dest_object11)
(TempArea dest_object12)
(TempArea dest_object21)
(TempArea dest_object22)
(TempArea dest_object31)
(TempArea dest_object32)
(TempArea table6)



(TrayLocation trayLoc1)
(TrayLocation trayLoc2)



(InRoom1 loc_object11)
(InRoom1 loc_object21)
(InRoom1 loc_object12)
(InRoom1 loc_object22)
(InRoom1 loc_object31)
(InRoom1 loc_object32)

(InRoom1 rarm_gp_object11)
(InRoom1 rarm_gp_object21)
(InRoom1 rarm_gp_object12)
(InRoom1 rarm_gp_object22)
(InRoom1 rarm_gp_object31)
(InRoom1 rarm_gp_object32)

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
(InRoom2 	dest_object31)
(InRoom2 	dest_object32)

(InRoom2 blf_dest_object11)
(InRoom2 	blf_dest_object12)
(InRoom2 	blf_dest_object21)
(InRoom2 	blf_dest_object22)
(InRoom2 	blf_dest_object31)
(InRoom2 	blf_dest_object32)




(At object11 loc_object11)
(At object21 loc_object21)
(At object12 loc_object12)
(At object22 loc_object22)
(At object31 loc_object31)
(At object32 loc_object32)
(At tray trayLoc1)
(RobotAt robotInitLoc)

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
(IsAccessPointFor blf_dest_object31 dest_object31)
(IsAccessPointFor blf_dest_object32 dest_object32)

(IsAccessPointForTray blf_tray tray)


(IsGP rarm_gp_object11 object11 rgripper)
(IsGP rarm_gp_object21 object21 rgripper)
(IsGP rarm_gp_object12 object12 rgripper)
(IsGP rarm_gp_object22 object22 rgripper)
(IsGP rarm_gp_object31 object31 rgripper)
(IsGP rarm_gp_object32 object32 rgripper)

(Smaller object11 object12)
(Smaller object11 object21)
(Smaller object11 object22)
(Smaller object11 object31)
(Smaller object11 object32)

(Smaller object12 object11)
(Smaller object12 object21)
(Smaller object12 object22)
(Smaller object12 object31)
(Smaller object12 object32)

(Smaller object21 object12)
(Smaller object21 object11)
(Smaller object21 object22)
(Smaller object21 object31)
(Smaller object21 object32)

(Smaller object22 object12)
(Smaller object22 object21)
(Smaller object22 object11)
(Smaller object22 object31)
(Smaller object22 object32)

(Smaller object31 object12)
(Smaller object31 object21)
(Smaller object31 object22)
(Smaller object31 object11)
(Smaller object31 object32)

(Smaller object32 object12)
(Smaller object32 object21)
(Smaller object32 object22)
(Smaller object32 object31)
(Smaller object32 object11)

(Smaller object11 none)
(Smaller object21 none)
(Smaller  object31 none)
(Smaller  object12 none)
(Smaller  object22 none)
(Smaller  object32 none)

(Topmost None tray)
)

(:goal (and 
	     (At object11 dest_object11)
       	     (At object12 dest_object12)
	     (At object21 dest_object21)
	     (At object22 dest_object22)
	     (At object31 dest_object31)
	     (At object32 dest_object32)
	    ))

(:metric minimize (total-cost))

)

