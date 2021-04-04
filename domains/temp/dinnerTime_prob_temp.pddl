(define (problem dinnerTime1) 
(:domain robotics)
(:objects
	object11 object12 object21
	object22 object31   object32
	
	loc_object11 loc_object21 loc_object12
	loc_object22 loc_object31 loc_object32 

	gp_object11 gp_object21 gp_object12
	gp_object22 gp_object31 gp_object32  

	targettable sourcetable
	tray 
	trayLoc1 
	trayLoc2	
	
	blf_tray
	blf_targettable
	blf_sourcetable	
	blf_trayLoc1 
	blf_trayLoc2	
	None


	table1
	blf_table1

	table2
	blf_table2

	table3
	blf_table3

	table4
	blf_table4
	
	table6
	blf_table6

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
	blf_dest_object32

)

(:init

(empty gripper)
(Object object11)
(Object object21)
(Object object12)
(Object object22)
(Object object31)
(Object object32)
(Object None)
(IsTray tray)

(Location loc_object11)
(Location loc_object21)
(Location loc_object12)
(Location loc_object22)
(Location loc_object31)
(Location loc_object32)
(Location trayLoc1)
(Location trayLoc2)
(Location table6)
(Location table1)
(Location table2)
(Location table3)
(Location table4)
(Location table1)
(TempArea table6)
(TempArea table1)
(TempArea table2)
(TempArea table3)
(TempArea table4)

(Location dest_object11)
(Location dest_object12)
(Location dest_object21)
(Location dest_object22)
(Location dest_object31)
(Location dest_object32)
(Location blf_table1)

(Location blf_dest_object11)
(Location blf_dest_object12)
(Location blf_dest_object21)
(Location blf_dest_object22)
(Location blf_dest_object31)
(Location blf_dest_object32)

(TempArea dest_object11)
(TempArea dest_object12)
(TempArea dest_object21)
(TempArea dest_object22)
(TempArea dest_object31)
(TempArea dest_object32)
(TempArea table6)



(TrayLocation trayLoc1)
(TrayLocation trayLoc2)

(Location gp_object11)
(Location gp_object21)
(Location gp_object12)
(Location gp_object22)
(Location gp_object31)
(Location gp_object32)

(Location targettable)
(Location sourcetable)
(Location blf_targettable)
(Location blf_sourcetable)
(Location robotInitLoc)
(Location blf_trayLoc1)
(Location blf_trayLoc2)
(Location blf_table6)
(Location blf_table1)
(Location blf_table2)
(Location blf_table3)
(Location blf_table4)



(InRoom1 loc_object11)
(InRoom1 loc_object21)
(InRoom1 loc_object12)
(InRoom1 loc_object22)
(InRoom1 loc_object31)
(InRoom1 loc_object32)
(InRoom1 gp_object11)
(InRoom1 gp_object21)
(InRoom1 gp_object12)
(InRoom1 gp_object22)
(InRoom1 gp_object31)
(InRoom1 gp_object32)

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

(IsGP gp_object11 object11)
(IsGP gp_object21 object21)
(IsGP gp_object12 object12)
(IsGP gp_object22 object22)
(IsGP gp_object31 object31)
(IsGP gp_object32 object32)

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

(:goal (and (At object11 dest_object11)
       	    (At object12 dest_object12)
	    (At object21 dest_object21)
	    (At object22 dest_object22)
	    (At object31 dest_object31)
	    (At object32 dest_object32)
;	    (InGripper tray)
;	    (RobotAt blf_trayLoc2)
	    ))


)

