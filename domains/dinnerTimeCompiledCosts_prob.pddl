(define (problem dinnerTime1) 
(:domain robotics)
(:objects
	object11 object12 object21
	object22 object23   object13
	
	loc11 loc21 loc12
	loc22 loc31 loc32 

	gp_object11 gp_object21 gp_object12
	gp_object22 gp_object23 gp_object13  

	targettable sourcetable
	tray 
	trayLoc1 
	trayLoc2	
	
	blf_tray
	blf_targettable
	blf_sourcetable	
	blf_trayLoc1 
	;blf_trayLoc2	
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

	destobject11
	destobject12
	destobject21
	destobject22
	destobject23
	destobject13

pdp_object11_loc11
pdp_object12_loc12
pdp_object21_loc21
pdp_object22_loc22
pdp_object23_loc31
pdp_object13_loc32

	blf_destobject11
	blf_destobject12
	blf_destobject21
	blf_destobject22
	blf_destobject23
	blf_destobject13

)

(:init

(empty gripper)
(Object object11)
(Object object21)
(Object object12)
(Object object22)
(Object object23)
(Object object13)
(Object None)
(IsTray tray)

(Location pdp_object11_loc11)
(Location pdp_object12_loc12)
(Location pdp_object21_loc21)
(Location pdp_object22_loc22)
(Location pdp_object23_loc31)
(Location pdp_object13_loc32)

(Location loc11)
(Location loc21)
(Location loc12)
(Location loc22)
(Location loc31)
(Location loc32)
(Location trayLoc1)
(Location trayLoc2)

(Location table1)
(Location table2)
(Location table3)
(Location table4)
(Location table1)

(TempArea loc11)
(TempArea loc12)
(TempArea loc21)
(TempArea loc22)
(TempArea loc31)
(TempArea loc32)

(Location destobject11)
(Location destobject12)
(Location destobject21)
(Location destobject22)
(Location destobject23)
(Location destobject13)
(Location blf_table1)


(clear destobject11)
(clear destobject12)
(clear destobject21)
(clear destobject22)
(clear destobject23)
(clear destobject13)


(Location blf_destobject11)
(Location blf_destobject12)
(Location blf_destobject21)
(Location blf_destobject22)
(Location blf_destobject23)
(Location blf_destobject13)

(TempArea destobject11)
(TempArea destobject12)
(TempArea destobject21)
(TempArea destobject22)
(TempArea destobject23)
(TempArea destobject13)
(TempArea table6)



(TrayLocation trayLoc1)
(TrayLocation trayLoc2)

(Location gp_object11)
(Location gp_object21)
(Location gp_object12)
(Location gp_object22)
(Location gp_object23)
(Location gp_object13)

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



(InRoom1 loc11)
(InRoom1 loc21)
(InRoom1 loc12)
(InRoom1 loc22)
(InRoom1 loc31)
(InRoom1 loc32)
(InRoom1 gp_object11)
(InRoom1 gp_object21)
(InRoom1 gp_object12)
(InRoom1 gp_object22)
(InRoom1 gp_object23)
(InRoom1 gp_object13)

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

(InRoom2 destobject11)
(InRoom2 	destobject12)
(InRoom2 	destobject21)
(InRoom2 	destobject22)
(InRoom2 	destobject23)
(InRoom2 	destobject13)

(InRoom2 blf_destobject11)
(InRoom2 	blf_destobject12)
(InRoom2 	blf_destobject21)
(InRoom2 	blf_destobject22)
(InRoom2 	blf_destobject23)
(InRoom2 	blf_destobject13)




(At object11 loc11)
(At object21 loc21)
(At object12 loc12)
(At object22 loc22)
(At object23 loc31)
(At object13 loc32)
(At tray trayLoc1)
(RobotAt robotInitLoc)

(IsAccessPointFor blf_table1 table1)

(IsAccessPointFor blf_sourcetable sourcetable)
(IsAccessPointFor blf_targettable targettable)
(IsAccessPointFor blf_trayLoc1 trayLoc1)
(IsAccessPointFor blf_trayLoc2 trayLoc2)

(IsAccessPointFor pdp_object11_loc11 loc11)
(IsAccessPointFor pdp_object12_loc12 loc12)
(IsAccessPointFor pdp_object21_loc21 loc21)
(IsAccessPointFor pdp_object22_loc22 loc22)
(IsAccessPointFor pdp_object23_loc31 loc31)
(IsAccessPointFor pdp_object13_loc32 loc32)


(IsAccessPointFor blf_table1 table1)
(IsAccessPointFor blf_table2 table2)
(IsAccessPointFor blf_table3 table3)
(IsAccessPointFor blf_table4 table4)
(IsAccessPointFor blf_destobject11 destobject11)
(IsAccessPointFor blf_destobject12 destobject12)
(IsAccessPointFor blf_destobject21 destobject21)
(IsAccessPointFor blf_destobject22 destobject22)
(IsAccessPointFor blf_destobject23 destobject23)
(IsAccessPointFor blf_destobject13 destobject13)


(IsAccessPointForTray blf_tray tray)

(IsGP gp_object11 object11)
(IsGP gp_object21 object21)
(IsGP gp_object12 object12)
(IsGP gp_object22 object22)
(IsGP gp_object23 object23)
(IsGP gp_object13 object13)

(Smaller object11 object12)
(Smaller object11 object21)
(Smaller object11 object22)
(Smaller object11 object23)
(Smaller object11 object13)

(Smaller object12 object11)
(Smaller object12 object21)
(Smaller object12 object22)
(Smaller object12 object23)
(Smaller object12 object13)

(Smaller object21 object12)
(Smaller object21 object11)
(Smaller object21 object22)
(Smaller object21 object23)
(Smaller object21 object13)

(Smaller object22 object12)
(Smaller object22 object21)
(Smaller object22 object11)
(Smaller object22 object23)
(Smaller object22 object13)

(Smaller object23 object12)
(Smaller object23 object21)
(Smaller object23 object22)
(Smaller object23 object11)
(Smaller object23 object13)

(Smaller object13 object12)
(Smaller object13 object21)
(Smaller object13 object22)
(Smaller object13 object23)
(Smaller object13 object11)

(Smaller object11 none)
(Smaller object21 none)
(Smaller  object23 none)
(Smaller  object12 none)
(Smaller  object22 none)
(Smaller  object13 none)

(Topmost None tray)
)

(:goal (and (At object11 destobject11)
       	    (At object12 destobject12)
	    (At object21 destobject21)
	    (At object22 destobject22)
	    (At object23 destobject23)
	    (At object13 destobject13)

	    ))


)

