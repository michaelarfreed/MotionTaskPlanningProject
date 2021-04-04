(define (problem dinnerTimeHandoff1) (:domain robotics)
(:objects
object1 object2 object3 object4 tray None - movable

loc_object1 loc_object2 loc_object3 loc_object4 - location
trayLoc1 trayLoc2 - location
destobject1 destobject2 destobject3 destobject4 - location

larm_gp_object1 larm_gp_object2 larm_gp_object3 larm_gp_object4 larm_blf_trayLoc1 - pose
rarm_gp_object1 rarm_gp_object2 rarm_gp_object3 rarm_gp_object4 rarm_blf_trayLoc1 - pose
larm_blf_destobject1 larm_blf_destobject2 larm_blf_destobject3 larm_blf_destobject4 larm_blf_trayLoc2 - pose
rarm_blf_destobject1 rarm_blf_destobject2 rarm_blf_destobject3 rarm_blf_destobject4 rarm_blf_trayLoc2 - pose
robotInitLoc - pose

)

(:init
(At object1 loc_object1)
(At object2 loc_object2)
(At object3 loc_object3)
(At object4 loc_object4)
(At tray trayLoc1)
(RobotAt robotInitLoc)


(Object object1)
(Object object2)
(Object object3)
(Object object4)

(TempArea destobject1)
(TempArea destobject2)
(TempArea destobject3)
(TempArea destobject4)

(Clear destobject1)
(Clear destobject2)
(Clear destobject3)
(Clear destobject4)

(TrayLocation trayLoc1)
(TrayLocation trayLoc2)

(empty rgripper)
(empty lgripper)
(IsTray tray)

(InRoom1 robotInitLoc)

(InRoom1 larm_gp_object1)
(InRoom1 larm_gp_object2)
(InRoom1 larm_gp_object3)
(InRoom1 larm_gp_object4)
(InRoom1 larm_blf_trayLoc1)
(InRoom1 rarm_gp_object1)
(InRoom1 rarm_gp_object2)
(InRoom1 rarm_gp_object3)
(InRoom1 rarm_gp_object4)
(InRoom1 rarm_blf_trayLoc1)

(InRoom2 larm_blf_destobject1)
(InRoom2 larm_blf_destobject2)
(InRoom2 larm_blf_destobject3)
(InRoom2 larm_blf_destobject4)
(InRoom2 larm_blf_trayLoc2)
(InRoom2 rarm_blf_destobject1)
(InRoom2 rarm_blf_destobject2)
(InRoom2 rarm_blf_destobject3)
(InRoom2 rarm_blf_destobject4)
(InRoom2 rarm_blf_trayLoc2)

(IsGP larm_gp_object1 object1 lgripper)
(IsGP larm_gp_object2 object2 lgripper)
(IsGP larm_gp_object3 object3 lgripper)
(IsGP larm_gp_object4 object4 lgripper)
(IsGP rarm_gp_object1 object1 rgripper)
(IsGP rarm_gp_object2 object2 rgripper)
(IsGP rarm_gp_object3 object3 rgripper)
(IsGP rarm_gp_object4 object4 rgripper)

(IsAccessPointForTray larm_blf_trayLoc1 trayLoc1)
(IsAccessPointForTray larm_blf_trayLoc2 trayLoc2)
(IsAccessPointForTray rarm_blf_trayLoc1 trayLoc1)
(IsAccessPointForTray rarm_blf_trayLoc2 trayLoc2)

(IsAccessPointFor larm_blf_destobject1 destobject1 lgripper)
(IsAccessPointFor larm_blf_destobject2 destobject2 lgripper)
(IsAccessPointFor larm_blf_destobject3 destobject3 lgripper)
(IsAccessPointFor larm_blf_destobject4 destobject4 lgripper)
(IsAccessPointFor rarm_blf_destobject1 destobject1 rgripper)
(IsAccessPointFor rarm_blf_destobject2 destobject2 rgripper)
(IsAccessPointFor rarm_blf_destobject3 destobject3 rgripper)
(IsAccessPointFor rarm_blf_destobject4 destobject4 rgripper)

(IsAccessPointFor larm_blf_trayLoc1 trayLoc1 lgripper)
(IsAccessPointFor larm_blf_trayLoc2 trayLoc2 lgripper)
(IsAccessPointFor rarm_blf_trayLoc1 trayLoc1 rgripper)
(IsAccessPointFor rarm_blf_trayLoc2 trayLoc2 rgripper)

(Smaller object1 object2)
(Smaller object1 object4)
(Smaller object1 object3)

(Smaller object2 object1)
(Smaller object2 object4)
(Smaller object2 object3)

(Smaller object4 object2)
(Smaller object4 object1)
(Smaller object4 object3)

(Smaller object3 object2)
(Smaller object3 object4)
(Smaller object3 object1)

(Smaller object1 None)
(Smaller object4 None)
(Smaller object3 None)
(Smaller object2 None)
(Topmost None tray)

(OnLeft larm_gp_object1)
(OnLeft larm_gp_object2)
(OnLeft larm_gp_object3)
(OnLeft larm_gp_object4)
(OnLeft larm_blf_trayLoc1)

(OnLeft rarm_gp_object1)
(OnLeft rarm_gp_object2)
(OnLeft rarm_gp_object3)
(OnLeft rarm_gp_object4)
(OnLeft rarm_blf_trayLoc1)

(OnLeft larm_blf_destobject1)
(OnLeft larm_blf_destobject2)
(OnLeft larm_blf_destobject3)
(OnLeft larm_blf_destobject4)
(OnLeft larm_blf_trayLoc2)

(OnLeft rarm_blf_destobject1)
(OnLeft rarm_blf_destobject2)
(OnLeft rarm_blf_destobject3)
(OnLeft rarm_blf_destobject4)
(OnLeft rarm_blf_trayLoc2)
)

(:goal (and (at object1 destobject1) (at object2 destobject2) (at object3 destobject3) (at object4 destobject4)))
            
(:metric minimize (total-cost))

)