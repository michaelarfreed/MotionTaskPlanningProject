(define (problem dinnerTimeHandoff1) (:domain robotics)
(:objects
object1 object2 tray None - movable

loc_object1 loc_object2 - location
trayLoc1 trayLoc2 - location
destobject1 destobject2 - location

larm_gp_object1 larm_gp_object2 larm_blf_trayLoc1 - pose
rarm_gp_object1 rarm_gp_object2 rarm_blf_trayLoc1 - pose
larm_blf_destobject1 larm_blf_destobject2 larm_blf_trayLoc2 - pose
rarm_blf_destobject1 rarm_blf_destobject2 rarm_blf_trayLoc2 - pose
robotInitLoc - pose

)

(:init
(At object1 loc_object1)
(At object2 loc_object2)
(At tray trayLoc1)
(RobotAt robotInitLoc)


(Object object1)
(Object object2)

(TempArea destobject1)
(TempArea destobject2)

(Clear destobject1)
(Clear destobject2)

(TrayLocation trayLoc1)
(TrayLocation trayLoc2)

(empty rgripper)
(empty lgripper)
(IsTray tray)

(InRoom1 robotInitLoc)

(InRoom1 larm_gp_object1)
(InRoom1 larm_gp_object2)
(InRoom1 larm_blf_trayLoc1)
(InRoom1 rarm_gp_object1)
(InRoom1 rarm_gp_object2)
(InRoom1 rarm_blf_trayLoc1)

(InRoom2 larm_blf_destobject1)
(InRoom2 larm_blf_destobject2)
(InRoom2 larm_blf_trayLoc2)
(InRoom2 rarm_blf_destobject1)
(InRoom2 rarm_blf_destobject2)
(InRoom2 rarm_blf_trayLoc2)

(IsGP larm_gp_object1 object1 lgripper)
(IsGP larm_gp_object2 object2 lgripper)
(IsGP rarm_gp_object1 object1 rgripper)
(IsGP rarm_gp_object2 object2 rgripper)

(IsAccessPointForTray larm_blf_trayLoc1 trayLoc1)
(IsAccessPointForTray larm_blf_trayLoc2 trayLoc2)
(IsAccessPointForTray rarm_blf_trayLoc1 trayLoc1)
(IsAccessPointForTray rarm_blf_trayLoc2 trayLoc2)

(IsAccessPointFor larm_blf_destobject1 destobject1 lgripper)
(IsAccessPointFor larm_blf_destobject2 destobject2 lgripper)
(IsAccessPointFor rarm_blf_destobject1 destobject1 rgripper)
(IsAccessPointFor rarm_blf_destobject2 destobject2 rgripper)

(IsAccessPointFor larm_blf_trayLoc1 trayLoc1 lgripper)
(IsAccessPointFor larm_blf_trayLoc2 trayLoc2 lgripper)
(IsAccessPointFor rarm_blf_trayLoc1 trayLoc1 rgripper)
(IsAccessPointFor rarm_blf_trayLoc2 trayLoc2 rgripper)

(Smaller object1 object2)

(Smaller object2 object1)

(Smaller object1 None)
(Smaller object2 None)
(Topmost None tray)

(OnLeft larm_gp_object1)
(OnLeft larm_gp_object2)
(OnLeft larm_blf_trayLoc1)

(OnLeft rarm_gp_object1)
(OnLeft rarm_gp_object2)
(OnLeft rarm_blf_trayLoc1)

(OnLeft larm_blf_destobject1)
(OnLeft larm_blf_destobject2)
(OnLeft larm_blf_trayLoc2)

(OnLeft rarm_blf_destobject1)
(OnLeft rarm_blf_destobject2)
(OnLeft rarm_blf_trayLoc2)
)

(:goal (and (at object1 destobject1) (at object2 destobject2)))
            
(:metric minimize (total-cost))

)