(define (problem dinnerTimeHandoff1) (:domain robotics)
(:objects
object11 object12 object13 object21 object22 object23 tray None - movable

loc_object11 loc_object12 loc_object13 loc_object21 loc_object22 loc_object23 - location
trayLoc1 trayLoc2 - location
destobject11 destobject12 destobject13 destobject21 destobject22 destobject23 - location

larm_gp_object11 larm_gp_object12 larm_gp_object13 larm_gp_object21 larm_gp_object22  larm_gp_object23 larm_blf_trayLoc1 - pose
rarm_gp_object11 rarm_gp_object12 rarm_gp_object13 rarm_gp_object21 rarm_gp_object22  rarm_gp_object23 rarm_blf_trayLoc1 - pose
larm_blf_destobject11 larm_blf_destobject12 larm_blf_destobject13 larm_blf_destobject21 larm_blf_destobject22 larm_blf_destobject23 larm_blf_trayLoc2 - pose
rarm_blf_destobject11 rarm_blf_destobject12 rarm_blf_destobject13 rarm_blf_destobject21 rarm_blf_destobject22 rarm_blf_destobject23 rarm_blf_trayLoc2 - pose
robotInitLoc - pose

)

(:init
(At object11 loc_object11)
(At object12 loc_object12)
(At object13 loc_object13)
(At object21 loc_object21)
(At object22 loc_object22)
(At object23 loc_object23)
(At tray trayLoc1)
(RobotAt robotInitLoc)


(Object object11)
(Object object12)
(Object object13)
(Object object21)
(Object object22)
(Object object23)

(TempArea destobject11)
(TempArea destobject12)
(TempArea destobject13)
(TempArea destobject21)
(TempArea destobject22)
(TempArea destobject23)

(TrayLocation trayLoc1)
(TrayLocation trayLoc2)

(empty rgripper)
(empty lgripper)
(IsTray tray)

(InRoom1 robotInitLoc)

(InRoom1 larm_gp_object11)
(InRoom1 larm_gp_object12)
(InRoom1 larm_gp_object13)
(InRoom1 larm_gp_object21)
(InRoom1 larm_gp_object22)
(InRoom1 larm_gp_object23)
(InRoom1 larm_blf_trayLoc1)
(InRoom1 rarm_gp_object11)
(InRoom1 rarm_gp_object12)
(InRoom1 rarm_gp_object13)
(InRoom1 rarm_gp_object21)
(InRoom1 rarm_gp_object22)
(InRoom1 rarm_gp_object23)
(InRoom1 rarm_blf_trayLoc1)

(InRoom2 larm_blf_destobject11)
(InRoom2 larm_blf_destobject12)
(InRoom2 larm_blf_destobject13)
(InRoom2 larm_blf_destobject21)
(InRoom2 larm_blf_destobject22)
(InRoom2 larm_blf_destobject23)
(InRoom2 larm_blf_trayLoc2)
(InRoom2 rarm_blf_destobject11)
(InRoom2 rarm_blf_destobject12)
(InRoom2 rarm_blf_destobject13)
(InRoom2 rarm_blf_destobject21)
(InRoom2 rarm_blf_destobject22)
(InRoom2 rarm_blf_destobject23)
(InRoom2 rarm_blf_trayLoc2)

(IsGP larm_gp_object11 object11 lgripper)
(IsGP larm_gp_object12 object12 lgripper)
(IsGP larm_gp_object13 object13 lgripper)
(IsGP larm_gp_object21 object21 lgripper)
(IsGP larm_gp_object22 object22 lgripper)
(IsGP larm_gp_object23 object23 lgripper)
(IsGP rarm_gp_object11 object11 rgripper)
(IsGP rarm_gp_object12 object12 rgripper)
(IsGP rarm_gp_object13 object13 rgripper)
(IsGP rarm_gp_object21 object21 rgripper)
(IsGP rarm_gp_object22 object22 rgripper)
(IsGP rarm_gp_object23 object23 rgripper)

(IsAccessPointForTray larm_blf_trayLoc1 trayLoc1)
(IsAccessPointForTray larm_blf_trayLoc2 trayLoc2)
(IsAccessPointForTray rarm_blf_trayLoc1 trayLoc1)
(IsAccessPointForTray rarm_blf_trayLoc2 trayLoc2)

(IsAccessPointFor larm_blf_destobject11 destobject11 lgripper)
(IsAccessPointFor larm_blf_destobject12 destobject12 lgripper)
(IsAccessPointFor larm_blf_destobject13 destobject13 lgripper)
(IsAccessPointFor larm_blf_destobject21 destobject21 lgripper)
(IsAccessPointFor larm_blf_destobject22 destobject22 lgripper)
(IsAccessPointFor larm_blf_destobject23 destobject23 lgripper)
(IsAccessPointFor rarm_blf_destobject11 destobject11 rgripper)
(IsAccessPointFor rarm_blf_destobject12 destobject12 rgripper)
(IsAccessPointFor rarm_blf_destobject13 destobject13 rgripper)
(IsAccessPointFor rarm_blf_destobject21 destobject21 rgripper)
(IsAccessPointFor rarm_blf_destobject22 destobject22 rgripper)
(IsAccessPointFor rarm_blf_destobject23 destobject23 rgripper)

(IsAccessPointFor larm_blf_trayLoc1 trayLoc1 lgripper)
(IsAccessPointFor larm_blf_trayLoc2 trayLoc2 lgripper)
(IsAccessPointFor rarm_blf_trayLoc1 trayLoc1 rgripper)
(IsAccessPointFor rarm_blf_trayLoc2 trayLoc2 rgripper)

(Smaller object11 object12)
(Smaller object11 object21)
(Smaller object11 object22)
(Smaller object11 object13)
(Smaller object11 object23)

(Smaller object12 object11)
(Smaller object12 object21)

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

(Smaller object11 None)
(Smaller object21 None)
(Smaller object13 None)
(Smaller object12 None)
(Smaller object22 None)
(Smaller object23 None)
(Topmost None tray)

(OnLeft larm_gp_object11)
(OnLeft larm_gp_object12)
(OnLeft larm_gp_object13)
(OnLeft larm_gp_object21)
(OnLeft larm_gp_object22)
(OnLeft larm_gp_object23)
(OnLeft larm_blf_trayLoc1)

(OnLeft rarm_gp_object11)
(OnLeft rarm_gp_object12)
(OnLeft rarm_gp_object13)
(OnLeft rarm_gp_object21)
(OnLeft rarm_gp_object22)
(OnLeft rarm_gp_object23)
(OnLeft rarm_blf_trayLoc1)

(OnLeft larm_blf_destobject11)
(OnLeft larm_blf_destobject12)
(OnLeft larm_blf_destobject13)
(OnLeft larm_blf_destobject21)
(OnLeft larm_blf_destobject22)
(OnLeft larm_blf_destobject23)
(OnLeft larm_blf_trayLoc2)

(OnLeft rarm_blf_destobject11)
(OnLeft rarm_blf_destobject12)
(OnLeft rarm_blf_destobject13)
(OnLeft rarm_blf_destobject21)
(OnLeft rarm_blf_destobject22)
(OnLeft rarm_blf_destobject23)
(OnLeft rarm_blf_trayLoc2)
)

(:goal (and (at object11 destobject11) (at object12 destobject12) (at object13 destobject13) (at object21 destobject21) (at object22 destobject22) (at object23 destobject23)))
            
(:metric minimize (total-cost))

)