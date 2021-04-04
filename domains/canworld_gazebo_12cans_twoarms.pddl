(define (problem canworld1) (:domain robotics)
(:objects
object0 object1 object2 object3 object4 object5 object6 object7 object8 object9 object10 object11 - object

loc_object0 loc_object1 loc_object2 loc_object3 loc_object4 loc_object5 loc_object6 loc_object7 loc_object8 loc_object9 loc_object10 loc_object11 rarm_gp_object0 rarm_gp_object1 rarm_gp_object2 rarm_gp_object3 rarm_gp_object4 rarm_gp_object5 rarm_gp_object6 rarm_gp_object7 rarm_gp_object8 rarm_gp_object9 rarm_gp_object10 rarm_gp_object11 larm_gp_object0 larm_gp_object1 larm_gp_object2 larm_gp_object3 larm_gp_object4 larm_gp_object5 larm_gp_object6 larm_gp_object7 larm_gp_object8 larm_gp_object9 larm_gp_object10 larm_gp_object11 - location

pdl0 pdl1 pdl2 pdl3 pdl4 rarm_pdp_0 rarm_pdp_1 rarm_pdp_2 rarm_pdp_3 rarm_pdp_4 larm_pdp_0 larm_pdp_1 larm_pdp_2 larm_pdp_3 larm_pdp_4 - location

robotInitLoc - location
table6 - location
larm_pdp_table6 - location
rarm_pdp_table6 - location


rgripper lgripper - manip

)

(:init
(At object0 loc_object0)
(At object1 loc_object1)
(At object2 loc_object2)
(At object3 loc_object3)
(At object4 loc_object4)
(At object5 loc_object5)
(At object6 loc_object6)
(At object7 loc_object7)
(At object8 loc_object8)
(At object9 loc_object9)
(At object10 loc_object10)
(At object11 loc_object11)
(RobotAt robotInitLoc)

(empty rgripper)
(empty lgripper)

(IsGP rarm_gp_object0 object0 rgripper)
(IsGP rarm_gp_object1 object1 rgripper)
(IsGP rarm_gp_object2 object2 rgripper)
(IsGP rarm_gp_object3 object3 rgripper)
(IsGP rarm_gp_object4 object4 rgripper)
(IsGP rarm_gp_object5 object5 rgripper)
(IsGP rarm_gp_object6 object6 rgripper)
(IsGP rarm_gp_object7 object7 rgripper)
(IsGP rarm_gp_object8 object8 rgripper)
(IsGP rarm_gp_object9 object9 rgripper)
(IsGP rarm_gp_object10 object10 rgripper)
(IsGP rarm_gp_object11 object11 rgripper)

(IsGP larm_gp_object0 object0 lgripper)
(IsGP larm_gp_object1 object1 lgripper)
(IsGP larm_gp_object2 object2 lgripper)
(IsGP larm_gp_object3 object3 lgripper)
(IsGP larm_gp_object4 object4 lgripper)
(IsGP larm_gp_object5 object5 lgripper)
(IsGP larm_gp_object6 object6 lgripper)
(IsGP larm_gp_object7 object7 lgripper)
(IsGP larm_gp_object8 object8 lgripper)
(IsGP larm_gp_object9 object9 lgripper)
(IsGP larm_gp_object10 object10 lgripper)
(IsGP larm_gp_object11 object11 lgripper)

(IsAccessPointFor rarm_pdp_0 pdl0 rgripper)
(IsAccessPointFor rarm_pdp_1 pdl1 rgripper)
(IsAccessPointFor rarm_pdp_2 pdl2 rgripper)
(IsAccessPointFor rarm_pdp_3 pdl3 rgripper)
(IsAccessPointFor rarm_pdp_4 pdl4 rgripper)

(IsAccessPointFor larm_pdp_0 pdl0 lgripper)
(IsAccessPointFor larm_pdp_1 pdl1 lgripper)
(IsAccessPointFor larm_pdp_2 pdl2 lgripper)
(IsAccessPointFor larm_pdp_3 pdl3 lgripper)
(IsAccessPointFor larm_pdp_4 pdl4 lgripper)

(clear pdl0)
(clear pdl1)
(clear pdl2)
(clear pdl3)
(clear pdl4)


(IsAccessPointFor rarm_pdp_table6 table6 rgripper)
(IsAccessPointFor larm_pdp_table6 table6 lgripper)


(clear table6)
)

(:goal (and (At object5 table6)))

)