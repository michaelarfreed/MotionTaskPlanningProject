(define (problem dinnerTimeHandoff1) (:domain robotics)
(:objects
object1 object2 object3 drawer - movable

loc_object1 loc_object2 loc_object3 table6 - location

larm_gp_object1 rarm_gp_object1 larm_gp_object2 rarm_gp_object2 larm_gp_object3 rarm_gp_object3 larm_gp_drawer rarm_gp_drawer - pose
larm_pdp_object1_table6 rarm_pdp_object1_table6 larm_pdp_object2_table6 rarm_pdp_object2_table6 larm_pdp_object3_table6 rarm_pdp_object3_table6 - pose
robotInitLoc - pose

)

(:init
(At object1 loc_object1)
(At object2 loc_object2)
(At object3 loc_object3)
(RobotAt robotInitLoc)

(Object object1)
(Object object2)
(Object object3)

(TempArea table6)

(Empty rgripper)
(Empty lgripper)
(IsDrawer drawer)

(InDrawer object1 drawer)
(Closed drawer)

(IsGP larm_gp_object1 object1 lgripper)
(IsGP rarm_gp_object1 object1 rgripper)
(IsGP larm_gp_object2 object2 lgripper)
(IsGP rarm_gp_object2 object2 rgripper)
(IsGP larm_gp_object3 object3 lgripper)
(IsGP rarm_gp_object3 object3 rgripper)
(IsGP larm_gp_drawer drawer lgripper)
(IsGP rarm_gp_drawer drawer rgripper)

(IsAccessPointFor larm_pdp_object1_table6 object1 table6 lgripper)
(IsAccessPointFor rarm_pdp_object1_table6 object1 table6 rgripper)
(IsAccessPointFor larm_pdp_object2_table6 object2 table6 lgripper)
(IsAccessPointFor rarm_pdp_object2_table6 object2 table6 rgripper)
(IsAccessPointFor larm_pdp_object3_table6 object3 table6 lgripper)
(IsAccessPointFor rarm_pdp_object3_table6 object3 table6 rgripper)
)

(:goal (and (at object1 table6) (closed drawer)))

)