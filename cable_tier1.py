import omni.usd
from pxr import UsdGeom, UsdPhysics, PhysxSchema, Sdf, Gf

stage = omni.usd.get_context().get_stage()

# ============================================================
# ADJUST THESE
# ============================================================
start_pos        = Gf.Vec3f(14.4, 7.75, 1.0)
end_pos          = Gf.Vec3f(12.9, 8.0, 1.0)
num_links        = 50
link_half_length = 0.03
link_radius      = 0.015
cone_angle_limit = 110
rope_damping     = 10.0
rope_stiffness   = 1.0
# ============================================================

# --- Configure physics scene for flexible small-scale rope ---
# Find existing physics scene or create one
physics_scene = None
for prim in stage.Traverse():
    if prim.IsA(UsdPhysics.Scene):
        physics_scene = prim
        break

if physics_scene is None:
    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    physics_scene = scene.GetPrim()

# Apply PhysX scene settings - TGS solver handles small segments much better
physx_scene = PhysxSchema.PhysxSceneAPI.Apply(physics_scene)
physx_scene.GetSolverTypeAttr().Set("TGS")          # TGS is more stable than PGS for chains
physx_scene.GetTimeStepsPerSecondAttr().Set(120)     # more physics steps per second

# More solver iterations = more flexible rope
pos_itr = physics_scene.GetAttribute("physxScene:solverPositionIterationCount")
if not pos_itr:
    physics_scene.CreateAttribute("physxScene:solverPositionIterationCount",
                                   Sdf.ValueTypeNames.UInt).Set(32)
else:
    pos_itr.Set(32)

vel_itr = physics_scene.GetAttribute("physxScene:solverVelocityIterationCount")
if not vel_itr:
    physics_scene.CreateAttribute("physxScene:solverVelocityIterationCount",
                                   Sdf.ValueTypeNames.UInt).Set(8)
else:
    vel_itr.Set(8)

# --- Build cable ---
existing = stage.GetPrimAtPath("/World/RackCable")
if existing.IsValid():
    stage.RemovePrim("/World/RackCable")

UsdGeom.Scope.Define(stage, "/World/RackCable")

dx = (end_pos[0] - start_pos[0]) / (num_links - 1)
dy = (end_pos[1] - start_pos[1]) / (num_links - 1)
dz = (end_pos[2] - start_pos[2]) / (num_links - 1)

joint_x = link_half_length - 0.5 * link_radius

for i in range(num_links):
    path = Sdf.Path(f"/World/RackCable/link_{i}")
    pos  = Gf.Vec3f(
        start_pos[0] + dx * i,
        start_pos[1] + dy * i,
        start_pos[2] + dz * i
    )

    capsule = UsdGeom.Capsule.Define(stage, path)
    capsule.CreateHeightAttr(link_half_length)
    capsule.CreateRadiusAttr(link_radius)
    capsule.CreateAxisAttr("X")
    capsule.GetDisplayColorAttr().Set([Gf.Vec3f(0.05, 0.05, 0.05)])
    capsule.AddTranslateOp().Set(pos)

    UsdPhysics.CollisionAPI.Apply(capsule.GetPrim())
    rb = UsdPhysics.RigidBodyAPI.Apply(capsule.GetPrim())
    mass = UsdPhysics.MassAPI.Apply(capsule.GetPrim())
    mass.CreateDensityAttr().Set(0.00005)
    # NO explicit inertia tensor - let PhysX compute it from geometry and density

    # Apply PhysX rigid body settings for better solver convergence
    physx_rb = PhysxSchema.PhysxRigidBodyAPI.Apply(capsule.GetPrim())
    physx_rb.CreateSolverPositionIterationCountAttr().Set(32)
    physx_rb.CreateSolverVelocityIterationCountAttr().Set(8)
    physx_rb.CreateLinearDampingAttr().Set(0.1)
    physx_rb.CreateAngularDampingAttr().Set(0.1)

    # Pin endpoints kinematically
    if i == 0 or i == num_links - 1:
        rb.GetKinematicEnabledAttr().Set(True)

    # D6 joint connecting to previous link
    if i > 0:
        jpath = Sdf.Path(f"/World/RackCable/joint_{i}")
        joint = UsdPhysics.Joint.Define(stage, jpath)
        joint.GetBody0Rel().SetTargets([Sdf.Path(f"/World/RackCable/link_{i-1}")])
        joint.GetBody1Rel().SetTargets([path])
        joint.GetLocalPos0Attr().Set(Gf.Vec3f(joint_x, 0, 0))
        joint.GetLocalPos1Attr().Set(Gf.Vec3f(-joint_x, 0, 0))

        d6 = joint.GetPrim()

        # Lock translation and rotX
        for dof in ["transX", "transY", "transZ", "rotX"]:
            lim = UsdPhysics.LimitAPI.Apply(d6, dof)
            lim.CreateLowAttr(1.0)
            lim.CreateHighAttr(-1.0)

        # Free rotY and rotZ with damping
        for dof in ["rotY", "rotZ"]:
            lim = UsdPhysics.LimitAPI.Apply(d6, dof)
            lim.CreateLowAttr(-cone_angle_limit)
            lim.CreateHighAttr(cone_angle_limit)
            drive = UsdPhysics.DriveAPI.Apply(d6, dof)
            drive.CreateTypeAttr("force")
            drive.CreateDampingAttr(rope_damping)
            drive.CreateStiffnessAttr(rope_stiffness)

print(f"Cable: {num_links} links, endpoints kinematically pinned")
print(f"Physics scene: TGS solver, 120 steps/sec, 32 position iterations")
print("Press Play to simulate")

