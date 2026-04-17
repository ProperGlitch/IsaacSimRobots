import carb.input
import omni.appwindow
import omni.usd
from pxr import UsdPhysics, Sdf

# Setup input
input_iface = carb.input.acquire_input_interface()
app_window = omni.appwindow.get_default_app_window()
keyboard = app_window.get_keyboard()

# Get stage
stage = omni.usd.get_context().get_stage()

# Fetch only has two drive wheels
# Update the base path if your Fetch prim is named differently in the Stage
wheel_joints = [
    "/World/fetch/joints/l_wheel_joint",
    "/World/fetch/joints/r_wheel_joint",
]

def setup_drives():
    for path in wheel_joints:
        prim = stage.GetPrimAtPath(Sdf.Path(path))
        if not prim.IsValid():
            print(f"WARNING: Could not find joint at {path}")
            continue
        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        if not drive:
            drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.GetDampingAttr().Set(1000.0)
        drive.GetStiffnessAttr().Set(0.0)
        drive.GetMaxForceAttr().Set(1000.0)
        print(f"Drive configured for {path}")

def set_wheel_velocity(left_speed, right_speed):
    stage = omni.usd.get_context().get_stage()  # get fresh stage every call
    for path in wheel_joints:
        prim = stage.GetPrimAtPath(Sdf.Path(path))
        if not prim.IsValid():
            print(f"WARNING: Could not find joint at {path}")
            continue
        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        if not drive:
            print(f"No drive on {path} - run setup first")
            continue
        speed = left_speed if "l_wheel" in path else right_speed
        drive.GetTargetVelocityAttr().Set(speed)

def on_keyboard_event(event, *args, **kwargs):
    if event.type == carb.input.KeyboardEventType.KEY_PRESS:
        if event.input == carb.input.KeyboardInput.UP:
            print("forward")
            set_wheel_velocity(400, 400)
        elif event.input == carb.input.KeyboardInput.DOWN:
            print("backward")
            set_wheel_velocity(-400, -400)
        elif event.input == carb.input.KeyboardInput.LEFT:
            print("turn left")
            set_wheel_velocity(-200, 200)
        elif event.input == carb.input.KeyboardInput.RIGHT:
            print("turn right")
            set_wheel_velocity(200, -200)
        elif event.input == carb.input.KeyboardInput.SPACE:
            print("stop")
            set_wheel_velocity(0, 0)
    return True

# Configure drives then start listening
setup_drives()
sub = input_iface.subscribe_to_keyboard_events(keyboard, on_keyboard_event)
print("Ready - use arrow keys to drive, space to stop")
