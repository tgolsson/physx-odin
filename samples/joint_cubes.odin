package samples

import physx "../"
import "core:fmt"
import "core:log"
import "core:math"
import lg "core:math/linalg"
import "core:strings"
import "core:time"
import mu "vendor:microui"
import "vendor:raylib"

TARGET_FPS :: 60
DT :: 1.0 / TARGET_FPS

AngleMode :: struct {}
Rad :: distinct AngleMode
Deg :: distinct AngleMode

Rotator :: struct($am: typeid) {
	pitch, yaw, roll: f32,
}

deg_rotator_from_pxquat :: proc(q: physx.Quat, am: Deg) -> Rotator(Deg) {
	quat := transmute(quaternion128)q
	yaw, pitch, roll := lg.pitch_yaw_roll_from_quaternion_f32(quat)

	return rotator_to_degrees(Rotator(Rad){yaw, pitch, roll})
}

rad_rotator_from_pxquat :: proc(q: physx.Quat, am: Rad) -> Rotator(Rad) {
	quat := transmute(quaternion128)q
	yaw, pitch, roll := lg.pitch_yaw_roll_from_quaternion_f32(quat)

	return Rotator(Rad){yaw, pitch, roll}
}

rotator_from_pxquat :: proc {
	rad_rotator_from_pxquat,
	deg_rotator_from_pxquat,
}

rad_to_degrees :: proc(r: Rotator(Rad)) -> Rotator(Deg) {
	return Rotator(Deg) {
		yaw = math.to_degrees(r.yaw),
		pitch = math.to_degrees(r.pitch),
		roll = math.to_degrees(r.roll),
	}
}
deg_to_degrees :: proc(r: Rotator(Deg)) -> Rotator(Deg) {
	return r
}
rotator_to_degrees :: proc {
	deg_to_degrees,
	rad_to_degrees,
}


rad_to_radians :: proc(r: Rotator(Rad)) -> Rotator(Rad) {
	return r
}

deg_to_radians :: proc(r: Rotator(Deg)) -> Rotator(Rad) {
	return Rotator(Rad) {
		yaw = math.to_radians(r.yaw),
		pitch = math.to_radians(r.pitch),
		roll = math.to_radians(r.roll),
	}

}
rotator_to_radians :: proc {
	deg_to_radians,
	rad_to_radians,
}

rotator_to_pxquat :: proc(r: Rotator($T)) -> physx.Quat {
	as_rad := rotator_to_radians(r)
	quat := lg.quaternion_from_pitch_yaw_roll_f32(as_rad.yaw, as_rad.pitch, as_rad.roll)

	return transmute(physx.Quat)(quat)
}

ROTATORS: [2]Rotator(Deg)
mui_all_windows :: proc(state: ^MuiState, joints: []^physx.D6Joint) {
	@(static) opts := mu.Options{.NO_CLOSE}
	ctx := state.mu_ctx

	DRIVES :: []physx.D6Drive{.Swing, .Twist}

	if mu.window(ctx, "Link Settings", {40, 40, 300, 450}, opts) {
		for joint, index in joints {
			label := fmt.tprintf("Joint %v", index)
			mu.push_id(ctx, label)
			defer mu.pop_id(ctx)
			if .ACTIVE in mu.header(ctx, label, {.EXPANDED}) {
				for d6drive in DRIVES {
					label := fmt.tprintf("Drive %v Settings", d6drive)
					mu.push_id(ctx, label)
					defer mu.pop_id(ctx)
					if .ACTIVE in mu.header(ctx, label, {.EXPANDED, .NO_CLOSE}) {
						drive := physx.d6_joint_get_drive(joint, d6drive)
						drive_d := false

						mu.layout_row(ctx, {46, -1}, 0)
						mu.label(ctx, "Stiffness")
						drive_d |= .CHANGE in mui_f32_slider(ctx, &drive.stiffness, 0, 1e4)
						mu.label(ctx, "Damping")
						drive_d |= .CHANGE in mui_f32_slider(ctx, &drive.damping, 0, 1e3)
						mu.label(ctx, "Force limit")
						drive_d |= .CHANGE in mui_f32_slider(ctx, &drive.forceLimit, 0, 1e5)
						acceleration_mode := .Acceleration in drive.flags
						if .CHANGE in mu.checkbox(ctx, "Acceleration mode", &acceleration_mode) {
							if acceleration_mode {
								drive.flags = {.Acceleration}
							} else {
								drive.flags = {}
							}
							drive_d = true
						}

						if drive_d {
							physx.d6_joint_set_drive_mut(joint, d6drive, drive)
						}
					}
				}

				label := fmt.tprintf("Drive Target %v", index)
				mu.push_id(ctx, label)
				defer mu.pop_id(ctx)
				if .ACTIVE in mu.header(ctx, "Drive Target", {.EXPANDED, .NO_CLOSE}) {
					target := physx.d6_joint_get_drive_position(joint)
					r := &ROTATORS[index]
					drive_d := false

					mu.layout_row(ctx, {46, -1}, 0)
					mu.label(ctx, "Yaw")
					drive_d |= .CHANGE in mui_f32_slider(ctx, &r.yaw, -180, 180)
					mu.label(ctx, "Pitch")
					drive_d |= .CHANGE in mui_f32_slider(ctx, &r.pitch, -180, 180)
					mu.label(ctx, "Roll")
					drive_d |= .CHANGE in mui_f32_slider(ctx, &r.roll, -180, 180)

					if drive_d {
						target.q = rotator_to_pxquat(rotator_to_radians(r^))
						physx.d6_joint_set_drive_position_mut(joint, target, true)
					}
				}
			}
		}
	}
}

create_link :: proc(
	physics: ^physx.Physics,
	actor0: ^physx.RigidActor,
	actor1: ^physx.RigidActor,
) -> ^physx.D6Joint {
	joint_frame_0 := physx.transform_new_1({0.0, 0.0, 0.0})
	joint_frame_1 := physx.transform_new_1({0.0, -4.0, 0.0})
	joint_frame_0.q = physx.quat_new_1(.Identity)
	joint_frame_1.q = physx.quat_new_1(.Identity)
	joint := physx.d6_joint_create(physics, actor0, joint_frame_0, actor1, joint_frame_1)
	for axis in physx.D6Axis {
		physx.d6_joint_set_motion_mut(joint, axis, .Locked)
	}

	physx.d6_joint_set_motion_mut(joint, .Swing1, .Free)
	physx.d6_joint_set_motion_mut(joint, .Swing2, .Free)
	physx.d6_joint_set_motion_mut(joint, .Twist, .Free)

	physx.d6_joint_set_drive_mut(
		joint,
		.Slerp,
		physx.D6JointDrive {
			stiffness = 2000.0,
			damping = 200.0,
			forceLimit = 10000.0,
			flags = {physx.D6JointDriveFlag.Acceleration},
		},
	)

	physx.d6_joint_set_drive_position_mut(joint, physx.transform_new_2(.Identity), true)

	return joint
}


run :: proc(physics: ^physx.Physics, scene: ^physx.Scene) {
	using physx

	mui := mui_init()

	camera: raylib.Camera
	camera.position = raylib.Vector3{0.0, 1.5, 5.0}
	camera.target = raylib.Vector3{0.0, 1.5, 0.0}
	camera.up = {0.0, 1.0, 0.0}
	camera.fovy = 60.0
	camera.projection = raylib.CameraProjection.PERSPECTIVE

	material := physics_create_material_mut(physics, 0.5, 0.5, 0.2)
	ground_plane := create_plane(physics, plane_new_1(0.0, 1.0, 0.0, 0.0), material)
	scene_add_actor_mut(scene, ground_plane, nil)
	defer {
		scene_remove_actor_mut(scene, ground_plane, false)
		actor_release_mut(ground_plane)
		ref_counted_release_mut(material)
	}
	log.debug("Succesfully created ground plane")

	cube_geo := box_geometry_new_1({0.5, 0.5, 0.5})
	c1 := create_static(
		physics,
		transform_new_1({0.0, 0.5, 0.0}),
		&cube_geo,
		material,
		transform_new_2(IDENTITY.Identity),
	)
	scene_add_actor_mut(scene, c1, nil)

	c2 := create_dynamic(
		physics,
		transform_new_1({0.0, 4.0, 0.0}),
		&cube_geo,
		material,
		10.0,
		transform_new_2(IDENTITY.Identity),
	)
	scene_add_actor_mut(scene, c2, nil)
	link := create_link(physics, c1, c2)
	physx.rigid_body_set_linear_damping_mut(c2, 0.9)
	physx.rigid_body_set_angular_damping_mut(c2, 0.9)

	c3 := create_dynamic(
		physics,
		transform_new_1({0.0, 8.0, 0.0}),
		&cube_geo,
		material,
		10.0,
		transform_new_2(IDENTITY.Identity),
	)
	physx.rigid_body_set_linear_damping_mut(c3, 0.9)
	physx.rigid_body_set_angular_damping_mut(c3, 0.9)
	scene_add_actor_mut(scene, c3, nil)
	link2 := create_link(physics, c2, c3)

	defer {
		joint_release_mut(link2)

		scene_remove_actor_mut(scene, c3, false)
		actor_release_mut(c3)

		joint_release_mut(link)
		scene_remove_actor_mut(scene, c2, false)
		actor_release_mut(c2)

		scene_remove_actor_mut(scene, c1, false)
		actor_release_mut(c1)
	}

	start := time.tick_now()
	frames := 0
	t := 0.0
	now := time.tick_now()
	accumulator := 0.0

	camera_controlled := true

	for !raylib.WindowShouldClose() {

		new := time.tick_now()
		defer {
			free_all(context.temp_allocator)
			now = new
		}

		elapsed_s := math.min(time.duration_seconds(time.tick_diff(now, new)), 0.25)
		accumulator += elapsed_s
		for accumulator >= DT {
			scene_simulate_mut(scene, DT, nil, nil, 0, true)
			error: u32 = 0
			scene_fetch_results_mut(scene, true, &error)
			if error != 0 {
				log.errorf("error during physx scene update: %v", error)
			}
			t += DT
			accumulator -= DT
		}

		mui_update(&mui)
		if raylib.IsKeyPressed(raylib.KeyboardKey.R) {
			rigid_actor_set_global_pose_mut(c2, transform_new_1(Vec3{0.0, 40.0, 0.0}), true)
		}
		if raylib.IsKeyPressed(raylib.KeyboardKey.T) {
			camera_controlled = !camera_controlled
		}

		if camera_controlled {
			raylib.UpdateCamera(&camera, raylib.CameraMode.FREE)
		}

		mu.begin(mui.mu_ctx)
		mui_all_windows(&mui, []^physx.D6Joint{link, link2})
		mu.end(mui.mu_ctx)


		raylib.BeginDrawing()
		defer raylib.EndDrawing()

		raylib.ClearBackground(raylib.SKYBLUE)
		{
			raylib.DrawText(
				fmt.ctprintf(
					"FPS: %.1f (%v)",
					1000.0 *
					f64(frames) /
					time.duration_milliseconds(time.tick_diff(start, time.tick_now())),
					frames,
				),
				10,
				10,
				40,
				raylib.BLACK,
			)
			raylib.BeginMode3D(camera)
			defer raylib.EndMode3D()

			raylib.DrawPlane({0.0, 0.0, 0.0}, {50.0, 50.0}, raylib.RED)
			previous: Vec3
			first := true
			slice := []^RigidActor{c1, c2, c3}
			for cube in slice {
				pose := rigid_actor_get_global_pose(cube)
				defer {
					previous = pose.p
					first = false
				}

				raylib.DrawCube({pose.p.x, pose.p.y, pose.p.z}, 1.0, 1.0, 1.0, raylib.GREEN)

				if !first {
					raylib.DrawLine3D(
						{previous.x, previous.y, previous.z},
						{pose.p.x, pose.p.y, pose.p.z},
						raylib.BLACK,
					)
				}
			}
		}

		mui_render(&mui)
		frames += 1
	}
	end := time.tick_now()

	log.debugf(
		"Simulated %v timesteps in %v ms.",
		frames,
		time.duration_milliseconds(time.tick_diff(start, end)),
	)
}

SCREEN_HEIGHT :: 450
SCREEN_WIDTH :: 800


import "base:runtime"
import _c "core:c"
import "core:c/libc"

raylib_logger: log.Logger

handle_raylib_log_message :: proc "c" (
	logLevel: raylib.TraceLogLevel,
	text: cstring,
	args: ^_c.va_list,
) {
	context = runtime.default_context()
	context.logger = raylib_logger

	buf: [150]u8
	raw := raw_data(buf[:])
	count := libc.vsnprintf(raw, 149, text, args)
	str := strings.string_from_ptr(raw, int(count))
	location := runtime.Source_Code_Location{"vendor:raylib", 0, 0, "ffi"}

	switch logLevel {
	case .TRACE, .DEBUG:
		log.debug(str, location = location)
	case .INFO:
		log.info(str, location = location)

	case .WARNING:
		log.warn(str, location = location)

	case .ERROR:
		log.error(str, location = location)

	case .FATAL:
		log.fatal(str, location = location)

	case .ALL, .NONE:
		log.logf(log.Level.Info, "%v", str, "RAYLIB")
	}
}

main :: proc() {
	using physx

	cls := log.create_console_logger(
		log.Level.Debug,
		{
			log.Option.Level,
			log.Option.Short_File_Path,
			log.Option.Line,
			log.Option.Procedure,
			log.Option.Terminal_Color,
		},
	)
	context.logger = cls

	raylib_logger = log.create_console_logger(
		log.Level.Debug,
		{log.Option.Level, log.Option.Short_File_Path, log.Option.Terminal_Color},
	)

	raylib.SetTraceLogCallback(handle_raylib_log_message)
	raylib.InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Raylib bouncing ball")
	raylib.SetTargetFPS(cast(i32)TARGET_FPS)
	defer raylib.CloseWindow()

	foundation := create_foundation_ext()
	dispatcher := default_cpu_dispatcher_create(
		1,
		nil,
		physx.DefaultCpuDispatcherWaitForWorkMode.WaitForWork,
		0,
	)
	physics := create_physics_ext(foundation)

	scene_desc := scene_desc_new(tolerances_scale_new(1.0, 10.0))
	scene_desc.gravity = vec3_new_3(0.0, -9.81, 0.0)
	scene_desc.cpuDispatcher = dispatcher
	scene_desc.filterShader = get_default_simulation_filter_shader()
	scene := physics_create_scene_mut(physics, scene_desc)

	defer {
		scene_release_mut(scene)
		physics_release_mut(physics)
		default_cpu_dispatcher_release_mut(dispatcher)
		foundation_release_mut(foundation)
	}

	run(physics, scene)
}
