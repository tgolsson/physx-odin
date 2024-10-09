package ball_raylib

import physx "../"
import "core:fmt"
import "core:log"
import "core:math"
import "core:strings"
import "core:time"
import "vendor:raylib"

TARGET_FPS :: 60.0
DT :: 1.0 / TARGET_FPS

create_link :: proc(
	physics: ^physx.Physics,
	actor0: ^physx.RigidActor,
	actor1: ^physx.RigidActor,
) -> ^physx.D6Joint {
	joint_frame_0 := physx.transform_new_1({0.0, 0.0, 0.0})
	joint_frame_1 := physx.transform_new_1({0.0, -4.0, 0.4})
	joint_frame_0.q = physx.quat_new_1(.Identity)
	joint_frame_1.q = physx.quat_new_1(.Identity)
	joint := physx.d6_joint_create(physics, actor0, joint_frame_0, actor1, joint_frame_1)
	for axis in physx.D6Axis {
		physx.d6_joint_set_motion_mut(joint, axis, .Locked)
	}

	physx.d6_joint_set_motion_mut(joint, .Swing1, .Free)
	physx.d6_joint_set_motion_mut(joint, .Swing2, .Free)
	physx.d6_joint_set_motion_mut(joint, .Twist, .Free)
	return joint
}


run :: proc(physics: ^physx.Physics, scene: ^physx.Scene) {
	using physx

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
	root := create_static(
		physics,
		transform_new_1({0.0, 0.5, 0.0}),
		&cube_geo,
		material,
		transform_new_2(IDENTITY.Identity),
	)
	scene_add_actor_mut(scene, root, nil)

	top := create_dynamic(
		physics,
		transform_new_1({0.0, 4.0, 0.0}),
		&cube_geo,
		material,
		10.0,
		transform_new_2(IDENTITY.Identity),
	)
	scene_add_actor_mut(scene, top, nil)

	link := create_link(physics, root, top)

	defer {
		joint_release_mut(link)

		scene_remove_actor_mut(scene, top, false)
		actor_release_mut(top)

		scene_remove_actor_mut(scene, root, false)
		actor_release_mut(root)
	}

	start := time.tick_now()
	frames := 0
	t := 0.0
	now := time.tick_now()
	accumulator := 0.0

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

		if raylib.IsKeyPressed(raylib.KeyboardKey.R) {
			rigid_actor_set_global_pose_mut(top, transform_new_1(Vec3{0.0, 40.0, 0.0}), true)
		}

		raylib.UpdateCamera(&camera, raylib.CameraMode.ORBITAL)

		root_pose := rigid_actor_get_global_pose(root)
		top_pose := rigid_actor_get_global_pose(top)

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
			raylib.DrawCube(
				{root_pose.p.x, root_pose.p.y, root_pose.p.z},
				1.0,
				1.0,
				1.0,
				raylib.GREEN,
			)
			raylib.DrawCube(
				{top_pose.p.x, top_pose.p.y, top_pose.p.z},
				1.0,
				1.0,
				1.0,
				raylib.ORANGE,
			)
			raylib.DrawLine3D(
				{top_pose.p.x, top_pose.p.y, top_pose.p.z},
				{root_pose.p.x, root_pose.p.y, root_pose.p.z},
				raylib.PURPLE,
			)
		}

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

init_raylib :: proc() {

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
