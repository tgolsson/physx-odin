package ball_raylib

import physx "../"
import "core:fmt"
import "core:strings"
import "core:time"
import "vendor:raylib"

DT :: 1.0 / 60.0
SCREEN_HEIGHT :: 450
SCREEN_WIDTH :: 800

run :: proc(physics: ^physx.Physics, scene: ^physx.Scene) {
	using physx

	camera: raylib.Camera
	camera.position = raylib.Vector3{0.0, 20.0, 40.0}
	camera.target = raylib.Vector3{0.0, 20.0, 0.0}
	camera.up = {0.0, 1.0, 0.0}
	camera.fovy = 60.0
	camera.projection = raylib.CameraProjection.PERSPECTIVE


	// create a ground plane to the scene
	ground_material := physics_create_material_mut(physics, 0.5, 0.5, 0.1)
	ground_plane := create_plane(physics, plane_new_1(0.0, 1.0, 0.0, 0.0), ground_material)
	scene_add_actor_mut(scene, ground_plane, nil)
	defer {
		scene_remove_actor_mut(scene, ground_plane, false)
		actor_release_mut(ground_plane)
		ref_counted_release_mut(ground_material)
	}

	sphere_material := physics_create_material_mut(physics, 0.5, 0.5, 0.95)
	sphere_geo := sphere_geometry_new(5.0)
	sphere := create_dynamic(
		physics,
		transform_new_1(Vec3{0.0, 40.0, 0.0}),
		&sphere_geo,
		sphere_material,
		10.0,
		transform_new_2(IDENTITY.Identity),
	)
	rigid_body_set_angular_damping_mut(sphere, 0.8)
	scene_add_actor_mut(scene, sphere, nil)
	defer {
		scene_remove_actor_mut(scene, sphere, false)
		actor_release_mut(sphere)
		ref_counted_release_mut(sphere_material)
	}

	start := time.tick_now()
	frames := 0

	raylib.SetTargetFPS(60)
	for !raylib.WindowShouldClose() {
		if raylib.IsKeyPressed(raylib.KeyboardKey.R) {
			rigid_actor_set_global_pose_mut(sphere, transform_new_1(Vec3{0.0, 40.0, 0.0}), true)
			rigid_dynamic_wake_up_mut(sphere)
		}

		raylib.UpdateCamera(&camera, raylib.CameraMode.ORBITAL)
		scene_simulate_mut(scene, 0.1, nil, nil, 0, true)
		error: u32 = 0
		scene_fetch_results_mut(scene, true, &error)
		if error != 0 {
			fmt.printf("error during physx scene update: %v", error)
			return
		}

		pose := rigid_actor_get_global_pose(sphere)
		{
			raylib.BeginDrawing()
			defer raylib.EndDrawing()

			raylib.ClearBackground(raylib.SKYBLUE)
			{
				raylib.DrawText("Press R to respawn the ball", 10, 10, 40, raylib.BLACK)
				raylib.BeginMode3D(camera)
				defer raylib.EndMode3D()

				raylib.DrawPlane({0.0, 0.0, 0.0}, {50.0, 50.0}, raylib.RED)
				raylib.DrawSphere({pose.p.x, pose.p.y, pose.p.z}, 5.0, raylib.GREEN)
			}
		}
		frames += 1
	}
	end := time.tick_now()

	fmt.printf(
		"Simulated %v timesteps in %v ms.",
		frames,
		time.duration_milliseconds(time.tick_diff(start, end)),
	)
}


main :: proc() {
	using physx
	raylib.SetTraceLogLevel(raylib.TraceLogLevel.ERROR)
	raylib.InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Raylib bouncing ball")

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
		raylib.CloseWindow()
	}

	run(physics, scene)
}
