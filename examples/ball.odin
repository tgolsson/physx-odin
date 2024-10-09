package ball

import physx "../"
import "core:fmt"
import "core:strings"
import "core:time"
import "core:math"

run :: proc(physics: ^physx.Physics, scene: ^physx.Scene) {
	using physx

	// create a ground plane to the scene
	ground_material := physics_create_material_mut(physics, 0.5, 0.5, 0.3)
	ground_plane := create_plane(physics, plane_new_1(0.0, 1.0, 0.0, 0.0), ground_material)
	scene_add_actor_mut(scene, ground_plane, nil)
	defer {
		scene_remove_actor_mut(scene, ground_plane, false)
		actor_release_mut(ground_plane)
		ref_counted_release_mut(ground_material)
	}

	sphere_material := physics_create_material_mut(physics, 0.5, 0.5, 0.9)
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

	heights: [100]f32
	start := time.tick_now()
	for i in 0 ..< 100 {
		scene_simulate_mut(scene, 0.1, nil, nil, 0, true)
		error: u32 = 0
		scene_fetch_results_mut(scene, true, &error)

		pose := rigid_actor_get_global_pose(sphere)
		heights[i] = pose.p.y - 5
	}
	end := time.tick_now()

	fmt.printfln(
		"Simulated 100 timesteps in %v ms.",
		time.duration_milliseconds(time.tick_diff(start, end)),
	)
	fmt.println(
		"Heights over time (x=time, dt=100ms)",
	)


	max_h := 18
	{
		output := strings.builder_make(context.temp_allocator)
		defer strings.builder_destroy(&output)
		for h in 0 ..< 18 {
			this_h := max_h - 1 - h

			for idx in 0 ..< 100 {
				if cast(i32)this_h == cast(i32)heights[idx] {
					strings.write_rune(&output, 'o')
				} else {
					strings.write_rune(&output, ' ')
				}
			}
			fmt.println(string(output.buf[:]))
			strings.builder_reset(&output)
		}
	}
}


main :: proc() {
	using physx

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
