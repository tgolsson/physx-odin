package ball

import physx "../"
import "core:fmt"
import corelog "core:log"
import "core:strings"
import "core:time"


run :: proc(physics: ^physx.PxPhysics, scene: ^physx.PxScene) {
	using physx
	// create a ground plane to the scene
	material := physics_create_material_mut(physics, 0.5, 0.5, 0.6)
	ground_plane := create_plane(physics, plane_new_1(0.0, 1.0, 0.0, 0.0), material)
	scene_add_actor_mut(scene, ground_plane, nil)
	corelog.info("Succesfully created ground plane")

	sphere_geo := sphere_geometry_new(10.0)
	sphere := create_dynamic(
		physics,
		transform_new_1(PxVec3{0.0, 40.0, 100.0}),
		&sphere_geo,
		material,
		10.0,
		transform_new_2(PxIDENTITY.PxIdentity),
	)
	rigid_body_set_angular_damping_mut(sphere, 0.5)
	scene_add_actor_mut(scene, sphere, nil)
	corelog.info("Succesfully added ball to scene")

	heights: [100]i32

	start := time.tick_now()
	for i in 0 ..< 100 {
		scene_simulate_mut(scene, 0.1, nil, nil, 0, true)
		error: u32 = 0
		scene_fetch_results_mut(scene, true, &error)

		pose := rigid_actor_get_global_pose(sphere)
		heights[i] = cast(i32)(pose.p.y - 10)
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
				if cast(i32)this_h == heights[idx] {
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

	cls := corelog.create_console_logger(
		corelog.Level.Debug,
		{
			corelog.Option.Level,
			corelog.Option.Short_File_Path,
			corelog.Option.Line,
			corelog.Option.Procedure,
			corelog.Option.Terminal_Color,
		},
		"System API",
	)

	context.logger = cls

	foundation := physx_create_foundation()
	defer foundation_release_mut(foundation)
	corelog.info("Succesfully created PxFoundation")

	dispatcher := default_cpu_dispatcher_create(
		1,
		nil,
		physx.PxDefaultCpuDispatcherWaitForWorkMode.WaitForWork,
		0,
	)
	defer default_cpu_dispatcher_release_mut(dispatcher)
	corelog.debug("Created dispatcher with 1 thread")

	physics := physx_create_physics(foundation)
	defer physics_release_mut(physics)
	corelog.info("Succesfully created PxPhysics")

	scene_desc := scene_desc_new(tolerances_scale_new(1.0, 10.0))
	scene_desc.gravity = vec3_new_3(0.0, -9.81, 0.0)
	scene_desc.cpuDispatcher = dispatcher
	scene_desc.filterShader = get_default_simulation_filter_shader()
	scene := physics_create_scene_mut(physics, scene_desc)
	defer scene_release_mut(scene)
	corelog.info("Succesfully created PxScene")

	run(physics, scene)
}
