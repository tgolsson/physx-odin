package physx

when ODIN_OS == .Linux   do foreign import px_api "libphysx_api.a"

import "core:fmt"
import "core:log"

VERSION: u32 : (5 << 24) + (1 << 16) + (3 << 8)

@(default_calling_convention = "c")
foreign px_api {
	@(link_name = "physx_create_foundation")
	physx_create_foundation :: proc() -> ^PxFoundation ---

	@(link_name = "physx_create_physics")
	physx_create_physics :: proc(foundation: ^PxFoundation) -> ^PxPhysics ---

    get_default_simulation_filter_shader :: proc() -> rawptr ---
}

main :: proc() {
	cls := log.create_console_logger(log.Level.Debug,{
		log.Option.Level,
		log.Option.Short_File_Path,
		log.Option.Line,
		log.Option.Procedure,
		log.Option.Terminal_Color
	}, "System API");

	context.logger = cls
	log.info("Created logger")
	foundation := physx_create_foundation()
	defer PxFoundation_release_mut(foundation)
	log.info("Succesfully created PxFoundation")

	physics := physx_create_physics(foundation)
	defer PxPhysics_release_mut(physics)
	log.info("Succesfully created PxPhysics")

	scene_desc := PxSceneDesc_new(PxPhysics_getTolerancesScale(physics))
	scene_desc.gravity = PxVec3_new_3(0.0, -9.81, 0.0)

	dispatcher := phys_PxDefaultCpuDispatcherCreate(1, nil, auto_cast(PxDefaultCpuDispatcherWaitForWorkMode.WaitForWork), 0)
	log.debug("Created dispatcher with 1 thread")

	scene_desc.cpuDispatcher = auto_cast(dispatcher)
	scene_desc.filterShader = get_default_simulation_filter_shader();
	scene:= PxPhysics_createScene_mut(physics, &scene_desc)

	log.info("Succesfully created PxScene")
}
