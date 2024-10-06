package px

// when ODIN_OS == .Linux   do foreign import px_api "libphysx_api.a"
when ODIN_OS == .Linux do foreign import px_raw "libphysx.a"
when ODIN_OS == .Linux do foreign import px_api "libphysx_api.a"

import "core:fmt"

VERSION: u32 : (5 << 24) + (1 << 16) + (3 << 8)
physx_PxActor_Pod :: distinct rawptr
physx_PxFoundation_Pod :: struct {
	ptr: rawptr,
}

physx_PxPhysics_Pod :: struct {
	ptr: rawptr,
}
physx_PxFoundation_Pod2 :: struct {
	using _: physx_PxFoundation_Pod,
}

@(default_calling_convention = "c")
foreign px_raw {}

@(default_calling_convention = "c")
foreign px_api {
	@(link_name = "physx_create_foundation")
	physx_create_foundation :: proc() -> physx_PxFoundation_Pod ---

	//     @(link_name="physx_create_foundation_with_alloc")
	//     physx_create_foundation_with_alloc :: proc(allocator : ^physx_PxAllocatorCallback_Pod) -> ^physx_PxFoundation_Pod ---;

	@(link_name = "physx_create_physics")
	physx_create_physics :: proc(foundation: physx_PxFoundation_Pod) -> physx_PxPhysics_Pod ---

}

works_on_pxfound :: proc(foundation: physx_PxFoundation_Pod) {
	fmt.printf("Hello!\n")
}

works_on_pxfound2 :: proc(foundation: physx_PxFoundation_Pod2) {
	fmt.printf("Hello!\n")
}

main :: proc() {
	a: physx_PxFoundation_Pod = physx_create_foundation()
	b: physx_PxFoundation_Pod2
	c: ^physx_PxActor_Pod = nil

	works_on_pxfound(a)
	works_on_pxfound(b)
	works_on_pxfound2(b)

	// fmt.printf("Foobar\n")

	// foundation := physx_create_foundation()
	// physics := physx_create_physics()


	// scene_desc := PxSceneDesc_new(PxPhysics_getTolerancesScale(physics))
	// scene_desc.gravity = PxVec3_new_3(0.0, -9.81, 0.0)

	// dispatcher := phys_PxDefaultCpuDispatcherCreate(1, nil, 0, 0) // XXXX[TSOL]: ENUM 0 => WaitForWork
	// scene_desc.cpuDispatcher = dispatcher
	// scene_desc.filterShader = get_default_simulation_filter_shader()

	// scene:= PxPhysics_createScene_mut(physics, ^scene_desc)
}
