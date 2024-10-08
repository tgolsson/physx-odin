package physx

VERSION: u32 : (5 << 24) + (1 << 16) + (3 << 8)

when ODIN_OS == .Linux do foreign import px_api "libphysx_api.so"

@(default_calling_convention = "c")
foreign px_api {
	@(link_name = "physx_create_foundation")
	physx_create_foundation :: proc() -> ^PxFoundation ---

	@(link_name = "physx_create_physics")
	physx_create_physics :: proc(foundation: ^PxFoundation) -> ^PxPhysics ---

	get_default_simulation_filter_shader :: proc() -> rawptr ---
}
