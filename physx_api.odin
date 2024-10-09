package physx

PHYSX_RELEASE :: #config(PHYSX_RELEASE, true)

when ODIN_OS == .Linux {
    foreign import libphysx_api { "libphysx_api_release.so" when PHYSX_RELEASE else "libphysx_api.so" }
}
else when ODIN_OS == .Windows {
    foreign import libphysx_api { "physx_api_release.lib" when PHYSX_RELEASE else "physx_api.lib" }
}

VERSION: u32 : (5 << 24) + (1 << 16) + (3 << 8)


@(default_calling_convention = "c")
foreign libphysx_api {
	@(link_name = "physx_create_foundation")
	create_foundation_ext :: proc() -> ^Foundation ---

	@(link_name = "physx_create_physics")
	create_physics_ext :: proc(foundation: ^Foundation) -> ^Physics ---

	get_default_simulation_filter_shader :: proc() -> rawptr ---
}
