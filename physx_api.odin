package physx

PHYSX_RELEASE :: #config(PHYSX_RELEASE, true)

when ODIN_OS == .Linux {
	foreign import libphysx_api { "libphysx_api_release.so" when PHYSX_RELEASE else "libphysx_api.so" }
}
else when ODIN_OS == .Windows {
	foreign import libphysx_api { "physx_api_release.lib" when PHYSX_RELEASE else "physx_api.lib" }
}

VERSION: u32 : (5 << 24) + (1 << 16) + (3 << 8)

CollisionCallbackExt :: #type proc "c" (user_data: rawptr, pair_header: ^ContactPairHeader, pairs: ^ContactPair, nb_pairs: u32)
TriggerCallbackExt :: #type proc "c" (user_data: rawptr, pairs: ^TriggerPair, nb_pairs: u32);
ConstraintBreakCallbackExt :: #type proc "c" (user_data: rawptr, constraints: ^ConstraintInfo, nb_pairs: u32);
WakeSleepCallbackExt :: #type proc "c" (user_data: rawptr, actors: [^]^Actor, count: u32, is_wake: bool);
AdvanceCallbackExt :: #type proc "c" (user_data: rawptr, body_buffer: [^]^RigidBody, pose_buffer: [^]Transform, count: u32)

SimulationEventCallbackInfo :: struct {
	// Callback for collision events.
	collision_callback: CollisionCallbackExt,
	collision_user_data: rawptr,
	// Callback for trigger shape events (an object entered or left a trigger shape).
	trigger_callback: TriggerCallbackExt,
	trigger_user_data: rawptr,
	// Callback for when a constraint breaks (such as a joint with a force limit)
	constraint_break_callback: ConstraintBreakCallbackExt,
	constraint_break_user_data: rawptr,
	// Callback for when an object falls asleep or is awoken.
	wake_sleep_callback: WakeSleepCallbackExt,
	wake_sleep_user_data: rawptr,
	// Callback to get the next pose early for objects (if flagged with eENABLE_POSE_INTEGRATION_PREVIEW).
	advance_callback: AdvanceCallbackExt,
	advance_user_data: rawptr,
}

RaycastHitCallbackExt :: #type proc "c" (
	actor: ^RigidActor,
	filter_data: ^FilterData,
	shape: ^Shape,
	hit_flags: u32,
	user_data: rawptr,
) -> QueryHitType;

PostFilterCallbackExt :: #type proc "c" (filter_data: ^FilterData, hit: ^QueryHit, user_data: rawptr) -> QueryHitType;

FilterShaderCallbackInfo :: struct {
	attributes0: u32,
	attributes1: u32,
	filter_data0: FilterData,
	filter_data1: FilterData,
	pair_flags: ^PairFlags_Set,
	constant_block: rawptr,
	constant_block_size: u32,
}

SimulationFilterShaderExt :: #type proc "c" (^FilterShaderCallbackInfo) -> FilterFlags_Set;

RaycastProcessTouchesCallbackExt :: #type proc "c" (^RaycastHit, u32, rawptr) -> bool;
SweepProcessTouchesCallbackExt :: #type proc "c" (^SweepHit, u32, rawptr) -> bool;
OverlapProcessTouchesCallbackExt :: #type proc "c" (^OverlapHit, u32, rawptr) -> bool;

FinalizeQueryCallbackExt :: #type proc "c" (rawptr);

AllocCallbackExt :: #type proc "c" (size: u64, type_name: cstring, filename: cstring, line: i32, user_data: rawptr) -> rawptr;
DeallocCallbackExt :: #type proc "c" (ptr: rawptr, user_data: rawptr);
ZoneStartCallbackExt :: #type proc "c" (type_name: cstring, detached: bool, ctx: u64, user_data: rawptr) -> rawptr;
ZoneEndCallbackExt :: #type proc "c" (profiler_data: rawptr, type_name: cstring, detached: bool, ctx: u64, user_data: rawptr);
ErrorCallbackExt :: #type proc "c" (code: ErrorCode, message: cstring, file: cstring, line: i32, user_data: rawptr);
AssertHandlerExt :: #type proc "c" (expr: cstring, file: cstring, line: i32, should_ignore: ^bool, user_data: rawptr);

@(default_calling_convention = "c")
foreign libphysx_api {
	@(link_name = "physx_create_foundation")
	create_foundation_ext :: proc() -> ^Foundation ---

	@(link_name = "physx_create_physics")
	create_physics_ext :: proc(foundation: ^Foundation) -> ^Physics ---

	get_default_simulation_filter_shader :: proc() -> rawptr ---

	@(link_name = "physx_create_foundation_with_alloc")
	create_foundation_with_alloc_ext :: proc(
		allocator: ^DefaultAllocator,
	) -> ^Foundation ---

	get_default_allocator :: proc() -> ^DefaultAllocator ---
	get_default_error_callback :: proc() -> ^DefaultErrorCallback ---

	/// Destroy the returned callback object using QueryFilterCallback_delete.
	create_raycast_filter_callback :: proc(
		actor_to_ignore: ^RigidActor,
	) -> ^QueryFilterCallback ---

	/// Destroy the returned callback object using QueryFilterCallback_delete.
	create_raycast_filter_callback_func :: proc(
		callback: RaycastHitCallbackExt,
		userdata: rawptr,
	) -> ^QueryFilterCallback ---

	/// Destroy the returned callback object using QueryFilterCallback_delete.
	create_pre_and_post_raycast_filter_callback_func :: proc(
		preFilter: RaycastHitCallbackExt,
		postFilter: PostFilterCallbackExt,
		userdata: rawptr,
	) -> ^QueryFilterCallback ---

	create_raycast_buffer :: proc() -> ^RaycastCallback ---
	create_sweep_buffer :: proc() -> ^SweepCallback ---
	create_overlap_buffer :: proc() -> ^OverlapCallback ---

	create_raycast_callback :: proc(
		process_touches_callback: RaycastProcessTouchesCallbackExt,
		finalize_query_callback: FinalizeQueryCallbackExt,
		touches_buffer: ^RaycastHit,
		num_touches: u32,
		userdata: rawptr,
	) -> ^RaycastCallback ---
	create_sweep_callback :: proc(
		process_touches_callback: SweepProcessTouchesCallbackExt,
		finalize_query_callback: FinalizeQueryCallbackExt,
		touches_buffer: ^SweepHit,
		num_touches: u32,
		userdata: rawptr,
	) -> ^SweepCallback ---
	create_overlap_callback :: proc(
		process_touches_callback: OverlapProcessTouchesCallbackExt,
		finalize_query_callback: FinalizeQueryCallbackExt,
		touches_buffer: ^OverlapHit,
		num_touches: u32,
		userdata: rawptr,
	) -> ^OverlapCallback ---

	delete_raycast_callback :: proc(callback: ^RaycastCallback) ---
	delete_sweep_callback :: proc(callback: ^SweepCallback) ---
	delete_overlap_callback :: proc(callback: ^OverlapCallback) ---

	create_alloc_callback :: proc(
		alloc_callback: AllocCallbackExt,
		dealloc_callback: DeallocCallbackExt,
		userdata: rawptr,
	) -> ^AllocatorCallback ---

	create_profiler_callback :: proc(
		zone_start_callback: ZoneStartCallbackExt,
		zone_end_callback: ZoneEndCallbackExt,
		userdata: rawptr,
	) -> ^ProfilerCallback ---

	get_alloc_callback_user_data :: proc(alloc_callback: ^AllocatorCallback) -> rawptr ---

	create_error_callback :: proc(
		error_callback: ErrorCallbackExt,
		userdata: rawptr,
	) -> ^ErrorCallback ---

	create_assert_handler :: proc(
		error_callback: AssertHandlerExt,
		userdata: rawptr,
	) -> ^AssertHandler ---

	/// New interface to handle simulation events, replacing create_contact_callback.
	create_simulation_event_callbacks :: proc(
		callbacks: ^SimulationEventCallbackInfo,
	) -> ^SimulationEventCallback ---

	get_simulation_event_info :: proc(
		callback: ^SimulationEventCallback,
	) -> ^SimulationEventCallbackInfo ---

	destroy_simulation_event_callbacks :: proc(callback: ^SimulationEventCallback) ---

	/// Override the default filter shader in the scene with a custom function.
	/// If call_default_filter_shader_first is set to true, this will first call the
	/// built-in PhysX filter (that matches Physx 2.8 behavior) before your callback.
	enable_custom_filter_shader :: proc(
		scene_desc: ^SceneDesc,
		shader: SimulationFilterShaderExt,
		call_default_filter_shader_first: u32,
	) ---

	/// Should only be used in testing etc! This isn't generated as we don't generate op functions.
	AssertHandler_opCall_mut :: proc(
		self_: ^AssertHandler,
		expr: cstring,
		file: cstring,
		line: i32,
		ignore: ^bool,
	) ---
}
