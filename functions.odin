package physx
import _c "core:c"
when ODIN_OS == .Linux {
    foreign import libphysx { "libphysx_release.so" when PHYSX_RELEASE else "libphysx.so" }
}
else when ODIN_OS == .Windows {
    foreign import libphysx { "physx_release.lib" when PHYSX_RELEASE else "physx.lib", "system:msvcrt.lib" }
}

@(default_calling_convention = "c")
foreign libphysx {
    @(link_name = "PxAllocatorCallback_delete")
    allocator_callback_delete :: proc(self_: ^AllocatorCallback) ---

    /// Allocates size bytes of memory, which must be 16-byte aligned.
    ///
    /// This method should never return NULL.  If you run out of memory, then
    /// you should terminate the app or take some other appropriate action.
    ///
    /// Threading:
    /// This function should be thread safe as it can be called in the context of the user thread
    /// and physics processing thread(s).
    ///
    /// The allocated block of memory.
    @(link_name = "PxAllocatorCallback_allocate_mut")
    allocator_callback_allocate_mut :: proc(self_: ^AllocatorCallback, size: _c.size_t, typeName: cstring, filename: cstring, line: _c.int32_t) -> rawptr ---

    /// Frees memory previously allocated by allocate().
    ///
    /// Threading:
    /// This function should be thread safe as it can be called in the context of the user thread
    /// and physics processing thread(s).
    @(link_name = "PxAllocatorCallback_deallocate_mut")
    allocator_callback_deallocate_mut :: proc(self_: ^AllocatorCallback, ptr: rawptr) ---

    @(link_name = "PxAssertHandler_delete")
    assert_handler_delete :: proc(self_: ^AssertHandler) ---

    @(link_name = "phys_PxGetAssertHandler")
    get_assert_handler :: proc() -> ^AssertHandler ---

    @(link_name = "phys_PxSetAssertHandler")
    set_assert_handler :: proc(handler: ^AssertHandler) ---

    /// Destroys the instance it is called on.
    ///
    /// The operation will fail, if there are still modules referencing the foundation object. Release all dependent modules
    /// prior to calling this method.
    @(link_name = "PxFoundation_release_mut")
    foundation_release_mut :: proc(self_: ^Foundation) ---

    /// retrieves error callback
    @(link_name = "PxFoundation_getErrorCallback_mut")
    foundation_get_error_callback_mut :: proc(self_: ^Foundation) -> ^ErrorCallback ---

    /// Sets mask of errors to report.
    @(link_name = "PxFoundation_setErrorLevel_mut")
    foundation_set_error_level_mut :: proc(self_: ^Foundation, mask: _c.uint32_t) ---

    /// Retrieves mask of errors to be reported.
    @(link_name = "PxFoundation_getErrorLevel")
    foundation_get_error_level :: proc(self_: ^Foundation) -> _c.uint32_t ---

    /// Retrieves the allocator this object was created with.
    @(link_name = "PxFoundation_getAllocatorCallback_mut")
    foundation_get_allocator_callback_mut :: proc(self_: ^Foundation) -> ^AllocatorCallback ---

    /// Retrieves if allocation names are being passed to allocator callback.
    @(link_name = "PxFoundation_getReportAllocationNames")
    foundation_get_report_allocation_names :: proc(self_: ^Foundation) -> _c.bool ---

    /// Set if allocation names are being passed to allocator callback.
    ///
    /// Enabled by default in debug and checked build, disabled by default in profile and release build.
    @(link_name = "PxFoundation_setReportAllocationNames_mut")
    foundation_set_report_allocation_names_mut :: proc(self_: ^Foundation, value: _c.bool) ---

    @(link_name = "PxFoundation_registerAllocationListener_mut")
    foundation_register_allocation_listener_mut :: proc(self_: ^Foundation, listener: ^AllocationListener) ---

    @(link_name = "PxFoundation_deregisterAllocationListener_mut")
    foundation_deregister_allocation_listener_mut :: proc(self_: ^Foundation, listener: ^AllocationListener) ---

    @(link_name = "PxFoundation_registerErrorCallback_mut")
    foundation_register_error_callback_mut :: proc(self_: ^Foundation, callback: ^ErrorCallback) ---

    @(link_name = "PxFoundation_deregisterErrorCallback_mut")
    foundation_deregister_error_callback_mut :: proc(self_: ^Foundation, callback: ^ErrorCallback) ---

    /// Creates an instance of the foundation class
    ///
    /// The foundation class is needed to initialize higher level SDKs. There may be only one instance per process.
    /// Calling this method after an instance has been created already will result in an error message and NULL will be
    /// returned.
    ///
    /// Foundation instance on success, NULL if operation failed
    @(link_name = "phys_PxCreateFoundation")
    create_foundation :: proc(version: _c.uint32_t, allocator: ^AllocatorCallback, errorCallback: ^ErrorCallback) -> ^Foundation ---

    @(link_name = "phys_PxSetFoundationInstance")
    set_foundation_instance :: proc(foundation: ^Foundation) ---

    @(link_name = "phys_PxGetFoundation")
    get_foundation :: proc() -> ^Foundation ---

    /// Get the callback that will be used for all profiling.
    @(link_name = "phys_PxGetProfilerCallback")
    get_profiler_callback :: proc() -> ^ProfilerCallback ---

    /// Set the callback that will be used for all profiling.
    @(link_name = "phys_PxSetProfilerCallback")
    set_profiler_callback :: proc(profiler: ^ProfilerCallback) ---

    /// Get the allocator callback
    @(link_name = "phys_PxGetAllocatorCallback")
    get_allocator_callback :: proc() -> ^AllocatorCallback ---

    /// Get the broadcasting allocator callback
    @(link_name = "phys_PxGetBroadcastAllocator")
    get_broadcast_allocator :: proc() -> ^AllocatorCallback ---

    /// Get the error callback
    @(link_name = "phys_PxGetErrorCallback")
    get_error_callback :: proc() -> ^ErrorCallback ---

    /// Get the broadcasting error callback
    @(link_name = "phys_PxGetBroadcastError")
    get_broadcast_error :: proc() -> ^ErrorCallback ---

    /// Get the warn once timestamp
    @(link_name = "phys_PxGetWarnOnceTimeStamp")
    get_warn_once_time_stamp :: proc() -> _c.uint32_t ---

    /// Decrement the ref count of PxFoundation
    @(link_name = "phys_PxDecFoundationRefCount")
    dec_foundation_ref_count :: proc() ---

    /// Increment the ref count of PxFoundation
    @(link_name = "phys_PxIncFoundationRefCount")
    inc_foundation_ref_count :: proc() ---

    @(link_name = "PxAllocator_new")
    allocator_new :: proc(anon_param0: cstring) -> Allocator ---

    @(link_name = "PxAllocator_allocate_mut")
    allocator_allocate_mut :: proc(self_: ^Allocator, size: _c.size_t, file: cstring, line: _c.int32_t) -> rawptr ---

    @(link_name = "PxAllocator_deallocate_mut")
    allocator_deallocate_mut :: proc(self_: ^Allocator, ptr: rawptr) ---

    @(link_name = "PxRawAllocator_new")
    raw_allocator_new :: proc(anon_param0: cstring) -> RawAllocator ---

    @(link_name = "PxRawAllocator_allocate_mut")
    raw_allocator_allocate_mut :: proc(self_: ^RawAllocator, size: _c.size_t, anon_param1: cstring, anon_param2: _c.int32_t) -> rawptr ---

    @(link_name = "PxRawAllocator_deallocate_mut")
    raw_allocator_deallocate_mut :: proc(self_: ^RawAllocator, ptr: rawptr) ---

    @(link_name = "PxVirtualAllocatorCallback_delete")
    virtual_allocator_callback_delete :: proc(self_: ^VirtualAllocatorCallback) ---

    @(link_name = "PxVirtualAllocatorCallback_allocate_mut")
    virtual_allocator_callback_allocate_mut :: proc(self_: ^VirtualAllocatorCallback, size: _c.size_t, group: _c.int32_t, file: cstring, line: _c.int32_t) -> rawptr ---

    @(link_name = "PxVirtualAllocatorCallback_deallocate_mut")
    virtual_allocator_callback_deallocate_mut :: proc(self_: ^VirtualAllocatorCallback, ptr: rawptr) ---

    @(link_name = "PxVirtualAllocator_new")
    virtual_allocator_new :: proc(callback: ^VirtualAllocatorCallback, group: _c.int32_t) -> VirtualAllocator ---

    @(link_name = "PxVirtualAllocator_allocate_mut")
    virtual_allocator_allocate_mut :: proc(self_: ^VirtualAllocator, size: _c.size_t, file: cstring, line: _c.int32_t) -> rawptr ---

    @(link_name = "PxVirtualAllocator_deallocate_mut")
    virtual_allocator_deallocate_mut :: proc(self_: ^VirtualAllocator, ptr: rawptr) ---

    @(link_name = "PxTempAllocatorChunk_new")
    temp_allocator_chunk_new :: proc() -> TempAllocatorChunk ---

    @(link_name = "PxTempAllocator_new")
    temp_allocator_new :: proc(anon_param0: cstring) -> TempAllocator ---

    @(link_name = "PxTempAllocator_allocate_mut")
    temp_allocator_allocate_mut :: proc(self_: ^TempAllocator, size: _c.size_t, file: cstring, line: _c.int32_t) -> rawptr ---

    @(link_name = "PxTempAllocator_deallocate_mut")
    temp_allocator_deallocate_mut :: proc(self_: ^TempAllocator, ptr: rawptr) ---

    /// Sets the bytes of the provided buffer to zero.
    ///
    /// Pointer to memory block (same as input)
    @(link_name = "phys_PxMemZero")
    mem_zero :: proc(dest: rawptr, count: _c.uint32_t) -> rawptr ---

    /// Sets the bytes of the provided buffer to the specified value.
    ///
    /// Pointer to memory block (same as input)
    @(link_name = "phys_PxMemSet")
    mem_set :: proc(dest: rawptr, c: _c.int32_t, count: _c.uint32_t) -> rawptr ---

    /// Copies the bytes of one memory block to another. The memory blocks must not overlap.
    ///
    /// Use [`PxMemMove`] if memory blocks overlap.
    ///
    /// Pointer to destination memory block
    @(link_name = "phys_PxMemCopy")
    mem_copy :: proc(dest: rawptr, src: rawptr, count: _c.uint32_t) -> rawptr ---

    /// Copies the bytes of one memory block to another. The memory blocks can overlap.
    ///
    /// Use [`PxMemCopy`] if memory blocks do not overlap.
    ///
    /// Pointer to destination memory block
    @(link_name = "phys_PxMemMove")
    mem_move :: proc(dest: rawptr, src: rawptr, count: _c.uint32_t) -> rawptr ---

    /// Mark a specified amount of memory with 0xcd pattern. This is used to check that the meta data
    /// definition for serialized classes is complete in checked builds.
    @(link_name = "phys_PxMarkSerializedMemory")
    mark_serialized_memory :: proc(ptr: rawptr, byteSize: _c.uint32_t) ---

    @(link_name = "phys_PxMemoryBarrier")
    memory_barrier :: proc() ---

    /// Return the index of the highest set bit. Undefined for zero arg.
    @(link_name = "phys_PxHighestSetBitUnsafe")
    highest_set_bit_unsafe :: proc(v: _c.uint32_t) -> _c.uint32_t ---

    /// Return the index of the highest set bit. Undefined for zero arg.
    @(link_name = "phys_PxLowestSetBitUnsafe")
    lowest_set_bit_unsafe :: proc(v: _c.uint32_t) -> _c.uint32_t ---

    /// Returns the index of the highest set bit. Returns 32 for v=0.
    @(link_name = "phys_PxCountLeadingZeros")
    count_leading_zeros :: proc(v: _c.uint32_t) -> _c.uint32_t ---

    /// Prefetch aligned 64B x86, 32b ARM around
    @(link_name = "phys_PxPrefetchLine")
    prefetch_line :: proc(ptr: rawptr, offset: _c.uint32_t) ---

    /// Prefetch
    /// bytes starting at
    @(link_name = "phys_PxPrefetch")
    prefetch :: proc(ptr: rawptr, count: _c.uint32_t) ---

    @(link_name = "phys_PxBitCount")
    bit_count :: proc(v: _c.uint32_t) -> _c.uint32_t ---

    @(link_name = "phys_PxIsPowerOfTwo")
    is_power_of_two :: proc(x: _c.uint32_t) -> _c.bool ---

    @(link_name = "phys_PxNextPowerOfTwo")
    next_power_of_two :: proc(x: _c.uint32_t) -> _c.uint32_t ---

    /// Return the index of the highest set bit. Not valid for zero arg.
    @(link_name = "phys_PxLowestSetBit")
    lowest_set_bit :: proc(x: _c.uint32_t) -> _c.uint32_t ---

    /// Return the index of the highest set bit. Not valid for zero arg.
    @(link_name = "phys_PxHighestSetBit")
    highest_set_bit :: proc(x: _c.uint32_t) -> _c.uint32_t ---

    @(link_name = "phys_PxILog2")
    i_log2 :: proc(num: _c.uint32_t) -> _c.uint32_t ---

    /// default constructor leaves data uninitialized.
    @(link_name = "PxVec3_new")
    vec3_new :: proc() -> Vec3 ---

    /// zero constructor.
    @(link_name = "PxVec3_new_1")
    vec3_new_1 :: proc(anon_param0: ZERO) -> Vec3 ---

    /// Assigns scalar parameter to all elements.
    ///
    /// Useful to initialize to zero or one.
    @(link_name = "PxVec3_new_2")
    vec3_new_2 :: proc(a: _c.float) -> Vec3 ---

    /// Initializes from 3 scalar parameters.
    @(link_name = "PxVec3_new_3")
    vec3_new_3 :: proc(nx: _c.float, ny: _c.float, nz: _c.float) -> Vec3 ---

    /// tests for exact zero vector
    @(link_name = "PxVec3_isZero")
    vec3_is_zero :: proc(self_: ^Vec3) -> _c.bool ---

    /// returns true if all 3 elems of the vector are finite (not NAN or INF, etc.)
    @(link_name = "PxVec3_isFinite")
    vec3_is_finite :: proc(self_: ^Vec3) -> _c.bool ---

    /// is normalized - used by API parameter validation
    @(link_name = "PxVec3_isNormalized")
    vec3_is_normalized :: proc(self_: ^Vec3) -> _c.bool ---

    /// returns the squared magnitude
    ///
    /// Avoids calling PxSqrt()!
    @(link_name = "PxVec3_magnitudeSquared")
    vec3_magnitude_squared :: proc(self_: ^Vec3) -> _c.float ---

    /// returns the magnitude
    @(link_name = "PxVec3_magnitude")
    vec3_magnitude :: proc(self_: ^Vec3) -> _c.float ---

    /// returns the scalar product of this and other.
    @(link_name = "PxVec3_dot")
    vec3_dot :: proc(self_: ^Vec3, #by_ptr v: Vec3) -> _c.float ---

    /// cross product
    @(link_name = "PxVec3_cross")
    vec3_cross :: proc(self_: ^Vec3, #by_ptr v: Vec3) -> Vec3 ---

    /// returns a unit vector
    @(link_name = "PxVec3_getNormalized")
    vec3_get_normalized :: proc(self_: ^Vec3) -> Vec3 ---

    /// normalizes the vector in place
    @(link_name = "PxVec3_normalize_mut")
    vec3_normalize_mut :: proc(self_: ^Vec3) -> _c.float ---

    /// normalizes the vector in place. Does nothing if vector magnitude is under PX_NORMALIZATION_EPSILON.
    /// Returns vector magnitude if >= PX_NORMALIZATION_EPSILON and 0.0f otherwise.
    @(link_name = "PxVec3_normalizeSafe_mut")
    vec3_normalize_safe_mut :: proc(self_: ^Vec3) -> _c.float ---

    /// normalizes the vector in place. Asserts if vector magnitude is under PX_NORMALIZATION_EPSILON.
    /// returns vector magnitude.
    @(link_name = "PxVec3_normalizeFast_mut")
    vec3_normalize_fast_mut :: proc(self_: ^Vec3) -> _c.float ---

    /// a[i] * b[i], for all i.
    @(link_name = "PxVec3_multiply")
    vec3_multiply :: proc(self_: ^Vec3, #by_ptr a: Vec3) -> Vec3 ---

    /// element-wise minimum
    @(link_name = "PxVec3_minimum")
    vec3_minimum :: proc(self_: ^Vec3, #by_ptr v: Vec3) -> Vec3 ---

    /// returns MIN(x, y, z);
    @(link_name = "PxVec3_minElement")
    vec3_min_element :: proc(self_: ^Vec3) -> _c.float ---

    /// element-wise maximum
    @(link_name = "PxVec3_maximum")
    vec3_maximum :: proc(self_: ^Vec3, #by_ptr v: Vec3) -> Vec3 ---

    /// returns MAX(x, y, z);
    @(link_name = "PxVec3_maxElement")
    vec3_max_element :: proc(self_: ^Vec3) -> _c.float ---

    /// returns absolute values of components;
    @(link_name = "PxVec3_abs")
    vec3_abs :: proc(self_: ^Vec3) -> Vec3 ---

    @(link_name = "PxVec3Padded_new_alloc")
    vec3_padded_new_alloc :: proc() -> ^Vec3Padded ---

    @(link_name = "PxVec3Padded_delete")
    vec3_padded_delete :: proc(self_: ^Vec3Padded) ---

    @(link_name = "PxVec3Padded_new_alloc_1")
    vec3_padded_new_alloc_1 :: proc(#by_ptr p: Vec3) -> ^Vec3Padded ---

    @(link_name = "PxVec3Padded_new_alloc_2")
    vec3_padded_new_alloc_2 :: proc(f: _c.float) -> ^Vec3Padded ---

    /// Default constructor, does not do any initialization.
    @(link_name = "PxQuat_new")
    quat_new :: proc() -> Quat ---

    /// identity constructor
    @(link_name = "PxQuat_new_1")
    quat_new_1 :: proc(anon_param0: IDENTITY) -> Quat ---

    /// Constructor from a scalar: sets the real part w to the scalar value, and the imaginary parts (x,y,z) to zero
    @(link_name = "PxQuat_new_2")
    quat_new_2 :: proc(r: _c.float) -> Quat ---

    /// Constructor. Take note of the order of the elements!
    @(link_name = "PxQuat_new_3")
    quat_new_3 :: proc(nx: _c.float, ny: _c.float, nz: _c.float, nw: _c.float) -> Quat ---

    /// Creates from angle-axis representation.
    ///
    /// Axis must be normalized!
    ///
    /// Angle is in radians!
    ///
    /// Unit:
    /// Radians
    @(link_name = "PxQuat_new_4")
    quat_new_4 :: proc(angleRadians: _c.float, #by_ptr unitAxis: Vec3) -> Quat ---

    /// Creates from orientation matrix.
    @(link_name = "PxQuat_new_5")
    quat_new_5 :: proc(#by_ptr m: Mat33) -> Quat ---

    /// returns true if quat is identity
    @(link_name = "PxQuat_isIdentity")
    quat_is_identity :: proc(self_: ^Quat) -> _c.bool ---

    /// returns true if all elements are finite (not NAN or INF, etc.)
    @(link_name = "PxQuat_isFinite")
    quat_is_finite :: proc(self_: ^Quat) -> _c.bool ---

    /// returns true if finite and magnitude is close to unit
    @(link_name = "PxQuat_isUnit")
    quat_is_unit :: proc(self_: ^Quat) -> _c.bool ---

    /// returns true if finite and magnitude is reasonably close to unit to allow for some accumulation of error vs
    /// isValid
    @(link_name = "PxQuat_isSane")
    quat_is_sane :: proc(self_: ^Quat) -> _c.bool ---

    /// converts this quaternion to angle-axis representation
    @(link_name = "PxQuat_toRadiansAndUnitAxis")
    quat_to_radians_and_unit_axis :: proc(self_: ^Quat, angle: ^_c.float, axis: ^Vec3) ---

    /// Gets the angle between this quat and the identity quaternion.
    ///
    /// Unit:
    /// Radians
    @(link_name = "PxQuat_getAngle")
    quat_get_angle :: proc(self_: ^Quat) -> _c.float ---

    /// Gets the angle between this quat and the argument
    ///
    /// Unit:
    /// Radians
    @(link_name = "PxQuat_getAngle_1")
    quat_get_angle_1 :: proc(self_: ^Quat, #by_ptr q: Quat) -> _c.float ---

    /// This is the squared 4D vector length, should be 1 for unit quaternions.
    @(link_name = "PxQuat_magnitudeSquared")
    quat_magnitude_squared :: proc(self_: ^Quat) -> _c.float ---

    /// returns the scalar product of this and other.
    @(link_name = "PxQuat_dot")
    quat_dot :: proc(self_: ^Quat, #by_ptr v: Quat) -> _c.float ---

    @(link_name = "PxQuat_getNormalized")
    quat_get_normalized :: proc(self_: ^Quat) -> Quat ---

    @(link_name = "PxQuat_magnitude")
    quat_magnitude :: proc(self_: ^Quat) -> _c.float ---

    /// maps to the closest unit quaternion.
    @(link_name = "PxQuat_normalize_mut")
    quat_normalize_mut :: proc(self_: ^Quat) -> _c.float ---

    @(link_name = "PxQuat_getConjugate")
    quat_get_conjugate :: proc(self_: ^Quat) -> Quat ---

    @(link_name = "PxQuat_getImaginaryPart")
    quat_get_imaginary_part :: proc(self_: ^Quat) -> Vec3 ---

    /// brief computes rotation of x-axis
    @(link_name = "PxQuat_getBasisVector0")
    quat_get_basis_vector0 :: proc(self_: ^Quat) -> Vec3 ---

    /// brief computes rotation of y-axis
    @(link_name = "PxQuat_getBasisVector1")
    quat_get_basis_vector1 :: proc(self_: ^Quat) -> Vec3 ---

    /// brief computes rotation of z-axis
    @(link_name = "PxQuat_getBasisVector2")
    quat_get_basis_vector2 :: proc(self_: ^Quat) -> Vec3 ---

    /// rotates passed vec by this (assumed unitary)
    @(link_name = "PxQuat_rotate")
    quat_rotate :: proc(self_: ^Quat, #by_ptr v: Vec3) -> Vec3 ---

    /// inverse rotates passed vec by this (assumed unitary)
    @(link_name = "PxQuat_rotateInv")
    quat_rotate_inv :: proc(self_: ^Quat, #by_ptr v: Vec3) -> Vec3 ---

    @(link_name = "PxTransform_new")
    transform_new :: proc() -> Transform ---

    @(link_name = "PxTransform_new_1")
    transform_new_1 :: proc(#by_ptr position: Vec3) -> Transform ---

    @(link_name = "PxTransform_new_2")
    transform_new_2 :: proc(anon_param0: IDENTITY) -> Transform ---

    @(link_name = "PxTransform_new_3")
    transform_new_3 :: proc(#by_ptr orientation: Quat) -> Transform ---

    @(link_name = "PxTransform_new_4")
    transform_new_4 :: proc(x: _c.float, y: _c.float, z: _c.float, aQ: Quat) -> Transform ---

    @(link_name = "PxTransform_new_5")
    transform_new_5 :: proc(#by_ptr p0: Vec3, #by_ptr q0: Quat) -> Transform ---

    @(link_name = "PxTransform_new_6")
    transform_new_6 :: proc(#by_ptr m: Mat44) -> Transform ---

    @(link_name = "PxTransform_getInverse")
    transform_get_inverse :: proc(self_: ^Transform) -> Transform ---

    @(link_name = "PxTransform_transform")
    transform_transform :: proc(self_: ^Transform, #by_ptr input: Vec3) -> Vec3 ---

    @(link_name = "PxTransform_transformInv")
    transform_transform_inv :: proc(self_: ^Transform, #by_ptr input: Vec3) -> Vec3 ---

    @(link_name = "PxTransform_rotate")
    transform_rotate :: proc(self_: ^Transform, #by_ptr input: Vec3) -> Vec3 ---

    @(link_name = "PxTransform_rotateInv")
    transform_rotate_inv :: proc(self_: ^Transform, #by_ptr input: Vec3) -> Vec3 ---

    /// Transform transform to parent (returns compound transform: first src, then *this)
    @(link_name = "PxTransform_transform_1")
    transform_transform_1 :: proc(self_: ^Transform, #by_ptr src: Transform) -> Transform ---

    /// returns true if finite and q is a unit quaternion
    @(link_name = "PxTransform_isValid")
    transform_is_valid :: proc(self_: ^Transform) -> _c.bool ---

    /// returns true if finite and quat magnitude is reasonably close to unit to allow for some accumulation of error
    /// vs isValid
    @(link_name = "PxTransform_isSane")
    transform_is_sane :: proc(self_: ^Transform) -> _c.bool ---

    /// returns true if all elems are finite (not NAN or INF, etc.)
    @(link_name = "PxTransform_isFinite")
    transform_is_finite :: proc(self_: ^Transform) -> _c.bool ---

    /// Transform transform from parent (returns compound transform: first src, then this->inverse)
    @(link_name = "PxTransform_transformInv_1")
    transform_transform_inv_1 :: proc(self_: ^Transform, #by_ptr src: Transform) -> Transform ---

    /// return a normalized transform (i.e. one in which the quaternion has unit magnitude)
    @(link_name = "PxTransform_getNormalized")
    transform_get_normalized :: proc(self_: ^Transform) -> Transform ---

    /// Default constructor
    @(link_name = "PxMat33_new")
    mat33_new :: proc() -> Mat33 ---

    /// identity constructor
    @(link_name = "PxMat33_new_1")
    mat33_new_1 :: proc(anon_param0: IDENTITY) -> Mat33 ---

    /// zero constructor
    @(link_name = "PxMat33_new_2")
    mat33_new_2 :: proc(anon_param0: ZERO) -> Mat33 ---

    /// Construct from three base vectors
    @(link_name = "PxMat33_new_3")
    mat33_new_3 :: proc(#by_ptr col0: Vec3, #by_ptr col1: Vec3, #by_ptr col2: Vec3) -> Mat33 ---

    /// constructor from a scalar, which generates a multiple of the identity matrix
    @(link_name = "PxMat33_new_4")
    mat33_new_4 :: proc(r: _c.float) -> Mat33 ---

    /// Construct from float[9]
    @(link_name = "PxMat33_new_5")
    mat33_new_5 :: proc(values: ^_c.float) -> Mat33 ---

    /// Construct from a quaternion
    @(link_name = "PxMat33_new_6")
    mat33_new_6 :: proc(#by_ptr q: Quat) -> Mat33 ---

    /// Construct from diagonal, off-diagonals are zero.
    @(link_name = "PxMat33_createDiagonal")
    mat33_create_diagonal :: proc(#by_ptr d: Vec3) -> Mat33 ---

    /// Computes the outer product of two vectors
    @(link_name = "PxMat33_outer")
    mat33_outer :: proc(#by_ptr a: Vec3, #by_ptr b: Vec3) -> Mat33 ---

    /// Get transposed matrix
    @(link_name = "PxMat33_getTranspose")
    mat33_get_transpose :: proc(self_: ^Mat33) -> Mat33 ---

    /// Get the real inverse
    @(link_name = "PxMat33_getInverse")
    mat33_get_inverse :: proc(self_: ^Mat33) -> Mat33 ---

    /// Get determinant
    @(link_name = "PxMat33_getDeterminant")
    mat33_get_determinant :: proc(self_: ^Mat33) -> _c.float ---

    /// Transform vector by matrix, equal to v' = M*v
    @(link_name = "PxMat33_transform")
    mat33_transform :: proc(self_: ^Mat33, #by_ptr other: Vec3) -> Vec3 ---

    /// Transform vector by matrix transpose, v' = M^t*v
    @(link_name = "PxMat33_transformTranspose")
    mat33_transform_transpose :: proc(self_: ^Mat33, #by_ptr other: Vec3) -> Vec3 ---

    @(link_name = "PxMat33_front")
    mat33_front :: proc(self_: ^Mat33) -> ^_c.float ---

    /// Default constructor, not performing any initialization for performance reason.
    ///
    /// Use empty() function below to construct empty bounds.
    @(link_name = "PxBounds3_new")
    bounds3_new :: proc() -> Bounds3 ---

    /// Construct from two bounding points
    @(link_name = "PxBounds3_new_1")
    bounds3_new_1 :: proc(#by_ptr minimum: Vec3, #by_ptr maximum: Vec3) -> Bounds3 ---

    /// Return empty bounds.
    @(link_name = "PxBounds3_empty")
    bounds3_empty :: proc() -> Bounds3 ---

    /// returns the AABB containing v0 and v1.
    @(link_name = "PxBounds3_boundsOfPoints")
    bounds3_bounds_of_points :: proc(#by_ptr v0: Vec3, #by_ptr v1: Vec3) -> Bounds3 ---

    /// returns the AABB from center and extents vectors.
    @(link_name = "PxBounds3_centerExtents")
    bounds3_center_extents :: proc(#by_ptr center: Vec3, #by_ptr extent: Vec3) -> Bounds3 ---

    /// Construct from center, extent, and (not necessarily orthogonal) basis
    @(link_name = "PxBounds3_basisExtent")
    bounds3_basis_extent :: proc(#by_ptr center: Vec3, #by_ptr basis: Mat33, #by_ptr extent: Vec3) -> Bounds3 ---

    /// Construct from pose and extent
    @(link_name = "PxBounds3_poseExtent")
    bounds3_pose_extent :: proc(#by_ptr pose: Transform, #by_ptr extent: Vec3) -> Bounds3 ---

    /// gets the transformed bounds of the passed AABB (resulting in a bigger AABB).
    ///
    /// This version is safe to call for empty bounds.
    @(link_name = "PxBounds3_transformSafe")
    bounds3_transform_safe :: proc(#by_ptr matrix_: Mat33, #by_ptr bounds: Bounds3) -> Bounds3 ---

    /// gets the transformed bounds of the passed AABB (resulting in a bigger AABB).
    ///
    /// Calling this method for empty bounds leads to undefined behavior. Use [`transformSafe`]() instead.
    @(link_name = "PxBounds3_transformFast")
    bounds3_transform_fast :: proc(#by_ptr matrix_: Mat33, #by_ptr bounds: Bounds3) -> Bounds3 ---

    /// gets the transformed bounds of the passed AABB (resulting in a bigger AABB).
    ///
    /// This version is safe to call for empty bounds.
    @(link_name = "PxBounds3_transformSafe_1")
    bounds3_transform_safe_1 :: proc(#by_ptr transform: Transform, #by_ptr bounds: Bounds3) -> Bounds3 ---

    /// gets the transformed bounds of the passed AABB (resulting in a bigger AABB).
    ///
    /// Calling this method for empty bounds leads to undefined behavior. Use [`transformSafe`]() instead.
    @(link_name = "PxBounds3_transformFast_1")
    bounds3_transform_fast_1 :: proc(#by_ptr transform: Transform, #by_ptr bounds: Bounds3) -> Bounds3 ---

    /// Sets empty to true
    @(link_name = "PxBounds3_setEmpty_mut")
    bounds3_set_empty_mut :: proc(self_: ^Bounds3) ---

    /// Sets the bounds to maximum size [-PX_MAX_BOUNDS_EXTENTS, PX_MAX_BOUNDS_EXTENTS].
    @(link_name = "PxBounds3_setMaximal_mut")
    bounds3_set_maximal_mut :: proc(self_: ^Bounds3) ---

    /// expands the volume to include v
    @(link_name = "PxBounds3_include_mut")
    bounds3_include_mut :: proc(self_: ^Bounds3, #by_ptr v: Vec3) ---

    /// expands the volume to include b.
    @(link_name = "PxBounds3_include_mut_1")
    bounds3_include_mut_1 :: proc(self_: ^Bounds3, #by_ptr b: Bounds3) ---

    @(link_name = "PxBounds3_isEmpty")
    bounds3_is_empty :: proc(self_: ^Bounds3) -> _c.bool ---

    /// indicates whether the intersection of this and b is empty or not.
    @(link_name = "PxBounds3_intersects")
    bounds3_intersects :: proc(self_: ^Bounds3, #by_ptr b: Bounds3) -> _c.bool ---

    /// computes the 1D-intersection between two AABBs, on a given axis.
    @(link_name = "PxBounds3_intersects1D")
    bounds3_intersects1_d :: proc(self_: ^Bounds3, #by_ptr a: Bounds3, axis: _c.uint32_t) -> _c.bool ---

    /// indicates if these bounds contain v.
    @(link_name = "PxBounds3_contains")
    bounds3_contains :: proc(self_: ^Bounds3, #by_ptr v: Vec3) -> _c.bool ---

    /// checks a box is inside another box.
    @(link_name = "PxBounds3_isInside")
    bounds3_is_inside :: proc(self_: ^Bounds3, #by_ptr box: Bounds3) -> _c.bool ---

    /// returns the center of this axis aligned box.
    @(link_name = "PxBounds3_getCenter")
    bounds3_get_center :: proc(self_: ^Bounds3) -> Vec3 ---

    /// get component of the box's center along a given axis
    @(link_name = "PxBounds3_getCenter_1")
    bounds3_get_center_1 :: proc(self_: ^Bounds3, axis: _c.uint32_t) -> _c.float ---

    /// get component of the box's extents along a given axis
    @(link_name = "PxBounds3_getExtents")
    bounds3_get_extents :: proc(self_: ^Bounds3, axis: _c.uint32_t) -> _c.float ---

    /// returns the dimensions (width/height/depth) of this axis aligned box.
    @(link_name = "PxBounds3_getDimensions")
    bounds3_get_dimensions :: proc(self_: ^Bounds3) -> Vec3 ---

    /// returns the extents, which are half of the width/height/depth.
    @(link_name = "PxBounds3_getExtents_1")
    bounds3_get_extents_1 :: proc(self_: ^Bounds3) -> Vec3 ---

    /// scales the AABB.
    ///
    /// This version is safe to call for empty bounds.
    @(link_name = "PxBounds3_scaleSafe_mut")
    bounds3_scale_safe_mut :: proc(self_: ^Bounds3, scale: _c.float) ---

    /// scales the AABB.
    ///
    /// Calling this method for empty bounds leads to undefined behavior. Use [`scaleSafe`]() instead.
    @(link_name = "PxBounds3_scaleFast_mut")
    bounds3_scale_fast_mut :: proc(self_: ^Bounds3, scale: _c.float) ---

    /// fattens the AABB in all 3 dimensions by the given distance.
    ///
    /// This version is safe to call for empty bounds.
    @(link_name = "PxBounds3_fattenSafe_mut")
    bounds3_fatten_safe_mut :: proc(self_: ^Bounds3, distance: _c.float) ---

    /// fattens the AABB in all 3 dimensions by the given distance.
    ///
    /// Calling this method for empty bounds leads to undefined behavior. Use [`fattenSafe`]() instead.
    @(link_name = "PxBounds3_fattenFast_mut")
    bounds3_fatten_fast_mut :: proc(self_: ^Bounds3, distance: _c.float) ---

    /// checks that the AABB values are not NaN
    @(link_name = "PxBounds3_isFinite")
    bounds3_is_finite :: proc(self_: ^Bounds3) -> _c.bool ---

    /// checks that the AABB values describe a valid configuration.
    @(link_name = "PxBounds3_isValid")
    bounds3_is_valid :: proc(self_: ^Bounds3) -> _c.bool ---

    /// Finds the closest point in the box to the point p. If p is contained, this will be p, otherwise it
    /// will be the closest point on the surface of the box.
    @(link_name = "PxBounds3_closestPoint")
    bounds3_closest_point :: proc(self_: ^Bounds3, #by_ptr p: Vec3) -> Vec3 ---

    @(link_name = "PxErrorCallback_delete")
    error_callback_delete :: proc(self_: ^ErrorCallback) ---

    /// Reports an error code.
    @(link_name = "PxErrorCallback_reportError_mut")
    error_callback_report_error_mut :: proc(self_: ^ErrorCallback, code: ErrorCode, message: cstring, file: cstring, line: _c.int32_t) ---

    /// callback when memory is allocated.
    @(link_name = "PxAllocationListener_onAllocation_mut")
    allocation_listener_on_allocation_mut :: proc(self_: ^AllocationListener, size: _c.size_t, typeName: cstring, filename: cstring, line: _c.int32_t, allocatedMemory: rawptr) ---

    /// callback when memory is deallocated.
    @(link_name = "PxAllocationListener_onDeallocation_mut")
    allocation_listener_on_deallocation_mut :: proc(self_: ^AllocationListener, allocatedMemory: rawptr) ---

    /// The default constructor.
    @(link_name = "PxBroadcastingAllocator_new_alloc")
    broadcasting_allocator_new_alloc :: proc(allocator: ^AllocatorCallback, error: ^ErrorCallback) -> ^BroadcastingAllocator ---

    /// The default constructor.
    @(link_name = "PxBroadcastingAllocator_delete")
    broadcasting_allocator_delete :: proc(self_: ^BroadcastingAllocator) ---

    /// Allocates size bytes of memory, which must be 16-byte aligned.
    ///
    /// This method should never return NULL.  If you run out of memory, then
    /// you should terminate the app or take some other appropriate action.
    ///
    /// Threading:
    /// This function should be thread safe as it can be called in the context of the user thread
    /// and physics processing thread(s).
    ///
    /// The allocated block of memory.
    @(link_name = "PxBroadcastingAllocator_allocate_mut")
    broadcasting_allocator_allocate_mut :: proc(self_: ^BroadcastingAllocator, size: _c.size_t, typeName: cstring, filename: cstring, line: _c.int32_t) -> rawptr ---

    /// Frees memory previously allocated by allocate().
    ///
    /// Threading:
    /// This function should be thread safe as it can be called in the context of the user thread
    /// and physics processing thread(s).
    @(link_name = "PxBroadcastingAllocator_deallocate_mut")
    broadcasting_allocator_deallocate_mut :: proc(self_: ^BroadcastingAllocator, ptr: rawptr) ---

    /// The default constructor.
    @(link_name = "PxBroadcastingErrorCallback_new_alloc")
    broadcasting_error_callback_new_alloc :: proc(errorCallback: ^ErrorCallback) -> ^BroadcastingErrorCallback ---

    /// The default destructor.
    @(link_name = "PxBroadcastingErrorCallback_delete")
    broadcasting_error_callback_delete :: proc(self_: ^BroadcastingErrorCallback) ---

    /// Reports an error code.
    @(link_name = "PxBroadcastingErrorCallback_reportError_mut")
    broadcasting_error_callback_report_error_mut :: proc(self_: ^BroadcastingErrorCallback, code: ErrorCode, message: cstring, file: cstring, line: _c.int32_t) ---

    /// Enables floating point exceptions for the scalar and SIMD unit
    @(link_name = "phys_PxEnableFPExceptions")
    enable_f_p_exceptions :: proc() ---

    /// Disables floating point exceptions for the scalar and SIMD unit
    @(link_name = "phys_PxDisableFPExceptions")
    disable_f_p_exceptions :: proc() ---

    /// read from the stream. The number of bytes read may be less than the number requested.
    ///
    /// the number of bytes read from the stream.
    @(link_name = "PxInputStream_read_mut")
    input_stream_read_mut :: proc(self_: ^InputStream, dest: rawptr, count: _c.uint32_t) -> _c.uint32_t ---

    @(link_name = "PxInputStream_delete")
    input_stream_delete :: proc(self_: ^InputStream) ---

    /// return the length of the input data
    ///
    /// size in bytes of the input data
    @(link_name = "PxInputData_getLength")
    input_data_get_length :: proc(self_: ^InputData) -> _c.uint32_t ---

    /// seek to the given offset from the start of the data.
    @(link_name = "PxInputData_seek_mut")
    input_data_seek_mut :: proc(self_: ^InputData, offset: _c.uint32_t) ---

    /// return the current offset from the start of the data
    ///
    /// the offset to seek to.
    @(link_name = "PxInputData_tell")
    input_data_tell :: proc(self_: ^InputData) -> _c.uint32_t ---

    @(link_name = "PxInputData_delete")
    input_data_delete :: proc(self_: ^InputData) ---

    /// write to the stream. The number of bytes written may be less than the number sent.
    ///
    /// the number of bytes written to the stream by this call.
    @(link_name = "PxOutputStream_write_mut")
    output_stream_write_mut :: proc(self_: ^OutputStream, src: rawptr, count: _c.uint32_t) -> _c.uint32_t ---

    @(link_name = "PxOutputStream_delete")
    output_stream_delete :: proc(self_: ^OutputStream) ---

    /// default constructor leaves data uninitialized.
    @(link_name = "PxVec4_new")
    vec4_new :: proc() -> Vec4 ---

    /// zero constructor.
    @(link_name = "PxVec4_new_1")
    vec4_new_1 :: proc(anon_param0: ZERO) -> Vec4 ---

    /// Assigns scalar parameter to all elements.
    ///
    /// Useful to initialize to zero or one.
    @(link_name = "PxVec4_new_2")
    vec4_new_2 :: proc(a: _c.float) -> Vec4 ---

    /// Initializes from 3 scalar parameters.
    @(link_name = "PxVec4_new_3")
    vec4_new_3 :: proc(nx: _c.float, ny: _c.float, nz: _c.float, nw: _c.float) -> Vec4 ---

    /// Initializes from 3 scalar parameters.
    @(link_name = "PxVec4_new_4")
    vec4_new_4 :: proc(#by_ptr v: Vec3, nw: _c.float) -> Vec4 ---

    /// Initializes from an array of scalar parameters.
    @(link_name = "PxVec4_new_5")
    vec4_new_5 :: proc(v: ^_c.float) -> Vec4 ---

    /// tests for exact zero vector
    @(link_name = "PxVec4_isZero")
    vec4_is_zero :: proc(self_: ^Vec4) -> _c.bool ---

    /// returns true if all 3 elems of the vector are finite (not NAN or INF, etc.)
    @(link_name = "PxVec4_isFinite")
    vec4_is_finite :: proc(self_: ^Vec4) -> _c.bool ---

    /// is normalized - used by API parameter validation
    @(link_name = "PxVec4_isNormalized")
    vec4_is_normalized :: proc(self_: ^Vec4) -> _c.bool ---

    /// returns the squared magnitude
    ///
    /// Avoids calling PxSqrt()!
    @(link_name = "PxVec4_magnitudeSquared")
    vec4_magnitude_squared :: proc(self_: ^Vec4) -> _c.float ---

    /// returns the magnitude
    @(link_name = "PxVec4_magnitude")
    vec4_magnitude :: proc(self_: ^Vec4) -> _c.float ---

    /// returns the scalar product of this and other.
    @(link_name = "PxVec4_dot")
    vec4_dot :: proc(self_: ^Vec4, #by_ptr v: Vec4) -> _c.float ---

    /// returns a unit vector
    @(link_name = "PxVec4_getNormalized")
    vec4_get_normalized :: proc(self_: ^Vec4) -> Vec4 ---

    /// normalizes the vector in place
    @(link_name = "PxVec4_normalize_mut")
    vec4_normalize_mut :: proc(self_: ^Vec4) -> _c.float ---

    /// a[i] * b[i], for all i.
    @(link_name = "PxVec4_multiply")
    vec4_multiply :: proc(self_: ^Vec4, #by_ptr a: Vec4) -> Vec4 ---

    /// element-wise minimum
    @(link_name = "PxVec4_minimum")
    vec4_minimum :: proc(self_: ^Vec4, #by_ptr v: Vec4) -> Vec4 ---

    /// element-wise maximum
    @(link_name = "PxVec4_maximum")
    vec4_maximum :: proc(self_: ^Vec4, #by_ptr v: Vec4) -> Vec4 ---

    @(link_name = "PxVec4_getXYZ")
    vec4_get_x_y_z :: proc(self_: ^Vec4) -> Vec3 ---

    /// Default constructor
    @(link_name = "PxMat44_new")
    mat44_new :: proc() -> Mat44 ---

    /// identity constructor
    @(link_name = "PxMat44_new_1")
    mat44_new_1 :: proc(anon_param0: IDENTITY) -> Mat44 ---

    /// zero constructor
    @(link_name = "PxMat44_new_2")
    mat44_new_2 :: proc(anon_param0: ZERO) -> Mat44 ---

    /// Construct from four 4-vectors
    @(link_name = "PxMat44_new_3")
    mat44_new_3 :: proc(#by_ptr col0: Vec4, #by_ptr col1: Vec4, #by_ptr col2: Vec4, #by_ptr col3: Vec4) -> Mat44 ---

    /// constructor that generates a multiple of the identity matrix
    @(link_name = "PxMat44_new_4")
    mat44_new_4 :: proc(r: _c.float) -> Mat44 ---

    /// Construct from three base vectors and a translation
    @(link_name = "PxMat44_new_5")
    mat44_new_5 :: proc(#by_ptr col0: Vec3, #by_ptr col1: Vec3, #by_ptr col2: Vec3, #by_ptr col3: Vec3) -> Mat44 ---

    /// Construct from float[16]
    @(link_name = "PxMat44_new_6")
    mat44_new_6 :: proc(values: ^_c.float) -> Mat44 ---

    /// Construct from a quaternion
    @(link_name = "PxMat44_new_7")
    mat44_new_7 :: proc(#by_ptr q: Quat) -> Mat44 ---

    /// Construct from a diagonal vector
    @(link_name = "PxMat44_new_8")
    mat44_new_8 :: proc(#by_ptr diagonal: Vec4) -> Mat44 ---

    /// Construct from Mat33 and a translation
    @(link_name = "PxMat44_new_9")
    mat44_new_9 :: proc(#by_ptr axes: Mat33, #by_ptr position: Vec3) -> Mat44 ---

    @(link_name = "PxMat44_new_10")
    mat44_new_10 :: proc(#by_ptr t: Transform) -> Mat44 ---

    /// Get transposed matrix
    @(link_name = "PxMat44_getTranspose")
    mat44_get_transpose :: proc(self_: ^Mat44) -> Mat44 ---

    /// Transform vector by matrix, equal to v' = M*v
    @(link_name = "PxMat44_transform")
    mat44_transform :: proc(self_: ^Mat44, #by_ptr other: Vec4) -> Vec4 ---

    /// Transform vector by matrix, equal to v' = M*v
    @(link_name = "PxMat44_transform_1")
    mat44_transform_1 :: proc(self_: ^Mat44, #by_ptr other: Vec3) -> Vec3 ---

    /// Rotate vector by matrix, equal to v' = M*v
    @(link_name = "PxMat44_rotate")
    mat44_rotate :: proc(self_: ^Mat44, #by_ptr other: Vec4) -> Vec4 ---

    /// Rotate vector by matrix, equal to v' = M*v
    @(link_name = "PxMat44_rotate_1")
    mat44_rotate_1 :: proc(self_: ^Mat44, #by_ptr other: Vec3) -> Vec3 ---

    @(link_name = "PxMat44_getBasis")
    mat44_get_basis :: proc(self_: ^Mat44, num: _c.uint32_t) -> Vec3 ---

    @(link_name = "PxMat44_getPosition")
    mat44_get_position :: proc(self_: ^Mat44) -> Vec3 ---

    @(link_name = "PxMat44_setPosition_mut")
    mat44_set_position_mut :: proc(self_: ^Mat44, #by_ptr position: Vec3) ---

    @(link_name = "PxMat44_front")
    mat44_front :: proc(self_: ^Mat44) -> ^_c.float ---

    @(link_name = "PxMat44_scale_mut")
    mat44_scale_mut :: proc(self_: ^Mat44, #by_ptr p: Vec4) ---

    @(link_name = "PxMat44_inverseRT")
    mat44_inverse_r_t :: proc(self_: ^Mat44) -> Mat44 ---

    @(link_name = "PxMat44_isFinite")
    mat44_is_finite :: proc(self_: ^Mat44) -> _c.bool ---

    /// Constructor
    @(link_name = "PxPlane_new")
    plane_new :: proc() -> Plane ---

    /// Constructor from a normal and a distance
    @(link_name = "PxPlane_new_1")
    plane_new_1 :: proc(nx: _c.float, ny: _c.float, nz: _c.float, distance: _c.float) -> Plane ---

    /// Constructor from a normal and a distance
    @(link_name = "PxPlane_new_2")
    plane_new_2 :: proc(#by_ptr normal: Vec3, distance: _c.float) -> Plane ---

    /// Constructor from a point on the plane and a normal
    @(link_name = "PxPlane_new_3")
    plane_new_3 :: proc(#by_ptr point: Vec3, #by_ptr normal: Vec3) -> Plane ---

    /// Constructor from three points
    @(link_name = "PxPlane_new_4")
    plane_new_4 :: proc(#by_ptr p0: Vec3, #by_ptr p1: Vec3, #by_ptr p2: Vec3) -> Plane ---

    @(link_name = "PxPlane_distance")
    plane_distance :: proc(self_: ^Plane, #by_ptr p: Vec3) -> _c.float ---

    @(link_name = "PxPlane_contains")
    plane_contains :: proc(self_: ^Plane, #by_ptr p: Vec3) -> _c.bool ---

    /// projects p into the plane
    @(link_name = "PxPlane_project")
    plane_project :: proc(self_: ^Plane, #by_ptr p: Vec3) -> Vec3 ---

    /// find an arbitrary point in the plane
    @(link_name = "PxPlane_pointInPlane")
    plane_point_in_plane :: proc(self_: ^Plane) -> Vec3 ---

    /// equivalent plane with unit normal
    @(link_name = "PxPlane_normalize_mut")
    plane_normalize_mut :: proc(self_: ^Plane) ---

    /// transform plane
    @(link_name = "PxPlane_transform")
    plane_transform :: proc(self_: ^Plane, #by_ptr pose: Transform) -> Plane ---

    /// inverse-transform plane
    @(link_name = "PxPlane_inverseTransform")
    plane_inverse_transform :: proc(self_: ^Plane, #by_ptr pose: Transform) -> Plane ---

    /// finds the shortest rotation between two vectors.
    ///
    /// a rotation about an axis normal to the two vectors which takes one to the other via the shortest path
    @(link_name = "phys_PxShortestRotation")
    shortest_rotation :: proc(#by_ptr from: Vec3, #by_ptr target: Vec3) -> Quat ---

    @(link_name = "phys_PxDiagonalize")
    diagonalize :: proc(#by_ptr m: Mat33, axes: ^Quat) -> Vec3 ---

    /// creates a transform from the endpoints of a segment, suitable for an actor transform for a PxCapsuleGeometry
    ///
    /// A PxTransform which will transform the vector (1,0,0) to the capsule axis shrunk by the halfHeight
    @(link_name = "phys_PxTransformFromSegment")
    transform_from_segment :: proc(#by_ptr p0: Vec3, #by_ptr p1: Vec3, halfHeight: ^_c.float) -> Transform ---

    /// creates a transform from a plane equation, suitable for an actor transform for a PxPlaneGeometry
    ///
    /// a PxTransform which will transform the plane PxPlane(1,0,0,0) to the specified plane
    @(link_name = "phys_PxTransformFromPlaneEquation")
    transform_from_plane_equation :: proc(#by_ptr plane: Plane) -> Transform ---

    /// creates a plane equation from a transform, such as the actor transform for a PxPlaneGeometry
    ///
    /// the plane
    @(link_name = "phys_PxPlaneEquationFromTransform")
    plane_equation_from_transform :: proc(#by_ptr pose: Transform) -> Plane ---

    /// Spherical linear interpolation of two quaternions.
    ///
    /// Returns left when t=0, right when t=1 and a linear interpolation of left and right when 0
    /// <
    /// t
    /// <
    /// 1.
    /// Returns angle between -PI and PI in radians
    @(link_name = "phys_PxSlerp")
    slerp :: proc(t: _c.float, #by_ptr left: Quat, #by_ptr right: Quat) -> Quat ---

    /// integrate transform.
    @(link_name = "phys_PxIntegrateTransform")
    integrate_transform :: proc(#by_ptr curTrans: Transform, #by_ptr linvel: Vec3, #by_ptr angvel: Vec3, timeStep: _c.float, result: ^Transform) ---

    /// Compute the exponent of a PxVec3
    @(link_name = "phys_PxExp")
    exp :: proc(#by_ptr v: Vec3) -> Quat ---

    /// computes a oriented bounding box around the scaled basis.
    ///
    /// Bounding box extent.
    @(link_name = "phys_PxOptimizeBoundingBox")
    optimize_bounding_box :: proc(basis: ^Mat33) -> Vec3 ---

    /// return Returns the log of a PxQuat
    @(link_name = "phys_PxLog")
    quat_log :: proc(#by_ptr q: Quat) -> Vec3 ---

    /// return Returns 0 if v.x is largest element of v, 1 if v.y is largest element, 2 if v.z is largest element.
    @(link_name = "phys_PxLargestAxis")
    largest_axis :: proc(#by_ptr v: Vec3) -> _c.uint32_t ---

    /// Compute tan(theta/2) given sin(theta) and cos(theta) as inputs.
    ///
    /// Returns tan(theta/2)
    @(link_name = "phys_PxTanHalf")
    tan_half :: proc(sin: _c.float, cos: _c.float) -> _c.float ---

    /// Compute the closest point on an 2d ellipse to a given 2d point.
    ///
    /// Returns the 2d position on the surface of the ellipse that is closest to point.
    @(link_name = "phys_PxEllipseClamp")
    ellipse_clamp :: proc(#by_ptr point: Vec3, #by_ptr radii: Vec3) -> Vec3 ---

    /// Compute from an input quaternion q a pair of quaternions (swing, twist) such that
    /// q = swing * twist
    /// with the caveats that swing.x = twist.y = twist.z = 0.
    @(link_name = "phys_PxSeparateSwingTwist")
    separate_swing_twist :: proc(#by_ptr q: Quat, swing: ^Quat, twist: ^Quat) ---

    /// Compute the angle between two non-unit vectors
    ///
    /// Returns the angle (in radians) between the two vector v0 and v1.
    @(link_name = "phys_PxComputeAngle")
    compute_angle :: proc(#by_ptr v0: Vec3, #by_ptr v1: Vec3) -> _c.float ---

    /// Compute two normalized vectors (right and up) that are perpendicular to an input normalized vector (dir).
    @(link_name = "phys_PxComputeBasisVectors")
    compute_basis_vectors :: proc(#by_ptr dir: Vec3, right: ^Vec3, up: ^Vec3) ---

    /// Compute three normalized vectors (dir, right and up) that are parallel to (dir) and perpendicular to (right, up) the
    /// normalized direction vector (p1 - p0)/||p1 - p0||.
    @(link_name = "phys_PxComputeBasisVectors_1")
    compute_basis_vectors_1 :: proc(#by_ptr p0: Vec3, #by_ptr p1: Vec3, dir: ^Vec3, right: ^Vec3, up: ^Vec3) ---

    /// Compute (i+1)%3
    @(link_name = "phys_PxGetNextIndex3")
    get_next_index3 :: proc(i: _c.uint32_t) -> _c.uint32_t ---

    @(link_name = "phys_computeBarycentric")
    compute_barycentric :: proc(#by_ptr a: Vec3, #by_ptr b: Vec3, #by_ptr c: Vec3, #by_ptr d: Vec3, #by_ptr p: Vec3, bary: ^Vec4) ---

    @(link_name = "phys_computeBarycentric_1")
    compute_barycentric_1 :: proc(#by_ptr a: Vec3, #by_ptr b: Vec3, #by_ptr c: Vec3, #by_ptr p: Vec3, bary: ^Vec4) ---

    @(link_name = "Interpolation_PxLerp")
    lerp :: proc(a: _c.float, b: _c.float, t: _c.float) -> _c.float ---

    @(link_name = "Interpolation_PxBiLerp")
    bi_lerp :: proc(f00: _c.float, f10: _c.float, f01: _c.float, f11: _c.float, tx: _c.float, ty: _c.float) -> _c.float ---

    @(link_name = "Interpolation_PxTriLerp")
    tri_lerp :: proc(f000: _c.float, f100: _c.float, f010: _c.float, f110: _c.float, f001: _c.float, f101: _c.float, f011: _c.float, f111: _c.float, tx: _c.float, ty: _c.float, tz: _c.float) -> _c.float ---

    @(link_name = "Interpolation_PxSDFIdx")
    s_d_f_idx :: proc(i: _c.uint32_t, j: _c.uint32_t, k: _c.uint32_t, nbX: _c.uint32_t, nbY: _c.uint32_t) -> _c.uint32_t ---

    @(link_name = "Interpolation_PxSDFSampleImpl")
    s_d_f_sample_impl :: proc(sdf: ^_c.float, #by_ptr localPos: Vec3, #by_ptr sdfBoxLower: Vec3, #by_ptr sdfBoxHigher: Vec3, sdfDx: _c.float, invSdfDx: _c.float, dimX: _c.uint32_t, dimY: _c.uint32_t, dimZ: _c.uint32_t, tolerance: _c.float) -> _c.float ---

    @(link_name = "phys_PxSdfSample")
    sdf_sample :: proc(sdf: ^_c.float, #by_ptr localPos: Vec3, #by_ptr sdfBoxLower: Vec3, #by_ptr sdfBoxHigher: Vec3, sdfDx: _c.float, invSdfDx: _c.float, dimX: _c.uint32_t, dimY: _c.uint32_t, dimZ: _c.uint32_t, gradient: ^Vec3, tolerance: _c.float) -> _c.float ---

    /// The constructor for Mutex creates a mutex. It is initially unlocked.
    @(link_name = "PxMutexImpl_new_alloc")
    mutex_impl_new_alloc :: proc() -> ^MutexImpl ---

    /// The destructor for Mutex deletes the mutex.
    @(link_name = "PxMutexImpl_delete")
    mutex_impl_delete :: proc(self_: ^MutexImpl) ---

    /// Acquire (lock) the mutex. If the mutex is already locked
    /// by another thread, this method blocks until the mutex is
    /// unlocked.
    @(link_name = "PxMutexImpl_lock_mut")
    mutex_impl_lock_mut :: proc(self_: ^MutexImpl) ---

    /// Acquire (lock) the mutex. If the mutex is already locked
    /// by another thread, this method returns false without blocking.
    @(link_name = "PxMutexImpl_trylock_mut")
    mutex_impl_trylock_mut :: proc(self_: ^MutexImpl) -> _c.bool ---

    /// Release (unlock) the mutex.
    @(link_name = "PxMutexImpl_unlock_mut")
    mutex_impl_unlock_mut :: proc(self_: ^MutexImpl) ---

    /// Size of this class.
    @(link_name = "PxMutexImpl_getSize")
    mutex_impl_get_size :: proc() -> _c.uint32_t ---

    @(link_name = "PxReadWriteLock_new_alloc")
    read_write_lock_new_alloc :: proc() -> ^ReadWriteLock ---

    @(link_name = "PxReadWriteLock_delete")
    read_write_lock_delete :: proc(self_: ^ReadWriteLock) ---

    @(link_name = "PxReadWriteLock_lockReader_mut")
    read_write_lock_lock_reader_mut :: proc(self_: ^ReadWriteLock, takeLock: _c.bool) ---

    @(link_name = "PxReadWriteLock_lockWriter_mut")
    read_write_lock_lock_writer_mut :: proc(self_: ^ReadWriteLock) ---

    @(link_name = "PxReadWriteLock_unlockReader_mut")
    read_write_lock_unlock_reader_mut :: proc(self_: ^ReadWriteLock) ---

    @(link_name = "PxReadWriteLock_unlockWriter_mut")
    read_write_lock_unlock_writer_mut :: proc(self_: ^ReadWriteLock) ---

    /// Mark the beginning of a nested profile block
    ///
    /// Returns implementation-specific profiler data for this event
    @(link_name = "PxProfilerCallback_zoneStart_mut")
    profiler_callback_zone_start_mut :: proc(self_: ^ProfilerCallback, eventName: cstring, detached: _c.bool, contextId: _c.uint64_t) -> rawptr ---

    /// Mark the end of a nested profile block
    ///
    /// eventName plus contextId can be used to uniquely match up start and end of a zone.
    @(link_name = "PxProfilerCallback_zoneEnd_mut")
    profiler_callback_zone_end_mut :: proc(self_: ^ProfilerCallback, profilerData: rawptr, eventName: cstring, detached: _c.bool, contextId: _c.uint64_t) ---

    @(link_name = "PxProfileScoped_new_alloc")
    profile_scoped_new_alloc :: proc(callback: ^ProfilerCallback, eventName: cstring, detached: _c.bool, contextId: _c.uint64_t) -> ^ProfileScoped ---

    @(link_name = "PxProfileScoped_delete")
    profile_scoped_delete :: proc(self_: ^ProfileScoped) ---

    @(link_name = "PxSListEntry_new")
    s_list_entry_new :: proc() -> SListEntry ---

    @(link_name = "PxSListEntry_next_mut")
    s_list_entry_next_mut :: proc(self_: ^SListEntry) -> ^SListEntry ---

    @(link_name = "PxSListImpl_new_alloc")
    s_list_impl_new_alloc :: proc() -> ^SListImpl ---

    @(link_name = "PxSListImpl_delete")
    s_list_impl_delete :: proc(self_: ^SListImpl) ---

    @(link_name = "PxSListImpl_push_mut")
    s_list_impl_push_mut :: proc(self_: ^SListImpl, entry: ^SListEntry) ---

    @(link_name = "PxSListImpl_pop_mut")
    s_list_impl_pop_mut :: proc(self_: ^SListImpl) -> ^SListEntry ---

    @(link_name = "PxSListImpl_flush_mut")
    s_list_impl_flush_mut :: proc(self_: ^SListImpl) -> ^SListEntry ---

    @(link_name = "PxSListImpl_getSize")
    s_list_impl_get_size :: proc() -> _c.uint32_t ---

    @(link_name = "PxSyncImpl_new_alloc")
    sync_impl_new_alloc :: proc() -> ^SyncImpl ---

    @(link_name = "PxSyncImpl_delete")
    sync_impl_delete :: proc(self_: ^SyncImpl) ---

    /// Wait on the object for at most the given number of ms. Returns
    /// true if the object is signaled. Sync::waitForever will block forever
    /// or until the object is signaled.
    @(link_name = "PxSyncImpl_wait_mut")
    sync_impl_wait_mut :: proc(self_: ^SyncImpl, milliseconds: _c.uint32_t) -> _c.bool ---

    /// Signal the synchronization object, waking all threads waiting on it
    @(link_name = "PxSyncImpl_set_mut")
    sync_impl_set_mut :: proc(self_: ^SyncImpl) ---

    /// Reset the synchronization object
    @(link_name = "PxSyncImpl_reset_mut")
    sync_impl_reset_mut :: proc(self_: ^SyncImpl) ---

    /// Size of this class.
    @(link_name = "PxSyncImpl_getSize")
    sync_impl_get_size :: proc() -> _c.uint32_t ---

    @(link_name = "PxRunnable_new_alloc")
    runnable_new_alloc :: proc() -> ^Runnable ---

    @(link_name = "PxRunnable_delete")
    runnable_delete :: proc(self_: ^Runnable) ---

    @(link_name = "PxRunnable_execute_mut")
    runnable_execute_mut :: proc(self_: ^Runnable) ---

    @(link_name = "phys_PxTlsAlloc")
    tls_alloc :: proc() -> _c.uint32_t ---

    @(link_name = "phys_PxTlsFree")
    tls_free :: proc(index: _c.uint32_t) ---

    @(link_name = "phys_PxTlsGet")
    tls_get :: proc(index: _c.uint32_t) -> rawptr ---

    @(link_name = "phys_PxTlsGetValue")
    tls_get_value :: proc(index: _c.uint32_t) -> _c.size_t ---

    @(link_name = "phys_PxTlsSet")
    tls_set :: proc(index: _c.uint32_t, value: rawptr) -> _c.uint32_t ---

    @(link_name = "phys_PxTlsSetValue")
    tls_set_value :: proc(index: _c.uint32_t, value: _c.size_t) -> _c.uint32_t ---

    @(link_name = "PxCounterFrequencyToTensOfNanos_new")
    counter_frequency_to_tens_of_nanos_new :: proc(inNum: _c.uint64_t, inDenom: _c.uint64_t) -> CounterFrequencyToTensOfNanos ---

    @(link_name = "PxCounterFrequencyToTensOfNanos_toTensOfNanos")
    counter_frequency_to_tens_of_nanos_to_tens_of_nanos :: proc(self_: ^CounterFrequencyToTensOfNanos, inCounter: _c.uint64_t) -> _c.uint64_t ---

    @(link_name = "PxTime_getBootCounterFrequency")
    time_get_boot_counter_frequency :: proc() -> ^CounterFrequencyToTensOfNanos ---

    @(link_name = "PxTime_getCounterFrequency")
    time_get_counter_frequency :: proc() -> CounterFrequencyToTensOfNanos ---

    @(link_name = "PxTime_getCurrentCounterValue")
    time_get_current_counter_value :: proc() -> _c.uint64_t ---

    @(link_name = "PxTime_getCurrentTimeInTensOfNanoSeconds")
    time_get_current_time_in_tens_of_nano_seconds :: proc() -> _c.uint64_t ---

    @(link_name = "PxTime_new")
    time_new :: proc() -> Time ---

    @(link_name = "PxTime_getElapsedSeconds_mut")
    time_get_elapsed_seconds_mut :: proc(self_: ^Time) -> _c.double ---

    @(link_name = "PxTime_peekElapsedSeconds_mut")
    time_peek_elapsed_seconds_mut :: proc(self_: ^Time) -> _c.double ---

    @(link_name = "PxTime_getLastTime")
    time_get_last_time :: proc(self_: ^Time) -> _c.double ---

    /// default constructor leaves data uninitialized.
    @(link_name = "PxVec2_new")
    vec2_new :: proc() -> Vec2 ---

    /// zero constructor.
    @(link_name = "PxVec2_new_1")
    vec2_new_1 :: proc(anon_param0: ZERO) -> Vec2 ---

    /// Assigns scalar parameter to all elements.
    ///
    /// Useful to initialize to zero or one.
    @(link_name = "PxVec2_new_2")
    vec2_new_2 :: proc(a: _c.float) -> Vec2 ---

    /// Initializes from 2 scalar parameters.
    @(link_name = "PxVec2_new_3")
    vec2_new_3 :: proc(nx: _c.float, ny: _c.float) -> Vec2 ---

    /// tests for exact zero vector
    @(link_name = "PxVec2_isZero")
    vec2_is_zero :: proc(self_: ^Vec2) -> _c.bool ---

    /// returns true if all 2 elems of the vector are finite (not NAN or INF, etc.)
    @(link_name = "PxVec2_isFinite")
    vec2_is_finite :: proc(self_: ^Vec2) -> _c.bool ---

    /// is normalized - used by API parameter validation
    @(link_name = "PxVec2_isNormalized")
    vec2_is_normalized :: proc(self_: ^Vec2) -> _c.bool ---

    /// returns the squared magnitude
    ///
    /// Avoids calling PxSqrt()!
    @(link_name = "PxVec2_magnitudeSquared")
    vec2_magnitude_squared :: proc(self_: ^Vec2) -> _c.float ---

    /// returns the magnitude
    @(link_name = "PxVec2_magnitude")
    vec2_magnitude :: proc(self_: ^Vec2) -> _c.float ---

    /// returns the scalar product of this and other.
    @(link_name = "PxVec2_dot")
    vec2_dot :: proc(self_: ^Vec2, #by_ptr v: Vec2) -> _c.float ---

    /// returns a unit vector
    @(link_name = "PxVec2_getNormalized")
    vec2_get_normalized :: proc(self_: ^Vec2) -> Vec2 ---

    /// normalizes the vector in place
    @(link_name = "PxVec2_normalize_mut")
    vec2_normalize_mut :: proc(self_: ^Vec2) -> _c.float ---

    /// a[i] * b[i], for all i.
    @(link_name = "PxVec2_multiply")
    vec2_multiply :: proc(self_: ^Vec2, #by_ptr a: Vec2) -> Vec2 ---

    /// element-wise minimum
    @(link_name = "PxVec2_minimum")
    vec2_minimum :: proc(self_: ^Vec2, #by_ptr v: Vec2) -> Vec2 ---

    /// returns MIN(x, y);
    @(link_name = "PxVec2_minElement")
    vec2_min_element :: proc(self_: ^Vec2) -> _c.float ---

    /// element-wise maximum
    @(link_name = "PxVec2_maximum")
    vec2_maximum :: proc(self_: ^Vec2, #by_ptr v: Vec2) -> Vec2 ---

    /// returns MAX(x, y);
    @(link_name = "PxVec2_maxElement")
    vec2_max_element :: proc(self_: ^Vec2) -> _c.float ---

    @(link_name = "PxStridedData_new")
    strided_data_new :: proc() -> StridedData ---

    @(link_name = "PxBoundedData_new")
    bounded_data_new :: proc() -> BoundedData ---

    @(link_name = "PxDebugPoint_new")
    debug_point_new :: proc(#by_ptr p: Vec3, #by_ptr c: _c.uint32_t) -> DebugPoint ---

    @(link_name = "PxDebugLine_new")
    debug_line_new :: proc(#by_ptr p0: Vec3, #by_ptr p1: Vec3, #by_ptr c: _c.uint32_t) -> DebugLine ---

    @(link_name = "PxDebugTriangle_new")
    debug_triangle_new :: proc(#by_ptr p0: Vec3, #by_ptr p1: Vec3, #by_ptr p2: Vec3, #by_ptr c: _c.uint32_t) -> DebugTriangle ---

    @(link_name = "PxDebugText_new")
    debug_text_new :: proc() -> DebugText ---

    @(link_name = "PxDebugText_new_1")
    debug_text_new_1 :: proc(#by_ptr pos: Vec3, #by_ptr sz: _c.float, #by_ptr clr: _c.uint32_t, str: cstring) -> DebugText ---

    @(link_name = "PxRenderBuffer_delete")
    render_buffer_delete :: proc(self_: ^RenderBuffer) ---

    @(link_name = "PxRenderBuffer_getNbPoints")
    render_buffer_get_nb_points :: proc(self_: ^RenderBuffer) -> _c.uint32_t ---

    @(link_name = "PxRenderBuffer_getPoints")
    render_buffer_get_points :: proc(self_: ^RenderBuffer) -> [^]DebugPoint ---

    @(link_name = "PxRenderBuffer_addPoint_mut")
    render_buffer_add_point_mut :: proc(self_: ^RenderBuffer, #by_ptr point: DebugPoint) ---

    @(link_name = "PxRenderBuffer_getNbLines")
    render_buffer_get_nb_lines :: proc(self_: ^RenderBuffer) -> _c.uint32_t ---

    @(link_name = "PxRenderBuffer_getLines")
    render_buffer_get_lines :: proc(self_: ^RenderBuffer) -> [^]DebugLine ---

    @(link_name = "PxRenderBuffer_addLine_mut")
    render_buffer_add_line_mut :: proc(self_: ^RenderBuffer, #by_ptr line: DebugLine) ---

    @(link_name = "PxRenderBuffer_reserveLines_mut")
    render_buffer_reserve_lines_mut :: proc(self_: ^RenderBuffer, nbLines: _c.uint32_t) -> ^DebugLine ---

    @(link_name = "PxRenderBuffer_reservePoints_mut")
    render_buffer_reserve_points_mut :: proc(self_: ^RenderBuffer, nbLines: _c.uint32_t) -> ^DebugPoint ---

    @(link_name = "PxRenderBuffer_getNbTriangles")
    render_buffer_get_nb_triangles :: proc(self_: ^RenderBuffer) -> _c.uint32_t ---

    @(link_name = "PxRenderBuffer_getTriangles")
    render_buffer_get_triangles :: proc(self_: ^RenderBuffer) -> [^]DebugTriangle ---

    @(link_name = "PxRenderBuffer_addTriangle_mut")
    render_buffer_add_triangle_mut :: proc(self_: ^RenderBuffer, #by_ptr triangle: DebugTriangle) ---

    @(link_name = "PxRenderBuffer_append_mut")
    render_buffer_append_mut :: proc(self_: ^RenderBuffer, #by_ptr other: RenderBuffer) ---

    @(link_name = "PxRenderBuffer_clear_mut")
    render_buffer_clear_mut :: proc(self_: ^RenderBuffer) ---

    @(link_name = "PxRenderBuffer_shift_mut")
    render_buffer_shift_mut :: proc(self_: ^RenderBuffer, #by_ptr delta: Vec3) ---

    @(link_name = "PxRenderBuffer_empty")
    render_buffer_empty :: proc(self_: ^RenderBuffer) -> _c.bool ---

    @(link_name = "PxProcessPxBaseCallback_delete")
    process_px_base_callback_delete :: proc(self_: ^ProcessPxBaseCallback) ---

    @(link_name = "PxProcessPxBaseCallback_process_mut")
    process_px_base_callback_process_mut :: proc(self_: ^ProcessPxBaseCallback, anon_param0: ^Base) ---

    /// Registers a reference value corresponding to a PxBase object.
    ///
    /// This method is assumed to be called in the implementation of PxSerializer::registerReferences for serialized
    /// references that need to be resolved on deserialization.
    ///
    /// A reference needs to be associated with exactly one PxBase object in either the collection or the
    /// external references collection.
    ///
    /// Different kinds of references are supported and need to be specified. In the most common case
    /// (PX_SERIAL_REF_KIND_PXBASE) the PxBase object matches the reference value (which is the pointer
    /// to the PxBase object). Integer references maybe registered as well (used for internal material
    /// indices with PX_SERIAL_REF_KIND_MATERIAL_IDX). Other kinds could be added with the restriction that
    /// for pointer types the kind value needs to be marked with the PX_SERIAL_REF_KIND_PTR_TYPE_BIT.
    @(link_name = "PxSerializationContext_registerReference_mut")
    serialization_context_register_reference_mut :: proc(self_: ^SerializationContext, base: ^Base, kind: _c.uint32_t, reference: _c.size_t) ---

    /// Returns the collection that is being serialized.
    @(link_name = "PxSerializationContext_getCollection")
    serialization_context_get_collection :: proc(self_: ^SerializationContext) -> ^Collection ---

    /// Serializes object data and object extra data.
    ///
    /// This function is assumed to be called within the implementation of PxSerializer::exportData and PxSerializer::exportExtraData.
    @(link_name = "PxSerializationContext_writeData_mut")
    serialization_context_write_data_mut :: proc(self_: ^SerializationContext, data: rawptr, size: _c.uint32_t) ---

    /// Aligns the serialized data.
    ///
    /// This function is assumed to be called within the implementation of PxSerializer::exportData and PxSerializer::exportExtraData.
    @(link_name = "PxSerializationContext_alignData_mut")
    serialization_context_align_data_mut :: proc(self_: ^SerializationContext, alignment: _c.uint32_t) ---

    /// Helper function to write a name to the extraData if serialization is configured to save names.
    ///
    /// This function is assumed to be called within the implementation of PxSerializer::exportExtraData.
    @(link_name = "PxSerializationContext_writeName_mut")
    serialization_context_write_name_mut :: proc(self_: ^SerializationContext, name: cstring) ---

    /// Retrieves a pointer to a deserialized PxBase object given a corresponding deserialized reference value
    ///
    /// This method is assumed to be called in the implementation of PxSerializer::createObject in order
    /// to update reference values on deserialization.
    ///
    /// To update a PxBase reference the corresponding deserialized pointer value needs to be provided in order to retrieve
    /// the location of the corresponding deserialized PxBase object. (PxDeserializationContext::translatePxBase simplifies
    /// this common case).
    ///
    /// For other kinds of references the reverence values need to be updated by deduction given the corresponding PxBase instance.
    ///
    /// PxBase object associated with the reference value
    @(link_name = "PxDeserializationContext_resolveReference")
    deserialization_context_resolve_reference :: proc(self_: ^DeserializationContext, kind: _c.uint32_t, reference: _c.size_t) -> ^Base ---

    /// Helper function to read a name from the extra data during deserialization.
    ///
    /// This function is assumed to be called within the implementation of PxSerializer::createObject.
    @(link_name = "PxDeserializationContext_readName_mut")
    deserialization_context_read_name_mut :: proc(self_: ^DeserializationContext, name: ^cstring) ---

    /// Function to align the extra data stream to a power of 2 alignment
    ///
    /// This function is assumed to be called within the implementation of PxSerializer::createObject.
    @(link_name = "PxDeserializationContext_alignExtraData_mut")
    deserialization_context_align_extra_data_mut :: proc(self_: ^DeserializationContext, alignment: _c.uint32_t) ---

    /// Register a serializer for a concrete type
    @(link_name = "PxSerializationRegistry_registerSerializer_mut")
    serialization_registry_register_serializer_mut :: proc(self_: ^SerializationRegistry, type: _c.uint16_t, serializer: ^Serializer) ---

    /// Unregister a serializer for a concrete type, and retrieves the corresponding serializer object.
    ///
    /// Unregistered serializer corresponding to type, NULL for types for which no serializer has been registered.
    @(link_name = "PxSerializationRegistry_unregisterSerializer_mut")
    serialization_registry_unregister_serializer_mut :: proc(self_: ^SerializationRegistry, type: _c.uint16_t) -> ^Serializer ---

    /// Returns PxSerializer corresponding to type
    ///
    /// Registered PxSerializer object corresponding to type
    @(link_name = "PxSerializationRegistry_getSerializer")
    serialization_registry_get_serializer :: proc(self_: ^SerializationRegistry, type: _c.uint16_t) -> ^Serializer ---

    /// Register a RepX serializer for a concrete type
    @(link_name = "PxSerializationRegistry_registerRepXSerializer_mut")
    serialization_registry_register_rep_x_serializer_mut :: proc(self_: ^SerializationRegistry, type: _c.uint16_t, serializer: ^RepXSerializer) ---

    /// Unregister a RepX serializer for a concrete type, and retrieves the corresponding serializer object.
    ///
    /// Unregistered PxRepxSerializer corresponding to type, NULL for types for which no RepX serializer has been registered.
    @(link_name = "PxSerializationRegistry_unregisterRepXSerializer_mut")
    serialization_registry_unregister_rep_x_serializer_mut :: proc(self_: ^SerializationRegistry, type: _c.uint16_t) -> ^RepXSerializer ---

    /// Returns RepX serializer given the corresponding type name
    ///
    /// Registered PxRepXSerializer object corresponding to type name
    @(link_name = "PxSerializationRegistry_getRepXSerializer")
    serialization_registry_get_rep_x_serializer :: proc(self_: ^SerializationRegistry, typeName: cstring) -> ^RepXSerializer ---

    /// Releases PxSerializationRegistry instance.
    ///
    /// This unregisters all PhysX and PhysXExtension serializers. Make sure to unregister all custom type
    /// serializers before releasing the PxSerializationRegistry.
    @(link_name = "PxSerializationRegistry_release_mut")
    serialization_registry_release_mut :: proc(self_: ^SerializationRegistry) ---

    /// Adds a PxBase object to the collection.
    ///
    /// Adds a PxBase object to the collection. Optionally a PxSerialObjectId can be provided
    /// in order to resolve dependencies between collections. A PxSerialObjectId value of PX_SERIAL_OBJECT_ID_INVALID
    /// means the object remains without id. Objects can be added regardless of other objects they require. If the object
    /// is already in the collection, the ID will be set if it was PX_SERIAL_OBJECT_ID_INVALID previously, otherwise the
    /// operation fails.
    @(link_name = "PxCollection_add_mut")
    collection_add_mut :: proc(self_: ^Collection, object: ^Base, id: _c.uint64_t) ---

    /// Removes a PxBase member object from the collection.
    ///
    /// Object needs to be contained by the collection.
    @(link_name = "PxCollection_remove_mut")
    collection_remove_mut :: proc(self_: ^Collection, object: ^Base) ---

    /// Returns whether the collection contains a certain PxBase object.
    ///
    /// Whether object is contained.
    @(link_name = "PxCollection_contains")
    collection_contains :: proc(self_: ^Collection, object: ^Base) -> _c.bool ---

    /// Adds an id to a member PxBase object.
    ///
    /// If the object is already associated with an id within the collection, the id is replaced.
    /// May only be called for objects that are members of the collection. The id needs to be unique
    /// within the collection.
    @(link_name = "PxCollection_addId_mut")
    collection_add_id_mut :: proc(self_: ^Collection, object: ^Base, id: _c.uint64_t) ---

    /// Removes id from a contained PxBase object.
    ///
    /// May only be called for ids that are associated with an object in the collection.
    @(link_name = "PxCollection_removeId_mut")
    collection_remove_id_mut :: proc(self_: ^Collection, id: _c.uint64_t) ---

    /// Adds all PxBase objects and their ids of collection to this collection.
    ///
    /// PxBase objects already in this collection are ignored. Object ids need to be conflict
    /// free, i.e. the same object may not have two different ids within the two collections.
    @(link_name = "PxCollection_add_mut_1")
    collection_add_mut_1 :: proc(self_: ^Collection, collection: ^Collection) ---

    /// Removes all PxBase objects of collection from this collection.
    ///
    /// PxBase objects not present in this collection are ignored. Ids of objects
    /// which are removed are also removed.
    @(link_name = "PxCollection_remove_mut_1")
    collection_remove_mut_1 :: proc(self_: ^Collection, collection: ^Collection) ---

    /// Gets number of PxBase objects in this collection.
    ///
    /// Number of objects in this collection
    @(link_name = "PxCollection_getNbObjects")
    collection_get_nb_objects :: proc(self_: ^Collection) -> _c.uint32_t ---

    /// Gets the PxBase object of this collection given its index.
    ///
    /// PxBase object at index index
    @(link_name = "PxCollection_getObject")
    collection_get_object :: proc(self_: ^Collection, index: _c.uint32_t) -> ^Base ---

    /// Copies member PxBase pointers to a user specified buffer.
    ///
    /// number of members PxBase objects that have been written to the userBuffer
    @(link_name = "PxCollection_getObjects")
    collection_get_objects :: proc(self_: ^Collection, userBuffer: [^]^Base, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Looks for a PxBase object given a PxSerialObjectId value.
    ///
    /// If there is no PxBase object in the collection with the given id, NULL is returned.
    ///
    /// PxBase object with the given id value or NULL
    @(link_name = "PxCollection_find")
    collection_find :: proc(self_: ^Collection, id: _c.uint64_t) -> ^Base ---

    /// Gets number of PxSerialObjectId names in this collection.
    ///
    /// Number of PxSerialObjectId names in this collection
    @(link_name = "PxCollection_getNbIds")
    collection_get_nb_ids :: proc(self_: ^Collection) -> _c.uint32_t ---

    /// Copies member PxSerialObjectId values to a user specified buffer.
    ///
    /// number of members PxSerialObjectId values that have been written to the userBuffer
    @(link_name = "PxCollection_getIds")
    collection_get_ids :: proc(self_: ^Collection, userBuffer: ^_c.uint64_t, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Gets the PxSerialObjectId name of a PxBase object within the collection.
    ///
    /// The PxBase object needs to be a member of the collection.
    ///
    /// PxSerialObjectId name of the object or PX_SERIAL_OBJECT_ID_INVALID if the object is unnamed
    @(link_name = "PxCollection_getId")
    collection_get_id :: proc(self_: ^Collection, object: ^Base) -> _c.uint64_t ---

    /// Deletes a collection object.
    ///
    /// This function only deletes the collection object, i.e. the container class. It doesn't delete objects
    /// that are part of the collection.
    @(link_name = "PxCollection_release_mut")
    collection_release_mut :: proc(self_: ^Collection) ---

    /// Creates a collection object.
    ///
    /// Objects can only be serialized or deserialized through a collection.
    /// For serialization, users must add objects to the collection and serialize the collection as a whole.
    /// For deserialization, the system gives back a collection of deserialized objects to users.
    ///
    /// The new collection object.
    @(link_name = "phys_PxCreateCollection")
    create_collection :: proc() -> ^Collection ---

    /// Releases the PxBase instance, please check documentation of release in derived class.
    @(link_name = "PxBase_release_mut")
    base_release_mut :: proc(self_: ^Base) ---

    /// Returns string name of dynamic type.
    ///
    /// Class name of most derived type of this object.
    @(link_name = "PxBase_getConcreteTypeName")
    base_get_concrete_type_name :: proc(self_: ^Base) -> cstring ---

    /// Returns concrete type of object.
    ///
    /// PxConcreteType::Enum of serialized object
    @(link_name = "PxBase_getConcreteType")
    base_get_concrete_type :: proc(self_: ^Base) -> _c.uint16_t ---

    /// Set PxBaseFlag
    @(link_name = "PxBase_setBaseFlag_mut")
    base_set_base_flag_mut :: proc(self_: ^Base, flag: BaseFlag, value: _c.bool) ---

    /// Set PxBaseFlags
    @(link_name = "PxBase_setBaseFlags_mut")
    base_set_base_flags_mut :: proc(self_: ^Base, inFlags: BaseFlags_Set) ---

    /// Returns PxBaseFlags
    ///
    /// PxBaseFlags
    @(link_name = "PxBase_getBaseFlags")
    base_get_base_flags :: proc(self_: ^Base) -> BaseFlags_Set ---

    /// Whether the object is subordinate.
    ///
    /// A class is subordinate, if it can only be instantiated in the context of another class.
    ///
    /// Whether the class is subordinate
    @(link_name = "PxBase_isReleasable")
    base_is_releasable :: proc(self_: ^Base) -> _c.bool ---

    /// Decrements the reference count of the object and releases it if the new reference count is zero.
    @(link_name = "PxRefCounted_release_mut")
    ref_counted_release_mut :: proc(self_: ^RefCounted) ---

    /// Returns the reference count of the object.
    ///
    /// At creation, the reference count of the object is 1. Every other object referencing this object increments the
    /// count by 1. When the reference count reaches 0, and only then, the object gets destroyed automatically.
    ///
    /// the current reference count.
    @(link_name = "PxRefCounted_getReferenceCount")
    ref_counted_get_reference_count :: proc(self_: ^RefCounted) -> _c.uint32_t ---

    /// Acquires a counted reference to this object.
    ///
    /// This method increases the reference count of the object by 1. Decrement the reference count by calling release()
    @(link_name = "PxRefCounted_acquireReference_mut")
    ref_counted_acquire_reference_mut :: proc(self_: ^RefCounted) ---

    /// constructor sets to default
    @(link_name = "PxTolerancesScale_new")
    tolerances_scale_new :: proc(defaultLength: _c.float, defaultSpeed: _c.float) -> TolerancesScale ---

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid (returns always true).
    @(link_name = "PxTolerancesScale_isValid")
    tolerances_scale_is_valid :: proc(self_: ^TolerancesScale) -> _c.bool ---

    /// Allocate a new string.
    ///
    /// *Always* a valid null terminated string.  "" is returned if "" or null is passed in.
    @(link_name = "PxStringTable_allocateStr_mut")
    string_table_allocate_str_mut :: proc(self_: ^StringTable, inSrc: cstring) -> cstring ---

    /// Release the string table and all the strings associated with it.
    @(link_name = "PxStringTable_release_mut")
    string_table_release_mut :: proc(self_: ^StringTable) ---

    /// Returns string name of dynamic type.
    ///
    /// Class name of most derived type of this object.
    @(link_name = "PxSerializer_getConcreteTypeName")
    serializer_get_concrete_type_name :: proc(self_: ^Serializer) -> cstring ---

    /// Adds required objects to the collection.
    ///
    /// This method does not add the required objects recursively, e.g. objects required by required objects.
    @(link_name = "PxSerializer_requiresObjects")
    serializer_requires_objects :: proc(self_: ^Serializer, anon_param0: ^Base, anon_param1: ^ProcessPxBaseCallback) ---

    /// Whether the object is subordinate.
    ///
    /// A class is subordinate, if it can only be instantiated in the context of another class.
    ///
    /// Whether the class is subordinate
    @(link_name = "PxSerializer_isSubordinate")
    serializer_is_subordinate :: proc(self_: ^Serializer) -> _c.bool ---

    /// Exports object's extra data to stream.
    @(link_name = "PxSerializer_exportExtraData")
    serializer_export_extra_data :: proc(self_: ^Serializer, anon_param0: ^Base, anon_param1: ^SerializationContext) ---

    /// Exports object's data to stream.
    @(link_name = "PxSerializer_exportData")
    serializer_export_data :: proc(self_: ^Serializer, anon_param0: ^Base, anon_param1: ^SerializationContext) ---

    /// Register references that the object maintains to other objects.
    @(link_name = "PxSerializer_registerReferences")
    serializer_register_references :: proc(self_: ^Serializer, obj: ^Base, s: ^SerializationContext) ---

    /// Returns size needed to create the class instance.
    ///
    /// sizeof class instance.
    @(link_name = "PxSerializer_getClassSize")
    serializer_get_class_size :: proc(self_: ^Serializer) -> _c.size_t ---

    /// Create object at a given address, resolve references and import extra data.
    ///
    /// Created PxBase pointer (needs to be identical to address before increment).
    @(link_name = "PxSerializer_createObject")
    serializer_create_object :: proc(self_: ^Serializer, address: ^^_c.uint8_t, context_: ^DeserializationContext) -> ^Base ---

    /// *******************************************************************************************************************
    @(link_name = "PxSerializer_delete")
    serializer_delete :: proc(self_: ^Serializer) ---

    /// Builds object (TriangleMesh, Heightfield, ConvexMesh or BVH) from given data in PxPhysics.
    ///
    /// PxBase Created object in PxPhysics.
    @(link_name = "PxInsertionCallback_buildObjectFromData_mut")
    insertion_callback_build_object_from_data_mut :: proc(self_: ^InsertionCallback, type: ConcreteType, data: rawptr) -> ^Base ---

    /// Set the user-provided dispatcher object for CPU tasks
    @(link_name = "PxTaskManager_setCpuDispatcher_mut")
    task_manager_set_cpu_dispatcher_mut :: proc(self_: ^TaskManager, ref: ^CpuDispatcher) ---

    /// Get the user-provided dispatcher object for CPU tasks
    ///
    /// The CPU dispatcher object.
    @(link_name = "PxTaskManager_getCpuDispatcher")
    task_manager_get_cpu_dispatcher :: proc(self_: ^TaskManager) -> ^CpuDispatcher ---

    /// Reset any dependencies between Tasks
    ///
    /// Will be called at the start of every frame before tasks are submitted.
    @(link_name = "PxTaskManager_resetDependencies_mut")
    task_manager_reset_dependencies_mut :: proc(self_: ^TaskManager) ---

    /// Called by the owning scene to start the task graph.
    ///
    /// All tasks with ref count of 1 will be dispatched.
    @(link_name = "PxTaskManager_startSimulation_mut")
    task_manager_start_simulation_mut :: proc(self_: ^TaskManager) ---

    /// Called by the owning scene at the end of a simulation step.
    @(link_name = "PxTaskManager_stopSimulation_mut")
    task_manager_stop_simulation_mut :: proc(self_: ^TaskManager) ---

    /// Called by the worker threads to inform the PxTaskManager that a task has completed processing.
    @(link_name = "PxTaskManager_taskCompleted_mut")
    task_manager_task_completed_mut :: proc(self_: ^TaskManager, task: ^Task) ---

    /// Retrieve a task by name
    ///
    /// The ID of the task with that name, or eNOT_PRESENT if not found
    @(link_name = "PxTaskManager_getNamedTask_mut")
    task_manager_get_named_task_mut :: proc(self_: ^TaskManager, name: cstring) -> _c.uint32_t ---

    /// Submit a task with a unique name.
    ///
    /// The ID of the task with that name, or eNOT_PRESENT if not found
    @(link_name = "PxTaskManager_submitNamedTask_mut")
    task_manager_submit_named_task_mut :: proc(self_: ^TaskManager, task: ^Task, name: cstring, type: TaskType) -> _c.uint32_t ---

    /// Submit an unnamed task.
    ///
    /// The ID of the task with that name, or eNOT_PRESENT if not found
    @(link_name = "PxTaskManager_submitUnnamedTask_mut")
    task_manager_submit_unnamed_task_mut :: proc(self_: ^TaskManager, task: ^Task, type: TaskType) -> _c.uint32_t ---

    /// Retrieve a task given a task ID
    ///
    /// The task associated with the ID
    @(link_name = "PxTaskManager_getTaskFromID_mut")
    task_manager_get_task_from_i_d_mut :: proc(self_: ^TaskManager, id: _c.uint32_t) -> ^Task ---

    /// Release the PxTaskManager object, referenced dispatchers will not be released
    @(link_name = "PxTaskManager_release_mut")
    task_manager_release_mut :: proc(self_: ^TaskManager) ---

    /// Construct a new PxTaskManager instance with the given [optional] dispatchers
    @(link_name = "PxTaskManager_createTaskManager")
    task_manager_create_task_manager :: proc(errorCallback: ^ErrorCallback, anon_param1: ^CpuDispatcher) -> ^TaskManager ---

    /// Called by the TaskManager when a task is to be queued for execution.
    ///
    /// Upon receiving a task, the dispatcher should schedule the task to run.
    /// After the task has been run, it should call the release() method and
    /// discard its pointer.
    @(link_name = "PxCpuDispatcher_submitTask_mut")
    cpu_dispatcher_submit_task_mut :: proc(self_: ^CpuDispatcher, task: ^BaseTask) ---

    /// Returns the number of available worker threads for this dispatcher.
    ///
    /// The SDK will use this count to control how many tasks are submitted. By
    /// matching the number of tasks with the number of execution units task
    /// overhead can be reduced.
    @(link_name = "PxCpuDispatcher_getWorkerCount")
    cpu_dispatcher_get_worker_count :: proc(self_: ^CpuDispatcher) -> _c.uint32_t ---

    @(link_name = "PxCpuDispatcher_delete")
    cpu_dispatcher_delete :: proc(self_: ^CpuDispatcher) ---

    /// The user-implemented run method where the task's work should be performed
    ///
    /// run() methods must be thread safe, stack friendly (no alloca, etc), and
    /// must never block.
    @(link_name = "PxBaseTask_run_mut")
    base_task_run_mut :: proc(self_: ^BaseTask) ---

    /// Return a user-provided task name for profiling purposes.
    ///
    /// It does not have to be unique, but unique names are helpful.
    ///
    /// The name of this task
    @(link_name = "PxBaseTask_getName")
    base_task_get_name :: proc(self_: ^BaseTask) -> cstring ---

    /// Implemented by derived implementation classes
    @(link_name = "PxBaseTask_addReference_mut")
    base_task_add_reference_mut :: proc(self_: ^BaseTask) ---

    /// Implemented by derived implementation classes
    @(link_name = "PxBaseTask_removeReference_mut")
    base_task_remove_reference_mut :: proc(self_: ^BaseTask) ---

    /// Implemented by derived implementation classes
    @(link_name = "PxBaseTask_getReference")
    base_task_get_reference :: proc(self_: ^BaseTask) -> _c.int32_t ---

    /// Implemented by derived implementation classes
    ///
    /// A task may assume in its release() method that the task system no longer holds
    /// references to it - so it may safely run its destructor, recycle itself, etc.
    /// provided no additional user references to the task exist
    @(link_name = "PxBaseTask_release_mut")
    base_task_release_mut :: proc(self_: ^BaseTask) ---

    /// Return PxTaskManager to which this task was submitted
    ///
    /// Note, can return NULL if task was not submitted, or has been
    /// completed.
    @(link_name = "PxBaseTask_getTaskManager")
    base_task_get_task_manager :: proc(self_: ^BaseTask) -> ^TaskManager ---

    @(link_name = "PxBaseTask_setContextId_mut")
    base_task_set_context_id_mut :: proc(self_: ^BaseTask, id: _c.uint64_t) ---

    @(link_name = "PxBaseTask_getContextId")
    base_task_get_context_id :: proc(self_: ^BaseTask) -> _c.uint64_t ---

    /// Release method implementation
    @(link_name = "PxTask_release_mut")
    task_release_mut :: proc(self_: ^Task) ---

    /// Inform the PxTaskManager this task must finish before the given
    @(link_name = "PxTask_finishBefore_mut")
    task_finish_before_mut :: proc(self_: ^Task, taskID: _c.uint32_t) ---

    /// Inform the PxTaskManager this task cannot start until the given
    @(link_name = "PxTask_startAfter_mut")
    task_start_after_mut :: proc(self_: ^Task, taskID: _c.uint32_t) ---

    /// Manually increment this task's reference count. The task will
    /// not be allowed to run until removeReference() is called.
    @(link_name = "PxTask_addReference_mut")
    task_add_reference_mut :: proc(self_: ^Task) ---

    /// Manually decrement this task's reference count. If the reference
    /// count reaches zero, the task will be dispatched.
    @(link_name = "PxTask_removeReference_mut")
    task_remove_reference_mut :: proc(self_: ^Task) ---

    /// Return the ref-count for this task
    @(link_name = "PxTask_getReference")
    task_get_reference :: proc(self_: ^Task) -> _c.int32_t ---

    /// Return the unique ID for this task
    @(link_name = "PxTask_getTaskID")
    task_get_task_i_d :: proc(self_: ^Task) -> _c.uint32_t ---

    /// Called by PxTaskManager at submission time for initialization
    ///
    /// Perform simulation step initialization here.
    @(link_name = "PxTask_submitted_mut")
    task_submitted_mut :: proc(self_: ^Task) ---

    /// Initialize this task and specify the task that will have its ref count decremented on completion.
    ///
    /// Submission is deferred until the task's mRefCount is decremented to zero.
    /// Note that we only use the PxTaskManager to query the appropriate dispatcher.
    @(link_name = "PxLightCpuTask_setContinuation_mut")
    light_cpu_task_set_continuation_mut :: proc(self_: ^LightCpuTask, tm: ^TaskManager, c: ^BaseTask) ---

    /// Initialize this task and specify the task that will have its ref count decremented on completion.
    ///
    /// This overload of setContinuation() queries the PxTaskManager from the continuation
    /// task, which cannot be NULL.
    @(link_name = "PxLightCpuTask_setContinuation_mut_1")
    light_cpu_task_set_continuation_mut_1 :: proc(self_: ^LightCpuTask, c: ^BaseTask) ---

    /// Retrieves continuation task
    @(link_name = "PxLightCpuTask_getContinuation")
    light_cpu_task_get_continuation :: proc(self_: ^LightCpuTask) -> ^BaseTask ---

    /// Manually decrement this task's reference count. If the reference
    /// count reaches zero, the task will be dispatched.
    @(link_name = "PxLightCpuTask_removeReference_mut")
    light_cpu_task_remove_reference_mut :: proc(self_: ^LightCpuTask) ---

    /// Return the ref-count for this task
    @(link_name = "PxLightCpuTask_getReference")
    light_cpu_task_get_reference :: proc(self_: ^LightCpuTask) -> _c.int32_t ---

    /// Manually increment this task's reference count. The task will
    /// not be allowed to run until removeReference() is called.
    @(link_name = "PxLightCpuTask_addReference_mut")
    light_cpu_task_add_reference_mut :: proc(self_: ^LightCpuTask) ---

    /// called by CpuDispatcher after run method has completed
    ///
    /// Decrements the continuation task's reference count, if specified.
    @(link_name = "PxLightCpuTask_release_mut")
    light_cpu_task_release_mut :: proc(self_: ^LightCpuTask) ---

    /// Returns the type of the geometry.
    ///
    /// The type of the object.
    @(link_name = "PxGeometry_getType")
    geometry_get_type :: proc(self_: ^Geometry) -> GeometryType ---

    /// Constructor to initialize half extents from scalar parameters.
    @(link_name = "PxBoxGeometry_new")
    box_geometry_new :: proc(hx: _c.float, hy: _c.float, hz: _c.float) -> BoxGeometry ---

    /// Constructor to initialize half extents from vector parameter.
    @(link_name = "PxBoxGeometry_new_1")
    box_geometry_new_1 :: proc(halfExtents_: Vec3) -> BoxGeometry ---

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid
    ///
    /// A valid box has a positive extent in each direction (halfExtents.x > 0, halfExtents.y > 0, halfExtents.z > 0).
    /// It is illegal to call PxRigidActor::createShape and PxPhysics::createShape with a box that has zero extent in any direction.
    @(link_name = "PxBoxGeometry_isValid")
    box_geometry_is_valid :: proc(self_: ^BoxGeometry) -> _c.bool ---

    @(link_name = "PxBVHRaycastCallback_delete")
    b_v_h_raycast_callback_delete :: proc(self_: ^BVHRaycastCallback) ---

    @(link_name = "PxBVHRaycastCallback_reportHit_mut")
    b_v_h_raycast_callback_report_hit_mut :: proc(self_: ^BVHRaycastCallback, boundsIndex: _c.uint32_t, distance: ^_c.float) -> _c.bool ---

    @(link_name = "PxBVHOverlapCallback_delete")
    b_v_h_overlap_callback_delete :: proc(self_: ^BVHOverlapCallback) ---

    @(link_name = "PxBVHOverlapCallback_reportHit_mut")
    b_v_h_overlap_callback_report_hit_mut :: proc(self_: ^BVHOverlapCallback, boundsIndex: _c.uint32_t) -> _c.bool ---

    @(link_name = "PxBVHTraversalCallback_delete")
    b_v_h_traversal_callback_delete :: proc(self_: ^BVHTraversalCallback) ---

    @(link_name = "PxBVHTraversalCallback_visitNode_mut")
    b_v_h_traversal_callback_visit_node_mut :: proc(self_: ^BVHTraversalCallback, #by_ptr bounds: Bounds3) -> _c.bool ---

    @(link_name = "PxBVHTraversalCallback_reportLeaf_mut")
    b_v_h_traversal_callback_report_leaf_mut :: proc(self_: ^BVHTraversalCallback, nbPrims: _c.uint32_t, prims: ^_c.uint32_t) -> _c.bool ---

    /// Raycast test against a BVH.
    ///
    /// false if query has been aborted
    @(link_name = "PxBVH_raycast")
    b_v_h_raycast :: proc(self_: ^BVH, #by_ptr origin: Vec3, #by_ptr unitDir: Vec3, maxDist: _c.float, cb: ^BVHRaycastCallback, queryFlags: GeometryQueryFlags_Set) -> _c.bool ---

    /// Sweep test against a BVH.
    ///
    /// false if query has been aborted
    @(link_name = "PxBVH_sweep")
    b_v_h_sweep :: proc(self_: ^BVH, geom: ^Geometry, #by_ptr pose: Transform, #by_ptr unitDir: Vec3, maxDist: _c.float, cb: ^BVHRaycastCallback, queryFlags: GeometryQueryFlags_Set) -> _c.bool ---

    /// Overlap test against a BVH.
    ///
    /// false if query has been aborted
    @(link_name = "PxBVH_overlap")
    b_v_h_overlap :: proc(self_: ^BVH, geom: ^Geometry, #by_ptr pose: Transform, cb: ^BVHOverlapCallback, queryFlags: GeometryQueryFlags_Set) -> _c.bool ---

    /// Frustum culling test against a BVH.
    ///
    /// This is similar in spirit to an overlap query using a convex object around the frustum.
    /// However this specialized query has better performance, and can support more than the 6 planes
    /// of a frustum, which can be useful in portal-based engines.
    ///
    /// On the other hand this test only returns a conservative number of bounds, i.e. some of the returned
    /// bounds may actually be outside the frustum volume, close to it but not touching it. This is usually
    /// an ok performance trade-off when the function is used for view-frustum culling.
    ///
    /// false if query has been aborted
    @(link_name = "PxBVH_cull")
    b_v_h_cull :: proc(self_: ^BVH, nbPlanes: _c.uint32_t, planes: ^Plane, cb: ^BVHOverlapCallback, queryFlags: GeometryQueryFlags_Set) -> _c.bool ---

    /// Returns the number of bounds in the BVH.
    ///
    /// You can use [`getBounds`]() to retrieve the bounds.
    ///
    /// These are the user-defined bounds passed to the BVH builder, not the internal bounds around each BVH node.
    ///
    /// Number of bounds in the BVH.
    @(link_name = "PxBVH_getNbBounds")
    b_v_h_get_nb_bounds :: proc(self_: ^BVH) -> _c.uint32_t ---

    /// Retrieve the read-only bounds in the BVH.
    ///
    /// These are the user-defined bounds passed to the BVH builder, not the internal bounds around each BVH node.
    @(link_name = "PxBVH_getBounds")
    b_v_h_get_bounds :: proc(self_: ^BVH) -> [^]Bounds3 ---

    /// Retrieve the bounds in the BVH.
    ///
    /// These bounds can be modified. Call refit() after modifications are done.
    ///
    /// These are the user-defined bounds passed to the BVH builder, not the internal bounds around each BVH node.
    @(link_name = "PxBVH_getBoundsForModification_mut")
    b_v_h_get_bounds_for_modification_mut :: proc(self_: ^BVH) -> ^Bounds3 ---

    /// Refit the BVH.
    ///
    /// This function "refits" the tree, i.e. takes the new (leaf) bounding boxes into account and
    /// recomputes all the BVH bounds accordingly. This is an O(n) operation with n = number of bounds in the BVH.
    ///
    /// This works best with minor bounds modifications, i.e. when the bounds remain close to their initial values.
    /// With large modifications the tree quality degrades more and more, and subsequent query performance suffers.
    /// It might be a better strategy to create a brand new BVH if bounds change drastically.
    ///
    /// This function refits the whole tree after an arbitrary number of bounds have potentially been modified by
    /// users (via getBoundsForModification()). If you only have a small number of bounds to update, it might be
    /// more efficient to use setBounds() and partialRefit() instead.
    @(link_name = "PxBVH_refit_mut")
    b_v_h_refit_mut :: proc(self_: ^BVH) ---

    /// Update single bounds.
    ///
    /// This is an alternative to getBoundsForModification() / refit(). If you only have a small set of bounds to
    /// update, it can be inefficient to call the refit() function, because it refits the whole BVH.
    ///
    /// Instead, one can update individual bounds with this updateBounds() function. It sets the new bounds and
    /// marks the corresponding BVH nodes for partial refit. Once all the individual bounds have been updated,
    /// call partialRefit() to only refit the subset of marked nodes.
    ///
    /// true if success
    @(link_name = "PxBVH_updateBounds_mut")
    b_v_h_update_bounds_mut :: proc(self_: ^BVH, boundsIndex: _c.uint32_t, #by_ptr newBounds: Bounds3) -> _c.bool ---

    /// Refits subset of marked nodes.
    ///
    /// This is an alternative to the refit() function, to be called after updateBounds() calls.
    /// See updateBounds() for details.
    @(link_name = "PxBVH_partialRefit_mut")
    b_v_h_partial_refit_mut :: proc(self_: ^BVH) ---

    /// Generic BVH traversal function.
    ///
    /// This can be used to implement custom BVH traversal functions if provided ones are not enough.
    /// In particular this can be used to visualize the tree's bounds.
    ///
    /// false if query has been aborted
    @(link_name = "PxBVH_traverse")
    b_v_h_traverse :: proc(self_: ^BVH, cb: ^BVHTraversalCallback) -> _c.bool ---

    @(link_name = "PxBVH_getConcreteTypeName")
    b_v_h_get_concrete_type_name :: proc(self_: ^BVH) -> cstring ---

    /// Constructor, initializes to a capsule with passed radius and half height.
    @(link_name = "PxCapsuleGeometry_new")
    capsule_geometry_new :: proc(radius_: _c.float, halfHeight_: _c.float) -> CapsuleGeometry ---

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid.
    ///
    /// A valid capsule has radius > 0, halfHeight >= 0.
    /// It is illegal to call PxRigidActor::createShape and PxPhysics::createShape with a capsule that has zero radius or height.
    @(link_name = "PxCapsuleGeometry_isValid")
    capsule_geometry_is_valid :: proc(self_: ^CapsuleGeometry) -> _c.bool ---

    /// Returns the number of vertices.
    ///
    /// Number of vertices.
    @(link_name = "PxConvexMesh_getNbVertices")
    convex_mesh_get_nb_vertices :: proc(self_: ^ConvexMesh) -> _c.uint32_t ---

    /// Returns the vertices.
    ///
    /// Array of vertices.
    @(link_name = "PxConvexMesh_getVertices")
    convex_mesh_get_vertices :: proc(self_: ^ConvexMesh) -> [^]Vec3 ---

    /// Returns the index buffer.
    ///
    /// Index buffer.
    @(link_name = "PxConvexMesh_getIndexBuffer")
    convex_mesh_get_index_buffer :: proc(self_: ^ConvexMesh) -> ^_c.uint8_t ---

    /// Returns the number of polygons.
    ///
    /// Number of polygons.
    @(link_name = "PxConvexMesh_getNbPolygons")
    convex_mesh_get_nb_polygons :: proc(self_: ^ConvexMesh) -> _c.uint32_t ---

    /// Returns the polygon data.
    ///
    /// True if success.
    @(link_name = "PxConvexMesh_getPolygonData")
    convex_mesh_get_polygon_data :: proc(self_: ^ConvexMesh, index: _c.uint32_t, data: ^HullPolygon) -> _c.bool ---

    /// Decrements the reference count of a convex mesh and releases it if the new reference count is zero.
    @(link_name = "PxConvexMesh_release_mut")
    convex_mesh_release_mut :: proc(self_: ^ConvexMesh) ---

    /// Returns the mass properties of the mesh assuming unit density.
    ///
    /// The following relationship holds between mass and volume:
    ///
    /// mass = volume * density
    ///
    /// The mass of a unit density mesh is equal to its volume, so this function returns the volume of the mesh.
    ///
    /// Similarly, to obtain the localInertia of an identically shaped object with a uniform density of d, simply multiply the
    /// localInertia of the unit density mesh by d.
    @(link_name = "PxConvexMesh_getMassInformation")
    convex_mesh_get_mass_information :: proc(self_: ^ConvexMesh, mass: ^_c.float, localInertia: ^Mat33, localCenterOfMass: ^Vec3) ---

    /// Returns the local-space (vertex space) AABB from the convex mesh.
    ///
    /// local-space bounds
    @(link_name = "PxConvexMesh_getLocalBounds")
    convex_mesh_get_local_bounds :: proc(self_: ^ConvexMesh) -> Bounds3 ---

    /// Returns the local-space Signed Distance Field for this mesh if it has one.
    ///
    /// local-space SDF.
    @(link_name = "PxConvexMesh_getSDF")
    convex_mesh_get_s_d_f :: proc(self_: ^ConvexMesh) -> ^_c.float ---

    @(link_name = "PxConvexMesh_getConcreteTypeName")
    convex_mesh_get_concrete_type_name :: proc(self_: ^ConvexMesh) -> cstring ---

    /// This method decides whether a convex mesh is gpu compatible. If the total number of vertices are more than 64 or any number of vertices in a polygon is more than 32, or
    /// convex hull data was not cooked with GPU data enabled during cooking or was loaded from a serialized collection, the convex hull is incompatible with GPU collision detection. Otherwise
    /// it is compatible.
    ///
    /// True if the convex hull is gpu compatible
    @(link_name = "PxConvexMesh_isGpuCompatible")
    convex_mesh_is_gpu_compatible :: proc(self_: ^ConvexMesh) -> _c.bool ---

    /// Constructor initializes to identity scale.
    @(link_name = "PxMeshScale_new")
    mesh_scale_new :: proc() -> MeshScale ---

    /// Constructor from scalar.
    @(link_name = "PxMeshScale_new_1")
    mesh_scale_new_1 :: proc(r: _c.float) -> MeshScale ---

    /// Constructor to initialize to arbitrary scale and identity scale rotation.
    @(link_name = "PxMeshScale_new_2")
    mesh_scale_new_2 :: proc(#by_ptr s: Vec3) -> MeshScale ---

    /// Constructor to initialize to arbitrary scaling.
    @(link_name = "PxMeshScale_new_3")
    mesh_scale_new_3 :: proc(#by_ptr s: Vec3, #by_ptr r: Quat) -> MeshScale ---

    /// Returns true if the scaling is an identity transformation.
    @(link_name = "PxMeshScale_isIdentity")
    mesh_scale_is_identity :: proc(self_: ^MeshScale) -> _c.bool ---

    /// Returns the inverse of this scaling transformation.
    @(link_name = "PxMeshScale_getInverse")
    mesh_scale_get_inverse :: proc(self_: ^MeshScale) -> MeshScale ---

    /// Converts this transformation to a 3x3 matrix representation.
    @(link_name = "PxMeshScale_toMat33")
    mesh_scale_to_mat33 :: proc(self_: ^MeshScale) -> Mat33 ---

    /// Returns true if combination of negative scale components will cause the triangle normal to flip. The SDK will flip the normals internally.
    @(link_name = "PxMeshScale_hasNegativeDeterminant")
    mesh_scale_has_negative_determinant :: proc(self_: ^MeshScale) -> _c.bool ---

    @(link_name = "PxMeshScale_transform")
    mesh_scale_transform :: proc(self_: ^MeshScale, #by_ptr v: Vec3) -> Vec3 ---

    @(link_name = "PxMeshScale_isValidForTriangleMesh")
    mesh_scale_is_valid_for_triangle_mesh :: proc(self_: ^MeshScale) -> _c.bool ---

    @(link_name = "PxMeshScale_isValidForConvexMesh")
    mesh_scale_is_valid_for_convex_mesh :: proc(self_: ^MeshScale) -> _c.bool ---

    /// Constructor. By default creates an empty object with a NULL mesh and identity scale.
    @(link_name = "PxConvexMeshGeometry_new")
    convex_mesh_geometry_new :: proc(mesh: ^ConvexMesh, #by_ptr scaling: MeshScale, flags: ConvexMeshGeometryFlags_Set) -> ConvexMeshGeometry ---

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid for shape creation.
    ///
    /// A valid convex mesh has a positive scale value in each direction (scale.x > 0, scale.y > 0, scale.z > 0).
    /// It is illegal to call PxRigidActor::createShape and PxPhysics::createShape with a convex that has zero extent in any direction.
    @(link_name = "PxConvexMeshGeometry_isValid")
    convex_mesh_geometry_is_valid :: proc(self_: ^ConvexMeshGeometry) -> _c.bool ---

    /// Constructor.
    @(link_name = "PxSphereGeometry_new")
    sphere_geometry_new :: proc(ir: _c.float) -> SphereGeometry ---

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid
    ///
    /// A valid sphere has radius > 0.
    /// It is illegal to call PxRigidActor::createShape and PxPhysics::createShape with a sphere that has zero radius.
    @(link_name = "PxSphereGeometry_isValid")
    sphere_geometry_is_valid :: proc(self_: ^SphereGeometry) -> _c.bool ---

    /// Constructor.
    @(link_name = "PxPlaneGeometry_new")
    plane_geometry_new :: proc() -> PlaneGeometry ---

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid
    @(link_name = "PxPlaneGeometry_isValid")
    plane_geometry_is_valid :: proc(self_: ^PlaneGeometry) -> _c.bool ---

    /// Constructor. By default creates an empty object with a NULL mesh and identity scale.
    @(link_name = "PxTriangleMeshGeometry_new")
    triangle_mesh_geometry_new :: proc(mesh: ^TriangleMesh, #by_ptr scaling: MeshScale, flags: MeshGeometryFlags_Set) -> TriangleMeshGeometry ---

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid for shape creation.
    ///
    /// A valid triangle mesh has a positive scale value in each direction (scale.scale.x > 0, scale.scale.y > 0, scale.scale.z > 0).
    /// It is illegal to call PxRigidActor::createShape and PxPhysics::createShape with a triangle mesh that has zero extents in any direction.
    @(link_name = "PxTriangleMeshGeometry_isValid")
    triangle_mesh_geometry_is_valid :: proc(self_: ^TriangleMeshGeometry) -> _c.bool ---

    /// Constructor.
    @(link_name = "PxHeightFieldGeometry_new")
    height_field_geometry_new :: proc(hf: ^HeightField, flags: MeshGeometryFlags_Set, heightScale_: _c.float, rowScale_: _c.float, columnScale_: _c.float) -> HeightFieldGeometry ---

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid
    ///
    /// A valid height field has a positive scale value in each direction (heightScale > 0, rowScale > 0, columnScale > 0).
    /// It is illegal to call PxRigidActor::createShape and PxPhysics::createShape with a height field that has zero extents in any direction.
    @(link_name = "PxHeightFieldGeometry_isValid")
    height_field_geometry_is_valid :: proc(self_: ^HeightFieldGeometry) -> _c.bool ---

    /// Default constructor.
    ///
    /// Creates an empty object with no particles.
    @(link_name = "PxParticleSystemGeometry_new")
    particle_system_geometry_new :: proc() -> ParticleSystemGeometry ---

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid for shape creation.
    @(link_name = "PxParticleSystemGeometry_isValid")
    particle_system_geometry_is_valid :: proc(self_: ^ParticleSystemGeometry) -> _c.bool ---

    /// Default constructor.
    @(link_name = "PxHairSystemGeometry_new")
    hair_system_geometry_new :: proc() -> HairSystemGeometry ---

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid for shape creation.
    @(link_name = "PxHairSystemGeometry_isValid")
    hair_system_geometry_is_valid :: proc(self_: ^HairSystemGeometry) -> _c.bool ---

    /// Constructor. By default creates an empty object with a NULL mesh and identity scale.
    @(link_name = "PxTetrahedronMeshGeometry_new")
    tetrahedron_mesh_geometry_new :: proc(mesh: ^TetrahedronMesh) -> TetrahedronMeshGeometry ---

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid for shape creation.
    ///
    /// A valid tetrahedron mesh has a positive scale value in each direction (scale.scale.x > 0, scale.scale.y > 0, scale.scale.z > 0).
    /// It is illegal to call PxRigidActor::createShape and PxPhysics::createShape with a tetrahedron mesh that has zero extents in any direction.
    @(link_name = "PxTetrahedronMeshGeometry_isValid")
    tetrahedron_mesh_geometry_is_valid :: proc(self_: ^TetrahedronMeshGeometry) -> _c.bool ---

    @(link_name = "PxQueryHit_new")
    query_hit_new :: proc() -> QueryHit ---

    @(link_name = "PxLocationHit_new")
    location_hit_new :: proc() -> LocationHit ---

    /// For raycast hits: true for shapes overlapping with raycast origin.
    ///
    /// For sweep hits: true for shapes overlapping at zero sweep distance.
    @(link_name = "PxLocationHit_hadInitialOverlap")
    location_hit_had_initial_overlap :: proc(self_: ^LocationHit) -> _c.bool ---

    @(link_name = "PxGeomRaycastHit_new")
    geom_raycast_hit_new :: proc() -> GeomRaycastHit ---

    @(link_name = "PxGeomOverlapHit_new")
    geom_overlap_hit_new :: proc() -> GeomOverlapHit ---

    @(link_name = "PxGeomSweepHit_new")
    geom_sweep_hit_new :: proc() -> GeomSweepHit ---

    @(link_name = "PxGeomIndexPair_new")
    geom_index_pair_new :: proc() -> GeomIndexPair ---

    @(link_name = "PxGeomIndexPair_new_1")
    geom_index_pair_new_1 :: proc(_id0: _c.uint32_t, _id1: _c.uint32_t) -> GeomIndexPair ---

    /// For internal use
    @(link_name = "phys_PxCustomGeometry_getUniqueID")
    custom_geometry_get_unique_i_d :: proc() -> _c.uint32_t ---

    /// Default constructor
    @(link_name = "PxCustomGeometryType_new")
    custom_geometry_type_new :: proc() -> CustomGeometryType ---

    /// Invalid type
    @(link_name = "PxCustomGeometryType_INVALID")
    custom_geometry_type__i_n_v_a_l_i_d :: proc() -> CustomGeometryType ---

    /// Return custom type. The type purpose is for user to differentiate custom geometries. Not used by PhysX.
    ///
    /// Unique ID of a custom geometry type.
    ///
    /// User should use DECLARE_CUSTOM_GEOMETRY_TYPE and IMPLEMENT_CUSTOM_GEOMETRY_TYPE intead of overwriting this function.
    @(link_name = "PxCustomGeometryCallbacks_getCustomType")
    custom_geometry_callbacks_get_custom_type :: proc(self_: ^CustomGeometryCallbacks) -> CustomGeometryType ---

    /// Return local bounds.
    ///
    /// Bounding box in the geometry local space.
    @(link_name = "PxCustomGeometryCallbacks_getLocalBounds")
    custom_geometry_callbacks_get_local_bounds :: proc(self_: ^CustomGeometryCallbacks, geometry: ^Geometry) -> Bounds3 ---

    /// Raycast. Cast a ray against the geometry in given pose.
    ///
    /// Number of hits.
    @(link_name = "PxCustomGeometryCallbacks_raycast")
    custom_geometry_callbacks_raycast :: proc(self_: ^CustomGeometryCallbacks, #by_ptr origin: Vec3, #by_ptr unitDir: Vec3, geom: ^Geometry, #by_ptr pose: Transform, maxDist: _c.float, hitFlags: HitFlags_Set, maxHits: _c.uint32_t, rayHits: ^GeomRaycastHit, stride: _c.uint32_t, threadContext: ^QueryThreadContext) -> _c.uint32_t ---

    /// Overlap. Test if geometries overlap.
    ///
    /// True if there is overlap. False otherwise.
    @(link_name = "PxCustomGeometryCallbacks_overlap")
    custom_geometry_callbacks_overlap :: proc(self_: ^CustomGeometryCallbacks, geom0: ^Geometry, #by_ptr pose0: Transform, geom1: ^Geometry, #by_ptr pose1: Transform, threadContext: ^QueryThreadContext) -> _c.bool ---

    /// Sweep. Sweep one geometry against the other.
    ///
    /// True if there is hit. False otherwise.
    @(link_name = "PxCustomGeometryCallbacks_sweep")
    custom_geometry_callbacks_sweep :: proc(self_: ^CustomGeometryCallbacks, #by_ptr unitDir: Vec3, maxDist: _c.float, geom0: ^Geometry, #by_ptr pose0: Transform, geom1: ^Geometry, #by_ptr pose1: Transform, sweepHit: ^GeomSweepHit, hitFlags: HitFlags_Set, inflation: _c.float, threadContext: ^QueryThreadContext) -> _c.bool ---

    /// Compute custom geometry mass properties. For geometries usable with dynamic rigidbodies.
    @(link_name = "PxCustomGeometryCallbacks_computeMassProperties")
    custom_geometry_callbacks_compute_mass_properties :: proc(self_: ^CustomGeometryCallbacks, geometry: ^Geometry, massProperties: ^MassProperties) ---

    /// Compatible with PhysX's PCM feature. Allows to optimize contact generation.
    @(link_name = "PxCustomGeometryCallbacks_usePersistentContactManifold")
    custom_geometry_callbacks_use_persistent_contact_manifold :: proc(self_: ^CustomGeometryCallbacks, geometry: ^Geometry, breakingThreshold: ^_c.float) -> _c.bool ---

    @(link_name = "PxCustomGeometryCallbacks_delete")
    custom_geometry_callbacks_delete :: proc(self_: ^CustomGeometryCallbacks) ---

    /// Default constructor.
    ///
    /// Creates an empty object with a NULL callbacks pointer.
    @(link_name = "PxCustomGeometry_new")
    custom_geometry_new :: proc() -> CustomGeometry ---

    /// Constructor.
    @(link_name = "PxCustomGeometry_new_1")
    custom_geometry_new_1 :: proc(_callbacks: ^CustomGeometryCallbacks) -> CustomGeometry ---

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid for shape creation.
    @(link_name = "PxCustomGeometry_isValid")
    custom_geometry_is_valid :: proc(self_: ^CustomGeometry) -> _c.bool ---

    /// Returns the custom type of the custom geometry.
    @(link_name = "PxCustomGeometry_getCustomType")
    custom_geometry_get_custom_type :: proc(self_: ^CustomGeometry) -> CustomGeometryType ---

    @(link_name = "PxGeometryHolder_getType")
    geometry_holder_get_type :: proc(self_: ^GeometryHolder) -> GeometryType ---

    @(link_name = "PxGeometryHolder_any_mut")
    geometry_holder_any_mut :: proc(self_: ^GeometryHolder) -> ^Geometry ---

    @(link_name = "PxGeometryHolder_any")
    geometry_holder_any :: proc(self_: ^GeometryHolder) -> ^Geometry ---

    @(link_name = "PxGeometryHolder_sphere_mut")
    geometry_holder_sphere_mut :: proc(self_: ^GeometryHolder) -> ^SphereGeometry ---

    @(link_name = "PxGeometryHolder_sphere")
    geometry_holder_sphere :: proc(self_: ^GeometryHolder) -> ^SphereGeometry ---

    @(link_name = "PxGeometryHolder_plane_mut")
    geometry_holder_plane_mut :: proc(self_: ^GeometryHolder) -> ^PlaneGeometry ---

    @(link_name = "PxGeometryHolder_plane")
    geometry_holder_plane :: proc(self_: ^GeometryHolder) -> ^PlaneGeometry ---

    @(link_name = "PxGeometryHolder_capsule_mut")
    geometry_holder_capsule_mut :: proc(self_: ^GeometryHolder) -> ^CapsuleGeometry ---

    @(link_name = "PxGeometryHolder_capsule")
    geometry_holder_capsule :: proc(self_: ^GeometryHolder) -> ^CapsuleGeometry ---

    @(link_name = "PxGeometryHolder_box_mut")
    geometry_holder_box_mut :: proc(self_: ^GeometryHolder) -> ^BoxGeometry ---

    @(link_name = "PxGeometryHolder_box")
    geometry_holder_box :: proc(self_: ^GeometryHolder) -> ^BoxGeometry ---

    @(link_name = "PxGeometryHolder_convexMesh_mut")
    geometry_holder_convex_mesh_mut :: proc(self_: ^GeometryHolder) -> ^ConvexMeshGeometry ---

    @(link_name = "PxGeometryHolder_convexMesh")
    geometry_holder_convex_mesh :: proc(self_: ^GeometryHolder) -> ^ConvexMeshGeometry ---

    @(link_name = "PxGeometryHolder_tetMesh_mut")
    geometry_holder_tet_mesh_mut :: proc(self_: ^GeometryHolder) -> ^TetrahedronMeshGeometry ---

    @(link_name = "PxGeometryHolder_tetMesh")
    geometry_holder_tet_mesh :: proc(self_: ^GeometryHolder) -> ^TetrahedronMeshGeometry ---

    @(link_name = "PxGeometryHolder_triangleMesh_mut")
    geometry_holder_triangle_mesh_mut :: proc(self_: ^GeometryHolder) -> ^TriangleMeshGeometry ---

    @(link_name = "PxGeometryHolder_triangleMesh")
    geometry_holder_triangle_mesh :: proc(self_: ^GeometryHolder) -> ^TriangleMeshGeometry ---

    @(link_name = "PxGeometryHolder_heightField_mut")
    geometry_holder_height_field_mut :: proc(self_: ^GeometryHolder) -> ^HeightFieldGeometry ---

    @(link_name = "PxGeometryHolder_heightField")
    geometry_holder_height_field :: proc(self_: ^GeometryHolder) -> ^HeightFieldGeometry ---

    @(link_name = "PxGeometryHolder_particleSystem_mut")
    geometry_holder_particle_system_mut :: proc(self_: ^GeometryHolder) -> ^ParticleSystemGeometry ---

    @(link_name = "PxGeometryHolder_particleSystem")
    geometry_holder_particle_system :: proc(self_: ^GeometryHolder) -> ^ParticleSystemGeometry ---

    @(link_name = "PxGeometryHolder_hairSystem_mut")
    geometry_holder_hair_system_mut :: proc(self_: ^GeometryHolder) -> ^HairSystemGeometry ---

    @(link_name = "PxGeometryHolder_hairSystem")
    geometry_holder_hair_system :: proc(self_: ^GeometryHolder) -> ^HairSystemGeometry ---

    @(link_name = "PxGeometryHolder_custom_mut")
    geometry_holder_custom_mut :: proc(self_: ^GeometryHolder) -> ^CustomGeometry ---

    @(link_name = "PxGeometryHolder_custom")
    geometry_holder_custom :: proc(self_: ^GeometryHolder) -> ^CustomGeometry ---

    @(link_name = "PxGeometryHolder_storeAny_mut")
    geometry_holder_store_any_mut :: proc(self_: ^GeometryHolder, geometry: ^Geometry) ---

    @(link_name = "PxGeometryHolder_new")
    geometry_holder_new :: proc() -> GeometryHolder ---

    @(link_name = "PxGeometryHolder_new_1")
    geometry_holder_new_1 :: proc(geometry: ^Geometry) -> GeometryHolder ---

    /// Raycast test against a geometry object.
    ///
    /// All geometry types are supported except PxParticleSystemGeometry, PxTetrahedronMeshGeometry and PxHairSystemGeometry.
    ///
    /// Number of hits between the ray and the geometry object
    @(link_name = "PxGeometryQuery_raycast")
    geometry_query_raycast :: proc(#by_ptr origin: Vec3, #by_ptr unitDir: Vec3, geom: ^Geometry, #by_ptr pose: Transform, maxDist: _c.float, hitFlags: HitFlags_Set, maxHits: _c.uint32_t, rayHits: ^GeomRaycastHit, stride: _c.uint32_t, queryFlags: GeometryQueryFlags_Set, threadContext: ^QueryThreadContext) -> _c.uint32_t ---

    /// Overlap test for two geometry objects.
    ///
    /// All combinations are supported except:
    ///
    /// PxPlaneGeometry vs. {PxPlaneGeometry, PxTriangleMeshGeometry, PxHeightFieldGeometry}
    ///
    /// PxTriangleMeshGeometry vs. PxHeightFieldGeometry
    ///
    /// PxHeightFieldGeometry vs. PxHeightFieldGeometry
    ///
    /// Anything involving PxParticleSystemGeometry, PxTetrahedronMeshGeometry or PxHairSystemGeometry.
    ///
    /// True if the two geometry objects overlap
    @(link_name = "PxGeometryQuery_overlap")
    geometry_query_overlap :: proc(geom0: ^Geometry, #by_ptr pose0: Transform, geom1: ^Geometry, #by_ptr pose1: Transform, queryFlags: GeometryQueryFlags_Set, threadContext: ^QueryThreadContext) -> _c.bool ---

    /// Sweep a specified geometry object in space and test for collision with a given object.
    ///
    /// The following combinations are supported.
    ///
    /// PxSphereGeometry vs. {PxSphereGeometry, PxPlaneGeometry, PxCapsuleGeometry, PxBoxGeometry, PxConvexMeshGeometry, PxTriangleMeshGeometry, PxHeightFieldGeometry}
    ///
    /// PxCapsuleGeometry vs. {PxSphereGeometry, PxPlaneGeometry, PxCapsuleGeometry, PxBoxGeometry, PxConvexMeshGeometry, PxTriangleMeshGeometry, PxHeightFieldGeometry}
    ///
    /// PxBoxGeometry vs. {PxSphereGeometry, PxPlaneGeometry, PxCapsuleGeometry, PxBoxGeometry, PxConvexMeshGeometry, PxTriangleMeshGeometry, PxHeightFieldGeometry}
    ///
    /// PxConvexMeshGeometry vs. {PxSphereGeometry, PxPlaneGeometry, PxCapsuleGeometry, PxBoxGeometry, PxConvexMeshGeometry, PxTriangleMeshGeometry, PxHeightFieldGeometry}
    ///
    /// True if the swept geometry object geom0 hits the object geom1
    @(link_name = "PxGeometryQuery_sweep")
    geometry_query_sweep :: proc(#by_ptr unitDir: Vec3, maxDist: _c.float, geom0: ^Geometry, #by_ptr pose0: Transform, geom1: ^Geometry, #by_ptr pose1: Transform, sweepHit: ^GeomSweepHit, hitFlags: HitFlags_Set, inflation: _c.float, queryFlags: GeometryQueryFlags_Set, threadContext: ^QueryThreadContext) -> _c.bool ---

    /// Compute minimum translational distance (MTD) between two geometry objects.
    ///
    /// All combinations of geom objects are supported except:
    /// - plane/plane
    /// - plane/mesh
    /// - plane/heightfield
    /// - mesh/mesh
    /// - mesh/heightfield
    /// - heightfield/heightfield
    /// - anything involving PxParticleSystemGeometry, PxTetrahedronMeshGeometry or PxHairSystemGeometry
    ///
    /// The function returns a unit vector ('direction') and a penetration depth ('depth').
    ///
    /// The depenetration vector D = direction * depth should be applied to the first object, to
    /// get out of the second object.
    ///
    /// Returned depth should always be positive or null.
    ///
    /// If objects do not overlap, the function can not compute the MTD and returns false.
    ///
    /// True if the MTD has successfully been computed, i.e. if objects do overlap.
    @(link_name = "PxGeometryQuery_computePenetration")
    geometry_query_compute_penetration :: proc(direction: ^Vec3, depth: ^_c.float, geom0: ^Geometry, #by_ptr pose0: Transform, geom1: ^Geometry, #by_ptr pose1: Transform, queryFlags: GeometryQueryFlags_Set) -> _c.bool ---

    /// Computes distance between a point and a geometry object.
    ///
    /// Currently supported geometry objects: box, sphere, capsule, convex, mesh.
    ///
    /// For meshes, only the BVH34 midphase data-structure is supported.
    ///
    /// Square distance between the point and the geom object, or 0.0 if the point is inside the object, or -1.0 if an error occured (geometry type is not supported, or invalid pose)
    @(link_name = "PxGeometryQuery_pointDistance")
    geometry_query_point_distance :: proc(#by_ptr point: Vec3, geom: ^Geometry, #by_ptr pose: Transform, closestPoint: ^Vec3, closestIndex: ^_c.uint32_t, queryFlags: GeometryQueryFlags_Set) -> _c.float ---

    /// computes the bounds for a geometry object
    @(link_name = "PxGeometryQuery_computeGeomBounds")
    geometry_query_compute_geom_bounds :: proc(bounds: ^Bounds3, geom: ^Geometry, #by_ptr pose: Transform, offset: _c.float, inflation: _c.float, queryFlags: GeometryQueryFlags_Set) ---

    /// Checks if provided geometry is valid.
    ///
    /// True if geometry is valid.
    @(link_name = "PxGeometryQuery_isValid")
    geometry_query_is_valid :: proc(geom: ^Geometry) -> _c.bool ---

    @(link_name = "PxHeightFieldSample_tessFlag")
    height_field_sample_tess_flag :: proc(self_: ^HeightFieldSample) -> _c.uint8_t ---

    @(link_name = "PxHeightFieldSample_setTessFlag_mut")
    height_field_sample_set_tess_flag_mut :: proc(self_: ^HeightFieldSample) ---

    @(link_name = "PxHeightFieldSample_clearTessFlag_mut")
    height_field_sample_clear_tess_flag_mut :: proc(self_: ^HeightFieldSample) ---

    /// Decrements the reference count of a height field and releases it if the new reference count is zero.
    @(link_name = "PxHeightField_release_mut")
    height_field_release_mut :: proc(self_: ^HeightField) ---

    /// Writes out the sample data array.
    ///
    /// The user provides destBufferSize bytes storage at destBuffer.
    /// The data is formatted and arranged as PxHeightFieldDesc.samples.
    ///
    /// The number of bytes written.
    @(link_name = "PxHeightField_saveCells")
    height_field_save_cells :: proc(self_: ^HeightField, destBuffer: rawptr, destBufferSize: _c.uint32_t) -> _c.uint32_t ---

    /// Replaces a rectangular subfield in the sample data array.
    ///
    /// The user provides the description of a rectangular subfield in subfieldDesc.
    /// The data is formatted and arranged as PxHeightFieldDesc.samples.
    ///
    /// True on success, false on failure. Failure can occur due to format mismatch.
    ///
    /// Modified samples are constrained to the same height quantization range as the original heightfield.
    /// Source samples that are out of range of target heightfield will be clipped with no error.
    /// PhysX does not keep a mapping from the heightfield to heightfield shapes that reference it.
    /// Call PxShape::setGeometry on each shape which references the height field, to ensure that internal data structures are updated to reflect the new geometry.
    /// Please note that PxShape::setGeometry does not guarantee correct/continuous behavior when objects are resting on top of old or new geometry.
    @(link_name = "PxHeightField_modifySamples_mut")
    height_field_modify_samples_mut :: proc(self_: ^HeightField, startCol: _c.int32_t, startRow: _c.int32_t, #by_ptr subfieldDesc: HeightFieldDesc, shrinkBounds: _c.bool) -> _c.bool ---

    /// Retrieves the number of sample rows in the samples array.
    ///
    /// The number of sample rows in the samples array.
    @(link_name = "PxHeightField_getNbRows")
    height_field_get_nb_rows :: proc(self_: ^HeightField) -> _c.uint32_t ---

    /// Retrieves the number of sample columns in the samples array.
    ///
    /// The number of sample columns in the samples array.
    @(link_name = "PxHeightField_getNbColumns")
    height_field_get_nb_columns :: proc(self_: ^HeightField) -> _c.uint32_t ---

    /// Retrieves the format of the sample data.
    ///
    /// The format of the sample data.
    @(link_name = "PxHeightField_getFormat")
    height_field_get_format :: proc(self_: ^HeightField) -> HeightFieldFormat ---

    /// Retrieves the offset in bytes between consecutive samples in the array.
    ///
    /// The offset in bytes between consecutive samples in the array.
    @(link_name = "PxHeightField_getSampleStride")
    height_field_get_sample_stride :: proc(self_: ^HeightField) -> _c.uint32_t ---

    /// Retrieves the convex edge threshold.
    ///
    /// The convex edge threshold.
    @(link_name = "PxHeightField_getConvexEdgeThreshold")
    height_field_get_convex_edge_threshold :: proc(self_: ^HeightField) -> _c.float ---

    /// Retrieves the flags bits, combined from values of the enum ::PxHeightFieldFlag.
    ///
    /// The flags bits, combined from values of the enum ::PxHeightFieldFlag.
    @(link_name = "PxHeightField_getFlags")
    height_field_get_flags :: proc(self_: ^HeightField) -> HeightFieldFlags_Set ---

    /// Retrieves the height at the given coordinates in grid space.
    ///
    /// The height at the given coordinates or 0 if the coordinates are out of range.
    @(link_name = "PxHeightField_getHeight")
    height_field_get_height :: proc(self_: ^HeightField, x: _c.float, z: _c.float) -> _c.float ---

    /// Returns material table index of given triangle
    ///
    /// This function takes a post cooking triangle index.
    ///
    /// Material table index, or 0xffff if no per-triangle materials are used
    @(link_name = "PxHeightField_getTriangleMaterialIndex")
    height_field_get_triangle_material_index :: proc(self_: ^HeightField, triangleIndex: _c.uint32_t) -> _c.uint16_t ---

    /// Returns a triangle face normal for a given triangle index
    ///
    /// This function takes a post cooking triangle index.
    ///
    /// Triangle normal for a given triangle index
    @(link_name = "PxHeightField_getTriangleNormal")
    height_field_get_triangle_normal :: proc(self_: ^HeightField, triangleIndex: _c.uint32_t) -> Vec3 ---

    /// Returns heightfield sample of given row and column
    ///
    /// Heightfield sample
    @(link_name = "PxHeightField_getSample")
    height_field_get_sample :: proc(self_: ^HeightField, row: _c.uint32_t, column: _c.uint32_t) -> ^HeightFieldSample ---

    /// Returns the number of times the heightfield data has been modified
    ///
    /// This method returns the number of times modifySamples has been called on this heightfield, so that code that has
    /// retained state that depends on the heightfield can efficiently determine whether it has been modified.
    ///
    /// the number of times the heightfield sample data has been modified.
    @(link_name = "PxHeightField_getTimestamp")
    height_field_get_timestamp :: proc(self_: ^HeightField) -> _c.uint32_t ---

    @(link_name = "PxHeightField_getConcreteTypeName")
    height_field_get_concrete_type_name :: proc(self_: ^HeightField) -> cstring ---

    /// Constructor sets to default.
    @(link_name = "PxHeightFieldDesc_new")
    height_field_desc_new :: proc() -> HeightFieldDesc ---

    /// (re)sets the structure to the default.
    @(link_name = "PxHeightFieldDesc_setToDefault_mut")
    height_field_desc_set_to_default_mut :: proc(self_: ^HeightFieldDesc) ---

    /// Returns true if the descriptor is valid.
    ///
    /// True if the current settings are valid.
    @(link_name = "PxHeightFieldDesc_isValid")
    height_field_desc_is_valid :: proc(self_: ^HeightFieldDesc) -> _c.bool ---

    /// Retrieves triangle data from a triangle ID.
    ///
    /// This function can be used together with [`findOverlapTriangleMesh`]() to retrieve triangle properties.
    ///
    /// This function will flip the triangle normal whenever triGeom.scale.hasNegativeDeterminant() is true.
    @(link_name = "PxMeshQuery_getTriangle")
    mesh_query_get_triangle :: proc(#by_ptr triGeom: TriangleMeshGeometry, #by_ptr transform: Transform, triangleIndex: _c.uint32_t, triangle: ^Triangle, vertexIndices: ^_c.uint32_t, adjacencyIndices: ^_c.uint32_t) ---

    /// Retrieves triangle data from a triangle ID.
    ///
    /// This function can be used together with [`findOverlapHeightField`]() to retrieve triangle properties.
    ///
    /// This function will flip the triangle normal whenever triGeom.scale.hasNegativeDeterminant() is true.
    ///
    /// TriangleIndex is an index used in internal format, which does have an index out of the bounds in last row.
    /// To traverse all tri indices in the HF, the following code can be applied:
    /// for (PxU32 row = 0; row
    /// <
    /// (nbRows - 1); row++)
    /// {
    /// for (PxU32 col = 0; col
    /// <
    /// (nbCols - 1); col++)
    /// {
    /// for (PxU32 k = 0; k
    /// <
    /// 2; k++)
    /// {
    /// const PxU32 triIndex = 2 * (row*nbCols + col) + k;
    /// ....
    /// }
    /// }
    /// }
    @(link_name = "PxMeshQuery_getTriangle_1")
    mesh_query_get_triangle_1 :: proc(#by_ptr hfGeom: HeightFieldGeometry, #by_ptr transform: Transform, triangleIndex: _c.uint32_t, triangle: ^Triangle, vertexIndices: ^_c.uint32_t, adjacencyIndices: ^_c.uint32_t) ---

    /// Find the mesh triangles which touch the specified geometry object.
    ///
    /// For mesh-vs-mesh overlap tests, please use the specialized function below.
    ///
    /// Returned triangle indices can be used with [`getTriangle`]() to retrieve the triangle properties.
    ///
    /// Number of overlaps found, i.e. number of elements written to the results buffer
    @(link_name = "PxMeshQuery_findOverlapTriangleMesh")
    mesh_query_find_overlap_triangle_mesh :: proc(geom: ^Geometry, #by_ptr geomPose: Transform, #by_ptr meshGeom: TriangleMeshGeometry, #by_ptr meshPose: Transform, results: ^_c.uint32_t, maxResults: _c.uint32_t, startIndex: _c.uint32_t, overflow: ^_c.bool, queryFlags: GeometryQueryFlags_Set) -> _c.uint32_t ---

    /// Find the height field triangles which touch the specified geometry object.
    ///
    /// Returned triangle indices can be used with [`getTriangle`]() to retrieve the triangle properties.
    ///
    /// Number of overlaps found, i.e. number of elements written to the results buffer
    @(link_name = "PxMeshQuery_findOverlapHeightField")
    mesh_query_find_overlap_height_field :: proc(geom: ^Geometry, #by_ptr geomPose: Transform, #by_ptr hfGeom: HeightFieldGeometry, #by_ptr hfPose: Transform, results: ^_c.uint32_t, maxResults: _c.uint32_t, startIndex: _c.uint32_t, overflow: ^_c.bool, queryFlags: GeometryQueryFlags_Set) -> _c.uint32_t ---

    /// Sweep a specified geometry object in space and test for collision with a set of given triangles.
    ///
    /// This function simply sweeps input geometry against each input triangle, in the order they are given.
    /// This is an O(N) operation with N = number of input triangles. It does not use any particular acceleration structure.
    ///
    /// True if the swept geometry object hits the specified triangles
    ///
    /// Only the following geometry types are currently supported: PxSphereGeometry, PxCapsuleGeometry, PxBoxGeometry
    ///
    /// If a shape from the scene is already overlapping with the query shape in its starting position, the hit is returned unless eASSUME_NO_INITIAL_OVERLAP was specified.
    ///
    /// This function returns a single closest hit across all the input triangles. Multiple hits are not supported.
    ///
    /// Supported hitFlags are PxHitFlag::eDEFAULT, PxHitFlag::eASSUME_NO_INITIAL_OVERLAP, PxHitFlag::ePRECISE_SWEEP, PxHitFlag::eMESH_BOTH_SIDES, PxHitFlag::eMESH_ANY.
    ///
    /// ePOSITION is only defined when there is no initial overlap (sweepHit.hadInitialOverlap() == false)
    ///
    /// The returned normal for initially overlapping sweeps is set to -unitDir.
    ///
    /// Otherwise the returned normal is the front normal of the triangle even if PxHitFlag::eMESH_BOTH_SIDES is set.
    ///
    /// The returned PxGeomSweepHit::faceIndex parameter will hold the index of the hit triangle in input array, i.e. the range is [0; triangleCount). For initially overlapping sweeps, this is the index of overlapping triangle.
    ///
    /// The inflation parameter is not compatible with PxHitFlag::ePRECISE_SWEEP.
    @(link_name = "PxMeshQuery_sweep")
    mesh_query_sweep :: proc(#by_ptr unitDir: Vec3, distance: _c.float, geom: ^Geometry, #by_ptr pose: Transform, triangleCount: _c.uint32_t, triangles: ^Triangle, sweepHit: ^GeomSweepHit, hitFlags: HitFlags_Set, cachedIndex: ^_c.uint32_t, inflation: _c.float, doubleSided: _c.bool, queryFlags: GeometryQueryFlags_Set) -> _c.bool ---

    /// constructor sets to default.
    @(link_name = "PxSimpleTriangleMesh_new")
    simple_triangle_mesh_new :: proc() -> SimpleTriangleMesh ---

    /// (re)sets the structure to the default.
    @(link_name = "PxSimpleTriangleMesh_setToDefault_mut")
    simple_triangle_mesh_set_to_default_mut :: proc(self_: ^SimpleTriangleMesh) ---

    /// returns true if the current settings are valid
    @(link_name = "PxSimpleTriangleMesh_isValid")
    simple_triangle_mesh_is_valid :: proc(self_: ^SimpleTriangleMesh) -> _c.bool ---

    /// Constructor
    @(link_name = "PxTriangle_new_alloc")
    triangle_new_alloc :: proc() -> ^Triangle ---

    /// Constructor
    @(link_name = "PxTriangle_new_alloc_1")
    triangle_new_alloc_1 :: proc(#by_ptr p0: Vec3, #by_ptr p1: Vec3, #by_ptr p2: Vec3) -> ^Triangle ---

    /// Destructor
    @(link_name = "PxTriangle_delete")
    triangle_delete :: proc(self_: ^Triangle) ---

    /// Compute the normal of the Triangle.
    @(link_name = "PxTriangle_normal")
    triangle_normal :: proc(self_: ^Triangle, _normal: ^Vec3) ---

    /// Compute the unnormalized normal of the triangle.
    @(link_name = "PxTriangle_denormalizedNormal")
    triangle_denormalized_normal :: proc(self_: ^Triangle, _normal: ^Vec3) ---

    /// Compute the area of the triangle.
    ///
    /// Area of the triangle.
    @(link_name = "PxTriangle_area")
    triangle_area :: proc(self_: ^Triangle) -> _c.float ---

    /// Computes a point on the triangle from u and v barycentric coordinates.
    @(link_name = "PxTriangle_pointFromUV")
    triangle_point_from_u_v :: proc(self_: ^Triangle, u: _c.float, v: _c.float) -> Vec3 ---

    @(link_name = "PxTrianglePadded_new_alloc")
    triangle_padded_new_alloc :: proc() -> ^TrianglePadded ---

    @(link_name = "PxTrianglePadded_delete")
    triangle_padded_delete :: proc(self_: ^TrianglePadded) ---

    /// Returns the number of vertices.
    ///
    /// number of vertices
    @(link_name = "PxTriangleMesh_getNbVertices")
    triangle_mesh_get_nb_vertices :: proc(self_: ^TriangleMesh) -> _c.uint32_t ---

    /// Returns the vertices.
    ///
    /// array of vertices
    @(link_name = "PxTriangleMesh_getVertices")
    triangle_mesh_get_vertices :: proc(self_: ^TriangleMesh) -> [^]Vec3 ---

    /// Returns all mesh vertices for modification.
    ///
    /// This function will return the vertices of the mesh so that their positions can be changed in place.
    /// After modifying the vertices you must call refitBVH for the refitting to actually take place.
    /// This function maintains the old mesh topology (triangle indices).
    ///
    /// inplace vertex coordinates for each existing mesh vertex.
    ///
    /// It is recommended to use this feature for scene queries only.
    ///
    /// Size of array returned is equal to the number returned by getNbVertices().
    ///
    /// This function operates on cooked vertex indices.
    ///
    /// This means the index mapping and vertex count can be different from what was provided as an input to the cooking routine.
    ///
    /// To achieve unchanged 1-to-1 index mapping with orignal mesh data (before cooking) please use the following cooking flags:
    ///
    /// eWELD_VERTICES = 0, eDISABLE_CLEAN_MESH = 1.
    ///
    /// It is also recommended to make sure that a call to validateTriangleMesh returns true if mesh cleaning is disabled.
    @(link_name = "PxTriangleMesh_getVerticesForModification_mut")
    triangle_mesh_get_vertices_for_modification_mut :: proc(self_: ^TriangleMesh) -> ^Vec3 ---

    /// Refits BVH for mesh vertices.
    ///
    /// This function will refit the mesh BVH to correctly enclose the new positions updated by getVerticesForModification.
    /// Mesh BVH will not be reoptimized by this function so significantly different new positions will cause significantly reduced performance.
    ///
    /// New bounds for the entire mesh.
    ///
    /// For PxMeshMidPhase::eBVH34 trees the refit operation is only available on non-quantized trees (see PxBVH34MidphaseDesc::quantized)
    ///
    /// PhysX does not keep a mapping from the mesh to mesh shapes that reference it.
    ///
    /// Call PxShape::setGeometry on each shape which references the mesh, to ensure that internal data structures are updated to reflect the new geometry.
    ///
    /// PxShape::setGeometry does not guarantee correct/continuous behavior when objects are resting on top of old or new geometry.
    ///
    /// It is also recommended to make sure that a call to validateTriangleMesh returns true if mesh cleaning is disabled.
    ///
    /// Active edges information will be lost during refit, the rigid body mesh contact generation might not perform as expected.
    @(link_name = "PxTriangleMesh_refitBVH_mut")
    triangle_mesh_refit_b_v_h_mut :: proc(self_: ^TriangleMesh) -> Bounds3 ---

    /// Returns the number of triangles.
    ///
    /// number of triangles
    @(link_name = "PxTriangleMesh_getNbTriangles")
    triangle_mesh_get_nb_triangles :: proc(self_: ^TriangleMesh) -> _c.uint32_t ---

    /// Returns the triangle indices.
    ///
    /// The indices can be 16 or 32bit depending on the number of triangles in the mesh.
    /// Call getTriangleMeshFlags() to know if the indices are 16 or 32 bits.
    ///
    /// The number of indices is the number of triangles * 3.
    ///
    /// array of triangles
    @(link_name = "PxTriangleMesh_getTriangles")
    triangle_mesh_get_triangles :: proc(self_: ^TriangleMesh) -> rawptr ---

    /// Reads the PxTriangleMesh flags.
    ///
    /// See the list of flags [`PxTriangleMeshFlag`]
    ///
    /// The values of the PxTriangleMesh flags.
    @(link_name = "PxTriangleMesh_getTriangleMeshFlags")
    triangle_mesh_get_triangle_mesh_flags :: proc(self_: ^TriangleMesh) -> TriangleMeshFlags_Set ---

    /// Returns the triangle remapping table.
    ///
    /// The triangles are internally sorted according to various criteria. Hence the internal triangle order
    /// does not always match the original (user-defined) order. The remapping table helps finding the old
    /// indices knowing the new ones:
    ///
    /// remapTable[ internalTriangleIndex ] = originalTriangleIndex
    ///
    /// the remapping table (or NULL if 'PxCookingParams::suppressTriangleMeshRemapTable' has been used)
    @(link_name = "PxTriangleMesh_getTrianglesRemap")
    triangle_mesh_get_triangles_remap :: proc(self_: ^TriangleMesh) -> ^_c.uint32_t ---

    /// Decrements the reference count of a triangle mesh and releases it if the new reference count is zero.
    @(link_name = "PxTriangleMesh_release_mut")
    triangle_mesh_release_mut :: proc(self_: ^TriangleMesh) ---

    /// Returns material table index of given triangle
    ///
    /// This function takes a post cooking triangle index.
    ///
    /// Material table index, or 0xffff if no per-triangle materials are used
    @(link_name = "PxTriangleMesh_getTriangleMaterialIndex")
    triangle_mesh_get_triangle_material_index :: proc(self_: ^TriangleMesh, triangleIndex: _c.uint32_t) -> _c.uint16_t ---

    /// Returns the local-space (vertex space) AABB from the triangle mesh.
    ///
    /// local-space bounds
    @(link_name = "PxTriangleMesh_getLocalBounds")
    triangle_mesh_get_local_bounds :: proc(self_: ^TriangleMesh) -> Bounds3 ---

    /// Returns the local-space Signed Distance Field for this mesh if it has one.
    ///
    /// local-space SDF.
    @(link_name = "PxTriangleMesh_getSDF")
    triangle_mesh_get_s_d_f :: proc(self_: ^TriangleMesh) -> ^_c.float ---

    /// Returns the resolution of the local-space dense SDF.
    @(link_name = "PxTriangleMesh_getSDFDimensions")
    triangle_mesh_get_s_d_f_dimensions :: proc(self_: ^TriangleMesh, numX: ^_c.uint32_t, numY: ^_c.uint32_t, numZ: ^_c.uint32_t) ---

    /// Sets whether this mesh should be preferred for SDF projection.
    ///
    /// By default, meshes are flagged as preferring projection and the decisions on which mesh to project is based on the triangle and vertex
    /// count. The model with the fewer triangles is projected onto the SDF of the more detailed mesh.
    /// If one of the meshes is set to prefer SDF projection (default) and the other is set to not prefer SDF projection, model flagged as
    /// preferring SDF projection will be projected onto the model flagged as not preferring, regardless of the detail of the respective meshes.
    /// Where both models are flagged as preferring no projection, the less detailed model will be projected as before.
    @(link_name = "PxTriangleMesh_setPreferSDFProjection_mut")
    triangle_mesh_set_prefer_s_d_f_projection_mut :: proc(self_: ^TriangleMesh, preferProjection: _c.bool) ---

    /// Returns whether this mesh prefers SDF projection.
    ///
    /// whether this mesh prefers SDF projection.
    @(link_name = "PxTriangleMesh_getPreferSDFProjection")
    triangle_mesh_get_prefer_s_d_f_projection :: proc(self_: ^TriangleMesh) -> _c.bool ---

    /// Returns the mass properties of the mesh assuming unit density.
    ///
    /// The following relationship holds between mass and volume:
    ///
    /// mass = volume * density
    ///
    /// The mass of a unit density mesh is equal to its volume, so this function returns the volume of the mesh.
    ///
    /// Similarly, to obtain the localInertia of an identically shaped object with a uniform density of d, simply multiply the
    /// localInertia of the unit density mesh by d.
    @(link_name = "PxTriangleMesh_getMassInformation")
    triangle_mesh_get_mass_information :: proc(self_: ^TriangleMesh, mass: ^_c.float, localInertia: ^Mat33, localCenterOfMass: ^Vec3) ---

    /// Constructor
    @(link_name = "PxTetrahedron_new_alloc")
    tetrahedron_new_alloc :: proc() -> ^Tetrahedron ---

    /// Constructor
    @(link_name = "PxTetrahedron_new_alloc_1")
    tetrahedron_new_alloc_1 :: proc(#by_ptr p0: Vec3, #by_ptr p1: Vec3, #by_ptr p2: Vec3, #by_ptr p3: Vec3) -> ^Tetrahedron ---

    /// Destructor
    @(link_name = "PxTetrahedron_delete")
    tetrahedron_delete :: proc(self_: ^Tetrahedron) ---

    /// Decrements the reference count of a tetrahedron mesh and releases it if the new reference count is zero.
    @(link_name = "PxSoftBodyAuxData_release_mut")
    soft_body_aux_data_release_mut :: proc(self_: ^SoftBodyAuxData) ---

    /// Returns the number of vertices.
    ///
    /// number of vertices
    @(link_name = "PxTetrahedronMesh_getNbVertices")
    tetrahedron_mesh_get_nb_vertices :: proc(self_: ^TetrahedronMesh) -> _c.uint32_t ---

    /// Returns the vertices
    ///
    /// array of vertices
    @(link_name = "PxTetrahedronMesh_getVertices")
    tetrahedron_mesh_get_vertices :: proc(self_: ^TetrahedronMesh) -> [^]Vec3 ---

    /// Returns the number of tetrahedrons.
    ///
    /// number of tetrahedrons
    @(link_name = "PxTetrahedronMesh_getNbTetrahedrons")
    tetrahedron_mesh_get_nb_tetrahedrons :: proc(self_: ^TetrahedronMesh) -> _c.uint32_t ---

    /// Returns the tetrahedron indices.
    ///
    /// The indices can be 16 or 32bit depending on the number of tetrahedrons in the mesh.
    /// Call getTetrahedronMeshFlags() to know if the indices are 16 or 32 bits.
    ///
    /// The number of indices is the number of tetrahedrons * 4.
    ///
    /// array of tetrahedrons
    @(link_name = "PxTetrahedronMesh_getTetrahedrons")
    tetrahedron_mesh_get_tetrahedrons :: proc(self_: ^TetrahedronMesh) -> rawptr ---

    /// Reads the PxTetrahedronMesh flags.
    ///
    /// See the list of flags [`PxTetrahedronMeshFlags`]
    ///
    /// The values of the PxTetrahedronMesh flags.
    @(link_name = "PxTetrahedronMesh_getTetrahedronMeshFlags")
    tetrahedron_mesh_get_tetrahedron_mesh_flags :: proc(self_: ^TetrahedronMesh) -> TetrahedronMeshFlags_Set ---

    /// Returns the tetrahedra remapping table.
    ///
    /// The tetrahedra are internally sorted according to various criteria. Hence the internal tetrahedron order
    /// does not always match the original (user-defined) order. The remapping table helps finding the old
    /// indices knowing the new ones:
    ///
    /// remapTable[ internalTetrahedronIndex ] = originalTetrahedronIndex
    ///
    /// the remapping table (or NULL if 'PxCookingParams::suppressTriangleMeshRemapTable' has been used)
    @(link_name = "PxTetrahedronMesh_getTetrahedraRemap")
    tetrahedron_mesh_get_tetrahedra_remap :: proc(self_: ^TetrahedronMesh) -> ^_c.uint32_t ---

    /// Returns the local-space (vertex space) AABB from the tetrahedron mesh.
    ///
    /// local-space bounds
    @(link_name = "PxTetrahedronMesh_getLocalBounds")
    tetrahedron_mesh_get_local_bounds :: proc(self_: ^TetrahedronMesh) -> Bounds3 ---

    /// Decrements the reference count of a tetrahedron mesh and releases it if the new reference count is zero.
    @(link_name = "PxTetrahedronMesh_release_mut")
    tetrahedron_mesh_release_mut :: proc(self_: ^TetrahedronMesh) ---

    /// Const accecssor to the softbody's collision mesh.
    @(link_name = "PxSoftBodyMesh_getCollisionMesh")
    soft_body_mesh_get_collision_mesh :: proc(self_: ^SoftBodyMesh) -> ^TetrahedronMesh ---

    /// Accecssor to the softbody's collision mesh.
    @(link_name = "PxSoftBodyMesh_getCollisionMesh_mut")
    soft_body_mesh_get_collision_mesh_mut :: proc(self_: ^SoftBodyMesh) -> ^TetrahedronMesh ---

    /// Const accessor to the softbody's simulation mesh.
    @(link_name = "PxSoftBodyMesh_getSimulationMesh")
    soft_body_mesh_get_simulation_mesh :: proc(self_: ^SoftBodyMesh) -> ^TetrahedronMesh ---

    /// Accecssor to the softbody's simulation mesh.
    @(link_name = "PxSoftBodyMesh_getSimulationMesh_mut")
    soft_body_mesh_get_simulation_mesh_mut :: proc(self_: ^SoftBodyMesh) -> ^TetrahedronMesh ---

    /// Const accessor to the softbodies simulation state.
    @(link_name = "PxSoftBodyMesh_getSoftBodyAuxData")
    soft_body_mesh_get_soft_body_aux_data :: proc(self_: ^SoftBodyMesh) -> ^SoftBodyAuxData ---

    /// Accessor to the softbody's auxilary data like mass and rest pose information
    @(link_name = "PxSoftBodyMesh_getSoftBodyAuxData_mut")
    soft_body_mesh_get_soft_body_aux_data_mut :: proc(self_: ^SoftBodyMesh) -> ^SoftBodyAuxData ---

    /// Decrements the reference count of a tetrahedron mesh and releases it if the new reference count is zero.
    @(link_name = "PxSoftBodyMesh_release_mut")
    soft_body_mesh_release_mut :: proc(self_: ^SoftBodyMesh) ---

    @(link_name = "PxCollisionMeshMappingData_release_mut")
    collision_mesh_mapping_data_release_mut :: proc(self_: ^CollisionMeshMappingData) ---

    @(link_name = "PxCollisionTetrahedronMeshData_getMesh")
    collision_tetrahedron_mesh_data_get_mesh :: proc(self_: ^CollisionTetrahedronMeshData) -> ^TetrahedronMeshData ---

    @(link_name = "PxCollisionTetrahedronMeshData_getMesh_mut")
    collision_tetrahedron_mesh_data_get_mesh_mut :: proc(self_: ^CollisionTetrahedronMeshData) -> ^TetrahedronMeshData ---

    @(link_name = "PxCollisionTetrahedronMeshData_getData")
    collision_tetrahedron_mesh_data_get_data :: proc(self_: ^CollisionTetrahedronMeshData) -> ^SoftBodyCollisionData ---

    @(link_name = "PxCollisionTetrahedronMeshData_getData_mut")
    collision_tetrahedron_mesh_data_get_data_mut :: proc(self_: ^CollisionTetrahedronMeshData) -> ^SoftBodyCollisionData ---

    @(link_name = "PxCollisionTetrahedronMeshData_release_mut")
    collision_tetrahedron_mesh_data_release_mut :: proc(self_: ^CollisionTetrahedronMeshData) ---

    @(link_name = "PxSimulationTetrahedronMeshData_getMesh_mut")
    simulation_tetrahedron_mesh_data_get_mesh_mut :: proc(self_: ^SimulationTetrahedronMeshData) -> ^TetrahedronMeshData ---

    @(link_name = "PxSimulationTetrahedronMeshData_getData_mut")
    simulation_tetrahedron_mesh_data_get_data_mut :: proc(self_: ^SimulationTetrahedronMeshData) -> ^SoftBodySimulationData ---

    @(link_name = "PxSimulationTetrahedronMeshData_release_mut")
    simulation_tetrahedron_mesh_data_release_mut :: proc(self_: ^SimulationTetrahedronMeshData) ---

    /// Deletes the actor.
    ///
    /// Do not keep a reference to the deleted instance.
    ///
    /// If the actor belongs to a [`PxAggregate`] object, it is automatically removed from the aggregate.
    @(link_name = "PxActor_release_mut")
    actor_release_mut :: proc(self_: ^Actor) ---

    /// Retrieves the type of actor.
    ///
    /// The actor type of the actor.
    @(link_name = "PxActor_getType")
    actor_get_type :: proc(self_: ^Actor) -> ActorType ---

    /// Retrieves the scene which this actor belongs to.
    ///
    /// Owner Scene. NULL if not part of a scene.
    @(link_name = "PxActor_getScene")
    actor_get_scene :: proc(self_: ^Actor) -> ^Scene ---

    /// Sets a name string for the object that can be retrieved with getName().
    ///
    /// This is for debugging and is not used by the SDK. The string is not copied by the SDK,
    /// only the pointer is stored.
    ///
    /// Default:
    /// NULL
    @(link_name = "PxActor_setName_mut")
    actor_set_name_mut :: proc(self_: ^Actor, name: cstring) ---

    /// Retrieves the name string set with setName().
    ///
    /// Name string associated with object.
    @(link_name = "PxActor_getName")
    actor_get_name :: proc(self_: ^Actor) -> cstring ---

    /// Retrieves the axis aligned bounding box enclosing the actor.
    ///
    /// It is not allowed to use this method while the simulation is running (except during PxScene::collide(),
    /// in PxContactModifyCallback or in contact report callbacks).
    ///
    /// The actor's bounding box.
    @(link_name = "PxActor_getWorldBounds")
    actor_get_world_bounds :: proc(self_: ^Actor, inflation: _c.float) -> Bounds3 ---

    /// Raises or clears a particular actor flag.
    ///
    /// See the list of flags [`PxActorFlag`]
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the actor up automatically.
    @(link_name = "PxActor_setActorFlag_mut")
    actor_set_actor_flag_mut :: proc(self_: ^Actor, flag: ActorFlag, value: _c.bool) ---

    /// Sets the actor flags.
    ///
    /// See the list of flags [`PxActorFlag`]
    @(link_name = "PxActor_setActorFlags_mut")
    actor_set_actor_flags_mut :: proc(self_: ^Actor, inFlags: ActorFlags_Set) ---

    /// Reads the PxActor flags.
    ///
    /// See the list of flags [`PxActorFlag`]
    ///
    /// The values of the PxActor flags.
    @(link_name = "PxActor_getActorFlags")
    actor_get_actor_flags :: proc(self_: ^Actor) -> ActorFlags_Set ---

    /// Assigns dynamic actors a dominance group identifier.
    ///
    /// PxDominanceGroup is a 5 bit group identifier (legal range from 0 to 31).
    ///
    /// The PxScene::setDominanceGroupPair() lets you set certain behaviors for pairs of dominance groups.
    /// By default every dynamic actor is created in group 0.
    ///
    /// Default:
    /// 0
    ///
    /// Sleeping:
    /// Changing the dominance group does
    /// NOT
    /// wake the actor up automatically.
    @(link_name = "PxActor_setDominanceGroup_mut")
    actor_set_dominance_group_mut :: proc(self_: ^Actor, dominanceGroup: _c.uint8_t) ---

    /// Retrieves the value set with setDominanceGroup().
    ///
    /// The dominance group of this actor.
    @(link_name = "PxActor_getDominanceGroup")
    actor_get_dominance_group :: proc(self_: ^Actor) -> _c.uint8_t ---

    /// Sets the owner client of an actor.
    ///
    /// This cannot be done once the actor has been placed into a scene.
    ///
    /// Default:
    /// PX_DEFAULT_CLIENT
    @(link_name = "PxActor_setOwnerClient_mut")
    actor_set_owner_client_mut :: proc(self_: ^Actor, inClient: _c.uint8_t) ---

    /// Returns the owner client that was specified at creation time.
    ///
    /// This value cannot be changed once the object is placed into the scene.
    @(link_name = "PxActor_getOwnerClient")
    actor_get_owner_client :: proc(self_: ^Actor) -> _c.uint8_t ---

    /// Retrieves the aggregate the actor might be a part of.
    ///
    /// The aggregate the actor is a part of, or NULL if the actor does not belong to an aggregate.
    @(link_name = "PxActor_getAggregate")
    actor_get_aggregate :: proc(self_: ^Actor) -> ^Aggregate ---

    @(link_name = "phys_PxGetAggregateFilterHint")
    get_aggregate_filter_hint :: proc(type: AggregateType, enableSelfCollision: _c.bool) -> _c.uint32_t ---

    @(link_name = "phys_PxGetAggregateSelfCollisionBit")
    get_aggregate_self_collision_bit :: proc(hint: _c.uint32_t) -> _c.uint32_t ---

    @(link_name = "phys_PxGetAggregateType")
    get_aggregate_type :: proc(hint: _c.uint32_t) -> AggregateType ---

    /// Deletes the aggregate object.
    ///
    /// Deleting the PxAggregate object does not delete the aggregated actors. If the PxAggregate object
    /// belongs to a scene, the aggregated actors are automatically re-inserted in that scene. If you intend
    /// to delete both the PxAggregate and its actors, it is best to release the actors first, then release
    /// the PxAggregate when it is empty.
    @(link_name = "PxAggregate_release_mut")
    aggregate_release_mut :: proc(self_: ^Aggregate) ---

    /// Adds an actor to the aggregate object.
    ///
    /// A warning is output if the total number of actors is reached, or if the incoming actor already belongs
    /// to an aggregate.
    ///
    /// If the aggregate belongs to a scene, adding an actor to the aggregate also adds the actor to that scene.
    ///
    /// If the actor already belongs to a scene, a warning is output and the call is ignored. You need to remove
    /// the actor from the scene first, before adding it to the aggregate.
    ///
    /// When a BVH is provided the actor shapes are grouped together.
    /// The scene query pruning structure inside PhysX SDK will store/update one
    /// bound per actor. The scene queries against such an actor will query actor
    /// bounds and then make a local space query against the provided BVH, which is in actor's local space.
    @(link_name = "PxAggregate_addActor_mut")
    aggregate_add_actor_mut :: proc(self_: ^Aggregate, actor: ^Actor, bvh: ^BVH) -> _c.bool ---

    /// Removes an actor from the aggregate object.
    ///
    /// A warning is output if the incoming actor does not belong to the aggregate. Otherwise the actor is
    /// removed from the aggregate. If the aggregate belongs to a scene, the actor is reinserted in that
    /// scene. If you intend to delete the actor, it is best to call [`PxActor::release`]() directly. That way
    /// the actor will be automatically removed from its aggregate (if any) and not reinserted in a scene.
    @(link_name = "PxAggregate_removeActor_mut")
    aggregate_remove_actor_mut :: proc(self_: ^Aggregate, actor: ^Actor) -> _c.bool ---

    /// Adds an articulation to the aggregate object.
    ///
    /// A warning is output if the total number of actors is reached (every articulation link counts as an actor),
    /// or if the incoming articulation already belongs to an aggregate.
    ///
    /// If the aggregate belongs to a scene, adding an articulation to the aggregate also adds the articulation to that scene.
    ///
    /// If the articulation already belongs to a scene, a warning is output and the call is ignored. You need to remove
    /// the articulation from the scene first, before adding it to the aggregate.
    @(link_name = "PxAggregate_addArticulation_mut")
    aggregate_add_articulation_mut :: proc(self_: ^Aggregate, articulation: ^ArticulationReducedCoordinate) -> _c.bool ---

    /// Removes an articulation from the aggregate object.
    ///
    /// A warning is output if the incoming articulation does not belong to the aggregate. Otherwise the articulation is
    /// removed from the aggregate. If the aggregate belongs to a scene, the articulation is reinserted in that
    /// scene. If you intend to delete the articulation, it is best to call [`PxArticulationReducedCoordinate::release`]() directly. That way
    /// the articulation will be automatically removed from its aggregate (if any) and not reinserted in a scene.
    @(link_name = "PxAggregate_removeArticulation_mut")
    aggregate_remove_articulation_mut :: proc(self_: ^Aggregate, articulation: ^ArticulationReducedCoordinate) -> _c.bool ---

    /// Returns the number of actors contained in the aggregate.
    ///
    /// You can use [`getActors`]() to retrieve the actor pointers.
    ///
    /// Number of actors contained in the aggregate.
    @(link_name = "PxAggregate_getNbActors")
    aggregate_get_nb_actors :: proc(self_: ^Aggregate) -> _c.uint32_t ---

    /// Retrieves max amount of shapes that can be contained in the aggregate.
    ///
    /// Max shape size.
    @(link_name = "PxAggregate_getMaxNbShapes")
    aggregate_get_max_nb_shapes :: proc(self_: ^Aggregate) -> _c.uint32_t ---

    /// Retrieve all actors contained in the aggregate.
    ///
    /// You can retrieve the number of actor pointers by calling [`getNbActors`]()
    ///
    /// Number of actor pointers written to the buffer.
    @(link_name = "PxAggregate_getActors")
    aggregate_get_actors :: proc(self_: ^Aggregate, userBuffer: [^]^Actor, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Retrieves the scene which this aggregate belongs to.
    ///
    /// Owner Scene. NULL if not part of a scene.
    @(link_name = "PxAggregate_getScene_mut")
    aggregate_get_scene_mut :: proc(self_: ^Aggregate) -> ^Scene ---

    /// Retrieves aggregate's self-collision flag.
    ///
    /// self-collision flag
    @(link_name = "PxAggregate_getSelfCollision")
    aggregate_get_self_collision :: proc(self_: ^Aggregate) -> _c.bool ---

    @(link_name = "PxAggregate_getConcreteTypeName")
    aggregate_get_concrete_type_name :: proc(self_: ^Aggregate) -> cstring ---

    @(link_name = "PxConstraintInvMassScale_new")
    constraint_inv_mass_scale_new :: proc() -> ConstraintInvMassScale ---

    @(link_name = "PxConstraintInvMassScale_new_1")
    constraint_inv_mass_scale_new_1 :: proc(lin0: _c.float, ang0: _c.float, lin1: _c.float, ang1: _c.float) -> ConstraintInvMassScale ---

    /// Visualize joint frames
    @(link_name = "PxConstraintVisualizer_visualizeJointFrames_mut")
    constraint_visualizer_visualize_joint_frames_mut :: proc(self_: ^ConstraintVisualizer, #by_ptr parent: Transform, #by_ptr child: Transform) ---

    /// Visualize joint linear limit
    @(link_name = "PxConstraintVisualizer_visualizeLinearLimit_mut")
    constraint_visualizer_visualize_linear_limit_mut :: proc(self_: ^ConstraintVisualizer, #by_ptr t0: Transform, #by_ptr t1: Transform, value: _c.float, active: _c.bool) ---

    /// Visualize joint angular limit
    @(link_name = "PxConstraintVisualizer_visualizeAngularLimit_mut")
    constraint_visualizer_visualize_angular_limit_mut :: proc(self_: ^ConstraintVisualizer, #by_ptr t0: Transform, lower: _c.float, upper: _c.float, active: _c.bool) ---

    /// Visualize limit cone
    @(link_name = "PxConstraintVisualizer_visualizeLimitCone_mut")
    constraint_visualizer_visualize_limit_cone_mut :: proc(self_: ^ConstraintVisualizer, #by_ptr t: Transform, tanQSwingY: _c.float, tanQSwingZ: _c.float, active: _c.bool) ---

    /// Visualize joint double cone
    @(link_name = "PxConstraintVisualizer_visualizeDoubleCone_mut")
    constraint_visualizer_visualize_double_cone_mut :: proc(self_: ^ConstraintVisualizer, #by_ptr t: Transform, angle: _c.float, active: _c.bool) ---

    /// Visualize line
    @(link_name = "PxConstraintVisualizer_visualizeLine_mut")
    constraint_visualizer_visualize_line_mut :: proc(self_: ^ConstraintVisualizer, #by_ptr p0: Vec3, #by_ptr p1: Vec3, color: _c.uint32_t) ---

    /// Pre-simulation data preparation
    /// when the constraint is marked dirty, this function is called at the start of the simulation
    /// step for the SDK to copy the constraint data block.
    @(link_name = "PxConstraintConnector_prepareData_mut")
    constraint_connector_prepare_data_mut :: proc(self_: ^ConstraintConnector) -> rawptr ---

    /// Constraint release callback
    ///
    /// When the SDK deletes a PxConstraint object this function is called by the SDK. In general
    /// custom constraints should not be deleted directly by applications: rather, the constraint
    /// should respond to a release() request by calling PxConstraint::release(), then wait for
    /// this call to release its own resources.
    ///
    /// This function is also called when a PxConstraint object is deleted on cleanup due to
    /// destruction of the PxPhysics object.
    @(link_name = "PxConstraintConnector_onConstraintRelease_mut")
    constraint_connector_on_constraint_release_mut :: proc(self_: ^ConstraintConnector) ---

    /// Center-of-mass shift callback
    ///
    /// This function is called by the SDK when the CoM of one of the actors is moved. Since the
    /// API specifies constraint positions relative to actors, and the constraint shader functions
    /// are supplied with coordinates relative to bodies, some synchronization is usually required
    /// when the application moves an object's center of mass.
    @(link_name = "PxConstraintConnector_onComShift_mut")
    constraint_connector_on_com_shift_mut :: proc(self_: ^ConstraintConnector, actor: _c.uint32_t) ---

    /// Origin shift callback
    ///
    /// This function is called by the SDK when the scene origin gets shifted and allows to adjust
    /// custom data which contains world space transforms.
    ///
    /// If the adjustments affect constraint shader data, it is necessary to call PxConstraint::markDirty()
    /// to make sure that the data gets synced at the beginning of the next simulation step.
    @(link_name = "PxConstraintConnector_onOriginShift_mut")
    constraint_connector_on_origin_shift_mut :: proc(self_: ^ConstraintConnector, #by_ptr shift: Vec3) ---

    /// Obtain a reference to a PxBase interface if the constraint has one.
    ///
    /// If the constraint does not implement the PxBase interface, it should return NULL.
    @(link_name = "PxConstraintConnector_getSerializable_mut")
    constraint_connector_get_serializable_mut :: proc(self_: ^ConstraintConnector) -> ^Base ---

    /// Obtain the pointer to the constraint's constant data
    @(link_name = "PxConstraintConnector_getConstantBlock")
    constraint_connector_get_constant_block :: proc(self_: ^ConstraintConnector) -> rawptr ---

    /// Let the connector know it has been connected to a constraint.
    @(link_name = "PxConstraintConnector_connectToConstraint_mut")
    constraint_connector_connect_to_constraint_mut :: proc(self_: ^ConstraintConnector, anon_param0: ^Constraint) ---

    /// virtual destructor
    @(link_name = "PxConstraintConnector_delete")
    constraint_connector_delete :: proc(self_: ^ConstraintConnector) ---

    @(link_name = "PxSolverBody_new")
    solver_body_new :: proc() -> SolverBody ---

    @(link_name = "PxSolverBodyData_projectVelocity")
    solver_body_data_project_velocity :: proc(self_: ^SolverBodyData, #by_ptr lin: Vec3, #by_ptr ang: Vec3) -> _c.float ---

    @(link_name = "PxSolverConstraintPrepDesc_delete")
    solver_constraint_prep_desc_delete :: proc(self_: ^SolverConstraintPrepDesc) ---

    /// Allocates constraint data. It is the application's responsibility to release this memory after PxSolveConstraints has completed.
    ///
    /// The allocated memory. This address must be 16-byte aligned.
    @(link_name = "PxConstraintAllocator_reserveConstraintData_mut")
    constraint_allocator_reserve_constraint_data_mut :: proc(self_: ^ConstraintAllocator, byteSize: _c.uint32_t) -> ^_c.uint8_t ---

    /// Allocates friction data. Friction data can be retained by the application for a given pair and provided as an input to PxSolverContactDesc to improve simulation stability.
    /// It is the application's responsibility to release this memory. If this memory is released, the application should ensure it does not pass pointers to this memory to PxSolverContactDesc.
    ///
    /// The allocated memory. This address must be 4-byte aligned.
    @(link_name = "PxConstraintAllocator_reserveFrictionData_mut")
    constraint_allocator_reserve_friction_data_mut :: proc(self_: ^ConstraintAllocator, byteSize: _c.uint32_t) -> ^_c.uint8_t ---

    @(link_name = "PxConstraintAllocator_delete")
    constraint_allocator_delete :: proc(self_: ^ConstraintAllocator) ---

    @(link_name = "PxArticulationLimit_new")
    articulation_limit_new :: proc() -> ArticulationLimit ---

    @(link_name = "PxArticulationLimit_new_1")
    articulation_limit_new_1 :: proc(low_: _c.float, high_: _c.float) -> ArticulationLimit ---

    @(link_name = "PxArticulationDrive_new")
    articulation_drive_new :: proc() -> ArticulationDrive ---

    @(link_name = "PxArticulationDrive_new_1")
    articulation_drive_new_1 :: proc(stiffness_: _c.float, damping_: _c.float, maxForce_: _c.float, driveType_: ArticulationDriveType) -> ArticulationDrive ---

    @(link_name = "PxTGSSolverBodyVel_projectVelocity")
    t_g_s_solver_body_vel_project_velocity :: proc(self_: ^TGSSolverBodyVel, #by_ptr lin: Vec3, #by_ptr ang: Vec3) -> _c.float ---

    @(link_name = "PxTGSSolverBodyData_projectVelocity")
    t_g_s_solver_body_data_project_velocity :: proc(self_: ^TGSSolverBodyData, #by_ptr linear: Vec3, #by_ptr angular: Vec3) -> _c.float ---

    @(link_name = "PxTGSSolverConstraintPrepDesc_delete")
    t_g_s_solver_constraint_prep_desc_delete :: proc(self_: ^TGSSolverConstraintPrepDesc) ---

    /// Sets the spring rest length for the sub-tendon from the root to this leaf attachment.
    ///
    /// Setting this on non-leaf attachments has no effect.
    @(link_name = "PxArticulationAttachment_setRestLength_mut")
    articulation_attachment_set_rest_length_mut :: proc(self_: ^ArticulationAttachment, restLength: _c.float) ---

    /// Gets the spring rest length for the sub-tendon from the root to this leaf attachment.
    ///
    /// The rest length.
    @(link_name = "PxArticulationAttachment_getRestLength")
    articulation_attachment_get_rest_length :: proc(self_: ^ArticulationAttachment) -> _c.float ---

    /// Sets the low and high limit on the length of the sub-tendon from the root to this leaf attachment.
    ///
    /// Setting this on non-leaf attachments has no effect.
    @(link_name = "PxArticulationAttachment_setLimitParameters_mut")
    articulation_attachment_set_limit_parameters_mut :: proc(self_: ^ArticulationAttachment, #by_ptr parameters: ArticulationTendonLimit) ---

    /// Gets the low and high limit on the length of the sub-tendon from the root to this leaf attachment.
    ///
    /// Struct with the low and high limit.
    @(link_name = "PxArticulationAttachment_getLimitParameters")
    articulation_attachment_get_limit_parameters :: proc(self_: ^ArticulationAttachment) -> ArticulationTendonLimit ---

    /// Sets the attachment's relative offset in the link actor frame.
    @(link_name = "PxArticulationAttachment_setRelativeOffset_mut")
    articulation_attachment_set_relative_offset_mut :: proc(self_: ^ArticulationAttachment, #by_ptr offset: Vec3) ---

    /// Gets the attachment's relative offset in the link actor frame.
    ///
    /// The relative offset in the link actor frame.
    @(link_name = "PxArticulationAttachment_getRelativeOffset")
    articulation_attachment_get_relative_offset :: proc(self_: ^ArticulationAttachment) -> Vec3 ---

    /// Sets the attachment coefficient.
    @(link_name = "PxArticulationAttachment_setCoefficient_mut")
    articulation_attachment_set_coefficient_mut :: proc(self_: ^ArticulationAttachment, coefficient: _c.float) ---

    /// Gets the attachment coefficient.
    ///
    /// The scale that the distance between this attachment and its parent is multiplied by when summing up the spatial tendon's length.
    @(link_name = "PxArticulationAttachment_getCoefficient")
    articulation_attachment_get_coefficient :: proc(self_: ^ArticulationAttachment) -> _c.float ---

    /// Gets the articulation link.
    ///
    /// The articulation link that this attachment is attached to.
    @(link_name = "PxArticulationAttachment_getLink")
    articulation_attachment_get_link :: proc(self_: ^ArticulationAttachment) -> ^ArticulationLink ---

    /// Gets the parent attachment.
    ///
    /// The parent attachment.
    @(link_name = "PxArticulationAttachment_getParent")
    articulation_attachment_get_parent :: proc(self_: ^ArticulationAttachment) -> ^ArticulationAttachment ---

    /// Indicates that this attachment is a leaf, and thus defines a sub-tendon from the root to this attachment.
    ///
    /// True: This attachment is a leaf and has zero children; False: Not a leaf.
    @(link_name = "PxArticulationAttachment_isLeaf")
    articulation_attachment_is_leaf :: proc(self_: ^ArticulationAttachment) -> _c.bool ---

    /// Gets the spatial tendon that the attachment is a part of.
    ///
    /// The tendon.
    @(link_name = "PxArticulationAttachment_getTendon")
    articulation_attachment_get_tendon :: proc(self_: ^ArticulationAttachment) -> ^ArticulationSpatialTendon ---

    /// Releases the attachment.
    ///
    /// Releasing the attachment is not allowed while the articulation is in a scene. In order to
    /// release the attachment, remove and then re-add the articulation to the scene.
    @(link_name = "PxArticulationAttachment_release_mut")
    articulation_attachment_release_mut :: proc(self_: ^ArticulationAttachment) ---

    /// Returns the string name of the dynamic type.
    ///
    /// The string name.
    @(link_name = "PxArticulationAttachment_getConcreteTypeName")
    articulation_attachment_get_concrete_type_name :: proc(self_: ^ArticulationAttachment) -> cstring ---

    /// Sets the tendon joint coefficient.
    ///
    /// RecipCoefficient is commonly expected to be 1/coefficient, but it can be set to different values to tune behavior; for example, zero can be used to
    /// have a joint axis only participate in the length computation of the tendon, but not have any tendon force applied to it.
    @(link_name = "PxArticulationTendonJoint_setCoefficient_mut")
    articulation_tendon_joint_set_coefficient_mut :: proc(self_: ^ArticulationTendonJoint, axis: ArticulationAxis, coefficient: _c.float, recipCoefficient: _c.float) ---

    /// Gets the tendon joint coefficient.
    @(link_name = "PxArticulationTendonJoint_getCoefficient")
    articulation_tendon_joint_get_coefficient :: proc(self_: ^ArticulationTendonJoint, axis: ^ArticulationAxis, coefficient: ^_c.float, recipCoefficient: ^_c.float) ---

    /// Gets the articulation link.
    ///
    /// The articulation link (and its incoming joint in particular) that this tendon joint is associated with.
    @(link_name = "PxArticulationTendonJoint_getLink")
    articulation_tendon_joint_get_link :: proc(self_: ^ArticulationTendonJoint) -> ^ArticulationLink ---

    /// Gets the parent tendon joint.
    ///
    /// The parent tendon joint.
    @(link_name = "PxArticulationTendonJoint_getParent")
    articulation_tendon_joint_get_parent :: proc(self_: ^ArticulationTendonJoint) -> ^ArticulationTendonJoint ---

    /// Gets the tendon that the joint is a part of.
    ///
    /// The tendon.
    @(link_name = "PxArticulationTendonJoint_getTendon")
    articulation_tendon_joint_get_tendon :: proc(self_: ^ArticulationTendonJoint) -> ^ArticulationFixedTendon ---

    /// Releases a tendon joint.
    ///
    /// Releasing a tendon joint is not allowed while the articulation is in a scene. In order to
    /// release the joint, remove and then re-add the articulation to the scene.
    @(link_name = "PxArticulationTendonJoint_release_mut")
    articulation_tendon_joint_release_mut :: proc(self_: ^ArticulationTendonJoint) ---

    /// Returns the string name of the dynamic type.
    ///
    /// The string name.
    @(link_name = "PxArticulationTendonJoint_getConcreteTypeName")
    articulation_tendon_joint_get_concrete_type_name :: proc(self_: ^ArticulationTendonJoint) -> cstring ---

    /// Sets the spring stiffness term acting on the tendon length.
    @(link_name = "PxArticulationTendon_setStiffness_mut")
    articulation_tendon_set_stiffness_mut :: proc(self_: ^ArticulationTendon, stiffness: _c.float) ---

    /// Gets the spring stiffness of the tendon.
    ///
    /// The spring stiffness.
    @(link_name = "PxArticulationTendon_getStiffness")
    articulation_tendon_get_stiffness :: proc(self_: ^ArticulationTendon) -> _c.float ---

    /// Sets the damping term acting both on the tendon length and tendon-length limits.
    @(link_name = "PxArticulationTendon_setDamping_mut")
    articulation_tendon_set_damping_mut :: proc(self_: ^ArticulationTendon, damping: _c.float) ---

    /// Gets the damping term acting both on the tendon length and tendon-length limits.
    ///
    /// The damping term.
    @(link_name = "PxArticulationTendon_getDamping")
    articulation_tendon_get_damping :: proc(self_: ^ArticulationTendon) -> _c.float ---

    /// Sets the limit stiffness term acting on the tendon's length limits.
    ///
    /// For spatial tendons, this parameter applies to all its leaf attachments / sub-tendons.
    @(link_name = "PxArticulationTendon_setLimitStiffness_mut")
    articulation_tendon_set_limit_stiffness_mut :: proc(self_: ^ArticulationTendon, stiffness: _c.float) ---

    /// Gets the limit stiffness term acting on the tendon's length limits.
    ///
    /// For spatial tendons, this parameter applies to all its leaf attachments / sub-tendons.
    ///
    /// The limit stiffness term.
    @(link_name = "PxArticulationTendon_getLimitStiffness")
    articulation_tendon_get_limit_stiffness :: proc(self_: ^ArticulationTendon) -> _c.float ---

    /// Sets the length offset term for the tendon.
    ///
    /// An offset defines an amount to be added to the accumulated length computed for the tendon. It allows the
    /// application to actuate the tendon by shortening or lengthening it.
    @(link_name = "PxArticulationTendon_setOffset_mut")
    articulation_tendon_set_offset_mut :: proc(self_: ^ArticulationTendon, offset: _c.float, autowake: _c.bool) ---

    /// Gets the length offset term for the tendon.
    ///
    /// The offset term.
    @(link_name = "PxArticulationTendon_getOffset")
    articulation_tendon_get_offset :: proc(self_: ^ArticulationTendon) -> _c.float ---

    /// Gets the articulation that the tendon is a part of.
    ///
    /// The articulation.
    @(link_name = "PxArticulationTendon_getArticulation")
    articulation_tendon_get_articulation :: proc(self_: ^ArticulationTendon) -> ^ArticulationReducedCoordinate ---

    /// Releases a tendon to remove it from the articulation and free its associated memory.
    ///
    /// When an articulation is released, its attached tendons are automatically released.
    ///
    /// Releasing a tendon is not allowed while the articulation is in a scene. In order to
    /// release the tendon, remove and then re-add the articulation to the scene.
    @(link_name = "PxArticulationTendon_release_mut")
    articulation_tendon_release_mut :: proc(self_: ^ArticulationTendon) ---

    /// Creates an articulation attachment and adds it to the list of children in the parent attachment.
    ///
    /// Creating an attachment is not allowed while the articulation is in a scene. In order to
    /// add the attachment, remove and then re-add the articulation to the scene.
    ///
    /// The newly-created attachment if creation was successful, otherwise a null pointer.
    @(link_name = "PxArticulationSpatialTendon_createAttachment_mut")
    articulation_spatial_tendon_create_attachment_mut :: proc(self_: ^ArticulationSpatialTendon, parent: ^ArticulationAttachment, coefficient: _c.float, relativeOffset: Vec3, link: ^ArticulationLink) -> ^ArticulationAttachment ---

    /// Fills a user-provided buffer of attachment pointers with the set of attachments.
    ///
    /// The number of attachments that were filled into the user buffer.
    @(link_name = "PxArticulationSpatialTendon_getAttachments")
    articulation_spatial_tendon_get_attachments :: proc(self_: ^ArticulationSpatialTendon, userBuffer: [^]^ArticulationAttachment, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Returns the number of attachments in the tendon.
    ///
    /// The number of attachments.
    @(link_name = "PxArticulationSpatialTendon_getNbAttachments")
    articulation_spatial_tendon_get_nb_attachments :: proc(self_: ^ArticulationSpatialTendon) -> _c.uint32_t ---

    /// Returns the string name of the dynamic type.
    ///
    /// The string name.
    @(link_name = "PxArticulationSpatialTendon_getConcreteTypeName")
    articulation_spatial_tendon_get_concrete_type_name :: proc(self_: ^ArticulationSpatialTendon) -> cstring ---

    /// Creates an articulation tendon joint and adds it to the list of children in the parent tendon joint.
    ///
    /// Creating a tendon joint is not allowed while the articulation is in a scene. In order to
    /// add the joint, remove and then re-add the articulation to the scene.
    ///
    /// The newly-created tendon joint if creation was successful, otherwise a null pointer.
    ///
    /// - The axis motion must not be configured as PxArticulationMotion::eLOCKED.
    /// - The axis cannot be part of a fixed joint, i.e. joint configured as PxArticulationJointType::eFIX.
    @(link_name = "PxArticulationFixedTendon_createTendonJoint_mut")
    articulation_fixed_tendon_create_tendon_joint_mut :: proc(self_: ^ArticulationFixedTendon, parent: ^ArticulationTendonJoint, axis: ArticulationAxis, coefficient: _c.float, recipCoefficient: _c.float, link: ^ArticulationLink) -> ^ArticulationTendonJoint ---

    /// Fills a user-provided buffer of tendon-joint pointers with the set of tendon joints.
    ///
    /// The number of tendon joints filled into the user buffer.
    @(link_name = "PxArticulationFixedTendon_getTendonJoints")
    articulation_fixed_tendon_get_tendon_joints :: proc(self_: ^ArticulationFixedTendon, userBuffer: [^]^ArticulationTendonJoint, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Returns the number of tendon joints in the tendon.
    ///
    /// The number of tendon joints.
    @(link_name = "PxArticulationFixedTendon_getNbTendonJoints")
    articulation_fixed_tendon_get_nb_tendon_joints :: proc(self_: ^ArticulationFixedTendon) -> _c.uint32_t ---

    /// Sets the spring rest length of the tendon.
    ///
    /// The accumulated "length" of a fixed tendon is a linear combination of the joint axis positions that the tendon is
    /// associated with, scaled by the respective tendon joints' coefficients. As such, when the joint positions of all
    /// joints are zero, the accumulated length of a fixed tendon is zero.
    ///
    /// The spring of the tendon is not exerting any force on the articulation when the rest length is equal to the
    /// tendon's accumulated length plus the tendon offset.
    @(link_name = "PxArticulationFixedTendon_setRestLength_mut")
    articulation_fixed_tendon_set_rest_length_mut :: proc(self_: ^ArticulationFixedTendon, restLength: _c.float) ---

    /// Gets the spring rest length of the tendon.
    ///
    /// The spring rest length of the tendon.
    @(link_name = "PxArticulationFixedTendon_getRestLength")
    articulation_fixed_tendon_get_rest_length :: proc(self_: ^ArticulationFixedTendon) -> _c.float ---

    /// Sets the low and high limit on the length of the tendon.
    ///
    /// The limits, together with the damping and limit stiffness parameters, act on the accumulated length of the tendon.
    @(link_name = "PxArticulationFixedTendon_setLimitParameters_mut")
    articulation_fixed_tendon_set_limit_parameters_mut :: proc(self_: ^ArticulationFixedTendon, #by_ptr parameter: ArticulationTendonLimit) ---

    /// Gets the low and high limit on the length of the tendon.
    ///
    /// Struct with the low and high limit.
    @(link_name = "PxArticulationFixedTendon_getLimitParameters")
    articulation_fixed_tendon_get_limit_parameters :: proc(self_: ^ArticulationFixedTendon) -> ArticulationTendonLimit ---

    /// Returns the string name of the dynamic type.
    ///
    /// The string name.
    @(link_name = "PxArticulationFixedTendon_getConcreteTypeName")
    articulation_fixed_tendon_get_concrete_type_name :: proc(self_: ^ArticulationFixedTendon) -> cstring ---

    @(link_name = "PxArticulationCache_new")
    articulation_cache_new :: proc() -> ArticulationCache ---

    /// Releases an articulation cache.
    @(link_name = "PxArticulationCache_release_mut")
    articulation_cache_release_mut :: proc(self_: ^ArticulationCache) ---

    /// Releases the sensor.
    ///
    /// Releasing a sensor is not allowed while the articulation is in a scene. In order to
    /// release a sensor, remove and then re-add the articulation to the scene.
    @(link_name = "PxArticulationSensor_release_mut")
    articulation_sensor_release_mut :: proc(self_: ^ArticulationSensor) ---

    /// Returns the spatial force in the local frame of the sensor.
    ///
    /// The spatial force.
    ///
    /// This call is not allowed while the simulation is running except in a split simulation during [`PxScene::collide`]() and up to #PxScene::advance(),
    /// and in PxContactModifyCallback or in contact report callbacks.
    @(link_name = "PxArticulationSensor_getForces")
    articulation_sensor_get_forces :: proc(self_: ^ArticulationSensor) -> SpatialForce ---

    /// Returns the relative pose between this sensor and the body frame of the link that the sensor is attached to.
    ///
    /// The link body frame is at the center of mass and aligned with the principal axes of inertia, see PxRigidBody::getCMassLocalPose.
    ///
    /// The transform link body frame -> sensor frame.
    @(link_name = "PxArticulationSensor_getRelativePose")
    articulation_sensor_get_relative_pose :: proc(self_: ^ArticulationSensor) -> Transform ---

    /// Sets the relative pose between this sensor and the body frame of the link that the sensor is attached to.
    ///
    /// The link body frame is at the center of mass and aligned with the principal axes of inertia, see PxRigidBody::getCMassLocalPose.
    ///
    /// Setting the sensor relative pose is not allowed while the articulation is in a scene. In order to
    /// set the pose, remove and then re-add the articulation to the scene.
    @(link_name = "PxArticulationSensor_setRelativePose_mut")
    articulation_sensor_set_relative_pose_mut :: proc(self_: ^ArticulationSensor, #by_ptr pose: Transform) ---

    /// Returns the link that this sensor is attached to.
    ///
    /// A pointer to the link.
    @(link_name = "PxArticulationSensor_getLink")
    articulation_sensor_get_link :: proc(self_: ^ArticulationSensor) -> ^ArticulationLink ---

    /// Returns the index of this sensor inside the articulation.
    ///
    /// The return value is only valid for sensors attached to articulations that are in a scene.
    ///
    /// The low-level index, or 0xFFFFFFFF if the articulation is not in a scene.
    @(link_name = "PxArticulationSensor_getIndex")
    articulation_sensor_get_index :: proc(self_: ^ArticulationSensor) -> _c.uint32_t ---

    /// Returns the articulation that this sensor is part of.
    ///
    /// A pointer to the articulation.
    @(link_name = "PxArticulationSensor_getArticulation")
    articulation_sensor_get_articulation :: proc(self_: ^ArticulationSensor) -> ^ArticulationReducedCoordinate ---

    /// Returns the sensor's flags.
    ///
    /// The current set of flags of the sensor.
    @(link_name = "PxArticulationSensor_getFlags")
    articulation_sensor_get_flags :: proc(self_: ^ArticulationSensor) -> ArticulationSensorFlags_Set ---

    /// Sets a flag of the sensor.
    ///
    /// Setting the sensor flags is not allowed while the articulation is in a scene. In order to
    /// set the flags, remove and then re-add the articulation to the scene.
    @(link_name = "PxArticulationSensor_setFlag_mut")
    articulation_sensor_set_flag_mut :: proc(self_: ^ArticulationSensor, flag: ArticulationSensorFlag, enabled: _c.bool) ---

    /// Returns the string name of the dynamic type.
    ///
    /// The string name.
    @(link_name = "PxArticulationSensor_getConcreteTypeName")
    articulation_sensor_get_concrete_type_name :: proc(self_: ^ArticulationSensor) -> cstring ---

    /// Returns the scene which this articulation belongs to.
    ///
    /// Owner Scene. NULL if not part of a scene.
    @(link_name = "PxArticulationReducedCoordinate_getScene")
    articulation_reduced_coordinate_get_scene :: proc(self_: ^ArticulationReducedCoordinate) -> ^Scene ---

    /// Sets the solver iteration counts for the articulation.
    ///
    /// The solver iteration count determines how accurately contacts, drives, and limits are resolved.
    /// Setting a higher position iteration count may therefore help in scenarios where the articulation
    /// is subject to many constraints; for example, a manipulator articulation with drives and joint limits
    /// that is grasping objects, or several such articulations interacting through contacts. Other situations
    /// where higher position iterations may improve simulation fidelity are: large mass ratios within the
    /// articulation or between the articulation and an object in contact with it; or strong drives in the
    /// articulation being used to manipulate a light object.
    ///
    /// If intersecting bodies are being depenetrated too violently, increase the number of velocity
    /// iterations. More velocity iterations will drive the relative exit velocity of the intersecting
    /// objects closer to the correct value given the restitution.
    ///
    /// This call may not be made during simulation.
    @(link_name = "PxArticulationReducedCoordinate_setSolverIterationCounts_mut")
    articulation_reduced_coordinate_set_solver_iteration_counts_mut :: proc(self_: ^ArticulationReducedCoordinate, minPositionIters: _c.uint32_t, minVelocityIters: _c.uint32_t) ---

    /// Returns the solver iteration counts.
    @(link_name = "PxArticulationReducedCoordinate_getSolverIterationCounts")
    articulation_reduced_coordinate_get_solver_iteration_counts :: proc(self_: ^ArticulationReducedCoordinate, minPositionIters: ^_c.uint32_t, minVelocityIters: ^_c.uint32_t) ---

    /// Returns true if this articulation is sleeping.
    ///
    /// When an actor does not move for a period of time, it is no longer simulated in order to save time. This state
    /// is called sleeping. However, because the object automatically wakes up when it is either touched by an awake object,
    /// or a sleep-affecting property is changed by the user, the entire sleep mechanism should be transparent to the user.
    ///
    /// An articulation can only go to sleep if all links are ready for sleeping. An articulation is guaranteed to be awake
    /// if at least one of the following holds:
    ///
    /// The wake counter is positive (see [`setWakeCounter`]()).
    ///
    /// The linear or angular velocity of any link is non-zero.
    ///
    /// A non-zero force or torque has been applied to the articulation or any of its links.
    ///
    /// If an articulation is sleeping, the following state is guaranteed:
    ///
    /// The wake counter is zero.
    ///
    /// The linear and angular velocity of all links is zero.
    ///
    /// There is no force update pending.
    ///
    /// When an articulation gets inserted into a scene, it will be considered asleep if all the points above hold, else it will
    /// be treated as awake.
    ///
    /// If an articulation is asleep after the call to [`PxScene::fetchResults`]() returns, it is guaranteed that the poses of the
    /// links were not changed. You can use this information to avoid updating the transforms of associated objects.
    ///
    /// True if the articulation is sleeping.
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation,
    /// except in a split simulation in-between [`PxScene::fetchCollision`] and #PxScene::advance.
    @(link_name = "PxArticulationReducedCoordinate_isSleeping")
    articulation_reduced_coordinate_is_sleeping :: proc(self_: ^ArticulationReducedCoordinate) -> _c.bool ---

    /// Sets the mass-normalized energy threshold below which the articulation may go to sleep.
    ///
    /// The articulation will sleep if the energy of each link is below this threshold.
    ///
    /// This call may not be made during simulation.
    @(link_name = "PxArticulationReducedCoordinate_setSleepThreshold_mut")
    articulation_reduced_coordinate_set_sleep_threshold_mut :: proc(self_: ^ArticulationReducedCoordinate, threshold: _c.float) ---

    /// Returns the mass-normalized energy below which the articulation may go to sleep.
    ///
    /// The energy threshold for sleeping.
    @(link_name = "PxArticulationReducedCoordinate_getSleepThreshold")
    articulation_reduced_coordinate_get_sleep_threshold :: proc(self_: ^ArticulationReducedCoordinate) -> _c.float ---

    /// Sets the mass-normalized kinetic energy threshold below which the articulation may participate in stabilization.
    ///
    /// Articulations whose kinetic energy divided by their mass is above this threshold will not participate in stabilization.
    ///
    /// This value has no effect if PxSceneFlag::eENABLE_STABILIZATION was not enabled on the PxSceneDesc.
    ///
    /// Default:
    /// 0.01 * PxTolerancesScale::speed * PxTolerancesScale::speed
    ///
    /// This call may not be made during simulation.
    @(link_name = "PxArticulationReducedCoordinate_setStabilizationThreshold_mut")
    articulation_reduced_coordinate_set_stabilization_threshold_mut :: proc(self_: ^ArticulationReducedCoordinate, threshold: _c.float) ---

    /// Returns the mass-normalized kinetic energy below which the articulation may participate in stabilization.
    ///
    /// Articulations whose kinetic energy divided by their mass is above this threshold will not participate in stabilization.
    ///
    /// The energy threshold for participating in stabilization.
    @(link_name = "PxArticulationReducedCoordinate_getStabilizationThreshold")
    articulation_reduced_coordinate_get_stabilization_threshold :: proc(self_: ^ArticulationReducedCoordinate) -> _c.float ---

    /// Sets the wake counter for the articulation in seconds.
    ///
    /// - The wake counter value determines the minimum amount of time until the articulation can be put to sleep.
    /// - An articulation will not be put to sleep if the energy is above the specified threshold (see [`setSleepThreshold`]())
    /// or if other awake objects are touching it.
    /// - Passing in a positive value will wake up the articulation automatically.
    ///
    /// Default:
    /// 0.4s (which corresponds to 20 frames for a time step of 0.02s)
    ///
    /// This call may not be made during simulation, except in a split simulation in-between [`PxScene::fetchCollision`] and #PxScene::advance.
    @(link_name = "PxArticulationReducedCoordinate_setWakeCounter_mut")
    articulation_reduced_coordinate_set_wake_counter_mut :: proc(self_: ^ArticulationReducedCoordinate, wakeCounterValue: _c.float) ---

    /// Returns the wake counter of the articulation in seconds.
    ///
    /// The wake counter of the articulation in seconds.
    ///
    /// This call may not be made during simulation, except in a split simulation in-between [`PxScene::fetchCollision`] and #PxScene::advance.
    @(link_name = "PxArticulationReducedCoordinate_getWakeCounter")
    articulation_reduced_coordinate_get_wake_counter :: proc(self_: ^ArticulationReducedCoordinate) -> _c.float ---

    /// Wakes up the articulation if it is sleeping.
    ///
    /// - The articulation will get woken up and might cause other touching objects to wake up as well during the next simulation step.
    /// - This will set the wake counter of the articulation to the value specified in [`PxSceneDesc::wakeCounterResetValue`].
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation,
    /// except in a split simulation in-between [`PxScene::fetchCollision`] and #PxScene::advance.
    @(link_name = "PxArticulationReducedCoordinate_wakeUp_mut")
    articulation_reduced_coordinate_wake_up_mut :: proc(self_: ^ArticulationReducedCoordinate) ---

    /// Forces the articulation to sleep.
    ///
    /// - The articulation will stay asleep during the next simulation step if not touched by another non-sleeping actor.
    /// - This will set any applied force, the velocity, and the wake counter of all bodies in the articulation to zero.
    ///
    /// This call may not be made during simulation, and may only be made on articulations that are in a scene.
    @(link_name = "PxArticulationReducedCoordinate_putToSleep_mut")
    articulation_reduced_coordinate_put_to_sleep_mut :: proc(self_: ^ArticulationReducedCoordinate) ---

    /// Sets the limit on the magnitude of the linear velocity of the articulation's center of mass.
    ///
    /// - The limit acts on the linear velocity of the entire articulation. The velocity is calculated from the total momentum
    /// and the spatial inertia of the articulation.
    /// - The limit only applies to floating-base articulations.
    /// - A benefit of the COM velocity limit is that it is evenly applied to the whole articulation, which results in fewer visual
    /// artifacts compared to link rigid-body damping or joint-velocity limits. However, these per-link or per-degree-of-freedom
    /// limits may still help avoid numerical issues.
    ///
    /// This call may not be made during simulation.
    @(link_name = "PxArticulationReducedCoordinate_setMaxCOMLinearVelocity_mut")
    articulation_reduced_coordinate_set_max_c_o_m_linear_velocity_mut :: proc(self_: ^ArticulationReducedCoordinate, maxLinearVelocity: _c.float) ---

    /// Gets the limit on the magnitude of the linear velocity of the articulation's center of mass.
    ///
    /// The maximal linear velocity magnitude.
    @(link_name = "PxArticulationReducedCoordinate_getMaxCOMLinearVelocity")
    articulation_reduced_coordinate_get_max_c_o_m_linear_velocity :: proc(self_: ^ArticulationReducedCoordinate) -> _c.float ---

    /// Sets the limit on the magnitude of the angular velocity at the articulation's center of mass.
    ///
    /// - The limit acts on the angular velocity of the entire articulation. The velocity is calculated from the total momentum
    /// and the spatial inertia of the articulation.
    /// - The limit only applies to floating-base articulations.
    /// - A benefit of the COM velocity limit is that it is evenly applied to the whole articulation, which results in fewer visual
    /// artifacts compared to link rigid-body damping or joint-velocity limits. However, these per-link or per-degree-of-freedom
    /// limits may still help avoid numerical issues.
    ///
    /// This call may not be made during simulation.
    @(link_name = "PxArticulationReducedCoordinate_setMaxCOMAngularVelocity_mut")
    articulation_reduced_coordinate_set_max_c_o_m_angular_velocity_mut :: proc(self_: ^ArticulationReducedCoordinate, maxAngularVelocity: _c.float) ---

    /// Gets the limit on the magnitude of the angular velocity at the articulation's center of mass.
    ///
    /// The maximal angular velocity magnitude.
    @(link_name = "PxArticulationReducedCoordinate_getMaxCOMAngularVelocity")
    articulation_reduced_coordinate_get_max_c_o_m_angular_velocity :: proc(self_: ^ArticulationReducedCoordinate) -> _c.float ---

    /// Adds a link to the articulation with default attribute values.
    ///
    /// The new link, or NULL if the link cannot be created.
    ///
    /// Creating a link is not allowed while the articulation is in a scene. In order to add a link,
    /// remove and then re-add the articulation to the scene.
    @(link_name = "PxArticulationReducedCoordinate_createLink_mut")
    articulation_reduced_coordinate_create_link_mut :: proc(self_: ^ArticulationReducedCoordinate, parent: ^ArticulationLink, #by_ptr pose: Transform) -> ^ArticulationLink ---

    /// Releases the articulation, and all its links and corresponding joints.
    ///
    /// Attached sensors and tendons are released automatically when the articulation is released.
    ///
    /// This call may not be made during simulation.
    @(link_name = "PxArticulationReducedCoordinate_release_mut")
    articulation_reduced_coordinate_release_mut :: proc(self_: ^ArticulationReducedCoordinate) ---

    /// Returns the number of links in the articulation.
    ///
    /// The number of links.
    @(link_name = "PxArticulationReducedCoordinate_getNbLinks")
    articulation_reduced_coordinate_get_nb_links :: proc(self_: ^ArticulationReducedCoordinate) -> _c.uint32_t ---

    /// Returns the set of links in the articulation in the order that they were added to the articulation using createLink.
    ///
    /// The number of links written into the buffer.
    @(link_name = "PxArticulationReducedCoordinate_getLinks")
    articulation_reduced_coordinate_get_links :: proc(self_: ^ArticulationReducedCoordinate, userBuffer: [^]^ArticulationLink, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Returns the number of shapes in the articulation.
    ///
    /// The number of shapes.
    @(link_name = "PxArticulationReducedCoordinate_getNbShapes")
    articulation_reduced_coordinate_get_nb_shapes :: proc(self_: ^ArticulationReducedCoordinate) -> _c.uint32_t ---

    /// Sets a name string for the articulation that can be retrieved with getName().
    ///
    /// This is for debugging and is not used by the SDK. The string is not copied by the SDK,
    /// only the pointer is stored.
    @(link_name = "PxArticulationReducedCoordinate_setName_mut")
    articulation_reduced_coordinate_set_name_mut :: proc(self_: ^ArticulationReducedCoordinate, name: cstring) ---

    /// Returns the name string set with setName().
    ///
    /// Name string associated with the articulation.
    @(link_name = "PxArticulationReducedCoordinate_getName")
    articulation_reduced_coordinate_get_name :: proc(self_: ^ArticulationReducedCoordinate) -> cstring ---

    /// Returns the axis-aligned bounding box enclosing the articulation.
    ///
    /// The articulation's bounding box.
    ///
    /// It is not allowed to use this method while the simulation is running, except in a split simulation
    /// during [`PxScene::collide`]() and up to #PxScene::advance(), and in PxContactModifyCallback or in contact report callbacks.
    @(link_name = "PxArticulationReducedCoordinate_getWorldBounds")
    articulation_reduced_coordinate_get_world_bounds :: proc(self_: ^ArticulationReducedCoordinate, inflation: _c.float) -> Bounds3 ---

    /// Returns the aggregate the articulation might be a part of.
    ///
    /// The aggregate the articulation is a part of, or NULL if the articulation does not belong to an aggregate.
    @(link_name = "PxArticulationReducedCoordinate_getAggregate")
    articulation_reduced_coordinate_get_aggregate :: proc(self_: ^ArticulationReducedCoordinate) -> ^Aggregate ---

    /// Sets flags on the articulation.
    ///
    /// This call may not be made during simulation.
    @(link_name = "PxArticulationReducedCoordinate_setArticulationFlags_mut")
    articulation_reduced_coordinate_set_articulation_flags_mut :: proc(self_: ^ArticulationReducedCoordinate, flags: ArticulationFlags_Set) ---

    /// Raises or clears a flag on the articulation.
    ///
    /// This call may not be made during simulation.
    @(link_name = "PxArticulationReducedCoordinate_setArticulationFlag_mut")
    articulation_reduced_coordinate_set_articulation_flag_mut :: proc(self_: ^ArticulationReducedCoordinate, flag: ArticulationFlag, value: _c.bool) ---

    /// Returns the articulation's flags.
    ///
    /// The flags.
    @(link_name = "PxArticulationReducedCoordinate_getArticulationFlags")
    articulation_reduced_coordinate_get_articulation_flags :: proc(self_: ^ArticulationReducedCoordinate) -> ArticulationFlags_Set ---

    /// Returns the total number of joint degrees-of-freedom (DOFs) of the articulation.
    ///
    /// - The six DOFs of the base of a floating-base articulation are not included in this count.
    /// - Example: Both a fixed-base and a floating-base double-pendulum with two revolute joints will have getDofs() == 2.
    /// - The return value is only valid for articulations that are in a scene.
    ///
    /// The number of joint DOFs, or 0xFFFFFFFF if the articulation is not in a scene.
    @(link_name = "PxArticulationReducedCoordinate_getDofs")
    articulation_reduced_coordinate_get_dofs :: proc(self_: ^ArticulationReducedCoordinate) -> _c.uint32_t ---

    /// Creates an articulation cache that can be used to read and write internal articulation data.
    ///
    /// - When the structure of the articulation changes (e.g. adding a link or sensor) after the cache was created,
    /// the cache needs to be released and recreated.
    /// - Free the memory allocated for the cache by calling the release() method on the cache.
    /// - Caches can only be created by articulations that are in a scene.
    ///
    /// The cache, or NULL if the articulation is not in a scene.
    @(link_name = "PxArticulationReducedCoordinate_createCache")
    articulation_reduced_coordinate_create_cache :: proc(self_: ^ArticulationReducedCoordinate) -> ^ArticulationCache ---

    /// Returns the size of the articulation cache in bytes.
    ///
    /// - The size does not include: the user-allocated memory for the coefficient matrix or lambda values;
    /// the scratch-related memory/members; and the cache version. See comment in [`PxArticulationCache`].
    /// - The return value is only valid for articulations that are in a scene.
    ///
    /// The byte size of the cache, or 0xFFFFFFFF if the articulation is not in a scene.
    @(link_name = "PxArticulationReducedCoordinate_getCacheDataSize")
    articulation_reduced_coordinate_get_cache_data_size :: proc(self_: ^ArticulationReducedCoordinate) -> _c.uint32_t ---

    /// Zeroes all data in the articulation cache, except user-provided and scratch memory, and cache version.
    ///
    /// This call may only be made on articulations that are in a scene.
    @(link_name = "PxArticulationReducedCoordinate_zeroCache")
    articulation_reduced_coordinate_zero_cache :: proc(self_: ^ArticulationReducedCoordinate, cache: ^ArticulationCache) ---

    /// Applies the data in the cache to the articulation.
    ///
    /// This call wakes the articulation if it is sleeping, and the autowake parameter is true (default) or:
    /// - a nonzero joint velocity is applied or
    /// - a nonzero joint force is applied or
    /// - a nonzero root velocity is applied
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
    @(link_name = "PxArticulationReducedCoordinate_applyCache_mut")
    articulation_reduced_coordinate_apply_cache_mut :: proc(self_: ^ArticulationReducedCoordinate, cache: ^ArticulationCache, flags: ArticulationCacheFlags_Set, autowake: _c.bool) ---

    /// Copies internal data of the articulation to the cache.
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
    @(link_name = "PxArticulationReducedCoordinate_copyInternalStateToCache")
    articulation_reduced_coordinate_copy_internal_state_to_cache :: proc(self_: ^ArticulationReducedCoordinate, cache: ^ArticulationCache, flags: ArticulationCacheFlags_Set) ---

    /// Converts maximal-coordinate joint DOF data to reduced coordinates.
    ///
    /// - Indexing into the maximal joint DOF data is via the link's low-level index minus 1 (the root link is not included).
    /// - The reduced-coordinate data follows the cache indexing convention, see PxArticulationCache::jointVelocity.
    ///
    /// The articulation must be in a scene.
    @(link_name = "PxArticulationReducedCoordinate_packJointData")
    articulation_reduced_coordinate_pack_joint_data :: proc(self_: ^ArticulationReducedCoordinate, maximum: ^_c.float, reduced: ^_c.float) ---

    /// Converts reduced-coordinate joint DOF data to maximal coordinates.
    ///
    /// - Indexing into the maximal joint DOF data is via the link's low-level index minus 1 (the root link is not included).
    /// - The reduced-coordinate data follows the cache indexing convention, see PxArticulationCache::jointVelocity.
    ///
    /// The articulation must be in a scene.
    @(link_name = "PxArticulationReducedCoordinate_unpackJointData")
    articulation_reduced_coordinate_unpack_joint_data :: proc(self_: ^ArticulationReducedCoordinate, reduced: ^_c.float, maximum: ^_c.float) ---

    /// Prepares common articulation data based on articulation pose for inverse dynamics calculations.
    ///
    /// Usage:
    /// 1. Set articulation pose (joint positions and base transform) via articulation cache and applyCache().
    /// 1. Call commonInit.
    /// 1. Call inverse dynamics computation method.
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
    @(link_name = "PxArticulationReducedCoordinate_commonInit")
    articulation_reduced_coordinate_common_init :: proc(self_: ^ArticulationReducedCoordinate) ---

    /// Computes the joint DOF forces required to counteract gravitational forces for the given articulation pose.
    ///
    /// - Inputs - Articulation pose (joint positions + base transform).
    /// - Outputs - Joint forces to counteract gravity (in cache).
    ///
    /// - The joint forces returned are determined purely by gravity for the articulation in the current joint and base pose, and joints at rest;
    /// i.e. external forces, joint velocities, and joint accelerations are set to zero. Joint drives are also not considered in the computation.
    /// - commonInit() must be called before the computation, and after setting the articulation pose via applyCache().
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
    @(link_name = "PxArticulationReducedCoordinate_computeGeneralizedGravityForce")
    articulation_reduced_coordinate_compute_generalized_gravity_force :: proc(self_: ^ArticulationReducedCoordinate, cache: ^ArticulationCache) ---

    /// Computes the joint DOF forces required to counteract Coriolis and centrifugal forces for the given articulation state.
    ///
    /// - Inputs - Articulation state (joint positions and velocities (in cache), and base transform and spatial velocity).
    /// - Outputs - Joint forces to counteract Coriolis and centrifugal forces (in cache).
    ///
    /// - The joint forces returned are determined purely by the articulation's state; i.e. external forces, gravity, and joint accelerations are set to zero.
    /// Joint drives and potential damping terms, such as link angular or linear damping, or joint friction, are also not considered in the computation.
    /// - Prior to the computation, update/set the base spatial velocity with PxArticulationCache::rootLinkData and applyCache().
    /// - commonInit() must be called before the computation, and after setting the articulation pose via applyCache().
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
    @(link_name = "PxArticulationReducedCoordinate_computeCoriolisAndCentrifugalForce")
    articulation_reduced_coordinate_compute_coriolis_and_centrifugal_force :: proc(self_: ^ArticulationReducedCoordinate, cache: ^ArticulationCache) ---

    /// Computes the joint DOF forces required to counteract external spatial forces applied to articulation links.
    ///
    /// - Inputs - External forces on links (in cache), articulation pose (joint positions + base transform).
    /// - Outputs - Joint forces to counteract the external forces (in cache).
    ///
    /// - Only the external spatial forces provided in the cache and the articulation pose are considered in the computation.
    /// - The external spatial forces are with respect to the links' centers of mass, and not the actor's origin.
    /// - commonInit() must be called before the computation, and after setting the articulation pose via applyCache().
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
    @(link_name = "PxArticulationReducedCoordinate_computeGeneralizedExternalForce")
    articulation_reduced_coordinate_compute_generalized_external_force :: proc(self_: ^ArticulationReducedCoordinate, cache: ^ArticulationCache) ---

    /// Computes the joint accelerations for the given articulation state and joint forces.
    ///
    /// - Inputs - Joint forces (in cache) and articulation state (joint positions and velocities (in cache), and base transform and spatial velocity).
    /// - Outputs - Joint accelerations (in cache).
    ///
    /// - The computation includes Coriolis terms and gravity. However, joint drives and potential damping terms are not considered in the computation
    /// (for example, linear link damping or joint friction).
    /// - Prior to the computation, update/set the base spatial velocity with PxArticulationCache::rootLinkData and applyCache().
    /// - commonInit() must be called before the computation, and after setting the articulation pose via applyCache().
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
    @(link_name = "PxArticulationReducedCoordinate_computeJointAcceleration")
    articulation_reduced_coordinate_compute_joint_acceleration :: proc(self_: ^ArticulationReducedCoordinate, cache: ^ArticulationCache) ---

    /// Computes the joint forces for the given articulation state and joint accelerations, not considering gravity.
    ///
    /// - Inputs - Joint accelerations (in cache) and articulation state (joint positions and velocities (in cache), and base transform and spatial velocity).
    /// - Outputs - Joint forces (in cache).
    ///
    /// - The computation includes Coriolis terms. However, joint drives and potential damping terms are not considered in the computation
    /// (for example, linear link damping or joint friction).
    /// - Prior to the computation, update/set the base spatial velocity with PxArticulationCache::rootLinkData and applyCache().
    /// - commonInit() must be called before the computation, and after setting the articulation pose via applyCache().
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
    @(link_name = "PxArticulationReducedCoordinate_computeJointForce")
    articulation_reduced_coordinate_compute_joint_force :: proc(self_: ^ArticulationReducedCoordinate, cache: ^ArticulationCache) ---

    /// Compute the dense Jacobian for the articulation in world space, including the DOFs of a potentially floating base.
    ///
    /// This computes the dense representation of an inherently sparse matrix. Multiplication with this matrix maps
    /// joint space velocities to world-space linear and angular (i.e. spatial) velocities of the centers of mass of the links.
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
    @(link_name = "PxArticulationReducedCoordinate_computeDenseJacobian")
    articulation_reduced_coordinate_compute_dense_jacobian :: proc(self_: ^ArticulationReducedCoordinate, cache: ^ArticulationCache, nRows: ^_c.uint32_t, nCols: ^_c.uint32_t) ---

    /// Computes the coefficient matrix for contact forces.
    ///
    /// - The matrix dimension is getCoefficientMatrixSize() = getDofs() * getNbLoopJoints(), and the DOF (column) indexing follows the internal DOF order, see PxArticulationCache::jointVelocity.
    /// - Each column in the matrix is the joint forces effected by a contact based on impulse strength 1.
    /// - The user must allocate memory for PxArticulationCache::coefficientMatrix where the required size of the PxReal array is equal to getCoefficientMatrixSize().
    /// - commonInit() must be called before the computation, and after setting the articulation pose via applyCache().
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
    @(link_name = "PxArticulationReducedCoordinate_computeCoefficientMatrix")
    articulation_reduced_coordinate_compute_coefficient_matrix :: proc(self_: ^ArticulationReducedCoordinate, cache: ^ArticulationCache) ---

    /// Computes the lambda values when the test impulse is 1.
    ///
    /// - The user must allocate memory for PxArticulationCache::lambda where the required size of the PxReal array is equal to getNbLoopJoints().
    /// - commonInit() must be called before the computation, and after setting the articulation pose via applyCache().
    ///
    /// True if convergence was achieved within maxIter; False if convergence was not achieved or the operation failed otherwise.
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
    @(link_name = "PxArticulationReducedCoordinate_computeLambda")
    articulation_reduced_coordinate_compute_lambda :: proc(self_: ^ArticulationReducedCoordinate, cache: ^ArticulationCache, initialState: ^ArticulationCache, jointTorque: ^_c.float, maxIter: _c.uint32_t) -> _c.bool ---

    /// Compute the joint-space inertia matrix that maps joint accelerations to joint forces: forces = M * accelerations.
    ///
    /// - Inputs - Articulation pose (joint positions and base transform).
    /// - Outputs - Mass matrix (in cache).
    ///
    /// commonInit() must be called before the computation, and after setting the articulation pose via applyCache().
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
    @(link_name = "PxArticulationReducedCoordinate_computeGeneralizedMassMatrix")
    articulation_reduced_coordinate_compute_generalized_mass_matrix :: proc(self_: ^ArticulationReducedCoordinate, cache: ^ArticulationCache) ---

    /// Adds a loop joint to the articulation system for inverse dynamics.
    ///
    /// This call may not be made during simulation.
    @(link_name = "PxArticulationReducedCoordinate_addLoopJoint_mut")
    articulation_reduced_coordinate_add_loop_joint_mut :: proc(self_: ^ArticulationReducedCoordinate, joint: ^Constraint) ---

    /// Removes a loop joint from the articulation for inverse dynamics.
    ///
    /// This call may not be made during simulation.
    @(link_name = "PxArticulationReducedCoordinate_removeLoopJoint_mut")
    articulation_reduced_coordinate_remove_loop_joint_mut :: proc(self_: ^ArticulationReducedCoordinate, joint: ^Constraint) ---

    /// Returns the number of loop joints in the articulation for inverse dynamics.
    ///
    /// The number of loop joints.
    @(link_name = "PxArticulationReducedCoordinate_getNbLoopJoints")
    articulation_reduced_coordinate_get_nb_loop_joints :: proc(self_: ^ArticulationReducedCoordinate) -> _c.uint32_t ---

    /// Returns the set of loop constraints (i.e. joints) in the articulation.
    ///
    /// The number of constraints written into the buffer.
    @(link_name = "PxArticulationReducedCoordinate_getLoopJoints")
    articulation_reduced_coordinate_get_loop_joints :: proc(self_: ^ArticulationReducedCoordinate, userBuffer: [^]^Constraint, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Returns the required size of the coefficient matrix in the articulation.
    ///
    /// Size of the coefficient matrix (equal to getDofs() * getNbLoopJoints()).
    ///
    /// This call may only be made on articulations that are in a scene.
    @(link_name = "PxArticulationReducedCoordinate_getCoefficientMatrixSize")
    articulation_reduced_coordinate_get_coefficient_matrix_size :: proc(self_: ^ArticulationReducedCoordinate) -> _c.uint32_t ---

    /// Sets the root link transform (world to actor frame).
    ///
    /// - For performance, prefer PxArticulationCache::rootLinkData to set the root link transform in a batch articulation state update.
    /// - Use updateKinematic() after all state updates to the articulation via non-cache API such as this method,
    /// in order to update link states for the next simulation frame or querying.
    ///
    /// This call may not be made during simulation.
    @(link_name = "PxArticulationReducedCoordinate_setRootGlobalPose_mut")
    articulation_reduced_coordinate_set_root_global_pose_mut :: proc(self_: ^ArticulationReducedCoordinate, #by_ptr pose: Transform, autowake: _c.bool) ---

    /// Returns the root link transform (world to actor frame).
    ///
    /// For performance, prefer PxArticulationCache::rootLinkData to get the root link transform in a batch query.
    ///
    /// The root link transform.
    ///
    /// This call is not allowed while the simulation is running except in a split simulation during [`PxScene::collide`]() and up to #PxScene::advance(),
    /// and in PxContactModifyCallback or in contact report callbacks.
    @(link_name = "PxArticulationReducedCoordinate_getRootGlobalPose")
    articulation_reduced_coordinate_get_root_global_pose :: proc(self_: ^ArticulationReducedCoordinate) -> Transform ---

    /// Sets the root link linear center-of-mass velocity.
    ///
    /// - The linear velocity is with respect to the link's center of mass and not the actor frame origin.
    /// - For performance, prefer PxArticulationCache::rootLinkData to set the root link velocity in a batch articulation state update.
    /// - The articulation is woken up if the input velocity is nonzero (ignoring autowake) and the articulation is in a scene.
    /// - Use updateKinematic() after all state updates to the articulation via non-cache API such as this method,
    /// in order to update link states for the next simulation frame or querying.
    ///
    /// This call may not be made during simulation, except in a split simulation in-between [`PxScene::fetchCollision`] and #PxScene::advance.
    @(link_name = "PxArticulationReducedCoordinate_setRootLinearVelocity_mut")
    articulation_reduced_coordinate_set_root_linear_velocity_mut :: proc(self_: ^ArticulationReducedCoordinate, #by_ptr linearVelocity: Vec3, autowake: _c.bool) ---

    /// Gets the root link center-of-mass linear velocity.
    ///
    /// - The linear velocity is with respect to the link's center of mass and not the actor frame origin.
    /// - For performance, prefer PxArticulationCache::rootLinkData to get the root link velocity in a batch query.
    ///
    /// The root link center-of-mass linear velocity.
    ///
    /// This call is not allowed while the simulation is running except in a split simulation during [`PxScene::collide`]() and up to #PxScene::advance(),
    /// and in PxContactModifyCallback or in contact report callbacks.
    @(link_name = "PxArticulationReducedCoordinate_getRootLinearVelocity")
    articulation_reduced_coordinate_get_root_linear_velocity :: proc(self_: ^ArticulationReducedCoordinate) -> Vec3 ---

    /// Sets the root link angular velocity.
    ///
    /// - For performance, prefer PxArticulationCache::rootLinkData to set the root link velocity in a batch articulation state update.
    /// - The articulation is woken up if the input velocity is nonzero (ignoring autowake) and the articulation is in a scene.
    /// - Use updateKinematic() after all state updates to the articulation via non-cache API such as this method,
    /// in order to update link states for the next simulation frame or querying.
    ///
    /// This call may not be made during simulation, except in a split simulation in-between [`PxScene::fetchCollision`] and #PxScene::advance.
    @(link_name = "PxArticulationReducedCoordinate_setRootAngularVelocity_mut")
    articulation_reduced_coordinate_set_root_angular_velocity_mut :: proc(self_: ^ArticulationReducedCoordinate, #by_ptr angularVelocity: Vec3, autowake: _c.bool) ---

    /// Gets the root link angular velocity.
    ///
    /// For performance, prefer PxArticulationCache::rootLinkData to get the root link velocity in a batch query.
    ///
    /// The root link angular velocity.
    ///
    /// This call is not allowed while the simulation is running except in a split simulation during [`PxScene::collide`]() and up to #PxScene::advance(),
    /// and in PxContactModifyCallback or in contact report callbacks.
    @(link_name = "PxArticulationReducedCoordinate_getRootAngularVelocity")
    articulation_reduced_coordinate_get_root_angular_velocity :: proc(self_: ^ArticulationReducedCoordinate) -> Vec3 ---

    /// Returns the (classical) link acceleration in world space for the given low-level link index.
    ///
    /// - The returned acceleration is not a spatial, but a classical, i.e. body-fixed acceleration (https://en.wikipedia.org/wiki/Spatial_acceleration).
    /// - The (linear) acceleration is with respect to the link's center of mass and not the actor frame origin.
    ///
    /// The link's center-of-mass classical acceleration, or 0 if the call is made before the articulation participated in a first simulation step.
    ///
    /// This call may only be made on articulations that are in a scene, and it is not allowed to use this method while the simulation
    /// is running except in a split simulation during [`PxScene::collide`]() and up to #PxScene::advance(), and in PxContactModifyCallback or in contact report callbacks.
    @(link_name = "PxArticulationReducedCoordinate_getLinkAcceleration_mut")
    articulation_reduced_coordinate_get_link_acceleration_mut :: proc(self_: ^ArticulationReducedCoordinate, linkId: _c.uint32_t) -> SpatialVelocity ---

    /// Returns the GPU articulation index.
    ///
    /// The GPU index, or 0xFFFFFFFF if the articulation is not in a scene or PxSceneFlag::eSUPPRESS_READBACK is not set.
    @(link_name = "PxArticulationReducedCoordinate_getGpuArticulationIndex_mut")
    articulation_reduced_coordinate_get_gpu_articulation_index_mut :: proc(self_: ^ArticulationReducedCoordinate) -> _c.uint32_t ---

    /// Creates a spatial tendon to attach to the articulation with default attribute values.
    ///
    /// The new spatial tendon.
    ///
    /// Creating a spatial tendon is not allowed while the articulation is in a scene. In order to
    /// add the tendon, remove and then re-add the articulation to the scene.
    @(link_name = "PxArticulationReducedCoordinate_createSpatialTendon_mut")
    articulation_reduced_coordinate_create_spatial_tendon_mut :: proc(self_: ^ArticulationReducedCoordinate) -> ^ArticulationSpatialTendon ---

    /// Creates a fixed tendon to attach to the articulation with default attribute values.
    ///
    /// The new fixed tendon.
    ///
    /// Creating a fixed tendon is not allowed while the articulation is in a scene. In order to
    /// add the tendon, remove and then re-add the articulation to the scene.
    @(link_name = "PxArticulationReducedCoordinate_createFixedTendon_mut")
    articulation_reduced_coordinate_create_fixed_tendon_mut :: proc(self_: ^ArticulationReducedCoordinate) -> ^ArticulationFixedTendon ---

    /// Creates a force sensor attached to a link of the articulation.
    ///
    /// The new sensor.
    ///
    /// Creating a sensor is not allowed while the articulation is in a scene. In order to
    /// add the sensor, remove and then re-add the articulation to the scene.
    @(link_name = "PxArticulationReducedCoordinate_createSensor_mut")
    articulation_reduced_coordinate_create_sensor_mut :: proc(self_: ^ArticulationReducedCoordinate, link: ^ArticulationLink, #by_ptr relativePose: Transform) -> ^ArticulationSensor ---

    /// Returns the spatial tendons attached to the articulation.
    ///
    /// The order of the tendons in the buffer is not necessarily identical to the order in which the tendons were added to the articulation.
    ///
    /// The number of tendons written into the buffer.
    @(link_name = "PxArticulationReducedCoordinate_getSpatialTendons")
    articulation_reduced_coordinate_get_spatial_tendons :: proc(self_: ^ArticulationReducedCoordinate, userBuffer: [^]^ArticulationSpatialTendon, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Returns the number of spatial tendons in the articulation.
    ///
    /// The number of tendons.
    @(link_name = "PxArticulationReducedCoordinate_getNbSpatialTendons_mut")
    articulation_reduced_coordinate_get_nb_spatial_tendons_mut :: proc(self_: ^ArticulationReducedCoordinate) -> _c.uint32_t ---

    /// Returns the fixed tendons attached to the articulation.
    ///
    /// The order of the tendons in the buffer is not necessarily identical to the order in which the tendons were added to the articulation.
    ///
    /// The number of tendons written into the buffer.
    @(link_name = "PxArticulationReducedCoordinate_getFixedTendons")
    articulation_reduced_coordinate_get_fixed_tendons :: proc(self_: ^ArticulationReducedCoordinate, userBuffer: [^]^ArticulationFixedTendon, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Returns the number of fixed tendons in the articulation.
    ///
    /// The number of tendons.
    @(link_name = "PxArticulationReducedCoordinate_getNbFixedTendons_mut")
    articulation_reduced_coordinate_get_nb_fixed_tendons_mut :: proc(self_: ^ArticulationReducedCoordinate) -> _c.uint32_t ---

    /// Returns the sensors attached to the articulation.
    ///
    /// The order of the sensors in the buffer is not necessarily identical to the order in which the sensors were added to the articulation.
    ///
    /// The number of sensors written into the buffer.
    @(link_name = "PxArticulationReducedCoordinate_getSensors")
    articulation_reduced_coordinate_get_sensors :: proc(self_: ^ArticulationReducedCoordinate, userBuffer: [^]^ArticulationSensor, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Returns the number of sensors in the articulation.
    ///
    /// The number of sensors.
    @(link_name = "PxArticulationReducedCoordinate_getNbSensors_mut")
    articulation_reduced_coordinate_get_nb_sensors_mut :: proc(self_: ^ArticulationReducedCoordinate) -> _c.uint32_t ---

    /// Update link velocities and/or positions in the articulation.
    ///
    /// For performance, prefer the PxArticulationCache API that performs batch articulation state updates.
    ///
    /// If the application updates the root state (position and velocity) or joint state via any combination of
    /// the non-cache API calls
    ///
    /// - setRootGlobalPose(), setRootLinearVelocity(), setRootAngularVelocity()
    /// - PxArticulationJointReducedCoordinate::setJointPosition(), PxArticulationJointReducedCoordinate::setJointVelocity()
    ///
    /// the application needs to call this method after the state setting in order to update the link states for
    /// the next simulation frame or querying.
    ///
    /// Use
    /// - PxArticulationKinematicFlag::ePOSITION after any changes to the articulation root or joint positions using non-cache API calls. Updates links' positions and velocities.
    /// - PxArticulationKinematicFlag::eVELOCITY after velocity-only changes to the articulation root or joints using non-cache API calls. Updates links' velocities only.
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
    @(link_name = "PxArticulationReducedCoordinate_updateKinematic_mut")
    articulation_reduced_coordinate_update_kinematic_mut :: proc(self_: ^ArticulationReducedCoordinate, flags: ArticulationKinematicFlags_Set) ---

    /// Gets the parent articulation link of this joint.
    ///
    /// The parent link.
    @(link_name = "PxArticulationJointReducedCoordinate_getParentArticulationLink")
    articulation_joint_reduced_coordinate_get_parent_articulation_link :: proc(self_: ^ArticulationJointReducedCoordinate) -> ^ArticulationLink ---

    /// Sets the joint pose in the parent link actor frame.
    ///
    /// This call is not allowed while the simulation is running.
    @(link_name = "PxArticulationJointReducedCoordinate_setParentPose_mut")
    articulation_joint_reduced_coordinate_set_parent_pose_mut :: proc(self_: ^ArticulationJointReducedCoordinate, #by_ptr pose: Transform) ---

    /// Gets the joint pose in the parent link actor frame.
    ///
    /// The joint pose.
    @(link_name = "PxArticulationJointReducedCoordinate_getParentPose")
    articulation_joint_reduced_coordinate_get_parent_pose :: proc(self_: ^ArticulationJointReducedCoordinate) -> Transform ---

    /// Gets the child articulation link of this joint.
    ///
    /// The child link.
    @(link_name = "PxArticulationJointReducedCoordinate_getChildArticulationLink")
    articulation_joint_reduced_coordinate_get_child_articulation_link :: proc(self_: ^ArticulationJointReducedCoordinate) -> ^ArticulationLink ---

    /// Sets the joint pose in the child link actor frame.
    ///
    /// This call is not allowed while the simulation is running.
    @(link_name = "PxArticulationJointReducedCoordinate_setChildPose_mut")
    articulation_joint_reduced_coordinate_set_child_pose_mut :: proc(self_: ^ArticulationJointReducedCoordinate, #by_ptr pose: Transform) ---

    /// Gets the joint pose in the child link actor frame.
    ///
    /// The joint pose.
    @(link_name = "PxArticulationJointReducedCoordinate_getChildPose")
    articulation_joint_reduced_coordinate_get_child_pose :: proc(self_: ^ArticulationJointReducedCoordinate) -> Transform ---

    /// Sets the joint type (e.g. revolute).
    ///
    /// Setting the joint type is not allowed while the articulation is in a scene.
    /// In order to set the joint type, remove and then re-add the articulation to the scene.
    @(link_name = "PxArticulationJointReducedCoordinate_setJointType_mut")
    articulation_joint_reduced_coordinate_set_joint_type_mut :: proc(self_: ^ArticulationJointReducedCoordinate, jointType: ArticulationJointType) ---

    /// Gets the joint type.
    ///
    /// The joint type.
    @(link_name = "PxArticulationJointReducedCoordinate_getJointType")
    articulation_joint_reduced_coordinate_get_joint_type :: proc(self_: ^ArticulationJointReducedCoordinate) -> ArticulationJointType ---

    /// Sets the joint motion for a given axis.
    ///
    /// Setting the motion of joint axes is not allowed while the articulation is in a scene.
    /// In order to set the motion, remove and then re-add the articulation to the scene.
    @(link_name = "PxArticulationJointReducedCoordinate_setMotion_mut")
    articulation_joint_reduced_coordinate_set_motion_mut :: proc(self_: ^ArticulationJointReducedCoordinate, axis: ArticulationAxis, motion: ArticulationMotion) ---

    /// Returns the joint motion for the given axis.
    ///
    /// The joint motion of the given axis.
    @(link_name = "PxArticulationJointReducedCoordinate_getMotion")
    articulation_joint_reduced_coordinate_get_motion :: proc(self_: ^ArticulationJointReducedCoordinate, axis: ArticulationAxis) -> ArticulationMotion ---

    /// Sets the joint limits for a given axis.
    ///
    /// - The motion of the corresponding axis should be set to PxArticulationMotion::eLIMITED in order for the limits to be enforced.
    /// - The lower limit should be strictly smaller than the higher limit. If the limits should be equal, use PxArticulationMotion::eLOCKED
    /// and an appropriate offset in the parent/child joint frames.
    ///
    /// This call is not allowed while the simulation is running.
    ///
    /// For spherical joints, limit.min and limit.max must both be in range [-Pi, Pi].
    @(link_name = "PxArticulationJointReducedCoordinate_setLimitParams_mut")
    articulation_joint_reduced_coordinate_set_limit_params_mut :: proc(self_: ^ArticulationJointReducedCoordinate, axis: ArticulationAxis, #by_ptr limit: ArticulationLimit) ---

    /// Returns the joint limits for a given axis.
    ///
    /// The joint limits.
    @(link_name = "PxArticulationJointReducedCoordinate_getLimitParams")
    articulation_joint_reduced_coordinate_get_limit_params :: proc(self_: ^ArticulationJointReducedCoordinate, axis: ArticulationAxis) -> ArticulationLimit ---

    /// Configures a joint drive for the given axis.
    ///
    /// See PxArticulationDrive for parameter details; and the manual for further information, and the drives' implicit spring-damper (i.e. PD control) implementation in particular.
    ///
    /// This call is not allowed while the simulation is running.
    @(link_name = "PxArticulationJointReducedCoordinate_setDriveParams_mut")
    articulation_joint_reduced_coordinate_set_drive_params_mut :: proc(self_: ^ArticulationJointReducedCoordinate, axis: ArticulationAxis, #by_ptr drive: ArticulationDrive) ---

    /// Gets the joint drive configuration for the given axis.
    ///
    /// The drive parameters.
    @(link_name = "PxArticulationJointReducedCoordinate_getDriveParams")
    articulation_joint_reduced_coordinate_get_drive_params :: proc(self_: ^ArticulationJointReducedCoordinate, axis: ArticulationAxis) -> ArticulationDrive ---

    /// Sets the joint drive position target for the given axis.
    ///
    /// The target units are linear units (equivalent to scene units) for a translational axis, or rad for a rotational axis.
    ///
    /// This call is not allowed while the simulation is running.
    ///
    /// For spherical joints, target must be in range [-Pi, Pi].
    ///
    /// The target is specified in the parent frame of the joint. If Gp, Gc are the parent and child actor poses in the world frame and Lp, Lc are the parent and child joint frames expressed in the parent and child actor frames then the joint will drive the parent and child links to poses that obey Gp * Lp * J = Gc * Lc. For joints restricted to angular motion, J has the form PxTranfsorm(PxVec3(PxZero), PxExp(PxVec3(twistTarget, swing1Target, swing2Target))).  For joints restricted to linear motion, J has the form PxTransform(PxVec3(XTarget, YTarget, ZTarget), PxQuat(PxIdentity)).
    ///
    /// For spherical joints with more than 1 degree of freedom, the joint target angles taken together can collectively represent a rotation of greater than Pi around a vector. When this happens the rotation that matches the joint drive target is not the shortest path rotation.  The joint pose J that is the outcome after driving to the target pose will always be the equivalent of the shortest path rotation.
    @(link_name = "PxArticulationJointReducedCoordinate_setDriveTarget_mut")
    articulation_joint_reduced_coordinate_set_drive_target_mut :: proc(self_: ^ArticulationJointReducedCoordinate, axis: ArticulationAxis, target: _c.float, autowake: _c.bool) ---

    /// Returns the joint drive position target for the given axis.
    ///
    /// The target position.
    @(link_name = "PxArticulationJointReducedCoordinate_getDriveTarget")
    articulation_joint_reduced_coordinate_get_drive_target :: proc(self_: ^ArticulationJointReducedCoordinate, axis: ArticulationAxis) -> _c.float ---

    /// Sets the joint drive velocity target for the given axis.
    ///
    /// The target units are linear units (equivalent to scene units) per second for a translational axis, or radians per second for a rotational axis.
    ///
    /// This call is not allowed while the simulation is running.
    @(link_name = "PxArticulationJointReducedCoordinate_setDriveVelocity_mut")
    articulation_joint_reduced_coordinate_set_drive_velocity_mut :: proc(self_: ^ArticulationJointReducedCoordinate, axis: ArticulationAxis, targetVel: _c.float, autowake: _c.bool) ---

    /// Returns the joint drive velocity target for the given axis.
    ///
    /// The target velocity.
    @(link_name = "PxArticulationJointReducedCoordinate_getDriveVelocity")
    articulation_joint_reduced_coordinate_get_drive_velocity :: proc(self_: ^ArticulationJointReducedCoordinate, axis: ArticulationAxis) -> _c.float ---

    /// Sets the joint armature for the given axis.
    ///
    /// - The armature is directly added to the joint-space spatial inertia of the corresponding axis.
    /// - The armature is in mass units for a prismatic (i.e. linear) joint, and in mass units * (scene linear units)^2 for a rotational joint.
    ///
    /// This call is not allowed while the simulation is running.
    @(link_name = "PxArticulationJointReducedCoordinate_setArmature_mut")
    articulation_joint_reduced_coordinate_set_armature_mut :: proc(self_: ^ArticulationJointReducedCoordinate, axis: ArticulationAxis, armature: _c.float) ---

    /// Gets the joint armature for the given axis.
    ///
    /// The armature set on the given axis.
    @(link_name = "PxArticulationJointReducedCoordinate_getArmature")
    articulation_joint_reduced_coordinate_get_armature :: proc(self_: ^ArticulationJointReducedCoordinate, axis: ArticulationAxis) -> _c.float ---

    /// Sets the joint friction coefficient, which applies to all joint axes.
    ///
    /// - The joint friction is unitless and relates the magnitude of the spatial force [F_trans, T_trans] transmitted from parent to child link to
    /// the maximal friction force F_resist that may be applied by the solver to resist joint motion, per axis; i.e. |F_resist|
    /// <
    /// = coefficient * (|F_trans| + |T_trans|),
    /// where F_resist may refer to a linear force or torque depending on the joint axis.
    /// - The simulated friction effect is therefore similar to static and Coulomb friction. In order to simulate dynamic joint friction, use a joint drive with
    /// zero stiffness and zero velocity target, and an appropriately dimensioned damping parameter.
    ///
    /// This call is not allowed while the simulation is running.
    @(link_name = "PxArticulationJointReducedCoordinate_setFrictionCoefficient_mut")
    articulation_joint_reduced_coordinate_set_friction_coefficient_mut :: proc(self_: ^ArticulationJointReducedCoordinate, coefficient: _c.float) ---

    /// Gets the joint friction coefficient.
    ///
    /// The joint friction coefficient.
    @(link_name = "PxArticulationJointReducedCoordinate_getFrictionCoefficient")
    articulation_joint_reduced_coordinate_get_friction_coefficient :: proc(self_: ^ArticulationJointReducedCoordinate) -> _c.float ---

    /// Sets the maximal joint velocity enforced for all axes.
    ///
    /// - The solver will apply appropriate joint-space impulses in order to enforce the per-axis joint-velocity limit.
    /// - The velocity units are linear units (equivalent to scene units) per second for a translational axis, or radians per second for a rotational axis.
    ///
    /// This call is not allowed while the simulation is running.
    @(link_name = "PxArticulationJointReducedCoordinate_setMaxJointVelocity_mut")
    articulation_joint_reduced_coordinate_set_max_joint_velocity_mut :: proc(self_: ^ArticulationJointReducedCoordinate, maxJointV: _c.float) ---

    /// Gets the maximal joint velocity enforced for all axes.
    ///
    /// The maximal per-axis joint velocity.
    @(link_name = "PxArticulationJointReducedCoordinate_getMaxJointVelocity")
    articulation_joint_reduced_coordinate_get_max_joint_velocity :: proc(self_: ^ArticulationJointReducedCoordinate) -> _c.float ---

    /// Sets the joint position for the given axis.
    ///
    /// - For performance, prefer PxArticulationCache::jointPosition to set joint positions in a batch articulation state update.
    /// - Use PxArticulationReducedCoordinate::updateKinematic after all state updates to the articulation via non-cache API such as this method,
    /// in order to update link states for the next simulation frame or querying.
    ///
    /// This call is not allowed while the simulation is running.
    ///
    /// For spherical joints, jointPos must be in range [-Pi, Pi].
    ///
    /// Joint position is specified in the parent frame of the joint. If Gp, Gc are the parent and child actor poses in the world frame and Lp, Lc are the parent and child joint frames expressed in the parent and child actor frames then the parent and child links will be given poses that obey Gp * Lp * J = Gc * Lc with J denoting the joint pose. For joints restricted to angular motion, J has the form PxTranfsorm(PxVec3(PxZero), PxExp(PxVec3(twistPos, swing1Pos, swing2Pos))).  For joints restricted to linear motion, J has the form PxTransform(PxVec3(xPos, yPos, zPos), PxQuat(PxIdentity)).
    ///
    /// For spherical joints with more than 1 degree of freedom, the input joint positions taken together can collectively represent a rotation of greater than Pi around a vector. When this happens the rotation that matches the joint positions is not the shortest path rotation.  The joint pose J that is the outcome of setting and applying the joint positions will always be the equivalent of the shortest path rotation.
    @(link_name = "PxArticulationJointReducedCoordinate_setJointPosition_mut")
    articulation_joint_reduced_coordinate_set_joint_position_mut :: proc(self_: ^ArticulationJointReducedCoordinate, axis: ArticulationAxis, jointPos: _c.float) ---

    /// Gets the joint position for the given axis, i.e. joint degree of freedom (DOF).
    ///
    /// For performance, prefer PxArticulationCache::jointPosition to get joint positions in a batch query.
    ///
    /// The joint position in linear units (equivalent to scene units) for a translational axis, or radians for a rotational axis.
    ///
    /// This call is not allowed while the simulation is running except in a split simulation during [`PxScene::collide`]() and up to #PxScene::advance(),
    /// and in PxContactModifyCallback or in contact report callbacks.
    @(link_name = "PxArticulationJointReducedCoordinate_getJointPosition")
    articulation_joint_reduced_coordinate_get_joint_position :: proc(self_: ^ArticulationJointReducedCoordinate, axis: ArticulationAxis) -> _c.float ---

    /// Sets the joint velocity for the given axis.
    ///
    /// - For performance, prefer PxArticulationCache::jointVelocity to set joint velocities in a batch articulation state update.
    /// - Use PxArticulationReducedCoordinate::updateKinematic after all state updates to the articulation via non-cache API such as this method,
    /// in order to update link states for the next simulation frame or querying.
    ///
    /// This call is not allowed while the simulation is running.
    @(link_name = "PxArticulationJointReducedCoordinate_setJointVelocity_mut")
    articulation_joint_reduced_coordinate_set_joint_velocity_mut :: proc(self_: ^ArticulationJointReducedCoordinate, axis: ArticulationAxis, jointVel: _c.float) ---

    /// Gets the joint velocity for the given axis.
    ///
    /// For performance, prefer PxArticulationCache::jointVelocity to get joint velocities in a batch query.
    ///
    /// The joint velocity in linear units (equivalent to scene units) per second for a translational axis, or radians per second for a rotational axis.
    ///
    /// This call is not allowed while the simulation is running except in a split simulation during [`PxScene::collide`]() and up to #PxScene::advance(),
    /// and in PxContactModifyCallback or in contact report callbacks.
    @(link_name = "PxArticulationJointReducedCoordinate_getJointVelocity")
    articulation_joint_reduced_coordinate_get_joint_velocity :: proc(self_: ^ArticulationJointReducedCoordinate, axis: ArticulationAxis) -> _c.float ---

    /// Returns the string name of the dynamic type.
    ///
    /// The string name.
    @(link_name = "PxArticulationJointReducedCoordinate_getConcreteTypeName")
    articulation_joint_reduced_coordinate_get_concrete_type_name :: proc(self_: ^ArticulationJointReducedCoordinate) -> cstring ---

    /// Decrements the reference count of a shape and releases it if the new reference count is zero.
    ///
    /// Note that in releases prior to PhysX 3.3 this method did not have reference counting semantics and was used to destroy a shape
    /// created with PxActor::createShape(). In PhysX 3.3 and above, this usage is deprecated, instead, use PxRigidActor::detachShape() to detach
    /// a shape from an actor. If the shape to be detached was created with PxActor::createShape(), the actor holds the only counted reference,
    /// and so when the shape is detached it will also be destroyed.
    @(link_name = "PxShape_release_mut")
    shape_release_mut :: proc(self_: ^Shape) ---

    /// Adjust the geometry of the shape.
    ///
    /// The type of the passed in geometry must match the geometry type of the shape.
    ///
    /// It is not allowed to change the geometry type of a shape.
    ///
    /// This function does not guarantee correct/continuous behavior when objects are resting on top of old or new geometry.
    @(link_name = "PxShape_setGeometry_mut")
    shape_set_geometry_mut :: proc(self_: ^Shape, geometry: ^Geometry) ---

    /// Retrieve a reference to the shape's geometry.
    ///
    /// The returned reference has the same lifetime as the PxShape it comes from.
    ///
    /// Reference to internal PxGeometry object.
    @(link_name = "PxShape_getGeometry")
    shape_get_geometry :: proc(self_: ^Shape) -> ^Geometry ---

    /// Retrieves the actor which this shape is associated with.
    ///
    /// The actor this shape is associated with, if it is an exclusive shape, else NULL
    @(link_name = "PxShape_getActor")
    shape_get_actor :: proc(self_: ^Shape) -> ^RigidActor ---

    /// Sets the pose of the shape in actor space, i.e. relative to the actors to which they are attached.
    ///
    /// This transformation is identity by default.
    ///
    /// The local pose is an attribute of the shape, and so will apply to all actors to which the shape is attached.
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the associated actor up automatically.
    ///
    /// Note:
    /// Does not automatically update the inertia properties of the owning actor (if applicable); use the
    /// PhysX extensions method [`PxRigidBodyExt::updateMassAndInertia`]() to do this.
    ///
    /// Default:
    /// the identity transform
    @(link_name = "PxShape_setLocalPose_mut")
    shape_set_local_pose_mut :: proc(self_: ^Shape, #by_ptr pose: Transform) ---

    /// Retrieves the pose of the shape in actor space, i.e. relative to the actor they are owned by.
    ///
    /// This transformation is identity by default.
    ///
    /// Pose of shape relative to the actor's frame.
    @(link_name = "PxShape_getLocalPose")
    shape_get_local_pose :: proc(self_: ^Shape) -> Transform ---

    /// Sets the user definable collision filter data.
    ///
    /// Sleeping:
    /// Does wake up the actor if the filter data change causes a formerly suppressed
    /// collision pair to be enabled.
    ///
    /// Default:
    /// (0,0,0,0)
    @(link_name = "PxShape_setSimulationFilterData_mut")
    shape_set_simulation_filter_data_mut :: proc(self_: ^Shape, #by_ptr data: FilterData) ---

    /// Retrieves the shape's collision filter data.
    @(link_name = "PxShape_getSimulationFilterData")
    shape_get_simulation_filter_data :: proc(self_: ^Shape) -> FilterData ---

    /// Sets the user definable query filter data.
    ///
    /// Default:
    /// (0,0,0,0)
    @(link_name = "PxShape_setQueryFilterData_mut")
    shape_set_query_filter_data_mut :: proc(self_: ^Shape, #by_ptr data: FilterData) ---

    /// Retrieves the shape's Query filter data.
    @(link_name = "PxShape_getQueryFilterData")
    shape_get_query_filter_data :: proc(self_: ^Shape) -> FilterData ---

    /// Assigns material(s) to the shape. Will remove existing materials from the shape.
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the associated actor up automatically.
    @(link_name = "PxShape_setMaterials_mut")
    shape_set_materials_mut :: proc(self_: ^Shape, materials: [^]^Material, materialCount: _c.uint16_t) ---

    /// Returns the number of materials assigned to the shape.
    ///
    /// You can use [`getMaterials`]() to retrieve the material pointers.
    ///
    /// Number of materials associated with this shape.
    @(link_name = "PxShape_getNbMaterials")
    shape_get_nb_materials :: proc(self_: ^Shape) -> _c.uint16_t ---

    /// Retrieve all the material pointers associated with the shape.
    ///
    /// You can retrieve the number of material pointers by calling [`getNbMaterials`]()
    ///
    /// Note: The returned data may contain invalid pointers if you release materials using [`PxMaterial::release`]().
    ///
    /// Number of material pointers written to the buffer.
    @(link_name = "PxShape_getMaterials")
    shape_get_materials :: proc(self_: ^Shape, userBuffer: [^]^Material, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Retrieve material from given triangle index.
    ///
    /// The input index is the internal triangle index as used inside the SDK. This is the index
    /// returned to users by various SDK functions such as raycasts.
    ///
    /// This function is only useful for triangle meshes or heightfields, which have per-triangle
    /// materials. For other shapes or SDF triangle meshes, the function returns the single material
    /// associated with the shape, regardless of the index.
    ///
    /// Material from input triangle
    ///
    /// If faceIndex value of 0xFFFFffff is passed as an input for mesh and heightfield shapes, this function will issue a warning and return NULL.
    ///
    /// Scene queries set the value of PxQueryHit::faceIndex to 0xFFFFffff whenever it is undefined or does not apply.
    @(link_name = "PxShape_getMaterialFromInternalFaceIndex")
    shape_get_material_from_internal_face_index :: proc(self_: ^Shape, faceIndex: _c.uint32_t) -> ^BaseMaterial ---

    /// Sets the contact offset.
    ///
    /// Shapes whose distance is less than the sum of their contactOffset values will generate contacts. The contact offset must be positive and
    /// greater than the rest offset. Having a contactOffset greater than than the restOffset allows the collision detection system to
    /// predictively enforce the contact constraint even when the objects are slightly separated. This prevents jitter that would occur
    /// if the constraint were enforced only when shapes were within the rest distance.
    ///
    /// Default:
    /// 0.02f * PxTolerancesScale::length
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the associated actor up automatically.
    @(link_name = "PxShape_setContactOffset_mut")
    shape_set_contact_offset_mut :: proc(self_: ^Shape, contactOffset: _c.float) ---

    /// Retrieves the contact offset.
    ///
    /// The contact offset of the shape.
    @(link_name = "PxShape_getContactOffset")
    shape_get_contact_offset :: proc(self_: ^Shape) -> _c.float ---

    /// Sets the rest offset.
    ///
    /// Two shapes will come to rest at a distance equal to the sum of their restOffset values. If the restOffset is 0, they should converge to touching
    /// exactly.  Having a restOffset greater than zero is useful to have objects slide smoothly, so that they do not get hung up on irregularities of
    /// each others' surfaces.
    ///
    /// Default:
    /// 0.0f
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the associated actor up automatically.
    @(link_name = "PxShape_setRestOffset_mut")
    shape_set_rest_offset_mut :: proc(self_: ^Shape, restOffset: _c.float) ---

    /// Retrieves the rest offset.
    ///
    /// The rest offset of the shape.
    @(link_name = "PxShape_getRestOffset")
    shape_get_rest_offset :: proc(self_: ^Shape) -> _c.float ---

    /// Sets the density used to interact with fluids.
    ///
    /// To be physically accurate, the density of a rigid body should be computed as its mass divided by its volume. To
    /// simplify tuning the interaction of fluid and rigid bodies, the density for fluid can differ from the real density. This
    /// allows to create floating bodies, even if they are supposed to sink with their mass and volume.
    ///
    /// Default:
    /// 800.0f
    @(link_name = "PxShape_setDensityForFluid_mut")
    shape_set_density_for_fluid_mut :: proc(self_: ^Shape, densityForFluid: _c.float) ---

    /// Retrieves the density used to interact with fluids.
    ///
    /// The density of the body when interacting with fluid.
    @(link_name = "PxShape_getDensityForFluid")
    shape_get_density_for_fluid :: proc(self_: ^Shape) -> _c.float ---

    /// Sets torsional patch radius.
    ///
    /// This defines the radius of the contact patch used to apply torsional friction. If the radius is 0, no torsional friction
    /// will be applied. If the radius is > 0, some torsional friction will be applied. This is proportional to the penetration depth
    /// so, if the shapes are separated or penetration is zero, no torsional friction will be applied. It is used to approximate
    /// rotational friction introduced by the compression of contacting surfaces.
    ///
    /// Default:
    /// 0.0
    @(link_name = "PxShape_setTorsionalPatchRadius_mut")
    shape_set_torsional_patch_radius_mut :: proc(self_: ^Shape, radius: _c.float) ---

    /// Gets torsional patch radius.
    ///
    /// This defines the radius of the contact patch used to apply torsional friction. If the radius is 0, no torsional friction
    /// will be applied. If the radius is > 0, some torsional friction will be applied. This is proportional to the penetration depth
    /// so, if the shapes are separated or penetration is zero, no torsional friction will be applied. It is used to approximate
    /// rotational friction introduced by the compression of contacting surfaces.
    ///
    /// The torsional patch radius of the shape.
    @(link_name = "PxShape_getTorsionalPatchRadius")
    shape_get_torsional_patch_radius :: proc(self_: ^Shape) -> _c.float ---

    /// Sets minimum torsional patch radius.
    ///
    /// This defines the minimum radius of the contact patch used to apply torsional friction. If the radius is 0, the amount of torsional friction
    /// that will be applied will be entirely dependent on the value of torsionalPatchRadius.
    ///
    /// If the radius is > 0, some torsional friction will be applied regardless of the value of torsionalPatchRadius or the amount of penetration.
    ///
    /// Default:
    /// 0.0
    @(link_name = "PxShape_setMinTorsionalPatchRadius_mut")
    shape_set_min_torsional_patch_radius_mut :: proc(self_: ^Shape, radius: _c.float) ---

    /// Gets minimum torsional patch radius.
    ///
    /// This defines the minimum radius of the contact patch used to apply torsional friction. If the radius is 0, the amount of torsional friction
    /// that will be applied will be entirely dependent on the value of torsionalPatchRadius.
    ///
    /// If the radius is > 0, some torsional friction will be applied regardless of the value of torsionalPatchRadius or the amount of penetration.
    ///
    /// The minimum torsional patch radius of the shape.
    @(link_name = "PxShape_getMinTorsionalPatchRadius")
    shape_get_min_torsional_patch_radius :: proc(self_: ^Shape) -> _c.float ---

    /// Sets shape flags
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the associated actor up automatically.
    ///
    /// Default:
    /// PxShapeFlag::eVISUALIZATION | PxShapeFlag::eSIMULATION_SHAPE | PxShapeFlag::eSCENE_QUERY_SHAPE
    @(link_name = "PxShape_setFlag_mut")
    shape_set_flag_mut :: proc(self_: ^Shape, flag: ShapeFlag, value: _c.bool) ---

    /// Sets shape flags
    @(link_name = "PxShape_setFlags_mut")
    shape_set_flags_mut :: proc(self_: ^Shape, inFlags: ShapeFlags_Set) ---

    /// Retrieves shape flags.
    ///
    /// The values of the shape flags.
    @(link_name = "PxShape_getFlags")
    shape_get_flags :: proc(self_: ^Shape) -> ShapeFlags_Set ---

    /// Returns true if the shape is exclusive to an actor.
    @(link_name = "PxShape_isExclusive")
    shape_is_exclusive :: proc(self_: ^Shape) -> _c.bool ---

    /// Sets a name string for the object that can be retrieved with [`getName`]().
    ///
    /// This is for debugging and is not used by the SDK.
    /// The string is not copied by the SDK, only the pointer is stored.
    ///
    /// Default:
    /// NULL
    @(link_name = "PxShape_setName_mut")
    shape_set_name_mut :: proc(self_: ^Shape, name: cstring) ---

    /// retrieves the name string set with setName().
    ///
    /// The name associated with the shape.
    @(link_name = "PxShape_getName")
    shape_get_name :: proc(self_: ^Shape) -> cstring ---

    @(link_name = "PxShape_getConcreteTypeName")
    shape_get_concrete_type_name :: proc(self_: ^Shape) -> cstring ---

    /// Deletes the rigid actor object.
    ///
    /// Also releases any shapes associated with the actor.
    ///
    /// Releasing an actor will affect any objects that are connected to the actor (constraint shaders like joints etc.).
    /// Such connected objects will be deleted upon scene deletion, or explicitly by the user by calling release()
    /// on these objects. It is recommended to always remove all objects that reference actors before the actors
    /// themselves are removed. It is not possible to retrieve list of dead connected objects.
    ///
    /// Sleeping:
    /// This call will awaken any sleeping actors contacting the deleted actor (directly or indirectly).
    ///
    /// Calls [`PxActor::release`]() so you might want to check the documentation of that method as well.
    @(link_name = "PxRigidActor_release_mut")
    rigid_actor_release_mut :: proc(self_: ^RigidActor) ---

    /// Returns the internal actor index.
    ///
    /// This is only defined for actors that have been added to a scene.
    ///
    /// The internal actor index, or 0xffffffff if the actor is not part of a scene.
    @(link_name = "PxRigidActor_getInternalActorIndex")
    rigid_actor_get_internal_actor_index :: proc(self_: ^RigidActor) -> _c.uint32_t ---

    /// Retrieves the actors world space transform.
    ///
    /// The getGlobalPose() method retrieves the actor's current actor space to world space transformation.
    ///
    /// It is not allowed to use this method while the simulation is running (except during PxScene::collide(),
    /// in PxContactModifyCallback or in contact report callbacks).
    ///
    /// Global pose of object.
    @(link_name = "PxRigidActor_getGlobalPose")
    rigid_actor_get_global_pose :: proc(self_: ^RigidActor) -> Transform ---

    /// Method for setting an actor's pose in the world.
    ///
    /// This method instantaneously changes the actor space to world space transformation.
    ///
    /// This method is mainly for dynamic rigid bodies (see [`PxRigidDynamic`]). Calling this method on static actors is
    /// likely to result in a performance penalty, since internal optimization structures for static actors may need to be
    /// recomputed. In addition, moving static actors will not interact correctly with dynamic actors or joints.
    ///
    /// To directly control an actor's position and have it correctly interact with dynamic bodies and joints, create a dynamic
    /// body with the PxRigidBodyFlag::eKINEMATIC flag, then use the setKinematicTarget() commands to define its path.
    ///
    /// Even when moving dynamic actors, exercise restraint in making use of this method. Where possible, avoid:
    ///
    /// moving actors into other actors, thus causing overlap (an invalid physical state)
    ///
    /// moving an actor that is connected by a joint to another away from the other (thus causing joint error)
    ///
    /// Sleeping:
    /// This call wakes dynamic actors if they are sleeping and the autowake parameter is true (default).
    @(link_name = "PxRigidActor_setGlobalPose_mut")
    rigid_actor_set_global_pose_mut :: proc(self_: ^RigidActor, #by_ptr pose: Transform, autowake: _c.bool) ---

    /// Attach a shape to an actor
    ///
    /// This call will increment the reference count of the shape.
    ///
    /// Mass properties of dynamic rigid actors will not automatically be recomputed
    /// to reflect the new mass distribution implied by the shape. Follow this call with a call to
    /// the PhysX extensions method [`PxRigidBodyExt::updateMassAndInertia`]() to do that.
    ///
    /// Attaching a triangle mesh, heightfield or plane geometry shape configured as eSIMULATION_SHAPE is not supported for
    /// non-kinematic PxRigidDynamic instances.
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the actor up automatically.
    ///
    /// True if success.
    @(link_name = "PxRigidActor_attachShape_mut")
    rigid_actor_attach_shape_mut :: proc(self_: ^RigidActor, shape: ^Shape) -> _c.bool ---

    /// Detach a shape from an actor.
    ///
    /// This will also decrement the reference count of the PxShape, and if the reference count is zero, will cause it to be deleted.
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the actor up automatically.
    @(link_name = "PxRigidActor_detachShape_mut")
    rigid_actor_detach_shape_mut :: proc(self_: ^RigidActor, shape: ^Shape, wakeOnLostTouch: _c.bool) ---

    /// Returns the number of shapes assigned to the actor.
    ///
    /// You can use [`getShapes`]() to retrieve the shape pointers.
    ///
    /// Number of shapes associated with this actor.
    @(link_name = "PxRigidActor_getNbShapes")
    rigid_actor_get_nb_shapes :: proc(self_: ^RigidActor) -> _c.uint32_t ---

    /// Retrieve all the shape pointers belonging to the actor.
    ///
    /// These are the shapes used by the actor for collision detection.
    ///
    /// You can retrieve the number of shape pointers by calling [`getNbShapes`]()
    ///
    /// Note: Removing shapes with [`PxShape::release`]() will invalidate the pointer of the released shape.
    ///
    /// Number of shape pointers written to the buffer.
    @(link_name = "PxRigidActor_getShapes")
    rigid_actor_get_shapes :: proc(self_: ^RigidActor, userBuffer: [^]^Shape, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Returns the number of constraint shaders attached to the actor.
    ///
    /// You can use [`getConstraints`]() to retrieve the constraint shader pointers.
    ///
    /// Number of constraint shaders attached to this actor.
    @(link_name = "PxRigidActor_getNbConstraints")
    rigid_actor_get_nb_constraints :: proc(self_: ^RigidActor) -> _c.uint32_t ---

    /// Retrieve all the constraint shader pointers belonging to the actor.
    ///
    /// You can retrieve the number of constraint shader pointers by calling [`getNbConstraints`]()
    ///
    /// Note: Removing constraint shaders with [`PxConstraint::release`]() will invalidate the pointer of the released constraint.
    ///
    /// Number of constraint shader pointers written to the buffer.
    @(link_name = "PxRigidActor_getConstraints")
    rigid_actor_get_constraints :: proc(self_: ^RigidActor, userBuffer: [^]^Constraint, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    @(link_name = "PxNodeIndex_new")
    node_index_new :: proc(id: _c.uint32_t, articLinkId: _c.uint32_t) -> NodeIndex ---

    @(link_name = "PxNodeIndex_new_1")
    node_index_new_1 :: proc(id: _c.uint32_t) -> NodeIndex ---

    @(link_name = "PxNodeIndex_index")
    node_index_index :: proc(self_: ^NodeIndex) -> _c.uint32_t ---

    @(link_name = "PxNodeIndex_articulationLinkId")
    node_index_articulation_link_id :: proc(self_: ^NodeIndex) -> _c.uint32_t ---

    @(link_name = "PxNodeIndex_isArticulation")
    node_index_is_articulation :: proc(self_: ^NodeIndex) -> _c.uint32_t ---

    @(link_name = "PxNodeIndex_isStaticBody")
    node_index_is_static_body :: proc(self_: ^NodeIndex) -> _c.bool ---

    @(link_name = "PxNodeIndex_isValid")
    node_index_is_valid :: proc(self_: ^NodeIndex) -> _c.bool ---

    @(link_name = "PxNodeIndex_setIndices_mut")
    node_index_set_indices_mut :: proc(self_: ^NodeIndex, index: _c.uint32_t, articLinkId: _c.uint32_t) ---

    @(link_name = "PxNodeIndex_setIndices_mut_1")
    node_index_set_indices_mut_1 :: proc(self_: ^NodeIndex, index: _c.uint32_t) ---

    @(link_name = "PxNodeIndex_getInd")
    node_index_get_ind :: proc(self_: ^NodeIndex) -> _c.uint64_t ---

    /// Sets the pose of the center of mass relative to the actor.
    ///
    /// Changing this transform will not move the actor in the world!
    ///
    /// Setting an unrealistic center of mass which is a long way from the body can make it difficult for
    /// the SDK to solve constraints. Perhaps leading to instability and jittering bodies.
    ///
    /// Default:
    /// the identity transform
    @(link_name = "PxRigidBody_setCMassLocalPose_mut")
    rigid_body_set_c_mass_local_pose_mut :: proc(self_: ^RigidBody, #by_ptr pose: Transform) ---

    /// Retrieves the center of mass pose relative to the actor frame.
    ///
    /// The center of mass pose relative to the actor frame.
    @(link_name = "PxRigidBody_getCMassLocalPose")
    rigid_body_get_c_mass_local_pose :: proc(self_: ^RigidBody) -> Transform ---

    /// Sets the mass of a dynamic actor.
    ///
    /// The mass must be non-negative.
    ///
    /// setMass() does not update the inertial properties of the body, to change the inertia tensor
    /// use setMassSpaceInertiaTensor() or the PhysX extensions method [`PxRigidBodyExt::updateMassAndInertia`]().
    ///
    /// A value of 0 is interpreted as infinite mass.
    ///
    /// Values of 0 are not permitted for instances of PxArticulationLink but are permitted for instances of PxRigidDynamic.
    ///
    /// Default:
    /// 1.0
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the actor up automatically.
    @(link_name = "PxRigidBody_setMass_mut")
    rigid_body_set_mass_mut :: proc(self_: ^RigidBody, mass: _c.float) ---

    /// Retrieves the mass of the actor.
    ///
    /// A value of 0 is interpreted as infinite mass.
    ///
    /// The mass of this actor.
    @(link_name = "PxRigidBody_getMass")
    rigid_body_get_mass :: proc(self_: ^RigidBody) -> _c.float ---

    /// Retrieves the inverse mass of the actor.
    ///
    /// The inverse mass of this actor.
    @(link_name = "PxRigidBody_getInvMass")
    rigid_body_get_inv_mass :: proc(self_: ^RigidBody) -> _c.float ---

    /// Sets the inertia tensor, using a parameter specified in mass space coordinates.
    ///
    /// Note that such matrices are diagonal -- the passed vector is the diagonal.
    ///
    /// If you have a non diagonal world/actor space inertia tensor(3x3 matrix). Then you need to
    /// diagonalize it and set an appropriate mass space transform. See [`setCMassLocalPose`]().
    ///
    /// The inertia tensor elements must be non-negative.
    ///
    /// A value of 0 in an element is interpreted as infinite inertia along that axis.
    ///
    /// Values of 0 are not permitted for instances of PxArticulationLink but are permitted for instances of PxRigidDynamic.
    ///
    /// Default:
    /// (1.0, 1.0, 1.0)
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the actor up automatically.
    @(link_name = "PxRigidBody_setMassSpaceInertiaTensor_mut")
    rigid_body_set_mass_space_inertia_tensor_mut :: proc(self_: ^RigidBody, #by_ptr m: Vec3) ---

    /// Retrieves the diagonal inertia tensor of the actor relative to the mass coordinate frame.
    ///
    /// This method retrieves a mass frame inertia vector.
    ///
    /// The mass space inertia tensor of this actor.
    ///
    /// A value of 0 in an element is interpreted as infinite inertia along that axis.
    @(link_name = "PxRigidBody_getMassSpaceInertiaTensor")
    rigid_body_get_mass_space_inertia_tensor :: proc(self_: ^RigidBody) -> Vec3 ---

    /// Retrieves the diagonal inverse inertia tensor of the actor relative to the mass coordinate frame.
    ///
    /// This method retrieves a mass frame inverse inertia vector.
    ///
    /// A value of 0 in an element is interpreted as infinite inertia along that axis.
    ///
    /// The mass space inverse inertia tensor of this actor.
    @(link_name = "PxRigidBody_getMassSpaceInvInertiaTensor")
    rigid_body_get_mass_space_inv_inertia_tensor :: proc(self_: ^RigidBody) -> Vec3 ---

    /// Sets the linear damping coefficient.
    ///
    /// Zero represents no damping. The damping coefficient must be nonnegative.
    ///
    /// Default:
    /// 0.0
    @(link_name = "PxRigidBody_setLinearDamping_mut")
    rigid_body_set_linear_damping_mut :: proc(self_: ^RigidBody, linDamp: _c.float) ---

    /// Retrieves the linear damping coefficient.
    ///
    /// The linear damping coefficient associated with this actor.
    @(link_name = "PxRigidBody_getLinearDamping")
    rigid_body_get_linear_damping :: proc(self_: ^RigidBody) -> _c.float ---

    /// Sets the angular damping coefficient.
    ///
    /// Zero represents no damping.
    ///
    /// The angular damping coefficient must be nonnegative.
    ///
    /// Default:
    /// 0.05
    @(link_name = "PxRigidBody_setAngularDamping_mut")
    rigid_body_set_angular_damping_mut :: proc(self_: ^RigidBody, angDamp: _c.float) ---

    /// Retrieves the angular damping coefficient.
    ///
    /// The angular damping coefficient associated with this actor.
    @(link_name = "PxRigidBody_getAngularDamping")
    rigid_body_get_angular_damping :: proc(self_: ^RigidBody) -> _c.float ---

    /// Retrieves the linear velocity of an actor.
    ///
    /// It is not allowed to use this method while the simulation is running (except during PxScene::collide(),
    /// in PxContactModifyCallback or in contact report callbacks).
    ///
    /// The linear velocity of the actor.
    @(link_name = "PxRigidBody_getLinearVelocity")
    rigid_body_get_linear_velocity :: proc(self_: ^RigidBody) -> Vec3 ---

    /// Retrieves the angular velocity of the actor.
    ///
    /// It is not allowed to use this method while the simulation is running (except during PxScene::collide(),
    /// in PxContactModifyCallback or in contact report callbacks).
    ///
    /// The angular velocity of the actor.
    @(link_name = "PxRigidBody_getAngularVelocity")
    rigid_body_get_angular_velocity :: proc(self_: ^RigidBody) -> Vec3 ---

    /// Lets you set the maximum linear velocity permitted for this actor.
    ///
    /// With this function, you can set the  maximum linear velocity permitted for this rigid body.
    /// Higher angular velocities are clamped to this value.
    ///
    /// Note: The angular velocity is clamped to the set value
    /// before
    /// the solver, which means that
    /// the limit may still be momentarily exceeded.
    ///
    /// Default:
    /// PX_MAX_F32
    @(link_name = "PxRigidBody_setMaxLinearVelocity_mut")
    rigid_body_set_max_linear_velocity_mut :: proc(self_: ^RigidBody, maxLinVel: _c.float) ---

    /// Retrieves the maximum angular velocity permitted for this actor.
    ///
    /// The maximum allowed angular velocity for this actor.
    @(link_name = "PxRigidBody_getMaxLinearVelocity")
    rigid_body_get_max_linear_velocity :: proc(self_: ^RigidBody) -> _c.float ---

    /// Lets you set the maximum angular velocity permitted for this actor.
    ///
    /// For various internal computations, very quickly rotating actors introduce error
    /// into the simulation, which leads to undesired results.
    ///
    /// With this function, you can set the  maximum angular velocity permitted for this rigid body.
    /// Higher angular velocities are clamped to this value.
    ///
    /// Note: The angular velocity is clamped to the set value
    /// before
    /// the solver, which means that
    /// the limit may still be momentarily exceeded.
    ///
    /// Default:
    /// 100.0
    @(link_name = "PxRigidBody_setMaxAngularVelocity_mut")
    rigid_body_set_max_angular_velocity_mut :: proc(self_: ^RigidBody, maxAngVel: _c.float) ---

    /// Retrieves the maximum angular velocity permitted for this actor.
    ///
    /// The maximum allowed angular velocity for this actor.
    @(link_name = "PxRigidBody_getMaxAngularVelocity")
    rigid_body_get_max_angular_velocity :: proc(self_: ^RigidBody) -> _c.float ---

    /// Applies a force (or impulse) defined in the global coordinate frame to the actor at its center of mass.
    ///
    /// This will not induce a torque
    /// .
    ///
    /// ::PxForceMode determines if the force is to be conventional or impulsive.
    ///
    /// Each actor has an acceleration and a velocity change accumulator which are directly modified using the modes PxForceMode::eACCELERATION
    /// and PxForceMode::eVELOCITY_CHANGE respectively.  The modes PxForceMode::eFORCE and PxForceMode::eIMPULSE also modify these same
    /// accumulators and are just short hand for multiplying the vector parameter by inverse mass and then using PxForceMode::eACCELERATION and
    /// PxForceMode::eVELOCITY_CHANGE respectively.
    ///
    /// It is invalid to use this method if the actor has not been added to a scene already or if PxActorFlag::eDISABLE_SIMULATION is set.
    ///
    /// The force modes PxForceMode::eIMPULSE and PxForceMode::eVELOCITY_CHANGE can not be applied to articulation links.
    ///
    /// if this is called on an articulation link, only the link is updated, not the entire articulation.
    ///
    /// see [`PxRigidBodyExt::computeVelocityDeltaFromImpulse`] for details of how to compute the change in linear velocity that
    /// will arise from the application of an impulsive force, where an impulsive force is applied force multiplied by a timestep.
    ///
    /// Sleeping:
    /// This call wakes the actor if it is sleeping, and the autowake parameter is true (default) or the force is non-zero.
    @(link_name = "PxRigidBody_addForce_mut")
    rigid_body_add_force_mut :: proc(self_: ^RigidBody, #by_ptr force: Vec3, mode: ForceMode, autowake: _c.bool) ---

    /// Applies an impulsive torque defined in the global coordinate frame to the actor.
    ///
    /// ::PxForceMode determines if the torque is to be conventional or impulsive.
    ///
    /// Each actor has an angular acceleration and an angular velocity change accumulator which are directly modified using the modes
    /// PxForceMode::eACCELERATION and PxForceMode::eVELOCITY_CHANGE respectively.  The modes PxForceMode::eFORCE and PxForceMode::eIMPULSE
    /// also modify these same accumulators and are just short hand for multiplying the vector parameter by inverse inertia and then
    /// using PxForceMode::eACCELERATION and PxForceMode::eVELOCITY_CHANGE respectively.
    ///
    /// It is invalid to use this method if the actor has not been added to a scene already or if PxActorFlag::eDISABLE_SIMULATION is set.
    ///
    /// The force modes PxForceMode::eIMPULSE and PxForceMode::eVELOCITY_CHANGE can not be applied to articulation links.
    ///
    /// if this called on an articulation link, only the link is updated, not the entire articulation.
    ///
    /// see [`PxRigidBodyExt::computeVelocityDeltaFromImpulse`] for details of how to compute the change in angular velocity that
    /// will arise from the application of an impulsive torque, where an impulsive torque is an applied torque multiplied by a timestep.
    ///
    /// Sleeping:
    /// This call wakes the actor if it is sleeping, and the autowake parameter is true (default) or the torque is non-zero.
    @(link_name = "PxRigidBody_addTorque_mut")
    rigid_body_add_torque_mut :: proc(self_: ^RigidBody, #by_ptr torque: Vec3, mode: ForceMode, autowake: _c.bool) ---

    /// Clears the accumulated forces (sets the accumulated force back to zero).
    ///
    /// Each actor has an acceleration and a velocity change accumulator which are directly modified using the modes PxForceMode::eACCELERATION
    /// and PxForceMode::eVELOCITY_CHANGE respectively.  The modes PxForceMode::eFORCE and PxForceMode::eIMPULSE also modify these same
    /// accumulators (see PxRigidBody::addForce() for details); therefore the effect of calling clearForce(PxForceMode::eFORCE) is equivalent to calling
    /// clearForce(PxForceMode::eACCELERATION), and the effect of calling clearForce(PxForceMode::eIMPULSE) is equivalent to calling
    /// clearForce(PxForceMode::eVELOCITY_CHANGE).
    ///
    /// ::PxForceMode determines if the cleared force is to be conventional or impulsive.
    ///
    /// The force modes PxForceMode::eIMPULSE and PxForceMode::eVELOCITY_CHANGE can not be applied to articulation links.
    ///
    /// It is invalid to use this method if the actor has not been added to a scene already or if PxActorFlag::eDISABLE_SIMULATION is set.
    @(link_name = "PxRigidBody_clearForce_mut")
    rigid_body_clear_force_mut :: proc(self_: ^RigidBody, mode: ForceMode) ---

    /// Clears the impulsive torque defined in the global coordinate frame to the actor.
    ///
    /// ::PxForceMode determines if the cleared torque is to be conventional or impulsive.
    ///
    /// Each actor has an angular acceleration and a velocity change accumulator which are directly modified using the modes PxForceMode::eACCELERATION
    /// and PxForceMode::eVELOCITY_CHANGE respectively.  The modes PxForceMode::eFORCE and PxForceMode::eIMPULSE also modify these same
    /// accumulators (see PxRigidBody::addTorque() for details); therefore the effect of calling clearTorque(PxForceMode::eFORCE) is equivalent to calling
    /// clearTorque(PxForceMode::eACCELERATION), and the effect of calling clearTorque(PxForceMode::eIMPULSE) is equivalent to calling
    /// clearTorque(PxForceMode::eVELOCITY_CHANGE).
    ///
    /// The force modes PxForceMode::eIMPULSE and PxForceMode::eVELOCITY_CHANGE can not be applied to articulation links.
    ///
    /// It is invalid to use this method if the actor has not been added to a scene already or if PxActorFlag::eDISABLE_SIMULATION is set.
    @(link_name = "PxRigidBody_clearTorque_mut")
    rigid_body_clear_torque_mut :: proc(self_: ^RigidBody, mode: ForceMode) ---

    /// Sets the impulsive force and torque defined in the global coordinate frame to the actor.
    ///
    /// ::PxForceMode determines if the cleared torque is to be conventional or impulsive.
    ///
    /// The force modes PxForceMode::eIMPULSE and PxForceMode::eVELOCITY_CHANGE can not be applied to articulation links.
    ///
    /// It is invalid to use this method if the actor has not been added to a scene already or if PxActorFlag::eDISABLE_SIMULATION is set.
    @(link_name = "PxRigidBody_setForceAndTorque_mut")
    rigid_body_set_force_and_torque_mut :: proc(self_: ^RigidBody, #by_ptr force: Vec3, #by_ptr torque: Vec3, mode: ForceMode) ---

    /// Raises or clears a particular rigid body flag.
    ///
    /// See the list of flags [`PxRigidBodyFlag`]
    ///
    /// Default:
    /// no flags are set
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the actor up automatically.
    @(link_name = "PxRigidBody_setRigidBodyFlag_mut")
    rigid_body_set_rigid_body_flag_mut :: proc(self_: ^RigidBody, flag: RigidBodyFlag, value: _c.bool) ---

    @(link_name = "PxRigidBody_setRigidBodyFlags_mut")
    rigid_body_set_rigid_body_flags_mut :: proc(self_: ^RigidBody, inFlags: RigidBodyFlags_Set) ---

    /// Reads the PxRigidBody flags.
    ///
    /// See the list of flags [`PxRigidBodyFlag`]
    ///
    /// The values of the PxRigidBody flags.
    @(link_name = "PxRigidBody_getRigidBodyFlags")
    rigid_body_get_rigid_body_flags :: proc(self_: ^RigidBody) -> RigidBodyFlags_Set ---

    /// Sets the CCD minimum advance coefficient.
    ///
    /// The CCD minimum advance coefficient is a value in the range [0, 1] that is used to control the minimum amount of time a body is integrated when
    /// it has a CCD contact. The actual minimum amount of time that is integrated depends on various properties, including the relative speed and collision shapes
    /// of the bodies involved in the contact. From these properties, a numeric value is calculated that determines the maximum distance (and therefore maximum time)
    /// which these bodies could be integrated forwards that would ensure that these bodies did not pass through each-other. This value is then scaled by CCD minimum advance
    /// coefficient to determine the amount of time that will be consumed in the CCD pass.
    ///
    /// Things to consider:
    /// A large value (approaching 1) ensures that the objects will always advance some time. However, larger values increase the chances of objects gently drifting through each-other in
    /// scenes which the constraint solver can't converge, e.g. scenes where an object is being dragged through a wall with a constraint.
    /// A value of 0 ensures that the pair of objects stop at the exact time-of-impact and will not gently drift through each-other. However, with very small/thin objects initially in
    /// contact, this can lead to a large amount of time being dropped and increases the chances of jamming. Jamming occurs when the an object is persistently in contact with an object
    /// such that the time-of-impact is 0, which results in no time being advanced for those objects in that CCD pass.
    ///
    /// The chances of jamming can be reduced by increasing the number of CCD mass
    @(link_name = "PxRigidBody_setMinCCDAdvanceCoefficient_mut")
    rigid_body_set_min_ccd_advance_coefficient_mut :: proc(self_: ^RigidBody, advanceCoefficient: _c.float) ---

    /// Gets the CCD minimum advance coefficient.
    ///
    /// The value of the CCD min advance coefficient.
    @(link_name = "PxRigidBody_getMinCCDAdvanceCoefficient")
    rigid_body_get_min_ccd_advance_coefficient :: proc(self_: ^RigidBody) -> _c.float ---

    /// Sets the maximum depenetration velocity permitted to be introduced by the solver.
    /// This value controls how much velocity the solver can introduce to correct for penetrations in contacts.
    @(link_name = "PxRigidBody_setMaxDepenetrationVelocity_mut")
    rigid_body_set_max_depenetration_velocity_mut :: proc(self_: ^RigidBody, biasClamp: _c.float) ---

    /// Returns the maximum depenetration velocity the solver is permitted to introduced.
    /// This value controls how much velocity the solver can introduce to correct for penetrations in contacts.
    ///
    /// The maximum penetration bias applied by the solver.
    @(link_name = "PxRigidBody_getMaxDepenetrationVelocity")
    rigid_body_get_max_depenetration_velocity :: proc(self_: ^RigidBody) -> _c.float ---

    /// Sets a limit on the impulse that may be applied at a contact. The maximum impulse at a contact between two dynamic or kinematic
    /// bodies will be the minimum of the two limit values. For a collision between a static and a dynamic body, the impulse is limited
    /// by the value for the dynamic body.
    @(link_name = "PxRigidBody_setMaxContactImpulse_mut")
    rigid_body_set_max_contact_impulse_mut :: proc(self_: ^RigidBody, maxImpulse: _c.float) ---

    /// Returns the maximum impulse that may be applied at a contact.
    ///
    /// The maximum impulse that may be applied at a contact
    @(link_name = "PxRigidBody_getMaxContactImpulse")
    rigid_body_get_max_contact_impulse :: proc(self_: ^RigidBody) -> _c.float ---

    /// Sets a distance scale whereby the angular influence of a contact on the normal constraint in a contact is
    /// zeroed if normal.cross(offset) falls below this tolerance. Rather than acting as an absolute value, this tolerance
    /// is scaled by the ratio rXn.dot(angVel)/normal.dot(linVel) such that contacts that have relatively larger angular velocity
    /// than linear normal velocity (e.g. rolling wheels) achieve larger slop values as the angular velocity increases.
    @(link_name = "PxRigidBody_setContactSlopCoefficient_mut")
    rigid_body_set_contact_slop_coefficient_mut :: proc(self_: ^RigidBody, slopCoefficient: _c.float) ---

    /// Returns the contact slop coefficient.
    ///
    /// The contact slop coefficient.
    @(link_name = "PxRigidBody_getContactSlopCoefficient")
    rigid_body_get_contact_slop_coefficient :: proc(self_: ^RigidBody) -> _c.float ---

    /// Returns the island node index
    ///
    /// The island node index.
    @(link_name = "PxRigidBody_getInternalIslandNodeIndex")
    rigid_body_get_internal_island_node_index :: proc(self_: ^RigidBody) -> NodeIndex ---

    /// Releases the link from the articulation.
    ///
    /// Only a leaf articulation link can be released.
    ///
    /// Releasing a link is not allowed while the articulation link is in a scene. In order to release a link,
    /// remove and then re-add the corresponding articulation to the scene.
    @(link_name = "PxArticulationLink_release_mut")
    articulation_link_release_mut :: proc(self_: ^ArticulationLink) ---

    /// Gets the articulation that the link is a part of.
    ///
    /// The articulation.
    @(link_name = "PxArticulationLink_getArticulation")
    articulation_link_get_articulation :: proc(self_: ^ArticulationLink) -> ^ArticulationReducedCoordinate ---

    /// Gets the joint which connects this link to its parent.
    ///
    /// The joint connecting the link to the parent. NULL for the root link.
    @(link_name = "PxArticulationLink_getInboundJoint")
    articulation_link_get_inbound_joint :: proc(self_: ^ArticulationLink) -> ^ArticulationJointReducedCoordinate ---

    /// Gets the number of degrees of freedom of the joint which connects this link to its parent.
    ///
    /// - The root link DOF-count is defined to be 0 regardless of PxArticulationFlag::eFIX_BASE.
    /// - The return value is only valid for articulations that are in a scene.
    ///
    /// The number of degrees of freedom, or 0xFFFFFFFF if the articulation is not in a scene.
    @(link_name = "PxArticulationLink_getInboundJointDof")
    articulation_link_get_inbound_joint_dof :: proc(self_: ^ArticulationLink) -> _c.uint32_t ---

    /// Gets the number of child links.
    ///
    /// The number of child links.
    @(link_name = "PxArticulationLink_getNbChildren")
    articulation_link_get_nb_children :: proc(self_: ^ArticulationLink) -> _c.uint32_t ---

    /// Gets the low-level link index that may be used to index into members of PxArticulationCache.
    ///
    /// The return value is only valid for articulations that are in a scene.
    ///
    /// The low-level index, or 0xFFFFFFFF if the articulation is not in a scene.
    @(link_name = "PxArticulationLink_getLinkIndex")
    articulation_link_get_link_index :: proc(self_: ^ArticulationLink) -> _c.uint32_t ---

    /// Retrieves the child links.
    ///
    /// The number of articulation links written to the buffer.
    @(link_name = "PxArticulationLink_getChildren")
    articulation_link_get_children :: proc(self_: ^ArticulationLink, userBuffer: [^]^ArticulationLink, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Set the constraint-force-mixing scale term.
    ///
    /// The cfm scale term is a stabilization term that helps avoid instabilities with over-constrained
    /// configurations. It should be a small value that is multiplied by 1/mass internally to produce
    /// an additional bias added to the unit response term in the solver.
    ///
    /// Default:
    /// 0.025
    /// Range:
    /// [0, 1]
    ///
    /// This call is not allowed while the simulation is running.
    @(link_name = "PxArticulationLink_setCfmScale_mut")
    articulation_link_set_cfm_scale_mut :: proc(self_: ^ArticulationLink, cfm: _c.float) ---

    /// Get the constraint-force-mixing scale term.
    ///
    /// The constraint-force-mixing scale term.
    @(link_name = "PxArticulationLink_getCfmScale")
    articulation_link_get_cfm_scale :: proc(self_: ^ArticulationLink) -> _c.float ---

    /// Get the linear velocity of the link.
    ///
    /// - The linear velocity is with respect to the link's center of mass and not the actor frame origin.
    /// - For performance, prefer PxArticulationCache::linkVelocity to get link spatial velocities in a batch query.
    /// - When the articulation state is updated via non-cache API, use PxArticulationReducedCoordinate::updateKinematic before querying velocity.
    ///
    /// The linear velocity of the link.
    ///
    /// This call is not allowed while the simulation is running except in a split simulation during [`PxScene::collide`]() and up to #PxScene::advance(),
    /// and in PxContactModifyCallback or in contact report callbacks.
    @(link_name = "PxArticulationLink_getLinearVelocity")
    articulation_link_get_linear_velocity :: proc(self_: ^ArticulationLink) -> Vec3 ---

    /// Get the angular velocity of the link.
    ///
    /// - For performance, prefer PxArticulationCache::linkVelocity to get link spatial velocities in a batch query.
    /// - When the articulation state is updated via non-cache API, use PxArticulationReducedCoordinate::updateKinematic before querying velocity.
    ///
    /// The angular velocity of the link.
    ///
    /// This call is not allowed while the simulation is running except in a split simulation during [`PxScene::collide`]() and up to #PxScene::advance(),
    /// and in PxContactModifyCallback or in contact report callbacks.
    @(link_name = "PxArticulationLink_getAngularVelocity")
    articulation_link_get_angular_velocity :: proc(self_: ^ArticulationLink) -> Vec3 ---

    /// Returns the string name of the dynamic type.
    ///
    /// The string name.
    @(link_name = "PxArticulationLink_getConcreteTypeName")
    articulation_link_get_concrete_type_name :: proc(self_: ^ArticulationLink) -> cstring ---

    @(link_name = "PxConeLimitedConstraint_new")
    cone_limited_constraint_new :: proc() -> ConeLimitedConstraint ---

    /// Releases a PxConstraint instance.
    ///
    /// This call does not wake up the connected rigid bodies.
    @(link_name = "PxConstraint_release_mut")
    constraint_release_mut :: proc(self_: ^Constraint) ---

    /// Retrieves the scene which this constraint belongs to.
    ///
    /// Owner Scene. NULL if not part of a scene.
    @(link_name = "PxConstraint_getScene")
    constraint_get_scene :: proc(self_: ^Constraint) -> ^Scene ---

    /// Retrieves the actors for this constraint.
    @(link_name = "PxConstraint_getActors")
    constraint_get_actors :: proc(self_: ^Constraint, actor0: ^^RigidActor, actor1: ^^RigidActor) ---

    /// Sets the actors for this constraint.
    @(link_name = "PxConstraint_setActors_mut")
    constraint_set_actors_mut :: proc(self_: ^Constraint, actor0: ^RigidActor, actor1: ^RigidActor) ---

    /// Notify the scene that the constraint shader data has been updated by the application
    @(link_name = "PxConstraint_markDirty_mut")
    constraint_mark_dirty_mut :: proc(self_: ^Constraint) ---

    /// Retrieve the flags for this constraint
    ///
    /// the constraint flags
    @(link_name = "PxConstraint_getFlags")
    constraint_get_flags :: proc(self_: ^Constraint) -> ConstraintFlags_Set ---

    /// Set the flags for this constraint
    ///
    /// default: PxConstraintFlag::eDRIVE_LIMITS_ARE_FORCES
    @(link_name = "PxConstraint_setFlags_mut")
    constraint_set_flags_mut :: proc(self_: ^Constraint, flags: ConstraintFlags_Set) ---

    /// Set a flag for this constraint
    @(link_name = "PxConstraint_setFlag_mut")
    constraint_set_flag_mut :: proc(self_: ^Constraint, flag: ConstraintFlag, value: _c.bool) ---

    /// Retrieve the constraint force most recently applied to maintain this constraint.
    ///
    /// It is not allowed to use this method while the simulation is running (except during PxScene::collide(),
    /// in PxContactModifyCallback or in contact report callbacks).
    @(link_name = "PxConstraint_getForce")
    constraint_get_force :: proc(self_: ^Constraint, linear: ^Vec3, angular: ^Vec3) ---

    /// whether the constraint is valid.
    ///
    /// A constraint is valid if it has at least one dynamic rigid body or articulation link. A constraint that
    /// is not valid may not be inserted into a scene, and therefore a static actor to which an invalid constraint
    /// is attached may not be inserted into a scene.
    ///
    /// Invalid constraints arise only when an actor to which the constraint is attached has been deleted.
    @(link_name = "PxConstraint_isValid")
    constraint_is_valid :: proc(self_: ^Constraint) -> _c.bool ---

    /// Set the break force and torque thresholds for this constraint.
    ///
    /// If either the force or torque measured at the constraint exceed these thresholds the constraint will break.
    @(link_name = "PxConstraint_setBreakForce_mut")
    constraint_set_break_force_mut :: proc(self_: ^Constraint, linear: _c.float, angular: _c.float) ---

    /// Retrieve the constraint break force and torque thresholds
    @(link_name = "PxConstraint_getBreakForce")
    constraint_get_break_force :: proc(self_: ^Constraint, linear: ^_c.float, angular: ^_c.float) ---

    /// Set the minimum response threshold for a constraint row
    ///
    /// When using mass modification for a joint or infinite inertia for a jointed body, very stiff solver constraints can be generated which
    /// can destabilize simulation. Setting this value to a small positive value (e.g. 1e-8) will cause constraint rows to be ignored if very
    /// large changes in impulses will generate only small changes in velocity. When setting this value, also set
    /// PxConstraintFlag::eDISABLE_PREPROCESSING. The solver accuracy for this joint may be reduced.
    @(link_name = "PxConstraint_setMinResponseThreshold_mut")
    constraint_set_min_response_threshold_mut :: proc(self_: ^Constraint, threshold: _c.float) ---

    /// Retrieve the constraint break force and torque thresholds
    ///
    /// the minimum response threshold for a constraint row
    @(link_name = "PxConstraint_getMinResponseThreshold")
    constraint_get_min_response_threshold :: proc(self_: ^Constraint) -> _c.float ---

    /// Fetch external owner of the constraint.
    ///
    /// Provides a reference to the external owner of a constraint and a unique owner type ID.
    ///
    /// Reference to the external object which owns the constraint.
    @(link_name = "PxConstraint_getExternalReference_mut")
    constraint_get_external_reference_mut :: proc(self_: ^Constraint, typeID: ^_c.uint32_t) -> rawptr ---

    /// Set the constraint functions for this constraint
    @(link_name = "PxConstraint_setConstraintFunctions_mut")
    constraint_set_constraint_functions_mut :: proc(self_: ^Constraint, connector: ^ConstraintConnector, #by_ptr shaders: ConstraintShaderTable) ---

    @(link_name = "PxConstraint_getConcreteTypeName")
    constraint_get_concrete_type_name :: proc(self_: ^Constraint) -> cstring ---

    /// Constructor
    @(link_name = "PxContactStreamIterator_new")
    contact_stream_iterator_new :: proc(contactPatches: ^_c.uint8_t, contactPoints: ^_c.uint8_t, contactFaceIndices: ^_c.uint32_t, nbPatches: _c.uint32_t, nbContacts: _c.uint32_t) -> ContactStreamIterator ---

    /// Returns whether there are more patches in this stream.
    ///
    /// Whether there are more patches in this stream.
    @(link_name = "PxContactStreamIterator_hasNextPatch")
    contact_stream_iterator_has_next_patch :: proc(self_: ^ContactStreamIterator) -> _c.bool ---

    /// Returns the total contact count.
    ///
    /// Total contact count.
    @(link_name = "PxContactStreamIterator_getTotalContactCount")
    contact_stream_iterator_get_total_contact_count :: proc(self_: ^ContactStreamIterator) -> _c.uint32_t ---

    /// Returns the total patch count.
    ///
    /// Total patch count.
    @(link_name = "PxContactStreamIterator_getTotalPatchCount")
    contact_stream_iterator_get_total_patch_count :: proc(self_: ^ContactStreamIterator) -> _c.uint32_t ---

    /// Advances iterator to next contact patch.
    @(link_name = "PxContactStreamIterator_nextPatch_mut")
    contact_stream_iterator_next_patch_mut :: proc(self_: ^ContactStreamIterator) ---

    /// Returns if the current patch has more contacts.
    ///
    /// If there are more contacts in the current patch.
    @(link_name = "PxContactStreamIterator_hasNextContact")
    contact_stream_iterator_has_next_contact :: proc(self_: ^ContactStreamIterator) -> _c.bool ---

    /// Advances to the next contact in the patch.
    @(link_name = "PxContactStreamIterator_nextContact_mut")
    contact_stream_iterator_next_contact_mut :: proc(self_: ^ContactStreamIterator) ---

    /// Gets the current contact's normal
    ///
    /// The current contact's normal.
    @(link_name = "PxContactStreamIterator_getContactNormal")
    contact_stream_iterator_get_contact_normal :: proc(self_: ^ContactStreamIterator) -> ^Vec3 ---

    /// Gets the inverse mass scale for body 0.
    ///
    /// The inverse mass scale for body 0.
    @(link_name = "PxContactStreamIterator_getInvMassScale0")
    contact_stream_iterator_get_inv_mass_scale0 :: proc(self_: ^ContactStreamIterator) -> _c.float ---

    /// Gets the inverse mass scale for body 1.
    ///
    /// The inverse mass scale for body 1.
    @(link_name = "PxContactStreamIterator_getInvMassScale1")
    contact_stream_iterator_get_inv_mass_scale1 :: proc(self_: ^ContactStreamIterator) -> _c.float ---

    /// Gets the inverse inertia scale for body 0.
    ///
    /// The inverse inertia scale for body 0.
    @(link_name = "PxContactStreamIterator_getInvInertiaScale0")
    contact_stream_iterator_get_inv_inertia_scale0 :: proc(self_: ^ContactStreamIterator) -> _c.float ---

    /// Gets the inverse inertia scale for body 1.
    ///
    /// The inverse inertia scale for body 1.
    @(link_name = "PxContactStreamIterator_getInvInertiaScale1")
    contact_stream_iterator_get_inv_inertia_scale1 :: proc(self_: ^ContactStreamIterator) -> _c.float ---

    /// Gets the contact's max impulse.
    ///
    /// The contact's max impulse.
    @(link_name = "PxContactStreamIterator_getMaxImpulse")
    contact_stream_iterator_get_max_impulse :: proc(self_: ^ContactStreamIterator) -> _c.float ---

    /// Gets the contact's target velocity.
    ///
    /// The contact's target velocity.
    @(link_name = "PxContactStreamIterator_getTargetVel")
    contact_stream_iterator_get_target_vel :: proc(self_: ^ContactStreamIterator) -> ^Vec3 ---

    /// Gets the contact's contact point.
    ///
    /// The contact's contact point.
    @(link_name = "PxContactStreamIterator_getContactPoint")
    contact_stream_iterator_get_contact_point :: proc(self_: ^ContactStreamIterator) -> ^Vec3 ---

    /// Gets the contact's separation.
    ///
    /// The contact's separation.
    @(link_name = "PxContactStreamIterator_getSeparation")
    contact_stream_iterator_get_separation :: proc(self_: ^ContactStreamIterator) -> _c.float ---

    /// Gets the contact's face index for shape 0.
    ///
    /// The contact's face index for shape 0.
    @(link_name = "PxContactStreamIterator_getFaceIndex0")
    contact_stream_iterator_get_face_index0 :: proc(self_: ^ContactStreamIterator) -> _c.uint32_t ---

    /// Gets the contact's face index for shape 1.
    ///
    /// The contact's face index for shape 1.
    @(link_name = "PxContactStreamIterator_getFaceIndex1")
    contact_stream_iterator_get_face_index1 :: proc(self_: ^ContactStreamIterator) -> _c.uint32_t ---

    /// Gets the contact's static friction coefficient.
    ///
    /// The contact's static friction coefficient.
    @(link_name = "PxContactStreamIterator_getStaticFriction")
    contact_stream_iterator_get_static_friction :: proc(self_: ^ContactStreamIterator) -> _c.float ---

    /// Gets the contact's dynamic friction coefficient.
    ///
    /// The contact's dynamic friction coefficient.
    @(link_name = "PxContactStreamIterator_getDynamicFriction")
    contact_stream_iterator_get_dynamic_friction :: proc(self_: ^ContactStreamIterator) -> _c.float ---

    /// Gets the contact's restitution coefficient.
    ///
    /// The contact's restitution coefficient.
    @(link_name = "PxContactStreamIterator_getRestitution")
    contact_stream_iterator_get_restitution :: proc(self_: ^ContactStreamIterator) -> _c.float ---

    /// Gets the contact's damping value.
    ///
    /// The contact's damping value.
    @(link_name = "PxContactStreamIterator_getDamping")
    contact_stream_iterator_get_damping :: proc(self_: ^ContactStreamIterator) -> _c.float ---

    /// Gets the contact's material flags.
    ///
    /// The contact's material flags.
    @(link_name = "PxContactStreamIterator_getMaterialFlags")
    contact_stream_iterator_get_material_flags :: proc(self_: ^ContactStreamIterator) -> _c.uint32_t ---

    /// Gets the contact's material index for shape 0.
    ///
    /// The contact's material index for shape 0.
    @(link_name = "PxContactStreamIterator_getMaterialIndex0")
    contact_stream_iterator_get_material_index0 :: proc(self_: ^ContactStreamIterator) -> _c.uint16_t ---

    /// Gets the contact's material index for shape 1.
    ///
    /// The contact's material index for shape 1.
    @(link_name = "PxContactStreamIterator_getMaterialIndex1")
    contact_stream_iterator_get_material_index1 :: proc(self_: ^ContactStreamIterator) -> _c.uint16_t ---

    /// Advances the contact stream iterator to a specific contact index.
    ///
    /// True if advancing was possible
    @(link_name = "PxContactStreamIterator_advanceToIndex_mut")
    contact_stream_iterator_advance_to_index_mut :: proc(self_: ^ContactStreamIterator, initialIndex: _c.uint32_t) -> _c.bool ---

    /// Get the position of a specific contact point in the set.
    ///
    /// Position to the requested point in world space
    @(link_name = "PxContactSet_getPoint")
    contact_set_get_point :: proc(self_: ^ContactSet, i: _c.uint32_t) -> ^Vec3 ---

    /// Alter the position of a specific contact point in the set.
    @(link_name = "PxContactSet_setPoint_mut")
    contact_set_set_point_mut :: proc(self_: ^ContactSet, i: _c.uint32_t, #by_ptr p: Vec3) ---

    /// Get the contact normal of a specific contact point in the set.
    ///
    /// The requested normal in world space
    @(link_name = "PxContactSet_getNormal")
    contact_set_get_normal :: proc(self_: ^ContactSet, i: _c.uint32_t) -> ^Vec3 ---

    /// Alter the contact normal of a specific contact point in the set.
    ///
    /// Changing the normal can cause contact points to be ignored.
    @(link_name = "PxContactSet_setNormal_mut")
    contact_set_set_normal_mut :: proc(self_: ^ContactSet, i: _c.uint32_t, #by_ptr n: Vec3) ---

    /// Get the separation distance of a specific contact point in the set.
    ///
    /// The separation. Negative implies penetration.
    @(link_name = "PxContactSet_getSeparation")
    contact_set_get_separation :: proc(self_: ^ContactSet, i: _c.uint32_t) -> _c.float ---

    /// Alter the separation of a specific contact point in the set.
    @(link_name = "PxContactSet_setSeparation_mut")
    contact_set_set_separation_mut :: proc(self_: ^ContactSet, i: _c.uint32_t, s: _c.float) ---

    /// Get the target velocity of a specific contact point in the set.
    ///
    /// The target velocity in world frame
    @(link_name = "PxContactSet_getTargetVelocity")
    contact_set_get_target_velocity :: proc(self_: ^ContactSet, i: _c.uint32_t) -> ^Vec3 ---

    /// Alter the target velocity of a specific contact point in the set.
    @(link_name = "PxContactSet_setTargetVelocity_mut")
    contact_set_set_target_velocity_mut :: proc(self_: ^ContactSet, i: _c.uint32_t, #by_ptr v: Vec3) ---

    /// Get the face index with respect to the first shape of the pair for a specific contact point in the set.
    ///
    /// The face index of the first shape
    ///
    /// At the moment, the first shape is never a tri-mesh, therefore this function always returns PXC_CONTACT_NO_FACE_INDEX
    @(link_name = "PxContactSet_getInternalFaceIndex0")
    contact_set_get_internal_face_index0 :: proc(self_: ^ContactSet, i: _c.uint32_t) -> _c.uint32_t ---

    /// Get the face index with respect to the second shape of the pair for a specific contact point in the set.
    ///
    /// The face index of the second shape
    @(link_name = "PxContactSet_getInternalFaceIndex1")
    contact_set_get_internal_face_index1 :: proc(self_: ^ContactSet, i: _c.uint32_t) -> _c.uint32_t ---

    /// Get the maximum impulse for a specific contact point in the set.
    ///
    /// The maximum impulse
    @(link_name = "PxContactSet_getMaxImpulse")
    contact_set_get_max_impulse :: proc(self_: ^ContactSet, i: _c.uint32_t) -> _c.float ---

    /// Alter the maximum impulse for a specific contact point in the set.
    ///
    /// Must be nonnegative. If set to zero, the contact point will be ignored
    @(link_name = "PxContactSet_setMaxImpulse_mut")
    contact_set_set_max_impulse_mut :: proc(self_: ^ContactSet, i: _c.uint32_t, s: _c.float) ---

    /// Get the restitution coefficient for a specific contact point in the set.
    ///
    /// The restitution coefficient
    @(link_name = "PxContactSet_getRestitution")
    contact_set_get_restitution :: proc(self_: ^ContactSet, i: _c.uint32_t) -> _c.float ---

    /// Alter the restitution coefficient for a specific contact point in the set.
    ///
    /// Valid ranges [0,1]
    @(link_name = "PxContactSet_setRestitution_mut")
    contact_set_set_restitution_mut :: proc(self_: ^ContactSet, i: _c.uint32_t, r: _c.float) ---

    /// Get the static friction coefficient for a specific contact point in the set.
    ///
    /// The friction coefficient (dimensionless)
    @(link_name = "PxContactSet_getStaticFriction")
    contact_set_get_static_friction :: proc(self_: ^ContactSet, i: _c.uint32_t) -> _c.float ---

    /// Alter the static friction coefficient for a specific contact point in the set.
    @(link_name = "PxContactSet_setStaticFriction_mut")
    contact_set_set_static_friction_mut :: proc(self_: ^ContactSet, i: _c.uint32_t, f: _c.float) ---

    /// Get the static friction coefficient for a specific contact point in the set.
    ///
    /// The friction coefficient
    @(link_name = "PxContactSet_getDynamicFriction")
    contact_set_get_dynamic_friction :: proc(self_: ^ContactSet, i: _c.uint32_t) -> _c.float ---

    /// Alter the static dynamic coefficient for a specific contact point in the set.
    @(link_name = "PxContactSet_setDynamicFriction_mut")
    contact_set_set_dynamic_friction_mut :: proc(self_: ^ContactSet, i: _c.uint32_t, f: _c.float) ---

    /// Ignore the contact point.
    ///
    /// If a contact point is ignored then no force will get applied at this point. This can be used to disable collision in certain areas of a shape, for example.
    @(link_name = "PxContactSet_ignore_mut")
    contact_set_ignore_mut :: proc(self_: ^ContactSet, i: _c.uint32_t) ---

    /// The number of contact points in the set.
    @(link_name = "PxContactSet_size")
    contact_set_size :: proc(self_: ^ContactSet) -> _c.uint32_t ---

    /// Returns the invMassScale of body 0
    ///
    /// A value
    /// <
    /// 1.0 makes this contact treat the body as if it had larger mass. A value of 0.f makes this contact
    /// treat the body as if it had infinite mass. Any value > 1.f makes this contact treat the body as if it had smaller mass.
    @(link_name = "PxContactSet_getInvMassScale0")
    contact_set_get_inv_mass_scale0 :: proc(self_: ^ContactSet) -> _c.float ---

    /// Returns the invMassScale of body 1
    ///
    /// A value
    /// <
    /// 1.0 makes this contact treat the body as if it had larger mass. A value of 0.f makes this contact
    /// treat the body as if it had infinite mass. Any value > 1.f makes this contact treat the body as if it had smaller mass.
    @(link_name = "PxContactSet_getInvMassScale1")
    contact_set_get_inv_mass_scale1 :: proc(self_: ^ContactSet) -> _c.float ---

    /// Returns the invInertiaScale of body 0
    ///
    /// A value
    /// <
    /// 1.0 makes this contact treat the body as if it had larger inertia. A value of 0.f makes this contact
    /// treat the body as if it had infinite inertia. Any value > 1.f makes this contact treat the body as if it had smaller inertia.
    @(link_name = "PxContactSet_getInvInertiaScale0")
    contact_set_get_inv_inertia_scale0 :: proc(self_: ^ContactSet) -> _c.float ---

    /// Returns the invInertiaScale of body 1
    ///
    /// A value
    /// <
    /// 1.0 makes this contact treat the body as if it had larger inertia. A value of 0.f makes this contact
    /// treat the body as if it had infinite inertia. Any value > 1.f makes this contact treat the body as if it had smaller inertia.
    @(link_name = "PxContactSet_getInvInertiaScale1")
    contact_set_get_inv_inertia_scale1 :: proc(self_: ^ContactSet) -> _c.float ---

    /// Sets the invMassScale of body 0
    ///
    /// This can be set to any value in the range [0, PX_MAX_F32). A value
    /// <
    /// 1.0 makes this contact treat the body as if it had larger mass. A value of 0.f makes this contact
    /// treat the body as if it had infinite mass. Any value > 1.f makes this contact treat the body as if it had smaller mass.
    @(link_name = "PxContactSet_setInvMassScale0_mut")
    contact_set_set_inv_mass_scale0_mut :: proc(self_: ^ContactSet, scale: _c.float) ---

    /// Sets the invMassScale of body 1
    ///
    /// This can be set to any value in the range [0, PX_MAX_F32). A value
    /// <
    /// 1.0 makes this contact treat the body as if it had larger mass. A value of 0.f makes this contact
    /// treat the body as if it had infinite mass. Any value > 1.f makes this contact treat the body as if it had smaller mass.
    @(link_name = "PxContactSet_setInvMassScale1_mut")
    contact_set_set_inv_mass_scale1_mut :: proc(self_: ^ContactSet, scale: _c.float) ---

    /// Sets the invInertiaScale of body 0
    ///
    /// This can be set to any value in the range [0, PX_MAX_F32). A value
    /// <
    /// 1.0 makes this contact treat the body as if it had larger inertia. A value of 0.f makes this contact
    /// treat the body as if it had infinite inertia. Any value > 1.f makes this contact treat the body as if it had smaller inertia.
    @(link_name = "PxContactSet_setInvInertiaScale0_mut")
    contact_set_set_inv_inertia_scale0_mut :: proc(self_: ^ContactSet, scale: _c.float) ---

    /// Sets the invInertiaScale of body 1
    ///
    /// This can be set to any value in the range [0, PX_MAX_F32). A value
    /// <
    /// 1.0 makes this contact treat the body as if it had larger inertia. A value of 0.f makes this contact
    /// treat the body as if it had infinite inertia. Any value > 1.f makes this contact treat the body as if it had smaller inertia.
    @(link_name = "PxContactSet_setInvInertiaScale1_mut")
    contact_set_set_inv_inertia_scale1_mut :: proc(self_: ^ContactSet, scale: _c.float) ---

    /// Passes modifiable arrays of contacts to the application.
    ///
    /// The initial contacts are regenerated from scratch each frame by collision detection.
    ///
    /// The number of contacts can not be changed, so you cannot add your own contacts.  You may however
    /// disable contacts using PxContactSet::ignore().
    @(link_name = "PxContactModifyCallback_onContactModify_mut")
    contact_modify_callback_on_contact_modify_mut :: proc(self_: ^ContactModifyCallback, pairs: ^ContactModifyPair, count: _c.uint32_t) ---

    /// Passes modifiable arrays of contacts to the application.
    ///
    /// The initial contacts are regenerated from scratch each frame by collision detection.
    ///
    /// The number of contacts can not be changed, so you cannot add your own contacts.  You may however
    /// disable contacts using PxContactSet::ignore().
    @(link_name = "PxCCDContactModifyCallback_onCCDContactModify_mut")
    ccd_contact_modify_callback_on_ccd_contact_modify_mut :: proc(self_: ^CCDContactModifyCallback, pairs: ^ContactModifyPair, count: _c.uint32_t) ---

    /// Notification if an object or its memory gets released
    ///
    /// If release() gets called on a PxBase object, an eUSER_RELEASE event will get fired immediately. The object state can be queried in the callback but
    /// it is not allowed to change the state. Furthermore, when reading from the object it is the user's responsibility to make sure that no other thread
    /// is writing at the same time to the object (this includes the simulation itself, i.e., [`PxScene::fetchResults`]() must not get called at the same time).
    ///
    /// Calling release() on a PxBase object does not necessarily trigger its destructor immediately. For example, the object can be shared and might still
    /// be referenced by other objects or the simulation might still be running and accessing the object state. In such cases the destructor will be called
    /// as soon as it is safe to do so. After the destruction of the object and its memory, an eMEMORY_RELEASE event will get fired. In this case it is not
    /// allowed to dereference the object pointer in the callback.
    @(link_name = "PxDeletionListener_onRelease_mut")
    deletion_listener_on_release_mut :: proc(self_: ^DeletionListener, observed: ^Base, userData: rawptr, deletionEvent: DeletionEventFlag) ---

    @(link_name = "PxBaseMaterial_isKindOf")
    base_material_is_kind_of :: proc(self_: ^BaseMaterial, name: cstring) -> _c.bool ---

    /// Sets young's modulus which defines the body's stiffness
    @(link_name = "PxFEMMaterial_setYoungsModulus_mut")
    f_e_m_material_set_youngs_modulus_mut :: proc(self_: ^FEMMaterial, young: _c.float) ---

    /// Retrieves the young's modulus value.
    ///
    /// The young's modulus value.
    @(link_name = "PxFEMMaterial_getYoungsModulus")
    f_e_m_material_get_youngs_modulus :: proc(self_: ^FEMMaterial) -> _c.float ---

    /// Sets the Poisson's ratio which defines the body's volume preservation. Completely incompressible materials have a poisson ratio of 0.5. Its value should not be set to exactly 0.5 because this leads to numerical problems.
    @(link_name = "PxFEMMaterial_setPoissons_mut")
    f_e_m_material_set_poissons_mut :: proc(self_: ^FEMMaterial, poisson: _c.float) ---

    /// Retrieves the Poisson's ratio.
    ///
    /// The Poisson's ratio.
    @(link_name = "PxFEMMaterial_getPoissons")
    f_e_m_material_get_poissons :: proc(self_: ^FEMMaterial) -> _c.float ---

    /// Sets the dynamic friction value which defines the strength of resistance when two objects slide relative to each other while in contact.
    @(link_name = "PxFEMMaterial_setDynamicFriction_mut")
    f_e_m_material_set_dynamic_friction_mut :: proc(self_: ^FEMMaterial, dynamicFriction: _c.float) ---

    /// Retrieves the dynamic friction value
    ///
    /// The dynamic friction value
    @(link_name = "PxFEMMaterial_getDynamicFriction")
    f_e_m_material_get_dynamic_friction :: proc(self_: ^FEMMaterial) -> _c.float ---

    @(link_name = "PxFilterData_new")
    filter_data_new :: proc(anon_param0: EMPTY) -> FilterData ---

    /// Default constructor.
    @(link_name = "PxFilterData_new_1")
    filter_data_new_1 :: proc() -> FilterData ---

    /// Constructor to set filter data initially.
    @(link_name = "PxFilterData_new_2")
    filter_data_new_2 :: proc(w0: _c.uint32_t, w1: _c.uint32_t, w2: _c.uint32_t, w3: _c.uint32_t) -> FilterData ---

    /// (re)sets the structure to the default.
    @(link_name = "PxFilterData_setToDefault_mut")
    filter_data_set_to_default_mut :: proc(self_: ^FilterData) ---

    /// Extract filter object type from the filter attributes of a collision pair object
    ///
    /// The type of the collision pair object.
    @(link_name = "phys_PxGetFilterObjectType")
    get_filter_object_type :: proc(attr: _c.uint32_t) -> FilterObjectType ---

    /// Specifies whether the collision object belongs to a kinematic rigid body
    ///
    /// True if the object belongs to a kinematic rigid body, else false
    @(link_name = "phys_PxFilterObjectIsKinematic")
    filter_object_is_kinematic :: proc(attr: _c.uint32_t) -> _c.bool ---

    /// Specifies whether the collision object is a trigger shape
    ///
    /// True if the object is a trigger shape, else false
    @(link_name = "phys_PxFilterObjectIsTrigger")
    filter_object_is_trigger :: proc(attr: _c.uint32_t) -> _c.bool ---

    /// Filter method to specify how a pair of potentially colliding objects should be processed.
    ///
    /// This method gets called when the filter flags returned by the filter shader (see [`PxSimulationFilterShader`])
    /// indicate that the filter callback should be invoked ([`PxFilterFlag::eCALLBACK`] or #PxFilterFlag::eNOTIFY set).
    /// Return the PxFilterFlag flags and set the PxPairFlag flags to define what the simulation should do with the given
    /// collision pair.
    ///
    /// Filter flags defining whether the pair should be discarded, temporarily ignored or processed and whether the pair
    /// should be tracked and send a report on pair deletion through the filter callback
    @(link_name = "PxSimulationFilterCallback_pairFound_mut")
    simulation_filter_callback_pair_found_mut :: proc(self_: ^SimulationFilterCallback, pairID: _c.uint32_t, attributes0: _c.uint32_t, filterData0: FilterData, a0: ^Actor, s0: ^Shape, attributes1: _c.uint32_t, filterData1: FilterData, a1: ^Actor, s1: ^Shape, pairFlags: ^PairFlags_Set) -> FilterFlags_Set ---

    /// Callback to inform that a tracked collision pair is gone.
    ///
    /// This method gets called when a collision pair disappears or gets re-filtered. Only applies to
    /// collision pairs which have been marked as filter callback pairs ([`PxFilterFlag::eNOTIFY`] set in #pairFound()).
    @(link_name = "PxSimulationFilterCallback_pairLost_mut")
    simulation_filter_callback_pair_lost_mut :: proc(self_: ^SimulationFilterCallback, pairID: _c.uint32_t, attributes0: _c.uint32_t, filterData0: FilterData, attributes1: _c.uint32_t, filterData1: FilterData, objectRemoved: _c.bool) ---

    /// Callback to give the opportunity to change the filter state of a tracked collision pair.
    ///
    /// This method gets called once per simulation step to let the application change the filter and pair
    /// flags of a collision pair that has been reported in [`pairFound`]() and requested callbacks by
    /// setting [`PxFilterFlag::eNOTIFY`]. To request a change of filter status, the target pair has to be
    /// specified by its ID, the new filter and pair flags have to be provided and the method should return true.
    ///
    /// If this method changes the filter status of a collision pair and the pair should keep being tracked
    /// by the filter callbacks then [`PxFilterFlag::eNOTIFY`] has to be set.
    ///
    /// The application is responsible to ensure that this method does not get called for pairs that have been
    /// reported as lost, see [`pairLost`]().
    ///
    /// True if the changes should be applied. In this case the method will get called again. False if
    /// no more status changes should be done in the current simulation step. In that case the provided flags will be discarded.
    @(link_name = "PxSimulationFilterCallback_statusChange_mut")
    simulation_filter_callback_status_change_mut :: proc(self_: ^SimulationFilterCallback, pairID: ^_c.uint32_t, pairFlags: ^PairFlags_Set, filterFlags: ^FilterFlags_Set) -> _c.bool ---

    /// Any combination of PxDataAccessFlag::eREADABLE and PxDataAccessFlag::eWRITABLE
    @(link_name = "PxLockedData_getDataAccessFlags_mut")
    locked_data_get_data_access_flags_mut :: proc(self_: ^LockedData) -> DataAccessFlags_Set ---

    /// Unlocks the bulk data.
    @(link_name = "PxLockedData_unlock_mut")
    locked_data_unlock_mut :: proc(self_: ^LockedData) ---

    /// virtual destructor
    @(link_name = "PxLockedData_delete")
    locked_data_delete :: proc(self_: ^LockedData) ---

    /// Sets the coefficient of dynamic friction.
    ///
    /// The coefficient of dynamic friction should be in [0, PX_MAX_F32). If set to greater than staticFriction, the effective value of staticFriction will be increased to match.
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake any actors which may be affected.
    @(link_name = "PxMaterial_setDynamicFriction_mut")
    material_set_dynamic_friction_mut :: proc(self_: ^Material, coef: _c.float) ---

    /// Retrieves the DynamicFriction value.
    ///
    /// The coefficient of dynamic friction.
    @(link_name = "PxMaterial_getDynamicFriction")
    material_get_dynamic_friction :: proc(self_: ^Material) -> _c.float ---

    /// Sets the coefficient of static friction
    ///
    /// The coefficient of static friction should be in the range [0, PX_MAX_F32)
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake any actors which may be affected.
    @(link_name = "PxMaterial_setStaticFriction_mut")
    material_set_static_friction_mut :: proc(self_: ^Material, coef: _c.float) ---

    /// Retrieves the coefficient of static friction.
    ///
    /// The coefficient of static friction.
    @(link_name = "PxMaterial_getStaticFriction")
    material_get_static_friction :: proc(self_: ^Material) -> _c.float ---

    /// Sets the coefficient of restitution
    ///
    /// A coefficient of 0 makes the object bounce as little as possible, higher values up to 1.0 result in more bounce.
    ///
    /// This property is overloaded when PxMaterialFlag::eCOMPLIANT_CONTACT flag is enabled. This permits negative values for restitution to be provided.
    /// The negative values are converted into spring stiffness terms for an implicit spring simulated at the contact site, with the spring positional error defined by
    /// the contact separation value. Higher stiffness terms produce stiffer springs that behave more like a rigid contact.
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake any actors which may be affected.
    @(link_name = "PxMaterial_setRestitution_mut")
    material_set_restitution_mut :: proc(self_: ^Material, rest: _c.float) ---

    /// Retrieves the coefficient of restitution.
    ///
    /// See [`setRestitution`].
    ///
    /// The coefficient of restitution.
    @(link_name = "PxMaterial_getRestitution")
    material_get_restitution :: proc(self_: ^Material) -> _c.float ---

    /// Sets the coefficient of damping
    ///
    /// This property only affects the simulation if PxMaterialFlag::eCOMPLIANT_CONTACT is raised.
    /// Damping works together with spring stiffness (set through a negative restitution value). Spring stiffness corrects positional error while
    /// damping resists relative velocity. Setting a high damping coefficient can produce spongy contacts.
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake any actors which may be affected.
    @(link_name = "PxMaterial_setDamping_mut")
    material_set_damping_mut :: proc(self_: ^Material, damping: _c.float) ---

    /// Retrieves the coefficient of damping.
    ///
    /// See [`setDamping`].
    ///
    /// The coefficient of damping.
    @(link_name = "PxMaterial_getDamping")
    material_get_damping :: proc(self_: ^Material) -> _c.float ---

    /// Raises or clears a particular material flag.
    ///
    /// See the list of flags [`PxMaterialFlag`]
    ///
    /// Default:
    /// eIMPROVED_PATCH_FRICTION
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake any actors which may be affected.
    @(link_name = "PxMaterial_setFlag_mut")
    material_set_flag_mut :: proc(self_: ^Material, flag: MaterialFlag, b: _c.bool) ---

    /// sets all the material flags.
    ///
    /// See the list of flags [`PxMaterialFlag`]
    ///
    /// Default:
    /// eIMPROVED_PATCH_FRICTION
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake any actors which may be affected.
    @(link_name = "PxMaterial_setFlags_mut")
    material_set_flags_mut :: proc(self_: ^Material, flags: MaterialFlags_Set) ---

    /// Retrieves the flags. See [`PxMaterialFlag`].
    ///
    /// The material flags.
    @(link_name = "PxMaterial_getFlags")
    material_get_flags :: proc(self_: ^Material) -> MaterialFlags_Set ---

    /// Sets the friction combine mode.
    ///
    /// See the enum ::PxCombineMode .
    ///
    /// Default:
    /// PxCombineMode::eAVERAGE
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake any actors which may be affected.
    @(link_name = "PxMaterial_setFrictionCombineMode_mut")
    material_set_friction_combine_mode_mut :: proc(self_: ^Material, combMode: CombineMode) ---

    /// Retrieves the friction combine mode.
    ///
    /// See [`setFrictionCombineMode`].
    ///
    /// The friction combine mode for this material.
    @(link_name = "PxMaterial_getFrictionCombineMode")
    material_get_friction_combine_mode :: proc(self_: ^Material) -> CombineMode ---

    /// Sets the restitution combine mode.
    ///
    /// See the enum ::PxCombineMode .
    ///
    /// Default:
    /// PxCombineMode::eAVERAGE
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake any actors which may be affected.
    @(link_name = "PxMaterial_setRestitutionCombineMode_mut")
    material_set_restitution_combine_mode_mut :: proc(self_: ^Material, combMode: CombineMode) ---

    /// Retrieves the restitution combine mode.
    ///
    /// See [`setRestitutionCombineMode`].
    ///
    /// The coefficient of restitution combine mode for this material.
    @(link_name = "PxMaterial_getRestitutionCombineMode")
    material_get_restitution_combine_mode :: proc(self_: ^Material) -> CombineMode ---

    @(link_name = "PxMaterial_getConcreteTypeName")
    material_get_concrete_type_name :: proc(self_: ^Material) -> cstring ---

    /// Construct parameters with default values.
    @(link_name = "PxDiffuseParticleParams_new")
    diffuse_particle_params_new :: proc() -> DiffuseParticleParams ---

    /// (re)sets the structure to the default.
    @(link_name = "PxDiffuseParticleParams_setToDefault_mut")
    diffuse_particle_params_set_to_default_mut :: proc(self_: ^DiffuseParticleParams) ---

    /// Sets friction
    @(link_name = "PxParticleMaterial_setFriction_mut")
    particle_material_set_friction_mut :: proc(self_: ^ParticleMaterial, friction: _c.float) ---

    /// Retrieves the friction value.
    ///
    /// The friction value.
    @(link_name = "PxParticleMaterial_getFriction")
    particle_material_get_friction :: proc(self_: ^ParticleMaterial) -> _c.float ---

    /// Sets velocity damping term
    @(link_name = "PxParticleMaterial_setDamping_mut")
    particle_material_set_damping_mut :: proc(self_: ^ParticleMaterial, damping: _c.float) ---

    /// Retrieves the velocity damping term
    ///
    /// The velocity damping term.
    @(link_name = "PxParticleMaterial_getDamping")
    particle_material_get_damping :: proc(self_: ^ParticleMaterial) -> _c.float ---

    /// Sets adhesion term
    @(link_name = "PxParticleMaterial_setAdhesion_mut")
    particle_material_set_adhesion_mut :: proc(self_: ^ParticleMaterial, adhesion: _c.float) ---

    /// Retrieves the adhesion term
    ///
    /// The adhesion term.
    @(link_name = "PxParticleMaterial_getAdhesion")
    particle_material_get_adhesion :: proc(self_: ^ParticleMaterial) -> _c.float ---

    /// Sets gravity scale term
    @(link_name = "PxParticleMaterial_setGravityScale_mut")
    particle_material_set_gravity_scale_mut :: proc(self_: ^ParticleMaterial, scale: _c.float) ---

    /// Retrieves the gravity scale term
    ///
    /// The gravity scale term.
    @(link_name = "PxParticleMaterial_getGravityScale")
    particle_material_get_gravity_scale :: proc(self_: ^ParticleMaterial) -> _c.float ---

    /// Sets material adhesion radius scale. This is multiplied by the particle rest offset to compute the fall-off distance
    /// at which point adhesion ceases to operate.
    @(link_name = "PxParticleMaterial_setAdhesionRadiusScale_mut")
    particle_material_set_adhesion_radius_scale_mut :: proc(self_: ^ParticleMaterial, scale: _c.float) ---

    /// Retrieves the adhesion radius scale.
    ///
    /// The adhesion radius scale.
    @(link_name = "PxParticleMaterial_getAdhesionRadiusScale")
    particle_material_get_adhesion_radius_scale :: proc(self_: ^ParticleMaterial) -> _c.float ---

    /// Destroys the instance it is called on.
    ///
    /// Use this release method to destroy an instance of this class. Be sure
    /// to not keep a reference to this object after calling release.
    /// Avoid release calls while a scene is simulating (in between simulate() and fetchResults() calls).
    ///
    /// Note that this must be called once for each prior call to PxCreatePhysics, as
    /// there is a reference counter. Also note that you mustn't destroy the PxFoundation instance (holding the allocator, error callback etc.)
    /// until after the reference count reaches 0 and the SDK is actually removed.
    ///
    /// Releasing an SDK will also release any objects created through it (scenes, triangle meshes, convex meshes, heightfields, shapes etc.),
    /// provided the user hasn't already done so.
    ///
    /// Releasing the PxPhysics instance is a prerequisite to releasing the PxFoundation instance.
    @(link_name = "PxPhysics_release_mut")
    physics_release_mut :: proc(self_: ^Physics) ---

    /// Retrieves the Foundation instance.
    ///
    /// A reference to the Foundation object.
    @(link_name = "PxPhysics_getFoundation_mut")
    physics_get_foundation_mut :: proc(self_: ^Physics) -> ^Foundation ---

    /// Creates an aggregate with the specified maximum size and filtering hint.
    ///
    /// The previous API used "bool enableSelfCollision" which should now silently evaluates
    /// to a PxAggregateType::eGENERIC aggregate with its self-collision bit.
    ///
    /// Use PxAggregateType::eSTATIC or PxAggregateType::eKINEMATIC for aggregates that will
    /// only contain static or kinematic actors. This provides faster filtering when used in
    /// combination with PxPairFilteringMode.
    ///
    /// The new aggregate.
    @(link_name = "PxPhysics_createAggregate_mut")
    physics_create_aggregate_mut :: proc(self_: ^Physics, maxActor: _c.uint32_t, maxShape: _c.uint32_t, filterHint: _c.uint32_t) -> ^Aggregate ---

    /// Returns the simulation tolerance parameters.
    ///
    /// The current simulation tolerance parameters.
    @(link_name = "PxPhysics_getTolerancesScale")
    physics_get_tolerances_scale :: proc(self_: ^Physics) -> ^TolerancesScale ---

    /// Creates a triangle mesh object.
    ///
    /// This can then be instanced into [`PxShape`] objects.
    ///
    /// The new triangle mesh.
    @(link_name = "PxPhysics_createTriangleMesh_mut")
    physics_create_triangle_mesh_mut :: proc(self_: ^Physics, stream: ^InputStream) -> ^TriangleMesh ---

    /// Return the number of triangle meshes that currently exist.
    ///
    /// Number of triangle meshes.
    @(link_name = "PxPhysics_getNbTriangleMeshes")
    physics_get_nb_triangle_meshes :: proc(self_: ^Physics) -> _c.uint32_t ---

    /// Writes the array of triangle mesh pointers to a user buffer.
    ///
    /// Returns the number of pointers written.
    ///
    /// The ordering of the triangle meshes in the array is not specified.
    ///
    /// The number of triangle mesh pointers written to userBuffer, this should be less or equal to bufferSize.
    @(link_name = "PxPhysics_getTriangleMeshes")
    physics_get_triangle_meshes :: proc(self_: ^Physics, userBuffer: [^]^TriangleMesh, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Creates a tetrahedron mesh object.
    ///
    /// This can then be instanced into [`PxShape`] objects.
    ///
    /// The new tetrahedron mesh.
    @(link_name = "PxPhysics_createTetrahedronMesh_mut")
    physics_create_tetrahedron_mesh_mut :: proc(self_: ^Physics, stream: ^InputStream) -> ^TetrahedronMesh ---

    /// Creates a softbody mesh object.
    ///
    /// The new softbody mesh.
    @(link_name = "PxPhysics_createSoftBodyMesh_mut")
    physics_create_soft_body_mesh_mut :: proc(self_: ^Physics, stream: ^InputStream) -> ^SoftBodyMesh ---

    /// Return the number of tetrahedron meshes that currently exist.
    ///
    /// Number of tetrahedron meshes.
    @(link_name = "PxPhysics_getNbTetrahedronMeshes")
    physics_get_nb_tetrahedron_meshes :: proc(self_: ^Physics) -> _c.uint32_t ---

    /// Writes the array of tetrahedron mesh pointers to a user buffer.
    ///
    /// Returns the number of pointers written.
    ///
    /// The ordering of the tetrahedron meshes in the array is not specified.
    ///
    /// The number of tetrahedron mesh pointers written to userBuffer, this should be less or equal to bufferSize.
    @(link_name = "PxPhysics_getTetrahedronMeshes")
    physics_get_tetrahedron_meshes :: proc(self_: ^Physics, userBuffer: [^]^TetrahedronMesh, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Creates a heightfield object from previously cooked stream.
    ///
    /// This can then be instanced into [`PxShape`] objects.
    ///
    /// The new heightfield.
    @(link_name = "PxPhysics_createHeightField_mut")
    physics_create_height_field_mut :: proc(self_: ^Physics, stream: ^InputStream) -> ^HeightField ---

    /// Return the number of heightfields that currently exist.
    ///
    /// Number of heightfields.
    @(link_name = "PxPhysics_getNbHeightFields")
    physics_get_nb_height_fields :: proc(self_: ^Physics) -> _c.uint32_t ---

    /// Writes the array of heightfield pointers to a user buffer.
    ///
    /// Returns the number of pointers written.
    ///
    /// The ordering of the heightfields in the array is not specified.
    ///
    /// The number of heightfield pointers written to userBuffer, this should be less or equal to bufferSize.
    @(link_name = "PxPhysics_getHeightFields")
    physics_get_height_fields :: proc(self_: ^Physics, userBuffer: [^]^HeightField, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Creates a convex mesh object.
    ///
    /// This can then be instanced into [`PxShape`] objects.
    ///
    /// The new convex mesh.
    @(link_name = "PxPhysics_createConvexMesh_mut")
    physics_create_convex_mesh_mut :: proc(self_: ^Physics, stream: ^InputStream) -> ^ConvexMesh ---

    /// Return the number of convex meshes that currently exist.
    ///
    /// Number of convex meshes.
    @(link_name = "PxPhysics_getNbConvexMeshes")
    physics_get_nb_convex_meshes :: proc(self_: ^Physics) -> _c.uint32_t ---

    /// Writes the array of convex mesh pointers to a user buffer.
    ///
    /// Returns the number of pointers written.
    ///
    /// The ordering of the convex meshes in the array is not specified.
    ///
    /// The number of convex mesh pointers written to userBuffer, this should be less or equal to bufferSize.
    @(link_name = "PxPhysics_getConvexMeshes")
    physics_get_convex_meshes :: proc(self_: ^Physics, userBuffer: [^]^ConvexMesh, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Creates a bounding volume hierarchy.
    ///
    /// The new BVH.
    @(link_name = "PxPhysics_createBVH_mut")
    physics_create_b_v_h_mut :: proc(self_: ^Physics, stream: ^InputStream) -> ^BVH ---

    /// Return the number of bounding volume hierarchies that currently exist.
    ///
    /// Number of bounding volume hierarchies.
    @(link_name = "PxPhysics_getNbBVHs")
    physics_get_nb_b_v_hs :: proc(self_: ^Physics) -> _c.uint32_t ---

    /// Writes the array of bounding volume hierarchy pointers to a user buffer.
    ///
    /// Returns the number of pointers written.
    ///
    /// The ordering of the BVHs in the array is not specified.
    ///
    /// The number of BVH pointers written to userBuffer, this should be less or equal to bufferSize.
    @(link_name = "PxPhysics_getBVHs")
    physics_get_b_v_hs :: proc(self_: ^Physics, userBuffer: [^]^BVH, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Creates a scene.
    ///
    /// Every scene uses a Thread Local Storage slot. This imposes a platform specific limit on the
    /// number of scenes that can be created.
    ///
    /// The new scene object.
    @(link_name = "PxPhysics_createScene_mut")
    physics_create_scene_mut :: proc(self_: ^Physics, #by_ptr sceneDesc: SceneDesc) -> ^Scene ---

    /// Gets number of created scenes.
    ///
    /// The number of scenes created.
    @(link_name = "PxPhysics_getNbScenes")
    physics_get_nb_scenes :: proc(self_: ^Physics) -> _c.uint32_t ---

    /// Writes the array of scene pointers to a user buffer.
    ///
    /// Returns the number of pointers written.
    ///
    /// The ordering of the scene pointers in the array is not specified.
    ///
    /// The number of scene pointers written to userBuffer, this should be less or equal to bufferSize.
    @(link_name = "PxPhysics_getScenes")
    physics_get_scenes :: proc(self_: ^Physics, userBuffer: [^]^Scene, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Creates a static rigid actor with the specified pose and all other fields initialized
    /// to their default values.
    @(link_name = "PxPhysics_createRigidStatic_mut")
    physics_create_rigid_static_mut :: proc(self_: ^Physics, #by_ptr pose: Transform) -> ^RigidStatic ---

    /// Creates a dynamic rigid actor with the specified pose and all other fields initialized
    /// to their default values.
    @(link_name = "PxPhysics_createRigidDynamic_mut")
    physics_create_rigid_dynamic_mut :: proc(self_: ^Physics, #by_ptr pose: Transform) -> ^RigidDynamic ---

    /// Creates a pruning structure from actors.
    ///
    /// Every provided actor needs at least one shape with the eSCENE_QUERY_SHAPE flag set.
    ///
    /// Both static and dynamic actors can be provided.
    ///
    /// It is not allowed to pass in actors which are already part of a scene.
    ///
    /// Articulation links cannot be provided.
    ///
    /// Pruning structure created from given actors, or NULL if any of the actors did not comply with the above requirements.
    @(link_name = "PxPhysics_createPruningStructure_mut")
    physics_create_pruning_structure_mut :: proc(self_: ^Physics, actors: [^]^RigidActor, nbActors: _c.uint32_t) -> ^PruningStructure ---

    /// Creates a shape which may be attached to multiple actors
    ///
    /// The shape will be created with a reference count of 1.
    ///
    /// The shape
    ///
    /// Shared shapes are not mutable when they are attached to an actor
    @(link_name = "PxPhysics_createShape_mut")
    physics_create_shape_mut :: proc(self_: ^Physics, geometry: ^Geometry, #by_ptr material: Material, isExclusive: _c.bool, shapeFlags: ShapeFlags_Set) -> ^Shape ---

    /// Creates a shape which may be attached to multiple actors
    ///
    /// The shape will be created with a reference count of 1.
    ///
    /// The shape
    ///
    /// Shared shapes are not mutable when they are attached to an actor
    ///
    /// Shapes created from *SDF* triangle-mesh geometries do not support more than one material.
    @(link_name = "PxPhysics_createShape_mut_1")
    physics_create_shape_mut_1 :: proc(self_: ^Physics, geometry: ^Geometry, materials: [^]^Material, materialCount: _c.uint16_t, isExclusive: _c.bool, shapeFlags: ShapeFlags_Set) -> ^Shape ---

    /// Return the number of shapes that currently exist.
    ///
    /// Number of shapes.
    @(link_name = "PxPhysics_getNbShapes")
    physics_get_nb_shapes :: proc(self_: ^Physics) -> _c.uint32_t ---

    /// Writes the array of shape pointers to a user buffer.
    ///
    /// Returns the number of pointers written.
    ///
    /// The ordering of the shapes in the array is not specified.
    ///
    /// The number of shape pointers written to userBuffer, this should be less or equal to bufferSize.
    @(link_name = "PxPhysics_getShapes")
    physics_get_shapes :: proc(self_: ^Physics, userBuffer: [^]^Shape, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Creates a constraint shader.
    ///
    /// A constraint shader will get added automatically to the scene the two linked actors belong to. Either, but not both, of actor0 and actor1 may
    /// be NULL to denote attachment to the world.
    ///
    /// The new constraint shader.
    @(link_name = "PxPhysics_createConstraint_mut")
    physics_create_constraint_mut :: proc(self_: ^Physics, actor0: ^RigidActor, actor1: ^RigidActor, connector: ^ConstraintConnector, #by_ptr shaders: ConstraintShaderTable, dataSize: _c.uint32_t) -> ^Constraint ---

    /// Creates a reduced-coordinate articulation with all fields initialized to their default values.
    ///
    /// the new articulation
    @(link_name = "PxPhysics_createArticulationReducedCoordinate_mut")
    physics_create_articulation_reduced_coordinate_mut :: proc(self_: ^Physics) -> ^ArticulationReducedCoordinate ---

    /// Creates a new rigid body material with certain default properties.
    ///
    /// The new rigid body material.
    @(link_name = "PxPhysics_createMaterial_mut")
    physics_create_material_mut :: proc(self_: ^Physics, staticFriction: _c.float, dynamicFriction: _c.float, restitution: _c.float) -> ^Material ---

    /// Return the number of rigid body materials that currently exist.
    ///
    /// Number of rigid body materials.
    @(link_name = "PxPhysics_getNbMaterials")
    physics_get_nb_materials :: proc(self_: ^Physics) -> _c.uint32_t ---

    /// Writes the array of rigid body material pointers to a user buffer.
    ///
    /// Returns the number of pointers written.
    ///
    /// The ordering of the materials in the array is not specified.
    ///
    /// The number of material pointers written to userBuffer, this should be less or equal to bufferSize.
    @(link_name = "PxPhysics_getMaterials")
    physics_get_materials :: proc(self_: ^Physics, userBuffer: [^]^Material, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Register a deletion listener. Listeners will be called whenever an object is deleted.
    ///
    /// It is illegal to register or unregister a deletion listener while deletions are being processed.
    ///
    /// By default a registered listener will receive events from all objects. Set the restrictedObjectSet parameter to true on registration and use [`registerDeletionListenerObjects`] to restrict the received events to specific objects.
    ///
    /// The deletion events are only supported on core PhysX objects. In general, objects in extension modules do not provide this functionality, however, in the case of PxJoint objects, the underlying PxConstraint will send the events.
    @(link_name = "PxPhysics_registerDeletionListener_mut")
    physics_register_deletion_listener_mut :: proc(self_: ^Physics, observer: ^DeletionListener, #by_ptr deletionEvents: DeletionEventFlags_Set, restrictedObjectSet: _c.bool) ---

    /// Unregister a deletion listener.
    ///
    /// It is illegal to register or unregister a deletion listener while deletions are being processed.
    @(link_name = "PxPhysics_unregisterDeletionListener_mut")
    physics_unregister_deletion_listener_mut :: proc(self_: ^Physics, observer: ^DeletionListener) ---

    /// Register specific objects for deletion events.
    ///
    /// This method allows for a deletion listener to limit deletion events to specific objects only.
    ///
    /// It is illegal to register or unregister objects while deletions are being processed.
    ///
    /// The deletion listener has to be registered through [`registerDeletionListener`]() and configured to support restricted object sets prior to this method being used.
    @(link_name = "PxPhysics_registerDeletionListenerObjects_mut")
    physics_register_deletion_listener_objects_mut :: proc(self_: ^Physics, observer: ^DeletionListener, observables: [^]^Base, observableCount: _c.uint32_t) ---

    /// Unregister specific objects for deletion events.
    ///
    /// This method allows to clear previously registered objects for a deletion listener (see [`registerDeletionListenerObjects`]()).
    ///
    /// It is illegal to register or unregister objects while deletions are being processed.
    ///
    /// The deletion listener has to be registered through [`registerDeletionListener`]() and configured to support restricted object sets prior to this method being used.
    @(link_name = "PxPhysics_unregisterDeletionListenerObjects_mut")
    physics_unregister_deletion_listener_objects_mut :: proc(self_: ^Physics, observer: ^DeletionListener, observables: [^]^Base, observableCount: _c.uint32_t) ---

    /// Gets PxPhysics object insertion interface.
    ///
    /// The insertion interface is needed for PxCreateTriangleMesh, PxCooking::createTriangleMesh etc., this allows runtime mesh creation.
    @(link_name = "PxPhysics_getPhysicsInsertionCallback_mut")
    physics_get_physics_insertion_callback_mut :: proc(self_: ^Physics) -> ^InsertionCallback ---

    /// Creates an instance of the physics SDK.
    ///
    /// Creates an instance of this class. May not be a class member to avoid name mangling.
    /// Pass the constant [`PX_PHYSICS_VERSION`] as the argument.
    /// There may be only one instance of this class per process. Calling this method after an instance
    /// has been created already will result in an error message and NULL will be returned.
    ///
    /// Calling this will register all optional code modules (Articulations and HeightFields), preparing them for use.
    /// If you do not need some of these modules, consider calling PxCreateBasePhysics() instead and registering needed
    /// modules manually.
    ///
    /// PxPhysics instance on success, NULL if operation failed
    @(link_name = "phys_PxCreatePhysics")
    create_physics :: proc(version: _c.uint32_t, foundation: ^Foundation, #by_ptr scale: TolerancesScale, trackOutstandingAllocations: _c.bool, pvd: ^Pvd, omniPvd: ^OmniPvd) -> ^Physics ---

    @(link_name = "phys_PxGetPhysics")
    get_physics :: proc() -> ^Physics ---

    @(link_name = "PxActorShape_new")
    actor_shape_new :: proc() -> ActorShape ---

    @(link_name = "PxActorShape_new_1")
    actor_shape_new_1 :: proc(a: ^RigidActor, s: ^Shape) -> ActorShape ---

    /// constructor sets to default
    @(link_name = "PxQueryCache_new")
    query_cache_new :: proc() -> QueryCache ---

    /// constructor to set properties
    @(link_name = "PxQueryCache_new_1")
    query_cache_new_1 :: proc(s: ^Shape, findex: _c.uint32_t) -> QueryCache ---

    /// default constructor
    @(link_name = "PxQueryFilterData_new")
    query_filter_data_new :: proc() -> QueryFilterData ---

    /// constructor to set both filter data and filter flags
    @(link_name = "PxQueryFilterData_new_1")
    query_filter_data_new_1 :: proc(#by_ptr fd: FilterData, f: QueryFlags_Set) -> QueryFilterData ---

    /// constructor to set filter flags only
    @(link_name = "PxQueryFilterData_new_2")
    query_filter_data_new_2 :: proc(f: QueryFlags_Set) -> QueryFilterData ---

    /// This filter callback is executed before the exact intersection test if PxQueryFlag::ePREFILTER flag was set.
    ///
    /// the updated type for this hit  (see [`PxQueryHitType`])
    @(link_name = "PxQueryFilterCallback_preFilter_mut")
    query_filter_callback_pre_filter_mut :: proc(self_: ^QueryFilterCallback, #by_ptr filterData: FilterData, shape: ^Shape, actor: ^RigidActor, queryFlags: ^HitFlags_Set) -> QueryHitType ---

    /// This filter callback is executed if the exact intersection test returned true and PxQueryFlag::ePOSTFILTER flag was set.
    ///
    /// the updated hit type for this hit  (see [`PxQueryHitType`])
    @(link_name = "PxQueryFilterCallback_postFilter_mut")
    query_filter_callback_post_filter_mut :: proc(self_: ^QueryFilterCallback, #by_ptr filterData: FilterData, hit: ^QueryHit, shape: ^Shape, actor: ^RigidActor) -> QueryHitType ---

    /// virtual destructor
    @(link_name = "PxQueryFilterCallback_delete")
    query_filter_callback_delete :: proc(self_: ^QueryFilterCallback) ---

    /// Moves kinematically controlled dynamic actors through the game world.
    ///
    /// You set a dynamic actor to be kinematic using the PxRigidBodyFlag::eKINEMATIC flag
    /// with setRigidBodyFlag().
    ///
    /// The move command will result in a velocity that will move the body into
    /// the desired pose. After the move is carried out during a single time step,
    /// the velocity is returned to zero. Thus, you must continuously call
    /// this in every time step for kinematic actors so that they move along a path.
    ///
    /// This function simply stores the move destination until the next simulation
    /// step is processed, so consecutive calls will simply overwrite the stored target variable.
    ///
    /// The motion is always fully carried out.
    ///
    /// It is invalid to use this method if the actor has not been added to a scene already or if PxActorFlag::eDISABLE_SIMULATION is set.
    ///
    /// Sleeping:
    /// This call wakes the actor if it is sleeping and will set the wake counter to [`PxSceneDesc::wakeCounterResetValue`].
    @(link_name = "PxRigidDynamic_setKinematicTarget_mut")
    rigid_dynamic_set_kinematic_target_mut :: proc(self_: ^RigidDynamic, #by_ptr destination: Transform) ---

    /// Get target pose of a kinematically controlled dynamic actor.
    ///
    /// True if the actor is a kinematically controlled dynamic and the target has been set, else False.
    @(link_name = "PxRigidDynamic_getKinematicTarget")
    rigid_dynamic_get_kinematic_target :: proc(self_: ^RigidDynamic, target: ^Transform) -> _c.bool ---

    /// Returns true if this body is sleeping.
    ///
    /// When an actor does not move for a period of time, it is no longer simulated in order to save time. This state
    /// is called sleeping. However, because the object automatically wakes up when it is either touched by an awake object,
    /// or one of its properties is changed by the user, the entire sleep mechanism should be transparent to the user.
    ///
    /// In general, a dynamic rigid actor is guaranteed to be awake if at least one of the following holds:
    ///
    /// The wake counter is positive (see [`setWakeCounter`]()).
    ///
    /// The linear or angular velocity is non-zero.
    ///
    /// A non-zero force or torque has been applied.
    ///
    /// If a dynamic rigid actor is sleeping, the following state is guaranteed:
    ///
    /// The wake counter is zero.
    ///
    /// The linear and angular velocity is zero.
    ///
    /// There is no force update pending.
    ///
    /// When an actor gets inserted into a scene, it will be considered asleep if all the points above hold, else it will be treated as awake.
    ///
    /// If an actor is asleep after the call to PxScene::fetchResults() returns, it is guaranteed that the pose of the actor
    /// was not changed. You can use this information to avoid updating the transforms of associated objects.
    ///
    /// A kinematic actor is asleep unless a target pose has been set (in which case it will stay awake until two consecutive
    /// simulation steps without a target pose being set have passed). The wake counter will get set to zero or to the reset value
    /// [`PxSceneDesc::wakeCounterResetValue`] in the case where a target pose has been set to be consistent with the definitions above.
    ///
    /// It is invalid to use this method if the actor has not been added to a scene already.
    ///
    /// It is not allowed to use this method while the simulation is running.
    ///
    /// True if the actor is sleeping.
    @(link_name = "PxRigidDynamic_isSleeping")
    rigid_dynamic_is_sleeping :: proc(self_: ^RigidDynamic) -> _c.bool ---

    /// Sets the mass-normalized kinetic energy threshold below which an actor may go to sleep.
    ///
    /// Actors whose kinetic energy divided by their mass is below this threshold will be candidates for sleeping.
    ///
    /// Default:
    /// 5e-5f * PxTolerancesScale::speed * PxTolerancesScale::speed
    @(link_name = "PxRigidDynamic_setSleepThreshold_mut")
    rigid_dynamic_set_sleep_threshold_mut :: proc(self_: ^RigidDynamic, threshold: _c.float) ---

    /// Returns the mass-normalized kinetic energy below which an actor may go to sleep.
    ///
    /// The energy threshold for sleeping.
    @(link_name = "PxRigidDynamic_getSleepThreshold")
    rigid_dynamic_get_sleep_threshold :: proc(self_: ^RigidDynamic) -> _c.float ---

    /// Sets the mass-normalized kinetic energy threshold below which an actor may participate in stabilization.
    ///
    /// Actors whose kinetic energy divided by their mass is above this threshold will not participate in stabilization.
    ///
    /// This value has no effect if PxSceneFlag::eENABLE_STABILIZATION was not enabled on the PxSceneDesc.
    ///
    /// Default:
    /// 1e-5f * PxTolerancesScale::speed * PxTolerancesScale::speed
    @(link_name = "PxRigidDynamic_setStabilizationThreshold_mut")
    rigid_dynamic_set_stabilization_threshold_mut :: proc(self_: ^RigidDynamic, threshold: _c.float) ---

    /// Returns the mass-normalized kinetic energy below which an actor may participate in stabilization.
    ///
    /// Actors whose kinetic energy divided by their mass is above this threshold will not participate in stabilization.
    ///
    /// The energy threshold for participating in stabilization.
    @(link_name = "PxRigidDynamic_getStabilizationThreshold")
    rigid_dynamic_get_stabilization_threshold :: proc(self_: ^RigidDynamic) -> _c.float ---

    /// Reads the PxRigidDynamic lock flags.
    ///
    /// See the list of flags [`PxRigidDynamicLockFlag`]
    ///
    /// The values of the PxRigidDynamicLock flags.
    @(link_name = "PxRigidDynamic_getRigidDynamicLockFlags")
    rigid_dynamic_get_rigid_dynamic_lock_flags :: proc(self_: ^RigidDynamic) -> RigidDynamicLockFlags_Set ---

    /// Raises or clears a particular rigid dynamic lock flag.
    ///
    /// See the list of flags [`PxRigidDynamicLockFlag`]
    ///
    /// Default:
    /// no flags are set
    @(link_name = "PxRigidDynamic_setRigidDynamicLockFlag_mut")
    rigid_dynamic_set_rigid_dynamic_lock_flag_mut :: proc(self_: ^RigidDynamic, flag: RigidDynamicLockFlag, value: _c.bool) ---

    @(link_name = "PxRigidDynamic_setRigidDynamicLockFlags_mut")
    rigid_dynamic_set_rigid_dynamic_lock_flags_mut :: proc(self_: ^RigidDynamic, flags: RigidDynamicLockFlags_Set) ---

    /// Retrieves the linear velocity of an actor.
    ///
    /// It is not allowed to use this method while the simulation is running (except during PxScene::collide(),
    /// in PxContactModifyCallback or in contact report callbacks).
    ///
    /// The linear velocity of the actor.
    @(link_name = "PxRigidDynamic_getLinearVelocity")
    rigid_dynamic_get_linear_velocity :: proc(self_: ^RigidDynamic) -> Vec3 ---

    /// Sets the linear velocity of the actor.
    ///
    /// Note that if you continuously set the velocity of an actor yourself,
    /// forces such as gravity or friction will not be able to manifest themselves, because forces directly
    /// influence only the velocity/momentum of an actor.
    ///
    /// Default:
    /// (0.0, 0.0, 0.0)
    ///
    /// Sleeping:
    /// This call wakes the actor if it is sleeping, and the autowake parameter is true (default) or the
    /// new velocity is non-zero.
    ///
    /// It is invalid to use this method if PxActorFlag::eDISABLE_SIMULATION is set.
    @(link_name = "PxRigidDynamic_setLinearVelocity_mut")
    rigid_dynamic_set_linear_velocity_mut :: proc(self_: ^RigidDynamic, #by_ptr linVel: Vec3, autowake: _c.bool) ---

    /// Retrieves the angular velocity of the actor.
    ///
    /// It is not allowed to use this method while the simulation is running (except during PxScene::collide(),
    /// in PxContactModifyCallback or in contact report callbacks).
    ///
    /// The angular velocity of the actor.
    @(link_name = "PxRigidDynamic_getAngularVelocity")
    rigid_dynamic_get_angular_velocity :: proc(self_: ^RigidDynamic) -> Vec3 ---

    /// Sets the angular velocity of the actor.
    ///
    /// Note that if you continuously set the angular velocity of an actor yourself,
    /// forces such as friction will not be able to rotate the actor, because forces directly influence only the velocity/momentum.
    ///
    /// Default:
    /// (0.0, 0.0, 0.0)
    ///
    /// Sleeping:
    /// This call wakes the actor if it is sleeping, and the autowake parameter is true (default) or the
    /// new velocity is non-zero.
    ///
    /// It is invalid to use this method if PxActorFlag::eDISABLE_SIMULATION is set.
    @(link_name = "PxRigidDynamic_setAngularVelocity_mut")
    rigid_dynamic_set_angular_velocity_mut :: proc(self_: ^RigidDynamic, #by_ptr angVel: Vec3, autowake: _c.bool) ---

    /// Sets the wake counter for the actor.
    ///
    /// The wake counter value determines the minimum amount of time until the body can be put to sleep. Please note
    /// that a body will not be put to sleep if the energy is above the specified threshold (see [`setSleepThreshold`]())
    /// or if other awake bodies are touching it.
    ///
    /// Passing in a positive value will wake the actor up automatically.
    ///
    /// It is invalid to use this method for kinematic actors since the wake counter for kinematics is defined
    /// based on whether a target pose has been set (see the comment in [`isSleeping`]()).
    ///
    /// It is invalid to use this method if PxActorFlag::eDISABLE_SIMULATION is set.
    ///
    /// Default:
    /// 0.4 (which corresponds to 20 frames for a time step of 0.02)
    @(link_name = "PxRigidDynamic_setWakeCounter_mut")
    rigid_dynamic_set_wake_counter_mut :: proc(self_: ^RigidDynamic, wakeCounterValue: _c.float) ---

    /// Returns the wake counter of the actor.
    ///
    /// It is not allowed to use this method while the simulation is running.
    ///
    /// The wake counter of the actor.
    @(link_name = "PxRigidDynamic_getWakeCounter")
    rigid_dynamic_get_wake_counter :: proc(self_: ^RigidDynamic) -> _c.float ---

    /// Wakes up the actor if it is sleeping.
    ///
    /// The actor will get woken up and might cause other touching actors to wake up as well during the next simulation step.
    ///
    /// This will set the wake counter of the actor to the value specified in [`PxSceneDesc::wakeCounterResetValue`].
    ///
    /// It is invalid to use this method if the actor has not been added to a scene already or if PxActorFlag::eDISABLE_SIMULATION is set.
    ///
    /// It is invalid to use this method for kinematic actors since the sleep state for kinematics is defined
    /// based on whether a target pose has been set (see the comment in [`isSleeping`]()).
    @(link_name = "PxRigidDynamic_wakeUp_mut")
    rigid_dynamic_wake_up_mut :: proc(self_: ^RigidDynamic) ---

    /// Forces the actor to sleep.
    ///
    /// The actor will stay asleep during the next simulation step if not touched by another non-sleeping actor.
    ///
    /// Any applied force will be cleared and the velocity and the wake counter of the actor will be set to 0.
    ///
    /// It is invalid to use this method if the actor has not been added to a scene already or if PxActorFlag::eDISABLE_SIMULATION is set.
    ///
    /// It is invalid to use this method for kinematic actors since the sleep state for kinematics is defined
    /// based on whether a target pose has been set (see the comment in [`isSleeping`]()).
    @(link_name = "PxRigidDynamic_putToSleep_mut")
    rigid_dynamic_put_to_sleep_mut :: proc(self_: ^RigidDynamic) ---

    /// Sets the solver iteration counts for the body.
    ///
    /// The solver iteration count determines how accurately joints and contacts are resolved.
    /// If you are having trouble with jointed bodies oscillating and behaving erratically, then
    /// setting a higher position iteration count may improve their stability.
    ///
    /// If intersecting bodies are being depenetrated too violently, increase the number of velocity
    /// iterations. More velocity iterations will drive the relative exit velocity of the intersecting
    /// objects closer to the correct value given the restitution.
    ///
    /// Default:
    /// 4 position iterations, 1 velocity iteration
    @(link_name = "PxRigidDynamic_setSolverIterationCounts_mut")
    rigid_dynamic_set_solver_iteration_counts_mut :: proc(self_: ^RigidDynamic, minPositionIters: _c.uint32_t, minVelocityIters: _c.uint32_t) ---

    /// Retrieves the solver iteration counts.
    @(link_name = "PxRigidDynamic_getSolverIterationCounts")
    rigid_dynamic_get_solver_iteration_counts :: proc(self_: ^RigidDynamic, minPositionIters: ^_c.uint32_t, minVelocityIters: ^_c.uint32_t) ---

    /// Retrieves the force threshold for contact reports.
    ///
    /// The contact report threshold is a force threshold. If the force between
    /// two actors exceeds this threshold for either of the two actors, a contact report
    /// will be generated according to the contact report threshold flags provided by
    /// the filter shader/callback.
    /// See [`PxPairFlag`].
    ///
    /// The threshold used for a collision between a dynamic actor and the static environment is
    /// the threshold of the dynamic actor, and all contacts with static actors are summed to find
    /// the total normal force.
    ///
    /// Default:
    /// PX_MAX_F32
    ///
    /// Force threshold for contact reports.
    @(link_name = "PxRigidDynamic_getContactReportThreshold")
    rigid_dynamic_get_contact_report_threshold :: proc(self_: ^RigidDynamic) -> _c.float ---

    /// Sets the force threshold for contact reports.
    ///
    /// See [`getContactReportThreshold`]().
    @(link_name = "PxRigidDynamic_setContactReportThreshold_mut")
    rigid_dynamic_set_contact_report_threshold_mut :: proc(self_: ^RigidDynamic, threshold: _c.float) ---

    @(link_name = "PxRigidDynamic_getConcreteTypeName")
    rigid_dynamic_get_concrete_type_name :: proc(self_: ^RigidDynamic) -> cstring ---

    @(link_name = "PxRigidStatic_getConcreteTypeName")
    rigid_static_get_concrete_type_name :: proc(self_: ^RigidStatic) -> cstring ---

    /// constructor sets to default.
    @(link_name = "PxSceneQueryDesc_new")
    scene_query_desc_new :: proc() -> SceneQueryDesc ---

    /// (re)sets the structure to the default.
    @(link_name = "PxSceneQueryDesc_setToDefault_mut")
    scene_query_desc_set_to_default_mut :: proc(self_: ^SceneQueryDesc) ---

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid.
    @(link_name = "PxSceneQueryDesc_isValid")
    scene_query_desc_is_valid :: proc(self_: ^SceneQueryDesc) -> _c.bool ---

    /// Sets the rebuild rate of the dynamic tree pruning structures.
    @(link_name = "PxSceneQuerySystemBase_setDynamicTreeRebuildRateHint_mut")
    scene_query_system_base_set_dynamic_tree_rebuild_rate_hint_mut :: proc(self_: ^SceneQuerySystemBase, dynamicTreeRebuildRateHint: _c.uint32_t) ---

    /// Retrieves the rebuild rate of the dynamic tree pruning structures.
    ///
    /// The rebuild rate of the dynamic tree pruning structures.
    @(link_name = "PxSceneQuerySystemBase_getDynamicTreeRebuildRateHint")
    scene_query_system_base_get_dynamic_tree_rebuild_rate_hint :: proc(self_: ^SceneQuerySystemBase) -> _c.uint32_t ---

    /// Forces dynamic trees to be immediately rebuilt.
    ///
    /// PxScene will call this function with the PX_SCENE_PRUNER_STATIC or PX_SCENE_PRUNER_DYNAMIC value.
    @(link_name = "PxSceneQuerySystemBase_forceRebuildDynamicTree_mut")
    scene_query_system_base_force_rebuild_dynamic_tree_mut :: proc(self_: ^SceneQuerySystemBase, prunerIndex: _c.uint32_t) ---

    /// Sets scene query update mode
    @(link_name = "PxSceneQuerySystemBase_setUpdateMode_mut")
    scene_query_system_base_set_update_mode_mut :: proc(self_: ^SceneQuerySystemBase, updateMode: SceneQueryUpdateMode) ---

    /// Gets scene query update mode
    ///
    /// Current scene query update mode.
    @(link_name = "PxSceneQuerySystemBase_getUpdateMode")
    scene_query_system_base_get_update_mode :: proc(self_: ^SceneQuerySystemBase) -> SceneQueryUpdateMode ---

    /// Retrieves the system's internal scene query timestamp, increased each time a change to the
    /// static scene query structure is performed.
    ///
    /// scene query static timestamp
    @(link_name = "PxSceneQuerySystemBase_getStaticTimestamp")
    scene_query_system_base_get_static_timestamp :: proc(self_: ^SceneQuerySystemBase) -> _c.uint32_t ---

    /// Flushes any changes to the scene query representation.
    ///
    /// This method updates the state of the scene query representation to match changes in the scene state.
    ///
    /// By default, these changes are buffered until the next query is submitted. Calling this function will not change
    /// the results from scene queries, but can be used to ensure that a query will not perform update work in the course of
    /// its execution.
    ///
    /// A thread performing updates will hold a write lock on the query structure, and thus stall other querying threads. In multithread
    /// scenarios it can be useful to explicitly schedule the period where this lock may be held for a significant period, so that
    /// subsequent queries issued from multiple threads will not block.
    @(link_name = "PxSceneQuerySystemBase_flushUpdates_mut")
    scene_query_system_base_flush_updates_mut :: proc(self_: ^SceneQuerySystemBase) ---

    /// Performs a raycast against objects in the scene, returns results in a PxRaycastBuffer object
    /// or via a custom user callback implementation inheriting from PxRaycastCallback.
    ///
    /// Touching hits are not ordered.
    ///
    /// Shooting a ray from within an object leads to different results depending on the shape type. Please check the details in user guide article SceneQuery. User can ignore such objects by employing one of the provided filter mechanisms.
    ///
    /// True if any touching or blocking hits were found or any hit was found in case PxQueryFlag::eANY_HIT was specified.
    @(link_name = "PxSceneQuerySystemBase_raycast")
    scene_query_system_base_raycast :: proc(self_: ^SceneQuerySystemBase, #by_ptr origin: Vec3, #by_ptr unitDir: Vec3, distance: _c.float, hitCall: ^RaycastCallback, hitFlags: HitFlags_Set, #by_ptr filterData: QueryFilterData, filterCall: ^QueryFilterCallback, cache: ^QueryCache, queryFlags: GeometryQueryFlags_Set) -> _c.bool ---

    /// Performs a sweep test against objects in the scene, returns results in a PxSweepBuffer object
    /// or via a custom user callback implementation inheriting from PxSweepCallback.
    ///
    /// Touching hits are not ordered.
    ///
    /// If a shape from the scene is already overlapping with the query shape in its starting position,
    /// the hit is returned unless eASSUME_NO_INITIAL_OVERLAP was specified.
    ///
    /// True if any touching or blocking hits were found or any hit was found in case PxQueryFlag::eANY_HIT was specified.
    @(link_name = "PxSceneQuerySystemBase_sweep")
    scene_query_system_base_sweep :: proc(self_: ^SceneQuerySystemBase, geometry: ^Geometry, #by_ptr pose: Transform, #by_ptr unitDir: Vec3, distance: _c.float, hitCall: ^SweepCallback, hitFlags: HitFlags_Set, #by_ptr filterData: QueryFilterData, filterCall: ^QueryFilterCallback, cache: ^QueryCache, inflation: _c.float, queryFlags: GeometryQueryFlags_Set) -> _c.bool ---

    /// Performs an overlap test of a given geometry against objects in the scene, returns results in a PxOverlapBuffer object
    /// or via a custom user callback implementation inheriting from PxOverlapCallback.
    ///
    /// Filtering: returning eBLOCK from user filter for overlap queries will cause a warning (see [`PxQueryHitType`]).
    ///
    /// True if any touching or blocking hits were found or any hit was found in case PxQueryFlag::eANY_HIT was specified.
    ///
    /// eBLOCK should not be returned from user filters for overlap(). Doing so will result in undefined behavior, and a warning will be issued.
    ///
    /// If the PxQueryFlag::eNO_BLOCK flag is set, the eBLOCK will instead be automatically converted to an eTOUCH and the warning suppressed.
    @(link_name = "PxSceneQuerySystemBase_overlap")
    scene_query_system_base_overlap :: proc(self_: ^SceneQuerySystemBase, geometry: ^Geometry, #by_ptr pose: Transform, hitCall: ^OverlapCallback, #by_ptr filterData: QueryFilterData, filterCall: ^QueryFilterCallback, cache: ^QueryCache, queryFlags: GeometryQueryFlags_Set) -> _c.bool ---

    /// Sets scene query update mode
    @(link_name = "PxSceneSQSystem_setSceneQueryUpdateMode_mut")
    scene_s_q_system_set_scene_query_update_mode_mut :: proc(self_: ^SceneSQSystem, updateMode: SceneQueryUpdateMode) ---

    /// Gets scene query update mode
    ///
    /// Current scene query update mode.
    @(link_name = "PxSceneSQSystem_getSceneQueryUpdateMode")
    scene_s_q_system_get_scene_query_update_mode :: proc(self_: ^SceneSQSystem) -> SceneQueryUpdateMode ---

    /// Retrieves the scene's internal scene query timestamp, increased each time a change to the
    /// static scene query structure is performed.
    ///
    /// scene query static timestamp
    @(link_name = "PxSceneSQSystem_getSceneQueryStaticTimestamp")
    scene_s_q_system_get_scene_query_static_timestamp :: proc(self_: ^SceneSQSystem) -> _c.uint32_t ---

    /// Flushes any changes to the scene query representation.
    @(link_name = "PxSceneSQSystem_flushQueryUpdates_mut")
    scene_s_q_system_flush_query_updates_mut :: proc(self_: ^SceneSQSystem) ---

    /// Forces dynamic trees to be immediately rebuilt.
    @(link_name = "PxSceneSQSystem_forceDynamicTreeRebuild_mut")
    scene_s_q_system_force_dynamic_tree_rebuild_mut :: proc(self_: ^SceneSQSystem, rebuildStaticStructure: _c.bool, rebuildDynamicStructure: _c.bool) ---

    /// Return the value of PxSceneQueryDesc::staticStructure that was set when creating the scene with PxPhysics::createScene
    @(link_name = "PxSceneSQSystem_getStaticStructure")
    scene_s_q_system_get_static_structure :: proc(self_: ^SceneSQSystem) -> PruningStructureType ---

    /// Return the value of PxSceneQueryDesc::dynamicStructure that was set when creating the scene with PxPhysics::createScene
    @(link_name = "PxSceneSQSystem_getDynamicStructure")
    scene_s_q_system_get_dynamic_structure :: proc(self_: ^SceneSQSystem) -> PruningStructureType ---

    /// Executes scene queries update tasks.
    ///
    /// This function will refit dirty shapes within the pruner and will execute a task to build a new AABB tree, which is
    /// build on a different thread. The new AABB tree is built based on the dynamic tree rebuild hint rate. Once
    /// the new tree is ready it will be commited in next fetchQueries call, which must be called after.
    ///
    /// This function is equivalent to the following PxSceneQuerySystem calls:
    /// Synchronous calls:
    /// - PxSceneQuerySystemBase::flushUpdates()
    /// - handle0 = PxSceneQuerySystem::prepareSceneQueryBuildStep(PX_SCENE_PRUNER_STATIC)
    /// - handle1 = PxSceneQuerySystem::prepareSceneQueryBuildStep(PX_SCENE_PRUNER_DYNAMIC)
    /// Asynchronous calls:
    /// - PxSceneQuerySystem::sceneQueryBuildStep(handle0);
    /// - PxSceneQuerySystem::sceneQueryBuildStep(handle1);
    ///
    /// This function is part of the PxSceneSQSystem interface because it uses the PxScene task system under the hood. But
    /// it calls PxSceneQuerySystem functions, which are independent from this system and could be called in a similar
    /// fashion by a separate, possibly user-defined task manager.
    ///
    /// If PxSceneQueryUpdateMode::eBUILD_DISABLED_COMMIT_DISABLED is used, it is required to update the scene queries
    /// using this function.
    @(link_name = "PxSceneSQSystem_sceneQueriesUpdate_mut")
    scene_s_q_system_scene_queries_update_mut :: proc(self_: ^SceneSQSystem, completionTask: ^BaseTask, controlSimulation: _c.bool) ---

    /// This checks to see if the scene queries update has completed.
    ///
    /// This does not cause the data available for reading to be updated with the results of the scene queries update, it is simply a status check.
    /// The bool will allow it to either return immediately or block waiting for the condition to be met so that it can return true
    ///
    /// True if the results are available.
    @(link_name = "PxSceneSQSystem_checkQueries_mut")
    scene_s_q_system_check_queries_mut :: proc(self_: ^SceneSQSystem, block: _c.bool) -> _c.bool ---

    /// This method must be called after sceneQueriesUpdate. It will wait for the scene queries update to finish. If the user makes an illegal scene queries update call,
    /// the SDK will issue an error message.
    ///
    /// If a new AABB tree build finished, then during fetchQueries the current tree within the pruning structure is swapped with the new tree.
    @(link_name = "PxSceneSQSystem_fetchQueries_mut")
    scene_s_q_system_fetch_queries_mut :: proc(self_: ^SceneSQSystem, block: _c.bool) -> _c.bool ---

    /// Decrements the reference count of the object and releases it if the new reference count is zero.
    @(link_name = "PxSceneQuerySystem_release_mut")
    scene_query_system_release_mut :: proc(self_: ^SceneQuerySystem) ---

    /// Acquires a counted reference to this object.
    ///
    /// This method increases the reference count of the object by 1. Decrement the reference count by calling release()
    @(link_name = "PxSceneQuerySystem_acquireReference_mut")
    scene_query_system_acquire_reference_mut :: proc(self_: ^SceneQuerySystem) ---

    /// Preallocates internal arrays to minimize the amount of reallocations.
    ///
    /// The system does not prevent more allocations than given numbers. It is legal to not call this function at all,
    /// or to add more shapes to the system than the preallocated amounts.
    @(link_name = "PxSceneQuerySystem_preallocate_mut")
    scene_query_system_preallocate_mut :: proc(self_: ^SceneQuerySystem, prunerIndex: _c.uint32_t, nbShapes: _c.uint32_t) ---

    /// Frees internal memory that may not be in-use anymore.
    ///
    /// This is an entry point for reclaiming transient memory allocated at some point by the SQ system,
    /// but which wasn't been immediately freed for performance reason. Calling this function might free
    /// some memory, but it might also produce a new set of allocations in the next frame.
    @(link_name = "PxSceneQuerySystem_flushMemory_mut")
    scene_query_system_flush_memory_mut :: proc(self_: ^SceneQuerySystem) ---

    /// Adds a shape to the SQ system.
    ///
    /// The same function is used to add either a regular shape, or a SQ compound shape.
    @(link_name = "PxSceneQuerySystem_addSQShape_mut")
    scene_query_system_add_s_q_shape_mut :: proc(self_: ^SceneQuerySystem, actor: ^RigidActor, #by_ptr shape: Shape, #by_ptr bounds: Bounds3, #by_ptr transform: Transform, compoundHandle: ^_c.uint32_t, hasPruningStructure: _c.bool) ---

    /// Removes a shape from the SQ system.
    ///
    /// The same function is used to remove either a regular shape, or a SQ compound shape.
    @(link_name = "PxSceneQuerySystem_removeSQShape_mut")
    scene_query_system_remove_s_q_shape_mut :: proc(self_: ^SceneQuerySystem, actor: ^RigidActor, #by_ptr shape: Shape) ---

    /// Updates a shape in the SQ system.
    ///
    /// The same function is used to update either a regular shape, or a SQ compound shape.
    ///
    /// The transforms are eager-evaluated, but the bounds are lazy-evaluated. This means that
    /// the updated transform has to be passed to the update function, while the bounds are automatically
    /// recomputed by the system whenever needed.
    @(link_name = "PxSceneQuerySystem_updateSQShape_mut")
    scene_query_system_update_s_q_shape_mut :: proc(self_: ^SceneQuerySystem, actor: ^RigidActor, #by_ptr shape: Shape, #by_ptr transform: Transform) ---

    /// Adds a compound to the SQ system.
    ///
    /// SQ compound handle
    @(link_name = "PxSceneQuerySystem_addSQCompound_mut")
    scene_query_system_add_s_q_compound_mut :: proc(self_: ^SceneQuerySystem, actor: ^RigidActor, shapes: [^]^Shape, #by_ptr bvh: BVH, transforms: ^Transform) -> _c.uint32_t ---

    /// Removes a compound from the SQ system.
    @(link_name = "PxSceneQuerySystem_removeSQCompound_mut")
    scene_query_system_remove_s_q_compound_mut :: proc(self_: ^SceneQuerySystem, compoundHandle: _c.uint32_t) ---

    /// Updates a compound in the SQ system.
    ///
    /// The compound structures are immediately updated when the call occurs.
    @(link_name = "PxSceneQuerySystem_updateSQCompound_mut")
    scene_query_system_update_s_q_compound_mut :: proc(self_: ^SceneQuerySystem, compoundHandle: _c.uint32_t, #by_ptr compoundTransform: Transform) ---

    /// Shift the data structures' origin by the specified vector.
    ///
    /// Please refer to the notes of the similar function in PxScene.
    @(link_name = "PxSceneQuerySystem_shiftOrigin_mut")
    scene_query_system_shift_origin_mut :: proc(self_: ^SceneQuerySystem, #by_ptr shift: Vec3) ---

    /// Merges a pruning structure with the SQ system's internal pruners.
    @(link_name = "PxSceneQuerySystem_merge_mut")
    scene_query_system_merge_mut :: proc(self_: ^SceneQuerySystem, #by_ptr pruningStructure: PruningStructure) ---

    /// Shape to SQ-pruner-handle mapping function.
    ///
    /// This function finds and returns the SQ pruner handle associated with a given (actor/shape) couple
    /// that was previously added to the system. This is needed for the sync function.
    ///
    /// Associated SQ pruner handle.
    @(link_name = "PxSceneQuerySystem_getHandle")
    scene_query_system_get_handle :: proc(self_: ^SceneQuerySystem, actor: ^RigidActor, #by_ptr shape: Shape, prunerIndex: ^_c.uint32_t) -> _c.uint32_t ---

    /// Synchronizes the scene-query system with another system that references the same objects.
    ///
    /// This function is used when the scene-query objects also exist in another system that can also update them. For example the scene-query objects
    /// (used for raycast, overlap or sweep queries) might be driven by equivalent objects in an external rigid-body simulation engine. In this case
    /// the rigid-body simulation engine computes the new poses and transforms, and passes them to the scene-query system using this function. It is
    /// more efficient than calling updateSQShape on each object individually, since updateSQShape would end up recomputing the bounds already available
    /// in the rigid-body engine.
    @(link_name = "PxSceneQuerySystem_sync_mut")
    scene_query_system_sync_mut :: proc(self_: ^SceneQuerySystem, prunerIndex: _c.uint32_t, handles: ^_c.uint32_t, indices: ^_c.uint32_t, bounds: ^Bounds3, transforms: ^TransformPadded, count: _c.uint32_t, #by_ptr ignoredIndices: BitMap) ---

    /// Finalizes updates made to the SQ system.
    ///
    /// This function should be called after updates have been made to the SQ system, to fully reflect the changes
    /// inside the internal pruners. In particular it should be called:
    /// - after calls to updateSQShape
    /// - after calls to sync
    ///
    /// This function:
    /// - recomputes bounds of manually updated shapes (i.e. either regular or SQ compound shapes modified by updateSQShape)
    /// - updates dynamic pruners (refit operations)
    /// - incrementally rebuilds AABB-trees
    ///
    /// The amount of work performed in this function depends on PxSceneQueryUpdateMode.
    @(link_name = "PxSceneQuerySystem_finalizeUpdates_mut")
    scene_query_system_finalize_updates_mut :: proc(self_: ^SceneQuerySystem) ---

    /// Prepares asynchronous build step.
    ///
    /// This is directly called (synchronously) by PxSceneSQSystem::sceneQueriesUpdate(). See the comments there.
    ///
    /// This function is called to let the system execute any necessary synchronous operation before the
    /// asynchronous sceneQueryBuildStep() function is called.
    ///
    /// If there is any work to do for the specific pruner, the function returns a pruner-specific handle that
    /// will be passed to the corresponding, asynchronous sceneQueryBuildStep function.
    ///
    /// A pruner-specific handle that will be sent to sceneQueryBuildStep if there is any work to do, i.e. to execute the corresponding sceneQueryBuildStep() call.
    ///
    /// Null if there is no work to do, otherwise a pruner-specific handle.
    @(link_name = "PxSceneQuerySystem_prepareSceneQueryBuildStep_mut")
    scene_query_system_prepare_scene_query_build_step_mut :: proc(self_: ^SceneQuerySystem, prunerIndex: _c.uint32_t) -> rawptr ---

    /// Executes asynchronous build step.
    ///
    /// This is directly called (asynchronously) by PxSceneSQSystem::sceneQueriesUpdate(). See the comments there.
    ///
    /// This function incrementally builds the internal trees/pruners. It is called asynchronously, i.e. this can be
    /// called from different threads for building multiple trees at the same time.
    @(link_name = "PxSceneQuerySystem_sceneQueryBuildStep_mut")
    scene_query_system_scene_query_build_step_mut :: proc(self_: ^SceneQuerySystem, handle: rawptr) ---

    @(link_name = "PxBroadPhaseDesc_new")
    broad_phase_desc_new :: proc(type: BroadPhaseType) -> BroadPhaseDesc ---

    @(link_name = "PxBroadPhaseDesc_isValid")
    broad_phase_desc_is_valid :: proc(self_: ^BroadPhaseDesc) -> _c.bool ---

    /// Retrieves the filter group for static objects.
    ///
    /// Mark static objects with this group when adding them to the broadphase.
    /// Overlaps between static objects will not be detected. All static objects
    /// should have the same group.
    ///
    /// Filter group for static objects.
    @(link_name = "phys_PxGetBroadPhaseStaticFilterGroup")
    get_broad_phase_static_filter_group :: proc() -> _c.uint32_t ---

    /// Retrieves a filter group for dynamic objects.
    ///
    /// Mark dynamic objects with this group when adding them to the broadphase.
    /// Each dynamic object must have an ID, and overlaps between dynamic objects that have
    /// the same ID will not be detected. This is useful to dismiss overlaps between shapes
    /// of the same (compound) actor directly within the broadphase.
    ///
    /// Filter group for the object.
    @(link_name = "phys_PxGetBroadPhaseDynamicFilterGroup")
    get_broad_phase_dynamic_filter_group :: proc(id: _c.uint32_t) -> _c.uint32_t ---

    /// Retrieves a filter group for kinematic objects.
    ///
    /// Mark kinematic objects with this group when adding them to the broadphase.
    /// Each kinematic object must have an ID, and overlaps between kinematic objects that have
    /// the same ID will not be detected.
    ///
    /// Filter group for the object.
    @(link_name = "phys_PxGetBroadPhaseKinematicFilterGroup")
    get_broad_phase_kinematic_filter_group :: proc(id: _c.uint32_t) -> _c.uint32_t ---

    @(link_name = "PxBroadPhaseUpdateData_new")
    broad_phase_update_data_new :: proc(created: ^_c.uint32_t, nbCreated: _c.uint32_t, updated: ^_c.uint32_t, nbUpdated: _c.uint32_t, removed: ^_c.uint32_t, nbRemoved: _c.uint32_t, bounds: ^Bounds3, groups: ^_c.uint32_t, distances: ^_c.float, capacity: _c.uint32_t) -> BroadPhaseUpdateData ---

    @(link_name = "PxBroadPhaseResults_new")
    broad_phase_results_new :: proc() -> BroadPhaseResults ---

    /// Returns number of regions currently registered in the broad-phase.
    ///
    /// Number of regions
    @(link_name = "PxBroadPhaseRegions_getNbRegions")
    broad_phase_regions_get_nb_regions :: proc(self_: ^BroadPhaseRegions) -> _c.uint32_t ---

    /// Gets broad-phase regions.
    ///
    /// Number of written out regions.
    @(link_name = "PxBroadPhaseRegions_getRegions")
    broad_phase_regions_get_regions :: proc(self_: ^BroadPhaseRegions, userBuffer: ^BroadPhaseRegionInfo, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Adds a new broad-phase region.
    ///
    /// The total number of regions is limited to PxBroadPhaseCaps::mMaxNbRegions. If that number is exceeded, the call is ignored.
    ///
    /// The newly added region will be automatically populated with already existing objects that touch it, if the
    /// 'populateRegion' parameter is set to true. Otherwise the newly added region will be empty, and it will only be
    /// populated with objects when those objects are added to the simulation, or updated if they already exist.
    ///
    /// Using 'populateRegion=true' has a cost, so it is best to avoid it if possible. In particular it is more efficient
    /// to create the empty regions first (with populateRegion=false) and then add the objects afterwards (rather than
    /// the opposite).
    ///
    /// Objects automatically move from one region to another during their lifetime. The system keeps tracks of what
    /// regions a given object is in. It is legal for an object to be in an arbitrary number of regions. However if an
    /// object leaves all regions, or is created outside of all regions, several things happen:
    /// - collisions get disabled for this object
    /// - the object appears in the getOutOfBoundsObjects() array
    ///
    /// If an out-of-bounds object, whose collisions are disabled, re-enters a valid broadphase region, then collisions
    /// are re-enabled for that object.
    ///
    /// Handle for newly created region, or 0xffffffff in case of failure.
    @(link_name = "PxBroadPhaseRegions_addRegion_mut")
    broad_phase_regions_add_region_mut :: proc(self_: ^BroadPhaseRegions, #by_ptr region: BroadPhaseRegion, populateRegion: _c.bool, bounds: ^Bounds3, distances: ^_c.float) -> _c.uint32_t ---

    /// Removes a broad-phase region.
    ///
    /// If the region still contains objects, and if those objects do not overlap any region any more, they are not
    /// automatically removed from the simulation. Instead, the PxBroadPhaseCallback::onObjectOutOfBounds notification
    /// is used for each object. Users are responsible for removing the objects from the simulation if this is the
    /// desired behavior.
    ///
    /// If the handle is invalid, or if a valid handle is removed twice, an error message is sent to the error stream.
    ///
    /// True if success
    @(link_name = "PxBroadPhaseRegions_removeRegion_mut")
    broad_phase_regions_remove_region_mut :: proc(self_: ^BroadPhaseRegions, handle: _c.uint32_t) -> _c.bool ---

    @(link_name = "PxBroadPhaseRegions_getNbOutOfBoundsObjects")
    broad_phase_regions_get_nb_out_of_bounds_objects :: proc(self_: ^BroadPhaseRegions) -> _c.uint32_t ---

    @(link_name = "PxBroadPhaseRegions_getOutOfBoundsObjects")
    broad_phase_regions_get_out_of_bounds_objects :: proc(self_: ^BroadPhaseRegions) -> [^]_c.uint32_t ---

    @(link_name = "PxBroadPhase_release_mut")
    broad_phase_release_mut :: proc(self_: ^BroadPhase) ---

    /// Gets the broadphase type.
    ///
    /// Broadphase type.
    @(link_name = "PxBroadPhase_getType")
    broad_phase_get_type :: proc(self_: ^BroadPhase) -> BroadPhaseType ---

    /// Gets broad-phase caps.
    @(link_name = "PxBroadPhase_getCaps")
    broad_phase_get_caps :: proc(self_: ^BroadPhase, caps: ^BroadPhaseCaps) ---

    /// Retrieves the regions API if applicable.
    ///
    /// For broadphases that do not use explicit user-defined regions, this call returns NULL.
    ///
    /// Region API, or NULL.
    @(link_name = "PxBroadPhase_getRegions_mut")
    broad_phase_get_regions_mut :: proc(self_: ^BroadPhase) -> ^BroadPhaseRegions ---

    /// Retrieves the broadphase allocator.
    ///
    /// User-provided buffers should ideally be allocated with this allocator, for best performance.
    /// This is especially true for the GPU broadphases, whose buffers need to be allocated in CUDA
    /// host memory.
    ///
    /// The broadphase allocator.
    @(link_name = "PxBroadPhase_getAllocator_mut")
    broad_phase_get_allocator_mut :: proc(self_: ^BroadPhase) -> ^AllocatorCallback ---

    /// Retrieves the profiler's context ID.
    ///
    /// The context ID.
    @(link_name = "PxBroadPhase_getContextID")
    broad_phase_get_context_i_d :: proc(self_: ^BroadPhase) -> _c.uint64_t ---

    /// Sets a scratch buffer
    ///
    /// Some broadphases might take advantage of a scratch buffer to limit runtime allocations.
    ///
    /// All broadphases still work without providing a scratch buffer, this is an optional function
    /// that can potentially reduce runtime allocations.
    @(link_name = "PxBroadPhase_setScratchBlock_mut")
    broad_phase_set_scratch_block_mut :: proc(self_: ^BroadPhase, scratchBlock: rawptr, size: _c.uint32_t) ---

    /// Updates the broadphase and computes the lists of created/deleted pairs.
    ///
    /// The provided update data describes changes to objects since the last broadphase update.
    ///
    /// To benefit from potentially multithreaded implementations, it is necessary to provide a continuation
    /// task to the function. It is legal to pass NULL there, but the underlying (CPU) implementations will
    /// then run single-threaded.
    @(link_name = "PxBroadPhase_update_mut")
    broad_phase_update_mut :: proc(self_: ^BroadPhase, #by_ptr updateData: BroadPhaseUpdateData, continuation: ^BaseTask) ---

    /// Retrieves the broadphase results after an update.
    ///
    /// This should be called once after each update call to retrieve the results of the broadphase. The
    /// results are incremental, i.e. the system only returns new and lost pairs, not all current pairs.
    @(link_name = "PxBroadPhase_fetchResults_mut")
    broad_phase_fetch_results_mut :: proc(self_: ^BroadPhase, results: ^BroadPhaseResults) ---

    /// Helper for single-threaded updates.
    ///
    /// This short helper function performs a single-theaded update and reports the results in a single call.
    @(link_name = "PxBroadPhase_update_mut_1")
    broad_phase_update_mut_1 :: proc(self_: ^BroadPhase, results: ^BroadPhaseResults, #by_ptr updateData: BroadPhaseUpdateData) ---

    /// Broadphase factory function.
    ///
    /// Use this function to create a new standalone broadphase.
    ///
    /// Newly created broadphase, or NULL
    @(link_name = "phys_PxCreateBroadPhase")
    create_broad_phase :: proc(#by_ptr desc: BroadPhaseDesc) -> ^BroadPhase ---

    @(link_name = "PxAABBManager_release_mut")
    a_a_b_b_manager_release_mut :: proc(self_: ^AABBManager) ---

    /// Retrieves the underlying broadphase.
    ///
    /// The managed broadphase.
    @(link_name = "PxAABBManager_getBroadPhase_mut")
    a_a_b_b_manager_get_broad_phase_mut :: proc(self_: ^AABBManager) -> ^BroadPhase ---

    /// Retrieves the managed bounds.
    ///
    /// This is needed as input parameters to functions like PxBroadPhaseRegions::addRegion.
    ///
    /// The managed object bounds.
    @(link_name = "PxAABBManager_getBounds")
    a_a_b_b_manager_get_bounds :: proc(self_: ^AABBManager) -> [^]Bounds3 ---

    /// Retrieves the managed distances.
    ///
    /// This is needed as input parameters to functions like PxBroadPhaseRegions::addRegion.
    ///
    /// The managed object distances.
    @(link_name = "PxAABBManager_getDistances")
    a_a_b_b_manager_get_distances :: proc(self_: ^AABBManager) -> [^]_c.float ---

    /// Retrieves the managed filter groups.
    ///
    /// The managed object groups.
    @(link_name = "PxAABBManager_getGroups")
    a_a_b_b_manager_get_groups :: proc(self_: ^AABBManager) -> [^]_c.uint32_t ---

    /// Retrieves the managed buffers' capacity.
    ///
    /// Bounds, distances and groups buffers have the same capacity.
    ///
    /// The managed buffers' capacity.
    @(link_name = "PxAABBManager_getCapacity")
    a_a_b_b_manager_get_capacity :: proc(self_: ^AABBManager) -> _c.uint32_t ---

    /// Adds an object to the manager.
    ///
    /// Objects' indices are externally managed, i.e. they must be provided by users (as opposed to handles
    /// that could be returned by this manager). The design allows users to identify an object by a single ID,
    /// and use the same ID in multiple sub-systems.
    @(link_name = "PxAABBManager_addObject_mut")
    a_a_b_b_manager_add_object_mut :: proc(self_: ^AABBManager, index: _c.uint32_t, #by_ptr bounds: Bounds3, group: _c.uint32_t, distance: _c.float) ---

    /// Removes an object from the manager.
    @(link_name = "PxAABBManager_removeObject_mut")
    a_a_b_b_manager_remove_object_mut :: proc(self_: ^AABBManager, index: _c.uint32_t) ---

    /// Updates an object in the manager.
    ///
    /// This call can update an object's bounds, distance, or both.
    /// It is not possible to update an object's filter group.
    @(link_name = "PxAABBManager_updateObject_mut")
    a_a_b_b_manager_update_object_mut :: proc(self_: ^AABBManager, index: _c.uint32_t, bounds: ^Bounds3, distance: ^_c.float) ---

    /// Updates the broadphase and computes the lists of created/deleted pairs.
    ///
    /// The data necessary for updating the broadphase is internally computed by the AABB manager.
    ///
    /// To benefit from potentially multithreaded implementations, it is necessary to provide a continuation
    /// task to the function. It is legal to pass NULL there, but the underlying (CPU) implementations will
    /// then run single-threaded.
    @(link_name = "PxAABBManager_update_mut")
    a_a_b_b_manager_update_mut :: proc(self_: ^AABBManager, continuation: ^BaseTask) ---

    /// Retrieves the broadphase results after an update.
    ///
    /// This should be called once after each update call to retrieve the results of the broadphase. The
    /// results are incremental, i.e. the system only returns new and lost pairs, not all current pairs.
    @(link_name = "PxAABBManager_fetchResults_mut")
    a_a_b_b_manager_fetch_results_mut :: proc(self_: ^AABBManager, results: ^BroadPhaseResults) ---

    /// Helper for single-threaded updates.
    ///
    /// This short helper function performs a single-theaded update and reports the results in a single call.
    @(link_name = "PxAABBManager_update_mut_1")
    a_a_b_b_manager_update_mut_1 :: proc(self_: ^AABBManager, results: ^BroadPhaseResults) ---

    /// AABB manager factory function.
    ///
    /// Use this function to create a new standalone high-level broadphase.
    ///
    /// Newly created AABB manager, or NULL
    @(link_name = "phys_PxCreateAABBManager")
    create_a_a_b_b_manager :: proc(broadphase: ^BroadPhase) -> ^AABBManager ---

    /// constructor sets to default
    @(link_name = "PxSceneLimits_new")
    scene_limits_new :: proc() -> SceneLimits ---

    /// (re)sets the structure to the default
    @(link_name = "PxSceneLimits_setToDefault_mut")
    scene_limits_set_to_default_mut :: proc(self_: ^SceneLimits) ---

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid.
    @(link_name = "PxSceneLimits_isValid")
    scene_limits_is_valid :: proc(self_: ^SceneLimits) -> _c.bool ---

    @(link_name = "PxgDynamicsMemoryConfig_new")
    g_dynamics_memory_config_new :: proc() -> gDynamicsMemoryConfig ---

    @(link_name = "PxgDynamicsMemoryConfig_isValid")
    g_dynamics_memory_config_is_valid :: proc(self_: ^gDynamicsMemoryConfig) -> _c.bool ---

    /// constructor sets to default.
    @(link_name = "PxSceneDesc_new")
    scene_desc_new :: proc(#by_ptr scale: TolerancesScale) -> SceneDesc ---

    /// (re)sets the structure to the default.
    @(link_name = "PxSceneDesc_setToDefault_mut")
    scene_desc_set_to_default_mut :: proc(self_: ^SceneDesc, #by_ptr scale: TolerancesScale) ---

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid.
    @(link_name = "PxSceneDesc_isValid")
    scene_desc_is_valid :: proc(self_: ^SceneDesc) -> _c.bool ---

    @(link_name = "PxSceneDesc_getTolerancesScale")
    scene_desc_get_tolerances_scale :: proc(self_: ^SceneDesc) -> ^TolerancesScale ---

    /// Get number of broadphase volumes added for the current simulation step.
    ///
    /// Number of broadphase volumes added.
    @(link_name = "PxSimulationStatistics_getNbBroadPhaseAdds")
    simulation_statistics_get_nb_broad_phase_adds :: proc(self_: ^SimulationStatistics) -> _c.uint32_t ---

    /// Get number of broadphase volumes removed for the current simulation step.
    ///
    /// Number of broadphase volumes removed.
    @(link_name = "PxSimulationStatistics_getNbBroadPhaseRemoves")
    simulation_statistics_get_nb_broad_phase_removes :: proc(self_: ^SimulationStatistics) -> _c.uint32_t ---

    /// Get number of shape collision pairs of a certain type processed for the current simulation step.
    ///
    /// There is an entry for each geometry pair type.
    ///
    /// entry[i][j] = entry[j][i], hence, if you want the sum of all pair
    /// types, you need to discard the symmetric entries
    ///
    /// Number of processed pairs of the specified geometry types.
    @(link_name = "PxSimulationStatistics_getRbPairStats")
    simulation_statistics_get_rb_pair_stats :: proc(self_: ^SimulationStatistics, pairType: RbPairStatsType, g0: GeometryType, g1: GeometryType) -> _c.uint32_t ---

    @(link_name = "PxSimulationStatistics_new")
    simulation_statistics_new :: proc() -> SimulationStatistics ---

    /// Sets the PVD flag. See PxPvdSceneFlag.
    @(link_name = "PxPvdSceneClient_setScenePvdFlag_mut")
    pvd_scene_client_set_scene_pvd_flag_mut :: proc(self_: ^PvdSceneClient, flag: PvdSceneFlag, value: _c.bool) ---

    /// Sets the PVD flags. See PxPvdSceneFlags.
    @(link_name = "PxPvdSceneClient_setScenePvdFlags_mut")
    pvd_scene_client_set_scene_pvd_flags_mut :: proc(self_: ^PvdSceneClient, flags: PvdSceneFlags_Set) ---

    /// Retrieves the PVD flags. See PxPvdSceneFlags.
    @(link_name = "PxPvdSceneClient_getScenePvdFlags")
    pvd_scene_client_get_scene_pvd_flags :: proc(self_: ^PvdSceneClient) -> PvdSceneFlags_Set ---

    /// update camera on PVD application's render window
    @(link_name = "PxPvdSceneClient_updateCamera_mut")
    pvd_scene_client_update_camera_mut :: proc(self_: ^PvdSceneClient, name: cstring, #by_ptr origin: Vec3, #by_ptr up: Vec3, #by_ptr target: Vec3) ---

    /// draw points on PVD application's render window
    @(link_name = "PxPvdSceneClient_drawPoints_mut")
    pvd_scene_client_draw_points_mut :: proc(self_: ^PvdSceneClient, points: ^DebugPoint, count: _c.uint32_t) ---

    /// draw lines on PVD application's render window
    @(link_name = "PxPvdSceneClient_drawLines_mut")
    pvd_scene_client_draw_lines_mut :: proc(self_: ^PvdSceneClient, lines: ^DebugLine, count: _c.uint32_t) ---

    /// draw triangles on PVD application's render window
    @(link_name = "PxPvdSceneClient_drawTriangles_mut")
    pvd_scene_client_draw_triangles_mut :: proc(self_: ^PvdSceneClient, triangles: ^DebugTriangle, count: _c.uint32_t) ---

    /// draw text on PVD application's render window
    @(link_name = "PxPvdSceneClient_drawText_mut")
    pvd_scene_client_draw_text_mut :: proc(self_: ^PvdSceneClient, #by_ptr text: DebugText) ---

    @(link_name = "PxDominanceGroupPair_new")
    dominance_group_pair_new :: proc(a: _c.uint8_t, b: _c.uint8_t) -> DominanceGroupPair ---

    @(link_name = "PxBroadPhaseCallback_delete")
    broad_phase_callback_delete :: proc(self_: ^BroadPhaseCallback) ---

    /// Out-of-bounds notification.
    ///
    /// This function is called when an object leaves the broad-phase.
    @(link_name = "PxBroadPhaseCallback_onObjectOutOfBounds_mut")
    broad_phase_callback_on_object_out_of_bounds_mut :: proc(self_: ^BroadPhaseCallback, shape: ^Shape, actor: ^Actor) ---

    /// Out-of-bounds notification.
    ///
    /// This function is called when an aggregate leaves the broad-phase.
    @(link_name = "PxBroadPhaseCallback_onObjectOutOfBounds_mut_1")
    broad_phase_callback_on_object_out_of_bounds_mut_1 :: proc(self_: ^BroadPhaseCallback, aggregate: ^Aggregate) ---

    /// Deletes the scene.
    ///
    /// Removes any actors and constraint shaders from this scene
    /// (if the user hasn't already done so).
    ///
    /// Be sure to not keep a reference to this object after calling release.
    /// Avoid release calls while the scene is simulating (in between simulate() and fetchResults() calls).
    @(link_name = "PxScene_release_mut")
    scene_release_mut :: proc(self_: ^Scene) ---

    /// Sets a scene flag. You can only set one flag at a time.
    ///
    /// Not all flags are mutable and changing some will result in an error. Please check [`PxSceneFlag`] to see which flags can be changed.
    @(link_name = "PxScene_setFlag_mut")
    scene_set_flag_mut :: proc(self_: ^Scene, flag: SceneFlag, value: _c.bool) ---

    /// Get the scene flags.
    ///
    /// The scene flags. See [`PxSceneFlag`]
    @(link_name = "PxScene_getFlags")
    scene_get_flags :: proc(self_: ^Scene) -> SceneFlags_Set ---

    /// Set new scene limits.
    ///
    /// Increase the maximum capacity of various data structures in the scene. The new capacities will be
    /// at least as large as required to deal with the objects currently in the scene. Further, these values
    /// are for preallocation and do not represent hard limits.
    @(link_name = "PxScene_setLimits_mut")
    scene_set_limits_mut :: proc(self_: ^Scene, #by_ptr limits: SceneLimits) ---

    /// Get current scene limits.
    ///
    /// Current scene limits.
    @(link_name = "PxScene_getLimits")
    scene_get_limits :: proc(self_: ^Scene) -> SceneLimits ---

    /// Call this method to retrieve the Physics SDK.
    ///
    /// The physics SDK this scene is associated with.
    @(link_name = "PxScene_getPhysics_mut")
    scene_get_physics_mut :: proc(self_: ^Scene) -> ^Physics ---

    /// Retrieves the scene's internal timestamp, increased each time a simulation step is completed.
    ///
    /// scene timestamp
    @(link_name = "PxScene_getTimestamp")
    scene_get_timestamp :: proc(self_: ^Scene) -> _c.uint32_t ---

    /// Adds an articulation to this scene.
    ///
    /// If the articulation is already assigned to a scene (see [`PxArticulationReducedCoordinate::getScene`]), the call is ignored and an error is issued.
    ///
    /// True if success
    @(link_name = "PxScene_addArticulation_mut")
    scene_add_articulation_mut :: proc(self_: ^Scene, articulation: ^ArticulationReducedCoordinate) -> _c.bool ---

    /// Removes an articulation from this scene.
    ///
    /// If the articulation is not part of this scene (see [`PxArticulationReducedCoordinate::getScene`]), the call is ignored and an error is issued.
    ///
    /// If the articulation is in an aggregate it will be removed from the aggregate.
    @(link_name = "PxScene_removeArticulation_mut")
    scene_remove_articulation_mut :: proc(self_: ^Scene, articulation: ^ArticulationReducedCoordinate, wakeOnLostTouch: _c.bool) ---

    /// Adds an actor to this scene.
    ///
    /// If the actor is already assigned to a scene (see [`PxActor::getScene`]), the call is ignored and an error is issued.
    ///
    /// If the actor has an invalid constraint, in checked builds the call is ignored and an error is issued.
    ///
    /// You can not add individual articulation links (see [`PxArticulationLink`]) to the scene. Use #addArticulation() instead.
    ///
    /// If the actor is a PxRigidActor then each assigned PxConstraint object will get added to the scene automatically if
    /// it connects to another actor that is part of the scene already.
    ///
    /// When a BVH is provided the actor shapes are grouped together.
    /// The scene query pruning structure inside PhysX SDK will store/update one
    /// bound per actor. The scene queries against such an actor will query actor
    /// bounds and then make a local space query against the provided BVH, which is in actor's local space.
    ///
    /// True if success
    @(link_name = "PxScene_addActor_mut")
    scene_add_actor_mut :: proc(self_: ^Scene, actor: ^Actor, bvh: ^BVH) -> _c.bool ---

    /// Adds actors to this scene. Only supports actors of type PxRigidStatic and PxRigidDynamic.
    ///
    /// This method only supports actors of type PxRigidStatic and PxRigidDynamic. For other actors, use addActor() instead.
    /// For articulation links, use addArticulation().
    ///
    /// If one of the actors is already assigned to a scene (see [`PxActor::getScene`]), the call is ignored and an error is issued.
    ///
    /// If an actor in the array contains an invalid constraint, in checked builds the call is ignored and an error is issued.
    ///
    /// If an actor in the array is a PxRigidActor then each assigned PxConstraint object will get added to the scene automatically if
    /// it connects to another actor that is part of the scene already.
    ///
    /// this method is optimized for high performance.
    ///
    /// True if success
    @(link_name = "PxScene_addActors_mut")
    scene_add_actors_mut :: proc(self_: ^Scene, actors: [^]^Actor, nbActors: _c.uint32_t) -> _c.bool ---

    /// Adds a pruning structure together with its actors to this scene. Only supports actors of type PxRigidStatic and PxRigidDynamic.
    ///
    /// This method only supports actors of type PxRigidStatic and PxRigidDynamic. For other actors, use addActor() instead.
    /// For articulation links, use addArticulation().
    ///
    /// If an actor in the pruning structure contains an invalid constraint, in checked builds the call is ignored and an error is issued.
    ///
    /// For all actors in the pruning structure each assigned PxConstraint object will get added to the scene automatically if
    /// it connects to another actor that is part of the scene already.
    ///
    /// This method is optimized for high performance.
    ///
    /// Merging a PxPruningStructure into an active scene query optimization AABB tree might unbalance the tree. A typical use case for
    /// PxPruningStructure is a large world scenario where blocks of closely positioned actors get streamed in. The merge process finds the
    /// best node in the active scene query optimization AABB tree and inserts the PxPruningStructure. Therefore using PxPruningStructure
    /// for actors scattered throughout the world will result in an unbalanced tree.
    ///
    /// True if success
    @(link_name = "PxScene_addActors_mut_1")
    scene_add_actors_mut_1 :: proc(self_: ^Scene, #by_ptr pruningStructure: PruningStructure) -> _c.bool ---

    /// Removes an actor from this scene.
    ///
    /// If the actor is not part of this scene (see [`PxActor::getScene`]), the call is ignored and an error is issued.
    ///
    /// You can not remove individual articulation links (see [`PxArticulationLink`]) from the scene. Use #removeArticulation() instead.
    ///
    /// If the actor is a PxRigidActor then all assigned PxConstraint objects will get removed from the scene automatically.
    ///
    /// If the actor is in an aggregate it will be removed from the aggregate.
    @(link_name = "PxScene_removeActor_mut")
    scene_remove_actor_mut :: proc(self_: ^Scene, actor: ^Actor, wakeOnLostTouch: _c.bool) ---

    /// Removes actors from this scene. Only supports actors of type PxRigidStatic and PxRigidDynamic.
    ///
    /// This method only supports actors of type PxRigidStatic and PxRigidDynamic. For other actors, use removeActor() instead.
    /// For articulation links, use removeArticulation().
    ///
    /// If some actor is not part of this scene (see [`PxActor::getScene`]), the actor remove is ignored and an error is issued.
    ///
    /// You can not remove individual articulation links (see [`PxArticulationLink`]) from the scene. Use #removeArticulation() instead.
    ///
    /// If the actor is a PxRigidActor then all assigned PxConstraint objects will get removed from the scene automatically.
    @(link_name = "PxScene_removeActors_mut")
    scene_remove_actors_mut :: proc(self_: ^Scene, actors: [^]^Actor, nbActors: _c.uint32_t, wakeOnLostTouch: _c.bool) ---

    /// Adds an aggregate to this scene.
    ///
    /// If the aggregate is already assigned to a scene (see [`PxAggregate::getScene`]), the call is ignored and an error is issued.
    ///
    /// If the aggregate contains an actor with an invalid constraint, in checked builds the call is ignored and an error is issued.
    ///
    /// If the aggregate already contains actors, those actors are added to the scene as well.
    ///
    /// True if success
    @(link_name = "PxScene_addAggregate_mut")
    scene_add_aggregate_mut :: proc(self_: ^Scene, aggregate: ^Aggregate) -> _c.bool ---

    /// Removes an aggregate from this scene.
    ///
    /// If the aggregate is not part of this scene (see [`PxAggregate::getScene`]), the call is ignored and an error is issued.
    ///
    /// If the aggregate contains actors, those actors are removed from the scene as well.
    @(link_name = "PxScene_removeAggregate_mut")
    scene_remove_aggregate_mut :: proc(self_: ^Scene, aggregate: ^Aggregate, wakeOnLostTouch: _c.bool) ---

    /// Adds objects in the collection to this scene.
    ///
    /// This function adds the following types of objects to this scene: PxRigidActor (except PxArticulationLink), PxAggregate, PxArticulationReducedCoordinate.
    /// This method is typically used after deserializing the collection in order to populate the scene with deserialized objects.
    ///
    /// If the collection contains an actor with an invalid constraint, in checked builds the call is ignored and an error is issued.
    ///
    /// True if success
    @(link_name = "PxScene_addCollection_mut")
    scene_add_collection_mut :: proc(self_: ^Scene, #by_ptr collection: Collection) -> _c.bool ---

    /// Retrieve the number of actors of certain types in the scene. For supported types, see PxActorTypeFlags.
    ///
    /// the number of actors.
    @(link_name = "PxScene_getNbActors")
    scene_get_nb_actors :: proc(self_: ^Scene, types: ActorTypeFlags_Set) -> _c.uint32_t ---

    /// Retrieve an array of all the actors of certain types in the scene. For supported types, see PxActorTypeFlags.
    ///
    /// Number of actors written to the buffer.
    @(link_name = "PxScene_getActors")
    scene_get_actors :: proc(self_: ^Scene, types: ActorTypeFlags_Set, userBuffer: [^]^Actor, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Queries the PxScene for a list of the PxActors whose transforms have been
    /// updated during the previous simulation step. Only includes actors of type PxRigidDynamic and PxArticulationLink.
    ///
    /// PxSceneFlag::eENABLE_ACTIVE_ACTORS must be set.
    ///
    /// Do not use this method while the simulation is running. Calls to this method while the simulation is running will be ignored and NULL will be returned.
    ///
    /// A pointer to the list of active PxActors generated during the last call to fetchResults().
    @(link_name = "PxScene_getActiveActors_mut")
    scene_get_active_actors_mut :: proc(self_: ^Scene, nbActorsOut: ^_c.uint32_t) -> [^]^Actor ---

    /// Returns the number of articulations in the scene.
    ///
    /// the number of articulations in this scene.
    @(link_name = "PxScene_getNbArticulations")
    scene_get_nb_articulations :: proc(self_: ^Scene) -> _c.uint32_t ---

    /// Retrieve all the articulations in the scene.
    ///
    /// Number of articulations written to the buffer.
    @(link_name = "PxScene_getArticulations")
    scene_get_articulations :: proc(self_: ^Scene, userBuffer: [^]^ArticulationReducedCoordinate, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Returns the number of constraint shaders in the scene.
    ///
    /// the number of constraint shaders in this scene.
    @(link_name = "PxScene_getNbConstraints")
    scene_get_nb_constraints :: proc(self_: ^Scene) -> _c.uint32_t ---

    /// Retrieve all the constraint shaders in the scene.
    ///
    /// Number of constraint shaders written to the buffer.
    @(link_name = "PxScene_getConstraints")
    scene_get_constraints :: proc(self_: ^Scene, userBuffer: [^]^Constraint, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Returns the number of aggregates in the scene.
    ///
    /// the number of aggregates in this scene.
    @(link_name = "PxScene_getNbAggregates")
    scene_get_nb_aggregates :: proc(self_: ^Scene) -> _c.uint32_t ---

    /// Retrieve all the aggregates in the scene.
    ///
    /// Number of aggregates written to the buffer.
    @(link_name = "PxScene_getAggregates")
    scene_get_aggregates :: proc(self_: ^Scene, userBuffer: [^]^Aggregate, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Specifies the dominance behavior of contacts between two actors with two certain dominance groups.
    ///
    /// It is possible to assign each actor to a dominance groups using [`PxActor::setDominanceGroup`]().
    ///
    /// With dominance groups one can have all contacts created between actors act in one direction only. This is useful, for example, if you
    /// want an object to push debris out of its way and be unaffected,while still responding physically to forces and collisions
    /// with non-debris objects.
    ///
    /// Whenever a contact between two actors (a0, a1) needs to be solved, the groups (g0, g1) of both
    /// actors are retrieved. Then the PxDominanceGroupPair setting for this group pair is retrieved with getDominanceGroupPair(g0, g1).
    ///
    /// In the contact, PxDominanceGroupPair::dominance0 becomes the dominance setting for a0, and
    /// PxDominanceGroupPair::dominance1 becomes the dominance setting for a1. A dominanceN setting of 1.0f, the default,
    /// will permit aN to be pushed or pulled by a(1-N) through the contact. A dominanceN setting of 0.0f, will however
    /// prevent aN to be pushed by a(1-N) via the contact. Thus, a PxDominanceGroupPair of (1.0f, 0.0f) makes
    /// the interaction one-way.
    ///
    /// The matrix sampled by getDominanceGroupPair(g1, g2) is initialised by default such that:
    ///
    /// if g1 == g2, then (1.0f, 1.0f) is returned
    /// if g1
    /// <
    /// g2, then (0.0f, 1.0f) is returned
    /// if g1 >  g2, then (1.0f, 0.0f) is returned
    ///
    /// In other words, we permit actors in higher groups to be pushed around by actors in lower groups by default.
    ///
    /// These settings should cover most applications, and in fact not overriding these settings may likely result in higher performance.
    ///
    /// It is not possible to make the matrix asymetric, or to change the diagonal. In other words:
    ///
    /// it is not possible to change (g1, g2) if (g1==g2)
    /// if you set
    ///
    /// (g1, g2) to X, then (g2, g1) will implicitly and automatically be set to ~X, where:
    ///
    /// ~(1.0f, 1.0f) is (1.0f, 1.0f)
    /// ~(0.0f, 1.0f) is (1.0f, 0.0f)
    /// ~(1.0f, 0.0f) is (0.0f, 1.0f)
    ///
    /// These two restrictions are to make sure that contacts between two actors will always evaluate to the same dominance
    /// setting, regardless of the order of the actors.
    ///
    /// Dominance settings are currently specified as floats 0.0f or 1.0f because in the future we may permit arbitrary
    /// fractional settings to express 'partly-one-way' interactions.
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake actors up automatically.
    @(link_name = "PxScene_setDominanceGroupPair_mut")
    scene_set_dominance_group_pair_mut :: proc(self_: ^Scene, group1: _c.uint8_t, group2: _c.uint8_t, #by_ptr dominance: DominanceGroupPair) ---

    /// Samples the dominance matrix.
    @(link_name = "PxScene_getDominanceGroupPair")
    scene_get_dominance_group_pair :: proc(self_: ^Scene, group1: _c.uint8_t, group2: _c.uint8_t) -> DominanceGroupPair ---

    /// Return the cpu dispatcher that was set in PxSceneDesc::cpuDispatcher when creating the scene with PxPhysics::createScene
    @(link_name = "PxScene_getCpuDispatcher")
    scene_get_cpu_dispatcher :: proc(self_: ^Scene) -> ^CpuDispatcher ---

    /// Reserves a new client ID.
    ///
    /// PX_DEFAULT_CLIENT is always available as the default clientID.
    /// Additional clients are returned by this function. Clients cannot be released once created.
    /// An error is reported when more than a supported number of clients (currently 128) are created.
    @(link_name = "PxScene_createClient_mut")
    scene_create_client_mut :: proc(self_: ^Scene) -> _c.uint8_t ---

    /// Sets a user notify object which receives special simulation events when they occur.
    ///
    /// Do not set the callback while the simulation is running. Calls to this method while the simulation is running will be ignored.
    @(link_name = "PxScene_setSimulationEventCallback_mut")
    scene_set_simulation_event_callback_mut :: proc(self_: ^Scene, callback: ^SimulationEventCallback) ---

    /// Retrieves the simulationEventCallback pointer set with setSimulationEventCallback().
    ///
    /// The current user notify pointer. See [`PxSimulationEventCallback`].
    @(link_name = "PxScene_getSimulationEventCallback")
    scene_get_simulation_event_callback :: proc(self_: ^Scene) -> ^SimulationEventCallback ---

    /// Sets a user callback object, which receives callbacks on all contacts generated for specified actors.
    ///
    /// Do not set the callback while the simulation is running. Calls to this method while the simulation is running will be ignored.
    @(link_name = "PxScene_setContactModifyCallback_mut")
    scene_set_contact_modify_callback_mut :: proc(self_: ^Scene, callback: ^ContactModifyCallback) ---

    /// Sets a user callback object, which receives callbacks on all CCD contacts generated for specified actors.
    ///
    /// Do not set the callback while the simulation is running. Calls to this method while the simulation is running will be ignored.
    @(link_name = "PxScene_setCCDContactModifyCallback_mut")
    scene_set_ccd_contact_modify_callback_mut :: proc(self_: ^Scene, callback: ^CCDContactModifyCallback) ---

    /// Retrieves the PxContactModifyCallback pointer set with setContactModifyCallback().
    ///
    /// The current user contact modify callback pointer. See [`PxContactModifyCallback`].
    @(link_name = "PxScene_getContactModifyCallback")
    scene_get_contact_modify_callback :: proc(self_: ^Scene) -> ^ContactModifyCallback ---

    /// Retrieves the PxCCDContactModifyCallback pointer set with setContactModifyCallback().
    ///
    /// The current user contact modify callback pointer. See [`PxContactModifyCallback`].
    @(link_name = "PxScene_getCCDContactModifyCallback")
    scene_get_ccd_contact_modify_callback :: proc(self_: ^Scene) -> ^CCDContactModifyCallback ---

    /// Sets a broad-phase user callback object.
    ///
    /// Do not set the callback while the simulation is running. Calls to this method while the simulation is running will be ignored.
    @(link_name = "PxScene_setBroadPhaseCallback_mut")
    scene_set_broad_phase_callback_mut :: proc(self_: ^Scene, callback: ^BroadPhaseCallback) ---

    /// Retrieves the PxBroadPhaseCallback pointer set with setBroadPhaseCallback().
    ///
    /// The current broad-phase callback pointer. See [`PxBroadPhaseCallback`].
    @(link_name = "PxScene_getBroadPhaseCallback")
    scene_get_broad_phase_callback :: proc(self_: ^Scene) -> ^BroadPhaseCallback ---

    /// Sets the shared global filter data which will get passed into the filter shader.
    ///
    /// It is the user's responsibility to ensure that changing the shared global filter data does not change the filter output value for existing pairs.
    /// If the filter output for existing pairs does change nonetheless then such a change will not take effect until the pair gets refiltered.
    /// resetFiltering() can be used to explicitly refilter the pairs of specific objects.
    ///
    /// The provided data will get copied to internal buffers and this copy will be used for filtering calls.
    ///
    /// Do not use this method while the simulation is running. Calls to this method while the simulation is running will be ignored.
    @(link_name = "PxScene_setFilterShaderData_mut")
    scene_set_filter_shader_data_mut :: proc(self_: ^Scene, data: rawptr, dataSize: _c.uint32_t) ---

    /// Gets the shared global filter data in use for this scene.
    ///
    /// The reference points to a copy of the original filter data specified in [`PxSceneDesc`].filterShaderData or provided by #setFilterShaderData().
    ///
    /// Shared filter data for filter shader.
    @(link_name = "PxScene_getFilterShaderData")
    scene_get_filter_shader_data :: proc(self_: ^Scene) -> rawptr ---

    /// Gets the size of the shared global filter data ([`PxSceneDesc`].filterShaderData)
    ///
    /// Size of shared filter data [bytes].
    @(link_name = "PxScene_getFilterShaderDataSize")
    scene_get_filter_shader_data_size :: proc(self_: ^Scene) -> _c.uint32_t ---

    /// Marks the object to reset interactions and re-run collision filters in the next simulation step.
    ///
    /// This call forces the object to remove all existing collision interactions, to search anew for existing contact
    /// pairs and to run the collision filters again for found collision pairs.
    ///
    /// The operation is supported for PxRigidActor objects only.
    ///
    /// All persistent state of existing interactions will be lost and can not be retrieved even if the same collison pair
    /// is found again in the next step. This will mean, for example, that you will not get notified about persistent contact
    /// for such an interaction (see [`PxPairFlag::eNOTIFY_TOUCH_PERSISTS`]), the contact pair will be interpreted as newly found instead.
    ///
    /// Lost touch contact reports will be sent for every collision pair which includes this shape, if they have
    /// been requested through [`PxPairFlag::eNOTIFY_TOUCH_LOST`] or #PxPairFlag::eNOTIFY_THRESHOLD_FORCE_LOST.
    ///
    /// This is an expensive operation, don't use it if you don't have to.
    ///
    /// Can be used to retrieve collision pairs that were killed by the collision filters (see [`PxFilterFlag::eKILL`])
    ///
    /// It is invalid to use this method if the actor has not been added to a scene already.
    ///
    /// It is invalid to use this method if PxActorFlag::eDISABLE_SIMULATION is set.
    ///
    /// Do not use this method while the simulation is running.
    ///
    /// Sleeping:
    /// Does wake up the actor.
    ///
    /// True if success
    @(link_name = "PxScene_resetFiltering_mut")
    scene_reset_filtering_mut :: proc(self_: ^Scene, actor: ^Actor) -> _c.bool ---

    /// Marks the object to reset interactions and re-run collision filters for specified shapes in the next simulation step.
    ///
    /// This is a specialization of the resetFiltering(PxActor
    /// &
    /// actor) method and allows to reset interactions for specific shapes of
    /// a PxRigidActor.
    ///
    /// Do not use this method while the simulation is running.
    ///
    /// Sleeping:
    /// Does wake up the actor.
    @(link_name = "PxScene_resetFiltering_mut_1")
    scene_reset_filtering_mut_1 :: proc(self_: ^Scene, actor: ^RigidActor, shapes: [^]^Shape, shapeCount: _c.uint32_t) -> _c.bool ---

    /// Gets the pair filtering mode for kinematic-kinematic pairs.
    ///
    /// Filtering mode for kinematic-kinematic pairs.
    @(link_name = "PxScene_getKinematicKinematicFilteringMode")
    scene_get_kinematic_kinematic_filtering_mode :: proc(self_: ^Scene) -> PairFilteringMode ---

    /// Gets the pair filtering mode for static-kinematic pairs.
    ///
    /// Filtering mode for static-kinematic pairs.
    @(link_name = "PxScene_getStaticKinematicFilteringMode")
    scene_get_static_kinematic_filtering_mode :: proc(self_: ^Scene) -> PairFilteringMode ---

    /// Advances the simulation by an elapsedTime time.
    ///
    /// Large elapsedTime values can lead to instabilities. In such cases elapsedTime
    /// should be subdivided into smaller time intervals and simulate() should be called
    /// multiple times for each interval.
    ///
    /// Calls to simulate() should pair with calls to fetchResults():
    /// Each fetchResults() invocation corresponds to exactly one simulate()
    /// invocation; calling simulate() twice without an intervening fetchResults()
    /// or fetchResults() twice without an intervening simulate() causes an error
    /// condition.
    ///
    /// scene->simulate();
    /// ...do some processing until physics is computed...
    /// scene->fetchResults();
    /// ...now results of run may be retrieved.
    ///
    /// True if success
    @(link_name = "PxScene_simulate_mut")
    scene_simulate_mut :: proc(self_: ^Scene, elapsedTime: _c.float, completionTask: ^BaseTask, scratchMemBlock: rawptr, scratchMemBlockSize: _c.uint32_t, controlSimulation: _c.bool) -> _c.bool ---

    /// Performs dynamics phase of the simulation pipeline.
    ///
    /// Calls to advance() should follow calls to fetchCollision(). An error message will be issued if this sequence is not followed.
    ///
    /// True if success
    @(link_name = "PxScene_advance_mut")
    scene_advance_mut :: proc(self_: ^Scene, completionTask: ^BaseTask) -> _c.bool ---

    /// Performs collision detection for the scene over elapsedTime
    ///
    /// Calls to collide() should be the first method called to simulate a frame.
    ///
    /// True if success
    @(link_name = "PxScene_collide_mut")
    scene_collide_mut :: proc(self_: ^Scene, elapsedTime: _c.float, completionTask: ^BaseTask, scratchMemBlock: rawptr, scratchMemBlockSize: _c.uint32_t, controlSimulation: _c.bool) -> _c.bool ---

    /// This checks to see if the simulation run has completed.
    ///
    /// This does not cause the data available for reading to be updated with the results of the simulation, it is simply a status check.
    /// The bool will allow it to either return immediately or block waiting for the condition to be met so that it can return true
    ///
    /// True if the results are available.
    @(link_name = "PxScene_checkResults_mut")
    scene_check_results_mut :: proc(self_: ^Scene, block: _c.bool) -> _c.bool ---

    /// This method must be called after collide() and before advance(). It will wait for the collision phase to finish. If the user makes an illegal simulation call, the SDK will issue an error
    /// message.
    @(link_name = "PxScene_fetchCollision_mut")
    scene_fetch_collision_mut :: proc(self_: ^Scene, block: _c.bool) -> _c.bool ---

    /// This is the big brother to checkResults() it basically does the following:
    ///
    /// True if the results have been fetched.
    @(link_name = "PxScene_fetchResults_mut")
    scene_fetch_results_mut :: proc(self_: ^Scene, block: _c.bool, errorState: ^_c.uint32_t) -> _c.bool ---

    /// This call performs the first section of fetchResults, and returns a pointer to the contact streams output by the simulation. It can be used to process contact pairs in parallel, which is often a limiting factor
    /// for fetchResults() performance.
    ///
    /// After calling this function and processing the contact streams, call fetchResultsFinish(). Note that writes to the simulation are not
    /// permitted between the start of fetchResultsStart() and the end of fetchResultsFinish().
    ///
    /// True if the results have been fetched.
    @(link_name = "PxScene_fetchResultsStart_mut")
    scene_fetch_results_start_mut :: proc(self_: ^Scene, contactPairs: ^^ContactPairHeader, nbContactPairs: ^_c.uint32_t, block: _c.bool) -> _c.bool ---

    /// This call processes all event callbacks in parallel. It takes a continuation task, which will be executed once all callbacks have been processed.
    ///
    /// This is a utility function to make it easier to process callbacks in parallel using the PhysX task system. It can only be used in conjunction with
    /// fetchResultsStart(...) and fetchResultsFinish(...)
    @(link_name = "PxScene_processCallbacks_mut")
    scene_process_callbacks_mut :: proc(self_: ^Scene, continuation: ^BaseTask) ---

    /// This call performs the second section of fetchResults.
    ///
    /// It must be called after fetchResultsStart() returns and contact reports have been processed.
    ///
    /// Note that once fetchResultsFinish() has been called, the contact streams returned in fetchResultsStart() will be invalid.
    @(link_name = "PxScene_fetchResultsFinish_mut")
    scene_fetch_results_finish_mut :: proc(self_: ^Scene, errorState: ^_c.uint32_t) ---

    /// This call performs the synchronization of particle system data copies.
    @(link_name = "PxScene_fetchResultsParticleSystem_mut")
    scene_fetch_results_particle_system_mut :: proc(self_: ^Scene) ---

    /// Clear internal buffers and free memory.
    ///
    /// This method can be used to clear buffers and free internal memory without having to destroy the scene. Can be useful if
    /// the physics data gets streamed in and a checkpoint with a clean state should be created.
    ///
    /// It is not allowed to call this method while the simulation is running. The call will fail.
    @(link_name = "PxScene_flushSimulation_mut")
    scene_flush_simulation_mut :: proc(self_: ^Scene, sendPendingReports: _c.bool) ---

    /// Sets a constant gravity for the entire scene.
    ///
    /// Do not use this method while the simulation is running.
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the actor up automatically.
    @(link_name = "PxScene_setGravity_mut")
    scene_set_gravity_mut :: proc(self_: ^Scene, #by_ptr vec: Vec3) ---

    /// Retrieves the current gravity setting.
    ///
    /// The current gravity for the scene.
    @(link_name = "PxScene_getGravity")
    scene_get_gravity :: proc(self_: ^Scene) -> Vec3 ---

    /// Set the bounce threshold velocity.  Collision speeds below this threshold will not cause a bounce.
    ///
    /// Do not use this method while the simulation is running.
    @(link_name = "PxScene_setBounceThresholdVelocity_mut")
    scene_set_bounce_threshold_velocity_mut :: proc(self_: ^Scene, t: _c.float) ---

    /// Return the bounce threshold velocity.
    @(link_name = "PxScene_getBounceThresholdVelocity")
    scene_get_bounce_threshold_velocity :: proc(self_: ^Scene) -> _c.float ---

    /// Sets the maximum number of CCD passes
    ///
    /// Do not use this method while the simulation is running.
    @(link_name = "PxScene_setCCDMaxPasses_mut")
    scene_set_ccd_max_passes_mut :: proc(self_: ^Scene, ccdMaxPasses: _c.uint32_t) ---

    /// Gets the maximum number of CCD passes.
    ///
    /// The maximum number of CCD passes.
    @(link_name = "PxScene_getCCDMaxPasses")
    scene_get_ccd_max_passes :: proc(self_: ^Scene) -> _c.uint32_t ---

    /// Set the maximum CCD separation.
    ///
    /// Do not use this method while the simulation is running.
    @(link_name = "PxScene_setCCDMaxSeparation_mut")
    scene_set_ccd_max_separation_mut :: proc(self_: ^Scene, t: _c.float) ---

    /// Gets the maximum CCD separation.
    ///
    /// The maximum CCD separation.
    @(link_name = "PxScene_getCCDMaxSeparation")
    scene_get_ccd_max_separation :: proc(self_: ^Scene) -> _c.float ---

    /// Set the CCD threshold.
    ///
    /// Do not use this method while the simulation is running.
    @(link_name = "PxScene_setCCDThreshold_mut")
    scene_set_ccd_threshold_mut :: proc(self_: ^Scene, t: _c.float) ---

    /// Gets the CCD threshold.
    ///
    /// The CCD threshold.
    @(link_name = "PxScene_getCCDThreshold")
    scene_get_ccd_threshold :: proc(self_: ^Scene) -> _c.float ---

    /// Set the max bias coefficient.
    ///
    /// Do not use this method while the simulation is running.
    @(link_name = "PxScene_setMaxBiasCoefficient_mut")
    scene_set_max_bias_coefficient_mut :: proc(self_: ^Scene, t: _c.float) ---

    /// Gets the max bias coefficient.
    ///
    /// The max bias coefficient.
    @(link_name = "PxScene_getMaxBiasCoefficient")
    scene_get_max_bias_coefficient :: proc(self_: ^Scene) -> _c.float ---

    /// Set the friction offset threshold.
    ///
    /// Do not use this method while the simulation is running.
    @(link_name = "PxScene_setFrictionOffsetThreshold_mut")
    scene_set_friction_offset_threshold_mut :: proc(self_: ^Scene, t: _c.float) ---

    /// Gets the friction offset threshold.
    @(link_name = "PxScene_getFrictionOffsetThreshold")
    scene_get_friction_offset_threshold :: proc(self_: ^Scene) -> _c.float ---

    /// Set the friction correlation distance.
    ///
    /// Do not use this method while the simulation is running.
    @(link_name = "PxScene_setFrictionCorrelationDistance_mut")
    scene_set_friction_correlation_distance_mut :: proc(self_: ^Scene, t: _c.float) ---

    /// Gets the friction correlation distance.
    @(link_name = "PxScene_getFrictionCorrelationDistance")
    scene_get_friction_correlation_distance :: proc(self_: ^Scene) -> _c.float ---

    /// Return the friction model.
    @(link_name = "PxScene_getFrictionType")
    scene_get_friction_type :: proc(self_: ^Scene) -> FrictionType ---

    /// Return the solver model.
    @(link_name = "PxScene_getSolverType")
    scene_get_solver_type :: proc(self_: ^Scene) -> SolverType ---

    /// Function that lets you set debug visualization parameters.
    ///
    /// Returns false if the value passed is out of range for usage specified by the enum.
    ///
    /// Do not use this method while the simulation is running.
    ///
    /// False if the parameter is out of range.
    @(link_name = "PxScene_setVisualizationParameter_mut")
    scene_set_visualization_parameter_mut :: proc(self_: ^Scene, param: VisualizationParameter, value: _c.float) -> _c.bool ---

    /// Function that lets you query debug visualization parameters.
    ///
    /// The value of the parameter.
    @(link_name = "PxScene_getVisualizationParameter")
    scene_get_visualization_parameter :: proc(self_: ^Scene, paramEnum: VisualizationParameter) -> _c.float ---

    /// Defines a box in world space to which visualization geometry will be (conservatively) culled. Use a non-empty culling box to enable the feature, and an empty culling box to disable it.
    ///
    /// Do not use this method while the simulation is running.
    @(link_name = "PxScene_setVisualizationCullingBox_mut")
    scene_set_visualization_culling_box_mut :: proc(self_: ^Scene, #by_ptr box: Bounds3) ---

    /// Retrieves the visualization culling box.
    ///
    /// the box to which the geometry will be culled.
    @(link_name = "PxScene_getVisualizationCullingBox")
    scene_get_visualization_culling_box :: proc(self_: ^Scene) -> Bounds3 ---

    /// Retrieves the render buffer.
    ///
    /// This will contain the results of any active visualization for this scene.
    ///
    /// Do not use this method while the simulation is running. Calls to this method while the simulation is running will result in undefined behaviour.
    ///
    /// The render buffer.
    @(link_name = "PxScene_getRenderBuffer_mut")
    scene_get_render_buffer_mut :: proc(self_: ^Scene) -> ^RenderBuffer ---

    /// Call this method to retrieve statistics for the current simulation step.
    ///
    /// Do not use this method while the simulation is running. Calls to this method while the simulation is running will be ignored.
    @(link_name = "PxScene_getSimulationStatistics")
    scene_get_simulation_statistics :: proc(self_: ^Scene, stats: ^SimulationStatistics) ---

    /// Returns broad-phase type.
    ///
    /// Broad-phase type
    @(link_name = "PxScene_getBroadPhaseType")
    scene_get_broad_phase_type :: proc(self_: ^Scene) -> BroadPhaseType ---

    /// Gets broad-phase caps.
    ///
    /// True if success
    @(link_name = "PxScene_getBroadPhaseCaps")
    scene_get_broad_phase_caps :: proc(self_: ^Scene, caps: ^BroadPhaseCaps) -> _c.bool ---

    /// Returns number of regions currently registered in the broad-phase.
    ///
    /// Number of regions
    @(link_name = "PxScene_getNbBroadPhaseRegions")
    scene_get_nb_broad_phase_regions :: proc(self_: ^Scene) -> _c.uint32_t ---

    /// Gets broad-phase regions.
    ///
    /// Number of written out regions
    @(link_name = "PxScene_getBroadPhaseRegions")
    scene_get_broad_phase_regions :: proc(self_: ^Scene, userBuffer: ^BroadPhaseRegionInfo, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Adds a new broad-phase region.
    ///
    /// The bounds for the new region must be non-empty, otherwise an error occurs and the call is ignored.
    ///
    /// Note that by default, objects already existing in the SDK that might touch this region will not be automatically
    /// added to the region. In other words the newly created region will be empty, and will only be populated with new
    /// objects when they are added to the simulation, or with already existing objects when they are updated.
    ///
    /// It is nonetheless possible to override this default behavior and let the SDK populate the new region automatically
    /// with already existing objects overlapping the incoming region. This has a cost though, and it should only be used
    /// when the game can not guarantee that all objects within the new region will be added to the simulation after the
    /// region itself.
    ///
    /// Objects automatically move from one region to another during their lifetime. The system keeps tracks of what
    /// regions a given object is in. It is legal for an object to be in an arbitrary number of regions. However if an
    /// object leaves all regions, or is created outside of all regions, several things happen:
    /// - collisions get disabled for this object
    /// - if a PxBroadPhaseCallback object is provided, an "out-of-bounds" event is generated via that callback
    /// - if a PxBroadPhaseCallback object is not provided, a warning/error message is sent to the error stream
    ///
    /// If an object goes out-of-bounds and user deletes it during the same frame, neither the out-of-bounds event nor the
    /// error message is generated.
    ///
    /// Handle for newly created region, or 0xffffffff in case of failure.
    @(link_name = "PxScene_addBroadPhaseRegion_mut")
    scene_add_broad_phase_region_mut :: proc(self_: ^Scene, #by_ptr region: BroadPhaseRegion, populateRegion: _c.bool) -> _c.uint32_t ---

    /// Removes a new broad-phase region.
    ///
    /// If the region still contains objects, and if those objects do not overlap any region any more, they are not
    /// automatically removed from the simulation. Instead, the PxBroadPhaseCallback::onObjectOutOfBounds notification
    /// is used for each object. Users are responsible for removing the objects from the simulation if this is the
    /// desired behavior.
    ///
    /// If the handle is invalid, or if a valid handle is removed twice, an error message is sent to the error stream.
    ///
    /// True if success
    @(link_name = "PxScene_removeBroadPhaseRegion_mut")
    scene_remove_broad_phase_region_mut :: proc(self_: ^Scene, handle: _c.uint32_t) -> _c.bool ---

    /// Get the task manager associated with this scene
    ///
    /// the task manager associated with the scene
    @(link_name = "PxScene_getTaskManager")
    scene_get_task_manager :: proc(self_: ^Scene) -> ^TaskManager ---

    /// Lock the scene for reading from the calling thread.
    ///
    /// When the PxSceneFlag::eREQUIRE_RW_LOCK flag is enabled lockRead() must be
    /// called before any read calls are made on the scene.
    ///
    /// Multiple threads may read at the same time, no threads may read while a thread is writing.
    /// If a call to lockRead() is made while another thread is holding a write lock
    /// then the calling thread will be blocked until the writing thread calls unlockWrite().
    ///
    /// Lock upgrading is *not* supported, that means it is an error to
    /// call lockRead() followed by lockWrite().
    ///
    /// Recursive locking is supported but each lockRead() call must be paired with an unlockRead().
    @(link_name = "PxScene_lockRead_mut")
    scene_lock_read_mut :: proc(self_: ^Scene, file: cstring, line: _c.uint32_t) ---

    /// Unlock the scene from reading.
    ///
    /// Each unlockRead() must be paired with a lockRead() from the same thread.
    @(link_name = "PxScene_unlockRead_mut")
    scene_unlock_read_mut :: proc(self_: ^Scene) ---

    /// Lock the scene for writing from this thread.
    ///
    /// When the PxSceneFlag::eREQUIRE_RW_LOCK flag is enabled lockWrite() must be
    /// called before any write calls are made on the scene.
    ///
    /// Only one thread may write at a time and no threads may read while a thread is writing.
    /// If a call to lockWrite() is made and there are other threads reading then the
    /// calling thread will be blocked until the readers complete.
    ///
    /// Writers have priority. If a thread is blocked waiting to write then subsequent calls to
    /// lockRead() from other threads will be blocked until the writer completes.
    ///
    /// If multiple threads are waiting to write then the thread that is first
    /// granted access depends on OS scheduling.
    ///
    /// Recursive locking is supported but each lockWrite() call must be paired
    /// with an unlockWrite().
    ///
    /// If a thread has already locked the scene for writing then it may call
    /// lockRead().
    @(link_name = "PxScene_lockWrite_mut")
    scene_lock_write_mut :: proc(self_: ^Scene, file: cstring, line: _c.uint32_t) ---

    /// Unlock the scene from writing.
    ///
    /// Each unlockWrite() must be paired with a lockWrite() from the same thread.
    @(link_name = "PxScene_unlockWrite_mut")
    scene_unlock_write_mut :: proc(self_: ^Scene) ---

    /// set the cache blocks that can be used during simulate().
    ///
    /// Each frame the simulation requires memory to store contact, friction, and contact cache data. This memory is used in blocks of 16K.
    /// Each frame the blocks used by the previous frame are freed, and may be retrieved by the application using PxScene::flushSimulation()
    ///
    /// This call will force allocation of cache blocks if the numBlocks parameter is greater than the currently allocated number
    /// of blocks, and less than the max16KContactDataBlocks parameter specified at scene creation time.
    ///
    /// Do not use this method while the simulation is running.
    @(link_name = "PxScene_setNbContactDataBlocks_mut")
    scene_set_nb_contact_data_blocks_mut :: proc(self_: ^Scene, numBlocks: _c.uint32_t) ---

    /// get the number of cache blocks currently used by the scene
    ///
    /// This function may not be called while the scene is simulating
    ///
    /// the number of cache blocks currently used by the scene
    @(link_name = "PxScene_getNbContactDataBlocksUsed")
    scene_get_nb_contact_data_blocks_used :: proc(self_: ^Scene) -> _c.uint32_t ---

    /// get the maximum number of cache blocks used by the scene
    ///
    /// This function may not be called while the scene is simulating
    ///
    /// the maximum number of cache blocks everused by the scene
    @(link_name = "PxScene_getMaxNbContactDataBlocksUsed")
    scene_get_max_nb_contact_data_blocks_used :: proc(self_: ^Scene) -> _c.uint32_t ---

    /// Return the value of PxSceneDesc::contactReportStreamBufferSize that was set when creating the scene with PxPhysics::createScene
    @(link_name = "PxScene_getContactReportStreamBufferSize")
    scene_get_contact_report_stream_buffer_size :: proc(self_: ^Scene) -> _c.uint32_t ---

    /// Sets the number of actors required to spawn a separate rigid body solver thread.
    ///
    /// Do not use this method while the simulation is running.
    @(link_name = "PxScene_setSolverBatchSize_mut")
    scene_set_solver_batch_size_mut :: proc(self_: ^Scene, solverBatchSize: _c.uint32_t) ---

    /// Retrieves the number of actors required to spawn a separate rigid body solver thread.
    ///
    /// Current number of actors required to spawn a separate rigid body solver thread.
    @(link_name = "PxScene_getSolverBatchSize")
    scene_get_solver_batch_size :: proc(self_: ^Scene) -> _c.uint32_t ---

    /// Sets the number of articulations required to spawn a separate rigid body solver thread.
    ///
    /// Do not use this method while the simulation is running.
    @(link_name = "PxScene_setSolverArticulationBatchSize_mut")
    scene_set_solver_articulation_batch_size_mut :: proc(self_: ^Scene, solverBatchSize: _c.uint32_t) ---

    /// Retrieves the number of articulations required to spawn a separate rigid body solver thread.
    ///
    /// Current number of articulations required to spawn a separate rigid body solver thread.
    @(link_name = "PxScene_getSolverArticulationBatchSize")
    scene_get_solver_articulation_batch_size :: proc(self_: ^Scene) -> _c.uint32_t ---

    /// Returns the wake counter reset value.
    ///
    /// Wake counter reset value
    @(link_name = "PxScene_getWakeCounterResetValue")
    scene_get_wake_counter_reset_value :: proc(self_: ^Scene) -> _c.float ---

    /// Shift the scene origin by the specified vector.
    ///
    /// The poses of all objects in the scene and the corresponding data structures will get adjusted to reflect the new origin location
    /// (the shift vector will get subtracted from all object positions).
    ///
    /// It is the user's responsibility to keep track of the summed total origin shift and adjust all input/output to/from PhysX accordingly.
    ///
    /// Do not use this method while the simulation is running. Calls to this method while the simulation is running will be ignored.
    ///
    /// Make sure to propagate the origin shift to other dependent modules (for example, the character controller module etc.).
    ///
    /// This is an expensive operation and we recommend to use it only in the case where distance related precision issues may arise in areas far from the origin.
    @(link_name = "PxScene_shiftOrigin_mut")
    scene_shift_origin_mut :: proc(self_: ^Scene, #by_ptr shift: Vec3) ---

    /// Returns the Pvd client associated with the scene.
    ///
    /// the client, NULL if no PVD supported.
    @(link_name = "PxScene_getScenePvdClient_mut")
    scene_get_scene_pvd_client_mut :: proc(self_: ^Scene) -> ^PvdSceneClient ---

    /// Copy GPU articulation data from the internal GPU buffer to a user-provided device buffer.
    @(link_name = "PxScene_copyArticulationData_mut")
    scene_copy_articulation_data_mut :: proc(self_: ^Scene, data: rawptr, index: rawptr, dataType: ArticulationGpuDataType, nbCopyArticulations: _c.uint32_t, copyEvent: rawptr) ---

    /// Apply GPU articulation data from a user-provided device buffer to the internal GPU buffer.
    @(link_name = "PxScene_applyArticulationData_mut")
    scene_apply_articulation_data_mut :: proc(self_: ^Scene, data: rawptr, index: rawptr, dataType: ArticulationGpuDataType, nbUpdatedArticulations: _c.uint32_t, waitEvent: rawptr, signalEvent: rawptr) ---

    /// Copy GPU softbody data from the internal GPU buffer to a user-provided device buffer.
    @(link_name = "PxScene_copySoftBodyData_mut")
    scene_copy_soft_body_data_mut :: proc(self_: ^Scene, data: [^]rawptr, dataSizes: rawptr, softBodyIndices: rawptr, flag: SoftBodyDataFlag, nbCopySoftBodies: _c.uint32_t, maxSize: _c.uint32_t, copyEvent: rawptr) ---

    /// Apply user-provided data to the internal softbody system.
    @(link_name = "PxScene_applySoftBodyData_mut")
    scene_apply_soft_body_data_mut :: proc(self_: ^Scene, data: [^]rawptr, dataSizes: rawptr, softBodyIndices: rawptr, flag: SoftBodyDataFlag, nbUpdatedSoftBodies: _c.uint32_t, maxSize: _c.uint32_t, applyEvent: rawptr) ---

    /// Copy contact data from the internal GPU buffer to a user-provided device buffer.
    ///
    /// The contact data contains pointers to internal state and is only valid until the next call to simulate().
    @(link_name = "PxScene_copyContactData_mut")
    scene_copy_contact_data_mut :: proc(self_: ^Scene, data: rawptr, maxContactPairs: _c.uint32_t, numContactPairs: rawptr, copyEvent: rawptr) ---

    /// Copy GPU rigid body data from the internal GPU buffer to a user-provided device buffer.
    @(link_name = "PxScene_copyBodyData_mut")
    scene_copy_body_data_mut :: proc(self_: ^Scene, data: ^GpuBodyData, index: ^GpuActorPair, nbCopyActors: _c.uint32_t, copyEvent: rawptr) ---

    /// Apply user-provided data to rigid body.
    @(link_name = "PxScene_applyActorData_mut")
    scene_apply_actor_data_mut :: proc(self_: ^Scene, data: rawptr, index: ^GpuActorPair, flag: ActorCacheFlag, nbUpdatedActors: _c.uint32_t, waitEvent: rawptr, signalEvent: rawptr) ---

    /// Compute dense Jacobian matrices for specified articulations on the GPU.
    ///
    /// The size of Jacobians can vary by articulation, since it depends on the number of links, degrees-of-freedom, and whether the base is fixed.
    ///
    /// The size is determined using these formulas:
    /// nCols = (fixedBase ? 0 : 6) + dofCount
    /// nRows = (fixedBase ? 0 : 6) + (linkCount - 1) * 6;
    ///
    /// The user must ensure that adequate space is provided for each Jacobian matrix.
    @(link_name = "PxScene_computeDenseJacobians_mut")
    scene_compute_dense_jacobians_mut :: proc(self_: ^Scene, indices: ^IndexDataPair, nbIndices: _c.uint32_t, computeEvent: rawptr) ---

    /// Compute the joint-space inertia matrices that maps joint accelerations to joint forces: forces = M * accelerations on the GPU.
    ///
    /// The size of matrices can vary by articulation, since it depends on the number of links and degrees-of-freedom.
    ///
    /// The size is determined using this formula:
    /// sizeof(float) * dofCount * dofCount
    ///
    /// The user must ensure that adequate space is provided for each mass matrix.
    @(link_name = "PxScene_computeGeneralizedMassMatrices_mut")
    scene_compute_generalized_mass_matrices_mut :: proc(self_: ^Scene, indices: ^IndexDataPair, nbIndices: _c.uint32_t, computeEvent: rawptr) ---

    /// Computes the joint DOF forces required to counteract gravitational forces for the given articulation pose.
    ///
    /// The size of the result can vary by articulation, since it depends on the number of links and degrees-of-freedom.
    ///
    /// The size is determined using this formula:
    /// sizeof(float) * dofCount
    ///
    /// The user must ensure that adequate space is provided for each articulation.
    @(link_name = "PxScene_computeGeneralizedGravityForces_mut")
    scene_compute_generalized_gravity_forces_mut :: proc(self_: ^Scene, indices: ^IndexDataPair, nbIndices: _c.uint32_t, computeEvent: rawptr) ---

    /// Computes the joint DOF forces required to counteract coriolis and centrifugal forces for the given articulation pose.
    ///
    /// The size of the result can vary by articulation, since it depends on the number of links and degrees-of-freedom.
    ///
    /// The size is determined using this formula:
    /// sizeof(float) * dofCount
    ///
    /// The user must ensure that adequate space is provided for each articulation.
    @(link_name = "PxScene_computeCoriolisAndCentrifugalForces_mut")
    scene_compute_coriolis_and_centrifugal_forces_mut :: proc(self_: ^Scene, indices: ^IndexDataPair, nbIndices: _c.uint32_t, computeEvent: rawptr) ---

    @(link_name = "PxScene_getGpuDynamicsConfig")
    scene_get_gpu_dynamics_config :: proc(self_: ^Scene) -> gDynamicsMemoryConfig ---

    /// Apply user-provided data to particle buffers.
    ///
    /// This function should be used if the particle buffer flags are already on the device. Otherwise, use PxParticleBuffer::raiseFlags()
    /// from the CPU.
    ///
    /// This assumes the data has been changed directly in the PxParticleBuffer.
    @(link_name = "PxScene_applyParticleBufferData_mut")
    scene_apply_particle_buffer_data_mut :: proc(self_: ^Scene, indices: ^_c.uint32_t, bufferIndexPair: ^GpuParticleBufferIndexPair, flags: ^ParticleBufferFlags_Set, nbUpdatedBuffers: _c.uint32_t, waitEvent: rawptr, signalEvent: rawptr) ---

    /// Constructor
    @(link_name = "PxSceneReadLock_new_alloc")
    scene_read_lock_new_alloc :: proc(scene: ^Scene, file: cstring, line: _c.uint32_t) -> ^SceneReadLock ---

    @(link_name = "PxSceneReadLock_delete")
    scene_read_lock_delete :: proc(self_: ^SceneReadLock) ---

    /// Constructor
    @(link_name = "PxSceneWriteLock_new_alloc")
    scene_write_lock_new_alloc :: proc(scene: ^Scene, file: cstring, line: _c.uint32_t) -> ^SceneWriteLock ---

    @(link_name = "PxSceneWriteLock_delete")
    scene_write_lock_delete :: proc(self_: ^SceneWriteLock) ---

    @(link_name = "PxContactPairExtraDataItem_new")
    contact_pair_extra_data_item_new :: proc() -> ContactPairExtraDataItem ---

    @(link_name = "PxContactPairVelocity_new")
    contact_pair_velocity_new :: proc() -> ContactPairVelocity ---

    @(link_name = "PxContactPairPose_new")
    contact_pair_pose_new :: proc() -> ContactPairPose ---

    @(link_name = "PxContactPairIndex_new")
    contact_pair_index_new :: proc() -> ContactPairIndex ---

    /// Constructor
    @(link_name = "PxContactPairExtraDataIterator_new")
    contact_pair_extra_data_iterator_new :: proc(stream: ^_c.uint8_t, size: _c.uint32_t) -> ContactPairExtraDataIterator ---

    /// Advances the iterator to next set of extra data items.
    ///
    /// The contact pair extra data stream contains sets of items as requested by the corresponding [`PxPairFlag`] flags
    /// [`PxPairFlag::ePRE_SOLVER_VELOCITY`], #PxPairFlag::ePOST_SOLVER_VELOCITY, #PxPairFlag::eCONTACT_EVENT_POSE. A set can contain one
    /// item of each plus the PxContactPairIndex item. This method parses the stream and points the iterator
    /// member variables to the corresponding items of the current set, if they are available. If CCD is not enabled,
    /// you should only get one set of items. If CCD with multiple passes is enabled, you might get more than one item
    /// set.
    ///
    /// Even though contact pair extra data is requested per shape pair, you will not get an item set per shape pair
    /// but one per actor pair. If, for example, an actor has two shapes and both collide with another actor, then
    /// there will only be one item set (since it applies to both shape pairs).
    ///
    /// True if there was another set of extra data items in the stream, else false.
    @(link_name = "PxContactPairExtraDataIterator_nextItemSet_mut")
    contact_pair_extra_data_iterator_next_item_set_mut :: proc(self_: ^ContactPairExtraDataIterator) -> _c.bool ---

    @(link_name = "PxContactPairHeader_new")
    contact_pair_header_new :: proc() -> ContactPairHeader ---

    @(link_name = "PxContactPair_new")
    contact_pair_new :: proc() -> ContactPair ---

    /// Extracts the contact points from the stream and stores them in a convenient format.
    ///
    /// Number of contact points written to the buffer.
    @(link_name = "PxContactPair_extractContacts")
    contact_pair_extract_contacts :: proc(self_: ^ContactPair, userBuffer: ^ContactPairPoint, bufferSize: _c.uint32_t) -> _c.uint32_t ---

    /// Helper method to clone the contact pair and copy the contact data stream into a user buffer.
    ///
    /// The contact data stream is only accessible during the contact report callback. This helper function provides copy functionality
    /// to buffer the contact stream information such that it can get accessed at a later stage.
    @(link_name = "PxContactPair_bufferContacts")
    contact_pair_buffer_contacts :: proc(self_: ^ContactPair, newPair: ^ContactPair, bufferMemory: ^_c.uint8_t) ---

    @(link_name = "PxContactPair_getInternalFaceIndices")
    contact_pair_get_internal_face_indices :: proc(self_: ^ContactPair) -> [^]_c.uint32_t ---

    @(link_name = "PxTriggerPair_new")
    trigger_pair_new :: proc() -> TriggerPair ---

    @(link_name = "PxConstraintInfo_new")
    constraint_info_new :: proc() -> ConstraintInfo ---

    @(link_name = "PxConstraintInfo_new_1")
    constraint_info_new_1 :: proc(c: ^Constraint, extRef: rawptr, t: _c.uint32_t) -> ConstraintInfo ---

    /// This is called when a breakable constraint breaks.
    ///
    /// The user should not release the constraint shader inside this call!
    ///
    /// No event will get reported if the constraint breaks but gets deleted while the time step is still being simulated.
    @(link_name = "PxSimulationEventCallback_onConstraintBreak_mut")
    simulation_event_callback_on_constraint_break_mut :: proc(self_: ^SimulationEventCallback, constraints: ^ConstraintInfo, count: _c.uint32_t) ---

    /// This is called with the actors which have just been woken up.
    ///
    /// Only supported by rigid bodies yet.
    ///
    /// Only called on actors for which the PxActorFlag eSEND_SLEEP_NOTIFIES has been set.
    ///
    /// Only the latest sleep state transition happening between fetchResults() of the previous frame and fetchResults() of the current frame
    /// will get reported. For example, let us assume actor A is awake, then A->putToSleep() gets called, then later A->wakeUp() gets called.
    /// At the next simulate/fetchResults() step only an onWake() event will get triggered because that was the last transition.
    ///
    /// If an actor gets newly added to a scene with properties such that it is awake and the sleep state does not get changed by
    /// the user or simulation, then an onWake() event will get sent at the next simulate/fetchResults() step.
    @(link_name = "PxSimulationEventCallback_onWake_mut")
    simulation_event_callback_on_wake_mut :: proc(self_: ^SimulationEventCallback, actors: [^]^Actor, count: _c.uint32_t) ---

    /// This is called with the actors which have just been put to sleep.
    ///
    /// Only supported by rigid bodies yet.
    ///
    /// Only called on actors for which the PxActorFlag eSEND_SLEEP_NOTIFIES has been set.
    ///
    /// Only the latest sleep state transition happening between fetchResults() of the previous frame and fetchResults() of the current frame
    /// will get reported. For example, let us assume actor A is asleep, then A->wakeUp() gets called, then later A->putToSleep() gets called.
    /// At the next simulate/fetchResults() step only an onSleep() event will get triggered because that was the last transition (assuming the simulation
    /// does not wake the actor up).
    ///
    /// If an actor gets newly added to a scene with properties such that it is asleep and the sleep state does not get changed by
    /// the user or simulation, then an onSleep() event will get sent at the next simulate/fetchResults() step.
    @(link_name = "PxSimulationEventCallback_onSleep_mut")
    simulation_event_callback_on_sleep_mut :: proc(self_: ^SimulationEventCallback, actors: [^]^Actor, count: _c.uint32_t) ---

    /// This is called when certain contact events occur.
    ///
    /// The method will be called for a pair of actors if one of the colliding shape pairs requested contact notification.
    /// You request which events are reported using the filter shader/callback mechanism (see [`PxSimulationFilterShader`],
    /// [`PxSimulationFilterCallback`], #PxPairFlag).
    ///
    /// Do not keep references to the passed objects, as they will be
    /// invalid after this function returns.
    @(link_name = "PxSimulationEventCallback_onContact_mut")
    simulation_event_callback_on_contact_mut :: proc(self_: ^SimulationEventCallback, #by_ptr pairHeader: ContactPairHeader, pairs: ^ContactPair, nbPairs: _c.uint32_t) ---

    /// This is called with the current trigger pair events.
    ///
    /// Shapes which have been marked as triggers using PxShapeFlag::eTRIGGER_SHAPE will send events
    /// according to the pair flag specification in the filter shader (see [`PxPairFlag`], #PxSimulationFilterShader).
    ///
    /// Trigger shapes will no longer send notification events for interactions with other trigger shapes.
    @(link_name = "PxSimulationEventCallback_onTrigger_mut")
    simulation_event_callback_on_trigger_mut :: proc(self_: ^SimulationEventCallback, pairs: ^TriggerPair, count: _c.uint32_t) ---

    /// Provides early access to the new pose of moving rigid bodies.
    ///
    /// When this call occurs, rigid bodies having the [`PxRigidBodyFlag::eENABLE_POSE_INTEGRATION_PREVIEW`]
    /// flag set, were moved by the simulation and their new poses can be accessed through the provided buffers.
    ///
    /// The provided buffers are valid and can be read until the next call to [`PxScene::simulate`]() or #PxScene::collide().
    ///
    /// This callback gets triggered while the simulation is running. If the provided rigid body references are used to
    /// read properties of the object, then the callback has to guarantee no other thread is writing to the same body at the same
    /// time.
    ///
    /// The code in this callback should be lightweight as it can block the simulation, that is, the
    /// [`PxScene::fetchResults`]() call.
    @(link_name = "PxSimulationEventCallback_onAdvance_mut")
    simulation_event_callback_on_advance_mut :: proc(self_: ^SimulationEventCallback, bodyBuffer: [^]^RigidBody, poseBuffer: ^Transform, count: _c.uint32_t) ---

    @(link_name = "PxSimulationEventCallback_delete")
    simulation_event_callback_delete :: proc(self_: ^SimulationEventCallback) ---

    @(link_name = "PxFEMParameters_new")
    f_e_m_parameters_new :: proc() -> FEMParameters ---

    /// Release this object.
    @(link_name = "PxPruningStructure_release_mut")
    pruning_structure_release_mut :: proc(self_: ^PruningStructure) ---

    /// Retrieve rigid actors in the pruning structure.
    ///
    /// You can retrieve the number of rigid actor pointers by calling [`getNbRigidActors`]()
    ///
    /// Number of rigid actor pointers written to the buffer.
    @(link_name = "PxPruningStructure_getRigidActors")
    pruning_structure_get_rigid_actors :: proc(self_: ^PruningStructure, userBuffer: [^]^RigidActor, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Returns the number of rigid actors in the pruning structure.
    ///
    /// You can use [`getRigidActors`]() to retrieve the rigid actor pointers.
    ///
    /// Number of rigid actors in the pruning structure.
    @(link_name = "PxPruningStructure_getNbRigidActors")
    pruning_structure_get_nb_rigid_actors :: proc(self_: ^PruningStructure) -> _c.uint32_t ---

    /// Gets the merge data for static actors
    ///
    /// This is mainly called by the PxSceneQuerySystem::merge() function to merge a PxPruningStructure
    /// with the internal data-structures of the scene-query system.
    ///
    /// Implementation-dependent merge data for static actors.
    @(link_name = "PxPruningStructure_getStaticMergeData")
    pruning_structure_get_static_merge_data :: proc(self_: ^PruningStructure) -> rawptr ---

    /// Gets the merge data for dynamic actors
    ///
    /// This is mainly called by the PxSceneQuerySystem::merge() function to merge a PxPruningStructure
    /// with the internal data-structures of the scene-query system.
    ///
    /// Implementation-dependent merge data for dynamic actors.
    @(link_name = "PxPruningStructure_getDynamicMergeData")
    pruning_structure_get_dynamic_merge_data :: proc(self_: ^PruningStructure) -> rawptr ---

    @(link_name = "PxPruningStructure_getConcreteTypeName")
    pruning_structure_get_concrete_type_name :: proc(self_: ^PruningStructure) -> cstring ---

    @(link_name = "PxExtendedVec3_new")
    extended_vec3_new :: proc() -> ExtendedVec3 ---

    @(link_name = "PxExtendedVec3_new_1")
    extended_vec3_new_1 :: proc(_x: _c.double, _y: _c.double, _z: _c.double) -> ExtendedVec3 ---

    @(link_name = "PxExtendedVec3_isZero")
    extended_vec3_is_zero :: proc(self_: ^ExtendedVec3) -> _c.bool ---

    @(link_name = "PxExtendedVec3_dot")
    extended_vec3_dot :: proc(self_: ^ExtendedVec3, #by_ptr v: Vec3) -> _c.double ---

    @(link_name = "PxExtendedVec3_distanceSquared")
    extended_vec3_distance_squared :: proc(self_: ^ExtendedVec3, #by_ptr v: ExtendedVec3) -> _c.double ---

    @(link_name = "PxExtendedVec3_magnitudeSquared")
    extended_vec3_magnitude_squared :: proc(self_: ^ExtendedVec3) -> _c.double ---

    @(link_name = "PxExtendedVec3_magnitude")
    extended_vec3_magnitude :: proc(self_: ^ExtendedVec3) -> _c.double ---

    @(link_name = "PxExtendedVec3_normalize_mut")
    extended_vec3_normalize_mut :: proc(self_: ^ExtendedVec3) -> _c.double ---

    @(link_name = "PxExtendedVec3_isFinite")
    extended_vec3_is_finite :: proc(self_: ^ExtendedVec3) -> _c.bool ---

    @(link_name = "PxExtendedVec3_maximum_mut")
    extended_vec3_maximum_mut :: proc(self_: ^ExtendedVec3, #by_ptr v: ExtendedVec3) ---

    @(link_name = "PxExtendedVec3_minimum_mut")
    extended_vec3_minimum_mut :: proc(self_: ^ExtendedVec3, #by_ptr v: ExtendedVec3) ---

    @(link_name = "PxExtendedVec3_set_mut")
    extended_vec3_set_mut :: proc(self_: ^ExtendedVec3, x_: _c.double, y_: _c.double, z_: _c.double) ---

    @(link_name = "PxExtendedVec3_setPlusInfinity_mut")
    extended_vec3_set_plus_infinity_mut :: proc(self_: ^ExtendedVec3) ---

    @(link_name = "PxExtendedVec3_setMinusInfinity_mut")
    extended_vec3_set_minus_infinity_mut :: proc(self_: ^ExtendedVec3) ---

    @(link_name = "PxExtendedVec3_cross_mut")
    extended_vec3_cross_mut :: proc(self_: ^ExtendedVec3, #by_ptr left: ExtendedVec3, #by_ptr right: Vec3) ---

    @(link_name = "PxExtendedVec3_cross_mut_1")
    extended_vec3_cross_mut_1 :: proc(self_: ^ExtendedVec3, #by_ptr left: ExtendedVec3, #by_ptr right: ExtendedVec3) ---

    @(link_name = "PxExtendedVec3_cross")
    extended_vec3_cross :: proc(self_: ^ExtendedVec3, #by_ptr v: ExtendedVec3) -> ExtendedVec3 ---

    @(link_name = "PxExtendedVec3_cross_mut_2")
    extended_vec3_cross_mut_2 :: proc(self_: ^ExtendedVec3, #by_ptr left: Vec3, #by_ptr right: ExtendedVec3) ---

    @(link_name = "phys_toVec3")
    to_vec3 :: proc(#by_ptr v: ExtendedVec3) -> Vec3 ---

    @(link_name = "PxObstacle_getType")
    obstacle_get_type :: proc(self_: ^Obstacle) -> GeometryType ---

    @(link_name = "PxBoxObstacle_new")
    box_obstacle_new :: proc() -> BoxObstacle ---

    @(link_name = "PxCapsuleObstacle_new")
    capsule_obstacle_new :: proc() -> CapsuleObstacle ---

    /// Releases the context.
    @(link_name = "PxObstacleContext_release_mut")
    obstacle_context_release_mut :: proc(self_: ^ObstacleContext) ---

    /// Retrieves the controller manager associated with this context.
    ///
    /// The associated controller manager
    @(link_name = "PxObstacleContext_getControllerManager")
    obstacle_context_get_controller_manager :: proc(self_: ^ObstacleContext) -> ^ControllerManager ---

    /// Adds an obstacle to the context.
    ///
    /// Handle for newly-added obstacle
    @(link_name = "PxObstacleContext_addObstacle_mut")
    obstacle_context_add_obstacle_mut :: proc(self_: ^ObstacleContext, obstacle: ^Obstacle) -> _c.uint32_t ---

    /// Removes an obstacle from the context.
    ///
    /// True if success
    @(link_name = "PxObstacleContext_removeObstacle_mut")
    obstacle_context_remove_obstacle_mut :: proc(self_: ^ObstacleContext, handle: _c.uint32_t) -> _c.bool ---

    /// Updates data for an existing obstacle.
    ///
    /// True if success
    @(link_name = "PxObstacleContext_updateObstacle_mut")
    obstacle_context_update_obstacle_mut :: proc(self_: ^ObstacleContext, handle: _c.uint32_t, obstacle: ^Obstacle) -> _c.bool ---

    /// Retrieves number of obstacles in the context.
    ///
    /// Number of obstacles in the context
    @(link_name = "PxObstacleContext_getNbObstacles")
    obstacle_context_get_nb_obstacles :: proc(self_: ^ObstacleContext) -> _c.uint32_t ---

    /// Retrieves desired obstacle.
    ///
    /// Desired obstacle
    @(link_name = "PxObstacleContext_getObstacle")
    obstacle_context_get_obstacle :: proc(self_: ^ObstacleContext, i: _c.uint32_t) -> ^Obstacle ---

    /// Retrieves desired obstacle by given handle.
    ///
    /// Desired obstacle
    @(link_name = "PxObstacleContext_getObstacleByHandle")
    obstacle_context_get_obstacle_by_handle :: proc(self_: ^ObstacleContext, handle: _c.uint32_t) -> ^Obstacle ---

    /// Called when current controller hits a shape.
    ///
    /// This is called when the CCT moves and hits a shape. This will not be called when a moving shape hits a non-moving CCT.
    @(link_name = "PxUserControllerHitReport_onShapeHit_mut")
    user_controller_hit_report_on_shape_hit_mut :: proc(self_: ^UserControllerHitReport, #by_ptr hit: ControllerShapeHit) ---

    /// Called when current controller hits another controller.
    @(link_name = "PxUserControllerHitReport_onControllerHit_mut")
    user_controller_hit_report_on_controller_hit_mut :: proc(self_: ^UserControllerHitReport, #by_ptr hit: ControllersHit) ---

    /// Called when current controller hits a user-defined obstacle.
    @(link_name = "PxUserControllerHitReport_onObstacleHit_mut")
    user_controller_hit_report_on_obstacle_hit_mut :: proc(self_: ^UserControllerHitReport, #by_ptr hit: ControllerObstacleHit) ---

    @(link_name = "PxControllerFilterCallback_delete")
    controller_filter_callback_delete :: proc(self_: ^ControllerFilterCallback) ---

    /// Filtering method for CCT-vs-CCT.
    ///
    /// true to keep the pair, false to filter it out
    @(link_name = "PxControllerFilterCallback_filter_mut")
    controller_filter_callback_filter_mut :: proc(self_: ^ControllerFilterCallback, a: ^Controller, b: ^Controller) -> _c.bool ---

    @(link_name = "PxControllerFilters_new")
    controller_filters_new :: proc(filterData: ^FilterData, cb: ^QueryFilterCallback, cctFilterCb: ^ControllerFilterCallback) -> ControllerFilters ---

    /// returns true if the current settings are valid
    ///
    /// True if the descriptor is valid.
    @(link_name = "PxControllerDesc_isValid")
    controller_desc_is_valid :: proc(self_: ^ControllerDesc) -> _c.bool ---

    /// Returns the character controller type
    ///
    /// The controllers type.
    @(link_name = "PxControllerDesc_getType")
    controller_desc_get_type :: proc(self_: ^ControllerDesc) -> ControllerShapeType ---

    /// Return the type of controller
    @(link_name = "PxController_getType")
    controller_get_type :: proc(self_: ^Controller) -> ControllerShapeType ---

    /// Releases the controller.
    @(link_name = "PxController_release_mut")
    controller_release_mut :: proc(self_: ^Controller) ---

    /// Moves the character using a "collide-and-slide" algorithm.
    ///
    /// Collision flags, collection of ::PxControllerCollisionFlags
    @(link_name = "PxController_move_mut")
    controller_move_mut :: proc(self_: ^Controller, #by_ptr disp: Vec3, minDist: _c.float, elapsedTime: _c.float, #by_ptr filters: ControllerFilters, obstacles: ^ObstacleContext) -> ControllerCollisionFlags_Set ---

    /// Sets controller's position.
    ///
    /// The position controlled by this function is the center of the collision shape.
    ///
    /// This is a 'teleport' function, it doesn't check for collisions.
    ///
    /// The character's position must be such that it does not overlap the static geometry.
    ///
    /// To move the character under normal conditions use the [`move`]() function.
    ///
    /// Currently always returns true.
    @(link_name = "PxController_setPosition_mut")
    controller_set_position_mut :: proc(self_: ^Controller, #by_ptr position: ExtendedVec3) -> _c.bool ---

    /// Retrieve the raw position of the controller.
    ///
    /// The position retrieved by this function is the center of the collision shape. To retrieve the bottom position of the shape,
    /// a.k.a. the foot position, use the getFootPosition() function.
    ///
    /// The position is updated by calls to move(). Calling this method without calling
    /// move() will return the last position or the initial position of the controller.
    ///
    /// The controller's center position
    @(link_name = "PxController_getPosition")
    controller_get_position :: proc(self_: ^Controller) -> ^ExtendedVec3 ---

    /// Set controller's foot position.
    ///
    /// The position controlled by this function is the bottom of the collision shape, a.k.a. the foot position.
    ///
    /// The foot position takes the contact offset into account
    ///
    /// This is a 'teleport' function, it doesn't check for collisions.
    ///
    /// To move the character under normal conditions use the [`move`]() function.
    ///
    /// Currently always returns true.
    @(link_name = "PxController_setFootPosition_mut")
    controller_set_foot_position_mut :: proc(self_: ^Controller, #by_ptr position: ExtendedVec3) -> _c.bool ---

    /// Retrieve the "foot" position of the controller, i.e. the position of the bottom of the CCT's shape.
    ///
    /// The foot position takes the contact offset into account
    ///
    /// The controller's foot position
    @(link_name = "PxController_getFootPosition")
    controller_get_foot_position :: proc(self_: ^Controller) -> ExtendedVec3 ---

    /// Get the rigid body actor associated with this controller (see PhysX documentation).
    /// The behavior upon manually altering this actor is undefined, you should primarily
    /// use it for reading const properties.
    ///
    /// the actor associated with the controller.
    @(link_name = "PxController_getActor")
    controller_get_actor :: proc(self_: ^Controller) -> ^RigidDynamic ---

    /// The step height.
    @(link_name = "PxController_setStepOffset_mut")
    controller_set_step_offset_mut :: proc(self_: ^Controller, offset: _c.float) ---

    /// Retrieve the step height.
    ///
    /// The step offset for the controller.
    @(link_name = "PxController_getStepOffset")
    controller_get_step_offset :: proc(self_: ^Controller) -> _c.float ---

    /// Sets the non-walkable mode for the CCT.
    @(link_name = "PxController_setNonWalkableMode_mut")
    controller_set_non_walkable_mode_mut :: proc(self_: ^Controller, flag: ControllerNonWalkableMode) ---

    /// Retrieves the non-walkable mode for the CCT.
    ///
    /// The current non-walkable mode.
    @(link_name = "PxController_getNonWalkableMode")
    controller_get_non_walkable_mode :: proc(self_: ^Controller) -> ControllerNonWalkableMode ---

    /// Retrieve the contact offset.
    ///
    /// The contact offset for the controller.
    @(link_name = "PxController_getContactOffset")
    controller_get_contact_offset :: proc(self_: ^Controller) -> _c.float ---

    /// Sets the contact offset.
    @(link_name = "PxController_setContactOffset_mut")
    controller_set_contact_offset_mut :: proc(self_: ^Controller, offset: _c.float) ---

    /// Retrieve the 'up' direction.
    ///
    /// The up direction for the controller.
    @(link_name = "PxController_getUpDirection")
    controller_get_up_direction :: proc(self_: ^Controller) -> Vec3 ---

    /// Sets the 'up' direction.
    @(link_name = "PxController_setUpDirection_mut")
    controller_set_up_direction_mut :: proc(self_: ^Controller, #by_ptr up: Vec3) ---

    /// Retrieve the slope limit.
    ///
    /// The slope limit for the controller.
    @(link_name = "PxController_getSlopeLimit")
    controller_get_slope_limit :: proc(self_: ^Controller) -> _c.float ---

    /// Sets the slope limit.
    ///
    /// This feature can not be enabled at runtime, i.e. if the slope limit is zero when creating the CCT
    /// (which disables the feature) then changing the slope limit at runtime will not have any effect, and the call
    /// will be ignored.
    @(link_name = "PxController_setSlopeLimit_mut")
    controller_set_slope_limit_mut :: proc(self_: ^Controller, slopeLimit: _c.float) ---

    /// Flushes internal geometry cache.
    ///
    /// The character controller uses caching in order to speed up collision testing. The cache is
    /// automatically flushed when a change to static objects is detected in the scene. For example when a
    /// static shape is added, updated, or removed from the scene, the cache is automatically invalidated.
    ///
    /// However there may be situations that cannot be automatically detected, and those require manual
    /// invalidation of the cache. Currently the user must call this when the filtering behavior changes (the
    /// PxControllerFilters parameter of the PxController::move call).  While the controller in principle
    /// could detect a change in these parameters, it cannot detect a change in the behavior of the filtering
    /// function.
    @(link_name = "PxController_invalidateCache_mut")
    controller_invalidate_cache_mut :: proc(self_: ^Controller) ---

    /// Retrieve the scene associated with the controller.
    ///
    /// The physics scene
    @(link_name = "PxController_getScene_mut")
    controller_get_scene_mut :: proc(self_: ^Controller) -> ^Scene ---

    /// Returns the user data associated with this controller.
    ///
    /// The user pointer associated with the controller.
    @(link_name = "PxController_getUserData")
    controller_get_user_data :: proc(self_: ^Controller) -> rawptr ---

    /// Sets the user data associated with this controller.
    @(link_name = "PxController_setUserData_mut")
    controller_set_user_data_mut :: proc(self_: ^Controller, userData: rawptr) ---

    /// Returns information about the controller's internal state.
    @(link_name = "PxController_getState")
    controller_get_state :: proc(self_: ^Controller, state: ^ControllerState) ---

    /// Returns the controller's internal statistics.
    @(link_name = "PxController_getStats")
    controller_get_stats :: proc(self_: ^Controller, stats: ^ControllerStats) ---

    /// Resizes the controller.
    ///
    /// This function attempts to resize the controller to a given size, while making sure the bottom
    /// position of the controller remains constant. In other words the function modifies both the
    /// height and the (center) position of the controller. This is a helper function that can be used
    /// to implement a 'crouch' functionality for example.
    @(link_name = "PxController_resize_mut")
    controller_resize_mut :: proc(self_: ^Controller, height: _c.float) ---

    /// constructor sets to default.
    @(link_name = "PxBoxControllerDesc_new_alloc")
    box_controller_desc_new_alloc :: proc() -> ^BoxControllerDesc ---

    @(link_name = "PxBoxControllerDesc_delete")
    box_controller_desc_delete :: proc(self_: ^BoxControllerDesc) ---

    /// (re)sets the structure to the default.
    @(link_name = "PxBoxControllerDesc_setToDefault_mut")
    box_controller_desc_set_to_default_mut :: proc(self_: ^BoxControllerDesc) ---

    /// returns true if the current settings are valid
    ///
    /// True if the descriptor is valid.
    @(link_name = "PxBoxControllerDesc_isValid")
    box_controller_desc_is_valid :: proc(self_: ^BoxControllerDesc) -> _c.bool ---

    /// Gets controller's half height.
    ///
    /// The half height of the controller.
    @(link_name = "PxBoxController_getHalfHeight")
    box_controller_get_half_height :: proc(self_: ^BoxController) -> _c.float ---

    /// Gets controller's half side extent.
    ///
    /// The half side extent of the controller.
    @(link_name = "PxBoxController_getHalfSideExtent")
    box_controller_get_half_side_extent :: proc(self_: ^BoxController) -> _c.float ---

    /// Gets controller's half forward extent.
    ///
    /// The half forward extent of the controller.
    @(link_name = "PxBoxController_getHalfForwardExtent")
    box_controller_get_half_forward_extent :: proc(self_: ^BoxController) -> _c.float ---

    /// Sets controller's half height.
    ///
    /// this doesn't check for collisions.
    ///
    /// Currently always true.
    @(link_name = "PxBoxController_setHalfHeight_mut")
    box_controller_set_half_height_mut :: proc(self_: ^BoxController, halfHeight: _c.float) -> _c.bool ---

    /// Sets controller's half side extent.
    ///
    /// this doesn't check for collisions.
    ///
    /// Currently always true.
    @(link_name = "PxBoxController_setHalfSideExtent_mut")
    box_controller_set_half_side_extent_mut :: proc(self_: ^BoxController, halfSideExtent: _c.float) -> _c.bool ---

    /// Sets controller's half forward extent.
    ///
    /// this doesn't check for collisions.
    ///
    /// Currently always true.
    @(link_name = "PxBoxController_setHalfForwardExtent_mut")
    box_controller_set_half_forward_extent_mut :: proc(self_: ^BoxController, halfForwardExtent: _c.float) -> _c.bool ---

    /// constructor sets to default.
    @(link_name = "PxCapsuleControllerDesc_new_alloc")
    capsule_controller_desc_new_alloc :: proc() -> ^CapsuleControllerDesc ---

    @(link_name = "PxCapsuleControllerDesc_delete")
    capsule_controller_desc_delete :: proc(self_: ^CapsuleControllerDesc) ---

    /// (re)sets the structure to the default.
    @(link_name = "PxCapsuleControllerDesc_setToDefault_mut")
    capsule_controller_desc_set_to_default_mut :: proc(self_: ^CapsuleControllerDesc) ---

    /// returns true if the current settings are valid
    ///
    /// True if the descriptor is valid.
    @(link_name = "PxCapsuleControllerDesc_isValid")
    capsule_controller_desc_is_valid :: proc(self_: ^CapsuleControllerDesc) -> _c.bool ---

    /// Gets controller's radius.
    ///
    /// The radius of the controller.
    @(link_name = "PxCapsuleController_getRadius")
    capsule_controller_get_radius :: proc(self_: ^CapsuleController) -> _c.float ---

    /// Sets controller's radius.
    ///
    /// this doesn't check for collisions.
    ///
    /// Currently always true.
    @(link_name = "PxCapsuleController_setRadius_mut")
    capsule_controller_set_radius_mut :: proc(self_: ^CapsuleController, radius: _c.float) -> _c.bool ---

    /// Gets controller's height.
    ///
    /// The height of the capsule controller.
    @(link_name = "PxCapsuleController_getHeight")
    capsule_controller_get_height :: proc(self_: ^CapsuleController) -> _c.float ---

    /// Resets controller's height.
    ///
    /// this doesn't check for collisions.
    ///
    /// Currently always true.
    @(link_name = "PxCapsuleController_setHeight_mut")
    capsule_controller_set_height_mut :: proc(self_: ^CapsuleController, height: _c.float) -> _c.bool ---

    /// Gets controller's climbing mode.
    ///
    /// The capsule controller's climbing mode.
    @(link_name = "PxCapsuleController_getClimbingMode")
    capsule_controller_get_climbing_mode :: proc(self_: ^CapsuleController) -> CapsuleClimbingMode ---

    /// Sets controller's climbing mode.
    @(link_name = "PxCapsuleController_setClimbingMode_mut")
    capsule_controller_set_climbing_mode_mut :: proc(self_: ^CapsuleController, mode: CapsuleClimbingMode) -> _c.bool ---

    /// Retrieve behavior flags for a shape.
    ///
    /// When the CCT touches a shape, the CCT's behavior w.r.t. this shape can be customized by users.
    /// This function retrieves the desired PxControllerBehaviorFlag flags capturing the desired behavior.
    ///
    /// See comments about deprecated functions at the start of this class
    ///
    /// Desired behavior flags for the given shape
    @(link_name = "PxControllerBehaviorCallback_getBehaviorFlags_mut")
    controller_behavior_callback_get_behavior_flags_mut :: proc(self_: ^ControllerBehaviorCallback, #by_ptr shape: Shape, actor: ^Actor) -> ControllerBehaviorFlags_Set ---

    /// Retrieve behavior flags for a controller.
    ///
    /// When the CCT touches a controller, the CCT's behavior w.r.t. this controller can be customized by users.
    /// This function retrieves the desired PxControllerBehaviorFlag flags capturing the desired behavior.
    ///
    /// The flag PxControllerBehaviorFlag::eCCT_CAN_RIDE_ON_OBJECT is not supported.
    ///
    /// See comments about deprecated functions at the start of this class
    ///
    /// Desired behavior flags for the given controller
    @(link_name = "PxControllerBehaviorCallback_getBehaviorFlags_mut_1")
    controller_behavior_callback_get_behavior_flags_mut_1 :: proc(self_: ^ControllerBehaviorCallback, controller: ^Controller) -> ControllerBehaviorFlags_Set ---

    /// Retrieve behavior flags for an obstacle.
    ///
    /// When the CCT touches an obstacle, the CCT's behavior w.r.t. this obstacle can be customized by users.
    /// This function retrieves the desired PxControllerBehaviorFlag flags capturing the desired behavior.
    ///
    /// See comments about deprecated functions at the start of this class
    ///
    /// Desired behavior flags for the given obstacle
    @(link_name = "PxControllerBehaviorCallback_getBehaviorFlags_mut_2")
    controller_behavior_callback_get_behavior_flags_mut_2 :: proc(self_: ^ControllerBehaviorCallback, obstacle: ^Obstacle) -> ControllerBehaviorFlags_Set ---

    /// Releases the controller manager.
    ///
    /// This will release all associated controllers and obstacle contexts.
    ///
    /// This function is required to be called to release foundation usage.
    @(link_name = "PxControllerManager_release_mut")
    controller_manager_release_mut :: proc(self_: ^ControllerManager) ---

    /// Returns the scene the manager is adding the controllers to.
    ///
    /// The associated physics scene.
    @(link_name = "PxControllerManager_getScene")
    controller_manager_get_scene :: proc(self_: ^ControllerManager) -> ^Scene ---

    /// Returns the number of controllers that are being managed.
    ///
    /// The number of controllers.
    @(link_name = "PxControllerManager_getNbControllers")
    controller_manager_get_nb_controllers :: proc(self_: ^ControllerManager) -> _c.uint32_t ---

    /// Retrieve one of the controllers in the manager.
    ///
    /// The controller with the specified index.
    @(link_name = "PxControllerManager_getController_mut")
    controller_manager_get_controller_mut :: proc(self_: ^ControllerManager, index: _c.uint32_t) -> ^Controller ---

    /// Creates a new character controller.
    ///
    /// The new controller
    @(link_name = "PxControllerManager_createController_mut")
    controller_manager_create_controller_mut :: proc(self_: ^ControllerManager, desc: ^ControllerDesc) -> ^Controller ---

    /// Releases all the controllers that are being managed.
    @(link_name = "PxControllerManager_purgeControllers_mut")
    controller_manager_purge_controllers_mut :: proc(self_: ^ControllerManager) ---

    /// Retrieves debug data.
    ///
    /// The render buffer filled with debug-render data
    @(link_name = "PxControllerManager_getRenderBuffer_mut")
    controller_manager_get_render_buffer_mut :: proc(self_: ^ControllerManager) -> ^RenderBuffer ---

    /// Sets debug rendering flags
    @(link_name = "PxControllerManager_setDebugRenderingFlags_mut")
    controller_manager_set_debug_rendering_flags_mut :: proc(self_: ^ControllerManager, flags: ControllerDebugRenderFlags_Set) ---

    /// Returns the number of obstacle contexts that are being managed.
    ///
    /// The number of obstacle contexts.
    @(link_name = "PxControllerManager_getNbObstacleContexts")
    controller_manager_get_nb_obstacle_contexts :: proc(self_: ^ControllerManager) -> _c.uint32_t ---

    /// Retrieve one of the obstacle contexts in the manager.
    ///
    /// The obstacle context with the specified index.
    @(link_name = "PxControllerManager_getObstacleContext_mut")
    controller_manager_get_obstacle_context_mut :: proc(self_: ^ControllerManager, index: _c.uint32_t) -> ^ObstacleContext ---

    /// Creates an obstacle context.
    ///
    /// New obstacle context
    @(link_name = "PxControllerManager_createObstacleContext_mut")
    controller_manager_create_obstacle_context_mut :: proc(self_: ^ControllerManager) -> ^ObstacleContext ---

    /// Computes character-character interactions.
    ///
    /// This function is an optional helper to properly resolve interactions between characters, in case they overlap (which can happen for gameplay reasons, etc).
    ///
    /// You should call this once per frame, before your PxController::move() calls. The function will not move the characters directly, but it will
    /// compute overlap information for each character that will be used in the next move() call.
    ///
    /// You need to provide a proper time value here so that interactions are resolved in a way that do not depend on the framerate.
    ///
    /// If you only have one character in the scene, or if you can guarantee your characters will never overlap, then you do not need to call this function.
    ///
    /// Releasing the manager will automatically release all the associated obstacle contexts.
    @(link_name = "PxControllerManager_computeInteractions_mut")
    controller_manager_compute_interactions_mut :: proc(self_: ^ControllerManager, elapsedTime: _c.float, cctFilterCb: ^ControllerFilterCallback) ---

    /// Enables or disables runtime tessellation.
    ///
    /// Large triangles can create accuracy issues in the sweep code, which in turn can lead to characters not sliding smoothly
    /// against geometries, or even penetrating them. This feature allows one to reduce those issues by tessellating large
    /// triangles at runtime, before performing sweeps against them. The amount of tessellation is controlled by the 'maxEdgeLength' parameter.
    /// Any triangle with at least one edge length greater than the maxEdgeLength will get recursively tessellated, until resulting triangles are small enough.
    ///
    /// This features only applies to triangle meshes, convex meshes, heightfields and boxes.
    @(link_name = "PxControllerManager_setTessellation_mut")
    controller_manager_set_tessellation_mut :: proc(self_: ^ControllerManager, flag: _c.bool, maxEdgeLength: _c.float) ---

    /// Enables or disables the overlap recovery module.
    ///
    /// The overlap recovery module can be used to depenetrate CCTs from static objects when an overlap is detected. This can happen
    /// in three main cases:
    /// - when the CCT is directly spawned or teleported in another object
    /// - when the CCT algorithm fails due to limited FPU accuracy
    /// - when the "up vector" is modified, making the rotated CCT shape overlap surrounding objects
    ///
    /// When activated, the CCT module will automatically try to resolve the penetration, and move the CCT to a safe place where it does
    /// not overlap other objects anymore. This only concerns static objects, dynamic objects are ignored by the recovery module.
    ///
    /// When the recovery module is not activated, it is possible for the CCTs to go through static objects. By default, the recovery
    /// module is enabled.
    ///
    /// The recovery module currently works with all geometries except heightfields.
    @(link_name = "PxControllerManager_setOverlapRecoveryModule_mut")
    controller_manager_set_overlap_recovery_module_mut :: proc(self_: ^ControllerManager, flag: _c.bool) ---

    /// Enables or disables the precise sweeps.
    ///
    /// Precise sweeps are more accurate, but also potentially slower than regular sweeps.
    ///
    /// By default, precise sweeps are enabled.
    @(link_name = "PxControllerManager_setPreciseSweeps_mut")
    controller_manager_set_precise_sweeps_mut :: proc(self_: ^ControllerManager, flag: _c.bool) ---

    /// Enables or disables vertical sliding against ceilings.
    ///
    /// Geometry is seen as "ceilings" when the following condition is met:
    ///
    /// dot product(contact normal, up direction)
    /// <
    /// 0.0f
    ///
    /// This flag controls whether characters should slide vertically along the geometry in that case.
    ///
    /// By default, sliding is allowed.
    @(link_name = "PxControllerManager_setPreventVerticalSlidingAgainstCeiling_mut")
    controller_manager_set_prevent_vertical_sliding_against_ceiling_mut :: proc(self_: ^ControllerManager, flag: _c.bool) ---

    /// Shift the origin of the character controllers and obstacle objects by the specified vector.
    ///
    /// The positions of all character controllers, obstacle objects and the corresponding data structures will get adjusted to reflect the shifted origin location
    /// (the shift vector will get subtracted from all character controller and obstacle object positions).
    ///
    /// It is the user's responsibility to keep track of the summed total origin shift and adjust all input/output to/from PhysXCharacterKinematic accordingly.
    ///
    /// This call will not automatically shift the PhysX scene and its objects. You need to call PxScene::shiftOrigin() seperately to keep the systems in sync.
    @(link_name = "PxControllerManager_shiftOrigin_mut")
    controller_manager_shift_origin_mut :: proc(self_: ^ControllerManager, #by_ptr shift: Vec3) ---

    /// Creates the controller manager.
    ///
    /// The character controller is informed by [`PxDeletionListener::onRelease`]() when actors or shapes are released, and updates its internal
    /// caches accordingly. If character controller movement or a call to [`PxControllerManager::shiftOrigin`]() may overlap with actor/shape releases,
    /// internal data structures must be guarded against concurrent access.
    ///
    /// Locking guarantees thread safety in such scenarios.
    ///
    /// locking may result in significant slowdown for release of actors or shapes.
    ///
    /// By default, locking is disabled.
    @(link_name = "phys_PxCreateControllerManager")
    create_controller_manager :: proc(scene: ^Scene, lockingEnabled: _c.bool) -> ^ControllerManager ---

    @(link_name = "PxDim3_new")
    dim3_new :: proc() -> Dim3 ---

    /// Constructor
    @(link_name = "PxSDFDesc_new")
    s_d_f_desc_new :: proc() -> SDFDesc ---

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid
    @(link_name = "PxSDFDesc_isValid")
    s_d_f_desc_is_valid :: proc(self_: ^SDFDesc) -> _c.bool ---

    /// constructor sets to default.
    @(link_name = "PxConvexMeshDesc_new")
    convex_mesh_desc_new :: proc() -> ConvexMeshDesc ---

    /// (re)sets the structure to the default.
    @(link_name = "PxConvexMeshDesc_setToDefault_mut")
    convex_mesh_desc_set_to_default_mut :: proc(self_: ^ConvexMeshDesc) ---

    /// Returns true if the descriptor is valid.
    ///
    /// True if the current settings are valid
    @(link_name = "PxConvexMeshDesc_isValid")
    convex_mesh_desc_is_valid :: proc(self_: ^ConvexMeshDesc) -> _c.bool ---

    /// Constructor sets to default.
    @(link_name = "PxTriangleMeshDesc_new")
    triangle_mesh_desc_new :: proc() -> TriangleMeshDesc ---

    /// (re)sets the structure to the default.
    @(link_name = "PxTriangleMeshDesc_setToDefault_mut")
    triangle_mesh_desc_set_to_default_mut :: proc(self_: ^TriangleMeshDesc) ---

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid
    @(link_name = "PxTriangleMeshDesc_isValid")
    triangle_mesh_desc_is_valid :: proc(self_: ^TriangleMeshDesc) -> _c.bool ---

    /// Constructor to build an empty tetmesh description
    @(link_name = "PxTetrahedronMeshDesc_new")
    tetrahedron_mesh_desc_new :: proc() -> TetrahedronMeshDesc ---

    @(link_name = "PxTetrahedronMeshDesc_isValid")
    tetrahedron_mesh_desc_is_valid :: proc(self_: ^TetrahedronMeshDesc) -> _c.bool ---

    /// Constructor to build an empty simulation description
    @(link_name = "PxSoftBodySimulationDataDesc_new")
    soft_body_simulation_data_desc_new :: proc() -> SoftBodySimulationDataDesc ---

    @(link_name = "PxSoftBodySimulationDataDesc_isValid")
    soft_body_simulation_data_desc_is_valid :: proc(self_: ^SoftBodySimulationDataDesc) -> _c.bool ---

    /// Desc initialization to default value.
    @(link_name = "PxBVH34MidphaseDesc_setToDefault_mut")
    b_v_h34_midphase_desc_set_to_default_mut :: proc(self_: ^BVH34MidphaseDesc) ---

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid.
    @(link_name = "PxBVH34MidphaseDesc_isValid")
    b_v_h34_midphase_desc_is_valid :: proc(self_: ^BVH34MidphaseDesc) -> _c.bool ---

    @(link_name = "PxMidphaseDesc_new")
    midphase_desc_new :: proc() -> MidphaseDesc ---

    /// Returns type of midphase mesh structure.
    ///
    /// PxMeshMidPhase::Enum
    @(link_name = "PxMidphaseDesc_getType")
    midphase_desc_get_type :: proc(self_: ^MidphaseDesc) -> MeshMidPhase ---

    /// Initialize the midphase mesh structure descriptor
    @(link_name = "PxMidphaseDesc_setToDefault_mut")
    midphase_desc_set_to_default_mut :: proc(self_: ^MidphaseDesc, type: MeshMidPhase) ---

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid.
    @(link_name = "PxMidphaseDesc_isValid")
    midphase_desc_is_valid :: proc(self_: ^MidphaseDesc) -> _c.bool ---

    @(link_name = "PxBVHDesc_new")
    b_v_h_desc_new :: proc() -> BVHDesc ---

    /// Initialize the BVH descriptor
    @(link_name = "PxBVHDesc_setToDefault_mut")
    b_v_h_desc_set_to_default_mut :: proc(self_: ^BVHDesc) ---

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid.
    @(link_name = "PxBVHDesc_isValid")
    b_v_h_desc_is_valid :: proc(self_: ^BVHDesc) -> _c.bool ---

    @(link_name = "PxCookingParams_new")
    cooking_params_new :: proc(#by_ptr sc: TolerancesScale) -> CookingParams ---

    @(link_name = "phys_PxGetStandaloneInsertionCallback")
    get_standalone_insertion_callback :: proc() -> ^InsertionCallback ---

    /// Cooks a bounding volume hierarchy. The results are written to the stream.
    ///
    /// PxCookBVH() allows a BVH description to be cooked into a binary stream
    /// suitable for loading and performing BVH detection at runtime.
    ///
    /// true on success.
    @(link_name = "phys_PxCookBVH")
    cook_b_v_h :: proc(#by_ptr desc: BVHDesc, stream: ^OutputStream) -> _c.bool ---

    /// Cooks and creates a bounding volume hierarchy without going through a stream.
    ///
    /// This method does the same as cookBVH, but the produced BVH is not stored
    /// into a stream but is either directly inserted in PxPhysics, or created as a standalone
    /// object. Use this method if you are unable to cook offline.
    ///
    /// PxInsertionCallback can be obtained through PxPhysics::getPhysicsInsertionCallback()
    /// or PxCooking::getStandaloneInsertionCallback().
    ///
    /// PxBVH pointer on success
    @(link_name = "phys_PxCreateBVH")
    create_b_v_h :: proc(#by_ptr desc: BVHDesc, insertionCallback: ^InsertionCallback) -> ^BVH ---

    /// Cooks a heightfield. The results are written to the stream.
    ///
    /// To create a heightfield object there is an option to precompute some of calculations done while loading the heightfield data.
    ///
    /// cookHeightField() allows a heightfield description to be cooked into a binary stream
    /// suitable for loading and performing collision detection at runtime.
    ///
    /// true on success
    @(link_name = "phys_PxCookHeightField")
    cook_height_field :: proc(#by_ptr desc: HeightFieldDesc, stream: ^OutputStream) -> _c.bool ---

    /// Cooks and creates a heightfield mesh and inserts it into PxPhysics.
    ///
    /// PxHeightField pointer on success
    @(link_name = "phys_PxCreateHeightField")
    create_height_field :: proc(#by_ptr desc: HeightFieldDesc, insertionCallback: ^InsertionCallback) -> ^HeightField ---

    /// Cooks a convex mesh. The results are written to the stream.
    ///
    /// To create a triangle mesh object it is necessary to first 'cook' the mesh data into
    /// a form which allows the SDK to perform efficient collision detection.
    ///
    /// cookConvexMesh() allows a mesh description to be cooked into a binary stream
    /// suitable for loading and performing collision detection at runtime.
    ///
    /// The number of vertices and the number of convex polygons in a cooked convex mesh is limited to 255.
    ///
    /// If those limits are exceeded in either the user-provided data or the final cooked mesh, an error is reported.
    ///
    /// true on success.
    @(link_name = "phys_PxCookConvexMesh")
    cook_convex_mesh :: proc(#by_ptr params: CookingParams, #by_ptr desc: ConvexMeshDesc, stream: ^OutputStream, condition: ^ConvexMeshCookingResult) -> _c.bool ---

    /// Cooks and creates a convex mesh without going through a stream.
    ///
    /// This method does the same as cookConvexMesh, but the produced mesh is not stored
    /// into a stream but is either directly inserted in PxPhysics, or created as a standalone
    /// object. Use this method if you are unable to cook offline.
    ///
    /// PxInsertionCallback can be obtained through PxPhysics::getPhysicsInsertionCallback()
    /// or PxCooking::getStandaloneInsertionCallback().
    ///
    /// PxConvexMesh pointer on success
    @(link_name = "phys_PxCreateConvexMesh")
    create_convex_mesh :: proc(#by_ptr params: CookingParams, #by_ptr desc: ConvexMeshDesc, insertionCallback: ^InsertionCallback, condition: ^ConvexMeshCookingResult) -> ^ConvexMesh ---

    /// Verifies if the convex mesh is valid. Prints an error message for each inconsistency found.
    ///
    /// The convex mesh descriptor must contain an already created convex mesh - the vertices, indices and polygons must be provided.
    ///
    /// This function should be used if PxConvexFlag::eDISABLE_MESH_VALIDATION is planned to be used in release builds.
    ///
    /// true if all the validity conditions hold, false otherwise.
    @(link_name = "phys_PxValidateConvexMesh")
    validate_convex_mesh :: proc(#by_ptr params: CookingParams, #by_ptr desc: ConvexMeshDesc) -> _c.bool ---

    /// Computed hull polygons from given vertices and triangles. Polygons are needed for PxConvexMeshDesc rather than triangles.
    ///
    /// Please note that the resulting polygons may have different number of vertices. Some vertices may be removed.
    /// The output vertices, indices and polygons must be used to construct a hull.
    ///
    /// The provided PxAllocatorCallback does allocate the out array's. It is the user responsibility to deallocated those
    /// array's.
    ///
    /// true on success
    @(link_name = "phys_PxComputeHullPolygons")
    compute_hull_polygons :: proc(#by_ptr params: CookingParams, mesh: ^SimpleTriangleMesh, inCallback: ^AllocatorCallback, nbVerts: ^_c.uint32_t, vertices: ^^Vec3, nbIndices: ^_c.uint32_t, indices: ^^_c.uint32_t, nbPolygons: ^_c.uint32_t, hullPolygons: ^^HullPolygon) -> _c.bool ---

    /// Verifies if the triangle mesh is valid. Prints an error message for each inconsistency found.
    ///
    /// The following conditions are true for a valid triangle mesh:
    /// 1. There are no duplicate vertices (within specified vertexWeldTolerance. See PxCookingParams::meshWeldTolerance)
    /// 2. There are no large triangles (within specified PxTolerancesScale.)
    ///
    /// true if all the validity conditions hold, false otherwise.
    @(link_name = "phys_PxValidateTriangleMesh")
    validate_triangle_mesh :: proc(#by_ptr params: CookingParams, #by_ptr desc: TriangleMeshDesc) -> _c.bool ---

    /// Cooks and creates a triangle mesh without going through a stream.
    ///
    /// This method does the same as cookTriangleMesh, but the produced mesh is not stored
    /// into a stream but is either directly inserted in PxPhysics, or created as a standalone
    /// object. Use this method if you are unable to cook offline.
    ///
    /// PxInsertionCallback can be obtained through PxPhysics::getPhysicsInsertionCallback()
    /// or PxCooking::getStandaloneInsertionCallback().
    ///
    /// PxTriangleMesh pointer on success.
    @(link_name = "phys_PxCreateTriangleMesh")
    create_triangle_mesh :: proc(#by_ptr params: CookingParams, #by_ptr desc: TriangleMeshDesc, insertionCallback: ^InsertionCallback, condition: ^TriangleMeshCookingResult) -> ^TriangleMesh ---

    /// Cooks a triangle mesh. The results are written to the stream.
    ///
    /// To create a triangle mesh object it is necessary to first 'cook' the mesh data into
    /// a form which allows the SDK to perform efficient collision detection.
    ///
    /// PxCookTriangleMesh() allows a mesh description to be cooked into a binary stream
    /// suitable for loading and performing collision detection at runtime.
    ///
    /// true on success
    @(link_name = "phys_PxCookTriangleMesh")
    cook_triangle_mesh :: proc(#by_ptr params: CookingParams, #by_ptr desc: TriangleMeshDesc, stream: ^OutputStream, condition: ^TriangleMeshCookingResult) -> _c.bool ---

    @(link_name = "PxDefaultMemoryOutputStream_new_alloc")
    default_memory_output_stream_new_alloc :: proc(allocator: ^AllocatorCallback) -> ^DefaultMemoryOutputStream ---

    @(link_name = "PxDefaultMemoryOutputStream_delete")
    default_memory_output_stream_delete :: proc(self_: ^DefaultMemoryOutputStream) ---

    @(link_name = "PxDefaultMemoryOutputStream_write_mut")
    default_memory_output_stream_write_mut :: proc(self_: ^DefaultMemoryOutputStream, src: rawptr, count: _c.uint32_t) -> _c.uint32_t ---

    @(link_name = "PxDefaultMemoryOutputStream_getSize")
    default_memory_output_stream_get_size :: proc(self_: ^DefaultMemoryOutputStream) -> _c.uint32_t ---

    @(link_name = "PxDefaultMemoryOutputStream_getData")
    default_memory_output_stream_get_data :: proc(self_: ^DefaultMemoryOutputStream) -> ^_c.uint8_t ---

    @(link_name = "PxDefaultMemoryInputData_new_alloc")
    default_memory_input_data_new_alloc :: proc(data: ^_c.uint8_t, length: _c.uint32_t) -> ^DefaultMemoryInputData ---

    @(link_name = "PxDefaultMemoryInputData_read_mut")
    default_memory_input_data_read_mut :: proc(self_: ^DefaultMemoryInputData, dest: rawptr, count: _c.uint32_t) -> _c.uint32_t ---

    @(link_name = "PxDefaultMemoryInputData_getLength")
    default_memory_input_data_get_length :: proc(self_: ^DefaultMemoryInputData) -> _c.uint32_t ---

    @(link_name = "PxDefaultMemoryInputData_seek_mut")
    default_memory_input_data_seek_mut :: proc(self_: ^DefaultMemoryInputData, pos: _c.uint32_t) ---

    @(link_name = "PxDefaultMemoryInputData_tell")
    default_memory_input_data_tell :: proc(self_: ^DefaultMemoryInputData) -> _c.uint32_t ---

    @(link_name = "PxDefaultFileOutputStream_new_alloc")
    default_file_output_stream_new_alloc :: proc(name: cstring) -> ^DefaultFileOutputStream ---

    @(link_name = "PxDefaultFileOutputStream_delete")
    default_file_output_stream_delete :: proc(self_: ^DefaultFileOutputStream) ---

    @(link_name = "PxDefaultFileOutputStream_write_mut")
    default_file_output_stream_write_mut :: proc(self_: ^DefaultFileOutputStream, src: rawptr, count: _c.uint32_t) -> _c.uint32_t ---

    @(link_name = "PxDefaultFileOutputStream_isValid_mut")
    default_file_output_stream_is_valid_mut :: proc(self_: ^DefaultFileOutputStream) -> _c.bool ---

    @(link_name = "PxDefaultFileInputData_new_alloc")
    default_file_input_data_new_alloc :: proc(name: cstring) -> ^DefaultFileInputData ---

    @(link_name = "PxDefaultFileInputData_delete")
    default_file_input_data_delete :: proc(self_: ^DefaultFileInputData) ---

    @(link_name = "PxDefaultFileInputData_read_mut")
    default_file_input_data_read_mut :: proc(self_: ^DefaultFileInputData, dest: rawptr, count: _c.uint32_t) -> _c.uint32_t ---

    @(link_name = "PxDefaultFileInputData_seek_mut")
    default_file_input_data_seek_mut :: proc(self_: ^DefaultFileInputData, pos: _c.uint32_t) ---

    @(link_name = "PxDefaultFileInputData_tell")
    default_file_input_data_tell :: proc(self_: ^DefaultFileInputData) -> _c.uint32_t ---

    @(link_name = "PxDefaultFileInputData_getLength")
    default_file_input_data_get_length :: proc(self_: ^DefaultFileInputData) -> _c.uint32_t ---

    @(link_name = "PxDefaultFileInputData_isValid")
    default_file_input_data_is_valid :: proc(self_: ^DefaultFileInputData) -> _c.bool ---

    @(link_name = "phys_platformAlignedAlloc")
    platform_aligned_alloc :: proc(size: _c.size_t) -> rawptr ---

    @(link_name = "phys_platformAlignedFree")
    platform_aligned_free :: proc(ptr: rawptr) ---

    @(link_name = "PxDefaultAllocator_allocate_mut")
    default_allocator_allocate_mut :: proc(self_: ^DefaultAllocator, size: _c.size_t, anon_param1: cstring, anon_param2: cstring, anon_param3: _c.int32_t) -> rawptr ---

    @(link_name = "PxDefaultAllocator_deallocate_mut")
    default_allocator_deallocate_mut :: proc(self_: ^DefaultAllocator, ptr: rawptr) ---

    @(link_name = "PxDefaultAllocator_delete")
    default_allocator_delete :: proc(self_: ^DefaultAllocator) ---

    /// Set the actors for this joint.
    ///
    /// An actor may be NULL to indicate the world frame. At most one of the actors may be NULL.
    @(link_name = "PxJoint_setActors_mut")
    joint_set_actors_mut :: proc(self_: ^Joint, actor0: ^RigidActor, actor1: ^RigidActor) ---

    /// Get the actors for this joint.
    @(link_name = "PxJoint_getActors")
    joint_get_actors :: proc(self_: ^Joint, actor0: ^^RigidActor, actor1: ^^RigidActor) ---

    /// Set the joint local pose for an actor.
    ///
    /// This is the relative pose which locates the joint frame relative to the actor.
    @(link_name = "PxJoint_setLocalPose_mut")
    joint_set_local_pose_mut :: proc(self_: ^Joint, actor: JointActorIndex, #by_ptr localPose: Transform) ---

    /// get the joint local pose for an actor.
    ///
    /// return the local pose for this joint
    @(link_name = "PxJoint_getLocalPose")
    joint_get_local_pose :: proc(self_: ^Joint, actor: JointActorIndex) -> Transform ---

    /// get the relative pose for this joint
    ///
    /// This function returns the pose of the joint frame of actor1 relative to actor0
    @(link_name = "PxJoint_getRelativeTransform")
    joint_get_relative_transform :: proc(self_: ^Joint) -> Transform ---

    /// get the relative linear velocity of the joint
    ///
    /// This function returns the linear velocity of the origin of the constraint frame of actor1, relative to the origin of the constraint
    /// frame of actor0. The value is returned in the constraint frame of actor0
    @(link_name = "PxJoint_getRelativeLinearVelocity")
    joint_get_relative_linear_velocity :: proc(self_: ^Joint) -> Vec3 ---

    /// get the relative angular velocity of the joint
    ///
    /// This function returns the angular velocity of  actor1 relative to actor0. The value is returned in the constraint frame of actor0
    @(link_name = "PxJoint_getRelativeAngularVelocity")
    joint_get_relative_angular_velocity :: proc(self_: ^Joint) -> Vec3 ---

    /// set the break force for this joint.
    ///
    /// if the constraint force or torque on the joint exceeds the specified values, the joint will break,
    /// at which point it will not constrain the two actors and the flag PxConstraintFlag::eBROKEN will be set. The
    /// force and torque are measured in the joint frame of the first actor
    @(link_name = "PxJoint_setBreakForce_mut")
    joint_set_break_force_mut :: proc(self_: ^Joint, force: _c.float, torque: _c.float) ---

    /// get the break force for this joint.
    @(link_name = "PxJoint_getBreakForce")
    joint_get_break_force :: proc(self_: ^Joint, force: ^_c.float, torque: ^_c.float) ---

    /// set the constraint flags for this joint.
    @(link_name = "PxJoint_setConstraintFlags_mut")
    joint_set_constraint_flags_mut :: proc(self_: ^Joint, flags: ConstraintFlags_Set) ---

    /// set a constraint flags for this joint to a specified value.
    @(link_name = "PxJoint_setConstraintFlag_mut")
    joint_set_constraint_flag_mut :: proc(self_: ^Joint, flag: ConstraintFlag, value: _c.bool) ---

    /// get the constraint flags for this joint.
    ///
    /// the constraint flags
    @(link_name = "PxJoint_getConstraintFlags")
    joint_get_constraint_flags :: proc(self_: ^Joint) -> ConstraintFlags_Set ---

    /// set the inverse mass scale for actor0.
    @(link_name = "PxJoint_setInvMassScale0_mut")
    joint_set_inv_mass_scale0_mut :: proc(self_: ^Joint, invMassScale: _c.float) ---

    /// get the inverse mass scale for actor0.
    ///
    /// inverse mass scale for actor0
    @(link_name = "PxJoint_getInvMassScale0")
    joint_get_inv_mass_scale0 :: proc(self_: ^Joint) -> _c.float ---

    /// set the inverse inertia scale for actor0.
    @(link_name = "PxJoint_setInvInertiaScale0_mut")
    joint_set_inv_inertia_scale0_mut :: proc(self_: ^Joint, invInertiaScale: _c.float) ---

    /// get the inverse inertia scale for actor0.
    ///
    /// inverse inertia scale for actor0
    @(link_name = "PxJoint_getInvInertiaScale0")
    joint_get_inv_inertia_scale0 :: proc(self_: ^Joint) -> _c.float ---

    /// set the inverse mass scale for actor1.
    @(link_name = "PxJoint_setInvMassScale1_mut")
    joint_set_inv_mass_scale1_mut :: proc(self_: ^Joint, invMassScale: _c.float) ---

    /// get the inverse mass scale for actor1.
    ///
    /// inverse mass scale for actor1
    @(link_name = "PxJoint_getInvMassScale1")
    joint_get_inv_mass_scale1 :: proc(self_: ^Joint) -> _c.float ---

    /// set the inverse inertia scale for actor1.
    @(link_name = "PxJoint_setInvInertiaScale1_mut")
    joint_set_inv_inertia_scale1_mut :: proc(self_: ^Joint, invInertiaScale: _c.float) ---

    /// get the inverse inertia scale for actor1.
    ///
    /// inverse inertia scale for actor1
    @(link_name = "PxJoint_getInvInertiaScale1")
    joint_get_inv_inertia_scale1 :: proc(self_: ^Joint) -> _c.float ---

    /// Retrieves the PxConstraint corresponding to this joint.
    ///
    /// This can be used to determine, among other things, the force applied at the joint.
    ///
    /// the constraint
    @(link_name = "PxJoint_getConstraint")
    joint_get_constraint :: proc(self_: ^Joint) -> ^Constraint ---

    /// Sets a name string for the object that can be retrieved with getName().
    ///
    /// This is for debugging and is not used by the SDK. The string is not copied by the SDK,
    /// only the pointer is stored.
    @(link_name = "PxJoint_setName_mut")
    joint_set_name_mut :: proc(self_: ^Joint, name: cstring) ---

    /// Retrieves the name string set with setName().
    ///
    /// Name string associated with object.
    @(link_name = "PxJoint_getName")
    joint_get_name :: proc(self_: ^Joint) -> cstring ---

    /// Deletes the joint.
    ///
    /// This call does not wake up the connected rigid bodies.
    @(link_name = "PxJoint_release_mut")
    joint_release_mut :: proc(self_: ^Joint) ---

    /// Retrieves the scene which this joint belongs to.
    ///
    /// Owner Scene. NULL if not part of a scene.
    @(link_name = "PxJoint_getScene")
    joint_get_scene :: proc(self_: ^Joint) -> ^Scene ---

    /// Put class meta data in stream, used for serialization
    @(link_name = "PxJoint_getBinaryMetaData")
    joint_get_binary_meta_data :: proc(stream: ^OutputStream) ---

    @(link_name = "PxSpring_new")
    spring_new :: proc(stiffness_: _c.float, damping_: _c.float) -> Spring ---

    /// Helper function to setup a joint's global frame
    ///
    /// This replaces the following functions from previous SDK versions:
    ///
    /// void NxJointDesc::setGlobalAnchor(const NxVec3
    /// &
    /// wsAnchor);
    /// void NxJointDesc::setGlobalAxis(const NxVec3
    /// &
    /// wsAxis);
    ///
    /// The function sets the joint's localPose using world-space input parameters.
    @(link_name = "phys_PxSetJointGlobalFrame")
    set_joint_global_frame :: proc(joint: ^Joint, wsAnchor: ^Vec3, wsAxis: ^Vec3) ---

    /// Create a distance Joint.
    @(link_name = "phys_PxDistanceJointCreate")
    distance_joint_create :: proc(physics: ^Physics, actor0: ^RigidActor, #by_ptr localFrame0: Transform, actor1: ^RigidActor, #by_ptr localFrame1: Transform) -> ^DistanceJoint ---

    /// Return the current distance of the joint
    @(link_name = "PxDistanceJoint_getDistance")
    distance_joint_get_distance :: proc(self_: ^DistanceJoint) -> _c.float ---

    /// Set the allowed minimum distance for the joint.
    ///
    /// The minimum distance must be no more than the maximum distance
    ///
    /// Default
    /// 0.0f
    /// Range
    /// [0, PX_MAX_F32)
    @(link_name = "PxDistanceJoint_setMinDistance_mut")
    distance_joint_set_min_distance_mut :: proc(self_: ^DistanceJoint, distance: _c.float) ---

    /// Get the allowed minimum distance for the joint.
    ///
    /// the allowed minimum distance
    @(link_name = "PxDistanceJoint_getMinDistance")
    distance_joint_get_min_distance :: proc(self_: ^DistanceJoint) -> _c.float ---

    /// Set the allowed maximum distance for the joint.
    ///
    /// The maximum distance must be no less than the minimum distance.
    ///
    /// Default
    /// 0.0f
    /// Range
    /// [0, PX_MAX_F32)
    @(link_name = "PxDistanceJoint_setMaxDistance_mut")
    distance_joint_set_max_distance_mut :: proc(self_: ^DistanceJoint, distance: _c.float) ---

    /// Get the allowed maximum distance for the joint.
    ///
    /// the allowed maximum distance
    @(link_name = "PxDistanceJoint_getMaxDistance")
    distance_joint_get_max_distance :: proc(self_: ^DistanceJoint) -> _c.float ---

    /// Set the error tolerance of the joint.
    @(link_name = "PxDistanceJoint_setTolerance_mut")
    distance_joint_set_tolerance_mut :: proc(self_: ^DistanceJoint, tolerance: _c.float) ---

    /// Get the error tolerance of the joint.
    ///
    /// the distance beyond the joint's [min, max] range before the joint becomes active.
    ///
    /// Default
    /// 0.25f * PxTolerancesScale::length
    /// Range
    /// (0, PX_MAX_F32)
    ///
    /// This value should be used to ensure that if the minimum distance is zero and the
    /// spring function is in use, the rest length of the spring is non-zero.
    @(link_name = "PxDistanceJoint_getTolerance")
    distance_joint_get_tolerance :: proc(self_: ^DistanceJoint) -> _c.float ---

    /// Set the strength of the joint spring.
    ///
    /// The spring is used if enabled, and the distance exceeds the range [min-error, max+error].
    ///
    /// Default
    /// 0.0f
    /// Range
    /// [0, PX_MAX_F32)
    @(link_name = "PxDistanceJoint_setStiffness_mut")
    distance_joint_set_stiffness_mut :: proc(self_: ^DistanceJoint, stiffness: _c.float) ---

    /// Get the strength of the joint spring.
    ///
    /// stiffness the spring strength of the joint
    @(link_name = "PxDistanceJoint_getStiffness")
    distance_joint_get_stiffness :: proc(self_: ^DistanceJoint) -> _c.float ---

    /// Set the damping of the joint spring.
    ///
    /// The spring is used if enabled, and the distance exceeds the range [min-error, max+error].
    ///
    /// Default
    /// 0.0f
    /// Range
    /// [0, PX_MAX_F32)
    @(link_name = "PxDistanceJoint_setDamping_mut")
    distance_joint_set_damping_mut :: proc(self_: ^DistanceJoint, damping: _c.float) ---

    /// Get the damping of the joint spring.
    ///
    /// the degree of damping of the joint spring of the joint
    @(link_name = "PxDistanceJoint_getDamping")
    distance_joint_get_damping :: proc(self_: ^DistanceJoint) -> _c.float ---

    /// Set the contact distance for the min
    /// &
    /// max distance limits.
    ///
    /// This is similar to the PxJointLimitParameters::contactDistance parameter for regular limits.
    ///
    /// The two most common values are 0 and infinite. Infinite means the internal constraints are
    /// always created, resulting in the best simulation quality but slower performance. Zero means
    /// the internal constraints are only created when the limits are violated, resulting in best
    /// performance but worse simulation quality.
    ///
    /// Default
    /// 0.0f
    /// Range
    /// [0, PX_MAX_F32)
    @(link_name = "PxDistanceJoint_setContactDistance_mut")
    distance_joint_set_contact_distance_mut :: proc(self_: ^DistanceJoint, contactDistance: _c.float) ---

    /// Get the contact distance.
    ///
    /// the contact distance
    @(link_name = "PxDistanceJoint_getContactDistance")
    distance_joint_get_contact_distance :: proc(self_: ^DistanceJoint) -> _c.float ---

    /// Set the flags specific to the Distance Joint.
    ///
    /// Default
    /// PxDistanceJointFlag::eMAX_DISTANCE_ENABLED
    @(link_name = "PxDistanceJoint_setDistanceJointFlags_mut")
    distance_joint_set_distance_joint_flags_mut :: proc(self_: ^DistanceJoint, flags: DistanceJointFlags_Set) ---

    /// Set a single flag specific to a Distance Joint to true or false.
    @(link_name = "PxDistanceJoint_setDistanceJointFlag_mut")
    distance_joint_set_distance_joint_flag_mut :: proc(self_: ^DistanceJoint, flag: DistanceJointFlag, value: _c.bool) ---

    /// Get the flags specific to the Distance Joint.
    ///
    /// the joint flags
    @(link_name = "PxDistanceJoint_getDistanceJointFlags")
    distance_joint_get_distance_joint_flags :: proc(self_: ^DistanceJoint) -> DistanceJointFlags_Set ---

    /// Returns string name of PxDistanceJoint, used for serialization
    @(link_name = "PxDistanceJoint_getConcreteTypeName")
    distance_joint_get_concrete_type_name :: proc(self_: ^DistanceJoint) -> cstring ---

    /// Create a distance Joint.
    @(link_name = "phys_PxContactJointCreate")
    contact_joint_create :: proc(physics: ^Physics, actor0: ^RigidActor, #by_ptr localFrame0: Transform, actor1: ^RigidActor, #by_ptr localFrame1: Transform) -> ^ContactJoint ---

    @(link_name = "PxJacobianRow_new")
    jacobian_row_new :: proc() -> JacobianRow ---

    @(link_name = "PxJacobianRow_new_1")
    jacobian_row_new_1 :: proc(#by_ptr lin0: Vec3, #by_ptr lin1: Vec3, #by_ptr ang0: Vec3, #by_ptr ang1: Vec3) -> JacobianRow ---

    /// Set the current contact of the joint
    @(link_name = "PxContactJoint_setContact_mut")
    contact_joint_set_contact_mut :: proc(self_: ^ContactJoint, #by_ptr contact: Vec3) ---

    /// Set the current contact normal of the joint
    @(link_name = "PxContactJoint_setContactNormal_mut")
    contact_joint_set_contact_normal_mut :: proc(self_: ^ContactJoint, #by_ptr contactNormal: Vec3) ---

    /// Set the current penetration of the joint
    @(link_name = "PxContactJoint_setPenetration_mut")
    contact_joint_set_penetration_mut :: proc(self_: ^ContactJoint, penetration: _c.float) ---

    /// Return the current contact of the joint
    @(link_name = "PxContactJoint_getContact")
    contact_joint_get_contact :: proc(self_: ^ContactJoint) -> Vec3 ---

    /// Return the current contact normal of the joint
    @(link_name = "PxContactJoint_getContactNormal")
    contact_joint_get_contact_normal :: proc(self_: ^ContactJoint) -> Vec3 ---

    /// Return the current penetration value of the joint
    @(link_name = "PxContactJoint_getPenetration")
    contact_joint_get_penetration :: proc(self_: ^ContactJoint) -> _c.float ---

    @(link_name = "PxContactJoint_getRestitution")
    contact_joint_get_restitution :: proc(self_: ^ContactJoint) -> _c.float ---

    @(link_name = "PxContactJoint_setRestitution_mut")
    contact_joint_set_restitution_mut :: proc(self_: ^ContactJoint, restitution: _c.float) ---

    @(link_name = "PxContactJoint_getBounceThreshold")
    contact_joint_get_bounce_threshold :: proc(self_: ^ContactJoint) -> _c.float ---

    @(link_name = "PxContactJoint_setBounceThreshold_mut")
    contact_joint_set_bounce_threshold_mut :: proc(self_: ^ContactJoint, bounceThreshold: _c.float) ---

    /// Returns string name of PxContactJoint, used for serialization
    @(link_name = "PxContactJoint_getConcreteTypeName")
    contact_joint_get_concrete_type_name :: proc(self_: ^ContactJoint) -> cstring ---

    @(link_name = "PxContactJoint_computeJacobians")
    contact_joint_compute_jacobians :: proc(self_: ^ContactJoint, jacobian: ^JacobianRow) ---

    @(link_name = "PxContactJoint_getNbJacobianRows")
    contact_joint_get_nb_jacobian_rows :: proc(self_: ^ContactJoint) -> _c.uint32_t ---

    /// Create a fixed joint.
    @(link_name = "phys_PxFixedJointCreate")
    fixed_joint_create :: proc(physics: ^Physics, actor0: ^RigidActor, #by_ptr localFrame0: Transform, actor1: ^RigidActor, #by_ptr localFrame1: Transform) -> ^FixedJoint ---

    /// Returns string name of PxFixedJoint, used for serialization
    @(link_name = "PxFixedJoint_getConcreteTypeName")
    fixed_joint_get_concrete_type_name :: proc(self_: ^FixedJoint) -> cstring ---

    @(link_name = "PxJointLimitParameters_new_alloc")
    joint_limit_parameters_new_alloc :: proc() -> ^JointLimitParameters ---

    /// Returns true if the current settings are valid.
    ///
    /// true if the current settings are valid
    @(link_name = "PxJointLimitParameters_isValid")
    joint_limit_parameters_is_valid :: proc(self_: ^JointLimitParameters) -> _c.bool ---

    @(link_name = "PxJointLimitParameters_isSoft")
    joint_limit_parameters_is_soft :: proc(self_: ^JointLimitParameters) -> _c.bool ---

    /// construct a linear hard limit
    @(link_name = "PxJointLinearLimit_new")
    joint_linear_limit_new :: proc(#by_ptr scale: TolerancesScale, extent: _c.float, contactDist_deprecated: _c.float) -> JointLinearLimit ---

    /// construct a linear soft limit
    @(link_name = "PxJointLinearLimit_new_1")
    joint_linear_limit_new_1 :: proc(extent: _c.float, spring: ^Spring) -> JointLinearLimit ---

    /// Returns true if the limit is valid
    ///
    /// true if the current settings are valid
    @(link_name = "PxJointLinearLimit_isValid")
    joint_linear_limit_is_valid :: proc(self_: ^JointLinearLimit) -> _c.bool ---

    @(link_name = "PxJointLinearLimit_delete")
    joint_linear_limit_delete :: proc(self_: ^JointLinearLimit) ---

    /// Construct a linear hard limit pair. The lower distance value must be less than the upper distance value.
    @(link_name = "PxJointLinearLimitPair_new")
    joint_linear_limit_pair_new :: proc(#by_ptr scale: TolerancesScale, lowerLimit: _c.float, upperLimit: _c.float, contactDist_deprecated: _c.float) -> JointLinearLimitPair ---

    /// construct a linear soft limit pair
    @(link_name = "PxJointLinearLimitPair_new_1")
    joint_linear_limit_pair_new_1 :: proc(lowerLimit: _c.float, upperLimit: _c.float, spring: ^Spring) -> JointLinearLimitPair ---

    /// Returns true if the limit is valid.
    ///
    /// true if the current settings are valid
    @(link_name = "PxJointLinearLimitPair_isValid")
    joint_linear_limit_pair_is_valid :: proc(self_: ^JointLinearLimitPair) -> _c.bool ---

    @(link_name = "PxJointLinearLimitPair_delete")
    joint_linear_limit_pair_delete :: proc(self_: ^JointLinearLimitPair) ---

    /// construct an angular hard limit pair.
    ///
    /// The lower value must be less than the upper value.
    @(link_name = "PxJointAngularLimitPair_new")
    joint_angular_limit_pair_new :: proc(lowerLimit: _c.float, upperLimit: _c.float, contactDist_deprecated: _c.float) -> JointAngularLimitPair ---

    /// construct an angular soft limit pair.
    ///
    /// The lower value must be less than the upper value.
    @(link_name = "PxJointAngularLimitPair_new_1")
    joint_angular_limit_pair_new_1 :: proc(lowerLimit: _c.float, upperLimit: _c.float, spring: ^Spring) -> JointAngularLimitPair ---

    /// Returns true if the limit is valid.
    ///
    /// true if the current settings are valid
    @(link_name = "PxJointAngularLimitPair_isValid")
    joint_angular_limit_pair_is_valid :: proc(self_: ^JointAngularLimitPair) -> _c.bool ---

    @(link_name = "PxJointAngularLimitPair_delete")
    joint_angular_limit_pair_delete :: proc(self_: ^JointAngularLimitPair) ---

    /// Construct a cone hard limit.
    @(link_name = "PxJointLimitCone_new")
    joint_limit_cone_new :: proc(yLimitAngle: _c.float, zLimitAngle: _c.float, contactDist_deprecated: _c.float) -> JointLimitCone ---

    /// Construct a cone soft limit.
    @(link_name = "PxJointLimitCone_new_1")
    joint_limit_cone_new_1 :: proc(yLimitAngle: _c.float, zLimitAngle: _c.float, spring: ^Spring) -> JointLimitCone ---

    /// Returns true if the limit is valid.
    ///
    /// true if the current settings are valid
    @(link_name = "PxJointLimitCone_isValid")
    joint_limit_cone_is_valid :: proc(self_: ^JointLimitCone) -> _c.bool ---

    @(link_name = "PxJointLimitCone_delete")
    joint_limit_cone_delete :: proc(self_: ^JointLimitCone) ---

    /// Construct a pyramid hard limit.
    @(link_name = "PxJointLimitPyramid_new")
    joint_limit_pyramid_new :: proc(yLimitAngleMin: _c.float, yLimitAngleMax: _c.float, zLimitAngleMin: _c.float, zLimitAngleMax: _c.float, contactDist_deprecated: _c.float) -> JointLimitPyramid ---

    /// Construct a pyramid soft limit.
    @(link_name = "PxJointLimitPyramid_new_1")
    joint_limit_pyramid_new_1 :: proc(yLimitAngleMin: _c.float, yLimitAngleMax: _c.float, zLimitAngleMin: _c.float, zLimitAngleMax: _c.float, spring: ^Spring) -> JointLimitPyramid ---

    /// Returns true if the limit is valid.
    ///
    /// true if the current settings are valid
    @(link_name = "PxJointLimitPyramid_isValid")
    joint_limit_pyramid_is_valid :: proc(self_: ^JointLimitPyramid) -> _c.bool ---

    @(link_name = "PxJointLimitPyramid_delete")
    joint_limit_pyramid_delete :: proc(self_: ^JointLimitPyramid) ---

    /// Create a prismatic joint.
    @(link_name = "phys_PxPrismaticJointCreate")
    prismatic_joint_create :: proc(physics: ^Physics, actor0: ^RigidActor, #by_ptr localFrame0: Transform, actor1: ^RigidActor, #by_ptr localFrame1: Transform) -> ^PrismaticJoint ---

    /// returns the displacement of the joint along its axis.
    @(link_name = "PxPrismaticJoint_getPosition")
    prismatic_joint_get_position :: proc(self_: ^PrismaticJoint) -> _c.float ---

    /// returns the velocity of the joint along its axis
    @(link_name = "PxPrismaticJoint_getVelocity")
    prismatic_joint_get_velocity :: proc(self_: ^PrismaticJoint) -> _c.float ---

    /// sets the joint limit  parameters.
    ///
    /// The limit range is [-PX_MAX_F32, PX_MAX_F32], but note that the width of the limit (upper-lower) must also be
    /// a valid float.
    @(link_name = "PxPrismaticJoint_setLimit_mut")
    prismatic_joint_set_limit_mut :: proc(self_: ^PrismaticJoint, #by_ptr anon_param0: JointLinearLimitPair) ---

    /// gets the joint limit  parameters.
    @(link_name = "PxPrismaticJoint_getLimit")
    prismatic_joint_get_limit :: proc(self_: ^PrismaticJoint) -> JointLinearLimitPair ---

    /// Set the flags specific to the Prismatic Joint.
    ///
    /// Default
    /// PxPrismaticJointFlags(0)
    @(link_name = "PxPrismaticJoint_setPrismaticJointFlags_mut")
    prismatic_joint_set_prismatic_joint_flags_mut :: proc(self_: ^PrismaticJoint, flags: PrismaticJointFlags_Set) ---

    /// Set a single flag specific to a Prismatic Joint to true or false.
    @(link_name = "PxPrismaticJoint_setPrismaticJointFlag_mut")
    prismatic_joint_set_prismatic_joint_flag_mut :: proc(self_: ^PrismaticJoint, flag: PrismaticJointFlag, value: _c.bool) ---

    /// Get the flags specific to the Prismatic Joint.
    ///
    /// the joint flags
    @(link_name = "PxPrismaticJoint_getPrismaticJointFlags")
    prismatic_joint_get_prismatic_joint_flags :: proc(self_: ^PrismaticJoint) -> PrismaticJointFlags_Set ---

    /// Returns string name of PxPrismaticJoint, used for serialization
    @(link_name = "PxPrismaticJoint_getConcreteTypeName")
    prismatic_joint_get_concrete_type_name :: proc(self_: ^PrismaticJoint) -> cstring ---

    /// Create a revolute joint.
    @(link_name = "phys_PxRevoluteJointCreate")
    revolute_joint_create :: proc(physics: ^Physics, actor0: ^RigidActor, #by_ptr localFrame0: Transform, actor1: ^RigidActor, #by_ptr localFrame1: Transform) -> ^RevoluteJoint ---

    /// return the angle of the joint, in the range (-2*Pi, 2*Pi]
    @(link_name = "PxRevoluteJoint_getAngle")
    revolute_joint_get_angle :: proc(self_: ^RevoluteJoint) -> _c.float ---

    /// return the velocity of the joint
    @(link_name = "PxRevoluteJoint_getVelocity")
    revolute_joint_get_velocity :: proc(self_: ^RevoluteJoint) -> _c.float ---

    /// set the joint limit parameters.
    ///
    /// The limit is activated using the flag PxRevoluteJointFlag::eLIMIT_ENABLED
    ///
    /// The limit angle range is (-2*Pi, 2*Pi).
    @(link_name = "PxRevoluteJoint_setLimit_mut")
    revolute_joint_set_limit_mut :: proc(self_: ^RevoluteJoint, #by_ptr limits: JointAngularLimitPair) ---

    /// get the joint limit parameters.
    ///
    /// the joint limit parameters
    @(link_name = "PxRevoluteJoint_getLimit")
    revolute_joint_get_limit :: proc(self_: ^RevoluteJoint) -> JointAngularLimitPair ---

    /// set the target velocity for the drive model.
    ///
    /// The motor will only be able to reach this velocity if the maxForce is sufficiently large.
    /// If the joint is spinning faster than this velocity, the motor will actually try to brake
    /// (see PxRevoluteJointFlag::eDRIVE_FREESPIN.)
    ///
    /// The sign of this variable determines the rotation direction, with positive values going
    /// the same way as positive joint angles. Setting a very large target velocity may cause
    /// undesirable results.
    ///
    /// Range:
    /// (-PX_MAX_F32, PX_MAX_F32)
    /// Default:
    /// 0.0
    @(link_name = "PxRevoluteJoint_setDriveVelocity_mut")
    revolute_joint_set_drive_velocity_mut :: proc(self_: ^RevoluteJoint, velocity: _c.float, autowake: _c.bool) ---

    /// gets the target velocity for the drive model.
    ///
    /// the drive target velocity
    @(link_name = "PxRevoluteJoint_getDriveVelocity")
    revolute_joint_get_drive_velocity :: proc(self_: ^RevoluteJoint) -> _c.float ---

    /// sets the maximum torque the drive can exert.
    ///
    /// The value set here may be used either as an impulse limit or a force limit, depending on the flag PxConstraintFlag::eDRIVE_LIMITS_ARE_FORCES
    ///
    /// Range:
    /// [0, PX_MAX_F32)
    /// Default:
    /// PX_MAX_F32
    @(link_name = "PxRevoluteJoint_setDriveForceLimit_mut")
    revolute_joint_set_drive_force_limit_mut :: proc(self_: ^RevoluteJoint, limit: _c.float) ---

    /// gets the maximum torque the drive can exert.
    ///
    /// the torque limit
    @(link_name = "PxRevoluteJoint_getDriveForceLimit")
    revolute_joint_get_drive_force_limit :: proc(self_: ^RevoluteJoint) -> _c.float ---

    /// sets the gear ratio for the drive.
    ///
    /// When setting up the drive constraint, the velocity of the first actor is scaled by this value, and its response to drive torque is scaled down.
    /// So if the drive target velocity is zero, the second actor will be driven to the velocity of the first scaled by the gear ratio
    ///
    /// Range:
    /// [0, PX_MAX_F32)
    /// Default:
    /// 1.0
    @(link_name = "PxRevoluteJoint_setDriveGearRatio_mut")
    revolute_joint_set_drive_gear_ratio_mut :: proc(self_: ^RevoluteJoint, ratio: _c.float) ---

    /// gets the gear ratio.
    ///
    /// the drive gear ratio
    @(link_name = "PxRevoluteJoint_getDriveGearRatio")
    revolute_joint_get_drive_gear_ratio :: proc(self_: ^RevoluteJoint) -> _c.float ---

    /// sets the flags specific to the Revolute Joint.
    ///
    /// Default
    /// PxRevoluteJointFlags(0)
    @(link_name = "PxRevoluteJoint_setRevoluteJointFlags_mut")
    revolute_joint_set_revolute_joint_flags_mut :: proc(self_: ^RevoluteJoint, flags: RevoluteJointFlags_Set) ---

    /// sets a single flag specific to a Revolute Joint.
    @(link_name = "PxRevoluteJoint_setRevoluteJointFlag_mut")
    revolute_joint_set_revolute_joint_flag_mut :: proc(self_: ^RevoluteJoint, flag: RevoluteJointFlag, value: _c.bool) ---

    /// gets the flags specific to the Revolute Joint.
    ///
    /// the joint flags
    @(link_name = "PxRevoluteJoint_getRevoluteJointFlags")
    revolute_joint_get_revolute_joint_flags :: proc(self_: ^RevoluteJoint) -> RevoluteJointFlags_Set ---

    /// Returns string name of PxRevoluteJoint, used for serialization
    @(link_name = "PxRevoluteJoint_getConcreteTypeName")
    revolute_joint_get_concrete_type_name :: proc(self_: ^RevoluteJoint) -> cstring ---

    /// Create a spherical joint.
    @(link_name = "phys_PxSphericalJointCreate")
    spherical_joint_create :: proc(physics: ^Physics, actor0: ^RigidActor, #by_ptr localFrame0: Transform, actor1: ^RigidActor, #by_ptr localFrame1: Transform) -> ^SphericalJoint ---

    /// Set the limit cone.
    ///
    /// If enabled, the limit cone will constrain the angular movement of the joint to lie
    /// within an elliptical cone.
    ///
    /// the limit cone
    @(link_name = "PxSphericalJoint_getLimitCone")
    spherical_joint_get_limit_cone :: proc(self_: ^SphericalJoint) -> JointLimitCone ---

    /// Get the limit cone.
    @(link_name = "PxSphericalJoint_setLimitCone_mut")
    spherical_joint_set_limit_cone_mut :: proc(self_: ^SphericalJoint, #by_ptr limit: JointLimitCone) ---

    /// get the swing angle of the joint from the Y axis
    @(link_name = "PxSphericalJoint_getSwingYAngle")
    spherical_joint_get_swing_y_angle :: proc(self_: ^SphericalJoint) -> _c.float ---

    /// get the swing angle of the joint from the Z axis
    @(link_name = "PxSphericalJoint_getSwingZAngle")
    spherical_joint_get_swing_z_angle :: proc(self_: ^SphericalJoint) -> _c.float ---

    /// Set the flags specific to the Spherical Joint.
    ///
    /// Default
    /// PxSphericalJointFlags(0)
    @(link_name = "PxSphericalJoint_setSphericalJointFlags_mut")
    spherical_joint_set_spherical_joint_flags_mut :: proc(self_: ^SphericalJoint, flags: SphericalJointFlags_Set) ---

    /// Set a single flag specific to a Spherical Joint to true or false.
    @(link_name = "PxSphericalJoint_setSphericalJointFlag_mut")
    spherical_joint_set_spherical_joint_flag_mut :: proc(self_: ^SphericalJoint, flag: SphericalJointFlag, value: _c.bool) ---

    /// Get the flags specific to the Spherical Joint.
    ///
    /// the joint flags
    @(link_name = "PxSphericalJoint_getSphericalJointFlags")
    spherical_joint_get_spherical_joint_flags :: proc(self_: ^SphericalJoint) -> SphericalJointFlags_Set ---

    /// Returns string name of PxSphericalJoint, used for serialization
    @(link_name = "PxSphericalJoint_getConcreteTypeName")
    spherical_joint_get_concrete_type_name :: proc(self_: ^SphericalJoint) -> cstring ---

    /// Create a D6 joint.
    @(link_name = "phys_PxD6JointCreate")
    d6_joint_create :: proc(physics: ^Physics, actor0: ^RigidActor, #by_ptr localFrame0: Transform, actor1: ^RigidActor, #by_ptr localFrame1: Transform) -> ^D6Joint ---

    /// default constructor for PxD6JointDrive.
    @(link_name = "PxD6JointDrive_new")
    d6_joint_drive_new :: proc() -> D6JointDrive ---

    /// constructor a PxD6JointDrive.
    @(link_name = "PxD6JointDrive_new_1")
    d6_joint_drive_new_1 :: proc(driveStiffness: _c.float, driveDamping: _c.float, driveForceLimit: _c.float, isAcceleration: _c.bool) -> D6JointDrive ---

    /// returns true if the drive is valid
    @(link_name = "PxD6JointDrive_isValid")
    d6_joint_drive_is_valid :: proc(self_: ^D6JointDrive) -> _c.bool ---

    /// Set the motion type around the specified axis.
    ///
    /// Each axis may independently specify that the degree of freedom is locked (blocking relative movement
    /// along or around this axis), limited by the corresponding limit, or free.
    ///
    /// Default:
    /// all degrees of freedom are locked
    @(link_name = "PxD6Joint_setMotion_mut")
    d6_joint_set_motion_mut :: proc(self_: ^D6Joint, axis: D6Axis, type: D6Motion) ---

    /// Get the motion type around the specified axis.
    ///
    /// the motion type around the specified axis
    @(link_name = "PxD6Joint_getMotion")
    d6_joint_get_motion :: proc(self_: ^D6Joint, axis: D6Axis) -> D6Motion ---

    /// get the twist angle of the joint, in the range (-2*Pi, 2*Pi]
    @(link_name = "PxD6Joint_getTwistAngle")
    d6_joint_get_twist_angle :: proc(self_: ^D6Joint) -> _c.float ---

    /// get the swing angle of the joint from the Y axis
    @(link_name = "PxD6Joint_getSwingYAngle")
    d6_joint_get_swing_y_angle :: proc(self_: ^D6Joint) -> _c.float ---

    /// get the swing angle of the joint from the Z axis
    @(link_name = "PxD6Joint_getSwingZAngle")
    d6_joint_get_swing_z_angle :: proc(self_: ^D6Joint) -> _c.float ---

    /// Set the distance limit for the joint.
    ///
    /// A single limit constraints all linear limited degrees of freedom, forming a linear, circular
    /// or spherical constraint on motion depending on the number of limited degrees. This is similar
    /// to a distance limit.
    @(link_name = "PxD6Joint_setDistanceLimit_mut")
    d6_joint_set_distance_limit_mut :: proc(self_: ^D6Joint, #by_ptr limit: JointLinearLimit) ---

    /// Get the distance limit for the joint.
    ///
    /// the distance limit structure
    @(link_name = "PxD6Joint_getDistanceLimit")
    d6_joint_get_distance_limit :: proc(self_: ^D6Joint) -> JointLinearLimit ---

    /// Set the linear limit for a given linear axis.
    ///
    /// This function extends the previous setDistanceLimit call with the following features:
    /// - there can be a different limit for each linear axis
    /// - each limit is defined by two values, i.e. it can now be asymmetric
    ///
    /// This can be used to create prismatic joints similar to PxPrismaticJoint, or point-in-quad joints,
    /// or point-in-box joints.
    @(link_name = "PxD6Joint_setLinearLimit_mut")
    d6_joint_set_linear_limit_mut :: proc(self_: ^D6Joint, axis: D6Axis, #by_ptr limit: JointLinearLimitPair) ---

    /// Get the linear limit for a given linear axis.
    ///
    /// the linear limit pair structure from desired axis
    @(link_name = "PxD6Joint_getLinearLimit")
    d6_joint_get_linear_limit :: proc(self_: ^D6Joint, axis: D6Axis) -> JointLinearLimitPair ---

    /// Set the twist limit for the joint.
    ///
    /// The twist limit controls the range of motion around the twist axis.
    ///
    /// The limit angle range is (-2*Pi, 2*Pi).
    @(link_name = "PxD6Joint_setTwistLimit_mut")
    d6_joint_set_twist_limit_mut :: proc(self_: ^D6Joint, #by_ptr limit: JointAngularLimitPair) ---

    /// Get the twist limit for the joint.
    ///
    /// the twist limit structure
    @(link_name = "PxD6Joint_getTwistLimit")
    d6_joint_get_twist_limit :: proc(self_: ^D6Joint) -> JointAngularLimitPair ---

    /// Set the swing cone limit for the joint.
    ///
    /// The cone limit is used if either or both swing axes are limited. The extents are
    /// symmetrical and measured in the frame of the parent. If only one swing degree of freedom
    /// is limited, the corresponding value from the cone limit defines the limit range.
    @(link_name = "PxD6Joint_setSwingLimit_mut")
    d6_joint_set_swing_limit_mut :: proc(self_: ^D6Joint, #by_ptr limit: JointLimitCone) ---

    /// Get the cone limit for the joint.
    ///
    /// the swing limit structure
    @(link_name = "PxD6Joint_getSwingLimit")
    d6_joint_get_swing_limit :: proc(self_: ^D6Joint) -> JointLimitCone ---

    /// Set a pyramidal swing limit for the joint.
    ///
    /// The pyramid limits will only be used in the following cases:
    /// - both swing Y and Z are limited. The limit shape is then a pyramid.
    /// - Y is limited and Z is locked, or vice versa. The limit shape is an asymmetric angular section, similar to
    /// what is supported for the twist axis.
    /// The remaining cases (Y limited and Z is free, or vice versa) are not supported.
    @(link_name = "PxD6Joint_setPyramidSwingLimit_mut")
    d6_joint_set_pyramid_swing_limit_mut :: proc(self_: ^D6Joint, #by_ptr limit: JointLimitPyramid) ---

    /// Get the pyramidal swing limit for the joint.
    ///
    /// the swing limit structure
    @(link_name = "PxD6Joint_getPyramidSwingLimit")
    d6_joint_get_pyramid_swing_limit :: proc(self_: ^D6Joint) -> JointLimitPyramid ---

    /// Set the drive parameters for the specified drive type.
    ///
    /// Default
    /// The default drive spring and damping values are zero, the force limit is zero, and no flags are set.
    @(link_name = "PxD6Joint_setDrive_mut")
    d6_joint_set_drive_mut :: proc(self_: ^D6Joint, index: D6Drive, #by_ptr drive: D6JointDrive) ---

    /// Get the drive parameters for the specified drive type.
    @(link_name = "PxD6Joint_getDrive")
    d6_joint_get_drive :: proc(self_: ^D6Joint, index: D6Drive) -> D6JointDrive ---

    /// Set the drive goal pose
    ///
    /// The goal is relative to the constraint frame of actor[0]
    ///
    /// Default
    /// the identity transform
    @(link_name = "PxD6Joint_setDrivePosition_mut")
    d6_joint_set_drive_position_mut :: proc(self_: ^D6Joint, #by_ptr pose: Transform, autowake: _c.bool) ---

    /// Get the drive goal pose.
    @(link_name = "PxD6Joint_getDrivePosition")
    d6_joint_get_drive_position :: proc(self_: ^D6Joint) -> Transform ---

    /// Set the target goal velocity for drive.
    ///
    /// The velocity is measured in the constraint frame of actor[0]
    @(link_name = "PxD6Joint_setDriveVelocity_mut")
    d6_joint_set_drive_velocity_mut :: proc(self_: ^D6Joint, #by_ptr linear: Vec3, #by_ptr angular: Vec3, autowake: _c.bool) ---

    /// Get the target goal velocity for joint drive.
    @(link_name = "PxD6Joint_getDriveVelocity")
    d6_joint_get_drive_velocity :: proc(self_: ^D6Joint, linear: ^Vec3, angular: ^Vec3) ---

    /// Set the linear tolerance threshold for projection. Projection is enabled if PxConstraintFlag::ePROJECTION
    /// is set for the joint.
    ///
    /// If the joint separates by more than this distance along its locked degrees of freedom, the solver
    /// will move the bodies to close the distance.
    ///
    /// Setting a very small tolerance may result in simulation jitter or other artifacts.
    ///
    /// Sometimes it is not possible to project (for example when the joints form a cycle).
    ///
    /// Range:
    /// [0, PX_MAX_F32)
    /// Default:
    /// 1e10f
    @(link_name = "PxD6Joint_setProjectionLinearTolerance_mut")
    d6_joint_set_projection_linear_tolerance_mut :: proc(self_: ^D6Joint, tolerance: _c.float) ---

    /// Get the linear tolerance threshold for projection.
    ///
    /// the linear tolerance threshold
    @(link_name = "PxD6Joint_getProjectionLinearTolerance")
    d6_joint_get_projection_linear_tolerance :: proc(self_: ^D6Joint) -> _c.float ---

    /// Set the angular tolerance threshold for projection. Projection is enabled if
    /// PxConstraintFlag::ePROJECTION is set for the joint.
    ///
    /// If the joint deviates by more than this angle around its locked angular degrees of freedom,
    /// the solver will move the bodies to close the angle.
    ///
    /// Setting a very small tolerance may result in simulation jitter or other artifacts.
    ///
    /// Sometimes it is not possible to project (for example when the joints form a cycle).
    ///
    /// Range:
    /// [0,Pi]
    /// Default:
    /// Pi
    ///
    /// Angular projection is implemented only for the case of two or three locked angular degrees of freedom.
    @(link_name = "PxD6Joint_setProjectionAngularTolerance_mut")
    d6_joint_set_projection_angular_tolerance_mut :: proc(self_: ^D6Joint, tolerance: _c.float) ---

    /// Get the angular tolerance threshold for projection.
    ///
    /// tolerance the angular tolerance threshold in radians
    @(link_name = "PxD6Joint_getProjectionAngularTolerance")
    d6_joint_get_projection_angular_tolerance :: proc(self_: ^D6Joint) -> _c.float ---

    /// Returns string name of PxD6Joint, used for serialization
    @(link_name = "PxD6Joint_getConcreteTypeName")
    d6_joint_get_concrete_type_name :: proc(self_: ^D6Joint) -> cstring ---

    /// Create a gear Joint.
    @(link_name = "phys_PxGearJointCreate")
    gear_joint_create :: proc(physics: ^Physics, actor0: ^RigidActor, #by_ptr localFrame0: Transform, actor1: ^RigidActor, #by_ptr localFrame1: Transform) -> ^GearJoint ---

    /// Set the hinge/revolute joints connected by the gear joint.
    ///
    /// The passed joints can be either PxRevoluteJoint, PxD6Joint or PxArticulationJointReducedCoordinate.
    /// The joints must define degrees of freedom around the twist axis. They cannot be null.
    ///
    /// Note that these joints are only used to compute the positional error correction term,
    /// used to adjust potential drift between jointed actors. The gear joint can run without
    /// calling this function, but in that case some visible overlap may develop over time between
    /// the teeth of the gear meshes.
    ///
    /// Calling this function resets the internal positional error correction term.
    ///
    /// true if success
    @(link_name = "PxGearJoint_setHinges_mut")
    gear_joint_set_hinges_mut :: proc(self_: ^GearJoint, hinge0: ^Base, hinge1: ^Base) -> _c.bool ---

    /// Set the desired gear ratio.
    ///
    /// For two gears with n0 and n1 teeth respectively, the gear ratio is n0/n1.
    ///
    /// You may need to use a negative gear ratio if the joint frames of involved actors are not oriented in the same direction.
    ///
    /// Calling this function resets the internal positional error correction term.
    @(link_name = "PxGearJoint_setGearRatio_mut")
    gear_joint_set_gear_ratio_mut :: proc(self_: ^GearJoint, ratio: _c.float) ---

    /// Get the gear ratio.
    ///
    /// Current ratio
    @(link_name = "PxGearJoint_getGearRatio")
    gear_joint_get_gear_ratio :: proc(self_: ^GearJoint) -> _c.float ---

    @(link_name = "PxGearJoint_getConcreteTypeName")
    gear_joint_get_concrete_type_name :: proc(self_: ^GearJoint) -> cstring ---

    /// Create a rack
    /// &
    /// pinion Joint.
    @(link_name = "phys_PxRackAndPinionJointCreate")
    rack_and_pinion_joint_create :: proc(physics: ^Physics, actor0: ^RigidActor, #by_ptr localFrame0: Transform, actor1: ^RigidActor, #by_ptr localFrame1: Transform) -> ^RackAndPinionJoint ---

    /// Set the hinge
    /// &
    /// prismatic joints connected by the rack
    /// &
    /// pinion joint.
    ///
    /// The passed hinge joint can be either PxRevoluteJoint, PxD6Joint or PxArticulationJointReducedCoordinate. It cannot be null.
    /// The passed prismatic joint can be either PxPrismaticJoint or PxD6Joint. It cannot be null.
    ///
    /// Note that these joints are only used to compute the positional error correction term,
    /// used to adjust potential drift between jointed actors. The rack
    /// &
    /// pinion joint can run without
    /// calling this function, but in that case some visible overlap may develop over time between
    /// the teeth of the rack
    /// &
    /// pinion meshes.
    ///
    /// Calling this function resets the internal positional error correction term.
    ///
    /// true if success
    @(link_name = "PxRackAndPinionJoint_setJoints_mut")
    rack_and_pinion_joint_set_joints_mut :: proc(self_: ^RackAndPinionJoint, hinge: ^Base, prismatic: ^Base) -> _c.bool ---

    /// Set the desired ratio directly.
    ///
    /// You may need to use a negative gear ratio if the joint frames of involved actors are not oriented in the same direction.
    ///
    /// Calling this function resets the internal positional error correction term.
    @(link_name = "PxRackAndPinionJoint_setRatio_mut")
    rack_and_pinion_joint_set_ratio_mut :: proc(self_: ^RackAndPinionJoint, ratio: _c.float) ---

    /// Get the ratio.
    ///
    /// Current ratio
    @(link_name = "PxRackAndPinionJoint_getRatio")
    rack_and_pinion_joint_get_ratio :: proc(self_: ^RackAndPinionJoint) -> _c.float ---

    /// Set the desired ratio indirectly.
    ///
    /// This is a simple helper function that computes the ratio from passed data:
    ///
    /// ratio = (PI*2*nbRackTeeth)/(rackLength*nbPinionTeeth)
    ///
    /// Calling this function resets the internal positional error correction term.
    ///
    /// true if success
    @(link_name = "PxRackAndPinionJoint_setData_mut")
    rack_and_pinion_joint_set_data_mut :: proc(self_: ^RackAndPinionJoint, nbRackTeeth: _c.uint32_t, nbPinionTeeth: _c.uint32_t, rackLength: _c.float) -> _c.bool ---

    @(link_name = "PxRackAndPinionJoint_getConcreteTypeName")
    rack_and_pinion_joint_get_concrete_type_name :: proc(self_: ^RackAndPinionJoint) -> cstring ---

    @(link_name = "PxGroupsMask_new_alloc")
    groups_mask_new_alloc :: proc() -> ^GroupsMask ---

    @(link_name = "PxGroupsMask_delete")
    groups_mask_delete :: proc(self_: ^GroupsMask) ---

    /// Implementation of a simple filter shader that emulates PhysX 2.8.x filtering
    ///
    /// This shader provides the following logic:
    ///
    /// If one of the two filter objects is a trigger, the pair is acccepted and [`PxPairFlag::eTRIGGER_DEFAULT`] will be used for trigger reports
    ///
    /// Else, if the filter mask logic (see further below) discards the pair it will be suppressed ([`PxFilterFlag::eSUPPRESS`])
    ///
    /// Else, the pair gets accepted and collision response gets enabled ([`PxPairFlag::eCONTACT_DEFAULT`])
    ///
    /// Filter mask logic:
    /// Given the two [`PxFilterData`] structures fd0 and fd1 of two collision objects, the pair passes the filter if the following
    /// conditions are met:
    ///
    /// 1) Collision groups of the pair are enabled
    /// 2) Collision filtering equation is satisfied
    @(link_name = "phys_PxDefaultSimulationFilterShader")
    default_simulation_filter_shader :: proc(attributes0: _c.uint32_t, filterData0: FilterData, attributes1: _c.uint32_t, filterData1: FilterData, pairFlags: ^PairFlags_Set, constantBlock: rawptr, constantBlockSize: _c.uint32_t) -> FilterFlags_Set ---

    /// Determines if collision detection is performed between a pair of groups
    ///
    /// Collision group is an integer between 0 and 31.
    ///
    /// True if the groups could collide
    @(link_name = "phys_PxGetGroupCollisionFlag")
    get_group_collision_flag :: proc(group1: _c.uint16_t, group2: _c.uint16_t) -> _c.bool ---

    /// Specifies if collision should be performed by a pair of groups
    ///
    /// Collision group is an integer between 0 and 31.
    @(link_name = "phys_PxSetGroupCollisionFlag")
    set_group_collision_flag :: proc(group1: _c.uint16_t, group2: _c.uint16_t, enable: _c.bool) ---

    /// Retrieves the value set with PxSetGroup()
    ///
    /// Collision group is an integer between 0 and 31.
    ///
    /// The collision group this actor belongs to
    @(link_name = "phys_PxGetGroup")
    get_group :: proc(actor: ^Actor) -> _c.uint16_t ---

    /// Sets which collision group this actor is part of
    ///
    /// Collision group is an integer between 0 and 31.
    @(link_name = "phys_PxSetGroup")
    set_group :: proc(actor: ^Actor, collisionGroup: _c.uint16_t) ---

    /// Retrieves filtering operation. See comments for PxGroupsMask
    @(link_name = "phys_PxGetFilterOps")
    get_filter_ops :: proc(op0: ^FilterOp, op1: ^FilterOp, op2: ^FilterOp) ---

    /// Setups filtering operations. See comments for PxGroupsMask
    @(link_name = "phys_PxSetFilterOps")
    set_filter_ops :: proc(#by_ptr op0: FilterOp, #by_ptr op1: FilterOp, #by_ptr op2: FilterOp) ---

    /// Retrieves filtering's boolean value. See comments for PxGroupsMask
    ///
    /// flag Boolean value for filter.
    @(link_name = "phys_PxGetFilterBool")
    get_filter_bool :: proc() -> _c.bool ---

    /// Setups filtering's boolean value. See comments for PxGroupsMask
    @(link_name = "phys_PxSetFilterBool")
    set_filter_bool :: proc(enable: _c.bool) ---

    /// Gets filtering constant K0 and K1. See comments for PxGroupsMask
    @(link_name = "phys_PxGetFilterConstants")
    get_filter_constants :: proc(c0: ^GroupsMask, c1: ^GroupsMask) ---

    /// Setups filtering's K0 and K1 value. See comments for PxGroupsMask
    @(link_name = "phys_PxSetFilterConstants")
    set_filter_constants :: proc(#by_ptr c0: GroupsMask, #by_ptr c1: GroupsMask) ---

    /// Gets 64-bit mask used for collision filtering. See comments for PxGroupsMask
    ///
    /// The group mask for the actor.
    @(link_name = "phys_PxGetGroupsMask")
    get_groups_mask :: proc(actor: ^Actor) -> GroupsMask ---

    /// Sets 64-bit mask used for collision filtering. See comments for PxGroupsMask
    @(link_name = "phys_PxSetGroupsMask")
    set_groups_mask :: proc(actor: ^Actor, #by_ptr mask: GroupsMask) ---

    @(link_name = "PxDefaultErrorCallback_new_alloc")
    default_error_callback_new_alloc :: proc() -> ^DefaultErrorCallback ---

    @(link_name = "PxDefaultErrorCallback_delete")
    default_error_callback_delete :: proc(self_: ^DefaultErrorCallback) ---

    @(link_name = "PxDefaultErrorCallback_reportError_mut")
    default_error_callback_report_error_mut :: proc(self_: ^DefaultErrorCallback, code: ErrorCode, message: cstring, file: cstring, line: _c.int32_t) ---

    /// Creates a new shape with default properties and a list of materials and adds it to the list of shapes of this actor.
    ///
    /// This is equivalent to the following
    ///
    /// ```cpp
    /// // reference count is 1
    /// PxShape* shape(...) = PxGetPhysics().createShape(...);
    /// // increments reference count
    /// actor->attachShape(shape);
    /// // releases user reference, leaving reference count at 1
    /// shape->release();
    /// ```
    ///
    /// As a consequence, detachShape() will result in the release of the last reference, and the shape will be deleted.
    ///
    /// The default shape flags to be set are: eVISUALIZATION, eSIMULATION_SHAPE, eSCENE_QUERY_SHAPE (see [`PxShapeFlag`]).
    /// Triangle mesh, heightfield or plane geometry shapes configured as eSIMULATION_SHAPE are not supported for
    /// non-kinematic PxRigidDynamic instances.
    ///
    /// Creating compounds with a very large number of shapes may adversely affect performance and stability.
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the actor up automatically.
    ///
    /// The newly created shape.
    @(link_name = "PxRigidActorExt_createExclusiveShape")
    rigid_actor_ext_create_exclusive_shape :: proc(actor: ^RigidActor, geometry: ^Geometry, materials: [^]^Material, materialCount: _c.uint16_t, shapeFlags: ShapeFlags_Set) -> ^Shape ---

    /// Creates a new shape with default properties and a single material adds it to the list of shapes of this actor.
    ///
    /// This is equivalent to the following
    ///
    /// ```cpp
    /// // reference count is 1
    /// PxShape* shape(...) = PxGetPhysics().createShape(...);
    /// // increments reference count
    /// actor->attachShape(shape);
    /// // releases user reference, leaving reference count at 1
    /// shape->release();
    /// ```
    ///
    /// As a consequence, detachShape() will result in the release of the last reference, and the shape will be deleted.
    ///
    /// The default shape flags to be set are: eVISUALIZATION, eSIMULATION_SHAPE, eSCENE_QUERY_SHAPE (see [`PxShapeFlag`]).
    /// Triangle mesh, heightfield or plane geometry shapes configured as eSIMULATION_SHAPE are not supported for
    /// non-kinematic PxRigidDynamic instances.
    ///
    /// Creating compounds with a very large number of shapes may adversely affect performance and stability.
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the actor up automatically.
    ///
    /// The newly created shape.
    @(link_name = "PxRigidActorExt_createExclusiveShape_1")
    rigid_actor_ext_create_exclusive_shape_1 :: proc(actor: ^RigidActor, geometry: ^Geometry, #by_ptr material: Material, shapeFlags: ShapeFlags_Set) -> ^Shape ---

    /// Gets a list of bounds based on shapes in rigid actor. This list can be used to cook/create
    /// bounding volume hierarchy though PxCooking API.
    @(link_name = "PxRigidActorExt_getRigidActorShapeLocalBoundsList")
    rigid_actor_ext_get_rigid_actor_shape_local_bounds_list :: proc(actor: ^RigidActor, numBounds: ^_c.uint32_t) -> ^Bounds3 ---

    /// Convenience function to create a PxBVH object from a PxRigidActor.
    ///
    /// The computed PxBVH can then be used in PxScene::addActor() or PxAggregate::addActor().
    /// After adding the actor
    /// &
    /// BVH to the scene/aggregate, release the PxBVH object by calling PxBVH::release().
    ///
    /// The PxBVH for this actor.
    @(link_name = "PxRigidActorExt_createBVHFromActor")
    rigid_actor_ext_create_b_v_h_from_actor :: proc(physics: ^Physics, actor: ^RigidActor) -> ^BVH ---

    /// Default constructor.
    @(link_name = "PxMassProperties_new")
    mass_properties_new :: proc() -> MassProperties ---

    /// Construct from individual elements.
    @(link_name = "PxMassProperties_new_1")
    mass_properties_new_1 :: proc(m: _c.float, #by_ptr inertiaT: Mat33, #by_ptr com: Vec3) -> MassProperties ---

    /// Compute mass properties based on a provided geometry structure.
    ///
    /// This constructor assumes the geometry has a density of 1. Mass and inertia tensor scale linearly with density.
    @(link_name = "PxMassProperties_new_2")
    mass_properties_new_2 :: proc(geometry: ^Geometry) -> MassProperties ---

    /// Translate the center of mass by a given vector and adjust the inertia tensor accordingly.
    @(link_name = "PxMassProperties_translate_mut")
    mass_properties_translate_mut :: proc(self_: ^MassProperties, #by_ptr t: Vec3) ---

    /// Get the entries of the diagonalized inertia tensor and the corresponding reference rotation.
    ///
    /// The entries of the diagonalized inertia tensor.
    @(link_name = "PxMassProperties_getMassSpaceInertia")
    mass_properties_get_mass_space_inertia :: proc(#by_ptr inertia: Mat33, massFrame: ^Quat) -> Vec3 ---

    /// Translate an inertia tensor using the parallel axis theorem
    ///
    /// The translated inertia tensor.
    @(link_name = "PxMassProperties_translateInertia")
    mass_properties_translate_inertia :: proc(#by_ptr inertia: Mat33, mass: _c.float, #by_ptr t: Vec3) -> Mat33 ---

    /// Rotate an inertia tensor around the center of mass
    ///
    /// The rotated inertia tensor.
    @(link_name = "PxMassProperties_rotateInertia")
    mass_properties_rotate_inertia :: proc(#by_ptr inertia: Mat33, #by_ptr q: Quat) -> Mat33 ---

    /// Non-uniform scaling of the inertia tensor
    ///
    /// The scaled inertia tensor.
    @(link_name = "PxMassProperties_scaleInertia")
    mass_properties_scale_inertia :: proc(#by_ptr inertia: Mat33, #by_ptr scaleRotation: Quat, #by_ptr scale: Vec3) -> Mat33 ---

    /// Sum up individual mass properties.
    ///
    /// The summed up mass properties.
    @(link_name = "PxMassProperties_sum")
    mass_properties_sum :: proc(props: ^MassProperties, transforms: ^Transform, count: _c.uint32_t) -> MassProperties ---

    /// Computation of mass properties for a rigid body actor
    ///
    /// To simulate a dynamic rigid actor, the SDK needs a mass and an inertia tensor.
    ///
    /// This method offers functionality to compute the necessary mass and inertia properties based on the shapes declared in
    /// the PxRigidBody descriptor and some additionally specified parameters. For each shape, the shape geometry,
    /// the shape positioning within the actor and the specified shape density are used to compute the body's mass and
    /// inertia properties.
    ///
    /// Shapes without PxShapeFlag::eSIMULATION_SHAPE set are ignored unless includeNonSimShapes is true.
    /// Shapes with plane, triangle mesh or heightfield geometry and PxShapeFlag::eSIMULATION_SHAPE set are not allowed for PxRigidBody collision.
    ///
    /// This method will set the mass, center of mass, and inertia tensor
    ///
    /// if no collision shapes are found, the inertia tensor is set to (1,1,1) and the mass to 1
    ///
    /// if massLocalPose is non-NULL, the rigid body's center of mass parameter  will be set
    /// to the user provided value (massLocalPose) and the inertia tensor will be resolved at that point.
    ///
    /// If all shapes of the actor have the same density then the overloaded method updateMassAndInertia() with a single density parameter can be used instead.
    ///
    /// Boolean. True on success else false.
    @(link_name = "PxRigidBodyExt_updateMassAndInertia")
    rigid_body_ext_update_mass_and_inertia :: proc(body: ^RigidBody, shapeDensities: ^_c.float, shapeDensityCount: _c.uint32_t, massLocalPose: ^Vec3, includeNonSimShapes: _c.bool) -> _c.bool ---

    /// Computation of mass properties for a rigid body actor
    ///
    /// See previous method for details.
    ///
    /// Boolean. True on success else false.
    @(link_name = "PxRigidBodyExt_updateMassAndInertia_1")
    rigid_body_ext_update_mass_and_inertia_1 :: proc(body: ^RigidBody, density: _c.float, massLocalPose: ^Vec3, includeNonSimShapes: _c.bool) -> _c.bool ---

    /// Computation of mass properties for a rigid body actor
    ///
    /// This method sets the mass, inertia and center of mass of a rigid body. The mass is set to the sum of all user-supplied
    /// shape mass values, and the inertia and center of mass are computed according to the rigid body's shapes and the per shape mass input values.
    ///
    /// If no collision shapes are found, the inertia tensor is set to (1,1,1)
    ///
    /// If a single mass value should be used for the actor as a whole then the overloaded method setMassAndUpdateInertia() with a single mass parameter can be used instead.
    ///
    /// Boolean. True on success else false.
    @(link_name = "PxRigidBodyExt_setMassAndUpdateInertia")
    rigid_body_ext_set_mass_and_update_inertia :: proc(body: ^RigidBody, shapeMasses: ^_c.float, shapeMassCount: _c.uint32_t, massLocalPose: ^Vec3, includeNonSimShapes: _c.bool) -> _c.bool ---

    /// Computation of mass properties for a rigid body actor
    ///
    /// This method sets the mass, inertia and center of mass of a rigid body. The mass is set to the user-supplied
    /// value, and the inertia and center of mass are computed according to the rigid body's shapes and the input mass.
    ///
    /// If no collision shapes are found, the inertia tensor is set to (1,1,1)
    ///
    /// Boolean. True on success else false.
    @(link_name = "PxRigidBodyExt_setMassAndUpdateInertia_1")
    rigid_body_ext_set_mass_and_update_inertia_1 :: proc(body: ^RigidBody, mass: _c.float, massLocalPose: ^Vec3, includeNonSimShapes: _c.bool) -> _c.bool ---

    /// Compute the mass, inertia tensor and center of mass from a list of shapes.
    ///
    /// The mass properties from the combined shapes.
    @(link_name = "PxRigidBodyExt_computeMassPropertiesFromShapes")
    rigid_body_ext_compute_mass_properties_from_shapes :: proc(shapes: [^]^Shape, shapeCount: _c.uint32_t) -> MassProperties ---

    /// Applies a force (or impulse) defined in the global coordinate frame, acting at a particular
    /// point in global coordinates, to the actor.
    ///
    /// Note that if the force does not act along the center of mass of the actor, this
    /// will also add the corresponding torque. Because forces are reset at the end of every timestep,
    /// you can maintain a total external force on an object by calling this once every frame.
    ///
    /// if this call is used to apply a force or impulse to an articulation link, only the link is updated, not the entire
    /// articulation
    ///
    /// ::PxForceMode determines if the force is to be conventional or impulsive. Only eFORCE and eIMPULSE are supported, as the
    /// force required to produce a given velocity change or acceleration is underdetermined given only the desired change at a
    /// given point.
    ///
    /// Sleeping:
    /// This call wakes the actor if it is sleeping and the wakeup parameter is true (default).
    @(link_name = "PxRigidBodyExt_addForceAtPos")
    rigid_body_ext_add_force_at_pos :: proc(body: ^RigidBody, #by_ptr force: Vec3, #by_ptr pos: Vec3, mode: ForceMode, wakeup: _c.bool) ---

    /// Applies a force (or impulse) defined in the global coordinate frame, acting at a particular
    /// point in local coordinates, to the actor.
    ///
    /// Note that if the force does not act along the center of mass of the actor, this
    /// will also add the corresponding torque. Because forces are reset at the end of every timestep, you can maintain a
    /// total external force on an object by calling this once every frame.
    ///
    /// if this call is used to apply a force or impulse to an articulation link, only the link is updated, not the entire
    /// articulation
    ///
    /// ::PxForceMode determines if the force is to be conventional or impulsive. Only eFORCE and eIMPULSE are supported, as the
    /// force required to produce a given velocity change or acceleration is underdetermined given only the desired change at a
    /// given point.
    ///
    /// Sleeping:
    /// This call wakes the actor if it is sleeping and the wakeup parameter is true (default).
    @(link_name = "PxRigidBodyExt_addForceAtLocalPos")
    rigid_body_ext_add_force_at_local_pos :: proc(body: ^RigidBody, #by_ptr force: Vec3, #by_ptr pos: Vec3, mode: ForceMode, wakeup: _c.bool) ---

    /// Applies a force (or impulse) defined in the actor local coordinate frame, acting at a
    /// particular point in global coordinates, to the actor.
    ///
    /// Note that if the force does not act along the center of mass of the actor, this
    /// will also add the corresponding torque. Because forces are reset at the end of every timestep, you can maintain a
    /// total external force on an object by calling this once every frame.
    ///
    /// if this call is used to apply a force or impulse to an articulation link, only the link is updated, not the entire
    /// articulation
    ///
    /// ::PxForceMode determines if the force is to be conventional or impulsive. Only eFORCE and eIMPULSE are supported, as the
    /// force required to produce a given velocity change or acceleration is underdetermined given only the desired change at a
    /// given point.
    ///
    /// Sleeping:
    /// This call wakes the actor if it is sleeping and the wakeup parameter is true (default).
    @(link_name = "PxRigidBodyExt_addLocalForceAtPos")
    rigid_body_ext_add_local_force_at_pos :: proc(body: ^RigidBody, #by_ptr force: Vec3, #by_ptr pos: Vec3, mode: ForceMode, wakeup: _c.bool) ---

    /// Applies a force (or impulse) defined in the actor local coordinate frame, acting at a
    /// particular point in local coordinates, to the actor.
    ///
    /// Note that if the force does not act along the center of mass of the actor, this
    /// will also add the corresponding torque. Because forces are reset at the end of every timestep, you can maintain a
    /// total external force on an object by calling this once every frame.
    ///
    /// if this call is used to apply a force or impulse to an articulation link, only the link is updated, not the entire
    /// articulation
    ///
    /// ::PxForceMode determines if the force is to be conventional or impulsive. Only eFORCE and eIMPULSE are supported, as the
    /// force required to produce a given velocity change or acceleration is underdetermined given only the desired change at a
    /// given point.
    ///
    /// Sleeping:
    /// This call wakes the actor if it is sleeping and the wakeup parameter is true (default).
    @(link_name = "PxRigidBodyExt_addLocalForceAtLocalPos")
    rigid_body_ext_add_local_force_at_local_pos :: proc(body: ^RigidBody, #by_ptr force: Vec3, #by_ptr pos: Vec3, mode: ForceMode, wakeup: _c.bool) ---

    /// Computes the velocity of a point given in world coordinates if it were attached to the
    /// specified body and moving with it.
    ///
    /// The velocity of point in the global frame.
    @(link_name = "PxRigidBodyExt_getVelocityAtPos")
    rigid_body_ext_get_velocity_at_pos :: proc(body: ^RigidBody, #by_ptr pos: Vec3) -> Vec3 ---

    /// Computes the velocity of a point given in local coordinates if it were attached to the
    /// specified body and moving with it.
    ///
    /// The velocity of point in the local frame.
    @(link_name = "PxRigidBodyExt_getLocalVelocityAtLocalPos")
    rigid_body_ext_get_local_velocity_at_local_pos :: proc(body: ^RigidBody, #by_ptr pos: Vec3) -> Vec3 ---

    /// Computes the velocity of a point (offset from the origin of the body) given in world coordinates if it were attached to the
    /// specified body and moving with it.
    ///
    /// The velocity of point (offset from the origin of the body) in the global frame.
    @(link_name = "PxRigidBodyExt_getVelocityAtOffset")
    rigid_body_ext_get_velocity_at_offset :: proc(body: ^RigidBody, #by_ptr pos: Vec3) -> Vec3 ---

    /// Compute the change to linear and angular velocity that would occur if an impulsive force and torque were to be applied to a specified rigid body.
    ///
    /// The rigid body is left unaffected unless a subsequent independent call is executed that actually applies the computed changes to velocity and angular velocity.
    ///
    /// if this call is used to determine the velocity delta for an articulation link, only the mass properties of the link are taken into account.
    @(link_name = "PxRigidBodyExt_computeVelocityDeltaFromImpulse")
    rigid_body_ext_compute_velocity_delta_from_impulse :: proc(body: ^RigidBody, #by_ptr impulsiveForce: Vec3, #by_ptr impulsiveTorque: Vec3, deltaLinearVelocity: ^Vec3, deltaAngularVelocity: ^Vec3) ---

    /// Computes the linear and angular velocity change vectors for a given impulse at a world space position taking a mass and inertia scale into account
    ///
    /// This function is useful for extracting the respective linear and angular velocity changes from a contact or joint when the mass/inertia ratios have been adjusted.
    ///
    /// if this call is used to determine the velocity delta for an articulation link, only the mass properties of the link are taken into account.
    @(link_name = "PxRigidBodyExt_computeVelocityDeltaFromImpulse_1")
    rigid_body_ext_compute_velocity_delta_from_impulse_1 :: proc(body: ^RigidBody, #by_ptr globalPose: Transform, #by_ptr point: Vec3, #by_ptr impulse: Vec3, invMassScale: _c.float, invInertiaScale: _c.float, deltaLinearVelocity: ^Vec3, deltaAngularVelocity: ^Vec3) ---

    /// Computes the linear and angular impulse vectors for a given impulse at a world space position taking a mass and inertia scale into account
    ///
    /// This function is useful for extracting the respective linear and angular impulses from a contact or joint when the mass/inertia ratios have been adjusted.
    @(link_name = "PxRigidBodyExt_computeLinearAngularImpulse")
    rigid_body_ext_compute_linear_angular_impulse :: proc(body: ^RigidBody, #by_ptr globalPose: Transform, #by_ptr point: Vec3, #by_ptr impulse: Vec3, invMassScale: _c.float, invInertiaScale: _c.float, linearImpulse: ^Vec3, angularImpulse: ^Vec3) ---

    /// Performs a linear sweep through space with the body's geometry objects.
    ///
    /// Supported geometries are: box, sphere, capsule, convex. Other geometry types will be ignored.
    ///
    /// If eTOUCH is returned from the filter callback, it will trigger an error and the hit will be discarded.
    ///
    /// The function sweeps all shapes attached to a given rigid body through space and reports the nearest
    /// object in the scene which intersects any of of the shapes swept paths.
    /// Information about the closest intersection is written to a [`PxSweepHit`] structure.
    ///
    /// True if a blocking hit was found.
    @(link_name = "PxRigidBodyExt_linearSweepSingle")
    rigid_body_ext_linear_sweep_single :: proc(body: ^RigidBody, scene: ^Scene, #by_ptr unitDir: Vec3, distance: _c.float, outputFlags: HitFlags_Set, closestHit: ^SweepHit, shapeIndex: ^_c.uint32_t, #by_ptr filterData: QueryFilterData, filterCall: ^QueryFilterCallback, cache: ^QueryCache, inflation: _c.float) -> _c.bool ---

    /// Performs a linear sweep through space with the body's geometry objects, returning all overlaps.
    ///
    /// Supported geometries are: box, sphere, capsule, convex. Other geometry types will be ignored.
    ///
    /// This function sweeps all shapes attached to a given rigid body through space and reports all
    /// objects in the scene that intersect any of the shapes' swept paths until there are no more objects to report
    /// or a blocking hit is encountered.
    ///
    /// the number of touching hits. If overflow is set to true, the results are incomplete. In case of overflow there are also no guarantees that all touching hits returned are closer than the blocking hit.
    @(link_name = "PxRigidBodyExt_linearSweepMultiple")
    rigid_body_ext_linear_sweep_multiple :: proc(body: ^RigidBody, scene: ^Scene, #by_ptr unitDir: Vec3, distance: _c.float, outputFlags: HitFlags_Set, touchHitBuffer: ^SweepHit, touchHitShapeIndices: ^_c.uint32_t, touchHitBufferSize: _c.uint32_t, block: ^SweepHit, blockingShapeIndex: ^_c.int32_t, overflow: ^_c.bool, #by_ptr filterData: QueryFilterData, filterCall: ^QueryFilterCallback, cache: ^QueryCache, inflation: _c.float) -> _c.uint32_t ---

    /// Retrieves the world space pose of the shape.
    ///
    /// Global pose of shape.
    @(link_name = "PxShapeExt_getGlobalPose")
    shape_ext_get_global_pose :: proc(#by_ptr shape: Shape, actor: ^RigidActor) -> Transform ---

    /// Raycast test against the shape.
    ///
    /// Number of hits between the ray and the shape
    @(link_name = "PxShapeExt_raycast")
    shape_ext_raycast :: proc(#by_ptr shape: Shape, actor: ^RigidActor, #by_ptr rayOrigin: Vec3, #by_ptr rayDir: Vec3, maxDist: _c.float, hitFlags: HitFlags_Set, maxHits: _c.uint32_t, rayHits: ^RaycastHit) -> _c.uint32_t ---

    /// Test overlap between the shape and a geometry object
    ///
    /// True if the shape overlaps the geometry object
    @(link_name = "PxShapeExt_overlap")
    shape_ext_overlap :: proc(#by_ptr shape: Shape, actor: ^RigidActor, otherGeom: ^Geometry, #by_ptr otherGeomPose: Transform) -> _c.bool ---

    /// Sweep a geometry object against the shape.
    ///
    /// Currently only box, sphere, capsule and convex mesh shapes are supported, i.e. the swept geometry object must be one of those types.
    ///
    /// True if the swept geometry object hits the shape
    @(link_name = "PxShapeExt_sweep")
    shape_ext_sweep :: proc(#by_ptr shape: Shape, actor: ^RigidActor, #by_ptr unitDir: Vec3, distance: _c.float, otherGeom: ^Geometry, #by_ptr otherGeomPose: Transform, sweepHit: ^SweepHit, hitFlags: HitFlags_Set) -> _c.bool ---

    /// Retrieves the axis aligned bounding box enclosing the shape.
    ///
    /// The shape's bounding box.
    @(link_name = "PxShapeExt_getWorldBounds")
    shape_ext_get_world_bounds :: proc(#by_ptr shape: Shape, actor: ^RigidActor, inflation: _c.float) -> Bounds3 ---

    @(link_name = "PxMeshOverlapUtil_new_alloc")
    mesh_overlap_util_new_alloc :: proc() -> ^MeshOverlapUtil ---

    @(link_name = "PxMeshOverlapUtil_delete")
    mesh_overlap_util_delete :: proc(self_: ^MeshOverlapUtil) ---

    /// Find the mesh triangles which touch the specified geometry object.
    ///
    /// Number of overlaps found. Triangle indices can then be accessed through the [`getResults`]() function.
    @(link_name = "PxMeshOverlapUtil_findOverlap_mut")
    mesh_overlap_util_find_overlap_mut :: proc(self_: ^MeshOverlapUtil, geom: ^Geometry, #by_ptr geomPose: Transform, #by_ptr meshGeom: TriangleMeshGeometry, #by_ptr meshPose: Transform) -> _c.uint32_t ---

    /// Find the height field triangles which touch the specified geometry object.
    ///
    /// Number of overlaps found. Triangle indices can then be accessed through the [`getResults`]() function.
    @(link_name = "PxMeshOverlapUtil_findOverlap_mut_1")
    mesh_overlap_util_find_overlap_mut_1 :: proc(self_: ^MeshOverlapUtil, geom: ^Geometry, #by_ptr geomPose: Transform, #by_ptr hfGeom: HeightFieldGeometry, #by_ptr hfPose: Transform) -> _c.uint32_t ---

    /// Retrieves array of triangle indices after a findOverlap call.
    ///
    /// Indices of touched triangles
    @(link_name = "PxMeshOverlapUtil_getResults")
    mesh_overlap_util_get_results :: proc(self_: ^MeshOverlapUtil) -> [^]_c.uint32_t ---

    /// Retrieves number of triangle indices after a findOverlap call.
    ///
    /// Number of touched triangles
    @(link_name = "PxMeshOverlapUtil_getNbResults")
    mesh_overlap_util_get_nb_results :: proc(self_: ^MeshOverlapUtil) -> _c.uint32_t ---

    /// Computes an approximate minimum translational distance (MTD) between a geometry object and a mesh.
    ///
    /// This iterative function computes an approximate vector that can be used to depenetrate a geom object
    /// from a triangle mesh. Returned depenetration vector should be applied to 'geom', to get out of the mesh.
    ///
    /// The function works best when the amount of overlap between the geom object and the mesh is small. If the
    /// geom object's center goes inside the mesh, backface culling usually kicks in, no overlap is detected,
    /// and the function does not compute an MTD vector.
    ///
    /// The function early exits if no overlap is detected after a depenetration attempt. This means that if
    /// maxIter = N, the code will attempt at most N iterations but it might exit earlier if depenetration has
    /// been successful. Usually N = 4 gives good results.
    ///
    /// True if the MTD has successfully been computed, i.e. if objects do overlap.
    @(link_name = "phys_PxComputeTriangleMeshPenetration")
    compute_triangle_mesh_penetration :: proc(direction: ^Vec3, depth: ^_c.float, geom: ^Geometry, #by_ptr geomPose: Transform, #by_ptr meshGeom: TriangleMeshGeometry, #by_ptr meshPose: Transform, maxIter: _c.uint32_t, usedIter: ^_c.uint32_t) -> _c.bool ---

    /// Computes an approximate minimum translational distance (MTD) between a geometry object and a heightfield.
    ///
    /// This iterative function computes an approximate vector that can be used to depenetrate a geom object
    /// from a heightfield. Returned depenetration vector should be applied to 'geom', to get out of the heightfield.
    ///
    /// The function works best when the amount of overlap between the geom object and the mesh is small. If the
    /// geom object's center goes inside the heightfield, backface culling usually kicks in, no overlap is detected,
    /// and the function does not compute an MTD vector.
    ///
    /// The function early exits if no overlap is detected after a depenetration attempt. This means that if
    /// maxIter = N, the code will attempt at most N iterations but it might exit earlier if depenetration has
    /// been successful. Usually N = 4 gives good results.
    ///
    /// True if the MTD has successfully been computed, i.e. if objects do overlap.
    @(link_name = "phys_PxComputeHeightFieldPenetration")
    compute_height_field_penetration :: proc(direction: ^Vec3, depth: ^_c.float, geom: ^Geometry, #by_ptr geomPose: Transform, #by_ptr heightFieldGeom: HeightFieldGeometry, #by_ptr heightFieldPose: Transform, maxIter: _c.uint32_t, usedIter: ^_c.uint32_t) -> _c.bool ---

    @(link_name = "PxXmlMiscParameter_new")
    xml_misc_parameter_new :: proc() -> XmlMiscParameter ---

    @(link_name = "PxXmlMiscParameter_new_1")
    xml_misc_parameter_new_1 :: proc(inUpVector: ^Vec3, inScale: TolerancesScale) -> XmlMiscParameter ---

    /// Returns whether the collection is serializable with the externalReferences collection.
    ///
    /// Some definitions to explain whether a collection can be serialized or not:
    ///
    /// For definitions of
    /// requires
    /// and
    /// complete
    /// see [`PxSerialization::complete`]
    ///
    /// A serializable object is
    /// subordinate
    /// if it cannot be serialized on its own
    /// The following objects are subordinate:
    /// - articulation links
    /// - articulation joints
    /// - joints
    ///
    /// A collection C can be serialized with external references collection D iff
    /// - C is complete relative to D (no dangling references)
    /// - Every object in D required by an object in C has a valid ID (no unnamed references)
    /// - Every subordinate object in C is required by another object in C (no orphans)
    ///
    /// Whether the collection is serializable
    @(link_name = "PxSerialization_isSerializable")
    serialization_is_serializable :: proc(collection: ^Collection, sr: ^SerializationRegistry, externalReferences: ^Collection) -> _c.bool ---

    /// Adds to a collection all objects such that it can be successfully serialized.
    ///
    /// A collection C is complete relative to an other collection D if every object required by C is either in C or D.
    /// This function adds objects to a collection, such that it becomes complete with respect to the exceptFor collection.
    /// Completeness is needed for serialization. See [`PxSerialization::serializeCollectionToBinary`],
    /// [`PxSerialization::serializeCollectionToXml`].
    ///
    /// Sdk objects require other sdk object according to the following rules:
    /// - joints require their actors and constraint
    /// - rigid actors require their shapes
    /// - shapes require their material(s) and mesh (triangle mesh, convex mesh or height field), if any
    /// - articulations require their links and joints
    /// - aggregates require their actors
    ///
    /// If followJoints is specified another rule is added:
    /// - actors require their joints
    ///
    /// Specifying followJoints will make whole jointed actor chains being added to the collection. Following chains
    /// is interrupted whenever a object in exceptFor is encountered.
    @(link_name = "PxSerialization_complete")
    serialization_complete :: proc(collection: ^Collection, sr: ^SerializationRegistry, exceptFor: ^Collection, followJoints: _c.bool) ---

    /// Creates PxSerialObjectId values for unnamed objects in a collection.
    ///
    /// Creates PxSerialObjectId names for unnamed objects in a collection starting at a base value and incrementing,
    /// skipping values that are already assigned to objects in the collection.
    @(link_name = "PxSerialization_createSerialObjectIds")
    serialization_create_serial_object_ids :: proc(collection: ^Collection, base: _c.uint64_t) ---

    /// Creates a PxCollection from XML data.
    ///
    /// a pointer to a PxCollection if successful or NULL if it failed.
    @(link_name = "PxSerialization_createCollectionFromXml")
    serialization_create_collection_from_xml :: proc(inputData: ^InputData, cooking: ^Cooking, sr: ^SerializationRegistry, externalRefs: ^Collection, stringTable: ^StringTable, outArgs: ^XmlMiscParameter) -> ^Collection ---

    /// Deserializes a PxCollection from memory.
    ///
    /// Creates a collection from memory. If the collection has external dependencies another collection
    /// can be provided to resolve these.
    ///
    /// The memory block provided has to be 128 bytes aligned and contain a contiguous serialized collection as written
    /// by PxSerialization::serializeCollectionToBinary. The contained binary data needs to be compatible with the current binary format version
    /// which is defined by "PX_PHYSICS_VERSION_MAJOR.PX_PHYSICS_VERSION_MINOR.PX_PHYSICS_VERSION_BUGFIX-PX_BINARY_SERIAL_VERSION".
    /// For a list of compatible sdk releases refer to the documentation of PX_BINARY_SERIAL_VERSION.
    @(link_name = "PxSerialization_createCollectionFromBinary")
    serialization_create_collection_from_binary :: proc(memBlock: rawptr, sr: ^SerializationRegistry, externalRefs: ^Collection) -> ^Collection ---

    /// Serializes a physics collection to an XML output stream.
    ///
    /// The collection to be serialized needs to be complete
    ///
    /// Serialization of objects in a scene that is simultaneously being simulated is not supported and leads to undefined behavior.
    ///
    /// true if the collection is successfully serialized.
    @(link_name = "PxSerialization_serializeCollectionToXml")
    serialization_serialize_collection_to_xml :: proc(outputStream: ^OutputStream, collection: ^Collection, sr: ^SerializationRegistry, cooking: ^Cooking, externalRefs: ^Collection, inArgs: ^XmlMiscParameter) -> _c.bool ---

    /// Serializes a collection to a binary stream.
    ///
    /// Serializes a collection to a stream. In order to resolve external dependencies the externalReferences collection has to be provided.
    /// Optionally names of objects that where set for example with [`PxActor::setName`] are serialized along with the objects.
    ///
    /// The collection can be successfully serialized if isSerializable(collection) returns true. See [`isSerializable`].
    ///
    /// The implementation of the output stream needs to fulfill the requirements on the memory block input taken by
    /// PxSerialization::createCollectionFromBinary.
    ///
    /// Serialization of objects in a scene that is simultaneously being simulated is not supported and leads to undefined behavior.
    ///
    /// Whether serialization was successful
    @(link_name = "PxSerialization_serializeCollectionToBinary")
    serialization_serialize_collection_to_binary :: proc(outputStream: ^OutputStream, collection: ^Collection, sr: ^SerializationRegistry, externalRefs: ^Collection, exportNames: _c.bool) -> _c.bool ---

    /// Creates an application managed registry for serialization.
    ///
    /// PxSerializationRegistry instance.
    @(link_name = "PxSerialization_createSerializationRegistry")
    serialization_create_serialization_registry :: proc(physics: ^Physics) -> ^SerializationRegistry ---

    /// Deletes the dispatcher.
    ///
    /// Do not keep a reference to the deleted instance.
    @(link_name = "PxDefaultCpuDispatcher_release_mut")
    default_cpu_dispatcher_release_mut :: proc(self_: ^DefaultCpuDispatcher) ---

    /// Enables profiling at task level.
    ///
    /// By default enabled only in profiling builds.
    @(link_name = "PxDefaultCpuDispatcher_setRunProfiled_mut")
    default_cpu_dispatcher_set_run_profiled_mut :: proc(self_: ^DefaultCpuDispatcher, runProfiled: _c.bool) ---

    /// Checks if profiling is enabled at task level.
    ///
    /// True if tasks should be profiled.
    @(link_name = "PxDefaultCpuDispatcher_getRunProfiled")
    default_cpu_dispatcher_get_run_profiled :: proc(self_: ^DefaultCpuDispatcher) -> _c.bool ---

    /// Create default dispatcher, extensions SDK needs to be initialized first.
    ///
    /// numThreads may be zero in which case no worker thread are initialized and
    /// simulation tasks will be executed on the thread that calls PxScene::simulate()
    ///
    /// yieldProcessorCount must be greater than zero if eYIELD_PROCESSOR is the chosen mode and equal to zero for all other modes.
    ///
    /// eYIELD_THREAD and eYIELD_PROCESSOR modes will use compute resources even if the simulation is not running.
    /// It is left to users to keep threads inactive, if so desired, when no simulation is running.
    @(link_name = "phys_PxDefaultCpuDispatcherCreate")
    default_cpu_dispatcher_create :: proc(numThreads: _c.uint32_t, affinityMasks: ^_c.uint32_t, mode: DefaultCpuDispatcherWaitForWorkMode, yieldProcessorCount: _c.uint32_t) -> ^DefaultCpuDispatcher ---

    /// Builds smooth vertex normals over a mesh.
    ///
    /// - "smooth" because smoothing groups are not supported here
    /// - takes angles into account for correct cube normals computation
    ///
    /// To use 32bit indices pass a pointer in dFaces and set wFaces to zero. Alternatively pass a pointer to
    /// wFaces and set dFaces to zero.
    ///
    /// True on success.
    @(link_name = "phys_PxBuildSmoothNormals")
    build_smooth_normals :: proc(nbTris: _c.uint32_t, nbVerts: _c.uint32_t, verts: ^Vec3, dFaces: ^_c.uint32_t, wFaces: ^_c.uint16_t, normals: ^Vec3, flip: _c.bool) -> _c.bool ---

    /// simple method to create a PxRigidDynamic actor with a single PxShape.
    ///
    /// a new dynamic actor with the PxRigidBodyFlag, or NULL if it could
    /// not be constructed
    @(link_name = "phys_PxCreateDynamic")
    create_dynamic :: proc(sdk: ^Physics, #by_ptr transform: Transform, geometry: ^Geometry, material: ^Material, density: _c.float, #by_ptr shapeOffset: Transform) -> ^RigidDynamic ---

    /// simple method to create a PxRigidDynamic actor with a single PxShape.
    ///
    /// a new dynamic actor with the PxRigidBodyFlag, or NULL if it could
    /// not be constructed
    @(link_name = "phys_PxCreateDynamic_1")
    create_dynamic_1 :: proc(sdk: ^Physics, #by_ptr transform: Transform, shape: ^Shape, density: _c.float) -> ^RigidDynamic ---

    /// simple method to create a kinematic PxRigidDynamic actor with a single PxShape.
    ///
    /// unlike PxCreateDynamic, the geometry is not restricted to box, capsule, sphere or convex. However,
    /// kinematics of other geometry types may not participate in simulation collision and may be used only for
    /// triggers or scene queries of moving objects under animation control. In this case the density parameter
    /// will be ignored and the created shape will be set up as a scene query only shape (see [`PxShapeFlag::eSCENE_QUERY_SHAPE`])
    ///
    /// a new dynamic actor with the PxRigidBodyFlag::eKINEMATIC set, or NULL if it could
    /// not be constructed
    @(link_name = "phys_PxCreateKinematic")
    create_kinematic :: proc(sdk: ^Physics, #by_ptr transform: Transform, geometry: ^Geometry, material: ^Material, density: _c.float, #by_ptr shapeOffset: Transform) -> ^RigidDynamic ---

    /// simple method to create a kinematic PxRigidDynamic actor with a single PxShape.
    ///
    /// unlike PxCreateDynamic, the geometry is not restricted to box, capsule, sphere or convex. However,
    /// kinematics of other geometry types may not participate in simulation collision and may be used only for
    /// triggers or scene queries of moving objects under animation control. In this case the density parameter
    /// will be ignored and the created shape will be set up as a scene query only shape (see [`PxShapeFlag::eSCENE_QUERY_SHAPE`])
    ///
    /// a new dynamic actor with the PxRigidBodyFlag::eKINEMATIC set, or NULL if it could
    /// not be constructed
    @(link_name = "phys_PxCreateKinematic_1")
    create_kinematic_1 :: proc(sdk: ^Physics, #by_ptr transform: Transform, shape: ^Shape, density: _c.float) -> ^RigidDynamic ---

    /// simple method to create a PxRigidStatic actor with a single PxShape.
    ///
    /// a new static actor, or NULL if it could not be constructed
    @(link_name = "phys_PxCreateStatic")
    create_static :: proc(sdk: ^Physics, #by_ptr transform: Transform, geometry: ^Geometry, material: ^Material, #by_ptr shapeOffset: Transform) -> ^RigidStatic ---

    /// simple method to create a PxRigidStatic actor with a single PxShape.
    ///
    /// a new static actor, or NULL if it could not be constructed
    @(link_name = "phys_PxCreateStatic_1")
    create_static_1 :: proc(sdk: ^Physics, #by_ptr transform: Transform, shape: ^Shape) -> ^RigidStatic ---

    /// create a shape by copying attributes from another shape
    ///
    /// The function clones a PxShape. The following properties are copied:
    /// - geometry
    /// - flags
    /// - materials
    /// - actor-local pose
    /// - contact offset
    /// - rest offset
    /// - simulation filter data
    /// - query filter data
    /// - torsional patch radius
    /// - minimum torsional patch radius
    ///
    /// The following are not copied and retain their default values:
    /// - name
    /// - user data
    ///
    /// the newly-created rigid static
    @(link_name = "phys_PxCloneShape")
    clone_shape :: proc(physicsSDK: ^Physics, #by_ptr shape: Shape, isExclusive: _c.bool) -> ^Shape ---

    /// create a static body by copying attributes from another rigid actor
    ///
    /// The function clones a PxRigidDynamic or PxRigidStatic as a PxRigidStatic. A uniform scale is applied. The following properties are copied:
    /// - shapes
    /// - actor flags
    /// - owner client and client behavior bits
    /// - dominance group
    ///
    /// The following are not copied and retain their default values:
    /// - name
    /// - joints or observers
    /// - aggregate or scene membership
    /// - user data
    ///
    /// Transforms are not copied with bit-exact accuracy.
    ///
    /// the newly-created rigid static
    @(link_name = "phys_PxCloneStatic")
    clone_static :: proc(physicsSDK: ^Physics, #by_ptr transform: Transform, actor: ^RigidActor) -> ^RigidStatic ---

    /// create a dynamic body by copying attributes from an existing body
    ///
    /// The following properties are copied:
    /// - shapes
    /// - actor flags, rigidDynamic flags and rigidDynamic lock flags
    /// - mass, moment of inertia, and center of mass frame
    /// - linear and angular velocity
    /// - linear and angular damping
    /// - maximum linear velocity
    /// - maximum angular velocity
    /// - position and velocity solver iterations
    /// - maximum depenetration velocity
    /// - sleep threshold
    /// - contact report threshold
    /// - dominance group
    /// - owner client and client behavior bits
    /// - name pointer
    /// - kinematic target
    ///
    /// The following are not copied and retain their default values:
    /// - name
    /// - joints or observers
    /// - aggregate or scene membership
    /// - sleep timer
    /// - user data
    ///
    /// Transforms are not copied with bit-exact accuracy.
    ///
    /// the newly-created rigid static
    @(link_name = "phys_PxCloneDynamic")
    clone_dynamic :: proc(physicsSDK: ^Physics, #by_ptr transform: Transform, #by_ptr body: RigidDynamic) -> ^RigidDynamic ---

    /// create a plane actor. The plane equation is n.x + d = 0
    ///
    /// a new static actor, or NULL if it could not be constructed
    @(link_name = "phys_PxCreatePlane")
    create_plane :: proc(sdk: ^Physics, #by_ptr plane: Plane, material: ^Material) -> ^RigidStatic ---

    /// scale a rigid actor by a uniform scale
    ///
    /// The geometry and relative positions of the actor are multiplied by the given scale value. If the actor is a rigid body or an
    /// articulation link and the scaleMassProps value is true, the mass properties are scaled assuming the density is constant: the
    /// center of mass is linearly scaled, the mass is multiplied by the cube of the scale, and the inertia tensor by the fifth power of the scale.
    @(link_name = "phys_PxScaleRigidActor")
    scale_rigid_actor :: proc(actor: ^RigidActor, scale: _c.float, scaleMassProps: _c.bool) ---

    @(link_name = "PxStringTableExt_createStringTable")
    string_table_ext_create_string_table :: proc(inAllocator: ^AllocatorCallback) -> ^StringTable ---

    /// Creates regions for PxSceneDesc, from a global box.
    ///
    /// This helper simply subdivides the given global box into a 2D grid of smaller boxes. Each one of those smaller boxes
    /// is a region of interest for the broadphase. There are nbSubdiv*nbSubdiv regions in the 2D grid. The function does not
    /// subdivide along the given up axis.
    ///
    /// This is the simplest setup one can use with PxBroadPhaseType::eMBP. A more sophisticated setup would try to cover
    /// the game world with a non-uniform set of regions (i.e. not just a grid).
    ///
    /// number of regions written out to the 'regions' array
    @(link_name = "PxBroadPhaseExt_createRegionsFromWorldBounds")
    broad_phase_ext_create_regions_from_world_bounds :: proc(regions: ^Bounds3, #by_ptr globalBounds: Bounds3, nbSubdiv: _c.uint32_t, upAxis: _c.uint32_t) -> _c.uint32_t ---

    /// Raycast returning any blocking hit, not necessarily the closest.
    ///
    /// Returns whether any rigid actor is hit along the ray.
    ///
    /// Shooting a ray from within an object leads to different results depending on the shape type. Please check the details in article SceneQuery. User can ignore such objects by using one of the provided filter mechanisms.
    ///
    /// True if a blocking hit was found.
    @(link_name = "PxSceneQueryExt_raycastAny")
    scene_query_ext_raycast_any :: proc(#by_ptr scene: Scene, #by_ptr origin: Vec3, #by_ptr unitDir: Vec3, distance: _c.float, hit: ^QueryHit, #by_ptr filterData: QueryFilterData, filterCall: ^QueryFilterCallback, cache: ^QueryCache) -> _c.bool ---

    /// Raycast returning a single result.
    ///
    /// Returns the first rigid actor that is hit along the ray. Data for a blocking hit will be returned as specified by the outputFlags field. Touching hits will be ignored.
    ///
    /// Shooting a ray from within an object leads to different results depending on the shape type. Please check the details in article SceneQuery. User can ignore such objects by using one of the provided filter mechanisms.
    ///
    /// True if a blocking hit was found.
    @(link_name = "PxSceneQueryExt_raycastSingle")
    scene_query_ext_raycast_single :: proc(#by_ptr scene: Scene, #by_ptr origin: Vec3, #by_ptr unitDir: Vec3, distance: _c.float, outputFlags: HitFlags_Set, hit: ^RaycastHit, #by_ptr filterData: QueryFilterData, filterCall: ^QueryFilterCallback, cache: ^QueryCache) -> _c.bool ---

    /// Raycast returning multiple results.
    ///
    /// Find all rigid actors that get hit along the ray. Each result contains data as specified by the outputFlags field.
    ///
    /// Touching hits are not ordered.
    ///
    /// Shooting a ray from within an object leads to different results depending on the shape type. Please check the details in article SceneQuery. User can ignore such objects by using one of the provided filter mechanisms.
    ///
    /// Number of hits in the buffer, or -1 if the buffer overflowed.
    @(link_name = "PxSceneQueryExt_raycastMultiple")
    scene_query_ext_raycast_multiple :: proc(#by_ptr scene: Scene, #by_ptr origin: Vec3, #by_ptr unitDir: Vec3, distance: _c.float, outputFlags: HitFlags_Set, hitBuffer: ^RaycastHit, hitBufferSize: _c.uint32_t, blockingHit: ^_c.bool, #by_ptr filterData: QueryFilterData, filterCall: ^QueryFilterCallback, cache: ^QueryCache) -> _c.int32_t ---

    /// Sweep returning any blocking hit, not necessarily the closest.
    ///
    /// Returns whether any rigid actor is hit along the sweep path.
    ///
    /// If a shape from the scene is already overlapping with the query shape in its starting position, behavior is controlled by the PxSceneQueryFlag::eINITIAL_OVERLAP flag.
    ///
    /// True if a blocking hit was found.
    @(link_name = "PxSceneQueryExt_sweepAny")
    scene_query_ext_sweep_any :: proc(#by_ptr scene: Scene, geometry: ^Geometry, #by_ptr pose: Transform, #by_ptr unitDir: Vec3, distance: _c.float, queryFlags: HitFlags_Set, hit: ^QueryHit, #by_ptr filterData: QueryFilterData, filterCall: ^QueryFilterCallback, cache: ^QueryCache, inflation: _c.float) -> _c.bool ---

    /// Sweep returning a single result.
    ///
    /// Returns the first rigid actor that is hit along the ray. Data for a blocking hit will be returned as specified by the outputFlags field. Touching hits will be ignored.
    ///
    /// If a shape from the scene is already overlapping with the query shape in its starting position, behavior is controlled by the PxSceneQueryFlag::eINITIAL_OVERLAP flag.
    ///
    /// True if a blocking hit was found.
    @(link_name = "PxSceneQueryExt_sweepSingle")
    scene_query_ext_sweep_single :: proc(#by_ptr scene: Scene, geometry: ^Geometry, #by_ptr pose: Transform, #by_ptr unitDir: Vec3, distance: _c.float, outputFlags: HitFlags_Set, hit: ^SweepHit, #by_ptr filterData: QueryFilterData, filterCall: ^QueryFilterCallback, cache: ^QueryCache, inflation: _c.float) -> _c.bool ---

    /// Sweep returning multiple results.
    ///
    /// Find all rigid actors that get hit along the sweep. Each result contains data as specified by the outputFlags field.
    ///
    /// Touching hits are not ordered.
    ///
    /// If a shape from the scene is already overlapping with the query shape in its starting position, behavior is controlled by the PxSceneQueryFlag::eINITIAL_OVERLAP flag.
    ///
    /// Number of hits in the buffer, or -1 if the buffer overflowed.
    @(link_name = "PxSceneQueryExt_sweepMultiple")
    scene_query_ext_sweep_multiple :: proc(#by_ptr scene: Scene, geometry: ^Geometry, #by_ptr pose: Transform, #by_ptr unitDir: Vec3, distance: _c.float, outputFlags: HitFlags_Set, hitBuffer: ^SweepHit, hitBufferSize: _c.uint32_t, blockingHit: ^_c.bool, #by_ptr filterData: QueryFilterData, filterCall: ^QueryFilterCallback, cache: ^QueryCache, inflation: _c.float) -> _c.int32_t ---

    /// Test overlap between a geometry and objects in the scene.
    ///
    /// Filtering: Overlap tests do not distinguish between touching and blocking hit types. Both get written to the hit buffer.
    ///
    /// PxHitFlag::eMESH_MULTIPLE and PxHitFlag::eMESH_BOTH_SIDES have no effect in this case
    ///
    /// Number of hits in the buffer, or -1 if the buffer overflowed.
    @(link_name = "PxSceneQueryExt_overlapMultiple")
    scene_query_ext_overlap_multiple :: proc(#by_ptr scene: Scene, geometry: ^Geometry, #by_ptr pose: Transform, hitBuffer: ^OverlapHit, hitBufferSize: _c.uint32_t, #by_ptr filterData: QueryFilterData, filterCall: ^QueryFilterCallback) -> _c.int32_t ---

    /// Test returning, for a given geometry, any overlapping object in the scene.
    ///
    /// Filtering: Overlap tests do not distinguish between touching and blocking hit types. Both trigger a hit.
    ///
    /// PxHitFlag::eMESH_MULTIPLE and PxHitFlag::eMESH_BOTH_SIDES have no effect in this case
    ///
    /// True if an overlap was found.
    @(link_name = "PxSceneQueryExt_overlapAny")
    scene_query_ext_overlap_any :: proc(#by_ptr scene: Scene, geometry: ^Geometry, #by_ptr pose: Transform, hit: ^OverlapHit, #by_ptr filterData: QueryFilterData, filterCall: ^QueryFilterCallback) -> _c.bool ---

    @(link_name = "PxBatchQueryExt_release_mut")
    batch_query_ext_release_mut :: proc(self_: ^BatchQueryExt) ---

    /// Performs a raycast against objects in the scene.
    ///
    /// Touching hits are not ordered.
    ///
    /// Shooting a ray from within an object leads to different results depending on the shape type. Please check the details in article SceneQuery. User can ignore such objects by using one of the provided filter mechanisms.
    ///
    /// This query call writes to a list associated with the query object and is NOT thread safe (for performance reasons there is no lock
    /// and overlapping writes from different threads may result in undefined behavior).
    ///
    /// Returns a PxRaycastBuffer pointer that will store the result of the query after execute() is completed.
    /// This will point either to an element of the buffer allocated on construction or to a user buffer passed to the constructor.
    @(link_name = "PxBatchQueryExt_raycast_mut")
    batch_query_ext_raycast_mut :: proc(self_: ^BatchQueryExt, #by_ptr origin: Vec3, #by_ptr unitDir: Vec3, distance: _c.float, maxNbTouches: _c.uint16_t, hitFlags: HitFlags_Set, #by_ptr filterData: QueryFilterData, cache: ^QueryCache) -> ^RaycastBuffer ---

    /// Performs a sweep test against objects in the scene.
    ///
    /// Touching hits are not ordered.
    ///
    /// If a shape from the scene is already overlapping with the query shape in its starting position,
    /// the hit is returned unless eASSUME_NO_INITIAL_OVERLAP was specified.
    ///
    /// This query call writes to a list associated with the query object and is NOT thread safe (for performance reasons there is no lock
    /// and overlapping writes from different threads may result in undefined behavior).
    ///
    /// Returns a PxSweepBuffer pointer that will store the result of the query after execute() is completed.
    /// This will point either to an element of the buffer allocated on construction or to a user buffer passed to the constructor.
    @(link_name = "PxBatchQueryExt_sweep_mut")
    batch_query_ext_sweep_mut :: proc(self_: ^BatchQueryExt, geometry: ^Geometry, #by_ptr pose: Transform, #by_ptr unitDir: Vec3, distance: _c.float, maxNbTouches: _c.uint16_t, hitFlags: HitFlags_Set, #by_ptr filterData: QueryFilterData, cache: ^QueryCache, inflation: _c.float) -> ^SweepBuffer ---

    /// Performs an overlap test of a given geometry against objects in the scene.
    ///
    /// Filtering: returning eBLOCK from user filter for overlap queries will cause a warning (see [`PxQueryHitType`]).
    ///
    /// eBLOCK should not be returned from user filters for overlap(). Doing so will result in undefined behavior, and a warning will be issued.
    ///
    /// If the PxQueryFlag::eNO_BLOCK flag is set, the eBLOCK will instead be automatically converted to an eTOUCH and the warning suppressed.
    ///
    /// This query call writes to a list associated with the query object and is NOT thread safe (for performance reasons there is no lock
    /// and overlapping writes from different threads may result in undefined behavior).
    ///
    /// Returns a PxOverlapBuffer pointer that will store the result of the query after execute() is completed.
    /// This will point either to an element of the buffer allocated on construction or to a user buffer passed to the constructor.
    @(link_name = "PxBatchQueryExt_overlap_mut")
    batch_query_ext_overlap_mut :: proc(self_: ^BatchQueryExt, geometry: ^Geometry, #by_ptr pose: Transform, maxNbTouches: _c.uint16_t, #by_ptr filterData: QueryFilterData, cache: ^QueryCache) -> ^OverlapBuffer ---

    @(link_name = "PxBatchQueryExt_execute_mut")
    batch_query_ext_execute_mut :: proc(self_: ^BatchQueryExt) ---

    /// Create a PxBatchQueryExt without the need for pre-allocated result or touch buffers.
    ///
    /// Returns a PxBatchQueryExt instance. A NULL pointer will be returned if the subsequent allocations fail or if any of the arguments are illegal.
    /// In the event that a NULL pointer is returned a corresponding error will be issued to the error stream.
    @(link_name = "phys_PxCreateBatchQueryExt")
    create_batch_query_ext :: proc(#by_ptr scene: Scene, queryFilterCallback: ^QueryFilterCallback, maxNbRaycasts: _c.uint32_t, maxNbRaycastTouches: _c.uint32_t, maxNbSweeps: _c.uint32_t, maxNbSweepTouches: _c.uint32_t, maxNbOverlaps: _c.uint32_t, maxNbOverlapTouches: _c.uint32_t) -> ^BatchQueryExt ---

    /// Create a PxBatchQueryExt with user-supplied result and touch buffers.
    ///
    /// Returns a PxBatchQueryExt instance. A NULL pointer will be returned if the subsequent allocations fail or if any of the arguments are illegal.
    /// In the event that a NULL pointer is returned a corresponding error will be issued to the error stream.
    @(link_name = "phys_PxCreateBatchQueryExt_1")
    create_batch_query_ext_1 :: proc(#by_ptr scene: Scene, queryFilterCallback: ^QueryFilterCallback, raycastBuffers: ^RaycastBuffer, maxNbRaycasts: _c.uint32_t, raycastTouches: ^RaycastHit, maxNbRaycastTouches: _c.uint32_t, sweepBuffers: ^SweepBuffer, maxNbSweeps: _c.uint32_t, sweepTouches: ^SweepHit, maxNbSweepTouches: _c.uint32_t, overlapBuffers: ^OverlapBuffer, maxNbOverlaps: _c.uint32_t, overlapTouches: ^OverlapHit, maxNbOverlapTouches: _c.uint32_t) -> ^BatchQueryExt ---

    /// Creates an external scene query system.
    ///
    /// An external SQ system is the part of a PxScene that deals with scene queries (SQ). This is usually taken care of
    /// by an internal implementation inside PxScene, but it is also possible to re-route all SQ calls to an external
    /// implementation, potentially opening the door to some customizations in behavior and features for advanced users.
    ///
    /// The following external SQ system is an example of how an implementation would look like. It re-uses much of the
    /// same code as the internal version, but it could be re-implemented in a completely different way to match users'
    /// specific needs.
    ///
    /// An external SQ system instance
    @(link_name = "phys_PxCreateExternalSceneQuerySystem")
    create_external_scene_query_system :: proc(desc: ^SceneQueryDesc, contextID: _c.uint64_t) -> ^SceneQuerySystem ---

    /// Adds a pruner to the system.
    ///
    /// The internal PhysX scene-query system uses two regular pruners (one for static shapes, one for dynamic shapes) and an optional
    /// compound pruner. Our custom scene query system supports an arbitrary number of regular pruners.
    ///
    /// This can be useful to reduce the load on each pruner, in particular during updates, when internal trees are rebuilt in the
    /// background. On the other hand this implementation simply iterates over all created pruners to perform queries, so their cost
    /// might increase if a large number of pruners is used.
    ///
    /// In any case this serves as an example of how the PxSceneQuerySystem API can be used to customize scene queries.
    ///
    /// A pruner index
    @(link_name = "PxCustomSceneQuerySystem_addPruner_mut")
    custom_scene_query_system_add_pruner_mut :: proc(self_: ^CustomSceneQuerySystem, primaryType: PruningStructureType, secondaryType: DynamicTreeSecondaryPruner, preallocated: _c.uint32_t) -> _c.uint32_t ---

    /// Start custom build-steps for all pruners
    ///
    /// This function is used in combination with customBuildstep() and finishCustomBuildstep() to let users take control
    /// of the pruners' build-step
    /// &
    /// commit calls - basically the pruners' update functions. These functions should be used
    /// with the PxSceneQueryUpdateMode::eBUILD_DISABLED_COMMIT_DISABLED update mode, otherwise the build-steps will happen
    /// automatically in fetchResults. For N pruners it can be more efficient to use these custom build-step functions to
    /// perform the updates in parallel:
    ///
    /// - call startCustomBuildstep() first (one synchronous call)
    /// - for each pruner, call customBuildstep() (asynchronous calls from multiple threads)
    /// - once it is done, call finishCustomBuildstep() to finish the update (synchronous call)
    ///
    /// The multi-threaded update is more efficient here than what it is in PxScene, because the "flushShapes()" call is
    /// also multi-threaded (while it is not in PxScene).
    ///
    /// Note that users are responsible for locks here, and these calls should not overlap with other SQ calls. In particular
    /// one should not add new objects to the SQ system or perform queries while these calls are happening.
    ///
    /// The number of pruners in the system.
    @(link_name = "PxCustomSceneQuerySystem_startCustomBuildstep_mut")
    custom_scene_query_system_start_custom_buildstep_mut :: proc(self_: ^CustomSceneQuerySystem) -> _c.uint32_t ---

    /// Perform a custom build-step for a given pruner.
    @(link_name = "PxCustomSceneQuerySystem_customBuildstep_mut")
    custom_scene_query_system_custom_buildstep_mut :: proc(self_: ^CustomSceneQuerySystem, index: _c.uint32_t) ---

    /// Finish custom build-steps
    ///
    /// Call this function once after all the customBuildstep() calls are done.
    @(link_name = "PxCustomSceneQuerySystem_finishCustomBuildstep_mut")
    custom_scene_query_system_finish_custom_buildstep_mut :: proc(self_: ^CustomSceneQuerySystem) ---

    @(link_name = "PxCustomSceneQuerySystemAdapter_delete")
    custom_scene_query_system_adapter_delete :: proc(self_: ^CustomSceneQuerySystemAdapter) ---

    /// Gets a pruner index for an actor/shape.
    ///
    /// This user-defined function tells the system in which pruner a given actor/shape should go.
    ///
    /// The returned index must be valid, i.e. it must have been previously returned to users by PxCustomSceneQuerySystem::addPruner.
    ///
    /// A pruner index for this actor/shape.
    @(link_name = "PxCustomSceneQuerySystemAdapter_getPrunerIndex")
    custom_scene_query_system_adapter_get_pruner_index :: proc(self_: ^CustomSceneQuerySystemAdapter, actor: ^RigidActor, #by_ptr shape: Shape) -> _c.uint32_t ---

    /// Pruner filtering callback.
    ///
    /// This will be called for each query to validate whether it should process a given pruner.
    ///
    /// True to process the pruner, false to skip it entirely
    @(link_name = "PxCustomSceneQuerySystemAdapter_processPruner")
    custom_scene_query_system_adapter_process_pruner :: proc(self_: ^CustomSceneQuerySystemAdapter, prunerIndex: _c.uint32_t, context_: ^QueryThreadContext, #by_ptr filterData: QueryFilterData, filterCall: ^QueryFilterCallback) -> _c.bool ---

    /// Creates a custom scene query system.
    ///
    /// This is similar to PxCreateExternalSceneQuerySystem, except this function creates a PxCustomSceneQuerySystem object.
    /// It can be plugged to PxScene the same way, via PxSceneDesc::sceneQuerySystem.
    ///
    /// A custom SQ system instance
    @(link_name = "phys_PxCreateCustomSceneQuerySystem")
    create_custom_scene_query_system :: proc(sceneQueryUpdateMode: SceneQueryUpdateMode, contextID: _c.uint64_t, #by_ptr adapter: CustomSceneQuerySystemAdapter, usesTreeOfPruners: _c.bool) -> ^CustomSceneQuerySystem ---

    /// Computes closest polygon of the convex hull geometry for a given impact point
    /// and impact direction. When doing sweeps against a scene, one might want to delay
    /// the rather expensive computation of the hit face index for convexes until it is clear
    /// the information is really needed and then use this method to get the corresponding
    /// face index.
    ///
    /// Closest face index of the convex geometry.
    @(link_name = "phys_PxFindFaceIndex")
    find_face_index :: proc(#by_ptr convexGeom: ConvexMeshGeometry, #by_ptr geomPose: Transform, #by_ptr impactPos: Vec3, #by_ptr unitDir: Vec3) -> _c.uint32_t ---

    /// Sets the sampling radius
    ///
    /// Returns true if the sampling was successful and false if there was a problem. Usually an internal overflow is the problem for very big meshes or very small sampling radii.
    @(link_name = "PxPoissonSampler_setSamplingRadius_mut")
    poisson_sampler_set_sampling_radius_mut :: proc(self_: ^PoissonSampler, samplingRadius: _c.float) -> _c.bool ---

    /// Adds new Poisson Samples inside the sphere specified
    @(link_name = "PxPoissonSampler_addSamplesInSphere_mut")
    poisson_sampler_add_samples_in_sphere_mut :: proc(self_: ^PoissonSampler, #by_ptr sphereCenter: Vec3, sphereRadius: _c.float, createVolumeSamples: _c.bool) ---

    /// Adds new Poisson Samples inside the box specified
    @(link_name = "PxPoissonSampler_addSamplesInBox_mut")
    poisson_sampler_add_samples_in_box_mut :: proc(self_: ^PoissonSampler, #by_ptr axisAlignedBox: Bounds3, #by_ptr boxOrientation: Quat, createVolumeSamples: _c.bool) ---

    @(link_name = "PxPoissonSampler_delete")
    poisson_sampler_delete :: proc(self_: ^PoissonSampler) ---

    /// Creates a shape sampler
    ///
    /// Returns the sampler
    @(link_name = "phys_PxCreateShapeSampler")
    create_shape_sampler :: proc(geometry: ^Geometry, #by_ptr transform: Transform, #by_ptr worldBounds: Bounds3, initialSamplingRadius: _c.float, numSampleAttemptsAroundPoint: _c.int32_t) -> ^PoissonSampler ---

    /// Checks whether a point is inside the triangle mesh
    ///
    /// Returns true if the point is inside the triangle mesh
    @(link_name = "PxTriangleMeshPoissonSampler_isPointInTriangleMesh_mut")
    triangle_mesh_poisson_sampler_is_point_in_triangle_mesh_mut :: proc(self_: ^TriangleMeshPoissonSampler, #by_ptr p: Vec3) -> _c.bool ---

    @(link_name = "PxTriangleMeshPoissonSampler_delete")
    triangle_mesh_poisson_sampler_delete :: proc(self_: ^TriangleMeshPoissonSampler) ---

    /// Creates a triangle mesh sampler
    ///
    /// Returns the sampler
    @(link_name = "phys_PxCreateTriangleMeshSampler")
    create_triangle_mesh_sampler :: proc(triangles: ^_c.uint32_t, numTriangles: _c.uint32_t, vertices: ^Vec3, numVertices: _c.uint32_t, initialSamplingRadius: _c.float, numSampleAttemptsAroundPoint: _c.int32_t) -> ^TriangleMeshPoissonSampler ---

    /// Returns the index of the tetrahedron that contains a point
    ///
    /// The index of the tetrahedon containing the point, -1 if not tetrahedron contains the opoint
    @(link_name = "PxTetrahedronMeshExt_findTetrahedronContainingPoint")
    tetrahedron_mesh_ext_find_tetrahedron_containing_point :: proc(mesh: ^TetrahedronMesh, #by_ptr point: Vec3, bary: ^Vec4, tolerance: _c.float) -> _c.int32_t ---

    /// Returns the index of the tetrahedron closest to a point
    ///
    /// The index of the tetrahedon closest to the point
    @(link_name = "PxTetrahedronMeshExt_findTetrahedronClosestToPoint")
    tetrahedron_mesh_ext_find_tetrahedron_closest_to_point :: proc(mesh: ^TetrahedronMesh, #by_ptr point: Vec3, bary: ^Vec4) -> _c.int32_t ---

    /// Initialize the PhysXExtensions library.
    ///
    /// This should be called before calling any functions or methods in extensions which may require allocation.
    ///
    /// This function does not need to be called before creating a PxDefaultAllocator object.
    @(link_name = "phys_PxInitExtensions")
    init_extensions :: proc(physics: ^Physics, pvd: ^Pvd) -> _c.bool ---

    /// Shut down the PhysXExtensions library.
    ///
    /// This function should be called to cleanly shut down the PhysXExtensions library before application exit.
    ///
    /// This function is required to be called to release foundation usage.
    @(link_name = "phys_PxCloseExtensions")
    close_extensions :: proc() ---

    @(link_name = "PxRepXObject_new")
    rep_x_object_new :: proc(inTypeName: cstring, inSerializable: rawptr, inId: _c.uint64_t) -> RepXObject ---

    @(link_name = "PxRepXObject_isValid")
    rep_x_object_is_valid :: proc(self_: ^RepXObject) -> _c.bool ---

    @(link_name = "PxRepXInstantiationArgs_new")
    rep_x_instantiation_args_new :: proc(inPhysics: ^Physics, inCooking: ^Cooking, inStringTable: ^StringTable) -> RepXInstantiationArgs ---

    /// The type this Serializer is meant to operate on.
    @(link_name = "PxRepXSerializer_getTypeName_mut")
    rep_x_serializer_get_type_name_mut :: proc(self_: ^RepXSerializer) -> cstring ---

    /// Convert from a RepX object to a key-value pair hierarchy
    @(link_name = "PxRepXSerializer_objectToFile_mut")
    rep_x_serializer_object_to_file_mut :: proc(self_: ^RepXSerializer, #by_ptr inLiveObject: RepXObject, inCollection: ^Collection, inWriter: ^XmlWriter, inTempBuffer: ^MemoryBuffer, inArgs: ^RepXInstantiationArgs) ---

    /// Convert from a descriptor to a live object.  Must be an object of this Serializer type.
    ///
    /// The new live object.  It can be an invalid object if the instantiation cannot take place.
    @(link_name = "PxRepXSerializer_fileToObject_mut")
    rep_x_serializer_file_to_object_mut :: proc(self_: ^RepXSerializer, inReader: ^XmlReader, inAllocator: ^XmlMemoryAllocator, inArgs: ^RepXInstantiationArgs, inCollection: ^Collection) -> RepXObject ---

    /// Connects the SDK to the PhysX Visual Debugger application.
    @(link_name = "PxPvd_connect_mut")
    pvd_connect_mut :: proc(self_: ^Pvd, transport: ^PvdTransport, flags: PvdInstrumentationFlags_Set) -> _c.bool ---

    /// Disconnects the SDK from the PhysX Visual Debugger application.
    /// If we are still connected, this will kill the entire debugger connection.
    @(link_name = "PxPvd_disconnect_mut")
    pvd_disconnect_mut :: proc(self_: ^Pvd) ---

    /// Return if connection to PVD is created.
    @(link_name = "PxPvd_isConnected_mut")
    pvd_is_connected_mut :: proc(self_: ^Pvd, useCachedStatus: _c.bool) -> _c.bool ---

    /// returns the PVD data transport
    /// returns NULL if no transport is present.
    @(link_name = "PxPvd_getTransport_mut")
    pvd_get_transport_mut :: proc(self_: ^Pvd) -> ^PvdTransport ---

    /// Retrieves the PVD flags. See PxPvdInstrumentationFlags.
    @(link_name = "PxPvd_getInstrumentationFlags_mut")
    pvd_get_instrumentation_flags_mut :: proc(self_: ^Pvd) -> PvdInstrumentationFlags_Set ---

    /// Releases the pvd instance.
    @(link_name = "PxPvd_release_mut")
    pvd_release_mut :: proc(self_: ^Pvd) ---

    /// Create a pvd instance.
    @(link_name = "phys_PxCreatePvd")
    create_pvd :: proc(foundation: ^Foundation) -> ^Pvd ---

    /// Connects to the Visual Debugger application.
    /// return True if success
    @(link_name = "PxPvdTransport_connect_mut")
    pvd_transport_connect_mut :: proc(self_: ^PvdTransport) -> _c.bool ---

    /// Disconnects from the Visual Debugger application.
    /// If we are still connected, this will kill the entire debugger connection.
    @(link_name = "PxPvdTransport_disconnect_mut")
    pvd_transport_disconnect_mut :: proc(self_: ^PvdTransport) ---

    /// Return if connection to PVD is created.
    @(link_name = "PxPvdTransport_isConnected_mut")
    pvd_transport_is_connected_mut :: proc(self_: ^PvdTransport) -> _c.bool ---

    /// write bytes to the other endpoint of the connection. should lock before witre. If an error occurs
    /// this connection will assume to be dead.
    @(link_name = "PxPvdTransport_write_mut")
    pvd_transport_write_mut :: proc(self_: ^PvdTransport, inBytes: ^_c.uint8_t, inLength: _c.uint32_t) -> _c.bool ---

    @(link_name = "PxPvdTransport_lock_mut")
    pvd_transport_lock_mut :: proc(self_: ^PvdTransport) -> ^PvdTransport ---

    @(link_name = "PxPvdTransport_unlock_mut")
    pvd_transport_unlock_mut :: proc(self_: ^PvdTransport) ---

    /// send any data and block until we know it is at least on the wire.
    @(link_name = "PxPvdTransport_flush_mut")
    pvd_transport_flush_mut :: proc(self_: ^PvdTransport) ---

    /// Return size of written data.
    @(link_name = "PxPvdTransport_getWrittenDataSize_mut")
    pvd_transport_get_written_data_size_mut :: proc(self_: ^PvdTransport) -> _c.uint64_t ---

    @(link_name = "PxPvdTransport_release_mut")
    pvd_transport_release_mut :: proc(self_: ^PvdTransport) ---

    /// Create a default socket transport.
    @(link_name = "phys_PxDefaultPvdSocketTransportCreate")
    default_pvd_socket_transport_create :: proc(host: cstring, port: _c.int32_t, timeoutInMilliseconds: _c.uint32_t) -> ^PvdTransport ---

    /// Create a default file transport.
    @(link_name = "phys_PxDefaultPvdFileTransportCreate")
    default_pvd_file_transport_create :: proc(name: cstring) -> ^PvdTransport ---

}
