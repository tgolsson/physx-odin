package physx
import _c "core:c"
/// enum for empty constructor tag
PxEMPTY :: enum _c.int32_t {
    PxEmpty = 0,
}

/// enum for zero constructor tag for vectors and matrices
PxZERO :: enum _c.int32_t {
    PxZero = 0,
}

/// enum for identity constructor flag for quaternions, transforms, and matrices
PxIDENTITY :: enum _c.int32_t {
    PxIdentity = 0,
}

/// Error codes
///
/// These error codes are passed to [`PxErrorCallback`]
PxErrorCode :: enum _c.int32_t {
    NoError = 0,
    /// An informational message.
    DebugInfo = 1,
    /// a warning message for the user to help with debugging
    DebugWarning = 2,
    /// method called with invalid parameter(s)
    InvalidParameter = 4,
    /// method was called at a time when an operation is not possible
    InvalidOperation = 8,
    /// method failed to allocate some memory
    OutOfMemory = 16,
    /// The library failed for some reason.
    /// Possibly you have passed invalid values like NaNs, which are not checked for.
    InternalError = 32,
    /// An unrecoverable error, execution should be halted and log output flushed
    Abort = 64,
    /// The SDK has determined that an operation may result in poor performance.
    PerfWarning = 128,
    /// A bit mask for including all errors
    MaskAll = -1,
}

PxThreadPriority :: enum _c.uint32_t {
    /// High priority
    High = 0,
    /// Above Normal priority
    AboveNormal = 1,
    /// Normal/default priority
    Normal = 2,
    /// Below Normal priority
    BelowNormal = 3,
    /// Low priority.
    Low = 4,
    ForceDword = 4294967295,
}

/// Default color values used for debug rendering.
PxDebugColor :: enum _c.uint32_t {
    ArgbBlack = 4278190080,
    ArgbRed = 4294901760,
    ArgbGreen = 4278255360,
    ArgbBlue = 4278190335,
    ArgbYellow = 4294967040,
    ArgbMagenta = 4294902015,
    ArgbCyan = 4278255615,
    ArgbWhite = 4294967295,
    ArgbGrey = 4286611584,
    ArgbDarkred = 4287102976,
    ArgbDarkgreen = 4278224896,
    ArgbDarkblue = 4278190216,
}

/// an enumeration of concrete classes inheriting from PxBase
///
/// Enumeration space is reserved for future PhysX core types, PhysXExtensions,
/// PhysXVehicle and Custom application types.
PxConcreteType :: enum _c.int32_t {
    Undefined = 0,
    Heightfield = 1,
    ConvexMesh = 2,
    TriangleMeshBvh33 = 3,
    TriangleMeshBvh34 = 4,
    TetrahedronMesh = 5,
    SoftbodyMesh = 6,
    RigidDynamic = 7,
    RigidStatic = 8,
    Shape = 9,
    Material = 10,
    SoftbodyMaterial = 11,
    ClothMaterial = 12,
    PbdMaterial = 13,
    FlipMaterial = 14,
    MpmMaterial = 15,
    CustomMaterial = 16,
    Constraint = 17,
    Aggregate = 18,
    ArticulationReducedCoordinate = 19,
    ArticulationLink = 20,
    ArticulationJointReducedCoordinate = 21,
    ArticulationSensor = 22,
    ArticulationSpatialTendon = 23,
    ArticulationFixedTendon = 24,
    ArticulationAttachment = 25,
    ArticulationTendonJoint = 26,
    PruningStructure = 27,
    Bvh = 28,
    SoftBody = 29,
    SoftBodyState = 30,
    PbdParticlesystem = 31,
    FlipParticlesystem = 32,
    MpmParticlesystem = 33,
    CustomParticlesystem = 34,
    FemCloth = 35,
    HairSystem = 36,
    ParticleBuffer = 37,
    ParticleDiffuseBuffer = 38,
    ParticleClothBuffer = 39,
    ParticleRigidBuffer = 40,
    PhysxCoreCount = 41,
    FirstPhysxExtension = 256,
    FirstVehicleExtension = 512,
    FirstUserExtension = 1024,
}

/// Flags for PxBase.
PxBaseFlag :: enum _c.int32_t {
    OwnsMemory = 0,
    IsReleasable = 1,
}

/// Flags for [`PxBaseFlag`]
PxBaseFlags_Set :: bit_set[PxBaseFlag; _c.uint16_t]


/// Flags used to configure binary meta data entries, typically set through PX_DEF_BIN_METADATA defines.
PxMetaDataFlag :: enum _c.int32_t {
    /// declares a class
    Class = 1,
    /// declares class to be virtual
    Virtual = 2,
    /// declares a typedef
    Typedef = 4,
    /// declares a pointer
    Ptr = 8,
    /// declares a handle
    Handle = 16,
    /// declares extra data exported with PxSerializer::exportExtraData
    ExtraData = 32,
    /// specifies one element of extra data
    ExtraItem = 64,
    /// specifies an array of extra data
    ExtraItems = 128,
    /// specifies a name of extra data
    ExtraName = 256,
    /// declares a union
    Union = 512,
    /// declares explicit padding data
    Padding = 1024,
    /// declares aligned data
    Alignment = 2048,
    /// specifies that the count value's most significant bit needs to be masked out
    CountMaskMsb = 4096,
    /// specifies that the count value is treated as zero for a variable value of one - special case for single triangle meshes
    CountSkipIfOne = 8192,
    /// specifies that the control value is the negate of the variable value
    ControlFlip = 16384,
    /// specifies that the control value is masked - mask bits are assumed to be within eCONTROL_MASK_RANGE
    ControlMask = 32768,
    /// mask range allowed for eCONTROL_MASK
    ControlMaskRange = 255,
    ForceDword = 2147483647,
}

/// Identifies the type of each heavyweight PxTask object
PxTaskType :: enum _c.int32_t {
    /// PxTask will be run on the CPU
    Cpu = 0,
    /// Return code when attempting to find a task that does not exist
    NotPresent = 1,
    /// PxTask execution has been completed
    Completed = 2,
}

/// A geometry type.
///
/// Used to distinguish the type of a ::PxGeometry object.
PxGeometryType :: enum _c.int32_t {
    Sphere = 0,
    Plane = 1,
    Capsule = 2,
    Box = 3,
    Convexmesh = 4,
    Particlesystem = 5,
    Tetrahedronmesh = 6,
    Trianglemesh = 7,
    Heightfield = 8,
    Hairsystem = 9,
    Custom = 10,
    /// internal use only!
    GeometryCount = 11,
    /// internal use only!
    Invalid = -1,
}

/// Geometry-level query flags.
PxGeometryQueryFlag :: enum _c.int32_t {
    SimdGuard = 0,
}

/// Flags for [`PxGeometryQueryFlag`]
PxGeometryQueryFlags_Set :: bit_set[PxGeometryQueryFlag; _c.uint32_t]


/// Desired build strategy for bounding-volume hierarchies
PxBVHBuildStrategy :: enum _c.int32_t {
    /// Fast build strategy. Fast build speed, good runtime performance in most cases. Recommended for runtime cooking.
    Fast = 0,
    /// Default build strategy. Medium build speed, good runtime performance in all cases.
    Default = 1,
    /// SAH build strategy. Slower builds, slightly improved runtime performance in some cases.
    Sah = 2,
    Last = 3,
}

/// Flags controlling the simulated behavior of the convex mesh geometry.
///
/// Used in ::PxConvexMeshGeometryFlags.
PxConvexMeshGeometryFlag :: enum _c.int32_t {
    TightBounds = 0,
}

/// Flags for [`PxConvexMeshGeometryFlag`]
PxConvexMeshGeometryFlags_Set :: bit_set[PxConvexMeshGeometryFlag; _c.uint8_t]


/// Flags controlling the simulated behavior of the triangle mesh geometry.
///
/// Used in ::PxMeshGeometryFlags.
PxMeshGeometryFlag :: enum _c.int32_t {
    TightBounds = 0,
    DoubleSided = 1,
}

/// Flags for [`PxMeshGeometryFlag`]
PxMeshGeometryFlags_Set :: bit_set[PxMeshGeometryFlag; _c.uint8_t]


/// Identifies the solver to use for a particle system.
PxParticleSolverType :: enum _c.int32_t {
    /// The position based dynamics solver that can handle fluid, granular material, cloth, inflatables etc. See [`PxPBDParticleSystem`].
    Pbd = 1,
    /// The FLIP fluid solver. See [`PxFLIPParticleSystem`].
    Flip = 2,
    /// The MPM (material point method) solver that can handle a variety of materials. See [`PxMPMParticleSystem`].
    Mpm = 4,
    /// Custom solver. The user needs to specify the interaction of the particle by providing appropriate functions. Can be used e.g. for molecular dynamics simulations. See [`PxCustomParticleSystem`].
    Custom = 8,
}

/// Scene query and geometry query behavior flags.
///
/// PxHitFlags are used for 3 different purposes:
///
/// 1) To request hit fields to be filled in by scene queries (such as hit position, normal, face index or UVs).
/// 2) Once query is completed, to indicate which fields are valid (note that a query may produce more valid fields than requested).
/// 3) To specify additional options for the narrow phase and mid-phase intersection routines.
///
/// All these flags apply to both scene queries and geometry queries (PxGeometryQuery).
PxHitFlag :: enum _c.int32_t {
    Position = 0,
    Normal = 1,
    Uv = 3,
    AssumeNoInitialOverlap = 4,
    AnyHit = 5,
    MeshMultiple = 6,
    MeshBothSides = 7,
    PreciseSweep = 8,
    Mtd = 9,
    FaceIndex = 10,
}
PxHitFlag_Default :: PxHitFlag.Position | PxHitFlag.Normal | PxHitFlag.FaceIndex
PxHitFlag_ModifiableFlags :: PxHitFlag.AssumeNoInitialOverlap | PxHitFlag.MeshMultiple | PxHitFlag.MeshBothSides | PxHitFlag.PreciseSweep

/// Flags for [`PxHitFlag`]
PxHitFlags_Set :: bit_set[PxHitFlag; _c.uint16_t]


/// Describes the format of height field samples.
PxHeightFieldFormat :: enum _c.int32_t {
    /// Height field height data is 16 bit signed integers, followed by triangle materials.
    ///
    /// Each sample is 32 bits wide arranged as follows:
    ///
    /// 1) First there is a 16 bit height value.
    /// 2) Next, two one byte material indices, with the high bit of each byte reserved for special use.
    /// (so the material index is only 7 bits).
    /// The high bit of material0 is the tess-flag.
    /// The high bit of material1 is reserved for future use.
    ///
    /// There are zero or more unused bytes before the next sample depending on PxHeightFieldDesc.sampleStride,
    /// where the application may eventually keep its own data.
    ///
    /// This is the only format supported at the moment.
    S16Tm = 1,
}

/// Determines the tessellation of height field cells.
PxHeightFieldTessFlag :: enum _c.int32_t {
    /// This flag determines which way each quad cell is subdivided.
    ///
    /// The flag lowered indicates subdivision like this: (the 0th vertex is referenced by only one triangle)
    ///
    /// +--+--+--+---> column
    /// | /| /| /|
    /// |/ |/ |/ |
    /// +--+--+--+
    /// | /| /| /|
    /// |/ |/ |/ |
    /// +--+--+--+
    /// |
    /// |
    /// V row
    ///
    /// The flag raised indicates subdivision like this: (the 0th vertex is shared by two triangles)
    ///
    /// +--+--+--+---> column
    /// |
    /// \
    /// |
    /// \
    /// |
    /// \
    /// |
    /// |
    /// \
    /// |
    /// \
    /// |
    /// \
    /// |
    /// +--+--+--+
    /// |
    /// \
    /// |
    /// \
    /// |
    /// \
    /// |
    /// |
    /// \
    /// |
    /// \
    /// |
    /// \
    /// |
    /// +--+--+--+
    /// |
    /// |
    /// V row
    E0ThVertexShared = 1,
}

/// Enum with flag values to be used in PxHeightFieldDesc.flags.
PxHeightFieldFlag :: enum _c.int32_t {
    NoBoundaryEdges = 0,
}

/// Flags for [`PxHeightFieldFlag`]
PxHeightFieldFlags_Set :: bit_set[PxHeightFieldFlag; _c.uint16_t]


/// Special material index values for height field samples.
PxHeightFieldMaterial :: enum _c.int32_t {
    /// A material indicating that the triangle should be treated as a hole in the mesh.
    Hole = 127,
}

PxMeshMeshQueryFlag :: enum _c.int32_t {
    DiscardCoplanar = 0,
}
PxMeshMeshQueryFlag_Default :: 0

/// Flags for [`PxMeshMeshQueryFlag`]
PxMeshMeshQueryFlags_Set :: bit_set[PxMeshMeshQueryFlag; _c.uint32_t]


/// Enum with flag values to be used in PxSimpleTriangleMesh::flags.
PxMeshFlag :: enum _c.int32_t {
    Flipnormals = 0,
    E16BitIndices = 1,
}

/// Flags for [`PxMeshFlag`]
PxMeshFlags_Set :: bit_set[PxMeshFlag; _c.uint16_t]


/// Mesh midphase structure. This enum is used to select the desired acceleration structure for midphase queries
/// (i.e. raycasts, overlaps, sweeps vs triangle meshes).
///
/// The PxMeshMidPhase::eBVH33 structure is the one used in recent PhysX versions (up to PhysX 3.3). It has great performance and is
/// supported on all platforms. It is deprecated since PhysX 5.x.
///
/// The PxMeshMidPhase::eBVH34 structure is a revisited implementation introduced in PhysX 3.4. It can be significantly faster both
/// in terms of cooking performance and runtime performance.
PxMeshMidPhase :: enum _c.int32_t {
    /// Default midphase mesh structure, as used up to PhysX 3.3 (deprecated)
    Bvh33 = 0,
    /// New midphase mesh structure, introduced in PhysX 3.4
    Bvh34 = 1,
    Last = 2,
}

/// Flags for the mesh geometry properties.
///
/// Used in ::PxTriangleMeshFlags.
PxTriangleMeshFlag :: enum _c.int32_t {
    E16BitIndices = 1,
    AdjacencyInfo = 2,
    PreferNoSdfProj = 3,
}

/// Flags for [`PxTriangleMeshFlag`]
PxTriangleMeshFlags_Set :: bit_set[PxTriangleMeshFlag; _c.uint8_t]


PxTetrahedronMeshFlag :: enum _c.int32_t {
    E16BitIndices = 1,
}

/// Flags for [`PxTetrahedronMeshFlag`]
PxTetrahedronMeshFlags_Set :: bit_set[PxTetrahedronMeshFlag; _c.uint8_t]


/// Flags which control the behavior of an actor.
PxActorFlag :: enum _c.int32_t {
    Visualization = 0,
    DisableGravity = 1,
    SendSleepNotifies = 2,
    DisableSimulation = 3,
}

/// Flags for [`PxActorFlag`]
PxActorFlags_Set :: bit_set[PxActorFlag; _c.uint8_t]


/// Identifies each type of actor.
PxActorType :: enum _c.int32_t {
    /// A static rigid body
    RigidStatic = 0,
    /// A dynamic rigid body
    RigidDynamic = 1,
    /// An articulation link
    ArticulationLink = 2,
}

PxAggregateType :: enum _c.int32_t {
    /// Aggregate will contain various actors of unspecified types
    Generic = 0,
    /// Aggregate will only contain static actors
    Static = 1,
    /// Aggregate will only contain kinematic actors
    Kinematic = 2,
}

/// Constraint row flags
///
/// These flags configure the post-processing of constraint rows and the behavior of the solver while solving constraints
Px1DConstraintFlag :: enum _c.int32_t {
    Spring = 0,
    AccelerationSpring = 1,
    Restitution = 2,
    Keepbias = 3,
    OutputForce = 4,
    HasDriveLimit = 5,
    AngularConstraint = 6,
    DriveRow = 7,
}

/// Flags for [`Px1DConstraintFlag`]
Px1DConstraintFlags_Set :: bit_set[Px1DConstraintFlag; _c.uint16_t]


/// Constraint type hints which the solver uses to optimize constraint handling
PxConstraintSolveHint :: enum _c.int32_t {
    /// no special properties
    None = 0,
    /// a group of acceleration drive constraints with the same stiffness and drive parameters
    Acceleration1 = 256,
    /// temporary special value to identify SLERP drive rows
    SlerpSpring = 258,
    /// a group of acceleration drive constraints with the same stiffness and drive parameters
    Acceleration2 = 512,
    /// a group of acceleration drive constraints with the same stiffness and drive parameters
    Acceleration3 = 768,
    /// rotational equality constraints with no force limit and no velocity target
    RotationalEquality = 1024,
    /// rotational inequality constraints with (0, PX_MAX_FLT) force limits
    RotationalInequality = 1025,
    /// equality constraints with no force limit and no velocity target
    Equality = 2048,
    /// inequality constraints with (0, PX_MAX_FLT) force limits
    Inequality = 2049,
}

/// Flags for determining which components of the constraint should be visualized.
PxConstraintVisualizationFlag :: enum _c.int32_t {
    /// visualize constraint frames
    LocalFrames = 1,
    /// visualize constraint limits
    Limits = 2,
}

/// Flags for determining how PVD should serialize a constraint update
PxPvdUpdateType :: enum _c.int32_t {
    /// triggers createPvdInstance call, creates an instance of a constraint
    CreateInstance = 0,
    /// triggers releasePvdInstance call, releases an instance of a constraint
    ReleaseInstance = 1,
    /// triggers updatePvdProperties call, updates all properties of a constraint
    UpdateAllProperties = 2,
    /// triggers simUpdate call, updates all simulation properties of a constraint
    UpdateSimProperties = 3,
}

/// Constraint descriptor used inside the solver
ConstraintType :: enum _c.int32_t {
    /// Defines this pair is a contact constraint
    ContactConstraint = 0,
    /// Defines this pair is a joint constraint
    JointConstraint = 1,
}

/// Data structure used for preparing constraints before solving them
BodyState :: enum _c.int32_t {
    DynamicBody = 1,
    StaticBody = 2,
    KinematicBody = 4,
    Articulation = 8,
}

/// @
/// {
PxArticulationAxis :: enum _c.int32_t {
    /// Rotational about eX
    Twist = 0,
    /// Rotational about eY
    Swing1 = 1,
    /// Rotational about eZ
    Swing2 = 2,
    /// Linear in eX
    X = 3,
    /// Linear in eY
    Y = 4,
    /// Linear in eZ
    Z = 5,
    Count = 6,
}

PxArticulationMotion :: enum _c.int32_t {
    Limited = 0,
    Free = 1,
}
PxArticulationMotion_Locked :: 0

/// Flags for [`PxArticulationMotion`]
PxArticulationMotions_Set :: bit_set[PxArticulationMotion; _c.uint8_t]


PxArticulationJointType :: enum _c.int32_t {
    /// All joint axes, i.e. degrees of freedom (DOFs) locked
    Fix = 0,
    /// Single linear DOF, e.g. cart on a rail
    Prismatic = 1,
    /// Single rotational DOF, e.g. an elbow joint or a rotational motor, position wrapped at 2pi radians
    Revolute = 2,
    /// Single rotational DOF, e.g. an elbow joint or a rotational motor, position not wrapped
    RevoluteUnwrapped = 3,
    /// Ball and socket joint with two or three DOFs
    Spherical = 4,
    Undefined = 5,
}

PxArticulationFlag :: enum _c.int32_t {
    FixBase = 0,
    DriveLimitsAreForces = 1,
    DisableSelfCollision = 2,
    ComputeJointForces = 3,
}

/// Flags for [`PxArticulationFlag`]
PxArticulationFlags_Set :: bit_set[PxArticulationFlag; _c.uint8_t]


PxArticulationDriveType :: enum _c.int32_t {
    /// The output of the implicit spring drive controller is a force/torque.
    Force = 0,
    /// The output of the implicit spring drive controller is a joint acceleration (use this to get (spatial)-inertia-invariant behavior of the drive).
    Acceleration = 1,
    /// Sets the drive gains internally to track a target position almost kinematically (i.e. with very high drive gains).
    Target = 2,
    /// Sets the drive gains internally to track a target velocity almost kinematically (i.e. with very high drive gains).
    Velocity = 3,
    None = 4,
}

/// A description of the types of articulation data that may be directly written to and read from the GPU using the functions
/// PxScene::copyArticulationData() and PxScene::applyArticulationData(). Types that are read-only may only be used in conjunction with
/// PxScene::copyArticulationData(). Types that are write-only may only be used in conjunction with PxScene::applyArticulationData().
/// A subset of data types may be used in conjunction with both PxScene::applyArticulationData() and PxScene::applyArticulationData().
PxArticulationGpuDataType :: enum _c.int32_t {
    /// The joint positions, read and write, see PxScene::copyArticulationData(), PxScene::applyArticulationData()
    JointPosition = 0,
    /// The joint velocities, read and write,  see PxScene::copyArticulationData(), PxScene::applyArticulationData()
    JointVelocity = 1,
    /// The joint accelerations, read only, see PxScene::copyArticulationData()
    JointAcceleration = 2,
    /// The applied joint forces, write only, see PxScene::applyArticulationData()
    JointForce = 3,
    /// The computed joint constraint solver forces, read only, see PxScene::copyArticulationData()()
    JointSolverForce = 4,
    /// The velocity targets for the joint drives, write only, see PxScene::applyArticulationData()
    JointTargetVelocity = 5,
    /// The position targets for the joint drives, write only, see PxScene::applyArticulationData()
    JointTargetPosition = 6,
    /// The spatial sensor forces, read only, see PxScene::copyArticulationData()
    SensorForce = 7,
    /// The root link transform, read and write, see PxScene::copyArticulationData(), PxScene::applyArticulationData()
    RootTransform = 8,
    /// The root link velocity, read and write, see PxScene::copyArticulationData(), PxScene::applyArticulationData()
    RootVelocity = 9,
    /// The link transforms including root link, read only, see PxScene::copyArticulationData()
    LinkTransform = 10,
    /// The link velocities including root link, read only, see PxScene::copyArticulationData()
    LinkVelocity = 11,
    /// The forces to apply to links, write only, see PxScene::applyArticulationData()
    LinkForce = 12,
    /// The torques to apply to links, write only, see PxScene::applyArticulationData()
    LinkTorque = 13,
    /// Fixed tendon data, write only, see PxScene::applyArticulationData()
    FixedTendon = 14,
    /// Fixed tendon joint data, write only, see PxScene::applyArticulationData()
    FixedTendonJoint = 15,
    /// Spatial tendon data, write only, see PxScene::applyArticulationData()
    SpatialTendon = 16,
    /// Spatial tendon attachment data, write only, see PxScene::applyArticulationData()
    SpatialTendonAttachment = 17,
}

/// These flags determine what data is read or written to the internal articulation data via cache.
PxArticulationCacheFlag :: enum _c.int32_t {
    Velocity = 0,
    Acceleration = 1,
    Position = 2,
    Force = 3,
    LinkVelocity = 4,
    LinkAcceleration = 5,
    RootTransform = 6,
    RootVelocities = 7,
    SensorForces = 8,
    JointSolverForces = 9,
}
PxArticulationCacheFlag_All :: PxArticulationCacheFlag.Velocity | PxArticulationCacheFlag.Acceleration | PxArticulationCacheFlag.Position | PxArticulationCacheFlag.LinkVelocity | PxArticulationCacheFlag.LinkAcceleration | PxArticulationCacheFlag.RootTransform | PxArticulationCacheFlag.RootVelocities

/// Flags for [`PxArticulationCacheFlag`]
PxArticulationCacheFlags_Set :: bit_set[PxArticulationCacheFlag; _c.uint32_t]


/// Flags to configure the forces reported by articulation link sensors.
PxArticulationSensorFlag :: enum _c.int32_t {
    ForwardDynamicsForces = 0,
    ConstraintSolverForces = 1,
    WorldFrame = 2,
}

/// Flags for [`PxArticulationSensorFlag`]
PxArticulationSensorFlags_Set :: bit_set[PxArticulationSensorFlag; _c.uint8_t]


/// Flag that configures articulation-state updates by PxArticulationReducedCoordinate::updateKinematic.
PxArticulationKinematicFlag :: enum _c.int32_t {
    Position = 0,
    Velocity = 1,
}

/// Flags for [`PxArticulationKinematicFlag`]
PxArticulationKinematicFlags_Set :: bit_set[PxArticulationKinematicFlag; _c.uint8_t]


/// Flags which affect the behavior of PxShapes.
PxShapeFlag :: enum _c.int32_t {
    SimulationShape = 0,
    SceneQueryShape = 1,
    TriggerShape = 2,
    Visualization = 3,
}

/// Flags for [`PxShapeFlag`]
PxShapeFlags_Set :: bit_set[PxShapeFlag; _c.uint8_t]


/// Parameter to addForce() and addTorque() calls, determines the exact operation that is carried out.
PxForceMode :: enum _c.int32_t {
    /// parameter has unit of mass * length / time^2, i.e., a force
    Force = 0,
    /// parameter has unit of mass * length / time, i.e., force * time
    Impulse = 1,
    /// parameter has unit of length / time, i.e., the effect is mass independent: a velocity change.
    VelocityChange = 2,
    /// parameter has unit of length/ time^2, i.e., an acceleration. It gets treated just like a force except the mass is not divided out before integration.
    Acceleration = 3,
}

/// Collection of flags describing the behavior of a rigid body.
PxRigidBodyFlag :: enum _c.int32_t {
    Kinematic = 0,
    UseKinematicTargetForSceneQueries = 1,
    EnableCcd = 2,
    EnableCcdFriction = 3,
    EnableSpeculativeCcd = 4,
    EnablePoseIntegrationPreview = 5,
    EnableCcdMaxContactImpulse = 6,
    RetainAccelerations = 7,
    ForceKineKineNotifications = 8,
    ForceStaticKineNotifications = 9,
    EnableGyroscopicForces = 10,
}

/// Flags for [`PxRigidBodyFlag`]
PxRigidBodyFlags_Set :: bit_set[PxRigidBodyFlag; _c.uint16_t]


/// constraint flags
///
/// eBROKEN is a read only flag
PxConstraintFlag :: enum _c.int32_t {
    Broken = 0,
    ProjectToActor0 = 1,
    ProjectToActor1 = 2,
    CollisionEnabled = 3,
    Visualization = 4,
    DriveLimitsAreForces = 5,
    ImprovedSlerp = 7,
    DisablePreprocessing = 8,
    EnableExtendedLimits = 9,
    GpuCompatible = 10,
    AlwaysUpdate = 11,
    DisableConstraint = 12,
}
PxConstraintFlag_Projection :: PxConstraintFlag.ProjectToActor0 | PxConstraintFlag.ProjectToActor1

/// Flags for [`PxConstraintFlag`]
PxConstraintFlags_Set :: bit_set[PxConstraintFlag; _c.uint16_t]


/// Header for a contact patch where all points share same material and normal
PxContactPatchFlags :: enum _c.int32_t {
    /// Indicates this contact stream has face indices.
    HasFaceIndices = 1,
    /// Indicates this contact stream is modifiable.
    Modifiable = 2,
    /// Indicates this contact stream is notify-only (no contact response).
    ForceNoResponse = 4,
    /// Indicates this contact stream has modified mass ratios
    HasModifiedMassRatios = 8,
    /// Indicates this contact stream has target velocities set
    HasTargetVelocity = 16,
    /// Indicates this contact stream has max impulses set
    HasMaxImpulse = 32,
    /// Indicates this contact stream needs patches re-generated. This is required if the application modified either the contact normal or the material properties
    RegeneratePatches = 64,
    CompressedModifiedContact = 128,
}

/// A class to iterate over a compressed contact stream. This supports read-only access to the various contact formats.
StreamFormat :: enum _c.int32_t {
    SimpleStream = 0,
    ModifiableStream = 1,
    CompressedModifiableStream = 2,
}

/// Flags specifying deletion event types.
PxDeletionEventFlag :: enum _c.int32_t {
    UserRelease = 0,
    MemoryRelease = 1,
}

/// Flags for [`PxDeletionEventFlag`]
PxDeletionEventFlags_Set :: bit_set[PxDeletionEventFlag; _c.uint8_t]


/// Collection of flags describing the actions to take for a collision pair.
PxPairFlag :: enum _c.int32_t {
    SolveContact = 0,
    ModifyContacts = 1,
    NotifyTouchFound = 2,
    NotifyTouchPersists = 3,
    NotifyTouchLost = 4,
    NotifyTouchCcd = 5,
    NotifyThresholdForceFound = 6,
    NotifyThresholdForcePersists = 7,
    NotifyThresholdForceLost = 8,
    NotifyContactPoints = 9,
    DetectDiscreteContact = 10,
    DetectCcdContact = 11,
    PreSolverVelocity = 12,
    PostSolverVelocity = 13,
    ContactEventPose = 14,
    NextFree = 15,
}
PxPairFlag_ContactDefault :: PxPairFlag.SolveContact | PxPairFlag.DetectDiscreteContact
PxPairFlag_TriggerDefault :: PxPairFlag.NotifyTouchFound | PxPairFlag.NotifyTouchLost | PxPairFlag.DetectDiscreteContact

/// Flags for [`PxPairFlag`]
PxPairFlags_Set :: bit_set[PxPairFlag; _c.uint16_t]


/// Collection of flags describing the filter actions to take for a collision pair.
PxFilterFlag :: enum _c.int32_t {
    Kill = 0,
    Suppress = 1,
    Callback = 2,
}
PxFilterFlag_Notify :: PxFilterFlag.Callback
PxFilterFlag_Default :: 0

/// Flags for [`PxFilterFlag`]
PxFilterFlags_Set :: bit_set[PxFilterFlag; _c.uint16_t]


/// Identifies each type of filter object.
PxFilterObjectType :: enum _c.int32_t {
    /// A static rigid body
    RigidStatic = 0,
    /// A dynamic rigid body
    RigidDynamic = 1,
    /// An articulation
    Articulation = 2,
    /// A particle system
    Particlesystem = 3,
    /// A FEM-based soft body
    Softbody = 4,
    /// A FEM-based cloth
    ///
    /// In development
    Femcloth = 5,
    /// A hair system
    ///
    /// In development
    Hairsystem = 6,
    /// internal use only!
    MaxTypeCount = 16,
    /// internal use only!
    Undefined = 15,
}

PxFilterObjectFlag :: enum _c.int32_t {
    Kinematic = 16,
    Trigger = 32,
}

PxPairFilteringMode :: enum _c.int32_t {
    /// Output pair from BP, potentially send to user callbacks, create regular interaction object.
    ///
    /// Enable contact pair filtering between kinematic/static or kinematic/kinematic rigid bodies.
    ///
    /// By default contacts between these are suppressed (see [`PxFilterFlag::eSUPPRESS`]) and don't get reported to the filter mechanism.
    /// Use this mode if these pairs should go through the filtering pipeline nonetheless.
    ///
    /// This mode is not mutable, and must be set in PxSceneDesc at scene creation.
    Keep = 0,
    /// Output pair from BP, create interaction marker. Can be later switched to regular interaction.
    Suppress = 1,
    /// Don't output pair from BP. Cannot be later switched to regular interaction, needs "resetFiltering" call.
    Kill = 2,
}

PxDataAccessFlag :: enum _c.int32_t {
    Readable = 0,
    Writable = 1,
    Device = 2,
}

/// Flags for [`PxDataAccessFlag`]
PxDataAccessFlags_Set :: bit_set[PxDataAccessFlag; _c.uint8_t]


/// Flags which control the behavior of a material.
PxMaterialFlag :: enum _c.int32_t {
    DisableFriction = 0,
    DisableStrongFriction = 1,
    ImprovedPatchFriction = 2,
    CompliantContact = 3,
}

/// Flags for [`PxMaterialFlag`]
PxMaterialFlags_Set :: bit_set[PxMaterialFlag; _c.uint16_t]


/// Enumeration that determines the way in which two material properties will be combined to yield a friction or restitution coefficient for a collision.
///
/// When two actors come in contact with each other, they each have materials with various coefficients, but we only need a single set of coefficients for the pair.
///
/// Physics doesn't have any inherent combinations because the coefficients are determined empirically on a case by case
/// basis. However, simulating this with a pairwise lookup table is often impractical.
///
/// For this reason the following combine behaviors are available:
///
/// eAVERAGE
/// eMIN
/// eMULTIPLY
/// eMAX
///
/// The effective combine mode for the pair is maximum(material0.combineMode, material1.combineMode).
PxCombineMode :: enum _c.int32_t {
    /// Average: (a + b)/2
    Average = 0,
    /// Minimum: minimum(a,b)
    Min = 1,
    /// Multiply: a*b
    Multiply = 2,
    /// Maximum: maximum(a,b)
    Max = 3,
    /// This is not a valid combine mode, it is a sentinel to denote the number of possible values. We assert that the variable's value is smaller than this.
    NValues = 4,
    /// This is not a valid combine mode, it is to assure that the size of the enum type is big enough.
    Pad32 = 2147483647,
}

/// Identifies dirty particle buffers that need to be updated in the particle system.
///
/// This flag can be used mark the device user buffers that are dirty and need to be written to the particle system.
PxParticleBufferFlag :: enum _c.int32_t {
    UpdatePosition = 0,
    UpdateVelocity = 1,
    UpdatePhase = 2,
    UpdateRestposition = 3,
    UpdateCloth = 5,
    UpdateRigid = 6,
    UpdateDiffuseParam = 7,
    UpdateAttachments = 8,
}
PxParticleBufferFlag_None :: 0
PxParticleBufferFlag_All :: PxParticleBufferFlag.UpdatePosition | PxParticleBufferFlag.UpdateVelocity | PxParticleBufferFlag.UpdatePhase | PxParticleBufferFlag.UpdateRestposition | PxParticleBufferFlag.UpdateCloth | PxParticleBufferFlag.UpdateRigid | PxParticleBufferFlag.UpdateDiffuseParam | PxParticleBufferFlag.UpdateAttachments

/// Flags for [`PxParticleBufferFlag`]
PxParticleBufferFlags_Set :: bit_set[PxParticleBufferFlag; _c.uint32_t]


/// Identifies per-particle behavior for a PxParticleSystem.
///
/// See [`PxParticleSystem::createPhase`]().
PxParticlePhaseFlag :: enum _c.uint32_t {
    ParticlePhaseSelfCollide = 20,
    ParticlePhaseSelfCollideFilter = 21,
    ParticlePhaseFluid = 22,
}
PxParticlePhaseFlag_ParticlePhaseGroupMask :: 0x000fffff
PxParticlePhaseFlag_ParticlePhaseFlagsMask :: PxParticlePhaseFlag.ParticlePhaseSelfCollide | PxParticlePhaseFlag.ParticlePhaseSelfCollideFilter | PxParticlePhaseFlag.ParticlePhaseFluid

/// Flags for [`PxParticlePhaseFlag`]
PxParticlePhaseFlags_Set :: bit_set[PxParticlePhaseFlag; _c.uint32_t]


/// Specifies memory space for a PxBuffer instance.
PxBufferType :: enum _c.int32_t {
    Host = 0,
    Device = 1,
}

/// Filtering flags for scene queries.
PxQueryFlag :: enum _c.int32_t {
    Static = 0,
    Dynamic = 1,
    Prefilter = 2,
    Postfilter = 3,
    AnyHit = 4,
    NoBlock = 5,
    DisableHardcodedFilter = 6,
    Reserved = 15,
}

/// Flags for [`PxQueryFlag`]
PxQueryFlags_Set :: bit_set[PxQueryFlag; _c.uint16_t]


/// Classification of scene query hits (intersections).
///
/// - eNONE: Returning this hit type means that the hit should not be reported.
/// - eBLOCK: For all raycast, sweep and overlap queries the nearest eBLOCK type hit will always be returned in PxHitCallback::block member.
/// - eTOUCH: Whenever a raycast, sweep or overlap query was called with non-zero PxHitCallback::nbTouches and PxHitCallback::touches
/// parameters, eTOUCH type hits that are closer or same distance (touchDistance
/// <
/// = blockDistance condition)
/// as the globally nearest eBLOCK type hit, will be reported.
/// - For example, to record all hits from a raycast query, always return eTOUCH.
///
/// All hits in overlap() queries are treated as if the intersection distance were zero.
/// This means the hits are unsorted and all eTOUCH hits are recorded by the callback even if an eBLOCK overlap hit was encountered.
/// Even though all overlap() blocking hits have zero length, only one (arbitrary) eBLOCK overlap hit is recorded in PxHitCallback::block.
/// All overlap() eTOUCH type hits are reported (zero touchDistance
/// <
/// = zero blockDistance condition).
///
/// For raycast/sweep/overlap calls with zero touch buffer or PxHitCallback::nbTouches member,
/// only the closest hit of type eBLOCK is returned. All eTOUCH hits are discarded.
PxQueryHitType :: enum _c.int32_t {
    /// the query should ignore this shape
    None = 0,
    /// a hit on the shape touches the intersection geometry of the query but does not block it
    Touch = 1,
    /// a hit on the shape blocks the query (does not block overlap queries)
    Block = 2,
}

/// Collection of flags providing a mechanism to lock motion along/around a specific axis.
PxRigidDynamicLockFlag :: enum _c.int32_t {
    LockLinearX = 0,
    LockLinearY = 1,
    LockLinearZ = 2,
    LockAngularX = 3,
    LockAngularY = 4,
    LockAngularZ = 5,
}

/// Flags for [`PxRigidDynamicLockFlag`]
PxRigidDynamicLockFlags_Set :: bit_set[PxRigidDynamicLockFlag; _c.uint8_t]


/// Pruning structure used to accelerate scene queries.
///
/// eNONE uses a simple data structure that consumes less memory than the alternatives,
/// but generally has slower query performance.
///
/// eDYNAMIC_AABB_TREE usually provides the fastest queries. However there is a
/// constant per-frame management cost associated with this structure. How much work should
/// be done per frame can be tuned via the [`PxSceneQueryDesc::dynamicTreeRebuildRateHint`]
/// parameter.
///
/// eSTATIC_AABB_TREE is typically used for static objects. It is the same as the
/// dynamic AABB tree, without the per-frame overhead. This can be a good choice for static
/// objects, if no static objects are added, moved or removed after the scene has been
/// created. If there is no such guarantee (e.g. when streaming parts of the world in and out),
/// then the dynamic version is a better choice even for static objects.
PxPruningStructureType :: enum _c.int32_t {
    /// Using a simple data structure
    None = 0,
    /// Using a dynamic AABB tree
    DynamicAabbTree = 1,
    /// Using a static AABB tree
    StaticAabbTree = 2,
    Last = 3,
}

/// Secondary pruning structure used for newly added objects in dynamic trees.
///
/// Dynamic trees (PxPruningStructureType::eDYNAMIC_AABB_TREE) are slowly rebuilt
/// over several frames. A secondary pruning structure holds and manages objects
/// added to the scene while this rebuild is in progress.
///
/// eNONE ignores newly added objects. This means that for a number of frames (roughly
/// defined by PxSceneQueryDesc::dynamicTreeRebuildRateHint) newly added objects will
/// be ignored by scene queries. This can be acceptable when streaming large worlds, e.g.
/// when the objects added at the boundaries of the game world don't immediately need to be
/// visible from scene queries (it would be equivalent to streaming that data in a few frames
/// later). The advantage of this approach is that there is no CPU cost associated with
/// inserting the new objects in the scene query data structures, and no extra runtime cost
/// when performing queries.
///
/// eBUCKET uses a structure similar to PxPruningStructureType::eNONE. Insertion is fast but
/// query cost can be high.
///
/// eINCREMENTAL uses an incremental AABB-tree, with no direct PxPruningStructureType equivalent.
/// Query time is fast but insertion cost can be high.
///
/// eBVH uses a PxBVH structure. This usually offers the best overall performance.
PxDynamicTreeSecondaryPruner :: enum _c.int32_t {
    /// no secondary pruner, new objects aren't visible to SQ for a few frames
    None = 0,
    /// bucket-based secondary pruner, faster updates, slower query time
    Bucket = 1,
    /// incremental-BVH secondary pruner, faster query time, slower updates
    Incremental = 2,
    /// PxBVH-based secondary pruner, good overall performance
    Bvh = 3,
    Last = 4,
}

/// Scene query update mode
///
/// This enum controls what work is done when the scene query system is updated. The updates traditionally happen when PxScene::fetchResults
/// is called. This function then calls PxSceneQuerySystem::finalizeUpdates, where the update mode is used.
///
/// fetchResults/finalizeUpdates will sync changed bounds during simulation and update the scene query bounds in pruners, this work is mandatory.
///
/// eBUILD_ENABLED_COMMIT_ENABLED does allow to execute the new AABB tree build step during fetchResults/finalizeUpdates, additionally
/// the pruner commit is called where any changes are applied. During commit PhysX refits the dynamic scene query tree and if a new tree
/// was built and the build finished the tree is swapped with current AABB tree.
///
/// eBUILD_ENABLED_COMMIT_DISABLED does allow to execute the new AABB tree build step during fetchResults/finalizeUpdates. Pruner commit
/// is not called, this means that refit will then occur during the first scene query following fetchResults/finalizeUpdates, or may be forced
/// by the method PxScene::flushQueryUpdates() / PxSceneQuerySystemBase::flushUpdates().
///
/// eBUILD_DISABLED_COMMIT_DISABLED no further scene query work is executed. The scene queries update needs to be called manually, see
/// PxScene::sceneQueriesUpdate (see that function's doc for the equivalent PxSceneQuerySystem sequence). It is recommended to call
/// PxScene::sceneQueriesUpdate right after fetchResults/finalizeUpdates as the pruning structures are not updated.
PxSceneQueryUpdateMode :: enum _c.int32_t {
    /// Both scene query build and commit are executed.
    BuildEnabledCommitEnabled = 0,
    /// Scene query build only is executed.
    BuildEnabledCommitDisabled = 1,
    /// No work is done, no update of scene queries
    BuildDisabledCommitDisabled = 2,
}

/// Built-in enum for default PxScene pruners
///
/// This is passed as a pruner index to various functions in the following APIs.
PxScenePrunerIndex :: enum _c.uint32_t {
    PxScenePrunerStatic = 0,
    PxScenePrunerDynamic = 1,
    PxSceneCompoundPruner = 4294967295,
}

/// Broad phase algorithm used in the simulation
///
/// eSAP is a good generic choice with great performance when many objects are sleeping. Performance
/// can degrade significantly though, when all objects are moving, or when large numbers of objects
/// are added to or removed from the broad phase. This algorithm does not need world bounds to be
/// defined in order to work.
///
/// eMBP is an alternative broad phase algorithm that does not suffer from the same performance
/// issues as eSAP when all objects are moving or when inserting large numbers of objects. However
/// its generic performance when many objects are sleeping might be inferior to eSAP, and it requires
/// users to define world bounds in order to work.
///
/// eABP is a revisited implementation of MBP, which automatically manages broad-phase regions.
/// It offers the convenience of eSAP (no need to define world bounds or regions) and the performance
/// of eMBP when a lot of objects are moving. While eSAP can remain faster when most objects are
/// sleeping and eMBP can remain faster when it uses a large number of properly-defined regions,
/// eABP often gives the best performance on average and the best memory usage.
///
/// ePABP is a parallel implementation of ABP. It can often be the fastest (CPU) broadphase, but it
/// can use more memory than ABP.
///
/// eGPU is a GPU implementation of the incremental sweep and prune approach. Additionally, it uses a ABP-style
/// initial pair generation approach to avoid large spikes when inserting shapes. It not only has the advantage
/// of traditional SAP approch which is good for when many objects are sleeping, but due to being fully parallel,
/// it also is great when lots of shapes are moving or for runtime pair insertion and removal. It can become a
/// performance bottleneck if there are a very large number of shapes roughly projecting to the same values
/// on a given axis. If the scene has a very large number of shapes in an actor, e.g. a humanoid, it is recommended
/// to use an aggregate to represent multi-shape or multi-body actors to minimize stress placed on the broad phase.
PxBroadPhaseType :: enum _c.int32_t {
    /// 3-axes sweep-and-prune
    Sap = 0,
    /// Multi box pruning
    Mbp = 1,
    /// Automatic box pruning
    Abp = 2,
    /// Parallel automatic box pruning
    Pabp = 3,
    /// GPU broad phase
    Gpu = 4,
    Last = 5,
}

/// Enum for selecting the friction algorithm used for simulation.
///
/// [`PxFrictionType::ePATCH`] selects the patch friction model which typically leads to the most stable results at low solver iteration counts and is also quite inexpensive, as it uses only
/// up to four scalar solver constraints per pair of touching objects.  The patch friction model is the same basic strong friction algorithm as PhysX 3.2 and before.
///
/// [`PxFrictionType::eONE_DIRECTIONAL`] is a simplification of the Coulomb friction model, in which the friction for a given point of contact is applied in the alternating tangent directions of
/// the contact's normal.  This simplification allows us to reduce the number of iterations required for convergence but is not as accurate as the two directional model.
///
/// [`PxFrictionType::eTWO_DIRECTIONAL`] is identical to the one directional model, but it applies friction in both tangent directions simultaneously.  This hurts convergence a bit so it
/// requires more solver iterations, but is more accurate.  Like the one directional model, it is applied at every contact point, which makes it potentially more expensive
/// than patch friction for scenarios with many contact points.
///
/// [`PxFrictionType::eFRICTION_COUNT`] is the total numer of friction models supported by the SDK.
PxFrictionType :: enum _c.int32_t {
    /// Select default patch-friction model.
    Patch = 0,
    /// Select one directional per-contact friction model.
    OneDirectional = 1,
    /// Select two directional per-contact friction model.
    TwoDirectional = 2,
    /// The total number of friction models supported by the SDK.
    FrictionCount = 3,
}

/// Enum for selecting the type of solver used for the simulation.
///
/// [`PxSolverType::ePGS`] selects the iterative sequential impulse solver. This is the same kind of solver used in PhysX 3.4 and earlier releases.
///
/// [`PxSolverType::eTGS`] selects a non linear iterative solver. This kind of solver can lead to improved convergence and handle large mass ratios, long chains and jointed systems better. It is slightly more expensive than the default solver and can introduce more energy to correct joint and contact errors.
PxSolverType :: enum _c.int32_t {
    /// Projected Gauss-Seidel iterative solver
    Pgs = 0,
    /// Default Temporal Gauss-Seidel solver
    Tgs = 1,
}

/// flags for configuring properties of the scene
PxSceneFlag :: enum _c.int32_t {
    EnableActiveActors = 0,
    EnableCcd = 1,
    DisableCcdResweep = 2,
    EnablePcm = 6,
    DisableContactReportBufferResize = 7,
    DisableContactCache = 8,
    RequireRwLock = 9,
    EnableStabilization = 10,
    EnableAveragePoint = 11,
    ExcludeKinematicsFromActiveActors = 12,
    EnableGpuDynamics = 13,
    EnableEnhancedDeterminism = 14,
    EnableFrictionEveryIteration = 15,
    SuppressReadback = 16,
    ForceReadback = 17,
}
PxSceneFlag_MutableFlags :: PxSceneFlag.EnableActiveActors | PxSceneFlag.ExcludeKinematicsFromActiveActors | PxSceneFlag.SuppressReadback

/// Flags for [`PxSceneFlag`]
PxSceneFlags_Set :: bit_set[PxSceneFlag; _c.uint32_t]


/// Debug visualization parameters.
///
/// [`PxVisualizationParameter::eSCALE`] is the master switch for enabling visualization, please read the corresponding documentation
/// for further details.
PxVisualizationParameter :: enum _c.int32_t {
    /// This overall visualization scale gets multiplied with the individual scales. Setting to zero ignores all visualizations. Default is 0.
    ///
    /// The below settings permit the debug visualization of various simulation properties.
    /// The setting is either zero, in which case the property is not drawn. Otherwise it is a scaling factor
    /// that determines the size of the visualization widgets.
    ///
    /// Only objects for which visualization is turned on using setFlag(eVISUALIZATION) are visualized (see [`PxActorFlag::eVISUALIZATION`], #PxShapeFlag::eVISUALIZATION, ...).
    /// Contacts are visualized if they involve a body which is being visualized.
    /// Default is 0.
    ///
    /// Notes:
    /// - to see any visualization, you have to set PxVisualizationParameter::eSCALE to nonzero first.
    /// - the scale factor has been introduced because it's difficult (if not impossible) to come up with a
    /// good scale for 3D vectors. Normals are normalized and their length is always 1. But it doesn't mean
    /// we should render a line of length 1. Depending on your objects/scene, this might be completely invisible
    /// or extremely huge. That's why the scale factor is here, to let you tune the length until it's ok in
    /// your scene.
    /// - however, things like collision shapes aren't ambiguous. They are clearly defined for example by the
    /// triangles
    /// &
    /// polygons themselves, and there's no point in scaling that. So the visualization widgets
    /// are only scaled when it makes sense.
    ///
    /// Range:
    /// [0, PX_MAX_F32)
    /// Default:
    /// 0
    Scale = 0,
    /// Visualize the world axes.
    WorldAxes = 1,
    /// Visualize a bodies axes.
    BodyAxes = 2,
    /// Visualize a body's mass axes.
    ///
    /// This visualization is also useful for visualizing the sleep state of bodies. Sleeping bodies are drawn in
    /// black, while awake bodies are drawn in white. If the body is sleeping and part of a sleeping group, it is
    /// drawn in red.
    BodyMassAxes = 3,
    /// Visualize the bodies linear velocity.
    BodyLinVelocity = 4,
    /// Visualize the bodies angular velocity.
    BodyAngVelocity = 5,
    /// Visualize contact points. Will enable contact information.
    ContactPoint = 6,
    /// Visualize contact normals. Will enable contact information.
    ContactNormal = 7,
    /// Visualize contact errors. Will enable contact information.
    ContactError = 8,
    /// Visualize Contact forces. Will enable contact information.
    ContactForce = 9,
    /// Visualize actor axes.
    ActorAxes = 10,
    /// Visualize bounds (AABBs in world space)
    CollisionAabbs = 11,
    /// Shape visualization
    CollisionShapes = 12,
    /// Shape axis visualization
    CollisionAxes = 13,
    /// Compound visualization (compound AABBs in world space)
    CollisionCompounds = 14,
    /// Mesh
    /// &
    /// convex face normals
    CollisionFnormals = 15,
    /// Active edges for meshes
    CollisionEdges = 16,
    /// Static pruning structures
    CollisionStatic = 17,
    /// Dynamic pruning structures
    CollisionDynamic = 18,
    /// Joint local axes
    JointLocalFrames = 19,
    /// Joint limits
    JointLimits = 20,
    /// Visualize culling box
    CullBox = 21,
    /// MBP regions
    MbpRegions = 22,
    /// Renders the simulation mesh instead of the collision mesh (only available for tetmeshes)
    SimulationMesh = 23,
    /// Renders the SDF of a mesh instead of the collision mesh (only available for triangle meshes with SDFs)
    Sdf = 24,
    /// This is not a parameter, it just records the current number of parameters (as maximum(PxVisualizationParameter)+1) for use in loops.
    NumValues = 25,
    /// This is not a parameter, it just records the current number of parameters (as maximum(PxVisualizationParameter)+1) for use in loops.
    ForceDword = 2147483647,
}

/// Different types of rigid body collision pair statistics.
RbPairStatsType :: enum _c.int32_t {
    /// Shape pairs processed as discrete contact pairs for the current simulation step.
    DiscreteContactPairs = 0,
    /// Shape pairs processed as swept integration pairs for the current simulation step.
    ///
    /// Counts the pairs for which special CCD (continuous collision detection) work was actually done and NOT the number of pairs which were configured for CCD.
    /// Furthermore, there can be multiple CCD passes and all processed pairs of all passes are summed up, hence the number can be larger than the amount of pairs which have been configured for CCD.
    CcdPairs = 1,
    /// Shape pairs processed with user contact modification enabled for the current simulation step.
    ModifiedContactPairs = 2,
    /// Trigger shape pairs processed for the current simulation step.
    TriggerPairs = 3,
}

/// These flags determine what data is read or written to the gpu softbody.
PxSoftBodyDataFlag :: enum _c.int32_t {
    /// The collision mesh tetrahedron indices (quadruples of int32)
    TetIndices = 0,
    /// The collision mesh cauchy stress tensors (float 3x3 matrices)
    TetStress = 1,
    /// The collision mesh tetrahedron von Mises stress (float scalar)
    TetStresscoeff = 2,
    /// The collision mesh tetrahedron rest poses (float 3x3 matrices)
    TetRestPoses = 3,
    /// The collision mesh tetrahedron orientations (quaternions, quadruples of float)
    TetRotations = 4,
    /// The collision mesh vertex positions and their inverted mass in the 4th component (quadruples of float)
    TetPositionInvMass = 5,
    /// The simulation mesh tetrahedron indices (quadruples of int32)
    SimTetIndices = 6,
    /// The simulation mesh vertex velocities and their inverted mass in the 4th component (quadruples of float)
    SimVelocityInvMass = 7,
    /// The simulation mesh vertex positions and their inverted mass in the 4th component (quadruples of float)
    SimPositionInvMass = 8,
    /// The simulation mesh kinematic target positions
    SimKinematicTarget = 9,
}

/// Identifies input and output buffers for PxHairSystem
PxHairSystemData :: enum _c.int32_t {
    PositionInvmass = 0,
    Velocity = 1,
}
PxHairSystemData_None :: 0
PxHairSystemData_All :: PxHairSystemData.PositionInvmass | PxHairSystemData.Velocity

/// Flags for [`PxHairSystemData`]
PxHairSystemDataFlags_Set :: bit_set[PxHairSystemData; _c.uint32_t]


/// Binary settings for hair system simulation
PxHairSystemFlag :: enum _c.int32_t {
    DisableSelfCollision = 0,
    DisableExternalCollision = 1,
    DisableTwosidedAttachment = 2,
}

/// Flags for [`PxHairSystemFlag`]
PxHairSystemFlags_Set :: bit_set[PxHairSystemFlag; _c.uint32_t]


/// Identifies each type of information for retrieving from actor.
PxActorCacheFlag :: enum _c.int32_t {
    ActorData = 0,
    Force = 2,
    Torque = 3,
}

/// Flags for [`PxActorCacheFlag`]
PxActorCacheFlags_Set :: bit_set[PxActorCacheFlag; _c.uint16_t]


/// PVD scene Flags. They are disabled by default, and only works if PxPvdInstrumentationFlag::eDEBUG is set.
PxPvdSceneFlag :: enum _c.int32_t {
    TransmitContacts = 0,
    TransmitScenequeries = 1,
    TransmitConstraints = 2,
}

/// Flags for [`PxPvdSceneFlag`]
PxPvdSceneFlags_Set :: bit_set[PxPvdSceneFlag; _c.uint8_t]


/// Identifies each type of actor for retrieving actors from a scene.
///
/// [`PxArticulationLink`] objects are not supported. Use the #PxArticulationReducedCoordinate object to retrieve all its links.
PxActorTypeFlag :: enum _c.int32_t {
    RigidStatic = 0,
    RigidDynamic = 1,
}

/// Flags for [`PxActorTypeFlag`]
PxActorTypeFlags_Set :: bit_set[PxActorTypeFlag; _c.uint16_t]


/// Extra data item types for contact pairs.
PxContactPairExtraDataType :: enum _c.int32_t {
    /// see [`PxContactPairVelocity`]
    PreSolverVelocity = 0,
    /// see [`PxContactPairVelocity`]
    PostSolverVelocity = 1,
    /// see [`PxContactPairPose`]
    ContactEventPose = 2,
    /// see [`PxContactPairIndex`]
    ContactPairIndex = 3,
}

/// Collection of flags providing information on contact report pairs.
PxContactPairHeaderFlag :: enum _c.int32_t {
    RemovedActor0 = 0,
    RemovedActor1 = 1,
}

/// Flags for [`PxContactPairHeaderFlag`]
PxContactPairHeaderFlags_Set :: bit_set[PxContactPairHeaderFlag; _c.uint16_t]


/// Collection of flags providing information on contact report pairs.
PxContactPairFlag :: enum _c.int32_t {
    RemovedShape0 = 0,
    RemovedShape1 = 1,
    ActorPairHasFirstTouch = 2,
    ActorPairLostTouch = 3,
    InternalHasImpulses = 4,
    InternalContactsAreFlipped = 5,
}

/// Flags for [`PxContactPairFlag`]
PxContactPairFlags_Set :: bit_set[PxContactPairFlag; _c.uint16_t]


/// Collection of flags providing information on trigger report pairs.
PxTriggerPairFlag :: enum _c.int32_t {
    RemovedShapeTrigger = 0,
    RemovedShapeOther = 1,
    NextFree = 2,
}

/// Flags for [`PxTriggerPairFlag`]
PxTriggerPairFlags_Set :: bit_set[PxTriggerPairFlag; _c.uint8_t]


/// Identifies input and output buffers for PxSoftBody.
PxSoftBodyData :: enum _c.int32_t {
    PositionInvmass = 0,
    SimPositionInvmass = 2,
    SimVelocity = 3,
    SimKinematicTarget = 4,
}
PxSoftBodyData_None :: 0
PxSoftBodyData_All :: PxSoftBodyData.PositionInvmass | PxSoftBodyData.SimPositionInvmass | PxSoftBodyData.SimVelocity | PxSoftBodyData.SimKinematicTarget

/// Flags for [`PxSoftBodyData`]
PxSoftBodyDataFlags_Set :: bit_set[PxSoftBodyData; _c.uint32_t]


/// Flags to enable or disable special modes of a SoftBody
PxSoftBodyFlag :: enum _c.int32_t {
    DisableSelfCollision = 0,
    ComputeStressTensor = 1,
    EnableCcd = 2,
    DisplaySimMesh = 3,
    Kinematic = 4,
    PartiallyKinematic = 5,
}

/// Flags for [`PxSoftBodyFlag`]
PxSoftBodyFlags_Set :: bit_set[PxSoftBodyFlag; _c.uint32_t]


/// The type of controller, eg box, sphere or capsule.
PxControllerShapeType :: enum _c.int32_t {
    /// A box controller.
    Box = 0,
    /// A capsule controller
    Capsule = 1,
    /// A capsule controller
    ForceDword = 2147483647,
}

/// specifies how a CCT interacts with non-walkable parts.
///
/// This is only used when slopeLimit is non zero. It is currently enabled for static actors only, and not supported for spheres or capsules.
PxControllerNonWalkableMode :: enum _c.int32_t {
    /// Stops character from climbing up non-walkable slopes, but doesn't move it otherwise
    PreventClimbing = 0,
    /// Stops character from climbing up non-walkable slopes, and forces it to slide down those slopes
    PreventClimbingAndForceSliding = 1,
}

/// specifies which sides a character is colliding with.
PxControllerCollisionFlag :: enum _c.int32_t {
    CollisionSides = 0,
    CollisionUp = 1,
    CollisionDown = 2,
}

/// Flags for [`PxControllerCollisionFlag`]
PxControllerCollisionFlags_Set :: bit_set[PxControllerCollisionFlag; _c.uint8_t]


PxCapsuleClimbingMode :: enum _c.int32_t {
    /// Standard mode, let the capsule climb over surfaces according to impact normal
    Easy = 0,
    /// Constrained mode, try to limit climbing according to the step offset
    Constrained = 1,
    Last = 2,
}

/// specifies controller behavior
PxControllerBehaviorFlag :: enum _c.int32_t {
    CctCanRideOnObject = 0,
    CctSlide = 1,
    CctUserDefinedRide = 2,
}

/// Flags for [`PxControllerBehaviorFlag`]
PxControllerBehaviorFlags_Set :: bit_set[PxControllerBehaviorFlag; _c.uint8_t]


/// specifies debug-rendering flags
PxControllerDebugRenderFlag :: enum _c.uint32_t {
    TemporalBv = 0,
    CachedBv = 1,
    Obstacles = 2,
}
PxControllerDebugRenderFlag_None :: 0
PxControllerDebugRenderFlag_All :: PxControllerDebugRenderFlag.TemporalBv | PxControllerDebugRenderFlag.CachedBv | PxControllerDebugRenderFlag.Obstacles

/// Flags for [`PxControllerDebugRenderFlag`]
PxControllerDebugRenderFlags_Set :: bit_set[PxControllerDebugRenderFlag; _c.uint32_t]


/// Defines the number of bits per subgrid pixel
PxSdfBitsPerSubgridPixel :: enum _c.int32_t {
    /// 8 bit per subgrid pixel (values will be stored as normalized integers)
    E8BitPerPixel = 1,
    /// 16 bit per subgrid pixel (values will be stored as normalized integers)
    E16BitPerPixel = 2,
    /// 32 bit per subgrid pixel (values will be stored as floats in world scale units)
    E32BitPerPixel = 4,
}

/// Flags which describe the format and behavior of a convex mesh.
PxConvexFlag :: enum _c.int32_t {
    E16BitIndices = 0,
    ComputeConvex = 1,
    CheckZeroAreaTriangles = 2,
    QuantizeInput = 3,
    DisableMeshValidation = 4,
    PlaneShifting = 5,
    FastInertiaComputation = 6,
    GpuCompatible = 7,
    ShiftVertices = 8,
}

/// Flags for [`PxConvexFlag`]
PxConvexFlags_Set :: bit_set[PxConvexFlag; _c.uint16_t]


/// Defines the tetrahedron structure of a mesh.
PxMeshFormat :: enum _c.int32_t {
    /// Normal tetmesh with arbitrary tetrahedra
    TetMesh = 0,
    /// 6 tetrahedra in a row will form a hexahedron
    HexMesh = 1,
}

/// Desired build strategy for PxMeshMidPhase::eBVH34
PxBVH34BuildStrategy :: enum _c.int32_t {
    /// Fast build strategy. Fast build speed, good runtime performance in most cases. Recommended for runtime mesh cooking.
    Fast = 0,
    /// Default build strategy. Medium build speed, good runtime performance in all cases.
    Default = 1,
    /// SAH build strategy. Slower builds, slightly improved runtime performance in some cases.
    Sah = 2,
    Last = 3,
}

/// Result from convex cooking.
PxConvexMeshCookingResult :: enum _c.int32_t {
    /// Convex mesh cooking succeeded.
    Success = 0,
    /// Convex mesh cooking failed, algorithm couldn't find 4 initial vertices without a small triangle.
    ZeroAreaTestFailed = 1,
    /// Convex mesh cooking succeeded, but the algorithm has reached the 255 polygons limit.
    /// The produced hull does not contain all input vertices. Try to simplify the input vertices
    /// or try to use the eINFLATE_CONVEX or the eQUANTIZE_INPUT flags.
    PolygonsLimitReached = 2,
    /// Something unrecoverable happened. Check the error stream to find out what.
    Failure = 3,
}

/// Enumeration for convex mesh cooking algorithms.
PxConvexMeshCookingType :: enum _c.int32_t {
    /// The Quickhull algorithm constructs the hull from the given input points. The resulting hull
    /// will only contain a subset of the input points.
    Quickhull = 0,
}

/// Result from triangle mesh cooking
PxTriangleMeshCookingResult :: enum _c.int32_t {
    /// Everything is A-OK.
    Success = 0,
    /// a triangle is too large for well-conditioned results. Tessellate the mesh for better behavior, see the user guide section on cooking for more details.
    LargeTriangle = 1,
    /// Something unrecoverable happened. Check the error stream to find out what.
    Failure = 2,
}

/// Enum for the set of mesh pre-processing parameters.
PxMeshPreprocessingFlag :: enum _c.int32_t {
    WeldVertices = 0,
    DisableCleanMesh = 1,
    DisableActiveEdgesPrecompute = 2,
    Force32bitIndices = 3,
    EnableVertMapping = 4,
    EnableInertia = 5,
}

/// Flags for [`PxMeshPreprocessingFlag`]
PxMeshPreprocessingFlags_Set :: bit_set[PxMeshPreprocessingFlag; _c.uint32_t]


/// Unique identifiers for extensions classes which implement a constraint based on PxConstraint.
///
/// Users which want to create their own custom constraint types should choose an ID larger or equal to eNEXT_FREE_ID
/// and not eINVALID_ID.
PxConstraintExtIDs :: enum _c.int32_t {
    Joint = 0,
    VehicleSuspLimitDeprecated = 1,
    VehicleStickyTyreDeprecated = 2,
    VehicleJoint = 3,
    NextFreeId = 4,
    InvalidId = 2147483647,
}

/// an enumeration of PhysX' built-in joint types
PxJointConcreteType :: enum _c.int32_t {
    Spherical = 256,
    Revolute = 257,
    Prismatic = 258,
    Fixed = 259,
    Distance = 260,
    D6 = 261,
    Contact = 262,
    Gear = 263,
    RackAndPinion = 264,
    Last = 265,
}

/// an enumeration for specifying one or other of the actors referenced by a joint
PxJointActorIndex :: enum _c.int32_t {
    Actor0 = 0,
    Actor1 = 1,
    Count = 2,
}

/// flags for configuring the drive of a PxDistanceJoint
PxDistanceJointFlag :: enum _c.int32_t {
    MaxDistanceEnabled = 1,
    MinDistanceEnabled = 2,
    SpringEnabled = 3,
}

/// Flags for [`PxDistanceJointFlag`]
PxDistanceJointFlags_Set :: bit_set[PxDistanceJointFlag; _c.uint16_t]


/// Flags specific to the prismatic joint.
PxPrismaticJointFlag :: enum _c.int32_t {
    LimitEnabled = 1,
}

/// Flags for [`PxPrismaticJointFlag`]
PxPrismaticJointFlags_Set :: bit_set[PxPrismaticJointFlag; _c.uint16_t]


/// Flags specific to the Revolute Joint.
PxRevoluteJointFlag :: enum _c.int32_t {
    LimitEnabled = 0,
    DriveEnabled = 1,
    DriveFreespin = 2,
}

/// Flags for [`PxRevoluteJointFlag`]
PxRevoluteJointFlags_Set :: bit_set[PxRevoluteJointFlag; _c.uint16_t]


/// Flags specific to the spherical joint.
PxSphericalJointFlag :: enum _c.int32_t {
    LimitEnabled = 1,
}

/// Flags for [`PxSphericalJointFlag`]
PxSphericalJointFlags_Set :: bit_set[PxSphericalJointFlag; _c.uint16_t]


/// Used to specify one of the degrees of freedom of  a D6 joint.
PxD6Axis :: enum _c.int32_t {
    /// motion along the X axis
    X = 0,
    /// motion along the Y axis
    Y = 1,
    /// motion along the Z axis
    Z = 2,
    /// motion around the X axis
    Twist = 3,
    /// motion around the Y axis
    Swing1 = 4,
    /// motion around the Z axis
    Swing2 = 5,
    Count = 6,
}

/// Used to specify the range of motions allowed for a degree of freedom in a D6 joint.
PxD6Motion :: enum _c.int32_t {
    /// The DOF is locked, it does not allow relative motion.
    Locked = 0,
    /// The DOF is limited, it only allows motion within a specific range.
    Limited = 1,
    /// The DOF is free and has its full range of motion.
    Free = 2,
}

/// Used to specify which axes of a D6 joint are driven.
///
/// Each drive is an implicit force-limited damped spring:
///
/// force = spring * (target position - position) + damping * (targetVelocity - velocity)
///
/// Alternatively, the spring may be configured to generate a specified acceleration instead of a force.
///
/// A linear axis is affected by drive only if the corresponding drive flag is set. There are two possible models
/// for angular drive: swing/twist, which may be used to drive one or more angular degrees of freedom, or slerp,
/// which may only be used to drive all three angular degrees simultaneously.
PxD6Drive :: enum _c.int32_t {
    /// drive along the X-axis
    X = 0,
    /// drive along the Y-axis
    Y = 1,
    /// drive along the Z-axis
    Z = 2,
    /// drive of displacement from the X-axis
    Swing = 3,
    /// drive of the displacement around the X-axis
    Twist = 4,
    /// drive of all three angular degrees along a SLERP-path
    Slerp = 5,
    Count = 6,
}

/// flags for configuring the drive model of a PxD6Joint
PxD6JointDriveFlag :: enum _c.int32_t {
    Acceleration = 0,
}

/// Flags for [`PxD6JointDriveFlag`]
PxD6JointDriveFlags_Set :: bit_set[PxD6JointDriveFlag; _c.uint32_t]


/// Collision filtering operations.
PxFilterOp :: enum _c.int32_t {
    PxFilteropAnd = 0,
    PxFilteropOr = 1,
    PxFilteropXor = 2,
    PxFilteropNand = 3,
    PxFilteropNor = 4,
    PxFilteropNxor = 5,
    PxFilteropSwapAnd = 6,
}

/// If a thread ends up waiting for work it will find itself in a spin-wait loop until work becomes available.
/// Three strategies are available to limit wasted cycles.
/// The strategies are as follows:
/// a) wait until a work task signals the end of the spin-wait period.
/// b) yield the thread by providing a hint to reschedule thread execution, thereby allowing other threads to run.
/// c) yield the processor by informing it that it is waiting for work and requesting it to more efficiently use compute resources.
PxDefaultCpuDispatcherWaitForWorkMode :: enum _c.int32_t {
    WaitForWork = 0,
    YieldThread = 1,
    YieldProcessor = 2,
}

PxBatchQueryStatus :: enum _c.int32_t {
    /// This is the initial state before a query starts.
    Pending = 0,
    /// The query is finished; results have been written into the result and hit buffers.
    Success = 1,
    /// The query results were incomplete due to touch hit buffer overflow. Blocking hit is still correct.
    Overflow = 2,
}

/// types of instrumentation that PVD can do.
PxPvdInstrumentationFlag :: enum _c.int32_t {
    Debug = 0,
    Profile = 1,
    Memory = 2,
}
PxPvdInstrumentationFlag_All :: PxPvdInstrumentationFlag.Debug | PxPvdInstrumentationFlag.Profile | PxPvdInstrumentationFlag.Memory

/// Flags for [`PxPvdInstrumentationFlag`]
PxPvdInstrumentationFlags_Set :: bit_set[PxPvdInstrumentationFlag; _c.uint8_t]


PxMat34 :: distinct rawptr

PxAllocatorCallback :: struct {
    _pad0: [8]u8,
}


PxAssertHandler :: struct {
    _pad0: [8]u8,
}


PxFoundation :: struct {
    _pad0: [8]u8,
}


PxAllocator :: struct {
};

PxRawAllocator :: struct {
};

PxVirtualAllocatorCallback :: struct {
    _pad0: [8]u8,
}


PxVirtualAllocator :: struct {
    _pad0: [16]u8,
}


PxUserAllocated :: struct {
};

PxTempAllocatorChunk :: struct #raw_union {
    mNext: ^PxTempAllocatorChunk,
    mIndex: _c.uint32_t,
    mPad: [16]_c.uint8_t,
};

PxTempAllocator :: struct {
};

PxLogTwo :: distinct rawptr

PxUnConst :: distinct rawptr

PxBitAndByte :: struct {
    _pad0: [1]u8,
}


PxBitMap :: struct {
    _pad0: [16]u8,
}


PxVec3 :: struct {
    x: _c.float,
    y: _c.float,
    z: _c.float,
}


PxVec3Padded :: struct {
    using _: PxVec3,
    padding: _c.uint32_t,
}


PxQuat :: struct {
    x: _c.float,
    y: _c.float,
    z: _c.float,
    w: _c.float,
}


PxTransform :: struct {
    q: PxQuat,
    p: PxVec3,
}


PxTransformPadded :: struct {
    transform: PxTransform,
    padding: _c.uint32_t,
}


PxMat33 :: struct {
    column0: PxVec3,
    column1: PxVec3,
    column2: PxVec3,
}


PxBounds3 :: struct {
    minimum: PxVec3,
    maximum: PxVec3,
}


PxErrorCallback :: struct {
    _pad0: [8]u8,
}


PxAllocationListener :: struct {
    _pad0: [8]u8,
}


PxBroadcastingAllocator :: struct {
    using _: PxAllocatorCallback,
    _pad1: [168]u8,
}


PxBroadcastingErrorCallback :: struct {
    using _: PxErrorCallback,
    _pad1: [152]u8,
}


PxHash :: distinct rawptr

PxInputStream :: struct {
    _pad0: [8]u8,
}


PxInputData :: struct {
    using _: PxInputStream,
}


PxOutputStream :: struct {
    _pad0: [8]u8,
}


PxVec4 :: struct {
    x: _c.float,
    y: _c.float,
    z: _c.float,
    w: _c.float,
}


PxMat44 :: struct {
    column0: PxVec4,
    column1: PxVec4,
    column2: PxVec4,
    column3: PxVec4,
}


PxPlane :: struct {
    n: PxVec3,
    d: _c.float,
}


Interpolation :: struct {
};

PxMutexImpl :: struct {
};

PxReadWriteLock :: struct {
    _pad0: [8]u8,
}


PxProfilerCallback :: struct {
    _pad0: [8]u8,
}


PxProfileScoped :: struct {
    mCallback: ^PxProfilerCallback,
    mEventName: ^_c.char,
    mProfilerData: rawptr,
    mContextId: _c.uint64_t,
    mDetached: _c.bool,
    _pad5: [7]u8,
}


PxSListEntry :: struct #align(16){
    _pad0: [16]u8,
}


PxSListImpl :: struct {
};

PxSyncImpl :: struct {
};

PxRunnable :: struct {
    _pad0: [8]u8,
}


PxCounterFrequencyToTensOfNanos :: struct {
    mNumerator: _c.uint64_t,
    mDenominator: _c.uint64_t,
}


PxTime :: struct {
    _pad0: [8]u8,
}


PxVec2 :: struct {
    x: _c.float,
    y: _c.float,
}


PxStridedData :: struct {
    stride: _c.uint32_t,
    _pad1: [4]u8,
    data: rawptr,
}


PxBoundedData :: struct {
    using _: PxStridedData,
    count: _c.uint32_t,
    _pad5: [4]u8,
}


PxDebugPoint :: struct {
    pos: PxVec3,
    color: _c.uint32_t,
}


PxDebugLine :: struct {
    pos0: PxVec3,
    color0: _c.uint32_t,
    pos1: PxVec3,
    color1: _c.uint32_t,
}


PxDebugTriangle :: struct {
    pos0: PxVec3,
    color0: _c.uint32_t,
    pos1: PxVec3,
    color1: _c.uint32_t,
    pos2: PxVec3,
    color2: _c.uint32_t,
}


PxDebugText :: struct {
    position: PxVec3,
    size: _c.float,
    color: _c.uint32_t,
    _pad3: [4]u8,
    string: ^_c.char,
}


PxRenderBuffer :: struct {
    _pad0: [8]u8,
}


PxProcessPxBaseCallback :: struct {
    _pad0: [8]u8,
}


PxSerializationContext :: struct {
    _pad0: [8]u8,
}


PxDeserializationContext :: struct {
    _pad0: [16]u8,
}


PxSerializationRegistry :: struct {
    _pad0: [8]u8,
}


PxCollection :: struct {
    _pad0: [8]u8,
}


PxTypeInfo :: distinct rawptr

PxFEMSoftBodyMaterial :: distinct rawptr

PxFEMClothMaterial :: distinct rawptr

PxPBDMaterial :: distinct rawptr

PxFLIPMaterial :: distinct rawptr

PxMPMMaterial :: distinct rawptr

PxCustomMaterial :: distinct rawptr

PxBVH33TriangleMesh :: distinct rawptr

PxParticleSystem :: distinct rawptr

PxPBDParticleSystem :: distinct rawptr

PxFLIPParticleSystem :: distinct rawptr

PxMPMParticleSystem :: distinct rawptr

PxCustomParticleSystem :: distinct rawptr

PxSoftBody :: distinct rawptr

PxFEMCloth :: distinct rawptr

PxHairSystem :: distinct rawptr

PxParticleBuffer :: distinct rawptr

PxParticleAndDiffuseBuffer :: distinct rawptr

PxParticleClothBuffer :: distinct rawptr

PxParticleRigidBuffer :: distinct rawptr

PxBase :: struct {
    _pad0: [16]u8,
}


PxRefCounted :: struct {
    using _: PxBase,
}


PxTolerancesScale :: struct {
    length: _c.float,
    speed: _c.float,
}


PxStringTable :: struct {
    _pad0: [8]u8,
}


PxSerializer :: struct {
    _pad0: [8]u8,
}


PxMetaDataEntry :: struct {
    type: ^_c.char,
    name: ^_c.char,
    offset: _c.uint32_t,
    size: _c.uint32_t,
    count: _c.uint32_t,
    offsetSize: _c.uint32_t,
    flags: _c.uint32_t,
    alignment: _c.uint32_t,
}


PxInsertionCallback :: struct {
    _pad0: [8]u8,
}


PxTaskManager :: struct {
    _pad0: [8]u8,
}


PxCpuDispatcher :: struct {
    _pad0: [8]u8,
}


PxBaseTask :: struct {
    _pad0: [24]u8,
}


PxTask :: struct {
    using _: PxBaseTask,
    _pad1: [8]u8,
}


PxLightCpuTask :: struct {
    using _: PxBaseTask,
    _pad1: [16]u8,
}


PxGeometry :: struct {
    _pad0: [4]u8,
    mTypePadding: _c.float,
}


PxBoxGeometry :: struct {
    using _: PxGeometry,
    halfExtents: PxVec3,
}


PxBVHRaycastCallback :: struct {
    _pad0: [8]u8,
}


PxBVHOverlapCallback :: struct {
    _pad0: [8]u8,
}


PxBVHTraversalCallback :: struct {
    _pad0: [8]u8,
}


PxBVH :: struct {
    using _: PxBase,
}


PxCapsuleGeometry :: struct {
    using _: PxGeometry,
    radius: _c.float,
    halfHeight: _c.float,
}


PxHullPolygon :: struct {
    mPlane: [4]_c.float,
    mNbVerts: _c.uint16_t,
    mIndexBase: _c.uint16_t,
}


PxConvexMesh :: struct {
    using _: PxRefCounted,
}


PxMeshScale :: struct {
    scale: PxVec3,
    rotation: PxQuat,
}


PxConvexMeshGeometry :: struct {
    using _: PxGeometry,
    scale: PxMeshScale,
    _pad4: [4]u8,
    convexMesh: ^PxConvexMesh,
    meshFlags: PxConvexMeshGeometryFlags_Set,
    _pad7: [7]u8,
}


PxSphereGeometry :: struct {
    using _: PxGeometry,
    radius: _c.float,
}


PxPlaneGeometry :: struct {
    using _: PxGeometry,
}


PxTriangleMeshGeometry :: struct {
    using _: PxGeometry,
    scale: PxMeshScale,
    meshFlags: PxMeshGeometryFlags_Set,
    _pad5: [3]u8,
    triangleMesh: ^PxTriangleMesh,
}


PxHeightFieldGeometry :: struct {
    using _: PxGeometry,
    heightField: ^PxHeightField,
    heightScale: _c.float,
    rowScale: _c.float,
    columnScale: _c.float,
    heightFieldFlags: PxMeshGeometryFlags_Set,
    _pad8: [3]u8,
}


PxParticleSystemGeometry :: struct {
    using _: PxGeometry,
    mSolverType: PxParticleSolverType,
}


PxHairSystemGeometry :: struct {
    using _: PxGeometry,
}


PxTetrahedronMeshGeometry :: struct {
    using _: PxGeometry,
    tetrahedronMesh: ^PxTetrahedronMesh,
}


PxQueryHit :: struct {
    faceIndex: _c.uint32_t,
}


PxLocationHit :: struct {
    using _: PxQueryHit,
    flags: PxHitFlags_Set,
    _pad3: [2]u8,
    position: PxVec3,
    normal: PxVec3,
    distance: _c.float,
}


PxGeomRaycastHit :: struct {
    using _: PxLocationHit,
    u: _c.float,
    v: _c.float,
}


PxGeomOverlapHit :: struct {
    using _: PxQueryHit,
}


PxGeomSweepHit :: struct {
    using _: PxLocationHit,
}


PxGeomIndexPair :: struct {
    id0: _c.uint32_t,
    id1: _c.uint32_t,
}


PxQueryThreadContext :: struct {
};

PxContactBuffer :: distinct rawptr

PxRenderOutput :: distinct rawptr

PxCustomGeometryType :: struct {
    _pad0: [4]u8,
}


PxCustomGeometryCallbacks :: struct {
    _pad0: [8]u8,
}


PxCustomGeometry :: struct {
    using _: PxGeometry,
    callbacks: ^PxCustomGeometryCallbacks,
}


PxGeometryHolder :: struct {
    _pad0: [56]u8,
}


PxGeometryQuery :: struct {
};

PxHeightFieldSample :: struct {
    height: _c.int16_t,
    materialIndex0: PxBitAndByte,
    materialIndex1: PxBitAndByte,
}


PxHeightField :: struct {
    using _: PxRefCounted,
}


PxHeightFieldDesc :: struct {
    nbRows: _c.uint32_t,
    nbColumns: _c.uint32_t,
    format: PxHeightFieldFormat,
    _pad3: [4]u8,
    samples: PxStridedData,
    convexEdgeThreshold: _c.float,
    flags: PxHeightFieldFlags_Set,
    _pad7: [2]u8,
}


PxMeshQuery :: struct {
};

PxSimpleTriangleMesh :: struct {
    points: PxBoundedData,
    triangles: PxBoundedData,
    flags: PxMeshFlags_Set,
    _pad3: [6]u8,
}


PxTriangle :: struct {
    verts: [3]PxVec3,
}


PxTrianglePadded :: struct {
    using _: PxTriangle,
    padding: _c.uint32_t,
}


PxTriangleMesh :: struct {
    using _: PxRefCounted,
}


PxBVH34TriangleMesh :: struct {
    using _: PxTriangleMesh,
}


PxTetrahedron :: struct {
    verts: [4]PxVec3,
}


PxSoftBodyAuxData :: struct {
    using _: PxRefCounted,
}


PxTetrahedronMesh :: struct {
    using _: PxRefCounted,
}


PxSoftBodyMesh :: struct {
    using _: PxRefCounted,
}


PxCollisionMeshMappingData :: struct {
    using _: PxUserAllocated,
    _pad0: [8]u8,
}


PxSoftBodyCollisionData :: struct {
    using _: PxUserAllocated,
};

PxTetrahedronMeshData :: struct {
    using _: PxUserAllocated,
};

PxSoftBodySimulationData :: struct {
    using _: PxUserAllocated,
};

PxCollisionTetrahedronMeshData :: struct {
    using _: PxUserAllocated,
    _pad0: [8]u8,
}


PxSimulationTetrahedronMeshData :: struct {
    using _: PxUserAllocated,
    _pad0: [8]u8,
}


PxActor :: struct {
    using _: PxBase,
    userData: rawptr,
}


PxAggregate :: struct {
    using _: PxBase,
    userData: rawptr,
}


PxSpringModifiers :: struct #align(16){
    stiffness: _c.float,
    damping: _c.float,
    _pad2: [8]u8,
}


PxRestitutionModifiers :: struct #align(16){
    restitution: _c.float,
    velocityThreshold: _c.float,
    _pad2: [8]u8,
}


Px1DConstraintMods :: struct #raw_union {
    spring: PxSpringModifiers,
    bounce: PxRestitutionModifiers,
};

Px1DConstraint :: struct {
    linear0: PxVec3,
    geometricError: _c.float,
    angular0: PxVec3,
    velocityTarget: _c.float,
    linear1: PxVec3,
    minImpulse: _c.float,
    angular1: PxVec3,
    maxImpulse: _c.float,
    mods: Px1DConstraintMods,
    forInternalUse: _c.float,
    flags: _c.uint16_t,
    solveHint: _c.uint16_t,
    _pad12: [8]u8,
}


PxConstraintInvMassScale :: struct #align(16){
    linear0: _c.float,
    angular0: _c.float,
    linear1: _c.float,
    angular1: _c.float,
}


PxConstraintVisualizer :: struct {
    _pad0: [8]u8,
}


PxConstraintConnector :: struct {
    _pad0: [8]u8,
}


PxContactPoint :: struct #align(16){
    normal: PxVec3,
    separation: _c.float,
    point: PxVec3,
    maxImpulse: _c.float,
    targetVel: PxVec3,
    staticFriction: _c.float,
    materialFlags: _c.uint8_t,
    _pad7: [3]u8,
    internalFaceIndex1: _c.uint32_t,
    dynamicFriction: _c.float,
    restitution: _c.float,
    damping: _c.float,
    _pad12: [12]u8,
}


PxSolverBody :: struct {
    linearVelocity: PxVec3,
    maxSolverNormalProgress: _c.uint16_t,
    maxSolverFrictionProgress: _c.uint16_t,
    angularState: PxVec3,
    solverProgress: _c.uint32_t,
}


PxSolverBodyData :: struct {
    linearVelocity: PxVec3,
    invMass: _c.float,
    angularVelocity: PxVec3,
    reportThreshold: _c.float,
    sqrtInvInertia: PxMat33,
    penBiasClamp: _c.float,
    nodeIndex: _c.uint32_t,
    maxContactImpulse: _c.float,
    body2World: PxTransform,
    pad: _c.uint16_t,
    _pad10: [2]u8,
}


PxConstraintBatchHeader :: struct {
    startIndex: _c.uint32_t,
    stride: _c.uint16_t,
    constraintType: _c.uint16_t,
}


PxSolverConstraintDesc :: struct {
    bodyA: ^PxSolverBody,
    bodyB: ^PxSolverBody,
    bodyADataIndex: _c.uint32_t,
    bodyBDataIndex: _c.uint32_t,
    linkIndexA: _c.uint32_t,
    linkIndexB: _c.uint32_t,
    constraint: ^_c.uint8_t,
    writeBack: rawptr,
    progressA: _c.uint16_t,
    progressB: _c.uint16_t,
    constraintLengthOver16: _c.uint16_t,
    padding: [10]_c.uint8_t,
}


PxSolverConstraintPrepDescBase :: struct {
    invMassScales: PxConstraintInvMassScale,
    desc: ^PxSolverConstraintDesc,
    body0: ^PxSolverBody,
    body1: ^PxSolverBody,
    data0: ^PxSolverBodyData,
    data1: ^PxSolverBodyData,
    bodyFrame0: PxTransform,
    bodyFrame1: PxTransform,
    bodyState0: BodyState,
    bodyState1: BodyState,
    _pad10: [8]u8,
}


PxSolverConstraintPrepDesc :: struct {
    using _: PxSolverConstraintPrepDescBase,
    rows: ^Px1DConstraint,
    numRows: _c.uint32_t,
    linBreakForce: _c.float,
    angBreakForce: _c.float,
    minResponseThreshold: _c.float,
    writeback: rawptr,
    disablePreprocessing: _c.bool,
    improvedSlerp: _c.bool,
    driveLimitsAreForces: _c.bool,
    extendedLimits: _c.bool,
    disableConstraint: _c.bool,
    _pad23: [3]u8,
    body0WorldOffset: PxVec3Padded,
    _pad25: [8]u8,
}


PxSolverContactDesc :: struct {
    invMassScales: PxConstraintInvMassScale,
    desc: ^PxSolverConstraintDesc,
    body0: ^PxSolverBody,
    body1: ^PxSolverBody,
    data0: ^PxSolverBodyData,
    data1: ^PxSolverBodyData,
    bodyFrame0: PxTransform,
    bodyFrame1: PxTransform,
    bodyState0: BodyState,
    bodyState1: BodyState,
    shapeInteraction: rawptr,
    contacts: ^PxContactPoint,
    numContacts: _c.uint32_t,
    hasMaxImpulse: _c.bool,
    disableStrongFriction: _c.bool,
    hasForceThresholds: _c.bool,
    _pad16: [1]u8,
    restDistance: _c.float,
    maxCCDSeparation: _c.float,
    frictionPtr: ^_c.uint8_t,
    frictionCount: _c.uint8_t,
    _pad21: [7]u8,
    contactForces: ^_c.float,
    startFrictionPatchIndex: _c.uint32_t,
    numFrictionPatches: _c.uint32_t,
    startContactPatchIndex: _c.uint32_t,
    numContactPatches: _c.uint16_t,
    axisConstraintCount: _c.uint16_t,
    offsetSlop: _c.float,
    _pad29: [12]u8,
}


PxConstraintAllocator :: struct {
    _pad0: [8]u8,
}


PxArticulationLimit :: struct {
    low: _c.float,
    high: _c.float,
}


PxArticulationDrive :: struct {
    stiffness: _c.float,
    damping: _c.float,
    maxForce: _c.float,
    driveType: PxArticulationDriveType,
}


PxTGSSolverBodyVel :: struct {
    linearVelocity: PxVec3,
    nbStaticInteractions: _c.uint16_t,
    maxDynamicPartition: _c.uint16_t,
    angularVelocity: PxVec3,
    partitionMask: _c.uint32_t,
    deltaAngDt: PxVec3,
    maxAngVel: _c.float,
    deltaLinDt: PxVec3,
    lockFlags: _c.uint16_t,
    isKinematic: _c.bool,
    pad: _c.uint8_t,
}


PxTGSSolverBodyTxInertia :: struct {
    deltaBody2World: PxTransform,
    sqrtInvInertia: PxMat33,
}


PxTGSSolverBodyData :: struct {
    originalLinearVelocity: PxVec3,
    maxContactImpulse: _c.float,
    originalAngularVelocity: PxVec3,
    penBiasClamp: _c.float,
    invMass: _c.float,
    nodeIndex: _c.uint32_t,
    reportThreshold: _c.float,
    pad: _c.uint32_t,
}


PxTGSSolverConstraintPrepDescBase :: struct {
    invMassScales: PxConstraintInvMassScale,
    desc: ^PxSolverConstraintDesc,
    body0: ^PxTGSSolverBodyVel,
    body1: ^PxTGSSolverBodyVel,
    body0TxI: ^PxTGSSolverBodyTxInertia,
    body1TxI: ^PxTGSSolverBodyTxInertia,
    bodyData0: ^PxTGSSolverBodyData,
    bodyData1: ^PxTGSSolverBodyData,
    bodyFrame0: PxTransform,
    bodyFrame1: PxTransform,
    bodyState0: BodyState,
    bodyState1: BodyState,
    _pad12: [8]u8,
}


PxTGSSolverConstraintPrepDesc :: struct {
    invMassScales: PxConstraintInvMassScale,
    desc: ^PxSolverConstraintDesc,
    body0: ^PxTGSSolverBodyVel,
    body1: ^PxTGSSolverBodyVel,
    body0TxI: ^PxTGSSolverBodyTxInertia,
    body1TxI: ^PxTGSSolverBodyTxInertia,
    bodyData0: ^PxTGSSolverBodyData,
    bodyData1: ^PxTGSSolverBodyData,
    bodyFrame0: PxTransform,
    bodyFrame1: PxTransform,
    bodyState0: BodyState,
    bodyState1: BodyState,
    rows: ^Px1DConstraint,
    numRows: _c.uint32_t,
    linBreakForce: _c.float,
    angBreakForce: _c.float,
    minResponseThreshold: _c.float,
    writeback: rawptr,
    disablePreprocessing: _c.bool,
    improvedSlerp: _c.bool,
    driveLimitsAreForces: _c.bool,
    extendedLimits: _c.bool,
    disableConstraint: _c.bool,
    _pad23: [3]u8,
    body0WorldOffset: PxVec3Padded,
    cA2w: PxVec3Padded,
    cB2w: PxVec3Padded,
}


PxTGSSolverContactDesc :: struct {
    invMassScales: PxConstraintInvMassScale,
    desc: ^PxSolverConstraintDesc,
    body0: ^PxTGSSolverBodyVel,
    body1: ^PxTGSSolverBodyVel,
    body0TxI: ^PxTGSSolverBodyTxInertia,
    body1TxI: ^PxTGSSolverBodyTxInertia,
    bodyData0: ^PxTGSSolverBodyData,
    bodyData1: ^PxTGSSolverBodyData,
    bodyFrame0: PxTransform,
    bodyFrame1: PxTransform,
    bodyState0: BodyState,
    bodyState1: BodyState,
    shapeInteraction: rawptr,
    contacts: ^PxContactPoint,
    numContacts: _c.uint32_t,
    hasMaxImpulse: _c.bool,
    disableStrongFriction: _c.bool,
    hasForceThresholds: _c.bool,
    _pad18: [1]u8,
    restDistance: _c.float,
    maxCCDSeparation: _c.float,
    frictionPtr: ^_c.uint8_t,
    frictionCount: _c.uint8_t,
    _pad23: [7]u8,
    contactForces: ^_c.float,
    startFrictionPatchIndex: _c.uint32_t,
    numFrictionPatches: _c.uint32_t,
    startContactPatchIndex: _c.uint32_t,
    numContactPatches: _c.uint16_t,
    axisConstraintCount: _c.uint16_t,
    maxImpulse: _c.float,
    torsionalPatchRadius: _c.float,
    minTorsionalPatchRadius: _c.float,
    offsetSlop: _c.float,
}


PxArticulationTendonLimit :: struct {
    lowLimit: _c.float,
    highLimit: _c.float,
}


PxArticulationAttachment :: struct {
    using _: PxBase,
    userData: rawptr,
}


PxArticulationTendonJoint :: struct {
    using _: PxBase,
    userData: rawptr,
}


PxArticulationTendon :: struct {
    using _: PxBase,
    userData: rawptr,
}


PxArticulationSpatialTendon :: struct {
    using _: PxArticulationTendon,
}


PxArticulationFixedTendon :: struct {
    using _: PxArticulationTendon,
}


PxSpatialForce :: struct {
    force: PxVec3,
    pad0: _c.float,
    torque: PxVec3,
    pad1: _c.float,
}


PxSpatialVelocity :: struct {
    linear: PxVec3,
    pad0: _c.float,
    angular: PxVec3,
    pad1: _c.float,
}


PxArticulationRootLinkData :: struct {
    transform: PxTransform,
    worldLinVel: PxVec3,
    worldAngVel: PxVec3,
    worldLinAccel: PxVec3,
    worldAngAccel: PxVec3,
}


PxArticulationCache :: struct {
    externalForces: ^PxSpatialForce,
    denseJacobian: ^_c.float,
    massMatrix: ^_c.float,
    jointVelocity: ^_c.float,
    jointAcceleration: ^_c.float,
    jointPosition: ^_c.float,
    jointForce: ^_c.float,
    jointSolverForces: ^_c.float,
    linkVelocity: ^PxSpatialVelocity,
    linkAcceleration: ^PxSpatialVelocity,
    rootLinkData: ^PxArticulationRootLinkData,
    sensorForces: ^PxSpatialForce,
    coefficientMatrix: ^_c.float,
    lambda: ^_c.float,
    scratchMemory: rawptr,
    scratchAllocator: rawptr,
    version: _c.uint32_t,
    _pad17: [4]u8,
}


PxArticulationSensor :: struct {
    using _: PxBase,
    userData: rawptr,
}


PxArticulationReducedCoordinate :: struct {
    using _: PxBase,
    userData: rawptr,
}


PxArticulationJointReducedCoordinate :: struct {
    using _: PxBase,
    userData: rawptr,
}


PxShape :: struct {
    using _: PxRefCounted,
    userData: rawptr,
}


PxRigidActor :: struct {
    using _: PxActor,
}


PxNodeIndex :: struct {
    _pad0: [8]u8,
}


PxRigidBody :: struct {
    using _: PxRigidActor,
}


PxArticulationLink :: struct {
    using _: PxRigidBody,
}


PxConeLimitedConstraint :: struct {
    mAxis: PxVec3,
    mAngle: _c.float,
    mLowLimit: _c.float,
    mHighLimit: _c.float,
}


PxConeLimitParams :: struct {
    lowHighLimits: PxVec4,
    axisAngle: PxVec4,
}


PxConstraintShaderTable :: struct {
    solverPrep: rawptr,
    _pad1: [8]u8,
    visualize: rawptr,
    flag: PxConstraintFlag,
    _pad4: [4]u8,
}


PxConstraint :: struct {
    using _: PxBase,
    userData: rawptr,
}


PxMassModificationProps :: struct {
    mInvMassScale0: _c.float,
    mInvInertiaScale0: _c.float,
    mInvMassScale1: _c.float,
    mInvInertiaScale1: _c.float,
}


PxContactPatch :: struct {
    mMassModification: PxMassModificationProps,
    normal: PxVec3,
    restitution: _c.float,
    dynamicFriction: _c.float,
    staticFriction: _c.float,
    damping: _c.float,
    startContactIndex: _c.uint16_t,
    nbContacts: _c.uint8_t,
    materialFlags: _c.uint8_t,
    internalFlags: _c.uint16_t,
    materialIndex0: _c.uint16_t,
    materialIndex1: _c.uint16_t,
    pad: [5]_c.uint16_t,
}


PxContact :: struct {
    contact: PxVec3,
    separation: _c.float,
}


PxExtendedContact :: struct {
    using _: PxContact,
    targetVelocity: PxVec3,
    maxImpulse: _c.float,
}


PxModifiableContact :: struct {
    using _: PxExtendedContact,
    normal: PxVec3,
    restitution: _c.float,
    materialFlags: _c.uint32_t,
    materialIndex0: _c.uint16_t,
    materialIndex1: _c.uint16_t,
    staticFriction: _c.float,
    dynamicFriction: _c.float,
}


PxContactStreamIterator :: struct {
    zero: PxVec3,
    _pad1: [4]u8,
    patch: ^PxContactPatch,
    contact: ^PxContact,
    faceIndice: ^_c.uint32_t,
    totalPatches: _c.uint32_t,
    totalContacts: _c.uint32_t,
    nextContactIndex: _c.uint32_t,
    nextPatchIndex: _c.uint32_t,
    contactPatchHeaderSize: _c.uint32_t,
    contactPointSize: _c.uint32_t,
    mStreamFormat: StreamFormat,
    forceNoResponse: _c.uint32_t,
    pointStepped: _c.bool,
    _pad14: [3]u8,
    hasFaceIndices: _c.uint32_t,
}


PxGpuContactPair :: struct {
    contactPatches: ^_c.uint8_t,
    contactPoints: ^_c.uint8_t,
    contactForces: ^_c.float,
    transformCacheRef0: _c.uint32_t,
    transformCacheRef1: _c.uint32_t,
    nodeIndex0: PxNodeIndex,
    nodeIndex1: PxNodeIndex,
    actor0: ^PxActor,
    actor1: ^PxActor,
    nbContacts: _c.uint16_t,
    nbPatches: _c.uint16_t,
    _pad11: [4]u8,
}


PxContactSet :: struct {
    _pad0: [16]u8,
}


PxContactModifyPair :: struct {
    actor: [2]^PxRigidActor,
    shape: [2]^PxShape,
    transform: [2]PxTransform,
    contacts: PxContactSet,
}


PxContactModifyCallback :: struct {
    _pad0: [8]u8,
}


PxCCDContactModifyCallback :: struct {
    _pad0: [8]u8,
}


PxDeletionListener :: struct {
    _pad0: [8]u8,
}


PxBaseMaterial :: struct {
    using _: PxRefCounted,
    userData: rawptr,
}


PxFEMMaterial :: struct {
    using _: PxBaseMaterial,
}


PxFilterData :: struct {
    word0: _c.uint32_t,
    word1: _c.uint32_t,
    word2: _c.uint32_t,
    word3: _c.uint32_t,
}


PxSimulationFilterCallback :: struct {
    _pad0: [8]u8,
}


PxParticleRigidFilterPair :: struct {
    mID0: _c.uint64_t,
    mID1: _c.uint64_t,
}


PxLockedData :: struct {
    _pad0: [8]u8,
}


PxMaterial :: struct {
    using _: PxBaseMaterial,
}


PxGpuParticleBufferIndexPair :: struct {
    systemIndex: _c.uint32_t,
    bufferIndex: _c.uint32_t,
}


PxCudaContextManager :: distinct rawptr

PxParticleRigidAttachment :: distinct rawptr

PxParticleVolume :: struct {
    bound: PxBounds3,
    particleIndicesOffset: _c.uint32_t,
    numParticles: _c.uint32_t,
}


PxDiffuseParticleParams :: struct {
    threshold: _c.float,
    lifetime: _c.float,
    airDrag: _c.float,
    bubbleDrag: _c.float,
    buoyancy: _c.float,
    kineticEnergyWeight: _c.float,
    pressureWeight: _c.float,
    divergenceWeight: _c.float,
    collisionDecay: _c.float,
    useAccurateVelocity: _c.bool,
    _pad10: [3]u8,
}


PxParticleSpring :: struct {
    ind0: _c.uint32_t,
    ind1: _c.uint32_t,
    length: _c.float,
    stiffness: _c.float,
    damping: _c.float,
    pad: _c.float,
}


PxParticleMaterial :: struct {
    using _: PxBaseMaterial,
}


PxOmniPvd :: distinct rawptr

PxPhysics :: struct {
    _pad0: [8]u8,
}


PxActorShape :: struct {
    actor: ^PxRigidActor,
    shape: ^PxShape,
}


PxRaycastHit :: struct #packed {
    using _: PxGeomRaycastHit,
    using _: PxActorShape,
    _pad12: [4]u8,
}


PxOverlapHit :: struct #packed {
    using _: PxGeomOverlapHit,
    using _: PxActorShape,
    _pad4: [4]u8,
}


PxSweepHit :: struct #packed {
    using _: PxGeomSweepHit,
    using _: PxActorShape,
    _pad10: [4]u8,
}


PxRaycastCallback :: struct {
    _pad0: [8]u8,
    block: PxRaycastHit,
    hasBlock: _c.bool,
    _pad3: [7]u8,
    touches: ^PxRaycastHit,
    maxNbTouches: _c.uint32_t,
    nbTouches: _c.uint32_t,
}


PxOverlapCallback :: struct {
    _pad0: [8]u8,
    block: PxOverlapHit,
    hasBlock: _c.bool,
    _pad3: [7]u8,
    touches: ^PxOverlapHit,
    maxNbTouches: _c.uint32_t,
    nbTouches: _c.uint32_t,
}


PxSweepCallback :: struct {
    _pad0: [8]u8,
    block: PxSweepHit,
    hasBlock: _c.bool,
    _pad3: [7]u8,
    touches: ^PxSweepHit,
    maxNbTouches: _c.uint32_t,
    nbTouches: _c.uint32_t,
}


PxRaycastBuffer :: struct {
    _pad0: [8]u8,
    block: PxRaycastHit,
    hasBlock: _c.bool,
    _pad3: [7]u8,
    touches: ^PxRaycastHit,
    maxNbTouches: _c.uint32_t,
    nbTouches: _c.uint32_t,
}


PxOverlapBuffer :: struct {
    _pad0: [8]u8,
    block: PxOverlapHit,
    hasBlock: _c.bool,
    _pad3: [7]u8,
    touches: ^PxOverlapHit,
    maxNbTouches: _c.uint32_t,
    nbTouches: _c.uint32_t,
}


PxSweepBuffer :: struct {
    _pad0: [8]u8,
    block: PxSweepHit,
    hasBlock: _c.bool,
    _pad3: [7]u8,
    touches: ^PxSweepHit,
    maxNbTouches: _c.uint32_t,
    nbTouches: _c.uint32_t,
}


PxQueryCache :: struct {
    shape: ^PxShape,
    actor: ^PxRigidActor,
    faceIndex: _c.uint32_t,
    _pad3: [4]u8,
}


PxQueryFilterData :: struct {
    data: PxFilterData,
    flags: PxQueryFlags_Set,
    _pad2: [2]u8,
}


PxQueryFilterCallback :: struct {
    _pad0: [8]u8,
}


PxRigidDynamic :: struct {
    using _: PxRigidBody,
}


PxRigidStatic :: struct {
    using _: PxRigidActor,
}


PxSceneQueryDesc :: struct {
    staticStructure: PxPruningStructureType,
    dynamicStructure: PxPruningStructureType,
    dynamicTreeRebuildRateHint: _c.uint32_t,
    dynamicTreeSecondaryPruner: PxDynamicTreeSecondaryPruner,
    staticBVHBuildStrategy: PxBVHBuildStrategy,
    dynamicBVHBuildStrategy: PxBVHBuildStrategy,
    staticNbObjectsPerNode: _c.uint32_t,
    dynamicNbObjectsPerNode: _c.uint32_t,
    sceneQueryUpdateMode: PxSceneQueryUpdateMode,
}


PxSceneQuerySystemBase :: struct {
    _pad0: [8]u8,
}


PxSceneSQSystem :: struct {
    using _: PxSceneQuerySystemBase,
}


PxSceneQuerySystem :: struct {
    using _: PxSceneQuerySystemBase,
}


PxBroadPhaseRegion :: struct {
    mBounds: PxBounds3,
    mUserData: rawptr,
}


PxBroadPhaseRegionInfo :: struct {
    mRegion: PxBroadPhaseRegion,
    mNbStaticObjects: _c.uint32_t,
    mNbDynamicObjects: _c.uint32_t,
    mActive: _c.bool,
    mOverlap: _c.bool,
    _pad5: [6]u8,
}


PxBroadPhaseCaps :: struct {
    mMaxNbRegions: _c.uint32_t,
}


PxBroadPhaseDesc :: struct {
    mType: PxBroadPhaseType,
    _pad1: [4]u8,
    mContextID: _c.uint64_t,
    _pad3: [8]u8,
    mFoundLostPairsCapacity: _c.uint32_t,
    mDiscardStaticVsKinematic: _c.bool,
    mDiscardKinematicVsKinematic: _c.bool,
    _pad7: [2]u8,
}


PxBroadPhaseUpdateData :: struct {
    mCreated: ^_c.uint32_t,
    mNbCreated: _c.uint32_t,
    _pad2: [4]u8,
    mUpdated: ^_c.uint32_t,
    mNbUpdated: _c.uint32_t,
    _pad5: [4]u8,
    mRemoved: ^_c.uint32_t,
    mNbRemoved: _c.uint32_t,
    _pad8: [4]u8,
    mBounds: ^PxBounds3,
    mGroups: ^_c.uint32_t,
    mDistances: ^_c.float,
    mCapacity: _c.uint32_t,
    _pad13: [4]u8,
}


PxBroadPhasePair :: struct {
    mID0: _c.uint32_t,
    mID1: _c.uint32_t,
}


PxBroadPhaseResults :: struct {
    mNbCreatedPairs: _c.uint32_t,
    _pad1: [4]u8,
    mCreatedPairs: ^PxBroadPhasePair,
    mNbDeletedPairs: _c.uint32_t,
    _pad4: [4]u8,
    mDeletedPairs: ^PxBroadPhasePair,
}


PxBroadPhaseRegions :: struct {
    _pad0: [8]u8,
}


PxBroadPhase :: struct {
    _pad0: [8]u8,
}


PxAABBManager :: struct {
    _pad0: [8]u8,
}


PxSceneLimits :: struct {
    maxNbActors: _c.uint32_t,
    maxNbBodies: _c.uint32_t,
    maxNbStaticShapes: _c.uint32_t,
    maxNbDynamicShapes: _c.uint32_t,
    maxNbAggregates: _c.uint32_t,
    maxNbConstraints: _c.uint32_t,
    maxNbRegions: _c.uint32_t,
    maxNbBroadPhaseOverlaps: _c.uint32_t,
}


PxgDynamicsMemoryConfig :: struct {
    tempBufferCapacity: _c.uint32_t,
    maxRigidContactCount: _c.uint32_t,
    maxRigidPatchCount: _c.uint32_t,
    heapCapacity: _c.uint32_t,
    foundLostPairsCapacity: _c.uint32_t,
    foundLostAggregatePairsCapacity: _c.uint32_t,
    totalAggregatePairsCapacity: _c.uint32_t,
    maxSoftBodyContacts: _c.uint32_t,
    maxFemClothContacts: _c.uint32_t,
    maxParticleContacts: _c.uint32_t,
    collisionStackSize: _c.uint32_t,
    maxHairContacts: _c.uint32_t,
}


PxSceneDesc :: struct {
    using _: PxSceneQueryDesc,
    gravity: PxVec3,
    simulationEventCallback: ^PxSimulationEventCallback,
    contactModifyCallback: ^PxContactModifyCallback,
    ccdContactModifyCallback: ^PxCCDContactModifyCallback,
    filterShaderData: rawptr,
    filterShaderDataSize: _c.uint32_t,
    _pad16: [4]u8,
    filterShader: rawptr,
    filterCallback: ^PxSimulationFilterCallback,
    kineKineFilteringMode: PxPairFilteringMode,
    staticKineFilteringMode: PxPairFilteringMode,
    broadPhaseType: PxBroadPhaseType,
    _pad22: [4]u8,
    broadPhaseCallback: ^PxBroadPhaseCallback,
    limits: PxSceneLimits,
    frictionType: PxFrictionType,
    solverType: PxSolverType,
    bounceThresholdVelocity: _c.float,
    frictionOffsetThreshold: _c.float,
    frictionCorrelationDistance: _c.float,
    flags: PxSceneFlags_Set,
    cpuDispatcher: ^PxCpuDispatcher,
    _pad32: [8]u8,
    userData: rawptr,
    solverBatchSize: _c.uint32_t,
    solverArticulationBatchSize: _c.uint32_t,
    nbContactDataBlocks: _c.uint32_t,
    maxNbContactDataBlocks: _c.uint32_t,
    maxBiasCoefficient: _c.float,
    contactReportStreamBufferSize: _c.uint32_t,
    ccdMaxPasses: _c.uint32_t,
    ccdThreshold: _c.float,
    ccdMaxSeparation: _c.float,
    wakeCounterResetValue: _c.float,
    sanityBounds: PxBounds3,
    gpuDynamicsConfig: PxgDynamicsMemoryConfig,
    gpuMaxNumPartitions: _c.uint32_t,
    gpuMaxNumStaticPartitions: _c.uint32_t,
    gpuComputeVersion: _c.uint32_t,
    contactPairSlabSize: _c.uint32_t,
    sceneQuerySystem: ^PxSceneQuerySystem,
    _pad51: [8]u8,
}


PxSimulationStatistics :: struct {
    nbActiveConstraints: _c.uint32_t,
    nbActiveDynamicBodies: _c.uint32_t,
    nbActiveKinematicBodies: _c.uint32_t,
    nbStaticBodies: _c.uint32_t,
    nbDynamicBodies: _c.uint32_t,
    nbKinematicBodies: _c.uint32_t,
    nbShapes: [11]_c.uint32_t,
    nbAggregates: _c.uint32_t,
    nbArticulations: _c.uint32_t,
    nbAxisSolverConstraints: _c.uint32_t,
    compressedContactSize: _c.uint32_t,
    requiredContactConstraintMemory: _c.uint32_t,
    peakConstraintMemory: _c.uint32_t,
    nbDiscreteContactPairsTotal: _c.uint32_t,
    nbDiscreteContactPairsWithCacheHits: _c.uint32_t,
    nbDiscreteContactPairsWithContacts: _c.uint32_t,
    nbNewPairs: _c.uint32_t,
    nbLostPairs: _c.uint32_t,
    nbNewTouches: _c.uint32_t,
    nbLostTouches: _c.uint32_t,
    nbPartitions: _c.uint32_t,
    _pad21: [4]u8,
    gpuMemParticles: _c.uint64_t,
    gpuMemSoftBodies: _c.uint64_t,
    gpuMemFEMCloths: _c.uint64_t,
    gpuMemHairSystems: _c.uint64_t,
    gpuMemHeap: _c.uint64_t,
    gpuMemHeapBroadPhase: _c.uint64_t,
    gpuMemHeapNarrowPhase: _c.uint64_t,
    gpuMemHeapSolver: _c.uint64_t,
    gpuMemHeapArticulation: _c.uint64_t,
    gpuMemHeapSimulation: _c.uint64_t,
    gpuMemHeapSimulationArticulation: _c.uint64_t,
    gpuMemHeapSimulationParticles: _c.uint64_t,
    gpuMemHeapSimulationSoftBody: _c.uint64_t,
    gpuMemHeapSimulationFEMCloth: _c.uint64_t,
    gpuMemHeapSimulationHairSystem: _c.uint64_t,
    gpuMemHeapParticles: _c.uint64_t,
    gpuMemHeapSoftBodies: _c.uint64_t,
    gpuMemHeapFEMCloths: _c.uint64_t,
    gpuMemHeapHairSystems: _c.uint64_t,
    gpuMemHeapOther: _c.uint64_t,
    nbBroadPhaseAdds: _c.uint32_t,
    nbBroadPhaseRemoves: _c.uint32_t,
    nbDiscreteContactPairs: [11][11]_c.uint32_t,
    nbCCDPairs: [11][11]_c.uint32_t,
    nbModifiedContactPairs: [11][11]_c.uint32_t,
    nbTriggerPairs: [11][11]_c.uint32_t,
}


PxGpuBodyData :: struct {
    quat: PxQuat,
    pos: PxVec4,
    linVel: PxVec4,
    angVel: PxVec4,
}


PxGpuActorPair :: struct {
    srcIndex: _c.uint32_t,
    _pad1: [4]u8,
    nodeIndex: PxNodeIndex,
}


PxIndexDataPair :: struct {
    index: _c.uint32_t,
    _pad1: [4]u8,
    data: rawptr,
}


PxPvdSceneClient :: struct {
    _pad0: [8]u8,
}


PxDominanceGroupPair :: struct {
    dominance0: _c.uint8_t,
    dominance1: _c.uint8_t,
}


PxBroadPhaseCallback :: struct {
    _pad0: [8]u8,
}


PxScene :: struct {
    using _: PxSceneSQSystem,
    userData: rawptr,
}


PxSceneReadLock :: struct {
    _pad0: [8]u8,
}


PxSceneWriteLock :: struct {
    _pad0: [8]u8,
}


PxContactPairExtraDataItem :: struct {
    type: _c.uint8_t,
}


PxContactPairVelocity :: struct {
    using _: PxContactPairExtraDataItem,
    _pad1: [3]u8,
    linearVelocity: [2]PxVec3,
    angularVelocity: [2]PxVec3,
}


PxContactPairPose :: struct {
    using _: PxContactPairExtraDataItem,
    _pad1: [3]u8,
    globalPose: [2]PxTransform,
}


PxContactPairIndex :: struct {
    using _: PxContactPairExtraDataItem,
    _pad1: [1]u8,
    index: _c.uint16_t,
}


PxContactPairExtraDataIterator :: struct {
    currPtr: ^_c.uint8_t,
    endPtr: ^_c.uint8_t,
    preSolverVelocity: ^PxContactPairVelocity,
    postSolverVelocity: ^PxContactPairVelocity,
    eventPose: ^PxContactPairPose,
    contactPairIndex: _c.uint32_t,
    _pad6: [4]u8,
}


PxContactPairHeader :: struct {
    actors: [2]^PxActor,
    extraDataStream: ^_c.uint8_t,
    extraDataStreamSize: _c.uint16_t,
    flags: PxContactPairHeaderFlags_Set,
    _pad4: [4]u8,
    pairs: ^PxContactPair,
    nbPairs: _c.uint32_t,
    _pad7: [4]u8,
}


PxContactPairPoint :: struct {
    position: PxVec3,
    separation: _c.float,
    normal: PxVec3,
    internalFaceIndex0: _c.uint32_t,
    impulse: PxVec3,
    internalFaceIndex1: _c.uint32_t,
}


PxContactPair :: struct {
    shapes: [2]^PxShape,
    contactPatches: ^_c.uint8_t,
    contactPoints: ^_c.uint8_t,
    contactImpulses: ^_c.float,
    requiredBufferSize: _c.uint32_t,
    contactCount: _c.uint8_t,
    patchCount: _c.uint8_t,
    contactStreamSize: _c.uint16_t,
    flags: PxContactPairFlags_Set,
    events: PxPairFlags_Set,
    internalData: [2]_c.uint32_t,
    _pad11: [4]u8,
}


PxTriggerPair :: struct {
    triggerShape: ^PxShape,
    triggerActor: ^PxActor,
    otherShape: ^PxShape,
    otherActor: ^PxActor,
    status: PxPairFlag,
    flags: PxTriggerPairFlags_Set,
    _pad6: [3]u8,
}


PxConstraintInfo :: struct {
    constraint: ^PxConstraint,
    externalReference: rawptr,
    type: _c.uint32_t,
    _pad3: [4]u8,
}


PxSimulationEventCallback :: struct {
    _pad0: [8]u8,
}


PxFEMParameters :: struct {
    velocityDamping: _c.float,
    settlingThreshold: _c.float,
    sleepThreshold: _c.float,
    sleepDamping: _c.float,
    selfCollisionFilterDistance: _c.float,
    selfCollisionStressTolerance: _c.float,
}


PxPruningStructure :: struct {
    using _: PxBase,
}


PxExtendedVec3 :: struct {
    x: _c.double,
    y: _c.double,
    z: _c.double,
}


PxObstacle :: struct {
    _pad0: [8]u8,
    mUserData: rawptr,
    mPos: PxExtendedVec3,
    mRot: PxQuat,
}


PxBoxObstacle :: struct {
    using _: PxObstacle,
    mHalfExtents: PxVec3,
    _pad6: [4]u8,
}


PxCapsuleObstacle :: struct {
    using _: PxObstacle,
    mHalfHeight: _c.float,
    mRadius: _c.float,
}


PxObstacleContext :: struct {
    _pad0: [8]u8,
}


PxControllerState :: struct {
    deltaXP: PxVec3,
    _pad1: [4]u8,
    touchedShape: ^PxShape,
    touchedActor: ^PxRigidActor,
    touchedObstacleHandle: _c.uint32_t,
    collisionFlags: _c.uint32_t,
    standOnAnotherCCT: _c.bool,
    standOnObstacle: _c.bool,
    isMovingUp: _c.bool,
    _pad9: [5]u8,
}


PxControllerStats :: struct {
    nbIterations: _c.uint16_t,
    nbFullUpdates: _c.uint16_t,
    nbPartialUpdates: _c.uint16_t,
    nbTessellation: _c.uint16_t,
}


PxControllerHit :: struct {
    controller: ^PxController,
    worldPos: PxExtendedVec3,
    worldNormal: PxVec3,
    dir: PxVec3,
    length: _c.float,
    _pad5: [4]u8,
}


PxControllerShapeHit :: struct {
    using _: PxControllerHit,
    shape: ^PxShape,
    actor: ^PxRigidActor,
    triangleIndex: _c.uint32_t,
    _pad10: [4]u8,
}


PxControllersHit :: struct {
    using _: PxControllerHit,
    other: ^PxController,
}


PxControllerObstacleHit :: struct {
    using _: PxControllerHit,
    userData: rawptr,
}


PxUserControllerHitReport :: struct {
    _pad0: [8]u8,
}


PxControllerFilterCallback :: struct {
    _pad0: [8]u8,
}


PxControllerFilters :: struct {
    mFilterData: ^PxFilterData,
    mFilterCallback: ^PxQueryFilterCallback,
    mFilterFlags: PxQueryFlags_Set,
    _pad3: [6]u8,
    mCCTFilterCallback: ^PxControllerFilterCallback,
}


PxControllerDesc :: struct #packed {
    _pad0: [8]u8,
    position: PxExtendedVec3,
    upDirection: PxVec3,
    slopeLimit: _c.float,
    invisibleWallHeight: _c.float,
    maxJumpHeight: _c.float,
    contactOffset: _c.float,
    stepOffset: _c.float,
    density: _c.float,
    scaleCoeff: _c.float,
    volumeGrowth: _c.float,
    _pad11: [4]u8,
    reportCallback: ^PxUserControllerHitReport,
    behaviorCallback: ^PxControllerBehaviorCallback,
    nonWalkableMode: PxControllerNonWalkableMode,
    _pad15: [4]u8,
    material: ^PxMaterial,
    registerDeletionListener: _c.bool,
    clientID: _c.uint8_t,
    _pad19: [6]u8,
    userData: rawptr,
    _pad21: [4]u8,
}


PxController :: struct {
    _pad0: [8]u8,
}


PxBoxControllerDesc :: struct {
    using _: PxControllerDesc,
    halfHeight: _c.float,
    halfSideExtent: _c.float,
    halfForwardExtent: _c.float,
}


PxBoxController :: struct {
    using _: PxController,
}


PxCapsuleControllerDesc :: struct {
    using _: PxControllerDesc,
    radius: _c.float,
    height: _c.float,
    climbingMode: PxCapsuleClimbingMode,
}


PxCapsuleController :: struct {
    using _: PxController,
}


PxControllerBehaviorCallback :: struct {
    _pad0: [8]u8,
}


PxControllerManager :: struct {
    _pad0: [8]u8,
}


PxDim3 :: struct {
    x: _c.uint32_t,
    y: _c.uint32_t,
    z: _c.uint32_t,
}


PxSDFDesc :: struct {
    sdf: PxBoundedData,
    dims: PxDim3,
    meshLower: PxVec3,
    spacing: _c.float,
    subgridSize: _c.uint32_t,
    bitsPerSubgridPixel: PxSdfBitsPerSubgridPixel,
    sdfSubgrids3DTexBlockDim: PxDim3,
    sdfSubgrids: PxBoundedData,
    sdfStartSlots: PxBoundedData,
    subgridsMinSdfValue: _c.float,
    subgridsMaxSdfValue: _c.float,
    sdfBounds: PxBounds3,
    narrowBandThicknessRelativeToSdfBoundsDiagonal: _c.float,
    numThreadsForSdfConstruction: _c.uint32_t,
}


PxConvexMeshDesc :: struct {
    points: PxBoundedData,
    polygons: PxBoundedData,
    indices: PxBoundedData,
    flags: PxConvexFlags_Set,
    vertexLimit: _c.uint16_t,
    polygonLimit: _c.uint16_t,
    quantizedCount: _c.uint16_t,
    sdfDesc: ^PxSDFDesc,
}


PxTriangleMeshDesc :: struct {
    using _: PxSimpleTriangleMesh,
    materialIndices: [16]_c.char,
    sdfDesc: ^PxSDFDesc,
}


PxTetrahedronMeshDesc :: struct {
    materialIndices: [16]_c.char,
    points: PxBoundedData,
    tetrahedrons: PxBoundedData,
    flags: PxMeshFlags_Set,
    tetsPerElement: _c.uint16_t,
    _pad5: [4]u8,
}


PxSoftBodySimulationDataDesc :: struct {
    vertexToTet: PxBoundedData,
}


PxBVH34MidphaseDesc :: struct {
    numPrimsPerLeaf: _c.uint32_t,
    buildStrategy: PxBVH34BuildStrategy,
    quantized: _c.bool,
    _pad3: [3]u8,
}


PxMidphaseDesc :: struct {
    mBVH34Desc: [12]_c.char,
    _pad1: [4]u8,
}


PxBVHDesc :: struct {
    bounds: PxBoundedData,
    enlargement: _c.float,
    numPrimsPerLeaf: _c.uint32_t,
    buildStrategy: PxBVHBuildStrategy,
    _pad4: [4]u8,
}


PxCookingParams :: struct {
    areaTestEpsilon: _c.float,
    planeTolerance: _c.float,
    convexMeshCookingType: PxConvexMeshCookingType,
    suppressTriangleMeshRemapTable: _c.bool,
    buildTriangleAdjacencies: _c.bool,
    buildGPUData: _c.bool,
    _pad6: [1]u8,
    scale: PxTolerancesScale,
    meshPreprocessParams: PxMeshPreprocessingFlags_Set,
    meshWeldTolerance: _c.float,
    midphaseDesc: PxMidphaseDesc,
    gaussMapLimit: _c.uint32_t,
    maxWeightRatioInTet: _c.float,
}


PxDefaultMemoryOutputStream :: struct {
    using _: PxOutputStream,
    _pad1: [24]u8,
}


PxDefaultMemoryInputData :: struct {
    using _: PxInputData,
    _pad1: [24]u8,
}


PxDefaultFileOutputStream :: struct {
    using _: PxOutputStream,
    _pad1: [8]u8,
}


PxDefaultFileInputData :: struct {
    using _: PxInputData,
    _pad1: [16]u8,
}


PxDefaultAllocator :: struct {
    using _: PxAllocatorCallback,
}


PxJoint :: struct {
    using _: PxBase,
    userData: rawptr,
}


PxSpring :: struct {
    stiffness: _c.float,
    damping: _c.float,
}


PxDistanceJoint :: struct {
    using _: PxJoint,
}


PxJacobianRow :: struct {
    linear0: PxVec3,
    linear1: PxVec3,
    angular0: PxVec3,
    angular1: PxVec3,
}


PxContactJoint :: struct {
    using _: PxJoint,
}


PxFixedJoint :: struct {
    using _: PxJoint,
}


PxJointLimitParameters :: struct {
    restitution: _c.float,
    bounceThreshold: _c.float,
    stiffness: _c.float,
    damping: _c.float,
    contactDistance_deprecated: _c.float,
}


PxJointLinearLimit :: struct {
    using _: PxJointLimitParameters,
    value: _c.float,
}


PxJointLinearLimitPair :: struct {
    using _: PxJointLimitParameters,
    upper: _c.float,
    lower: _c.float,
}


PxJointAngularLimitPair :: struct {
    using _: PxJointLimitParameters,
    upper: _c.float,
    lower: _c.float,
}


PxJointLimitCone :: struct {
    using _: PxJointLimitParameters,
    yAngle: _c.float,
    zAngle: _c.float,
}


PxJointLimitPyramid :: struct {
    using _: PxJointLimitParameters,
    yAngleMin: _c.float,
    yAngleMax: _c.float,
    zAngleMin: _c.float,
    zAngleMax: _c.float,
}


PxPrismaticJoint :: struct {
    using _: PxJoint,
}


PxRevoluteJoint :: struct {
    using _: PxJoint,
}


PxSphericalJoint :: struct {
    using _: PxJoint,
}


PxD6JointDrive :: struct {
    using _: PxSpring,
    forceLimit: _c.float,
    flags: PxD6JointDriveFlags_Set,
}


PxD6Joint :: struct {
    using _: PxJoint,
}


PxGearJoint :: struct {
    using _: PxJoint,
}


PxRackAndPinionJoint :: struct {
    using _: PxJoint,
}


PxGroupsMask :: struct {
    bits0: _c.uint16_t,
    bits1: _c.uint16_t,
    bits2: _c.uint16_t,
    bits3: _c.uint16_t,
}


PxDefaultErrorCallback :: struct {
    using _: PxErrorCallback,
}


PxRigidActorExt :: struct {
};

PxMassProperties :: struct {
    inertiaTensor: PxMat33,
    centerOfMass: PxVec3,
    mass: _c.float,
}


PxRigidBodyExt :: struct {
};

PxShapeExt :: struct {
};

PxMeshOverlapUtil :: struct {
    _pad0: [1040]u8,
}


PxBinaryConverter :: distinct rawptr

PxXmlMiscParameter :: struct {
    upVector: PxVec3,
    scale: PxTolerancesScale,
}


PxSerialization :: struct {
};

PxDefaultCpuDispatcher :: struct {
    using _: PxCpuDispatcher,
}


PxStringTableExt :: struct {
};

PxBroadPhaseExt :: struct {
};

PxSceneQueryExt :: struct {
};

PxBatchQueryExt :: struct {
    _pad0: [8]u8,
}


PxCustomSceneQuerySystem :: struct {
    using _: PxSceneQuerySystem,
}


PxCustomSceneQuerySystemAdapter :: struct {
    _pad0: [8]u8,
}


PxSamplingExt :: struct {
};

PxPoissonSampler :: struct {
    using _: PxUserAllocated,
    _pad0: [8]u8,
}


PxTriangleMeshPoissonSampler :: struct {
    using _: PxPoissonSampler,
}


PxTetrahedronMeshExt :: struct {
};

PxRepXObject :: struct {
    typeName: ^_c.char,
    serializable: rawptr,
    id: _c.uint64_t,
}


PxCooking :: distinct rawptr

PxRepXInstantiationArgs :: struct {
    _pad0: [8]u8,
    cooker: ^PxCooking,
    stringTable: ^PxStringTable,
}


XmlMemoryAllocator :: distinct rawptr

XmlWriter :: distinct rawptr

XmlReader :: distinct rawptr

MemoryBuffer :: distinct rawptr

PxRepXSerializer :: struct {
    _pad0: [8]u8,
}


PxVehicleWheels4SimData :: distinct rawptr

PxVehicleWheels4DynData :: distinct rawptr

PxVehicleTireForceCalculator :: distinct rawptr

PxVehicleDrivableSurfaceToTireFrictionPairs :: distinct rawptr

PxVehicleTelemetryData :: distinct rawptr

PxPvd :: struct {
    using _: PxProfilerCallback,
}


PxPvdTransport :: struct {
    _pad0: [8]u8,
}

when ODIN_OS == .Linux do foreign import libphysx "libphysx.so"

@(default_calling_convention = "c")
foreign libphysx {
    @(link_name = "PxAllocatorCallback_delete")
    allocator_callback_delete :: proc(self_: ^PxAllocatorCallback) ---

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
    allocator_callback_allocate_mut :: proc(self_: ^PxAllocatorCallback, size: _c.size_t, typeName: ^_c.char, filename: ^_c.char, line: _c.int32_t) -> rawptr ---

    /// Frees memory previously allocated by allocate().
    ///
    /// Threading:
    /// This function should be thread safe as it can be called in the context of the user thread
    /// and physics processing thread(s).
    @(link_name = "PxAllocatorCallback_deallocate_mut")
    allocator_callback_deallocate_mut :: proc(self_: ^PxAllocatorCallback, ptr: rawptr) ---

    @(link_name = "PxAssertHandler_delete")
    assert_handler_delete :: proc(self_: ^PxAssertHandler) ---

    @(link_name = "phys_PxGetAssertHandler")
    get_assert_handler :: proc() -> ^PxAssertHandler ---

    @(link_name = "phys_PxSetAssertHandler")
    set_assert_handler :: proc(handler: ^PxAssertHandler) ---

    /// Destroys the instance it is called on.
    ///
    /// The operation will fail, if there are still modules referencing the foundation object. Release all dependent modules
    /// prior to calling this method.
    @(link_name = "PxFoundation_release_mut")
    foundation_release_mut :: proc(self_: ^PxFoundation) ---

    /// retrieves error callback
    @(link_name = "PxFoundation_getErrorCallback_mut")
    foundation_get_error_callback_mut :: proc(self_: ^PxFoundation) -> ^PxErrorCallback ---

    /// Sets mask of errors to report.
    @(link_name = "PxFoundation_setErrorLevel_mut")
    foundation_set_error_level_mut :: proc(self_: ^PxFoundation, mask: _c.uint32_t) ---

    /// Retrieves mask of errors to be reported.
    @(link_name = "PxFoundation_getErrorLevel")
    foundation_get_error_level :: proc(self_: ^PxFoundation) -> _c.uint32_t ---

    /// Retrieves the allocator this object was created with.
    @(link_name = "PxFoundation_getAllocatorCallback_mut")
    foundation_get_allocator_callback_mut :: proc(self_: ^PxFoundation) -> ^PxAllocatorCallback ---

    /// Retrieves if allocation names are being passed to allocator callback.
    @(link_name = "PxFoundation_getReportAllocationNames")
    foundation_get_report_allocation_names :: proc(self_: ^PxFoundation) -> _c.bool ---

    /// Set if allocation names are being passed to allocator callback.
    ///
    /// Enabled by default in debug and checked build, disabled by default in profile and release build.
    @(link_name = "PxFoundation_setReportAllocationNames_mut")
    foundation_set_report_allocation_names_mut :: proc(self_: ^PxFoundation, value: _c.bool) ---

    @(link_name = "PxFoundation_registerAllocationListener_mut")
    foundation_register_allocation_listener_mut :: proc(self_: ^PxFoundation, listener: ^PxAllocationListener) ---

    @(link_name = "PxFoundation_deregisterAllocationListener_mut")
    foundation_deregister_allocation_listener_mut :: proc(self_: ^PxFoundation, listener: ^PxAllocationListener) ---

    @(link_name = "PxFoundation_registerErrorCallback_mut")
    foundation_register_error_callback_mut :: proc(self_: ^PxFoundation, callback: ^PxErrorCallback) ---

    @(link_name = "PxFoundation_deregisterErrorCallback_mut")
    foundation_deregister_error_callback_mut :: proc(self_: ^PxFoundation, callback: ^PxErrorCallback) ---

    /// Creates an instance of the foundation class
    ///
    /// The foundation class is needed to initialize higher level SDKs. There may be only one instance per process.
    /// Calling this method after an instance has been created already will result in an error message and NULL will be
    /// returned.
    ///
    /// Foundation instance on success, NULL if operation failed
    @(link_name = "phys_PxCreateFoundation")
    create_foundation :: proc(version: _c.uint32_t, allocator: ^PxAllocatorCallback, errorCallback: ^PxErrorCallback) -> ^PxFoundation ---

    @(link_name = "phys_PxSetFoundationInstance")
    set_foundation_instance :: proc(foundation: ^PxFoundation) ---

    @(link_name = "phys_PxGetFoundation")
    get_foundation :: proc() -> ^PxFoundation ---

    /// Get the callback that will be used for all profiling.
    @(link_name = "phys_PxGetProfilerCallback")
    get_profiler_callback :: proc() -> ^PxProfilerCallback ---

    /// Set the callback that will be used for all profiling.
    @(link_name = "phys_PxSetProfilerCallback")
    set_profiler_callback :: proc(profiler: ^PxProfilerCallback) ---

    /// Get the allocator callback
    @(link_name = "phys_PxGetAllocatorCallback")
    get_allocator_callback :: proc() -> ^PxAllocatorCallback ---

    /// Get the broadcasting allocator callback
    @(link_name = "phys_PxGetBroadcastAllocator")
    get_broadcast_allocator :: proc() -> ^PxAllocatorCallback ---

    /// Get the error callback
    @(link_name = "phys_PxGetErrorCallback")
    get_error_callback :: proc() -> ^PxErrorCallback ---

    /// Get the broadcasting error callback
    @(link_name = "phys_PxGetBroadcastError")
    get_broadcast_error :: proc() -> ^PxErrorCallback ---

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
    allocator_new :: proc(anon_param0: ^_c.char) -> PxAllocator ---

    @(link_name = "PxAllocator_allocate_mut")
    allocator_allocate_mut :: proc(self_: ^PxAllocator, size: _c.size_t, file: ^_c.char, line: _c.int32_t) -> rawptr ---

    @(link_name = "PxAllocator_deallocate_mut")
    allocator_deallocate_mut :: proc(self_: ^PxAllocator, ptr: rawptr) ---

    @(link_name = "PxRawAllocator_new")
    raw_allocator_new :: proc(anon_param0: ^_c.char) -> PxRawAllocator ---

    @(link_name = "PxRawAllocator_allocate_mut")
    raw_allocator_allocate_mut :: proc(self_: ^PxRawAllocator, size: _c.size_t, anon_param1: ^_c.char, anon_param2: _c.int32_t) -> rawptr ---

    @(link_name = "PxRawAllocator_deallocate_mut")
    raw_allocator_deallocate_mut :: proc(self_: ^PxRawAllocator, ptr: rawptr) ---

    @(link_name = "PxVirtualAllocatorCallback_delete")
    virtual_allocator_callback_delete :: proc(self_: ^PxVirtualAllocatorCallback) ---

    @(link_name = "PxVirtualAllocatorCallback_allocate_mut")
    virtual_allocator_callback_allocate_mut :: proc(self_: ^PxVirtualAllocatorCallback, size: _c.size_t, group: _c.int32_t, file: ^_c.char, line: _c.int32_t) -> rawptr ---

    @(link_name = "PxVirtualAllocatorCallback_deallocate_mut")
    virtual_allocator_callback_deallocate_mut :: proc(self_: ^PxVirtualAllocatorCallback, ptr: rawptr) ---

    @(link_name = "PxVirtualAllocator_new")
    virtual_allocator_new :: proc(callback: ^PxVirtualAllocatorCallback, group: _c.int32_t) -> PxVirtualAllocator ---

    @(link_name = "PxVirtualAllocator_allocate_mut")
    virtual_allocator_allocate_mut :: proc(self_: ^PxVirtualAllocator, size: _c.size_t, file: ^_c.char, line: _c.int32_t) -> rawptr ---

    @(link_name = "PxVirtualAllocator_deallocate_mut")
    virtual_allocator_deallocate_mut :: proc(self_: ^PxVirtualAllocator, ptr: rawptr) ---

    @(link_name = "PxTempAllocatorChunk_new")
    temp_allocator_chunk_new :: proc() -> PxTempAllocatorChunk ---

    @(link_name = "PxTempAllocator_new")
    temp_allocator_new :: proc(anon_param0: ^_c.char) -> PxTempAllocator ---

    @(link_name = "PxTempAllocator_allocate_mut")
    temp_allocator_allocate_mut :: proc(self_: ^PxTempAllocator, size: _c.size_t, file: ^_c.char, line: _c.int32_t) -> rawptr ---

    @(link_name = "PxTempAllocator_deallocate_mut")
    temp_allocator_deallocate_mut :: proc(self_: ^PxTempAllocator, ptr: rawptr) ---

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
    vec3_new :: proc() -> PxVec3 ---

    /// zero constructor.
    @(link_name = "PxVec3_new_1")
    vec3_new_1 :: proc(anon_param0: PxZERO) -> PxVec3 ---

    /// Assigns scalar parameter to all elements.
    ///
    /// Useful to initialize to zero or one.
    @(link_name = "PxVec3_new_2")
    vec3_new_2 :: proc(a: _c.float) -> PxVec3 ---

    /// Initializes from 3 scalar parameters.
    @(link_name = "PxVec3_new_3")
    vec3_new_3 :: proc(nx: _c.float, ny: _c.float, nz: _c.float) -> PxVec3 ---

    /// tests for exact zero vector
    @(link_name = "PxVec3_isZero")
    vec3_is_zero :: proc(self_: ^PxVec3) -> _c.bool ---

    /// returns true if all 3 elems of the vector are finite (not NAN or INF, etc.)
    @(link_name = "PxVec3_isFinite")
    vec3_is_finite :: proc(self_: ^PxVec3) -> _c.bool ---

    /// is normalized - used by API parameter validation
    @(link_name = "PxVec3_isNormalized")
    vec3_is_normalized :: proc(self_: ^PxVec3) -> _c.bool ---

    /// returns the squared magnitude
    ///
    /// Avoids calling PxSqrt()!
    @(link_name = "PxVec3_magnitudeSquared")
    vec3_magnitude_squared :: proc(self_: ^PxVec3) -> _c.float ---

    /// returns the magnitude
    @(link_name = "PxVec3_magnitude")
    vec3_magnitude :: proc(self_: ^PxVec3) -> _c.float ---

    /// returns the scalar product of this and other.
    @(link_name = "PxVec3_dot")
    vec3_dot :: proc(self_: ^PxVec3, #by_ptr v: PxVec3) -> _c.float ---

    /// cross product
    @(link_name = "PxVec3_cross")
    vec3_cross :: proc(self_: ^PxVec3, #by_ptr v: PxVec3) -> PxVec3 ---

    /// returns a unit vector
    @(link_name = "PxVec3_getNormalized")
    vec3_get_normalized :: proc(self_: ^PxVec3) -> PxVec3 ---

    /// normalizes the vector in place
    @(link_name = "PxVec3_normalize_mut")
    vec3_normalize_mut :: proc(self_: ^PxVec3) -> _c.float ---

    /// normalizes the vector in place. Does nothing if vector magnitude is under PX_NORMALIZATION_EPSILON.
    /// Returns vector magnitude if >= PX_NORMALIZATION_EPSILON and 0.0f otherwise.
    @(link_name = "PxVec3_normalizeSafe_mut")
    vec3_normalize_safe_mut :: proc(self_: ^PxVec3) -> _c.float ---

    /// normalizes the vector in place. Asserts if vector magnitude is under PX_NORMALIZATION_EPSILON.
    /// returns vector magnitude.
    @(link_name = "PxVec3_normalizeFast_mut")
    vec3_normalize_fast_mut :: proc(self_: ^PxVec3) -> _c.float ---

    /// a[i] * b[i], for all i.
    @(link_name = "PxVec3_multiply")
    vec3_multiply :: proc(self_: ^PxVec3, #by_ptr a: PxVec3) -> PxVec3 ---

    /// element-wise minimum
    @(link_name = "PxVec3_minimum")
    vec3_minimum :: proc(self_: ^PxVec3, #by_ptr v: PxVec3) -> PxVec3 ---

    /// returns MIN(x, y, z);
    @(link_name = "PxVec3_minElement")
    vec3_min_element :: proc(self_: ^PxVec3) -> _c.float ---

    /// element-wise maximum
    @(link_name = "PxVec3_maximum")
    vec3_maximum :: proc(self_: ^PxVec3, #by_ptr v: PxVec3) -> PxVec3 ---

    /// returns MAX(x, y, z);
    @(link_name = "PxVec3_maxElement")
    vec3_max_element :: proc(self_: ^PxVec3) -> _c.float ---

    /// returns absolute values of components;
    @(link_name = "PxVec3_abs")
    vec3_abs :: proc(self_: ^PxVec3) -> PxVec3 ---

    @(link_name = "PxVec3Padded_new_alloc")
    vec3_padded_new_alloc :: proc() -> ^PxVec3Padded ---

    @(link_name = "PxVec3Padded_delete")
    vec3_padded_delete :: proc(self_: ^PxVec3Padded) ---

    @(link_name = "PxVec3Padded_new_alloc_1")
    vec3_padded_new_alloc_1 :: proc(#by_ptr p: PxVec3) -> ^PxVec3Padded ---

    @(link_name = "PxVec3Padded_new_alloc_2")
    vec3_padded_new_alloc_2 :: proc(f: _c.float) -> ^PxVec3Padded ---

    /// Default constructor, does not do any initialization.
    @(link_name = "PxQuat_new")
    quat_new :: proc() -> PxQuat ---

    /// identity constructor
    @(link_name = "PxQuat_new_1")
    quat_new_1 :: proc(anon_param0: PxIDENTITY) -> PxQuat ---

    /// Constructor from a scalar: sets the real part w to the scalar value, and the imaginary parts (x,y,z) to zero
    @(link_name = "PxQuat_new_2")
    quat_new_2 :: proc(r: _c.float) -> PxQuat ---

    /// Constructor. Take note of the order of the elements!
    @(link_name = "PxQuat_new_3")
    quat_new_3 :: proc(nx: _c.float, ny: _c.float, nz: _c.float, nw: _c.float) -> PxQuat ---

    /// Creates from angle-axis representation.
    ///
    /// Axis must be normalized!
    ///
    /// Angle is in radians!
    ///
    /// Unit:
    /// Radians
    @(link_name = "PxQuat_new_4")
    quat_new_4 :: proc(angleRadians: _c.float, #by_ptr unitAxis: PxVec3) -> PxQuat ---

    /// Creates from orientation matrix.
    @(link_name = "PxQuat_new_5")
    quat_new_5 :: proc(#by_ptr m: PxMat33) -> PxQuat ---

    /// returns true if quat is identity
    @(link_name = "PxQuat_isIdentity")
    quat_is_identity :: proc(self_: ^PxQuat) -> _c.bool ---

    /// returns true if all elements are finite (not NAN or INF, etc.)
    @(link_name = "PxQuat_isFinite")
    quat_is_finite :: proc(self_: ^PxQuat) -> _c.bool ---

    /// returns true if finite and magnitude is close to unit
    @(link_name = "PxQuat_isUnit")
    quat_is_unit :: proc(self_: ^PxQuat) -> _c.bool ---

    /// returns true if finite and magnitude is reasonably close to unit to allow for some accumulation of error vs
    /// isValid
    @(link_name = "PxQuat_isSane")
    quat_is_sane :: proc(self_: ^PxQuat) -> _c.bool ---

    /// converts this quaternion to angle-axis representation
    @(link_name = "PxQuat_toRadiansAndUnitAxis")
    quat_to_radians_and_unit_axis :: proc(self_: ^PxQuat, angle: ^_c.float, axis: ^PxVec3) ---

    /// Gets the angle between this quat and the identity quaternion.
    ///
    /// Unit:
    /// Radians
    @(link_name = "PxQuat_getAngle")
    quat_get_angle :: proc(self_: ^PxQuat) -> _c.float ---

    /// Gets the angle between this quat and the argument
    ///
    /// Unit:
    /// Radians
    @(link_name = "PxQuat_getAngle_1")
    quat_get_angle_1 :: proc(self_: ^PxQuat, #by_ptr q: PxQuat) -> _c.float ---

    /// This is the squared 4D vector length, should be 1 for unit quaternions.
    @(link_name = "PxQuat_magnitudeSquared")
    quat_magnitude_squared :: proc(self_: ^PxQuat) -> _c.float ---

    /// returns the scalar product of this and other.
    @(link_name = "PxQuat_dot")
    quat_dot :: proc(self_: ^PxQuat, #by_ptr v: PxQuat) -> _c.float ---

    @(link_name = "PxQuat_getNormalized")
    quat_get_normalized :: proc(self_: ^PxQuat) -> PxQuat ---

    @(link_name = "PxQuat_magnitude")
    quat_magnitude :: proc(self_: ^PxQuat) -> _c.float ---

    /// maps to the closest unit quaternion.
    @(link_name = "PxQuat_normalize_mut")
    quat_normalize_mut :: proc(self_: ^PxQuat) -> _c.float ---

    @(link_name = "PxQuat_getConjugate")
    quat_get_conjugate :: proc(self_: ^PxQuat) -> PxQuat ---

    @(link_name = "PxQuat_getImaginaryPart")
    quat_get_imaginary_part :: proc(self_: ^PxQuat) -> PxVec3 ---

    /// brief computes rotation of x-axis
    @(link_name = "PxQuat_getBasisVector0")
    quat_get_basis_vector0 :: proc(self_: ^PxQuat) -> PxVec3 ---

    /// brief computes rotation of y-axis
    @(link_name = "PxQuat_getBasisVector1")
    quat_get_basis_vector1 :: proc(self_: ^PxQuat) -> PxVec3 ---

    /// brief computes rotation of z-axis
    @(link_name = "PxQuat_getBasisVector2")
    quat_get_basis_vector2 :: proc(self_: ^PxQuat) -> PxVec3 ---

    /// rotates passed vec by this (assumed unitary)
    @(link_name = "PxQuat_rotate")
    quat_rotate :: proc(self_: ^PxQuat, #by_ptr v: PxVec3) -> PxVec3 ---

    /// inverse rotates passed vec by this (assumed unitary)
    @(link_name = "PxQuat_rotateInv")
    quat_rotate_inv :: proc(self_: ^PxQuat, #by_ptr v: PxVec3) -> PxVec3 ---

    @(link_name = "PxTransform_new")
    transform_new :: proc() -> PxTransform ---

    @(link_name = "PxTransform_new_1")
    transform_new_1 :: proc(#by_ptr position: PxVec3) -> PxTransform ---

    @(link_name = "PxTransform_new_2")
    transform_new_2 :: proc(anon_param0: PxIDENTITY) -> PxTransform ---

    @(link_name = "PxTransform_new_3")
    transform_new_3 :: proc(#by_ptr orientation: PxQuat) -> PxTransform ---

    @(link_name = "PxTransform_new_4")
    transform_new_4 :: proc(x: _c.float, y: _c.float, z: _c.float, aQ: PxQuat) -> PxTransform ---

    @(link_name = "PxTransform_new_5")
    transform_new_5 :: proc(#by_ptr p0: PxVec3, #by_ptr q0: PxQuat) -> PxTransform ---

    @(link_name = "PxTransform_new_6")
    transform_new_6 :: proc(#by_ptr m: PxMat44) -> PxTransform ---

    @(link_name = "PxTransform_getInverse")
    transform_get_inverse :: proc(self_: ^PxTransform) -> PxTransform ---

    @(link_name = "PxTransform_transform")
    transform_transform :: proc(self_: ^PxTransform, #by_ptr input: PxVec3) -> PxVec3 ---

    @(link_name = "PxTransform_transformInv")
    transform_transform_inv :: proc(self_: ^PxTransform, #by_ptr input: PxVec3) -> PxVec3 ---

    @(link_name = "PxTransform_rotate")
    transform_rotate :: proc(self_: ^PxTransform, #by_ptr input: PxVec3) -> PxVec3 ---

    @(link_name = "PxTransform_rotateInv")
    transform_rotate_inv :: proc(self_: ^PxTransform, #by_ptr input: PxVec3) -> PxVec3 ---

    /// Transform transform to parent (returns compound transform: first src, then *this)
    @(link_name = "PxTransform_transform_1")
    transform_transform_1 :: proc(self_: ^PxTransform, #by_ptr src: PxTransform) -> PxTransform ---

    /// returns true if finite and q is a unit quaternion
    @(link_name = "PxTransform_isValid")
    transform_is_valid :: proc(self_: ^PxTransform) -> _c.bool ---

    /// returns true if finite and quat magnitude is reasonably close to unit to allow for some accumulation of error
    /// vs isValid
    @(link_name = "PxTransform_isSane")
    transform_is_sane :: proc(self_: ^PxTransform) -> _c.bool ---

    /// returns true if all elems are finite (not NAN or INF, etc.)
    @(link_name = "PxTransform_isFinite")
    transform_is_finite :: proc(self_: ^PxTransform) -> _c.bool ---

    /// Transform transform from parent (returns compound transform: first src, then this->inverse)
    @(link_name = "PxTransform_transformInv_1")
    transform_transform_inv_1 :: proc(self_: ^PxTransform, #by_ptr src: PxTransform) -> PxTransform ---

    /// return a normalized transform (i.e. one in which the quaternion has unit magnitude)
    @(link_name = "PxTransform_getNormalized")
    transform_get_normalized :: proc(self_: ^PxTransform) -> PxTransform ---

    /// Default constructor
    @(link_name = "PxMat33_new")
    mat33_new :: proc() -> PxMat33 ---

    /// identity constructor
    @(link_name = "PxMat33_new_1")
    mat33_new_1 :: proc(anon_param0: PxIDENTITY) -> PxMat33 ---

    /// zero constructor
    @(link_name = "PxMat33_new_2")
    mat33_new_2 :: proc(anon_param0: PxZERO) -> PxMat33 ---

    /// Construct from three base vectors
    @(link_name = "PxMat33_new_3")
    mat33_new_3 :: proc(#by_ptr col0: PxVec3, #by_ptr col1: PxVec3, #by_ptr col2: PxVec3) -> PxMat33 ---

    /// constructor from a scalar, which generates a multiple of the identity matrix
    @(link_name = "PxMat33_new_4")
    mat33_new_4 :: proc(r: _c.float) -> PxMat33 ---

    /// Construct from float[9]
    @(link_name = "PxMat33_new_5")
    mat33_new_5 :: proc(values: ^_c.float) -> PxMat33 ---

    /// Construct from a quaternion
    @(link_name = "PxMat33_new_6")
    mat33_new_6 :: proc(#by_ptr q: PxQuat) -> PxMat33 ---

    /// Construct from diagonal, off-diagonals are zero.
    @(link_name = "PxMat33_createDiagonal")
    mat33_create_diagonal :: proc(#by_ptr d: PxVec3) -> PxMat33 ---

    /// Computes the outer product of two vectors
    @(link_name = "PxMat33_outer")
    mat33_outer :: proc(#by_ptr a: PxVec3, #by_ptr b: PxVec3) -> PxMat33 ---

    /// Get transposed matrix
    @(link_name = "PxMat33_getTranspose")
    mat33_get_transpose :: proc(self_: ^PxMat33) -> PxMat33 ---

    /// Get the real inverse
    @(link_name = "PxMat33_getInverse")
    mat33_get_inverse :: proc(self_: ^PxMat33) -> PxMat33 ---

    /// Get determinant
    @(link_name = "PxMat33_getDeterminant")
    mat33_get_determinant :: proc(self_: ^PxMat33) -> _c.float ---

    /// Transform vector by matrix, equal to v' = M*v
    @(link_name = "PxMat33_transform")
    mat33_transform :: proc(self_: ^PxMat33, #by_ptr other: PxVec3) -> PxVec3 ---

    /// Transform vector by matrix transpose, v' = M^t*v
    @(link_name = "PxMat33_transformTranspose")
    mat33_transform_transpose :: proc(self_: ^PxMat33, #by_ptr other: PxVec3) -> PxVec3 ---

    @(link_name = "PxMat33_front")
    mat33_front :: proc(self_: ^PxMat33) -> ^_c.float ---

    /// Default constructor, not performing any initialization for performance reason.
    ///
    /// Use empty() function below to construct empty bounds.
    @(link_name = "PxBounds3_new")
    bounds3_new :: proc() -> PxBounds3 ---

    /// Construct from two bounding points
    @(link_name = "PxBounds3_new_1")
    bounds3_new_1 :: proc(#by_ptr minimum: PxVec3, #by_ptr maximum: PxVec3) -> PxBounds3 ---

    /// Return empty bounds.
    @(link_name = "PxBounds3_empty")
    bounds3_empty :: proc() -> PxBounds3 ---

    /// returns the AABB containing v0 and v1.
    @(link_name = "PxBounds3_boundsOfPoints")
    bounds3_bounds_of_points :: proc(#by_ptr v0: PxVec3, #by_ptr v1: PxVec3) -> PxBounds3 ---

    /// returns the AABB from center and extents vectors.
    @(link_name = "PxBounds3_centerExtents")
    bounds3_center_extents :: proc(#by_ptr center: PxVec3, #by_ptr extent: PxVec3) -> PxBounds3 ---

    /// Construct from center, extent, and (not necessarily orthogonal) basis
    @(link_name = "PxBounds3_basisExtent")
    bounds3_basis_extent :: proc(#by_ptr center: PxVec3, #by_ptr basis: PxMat33, #by_ptr extent: PxVec3) -> PxBounds3 ---

    /// Construct from pose and extent
    @(link_name = "PxBounds3_poseExtent")
    bounds3_pose_extent :: proc(#by_ptr pose: PxTransform, #by_ptr extent: PxVec3) -> PxBounds3 ---

    /// gets the transformed bounds of the passed AABB (resulting in a bigger AABB).
    ///
    /// This version is safe to call for empty bounds.
    @(link_name = "PxBounds3_transformSafe")
    bounds3_transform_safe :: proc(#by_ptr matrix_: PxMat33, #by_ptr bounds: PxBounds3) -> PxBounds3 ---

    /// gets the transformed bounds of the passed AABB (resulting in a bigger AABB).
    ///
    /// Calling this method for empty bounds leads to undefined behavior. Use [`transformSafe`]() instead.
    @(link_name = "PxBounds3_transformFast")
    bounds3_transform_fast :: proc(#by_ptr matrix_: PxMat33, #by_ptr bounds: PxBounds3) -> PxBounds3 ---

    /// gets the transformed bounds of the passed AABB (resulting in a bigger AABB).
    ///
    /// This version is safe to call for empty bounds.
    @(link_name = "PxBounds3_transformSafe_1")
    bounds3_transform_safe_1 :: proc(#by_ptr transform: PxTransform, #by_ptr bounds: PxBounds3) -> PxBounds3 ---

    /// gets the transformed bounds of the passed AABB (resulting in a bigger AABB).
    ///
    /// Calling this method for empty bounds leads to undefined behavior. Use [`transformSafe`]() instead.
    @(link_name = "PxBounds3_transformFast_1")
    bounds3_transform_fast_1 :: proc(#by_ptr transform: PxTransform, #by_ptr bounds: PxBounds3) -> PxBounds3 ---

    /// Sets empty to true
    @(link_name = "PxBounds3_setEmpty_mut")
    bounds3_set_empty_mut :: proc(self_: ^PxBounds3) ---

    /// Sets the bounds to maximum size [-PX_MAX_BOUNDS_EXTENTS, PX_MAX_BOUNDS_EXTENTS].
    @(link_name = "PxBounds3_setMaximal_mut")
    bounds3_set_maximal_mut :: proc(self_: ^PxBounds3) ---

    /// expands the volume to include v
    @(link_name = "PxBounds3_include_mut")
    bounds3_include_mut :: proc(self_: ^PxBounds3, #by_ptr v: PxVec3) ---

    /// expands the volume to include b.
    @(link_name = "PxBounds3_include_mut_1")
    bounds3_include_mut_1 :: proc(self_: ^PxBounds3, #by_ptr b: PxBounds3) ---

    @(link_name = "PxBounds3_isEmpty")
    bounds3_is_empty :: proc(self_: ^PxBounds3) -> _c.bool ---

    /// indicates whether the intersection of this and b is empty or not.
    @(link_name = "PxBounds3_intersects")
    bounds3_intersects :: proc(self_: ^PxBounds3, #by_ptr b: PxBounds3) -> _c.bool ---

    /// computes the 1D-intersection between two AABBs, on a given axis.
    @(link_name = "PxBounds3_intersects1D")
    bounds3_intersects1_d :: proc(self_: ^PxBounds3, #by_ptr a: PxBounds3, axis: _c.uint32_t) -> _c.bool ---

    /// indicates if these bounds contain v.
    @(link_name = "PxBounds3_contains")
    bounds3_contains :: proc(self_: ^PxBounds3, #by_ptr v: PxVec3) -> _c.bool ---

    /// checks a box is inside another box.
    @(link_name = "PxBounds3_isInside")
    bounds3_is_inside :: proc(self_: ^PxBounds3, #by_ptr box: PxBounds3) -> _c.bool ---

    /// returns the center of this axis aligned box.
    @(link_name = "PxBounds3_getCenter")
    bounds3_get_center :: proc(self_: ^PxBounds3) -> PxVec3 ---

    /// get component of the box's center along a given axis
    @(link_name = "PxBounds3_getCenter_1")
    bounds3_get_center_1 :: proc(self_: ^PxBounds3, axis: _c.uint32_t) -> _c.float ---

    /// get component of the box's extents along a given axis
    @(link_name = "PxBounds3_getExtents")
    bounds3_get_extents :: proc(self_: ^PxBounds3, axis: _c.uint32_t) -> _c.float ---

    /// returns the dimensions (width/height/depth) of this axis aligned box.
    @(link_name = "PxBounds3_getDimensions")
    bounds3_get_dimensions :: proc(self_: ^PxBounds3) -> PxVec3 ---

    /// returns the extents, which are half of the width/height/depth.
    @(link_name = "PxBounds3_getExtents_1")
    bounds3_get_extents_1 :: proc(self_: ^PxBounds3) -> PxVec3 ---

    /// scales the AABB.
    ///
    /// This version is safe to call for empty bounds.
    @(link_name = "PxBounds3_scaleSafe_mut")
    bounds3_scale_safe_mut :: proc(self_: ^PxBounds3, scale: _c.float) ---

    /// scales the AABB.
    ///
    /// Calling this method for empty bounds leads to undefined behavior. Use [`scaleSafe`]() instead.
    @(link_name = "PxBounds3_scaleFast_mut")
    bounds3_scale_fast_mut :: proc(self_: ^PxBounds3, scale: _c.float) ---

    /// fattens the AABB in all 3 dimensions by the given distance.
    ///
    /// This version is safe to call for empty bounds.
    @(link_name = "PxBounds3_fattenSafe_mut")
    bounds3_fatten_safe_mut :: proc(self_: ^PxBounds3, distance: _c.float) ---

    /// fattens the AABB in all 3 dimensions by the given distance.
    ///
    /// Calling this method for empty bounds leads to undefined behavior. Use [`fattenSafe`]() instead.
    @(link_name = "PxBounds3_fattenFast_mut")
    bounds3_fatten_fast_mut :: proc(self_: ^PxBounds3, distance: _c.float) ---

    /// checks that the AABB values are not NaN
    @(link_name = "PxBounds3_isFinite")
    bounds3_is_finite :: proc(self_: ^PxBounds3) -> _c.bool ---

    /// checks that the AABB values describe a valid configuration.
    @(link_name = "PxBounds3_isValid")
    bounds3_is_valid :: proc(self_: ^PxBounds3) -> _c.bool ---

    /// Finds the closest point in the box to the point p. If p is contained, this will be p, otherwise it
    /// will be the closest point on the surface of the box.
    @(link_name = "PxBounds3_closestPoint")
    bounds3_closest_point :: proc(self_: ^PxBounds3, #by_ptr p: PxVec3) -> PxVec3 ---

    @(link_name = "PxErrorCallback_delete")
    error_callback_delete :: proc(self_: ^PxErrorCallback) ---

    /// Reports an error code.
    @(link_name = "PxErrorCallback_reportError_mut")
    error_callback_report_error_mut :: proc(self_: ^PxErrorCallback, code: PxErrorCode, message: ^_c.char, file: ^_c.char, line: _c.int32_t) ---

    /// callback when memory is allocated.
    @(link_name = "PxAllocationListener_onAllocation_mut")
    allocation_listener_on_allocation_mut :: proc(self_: ^PxAllocationListener, size: _c.size_t, typeName: ^_c.char, filename: ^_c.char, line: _c.int32_t, allocatedMemory: rawptr) ---

    /// callback when memory is deallocated.
    @(link_name = "PxAllocationListener_onDeallocation_mut")
    allocation_listener_on_deallocation_mut :: proc(self_: ^PxAllocationListener, allocatedMemory: rawptr) ---

    /// The default constructor.
    @(link_name = "PxBroadcastingAllocator_new_alloc")
    broadcasting_allocator_new_alloc :: proc(allocator: ^PxAllocatorCallback, error: ^PxErrorCallback) -> ^PxBroadcastingAllocator ---

    /// The default constructor.
    @(link_name = "PxBroadcastingAllocator_delete")
    broadcasting_allocator_delete :: proc(self_: ^PxBroadcastingAllocator) ---

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
    broadcasting_allocator_allocate_mut :: proc(self_: ^PxBroadcastingAllocator, size: _c.size_t, typeName: ^_c.char, filename: ^_c.char, line: _c.int32_t) -> rawptr ---

    /// Frees memory previously allocated by allocate().
    ///
    /// Threading:
    /// This function should be thread safe as it can be called in the context of the user thread
    /// and physics processing thread(s).
    @(link_name = "PxBroadcastingAllocator_deallocate_mut")
    broadcasting_allocator_deallocate_mut :: proc(self_: ^PxBroadcastingAllocator, ptr: rawptr) ---

    /// The default constructor.
    @(link_name = "PxBroadcastingErrorCallback_new_alloc")
    broadcasting_error_callback_new_alloc :: proc(errorCallback: ^PxErrorCallback) -> ^PxBroadcastingErrorCallback ---

    /// The default destructor.
    @(link_name = "PxBroadcastingErrorCallback_delete")
    broadcasting_error_callback_delete :: proc(self_: ^PxBroadcastingErrorCallback) ---

    /// Reports an error code.
    @(link_name = "PxBroadcastingErrorCallback_reportError_mut")
    broadcasting_error_callback_report_error_mut :: proc(self_: ^PxBroadcastingErrorCallback, code: PxErrorCode, message: ^_c.char, file: ^_c.char, line: _c.int32_t) ---

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
    input_stream_read_mut :: proc(self_: ^PxInputStream, dest: rawptr, count: _c.uint32_t) -> _c.uint32_t ---

    @(link_name = "PxInputStream_delete")
    input_stream_delete :: proc(self_: ^PxInputStream) ---

    /// return the length of the input data
    ///
    /// size in bytes of the input data
    @(link_name = "PxInputData_getLength")
    input_data_get_length :: proc(self_: ^PxInputData) -> _c.uint32_t ---

    /// seek to the given offset from the start of the data.
    @(link_name = "PxInputData_seek_mut")
    input_data_seek_mut :: proc(self_: ^PxInputData, offset: _c.uint32_t) ---

    /// return the current offset from the start of the data
    ///
    /// the offset to seek to.
    @(link_name = "PxInputData_tell")
    input_data_tell :: proc(self_: ^PxInputData) -> _c.uint32_t ---

    @(link_name = "PxInputData_delete")
    input_data_delete :: proc(self_: ^PxInputData) ---

    /// write to the stream. The number of bytes written may be less than the number sent.
    ///
    /// the number of bytes written to the stream by this call.
    @(link_name = "PxOutputStream_write_mut")
    output_stream_write_mut :: proc(self_: ^PxOutputStream, src: rawptr, count: _c.uint32_t) -> _c.uint32_t ---

    @(link_name = "PxOutputStream_delete")
    output_stream_delete :: proc(self_: ^PxOutputStream) ---

    /// default constructor leaves data uninitialized.
    @(link_name = "PxVec4_new")
    vec4_new :: proc() -> PxVec4 ---

    /// zero constructor.
    @(link_name = "PxVec4_new_1")
    vec4_new_1 :: proc(anon_param0: PxZERO) -> PxVec4 ---

    /// Assigns scalar parameter to all elements.
    ///
    /// Useful to initialize to zero or one.
    @(link_name = "PxVec4_new_2")
    vec4_new_2 :: proc(a: _c.float) -> PxVec4 ---

    /// Initializes from 3 scalar parameters.
    @(link_name = "PxVec4_new_3")
    vec4_new_3 :: proc(nx: _c.float, ny: _c.float, nz: _c.float, nw: _c.float) -> PxVec4 ---

    /// Initializes from 3 scalar parameters.
    @(link_name = "PxVec4_new_4")
    vec4_new_4 :: proc(#by_ptr v: PxVec3, nw: _c.float) -> PxVec4 ---

    /// Initializes from an array of scalar parameters.
    @(link_name = "PxVec4_new_5")
    vec4_new_5 :: proc(v: ^_c.float) -> PxVec4 ---

    /// tests for exact zero vector
    @(link_name = "PxVec4_isZero")
    vec4_is_zero :: proc(self_: ^PxVec4) -> _c.bool ---

    /// returns true if all 3 elems of the vector are finite (not NAN or INF, etc.)
    @(link_name = "PxVec4_isFinite")
    vec4_is_finite :: proc(self_: ^PxVec4) -> _c.bool ---

    /// is normalized - used by API parameter validation
    @(link_name = "PxVec4_isNormalized")
    vec4_is_normalized :: proc(self_: ^PxVec4) -> _c.bool ---

    /// returns the squared magnitude
    ///
    /// Avoids calling PxSqrt()!
    @(link_name = "PxVec4_magnitudeSquared")
    vec4_magnitude_squared :: proc(self_: ^PxVec4) -> _c.float ---

    /// returns the magnitude
    @(link_name = "PxVec4_magnitude")
    vec4_magnitude :: proc(self_: ^PxVec4) -> _c.float ---

    /// returns the scalar product of this and other.
    @(link_name = "PxVec4_dot")
    vec4_dot :: proc(self_: ^PxVec4, #by_ptr v: PxVec4) -> _c.float ---

    /// returns a unit vector
    @(link_name = "PxVec4_getNormalized")
    vec4_get_normalized :: proc(self_: ^PxVec4) -> PxVec4 ---

    /// normalizes the vector in place
    @(link_name = "PxVec4_normalize_mut")
    vec4_normalize_mut :: proc(self_: ^PxVec4) -> _c.float ---

    /// a[i] * b[i], for all i.
    @(link_name = "PxVec4_multiply")
    vec4_multiply :: proc(self_: ^PxVec4, #by_ptr a: PxVec4) -> PxVec4 ---

    /// element-wise minimum
    @(link_name = "PxVec4_minimum")
    vec4_minimum :: proc(self_: ^PxVec4, #by_ptr v: PxVec4) -> PxVec4 ---

    /// element-wise maximum
    @(link_name = "PxVec4_maximum")
    vec4_maximum :: proc(self_: ^PxVec4, #by_ptr v: PxVec4) -> PxVec4 ---

    @(link_name = "PxVec4_getXYZ")
    vec4_get_x_y_z :: proc(self_: ^PxVec4) -> PxVec3 ---

    /// Default constructor
    @(link_name = "PxMat44_new")
    mat44_new :: proc() -> PxMat44 ---

    /// identity constructor
    @(link_name = "PxMat44_new_1")
    mat44_new_1 :: proc(anon_param0: PxIDENTITY) -> PxMat44 ---

    /// zero constructor
    @(link_name = "PxMat44_new_2")
    mat44_new_2 :: proc(anon_param0: PxZERO) -> PxMat44 ---

    /// Construct from four 4-vectors
    @(link_name = "PxMat44_new_3")
    mat44_new_3 :: proc(#by_ptr col0: PxVec4, #by_ptr col1: PxVec4, #by_ptr col2: PxVec4, #by_ptr col3: PxVec4) -> PxMat44 ---

    /// constructor that generates a multiple of the identity matrix
    @(link_name = "PxMat44_new_4")
    mat44_new_4 :: proc(r: _c.float) -> PxMat44 ---

    /// Construct from three base vectors and a translation
    @(link_name = "PxMat44_new_5")
    mat44_new_5 :: proc(#by_ptr col0: PxVec3, #by_ptr col1: PxVec3, #by_ptr col2: PxVec3, #by_ptr col3: PxVec3) -> PxMat44 ---

    /// Construct from float[16]
    @(link_name = "PxMat44_new_6")
    mat44_new_6 :: proc(values: ^_c.float) -> PxMat44 ---

    /// Construct from a quaternion
    @(link_name = "PxMat44_new_7")
    mat44_new_7 :: proc(#by_ptr q: PxQuat) -> PxMat44 ---

    /// Construct from a diagonal vector
    @(link_name = "PxMat44_new_8")
    mat44_new_8 :: proc(#by_ptr diagonal: PxVec4) -> PxMat44 ---

    /// Construct from Mat33 and a translation
    @(link_name = "PxMat44_new_9")
    mat44_new_9 :: proc(#by_ptr axes: PxMat33, #by_ptr position: PxVec3) -> PxMat44 ---

    @(link_name = "PxMat44_new_10")
    mat44_new_10 :: proc(#by_ptr t: PxTransform) -> PxMat44 ---

    /// Get transposed matrix
    @(link_name = "PxMat44_getTranspose")
    mat44_get_transpose :: proc(self_: ^PxMat44) -> PxMat44 ---

    /// Transform vector by matrix, equal to v' = M*v
    @(link_name = "PxMat44_transform")
    mat44_transform :: proc(self_: ^PxMat44, #by_ptr other: PxVec4) -> PxVec4 ---

    /// Transform vector by matrix, equal to v' = M*v
    @(link_name = "PxMat44_transform_1")
    mat44_transform_1 :: proc(self_: ^PxMat44, #by_ptr other: PxVec3) -> PxVec3 ---

    /// Rotate vector by matrix, equal to v' = M*v
    @(link_name = "PxMat44_rotate")
    mat44_rotate :: proc(self_: ^PxMat44, #by_ptr other: PxVec4) -> PxVec4 ---

    /// Rotate vector by matrix, equal to v' = M*v
    @(link_name = "PxMat44_rotate_1")
    mat44_rotate_1 :: proc(self_: ^PxMat44, #by_ptr other: PxVec3) -> PxVec3 ---

    @(link_name = "PxMat44_getBasis")
    mat44_get_basis :: proc(self_: ^PxMat44, num: _c.uint32_t) -> PxVec3 ---

    @(link_name = "PxMat44_getPosition")
    mat44_get_position :: proc(self_: ^PxMat44) -> PxVec3 ---

    @(link_name = "PxMat44_setPosition_mut")
    mat44_set_position_mut :: proc(self_: ^PxMat44, #by_ptr position: PxVec3) ---

    @(link_name = "PxMat44_front")
    mat44_front :: proc(self_: ^PxMat44) -> ^_c.float ---

    @(link_name = "PxMat44_scale_mut")
    mat44_scale_mut :: proc(self_: ^PxMat44, #by_ptr p: PxVec4) ---

    @(link_name = "PxMat44_inverseRT")
    mat44_inverse_r_t :: proc(self_: ^PxMat44) -> PxMat44 ---

    @(link_name = "PxMat44_isFinite")
    mat44_is_finite :: proc(self_: ^PxMat44) -> _c.bool ---

    /// Constructor
    @(link_name = "PxPlane_new")
    plane_new :: proc() -> PxPlane ---

    /// Constructor from a normal and a distance
    @(link_name = "PxPlane_new_1")
    plane_new_1 :: proc(nx: _c.float, ny: _c.float, nz: _c.float, distance: _c.float) -> PxPlane ---

    /// Constructor from a normal and a distance
    @(link_name = "PxPlane_new_2")
    plane_new_2 :: proc(#by_ptr normal: PxVec3, distance: _c.float) -> PxPlane ---

    /// Constructor from a point on the plane and a normal
    @(link_name = "PxPlane_new_3")
    plane_new_3 :: proc(#by_ptr point: PxVec3, #by_ptr normal: PxVec3) -> PxPlane ---

    /// Constructor from three points
    @(link_name = "PxPlane_new_4")
    plane_new_4 :: proc(#by_ptr p0: PxVec3, #by_ptr p1: PxVec3, #by_ptr p2: PxVec3) -> PxPlane ---

    @(link_name = "PxPlane_distance")
    plane_distance :: proc(self_: ^PxPlane, #by_ptr p: PxVec3) -> _c.float ---

    @(link_name = "PxPlane_contains")
    plane_contains :: proc(self_: ^PxPlane, #by_ptr p: PxVec3) -> _c.bool ---

    /// projects p into the plane
    @(link_name = "PxPlane_project")
    plane_project :: proc(self_: ^PxPlane, #by_ptr p: PxVec3) -> PxVec3 ---

    /// find an arbitrary point in the plane
    @(link_name = "PxPlane_pointInPlane")
    plane_point_in_plane :: proc(self_: ^PxPlane) -> PxVec3 ---

    /// equivalent plane with unit normal
    @(link_name = "PxPlane_normalize_mut")
    plane_normalize_mut :: proc(self_: ^PxPlane) ---

    /// transform plane
    @(link_name = "PxPlane_transform")
    plane_transform :: proc(self_: ^PxPlane, #by_ptr pose: PxTransform) -> PxPlane ---

    /// inverse-transform plane
    @(link_name = "PxPlane_inverseTransform")
    plane_inverse_transform :: proc(self_: ^PxPlane, #by_ptr pose: PxTransform) -> PxPlane ---

    /// finds the shortest rotation between two vectors.
    ///
    /// a rotation about an axis normal to the two vectors which takes one to the other via the shortest path
    @(link_name = "phys_PxShortestRotation")
    shortest_rotation :: proc(#by_ptr from: PxVec3, #by_ptr target: PxVec3) -> PxQuat ---

    @(link_name = "phys_PxDiagonalize")
    diagonalize :: proc(#by_ptr m: PxMat33, axes: ^PxQuat) -> PxVec3 ---

    /// creates a transform from the endpoints of a segment, suitable for an actor transform for a PxCapsuleGeometry
    ///
    /// A PxTransform which will transform the vector (1,0,0) to the capsule axis shrunk by the halfHeight
    @(link_name = "phys_PxTransformFromSegment")
    transform_from_segment :: proc(#by_ptr p0: PxVec3, #by_ptr p1: PxVec3, halfHeight: ^_c.float) -> PxTransform ---

    /// creates a transform from a plane equation, suitable for an actor transform for a PxPlaneGeometry
    ///
    /// a PxTransform which will transform the plane PxPlane(1,0,0,0) to the specified plane
    @(link_name = "phys_PxTransformFromPlaneEquation")
    transform_from_plane_equation :: proc(#by_ptr plane: PxPlane) -> PxTransform ---

    /// creates a plane equation from a transform, such as the actor transform for a PxPlaneGeometry
    ///
    /// the plane
    @(link_name = "phys_PxPlaneEquationFromTransform")
    plane_equation_from_transform :: proc(#by_ptr pose: PxTransform) -> PxPlane ---

    /// Spherical linear interpolation of two quaternions.
    ///
    /// Returns left when t=0, right when t=1 and a linear interpolation of left and right when 0
    /// <
    /// t
    /// <
    /// 1.
    /// Returns angle between -PI and PI in radians
    @(link_name = "phys_PxSlerp")
    slerp :: proc(t: _c.float, #by_ptr left: PxQuat, #by_ptr right: PxQuat) -> PxQuat ---

    /// integrate transform.
    @(link_name = "phys_PxIntegrateTransform")
    integrate_transform :: proc(#by_ptr curTrans: PxTransform, #by_ptr linvel: PxVec3, #by_ptr angvel: PxVec3, timeStep: _c.float, result: ^PxTransform) ---

    /// Compute the exponent of a PxVec3
    @(link_name = "phys_PxExp")
    exp :: proc(#by_ptr v: PxVec3) -> PxQuat ---

    /// computes a oriented bounding box around the scaled basis.
    ///
    /// Bounding box extent.
    @(link_name = "phys_PxOptimizeBoundingBox")
    optimize_bounding_box :: proc(basis: ^PxMat33) -> PxVec3 ---

    /// return Returns the log of a PxQuat
    @(link_name = "phys_PxLog")
    log :: proc(#by_ptr q: PxQuat) -> PxVec3 ---

    /// return Returns 0 if v.x is largest element of v, 1 if v.y is largest element, 2 if v.z is largest element.
    @(link_name = "phys_PxLargestAxis")
    largest_axis :: proc(#by_ptr v: PxVec3) -> _c.uint32_t ---

    /// Compute tan(theta/2) given sin(theta) and cos(theta) as inputs.
    ///
    /// Returns tan(theta/2)
    @(link_name = "phys_PxTanHalf")
    tan_half :: proc(sin: _c.float, cos: _c.float) -> _c.float ---

    /// Compute the closest point on an 2d ellipse to a given 2d point.
    ///
    /// Returns the 2d position on the surface of the ellipse that is closest to point.
    @(link_name = "phys_PxEllipseClamp")
    ellipse_clamp :: proc(#by_ptr point: PxVec3, #by_ptr radii: PxVec3) -> PxVec3 ---

    /// Compute from an input quaternion q a pair of quaternions (swing, twist) such that
    /// q = swing * twist
    /// with the caveats that swing.x = twist.y = twist.z = 0.
    @(link_name = "phys_PxSeparateSwingTwist")
    separate_swing_twist :: proc(#by_ptr q: PxQuat, swing: ^PxQuat, twist: ^PxQuat) ---

    /// Compute the angle between two non-unit vectors
    ///
    /// Returns the angle (in radians) between the two vector v0 and v1.
    @(link_name = "phys_PxComputeAngle")
    compute_angle :: proc(#by_ptr v0: PxVec3, #by_ptr v1: PxVec3) -> _c.float ---

    /// Compute two normalized vectors (right and up) that are perpendicular to an input normalized vector (dir).
    @(link_name = "phys_PxComputeBasisVectors")
    compute_basis_vectors :: proc(#by_ptr dir: PxVec3, right: ^PxVec3, up: ^PxVec3) ---

    /// Compute three normalized vectors (dir, right and up) that are parallel to (dir) and perpendicular to (right, up) the
    /// normalized direction vector (p1 - p0)/||p1 - p0||.
    @(link_name = "phys_PxComputeBasisVectors_1")
    compute_basis_vectors_1 :: proc(#by_ptr p0: PxVec3, #by_ptr p1: PxVec3, dir: ^PxVec3, right: ^PxVec3, up: ^PxVec3) ---

    /// Compute (i+1)%3
    @(link_name = "phys_PxGetNextIndex3")
    get_next_index3 :: proc(i: _c.uint32_t) -> _c.uint32_t ---

    @(link_name = "phys_computeBarycentric")
    compute_barycentric :: proc(#by_ptr a: PxVec3, #by_ptr b: PxVec3, #by_ptr c: PxVec3, #by_ptr d: PxVec3, #by_ptr p: PxVec3, bary: ^PxVec4) ---

    @(link_name = "phys_computeBarycentric_1")
    compute_barycentric_1 :: proc(#by_ptr a: PxVec3, #by_ptr b: PxVec3, #by_ptr c: PxVec3, #by_ptr p: PxVec3, bary: ^PxVec4) ---

    @(link_name = "Interpolation_PxLerp")
    lerp :: proc(a: _c.float, b: _c.float, t: _c.float) -> _c.float ---

    @(link_name = "Interpolation_PxBiLerp")
    bi_lerp :: proc(f00: _c.float, f10: _c.float, f01: _c.float, f11: _c.float, tx: _c.float, ty: _c.float) -> _c.float ---

    @(link_name = "Interpolation_PxTriLerp")
    tri_lerp :: proc(f000: _c.float, f100: _c.float, f010: _c.float, f110: _c.float, f001: _c.float, f101: _c.float, f011: _c.float, f111: _c.float, tx: _c.float, ty: _c.float, tz: _c.float) -> _c.float ---

    @(link_name = "Interpolation_PxSDFIdx")
    s_d_f_idx :: proc(i: _c.uint32_t, j: _c.uint32_t, k: _c.uint32_t, nbX: _c.uint32_t, nbY: _c.uint32_t) -> _c.uint32_t ---

    @(link_name = "Interpolation_PxSDFSampleImpl")
    s_d_f_sample_impl :: proc(sdf: ^_c.float, #by_ptr localPos: PxVec3, #by_ptr sdfBoxLower: PxVec3, #by_ptr sdfBoxHigher: PxVec3, sdfDx: _c.float, invSdfDx: _c.float, dimX: _c.uint32_t, dimY: _c.uint32_t, dimZ: _c.uint32_t, tolerance: _c.float) -> _c.float ---

    @(link_name = "phys_PxSdfSample")
    sdf_sample :: proc(sdf: ^_c.float, #by_ptr localPos: PxVec3, #by_ptr sdfBoxLower: PxVec3, #by_ptr sdfBoxHigher: PxVec3, sdfDx: _c.float, invSdfDx: _c.float, dimX: _c.uint32_t, dimY: _c.uint32_t, dimZ: _c.uint32_t, gradient: ^PxVec3, tolerance: _c.float) -> _c.float ---

    /// The constructor for Mutex creates a mutex. It is initially unlocked.
    @(link_name = "PxMutexImpl_new_alloc")
    mutex_impl_new_alloc :: proc() -> ^PxMutexImpl ---

    /// The destructor for Mutex deletes the mutex.
    @(link_name = "PxMutexImpl_delete")
    mutex_impl_delete :: proc(self_: ^PxMutexImpl) ---

    /// Acquire (lock) the mutex. If the mutex is already locked
    /// by another thread, this method blocks until the mutex is
    /// unlocked.
    @(link_name = "PxMutexImpl_lock_mut")
    mutex_impl_lock_mut :: proc(self_: ^PxMutexImpl) ---

    /// Acquire (lock) the mutex. If the mutex is already locked
    /// by another thread, this method returns false without blocking.
    @(link_name = "PxMutexImpl_trylock_mut")
    mutex_impl_trylock_mut :: proc(self_: ^PxMutexImpl) -> _c.bool ---

    /// Release (unlock) the mutex.
    @(link_name = "PxMutexImpl_unlock_mut")
    mutex_impl_unlock_mut :: proc(self_: ^PxMutexImpl) ---

    /// Size of this class.
    @(link_name = "PxMutexImpl_getSize")
    mutex_impl_get_size :: proc() -> _c.uint32_t ---

    @(link_name = "PxReadWriteLock_new_alloc")
    read_write_lock_new_alloc :: proc() -> ^PxReadWriteLock ---

    @(link_name = "PxReadWriteLock_delete")
    read_write_lock_delete :: proc(self_: ^PxReadWriteLock) ---

    @(link_name = "PxReadWriteLock_lockReader_mut")
    read_write_lock_lock_reader_mut :: proc(self_: ^PxReadWriteLock, takeLock: _c.bool) ---

    @(link_name = "PxReadWriteLock_lockWriter_mut")
    read_write_lock_lock_writer_mut :: proc(self_: ^PxReadWriteLock) ---

    @(link_name = "PxReadWriteLock_unlockReader_mut")
    read_write_lock_unlock_reader_mut :: proc(self_: ^PxReadWriteLock) ---

    @(link_name = "PxReadWriteLock_unlockWriter_mut")
    read_write_lock_unlock_writer_mut :: proc(self_: ^PxReadWriteLock) ---

    /// Mark the beginning of a nested profile block
    ///
    /// Returns implementation-specific profiler data for this event
    @(link_name = "PxProfilerCallback_zoneStart_mut")
    profiler_callback_zone_start_mut :: proc(self_: ^PxProfilerCallback, eventName: ^_c.char, detached: _c.bool, contextId: _c.uint64_t) -> rawptr ---

    /// Mark the end of a nested profile block
    ///
    /// eventName plus contextId can be used to uniquely match up start and end of a zone.
    @(link_name = "PxProfilerCallback_zoneEnd_mut")
    profiler_callback_zone_end_mut :: proc(self_: ^PxProfilerCallback, profilerData: rawptr, eventName: ^_c.char, detached: _c.bool, contextId: _c.uint64_t) ---

    @(link_name = "PxProfileScoped_new_alloc")
    profile_scoped_new_alloc :: proc(callback: ^PxProfilerCallback, eventName: ^_c.char, detached: _c.bool, contextId: _c.uint64_t) -> ^PxProfileScoped ---

    @(link_name = "PxProfileScoped_delete")
    profile_scoped_delete :: proc(self_: ^PxProfileScoped) ---

    @(link_name = "PxSListEntry_new")
    s_list_entry_new :: proc() -> PxSListEntry ---

    @(link_name = "PxSListEntry_next_mut")
    s_list_entry_next_mut :: proc(self_: ^PxSListEntry) -> ^PxSListEntry ---

    @(link_name = "PxSListImpl_new_alloc")
    s_list_impl_new_alloc :: proc() -> ^PxSListImpl ---

    @(link_name = "PxSListImpl_delete")
    s_list_impl_delete :: proc(self_: ^PxSListImpl) ---

    @(link_name = "PxSListImpl_push_mut")
    s_list_impl_push_mut :: proc(self_: ^PxSListImpl, entry: ^PxSListEntry) ---

    @(link_name = "PxSListImpl_pop_mut")
    s_list_impl_pop_mut :: proc(self_: ^PxSListImpl) -> ^PxSListEntry ---

    @(link_name = "PxSListImpl_flush_mut")
    s_list_impl_flush_mut :: proc(self_: ^PxSListImpl) -> ^PxSListEntry ---

    @(link_name = "PxSListImpl_getSize")
    s_list_impl_get_size :: proc() -> _c.uint32_t ---

    @(link_name = "PxSyncImpl_new_alloc")
    sync_impl_new_alloc :: proc() -> ^PxSyncImpl ---

    @(link_name = "PxSyncImpl_delete")
    sync_impl_delete :: proc(self_: ^PxSyncImpl) ---

    /// Wait on the object for at most the given number of ms. Returns
    /// true if the object is signaled. Sync::waitForever will block forever
    /// or until the object is signaled.
    @(link_name = "PxSyncImpl_wait_mut")
    sync_impl_wait_mut :: proc(self_: ^PxSyncImpl, milliseconds: _c.uint32_t) -> _c.bool ---

    /// Signal the synchronization object, waking all threads waiting on it
    @(link_name = "PxSyncImpl_set_mut")
    sync_impl_set_mut :: proc(self_: ^PxSyncImpl) ---

    /// Reset the synchronization object
    @(link_name = "PxSyncImpl_reset_mut")
    sync_impl_reset_mut :: proc(self_: ^PxSyncImpl) ---

    /// Size of this class.
    @(link_name = "PxSyncImpl_getSize")
    sync_impl_get_size :: proc() -> _c.uint32_t ---

    @(link_name = "PxRunnable_new_alloc")
    runnable_new_alloc :: proc() -> ^PxRunnable ---

    @(link_name = "PxRunnable_delete")
    runnable_delete :: proc(self_: ^PxRunnable) ---

    @(link_name = "PxRunnable_execute_mut")
    runnable_execute_mut :: proc(self_: ^PxRunnable) ---

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
    counter_frequency_to_tens_of_nanos_new :: proc(inNum: _c.uint64_t, inDenom: _c.uint64_t) -> PxCounterFrequencyToTensOfNanos ---

    @(link_name = "PxCounterFrequencyToTensOfNanos_toTensOfNanos")
    counter_frequency_to_tens_of_nanos_to_tens_of_nanos :: proc(self_: ^PxCounterFrequencyToTensOfNanos, inCounter: _c.uint64_t) -> _c.uint64_t ---

    @(link_name = "PxTime_getBootCounterFrequency")
    time_get_boot_counter_frequency :: proc() -> ^PxCounterFrequencyToTensOfNanos ---

    @(link_name = "PxTime_getCounterFrequency")
    time_get_counter_frequency :: proc() -> PxCounterFrequencyToTensOfNanos ---

    @(link_name = "PxTime_getCurrentCounterValue")
    time_get_current_counter_value :: proc() -> _c.uint64_t ---

    @(link_name = "PxTime_getCurrentTimeInTensOfNanoSeconds")
    time_get_current_time_in_tens_of_nano_seconds :: proc() -> _c.uint64_t ---

    @(link_name = "PxTime_new")
    time_new :: proc() -> PxTime ---

    @(link_name = "PxTime_getElapsedSeconds_mut")
    time_get_elapsed_seconds_mut :: proc(self_: ^PxTime) -> _c.double ---

    @(link_name = "PxTime_peekElapsedSeconds_mut")
    time_peek_elapsed_seconds_mut :: proc(self_: ^PxTime) -> _c.double ---

    @(link_name = "PxTime_getLastTime")
    time_get_last_time :: proc(self_: ^PxTime) -> _c.double ---

    /// default constructor leaves data uninitialized.
    @(link_name = "PxVec2_new")
    vec2_new :: proc() -> PxVec2 ---

    /// zero constructor.
    @(link_name = "PxVec2_new_1")
    vec2_new_1 :: proc(anon_param0: PxZERO) -> PxVec2 ---

    /// Assigns scalar parameter to all elements.
    ///
    /// Useful to initialize to zero or one.
    @(link_name = "PxVec2_new_2")
    vec2_new_2 :: proc(a: _c.float) -> PxVec2 ---

    /// Initializes from 2 scalar parameters.
    @(link_name = "PxVec2_new_3")
    vec2_new_3 :: proc(nx: _c.float, ny: _c.float) -> PxVec2 ---

    /// tests for exact zero vector
    @(link_name = "PxVec2_isZero")
    vec2_is_zero :: proc(self_: ^PxVec2) -> _c.bool ---

    /// returns true if all 2 elems of the vector are finite (not NAN or INF, etc.)
    @(link_name = "PxVec2_isFinite")
    vec2_is_finite :: proc(self_: ^PxVec2) -> _c.bool ---

    /// is normalized - used by API parameter validation
    @(link_name = "PxVec2_isNormalized")
    vec2_is_normalized :: proc(self_: ^PxVec2) -> _c.bool ---

    /// returns the squared magnitude
    ///
    /// Avoids calling PxSqrt()!
    @(link_name = "PxVec2_magnitudeSquared")
    vec2_magnitude_squared :: proc(self_: ^PxVec2) -> _c.float ---

    /// returns the magnitude
    @(link_name = "PxVec2_magnitude")
    vec2_magnitude :: proc(self_: ^PxVec2) -> _c.float ---

    /// returns the scalar product of this and other.
    @(link_name = "PxVec2_dot")
    vec2_dot :: proc(self_: ^PxVec2, #by_ptr v: PxVec2) -> _c.float ---

    /// returns a unit vector
    @(link_name = "PxVec2_getNormalized")
    vec2_get_normalized :: proc(self_: ^PxVec2) -> PxVec2 ---

    /// normalizes the vector in place
    @(link_name = "PxVec2_normalize_mut")
    vec2_normalize_mut :: proc(self_: ^PxVec2) -> _c.float ---

    /// a[i] * b[i], for all i.
    @(link_name = "PxVec2_multiply")
    vec2_multiply :: proc(self_: ^PxVec2, #by_ptr a: PxVec2) -> PxVec2 ---

    /// element-wise minimum
    @(link_name = "PxVec2_minimum")
    vec2_minimum :: proc(self_: ^PxVec2, #by_ptr v: PxVec2) -> PxVec2 ---

    /// returns MIN(x, y);
    @(link_name = "PxVec2_minElement")
    vec2_min_element :: proc(self_: ^PxVec2) -> _c.float ---

    /// element-wise maximum
    @(link_name = "PxVec2_maximum")
    vec2_maximum :: proc(self_: ^PxVec2, #by_ptr v: PxVec2) -> PxVec2 ---

    /// returns MAX(x, y);
    @(link_name = "PxVec2_maxElement")
    vec2_max_element :: proc(self_: ^PxVec2) -> _c.float ---

    @(link_name = "PxStridedData_new")
    strided_data_new :: proc() -> PxStridedData ---

    @(link_name = "PxBoundedData_new")
    bounded_data_new :: proc() -> PxBoundedData ---

    @(link_name = "PxDebugPoint_new")
    debug_point_new :: proc(#by_ptr p: PxVec3, #by_ptr c: _c.uint32_t) -> PxDebugPoint ---

    @(link_name = "PxDebugLine_new")
    debug_line_new :: proc(#by_ptr p0: PxVec3, #by_ptr p1: PxVec3, #by_ptr c: _c.uint32_t) -> PxDebugLine ---

    @(link_name = "PxDebugTriangle_new")
    debug_triangle_new :: proc(#by_ptr p0: PxVec3, #by_ptr p1: PxVec3, #by_ptr p2: PxVec3, #by_ptr c: _c.uint32_t) -> PxDebugTriangle ---

    @(link_name = "PxDebugText_new")
    debug_text_new :: proc() -> PxDebugText ---

    @(link_name = "PxDebugText_new_1")
    debug_text_new_1 :: proc(#by_ptr pos: PxVec3, #by_ptr sz: _c.float, #by_ptr clr: _c.uint32_t, str: ^_c.char) -> PxDebugText ---

    @(link_name = "PxRenderBuffer_delete")
    render_buffer_delete :: proc(self_: ^PxRenderBuffer) ---

    @(link_name = "PxRenderBuffer_getNbPoints")
    render_buffer_get_nb_points :: proc(self_: ^PxRenderBuffer) -> _c.uint32_t ---

    @(link_name = "PxRenderBuffer_getPoints")
    render_buffer_get_points :: proc(self_: ^PxRenderBuffer) -> ^PxDebugPoint ---

    @(link_name = "PxRenderBuffer_addPoint_mut")
    render_buffer_add_point_mut :: proc(self_: ^PxRenderBuffer, #by_ptr point: PxDebugPoint) ---

    @(link_name = "PxRenderBuffer_getNbLines")
    render_buffer_get_nb_lines :: proc(self_: ^PxRenderBuffer) -> _c.uint32_t ---

    @(link_name = "PxRenderBuffer_getLines")
    render_buffer_get_lines :: proc(self_: ^PxRenderBuffer) -> ^PxDebugLine ---

    @(link_name = "PxRenderBuffer_addLine_mut")
    render_buffer_add_line_mut :: proc(self_: ^PxRenderBuffer, #by_ptr line: PxDebugLine) ---

    @(link_name = "PxRenderBuffer_reserveLines_mut")
    render_buffer_reserve_lines_mut :: proc(self_: ^PxRenderBuffer, nbLines: _c.uint32_t) -> ^PxDebugLine ---

    @(link_name = "PxRenderBuffer_reservePoints_mut")
    render_buffer_reserve_points_mut :: proc(self_: ^PxRenderBuffer, nbLines: _c.uint32_t) -> ^PxDebugPoint ---

    @(link_name = "PxRenderBuffer_getNbTriangles")
    render_buffer_get_nb_triangles :: proc(self_: ^PxRenderBuffer) -> _c.uint32_t ---

    @(link_name = "PxRenderBuffer_getTriangles")
    render_buffer_get_triangles :: proc(self_: ^PxRenderBuffer) -> ^PxDebugTriangle ---

    @(link_name = "PxRenderBuffer_addTriangle_mut")
    render_buffer_add_triangle_mut :: proc(self_: ^PxRenderBuffer, #by_ptr triangle: PxDebugTriangle) ---

    @(link_name = "PxRenderBuffer_append_mut")
    render_buffer_append_mut :: proc(self_: ^PxRenderBuffer, #by_ptr other: PxRenderBuffer) ---

    @(link_name = "PxRenderBuffer_clear_mut")
    render_buffer_clear_mut :: proc(self_: ^PxRenderBuffer) ---

    @(link_name = "PxRenderBuffer_shift_mut")
    render_buffer_shift_mut :: proc(self_: ^PxRenderBuffer, #by_ptr delta: PxVec3) ---

    @(link_name = "PxRenderBuffer_empty")
    render_buffer_empty :: proc(self_: ^PxRenderBuffer) -> _c.bool ---

    @(link_name = "PxProcessPxBaseCallback_delete")
    process_px_base_callback_delete :: proc(self_: ^PxProcessPxBaseCallback) ---

    @(link_name = "PxProcessPxBaseCallback_process_mut")
    process_px_base_callback_process_mut :: proc(self_: ^PxProcessPxBaseCallback, anon_param0: ^PxBase) ---

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
    serialization_context_register_reference_mut :: proc(self_: ^PxSerializationContext, base: ^PxBase, kind: _c.uint32_t, reference: _c.size_t) ---

    /// Returns the collection that is being serialized.
    @(link_name = "PxSerializationContext_getCollection")
    serialization_context_get_collection :: proc(self_: ^PxSerializationContext) -> ^PxCollection ---

    /// Serializes object data and object extra data.
    ///
    /// This function is assumed to be called within the implementation of PxSerializer::exportData and PxSerializer::exportExtraData.
    @(link_name = "PxSerializationContext_writeData_mut")
    serialization_context_write_data_mut :: proc(self_: ^PxSerializationContext, data: rawptr, size: _c.uint32_t) ---

    /// Aligns the serialized data.
    ///
    /// This function is assumed to be called within the implementation of PxSerializer::exportData and PxSerializer::exportExtraData.
    @(link_name = "PxSerializationContext_alignData_mut")
    serialization_context_align_data_mut :: proc(self_: ^PxSerializationContext, alignment: _c.uint32_t) ---

    /// Helper function to write a name to the extraData if serialization is configured to save names.
    ///
    /// This function is assumed to be called within the implementation of PxSerializer::exportExtraData.
    @(link_name = "PxSerializationContext_writeName_mut")
    serialization_context_write_name_mut :: proc(self_: ^PxSerializationContext, name: ^_c.char) ---

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
    deserialization_context_resolve_reference :: proc(self_: ^PxDeserializationContext, kind: _c.uint32_t, reference: _c.size_t) -> ^PxBase ---

    /// Helper function to read a name from the extra data during deserialization.
    ///
    /// This function is assumed to be called within the implementation of PxSerializer::createObject.
    @(link_name = "PxDeserializationContext_readName_mut")
    deserialization_context_read_name_mut :: proc(self_: ^PxDeserializationContext, name: ^^_c.char) ---

    /// Function to align the extra data stream to a power of 2 alignment
    ///
    /// This function is assumed to be called within the implementation of PxSerializer::createObject.
    @(link_name = "PxDeserializationContext_alignExtraData_mut")
    deserialization_context_align_extra_data_mut :: proc(self_: ^PxDeserializationContext, alignment: _c.uint32_t) ---

    /// Register a serializer for a concrete type
    @(link_name = "PxSerializationRegistry_registerSerializer_mut")
    serialization_registry_register_serializer_mut :: proc(self_: ^PxSerializationRegistry, type: _c.uint16_t, serializer: ^PxSerializer) ---

    /// Unregister a serializer for a concrete type, and retrieves the corresponding serializer object.
    ///
    /// Unregistered serializer corresponding to type, NULL for types for which no serializer has been registered.
    @(link_name = "PxSerializationRegistry_unregisterSerializer_mut")
    serialization_registry_unregister_serializer_mut :: proc(self_: ^PxSerializationRegistry, type: _c.uint16_t) -> ^PxSerializer ---

    /// Returns PxSerializer corresponding to type
    ///
    /// Registered PxSerializer object corresponding to type
    @(link_name = "PxSerializationRegistry_getSerializer")
    serialization_registry_get_serializer :: proc(self_: ^PxSerializationRegistry, type: _c.uint16_t) -> ^PxSerializer ---

    /// Register a RepX serializer for a concrete type
    @(link_name = "PxSerializationRegistry_registerRepXSerializer_mut")
    serialization_registry_register_rep_x_serializer_mut :: proc(self_: ^PxSerializationRegistry, type: _c.uint16_t, serializer: ^PxRepXSerializer) ---

    /// Unregister a RepX serializer for a concrete type, and retrieves the corresponding serializer object.
    ///
    /// Unregistered PxRepxSerializer corresponding to type, NULL for types for which no RepX serializer has been registered.
    @(link_name = "PxSerializationRegistry_unregisterRepXSerializer_mut")
    serialization_registry_unregister_rep_x_serializer_mut :: proc(self_: ^PxSerializationRegistry, type: _c.uint16_t) -> ^PxRepXSerializer ---

    /// Returns RepX serializer given the corresponding type name
    ///
    /// Registered PxRepXSerializer object corresponding to type name
    @(link_name = "PxSerializationRegistry_getRepXSerializer")
    serialization_registry_get_rep_x_serializer :: proc(self_: ^PxSerializationRegistry, typeName: ^_c.char) -> ^PxRepXSerializer ---

    /// Releases PxSerializationRegistry instance.
    ///
    /// This unregisters all PhysX and PhysXExtension serializers. Make sure to unregister all custom type
    /// serializers before releasing the PxSerializationRegistry.
    @(link_name = "PxSerializationRegistry_release_mut")
    serialization_registry_release_mut :: proc(self_: ^PxSerializationRegistry) ---

    /// Adds a PxBase object to the collection.
    ///
    /// Adds a PxBase object to the collection. Optionally a PxSerialObjectId can be provided
    /// in order to resolve dependencies between collections. A PxSerialObjectId value of PX_SERIAL_OBJECT_ID_INVALID
    /// means the object remains without id. Objects can be added regardless of other objects they require. If the object
    /// is already in the collection, the ID will be set if it was PX_SERIAL_OBJECT_ID_INVALID previously, otherwise the
    /// operation fails.
    @(link_name = "PxCollection_add_mut")
    collection_add_mut :: proc(self_: ^PxCollection, object: ^PxBase, id: _c.uint64_t) ---

    /// Removes a PxBase member object from the collection.
    ///
    /// Object needs to be contained by the collection.
    @(link_name = "PxCollection_remove_mut")
    collection_remove_mut :: proc(self_: ^PxCollection, object: ^PxBase) ---

    /// Returns whether the collection contains a certain PxBase object.
    ///
    /// Whether object is contained.
    @(link_name = "PxCollection_contains")
    collection_contains :: proc(self_: ^PxCollection, object: ^PxBase) -> _c.bool ---

    /// Adds an id to a member PxBase object.
    ///
    /// If the object is already associated with an id within the collection, the id is replaced.
    /// May only be called for objects that are members of the collection. The id needs to be unique
    /// within the collection.
    @(link_name = "PxCollection_addId_mut")
    collection_add_id_mut :: proc(self_: ^PxCollection, object: ^PxBase, id: _c.uint64_t) ---

    /// Removes id from a contained PxBase object.
    ///
    /// May only be called for ids that are associated with an object in the collection.
    @(link_name = "PxCollection_removeId_mut")
    collection_remove_id_mut :: proc(self_: ^PxCollection, id: _c.uint64_t) ---

    /// Adds all PxBase objects and their ids of collection to this collection.
    ///
    /// PxBase objects already in this collection are ignored. Object ids need to be conflict
    /// free, i.e. the same object may not have two different ids within the two collections.
    @(link_name = "PxCollection_add_mut_1")
    collection_add_mut_1 :: proc(self_: ^PxCollection, collection: ^PxCollection) ---

    /// Removes all PxBase objects of collection from this collection.
    ///
    /// PxBase objects not present in this collection are ignored. Ids of objects
    /// which are removed are also removed.
    @(link_name = "PxCollection_remove_mut_1")
    collection_remove_mut_1 :: proc(self_: ^PxCollection, collection: ^PxCollection) ---

    /// Gets number of PxBase objects in this collection.
    ///
    /// Number of objects in this collection
    @(link_name = "PxCollection_getNbObjects")
    collection_get_nb_objects :: proc(self_: ^PxCollection) -> _c.uint32_t ---

    /// Gets the PxBase object of this collection given its index.
    ///
    /// PxBase object at index index
    @(link_name = "PxCollection_getObject")
    collection_get_object :: proc(self_: ^PxCollection, index: _c.uint32_t) -> ^PxBase ---

    /// Copies member PxBase pointers to a user specified buffer.
    ///
    /// number of members PxBase objects that have been written to the userBuffer
    @(link_name = "PxCollection_getObjects")
    collection_get_objects :: proc(self_: ^PxCollection, userBuffer: ^^PxBase, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Looks for a PxBase object given a PxSerialObjectId value.
    ///
    /// If there is no PxBase object in the collection with the given id, NULL is returned.
    ///
    /// PxBase object with the given id value or NULL
    @(link_name = "PxCollection_find")
    collection_find :: proc(self_: ^PxCollection, id: _c.uint64_t) -> ^PxBase ---

    /// Gets number of PxSerialObjectId names in this collection.
    ///
    /// Number of PxSerialObjectId names in this collection
    @(link_name = "PxCollection_getNbIds")
    collection_get_nb_ids :: proc(self_: ^PxCollection) -> _c.uint32_t ---

    /// Copies member PxSerialObjectId values to a user specified buffer.
    ///
    /// number of members PxSerialObjectId values that have been written to the userBuffer
    @(link_name = "PxCollection_getIds")
    collection_get_ids :: proc(self_: ^PxCollection, userBuffer: ^_c.uint64_t, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Gets the PxSerialObjectId name of a PxBase object within the collection.
    ///
    /// The PxBase object needs to be a member of the collection.
    ///
    /// PxSerialObjectId name of the object or PX_SERIAL_OBJECT_ID_INVALID if the object is unnamed
    @(link_name = "PxCollection_getId")
    collection_get_id :: proc(self_: ^PxCollection, object: ^PxBase) -> _c.uint64_t ---

    /// Deletes a collection object.
    ///
    /// This function only deletes the collection object, i.e. the container class. It doesn't delete objects
    /// that are part of the collection.
    @(link_name = "PxCollection_release_mut")
    collection_release_mut :: proc(self_: ^PxCollection) ---

    /// Creates a collection object.
    ///
    /// Objects can only be serialized or deserialized through a collection.
    /// For serialization, users must add objects to the collection and serialize the collection as a whole.
    /// For deserialization, the system gives back a collection of deserialized objects to users.
    ///
    /// The new collection object.
    @(link_name = "phys_PxCreateCollection")
    create_collection :: proc() -> ^PxCollection ---

    /// Releases the PxBase instance, please check documentation of release in derived class.
    @(link_name = "PxBase_release_mut")
    base_release_mut :: proc(self_: ^PxBase) ---

    /// Returns string name of dynamic type.
    ///
    /// Class name of most derived type of this object.
    @(link_name = "PxBase_getConcreteTypeName")
    base_get_concrete_type_name :: proc(self_: ^PxBase) -> ^_c.char ---

    /// Returns concrete type of object.
    ///
    /// PxConcreteType::Enum of serialized object
    @(link_name = "PxBase_getConcreteType")
    base_get_concrete_type :: proc(self_: ^PxBase) -> _c.uint16_t ---

    /// Set PxBaseFlag
    @(link_name = "PxBase_setBaseFlag_mut")
    base_set_base_flag_mut :: proc(self_: ^PxBase, flag: PxBaseFlag, value: _c.bool) ---

    /// Set PxBaseFlags
    @(link_name = "PxBase_setBaseFlags_mut")
    base_set_base_flags_mut :: proc(self_: ^PxBase, inFlags: PxBaseFlags_Set) ---

    /// Returns PxBaseFlags
    ///
    /// PxBaseFlags
    @(link_name = "PxBase_getBaseFlags")
    base_get_base_flags :: proc(self_: ^PxBase) -> PxBaseFlags_Set ---

    /// Whether the object is subordinate.
    ///
    /// A class is subordinate, if it can only be instantiated in the context of another class.
    ///
    /// Whether the class is subordinate
    @(link_name = "PxBase_isReleasable")
    base_is_releasable :: proc(self_: ^PxBase) -> _c.bool ---

    /// Decrements the reference count of the object and releases it if the new reference count is zero.
    @(link_name = "PxRefCounted_release_mut")
    ref_counted_release_mut :: proc(self_: ^PxRefCounted) ---

    /// Returns the reference count of the object.
    ///
    /// At creation, the reference count of the object is 1. Every other object referencing this object increments the
    /// count by 1. When the reference count reaches 0, and only then, the object gets destroyed automatically.
    ///
    /// the current reference count.
    @(link_name = "PxRefCounted_getReferenceCount")
    ref_counted_get_reference_count :: proc(self_: ^PxRefCounted) -> _c.uint32_t ---

    /// Acquires a counted reference to this object.
    ///
    /// This method increases the reference count of the object by 1. Decrement the reference count by calling release()
    @(link_name = "PxRefCounted_acquireReference_mut")
    ref_counted_acquire_reference_mut :: proc(self_: ^PxRefCounted) ---

    /// constructor sets to default
    @(link_name = "PxTolerancesScale_new")
    tolerances_scale_new :: proc(defaultLength: _c.float, defaultSpeed: _c.float) -> PxTolerancesScale ---

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid (returns always true).
    @(link_name = "PxTolerancesScale_isValid")
    tolerances_scale_is_valid :: proc(self_: ^PxTolerancesScale) -> _c.bool ---

    /// Allocate a new string.
    ///
    /// *Always* a valid null terminated string.  "" is returned if "" or null is passed in.
    @(link_name = "PxStringTable_allocateStr_mut")
    string_table_allocate_str_mut :: proc(self_: ^PxStringTable, inSrc: ^_c.char) -> ^_c.char ---

    /// Release the string table and all the strings associated with it.
    @(link_name = "PxStringTable_release_mut")
    string_table_release_mut :: proc(self_: ^PxStringTable) ---

    /// Returns string name of dynamic type.
    ///
    /// Class name of most derived type of this object.
    @(link_name = "PxSerializer_getConcreteTypeName")
    serializer_get_concrete_type_name :: proc(self_: ^PxSerializer) -> ^_c.char ---

    /// Adds required objects to the collection.
    ///
    /// This method does not add the required objects recursively, e.g. objects required by required objects.
    @(link_name = "PxSerializer_requiresObjects")
    serializer_requires_objects :: proc(self_: ^PxSerializer, anon_param0: ^PxBase, anon_param1: ^PxProcessPxBaseCallback) ---

    /// Whether the object is subordinate.
    ///
    /// A class is subordinate, if it can only be instantiated in the context of another class.
    ///
    /// Whether the class is subordinate
    @(link_name = "PxSerializer_isSubordinate")
    serializer_is_subordinate :: proc(self_: ^PxSerializer) -> _c.bool ---

    /// Exports object's extra data to stream.
    @(link_name = "PxSerializer_exportExtraData")
    serializer_export_extra_data :: proc(self_: ^PxSerializer, anon_param0: ^PxBase, anon_param1: ^PxSerializationContext) ---

    /// Exports object's data to stream.
    @(link_name = "PxSerializer_exportData")
    serializer_export_data :: proc(self_: ^PxSerializer, anon_param0: ^PxBase, anon_param1: ^PxSerializationContext) ---

    /// Register references that the object maintains to other objects.
    @(link_name = "PxSerializer_registerReferences")
    serializer_register_references :: proc(self_: ^PxSerializer, obj: ^PxBase, s: ^PxSerializationContext) ---

    /// Returns size needed to create the class instance.
    ///
    /// sizeof class instance.
    @(link_name = "PxSerializer_getClassSize")
    serializer_get_class_size :: proc(self_: ^PxSerializer) -> _c.size_t ---

    /// Create object at a given address, resolve references and import extra data.
    ///
    /// Created PxBase pointer (needs to be identical to address before increment).
    @(link_name = "PxSerializer_createObject")
    serializer_create_object :: proc(self_: ^PxSerializer, address: ^^_c.uint8_t, context_: ^PxDeserializationContext) -> ^PxBase ---

    /// *******************************************************************************************************************
    @(link_name = "PxSerializer_delete")
    serializer_delete :: proc(self_: ^PxSerializer) ---

    /// Builds object (TriangleMesh, Heightfield, ConvexMesh or BVH) from given data in PxPhysics.
    ///
    /// PxBase Created object in PxPhysics.
    @(link_name = "PxInsertionCallback_buildObjectFromData_mut")
    insertion_callback_build_object_from_data_mut :: proc(self_: ^PxInsertionCallback, type: PxConcreteType, data: rawptr) -> ^PxBase ---

    /// Set the user-provided dispatcher object for CPU tasks
    @(link_name = "PxTaskManager_setCpuDispatcher_mut")
    task_manager_set_cpu_dispatcher_mut :: proc(self_: ^PxTaskManager, ref: ^PxCpuDispatcher) ---

    /// Get the user-provided dispatcher object for CPU tasks
    ///
    /// The CPU dispatcher object.
    @(link_name = "PxTaskManager_getCpuDispatcher")
    task_manager_get_cpu_dispatcher :: proc(self_: ^PxTaskManager) -> ^PxCpuDispatcher ---

    /// Reset any dependencies between Tasks
    ///
    /// Will be called at the start of every frame before tasks are submitted.
    @(link_name = "PxTaskManager_resetDependencies_mut")
    task_manager_reset_dependencies_mut :: proc(self_: ^PxTaskManager) ---

    /// Called by the owning scene to start the task graph.
    ///
    /// All tasks with ref count of 1 will be dispatched.
    @(link_name = "PxTaskManager_startSimulation_mut")
    task_manager_start_simulation_mut :: proc(self_: ^PxTaskManager) ---

    /// Called by the owning scene at the end of a simulation step.
    @(link_name = "PxTaskManager_stopSimulation_mut")
    task_manager_stop_simulation_mut :: proc(self_: ^PxTaskManager) ---

    /// Called by the worker threads to inform the PxTaskManager that a task has completed processing.
    @(link_name = "PxTaskManager_taskCompleted_mut")
    task_manager_task_completed_mut :: proc(self_: ^PxTaskManager, task: ^PxTask) ---

    /// Retrieve a task by name
    ///
    /// The ID of the task with that name, or eNOT_PRESENT if not found
    @(link_name = "PxTaskManager_getNamedTask_mut")
    task_manager_get_named_task_mut :: proc(self_: ^PxTaskManager, name: ^_c.char) -> _c.uint32_t ---

    /// Submit a task with a unique name.
    ///
    /// The ID of the task with that name, or eNOT_PRESENT if not found
    @(link_name = "PxTaskManager_submitNamedTask_mut")
    task_manager_submit_named_task_mut :: proc(self_: ^PxTaskManager, task: ^PxTask, name: ^_c.char, type: PxTaskType) -> _c.uint32_t ---

    /// Submit an unnamed task.
    ///
    /// The ID of the task with that name, or eNOT_PRESENT if not found
    @(link_name = "PxTaskManager_submitUnnamedTask_mut")
    task_manager_submit_unnamed_task_mut :: proc(self_: ^PxTaskManager, task: ^PxTask, type: PxTaskType) -> _c.uint32_t ---

    /// Retrieve a task given a task ID
    ///
    /// The task associated with the ID
    @(link_name = "PxTaskManager_getTaskFromID_mut")
    task_manager_get_task_from_i_d_mut :: proc(self_: ^PxTaskManager, id: _c.uint32_t) -> ^PxTask ---

    /// Release the PxTaskManager object, referenced dispatchers will not be released
    @(link_name = "PxTaskManager_release_mut")
    task_manager_release_mut :: proc(self_: ^PxTaskManager) ---

    /// Construct a new PxTaskManager instance with the given [optional] dispatchers
    @(link_name = "PxTaskManager_createTaskManager")
    task_manager_create_task_manager :: proc(errorCallback: ^PxErrorCallback, anon_param1: ^PxCpuDispatcher) -> ^PxTaskManager ---

    /// Called by the TaskManager when a task is to be queued for execution.
    ///
    /// Upon receiving a task, the dispatcher should schedule the task to run.
    /// After the task has been run, it should call the release() method and
    /// discard its pointer.
    @(link_name = "PxCpuDispatcher_submitTask_mut")
    cpu_dispatcher_submit_task_mut :: proc(self_: ^PxCpuDispatcher, task: ^PxBaseTask) ---

    /// Returns the number of available worker threads for this dispatcher.
    ///
    /// The SDK will use this count to control how many tasks are submitted. By
    /// matching the number of tasks with the number of execution units task
    /// overhead can be reduced.
    @(link_name = "PxCpuDispatcher_getWorkerCount")
    cpu_dispatcher_get_worker_count :: proc(self_: ^PxCpuDispatcher) -> _c.uint32_t ---

    @(link_name = "PxCpuDispatcher_delete")
    cpu_dispatcher_delete :: proc(self_: ^PxCpuDispatcher) ---

    /// The user-implemented run method where the task's work should be performed
    ///
    /// run() methods must be thread safe, stack friendly (no alloca, etc), and
    /// must never block.
    @(link_name = "PxBaseTask_run_mut")
    base_task_run_mut :: proc(self_: ^PxBaseTask) ---

    /// Return a user-provided task name for profiling purposes.
    ///
    /// It does not have to be unique, but unique names are helpful.
    ///
    /// The name of this task
    @(link_name = "PxBaseTask_getName")
    base_task_get_name :: proc(self_: ^PxBaseTask) -> ^_c.char ---

    /// Implemented by derived implementation classes
    @(link_name = "PxBaseTask_addReference_mut")
    base_task_add_reference_mut :: proc(self_: ^PxBaseTask) ---

    /// Implemented by derived implementation classes
    @(link_name = "PxBaseTask_removeReference_mut")
    base_task_remove_reference_mut :: proc(self_: ^PxBaseTask) ---

    /// Implemented by derived implementation classes
    @(link_name = "PxBaseTask_getReference")
    base_task_get_reference :: proc(self_: ^PxBaseTask) -> _c.int32_t ---

    /// Implemented by derived implementation classes
    ///
    /// A task may assume in its release() method that the task system no longer holds
    /// references to it - so it may safely run its destructor, recycle itself, etc.
    /// provided no additional user references to the task exist
    @(link_name = "PxBaseTask_release_mut")
    base_task_release_mut :: proc(self_: ^PxBaseTask) ---

    /// Return PxTaskManager to which this task was submitted
    ///
    /// Note, can return NULL if task was not submitted, or has been
    /// completed.
    @(link_name = "PxBaseTask_getTaskManager")
    base_task_get_task_manager :: proc(self_: ^PxBaseTask) -> ^PxTaskManager ---

    @(link_name = "PxBaseTask_setContextId_mut")
    base_task_set_context_id_mut :: proc(self_: ^PxBaseTask, id: _c.uint64_t) ---

    @(link_name = "PxBaseTask_getContextId")
    base_task_get_context_id :: proc(self_: ^PxBaseTask) -> _c.uint64_t ---

    /// Release method implementation
    @(link_name = "PxTask_release_mut")
    task_release_mut :: proc(self_: ^PxTask) ---

    /// Inform the PxTaskManager this task must finish before the given
    @(link_name = "PxTask_finishBefore_mut")
    task_finish_before_mut :: proc(self_: ^PxTask, taskID: _c.uint32_t) ---

    /// Inform the PxTaskManager this task cannot start until the given
    @(link_name = "PxTask_startAfter_mut")
    task_start_after_mut :: proc(self_: ^PxTask, taskID: _c.uint32_t) ---

    /// Manually increment this task's reference count. The task will
    /// not be allowed to run until removeReference() is called.
    @(link_name = "PxTask_addReference_mut")
    task_add_reference_mut :: proc(self_: ^PxTask) ---

    /// Manually decrement this task's reference count. If the reference
    /// count reaches zero, the task will be dispatched.
    @(link_name = "PxTask_removeReference_mut")
    task_remove_reference_mut :: proc(self_: ^PxTask) ---

    /// Return the ref-count for this task
    @(link_name = "PxTask_getReference")
    task_get_reference :: proc(self_: ^PxTask) -> _c.int32_t ---

    /// Return the unique ID for this task
    @(link_name = "PxTask_getTaskID")
    task_get_task_i_d :: proc(self_: ^PxTask) -> _c.uint32_t ---

    /// Called by PxTaskManager at submission time for initialization
    ///
    /// Perform simulation step initialization here.
    @(link_name = "PxTask_submitted_mut")
    task_submitted_mut :: proc(self_: ^PxTask) ---

    /// Initialize this task and specify the task that will have its ref count decremented on completion.
    ///
    /// Submission is deferred until the task's mRefCount is decremented to zero.
    /// Note that we only use the PxTaskManager to query the appropriate dispatcher.
    @(link_name = "PxLightCpuTask_setContinuation_mut")
    light_cpu_task_set_continuation_mut :: proc(self_: ^PxLightCpuTask, tm: ^PxTaskManager, c: ^PxBaseTask) ---

    /// Initialize this task and specify the task that will have its ref count decremented on completion.
    ///
    /// This overload of setContinuation() queries the PxTaskManager from the continuation
    /// task, which cannot be NULL.
    @(link_name = "PxLightCpuTask_setContinuation_mut_1")
    light_cpu_task_set_continuation_mut_1 :: proc(self_: ^PxLightCpuTask, c: ^PxBaseTask) ---

    /// Retrieves continuation task
    @(link_name = "PxLightCpuTask_getContinuation")
    light_cpu_task_get_continuation :: proc(self_: ^PxLightCpuTask) -> ^PxBaseTask ---

    /// Manually decrement this task's reference count. If the reference
    /// count reaches zero, the task will be dispatched.
    @(link_name = "PxLightCpuTask_removeReference_mut")
    light_cpu_task_remove_reference_mut :: proc(self_: ^PxLightCpuTask) ---

    /// Return the ref-count for this task
    @(link_name = "PxLightCpuTask_getReference")
    light_cpu_task_get_reference :: proc(self_: ^PxLightCpuTask) -> _c.int32_t ---

    /// Manually increment this task's reference count. The task will
    /// not be allowed to run until removeReference() is called.
    @(link_name = "PxLightCpuTask_addReference_mut")
    light_cpu_task_add_reference_mut :: proc(self_: ^PxLightCpuTask) ---

    /// called by CpuDispatcher after run method has completed
    ///
    /// Decrements the continuation task's reference count, if specified.
    @(link_name = "PxLightCpuTask_release_mut")
    light_cpu_task_release_mut :: proc(self_: ^PxLightCpuTask) ---

    /// Returns the type of the geometry.
    ///
    /// The type of the object.
    @(link_name = "PxGeometry_getType")
    geometry_get_type :: proc(self_: ^PxGeometry) -> PxGeometryType ---

    /// Constructor to initialize half extents from scalar parameters.
    @(link_name = "PxBoxGeometry_new")
    box_geometry_new :: proc(hx: _c.float, hy: _c.float, hz: _c.float) -> PxBoxGeometry ---

    /// Constructor to initialize half extents from vector parameter.
    @(link_name = "PxBoxGeometry_new_1")
    box_geometry_new_1 :: proc(halfExtents_: PxVec3) -> PxBoxGeometry ---

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid
    ///
    /// A valid box has a positive extent in each direction (halfExtents.x > 0, halfExtents.y > 0, halfExtents.z > 0).
    /// It is illegal to call PxRigidActor::createShape and PxPhysics::createShape with a box that has zero extent in any direction.
    @(link_name = "PxBoxGeometry_isValid")
    box_geometry_is_valid :: proc(self_: ^PxBoxGeometry) -> _c.bool ---

    @(link_name = "PxBVHRaycastCallback_delete")
    b_v_h_raycast_callback_delete :: proc(self_: ^PxBVHRaycastCallback) ---

    @(link_name = "PxBVHRaycastCallback_reportHit_mut")
    b_v_h_raycast_callback_report_hit_mut :: proc(self_: ^PxBVHRaycastCallback, boundsIndex: _c.uint32_t, distance: ^_c.float) -> _c.bool ---

    @(link_name = "PxBVHOverlapCallback_delete")
    b_v_h_overlap_callback_delete :: proc(self_: ^PxBVHOverlapCallback) ---

    @(link_name = "PxBVHOverlapCallback_reportHit_mut")
    b_v_h_overlap_callback_report_hit_mut :: proc(self_: ^PxBVHOverlapCallback, boundsIndex: _c.uint32_t) -> _c.bool ---

    @(link_name = "PxBVHTraversalCallback_delete")
    b_v_h_traversal_callback_delete :: proc(self_: ^PxBVHTraversalCallback) ---

    @(link_name = "PxBVHTraversalCallback_visitNode_mut")
    b_v_h_traversal_callback_visit_node_mut :: proc(self_: ^PxBVHTraversalCallback, #by_ptr bounds: PxBounds3) -> _c.bool ---

    @(link_name = "PxBVHTraversalCallback_reportLeaf_mut")
    b_v_h_traversal_callback_report_leaf_mut :: proc(self_: ^PxBVHTraversalCallback, nbPrims: _c.uint32_t, prims: ^_c.uint32_t) -> _c.bool ---

    /// Raycast test against a BVH.
    ///
    /// false if query has been aborted
    @(link_name = "PxBVH_raycast")
    b_v_h_raycast :: proc(self_: ^PxBVH, #by_ptr origin: PxVec3, #by_ptr unitDir: PxVec3, maxDist: _c.float, cb: ^PxBVHRaycastCallback, queryFlags: PxGeometryQueryFlags_Set) -> _c.bool ---

    /// Sweep test against a BVH.
    ///
    /// false if query has been aborted
    @(link_name = "PxBVH_sweep")
    b_v_h_sweep :: proc(self_: ^PxBVH, geom: ^PxGeometry, #by_ptr pose: PxTransform, #by_ptr unitDir: PxVec3, maxDist: _c.float, cb: ^PxBVHRaycastCallback, queryFlags: PxGeometryQueryFlags_Set) -> _c.bool ---

    /// Overlap test against a BVH.
    ///
    /// false if query has been aborted
    @(link_name = "PxBVH_overlap")
    b_v_h_overlap :: proc(self_: ^PxBVH, geom: ^PxGeometry, #by_ptr pose: PxTransform, cb: ^PxBVHOverlapCallback, queryFlags: PxGeometryQueryFlags_Set) -> _c.bool ---

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
    b_v_h_cull :: proc(self_: ^PxBVH, nbPlanes: _c.uint32_t, planes: ^PxPlane, cb: ^PxBVHOverlapCallback, queryFlags: PxGeometryQueryFlags_Set) -> _c.bool ---

    /// Returns the number of bounds in the BVH.
    ///
    /// You can use [`getBounds`]() to retrieve the bounds.
    ///
    /// These are the user-defined bounds passed to the BVH builder, not the internal bounds around each BVH node.
    ///
    /// Number of bounds in the BVH.
    @(link_name = "PxBVH_getNbBounds")
    b_v_h_get_nb_bounds :: proc(self_: ^PxBVH) -> _c.uint32_t ---

    /// Retrieve the read-only bounds in the BVH.
    ///
    /// These are the user-defined bounds passed to the BVH builder, not the internal bounds around each BVH node.
    @(link_name = "PxBVH_getBounds")
    b_v_h_get_bounds :: proc(self_: ^PxBVH) -> ^PxBounds3 ---

    /// Retrieve the bounds in the BVH.
    ///
    /// These bounds can be modified. Call refit() after modifications are done.
    ///
    /// These are the user-defined bounds passed to the BVH builder, not the internal bounds around each BVH node.
    @(link_name = "PxBVH_getBoundsForModification_mut")
    b_v_h_get_bounds_for_modification_mut :: proc(self_: ^PxBVH) -> ^PxBounds3 ---

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
    b_v_h_refit_mut :: proc(self_: ^PxBVH) ---

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
    b_v_h_update_bounds_mut :: proc(self_: ^PxBVH, boundsIndex: _c.uint32_t, #by_ptr newBounds: PxBounds3) -> _c.bool ---

    /// Refits subset of marked nodes.
    ///
    /// This is an alternative to the refit() function, to be called after updateBounds() calls.
    /// See updateBounds() for details.
    @(link_name = "PxBVH_partialRefit_mut")
    b_v_h_partial_refit_mut :: proc(self_: ^PxBVH) ---

    /// Generic BVH traversal function.
    ///
    /// This can be used to implement custom BVH traversal functions if provided ones are not enough.
    /// In particular this can be used to visualize the tree's bounds.
    ///
    /// false if query has been aborted
    @(link_name = "PxBVH_traverse")
    b_v_h_traverse :: proc(self_: ^PxBVH, cb: ^PxBVHTraversalCallback) -> _c.bool ---

    @(link_name = "PxBVH_getConcreteTypeName")
    b_v_h_get_concrete_type_name :: proc(self_: ^PxBVH) -> ^_c.char ---

    /// Constructor, initializes to a capsule with passed radius and half height.
    @(link_name = "PxCapsuleGeometry_new")
    capsule_geometry_new :: proc(radius_: _c.float, halfHeight_: _c.float) -> PxCapsuleGeometry ---

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid.
    ///
    /// A valid capsule has radius > 0, halfHeight >= 0.
    /// It is illegal to call PxRigidActor::createShape and PxPhysics::createShape with a capsule that has zero radius or height.
    @(link_name = "PxCapsuleGeometry_isValid")
    capsule_geometry_is_valid :: proc(self_: ^PxCapsuleGeometry) -> _c.bool ---

    /// Returns the number of vertices.
    ///
    /// Number of vertices.
    @(link_name = "PxConvexMesh_getNbVertices")
    convex_mesh_get_nb_vertices :: proc(self_: ^PxConvexMesh) -> _c.uint32_t ---

    /// Returns the vertices.
    ///
    /// Array of vertices.
    @(link_name = "PxConvexMesh_getVertices")
    convex_mesh_get_vertices :: proc(self_: ^PxConvexMesh) -> ^PxVec3 ---

    /// Returns the index buffer.
    ///
    /// Index buffer.
    @(link_name = "PxConvexMesh_getIndexBuffer")
    convex_mesh_get_index_buffer :: proc(self_: ^PxConvexMesh) -> ^_c.uint8_t ---

    /// Returns the number of polygons.
    ///
    /// Number of polygons.
    @(link_name = "PxConvexMesh_getNbPolygons")
    convex_mesh_get_nb_polygons :: proc(self_: ^PxConvexMesh) -> _c.uint32_t ---

    /// Returns the polygon data.
    ///
    /// True if success.
    @(link_name = "PxConvexMesh_getPolygonData")
    convex_mesh_get_polygon_data :: proc(self_: ^PxConvexMesh, index: _c.uint32_t, data: ^PxHullPolygon) -> _c.bool ---

    /// Decrements the reference count of a convex mesh and releases it if the new reference count is zero.
    @(link_name = "PxConvexMesh_release_mut")
    convex_mesh_release_mut :: proc(self_: ^PxConvexMesh) ---

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
    convex_mesh_get_mass_information :: proc(self_: ^PxConvexMesh, mass: ^_c.float, localInertia: ^PxMat33, localCenterOfMass: ^PxVec3) ---

    /// Returns the local-space (vertex space) AABB from the convex mesh.
    ///
    /// local-space bounds
    @(link_name = "PxConvexMesh_getLocalBounds")
    convex_mesh_get_local_bounds :: proc(self_: ^PxConvexMesh) -> PxBounds3 ---

    /// Returns the local-space Signed Distance Field for this mesh if it has one.
    ///
    /// local-space SDF.
    @(link_name = "PxConvexMesh_getSDF")
    convex_mesh_get_s_d_f :: proc(self_: ^PxConvexMesh) -> ^_c.float ---

    @(link_name = "PxConvexMesh_getConcreteTypeName")
    convex_mesh_get_concrete_type_name :: proc(self_: ^PxConvexMesh) -> ^_c.char ---

    /// This method decides whether a convex mesh is gpu compatible. If the total number of vertices are more than 64 or any number of vertices in a polygon is more than 32, or
    /// convex hull data was not cooked with GPU data enabled during cooking or was loaded from a serialized collection, the convex hull is incompatible with GPU collision detection. Otherwise
    /// it is compatible.
    ///
    /// True if the convex hull is gpu compatible
    @(link_name = "PxConvexMesh_isGpuCompatible")
    convex_mesh_is_gpu_compatible :: proc(self_: ^PxConvexMesh) -> _c.bool ---

    /// Constructor initializes to identity scale.
    @(link_name = "PxMeshScale_new")
    mesh_scale_new :: proc() -> PxMeshScale ---

    /// Constructor from scalar.
    @(link_name = "PxMeshScale_new_1")
    mesh_scale_new_1 :: proc(r: _c.float) -> PxMeshScale ---

    /// Constructor to initialize to arbitrary scale and identity scale rotation.
    @(link_name = "PxMeshScale_new_2")
    mesh_scale_new_2 :: proc(#by_ptr s: PxVec3) -> PxMeshScale ---

    /// Constructor to initialize to arbitrary scaling.
    @(link_name = "PxMeshScale_new_3")
    mesh_scale_new_3 :: proc(#by_ptr s: PxVec3, #by_ptr r: PxQuat) -> PxMeshScale ---

    /// Returns true if the scaling is an identity transformation.
    @(link_name = "PxMeshScale_isIdentity")
    mesh_scale_is_identity :: proc(self_: ^PxMeshScale) -> _c.bool ---

    /// Returns the inverse of this scaling transformation.
    @(link_name = "PxMeshScale_getInverse")
    mesh_scale_get_inverse :: proc(self_: ^PxMeshScale) -> PxMeshScale ---

    /// Converts this transformation to a 3x3 matrix representation.
    @(link_name = "PxMeshScale_toMat33")
    mesh_scale_to_mat33 :: proc(self_: ^PxMeshScale) -> PxMat33 ---

    /// Returns true if combination of negative scale components will cause the triangle normal to flip. The SDK will flip the normals internally.
    @(link_name = "PxMeshScale_hasNegativeDeterminant")
    mesh_scale_has_negative_determinant :: proc(self_: ^PxMeshScale) -> _c.bool ---

    @(link_name = "PxMeshScale_transform")
    mesh_scale_transform :: proc(self_: ^PxMeshScale, #by_ptr v: PxVec3) -> PxVec3 ---

    @(link_name = "PxMeshScale_isValidForTriangleMesh")
    mesh_scale_is_valid_for_triangle_mesh :: proc(self_: ^PxMeshScale) -> _c.bool ---

    @(link_name = "PxMeshScale_isValidForConvexMesh")
    mesh_scale_is_valid_for_convex_mesh :: proc(self_: ^PxMeshScale) -> _c.bool ---

    /// Constructor. By default creates an empty object with a NULL mesh and identity scale.
    @(link_name = "PxConvexMeshGeometry_new")
    convex_mesh_geometry_new :: proc(mesh: ^PxConvexMesh, #by_ptr scaling: PxMeshScale, flags: PxConvexMeshGeometryFlags_Set) -> PxConvexMeshGeometry ---

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid for shape creation.
    ///
    /// A valid convex mesh has a positive scale value in each direction (scale.x > 0, scale.y > 0, scale.z > 0).
    /// It is illegal to call PxRigidActor::createShape and PxPhysics::createShape with a convex that has zero extent in any direction.
    @(link_name = "PxConvexMeshGeometry_isValid")
    convex_mesh_geometry_is_valid :: proc(self_: ^PxConvexMeshGeometry) -> _c.bool ---

    /// Constructor.
    @(link_name = "PxSphereGeometry_new")
    sphere_geometry_new :: proc(ir: _c.float) -> PxSphereGeometry ---

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid
    ///
    /// A valid sphere has radius > 0.
    /// It is illegal to call PxRigidActor::createShape and PxPhysics::createShape with a sphere that has zero radius.
    @(link_name = "PxSphereGeometry_isValid")
    sphere_geometry_is_valid :: proc(self_: ^PxSphereGeometry) -> _c.bool ---

    /// Constructor.
    @(link_name = "PxPlaneGeometry_new")
    plane_geometry_new :: proc() -> PxPlaneGeometry ---

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid
    @(link_name = "PxPlaneGeometry_isValid")
    plane_geometry_is_valid :: proc(self_: ^PxPlaneGeometry) -> _c.bool ---

    /// Constructor. By default creates an empty object with a NULL mesh and identity scale.
    @(link_name = "PxTriangleMeshGeometry_new")
    triangle_mesh_geometry_new :: proc(mesh: ^PxTriangleMesh, #by_ptr scaling: PxMeshScale, flags: PxMeshGeometryFlags_Set) -> PxTriangleMeshGeometry ---

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid for shape creation.
    ///
    /// A valid triangle mesh has a positive scale value in each direction (scale.scale.x > 0, scale.scale.y > 0, scale.scale.z > 0).
    /// It is illegal to call PxRigidActor::createShape and PxPhysics::createShape with a triangle mesh that has zero extents in any direction.
    @(link_name = "PxTriangleMeshGeometry_isValid")
    triangle_mesh_geometry_is_valid :: proc(self_: ^PxTriangleMeshGeometry) -> _c.bool ---

    /// Constructor.
    @(link_name = "PxHeightFieldGeometry_new")
    height_field_geometry_new :: proc(hf: ^PxHeightField, flags: PxMeshGeometryFlags_Set, heightScale_: _c.float, rowScale_: _c.float, columnScale_: _c.float) -> PxHeightFieldGeometry ---

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid
    ///
    /// A valid height field has a positive scale value in each direction (heightScale > 0, rowScale > 0, columnScale > 0).
    /// It is illegal to call PxRigidActor::createShape and PxPhysics::createShape with a height field that has zero extents in any direction.
    @(link_name = "PxHeightFieldGeometry_isValid")
    height_field_geometry_is_valid :: proc(self_: ^PxHeightFieldGeometry) -> _c.bool ---

    /// Default constructor.
    ///
    /// Creates an empty object with no particles.
    @(link_name = "PxParticleSystemGeometry_new")
    particle_system_geometry_new :: proc() -> PxParticleSystemGeometry ---

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid for shape creation.
    @(link_name = "PxParticleSystemGeometry_isValid")
    particle_system_geometry_is_valid :: proc(self_: ^PxParticleSystemGeometry) -> _c.bool ---

    /// Default constructor.
    @(link_name = "PxHairSystemGeometry_new")
    hair_system_geometry_new :: proc() -> PxHairSystemGeometry ---

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid for shape creation.
    @(link_name = "PxHairSystemGeometry_isValid")
    hair_system_geometry_is_valid :: proc(self_: ^PxHairSystemGeometry) -> _c.bool ---

    /// Constructor. By default creates an empty object with a NULL mesh and identity scale.
    @(link_name = "PxTetrahedronMeshGeometry_new")
    tetrahedron_mesh_geometry_new :: proc(mesh: ^PxTetrahedronMesh) -> PxTetrahedronMeshGeometry ---

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid for shape creation.
    ///
    /// A valid tetrahedron mesh has a positive scale value in each direction (scale.scale.x > 0, scale.scale.y > 0, scale.scale.z > 0).
    /// It is illegal to call PxRigidActor::createShape and PxPhysics::createShape with a tetrahedron mesh that has zero extents in any direction.
    @(link_name = "PxTetrahedronMeshGeometry_isValid")
    tetrahedron_mesh_geometry_is_valid :: proc(self_: ^PxTetrahedronMeshGeometry) -> _c.bool ---

    @(link_name = "PxQueryHit_new")
    query_hit_new :: proc() -> PxQueryHit ---

    @(link_name = "PxLocationHit_new")
    location_hit_new :: proc() -> PxLocationHit ---

    /// For raycast hits: true for shapes overlapping with raycast origin.
    ///
    /// For sweep hits: true for shapes overlapping at zero sweep distance.
    @(link_name = "PxLocationHit_hadInitialOverlap")
    location_hit_had_initial_overlap :: proc(self_: ^PxLocationHit) -> _c.bool ---

    @(link_name = "PxGeomRaycastHit_new")
    geom_raycast_hit_new :: proc() -> PxGeomRaycastHit ---

    @(link_name = "PxGeomOverlapHit_new")
    geom_overlap_hit_new :: proc() -> PxGeomOverlapHit ---

    @(link_name = "PxGeomSweepHit_new")
    geom_sweep_hit_new :: proc() -> PxGeomSweepHit ---

    @(link_name = "PxGeomIndexPair_new")
    geom_index_pair_new :: proc() -> PxGeomIndexPair ---

    @(link_name = "PxGeomIndexPair_new_1")
    geom_index_pair_new_1 :: proc(_id0: _c.uint32_t, _id1: _c.uint32_t) -> PxGeomIndexPair ---

    /// For internal use
    @(link_name = "phys_PxCustomGeometry_getUniqueID")
    custom_geometry_get_unique_i_d :: proc() -> _c.uint32_t ---

    /// Default constructor
    @(link_name = "PxCustomGeometryType_new")
    custom_geometry_type_new :: proc() -> PxCustomGeometryType ---

    /// Invalid type
    @(link_name = "PxCustomGeometryType_INVALID")
    custom_geometry_type__i_n_v_a_l_i_d :: proc() -> PxCustomGeometryType ---

    /// Return custom type. The type purpose is for user to differentiate custom geometries. Not used by PhysX.
    ///
    /// Unique ID of a custom geometry type.
    ///
    /// User should use DECLARE_CUSTOM_GEOMETRY_TYPE and IMPLEMENT_CUSTOM_GEOMETRY_TYPE intead of overwriting this function.
    @(link_name = "PxCustomGeometryCallbacks_getCustomType")
    custom_geometry_callbacks_get_custom_type :: proc(self_: ^PxCustomGeometryCallbacks) -> PxCustomGeometryType ---

    /// Return local bounds.
    ///
    /// Bounding box in the geometry local space.
    @(link_name = "PxCustomGeometryCallbacks_getLocalBounds")
    custom_geometry_callbacks_get_local_bounds :: proc(self_: ^PxCustomGeometryCallbacks, geometry: ^PxGeometry) -> PxBounds3 ---

    /// Raycast. Cast a ray against the geometry in given pose.
    ///
    /// Number of hits.
    @(link_name = "PxCustomGeometryCallbacks_raycast")
    custom_geometry_callbacks_raycast :: proc(self_: ^PxCustomGeometryCallbacks, #by_ptr origin: PxVec3, #by_ptr unitDir: PxVec3, geom: ^PxGeometry, #by_ptr pose: PxTransform, maxDist: _c.float, hitFlags: PxHitFlags_Set, maxHits: _c.uint32_t, rayHits: ^PxGeomRaycastHit, stride: _c.uint32_t, threadContext: ^PxQueryThreadContext) -> _c.uint32_t ---

    /// Overlap. Test if geometries overlap.
    ///
    /// True if there is overlap. False otherwise.
    @(link_name = "PxCustomGeometryCallbacks_overlap")
    custom_geometry_callbacks_overlap :: proc(self_: ^PxCustomGeometryCallbacks, geom0: ^PxGeometry, #by_ptr pose0: PxTransform, geom1: ^PxGeometry, #by_ptr pose1: PxTransform, threadContext: ^PxQueryThreadContext) -> _c.bool ---

    /// Sweep. Sweep one geometry against the other.
    ///
    /// True if there is hit. False otherwise.
    @(link_name = "PxCustomGeometryCallbacks_sweep")
    custom_geometry_callbacks_sweep :: proc(self_: ^PxCustomGeometryCallbacks, #by_ptr unitDir: PxVec3, maxDist: _c.float, geom0: ^PxGeometry, #by_ptr pose0: PxTransform, geom1: ^PxGeometry, #by_ptr pose1: PxTransform, sweepHit: ^PxGeomSweepHit, hitFlags: PxHitFlags_Set, inflation: _c.float, threadContext: ^PxQueryThreadContext) -> _c.bool ---

    /// Compute custom geometry mass properties. For geometries usable with dynamic rigidbodies.
    @(link_name = "PxCustomGeometryCallbacks_computeMassProperties")
    custom_geometry_callbacks_compute_mass_properties :: proc(self_: ^PxCustomGeometryCallbacks, geometry: ^PxGeometry, massProperties: ^PxMassProperties) ---

    /// Compatible with PhysX's PCM feature. Allows to optimize contact generation.
    @(link_name = "PxCustomGeometryCallbacks_usePersistentContactManifold")
    custom_geometry_callbacks_use_persistent_contact_manifold :: proc(self_: ^PxCustomGeometryCallbacks, geometry: ^PxGeometry, breakingThreshold: ^_c.float) -> _c.bool ---

    @(link_name = "PxCustomGeometryCallbacks_delete")
    custom_geometry_callbacks_delete :: proc(self_: ^PxCustomGeometryCallbacks) ---

    /// Default constructor.
    ///
    /// Creates an empty object with a NULL callbacks pointer.
    @(link_name = "PxCustomGeometry_new")
    custom_geometry_new :: proc() -> PxCustomGeometry ---

    /// Constructor.
    @(link_name = "PxCustomGeometry_new_1")
    custom_geometry_new_1 :: proc(_callbacks: ^PxCustomGeometryCallbacks) -> PxCustomGeometry ---

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid for shape creation.
    @(link_name = "PxCustomGeometry_isValid")
    custom_geometry_is_valid :: proc(self_: ^PxCustomGeometry) -> _c.bool ---

    /// Returns the custom type of the custom geometry.
    @(link_name = "PxCustomGeometry_getCustomType")
    custom_geometry_get_custom_type :: proc(self_: ^PxCustomGeometry) -> PxCustomGeometryType ---

    @(link_name = "PxGeometryHolder_getType")
    geometry_holder_get_type :: proc(self_: ^PxGeometryHolder) -> PxGeometryType ---

    @(link_name = "PxGeometryHolder_any_mut")
    geometry_holder_any_mut :: proc(self_: ^PxGeometryHolder) -> ^PxGeometry ---

    @(link_name = "PxGeometryHolder_any")
    geometry_holder_any :: proc(self_: ^PxGeometryHolder) -> ^PxGeometry ---

    @(link_name = "PxGeometryHolder_sphere_mut")
    geometry_holder_sphere_mut :: proc(self_: ^PxGeometryHolder) -> ^PxSphereGeometry ---

    @(link_name = "PxGeometryHolder_sphere")
    geometry_holder_sphere :: proc(self_: ^PxGeometryHolder) -> ^PxSphereGeometry ---

    @(link_name = "PxGeometryHolder_plane_mut")
    geometry_holder_plane_mut :: proc(self_: ^PxGeometryHolder) -> ^PxPlaneGeometry ---

    @(link_name = "PxGeometryHolder_plane")
    geometry_holder_plane :: proc(self_: ^PxGeometryHolder) -> ^PxPlaneGeometry ---

    @(link_name = "PxGeometryHolder_capsule_mut")
    geometry_holder_capsule_mut :: proc(self_: ^PxGeometryHolder) -> ^PxCapsuleGeometry ---

    @(link_name = "PxGeometryHolder_capsule")
    geometry_holder_capsule :: proc(self_: ^PxGeometryHolder) -> ^PxCapsuleGeometry ---

    @(link_name = "PxGeometryHolder_box_mut")
    geometry_holder_box_mut :: proc(self_: ^PxGeometryHolder) -> ^PxBoxGeometry ---

    @(link_name = "PxGeometryHolder_box")
    geometry_holder_box :: proc(self_: ^PxGeometryHolder) -> ^PxBoxGeometry ---

    @(link_name = "PxGeometryHolder_convexMesh_mut")
    geometry_holder_convex_mesh_mut :: proc(self_: ^PxGeometryHolder) -> ^PxConvexMeshGeometry ---

    @(link_name = "PxGeometryHolder_convexMesh")
    geometry_holder_convex_mesh :: proc(self_: ^PxGeometryHolder) -> ^PxConvexMeshGeometry ---

    @(link_name = "PxGeometryHolder_tetMesh_mut")
    geometry_holder_tet_mesh_mut :: proc(self_: ^PxGeometryHolder) -> ^PxTetrahedronMeshGeometry ---

    @(link_name = "PxGeometryHolder_tetMesh")
    geometry_holder_tet_mesh :: proc(self_: ^PxGeometryHolder) -> ^PxTetrahedronMeshGeometry ---

    @(link_name = "PxGeometryHolder_triangleMesh_mut")
    geometry_holder_triangle_mesh_mut :: proc(self_: ^PxGeometryHolder) -> ^PxTriangleMeshGeometry ---

    @(link_name = "PxGeometryHolder_triangleMesh")
    geometry_holder_triangle_mesh :: proc(self_: ^PxGeometryHolder) -> ^PxTriangleMeshGeometry ---

    @(link_name = "PxGeometryHolder_heightField_mut")
    geometry_holder_height_field_mut :: proc(self_: ^PxGeometryHolder) -> ^PxHeightFieldGeometry ---

    @(link_name = "PxGeometryHolder_heightField")
    geometry_holder_height_field :: proc(self_: ^PxGeometryHolder) -> ^PxHeightFieldGeometry ---

    @(link_name = "PxGeometryHolder_particleSystem_mut")
    geometry_holder_particle_system_mut :: proc(self_: ^PxGeometryHolder) -> ^PxParticleSystemGeometry ---

    @(link_name = "PxGeometryHolder_particleSystem")
    geometry_holder_particle_system :: proc(self_: ^PxGeometryHolder) -> ^PxParticleSystemGeometry ---

    @(link_name = "PxGeometryHolder_hairSystem_mut")
    geometry_holder_hair_system_mut :: proc(self_: ^PxGeometryHolder) -> ^PxHairSystemGeometry ---

    @(link_name = "PxGeometryHolder_hairSystem")
    geometry_holder_hair_system :: proc(self_: ^PxGeometryHolder) -> ^PxHairSystemGeometry ---

    @(link_name = "PxGeometryHolder_custom_mut")
    geometry_holder_custom_mut :: proc(self_: ^PxGeometryHolder) -> ^PxCustomGeometry ---

    @(link_name = "PxGeometryHolder_custom")
    geometry_holder_custom :: proc(self_: ^PxGeometryHolder) -> ^PxCustomGeometry ---

    @(link_name = "PxGeometryHolder_storeAny_mut")
    geometry_holder_store_any_mut :: proc(self_: ^PxGeometryHolder, geometry: ^PxGeometry) ---

    @(link_name = "PxGeometryHolder_new")
    geometry_holder_new :: proc() -> PxGeometryHolder ---

    @(link_name = "PxGeometryHolder_new_1")
    geometry_holder_new_1 :: proc(geometry: ^PxGeometry) -> PxGeometryHolder ---

    /// Raycast test against a geometry object.
    ///
    /// All geometry types are supported except PxParticleSystemGeometry, PxTetrahedronMeshGeometry and PxHairSystemGeometry.
    ///
    /// Number of hits between the ray and the geometry object
    @(link_name = "PxGeometryQuery_raycast")
    geometry_query_raycast :: proc(#by_ptr origin: PxVec3, #by_ptr unitDir: PxVec3, geom: ^PxGeometry, #by_ptr pose: PxTransform, maxDist: _c.float, hitFlags: PxHitFlags_Set, maxHits: _c.uint32_t, rayHits: ^PxGeomRaycastHit, stride: _c.uint32_t, queryFlags: PxGeometryQueryFlags_Set, threadContext: ^PxQueryThreadContext) -> _c.uint32_t ---

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
    geometry_query_overlap :: proc(geom0: ^PxGeometry, #by_ptr pose0: PxTransform, geom1: ^PxGeometry, #by_ptr pose1: PxTransform, queryFlags: PxGeometryQueryFlags_Set, threadContext: ^PxQueryThreadContext) -> _c.bool ---

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
    geometry_query_sweep :: proc(#by_ptr unitDir: PxVec3, maxDist: _c.float, geom0: ^PxGeometry, #by_ptr pose0: PxTransform, geom1: ^PxGeometry, #by_ptr pose1: PxTransform, sweepHit: ^PxGeomSweepHit, hitFlags: PxHitFlags_Set, inflation: _c.float, queryFlags: PxGeometryQueryFlags_Set, threadContext: ^PxQueryThreadContext) -> _c.bool ---

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
    geometry_query_compute_penetration :: proc(direction: ^PxVec3, depth: ^_c.float, geom0: ^PxGeometry, #by_ptr pose0: PxTransform, geom1: ^PxGeometry, #by_ptr pose1: PxTransform, queryFlags: PxGeometryQueryFlags_Set) -> _c.bool ---

    /// Computes distance between a point and a geometry object.
    ///
    /// Currently supported geometry objects: box, sphere, capsule, convex, mesh.
    ///
    /// For meshes, only the BVH34 midphase data-structure is supported.
    ///
    /// Square distance between the point and the geom object, or 0.0 if the point is inside the object, or -1.0 if an error occured (geometry type is not supported, or invalid pose)
    @(link_name = "PxGeometryQuery_pointDistance")
    geometry_query_point_distance :: proc(#by_ptr point: PxVec3, geom: ^PxGeometry, #by_ptr pose: PxTransform, closestPoint: ^PxVec3, closestIndex: ^_c.uint32_t, queryFlags: PxGeometryQueryFlags_Set) -> _c.float ---

    /// computes the bounds for a geometry object
    @(link_name = "PxGeometryQuery_computeGeomBounds")
    geometry_query_compute_geom_bounds :: proc(bounds: ^PxBounds3, geom: ^PxGeometry, #by_ptr pose: PxTransform, offset: _c.float, inflation: _c.float, queryFlags: PxGeometryQueryFlags_Set) ---

    /// Checks if provided geometry is valid.
    ///
    /// True if geometry is valid.
    @(link_name = "PxGeometryQuery_isValid")
    geometry_query_is_valid :: proc(geom: ^PxGeometry) -> _c.bool ---

    @(link_name = "PxHeightFieldSample_tessFlag")
    height_field_sample_tess_flag :: proc(self_: ^PxHeightFieldSample) -> _c.uint8_t ---

    @(link_name = "PxHeightFieldSample_setTessFlag_mut")
    height_field_sample_set_tess_flag_mut :: proc(self_: ^PxHeightFieldSample) ---

    @(link_name = "PxHeightFieldSample_clearTessFlag_mut")
    height_field_sample_clear_tess_flag_mut :: proc(self_: ^PxHeightFieldSample) ---

    /// Decrements the reference count of a height field and releases it if the new reference count is zero.
    @(link_name = "PxHeightField_release_mut")
    height_field_release_mut :: proc(self_: ^PxHeightField) ---

    /// Writes out the sample data array.
    ///
    /// The user provides destBufferSize bytes storage at destBuffer.
    /// The data is formatted and arranged as PxHeightFieldDesc.samples.
    ///
    /// The number of bytes written.
    @(link_name = "PxHeightField_saveCells")
    height_field_save_cells :: proc(self_: ^PxHeightField, destBuffer: rawptr, destBufferSize: _c.uint32_t) -> _c.uint32_t ---

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
    height_field_modify_samples_mut :: proc(self_: ^PxHeightField, startCol: _c.int32_t, startRow: _c.int32_t, #by_ptr subfieldDesc: PxHeightFieldDesc, shrinkBounds: _c.bool) -> _c.bool ---

    /// Retrieves the number of sample rows in the samples array.
    ///
    /// The number of sample rows in the samples array.
    @(link_name = "PxHeightField_getNbRows")
    height_field_get_nb_rows :: proc(self_: ^PxHeightField) -> _c.uint32_t ---

    /// Retrieves the number of sample columns in the samples array.
    ///
    /// The number of sample columns in the samples array.
    @(link_name = "PxHeightField_getNbColumns")
    height_field_get_nb_columns :: proc(self_: ^PxHeightField) -> _c.uint32_t ---

    /// Retrieves the format of the sample data.
    ///
    /// The format of the sample data.
    @(link_name = "PxHeightField_getFormat")
    height_field_get_format :: proc(self_: ^PxHeightField) -> PxHeightFieldFormat ---

    /// Retrieves the offset in bytes between consecutive samples in the array.
    ///
    /// The offset in bytes between consecutive samples in the array.
    @(link_name = "PxHeightField_getSampleStride")
    height_field_get_sample_stride :: proc(self_: ^PxHeightField) -> _c.uint32_t ---

    /// Retrieves the convex edge threshold.
    ///
    /// The convex edge threshold.
    @(link_name = "PxHeightField_getConvexEdgeThreshold")
    height_field_get_convex_edge_threshold :: proc(self_: ^PxHeightField) -> _c.float ---

    /// Retrieves the flags bits, combined from values of the enum ::PxHeightFieldFlag.
    ///
    /// The flags bits, combined from values of the enum ::PxHeightFieldFlag.
    @(link_name = "PxHeightField_getFlags")
    height_field_get_flags :: proc(self_: ^PxHeightField) -> PxHeightFieldFlags_Set ---

    /// Retrieves the height at the given coordinates in grid space.
    ///
    /// The height at the given coordinates or 0 if the coordinates are out of range.
    @(link_name = "PxHeightField_getHeight")
    height_field_get_height :: proc(self_: ^PxHeightField, x: _c.float, z: _c.float) -> _c.float ---

    /// Returns material table index of given triangle
    ///
    /// This function takes a post cooking triangle index.
    ///
    /// Material table index, or 0xffff if no per-triangle materials are used
    @(link_name = "PxHeightField_getTriangleMaterialIndex")
    height_field_get_triangle_material_index :: proc(self_: ^PxHeightField, triangleIndex: _c.uint32_t) -> _c.uint16_t ---

    /// Returns a triangle face normal for a given triangle index
    ///
    /// This function takes a post cooking triangle index.
    ///
    /// Triangle normal for a given triangle index
    @(link_name = "PxHeightField_getTriangleNormal")
    height_field_get_triangle_normal :: proc(self_: ^PxHeightField, triangleIndex: _c.uint32_t) -> PxVec3 ---

    /// Returns heightfield sample of given row and column
    ///
    /// Heightfield sample
    @(link_name = "PxHeightField_getSample")
    height_field_get_sample :: proc(self_: ^PxHeightField, row: _c.uint32_t, column: _c.uint32_t) -> ^PxHeightFieldSample ---

    /// Returns the number of times the heightfield data has been modified
    ///
    /// This method returns the number of times modifySamples has been called on this heightfield, so that code that has
    /// retained state that depends on the heightfield can efficiently determine whether it has been modified.
    ///
    /// the number of times the heightfield sample data has been modified.
    @(link_name = "PxHeightField_getTimestamp")
    height_field_get_timestamp :: proc(self_: ^PxHeightField) -> _c.uint32_t ---

    @(link_name = "PxHeightField_getConcreteTypeName")
    height_field_get_concrete_type_name :: proc(self_: ^PxHeightField) -> ^_c.char ---

    /// Constructor sets to default.
    @(link_name = "PxHeightFieldDesc_new")
    height_field_desc_new :: proc() -> PxHeightFieldDesc ---

    /// (re)sets the structure to the default.
    @(link_name = "PxHeightFieldDesc_setToDefault_mut")
    height_field_desc_set_to_default_mut :: proc(self_: ^PxHeightFieldDesc) ---

    /// Returns true if the descriptor is valid.
    ///
    /// True if the current settings are valid.
    @(link_name = "PxHeightFieldDesc_isValid")
    height_field_desc_is_valid :: proc(self_: ^PxHeightFieldDesc) -> _c.bool ---

    /// Retrieves triangle data from a triangle ID.
    ///
    /// This function can be used together with [`findOverlapTriangleMesh`]() to retrieve triangle properties.
    ///
    /// This function will flip the triangle normal whenever triGeom.scale.hasNegativeDeterminant() is true.
    @(link_name = "PxMeshQuery_getTriangle")
    mesh_query_get_triangle :: proc(#by_ptr triGeom: PxTriangleMeshGeometry, #by_ptr transform: PxTransform, triangleIndex: _c.uint32_t, triangle: ^PxTriangle, vertexIndices: ^_c.uint32_t, adjacencyIndices: ^_c.uint32_t) ---

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
    mesh_query_get_triangle_1 :: proc(#by_ptr hfGeom: PxHeightFieldGeometry, #by_ptr transform: PxTransform, triangleIndex: _c.uint32_t, triangle: ^PxTriangle, vertexIndices: ^_c.uint32_t, adjacencyIndices: ^_c.uint32_t) ---

    /// Find the mesh triangles which touch the specified geometry object.
    ///
    /// For mesh-vs-mesh overlap tests, please use the specialized function below.
    ///
    /// Returned triangle indices can be used with [`getTriangle`]() to retrieve the triangle properties.
    ///
    /// Number of overlaps found, i.e. number of elements written to the results buffer
    @(link_name = "PxMeshQuery_findOverlapTriangleMesh")
    mesh_query_find_overlap_triangle_mesh :: proc(geom: ^PxGeometry, #by_ptr geomPose: PxTransform, #by_ptr meshGeom: PxTriangleMeshGeometry, #by_ptr meshPose: PxTransform, results: ^_c.uint32_t, maxResults: _c.uint32_t, startIndex: _c.uint32_t, overflow: ^_c.bool, queryFlags: PxGeometryQueryFlags_Set) -> _c.uint32_t ---

    /// Find the height field triangles which touch the specified geometry object.
    ///
    /// Returned triangle indices can be used with [`getTriangle`]() to retrieve the triangle properties.
    ///
    /// Number of overlaps found, i.e. number of elements written to the results buffer
    @(link_name = "PxMeshQuery_findOverlapHeightField")
    mesh_query_find_overlap_height_field :: proc(geom: ^PxGeometry, #by_ptr geomPose: PxTransform, #by_ptr hfGeom: PxHeightFieldGeometry, #by_ptr hfPose: PxTransform, results: ^_c.uint32_t, maxResults: _c.uint32_t, startIndex: _c.uint32_t, overflow: ^_c.bool, queryFlags: PxGeometryQueryFlags_Set) -> _c.uint32_t ---

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
    mesh_query_sweep :: proc(#by_ptr unitDir: PxVec3, distance: _c.float, geom: ^PxGeometry, #by_ptr pose: PxTransform, triangleCount: _c.uint32_t, triangles: ^PxTriangle, sweepHit: ^PxGeomSweepHit, hitFlags: PxHitFlags_Set, cachedIndex: ^_c.uint32_t, inflation: _c.float, doubleSided: _c.bool, queryFlags: PxGeometryQueryFlags_Set) -> _c.bool ---

    /// constructor sets to default.
    @(link_name = "PxSimpleTriangleMesh_new")
    simple_triangle_mesh_new :: proc() -> PxSimpleTriangleMesh ---

    /// (re)sets the structure to the default.
    @(link_name = "PxSimpleTriangleMesh_setToDefault_mut")
    simple_triangle_mesh_set_to_default_mut :: proc(self_: ^PxSimpleTriangleMesh) ---

    /// returns true if the current settings are valid
    @(link_name = "PxSimpleTriangleMesh_isValid")
    simple_triangle_mesh_is_valid :: proc(self_: ^PxSimpleTriangleMesh) -> _c.bool ---

    /// Constructor
    @(link_name = "PxTriangle_new_alloc")
    triangle_new_alloc :: proc() -> ^PxTriangle ---

    /// Constructor
    @(link_name = "PxTriangle_new_alloc_1")
    triangle_new_alloc_1 :: proc(#by_ptr p0: PxVec3, #by_ptr p1: PxVec3, #by_ptr p2: PxVec3) -> ^PxTriangle ---

    /// Destructor
    @(link_name = "PxTriangle_delete")
    triangle_delete :: proc(self_: ^PxTriangle) ---

    /// Compute the normal of the Triangle.
    @(link_name = "PxTriangle_normal")
    triangle_normal :: proc(self_: ^PxTriangle, _normal: ^PxVec3) ---

    /// Compute the unnormalized normal of the triangle.
    @(link_name = "PxTriangle_denormalizedNormal")
    triangle_denormalized_normal :: proc(self_: ^PxTriangle, _normal: ^PxVec3) ---

    /// Compute the area of the triangle.
    ///
    /// Area of the triangle.
    @(link_name = "PxTriangle_area")
    triangle_area :: proc(self_: ^PxTriangle) -> _c.float ---

    /// Computes a point on the triangle from u and v barycentric coordinates.
    @(link_name = "PxTriangle_pointFromUV")
    triangle_point_from_u_v :: proc(self_: ^PxTriangle, u: _c.float, v: _c.float) -> PxVec3 ---

    @(link_name = "PxTrianglePadded_new_alloc")
    triangle_padded_new_alloc :: proc() -> ^PxTrianglePadded ---

    @(link_name = "PxTrianglePadded_delete")
    triangle_padded_delete :: proc(self_: ^PxTrianglePadded) ---

    /// Returns the number of vertices.
    ///
    /// number of vertices
    @(link_name = "PxTriangleMesh_getNbVertices")
    triangle_mesh_get_nb_vertices :: proc(self_: ^PxTriangleMesh) -> _c.uint32_t ---

    /// Returns the vertices.
    ///
    /// array of vertices
    @(link_name = "PxTriangleMesh_getVertices")
    triangle_mesh_get_vertices :: proc(self_: ^PxTriangleMesh) -> ^PxVec3 ---

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
    triangle_mesh_get_vertices_for_modification_mut :: proc(self_: ^PxTriangleMesh) -> ^PxVec3 ---

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
    triangle_mesh_refit_b_v_h_mut :: proc(self_: ^PxTriangleMesh) -> PxBounds3 ---

    /// Returns the number of triangles.
    ///
    /// number of triangles
    @(link_name = "PxTriangleMesh_getNbTriangles")
    triangle_mesh_get_nb_triangles :: proc(self_: ^PxTriangleMesh) -> _c.uint32_t ---

    /// Returns the triangle indices.
    ///
    /// The indices can be 16 or 32bit depending on the number of triangles in the mesh.
    /// Call getTriangleMeshFlags() to know if the indices are 16 or 32 bits.
    ///
    /// The number of indices is the number of triangles * 3.
    ///
    /// array of triangles
    @(link_name = "PxTriangleMesh_getTriangles")
    triangle_mesh_get_triangles :: proc(self_: ^PxTriangleMesh) -> rawptr ---

    /// Reads the PxTriangleMesh flags.
    ///
    /// See the list of flags [`PxTriangleMeshFlag`]
    ///
    /// The values of the PxTriangleMesh flags.
    @(link_name = "PxTriangleMesh_getTriangleMeshFlags")
    triangle_mesh_get_triangle_mesh_flags :: proc(self_: ^PxTriangleMesh) -> PxTriangleMeshFlags_Set ---

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
    triangle_mesh_get_triangles_remap :: proc(self_: ^PxTriangleMesh) -> ^_c.uint32_t ---

    /// Decrements the reference count of a triangle mesh and releases it if the new reference count is zero.
    @(link_name = "PxTriangleMesh_release_mut")
    triangle_mesh_release_mut :: proc(self_: ^PxTriangleMesh) ---

    /// Returns material table index of given triangle
    ///
    /// This function takes a post cooking triangle index.
    ///
    /// Material table index, or 0xffff if no per-triangle materials are used
    @(link_name = "PxTriangleMesh_getTriangleMaterialIndex")
    triangle_mesh_get_triangle_material_index :: proc(self_: ^PxTriangleMesh, triangleIndex: _c.uint32_t) -> _c.uint16_t ---

    /// Returns the local-space (vertex space) AABB from the triangle mesh.
    ///
    /// local-space bounds
    @(link_name = "PxTriangleMesh_getLocalBounds")
    triangle_mesh_get_local_bounds :: proc(self_: ^PxTriangleMesh) -> PxBounds3 ---

    /// Returns the local-space Signed Distance Field for this mesh if it has one.
    ///
    /// local-space SDF.
    @(link_name = "PxTriangleMesh_getSDF")
    triangle_mesh_get_s_d_f :: proc(self_: ^PxTriangleMesh) -> ^_c.float ---

    /// Returns the resolution of the local-space dense SDF.
    @(link_name = "PxTriangleMesh_getSDFDimensions")
    triangle_mesh_get_s_d_f_dimensions :: proc(self_: ^PxTriangleMesh, numX: ^_c.uint32_t, numY: ^_c.uint32_t, numZ: ^_c.uint32_t) ---

    /// Sets whether this mesh should be preferred for SDF projection.
    ///
    /// By default, meshes are flagged as preferring projection and the decisions on which mesh to project is based on the triangle and vertex
    /// count. The model with the fewer triangles is projected onto the SDF of the more detailed mesh.
    /// If one of the meshes is set to prefer SDF projection (default) and the other is set to not prefer SDF projection, model flagged as
    /// preferring SDF projection will be projected onto the model flagged as not preferring, regardless of the detail of the respective meshes.
    /// Where both models are flagged as preferring no projection, the less detailed model will be projected as before.
    @(link_name = "PxTriangleMesh_setPreferSDFProjection_mut")
    triangle_mesh_set_prefer_s_d_f_projection_mut :: proc(self_: ^PxTriangleMesh, preferProjection: _c.bool) ---

    /// Returns whether this mesh prefers SDF projection.
    ///
    /// whether this mesh prefers SDF projection.
    @(link_name = "PxTriangleMesh_getPreferSDFProjection")
    triangle_mesh_get_prefer_s_d_f_projection :: proc(self_: ^PxTriangleMesh) -> _c.bool ---

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
    triangle_mesh_get_mass_information :: proc(self_: ^PxTriangleMesh, mass: ^_c.float, localInertia: ^PxMat33, localCenterOfMass: ^PxVec3) ---

    /// Constructor
    @(link_name = "PxTetrahedron_new_alloc")
    tetrahedron_new_alloc :: proc() -> ^PxTetrahedron ---

    /// Constructor
    @(link_name = "PxTetrahedron_new_alloc_1")
    tetrahedron_new_alloc_1 :: proc(#by_ptr p0: PxVec3, #by_ptr p1: PxVec3, #by_ptr p2: PxVec3, #by_ptr p3: PxVec3) -> ^PxTetrahedron ---

    /// Destructor
    @(link_name = "PxTetrahedron_delete")
    tetrahedron_delete :: proc(self_: ^PxTetrahedron) ---

    /// Decrements the reference count of a tetrahedron mesh and releases it if the new reference count is zero.
    @(link_name = "PxSoftBodyAuxData_release_mut")
    soft_body_aux_data_release_mut :: proc(self_: ^PxSoftBodyAuxData) ---

    /// Returns the number of vertices.
    ///
    /// number of vertices
    @(link_name = "PxTetrahedronMesh_getNbVertices")
    tetrahedron_mesh_get_nb_vertices :: proc(self_: ^PxTetrahedronMesh) -> _c.uint32_t ---

    /// Returns the vertices
    ///
    /// array of vertices
    @(link_name = "PxTetrahedronMesh_getVertices")
    tetrahedron_mesh_get_vertices :: proc(self_: ^PxTetrahedronMesh) -> ^PxVec3 ---

    /// Returns the number of tetrahedrons.
    ///
    /// number of tetrahedrons
    @(link_name = "PxTetrahedronMesh_getNbTetrahedrons")
    tetrahedron_mesh_get_nb_tetrahedrons :: proc(self_: ^PxTetrahedronMesh) -> _c.uint32_t ---

    /// Returns the tetrahedron indices.
    ///
    /// The indices can be 16 or 32bit depending on the number of tetrahedrons in the mesh.
    /// Call getTetrahedronMeshFlags() to know if the indices are 16 or 32 bits.
    ///
    /// The number of indices is the number of tetrahedrons * 4.
    ///
    /// array of tetrahedrons
    @(link_name = "PxTetrahedronMesh_getTetrahedrons")
    tetrahedron_mesh_get_tetrahedrons :: proc(self_: ^PxTetrahedronMesh) -> rawptr ---

    /// Reads the PxTetrahedronMesh flags.
    ///
    /// See the list of flags [`PxTetrahedronMeshFlags`]
    ///
    /// The values of the PxTetrahedronMesh flags.
    @(link_name = "PxTetrahedronMesh_getTetrahedronMeshFlags")
    tetrahedron_mesh_get_tetrahedron_mesh_flags :: proc(self_: ^PxTetrahedronMesh) -> PxTetrahedronMeshFlags_Set ---

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
    tetrahedron_mesh_get_tetrahedra_remap :: proc(self_: ^PxTetrahedronMesh) -> ^_c.uint32_t ---

    /// Returns the local-space (vertex space) AABB from the tetrahedron mesh.
    ///
    /// local-space bounds
    @(link_name = "PxTetrahedronMesh_getLocalBounds")
    tetrahedron_mesh_get_local_bounds :: proc(self_: ^PxTetrahedronMesh) -> PxBounds3 ---

    /// Decrements the reference count of a tetrahedron mesh and releases it if the new reference count is zero.
    @(link_name = "PxTetrahedronMesh_release_mut")
    tetrahedron_mesh_release_mut :: proc(self_: ^PxTetrahedronMesh) ---

    /// Const accecssor to the softbody's collision mesh.
    @(link_name = "PxSoftBodyMesh_getCollisionMesh")
    soft_body_mesh_get_collision_mesh :: proc(self_: ^PxSoftBodyMesh) -> ^PxTetrahedronMesh ---

    /// Accecssor to the softbody's collision mesh.
    @(link_name = "PxSoftBodyMesh_getCollisionMesh_mut")
    soft_body_mesh_get_collision_mesh_mut :: proc(self_: ^PxSoftBodyMesh) -> ^PxTetrahedronMesh ---

    /// Const accessor to the softbody's simulation mesh.
    @(link_name = "PxSoftBodyMesh_getSimulationMesh")
    soft_body_mesh_get_simulation_mesh :: proc(self_: ^PxSoftBodyMesh) -> ^PxTetrahedronMesh ---

    /// Accecssor to the softbody's simulation mesh.
    @(link_name = "PxSoftBodyMesh_getSimulationMesh_mut")
    soft_body_mesh_get_simulation_mesh_mut :: proc(self_: ^PxSoftBodyMesh) -> ^PxTetrahedronMesh ---

    /// Const accessor to the softbodies simulation state.
    @(link_name = "PxSoftBodyMesh_getSoftBodyAuxData")
    soft_body_mesh_get_soft_body_aux_data :: proc(self_: ^PxSoftBodyMesh) -> ^PxSoftBodyAuxData ---

    /// Accessor to the softbody's auxilary data like mass and rest pose information
    @(link_name = "PxSoftBodyMesh_getSoftBodyAuxData_mut")
    soft_body_mesh_get_soft_body_aux_data_mut :: proc(self_: ^PxSoftBodyMesh) -> ^PxSoftBodyAuxData ---

    /// Decrements the reference count of a tetrahedron mesh and releases it if the new reference count is zero.
    @(link_name = "PxSoftBodyMesh_release_mut")
    soft_body_mesh_release_mut :: proc(self_: ^PxSoftBodyMesh) ---

    @(link_name = "PxCollisionMeshMappingData_release_mut")
    collision_mesh_mapping_data_release_mut :: proc(self_: ^PxCollisionMeshMappingData) ---

    @(link_name = "PxCollisionTetrahedronMeshData_getMesh")
    collision_tetrahedron_mesh_data_get_mesh :: proc(self_: ^PxCollisionTetrahedronMeshData) -> ^PxTetrahedronMeshData ---

    @(link_name = "PxCollisionTetrahedronMeshData_getMesh_mut")
    collision_tetrahedron_mesh_data_get_mesh_mut :: proc(self_: ^PxCollisionTetrahedronMeshData) -> ^PxTetrahedronMeshData ---

    @(link_name = "PxCollisionTetrahedronMeshData_getData")
    collision_tetrahedron_mesh_data_get_data :: proc(self_: ^PxCollisionTetrahedronMeshData) -> ^PxSoftBodyCollisionData ---

    @(link_name = "PxCollisionTetrahedronMeshData_getData_mut")
    collision_tetrahedron_mesh_data_get_data_mut :: proc(self_: ^PxCollisionTetrahedronMeshData) -> ^PxSoftBodyCollisionData ---

    @(link_name = "PxCollisionTetrahedronMeshData_release_mut")
    collision_tetrahedron_mesh_data_release_mut :: proc(self_: ^PxCollisionTetrahedronMeshData) ---

    @(link_name = "PxSimulationTetrahedronMeshData_getMesh_mut")
    simulation_tetrahedron_mesh_data_get_mesh_mut :: proc(self_: ^PxSimulationTetrahedronMeshData) -> ^PxTetrahedronMeshData ---

    @(link_name = "PxSimulationTetrahedronMeshData_getData_mut")
    simulation_tetrahedron_mesh_data_get_data_mut :: proc(self_: ^PxSimulationTetrahedronMeshData) -> ^PxSoftBodySimulationData ---

    @(link_name = "PxSimulationTetrahedronMeshData_release_mut")
    simulation_tetrahedron_mesh_data_release_mut :: proc(self_: ^PxSimulationTetrahedronMeshData) ---

    /// Deletes the actor.
    ///
    /// Do not keep a reference to the deleted instance.
    ///
    /// If the actor belongs to a [`PxAggregate`] object, it is automatically removed from the aggregate.
    @(link_name = "PxActor_release_mut")
    actor_release_mut :: proc(self_: ^PxActor) ---

    /// Retrieves the type of actor.
    ///
    /// The actor type of the actor.
    @(link_name = "PxActor_getType")
    actor_get_type :: proc(self_: ^PxActor) -> PxActorType ---

    /// Retrieves the scene which this actor belongs to.
    ///
    /// Owner Scene. NULL if not part of a scene.
    @(link_name = "PxActor_getScene")
    actor_get_scene :: proc(self_: ^PxActor) -> ^PxScene ---

    /// Sets a name string for the object that can be retrieved with getName().
    ///
    /// This is for debugging and is not used by the SDK. The string is not copied by the SDK,
    /// only the pointer is stored.
    ///
    /// Default:
    /// NULL
    @(link_name = "PxActor_setName_mut")
    actor_set_name_mut :: proc(self_: ^PxActor, name: ^_c.char) ---

    /// Retrieves the name string set with setName().
    ///
    /// Name string associated with object.
    @(link_name = "PxActor_getName")
    actor_get_name :: proc(self_: ^PxActor) -> ^_c.char ---

    /// Retrieves the axis aligned bounding box enclosing the actor.
    ///
    /// It is not allowed to use this method while the simulation is running (except during PxScene::collide(),
    /// in PxContactModifyCallback or in contact report callbacks).
    ///
    /// The actor's bounding box.
    @(link_name = "PxActor_getWorldBounds")
    actor_get_world_bounds :: proc(self_: ^PxActor, inflation: _c.float) -> PxBounds3 ---

    /// Raises or clears a particular actor flag.
    ///
    /// See the list of flags [`PxActorFlag`]
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the actor up automatically.
    @(link_name = "PxActor_setActorFlag_mut")
    actor_set_actor_flag_mut :: proc(self_: ^PxActor, flag: PxActorFlag, value: _c.bool) ---

    /// Sets the actor flags.
    ///
    /// See the list of flags [`PxActorFlag`]
    @(link_name = "PxActor_setActorFlags_mut")
    actor_set_actor_flags_mut :: proc(self_: ^PxActor, inFlags: PxActorFlags_Set) ---

    /// Reads the PxActor flags.
    ///
    /// See the list of flags [`PxActorFlag`]
    ///
    /// The values of the PxActor flags.
    @(link_name = "PxActor_getActorFlags")
    actor_get_actor_flags :: proc(self_: ^PxActor) -> PxActorFlags_Set ---

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
    actor_set_dominance_group_mut :: proc(self_: ^PxActor, dominanceGroup: _c.uint8_t) ---

    /// Retrieves the value set with setDominanceGroup().
    ///
    /// The dominance group of this actor.
    @(link_name = "PxActor_getDominanceGroup")
    actor_get_dominance_group :: proc(self_: ^PxActor) -> _c.uint8_t ---

    /// Sets the owner client of an actor.
    ///
    /// This cannot be done once the actor has been placed into a scene.
    ///
    /// Default:
    /// PX_DEFAULT_CLIENT
    @(link_name = "PxActor_setOwnerClient_mut")
    actor_set_owner_client_mut :: proc(self_: ^PxActor, inClient: _c.uint8_t) ---

    /// Returns the owner client that was specified at creation time.
    ///
    /// This value cannot be changed once the object is placed into the scene.
    @(link_name = "PxActor_getOwnerClient")
    actor_get_owner_client :: proc(self_: ^PxActor) -> _c.uint8_t ---

    /// Retrieves the aggregate the actor might be a part of.
    ///
    /// The aggregate the actor is a part of, or NULL if the actor does not belong to an aggregate.
    @(link_name = "PxActor_getAggregate")
    actor_get_aggregate :: proc(self_: ^PxActor) -> ^PxAggregate ---

    @(link_name = "phys_PxGetAggregateFilterHint")
    get_aggregate_filter_hint :: proc(type: PxAggregateType, enableSelfCollision: _c.bool) -> _c.uint32_t ---

    @(link_name = "phys_PxGetAggregateSelfCollisionBit")
    get_aggregate_self_collision_bit :: proc(hint: _c.uint32_t) -> _c.uint32_t ---

    @(link_name = "phys_PxGetAggregateType")
    get_aggregate_type :: proc(hint: _c.uint32_t) -> PxAggregateType ---

    /// Deletes the aggregate object.
    ///
    /// Deleting the PxAggregate object does not delete the aggregated actors. If the PxAggregate object
    /// belongs to a scene, the aggregated actors are automatically re-inserted in that scene. If you intend
    /// to delete both the PxAggregate and its actors, it is best to release the actors first, then release
    /// the PxAggregate when it is empty.
    @(link_name = "PxAggregate_release_mut")
    aggregate_release_mut :: proc(self_: ^PxAggregate) ---

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
    aggregate_add_actor_mut :: proc(self_: ^PxAggregate, actor: ^PxActor, bvh: ^PxBVH) -> _c.bool ---

    /// Removes an actor from the aggregate object.
    ///
    /// A warning is output if the incoming actor does not belong to the aggregate. Otherwise the actor is
    /// removed from the aggregate. If the aggregate belongs to a scene, the actor is reinserted in that
    /// scene. If you intend to delete the actor, it is best to call [`PxActor::release`]() directly. That way
    /// the actor will be automatically removed from its aggregate (if any) and not reinserted in a scene.
    @(link_name = "PxAggregate_removeActor_mut")
    aggregate_remove_actor_mut :: proc(self_: ^PxAggregate, actor: ^PxActor) -> _c.bool ---

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
    aggregate_add_articulation_mut :: proc(self_: ^PxAggregate, articulation: ^PxArticulationReducedCoordinate) -> _c.bool ---

    /// Removes an articulation from the aggregate object.
    ///
    /// A warning is output if the incoming articulation does not belong to the aggregate. Otherwise the articulation is
    /// removed from the aggregate. If the aggregate belongs to a scene, the articulation is reinserted in that
    /// scene. If you intend to delete the articulation, it is best to call [`PxArticulationReducedCoordinate::release`]() directly. That way
    /// the articulation will be automatically removed from its aggregate (if any) and not reinserted in a scene.
    @(link_name = "PxAggregate_removeArticulation_mut")
    aggregate_remove_articulation_mut :: proc(self_: ^PxAggregate, articulation: ^PxArticulationReducedCoordinate) -> _c.bool ---

    /// Returns the number of actors contained in the aggregate.
    ///
    /// You can use [`getActors`]() to retrieve the actor pointers.
    ///
    /// Number of actors contained in the aggregate.
    @(link_name = "PxAggregate_getNbActors")
    aggregate_get_nb_actors :: proc(self_: ^PxAggregate) -> _c.uint32_t ---

    /// Retrieves max amount of shapes that can be contained in the aggregate.
    ///
    /// Max shape size.
    @(link_name = "PxAggregate_getMaxNbShapes")
    aggregate_get_max_nb_shapes :: proc(self_: ^PxAggregate) -> _c.uint32_t ---

    /// Retrieve all actors contained in the aggregate.
    ///
    /// You can retrieve the number of actor pointers by calling [`getNbActors`]()
    ///
    /// Number of actor pointers written to the buffer.
    @(link_name = "PxAggregate_getActors")
    aggregate_get_actors :: proc(self_: ^PxAggregate, userBuffer: ^^PxActor, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Retrieves the scene which this aggregate belongs to.
    ///
    /// Owner Scene. NULL if not part of a scene.
    @(link_name = "PxAggregate_getScene_mut")
    aggregate_get_scene_mut :: proc(self_: ^PxAggregate) -> ^PxScene ---

    /// Retrieves aggregate's self-collision flag.
    ///
    /// self-collision flag
    @(link_name = "PxAggregate_getSelfCollision")
    aggregate_get_self_collision :: proc(self_: ^PxAggregate) -> _c.bool ---

    @(link_name = "PxAggregate_getConcreteTypeName")
    aggregate_get_concrete_type_name :: proc(self_: ^PxAggregate) -> ^_c.char ---

    @(link_name = "PxConstraintInvMassScale_new")
    constraint_inv_mass_scale_new :: proc() -> PxConstraintInvMassScale ---

    @(link_name = "PxConstraintInvMassScale_new_1")
    constraint_inv_mass_scale_new_1 :: proc(lin0: _c.float, ang0: _c.float, lin1: _c.float, ang1: _c.float) -> PxConstraintInvMassScale ---

    /// Visualize joint frames
    @(link_name = "PxConstraintVisualizer_visualizeJointFrames_mut")
    constraint_visualizer_visualize_joint_frames_mut :: proc(self_: ^PxConstraintVisualizer, #by_ptr parent: PxTransform, #by_ptr child: PxTransform) ---

    /// Visualize joint linear limit
    @(link_name = "PxConstraintVisualizer_visualizeLinearLimit_mut")
    constraint_visualizer_visualize_linear_limit_mut :: proc(self_: ^PxConstraintVisualizer, #by_ptr t0: PxTransform, #by_ptr t1: PxTransform, value: _c.float, active: _c.bool) ---

    /// Visualize joint angular limit
    @(link_name = "PxConstraintVisualizer_visualizeAngularLimit_mut")
    constraint_visualizer_visualize_angular_limit_mut :: proc(self_: ^PxConstraintVisualizer, #by_ptr t0: PxTransform, lower: _c.float, upper: _c.float, active: _c.bool) ---

    /// Visualize limit cone
    @(link_name = "PxConstraintVisualizer_visualizeLimitCone_mut")
    constraint_visualizer_visualize_limit_cone_mut :: proc(self_: ^PxConstraintVisualizer, #by_ptr t: PxTransform, tanQSwingY: _c.float, tanQSwingZ: _c.float, active: _c.bool) ---

    /// Visualize joint double cone
    @(link_name = "PxConstraintVisualizer_visualizeDoubleCone_mut")
    constraint_visualizer_visualize_double_cone_mut :: proc(self_: ^PxConstraintVisualizer, #by_ptr t: PxTransform, angle: _c.float, active: _c.bool) ---

    /// Visualize line
    @(link_name = "PxConstraintVisualizer_visualizeLine_mut")
    constraint_visualizer_visualize_line_mut :: proc(self_: ^PxConstraintVisualizer, #by_ptr p0: PxVec3, #by_ptr p1: PxVec3, color: _c.uint32_t) ---

    /// Pre-simulation data preparation
    /// when the constraint is marked dirty, this function is called at the start of the simulation
    /// step for the SDK to copy the constraint data block.
    @(link_name = "PxConstraintConnector_prepareData_mut")
    constraint_connector_prepare_data_mut :: proc(self_: ^PxConstraintConnector) -> rawptr ---

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
    constraint_connector_on_constraint_release_mut :: proc(self_: ^PxConstraintConnector) ---

    /// Center-of-mass shift callback
    ///
    /// This function is called by the SDK when the CoM of one of the actors is moved. Since the
    /// API specifies constraint positions relative to actors, and the constraint shader functions
    /// are supplied with coordinates relative to bodies, some synchronization is usually required
    /// when the application moves an object's center of mass.
    @(link_name = "PxConstraintConnector_onComShift_mut")
    constraint_connector_on_com_shift_mut :: proc(self_: ^PxConstraintConnector, actor: _c.uint32_t) ---

    /// Origin shift callback
    ///
    /// This function is called by the SDK when the scene origin gets shifted and allows to adjust
    /// custom data which contains world space transforms.
    ///
    /// If the adjustments affect constraint shader data, it is necessary to call PxConstraint::markDirty()
    /// to make sure that the data gets synced at the beginning of the next simulation step.
    @(link_name = "PxConstraintConnector_onOriginShift_mut")
    constraint_connector_on_origin_shift_mut :: proc(self_: ^PxConstraintConnector, #by_ptr shift: PxVec3) ---

    /// Obtain a reference to a PxBase interface if the constraint has one.
    ///
    /// If the constraint does not implement the PxBase interface, it should return NULL.
    @(link_name = "PxConstraintConnector_getSerializable_mut")
    constraint_connector_get_serializable_mut :: proc(self_: ^PxConstraintConnector) -> ^PxBase ---

    /// Obtain the pointer to the constraint's constant data
    @(link_name = "PxConstraintConnector_getConstantBlock")
    constraint_connector_get_constant_block :: proc(self_: ^PxConstraintConnector) -> rawptr ---

    /// Let the connector know it has been connected to a constraint.
    @(link_name = "PxConstraintConnector_connectToConstraint_mut")
    constraint_connector_connect_to_constraint_mut :: proc(self_: ^PxConstraintConnector, anon_param0: ^PxConstraint) ---

    /// virtual destructor
    @(link_name = "PxConstraintConnector_delete")
    constraint_connector_delete :: proc(self_: ^PxConstraintConnector) ---

    @(link_name = "PxSolverBody_new")
    solver_body_new :: proc() -> PxSolverBody ---

    @(link_name = "PxSolverBodyData_projectVelocity")
    solver_body_data_project_velocity :: proc(self_: ^PxSolverBodyData, #by_ptr lin: PxVec3, #by_ptr ang: PxVec3) -> _c.float ---

    @(link_name = "PxSolverConstraintPrepDesc_delete")
    solver_constraint_prep_desc_delete :: proc(self_: ^PxSolverConstraintPrepDesc) ---

    /// Allocates constraint data. It is the application's responsibility to release this memory after PxSolveConstraints has completed.
    ///
    /// The allocated memory. This address must be 16-byte aligned.
    @(link_name = "PxConstraintAllocator_reserveConstraintData_mut")
    constraint_allocator_reserve_constraint_data_mut :: proc(self_: ^PxConstraintAllocator, byteSize: _c.uint32_t) -> ^_c.uint8_t ---

    /// Allocates friction data. Friction data can be retained by the application for a given pair and provided as an input to PxSolverContactDesc to improve simulation stability.
    /// It is the application's responsibility to release this memory. If this memory is released, the application should ensure it does not pass pointers to this memory to PxSolverContactDesc.
    ///
    /// The allocated memory. This address must be 4-byte aligned.
    @(link_name = "PxConstraintAllocator_reserveFrictionData_mut")
    constraint_allocator_reserve_friction_data_mut :: proc(self_: ^PxConstraintAllocator, byteSize: _c.uint32_t) -> ^_c.uint8_t ---

    @(link_name = "PxConstraintAllocator_delete")
    constraint_allocator_delete :: proc(self_: ^PxConstraintAllocator) ---

    @(link_name = "PxArticulationLimit_new")
    articulation_limit_new :: proc() -> PxArticulationLimit ---

    @(link_name = "PxArticulationLimit_new_1")
    articulation_limit_new_1 :: proc(low_: _c.float, high_: _c.float) -> PxArticulationLimit ---

    @(link_name = "PxArticulationDrive_new")
    articulation_drive_new :: proc() -> PxArticulationDrive ---

    @(link_name = "PxArticulationDrive_new_1")
    articulation_drive_new_1 :: proc(stiffness_: _c.float, damping_: _c.float, maxForce_: _c.float, driveType_: PxArticulationDriveType) -> PxArticulationDrive ---

    @(link_name = "PxTGSSolverBodyVel_projectVelocity")
    t_g_s_solver_body_vel_project_velocity :: proc(self_: ^PxTGSSolverBodyVel, #by_ptr lin: PxVec3, #by_ptr ang: PxVec3) -> _c.float ---

    @(link_name = "PxTGSSolverBodyData_projectVelocity")
    t_g_s_solver_body_data_project_velocity :: proc(self_: ^PxTGSSolverBodyData, #by_ptr linear: PxVec3, #by_ptr angular: PxVec3) -> _c.float ---

    @(link_name = "PxTGSSolverConstraintPrepDesc_delete")
    t_g_s_solver_constraint_prep_desc_delete :: proc(self_: ^PxTGSSolverConstraintPrepDesc) ---

    /// Sets the spring rest length for the sub-tendon from the root to this leaf attachment.
    ///
    /// Setting this on non-leaf attachments has no effect.
    @(link_name = "PxArticulationAttachment_setRestLength_mut")
    articulation_attachment_set_rest_length_mut :: proc(self_: ^PxArticulationAttachment, restLength: _c.float) ---

    /// Gets the spring rest length for the sub-tendon from the root to this leaf attachment.
    ///
    /// The rest length.
    @(link_name = "PxArticulationAttachment_getRestLength")
    articulation_attachment_get_rest_length :: proc(self_: ^PxArticulationAttachment) -> _c.float ---

    /// Sets the low and high limit on the length of the sub-tendon from the root to this leaf attachment.
    ///
    /// Setting this on non-leaf attachments has no effect.
    @(link_name = "PxArticulationAttachment_setLimitParameters_mut")
    articulation_attachment_set_limit_parameters_mut :: proc(self_: ^PxArticulationAttachment, #by_ptr parameters: PxArticulationTendonLimit) ---

    /// Gets the low and high limit on the length of the sub-tendon from the root to this leaf attachment.
    ///
    /// Struct with the low and high limit.
    @(link_name = "PxArticulationAttachment_getLimitParameters")
    articulation_attachment_get_limit_parameters :: proc(self_: ^PxArticulationAttachment) -> PxArticulationTendonLimit ---

    /// Sets the attachment's relative offset in the link actor frame.
    @(link_name = "PxArticulationAttachment_setRelativeOffset_mut")
    articulation_attachment_set_relative_offset_mut :: proc(self_: ^PxArticulationAttachment, #by_ptr offset: PxVec3) ---

    /// Gets the attachment's relative offset in the link actor frame.
    ///
    /// The relative offset in the link actor frame.
    @(link_name = "PxArticulationAttachment_getRelativeOffset")
    articulation_attachment_get_relative_offset :: proc(self_: ^PxArticulationAttachment) -> PxVec3 ---

    /// Sets the attachment coefficient.
    @(link_name = "PxArticulationAttachment_setCoefficient_mut")
    articulation_attachment_set_coefficient_mut :: proc(self_: ^PxArticulationAttachment, coefficient: _c.float) ---

    /// Gets the attachment coefficient.
    ///
    /// The scale that the distance between this attachment and its parent is multiplied by when summing up the spatial tendon's length.
    @(link_name = "PxArticulationAttachment_getCoefficient")
    articulation_attachment_get_coefficient :: proc(self_: ^PxArticulationAttachment) -> _c.float ---

    /// Gets the articulation link.
    ///
    /// The articulation link that this attachment is attached to.
    @(link_name = "PxArticulationAttachment_getLink")
    articulation_attachment_get_link :: proc(self_: ^PxArticulationAttachment) -> ^PxArticulationLink ---

    /// Gets the parent attachment.
    ///
    /// The parent attachment.
    @(link_name = "PxArticulationAttachment_getParent")
    articulation_attachment_get_parent :: proc(self_: ^PxArticulationAttachment) -> ^PxArticulationAttachment ---

    /// Indicates that this attachment is a leaf, and thus defines a sub-tendon from the root to this attachment.
    ///
    /// True: This attachment is a leaf and has zero children; False: Not a leaf.
    @(link_name = "PxArticulationAttachment_isLeaf")
    articulation_attachment_is_leaf :: proc(self_: ^PxArticulationAttachment) -> _c.bool ---

    /// Gets the spatial tendon that the attachment is a part of.
    ///
    /// The tendon.
    @(link_name = "PxArticulationAttachment_getTendon")
    articulation_attachment_get_tendon :: proc(self_: ^PxArticulationAttachment) -> ^PxArticulationSpatialTendon ---

    /// Releases the attachment.
    ///
    /// Releasing the attachment is not allowed while the articulation is in a scene. In order to
    /// release the attachment, remove and then re-add the articulation to the scene.
    @(link_name = "PxArticulationAttachment_release_mut")
    articulation_attachment_release_mut :: proc(self_: ^PxArticulationAttachment) ---

    /// Returns the string name of the dynamic type.
    ///
    /// The string name.
    @(link_name = "PxArticulationAttachment_getConcreteTypeName")
    articulation_attachment_get_concrete_type_name :: proc(self_: ^PxArticulationAttachment) -> ^_c.char ---

    /// Sets the tendon joint coefficient.
    ///
    /// RecipCoefficient is commonly expected to be 1/coefficient, but it can be set to different values to tune behavior; for example, zero can be used to
    /// have a joint axis only participate in the length computation of the tendon, but not have any tendon force applied to it.
    @(link_name = "PxArticulationTendonJoint_setCoefficient_mut")
    articulation_tendon_joint_set_coefficient_mut :: proc(self_: ^PxArticulationTendonJoint, axis: PxArticulationAxis, coefficient: _c.float, recipCoefficient: _c.float) ---

    /// Gets the tendon joint coefficient.
    @(link_name = "PxArticulationTendonJoint_getCoefficient")
    articulation_tendon_joint_get_coefficient :: proc(self_: ^PxArticulationTendonJoint, axis: ^PxArticulationAxis, coefficient: ^_c.float, recipCoefficient: ^_c.float) ---

    /// Gets the articulation link.
    ///
    /// The articulation link (and its incoming joint in particular) that this tendon joint is associated with.
    @(link_name = "PxArticulationTendonJoint_getLink")
    articulation_tendon_joint_get_link :: proc(self_: ^PxArticulationTendonJoint) -> ^PxArticulationLink ---

    /// Gets the parent tendon joint.
    ///
    /// The parent tendon joint.
    @(link_name = "PxArticulationTendonJoint_getParent")
    articulation_tendon_joint_get_parent :: proc(self_: ^PxArticulationTendonJoint) -> ^PxArticulationTendonJoint ---

    /// Gets the tendon that the joint is a part of.
    ///
    /// The tendon.
    @(link_name = "PxArticulationTendonJoint_getTendon")
    articulation_tendon_joint_get_tendon :: proc(self_: ^PxArticulationTendonJoint) -> ^PxArticulationFixedTendon ---

    /// Releases a tendon joint.
    ///
    /// Releasing a tendon joint is not allowed while the articulation is in a scene. In order to
    /// release the joint, remove and then re-add the articulation to the scene.
    @(link_name = "PxArticulationTendonJoint_release_mut")
    articulation_tendon_joint_release_mut :: proc(self_: ^PxArticulationTendonJoint) ---

    /// Returns the string name of the dynamic type.
    ///
    /// The string name.
    @(link_name = "PxArticulationTendonJoint_getConcreteTypeName")
    articulation_tendon_joint_get_concrete_type_name :: proc(self_: ^PxArticulationTendonJoint) -> ^_c.char ---

    /// Sets the spring stiffness term acting on the tendon length.
    @(link_name = "PxArticulationTendon_setStiffness_mut")
    articulation_tendon_set_stiffness_mut :: proc(self_: ^PxArticulationTendon, stiffness: _c.float) ---

    /// Gets the spring stiffness of the tendon.
    ///
    /// The spring stiffness.
    @(link_name = "PxArticulationTendon_getStiffness")
    articulation_tendon_get_stiffness :: proc(self_: ^PxArticulationTendon) -> _c.float ---

    /// Sets the damping term acting both on the tendon length and tendon-length limits.
    @(link_name = "PxArticulationTendon_setDamping_mut")
    articulation_tendon_set_damping_mut :: proc(self_: ^PxArticulationTendon, damping: _c.float) ---

    /// Gets the damping term acting both on the tendon length and tendon-length limits.
    ///
    /// The damping term.
    @(link_name = "PxArticulationTendon_getDamping")
    articulation_tendon_get_damping :: proc(self_: ^PxArticulationTendon) -> _c.float ---

    /// Sets the limit stiffness term acting on the tendon's length limits.
    ///
    /// For spatial tendons, this parameter applies to all its leaf attachments / sub-tendons.
    @(link_name = "PxArticulationTendon_setLimitStiffness_mut")
    articulation_tendon_set_limit_stiffness_mut :: proc(self_: ^PxArticulationTendon, stiffness: _c.float) ---

    /// Gets the limit stiffness term acting on the tendon's length limits.
    ///
    /// For spatial tendons, this parameter applies to all its leaf attachments / sub-tendons.
    ///
    /// The limit stiffness term.
    @(link_name = "PxArticulationTendon_getLimitStiffness")
    articulation_tendon_get_limit_stiffness :: proc(self_: ^PxArticulationTendon) -> _c.float ---

    /// Sets the length offset term for the tendon.
    ///
    /// An offset defines an amount to be added to the accumulated length computed for the tendon. It allows the
    /// application to actuate the tendon by shortening or lengthening it.
    @(link_name = "PxArticulationTendon_setOffset_mut")
    articulation_tendon_set_offset_mut :: proc(self_: ^PxArticulationTendon, offset: _c.float, autowake: _c.bool) ---

    /// Gets the length offset term for the tendon.
    ///
    /// The offset term.
    @(link_name = "PxArticulationTendon_getOffset")
    articulation_tendon_get_offset :: proc(self_: ^PxArticulationTendon) -> _c.float ---

    /// Gets the articulation that the tendon is a part of.
    ///
    /// The articulation.
    @(link_name = "PxArticulationTendon_getArticulation")
    articulation_tendon_get_articulation :: proc(self_: ^PxArticulationTendon) -> ^PxArticulationReducedCoordinate ---

    /// Releases a tendon to remove it from the articulation and free its associated memory.
    ///
    /// When an articulation is released, its attached tendons are automatically released.
    ///
    /// Releasing a tendon is not allowed while the articulation is in a scene. In order to
    /// release the tendon, remove and then re-add the articulation to the scene.
    @(link_name = "PxArticulationTendon_release_mut")
    articulation_tendon_release_mut :: proc(self_: ^PxArticulationTendon) ---

    /// Creates an articulation attachment and adds it to the list of children in the parent attachment.
    ///
    /// Creating an attachment is not allowed while the articulation is in a scene. In order to
    /// add the attachment, remove and then re-add the articulation to the scene.
    ///
    /// The newly-created attachment if creation was successful, otherwise a null pointer.
    @(link_name = "PxArticulationSpatialTendon_createAttachment_mut")
    articulation_spatial_tendon_create_attachment_mut :: proc(self_: ^PxArticulationSpatialTendon, parent: ^PxArticulationAttachment, coefficient: _c.float, relativeOffset: PxVec3, link: ^PxArticulationLink) -> ^PxArticulationAttachment ---

    /// Fills a user-provided buffer of attachment pointers with the set of attachments.
    ///
    /// The number of attachments that were filled into the user buffer.
    @(link_name = "PxArticulationSpatialTendon_getAttachments")
    articulation_spatial_tendon_get_attachments :: proc(self_: ^PxArticulationSpatialTendon, userBuffer: ^^PxArticulationAttachment, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Returns the number of attachments in the tendon.
    ///
    /// The number of attachments.
    @(link_name = "PxArticulationSpatialTendon_getNbAttachments")
    articulation_spatial_tendon_get_nb_attachments :: proc(self_: ^PxArticulationSpatialTendon) -> _c.uint32_t ---

    /// Returns the string name of the dynamic type.
    ///
    /// The string name.
    @(link_name = "PxArticulationSpatialTendon_getConcreteTypeName")
    articulation_spatial_tendon_get_concrete_type_name :: proc(self_: ^PxArticulationSpatialTendon) -> ^_c.char ---

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
    articulation_fixed_tendon_create_tendon_joint_mut :: proc(self_: ^PxArticulationFixedTendon, parent: ^PxArticulationTendonJoint, axis: PxArticulationAxis, coefficient: _c.float, recipCoefficient: _c.float, link: ^PxArticulationLink) -> ^PxArticulationTendonJoint ---

    /// Fills a user-provided buffer of tendon-joint pointers with the set of tendon joints.
    ///
    /// The number of tendon joints filled into the user buffer.
    @(link_name = "PxArticulationFixedTendon_getTendonJoints")
    articulation_fixed_tendon_get_tendon_joints :: proc(self_: ^PxArticulationFixedTendon, userBuffer: ^^PxArticulationTendonJoint, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Returns the number of tendon joints in the tendon.
    ///
    /// The number of tendon joints.
    @(link_name = "PxArticulationFixedTendon_getNbTendonJoints")
    articulation_fixed_tendon_get_nb_tendon_joints :: proc(self_: ^PxArticulationFixedTendon) -> _c.uint32_t ---

    /// Sets the spring rest length of the tendon.
    ///
    /// The accumulated "length" of a fixed tendon is a linear combination of the joint axis positions that the tendon is
    /// associated with, scaled by the respective tendon joints' coefficients. As such, when the joint positions of all
    /// joints are zero, the accumulated length of a fixed tendon is zero.
    ///
    /// The spring of the tendon is not exerting any force on the articulation when the rest length is equal to the
    /// tendon's accumulated length plus the tendon offset.
    @(link_name = "PxArticulationFixedTendon_setRestLength_mut")
    articulation_fixed_tendon_set_rest_length_mut :: proc(self_: ^PxArticulationFixedTendon, restLength: _c.float) ---

    /// Gets the spring rest length of the tendon.
    ///
    /// The spring rest length of the tendon.
    @(link_name = "PxArticulationFixedTendon_getRestLength")
    articulation_fixed_tendon_get_rest_length :: proc(self_: ^PxArticulationFixedTendon) -> _c.float ---

    /// Sets the low and high limit on the length of the tendon.
    ///
    /// The limits, together with the damping and limit stiffness parameters, act on the accumulated length of the tendon.
    @(link_name = "PxArticulationFixedTendon_setLimitParameters_mut")
    articulation_fixed_tendon_set_limit_parameters_mut :: proc(self_: ^PxArticulationFixedTendon, #by_ptr parameter: PxArticulationTendonLimit) ---

    /// Gets the low and high limit on the length of the tendon.
    ///
    /// Struct with the low and high limit.
    @(link_name = "PxArticulationFixedTendon_getLimitParameters")
    articulation_fixed_tendon_get_limit_parameters :: proc(self_: ^PxArticulationFixedTendon) -> PxArticulationTendonLimit ---

    /// Returns the string name of the dynamic type.
    ///
    /// The string name.
    @(link_name = "PxArticulationFixedTendon_getConcreteTypeName")
    articulation_fixed_tendon_get_concrete_type_name :: proc(self_: ^PxArticulationFixedTendon) -> ^_c.char ---

    @(link_name = "PxArticulationCache_new")
    articulation_cache_new :: proc() -> PxArticulationCache ---

    /// Releases an articulation cache.
    @(link_name = "PxArticulationCache_release_mut")
    articulation_cache_release_mut :: proc(self_: ^PxArticulationCache) ---

    /// Releases the sensor.
    ///
    /// Releasing a sensor is not allowed while the articulation is in a scene. In order to
    /// release a sensor, remove and then re-add the articulation to the scene.
    @(link_name = "PxArticulationSensor_release_mut")
    articulation_sensor_release_mut :: proc(self_: ^PxArticulationSensor) ---

    /// Returns the spatial force in the local frame of the sensor.
    ///
    /// The spatial force.
    ///
    /// This call is not allowed while the simulation is running except in a split simulation during [`PxScene::collide`]() and up to #PxScene::advance(),
    /// and in PxContactModifyCallback or in contact report callbacks.
    @(link_name = "PxArticulationSensor_getForces")
    articulation_sensor_get_forces :: proc(self_: ^PxArticulationSensor) -> PxSpatialForce ---

    /// Returns the relative pose between this sensor and the body frame of the link that the sensor is attached to.
    ///
    /// The link body frame is at the center of mass and aligned with the principal axes of inertia, see PxRigidBody::getCMassLocalPose.
    ///
    /// The transform link body frame -> sensor frame.
    @(link_name = "PxArticulationSensor_getRelativePose")
    articulation_sensor_get_relative_pose :: proc(self_: ^PxArticulationSensor) -> PxTransform ---

    /// Sets the relative pose between this sensor and the body frame of the link that the sensor is attached to.
    ///
    /// The link body frame is at the center of mass and aligned with the principal axes of inertia, see PxRigidBody::getCMassLocalPose.
    ///
    /// Setting the sensor relative pose is not allowed while the articulation is in a scene. In order to
    /// set the pose, remove and then re-add the articulation to the scene.
    @(link_name = "PxArticulationSensor_setRelativePose_mut")
    articulation_sensor_set_relative_pose_mut :: proc(self_: ^PxArticulationSensor, #by_ptr pose: PxTransform) ---

    /// Returns the link that this sensor is attached to.
    ///
    /// A pointer to the link.
    @(link_name = "PxArticulationSensor_getLink")
    articulation_sensor_get_link :: proc(self_: ^PxArticulationSensor) -> ^PxArticulationLink ---

    /// Returns the index of this sensor inside the articulation.
    ///
    /// The return value is only valid for sensors attached to articulations that are in a scene.
    ///
    /// The low-level index, or 0xFFFFFFFF if the articulation is not in a scene.
    @(link_name = "PxArticulationSensor_getIndex")
    articulation_sensor_get_index :: proc(self_: ^PxArticulationSensor) -> _c.uint32_t ---

    /// Returns the articulation that this sensor is part of.
    ///
    /// A pointer to the articulation.
    @(link_name = "PxArticulationSensor_getArticulation")
    articulation_sensor_get_articulation :: proc(self_: ^PxArticulationSensor) -> ^PxArticulationReducedCoordinate ---

    /// Returns the sensor's flags.
    ///
    /// The current set of flags of the sensor.
    @(link_name = "PxArticulationSensor_getFlags")
    articulation_sensor_get_flags :: proc(self_: ^PxArticulationSensor) -> PxArticulationSensorFlags_Set ---

    /// Sets a flag of the sensor.
    ///
    /// Setting the sensor flags is not allowed while the articulation is in a scene. In order to
    /// set the flags, remove and then re-add the articulation to the scene.
    @(link_name = "PxArticulationSensor_setFlag_mut")
    articulation_sensor_set_flag_mut :: proc(self_: ^PxArticulationSensor, flag: PxArticulationSensorFlag, enabled: _c.bool) ---

    /// Returns the string name of the dynamic type.
    ///
    /// The string name.
    @(link_name = "PxArticulationSensor_getConcreteTypeName")
    articulation_sensor_get_concrete_type_name :: proc(self_: ^PxArticulationSensor) -> ^_c.char ---

    /// Returns the scene which this articulation belongs to.
    ///
    /// Owner Scene. NULL if not part of a scene.
    @(link_name = "PxArticulationReducedCoordinate_getScene")
    articulation_reduced_coordinate_get_scene :: proc(self_: ^PxArticulationReducedCoordinate) -> ^PxScene ---

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
    articulation_reduced_coordinate_set_solver_iteration_counts_mut :: proc(self_: ^PxArticulationReducedCoordinate, minPositionIters: _c.uint32_t, minVelocityIters: _c.uint32_t) ---

    /// Returns the solver iteration counts.
    @(link_name = "PxArticulationReducedCoordinate_getSolverIterationCounts")
    articulation_reduced_coordinate_get_solver_iteration_counts :: proc(self_: ^PxArticulationReducedCoordinate, minPositionIters: ^_c.uint32_t, minVelocityIters: ^_c.uint32_t) ---

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
    articulation_reduced_coordinate_is_sleeping :: proc(self_: ^PxArticulationReducedCoordinate) -> _c.bool ---

    /// Sets the mass-normalized energy threshold below which the articulation may go to sleep.
    ///
    /// The articulation will sleep if the energy of each link is below this threshold.
    ///
    /// This call may not be made during simulation.
    @(link_name = "PxArticulationReducedCoordinate_setSleepThreshold_mut")
    articulation_reduced_coordinate_set_sleep_threshold_mut :: proc(self_: ^PxArticulationReducedCoordinate, threshold: _c.float) ---

    /// Returns the mass-normalized energy below which the articulation may go to sleep.
    ///
    /// The energy threshold for sleeping.
    @(link_name = "PxArticulationReducedCoordinate_getSleepThreshold")
    articulation_reduced_coordinate_get_sleep_threshold :: proc(self_: ^PxArticulationReducedCoordinate) -> _c.float ---

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
    articulation_reduced_coordinate_set_stabilization_threshold_mut :: proc(self_: ^PxArticulationReducedCoordinate, threshold: _c.float) ---

    /// Returns the mass-normalized kinetic energy below which the articulation may participate in stabilization.
    ///
    /// Articulations whose kinetic energy divided by their mass is above this threshold will not participate in stabilization.
    ///
    /// The energy threshold for participating in stabilization.
    @(link_name = "PxArticulationReducedCoordinate_getStabilizationThreshold")
    articulation_reduced_coordinate_get_stabilization_threshold :: proc(self_: ^PxArticulationReducedCoordinate) -> _c.float ---

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
    articulation_reduced_coordinate_set_wake_counter_mut :: proc(self_: ^PxArticulationReducedCoordinate, wakeCounterValue: _c.float) ---

    /// Returns the wake counter of the articulation in seconds.
    ///
    /// The wake counter of the articulation in seconds.
    ///
    /// This call may not be made during simulation, except in a split simulation in-between [`PxScene::fetchCollision`] and #PxScene::advance.
    @(link_name = "PxArticulationReducedCoordinate_getWakeCounter")
    articulation_reduced_coordinate_get_wake_counter :: proc(self_: ^PxArticulationReducedCoordinate) -> _c.float ---

    /// Wakes up the articulation if it is sleeping.
    ///
    /// - The articulation will get woken up and might cause other touching objects to wake up as well during the next simulation step.
    /// - This will set the wake counter of the articulation to the value specified in [`PxSceneDesc::wakeCounterResetValue`].
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation,
    /// except in a split simulation in-between [`PxScene::fetchCollision`] and #PxScene::advance.
    @(link_name = "PxArticulationReducedCoordinate_wakeUp_mut")
    articulation_reduced_coordinate_wake_up_mut :: proc(self_: ^PxArticulationReducedCoordinate) ---

    /// Forces the articulation to sleep.
    ///
    /// - The articulation will stay asleep during the next simulation step if not touched by another non-sleeping actor.
    /// - This will set any applied force, the velocity, and the wake counter of all bodies in the articulation to zero.
    ///
    /// This call may not be made during simulation, and may only be made on articulations that are in a scene.
    @(link_name = "PxArticulationReducedCoordinate_putToSleep_mut")
    articulation_reduced_coordinate_put_to_sleep_mut :: proc(self_: ^PxArticulationReducedCoordinate) ---

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
    articulation_reduced_coordinate_set_max_c_o_m_linear_velocity_mut :: proc(self_: ^PxArticulationReducedCoordinate, maxLinearVelocity: _c.float) ---

    /// Gets the limit on the magnitude of the linear velocity of the articulation's center of mass.
    ///
    /// The maximal linear velocity magnitude.
    @(link_name = "PxArticulationReducedCoordinate_getMaxCOMLinearVelocity")
    articulation_reduced_coordinate_get_max_c_o_m_linear_velocity :: proc(self_: ^PxArticulationReducedCoordinate) -> _c.float ---

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
    articulation_reduced_coordinate_set_max_c_o_m_angular_velocity_mut :: proc(self_: ^PxArticulationReducedCoordinate, maxAngularVelocity: _c.float) ---

    /// Gets the limit on the magnitude of the angular velocity at the articulation's center of mass.
    ///
    /// The maximal angular velocity magnitude.
    @(link_name = "PxArticulationReducedCoordinate_getMaxCOMAngularVelocity")
    articulation_reduced_coordinate_get_max_c_o_m_angular_velocity :: proc(self_: ^PxArticulationReducedCoordinate) -> _c.float ---

    /// Adds a link to the articulation with default attribute values.
    ///
    /// The new link, or NULL if the link cannot be created.
    ///
    /// Creating a link is not allowed while the articulation is in a scene. In order to add a link,
    /// remove and then re-add the articulation to the scene.
    @(link_name = "PxArticulationReducedCoordinate_createLink_mut")
    articulation_reduced_coordinate_create_link_mut :: proc(self_: ^PxArticulationReducedCoordinate, parent: ^PxArticulationLink, #by_ptr pose: PxTransform) -> ^PxArticulationLink ---

    /// Releases the articulation, and all its links and corresponding joints.
    ///
    /// Attached sensors and tendons are released automatically when the articulation is released.
    ///
    /// This call may not be made during simulation.
    @(link_name = "PxArticulationReducedCoordinate_release_mut")
    articulation_reduced_coordinate_release_mut :: proc(self_: ^PxArticulationReducedCoordinate) ---

    /// Returns the number of links in the articulation.
    ///
    /// The number of links.
    @(link_name = "PxArticulationReducedCoordinate_getNbLinks")
    articulation_reduced_coordinate_get_nb_links :: proc(self_: ^PxArticulationReducedCoordinate) -> _c.uint32_t ---

    /// Returns the set of links in the articulation in the order that they were added to the articulation using createLink.
    ///
    /// The number of links written into the buffer.
    @(link_name = "PxArticulationReducedCoordinate_getLinks")
    articulation_reduced_coordinate_get_links :: proc(self_: ^PxArticulationReducedCoordinate, userBuffer: ^^PxArticulationLink, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Returns the number of shapes in the articulation.
    ///
    /// The number of shapes.
    @(link_name = "PxArticulationReducedCoordinate_getNbShapes")
    articulation_reduced_coordinate_get_nb_shapes :: proc(self_: ^PxArticulationReducedCoordinate) -> _c.uint32_t ---

    /// Sets a name string for the articulation that can be retrieved with getName().
    ///
    /// This is for debugging and is not used by the SDK. The string is not copied by the SDK,
    /// only the pointer is stored.
    @(link_name = "PxArticulationReducedCoordinate_setName_mut")
    articulation_reduced_coordinate_set_name_mut :: proc(self_: ^PxArticulationReducedCoordinate, name: ^_c.char) ---

    /// Returns the name string set with setName().
    ///
    /// Name string associated with the articulation.
    @(link_name = "PxArticulationReducedCoordinate_getName")
    articulation_reduced_coordinate_get_name :: proc(self_: ^PxArticulationReducedCoordinate) -> ^_c.char ---

    /// Returns the axis-aligned bounding box enclosing the articulation.
    ///
    /// The articulation's bounding box.
    ///
    /// It is not allowed to use this method while the simulation is running, except in a split simulation
    /// during [`PxScene::collide`]() and up to #PxScene::advance(), and in PxContactModifyCallback or in contact report callbacks.
    @(link_name = "PxArticulationReducedCoordinate_getWorldBounds")
    articulation_reduced_coordinate_get_world_bounds :: proc(self_: ^PxArticulationReducedCoordinate, inflation: _c.float) -> PxBounds3 ---

    /// Returns the aggregate the articulation might be a part of.
    ///
    /// The aggregate the articulation is a part of, or NULL if the articulation does not belong to an aggregate.
    @(link_name = "PxArticulationReducedCoordinate_getAggregate")
    articulation_reduced_coordinate_get_aggregate :: proc(self_: ^PxArticulationReducedCoordinate) -> ^PxAggregate ---

    /// Sets flags on the articulation.
    ///
    /// This call may not be made during simulation.
    @(link_name = "PxArticulationReducedCoordinate_setArticulationFlags_mut")
    articulation_reduced_coordinate_set_articulation_flags_mut :: proc(self_: ^PxArticulationReducedCoordinate, flags: PxArticulationFlags_Set) ---

    /// Raises or clears a flag on the articulation.
    ///
    /// This call may not be made during simulation.
    @(link_name = "PxArticulationReducedCoordinate_setArticulationFlag_mut")
    articulation_reduced_coordinate_set_articulation_flag_mut :: proc(self_: ^PxArticulationReducedCoordinate, flag: PxArticulationFlag, value: _c.bool) ---

    /// Returns the articulation's flags.
    ///
    /// The flags.
    @(link_name = "PxArticulationReducedCoordinate_getArticulationFlags")
    articulation_reduced_coordinate_get_articulation_flags :: proc(self_: ^PxArticulationReducedCoordinate) -> PxArticulationFlags_Set ---

    /// Returns the total number of joint degrees-of-freedom (DOFs) of the articulation.
    ///
    /// - The six DOFs of the base of a floating-base articulation are not included in this count.
    /// - Example: Both a fixed-base and a floating-base double-pendulum with two revolute joints will have getDofs() == 2.
    /// - The return value is only valid for articulations that are in a scene.
    ///
    /// The number of joint DOFs, or 0xFFFFFFFF if the articulation is not in a scene.
    @(link_name = "PxArticulationReducedCoordinate_getDofs")
    articulation_reduced_coordinate_get_dofs :: proc(self_: ^PxArticulationReducedCoordinate) -> _c.uint32_t ---

    /// Creates an articulation cache that can be used to read and write internal articulation data.
    ///
    /// - When the structure of the articulation changes (e.g. adding a link or sensor) after the cache was created,
    /// the cache needs to be released and recreated.
    /// - Free the memory allocated for the cache by calling the release() method on the cache.
    /// - Caches can only be created by articulations that are in a scene.
    ///
    /// The cache, or NULL if the articulation is not in a scene.
    @(link_name = "PxArticulationReducedCoordinate_createCache")
    articulation_reduced_coordinate_create_cache :: proc(self_: ^PxArticulationReducedCoordinate) -> ^PxArticulationCache ---

    /// Returns the size of the articulation cache in bytes.
    ///
    /// - The size does not include: the user-allocated memory for the coefficient matrix or lambda values;
    /// the scratch-related memory/members; and the cache version. See comment in [`PxArticulationCache`].
    /// - The return value is only valid for articulations that are in a scene.
    ///
    /// The byte size of the cache, or 0xFFFFFFFF if the articulation is not in a scene.
    @(link_name = "PxArticulationReducedCoordinate_getCacheDataSize")
    articulation_reduced_coordinate_get_cache_data_size :: proc(self_: ^PxArticulationReducedCoordinate) -> _c.uint32_t ---

    /// Zeroes all data in the articulation cache, except user-provided and scratch memory, and cache version.
    ///
    /// This call may only be made on articulations that are in a scene.
    @(link_name = "PxArticulationReducedCoordinate_zeroCache")
    articulation_reduced_coordinate_zero_cache :: proc(self_: ^PxArticulationReducedCoordinate, cache: ^PxArticulationCache) ---

    /// Applies the data in the cache to the articulation.
    ///
    /// This call wakes the articulation if it is sleeping, and the autowake parameter is true (default) or:
    /// - a nonzero joint velocity is applied or
    /// - a nonzero joint force is applied or
    /// - a nonzero root velocity is applied
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
    @(link_name = "PxArticulationReducedCoordinate_applyCache_mut")
    articulation_reduced_coordinate_apply_cache_mut :: proc(self_: ^PxArticulationReducedCoordinate, cache: ^PxArticulationCache, flags: PxArticulationCacheFlags_Set, autowake: _c.bool) ---

    /// Copies internal data of the articulation to the cache.
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
    @(link_name = "PxArticulationReducedCoordinate_copyInternalStateToCache")
    articulation_reduced_coordinate_copy_internal_state_to_cache :: proc(self_: ^PxArticulationReducedCoordinate, cache: ^PxArticulationCache, flags: PxArticulationCacheFlags_Set) ---

    /// Converts maximal-coordinate joint DOF data to reduced coordinates.
    ///
    /// - Indexing into the maximal joint DOF data is via the link's low-level index minus 1 (the root link is not included).
    /// - The reduced-coordinate data follows the cache indexing convention, see PxArticulationCache::jointVelocity.
    ///
    /// The articulation must be in a scene.
    @(link_name = "PxArticulationReducedCoordinate_packJointData")
    articulation_reduced_coordinate_pack_joint_data :: proc(self_: ^PxArticulationReducedCoordinate, maximum: ^_c.float, reduced: ^_c.float) ---

    /// Converts reduced-coordinate joint DOF data to maximal coordinates.
    ///
    /// - Indexing into the maximal joint DOF data is via the link's low-level index minus 1 (the root link is not included).
    /// - The reduced-coordinate data follows the cache indexing convention, see PxArticulationCache::jointVelocity.
    ///
    /// The articulation must be in a scene.
    @(link_name = "PxArticulationReducedCoordinate_unpackJointData")
    articulation_reduced_coordinate_unpack_joint_data :: proc(self_: ^PxArticulationReducedCoordinate, reduced: ^_c.float, maximum: ^_c.float) ---

    /// Prepares common articulation data based on articulation pose for inverse dynamics calculations.
    ///
    /// Usage:
    /// 1. Set articulation pose (joint positions and base transform) via articulation cache and applyCache().
    /// 1. Call commonInit.
    /// 1. Call inverse dynamics computation method.
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
    @(link_name = "PxArticulationReducedCoordinate_commonInit")
    articulation_reduced_coordinate_common_init :: proc(self_: ^PxArticulationReducedCoordinate) ---

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
    articulation_reduced_coordinate_compute_generalized_gravity_force :: proc(self_: ^PxArticulationReducedCoordinate, cache: ^PxArticulationCache) ---

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
    articulation_reduced_coordinate_compute_coriolis_and_centrifugal_force :: proc(self_: ^PxArticulationReducedCoordinate, cache: ^PxArticulationCache) ---

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
    articulation_reduced_coordinate_compute_generalized_external_force :: proc(self_: ^PxArticulationReducedCoordinate, cache: ^PxArticulationCache) ---

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
    articulation_reduced_coordinate_compute_joint_acceleration :: proc(self_: ^PxArticulationReducedCoordinate, cache: ^PxArticulationCache) ---

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
    articulation_reduced_coordinate_compute_joint_force :: proc(self_: ^PxArticulationReducedCoordinate, cache: ^PxArticulationCache) ---

    /// Compute the dense Jacobian for the articulation in world space, including the DOFs of a potentially floating base.
    ///
    /// This computes the dense representation of an inherently sparse matrix. Multiplication with this matrix maps
    /// joint space velocities to world-space linear and angular (i.e. spatial) velocities of the centers of mass of the links.
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
    @(link_name = "PxArticulationReducedCoordinate_computeDenseJacobian")
    articulation_reduced_coordinate_compute_dense_jacobian :: proc(self_: ^PxArticulationReducedCoordinate, cache: ^PxArticulationCache, nRows: ^_c.uint32_t, nCols: ^_c.uint32_t) ---

    /// Computes the coefficient matrix for contact forces.
    ///
    /// - The matrix dimension is getCoefficientMatrixSize() = getDofs() * getNbLoopJoints(), and the DOF (column) indexing follows the internal DOF order, see PxArticulationCache::jointVelocity.
    /// - Each column in the matrix is the joint forces effected by a contact based on impulse strength 1.
    /// - The user must allocate memory for PxArticulationCache::coefficientMatrix where the required size of the PxReal array is equal to getCoefficientMatrixSize().
    /// - commonInit() must be called before the computation, and after setting the articulation pose via applyCache().
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
    @(link_name = "PxArticulationReducedCoordinate_computeCoefficientMatrix")
    articulation_reduced_coordinate_compute_coefficient_matrix :: proc(self_: ^PxArticulationReducedCoordinate, cache: ^PxArticulationCache) ---

    /// Computes the lambda values when the test impulse is 1.
    ///
    /// - The user must allocate memory for PxArticulationCache::lambda where the required size of the PxReal array is equal to getNbLoopJoints().
    /// - commonInit() must be called before the computation, and after setting the articulation pose via applyCache().
    ///
    /// True if convergence was achieved within maxIter; False if convergence was not achieved or the operation failed otherwise.
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
    @(link_name = "PxArticulationReducedCoordinate_computeLambda")
    articulation_reduced_coordinate_compute_lambda :: proc(self_: ^PxArticulationReducedCoordinate, cache: ^PxArticulationCache, initialState: ^PxArticulationCache, jointTorque: ^_c.float, maxIter: _c.uint32_t) -> _c.bool ---

    /// Compute the joint-space inertia matrix that maps joint accelerations to joint forces: forces = M * accelerations.
    ///
    /// - Inputs - Articulation pose (joint positions and base transform).
    /// - Outputs - Mass matrix (in cache).
    ///
    /// commonInit() must be called before the computation, and after setting the articulation pose via applyCache().
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
    @(link_name = "PxArticulationReducedCoordinate_computeGeneralizedMassMatrix")
    articulation_reduced_coordinate_compute_generalized_mass_matrix :: proc(self_: ^PxArticulationReducedCoordinate, cache: ^PxArticulationCache) ---

    /// Adds a loop joint to the articulation system for inverse dynamics.
    ///
    /// This call may not be made during simulation.
    @(link_name = "PxArticulationReducedCoordinate_addLoopJoint_mut")
    articulation_reduced_coordinate_add_loop_joint_mut :: proc(self_: ^PxArticulationReducedCoordinate, joint: ^PxConstraint) ---

    /// Removes a loop joint from the articulation for inverse dynamics.
    ///
    /// This call may not be made during simulation.
    @(link_name = "PxArticulationReducedCoordinate_removeLoopJoint_mut")
    articulation_reduced_coordinate_remove_loop_joint_mut :: proc(self_: ^PxArticulationReducedCoordinate, joint: ^PxConstraint) ---

    /// Returns the number of loop joints in the articulation for inverse dynamics.
    ///
    /// The number of loop joints.
    @(link_name = "PxArticulationReducedCoordinate_getNbLoopJoints")
    articulation_reduced_coordinate_get_nb_loop_joints :: proc(self_: ^PxArticulationReducedCoordinate) -> _c.uint32_t ---

    /// Returns the set of loop constraints (i.e. joints) in the articulation.
    ///
    /// The number of constraints written into the buffer.
    @(link_name = "PxArticulationReducedCoordinate_getLoopJoints")
    articulation_reduced_coordinate_get_loop_joints :: proc(self_: ^PxArticulationReducedCoordinate, userBuffer: ^^PxConstraint, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Returns the required size of the coefficient matrix in the articulation.
    ///
    /// Size of the coefficient matrix (equal to getDofs() * getNbLoopJoints()).
    ///
    /// This call may only be made on articulations that are in a scene.
    @(link_name = "PxArticulationReducedCoordinate_getCoefficientMatrixSize")
    articulation_reduced_coordinate_get_coefficient_matrix_size :: proc(self_: ^PxArticulationReducedCoordinate) -> _c.uint32_t ---

    /// Sets the root link transform (world to actor frame).
    ///
    /// - For performance, prefer PxArticulationCache::rootLinkData to set the root link transform in a batch articulation state update.
    /// - Use updateKinematic() after all state updates to the articulation via non-cache API such as this method,
    /// in order to update link states for the next simulation frame or querying.
    ///
    /// This call may not be made during simulation.
    @(link_name = "PxArticulationReducedCoordinate_setRootGlobalPose_mut")
    articulation_reduced_coordinate_set_root_global_pose_mut :: proc(self_: ^PxArticulationReducedCoordinate, #by_ptr pose: PxTransform, autowake: _c.bool) ---

    /// Returns the root link transform (world to actor frame).
    ///
    /// For performance, prefer PxArticulationCache::rootLinkData to get the root link transform in a batch query.
    ///
    /// The root link transform.
    ///
    /// This call is not allowed while the simulation is running except in a split simulation during [`PxScene::collide`]() and up to #PxScene::advance(),
    /// and in PxContactModifyCallback or in contact report callbacks.
    @(link_name = "PxArticulationReducedCoordinate_getRootGlobalPose")
    articulation_reduced_coordinate_get_root_global_pose :: proc(self_: ^PxArticulationReducedCoordinate) -> PxTransform ---

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
    articulation_reduced_coordinate_set_root_linear_velocity_mut :: proc(self_: ^PxArticulationReducedCoordinate, #by_ptr linearVelocity: PxVec3, autowake: _c.bool) ---

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
    articulation_reduced_coordinate_get_root_linear_velocity :: proc(self_: ^PxArticulationReducedCoordinate) -> PxVec3 ---

    /// Sets the root link angular velocity.
    ///
    /// - For performance, prefer PxArticulationCache::rootLinkData to set the root link velocity in a batch articulation state update.
    /// - The articulation is woken up if the input velocity is nonzero (ignoring autowake) and the articulation is in a scene.
    /// - Use updateKinematic() after all state updates to the articulation via non-cache API such as this method,
    /// in order to update link states for the next simulation frame or querying.
    ///
    /// This call may not be made during simulation, except in a split simulation in-between [`PxScene::fetchCollision`] and #PxScene::advance.
    @(link_name = "PxArticulationReducedCoordinate_setRootAngularVelocity_mut")
    articulation_reduced_coordinate_set_root_angular_velocity_mut :: proc(self_: ^PxArticulationReducedCoordinate, #by_ptr angularVelocity: PxVec3, autowake: _c.bool) ---

    /// Gets the root link angular velocity.
    ///
    /// For performance, prefer PxArticulationCache::rootLinkData to get the root link velocity in a batch query.
    ///
    /// The root link angular velocity.
    ///
    /// This call is not allowed while the simulation is running except in a split simulation during [`PxScene::collide`]() and up to #PxScene::advance(),
    /// and in PxContactModifyCallback or in contact report callbacks.
    @(link_name = "PxArticulationReducedCoordinate_getRootAngularVelocity")
    articulation_reduced_coordinate_get_root_angular_velocity :: proc(self_: ^PxArticulationReducedCoordinate) -> PxVec3 ---

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
    articulation_reduced_coordinate_get_link_acceleration_mut :: proc(self_: ^PxArticulationReducedCoordinate, linkId: _c.uint32_t) -> PxSpatialVelocity ---

    /// Returns the GPU articulation index.
    ///
    /// The GPU index, or 0xFFFFFFFF if the articulation is not in a scene or PxSceneFlag::eSUPPRESS_READBACK is not set.
    @(link_name = "PxArticulationReducedCoordinate_getGpuArticulationIndex_mut")
    articulation_reduced_coordinate_get_gpu_articulation_index_mut :: proc(self_: ^PxArticulationReducedCoordinate) -> _c.uint32_t ---

    /// Creates a spatial tendon to attach to the articulation with default attribute values.
    ///
    /// The new spatial tendon.
    ///
    /// Creating a spatial tendon is not allowed while the articulation is in a scene. In order to
    /// add the tendon, remove and then re-add the articulation to the scene.
    @(link_name = "PxArticulationReducedCoordinate_createSpatialTendon_mut")
    articulation_reduced_coordinate_create_spatial_tendon_mut :: proc(self_: ^PxArticulationReducedCoordinate) -> ^PxArticulationSpatialTendon ---

    /// Creates a fixed tendon to attach to the articulation with default attribute values.
    ///
    /// The new fixed tendon.
    ///
    /// Creating a fixed tendon is not allowed while the articulation is in a scene. In order to
    /// add the tendon, remove and then re-add the articulation to the scene.
    @(link_name = "PxArticulationReducedCoordinate_createFixedTendon_mut")
    articulation_reduced_coordinate_create_fixed_tendon_mut :: proc(self_: ^PxArticulationReducedCoordinate) -> ^PxArticulationFixedTendon ---

    /// Creates a force sensor attached to a link of the articulation.
    ///
    /// The new sensor.
    ///
    /// Creating a sensor is not allowed while the articulation is in a scene. In order to
    /// add the sensor, remove and then re-add the articulation to the scene.
    @(link_name = "PxArticulationReducedCoordinate_createSensor_mut")
    articulation_reduced_coordinate_create_sensor_mut :: proc(self_: ^PxArticulationReducedCoordinate, link: ^PxArticulationLink, #by_ptr relativePose: PxTransform) -> ^PxArticulationSensor ---

    /// Returns the spatial tendons attached to the articulation.
    ///
    /// The order of the tendons in the buffer is not necessarily identical to the order in which the tendons were added to the articulation.
    ///
    /// The number of tendons written into the buffer.
    @(link_name = "PxArticulationReducedCoordinate_getSpatialTendons")
    articulation_reduced_coordinate_get_spatial_tendons :: proc(self_: ^PxArticulationReducedCoordinate, userBuffer: ^^PxArticulationSpatialTendon, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Returns the number of spatial tendons in the articulation.
    ///
    /// The number of tendons.
    @(link_name = "PxArticulationReducedCoordinate_getNbSpatialTendons_mut")
    articulation_reduced_coordinate_get_nb_spatial_tendons_mut :: proc(self_: ^PxArticulationReducedCoordinate) -> _c.uint32_t ---

    /// Returns the fixed tendons attached to the articulation.
    ///
    /// The order of the tendons in the buffer is not necessarily identical to the order in which the tendons were added to the articulation.
    ///
    /// The number of tendons written into the buffer.
    @(link_name = "PxArticulationReducedCoordinate_getFixedTendons")
    articulation_reduced_coordinate_get_fixed_tendons :: proc(self_: ^PxArticulationReducedCoordinate, userBuffer: ^^PxArticulationFixedTendon, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Returns the number of fixed tendons in the articulation.
    ///
    /// The number of tendons.
    @(link_name = "PxArticulationReducedCoordinate_getNbFixedTendons_mut")
    articulation_reduced_coordinate_get_nb_fixed_tendons_mut :: proc(self_: ^PxArticulationReducedCoordinate) -> _c.uint32_t ---

    /// Returns the sensors attached to the articulation.
    ///
    /// The order of the sensors in the buffer is not necessarily identical to the order in which the sensors were added to the articulation.
    ///
    /// The number of sensors written into the buffer.
    @(link_name = "PxArticulationReducedCoordinate_getSensors")
    articulation_reduced_coordinate_get_sensors :: proc(self_: ^PxArticulationReducedCoordinate, userBuffer: ^^PxArticulationSensor, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Returns the number of sensors in the articulation.
    ///
    /// The number of sensors.
    @(link_name = "PxArticulationReducedCoordinate_getNbSensors_mut")
    articulation_reduced_coordinate_get_nb_sensors_mut :: proc(self_: ^PxArticulationReducedCoordinate) -> _c.uint32_t ---

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
    articulation_reduced_coordinate_update_kinematic_mut :: proc(self_: ^PxArticulationReducedCoordinate, flags: PxArticulationKinematicFlags_Set) ---

    /// Gets the parent articulation link of this joint.
    ///
    /// The parent link.
    @(link_name = "PxArticulationJointReducedCoordinate_getParentArticulationLink")
    articulation_joint_reduced_coordinate_get_parent_articulation_link :: proc(self_: ^PxArticulationJointReducedCoordinate) -> ^PxArticulationLink ---

    /// Sets the joint pose in the parent link actor frame.
    ///
    /// This call is not allowed while the simulation is running.
    @(link_name = "PxArticulationJointReducedCoordinate_setParentPose_mut")
    articulation_joint_reduced_coordinate_set_parent_pose_mut :: proc(self_: ^PxArticulationJointReducedCoordinate, #by_ptr pose: PxTransform) ---

    /// Gets the joint pose in the parent link actor frame.
    ///
    /// The joint pose.
    @(link_name = "PxArticulationJointReducedCoordinate_getParentPose")
    articulation_joint_reduced_coordinate_get_parent_pose :: proc(self_: ^PxArticulationJointReducedCoordinate) -> PxTransform ---

    /// Gets the child articulation link of this joint.
    ///
    /// The child link.
    @(link_name = "PxArticulationJointReducedCoordinate_getChildArticulationLink")
    articulation_joint_reduced_coordinate_get_child_articulation_link :: proc(self_: ^PxArticulationJointReducedCoordinate) -> ^PxArticulationLink ---

    /// Sets the joint pose in the child link actor frame.
    ///
    /// This call is not allowed while the simulation is running.
    @(link_name = "PxArticulationJointReducedCoordinate_setChildPose_mut")
    articulation_joint_reduced_coordinate_set_child_pose_mut :: proc(self_: ^PxArticulationJointReducedCoordinate, #by_ptr pose: PxTransform) ---

    /// Gets the joint pose in the child link actor frame.
    ///
    /// The joint pose.
    @(link_name = "PxArticulationJointReducedCoordinate_getChildPose")
    articulation_joint_reduced_coordinate_get_child_pose :: proc(self_: ^PxArticulationJointReducedCoordinate) -> PxTransform ---

    /// Sets the joint type (e.g. revolute).
    ///
    /// Setting the joint type is not allowed while the articulation is in a scene.
    /// In order to set the joint type, remove and then re-add the articulation to the scene.
    @(link_name = "PxArticulationJointReducedCoordinate_setJointType_mut")
    articulation_joint_reduced_coordinate_set_joint_type_mut :: proc(self_: ^PxArticulationJointReducedCoordinate, jointType: PxArticulationJointType) ---

    /// Gets the joint type.
    ///
    /// The joint type.
    @(link_name = "PxArticulationJointReducedCoordinate_getJointType")
    articulation_joint_reduced_coordinate_get_joint_type :: proc(self_: ^PxArticulationJointReducedCoordinate) -> PxArticulationJointType ---

    /// Sets the joint motion for a given axis.
    ///
    /// Setting the motion of joint axes is not allowed while the articulation is in a scene.
    /// In order to set the motion, remove and then re-add the articulation to the scene.
    @(link_name = "PxArticulationJointReducedCoordinate_setMotion_mut")
    articulation_joint_reduced_coordinate_set_motion_mut :: proc(self_: ^PxArticulationJointReducedCoordinate, axis: PxArticulationAxis, motion: PxArticulationMotion) ---

    /// Returns the joint motion for the given axis.
    ///
    /// The joint motion of the given axis.
    @(link_name = "PxArticulationJointReducedCoordinate_getMotion")
    articulation_joint_reduced_coordinate_get_motion :: proc(self_: ^PxArticulationJointReducedCoordinate, axis: PxArticulationAxis) -> PxArticulationMotion ---

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
    articulation_joint_reduced_coordinate_set_limit_params_mut :: proc(self_: ^PxArticulationJointReducedCoordinate, axis: PxArticulationAxis, #by_ptr limit: PxArticulationLimit) ---

    /// Returns the joint limits for a given axis.
    ///
    /// The joint limits.
    @(link_name = "PxArticulationJointReducedCoordinate_getLimitParams")
    articulation_joint_reduced_coordinate_get_limit_params :: proc(self_: ^PxArticulationJointReducedCoordinate, axis: PxArticulationAxis) -> PxArticulationLimit ---

    /// Configures a joint drive for the given axis.
    ///
    /// See PxArticulationDrive for parameter details; and the manual for further information, and the drives' implicit spring-damper (i.e. PD control) implementation in particular.
    ///
    /// This call is not allowed while the simulation is running.
    @(link_name = "PxArticulationJointReducedCoordinate_setDriveParams_mut")
    articulation_joint_reduced_coordinate_set_drive_params_mut :: proc(self_: ^PxArticulationJointReducedCoordinate, axis: PxArticulationAxis, #by_ptr drive: PxArticulationDrive) ---

    /// Gets the joint drive configuration for the given axis.
    ///
    /// The drive parameters.
    @(link_name = "PxArticulationJointReducedCoordinate_getDriveParams")
    articulation_joint_reduced_coordinate_get_drive_params :: proc(self_: ^PxArticulationJointReducedCoordinate, axis: PxArticulationAxis) -> PxArticulationDrive ---

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
    articulation_joint_reduced_coordinate_set_drive_target_mut :: proc(self_: ^PxArticulationJointReducedCoordinate, axis: PxArticulationAxis, target: _c.float, autowake: _c.bool) ---

    /// Returns the joint drive position target for the given axis.
    ///
    /// The target position.
    @(link_name = "PxArticulationJointReducedCoordinate_getDriveTarget")
    articulation_joint_reduced_coordinate_get_drive_target :: proc(self_: ^PxArticulationJointReducedCoordinate, axis: PxArticulationAxis) -> _c.float ---

    /// Sets the joint drive velocity target for the given axis.
    ///
    /// The target units are linear units (equivalent to scene units) per second for a translational axis, or radians per second for a rotational axis.
    ///
    /// This call is not allowed while the simulation is running.
    @(link_name = "PxArticulationJointReducedCoordinate_setDriveVelocity_mut")
    articulation_joint_reduced_coordinate_set_drive_velocity_mut :: proc(self_: ^PxArticulationJointReducedCoordinate, axis: PxArticulationAxis, targetVel: _c.float, autowake: _c.bool) ---

    /// Returns the joint drive velocity target for the given axis.
    ///
    /// The target velocity.
    @(link_name = "PxArticulationJointReducedCoordinate_getDriveVelocity")
    articulation_joint_reduced_coordinate_get_drive_velocity :: proc(self_: ^PxArticulationJointReducedCoordinate, axis: PxArticulationAxis) -> _c.float ---

    /// Sets the joint armature for the given axis.
    ///
    /// - The armature is directly added to the joint-space spatial inertia of the corresponding axis.
    /// - The armature is in mass units for a prismatic (i.e. linear) joint, and in mass units * (scene linear units)^2 for a rotational joint.
    ///
    /// This call is not allowed while the simulation is running.
    @(link_name = "PxArticulationJointReducedCoordinate_setArmature_mut")
    articulation_joint_reduced_coordinate_set_armature_mut :: proc(self_: ^PxArticulationJointReducedCoordinate, axis: PxArticulationAxis, armature: _c.float) ---

    /// Gets the joint armature for the given axis.
    ///
    /// The armature set on the given axis.
    @(link_name = "PxArticulationJointReducedCoordinate_getArmature")
    articulation_joint_reduced_coordinate_get_armature :: proc(self_: ^PxArticulationJointReducedCoordinate, axis: PxArticulationAxis) -> _c.float ---

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
    articulation_joint_reduced_coordinate_set_friction_coefficient_mut :: proc(self_: ^PxArticulationJointReducedCoordinate, coefficient: _c.float) ---

    /// Gets the joint friction coefficient.
    ///
    /// The joint friction coefficient.
    @(link_name = "PxArticulationJointReducedCoordinate_getFrictionCoefficient")
    articulation_joint_reduced_coordinate_get_friction_coefficient :: proc(self_: ^PxArticulationJointReducedCoordinate) -> _c.float ---

    /// Sets the maximal joint velocity enforced for all axes.
    ///
    /// - The solver will apply appropriate joint-space impulses in order to enforce the per-axis joint-velocity limit.
    /// - The velocity units are linear units (equivalent to scene units) per second for a translational axis, or radians per second for a rotational axis.
    ///
    /// This call is not allowed while the simulation is running.
    @(link_name = "PxArticulationJointReducedCoordinate_setMaxJointVelocity_mut")
    articulation_joint_reduced_coordinate_set_max_joint_velocity_mut :: proc(self_: ^PxArticulationJointReducedCoordinate, maxJointV: _c.float) ---

    /// Gets the maximal joint velocity enforced for all axes.
    ///
    /// The maximal per-axis joint velocity.
    @(link_name = "PxArticulationJointReducedCoordinate_getMaxJointVelocity")
    articulation_joint_reduced_coordinate_get_max_joint_velocity :: proc(self_: ^PxArticulationJointReducedCoordinate) -> _c.float ---

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
    articulation_joint_reduced_coordinate_set_joint_position_mut :: proc(self_: ^PxArticulationJointReducedCoordinate, axis: PxArticulationAxis, jointPos: _c.float) ---

    /// Gets the joint position for the given axis, i.e. joint degree of freedom (DOF).
    ///
    /// For performance, prefer PxArticulationCache::jointPosition to get joint positions in a batch query.
    ///
    /// The joint position in linear units (equivalent to scene units) for a translational axis, or radians for a rotational axis.
    ///
    /// This call is not allowed while the simulation is running except in a split simulation during [`PxScene::collide`]() and up to #PxScene::advance(),
    /// and in PxContactModifyCallback or in contact report callbacks.
    @(link_name = "PxArticulationJointReducedCoordinate_getJointPosition")
    articulation_joint_reduced_coordinate_get_joint_position :: proc(self_: ^PxArticulationJointReducedCoordinate, axis: PxArticulationAxis) -> _c.float ---

    /// Sets the joint velocity for the given axis.
    ///
    /// - For performance, prefer PxArticulationCache::jointVelocity to set joint velocities in a batch articulation state update.
    /// - Use PxArticulationReducedCoordinate::updateKinematic after all state updates to the articulation via non-cache API such as this method,
    /// in order to update link states for the next simulation frame or querying.
    ///
    /// This call is not allowed while the simulation is running.
    @(link_name = "PxArticulationJointReducedCoordinate_setJointVelocity_mut")
    articulation_joint_reduced_coordinate_set_joint_velocity_mut :: proc(self_: ^PxArticulationJointReducedCoordinate, axis: PxArticulationAxis, jointVel: _c.float) ---

    /// Gets the joint velocity for the given axis.
    ///
    /// For performance, prefer PxArticulationCache::jointVelocity to get joint velocities in a batch query.
    ///
    /// The joint velocity in linear units (equivalent to scene units) per second for a translational axis, or radians per second for a rotational axis.
    ///
    /// This call is not allowed while the simulation is running except in a split simulation during [`PxScene::collide`]() and up to #PxScene::advance(),
    /// and in PxContactModifyCallback or in contact report callbacks.
    @(link_name = "PxArticulationJointReducedCoordinate_getJointVelocity")
    articulation_joint_reduced_coordinate_get_joint_velocity :: proc(self_: ^PxArticulationJointReducedCoordinate, axis: PxArticulationAxis) -> _c.float ---

    /// Returns the string name of the dynamic type.
    ///
    /// The string name.
    @(link_name = "PxArticulationJointReducedCoordinate_getConcreteTypeName")
    articulation_joint_reduced_coordinate_get_concrete_type_name :: proc(self_: ^PxArticulationJointReducedCoordinate) -> ^_c.char ---

    /// Decrements the reference count of a shape and releases it if the new reference count is zero.
    ///
    /// Note that in releases prior to PhysX 3.3 this method did not have reference counting semantics and was used to destroy a shape
    /// created with PxActor::createShape(). In PhysX 3.3 and above, this usage is deprecated, instead, use PxRigidActor::detachShape() to detach
    /// a shape from an actor. If the shape to be detached was created with PxActor::createShape(), the actor holds the only counted reference,
    /// and so when the shape is detached it will also be destroyed.
    @(link_name = "PxShape_release_mut")
    shape_release_mut :: proc(self_: ^PxShape) ---

    /// Adjust the geometry of the shape.
    ///
    /// The type of the passed in geometry must match the geometry type of the shape.
    ///
    /// It is not allowed to change the geometry type of a shape.
    ///
    /// This function does not guarantee correct/continuous behavior when objects are resting on top of old or new geometry.
    @(link_name = "PxShape_setGeometry_mut")
    shape_set_geometry_mut :: proc(self_: ^PxShape, geometry: ^PxGeometry) ---

    /// Retrieve a reference to the shape's geometry.
    ///
    /// The returned reference has the same lifetime as the PxShape it comes from.
    ///
    /// Reference to internal PxGeometry object.
    @(link_name = "PxShape_getGeometry")
    shape_get_geometry :: proc(self_: ^PxShape) -> ^PxGeometry ---

    /// Retrieves the actor which this shape is associated with.
    ///
    /// The actor this shape is associated with, if it is an exclusive shape, else NULL
    @(link_name = "PxShape_getActor")
    shape_get_actor :: proc(self_: ^PxShape) -> ^PxRigidActor ---

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
    shape_set_local_pose_mut :: proc(self_: ^PxShape, #by_ptr pose: PxTransform) ---

    /// Retrieves the pose of the shape in actor space, i.e. relative to the actor they are owned by.
    ///
    /// This transformation is identity by default.
    ///
    /// Pose of shape relative to the actor's frame.
    @(link_name = "PxShape_getLocalPose")
    shape_get_local_pose :: proc(self_: ^PxShape) -> PxTransform ---

    /// Sets the user definable collision filter data.
    ///
    /// Sleeping:
    /// Does wake up the actor if the filter data change causes a formerly suppressed
    /// collision pair to be enabled.
    ///
    /// Default:
    /// (0,0,0,0)
    @(link_name = "PxShape_setSimulationFilterData_mut")
    shape_set_simulation_filter_data_mut :: proc(self_: ^PxShape, #by_ptr data: PxFilterData) ---

    /// Retrieves the shape's collision filter data.
    @(link_name = "PxShape_getSimulationFilterData")
    shape_get_simulation_filter_data :: proc(self_: ^PxShape) -> PxFilterData ---

    /// Sets the user definable query filter data.
    ///
    /// Default:
    /// (0,0,0,0)
    @(link_name = "PxShape_setQueryFilterData_mut")
    shape_set_query_filter_data_mut :: proc(self_: ^PxShape, #by_ptr data: PxFilterData) ---

    /// Retrieves the shape's Query filter data.
    @(link_name = "PxShape_getQueryFilterData")
    shape_get_query_filter_data :: proc(self_: ^PxShape) -> PxFilterData ---

    /// Assigns material(s) to the shape. Will remove existing materials from the shape.
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the associated actor up automatically.
    @(link_name = "PxShape_setMaterials_mut")
    shape_set_materials_mut :: proc(self_: ^PxShape, materials: ^^PxMaterial, materialCount: _c.uint16_t) ---

    /// Returns the number of materials assigned to the shape.
    ///
    /// You can use [`getMaterials`]() to retrieve the material pointers.
    ///
    /// Number of materials associated with this shape.
    @(link_name = "PxShape_getNbMaterials")
    shape_get_nb_materials :: proc(self_: ^PxShape) -> _c.uint16_t ---

    /// Retrieve all the material pointers associated with the shape.
    ///
    /// You can retrieve the number of material pointers by calling [`getNbMaterials`]()
    ///
    /// Note: The returned data may contain invalid pointers if you release materials using [`PxMaterial::release`]().
    ///
    /// Number of material pointers written to the buffer.
    @(link_name = "PxShape_getMaterials")
    shape_get_materials :: proc(self_: ^PxShape, userBuffer: ^^PxMaterial, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

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
    shape_get_material_from_internal_face_index :: proc(self_: ^PxShape, faceIndex: _c.uint32_t) -> ^PxBaseMaterial ---

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
    shape_set_contact_offset_mut :: proc(self_: ^PxShape, contactOffset: _c.float) ---

    /// Retrieves the contact offset.
    ///
    /// The contact offset of the shape.
    @(link_name = "PxShape_getContactOffset")
    shape_get_contact_offset :: proc(self_: ^PxShape) -> _c.float ---

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
    shape_set_rest_offset_mut :: proc(self_: ^PxShape, restOffset: _c.float) ---

    /// Retrieves the rest offset.
    ///
    /// The rest offset of the shape.
    @(link_name = "PxShape_getRestOffset")
    shape_get_rest_offset :: proc(self_: ^PxShape) -> _c.float ---

    /// Sets the density used to interact with fluids.
    ///
    /// To be physically accurate, the density of a rigid body should be computed as its mass divided by its volume. To
    /// simplify tuning the interaction of fluid and rigid bodies, the density for fluid can differ from the real density. This
    /// allows to create floating bodies, even if they are supposed to sink with their mass and volume.
    ///
    /// Default:
    /// 800.0f
    @(link_name = "PxShape_setDensityForFluid_mut")
    shape_set_density_for_fluid_mut :: proc(self_: ^PxShape, densityForFluid: _c.float) ---

    /// Retrieves the density used to interact with fluids.
    ///
    /// The density of the body when interacting with fluid.
    @(link_name = "PxShape_getDensityForFluid")
    shape_get_density_for_fluid :: proc(self_: ^PxShape) -> _c.float ---

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
    shape_set_torsional_patch_radius_mut :: proc(self_: ^PxShape, radius: _c.float) ---

    /// Gets torsional patch radius.
    ///
    /// This defines the radius of the contact patch used to apply torsional friction. If the radius is 0, no torsional friction
    /// will be applied. If the radius is > 0, some torsional friction will be applied. This is proportional to the penetration depth
    /// so, if the shapes are separated or penetration is zero, no torsional friction will be applied. It is used to approximate
    /// rotational friction introduced by the compression of contacting surfaces.
    ///
    /// The torsional patch radius of the shape.
    @(link_name = "PxShape_getTorsionalPatchRadius")
    shape_get_torsional_patch_radius :: proc(self_: ^PxShape) -> _c.float ---

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
    shape_set_min_torsional_patch_radius_mut :: proc(self_: ^PxShape, radius: _c.float) ---

    /// Gets minimum torsional patch radius.
    ///
    /// This defines the minimum radius of the contact patch used to apply torsional friction. If the radius is 0, the amount of torsional friction
    /// that will be applied will be entirely dependent on the value of torsionalPatchRadius.
    ///
    /// If the radius is > 0, some torsional friction will be applied regardless of the value of torsionalPatchRadius or the amount of penetration.
    ///
    /// The minimum torsional patch radius of the shape.
    @(link_name = "PxShape_getMinTorsionalPatchRadius")
    shape_get_min_torsional_patch_radius :: proc(self_: ^PxShape) -> _c.float ---

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
    shape_set_flag_mut :: proc(self_: ^PxShape, flag: PxShapeFlag, value: _c.bool) ---

    /// Sets shape flags
    @(link_name = "PxShape_setFlags_mut")
    shape_set_flags_mut :: proc(self_: ^PxShape, inFlags: PxShapeFlags_Set) ---

    /// Retrieves shape flags.
    ///
    /// The values of the shape flags.
    @(link_name = "PxShape_getFlags")
    shape_get_flags :: proc(self_: ^PxShape) -> PxShapeFlags_Set ---

    /// Returns true if the shape is exclusive to an actor.
    @(link_name = "PxShape_isExclusive")
    shape_is_exclusive :: proc(self_: ^PxShape) -> _c.bool ---

    /// Sets a name string for the object that can be retrieved with [`getName`]().
    ///
    /// This is for debugging and is not used by the SDK.
    /// The string is not copied by the SDK, only the pointer is stored.
    ///
    /// Default:
    /// NULL
    @(link_name = "PxShape_setName_mut")
    shape_set_name_mut :: proc(self_: ^PxShape, name: ^_c.char) ---

    /// retrieves the name string set with setName().
    ///
    /// The name associated with the shape.
    @(link_name = "PxShape_getName")
    shape_get_name :: proc(self_: ^PxShape) -> ^_c.char ---

    @(link_name = "PxShape_getConcreteTypeName")
    shape_get_concrete_type_name :: proc(self_: ^PxShape) -> ^_c.char ---

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
    rigid_actor_release_mut :: proc(self_: ^PxRigidActor) ---

    /// Returns the internal actor index.
    ///
    /// This is only defined for actors that have been added to a scene.
    ///
    /// The internal actor index, or 0xffffffff if the actor is not part of a scene.
    @(link_name = "PxRigidActor_getInternalActorIndex")
    rigid_actor_get_internal_actor_index :: proc(self_: ^PxRigidActor) -> _c.uint32_t ---

    /// Retrieves the actors world space transform.
    ///
    /// The getGlobalPose() method retrieves the actor's current actor space to world space transformation.
    ///
    /// It is not allowed to use this method while the simulation is running (except during PxScene::collide(),
    /// in PxContactModifyCallback or in contact report callbacks).
    ///
    /// Global pose of object.
    @(link_name = "PxRigidActor_getGlobalPose")
    rigid_actor_get_global_pose :: proc(self_: ^PxRigidActor) -> PxTransform ---

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
    rigid_actor_set_global_pose_mut :: proc(self_: ^PxRigidActor, #by_ptr pose: PxTransform, autowake: _c.bool) ---

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
    rigid_actor_attach_shape_mut :: proc(self_: ^PxRigidActor, shape: ^PxShape) -> _c.bool ---

    /// Detach a shape from an actor.
    ///
    /// This will also decrement the reference count of the PxShape, and if the reference count is zero, will cause it to be deleted.
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the actor up automatically.
    @(link_name = "PxRigidActor_detachShape_mut")
    rigid_actor_detach_shape_mut :: proc(self_: ^PxRigidActor, shape: ^PxShape, wakeOnLostTouch: _c.bool) ---

    /// Returns the number of shapes assigned to the actor.
    ///
    /// You can use [`getShapes`]() to retrieve the shape pointers.
    ///
    /// Number of shapes associated with this actor.
    @(link_name = "PxRigidActor_getNbShapes")
    rigid_actor_get_nb_shapes :: proc(self_: ^PxRigidActor) -> _c.uint32_t ---

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
    rigid_actor_get_shapes :: proc(self_: ^PxRigidActor, userBuffer: ^^PxShape, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Returns the number of constraint shaders attached to the actor.
    ///
    /// You can use [`getConstraints`]() to retrieve the constraint shader pointers.
    ///
    /// Number of constraint shaders attached to this actor.
    @(link_name = "PxRigidActor_getNbConstraints")
    rigid_actor_get_nb_constraints :: proc(self_: ^PxRigidActor) -> _c.uint32_t ---

    /// Retrieve all the constraint shader pointers belonging to the actor.
    ///
    /// You can retrieve the number of constraint shader pointers by calling [`getNbConstraints`]()
    ///
    /// Note: Removing constraint shaders with [`PxConstraint::release`]() will invalidate the pointer of the released constraint.
    ///
    /// Number of constraint shader pointers written to the buffer.
    @(link_name = "PxRigidActor_getConstraints")
    rigid_actor_get_constraints :: proc(self_: ^PxRigidActor, userBuffer: ^^PxConstraint, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    @(link_name = "PxNodeIndex_new")
    node_index_new :: proc(id: _c.uint32_t, articLinkId: _c.uint32_t) -> PxNodeIndex ---

    @(link_name = "PxNodeIndex_new_1")
    node_index_new_1 :: proc(id: _c.uint32_t) -> PxNodeIndex ---

    @(link_name = "PxNodeIndex_index")
    node_index_index :: proc(self_: ^PxNodeIndex) -> _c.uint32_t ---

    @(link_name = "PxNodeIndex_articulationLinkId")
    node_index_articulation_link_id :: proc(self_: ^PxNodeIndex) -> _c.uint32_t ---

    @(link_name = "PxNodeIndex_isArticulation")
    node_index_is_articulation :: proc(self_: ^PxNodeIndex) -> _c.uint32_t ---

    @(link_name = "PxNodeIndex_isStaticBody")
    node_index_is_static_body :: proc(self_: ^PxNodeIndex) -> _c.bool ---

    @(link_name = "PxNodeIndex_isValid")
    node_index_is_valid :: proc(self_: ^PxNodeIndex) -> _c.bool ---

    @(link_name = "PxNodeIndex_setIndices_mut")
    node_index_set_indices_mut :: proc(self_: ^PxNodeIndex, index: _c.uint32_t, articLinkId: _c.uint32_t) ---

    @(link_name = "PxNodeIndex_setIndices_mut_1")
    node_index_set_indices_mut_1 :: proc(self_: ^PxNodeIndex, index: _c.uint32_t) ---

    @(link_name = "PxNodeIndex_getInd")
    node_index_get_ind :: proc(self_: ^PxNodeIndex) -> _c.uint64_t ---

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
    rigid_body_set_c_mass_local_pose_mut :: proc(self_: ^PxRigidBody, #by_ptr pose: PxTransform) ---

    /// Retrieves the center of mass pose relative to the actor frame.
    ///
    /// The center of mass pose relative to the actor frame.
    @(link_name = "PxRigidBody_getCMassLocalPose")
    rigid_body_get_c_mass_local_pose :: proc(self_: ^PxRigidBody) -> PxTransform ---

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
    rigid_body_set_mass_mut :: proc(self_: ^PxRigidBody, mass: _c.float) ---

    /// Retrieves the mass of the actor.
    ///
    /// A value of 0 is interpreted as infinite mass.
    ///
    /// The mass of this actor.
    @(link_name = "PxRigidBody_getMass")
    rigid_body_get_mass :: proc(self_: ^PxRigidBody) -> _c.float ---

    /// Retrieves the inverse mass of the actor.
    ///
    /// The inverse mass of this actor.
    @(link_name = "PxRigidBody_getInvMass")
    rigid_body_get_inv_mass :: proc(self_: ^PxRigidBody) -> _c.float ---

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
    rigid_body_set_mass_space_inertia_tensor_mut :: proc(self_: ^PxRigidBody, #by_ptr m: PxVec3) ---

    /// Retrieves the diagonal inertia tensor of the actor relative to the mass coordinate frame.
    ///
    /// This method retrieves a mass frame inertia vector.
    ///
    /// The mass space inertia tensor of this actor.
    ///
    /// A value of 0 in an element is interpreted as infinite inertia along that axis.
    @(link_name = "PxRigidBody_getMassSpaceInertiaTensor")
    rigid_body_get_mass_space_inertia_tensor :: proc(self_: ^PxRigidBody) -> PxVec3 ---

    /// Retrieves the diagonal inverse inertia tensor of the actor relative to the mass coordinate frame.
    ///
    /// This method retrieves a mass frame inverse inertia vector.
    ///
    /// A value of 0 in an element is interpreted as infinite inertia along that axis.
    ///
    /// The mass space inverse inertia tensor of this actor.
    @(link_name = "PxRigidBody_getMassSpaceInvInertiaTensor")
    rigid_body_get_mass_space_inv_inertia_tensor :: proc(self_: ^PxRigidBody) -> PxVec3 ---

    /// Sets the linear damping coefficient.
    ///
    /// Zero represents no damping. The damping coefficient must be nonnegative.
    ///
    /// Default:
    /// 0.0
    @(link_name = "PxRigidBody_setLinearDamping_mut")
    rigid_body_set_linear_damping_mut :: proc(self_: ^PxRigidBody, linDamp: _c.float) ---

    /// Retrieves the linear damping coefficient.
    ///
    /// The linear damping coefficient associated with this actor.
    @(link_name = "PxRigidBody_getLinearDamping")
    rigid_body_get_linear_damping :: proc(self_: ^PxRigidBody) -> _c.float ---

    /// Sets the angular damping coefficient.
    ///
    /// Zero represents no damping.
    ///
    /// The angular damping coefficient must be nonnegative.
    ///
    /// Default:
    /// 0.05
    @(link_name = "PxRigidBody_setAngularDamping_mut")
    rigid_body_set_angular_damping_mut :: proc(self_: ^PxRigidBody, angDamp: _c.float) ---

    /// Retrieves the angular damping coefficient.
    ///
    /// The angular damping coefficient associated with this actor.
    @(link_name = "PxRigidBody_getAngularDamping")
    rigid_body_get_angular_damping :: proc(self_: ^PxRigidBody) -> _c.float ---

    /// Retrieves the linear velocity of an actor.
    ///
    /// It is not allowed to use this method while the simulation is running (except during PxScene::collide(),
    /// in PxContactModifyCallback or in contact report callbacks).
    ///
    /// The linear velocity of the actor.
    @(link_name = "PxRigidBody_getLinearVelocity")
    rigid_body_get_linear_velocity :: proc(self_: ^PxRigidBody) -> PxVec3 ---

    /// Retrieves the angular velocity of the actor.
    ///
    /// It is not allowed to use this method while the simulation is running (except during PxScene::collide(),
    /// in PxContactModifyCallback or in contact report callbacks).
    ///
    /// The angular velocity of the actor.
    @(link_name = "PxRigidBody_getAngularVelocity")
    rigid_body_get_angular_velocity :: proc(self_: ^PxRigidBody) -> PxVec3 ---

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
    rigid_body_set_max_linear_velocity_mut :: proc(self_: ^PxRigidBody, maxLinVel: _c.float) ---

    /// Retrieves the maximum angular velocity permitted for this actor.
    ///
    /// The maximum allowed angular velocity for this actor.
    @(link_name = "PxRigidBody_getMaxLinearVelocity")
    rigid_body_get_max_linear_velocity :: proc(self_: ^PxRigidBody) -> _c.float ---

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
    rigid_body_set_max_angular_velocity_mut :: proc(self_: ^PxRigidBody, maxAngVel: _c.float) ---

    /// Retrieves the maximum angular velocity permitted for this actor.
    ///
    /// The maximum allowed angular velocity for this actor.
    @(link_name = "PxRigidBody_getMaxAngularVelocity")
    rigid_body_get_max_angular_velocity :: proc(self_: ^PxRigidBody) -> _c.float ---

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
    rigid_body_add_force_mut :: proc(self_: ^PxRigidBody, #by_ptr force: PxVec3, mode: PxForceMode, autowake: _c.bool) ---

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
    rigid_body_add_torque_mut :: proc(self_: ^PxRigidBody, #by_ptr torque: PxVec3, mode: PxForceMode, autowake: _c.bool) ---

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
    rigid_body_clear_force_mut :: proc(self_: ^PxRigidBody, mode: PxForceMode) ---

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
    rigid_body_clear_torque_mut :: proc(self_: ^PxRigidBody, mode: PxForceMode) ---

    /// Sets the impulsive force and torque defined in the global coordinate frame to the actor.
    ///
    /// ::PxForceMode determines if the cleared torque is to be conventional or impulsive.
    ///
    /// The force modes PxForceMode::eIMPULSE and PxForceMode::eVELOCITY_CHANGE can not be applied to articulation links.
    ///
    /// It is invalid to use this method if the actor has not been added to a scene already or if PxActorFlag::eDISABLE_SIMULATION is set.
    @(link_name = "PxRigidBody_setForceAndTorque_mut")
    rigid_body_set_force_and_torque_mut :: proc(self_: ^PxRigidBody, #by_ptr force: PxVec3, #by_ptr torque: PxVec3, mode: PxForceMode) ---

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
    rigid_body_set_rigid_body_flag_mut :: proc(self_: ^PxRigidBody, flag: PxRigidBodyFlag, value: _c.bool) ---

    @(link_name = "PxRigidBody_setRigidBodyFlags_mut")
    rigid_body_set_rigid_body_flags_mut :: proc(self_: ^PxRigidBody, inFlags: PxRigidBodyFlags_Set) ---

    /// Reads the PxRigidBody flags.
    ///
    /// See the list of flags [`PxRigidBodyFlag`]
    ///
    /// The values of the PxRigidBody flags.
    @(link_name = "PxRigidBody_getRigidBodyFlags")
    rigid_body_get_rigid_body_flags :: proc(self_: ^PxRigidBody) -> PxRigidBodyFlags_Set ---

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
    rigid_body_set_min_c_c_d_advance_coefficient_mut :: proc(self_: ^PxRigidBody, advanceCoefficient: _c.float) ---

    /// Gets the CCD minimum advance coefficient.
    ///
    /// The value of the CCD min advance coefficient.
    @(link_name = "PxRigidBody_getMinCCDAdvanceCoefficient")
    rigid_body_get_min_c_c_d_advance_coefficient :: proc(self_: ^PxRigidBody) -> _c.float ---

    /// Sets the maximum depenetration velocity permitted to be introduced by the solver.
    /// This value controls how much velocity the solver can introduce to correct for penetrations in contacts.
    @(link_name = "PxRigidBody_setMaxDepenetrationVelocity_mut")
    rigid_body_set_max_depenetration_velocity_mut :: proc(self_: ^PxRigidBody, biasClamp: _c.float) ---

    /// Returns the maximum depenetration velocity the solver is permitted to introduced.
    /// This value controls how much velocity the solver can introduce to correct for penetrations in contacts.
    ///
    /// The maximum penetration bias applied by the solver.
    @(link_name = "PxRigidBody_getMaxDepenetrationVelocity")
    rigid_body_get_max_depenetration_velocity :: proc(self_: ^PxRigidBody) -> _c.float ---

    /// Sets a limit on the impulse that may be applied at a contact. The maximum impulse at a contact between two dynamic or kinematic
    /// bodies will be the minimum of the two limit values. For a collision between a static and a dynamic body, the impulse is limited
    /// by the value for the dynamic body.
    @(link_name = "PxRigidBody_setMaxContactImpulse_mut")
    rigid_body_set_max_contact_impulse_mut :: proc(self_: ^PxRigidBody, maxImpulse: _c.float) ---

    /// Returns the maximum impulse that may be applied at a contact.
    ///
    /// The maximum impulse that may be applied at a contact
    @(link_name = "PxRigidBody_getMaxContactImpulse")
    rigid_body_get_max_contact_impulse :: proc(self_: ^PxRigidBody) -> _c.float ---

    /// Sets a distance scale whereby the angular influence of a contact on the normal constraint in a contact is
    /// zeroed if normal.cross(offset) falls below this tolerance. Rather than acting as an absolute value, this tolerance
    /// is scaled by the ratio rXn.dot(angVel)/normal.dot(linVel) such that contacts that have relatively larger angular velocity
    /// than linear normal velocity (e.g. rolling wheels) achieve larger slop values as the angular velocity increases.
    @(link_name = "PxRigidBody_setContactSlopCoefficient_mut")
    rigid_body_set_contact_slop_coefficient_mut :: proc(self_: ^PxRigidBody, slopCoefficient: _c.float) ---

    /// Returns the contact slop coefficient.
    ///
    /// The contact slop coefficient.
    @(link_name = "PxRigidBody_getContactSlopCoefficient")
    rigid_body_get_contact_slop_coefficient :: proc(self_: ^PxRigidBody) -> _c.float ---

    /// Returns the island node index
    ///
    /// The island node index.
    @(link_name = "PxRigidBody_getInternalIslandNodeIndex")
    rigid_body_get_internal_island_node_index :: proc(self_: ^PxRigidBody) -> PxNodeIndex ---

    /// Releases the link from the articulation.
    ///
    /// Only a leaf articulation link can be released.
    ///
    /// Releasing a link is not allowed while the articulation link is in a scene. In order to release a link,
    /// remove and then re-add the corresponding articulation to the scene.
    @(link_name = "PxArticulationLink_release_mut")
    articulation_link_release_mut :: proc(self_: ^PxArticulationLink) ---

    /// Gets the articulation that the link is a part of.
    ///
    /// The articulation.
    @(link_name = "PxArticulationLink_getArticulation")
    articulation_link_get_articulation :: proc(self_: ^PxArticulationLink) -> ^PxArticulationReducedCoordinate ---

    /// Gets the joint which connects this link to its parent.
    ///
    /// The joint connecting the link to the parent. NULL for the root link.
    @(link_name = "PxArticulationLink_getInboundJoint")
    articulation_link_get_inbound_joint :: proc(self_: ^PxArticulationLink) -> ^PxArticulationJointReducedCoordinate ---

    /// Gets the number of degrees of freedom of the joint which connects this link to its parent.
    ///
    /// - The root link DOF-count is defined to be 0 regardless of PxArticulationFlag::eFIX_BASE.
    /// - The return value is only valid for articulations that are in a scene.
    ///
    /// The number of degrees of freedom, or 0xFFFFFFFF if the articulation is not in a scene.
    @(link_name = "PxArticulationLink_getInboundJointDof")
    articulation_link_get_inbound_joint_dof :: proc(self_: ^PxArticulationLink) -> _c.uint32_t ---

    /// Gets the number of child links.
    ///
    /// The number of child links.
    @(link_name = "PxArticulationLink_getNbChildren")
    articulation_link_get_nb_children :: proc(self_: ^PxArticulationLink) -> _c.uint32_t ---

    /// Gets the low-level link index that may be used to index into members of PxArticulationCache.
    ///
    /// The return value is only valid for articulations that are in a scene.
    ///
    /// The low-level index, or 0xFFFFFFFF if the articulation is not in a scene.
    @(link_name = "PxArticulationLink_getLinkIndex")
    articulation_link_get_link_index :: proc(self_: ^PxArticulationLink) -> _c.uint32_t ---

    /// Retrieves the child links.
    ///
    /// The number of articulation links written to the buffer.
    @(link_name = "PxArticulationLink_getChildren")
    articulation_link_get_children :: proc(self_: ^PxArticulationLink, userBuffer: ^^PxArticulationLink, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

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
    articulation_link_set_cfm_scale_mut :: proc(self_: ^PxArticulationLink, cfm: _c.float) ---

    /// Get the constraint-force-mixing scale term.
    ///
    /// The constraint-force-mixing scale term.
    @(link_name = "PxArticulationLink_getCfmScale")
    articulation_link_get_cfm_scale :: proc(self_: ^PxArticulationLink) -> _c.float ---

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
    articulation_link_get_linear_velocity :: proc(self_: ^PxArticulationLink) -> PxVec3 ---

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
    articulation_link_get_angular_velocity :: proc(self_: ^PxArticulationLink) -> PxVec3 ---

    /// Returns the string name of the dynamic type.
    ///
    /// The string name.
    @(link_name = "PxArticulationLink_getConcreteTypeName")
    articulation_link_get_concrete_type_name :: proc(self_: ^PxArticulationLink) -> ^_c.char ---

    @(link_name = "PxConeLimitedConstraint_new")
    cone_limited_constraint_new :: proc() -> PxConeLimitedConstraint ---

    /// Releases a PxConstraint instance.
    ///
    /// This call does not wake up the connected rigid bodies.
    @(link_name = "PxConstraint_release_mut")
    constraint_release_mut :: proc(self_: ^PxConstraint) ---

    /// Retrieves the scene which this constraint belongs to.
    ///
    /// Owner Scene. NULL if not part of a scene.
    @(link_name = "PxConstraint_getScene")
    constraint_get_scene :: proc(self_: ^PxConstraint) -> ^PxScene ---

    /// Retrieves the actors for this constraint.
    @(link_name = "PxConstraint_getActors")
    constraint_get_actors :: proc(self_: ^PxConstraint, actor0: ^^PxRigidActor, actor1: ^^PxRigidActor) ---

    /// Sets the actors for this constraint.
    @(link_name = "PxConstraint_setActors_mut")
    constraint_set_actors_mut :: proc(self_: ^PxConstraint, actor0: ^PxRigidActor, actor1: ^PxRigidActor) ---

    /// Notify the scene that the constraint shader data has been updated by the application
    @(link_name = "PxConstraint_markDirty_mut")
    constraint_mark_dirty_mut :: proc(self_: ^PxConstraint) ---

    /// Retrieve the flags for this constraint
    ///
    /// the constraint flags
    @(link_name = "PxConstraint_getFlags")
    constraint_get_flags :: proc(self_: ^PxConstraint) -> PxConstraintFlags_Set ---

    /// Set the flags for this constraint
    ///
    /// default: PxConstraintFlag::eDRIVE_LIMITS_ARE_FORCES
    @(link_name = "PxConstraint_setFlags_mut")
    constraint_set_flags_mut :: proc(self_: ^PxConstraint, flags: PxConstraintFlags_Set) ---

    /// Set a flag for this constraint
    @(link_name = "PxConstraint_setFlag_mut")
    constraint_set_flag_mut :: proc(self_: ^PxConstraint, flag: PxConstraintFlag, value: _c.bool) ---

    /// Retrieve the constraint force most recently applied to maintain this constraint.
    ///
    /// It is not allowed to use this method while the simulation is running (except during PxScene::collide(),
    /// in PxContactModifyCallback or in contact report callbacks).
    @(link_name = "PxConstraint_getForce")
    constraint_get_force :: proc(self_: ^PxConstraint, linear: ^PxVec3, angular: ^PxVec3) ---

    /// whether the constraint is valid.
    ///
    /// A constraint is valid if it has at least one dynamic rigid body or articulation link. A constraint that
    /// is not valid may not be inserted into a scene, and therefore a static actor to which an invalid constraint
    /// is attached may not be inserted into a scene.
    ///
    /// Invalid constraints arise only when an actor to which the constraint is attached has been deleted.
    @(link_name = "PxConstraint_isValid")
    constraint_is_valid :: proc(self_: ^PxConstraint) -> _c.bool ---

    /// Set the break force and torque thresholds for this constraint.
    ///
    /// If either the force or torque measured at the constraint exceed these thresholds the constraint will break.
    @(link_name = "PxConstraint_setBreakForce_mut")
    constraint_set_break_force_mut :: proc(self_: ^PxConstraint, linear: _c.float, angular: _c.float) ---

    /// Retrieve the constraint break force and torque thresholds
    @(link_name = "PxConstraint_getBreakForce")
    constraint_get_break_force :: proc(self_: ^PxConstraint, linear: ^_c.float, angular: ^_c.float) ---

    /// Set the minimum response threshold for a constraint row
    ///
    /// When using mass modification for a joint or infinite inertia for a jointed body, very stiff solver constraints can be generated which
    /// can destabilize simulation. Setting this value to a small positive value (e.g. 1e-8) will cause constraint rows to be ignored if very
    /// large changes in impulses will generate only small changes in velocity. When setting this value, also set
    /// PxConstraintFlag::eDISABLE_PREPROCESSING. The solver accuracy for this joint may be reduced.
    @(link_name = "PxConstraint_setMinResponseThreshold_mut")
    constraint_set_min_response_threshold_mut :: proc(self_: ^PxConstraint, threshold: _c.float) ---

    /// Retrieve the constraint break force and torque thresholds
    ///
    /// the minimum response threshold for a constraint row
    @(link_name = "PxConstraint_getMinResponseThreshold")
    constraint_get_min_response_threshold :: proc(self_: ^PxConstraint) -> _c.float ---

    /// Fetch external owner of the constraint.
    ///
    /// Provides a reference to the external owner of a constraint and a unique owner type ID.
    ///
    /// Reference to the external object which owns the constraint.
    @(link_name = "PxConstraint_getExternalReference_mut")
    constraint_get_external_reference_mut :: proc(self_: ^PxConstraint, typeID: ^_c.uint32_t) -> rawptr ---

    /// Set the constraint functions for this constraint
    @(link_name = "PxConstraint_setConstraintFunctions_mut")
    constraint_set_constraint_functions_mut :: proc(self_: ^PxConstraint, connector: ^PxConstraintConnector, #by_ptr shaders: PxConstraintShaderTable) ---

    @(link_name = "PxConstraint_getConcreteTypeName")
    constraint_get_concrete_type_name :: proc(self_: ^PxConstraint) -> ^_c.char ---

    /// Constructor
    @(link_name = "PxContactStreamIterator_new")
    contact_stream_iterator_new :: proc(contactPatches: ^_c.uint8_t, contactPoints: ^_c.uint8_t, contactFaceIndices: ^_c.uint32_t, nbPatches: _c.uint32_t, nbContacts: _c.uint32_t) -> PxContactStreamIterator ---

    /// Returns whether there are more patches in this stream.
    ///
    /// Whether there are more patches in this stream.
    @(link_name = "PxContactStreamIterator_hasNextPatch")
    contact_stream_iterator_has_next_patch :: proc(self_: ^PxContactStreamIterator) -> _c.bool ---

    /// Returns the total contact count.
    ///
    /// Total contact count.
    @(link_name = "PxContactStreamIterator_getTotalContactCount")
    contact_stream_iterator_get_total_contact_count :: proc(self_: ^PxContactStreamIterator) -> _c.uint32_t ---

    /// Returns the total patch count.
    ///
    /// Total patch count.
    @(link_name = "PxContactStreamIterator_getTotalPatchCount")
    contact_stream_iterator_get_total_patch_count :: proc(self_: ^PxContactStreamIterator) -> _c.uint32_t ---

    /// Advances iterator to next contact patch.
    @(link_name = "PxContactStreamIterator_nextPatch_mut")
    contact_stream_iterator_next_patch_mut :: proc(self_: ^PxContactStreamIterator) ---

    /// Returns if the current patch has more contacts.
    ///
    /// If there are more contacts in the current patch.
    @(link_name = "PxContactStreamIterator_hasNextContact")
    contact_stream_iterator_has_next_contact :: proc(self_: ^PxContactStreamIterator) -> _c.bool ---

    /// Advances to the next contact in the patch.
    @(link_name = "PxContactStreamIterator_nextContact_mut")
    contact_stream_iterator_next_contact_mut :: proc(self_: ^PxContactStreamIterator) ---

    /// Gets the current contact's normal
    ///
    /// The current contact's normal.
    @(link_name = "PxContactStreamIterator_getContactNormal")
    contact_stream_iterator_get_contact_normal :: proc(self_: ^PxContactStreamIterator) -> ^PxVec3 ---

    /// Gets the inverse mass scale for body 0.
    ///
    /// The inverse mass scale for body 0.
    @(link_name = "PxContactStreamIterator_getInvMassScale0")
    contact_stream_iterator_get_inv_mass_scale0 :: proc(self_: ^PxContactStreamIterator) -> _c.float ---

    /// Gets the inverse mass scale for body 1.
    ///
    /// The inverse mass scale for body 1.
    @(link_name = "PxContactStreamIterator_getInvMassScale1")
    contact_stream_iterator_get_inv_mass_scale1 :: proc(self_: ^PxContactStreamIterator) -> _c.float ---

    /// Gets the inverse inertia scale for body 0.
    ///
    /// The inverse inertia scale for body 0.
    @(link_name = "PxContactStreamIterator_getInvInertiaScale0")
    contact_stream_iterator_get_inv_inertia_scale0 :: proc(self_: ^PxContactStreamIterator) -> _c.float ---

    /// Gets the inverse inertia scale for body 1.
    ///
    /// The inverse inertia scale for body 1.
    @(link_name = "PxContactStreamIterator_getInvInertiaScale1")
    contact_stream_iterator_get_inv_inertia_scale1 :: proc(self_: ^PxContactStreamIterator) -> _c.float ---

    /// Gets the contact's max impulse.
    ///
    /// The contact's max impulse.
    @(link_name = "PxContactStreamIterator_getMaxImpulse")
    contact_stream_iterator_get_max_impulse :: proc(self_: ^PxContactStreamIterator) -> _c.float ---

    /// Gets the contact's target velocity.
    ///
    /// The contact's target velocity.
    @(link_name = "PxContactStreamIterator_getTargetVel")
    contact_stream_iterator_get_target_vel :: proc(self_: ^PxContactStreamIterator) -> ^PxVec3 ---

    /// Gets the contact's contact point.
    ///
    /// The contact's contact point.
    @(link_name = "PxContactStreamIterator_getContactPoint")
    contact_stream_iterator_get_contact_point :: proc(self_: ^PxContactStreamIterator) -> ^PxVec3 ---

    /// Gets the contact's separation.
    ///
    /// The contact's separation.
    @(link_name = "PxContactStreamIterator_getSeparation")
    contact_stream_iterator_get_separation :: proc(self_: ^PxContactStreamIterator) -> _c.float ---

    /// Gets the contact's face index for shape 0.
    ///
    /// The contact's face index for shape 0.
    @(link_name = "PxContactStreamIterator_getFaceIndex0")
    contact_stream_iterator_get_face_index0 :: proc(self_: ^PxContactStreamIterator) -> _c.uint32_t ---

    /// Gets the contact's face index for shape 1.
    ///
    /// The contact's face index for shape 1.
    @(link_name = "PxContactStreamIterator_getFaceIndex1")
    contact_stream_iterator_get_face_index1 :: proc(self_: ^PxContactStreamIterator) -> _c.uint32_t ---

    /// Gets the contact's static friction coefficient.
    ///
    /// The contact's static friction coefficient.
    @(link_name = "PxContactStreamIterator_getStaticFriction")
    contact_stream_iterator_get_static_friction :: proc(self_: ^PxContactStreamIterator) -> _c.float ---

    /// Gets the contact's dynamic friction coefficient.
    ///
    /// The contact's dynamic friction coefficient.
    @(link_name = "PxContactStreamIterator_getDynamicFriction")
    contact_stream_iterator_get_dynamic_friction :: proc(self_: ^PxContactStreamIterator) -> _c.float ---

    /// Gets the contact's restitution coefficient.
    ///
    /// The contact's restitution coefficient.
    @(link_name = "PxContactStreamIterator_getRestitution")
    contact_stream_iterator_get_restitution :: proc(self_: ^PxContactStreamIterator) -> _c.float ---

    /// Gets the contact's damping value.
    ///
    /// The contact's damping value.
    @(link_name = "PxContactStreamIterator_getDamping")
    contact_stream_iterator_get_damping :: proc(self_: ^PxContactStreamIterator) -> _c.float ---

    /// Gets the contact's material flags.
    ///
    /// The contact's material flags.
    @(link_name = "PxContactStreamIterator_getMaterialFlags")
    contact_stream_iterator_get_material_flags :: proc(self_: ^PxContactStreamIterator) -> _c.uint32_t ---

    /// Gets the contact's material index for shape 0.
    ///
    /// The contact's material index for shape 0.
    @(link_name = "PxContactStreamIterator_getMaterialIndex0")
    contact_stream_iterator_get_material_index0 :: proc(self_: ^PxContactStreamIterator) -> _c.uint16_t ---

    /// Gets the contact's material index for shape 1.
    ///
    /// The contact's material index for shape 1.
    @(link_name = "PxContactStreamIterator_getMaterialIndex1")
    contact_stream_iterator_get_material_index1 :: proc(self_: ^PxContactStreamIterator) -> _c.uint16_t ---

    /// Advances the contact stream iterator to a specific contact index.
    ///
    /// True if advancing was possible
    @(link_name = "PxContactStreamIterator_advanceToIndex_mut")
    contact_stream_iterator_advance_to_index_mut :: proc(self_: ^PxContactStreamIterator, initialIndex: _c.uint32_t) -> _c.bool ---

    /// Get the position of a specific contact point in the set.
    ///
    /// Position to the requested point in world space
    @(link_name = "PxContactSet_getPoint")
    contact_set_get_point :: proc(self_: ^PxContactSet, i: _c.uint32_t) -> ^PxVec3 ---

    /// Alter the position of a specific contact point in the set.
    @(link_name = "PxContactSet_setPoint_mut")
    contact_set_set_point_mut :: proc(self_: ^PxContactSet, i: _c.uint32_t, #by_ptr p: PxVec3) ---

    /// Get the contact normal of a specific contact point in the set.
    ///
    /// The requested normal in world space
    @(link_name = "PxContactSet_getNormal")
    contact_set_get_normal :: proc(self_: ^PxContactSet, i: _c.uint32_t) -> ^PxVec3 ---

    /// Alter the contact normal of a specific contact point in the set.
    ///
    /// Changing the normal can cause contact points to be ignored.
    @(link_name = "PxContactSet_setNormal_mut")
    contact_set_set_normal_mut :: proc(self_: ^PxContactSet, i: _c.uint32_t, #by_ptr n: PxVec3) ---

    /// Get the separation distance of a specific contact point in the set.
    ///
    /// The separation. Negative implies penetration.
    @(link_name = "PxContactSet_getSeparation")
    contact_set_get_separation :: proc(self_: ^PxContactSet, i: _c.uint32_t) -> _c.float ---

    /// Alter the separation of a specific contact point in the set.
    @(link_name = "PxContactSet_setSeparation_mut")
    contact_set_set_separation_mut :: proc(self_: ^PxContactSet, i: _c.uint32_t, s: _c.float) ---

    /// Get the target velocity of a specific contact point in the set.
    ///
    /// The target velocity in world frame
    @(link_name = "PxContactSet_getTargetVelocity")
    contact_set_get_target_velocity :: proc(self_: ^PxContactSet, i: _c.uint32_t) -> ^PxVec3 ---

    /// Alter the target velocity of a specific contact point in the set.
    @(link_name = "PxContactSet_setTargetVelocity_mut")
    contact_set_set_target_velocity_mut :: proc(self_: ^PxContactSet, i: _c.uint32_t, #by_ptr v: PxVec3) ---

    /// Get the face index with respect to the first shape of the pair for a specific contact point in the set.
    ///
    /// The face index of the first shape
    ///
    /// At the moment, the first shape is never a tri-mesh, therefore this function always returns PXC_CONTACT_NO_FACE_INDEX
    @(link_name = "PxContactSet_getInternalFaceIndex0")
    contact_set_get_internal_face_index0 :: proc(self_: ^PxContactSet, i: _c.uint32_t) -> _c.uint32_t ---

    /// Get the face index with respect to the second shape of the pair for a specific contact point in the set.
    ///
    /// The face index of the second shape
    @(link_name = "PxContactSet_getInternalFaceIndex1")
    contact_set_get_internal_face_index1 :: proc(self_: ^PxContactSet, i: _c.uint32_t) -> _c.uint32_t ---

    /// Get the maximum impulse for a specific contact point in the set.
    ///
    /// The maximum impulse
    @(link_name = "PxContactSet_getMaxImpulse")
    contact_set_get_max_impulse :: proc(self_: ^PxContactSet, i: _c.uint32_t) -> _c.float ---

    /// Alter the maximum impulse for a specific contact point in the set.
    ///
    /// Must be nonnegative. If set to zero, the contact point will be ignored
    @(link_name = "PxContactSet_setMaxImpulse_mut")
    contact_set_set_max_impulse_mut :: proc(self_: ^PxContactSet, i: _c.uint32_t, s: _c.float) ---

    /// Get the restitution coefficient for a specific contact point in the set.
    ///
    /// The restitution coefficient
    @(link_name = "PxContactSet_getRestitution")
    contact_set_get_restitution :: proc(self_: ^PxContactSet, i: _c.uint32_t) -> _c.float ---

    /// Alter the restitution coefficient for a specific contact point in the set.
    ///
    /// Valid ranges [0,1]
    @(link_name = "PxContactSet_setRestitution_mut")
    contact_set_set_restitution_mut :: proc(self_: ^PxContactSet, i: _c.uint32_t, r: _c.float) ---

    /// Get the static friction coefficient for a specific contact point in the set.
    ///
    /// The friction coefficient (dimensionless)
    @(link_name = "PxContactSet_getStaticFriction")
    contact_set_get_static_friction :: proc(self_: ^PxContactSet, i: _c.uint32_t) -> _c.float ---

    /// Alter the static friction coefficient for a specific contact point in the set.
    @(link_name = "PxContactSet_setStaticFriction_mut")
    contact_set_set_static_friction_mut :: proc(self_: ^PxContactSet, i: _c.uint32_t, f: _c.float) ---

    /// Get the static friction coefficient for a specific contact point in the set.
    ///
    /// The friction coefficient
    @(link_name = "PxContactSet_getDynamicFriction")
    contact_set_get_dynamic_friction :: proc(self_: ^PxContactSet, i: _c.uint32_t) -> _c.float ---

    /// Alter the static dynamic coefficient for a specific contact point in the set.
    @(link_name = "PxContactSet_setDynamicFriction_mut")
    contact_set_set_dynamic_friction_mut :: proc(self_: ^PxContactSet, i: _c.uint32_t, f: _c.float) ---

    /// Ignore the contact point.
    ///
    /// If a contact point is ignored then no force will get applied at this point. This can be used to disable collision in certain areas of a shape, for example.
    @(link_name = "PxContactSet_ignore_mut")
    contact_set_ignore_mut :: proc(self_: ^PxContactSet, i: _c.uint32_t) ---

    /// The number of contact points in the set.
    @(link_name = "PxContactSet_size")
    contact_set_size :: proc(self_: ^PxContactSet) -> _c.uint32_t ---

    /// Returns the invMassScale of body 0
    ///
    /// A value
    /// <
    /// 1.0 makes this contact treat the body as if it had larger mass. A value of 0.f makes this contact
    /// treat the body as if it had infinite mass. Any value > 1.f makes this contact treat the body as if it had smaller mass.
    @(link_name = "PxContactSet_getInvMassScale0")
    contact_set_get_inv_mass_scale0 :: proc(self_: ^PxContactSet) -> _c.float ---

    /// Returns the invMassScale of body 1
    ///
    /// A value
    /// <
    /// 1.0 makes this contact treat the body as if it had larger mass. A value of 0.f makes this contact
    /// treat the body as if it had infinite mass. Any value > 1.f makes this contact treat the body as if it had smaller mass.
    @(link_name = "PxContactSet_getInvMassScale1")
    contact_set_get_inv_mass_scale1 :: proc(self_: ^PxContactSet) -> _c.float ---

    /// Returns the invInertiaScale of body 0
    ///
    /// A value
    /// <
    /// 1.0 makes this contact treat the body as if it had larger inertia. A value of 0.f makes this contact
    /// treat the body as if it had infinite inertia. Any value > 1.f makes this contact treat the body as if it had smaller inertia.
    @(link_name = "PxContactSet_getInvInertiaScale0")
    contact_set_get_inv_inertia_scale0 :: proc(self_: ^PxContactSet) -> _c.float ---

    /// Returns the invInertiaScale of body 1
    ///
    /// A value
    /// <
    /// 1.0 makes this contact treat the body as if it had larger inertia. A value of 0.f makes this contact
    /// treat the body as if it had infinite inertia. Any value > 1.f makes this contact treat the body as if it had smaller inertia.
    @(link_name = "PxContactSet_getInvInertiaScale1")
    contact_set_get_inv_inertia_scale1 :: proc(self_: ^PxContactSet) -> _c.float ---

    /// Sets the invMassScale of body 0
    ///
    /// This can be set to any value in the range [0, PX_MAX_F32). A value
    /// <
    /// 1.0 makes this contact treat the body as if it had larger mass. A value of 0.f makes this contact
    /// treat the body as if it had infinite mass. Any value > 1.f makes this contact treat the body as if it had smaller mass.
    @(link_name = "PxContactSet_setInvMassScale0_mut")
    contact_set_set_inv_mass_scale0_mut :: proc(self_: ^PxContactSet, scale: _c.float) ---

    /// Sets the invMassScale of body 1
    ///
    /// This can be set to any value in the range [0, PX_MAX_F32). A value
    /// <
    /// 1.0 makes this contact treat the body as if it had larger mass. A value of 0.f makes this contact
    /// treat the body as if it had infinite mass. Any value > 1.f makes this contact treat the body as if it had smaller mass.
    @(link_name = "PxContactSet_setInvMassScale1_mut")
    contact_set_set_inv_mass_scale1_mut :: proc(self_: ^PxContactSet, scale: _c.float) ---

    /// Sets the invInertiaScale of body 0
    ///
    /// This can be set to any value in the range [0, PX_MAX_F32). A value
    /// <
    /// 1.0 makes this contact treat the body as if it had larger inertia. A value of 0.f makes this contact
    /// treat the body as if it had infinite inertia. Any value > 1.f makes this contact treat the body as if it had smaller inertia.
    @(link_name = "PxContactSet_setInvInertiaScale0_mut")
    contact_set_set_inv_inertia_scale0_mut :: proc(self_: ^PxContactSet, scale: _c.float) ---

    /// Sets the invInertiaScale of body 1
    ///
    /// This can be set to any value in the range [0, PX_MAX_F32). A value
    /// <
    /// 1.0 makes this contact treat the body as if it had larger inertia. A value of 0.f makes this contact
    /// treat the body as if it had infinite inertia. Any value > 1.f makes this contact treat the body as if it had smaller inertia.
    @(link_name = "PxContactSet_setInvInertiaScale1_mut")
    contact_set_set_inv_inertia_scale1_mut :: proc(self_: ^PxContactSet, scale: _c.float) ---

    /// Passes modifiable arrays of contacts to the application.
    ///
    /// The initial contacts are regenerated from scratch each frame by collision detection.
    ///
    /// The number of contacts can not be changed, so you cannot add your own contacts.  You may however
    /// disable contacts using PxContactSet::ignore().
    @(link_name = "PxContactModifyCallback_onContactModify_mut")
    contact_modify_callback_on_contact_modify_mut :: proc(self_: ^PxContactModifyCallback, pairs: ^PxContactModifyPair, count: _c.uint32_t) ---

    /// Passes modifiable arrays of contacts to the application.
    ///
    /// The initial contacts are regenerated from scratch each frame by collision detection.
    ///
    /// The number of contacts can not be changed, so you cannot add your own contacts.  You may however
    /// disable contacts using PxContactSet::ignore().
    @(link_name = "PxCCDContactModifyCallback_onCCDContactModify_mut")
    c_c_d_contact_modify_callback_on_c_c_d_contact_modify_mut :: proc(self_: ^PxCCDContactModifyCallback, pairs: ^PxContactModifyPair, count: _c.uint32_t) ---

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
    deletion_listener_on_release_mut :: proc(self_: ^PxDeletionListener, observed: ^PxBase, userData: rawptr, deletionEvent: PxDeletionEventFlag) ---

    @(link_name = "PxBaseMaterial_isKindOf")
    base_material_is_kind_of :: proc(self_: ^PxBaseMaterial, name: ^_c.char) -> _c.bool ---

    /// Sets young's modulus which defines the body's stiffness
    @(link_name = "PxFEMMaterial_setYoungsModulus_mut")
    f_e_m_material_set_youngs_modulus_mut :: proc(self_: ^PxFEMMaterial, young: _c.float) ---

    /// Retrieves the young's modulus value.
    ///
    /// The young's modulus value.
    @(link_name = "PxFEMMaterial_getYoungsModulus")
    f_e_m_material_get_youngs_modulus :: proc(self_: ^PxFEMMaterial) -> _c.float ---

    /// Sets the Poisson's ratio which defines the body's volume preservation. Completely incompressible materials have a poisson ratio of 0.5. Its value should not be set to exactly 0.5 because this leads to numerical problems.
    @(link_name = "PxFEMMaterial_setPoissons_mut")
    f_e_m_material_set_poissons_mut :: proc(self_: ^PxFEMMaterial, poisson: _c.float) ---

    /// Retrieves the Poisson's ratio.
    ///
    /// The Poisson's ratio.
    @(link_name = "PxFEMMaterial_getPoissons")
    f_e_m_material_get_poissons :: proc(self_: ^PxFEMMaterial) -> _c.float ---

    /// Sets the dynamic friction value which defines the strength of resistance when two objects slide relative to each other while in contact.
    @(link_name = "PxFEMMaterial_setDynamicFriction_mut")
    f_e_m_material_set_dynamic_friction_mut :: proc(self_: ^PxFEMMaterial, dynamicFriction: _c.float) ---

    /// Retrieves the dynamic friction value
    ///
    /// The dynamic friction value
    @(link_name = "PxFEMMaterial_getDynamicFriction")
    f_e_m_material_get_dynamic_friction :: proc(self_: ^PxFEMMaterial) -> _c.float ---

    @(link_name = "PxFilterData_new")
    filter_data_new :: proc(anon_param0: PxEMPTY) -> PxFilterData ---

    /// Default constructor.
    @(link_name = "PxFilterData_new_1")
    filter_data_new_1 :: proc() -> PxFilterData ---

    /// Constructor to set filter data initially.
    @(link_name = "PxFilterData_new_2")
    filter_data_new_2 :: proc(w0: _c.uint32_t, w1: _c.uint32_t, w2: _c.uint32_t, w3: _c.uint32_t) -> PxFilterData ---

    /// (re)sets the structure to the default.
    @(link_name = "PxFilterData_setToDefault_mut")
    filter_data_set_to_default_mut :: proc(self_: ^PxFilterData) ---

    /// Extract filter object type from the filter attributes of a collision pair object
    ///
    /// The type of the collision pair object.
    @(link_name = "phys_PxGetFilterObjectType")
    get_filter_object_type :: proc(attr: _c.uint32_t) -> PxFilterObjectType ---

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
    simulation_filter_callback_pair_found_mut :: proc(self_: ^PxSimulationFilterCallback, pairID: _c.uint32_t, attributes0: _c.uint32_t, filterData0: PxFilterData, a0: ^PxActor, s0: ^PxShape, attributes1: _c.uint32_t, filterData1: PxFilterData, a1: ^PxActor, s1: ^PxShape, pairFlags: ^PxPairFlags_Set) -> PxFilterFlags_Set ---

    /// Callback to inform that a tracked collision pair is gone.
    ///
    /// This method gets called when a collision pair disappears or gets re-filtered. Only applies to
    /// collision pairs which have been marked as filter callback pairs ([`PxFilterFlag::eNOTIFY`] set in #pairFound()).
    @(link_name = "PxSimulationFilterCallback_pairLost_mut")
    simulation_filter_callback_pair_lost_mut :: proc(self_: ^PxSimulationFilterCallback, pairID: _c.uint32_t, attributes0: _c.uint32_t, filterData0: PxFilterData, attributes1: _c.uint32_t, filterData1: PxFilterData, objectRemoved: _c.bool) ---

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
    simulation_filter_callback_status_change_mut :: proc(self_: ^PxSimulationFilterCallback, pairID: ^_c.uint32_t, pairFlags: ^PxPairFlags_Set, filterFlags: ^PxFilterFlags_Set) -> _c.bool ---

    /// Any combination of PxDataAccessFlag::eREADABLE and PxDataAccessFlag::eWRITABLE
    @(link_name = "PxLockedData_getDataAccessFlags_mut")
    locked_data_get_data_access_flags_mut :: proc(self_: ^PxLockedData) -> PxDataAccessFlags_Set ---

    /// Unlocks the bulk data.
    @(link_name = "PxLockedData_unlock_mut")
    locked_data_unlock_mut :: proc(self_: ^PxLockedData) ---

    /// virtual destructor
    @(link_name = "PxLockedData_delete")
    locked_data_delete :: proc(self_: ^PxLockedData) ---

    /// Sets the coefficient of dynamic friction.
    ///
    /// The coefficient of dynamic friction should be in [0, PX_MAX_F32). If set to greater than staticFriction, the effective value of staticFriction will be increased to match.
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake any actors which may be affected.
    @(link_name = "PxMaterial_setDynamicFriction_mut")
    material_set_dynamic_friction_mut :: proc(self_: ^PxMaterial, coef: _c.float) ---

    /// Retrieves the DynamicFriction value.
    ///
    /// The coefficient of dynamic friction.
    @(link_name = "PxMaterial_getDynamicFriction")
    material_get_dynamic_friction :: proc(self_: ^PxMaterial) -> _c.float ---

    /// Sets the coefficient of static friction
    ///
    /// The coefficient of static friction should be in the range [0, PX_MAX_F32)
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake any actors which may be affected.
    @(link_name = "PxMaterial_setStaticFriction_mut")
    material_set_static_friction_mut :: proc(self_: ^PxMaterial, coef: _c.float) ---

    /// Retrieves the coefficient of static friction.
    ///
    /// The coefficient of static friction.
    @(link_name = "PxMaterial_getStaticFriction")
    material_get_static_friction :: proc(self_: ^PxMaterial) -> _c.float ---

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
    material_set_restitution_mut :: proc(self_: ^PxMaterial, rest: _c.float) ---

    /// Retrieves the coefficient of restitution.
    ///
    /// See [`setRestitution`].
    ///
    /// The coefficient of restitution.
    @(link_name = "PxMaterial_getRestitution")
    material_get_restitution :: proc(self_: ^PxMaterial) -> _c.float ---

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
    material_set_damping_mut :: proc(self_: ^PxMaterial, damping: _c.float) ---

    /// Retrieves the coefficient of damping.
    ///
    /// See [`setDamping`].
    ///
    /// The coefficient of damping.
    @(link_name = "PxMaterial_getDamping")
    material_get_damping :: proc(self_: ^PxMaterial) -> _c.float ---

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
    material_set_flag_mut :: proc(self_: ^PxMaterial, flag: PxMaterialFlag, b: _c.bool) ---

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
    material_set_flags_mut :: proc(self_: ^PxMaterial, flags: PxMaterialFlags_Set) ---

    /// Retrieves the flags. See [`PxMaterialFlag`].
    ///
    /// The material flags.
    @(link_name = "PxMaterial_getFlags")
    material_get_flags :: proc(self_: ^PxMaterial) -> PxMaterialFlags_Set ---

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
    material_set_friction_combine_mode_mut :: proc(self_: ^PxMaterial, combMode: PxCombineMode) ---

    /// Retrieves the friction combine mode.
    ///
    /// See [`setFrictionCombineMode`].
    ///
    /// The friction combine mode for this material.
    @(link_name = "PxMaterial_getFrictionCombineMode")
    material_get_friction_combine_mode :: proc(self_: ^PxMaterial) -> PxCombineMode ---

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
    material_set_restitution_combine_mode_mut :: proc(self_: ^PxMaterial, combMode: PxCombineMode) ---

    /// Retrieves the restitution combine mode.
    ///
    /// See [`setRestitutionCombineMode`].
    ///
    /// The coefficient of restitution combine mode for this material.
    @(link_name = "PxMaterial_getRestitutionCombineMode")
    material_get_restitution_combine_mode :: proc(self_: ^PxMaterial) -> PxCombineMode ---

    @(link_name = "PxMaterial_getConcreteTypeName")
    material_get_concrete_type_name :: proc(self_: ^PxMaterial) -> ^_c.char ---

    /// Construct parameters with default values.
    @(link_name = "PxDiffuseParticleParams_new")
    diffuse_particle_params_new :: proc() -> PxDiffuseParticleParams ---

    /// (re)sets the structure to the default.
    @(link_name = "PxDiffuseParticleParams_setToDefault_mut")
    diffuse_particle_params_set_to_default_mut :: proc(self_: ^PxDiffuseParticleParams) ---

    /// Sets friction
    @(link_name = "PxParticleMaterial_setFriction_mut")
    particle_material_set_friction_mut :: proc(self_: ^PxParticleMaterial, friction: _c.float) ---

    /// Retrieves the friction value.
    ///
    /// The friction value.
    @(link_name = "PxParticleMaterial_getFriction")
    particle_material_get_friction :: proc(self_: ^PxParticleMaterial) -> _c.float ---

    /// Sets velocity damping term
    @(link_name = "PxParticleMaterial_setDamping_mut")
    particle_material_set_damping_mut :: proc(self_: ^PxParticleMaterial, damping: _c.float) ---

    /// Retrieves the velocity damping term
    ///
    /// The velocity damping term.
    @(link_name = "PxParticleMaterial_getDamping")
    particle_material_get_damping :: proc(self_: ^PxParticleMaterial) -> _c.float ---

    /// Sets adhesion term
    @(link_name = "PxParticleMaterial_setAdhesion_mut")
    particle_material_set_adhesion_mut :: proc(self_: ^PxParticleMaterial, adhesion: _c.float) ---

    /// Retrieves the adhesion term
    ///
    /// The adhesion term.
    @(link_name = "PxParticleMaterial_getAdhesion")
    particle_material_get_adhesion :: proc(self_: ^PxParticleMaterial) -> _c.float ---

    /// Sets gravity scale term
    @(link_name = "PxParticleMaterial_setGravityScale_mut")
    particle_material_set_gravity_scale_mut :: proc(self_: ^PxParticleMaterial, scale: _c.float) ---

    /// Retrieves the gravity scale term
    ///
    /// The gravity scale term.
    @(link_name = "PxParticleMaterial_getGravityScale")
    particle_material_get_gravity_scale :: proc(self_: ^PxParticleMaterial) -> _c.float ---

    /// Sets material adhesion radius scale. This is multiplied by the particle rest offset to compute the fall-off distance
    /// at which point adhesion ceases to operate.
    @(link_name = "PxParticleMaterial_setAdhesionRadiusScale_mut")
    particle_material_set_adhesion_radius_scale_mut :: proc(self_: ^PxParticleMaterial, scale: _c.float) ---

    /// Retrieves the adhesion radius scale.
    ///
    /// The adhesion radius scale.
    @(link_name = "PxParticleMaterial_getAdhesionRadiusScale")
    particle_material_get_adhesion_radius_scale :: proc(self_: ^PxParticleMaterial) -> _c.float ---

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
    physics_release_mut :: proc(self_: ^PxPhysics) ---

    /// Retrieves the Foundation instance.
    ///
    /// A reference to the Foundation object.
    @(link_name = "PxPhysics_getFoundation_mut")
    physics_get_foundation_mut :: proc(self_: ^PxPhysics) -> ^PxFoundation ---

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
    physics_create_aggregate_mut :: proc(self_: ^PxPhysics, maxActor: _c.uint32_t, maxShape: _c.uint32_t, filterHint: _c.uint32_t) -> ^PxAggregate ---

    /// Returns the simulation tolerance parameters.
    ///
    /// The current simulation tolerance parameters.
    @(link_name = "PxPhysics_getTolerancesScale")
    physics_get_tolerances_scale :: proc(self_: ^PxPhysics) -> ^PxTolerancesScale ---

    /// Creates a triangle mesh object.
    ///
    /// This can then be instanced into [`PxShape`] objects.
    ///
    /// The new triangle mesh.
    @(link_name = "PxPhysics_createTriangleMesh_mut")
    physics_create_triangle_mesh_mut :: proc(self_: ^PxPhysics, stream: ^PxInputStream) -> ^PxTriangleMesh ---

    /// Return the number of triangle meshes that currently exist.
    ///
    /// Number of triangle meshes.
    @(link_name = "PxPhysics_getNbTriangleMeshes")
    physics_get_nb_triangle_meshes :: proc(self_: ^PxPhysics) -> _c.uint32_t ---

    /// Writes the array of triangle mesh pointers to a user buffer.
    ///
    /// Returns the number of pointers written.
    ///
    /// The ordering of the triangle meshes in the array is not specified.
    ///
    /// The number of triangle mesh pointers written to userBuffer, this should be less or equal to bufferSize.
    @(link_name = "PxPhysics_getTriangleMeshes")
    physics_get_triangle_meshes :: proc(self_: ^PxPhysics, userBuffer: ^^PxTriangleMesh, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Creates a tetrahedron mesh object.
    ///
    /// This can then be instanced into [`PxShape`] objects.
    ///
    /// The new tetrahedron mesh.
    @(link_name = "PxPhysics_createTetrahedronMesh_mut")
    physics_create_tetrahedron_mesh_mut :: proc(self_: ^PxPhysics, stream: ^PxInputStream) -> ^PxTetrahedronMesh ---

    /// Creates a softbody mesh object.
    ///
    /// The new softbody mesh.
    @(link_name = "PxPhysics_createSoftBodyMesh_mut")
    physics_create_soft_body_mesh_mut :: proc(self_: ^PxPhysics, stream: ^PxInputStream) -> ^PxSoftBodyMesh ---

    /// Return the number of tetrahedron meshes that currently exist.
    ///
    /// Number of tetrahedron meshes.
    @(link_name = "PxPhysics_getNbTetrahedronMeshes")
    physics_get_nb_tetrahedron_meshes :: proc(self_: ^PxPhysics) -> _c.uint32_t ---

    /// Writes the array of tetrahedron mesh pointers to a user buffer.
    ///
    /// Returns the number of pointers written.
    ///
    /// The ordering of the tetrahedron meshes in the array is not specified.
    ///
    /// The number of tetrahedron mesh pointers written to userBuffer, this should be less or equal to bufferSize.
    @(link_name = "PxPhysics_getTetrahedronMeshes")
    physics_get_tetrahedron_meshes :: proc(self_: ^PxPhysics, userBuffer: ^^PxTetrahedronMesh, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Creates a heightfield object from previously cooked stream.
    ///
    /// This can then be instanced into [`PxShape`] objects.
    ///
    /// The new heightfield.
    @(link_name = "PxPhysics_createHeightField_mut")
    physics_create_height_field_mut :: proc(self_: ^PxPhysics, stream: ^PxInputStream) -> ^PxHeightField ---

    /// Return the number of heightfields that currently exist.
    ///
    /// Number of heightfields.
    @(link_name = "PxPhysics_getNbHeightFields")
    physics_get_nb_height_fields :: proc(self_: ^PxPhysics) -> _c.uint32_t ---

    /// Writes the array of heightfield pointers to a user buffer.
    ///
    /// Returns the number of pointers written.
    ///
    /// The ordering of the heightfields in the array is not specified.
    ///
    /// The number of heightfield pointers written to userBuffer, this should be less or equal to bufferSize.
    @(link_name = "PxPhysics_getHeightFields")
    physics_get_height_fields :: proc(self_: ^PxPhysics, userBuffer: ^^PxHeightField, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Creates a convex mesh object.
    ///
    /// This can then be instanced into [`PxShape`] objects.
    ///
    /// The new convex mesh.
    @(link_name = "PxPhysics_createConvexMesh_mut")
    physics_create_convex_mesh_mut :: proc(self_: ^PxPhysics, stream: ^PxInputStream) -> ^PxConvexMesh ---

    /// Return the number of convex meshes that currently exist.
    ///
    /// Number of convex meshes.
    @(link_name = "PxPhysics_getNbConvexMeshes")
    physics_get_nb_convex_meshes :: proc(self_: ^PxPhysics) -> _c.uint32_t ---

    /// Writes the array of convex mesh pointers to a user buffer.
    ///
    /// Returns the number of pointers written.
    ///
    /// The ordering of the convex meshes in the array is not specified.
    ///
    /// The number of convex mesh pointers written to userBuffer, this should be less or equal to bufferSize.
    @(link_name = "PxPhysics_getConvexMeshes")
    physics_get_convex_meshes :: proc(self_: ^PxPhysics, userBuffer: ^^PxConvexMesh, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Creates a bounding volume hierarchy.
    ///
    /// The new BVH.
    @(link_name = "PxPhysics_createBVH_mut")
    physics_create_b_v_h_mut :: proc(self_: ^PxPhysics, stream: ^PxInputStream) -> ^PxBVH ---

    /// Return the number of bounding volume hierarchies that currently exist.
    ///
    /// Number of bounding volume hierarchies.
    @(link_name = "PxPhysics_getNbBVHs")
    physics_get_nb_b_v_hs :: proc(self_: ^PxPhysics) -> _c.uint32_t ---

    /// Writes the array of bounding volume hierarchy pointers to a user buffer.
    ///
    /// Returns the number of pointers written.
    ///
    /// The ordering of the BVHs in the array is not specified.
    ///
    /// The number of BVH pointers written to userBuffer, this should be less or equal to bufferSize.
    @(link_name = "PxPhysics_getBVHs")
    physics_get_b_v_hs :: proc(self_: ^PxPhysics, userBuffer: ^^PxBVH, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Creates a scene.
    ///
    /// Every scene uses a Thread Local Storage slot. This imposes a platform specific limit on the
    /// number of scenes that can be created.
    ///
    /// The new scene object.
    @(link_name = "PxPhysics_createScene_mut")
    physics_create_scene_mut :: proc(self_: ^PxPhysics, #by_ptr sceneDesc: PxSceneDesc) -> ^PxScene ---

    /// Gets number of created scenes.
    ///
    /// The number of scenes created.
    @(link_name = "PxPhysics_getNbScenes")
    physics_get_nb_scenes :: proc(self_: ^PxPhysics) -> _c.uint32_t ---

    /// Writes the array of scene pointers to a user buffer.
    ///
    /// Returns the number of pointers written.
    ///
    /// The ordering of the scene pointers in the array is not specified.
    ///
    /// The number of scene pointers written to userBuffer, this should be less or equal to bufferSize.
    @(link_name = "PxPhysics_getScenes")
    physics_get_scenes :: proc(self_: ^PxPhysics, userBuffer: ^^PxScene, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Creates a static rigid actor with the specified pose and all other fields initialized
    /// to their default values.
    @(link_name = "PxPhysics_createRigidStatic_mut")
    physics_create_rigid_static_mut :: proc(self_: ^PxPhysics, #by_ptr pose: PxTransform) -> ^PxRigidStatic ---

    /// Creates a dynamic rigid actor with the specified pose and all other fields initialized
    /// to their default values.
    @(link_name = "PxPhysics_createRigidDynamic_mut")
    physics_create_rigid_dynamic_mut :: proc(self_: ^PxPhysics, #by_ptr pose: PxTransform) -> ^PxRigidDynamic ---

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
    physics_create_pruning_structure_mut :: proc(self_: ^PxPhysics, actors: ^^PxRigidActor, nbActors: _c.uint32_t) -> ^PxPruningStructure ---

    /// Creates a shape which may be attached to multiple actors
    ///
    /// The shape will be created with a reference count of 1.
    ///
    /// The shape
    ///
    /// Shared shapes are not mutable when they are attached to an actor
    @(link_name = "PxPhysics_createShape_mut")
    physics_create_shape_mut :: proc(self_: ^PxPhysics, geometry: ^PxGeometry, #by_ptr material: PxMaterial, isExclusive: _c.bool, shapeFlags: PxShapeFlags_Set) -> ^PxShape ---

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
    physics_create_shape_mut_1 :: proc(self_: ^PxPhysics, geometry: ^PxGeometry, materials: ^^PxMaterial, materialCount: _c.uint16_t, isExclusive: _c.bool, shapeFlags: PxShapeFlags_Set) -> ^PxShape ---

    /// Return the number of shapes that currently exist.
    ///
    /// Number of shapes.
    @(link_name = "PxPhysics_getNbShapes")
    physics_get_nb_shapes :: proc(self_: ^PxPhysics) -> _c.uint32_t ---

    /// Writes the array of shape pointers to a user buffer.
    ///
    /// Returns the number of pointers written.
    ///
    /// The ordering of the shapes in the array is not specified.
    ///
    /// The number of shape pointers written to userBuffer, this should be less or equal to bufferSize.
    @(link_name = "PxPhysics_getShapes")
    physics_get_shapes :: proc(self_: ^PxPhysics, userBuffer: ^^PxShape, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Creates a constraint shader.
    ///
    /// A constraint shader will get added automatically to the scene the two linked actors belong to. Either, but not both, of actor0 and actor1 may
    /// be NULL to denote attachment to the world.
    ///
    /// The new constraint shader.
    @(link_name = "PxPhysics_createConstraint_mut")
    physics_create_constraint_mut :: proc(self_: ^PxPhysics, actor0: ^PxRigidActor, actor1: ^PxRigidActor, connector: ^PxConstraintConnector, #by_ptr shaders: PxConstraintShaderTable, dataSize: _c.uint32_t) -> ^PxConstraint ---

    /// Creates a reduced-coordinate articulation with all fields initialized to their default values.
    ///
    /// the new articulation
    @(link_name = "PxPhysics_createArticulationReducedCoordinate_mut")
    physics_create_articulation_reduced_coordinate_mut :: proc(self_: ^PxPhysics) -> ^PxArticulationReducedCoordinate ---

    /// Creates a new rigid body material with certain default properties.
    ///
    /// The new rigid body material.
    @(link_name = "PxPhysics_createMaterial_mut")
    physics_create_material_mut :: proc(self_: ^PxPhysics, staticFriction: _c.float, dynamicFriction: _c.float, restitution: _c.float) -> ^PxMaterial ---

    /// Return the number of rigid body materials that currently exist.
    ///
    /// Number of rigid body materials.
    @(link_name = "PxPhysics_getNbMaterials")
    physics_get_nb_materials :: proc(self_: ^PxPhysics) -> _c.uint32_t ---

    /// Writes the array of rigid body material pointers to a user buffer.
    ///
    /// Returns the number of pointers written.
    ///
    /// The ordering of the materials in the array is not specified.
    ///
    /// The number of material pointers written to userBuffer, this should be less or equal to bufferSize.
    @(link_name = "PxPhysics_getMaterials")
    physics_get_materials :: proc(self_: ^PxPhysics, userBuffer: ^^PxMaterial, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Register a deletion listener. Listeners will be called whenever an object is deleted.
    ///
    /// It is illegal to register or unregister a deletion listener while deletions are being processed.
    ///
    /// By default a registered listener will receive events from all objects. Set the restrictedObjectSet parameter to true on registration and use [`registerDeletionListenerObjects`] to restrict the received events to specific objects.
    ///
    /// The deletion events are only supported on core PhysX objects. In general, objects in extension modules do not provide this functionality, however, in the case of PxJoint objects, the underlying PxConstraint will send the events.
    @(link_name = "PxPhysics_registerDeletionListener_mut")
    physics_register_deletion_listener_mut :: proc(self_: ^PxPhysics, observer: ^PxDeletionListener, #by_ptr deletionEvents: PxDeletionEventFlags_Set, restrictedObjectSet: _c.bool) ---

    /// Unregister a deletion listener.
    ///
    /// It is illegal to register or unregister a deletion listener while deletions are being processed.
    @(link_name = "PxPhysics_unregisterDeletionListener_mut")
    physics_unregister_deletion_listener_mut :: proc(self_: ^PxPhysics, observer: ^PxDeletionListener) ---

    /// Register specific objects for deletion events.
    ///
    /// This method allows for a deletion listener to limit deletion events to specific objects only.
    ///
    /// It is illegal to register or unregister objects while deletions are being processed.
    ///
    /// The deletion listener has to be registered through [`registerDeletionListener`]() and configured to support restricted object sets prior to this method being used.
    @(link_name = "PxPhysics_registerDeletionListenerObjects_mut")
    physics_register_deletion_listener_objects_mut :: proc(self_: ^PxPhysics, observer: ^PxDeletionListener, observables: ^^PxBase, observableCount: _c.uint32_t) ---

    /// Unregister specific objects for deletion events.
    ///
    /// This method allows to clear previously registered objects for a deletion listener (see [`registerDeletionListenerObjects`]()).
    ///
    /// It is illegal to register or unregister objects while deletions are being processed.
    ///
    /// The deletion listener has to be registered through [`registerDeletionListener`]() and configured to support restricted object sets prior to this method being used.
    @(link_name = "PxPhysics_unregisterDeletionListenerObjects_mut")
    physics_unregister_deletion_listener_objects_mut :: proc(self_: ^PxPhysics, observer: ^PxDeletionListener, observables: ^^PxBase, observableCount: _c.uint32_t) ---

    /// Gets PxPhysics object insertion interface.
    ///
    /// The insertion interface is needed for PxCreateTriangleMesh, PxCooking::createTriangleMesh etc., this allows runtime mesh creation.
    @(link_name = "PxPhysics_getPhysicsInsertionCallback_mut")
    physics_get_physics_insertion_callback_mut :: proc(self_: ^PxPhysics) -> ^PxInsertionCallback ---

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
    create_physics :: proc(version: _c.uint32_t, foundation: ^PxFoundation, #by_ptr scale: PxTolerancesScale, trackOutstandingAllocations: _c.bool, pvd: ^PxPvd, omniPvd: ^PxOmniPvd) -> ^PxPhysics ---

    @(link_name = "phys_PxGetPhysics")
    get_physics :: proc() -> ^PxPhysics ---

    @(link_name = "PxActorShape_new")
    actor_shape_new :: proc() -> PxActorShape ---

    @(link_name = "PxActorShape_new_1")
    actor_shape_new_1 :: proc(a: ^PxRigidActor, s: ^PxShape) -> PxActorShape ---

    /// constructor sets to default
    @(link_name = "PxQueryCache_new")
    query_cache_new :: proc() -> PxQueryCache ---

    /// constructor to set properties
    @(link_name = "PxQueryCache_new_1")
    query_cache_new_1 :: proc(s: ^PxShape, findex: _c.uint32_t) -> PxQueryCache ---

    /// default constructor
    @(link_name = "PxQueryFilterData_new")
    query_filter_data_new :: proc() -> PxQueryFilterData ---

    /// constructor to set both filter data and filter flags
    @(link_name = "PxQueryFilterData_new_1")
    query_filter_data_new_1 :: proc(#by_ptr fd: PxFilterData, f: PxQueryFlags_Set) -> PxQueryFilterData ---

    /// constructor to set filter flags only
    @(link_name = "PxQueryFilterData_new_2")
    query_filter_data_new_2 :: proc(f: PxQueryFlags_Set) -> PxQueryFilterData ---

    /// This filter callback is executed before the exact intersection test if PxQueryFlag::ePREFILTER flag was set.
    ///
    /// the updated type for this hit  (see [`PxQueryHitType`])
    @(link_name = "PxQueryFilterCallback_preFilter_mut")
    query_filter_callback_pre_filter_mut :: proc(self_: ^PxQueryFilterCallback, #by_ptr filterData: PxFilterData, shape: ^PxShape, actor: ^PxRigidActor, queryFlags: ^PxHitFlags_Set) -> PxQueryHitType ---

    /// This filter callback is executed if the exact intersection test returned true and PxQueryFlag::ePOSTFILTER flag was set.
    ///
    /// the updated hit type for this hit  (see [`PxQueryHitType`])
    @(link_name = "PxQueryFilterCallback_postFilter_mut")
    query_filter_callback_post_filter_mut :: proc(self_: ^PxQueryFilterCallback, #by_ptr filterData: PxFilterData, hit: ^PxQueryHit, shape: ^PxShape, actor: ^PxRigidActor) -> PxQueryHitType ---

    /// virtual destructor
    @(link_name = "PxQueryFilterCallback_delete")
    query_filter_callback_delete :: proc(self_: ^PxQueryFilterCallback) ---

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
    rigid_dynamic_set_kinematic_target_mut :: proc(self_: ^PxRigidDynamic, #by_ptr destination: PxTransform) ---

    /// Get target pose of a kinematically controlled dynamic actor.
    ///
    /// True if the actor is a kinematically controlled dynamic and the target has been set, else False.
    @(link_name = "PxRigidDynamic_getKinematicTarget")
    rigid_dynamic_get_kinematic_target :: proc(self_: ^PxRigidDynamic, target: ^PxTransform) -> _c.bool ---

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
    rigid_dynamic_is_sleeping :: proc(self_: ^PxRigidDynamic) -> _c.bool ---

    /// Sets the mass-normalized kinetic energy threshold below which an actor may go to sleep.
    ///
    /// Actors whose kinetic energy divided by their mass is below this threshold will be candidates for sleeping.
    ///
    /// Default:
    /// 5e-5f * PxTolerancesScale::speed * PxTolerancesScale::speed
    @(link_name = "PxRigidDynamic_setSleepThreshold_mut")
    rigid_dynamic_set_sleep_threshold_mut :: proc(self_: ^PxRigidDynamic, threshold: _c.float) ---

    /// Returns the mass-normalized kinetic energy below which an actor may go to sleep.
    ///
    /// The energy threshold for sleeping.
    @(link_name = "PxRigidDynamic_getSleepThreshold")
    rigid_dynamic_get_sleep_threshold :: proc(self_: ^PxRigidDynamic) -> _c.float ---

    /// Sets the mass-normalized kinetic energy threshold below which an actor may participate in stabilization.
    ///
    /// Actors whose kinetic energy divided by their mass is above this threshold will not participate in stabilization.
    ///
    /// This value has no effect if PxSceneFlag::eENABLE_STABILIZATION was not enabled on the PxSceneDesc.
    ///
    /// Default:
    /// 1e-5f * PxTolerancesScale::speed * PxTolerancesScale::speed
    @(link_name = "PxRigidDynamic_setStabilizationThreshold_mut")
    rigid_dynamic_set_stabilization_threshold_mut :: proc(self_: ^PxRigidDynamic, threshold: _c.float) ---

    /// Returns the mass-normalized kinetic energy below which an actor may participate in stabilization.
    ///
    /// Actors whose kinetic energy divided by their mass is above this threshold will not participate in stabilization.
    ///
    /// The energy threshold for participating in stabilization.
    @(link_name = "PxRigidDynamic_getStabilizationThreshold")
    rigid_dynamic_get_stabilization_threshold :: proc(self_: ^PxRigidDynamic) -> _c.float ---

    /// Reads the PxRigidDynamic lock flags.
    ///
    /// See the list of flags [`PxRigidDynamicLockFlag`]
    ///
    /// The values of the PxRigidDynamicLock flags.
    @(link_name = "PxRigidDynamic_getRigidDynamicLockFlags")
    rigid_dynamic_get_rigid_dynamic_lock_flags :: proc(self_: ^PxRigidDynamic) -> PxRigidDynamicLockFlags_Set ---

    /// Raises or clears a particular rigid dynamic lock flag.
    ///
    /// See the list of flags [`PxRigidDynamicLockFlag`]
    ///
    /// Default:
    /// no flags are set
    @(link_name = "PxRigidDynamic_setRigidDynamicLockFlag_mut")
    rigid_dynamic_set_rigid_dynamic_lock_flag_mut :: proc(self_: ^PxRigidDynamic, flag: PxRigidDynamicLockFlag, value: _c.bool) ---

    @(link_name = "PxRigidDynamic_setRigidDynamicLockFlags_mut")
    rigid_dynamic_set_rigid_dynamic_lock_flags_mut :: proc(self_: ^PxRigidDynamic, flags: PxRigidDynamicLockFlags_Set) ---

    /// Retrieves the linear velocity of an actor.
    ///
    /// It is not allowed to use this method while the simulation is running (except during PxScene::collide(),
    /// in PxContactModifyCallback or in contact report callbacks).
    ///
    /// The linear velocity of the actor.
    @(link_name = "PxRigidDynamic_getLinearVelocity")
    rigid_dynamic_get_linear_velocity :: proc(self_: ^PxRigidDynamic) -> PxVec3 ---

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
    rigid_dynamic_set_linear_velocity_mut :: proc(self_: ^PxRigidDynamic, #by_ptr linVel: PxVec3, autowake: _c.bool) ---

    /// Retrieves the angular velocity of the actor.
    ///
    /// It is not allowed to use this method while the simulation is running (except during PxScene::collide(),
    /// in PxContactModifyCallback or in contact report callbacks).
    ///
    /// The angular velocity of the actor.
    @(link_name = "PxRigidDynamic_getAngularVelocity")
    rigid_dynamic_get_angular_velocity :: proc(self_: ^PxRigidDynamic) -> PxVec3 ---

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
    rigid_dynamic_set_angular_velocity_mut :: proc(self_: ^PxRigidDynamic, #by_ptr angVel: PxVec3, autowake: _c.bool) ---

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
    rigid_dynamic_set_wake_counter_mut :: proc(self_: ^PxRigidDynamic, wakeCounterValue: _c.float) ---

    /// Returns the wake counter of the actor.
    ///
    /// It is not allowed to use this method while the simulation is running.
    ///
    /// The wake counter of the actor.
    @(link_name = "PxRigidDynamic_getWakeCounter")
    rigid_dynamic_get_wake_counter :: proc(self_: ^PxRigidDynamic) -> _c.float ---

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
    rigid_dynamic_wake_up_mut :: proc(self_: ^PxRigidDynamic) ---

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
    rigid_dynamic_put_to_sleep_mut :: proc(self_: ^PxRigidDynamic) ---

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
    rigid_dynamic_set_solver_iteration_counts_mut :: proc(self_: ^PxRigidDynamic, minPositionIters: _c.uint32_t, minVelocityIters: _c.uint32_t) ---

    /// Retrieves the solver iteration counts.
    @(link_name = "PxRigidDynamic_getSolverIterationCounts")
    rigid_dynamic_get_solver_iteration_counts :: proc(self_: ^PxRigidDynamic, minPositionIters: ^_c.uint32_t, minVelocityIters: ^_c.uint32_t) ---

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
    rigid_dynamic_get_contact_report_threshold :: proc(self_: ^PxRigidDynamic) -> _c.float ---

    /// Sets the force threshold for contact reports.
    ///
    /// See [`getContactReportThreshold`]().
    @(link_name = "PxRigidDynamic_setContactReportThreshold_mut")
    rigid_dynamic_set_contact_report_threshold_mut :: proc(self_: ^PxRigidDynamic, threshold: _c.float) ---

    @(link_name = "PxRigidDynamic_getConcreteTypeName")
    rigid_dynamic_get_concrete_type_name :: proc(self_: ^PxRigidDynamic) -> ^_c.char ---

    @(link_name = "PxRigidStatic_getConcreteTypeName")
    rigid_static_get_concrete_type_name :: proc(self_: ^PxRigidStatic) -> ^_c.char ---

    /// constructor sets to default.
    @(link_name = "PxSceneQueryDesc_new")
    scene_query_desc_new :: proc() -> PxSceneQueryDesc ---

    /// (re)sets the structure to the default.
    @(link_name = "PxSceneQueryDesc_setToDefault_mut")
    scene_query_desc_set_to_default_mut :: proc(self_: ^PxSceneQueryDesc) ---

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid.
    @(link_name = "PxSceneQueryDesc_isValid")
    scene_query_desc_is_valid :: proc(self_: ^PxSceneQueryDesc) -> _c.bool ---

    /// Sets the rebuild rate of the dynamic tree pruning structures.
    @(link_name = "PxSceneQuerySystemBase_setDynamicTreeRebuildRateHint_mut")
    scene_query_system_base_set_dynamic_tree_rebuild_rate_hint_mut :: proc(self_: ^PxSceneQuerySystemBase, dynamicTreeRebuildRateHint: _c.uint32_t) ---

    /// Retrieves the rebuild rate of the dynamic tree pruning structures.
    ///
    /// The rebuild rate of the dynamic tree pruning structures.
    @(link_name = "PxSceneQuerySystemBase_getDynamicTreeRebuildRateHint")
    scene_query_system_base_get_dynamic_tree_rebuild_rate_hint :: proc(self_: ^PxSceneQuerySystemBase) -> _c.uint32_t ---

    /// Forces dynamic trees to be immediately rebuilt.
    ///
    /// PxScene will call this function with the PX_SCENE_PRUNER_STATIC or PX_SCENE_PRUNER_DYNAMIC value.
    @(link_name = "PxSceneQuerySystemBase_forceRebuildDynamicTree_mut")
    scene_query_system_base_force_rebuild_dynamic_tree_mut :: proc(self_: ^PxSceneQuerySystemBase, prunerIndex: _c.uint32_t) ---

    /// Sets scene query update mode
    @(link_name = "PxSceneQuerySystemBase_setUpdateMode_mut")
    scene_query_system_base_set_update_mode_mut :: proc(self_: ^PxSceneQuerySystemBase, updateMode: PxSceneQueryUpdateMode) ---

    /// Gets scene query update mode
    ///
    /// Current scene query update mode.
    @(link_name = "PxSceneQuerySystemBase_getUpdateMode")
    scene_query_system_base_get_update_mode :: proc(self_: ^PxSceneQuerySystemBase) -> PxSceneQueryUpdateMode ---

    /// Retrieves the system's internal scene query timestamp, increased each time a change to the
    /// static scene query structure is performed.
    ///
    /// scene query static timestamp
    @(link_name = "PxSceneQuerySystemBase_getStaticTimestamp")
    scene_query_system_base_get_static_timestamp :: proc(self_: ^PxSceneQuerySystemBase) -> _c.uint32_t ---

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
    scene_query_system_base_flush_updates_mut :: proc(self_: ^PxSceneQuerySystemBase) ---

    /// Performs a raycast against objects in the scene, returns results in a PxRaycastBuffer object
    /// or via a custom user callback implementation inheriting from PxRaycastCallback.
    ///
    /// Touching hits are not ordered.
    ///
    /// Shooting a ray from within an object leads to different results depending on the shape type. Please check the details in user guide article SceneQuery. User can ignore such objects by employing one of the provided filter mechanisms.
    ///
    /// True if any touching or blocking hits were found or any hit was found in case PxQueryFlag::eANY_HIT was specified.
    @(link_name = "PxSceneQuerySystemBase_raycast")
    scene_query_system_base_raycast :: proc(self_: ^PxSceneQuerySystemBase, #by_ptr origin: PxVec3, #by_ptr unitDir: PxVec3, distance: _c.float, hitCall: ^PxRaycastCallback, hitFlags: PxHitFlags_Set, #by_ptr filterData: PxQueryFilterData, filterCall: ^PxQueryFilterCallback, cache: ^PxQueryCache, queryFlags: PxGeometryQueryFlags_Set) -> _c.bool ---

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
    scene_query_system_base_sweep :: proc(self_: ^PxSceneQuerySystemBase, geometry: ^PxGeometry, #by_ptr pose: PxTransform, #by_ptr unitDir: PxVec3, distance: _c.float, hitCall: ^PxSweepCallback, hitFlags: PxHitFlags_Set, #by_ptr filterData: PxQueryFilterData, filterCall: ^PxQueryFilterCallback, cache: ^PxQueryCache, inflation: _c.float, queryFlags: PxGeometryQueryFlags_Set) -> _c.bool ---

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
    scene_query_system_base_overlap :: proc(self_: ^PxSceneQuerySystemBase, geometry: ^PxGeometry, #by_ptr pose: PxTransform, hitCall: ^PxOverlapCallback, #by_ptr filterData: PxQueryFilterData, filterCall: ^PxQueryFilterCallback, cache: ^PxQueryCache, queryFlags: PxGeometryQueryFlags_Set) -> _c.bool ---

    /// Sets scene query update mode
    @(link_name = "PxSceneSQSystem_setSceneQueryUpdateMode_mut")
    scene_s_q_system_set_scene_query_update_mode_mut :: proc(self_: ^PxSceneSQSystem, updateMode: PxSceneQueryUpdateMode) ---

    /// Gets scene query update mode
    ///
    /// Current scene query update mode.
    @(link_name = "PxSceneSQSystem_getSceneQueryUpdateMode")
    scene_s_q_system_get_scene_query_update_mode :: proc(self_: ^PxSceneSQSystem) -> PxSceneQueryUpdateMode ---

    /// Retrieves the scene's internal scene query timestamp, increased each time a change to the
    /// static scene query structure is performed.
    ///
    /// scene query static timestamp
    @(link_name = "PxSceneSQSystem_getSceneQueryStaticTimestamp")
    scene_s_q_system_get_scene_query_static_timestamp :: proc(self_: ^PxSceneSQSystem) -> _c.uint32_t ---

    /// Flushes any changes to the scene query representation.
    @(link_name = "PxSceneSQSystem_flushQueryUpdates_mut")
    scene_s_q_system_flush_query_updates_mut :: proc(self_: ^PxSceneSQSystem) ---

    /// Forces dynamic trees to be immediately rebuilt.
    @(link_name = "PxSceneSQSystem_forceDynamicTreeRebuild_mut")
    scene_s_q_system_force_dynamic_tree_rebuild_mut :: proc(self_: ^PxSceneSQSystem, rebuildStaticStructure: _c.bool, rebuildDynamicStructure: _c.bool) ---

    /// Return the value of PxSceneQueryDesc::staticStructure that was set when creating the scene with PxPhysics::createScene
    @(link_name = "PxSceneSQSystem_getStaticStructure")
    scene_s_q_system_get_static_structure :: proc(self_: ^PxSceneSQSystem) -> PxPruningStructureType ---

    /// Return the value of PxSceneQueryDesc::dynamicStructure that was set when creating the scene with PxPhysics::createScene
    @(link_name = "PxSceneSQSystem_getDynamicStructure")
    scene_s_q_system_get_dynamic_structure :: proc(self_: ^PxSceneSQSystem) -> PxPruningStructureType ---

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
    scene_s_q_system_scene_queries_update_mut :: proc(self_: ^PxSceneSQSystem, completionTask: ^PxBaseTask, controlSimulation: _c.bool) ---

    /// This checks to see if the scene queries update has completed.
    ///
    /// This does not cause the data available for reading to be updated with the results of the scene queries update, it is simply a status check.
    /// The bool will allow it to either return immediately or block waiting for the condition to be met so that it can return true
    ///
    /// True if the results are available.
    @(link_name = "PxSceneSQSystem_checkQueries_mut")
    scene_s_q_system_check_queries_mut :: proc(self_: ^PxSceneSQSystem, block: _c.bool) -> _c.bool ---

    /// This method must be called after sceneQueriesUpdate. It will wait for the scene queries update to finish. If the user makes an illegal scene queries update call,
    /// the SDK will issue an error message.
    ///
    /// If a new AABB tree build finished, then during fetchQueries the current tree within the pruning structure is swapped with the new tree.
    @(link_name = "PxSceneSQSystem_fetchQueries_mut")
    scene_s_q_system_fetch_queries_mut :: proc(self_: ^PxSceneSQSystem, block: _c.bool) -> _c.bool ---

    /// Decrements the reference count of the object and releases it if the new reference count is zero.
    @(link_name = "PxSceneQuerySystem_release_mut")
    scene_query_system_release_mut :: proc(self_: ^PxSceneQuerySystem) ---

    /// Acquires a counted reference to this object.
    ///
    /// This method increases the reference count of the object by 1. Decrement the reference count by calling release()
    @(link_name = "PxSceneQuerySystem_acquireReference_mut")
    scene_query_system_acquire_reference_mut :: proc(self_: ^PxSceneQuerySystem) ---

    /// Preallocates internal arrays to minimize the amount of reallocations.
    ///
    /// The system does not prevent more allocations than given numbers. It is legal to not call this function at all,
    /// or to add more shapes to the system than the preallocated amounts.
    @(link_name = "PxSceneQuerySystem_preallocate_mut")
    scene_query_system_preallocate_mut :: proc(self_: ^PxSceneQuerySystem, prunerIndex: _c.uint32_t, nbShapes: _c.uint32_t) ---

    /// Frees internal memory that may not be in-use anymore.
    ///
    /// This is an entry point for reclaiming transient memory allocated at some point by the SQ system,
    /// but which wasn't been immediately freed for performance reason. Calling this function might free
    /// some memory, but it might also produce a new set of allocations in the next frame.
    @(link_name = "PxSceneQuerySystem_flushMemory_mut")
    scene_query_system_flush_memory_mut :: proc(self_: ^PxSceneQuerySystem) ---

    /// Adds a shape to the SQ system.
    ///
    /// The same function is used to add either a regular shape, or a SQ compound shape.
    @(link_name = "PxSceneQuerySystem_addSQShape_mut")
    scene_query_system_add_s_q_shape_mut :: proc(self_: ^PxSceneQuerySystem, actor: ^PxRigidActor, #by_ptr shape: PxShape, #by_ptr bounds: PxBounds3, #by_ptr transform: PxTransform, compoundHandle: ^_c.uint32_t, hasPruningStructure: _c.bool) ---

    /// Removes a shape from the SQ system.
    ///
    /// The same function is used to remove either a regular shape, or a SQ compound shape.
    @(link_name = "PxSceneQuerySystem_removeSQShape_mut")
    scene_query_system_remove_s_q_shape_mut :: proc(self_: ^PxSceneQuerySystem, actor: ^PxRigidActor, #by_ptr shape: PxShape) ---

    /// Updates a shape in the SQ system.
    ///
    /// The same function is used to update either a regular shape, or a SQ compound shape.
    ///
    /// The transforms are eager-evaluated, but the bounds are lazy-evaluated. This means that
    /// the updated transform has to be passed to the update function, while the bounds are automatically
    /// recomputed by the system whenever needed.
    @(link_name = "PxSceneQuerySystem_updateSQShape_mut")
    scene_query_system_update_s_q_shape_mut :: proc(self_: ^PxSceneQuerySystem, actor: ^PxRigidActor, #by_ptr shape: PxShape, #by_ptr transform: PxTransform) ---

    /// Adds a compound to the SQ system.
    ///
    /// SQ compound handle
    @(link_name = "PxSceneQuerySystem_addSQCompound_mut")
    scene_query_system_add_s_q_compound_mut :: proc(self_: ^PxSceneQuerySystem, actor: ^PxRigidActor, shapes: ^^PxShape, #by_ptr bvh: PxBVH, transforms: ^PxTransform) -> _c.uint32_t ---

    /// Removes a compound from the SQ system.
    @(link_name = "PxSceneQuerySystem_removeSQCompound_mut")
    scene_query_system_remove_s_q_compound_mut :: proc(self_: ^PxSceneQuerySystem, compoundHandle: _c.uint32_t) ---

    /// Updates a compound in the SQ system.
    ///
    /// The compound structures are immediately updated when the call occurs.
    @(link_name = "PxSceneQuerySystem_updateSQCompound_mut")
    scene_query_system_update_s_q_compound_mut :: proc(self_: ^PxSceneQuerySystem, compoundHandle: _c.uint32_t, #by_ptr compoundTransform: PxTransform) ---

    /// Shift the data structures' origin by the specified vector.
    ///
    /// Please refer to the notes of the similar function in PxScene.
    @(link_name = "PxSceneQuerySystem_shiftOrigin_mut")
    scene_query_system_shift_origin_mut :: proc(self_: ^PxSceneQuerySystem, #by_ptr shift: PxVec3) ---

    /// Merges a pruning structure with the SQ system's internal pruners.
    @(link_name = "PxSceneQuerySystem_merge_mut")
    scene_query_system_merge_mut :: proc(self_: ^PxSceneQuerySystem, #by_ptr pruningStructure: PxPruningStructure) ---

    /// Shape to SQ-pruner-handle mapping function.
    ///
    /// This function finds and returns the SQ pruner handle associated with a given (actor/shape) couple
    /// that was previously added to the system. This is needed for the sync function.
    ///
    /// Associated SQ pruner handle.
    @(link_name = "PxSceneQuerySystem_getHandle")
    scene_query_system_get_handle :: proc(self_: ^PxSceneQuerySystem, actor: ^PxRigidActor, #by_ptr shape: PxShape, prunerIndex: ^_c.uint32_t) -> _c.uint32_t ---

    /// Synchronizes the scene-query system with another system that references the same objects.
    ///
    /// This function is used when the scene-query objects also exist in another system that can also update them. For example the scene-query objects
    /// (used for raycast, overlap or sweep queries) might be driven by equivalent objects in an external rigid-body simulation engine. In this case
    /// the rigid-body simulation engine computes the new poses and transforms, and passes them to the scene-query system using this function. It is
    /// more efficient than calling updateSQShape on each object individually, since updateSQShape would end up recomputing the bounds already available
    /// in the rigid-body engine.
    @(link_name = "PxSceneQuerySystem_sync_mut")
    scene_query_system_sync_mut :: proc(self_: ^PxSceneQuerySystem, prunerIndex: _c.uint32_t, handles: ^_c.uint32_t, indices: ^_c.uint32_t, bounds: ^PxBounds3, transforms: ^PxTransformPadded, count: _c.uint32_t, #by_ptr ignoredIndices: PxBitMap) ---

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
    scene_query_system_finalize_updates_mut :: proc(self_: ^PxSceneQuerySystem) ---

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
    scene_query_system_prepare_scene_query_build_step_mut :: proc(self_: ^PxSceneQuerySystem, prunerIndex: _c.uint32_t) -> rawptr ---

    /// Executes asynchronous build step.
    ///
    /// This is directly called (asynchronously) by PxSceneSQSystem::sceneQueriesUpdate(). See the comments there.
    ///
    /// This function incrementally builds the internal trees/pruners. It is called asynchronously, i.e. this can be
    /// called from different threads for building multiple trees at the same time.
    @(link_name = "PxSceneQuerySystem_sceneQueryBuildStep_mut")
    scene_query_system_scene_query_build_step_mut :: proc(self_: ^PxSceneQuerySystem, handle: rawptr) ---

    @(link_name = "PxBroadPhaseDesc_new")
    broad_phase_desc_new :: proc(type: PxBroadPhaseType) -> PxBroadPhaseDesc ---

    @(link_name = "PxBroadPhaseDesc_isValid")
    broad_phase_desc_is_valid :: proc(self_: ^PxBroadPhaseDesc) -> _c.bool ---

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
    broad_phase_update_data_new :: proc(created: ^_c.uint32_t, nbCreated: _c.uint32_t, updated: ^_c.uint32_t, nbUpdated: _c.uint32_t, removed: ^_c.uint32_t, nbRemoved: _c.uint32_t, bounds: ^PxBounds3, groups: ^_c.uint32_t, distances: ^_c.float, capacity: _c.uint32_t) -> PxBroadPhaseUpdateData ---

    @(link_name = "PxBroadPhaseResults_new")
    broad_phase_results_new :: proc() -> PxBroadPhaseResults ---

    /// Returns number of regions currently registered in the broad-phase.
    ///
    /// Number of regions
    @(link_name = "PxBroadPhaseRegions_getNbRegions")
    broad_phase_regions_get_nb_regions :: proc(self_: ^PxBroadPhaseRegions) -> _c.uint32_t ---

    /// Gets broad-phase regions.
    ///
    /// Number of written out regions.
    @(link_name = "PxBroadPhaseRegions_getRegions")
    broad_phase_regions_get_regions :: proc(self_: ^PxBroadPhaseRegions, userBuffer: ^PxBroadPhaseRegionInfo, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

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
    broad_phase_regions_add_region_mut :: proc(self_: ^PxBroadPhaseRegions, #by_ptr region: PxBroadPhaseRegion, populateRegion: _c.bool, bounds: ^PxBounds3, distances: ^_c.float) -> _c.uint32_t ---

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
    broad_phase_regions_remove_region_mut :: proc(self_: ^PxBroadPhaseRegions, handle: _c.uint32_t) -> _c.bool ---

    @(link_name = "PxBroadPhaseRegions_getNbOutOfBoundsObjects")
    broad_phase_regions_get_nb_out_of_bounds_objects :: proc(self_: ^PxBroadPhaseRegions) -> _c.uint32_t ---

    @(link_name = "PxBroadPhaseRegions_getOutOfBoundsObjects")
    broad_phase_regions_get_out_of_bounds_objects :: proc(self_: ^PxBroadPhaseRegions) -> ^_c.uint32_t ---

    @(link_name = "PxBroadPhase_release_mut")
    broad_phase_release_mut :: proc(self_: ^PxBroadPhase) ---

    /// Gets the broadphase type.
    ///
    /// Broadphase type.
    @(link_name = "PxBroadPhase_getType")
    broad_phase_get_type :: proc(self_: ^PxBroadPhase) -> PxBroadPhaseType ---

    /// Gets broad-phase caps.
    @(link_name = "PxBroadPhase_getCaps")
    broad_phase_get_caps :: proc(self_: ^PxBroadPhase, caps: ^PxBroadPhaseCaps) ---

    /// Retrieves the regions API if applicable.
    ///
    /// For broadphases that do not use explicit user-defined regions, this call returns NULL.
    ///
    /// Region API, or NULL.
    @(link_name = "PxBroadPhase_getRegions_mut")
    broad_phase_get_regions_mut :: proc(self_: ^PxBroadPhase) -> ^PxBroadPhaseRegions ---

    /// Retrieves the broadphase allocator.
    ///
    /// User-provided buffers should ideally be allocated with this allocator, for best performance.
    /// This is especially true for the GPU broadphases, whose buffers need to be allocated in CUDA
    /// host memory.
    ///
    /// The broadphase allocator.
    @(link_name = "PxBroadPhase_getAllocator_mut")
    broad_phase_get_allocator_mut :: proc(self_: ^PxBroadPhase) -> ^PxAllocatorCallback ---

    /// Retrieves the profiler's context ID.
    ///
    /// The context ID.
    @(link_name = "PxBroadPhase_getContextID")
    broad_phase_get_context_i_d :: proc(self_: ^PxBroadPhase) -> _c.uint64_t ---

    /// Sets a scratch buffer
    ///
    /// Some broadphases might take advantage of a scratch buffer to limit runtime allocations.
    ///
    /// All broadphases still work without providing a scratch buffer, this is an optional function
    /// that can potentially reduce runtime allocations.
    @(link_name = "PxBroadPhase_setScratchBlock_mut")
    broad_phase_set_scratch_block_mut :: proc(self_: ^PxBroadPhase, scratchBlock: rawptr, size: _c.uint32_t) ---

    /// Updates the broadphase and computes the lists of created/deleted pairs.
    ///
    /// The provided update data describes changes to objects since the last broadphase update.
    ///
    /// To benefit from potentially multithreaded implementations, it is necessary to provide a continuation
    /// task to the function. It is legal to pass NULL there, but the underlying (CPU) implementations will
    /// then run single-threaded.
    @(link_name = "PxBroadPhase_update_mut")
    broad_phase_update_mut :: proc(self_: ^PxBroadPhase, #by_ptr updateData: PxBroadPhaseUpdateData, continuation: ^PxBaseTask) ---

    /// Retrieves the broadphase results after an update.
    ///
    /// This should be called once after each update call to retrieve the results of the broadphase. The
    /// results are incremental, i.e. the system only returns new and lost pairs, not all current pairs.
    @(link_name = "PxBroadPhase_fetchResults_mut")
    broad_phase_fetch_results_mut :: proc(self_: ^PxBroadPhase, results: ^PxBroadPhaseResults) ---

    /// Helper for single-threaded updates.
    ///
    /// This short helper function performs a single-theaded update and reports the results in a single call.
    @(link_name = "PxBroadPhase_update_mut_1")
    broad_phase_update_mut_1 :: proc(self_: ^PxBroadPhase, results: ^PxBroadPhaseResults, #by_ptr updateData: PxBroadPhaseUpdateData) ---

    /// Broadphase factory function.
    ///
    /// Use this function to create a new standalone broadphase.
    ///
    /// Newly created broadphase, or NULL
    @(link_name = "phys_PxCreateBroadPhase")
    create_broad_phase :: proc(#by_ptr desc: PxBroadPhaseDesc) -> ^PxBroadPhase ---

    @(link_name = "PxAABBManager_release_mut")
    a_a_b_b_manager_release_mut :: proc(self_: ^PxAABBManager) ---

    /// Retrieves the underlying broadphase.
    ///
    /// The managed broadphase.
    @(link_name = "PxAABBManager_getBroadPhase_mut")
    a_a_b_b_manager_get_broad_phase_mut :: proc(self_: ^PxAABBManager) -> ^PxBroadPhase ---

    /// Retrieves the managed bounds.
    ///
    /// This is needed as input parameters to functions like PxBroadPhaseRegions::addRegion.
    ///
    /// The managed object bounds.
    @(link_name = "PxAABBManager_getBounds")
    a_a_b_b_manager_get_bounds :: proc(self_: ^PxAABBManager) -> ^PxBounds3 ---

    /// Retrieves the managed distances.
    ///
    /// This is needed as input parameters to functions like PxBroadPhaseRegions::addRegion.
    ///
    /// The managed object distances.
    @(link_name = "PxAABBManager_getDistances")
    a_a_b_b_manager_get_distances :: proc(self_: ^PxAABBManager) -> ^_c.float ---

    /// Retrieves the managed filter groups.
    ///
    /// The managed object groups.
    @(link_name = "PxAABBManager_getGroups")
    a_a_b_b_manager_get_groups :: proc(self_: ^PxAABBManager) -> ^_c.uint32_t ---

    /// Retrieves the managed buffers' capacity.
    ///
    /// Bounds, distances and groups buffers have the same capacity.
    ///
    /// The managed buffers' capacity.
    @(link_name = "PxAABBManager_getCapacity")
    a_a_b_b_manager_get_capacity :: proc(self_: ^PxAABBManager) -> _c.uint32_t ---

    /// Adds an object to the manager.
    ///
    /// Objects' indices are externally managed, i.e. they must be provided by users (as opposed to handles
    /// that could be returned by this manager). The design allows users to identify an object by a single ID,
    /// and use the same ID in multiple sub-systems.
    @(link_name = "PxAABBManager_addObject_mut")
    a_a_b_b_manager_add_object_mut :: proc(self_: ^PxAABBManager, index: _c.uint32_t, #by_ptr bounds: PxBounds3, group: _c.uint32_t, distance: _c.float) ---

    /// Removes an object from the manager.
    @(link_name = "PxAABBManager_removeObject_mut")
    a_a_b_b_manager_remove_object_mut :: proc(self_: ^PxAABBManager, index: _c.uint32_t) ---

    /// Updates an object in the manager.
    ///
    /// This call can update an object's bounds, distance, or both.
    /// It is not possible to update an object's filter group.
    @(link_name = "PxAABBManager_updateObject_mut")
    a_a_b_b_manager_update_object_mut :: proc(self_: ^PxAABBManager, index: _c.uint32_t, bounds: ^PxBounds3, distance: ^_c.float) ---

    /// Updates the broadphase and computes the lists of created/deleted pairs.
    ///
    /// The data necessary for updating the broadphase is internally computed by the AABB manager.
    ///
    /// To benefit from potentially multithreaded implementations, it is necessary to provide a continuation
    /// task to the function. It is legal to pass NULL there, but the underlying (CPU) implementations will
    /// then run single-threaded.
    @(link_name = "PxAABBManager_update_mut")
    a_a_b_b_manager_update_mut :: proc(self_: ^PxAABBManager, continuation: ^PxBaseTask) ---

    /// Retrieves the broadphase results after an update.
    ///
    /// This should be called once after each update call to retrieve the results of the broadphase. The
    /// results are incremental, i.e. the system only returns new and lost pairs, not all current pairs.
    @(link_name = "PxAABBManager_fetchResults_mut")
    a_a_b_b_manager_fetch_results_mut :: proc(self_: ^PxAABBManager, results: ^PxBroadPhaseResults) ---

    /// Helper for single-threaded updates.
    ///
    /// This short helper function performs a single-theaded update and reports the results in a single call.
    @(link_name = "PxAABBManager_update_mut_1")
    a_a_b_b_manager_update_mut_1 :: proc(self_: ^PxAABBManager, results: ^PxBroadPhaseResults) ---

    /// AABB manager factory function.
    ///
    /// Use this function to create a new standalone high-level broadphase.
    ///
    /// Newly created AABB manager, or NULL
    @(link_name = "phys_PxCreateAABBManager")
    create_a_a_b_b_manager :: proc(broadphase: ^PxBroadPhase) -> ^PxAABBManager ---

    /// constructor sets to default
    @(link_name = "PxSceneLimits_new")
    scene_limits_new :: proc() -> PxSceneLimits ---

    /// (re)sets the structure to the default
    @(link_name = "PxSceneLimits_setToDefault_mut")
    scene_limits_set_to_default_mut :: proc(self_: ^PxSceneLimits) ---

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid.
    @(link_name = "PxSceneLimits_isValid")
    scene_limits_is_valid :: proc(self_: ^PxSceneLimits) -> _c.bool ---

    @(link_name = "PxgDynamicsMemoryConfig_new")
    g_dynamics_memory_config_new :: proc() -> PxgDynamicsMemoryConfig ---

    @(link_name = "PxgDynamicsMemoryConfig_isValid")
    g_dynamics_memory_config_is_valid :: proc(self_: ^PxgDynamicsMemoryConfig) -> _c.bool ---

    /// constructor sets to default.
    @(link_name = "PxSceneDesc_new")
    scene_desc_new :: proc(#by_ptr scale: PxTolerancesScale) -> PxSceneDesc ---

    /// (re)sets the structure to the default.
    @(link_name = "PxSceneDesc_setToDefault_mut")
    scene_desc_set_to_default_mut :: proc(self_: ^PxSceneDesc, #by_ptr scale: PxTolerancesScale) ---

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid.
    @(link_name = "PxSceneDesc_isValid")
    scene_desc_is_valid :: proc(self_: ^PxSceneDesc) -> _c.bool ---

    @(link_name = "PxSceneDesc_getTolerancesScale")
    scene_desc_get_tolerances_scale :: proc(self_: ^PxSceneDesc) -> ^PxTolerancesScale ---

    /// Get number of broadphase volumes added for the current simulation step.
    ///
    /// Number of broadphase volumes added.
    @(link_name = "PxSimulationStatistics_getNbBroadPhaseAdds")
    simulation_statistics_get_nb_broad_phase_adds :: proc(self_: ^PxSimulationStatistics) -> _c.uint32_t ---

    /// Get number of broadphase volumes removed for the current simulation step.
    ///
    /// Number of broadphase volumes removed.
    @(link_name = "PxSimulationStatistics_getNbBroadPhaseRemoves")
    simulation_statistics_get_nb_broad_phase_removes :: proc(self_: ^PxSimulationStatistics) -> _c.uint32_t ---

    /// Get number of shape collision pairs of a certain type processed for the current simulation step.
    ///
    /// There is an entry for each geometry pair type.
    ///
    /// entry[i][j] = entry[j][i], hence, if you want the sum of all pair
    /// types, you need to discard the symmetric entries
    ///
    /// Number of processed pairs of the specified geometry types.
    @(link_name = "PxSimulationStatistics_getRbPairStats")
    simulation_statistics_get_rb_pair_stats :: proc(self_: ^PxSimulationStatistics, pairType: RbPairStatsType, g0: PxGeometryType, g1: PxGeometryType) -> _c.uint32_t ---

    @(link_name = "PxSimulationStatistics_new")
    simulation_statistics_new :: proc() -> PxSimulationStatistics ---

    /// Sets the PVD flag. See PxPvdSceneFlag.
    @(link_name = "PxPvdSceneClient_setScenePvdFlag_mut")
    pvd_scene_client_set_scene_pvd_flag_mut :: proc(self_: ^PxPvdSceneClient, flag: PxPvdSceneFlag, value: _c.bool) ---

    /// Sets the PVD flags. See PxPvdSceneFlags.
    @(link_name = "PxPvdSceneClient_setScenePvdFlags_mut")
    pvd_scene_client_set_scene_pvd_flags_mut :: proc(self_: ^PxPvdSceneClient, flags: PxPvdSceneFlags_Set) ---

    /// Retrieves the PVD flags. See PxPvdSceneFlags.
    @(link_name = "PxPvdSceneClient_getScenePvdFlags")
    pvd_scene_client_get_scene_pvd_flags :: proc(self_: ^PxPvdSceneClient) -> PxPvdSceneFlags_Set ---

    /// update camera on PVD application's render window
    @(link_name = "PxPvdSceneClient_updateCamera_mut")
    pvd_scene_client_update_camera_mut :: proc(self_: ^PxPvdSceneClient, name: ^_c.char, #by_ptr origin: PxVec3, #by_ptr up: PxVec3, #by_ptr target: PxVec3) ---

    /// draw points on PVD application's render window
    @(link_name = "PxPvdSceneClient_drawPoints_mut")
    pvd_scene_client_draw_points_mut :: proc(self_: ^PxPvdSceneClient, points: ^PxDebugPoint, count: _c.uint32_t) ---

    /// draw lines on PVD application's render window
    @(link_name = "PxPvdSceneClient_drawLines_mut")
    pvd_scene_client_draw_lines_mut :: proc(self_: ^PxPvdSceneClient, lines: ^PxDebugLine, count: _c.uint32_t) ---

    /// draw triangles on PVD application's render window
    @(link_name = "PxPvdSceneClient_drawTriangles_mut")
    pvd_scene_client_draw_triangles_mut :: proc(self_: ^PxPvdSceneClient, triangles: ^PxDebugTriangle, count: _c.uint32_t) ---

    /// draw text on PVD application's render window
    @(link_name = "PxPvdSceneClient_drawText_mut")
    pvd_scene_client_draw_text_mut :: proc(self_: ^PxPvdSceneClient, #by_ptr text: PxDebugText) ---

    @(link_name = "PxDominanceGroupPair_new")
    dominance_group_pair_new :: proc(a: _c.uint8_t, b: _c.uint8_t) -> PxDominanceGroupPair ---

    @(link_name = "PxBroadPhaseCallback_delete")
    broad_phase_callback_delete :: proc(self_: ^PxBroadPhaseCallback) ---

    /// Out-of-bounds notification.
    ///
    /// This function is called when an object leaves the broad-phase.
    @(link_name = "PxBroadPhaseCallback_onObjectOutOfBounds_mut")
    broad_phase_callback_on_object_out_of_bounds_mut :: proc(self_: ^PxBroadPhaseCallback, shape: ^PxShape, actor: ^PxActor) ---

    /// Out-of-bounds notification.
    ///
    /// This function is called when an aggregate leaves the broad-phase.
    @(link_name = "PxBroadPhaseCallback_onObjectOutOfBounds_mut_1")
    broad_phase_callback_on_object_out_of_bounds_mut_1 :: proc(self_: ^PxBroadPhaseCallback, aggregate: ^PxAggregate) ---

    /// Deletes the scene.
    ///
    /// Removes any actors and constraint shaders from this scene
    /// (if the user hasn't already done so).
    ///
    /// Be sure to not keep a reference to this object after calling release.
    /// Avoid release calls while the scene is simulating (in between simulate() and fetchResults() calls).
    @(link_name = "PxScene_release_mut")
    scene_release_mut :: proc(self_: ^PxScene) ---

    /// Sets a scene flag. You can only set one flag at a time.
    ///
    /// Not all flags are mutable and changing some will result in an error. Please check [`PxSceneFlag`] to see which flags can be changed.
    @(link_name = "PxScene_setFlag_mut")
    scene_set_flag_mut :: proc(self_: ^PxScene, flag: PxSceneFlag, value: _c.bool) ---

    /// Get the scene flags.
    ///
    /// The scene flags. See [`PxSceneFlag`]
    @(link_name = "PxScene_getFlags")
    scene_get_flags :: proc(self_: ^PxScene) -> PxSceneFlags_Set ---

    /// Set new scene limits.
    ///
    /// Increase the maximum capacity of various data structures in the scene. The new capacities will be
    /// at least as large as required to deal with the objects currently in the scene. Further, these values
    /// are for preallocation and do not represent hard limits.
    @(link_name = "PxScene_setLimits_mut")
    scene_set_limits_mut :: proc(self_: ^PxScene, #by_ptr limits: PxSceneLimits) ---

    /// Get current scene limits.
    ///
    /// Current scene limits.
    @(link_name = "PxScene_getLimits")
    scene_get_limits :: proc(self_: ^PxScene) -> PxSceneLimits ---

    /// Call this method to retrieve the Physics SDK.
    ///
    /// The physics SDK this scene is associated with.
    @(link_name = "PxScene_getPhysics_mut")
    scene_get_physics_mut :: proc(self_: ^PxScene) -> ^PxPhysics ---

    /// Retrieves the scene's internal timestamp, increased each time a simulation step is completed.
    ///
    /// scene timestamp
    @(link_name = "PxScene_getTimestamp")
    scene_get_timestamp :: proc(self_: ^PxScene) -> _c.uint32_t ---

    /// Adds an articulation to this scene.
    ///
    /// If the articulation is already assigned to a scene (see [`PxArticulationReducedCoordinate::getScene`]), the call is ignored and an error is issued.
    ///
    /// True if success
    @(link_name = "PxScene_addArticulation_mut")
    scene_add_articulation_mut :: proc(self_: ^PxScene, articulation: ^PxArticulationReducedCoordinate) -> _c.bool ---

    /// Removes an articulation from this scene.
    ///
    /// If the articulation is not part of this scene (see [`PxArticulationReducedCoordinate::getScene`]), the call is ignored and an error is issued.
    ///
    /// If the articulation is in an aggregate it will be removed from the aggregate.
    @(link_name = "PxScene_removeArticulation_mut")
    scene_remove_articulation_mut :: proc(self_: ^PxScene, articulation: ^PxArticulationReducedCoordinate, wakeOnLostTouch: _c.bool) ---

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
    scene_add_actor_mut :: proc(self_: ^PxScene, actor: ^PxActor, bvh: ^PxBVH) -> _c.bool ---

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
    scene_add_actors_mut :: proc(self_: ^PxScene, actors: ^^PxActor, nbActors: _c.uint32_t) -> _c.bool ---

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
    scene_add_actors_mut_1 :: proc(self_: ^PxScene, #by_ptr pruningStructure: PxPruningStructure) -> _c.bool ---

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
    scene_remove_actor_mut :: proc(self_: ^PxScene, actor: ^PxActor, wakeOnLostTouch: _c.bool) ---

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
    scene_remove_actors_mut :: proc(self_: ^PxScene, actors: ^^PxActor, nbActors: _c.uint32_t, wakeOnLostTouch: _c.bool) ---

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
    scene_add_aggregate_mut :: proc(self_: ^PxScene, aggregate: ^PxAggregate) -> _c.bool ---

    /// Removes an aggregate from this scene.
    ///
    /// If the aggregate is not part of this scene (see [`PxAggregate::getScene`]), the call is ignored and an error is issued.
    ///
    /// If the aggregate contains actors, those actors are removed from the scene as well.
    @(link_name = "PxScene_removeAggregate_mut")
    scene_remove_aggregate_mut :: proc(self_: ^PxScene, aggregate: ^PxAggregate, wakeOnLostTouch: _c.bool) ---

    /// Adds objects in the collection to this scene.
    ///
    /// This function adds the following types of objects to this scene: PxRigidActor (except PxArticulationLink), PxAggregate, PxArticulationReducedCoordinate.
    /// This method is typically used after deserializing the collection in order to populate the scene with deserialized objects.
    ///
    /// If the collection contains an actor with an invalid constraint, in checked builds the call is ignored and an error is issued.
    ///
    /// True if success
    @(link_name = "PxScene_addCollection_mut")
    scene_add_collection_mut :: proc(self_: ^PxScene, #by_ptr collection: PxCollection) -> _c.bool ---

    /// Retrieve the number of actors of certain types in the scene. For supported types, see PxActorTypeFlags.
    ///
    /// the number of actors.
    @(link_name = "PxScene_getNbActors")
    scene_get_nb_actors :: proc(self_: ^PxScene, types: PxActorTypeFlags_Set) -> _c.uint32_t ---

    /// Retrieve an array of all the actors of certain types in the scene. For supported types, see PxActorTypeFlags.
    ///
    /// Number of actors written to the buffer.
    @(link_name = "PxScene_getActors")
    scene_get_actors :: proc(self_: ^PxScene, types: PxActorTypeFlags_Set, userBuffer: ^^PxActor, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Queries the PxScene for a list of the PxActors whose transforms have been
    /// updated during the previous simulation step. Only includes actors of type PxRigidDynamic and PxArticulationLink.
    ///
    /// PxSceneFlag::eENABLE_ACTIVE_ACTORS must be set.
    ///
    /// Do not use this method while the simulation is running. Calls to this method while the simulation is running will be ignored and NULL will be returned.
    ///
    /// A pointer to the list of active PxActors generated during the last call to fetchResults().
    @(link_name = "PxScene_getActiveActors_mut")
    scene_get_active_actors_mut :: proc(self_: ^PxScene, nbActorsOut: ^_c.uint32_t) -> ^^PxActor ---

    /// Returns the number of articulations in the scene.
    ///
    /// the number of articulations in this scene.
    @(link_name = "PxScene_getNbArticulations")
    scene_get_nb_articulations :: proc(self_: ^PxScene) -> _c.uint32_t ---

    /// Retrieve all the articulations in the scene.
    ///
    /// Number of articulations written to the buffer.
    @(link_name = "PxScene_getArticulations")
    scene_get_articulations :: proc(self_: ^PxScene, userBuffer: ^^PxArticulationReducedCoordinate, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Returns the number of constraint shaders in the scene.
    ///
    /// the number of constraint shaders in this scene.
    @(link_name = "PxScene_getNbConstraints")
    scene_get_nb_constraints :: proc(self_: ^PxScene) -> _c.uint32_t ---

    /// Retrieve all the constraint shaders in the scene.
    ///
    /// Number of constraint shaders written to the buffer.
    @(link_name = "PxScene_getConstraints")
    scene_get_constraints :: proc(self_: ^PxScene, userBuffer: ^^PxConstraint, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Returns the number of aggregates in the scene.
    ///
    /// the number of aggregates in this scene.
    @(link_name = "PxScene_getNbAggregates")
    scene_get_nb_aggregates :: proc(self_: ^PxScene) -> _c.uint32_t ---

    /// Retrieve all the aggregates in the scene.
    ///
    /// Number of aggregates written to the buffer.
    @(link_name = "PxScene_getAggregates")
    scene_get_aggregates :: proc(self_: ^PxScene, userBuffer: ^^PxAggregate, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

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
    scene_set_dominance_group_pair_mut :: proc(self_: ^PxScene, group1: _c.uint8_t, group2: _c.uint8_t, #by_ptr dominance: PxDominanceGroupPair) ---

    /// Samples the dominance matrix.
    @(link_name = "PxScene_getDominanceGroupPair")
    scene_get_dominance_group_pair :: proc(self_: ^PxScene, group1: _c.uint8_t, group2: _c.uint8_t) -> PxDominanceGroupPair ---

    /// Return the cpu dispatcher that was set in PxSceneDesc::cpuDispatcher when creating the scene with PxPhysics::createScene
    @(link_name = "PxScene_getCpuDispatcher")
    scene_get_cpu_dispatcher :: proc(self_: ^PxScene) -> ^PxCpuDispatcher ---

    /// Reserves a new client ID.
    ///
    /// PX_DEFAULT_CLIENT is always available as the default clientID.
    /// Additional clients are returned by this function. Clients cannot be released once created.
    /// An error is reported when more than a supported number of clients (currently 128) are created.
    @(link_name = "PxScene_createClient_mut")
    scene_create_client_mut :: proc(self_: ^PxScene) -> _c.uint8_t ---

    /// Sets a user notify object which receives special simulation events when they occur.
    ///
    /// Do not set the callback while the simulation is running. Calls to this method while the simulation is running will be ignored.
    @(link_name = "PxScene_setSimulationEventCallback_mut")
    scene_set_simulation_event_callback_mut :: proc(self_: ^PxScene, callback: ^PxSimulationEventCallback) ---

    /// Retrieves the simulationEventCallback pointer set with setSimulationEventCallback().
    ///
    /// The current user notify pointer. See [`PxSimulationEventCallback`].
    @(link_name = "PxScene_getSimulationEventCallback")
    scene_get_simulation_event_callback :: proc(self_: ^PxScene) -> ^PxSimulationEventCallback ---

    /// Sets a user callback object, which receives callbacks on all contacts generated for specified actors.
    ///
    /// Do not set the callback while the simulation is running. Calls to this method while the simulation is running will be ignored.
    @(link_name = "PxScene_setContactModifyCallback_mut")
    scene_set_contact_modify_callback_mut :: proc(self_: ^PxScene, callback: ^PxContactModifyCallback) ---

    /// Sets a user callback object, which receives callbacks on all CCD contacts generated for specified actors.
    ///
    /// Do not set the callback while the simulation is running. Calls to this method while the simulation is running will be ignored.
    @(link_name = "PxScene_setCCDContactModifyCallback_mut")
    scene_set_c_c_d_contact_modify_callback_mut :: proc(self_: ^PxScene, callback: ^PxCCDContactModifyCallback) ---

    /// Retrieves the PxContactModifyCallback pointer set with setContactModifyCallback().
    ///
    /// The current user contact modify callback pointer. See [`PxContactModifyCallback`].
    @(link_name = "PxScene_getContactModifyCallback")
    scene_get_contact_modify_callback :: proc(self_: ^PxScene) -> ^PxContactModifyCallback ---

    /// Retrieves the PxCCDContactModifyCallback pointer set with setContactModifyCallback().
    ///
    /// The current user contact modify callback pointer. See [`PxContactModifyCallback`].
    @(link_name = "PxScene_getCCDContactModifyCallback")
    scene_get_c_c_d_contact_modify_callback :: proc(self_: ^PxScene) -> ^PxCCDContactModifyCallback ---

    /// Sets a broad-phase user callback object.
    ///
    /// Do not set the callback while the simulation is running. Calls to this method while the simulation is running will be ignored.
    @(link_name = "PxScene_setBroadPhaseCallback_mut")
    scene_set_broad_phase_callback_mut :: proc(self_: ^PxScene, callback: ^PxBroadPhaseCallback) ---

    /// Retrieves the PxBroadPhaseCallback pointer set with setBroadPhaseCallback().
    ///
    /// The current broad-phase callback pointer. See [`PxBroadPhaseCallback`].
    @(link_name = "PxScene_getBroadPhaseCallback")
    scene_get_broad_phase_callback :: proc(self_: ^PxScene) -> ^PxBroadPhaseCallback ---

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
    scene_set_filter_shader_data_mut :: proc(self_: ^PxScene, data: rawptr, dataSize: _c.uint32_t) ---

    /// Gets the shared global filter data in use for this scene.
    ///
    /// The reference points to a copy of the original filter data specified in [`PxSceneDesc`].filterShaderData or provided by #setFilterShaderData().
    ///
    /// Shared filter data for filter shader.
    @(link_name = "PxScene_getFilterShaderData")
    scene_get_filter_shader_data :: proc(self_: ^PxScene) -> rawptr ---

    /// Gets the size of the shared global filter data ([`PxSceneDesc`].filterShaderData)
    ///
    /// Size of shared filter data [bytes].
    @(link_name = "PxScene_getFilterShaderDataSize")
    scene_get_filter_shader_data_size :: proc(self_: ^PxScene) -> _c.uint32_t ---

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
    scene_reset_filtering_mut :: proc(self_: ^PxScene, actor: ^PxActor) -> _c.bool ---

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
    scene_reset_filtering_mut_1 :: proc(self_: ^PxScene, actor: ^PxRigidActor, shapes: ^^PxShape, shapeCount: _c.uint32_t) -> _c.bool ---

    /// Gets the pair filtering mode for kinematic-kinematic pairs.
    ///
    /// Filtering mode for kinematic-kinematic pairs.
    @(link_name = "PxScene_getKinematicKinematicFilteringMode")
    scene_get_kinematic_kinematic_filtering_mode :: proc(self_: ^PxScene) -> PxPairFilteringMode ---

    /// Gets the pair filtering mode for static-kinematic pairs.
    ///
    /// Filtering mode for static-kinematic pairs.
    @(link_name = "PxScene_getStaticKinematicFilteringMode")
    scene_get_static_kinematic_filtering_mode :: proc(self_: ^PxScene) -> PxPairFilteringMode ---

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
    scene_simulate_mut :: proc(self_: ^PxScene, elapsedTime: _c.float, completionTask: ^PxBaseTask, scratchMemBlock: rawptr, scratchMemBlockSize: _c.uint32_t, controlSimulation: _c.bool) -> _c.bool ---

    /// Performs dynamics phase of the simulation pipeline.
    ///
    /// Calls to advance() should follow calls to fetchCollision(). An error message will be issued if this sequence is not followed.
    ///
    /// True if success
    @(link_name = "PxScene_advance_mut")
    scene_advance_mut :: proc(self_: ^PxScene, completionTask: ^PxBaseTask) -> _c.bool ---

    /// Performs collision detection for the scene over elapsedTime
    ///
    /// Calls to collide() should be the first method called to simulate a frame.
    ///
    /// True if success
    @(link_name = "PxScene_collide_mut")
    scene_collide_mut :: proc(self_: ^PxScene, elapsedTime: _c.float, completionTask: ^PxBaseTask, scratchMemBlock: rawptr, scratchMemBlockSize: _c.uint32_t, controlSimulation: _c.bool) -> _c.bool ---

    /// This checks to see if the simulation run has completed.
    ///
    /// This does not cause the data available for reading to be updated with the results of the simulation, it is simply a status check.
    /// The bool will allow it to either return immediately or block waiting for the condition to be met so that it can return true
    ///
    /// True if the results are available.
    @(link_name = "PxScene_checkResults_mut")
    scene_check_results_mut :: proc(self_: ^PxScene, block: _c.bool) -> _c.bool ---

    /// This method must be called after collide() and before advance(). It will wait for the collision phase to finish. If the user makes an illegal simulation call, the SDK will issue an error
    /// message.
    @(link_name = "PxScene_fetchCollision_mut")
    scene_fetch_collision_mut :: proc(self_: ^PxScene, block: _c.bool) -> _c.bool ---

    /// This is the big brother to checkResults() it basically does the following:
    ///
    /// True if the results have been fetched.
    @(link_name = "PxScene_fetchResults_mut")
    scene_fetch_results_mut :: proc(self_: ^PxScene, block: _c.bool, errorState: ^_c.uint32_t) -> _c.bool ---

    /// This call performs the first section of fetchResults, and returns a pointer to the contact streams output by the simulation. It can be used to process contact pairs in parallel, which is often a limiting factor
    /// for fetchResults() performance.
    ///
    /// After calling this function and processing the contact streams, call fetchResultsFinish(). Note that writes to the simulation are not
    /// permitted between the start of fetchResultsStart() and the end of fetchResultsFinish().
    ///
    /// True if the results have been fetched.
    @(link_name = "PxScene_fetchResultsStart_mut")
    scene_fetch_results_start_mut :: proc(self_: ^PxScene, contactPairs: ^^PxContactPairHeader, nbContactPairs: ^_c.uint32_t, block: _c.bool) -> _c.bool ---

    /// This call processes all event callbacks in parallel. It takes a continuation task, which will be executed once all callbacks have been processed.
    ///
    /// This is a utility function to make it easier to process callbacks in parallel using the PhysX task system. It can only be used in conjunction with
    /// fetchResultsStart(...) and fetchResultsFinish(...)
    @(link_name = "PxScene_processCallbacks_mut")
    scene_process_callbacks_mut :: proc(self_: ^PxScene, continuation: ^PxBaseTask) ---

    /// This call performs the second section of fetchResults.
    ///
    /// It must be called after fetchResultsStart() returns and contact reports have been processed.
    ///
    /// Note that once fetchResultsFinish() has been called, the contact streams returned in fetchResultsStart() will be invalid.
    @(link_name = "PxScene_fetchResultsFinish_mut")
    scene_fetch_results_finish_mut :: proc(self_: ^PxScene, errorState: ^_c.uint32_t) ---

    /// This call performs the synchronization of particle system data copies.
    @(link_name = "PxScene_fetchResultsParticleSystem_mut")
    scene_fetch_results_particle_system_mut :: proc(self_: ^PxScene) ---

    /// Clear internal buffers and free memory.
    ///
    /// This method can be used to clear buffers and free internal memory without having to destroy the scene. Can be useful if
    /// the physics data gets streamed in and a checkpoint with a clean state should be created.
    ///
    /// It is not allowed to call this method while the simulation is running. The call will fail.
    @(link_name = "PxScene_flushSimulation_mut")
    scene_flush_simulation_mut :: proc(self_: ^PxScene, sendPendingReports: _c.bool) ---

    /// Sets a constant gravity for the entire scene.
    ///
    /// Do not use this method while the simulation is running.
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the actor up automatically.
    @(link_name = "PxScene_setGravity_mut")
    scene_set_gravity_mut :: proc(self_: ^PxScene, #by_ptr vec: PxVec3) ---

    /// Retrieves the current gravity setting.
    ///
    /// The current gravity for the scene.
    @(link_name = "PxScene_getGravity")
    scene_get_gravity :: proc(self_: ^PxScene) -> PxVec3 ---

    /// Set the bounce threshold velocity.  Collision speeds below this threshold will not cause a bounce.
    ///
    /// Do not use this method while the simulation is running.
    @(link_name = "PxScene_setBounceThresholdVelocity_mut")
    scene_set_bounce_threshold_velocity_mut :: proc(self_: ^PxScene, t: _c.float) ---

    /// Return the bounce threshold velocity.
    @(link_name = "PxScene_getBounceThresholdVelocity")
    scene_get_bounce_threshold_velocity :: proc(self_: ^PxScene) -> _c.float ---

    /// Sets the maximum number of CCD passes
    ///
    /// Do not use this method while the simulation is running.
    @(link_name = "PxScene_setCCDMaxPasses_mut")
    scene_set_c_c_d_max_passes_mut :: proc(self_: ^PxScene, ccdMaxPasses: _c.uint32_t) ---

    /// Gets the maximum number of CCD passes.
    ///
    /// The maximum number of CCD passes.
    @(link_name = "PxScene_getCCDMaxPasses")
    scene_get_c_c_d_max_passes :: proc(self_: ^PxScene) -> _c.uint32_t ---

    /// Set the maximum CCD separation.
    ///
    /// Do not use this method while the simulation is running.
    @(link_name = "PxScene_setCCDMaxSeparation_mut")
    scene_set_c_c_d_max_separation_mut :: proc(self_: ^PxScene, t: _c.float) ---

    /// Gets the maximum CCD separation.
    ///
    /// The maximum CCD separation.
    @(link_name = "PxScene_getCCDMaxSeparation")
    scene_get_c_c_d_max_separation :: proc(self_: ^PxScene) -> _c.float ---

    /// Set the CCD threshold.
    ///
    /// Do not use this method while the simulation is running.
    @(link_name = "PxScene_setCCDThreshold_mut")
    scene_set_c_c_d_threshold_mut :: proc(self_: ^PxScene, t: _c.float) ---

    /// Gets the CCD threshold.
    ///
    /// The CCD threshold.
    @(link_name = "PxScene_getCCDThreshold")
    scene_get_c_c_d_threshold :: proc(self_: ^PxScene) -> _c.float ---

    /// Set the max bias coefficient.
    ///
    /// Do not use this method while the simulation is running.
    @(link_name = "PxScene_setMaxBiasCoefficient_mut")
    scene_set_max_bias_coefficient_mut :: proc(self_: ^PxScene, t: _c.float) ---

    /// Gets the max bias coefficient.
    ///
    /// The max bias coefficient.
    @(link_name = "PxScene_getMaxBiasCoefficient")
    scene_get_max_bias_coefficient :: proc(self_: ^PxScene) -> _c.float ---

    /// Set the friction offset threshold.
    ///
    /// Do not use this method while the simulation is running.
    @(link_name = "PxScene_setFrictionOffsetThreshold_mut")
    scene_set_friction_offset_threshold_mut :: proc(self_: ^PxScene, t: _c.float) ---

    /// Gets the friction offset threshold.
    @(link_name = "PxScene_getFrictionOffsetThreshold")
    scene_get_friction_offset_threshold :: proc(self_: ^PxScene) -> _c.float ---

    /// Set the friction correlation distance.
    ///
    /// Do not use this method while the simulation is running.
    @(link_name = "PxScene_setFrictionCorrelationDistance_mut")
    scene_set_friction_correlation_distance_mut :: proc(self_: ^PxScene, t: _c.float) ---

    /// Gets the friction correlation distance.
    @(link_name = "PxScene_getFrictionCorrelationDistance")
    scene_get_friction_correlation_distance :: proc(self_: ^PxScene) -> _c.float ---

    /// Return the friction model.
    @(link_name = "PxScene_getFrictionType")
    scene_get_friction_type :: proc(self_: ^PxScene) -> PxFrictionType ---

    /// Return the solver model.
    @(link_name = "PxScene_getSolverType")
    scene_get_solver_type :: proc(self_: ^PxScene) -> PxSolverType ---

    /// Function that lets you set debug visualization parameters.
    ///
    /// Returns false if the value passed is out of range for usage specified by the enum.
    ///
    /// Do not use this method while the simulation is running.
    ///
    /// False if the parameter is out of range.
    @(link_name = "PxScene_setVisualizationParameter_mut")
    scene_set_visualization_parameter_mut :: proc(self_: ^PxScene, param: PxVisualizationParameter, value: _c.float) -> _c.bool ---

    /// Function that lets you query debug visualization parameters.
    ///
    /// The value of the parameter.
    @(link_name = "PxScene_getVisualizationParameter")
    scene_get_visualization_parameter :: proc(self_: ^PxScene, paramEnum: PxVisualizationParameter) -> _c.float ---

    /// Defines a box in world space to which visualization geometry will be (conservatively) culled. Use a non-empty culling box to enable the feature, and an empty culling box to disable it.
    ///
    /// Do not use this method while the simulation is running.
    @(link_name = "PxScene_setVisualizationCullingBox_mut")
    scene_set_visualization_culling_box_mut :: proc(self_: ^PxScene, #by_ptr box: PxBounds3) ---

    /// Retrieves the visualization culling box.
    ///
    /// the box to which the geometry will be culled.
    @(link_name = "PxScene_getVisualizationCullingBox")
    scene_get_visualization_culling_box :: proc(self_: ^PxScene) -> PxBounds3 ---

    /// Retrieves the render buffer.
    ///
    /// This will contain the results of any active visualization for this scene.
    ///
    /// Do not use this method while the simulation is running. Calls to this method while the simulation is running will result in undefined behaviour.
    ///
    /// The render buffer.
    @(link_name = "PxScene_getRenderBuffer_mut")
    scene_get_render_buffer_mut :: proc(self_: ^PxScene) -> ^PxRenderBuffer ---

    /// Call this method to retrieve statistics for the current simulation step.
    ///
    /// Do not use this method while the simulation is running. Calls to this method while the simulation is running will be ignored.
    @(link_name = "PxScene_getSimulationStatistics")
    scene_get_simulation_statistics :: proc(self_: ^PxScene, stats: ^PxSimulationStatistics) ---

    /// Returns broad-phase type.
    ///
    /// Broad-phase type
    @(link_name = "PxScene_getBroadPhaseType")
    scene_get_broad_phase_type :: proc(self_: ^PxScene) -> PxBroadPhaseType ---

    /// Gets broad-phase caps.
    ///
    /// True if success
    @(link_name = "PxScene_getBroadPhaseCaps")
    scene_get_broad_phase_caps :: proc(self_: ^PxScene, caps: ^PxBroadPhaseCaps) -> _c.bool ---

    /// Returns number of regions currently registered in the broad-phase.
    ///
    /// Number of regions
    @(link_name = "PxScene_getNbBroadPhaseRegions")
    scene_get_nb_broad_phase_regions :: proc(self_: ^PxScene) -> _c.uint32_t ---

    /// Gets broad-phase regions.
    ///
    /// Number of written out regions
    @(link_name = "PxScene_getBroadPhaseRegions")
    scene_get_broad_phase_regions :: proc(self_: ^PxScene, userBuffer: ^PxBroadPhaseRegionInfo, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

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
    scene_add_broad_phase_region_mut :: proc(self_: ^PxScene, #by_ptr region: PxBroadPhaseRegion, populateRegion: _c.bool) -> _c.uint32_t ---

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
    scene_remove_broad_phase_region_mut :: proc(self_: ^PxScene, handle: _c.uint32_t) -> _c.bool ---

    /// Get the task manager associated with this scene
    ///
    /// the task manager associated with the scene
    @(link_name = "PxScene_getTaskManager")
    scene_get_task_manager :: proc(self_: ^PxScene) -> ^PxTaskManager ---

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
    scene_lock_read_mut :: proc(self_: ^PxScene, file: ^_c.char, line: _c.uint32_t) ---

    /// Unlock the scene from reading.
    ///
    /// Each unlockRead() must be paired with a lockRead() from the same thread.
    @(link_name = "PxScene_unlockRead_mut")
    scene_unlock_read_mut :: proc(self_: ^PxScene) ---

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
    scene_lock_write_mut :: proc(self_: ^PxScene, file: ^_c.char, line: _c.uint32_t) ---

    /// Unlock the scene from writing.
    ///
    /// Each unlockWrite() must be paired with a lockWrite() from the same thread.
    @(link_name = "PxScene_unlockWrite_mut")
    scene_unlock_write_mut :: proc(self_: ^PxScene) ---

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
    scene_set_nb_contact_data_blocks_mut :: proc(self_: ^PxScene, numBlocks: _c.uint32_t) ---

    /// get the number of cache blocks currently used by the scene
    ///
    /// This function may not be called while the scene is simulating
    ///
    /// the number of cache blocks currently used by the scene
    @(link_name = "PxScene_getNbContactDataBlocksUsed")
    scene_get_nb_contact_data_blocks_used :: proc(self_: ^PxScene) -> _c.uint32_t ---

    /// get the maximum number of cache blocks used by the scene
    ///
    /// This function may not be called while the scene is simulating
    ///
    /// the maximum number of cache blocks everused by the scene
    @(link_name = "PxScene_getMaxNbContactDataBlocksUsed")
    scene_get_max_nb_contact_data_blocks_used :: proc(self_: ^PxScene) -> _c.uint32_t ---

    /// Return the value of PxSceneDesc::contactReportStreamBufferSize that was set when creating the scene with PxPhysics::createScene
    @(link_name = "PxScene_getContactReportStreamBufferSize")
    scene_get_contact_report_stream_buffer_size :: proc(self_: ^PxScene) -> _c.uint32_t ---

    /// Sets the number of actors required to spawn a separate rigid body solver thread.
    ///
    /// Do not use this method while the simulation is running.
    @(link_name = "PxScene_setSolverBatchSize_mut")
    scene_set_solver_batch_size_mut :: proc(self_: ^PxScene, solverBatchSize: _c.uint32_t) ---

    /// Retrieves the number of actors required to spawn a separate rigid body solver thread.
    ///
    /// Current number of actors required to spawn a separate rigid body solver thread.
    @(link_name = "PxScene_getSolverBatchSize")
    scene_get_solver_batch_size :: proc(self_: ^PxScene) -> _c.uint32_t ---

    /// Sets the number of articulations required to spawn a separate rigid body solver thread.
    ///
    /// Do not use this method while the simulation is running.
    @(link_name = "PxScene_setSolverArticulationBatchSize_mut")
    scene_set_solver_articulation_batch_size_mut :: proc(self_: ^PxScene, solverBatchSize: _c.uint32_t) ---

    /// Retrieves the number of articulations required to spawn a separate rigid body solver thread.
    ///
    /// Current number of articulations required to spawn a separate rigid body solver thread.
    @(link_name = "PxScene_getSolverArticulationBatchSize")
    scene_get_solver_articulation_batch_size :: proc(self_: ^PxScene) -> _c.uint32_t ---

    /// Returns the wake counter reset value.
    ///
    /// Wake counter reset value
    @(link_name = "PxScene_getWakeCounterResetValue")
    scene_get_wake_counter_reset_value :: proc(self_: ^PxScene) -> _c.float ---

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
    scene_shift_origin_mut :: proc(self_: ^PxScene, #by_ptr shift: PxVec3) ---

    /// Returns the Pvd client associated with the scene.
    ///
    /// the client, NULL if no PVD supported.
    @(link_name = "PxScene_getScenePvdClient_mut")
    scene_get_scene_pvd_client_mut :: proc(self_: ^PxScene) -> ^PxPvdSceneClient ---

    /// Copy GPU articulation data from the internal GPU buffer to a user-provided device buffer.
    @(link_name = "PxScene_copyArticulationData_mut")
    scene_copy_articulation_data_mut :: proc(self_: ^PxScene, data: rawptr, index: rawptr, dataType: PxArticulationGpuDataType, nbCopyArticulations: _c.uint32_t, copyEvent: rawptr) ---

    /// Apply GPU articulation data from a user-provided device buffer to the internal GPU buffer.
    @(link_name = "PxScene_applyArticulationData_mut")
    scene_apply_articulation_data_mut :: proc(self_: ^PxScene, data: rawptr, index: rawptr, dataType: PxArticulationGpuDataType, nbUpdatedArticulations: _c.uint32_t, waitEvent: rawptr, signalEvent: rawptr) ---

    /// Copy GPU softbody data from the internal GPU buffer to a user-provided device buffer.
    @(link_name = "PxScene_copySoftBodyData_mut")
    scene_copy_soft_body_data_mut :: proc(self_: ^PxScene, data: ^rawptr, dataSizes: rawptr, softBodyIndices: rawptr, flag: PxSoftBodyDataFlag, nbCopySoftBodies: _c.uint32_t, maxSize: _c.uint32_t, copyEvent: rawptr) ---

    /// Apply user-provided data to the internal softbody system.
    @(link_name = "PxScene_applySoftBodyData_mut")
    scene_apply_soft_body_data_mut :: proc(self_: ^PxScene, data: ^rawptr, dataSizes: rawptr, softBodyIndices: rawptr, flag: PxSoftBodyDataFlag, nbUpdatedSoftBodies: _c.uint32_t, maxSize: _c.uint32_t, applyEvent: rawptr) ---

    /// Copy contact data from the internal GPU buffer to a user-provided device buffer.
    ///
    /// The contact data contains pointers to internal state and is only valid until the next call to simulate().
    @(link_name = "PxScene_copyContactData_mut")
    scene_copy_contact_data_mut :: proc(self_: ^PxScene, data: rawptr, maxContactPairs: _c.uint32_t, numContactPairs: rawptr, copyEvent: rawptr) ---

    /// Copy GPU rigid body data from the internal GPU buffer to a user-provided device buffer.
    @(link_name = "PxScene_copyBodyData_mut")
    scene_copy_body_data_mut :: proc(self_: ^PxScene, data: ^PxGpuBodyData, index: ^PxGpuActorPair, nbCopyActors: _c.uint32_t, copyEvent: rawptr) ---

    /// Apply user-provided data to rigid body.
    @(link_name = "PxScene_applyActorData_mut")
    scene_apply_actor_data_mut :: proc(self_: ^PxScene, data: rawptr, index: ^PxGpuActorPair, flag: PxActorCacheFlag, nbUpdatedActors: _c.uint32_t, waitEvent: rawptr, signalEvent: rawptr) ---

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
    scene_compute_dense_jacobians_mut :: proc(self_: ^PxScene, indices: ^PxIndexDataPair, nbIndices: _c.uint32_t, computeEvent: rawptr) ---

    /// Compute the joint-space inertia matrices that maps joint accelerations to joint forces: forces = M * accelerations on the GPU.
    ///
    /// The size of matrices can vary by articulation, since it depends on the number of links and degrees-of-freedom.
    ///
    /// The size is determined using this formula:
    /// sizeof(float) * dofCount * dofCount
    ///
    /// The user must ensure that adequate space is provided for each mass matrix.
    @(link_name = "PxScene_computeGeneralizedMassMatrices_mut")
    scene_compute_generalized_mass_matrices_mut :: proc(self_: ^PxScene, indices: ^PxIndexDataPair, nbIndices: _c.uint32_t, computeEvent: rawptr) ---

    /// Computes the joint DOF forces required to counteract gravitational forces for the given articulation pose.
    ///
    /// The size of the result can vary by articulation, since it depends on the number of links and degrees-of-freedom.
    ///
    /// The size is determined using this formula:
    /// sizeof(float) * dofCount
    ///
    /// The user must ensure that adequate space is provided for each articulation.
    @(link_name = "PxScene_computeGeneralizedGravityForces_mut")
    scene_compute_generalized_gravity_forces_mut :: proc(self_: ^PxScene, indices: ^PxIndexDataPair, nbIndices: _c.uint32_t, computeEvent: rawptr) ---

    /// Computes the joint DOF forces required to counteract coriolis and centrifugal forces for the given articulation pose.
    ///
    /// The size of the result can vary by articulation, since it depends on the number of links and degrees-of-freedom.
    ///
    /// The size is determined using this formula:
    /// sizeof(float) * dofCount
    ///
    /// The user must ensure that adequate space is provided for each articulation.
    @(link_name = "PxScene_computeCoriolisAndCentrifugalForces_mut")
    scene_compute_coriolis_and_centrifugal_forces_mut :: proc(self_: ^PxScene, indices: ^PxIndexDataPair, nbIndices: _c.uint32_t, computeEvent: rawptr) ---

    @(link_name = "PxScene_getGpuDynamicsConfig")
    scene_get_gpu_dynamics_config :: proc(self_: ^PxScene) -> PxgDynamicsMemoryConfig ---

    /// Apply user-provided data to particle buffers.
    ///
    /// This function should be used if the particle buffer flags are already on the device. Otherwise, use PxParticleBuffer::raiseFlags()
    /// from the CPU.
    ///
    /// This assumes the data has been changed directly in the PxParticleBuffer.
    @(link_name = "PxScene_applyParticleBufferData_mut")
    scene_apply_particle_buffer_data_mut :: proc(self_: ^PxScene, indices: ^_c.uint32_t, bufferIndexPair: ^PxGpuParticleBufferIndexPair, flags: ^PxParticleBufferFlags_Set, nbUpdatedBuffers: _c.uint32_t, waitEvent: rawptr, signalEvent: rawptr) ---

    /// Constructor
    @(link_name = "PxSceneReadLock_new_alloc")
    scene_read_lock_new_alloc :: proc(scene: ^PxScene, file: ^_c.char, line: _c.uint32_t) -> ^PxSceneReadLock ---

    @(link_name = "PxSceneReadLock_delete")
    scene_read_lock_delete :: proc(self_: ^PxSceneReadLock) ---

    /// Constructor
    @(link_name = "PxSceneWriteLock_new_alloc")
    scene_write_lock_new_alloc :: proc(scene: ^PxScene, file: ^_c.char, line: _c.uint32_t) -> ^PxSceneWriteLock ---

    @(link_name = "PxSceneWriteLock_delete")
    scene_write_lock_delete :: proc(self_: ^PxSceneWriteLock) ---

    @(link_name = "PxContactPairExtraDataItem_new")
    contact_pair_extra_data_item_new :: proc() -> PxContactPairExtraDataItem ---

    @(link_name = "PxContactPairVelocity_new")
    contact_pair_velocity_new :: proc() -> PxContactPairVelocity ---

    @(link_name = "PxContactPairPose_new")
    contact_pair_pose_new :: proc() -> PxContactPairPose ---

    @(link_name = "PxContactPairIndex_new")
    contact_pair_index_new :: proc() -> PxContactPairIndex ---

    /// Constructor
    @(link_name = "PxContactPairExtraDataIterator_new")
    contact_pair_extra_data_iterator_new :: proc(stream: ^_c.uint8_t, size: _c.uint32_t) -> PxContactPairExtraDataIterator ---

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
    contact_pair_extra_data_iterator_next_item_set_mut :: proc(self_: ^PxContactPairExtraDataIterator) -> _c.bool ---

    @(link_name = "PxContactPairHeader_new")
    contact_pair_header_new :: proc() -> PxContactPairHeader ---

    @(link_name = "PxContactPair_new")
    contact_pair_new :: proc() -> PxContactPair ---

    /// Extracts the contact points from the stream and stores them in a convenient format.
    ///
    /// Number of contact points written to the buffer.
    @(link_name = "PxContactPair_extractContacts")
    contact_pair_extract_contacts :: proc(self_: ^PxContactPair, userBuffer: ^PxContactPairPoint, bufferSize: _c.uint32_t) -> _c.uint32_t ---

    /// Helper method to clone the contact pair and copy the contact data stream into a user buffer.
    ///
    /// The contact data stream is only accessible during the contact report callback. This helper function provides copy functionality
    /// to buffer the contact stream information such that it can get accessed at a later stage.
    @(link_name = "PxContactPair_bufferContacts")
    contact_pair_buffer_contacts :: proc(self_: ^PxContactPair, newPair: ^PxContactPair, bufferMemory: ^_c.uint8_t) ---

    @(link_name = "PxContactPair_getInternalFaceIndices")
    contact_pair_get_internal_face_indices :: proc(self_: ^PxContactPair) -> ^_c.uint32_t ---

    @(link_name = "PxTriggerPair_new")
    trigger_pair_new :: proc() -> PxTriggerPair ---

    @(link_name = "PxConstraintInfo_new")
    constraint_info_new :: proc() -> PxConstraintInfo ---

    @(link_name = "PxConstraintInfo_new_1")
    constraint_info_new_1 :: proc(c: ^PxConstraint, extRef: rawptr, t: _c.uint32_t) -> PxConstraintInfo ---

    /// This is called when a breakable constraint breaks.
    ///
    /// The user should not release the constraint shader inside this call!
    ///
    /// No event will get reported if the constraint breaks but gets deleted while the time step is still being simulated.
    @(link_name = "PxSimulationEventCallback_onConstraintBreak_mut")
    simulation_event_callback_on_constraint_break_mut :: proc(self_: ^PxSimulationEventCallback, constraints: ^PxConstraintInfo, count: _c.uint32_t) ---

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
    simulation_event_callback_on_wake_mut :: proc(self_: ^PxSimulationEventCallback, actors: ^^PxActor, count: _c.uint32_t) ---

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
    simulation_event_callback_on_sleep_mut :: proc(self_: ^PxSimulationEventCallback, actors: ^^PxActor, count: _c.uint32_t) ---

    /// This is called when certain contact events occur.
    ///
    /// The method will be called for a pair of actors if one of the colliding shape pairs requested contact notification.
    /// You request which events are reported using the filter shader/callback mechanism (see [`PxSimulationFilterShader`],
    /// [`PxSimulationFilterCallback`], #PxPairFlag).
    ///
    /// Do not keep references to the passed objects, as they will be
    /// invalid after this function returns.
    @(link_name = "PxSimulationEventCallback_onContact_mut")
    simulation_event_callback_on_contact_mut :: proc(self_: ^PxSimulationEventCallback, #by_ptr pairHeader: PxContactPairHeader, pairs: ^PxContactPair, nbPairs: _c.uint32_t) ---

    /// This is called with the current trigger pair events.
    ///
    /// Shapes which have been marked as triggers using PxShapeFlag::eTRIGGER_SHAPE will send events
    /// according to the pair flag specification in the filter shader (see [`PxPairFlag`], #PxSimulationFilterShader).
    ///
    /// Trigger shapes will no longer send notification events for interactions with other trigger shapes.
    @(link_name = "PxSimulationEventCallback_onTrigger_mut")
    simulation_event_callback_on_trigger_mut :: proc(self_: ^PxSimulationEventCallback, pairs: ^PxTriggerPair, count: _c.uint32_t) ---

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
    simulation_event_callback_on_advance_mut :: proc(self_: ^PxSimulationEventCallback, bodyBuffer: ^^PxRigidBody, poseBuffer: ^PxTransform, count: _c.uint32_t) ---

    @(link_name = "PxSimulationEventCallback_delete")
    simulation_event_callback_delete :: proc(self_: ^PxSimulationEventCallback) ---

    @(link_name = "PxFEMParameters_new")
    f_e_m_parameters_new :: proc() -> PxFEMParameters ---

    /// Release this object.
    @(link_name = "PxPruningStructure_release_mut")
    pruning_structure_release_mut :: proc(self_: ^PxPruningStructure) ---

    /// Retrieve rigid actors in the pruning structure.
    ///
    /// You can retrieve the number of rigid actor pointers by calling [`getNbRigidActors`]()
    ///
    /// Number of rigid actor pointers written to the buffer.
    @(link_name = "PxPruningStructure_getRigidActors")
    pruning_structure_get_rigid_actors :: proc(self_: ^PxPruningStructure, userBuffer: ^^PxRigidActor, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Returns the number of rigid actors in the pruning structure.
    ///
    /// You can use [`getRigidActors`]() to retrieve the rigid actor pointers.
    ///
    /// Number of rigid actors in the pruning structure.
    @(link_name = "PxPruningStructure_getNbRigidActors")
    pruning_structure_get_nb_rigid_actors :: proc(self_: ^PxPruningStructure) -> _c.uint32_t ---

    /// Gets the merge data for static actors
    ///
    /// This is mainly called by the PxSceneQuerySystem::merge() function to merge a PxPruningStructure
    /// with the internal data-structures of the scene-query system.
    ///
    /// Implementation-dependent merge data for static actors.
    @(link_name = "PxPruningStructure_getStaticMergeData")
    pruning_structure_get_static_merge_data :: proc(self_: ^PxPruningStructure) -> rawptr ---

    /// Gets the merge data for dynamic actors
    ///
    /// This is mainly called by the PxSceneQuerySystem::merge() function to merge a PxPruningStructure
    /// with the internal data-structures of the scene-query system.
    ///
    /// Implementation-dependent merge data for dynamic actors.
    @(link_name = "PxPruningStructure_getDynamicMergeData")
    pruning_structure_get_dynamic_merge_data :: proc(self_: ^PxPruningStructure) -> rawptr ---

    @(link_name = "PxPruningStructure_getConcreteTypeName")
    pruning_structure_get_concrete_type_name :: proc(self_: ^PxPruningStructure) -> ^_c.char ---

    @(link_name = "PxExtendedVec3_new")
    extended_vec3_new :: proc() -> PxExtendedVec3 ---

    @(link_name = "PxExtendedVec3_new_1")
    extended_vec3_new_1 :: proc(_x: _c.double, _y: _c.double, _z: _c.double) -> PxExtendedVec3 ---

    @(link_name = "PxExtendedVec3_isZero")
    extended_vec3_is_zero :: proc(self_: ^PxExtendedVec3) -> _c.bool ---

    @(link_name = "PxExtendedVec3_dot")
    extended_vec3_dot :: proc(self_: ^PxExtendedVec3, #by_ptr v: PxVec3) -> _c.double ---

    @(link_name = "PxExtendedVec3_distanceSquared")
    extended_vec3_distance_squared :: proc(self_: ^PxExtendedVec3, #by_ptr v: PxExtendedVec3) -> _c.double ---

    @(link_name = "PxExtendedVec3_magnitudeSquared")
    extended_vec3_magnitude_squared :: proc(self_: ^PxExtendedVec3) -> _c.double ---

    @(link_name = "PxExtendedVec3_magnitude")
    extended_vec3_magnitude :: proc(self_: ^PxExtendedVec3) -> _c.double ---

    @(link_name = "PxExtendedVec3_normalize_mut")
    extended_vec3_normalize_mut :: proc(self_: ^PxExtendedVec3) -> _c.double ---

    @(link_name = "PxExtendedVec3_isFinite")
    extended_vec3_is_finite :: proc(self_: ^PxExtendedVec3) -> _c.bool ---

    @(link_name = "PxExtendedVec3_maximum_mut")
    extended_vec3_maximum_mut :: proc(self_: ^PxExtendedVec3, #by_ptr v: PxExtendedVec3) ---

    @(link_name = "PxExtendedVec3_minimum_mut")
    extended_vec3_minimum_mut :: proc(self_: ^PxExtendedVec3, #by_ptr v: PxExtendedVec3) ---

    @(link_name = "PxExtendedVec3_set_mut")
    extended_vec3_set_mut :: proc(self_: ^PxExtendedVec3, x_: _c.double, y_: _c.double, z_: _c.double) ---

    @(link_name = "PxExtendedVec3_setPlusInfinity_mut")
    extended_vec3_set_plus_infinity_mut :: proc(self_: ^PxExtendedVec3) ---

    @(link_name = "PxExtendedVec3_setMinusInfinity_mut")
    extended_vec3_set_minus_infinity_mut :: proc(self_: ^PxExtendedVec3) ---

    @(link_name = "PxExtendedVec3_cross_mut")
    extended_vec3_cross_mut :: proc(self_: ^PxExtendedVec3, #by_ptr left: PxExtendedVec3, #by_ptr right: PxVec3) ---

    @(link_name = "PxExtendedVec3_cross_mut_1")
    extended_vec3_cross_mut_1 :: proc(self_: ^PxExtendedVec3, #by_ptr left: PxExtendedVec3, #by_ptr right: PxExtendedVec3) ---

    @(link_name = "PxExtendedVec3_cross")
    extended_vec3_cross :: proc(self_: ^PxExtendedVec3, #by_ptr v: PxExtendedVec3) -> PxExtendedVec3 ---

    @(link_name = "PxExtendedVec3_cross_mut_2")
    extended_vec3_cross_mut_2 :: proc(self_: ^PxExtendedVec3, #by_ptr left: PxVec3, #by_ptr right: PxExtendedVec3) ---

    @(link_name = "phys_toVec3")
    to_vec3 :: proc(#by_ptr v: PxExtendedVec3) -> PxVec3 ---

    @(link_name = "PxObstacle_getType")
    obstacle_get_type :: proc(self_: ^PxObstacle) -> PxGeometryType ---

    @(link_name = "PxBoxObstacle_new")
    box_obstacle_new :: proc() -> PxBoxObstacle ---

    @(link_name = "PxCapsuleObstacle_new")
    capsule_obstacle_new :: proc() -> PxCapsuleObstacle ---

    /// Releases the context.
    @(link_name = "PxObstacleContext_release_mut")
    obstacle_context_release_mut :: proc(self_: ^PxObstacleContext) ---

    /// Retrieves the controller manager associated with this context.
    ///
    /// The associated controller manager
    @(link_name = "PxObstacleContext_getControllerManager")
    obstacle_context_get_controller_manager :: proc(self_: ^PxObstacleContext) -> ^PxControllerManager ---

    /// Adds an obstacle to the context.
    ///
    /// Handle for newly-added obstacle
    @(link_name = "PxObstacleContext_addObstacle_mut")
    obstacle_context_add_obstacle_mut :: proc(self_: ^PxObstacleContext, obstacle: ^PxObstacle) -> _c.uint32_t ---

    /// Removes an obstacle from the context.
    ///
    /// True if success
    @(link_name = "PxObstacleContext_removeObstacle_mut")
    obstacle_context_remove_obstacle_mut :: proc(self_: ^PxObstacleContext, handle: _c.uint32_t) -> _c.bool ---

    /// Updates data for an existing obstacle.
    ///
    /// True if success
    @(link_name = "PxObstacleContext_updateObstacle_mut")
    obstacle_context_update_obstacle_mut :: proc(self_: ^PxObstacleContext, handle: _c.uint32_t, obstacle: ^PxObstacle) -> _c.bool ---

    /// Retrieves number of obstacles in the context.
    ///
    /// Number of obstacles in the context
    @(link_name = "PxObstacleContext_getNbObstacles")
    obstacle_context_get_nb_obstacles :: proc(self_: ^PxObstacleContext) -> _c.uint32_t ---

    /// Retrieves desired obstacle.
    ///
    /// Desired obstacle
    @(link_name = "PxObstacleContext_getObstacle")
    obstacle_context_get_obstacle :: proc(self_: ^PxObstacleContext, i: _c.uint32_t) -> ^PxObstacle ---

    /// Retrieves desired obstacle by given handle.
    ///
    /// Desired obstacle
    @(link_name = "PxObstacleContext_getObstacleByHandle")
    obstacle_context_get_obstacle_by_handle :: proc(self_: ^PxObstacleContext, handle: _c.uint32_t) -> ^PxObstacle ---

    /// Called when current controller hits a shape.
    ///
    /// This is called when the CCT moves and hits a shape. This will not be called when a moving shape hits a non-moving CCT.
    @(link_name = "PxUserControllerHitReport_onShapeHit_mut")
    user_controller_hit_report_on_shape_hit_mut :: proc(self_: ^PxUserControllerHitReport, #by_ptr hit: PxControllerShapeHit) ---

    /// Called when current controller hits another controller.
    @(link_name = "PxUserControllerHitReport_onControllerHit_mut")
    user_controller_hit_report_on_controller_hit_mut :: proc(self_: ^PxUserControllerHitReport, #by_ptr hit: PxControllersHit) ---

    /// Called when current controller hits a user-defined obstacle.
    @(link_name = "PxUserControllerHitReport_onObstacleHit_mut")
    user_controller_hit_report_on_obstacle_hit_mut :: proc(self_: ^PxUserControllerHitReport, #by_ptr hit: PxControllerObstacleHit) ---

    @(link_name = "PxControllerFilterCallback_delete")
    controller_filter_callback_delete :: proc(self_: ^PxControllerFilterCallback) ---

    /// Filtering method for CCT-vs-CCT.
    ///
    /// true to keep the pair, false to filter it out
    @(link_name = "PxControllerFilterCallback_filter_mut")
    controller_filter_callback_filter_mut :: proc(self_: ^PxControllerFilterCallback, a: ^PxController, b: ^PxController) -> _c.bool ---

    @(link_name = "PxControllerFilters_new")
    controller_filters_new :: proc(filterData: ^PxFilterData, cb: ^PxQueryFilterCallback, cctFilterCb: ^PxControllerFilterCallback) -> PxControllerFilters ---

    /// returns true if the current settings are valid
    ///
    /// True if the descriptor is valid.
    @(link_name = "PxControllerDesc_isValid")
    controller_desc_is_valid :: proc(self_: ^PxControllerDesc) -> _c.bool ---

    /// Returns the character controller type
    ///
    /// The controllers type.
    @(link_name = "PxControllerDesc_getType")
    controller_desc_get_type :: proc(self_: ^PxControllerDesc) -> PxControllerShapeType ---

    /// Return the type of controller
    @(link_name = "PxController_getType")
    controller_get_type :: proc(self_: ^PxController) -> PxControllerShapeType ---

    /// Releases the controller.
    @(link_name = "PxController_release_mut")
    controller_release_mut :: proc(self_: ^PxController) ---

    /// Moves the character using a "collide-and-slide" algorithm.
    ///
    /// Collision flags, collection of ::PxControllerCollisionFlags
    @(link_name = "PxController_move_mut")
    controller_move_mut :: proc(self_: ^PxController, #by_ptr disp: PxVec3, minDist: _c.float, elapsedTime: _c.float, #by_ptr filters: PxControllerFilters, obstacles: ^PxObstacleContext) -> PxControllerCollisionFlags_Set ---

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
    controller_set_position_mut :: proc(self_: ^PxController, #by_ptr position: PxExtendedVec3) -> _c.bool ---

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
    controller_get_position :: proc(self_: ^PxController) -> ^PxExtendedVec3 ---

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
    controller_set_foot_position_mut :: proc(self_: ^PxController, #by_ptr position: PxExtendedVec3) -> _c.bool ---

    /// Retrieve the "foot" position of the controller, i.e. the position of the bottom of the CCT's shape.
    ///
    /// The foot position takes the contact offset into account
    ///
    /// The controller's foot position
    @(link_name = "PxController_getFootPosition")
    controller_get_foot_position :: proc(self_: ^PxController) -> PxExtendedVec3 ---

    /// Get the rigid body actor associated with this controller (see PhysX documentation).
    /// The behavior upon manually altering this actor is undefined, you should primarily
    /// use it for reading const properties.
    ///
    /// the actor associated with the controller.
    @(link_name = "PxController_getActor")
    controller_get_actor :: proc(self_: ^PxController) -> ^PxRigidDynamic ---

    /// The step height.
    @(link_name = "PxController_setStepOffset_mut")
    controller_set_step_offset_mut :: proc(self_: ^PxController, offset: _c.float) ---

    /// Retrieve the step height.
    ///
    /// The step offset for the controller.
    @(link_name = "PxController_getStepOffset")
    controller_get_step_offset :: proc(self_: ^PxController) -> _c.float ---

    /// Sets the non-walkable mode for the CCT.
    @(link_name = "PxController_setNonWalkableMode_mut")
    controller_set_non_walkable_mode_mut :: proc(self_: ^PxController, flag: PxControllerNonWalkableMode) ---

    /// Retrieves the non-walkable mode for the CCT.
    ///
    /// The current non-walkable mode.
    @(link_name = "PxController_getNonWalkableMode")
    controller_get_non_walkable_mode :: proc(self_: ^PxController) -> PxControllerNonWalkableMode ---

    /// Retrieve the contact offset.
    ///
    /// The contact offset for the controller.
    @(link_name = "PxController_getContactOffset")
    controller_get_contact_offset :: proc(self_: ^PxController) -> _c.float ---

    /// Sets the contact offset.
    @(link_name = "PxController_setContactOffset_mut")
    controller_set_contact_offset_mut :: proc(self_: ^PxController, offset: _c.float) ---

    /// Retrieve the 'up' direction.
    ///
    /// The up direction for the controller.
    @(link_name = "PxController_getUpDirection")
    controller_get_up_direction :: proc(self_: ^PxController) -> PxVec3 ---

    /// Sets the 'up' direction.
    @(link_name = "PxController_setUpDirection_mut")
    controller_set_up_direction_mut :: proc(self_: ^PxController, #by_ptr up: PxVec3) ---

    /// Retrieve the slope limit.
    ///
    /// The slope limit for the controller.
    @(link_name = "PxController_getSlopeLimit")
    controller_get_slope_limit :: proc(self_: ^PxController) -> _c.float ---

    /// Sets the slope limit.
    ///
    /// This feature can not be enabled at runtime, i.e. if the slope limit is zero when creating the CCT
    /// (which disables the feature) then changing the slope limit at runtime will not have any effect, and the call
    /// will be ignored.
    @(link_name = "PxController_setSlopeLimit_mut")
    controller_set_slope_limit_mut :: proc(self_: ^PxController, slopeLimit: _c.float) ---

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
    controller_invalidate_cache_mut :: proc(self_: ^PxController) ---

    /// Retrieve the scene associated with the controller.
    ///
    /// The physics scene
    @(link_name = "PxController_getScene_mut")
    controller_get_scene_mut :: proc(self_: ^PxController) -> ^PxScene ---

    /// Returns the user data associated with this controller.
    ///
    /// The user pointer associated with the controller.
    @(link_name = "PxController_getUserData")
    controller_get_user_data :: proc(self_: ^PxController) -> rawptr ---

    /// Sets the user data associated with this controller.
    @(link_name = "PxController_setUserData_mut")
    controller_set_user_data_mut :: proc(self_: ^PxController, userData: rawptr) ---

    /// Returns information about the controller's internal state.
    @(link_name = "PxController_getState")
    controller_get_state :: proc(self_: ^PxController, state: ^PxControllerState) ---

    /// Returns the controller's internal statistics.
    @(link_name = "PxController_getStats")
    controller_get_stats :: proc(self_: ^PxController, stats: ^PxControllerStats) ---

    /// Resizes the controller.
    ///
    /// This function attempts to resize the controller to a given size, while making sure the bottom
    /// position of the controller remains constant. In other words the function modifies both the
    /// height and the (center) position of the controller. This is a helper function that can be used
    /// to implement a 'crouch' functionality for example.
    @(link_name = "PxController_resize_mut")
    controller_resize_mut :: proc(self_: ^PxController, height: _c.float) ---

    /// constructor sets to default.
    @(link_name = "PxBoxControllerDesc_new_alloc")
    box_controller_desc_new_alloc :: proc() -> ^PxBoxControllerDesc ---

    @(link_name = "PxBoxControllerDesc_delete")
    box_controller_desc_delete :: proc(self_: ^PxBoxControllerDesc) ---

    /// (re)sets the structure to the default.
    @(link_name = "PxBoxControllerDesc_setToDefault_mut")
    box_controller_desc_set_to_default_mut :: proc(self_: ^PxBoxControllerDesc) ---

    /// returns true if the current settings are valid
    ///
    /// True if the descriptor is valid.
    @(link_name = "PxBoxControllerDesc_isValid")
    box_controller_desc_is_valid :: proc(self_: ^PxBoxControllerDesc) -> _c.bool ---

    /// Gets controller's half height.
    ///
    /// The half height of the controller.
    @(link_name = "PxBoxController_getHalfHeight")
    box_controller_get_half_height :: proc(self_: ^PxBoxController) -> _c.float ---

    /// Gets controller's half side extent.
    ///
    /// The half side extent of the controller.
    @(link_name = "PxBoxController_getHalfSideExtent")
    box_controller_get_half_side_extent :: proc(self_: ^PxBoxController) -> _c.float ---

    /// Gets controller's half forward extent.
    ///
    /// The half forward extent of the controller.
    @(link_name = "PxBoxController_getHalfForwardExtent")
    box_controller_get_half_forward_extent :: proc(self_: ^PxBoxController) -> _c.float ---

    /// Sets controller's half height.
    ///
    /// this doesn't check for collisions.
    ///
    /// Currently always true.
    @(link_name = "PxBoxController_setHalfHeight_mut")
    box_controller_set_half_height_mut :: proc(self_: ^PxBoxController, halfHeight: _c.float) -> _c.bool ---

    /// Sets controller's half side extent.
    ///
    /// this doesn't check for collisions.
    ///
    /// Currently always true.
    @(link_name = "PxBoxController_setHalfSideExtent_mut")
    box_controller_set_half_side_extent_mut :: proc(self_: ^PxBoxController, halfSideExtent: _c.float) -> _c.bool ---

    /// Sets controller's half forward extent.
    ///
    /// this doesn't check for collisions.
    ///
    /// Currently always true.
    @(link_name = "PxBoxController_setHalfForwardExtent_mut")
    box_controller_set_half_forward_extent_mut :: proc(self_: ^PxBoxController, halfForwardExtent: _c.float) -> _c.bool ---

    /// constructor sets to default.
    @(link_name = "PxCapsuleControllerDesc_new_alloc")
    capsule_controller_desc_new_alloc :: proc() -> ^PxCapsuleControllerDesc ---

    @(link_name = "PxCapsuleControllerDesc_delete")
    capsule_controller_desc_delete :: proc(self_: ^PxCapsuleControllerDesc) ---

    /// (re)sets the structure to the default.
    @(link_name = "PxCapsuleControllerDesc_setToDefault_mut")
    capsule_controller_desc_set_to_default_mut :: proc(self_: ^PxCapsuleControllerDesc) ---

    /// returns true if the current settings are valid
    ///
    /// True if the descriptor is valid.
    @(link_name = "PxCapsuleControllerDesc_isValid")
    capsule_controller_desc_is_valid :: proc(self_: ^PxCapsuleControllerDesc) -> _c.bool ---

    /// Gets controller's radius.
    ///
    /// The radius of the controller.
    @(link_name = "PxCapsuleController_getRadius")
    capsule_controller_get_radius :: proc(self_: ^PxCapsuleController) -> _c.float ---

    /// Sets controller's radius.
    ///
    /// this doesn't check for collisions.
    ///
    /// Currently always true.
    @(link_name = "PxCapsuleController_setRadius_mut")
    capsule_controller_set_radius_mut :: proc(self_: ^PxCapsuleController, radius: _c.float) -> _c.bool ---

    /// Gets controller's height.
    ///
    /// The height of the capsule controller.
    @(link_name = "PxCapsuleController_getHeight")
    capsule_controller_get_height :: proc(self_: ^PxCapsuleController) -> _c.float ---

    /// Resets controller's height.
    ///
    /// this doesn't check for collisions.
    ///
    /// Currently always true.
    @(link_name = "PxCapsuleController_setHeight_mut")
    capsule_controller_set_height_mut :: proc(self_: ^PxCapsuleController, height: _c.float) -> _c.bool ---

    /// Gets controller's climbing mode.
    ///
    /// The capsule controller's climbing mode.
    @(link_name = "PxCapsuleController_getClimbingMode")
    capsule_controller_get_climbing_mode :: proc(self_: ^PxCapsuleController) -> PxCapsuleClimbingMode ---

    /// Sets controller's climbing mode.
    @(link_name = "PxCapsuleController_setClimbingMode_mut")
    capsule_controller_set_climbing_mode_mut :: proc(self_: ^PxCapsuleController, mode: PxCapsuleClimbingMode) -> _c.bool ---

    /// Retrieve behavior flags for a shape.
    ///
    /// When the CCT touches a shape, the CCT's behavior w.r.t. this shape can be customized by users.
    /// This function retrieves the desired PxControllerBehaviorFlag flags capturing the desired behavior.
    ///
    /// See comments about deprecated functions at the start of this class
    ///
    /// Desired behavior flags for the given shape
    @(link_name = "PxControllerBehaviorCallback_getBehaviorFlags_mut")
    controller_behavior_callback_get_behavior_flags_mut :: proc(self_: ^PxControllerBehaviorCallback, #by_ptr shape: PxShape, actor: ^PxActor) -> PxControllerBehaviorFlags_Set ---

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
    controller_behavior_callback_get_behavior_flags_mut_1 :: proc(self_: ^PxControllerBehaviorCallback, controller: ^PxController) -> PxControllerBehaviorFlags_Set ---

    /// Retrieve behavior flags for an obstacle.
    ///
    /// When the CCT touches an obstacle, the CCT's behavior w.r.t. this obstacle can be customized by users.
    /// This function retrieves the desired PxControllerBehaviorFlag flags capturing the desired behavior.
    ///
    /// See comments about deprecated functions at the start of this class
    ///
    /// Desired behavior flags for the given obstacle
    @(link_name = "PxControllerBehaviorCallback_getBehaviorFlags_mut_2")
    controller_behavior_callback_get_behavior_flags_mut_2 :: proc(self_: ^PxControllerBehaviorCallback, obstacle: ^PxObstacle) -> PxControllerBehaviorFlags_Set ---

    /// Releases the controller manager.
    ///
    /// This will release all associated controllers and obstacle contexts.
    ///
    /// This function is required to be called to release foundation usage.
    @(link_name = "PxControllerManager_release_mut")
    controller_manager_release_mut :: proc(self_: ^PxControllerManager) ---

    /// Returns the scene the manager is adding the controllers to.
    ///
    /// The associated physics scene.
    @(link_name = "PxControllerManager_getScene")
    controller_manager_get_scene :: proc(self_: ^PxControllerManager) -> ^PxScene ---

    /// Returns the number of controllers that are being managed.
    ///
    /// The number of controllers.
    @(link_name = "PxControllerManager_getNbControllers")
    controller_manager_get_nb_controllers :: proc(self_: ^PxControllerManager) -> _c.uint32_t ---

    /// Retrieve one of the controllers in the manager.
    ///
    /// The controller with the specified index.
    @(link_name = "PxControllerManager_getController_mut")
    controller_manager_get_controller_mut :: proc(self_: ^PxControllerManager, index: _c.uint32_t) -> ^PxController ---

    /// Creates a new character controller.
    ///
    /// The new controller
    @(link_name = "PxControllerManager_createController_mut")
    controller_manager_create_controller_mut :: proc(self_: ^PxControllerManager, desc: ^PxControllerDesc) -> ^PxController ---

    /// Releases all the controllers that are being managed.
    @(link_name = "PxControllerManager_purgeControllers_mut")
    controller_manager_purge_controllers_mut :: proc(self_: ^PxControllerManager) ---

    /// Retrieves debug data.
    ///
    /// The render buffer filled with debug-render data
    @(link_name = "PxControllerManager_getRenderBuffer_mut")
    controller_manager_get_render_buffer_mut :: proc(self_: ^PxControllerManager) -> ^PxRenderBuffer ---

    /// Sets debug rendering flags
    @(link_name = "PxControllerManager_setDebugRenderingFlags_mut")
    controller_manager_set_debug_rendering_flags_mut :: proc(self_: ^PxControllerManager, flags: PxControllerDebugRenderFlags_Set) ---

    /// Returns the number of obstacle contexts that are being managed.
    ///
    /// The number of obstacle contexts.
    @(link_name = "PxControllerManager_getNbObstacleContexts")
    controller_manager_get_nb_obstacle_contexts :: proc(self_: ^PxControllerManager) -> _c.uint32_t ---

    /// Retrieve one of the obstacle contexts in the manager.
    ///
    /// The obstacle context with the specified index.
    @(link_name = "PxControllerManager_getObstacleContext_mut")
    controller_manager_get_obstacle_context_mut :: proc(self_: ^PxControllerManager, index: _c.uint32_t) -> ^PxObstacleContext ---

    /// Creates an obstacle context.
    ///
    /// New obstacle context
    @(link_name = "PxControllerManager_createObstacleContext_mut")
    controller_manager_create_obstacle_context_mut :: proc(self_: ^PxControllerManager) -> ^PxObstacleContext ---

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
    controller_manager_compute_interactions_mut :: proc(self_: ^PxControllerManager, elapsedTime: _c.float, cctFilterCb: ^PxControllerFilterCallback) ---

    /// Enables or disables runtime tessellation.
    ///
    /// Large triangles can create accuracy issues in the sweep code, which in turn can lead to characters not sliding smoothly
    /// against geometries, or even penetrating them. This feature allows one to reduce those issues by tessellating large
    /// triangles at runtime, before performing sweeps against them. The amount of tessellation is controlled by the 'maxEdgeLength' parameter.
    /// Any triangle with at least one edge length greater than the maxEdgeLength will get recursively tessellated, until resulting triangles are small enough.
    ///
    /// This features only applies to triangle meshes, convex meshes, heightfields and boxes.
    @(link_name = "PxControllerManager_setTessellation_mut")
    controller_manager_set_tessellation_mut :: proc(self_: ^PxControllerManager, flag: _c.bool, maxEdgeLength: _c.float) ---

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
    controller_manager_set_overlap_recovery_module_mut :: proc(self_: ^PxControllerManager, flag: _c.bool) ---

    /// Enables or disables the precise sweeps.
    ///
    /// Precise sweeps are more accurate, but also potentially slower than regular sweeps.
    ///
    /// By default, precise sweeps are enabled.
    @(link_name = "PxControllerManager_setPreciseSweeps_mut")
    controller_manager_set_precise_sweeps_mut :: proc(self_: ^PxControllerManager, flag: _c.bool) ---

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
    controller_manager_set_prevent_vertical_sliding_against_ceiling_mut :: proc(self_: ^PxControllerManager, flag: _c.bool) ---

    /// Shift the origin of the character controllers and obstacle objects by the specified vector.
    ///
    /// The positions of all character controllers, obstacle objects and the corresponding data structures will get adjusted to reflect the shifted origin location
    /// (the shift vector will get subtracted from all character controller and obstacle object positions).
    ///
    /// It is the user's responsibility to keep track of the summed total origin shift and adjust all input/output to/from PhysXCharacterKinematic accordingly.
    ///
    /// This call will not automatically shift the PhysX scene and its objects. You need to call PxScene::shiftOrigin() seperately to keep the systems in sync.
    @(link_name = "PxControllerManager_shiftOrigin_mut")
    controller_manager_shift_origin_mut :: proc(self_: ^PxControllerManager, #by_ptr shift: PxVec3) ---

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
    create_controller_manager :: proc(scene: ^PxScene, lockingEnabled: _c.bool) -> ^PxControllerManager ---

    @(link_name = "PxDim3_new")
    dim3_new :: proc() -> PxDim3 ---

    /// Constructor
    @(link_name = "PxSDFDesc_new")
    s_d_f_desc_new :: proc() -> PxSDFDesc ---

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid
    @(link_name = "PxSDFDesc_isValid")
    s_d_f_desc_is_valid :: proc(self_: ^PxSDFDesc) -> _c.bool ---

    /// constructor sets to default.
    @(link_name = "PxConvexMeshDesc_new")
    convex_mesh_desc_new :: proc() -> PxConvexMeshDesc ---

    /// (re)sets the structure to the default.
    @(link_name = "PxConvexMeshDesc_setToDefault_mut")
    convex_mesh_desc_set_to_default_mut :: proc(self_: ^PxConvexMeshDesc) ---

    /// Returns true if the descriptor is valid.
    ///
    /// True if the current settings are valid
    @(link_name = "PxConvexMeshDesc_isValid")
    convex_mesh_desc_is_valid :: proc(self_: ^PxConvexMeshDesc) -> _c.bool ---

    /// Constructor sets to default.
    @(link_name = "PxTriangleMeshDesc_new")
    triangle_mesh_desc_new :: proc() -> PxTriangleMeshDesc ---

    /// (re)sets the structure to the default.
    @(link_name = "PxTriangleMeshDesc_setToDefault_mut")
    triangle_mesh_desc_set_to_default_mut :: proc(self_: ^PxTriangleMeshDesc) ---

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid
    @(link_name = "PxTriangleMeshDesc_isValid")
    triangle_mesh_desc_is_valid :: proc(self_: ^PxTriangleMeshDesc) -> _c.bool ---

    /// Constructor to build an empty tetmesh description
    @(link_name = "PxTetrahedronMeshDesc_new")
    tetrahedron_mesh_desc_new :: proc() -> PxTetrahedronMeshDesc ---

    @(link_name = "PxTetrahedronMeshDesc_isValid")
    tetrahedron_mesh_desc_is_valid :: proc(self_: ^PxTetrahedronMeshDesc) -> _c.bool ---

    /// Constructor to build an empty simulation description
    @(link_name = "PxSoftBodySimulationDataDesc_new")
    soft_body_simulation_data_desc_new :: proc() -> PxSoftBodySimulationDataDesc ---

    @(link_name = "PxSoftBodySimulationDataDesc_isValid")
    soft_body_simulation_data_desc_is_valid :: proc(self_: ^PxSoftBodySimulationDataDesc) -> _c.bool ---

    /// Desc initialization to default value.
    @(link_name = "PxBVH34MidphaseDesc_setToDefault_mut")
    b_v_h34_midphase_desc_set_to_default_mut :: proc(self_: ^PxBVH34MidphaseDesc) ---

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid.
    @(link_name = "PxBVH34MidphaseDesc_isValid")
    b_v_h34_midphase_desc_is_valid :: proc(self_: ^PxBVH34MidphaseDesc) -> _c.bool ---

    @(link_name = "PxMidphaseDesc_new")
    midphase_desc_new :: proc() -> PxMidphaseDesc ---

    /// Returns type of midphase mesh structure.
    ///
    /// PxMeshMidPhase::Enum
    @(link_name = "PxMidphaseDesc_getType")
    midphase_desc_get_type :: proc(self_: ^PxMidphaseDesc) -> PxMeshMidPhase ---

    /// Initialize the midphase mesh structure descriptor
    @(link_name = "PxMidphaseDesc_setToDefault_mut")
    midphase_desc_set_to_default_mut :: proc(self_: ^PxMidphaseDesc, type: PxMeshMidPhase) ---

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid.
    @(link_name = "PxMidphaseDesc_isValid")
    midphase_desc_is_valid :: proc(self_: ^PxMidphaseDesc) -> _c.bool ---

    @(link_name = "PxBVHDesc_new")
    b_v_h_desc_new :: proc() -> PxBVHDesc ---

    /// Initialize the BVH descriptor
    @(link_name = "PxBVHDesc_setToDefault_mut")
    b_v_h_desc_set_to_default_mut :: proc(self_: ^PxBVHDesc) ---

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid.
    @(link_name = "PxBVHDesc_isValid")
    b_v_h_desc_is_valid :: proc(self_: ^PxBVHDesc) -> _c.bool ---

    @(link_name = "PxCookingParams_new")
    cooking_params_new :: proc(#by_ptr sc: PxTolerancesScale) -> PxCookingParams ---

    @(link_name = "phys_PxGetStandaloneInsertionCallback")
    get_standalone_insertion_callback :: proc() -> ^PxInsertionCallback ---

    /// Cooks a bounding volume hierarchy. The results are written to the stream.
    ///
    /// PxCookBVH() allows a BVH description to be cooked into a binary stream
    /// suitable for loading and performing BVH detection at runtime.
    ///
    /// true on success.
    @(link_name = "phys_PxCookBVH")
    cook_b_v_h :: proc(#by_ptr desc: PxBVHDesc, stream: ^PxOutputStream) -> _c.bool ---

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
    create_b_v_h :: proc(#by_ptr desc: PxBVHDesc, insertionCallback: ^PxInsertionCallback) -> ^PxBVH ---

    /// Cooks a heightfield. The results are written to the stream.
    ///
    /// To create a heightfield object there is an option to precompute some of calculations done while loading the heightfield data.
    ///
    /// cookHeightField() allows a heightfield description to be cooked into a binary stream
    /// suitable for loading and performing collision detection at runtime.
    ///
    /// true on success
    @(link_name = "phys_PxCookHeightField")
    cook_height_field :: proc(#by_ptr desc: PxHeightFieldDesc, stream: ^PxOutputStream) -> _c.bool ---

    /// Cooks and creates a heightfield mesh and inserts it into PxPhysics.
    ///
    /// PxHeightField pointer on success
    @(link_name = "phys_PxCreateHeightField")
    create_height_field :: proc(#by_ptr desc: PxHeightFieldDesc, insertionCallback: ^PxInsertionCallback) -> ^PxHeightField ---

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
    cook_convex_mesh :: proc(#by_ptr params: PxCookingParams, #by_ptr desc: PxConvexMeshDesc, stream: ^PxOutputStream, condition: ^PxConvexMeshCookingResult) -> _c.bool ---

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
    create_convex_mesh :: proc(#by_ptr params: PxCookingParams, #by_ptr desc: PxConvexMeshDesc, insertionCallback: ^PxInsertionCallback, condition: ^PxConvexMeshCookingResult) -> ^PxConvexMesh ---

    /// Verifies if the convex mesh is valid. Prints an error message for each inconsistency found.
    ///
    /// The convex mesh descriptor must contain an already created convex mesh - the vertices, indices and polygons must be provided.
    ///
    /// This function should be used if PxConvexFlag::eDISABLE_MESH_VALIDATION is planned to be used in release builds.
    ///
    /// true if all the validity conditions hold, false otherwise.
    @(link_name = "phys_PxValidateConvexMesh")
    validate_convex_mesh :: proc(#by_ptr params: PxCookingParams, #by_ptr desc: PxConvexMeshDesc) -> _c.bool ---

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
    compute_hull_polygons :: proc(#by_ptr params: PxCookingParams, mesh: ^PxSimpleTriangleMesh, inCallback: ^PxAllocatorCallback, nbVerts: ^_c.uint32_t, vertices: ^^PxVec3, nbIndices: ^_c.uint32_t, indices: ^^_c.uint32_t, nbPolygons: ^_c.uint32_t, hullPolygons: ^^PxHullPolygon) -> _c.bool ---

    /// Verifies if the triangle mesh is valid. Prints an error message for each inconsistency found.
    ///
    /// The following conditions are true for a valid triangle mesh:
    /// 1. There are no duplicate vertices (within specified vertexWeldTolerance. See PxCookingParams::meshWeldTolerance)
    /// 2. There are no large triangles (within specified PxTolerancesScale.)
    ///
    /// true if all the validity conditions hold, false otherwise.
    @(link_name = "phys_PxValidateTriangleMesh")
    validate_triangle_mesh :: proc(#by_ptr params: PxCookingParams, #by_ptr desc: PxTriangleMeshDesc) -> _c.bool ---

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
    create_triangle_mesh :: proc(#by_ptr params: PxCookingParams, #by_ptr desc: PxTriangleMeshDesc, insertionCallback: ^PxInsertionCallback, condition: ^PxTriangleMeshCookingResult) -> ^PxTriangleMesh ---

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
    cook_triangle_mesh :: proc(#by_ptr params: PxCookingParams, #by_ptr desc: PxTriangleMeshDesc, stream: ^PxOutputStream, condition: ^PxTriangleMeshCookingResult) -> _c.bool ---

    @(link_name = "PxDefaultMemoryOutputStream_new_alloc")
    default_memory_output_stream_new_alloc :: proc(allocator: ^PxAllocatorCallback) -> ^PxDefaultMemoryOutputStream ---

    @(link_name = "PxDefaultMemoryOutputStream_delete")
    default_memory_output_stream_delete :: proc(self_: ^PxDefaultMemoryOutputStream) ---

    @(link_name = "PxDefaultMemoryOutputStream_write_mut")
    default_memory_output_stream_write_mut :: proc(self_: ^PxDefaultMemoryOutputStream, src: rawptr, count: _c.uint32_t) -> _c.uint32_t ---

    @(link_name = "PxDefaultMemoryOutputStream_getSize")
    default_memory_output_stream_get_size :: proc(self_: ^PxDefaultMemoryOutputStream) -> _c.uint32_t ---

    @(link_name = "PxDefaultMemoryOutputStream_getData")
    default_memory_output_stream_get_data :: proc(self_: ^PxDefaultMemoryOutputStream) -> ^_c.uint8_t ---

    @(link_name = "PxDefaultMemoryInputData_new_alloc")
    default_memory_input_data_new_alloc :: proc(data: ^_c.uint8_t, length: _c.uint32_t) -> ^PxDefaultMemoryInputData ---

    @(link_name = "PxDefaultMemoryInputData_read_mut")
    default_memory_input_data_read_mut :: proc(self_: ^PxDefaultMemoryInputData, dest: rawptr, count: _c.uint32_t) -> _c.uint32_t ---

    @(link_name = "PxDefaultMemoryInputData_getLength")
    default_memory_input_data_get_length :: proc(self_: ^PxDefaultMemoryInputData) -> _c.uint32_t ---

    @(link_name = "PxDefaultMemoryInputData_seek_mut")
    default_memory_input_data_seek_mut :: proc(self_: ^PxDefaultMemoryInputData, pos: _c.uint32_t) ---

    @(link_name = "PxDefaultMemoryInputData_tell")
    default_memory_input_data_tell :: proc(self_: ^PxDefaultMemoryInputData) -> _c.uint32_t ---

    @(link_name = "PxDefaultFileOutputStream_new_alloc")
    default_file_output_stream_new_alloc :: proc(name: ^_c.char) -> ^PxDefaultFileOutputStream ---

    @(link_name = "PxDefaultFileOutputStream_delete")
    default_file_output_stream_delete :: proc(self_: ^PxDefaultFileOutputStream) ---

    @(link_name = "PxDefaultFileOutputStream_write_mut")
    default_file_output_stream_write_mut :: proc(self_: ^PxDefaultFileOutputStream, src: rawptr, count: _c.uint32_t) -> _c.uint32_t ---

    @(link_name = "PxDefaultFileOutputStream_isValid_mut")
    default_file_output_stream_is_valid_mut :: proc(self_: ^PxDefaultFileOutputStream) -> _c.bool ---

    @(link_name = "PxDefaultFileInputData_new_alloc")
    default_file_input_data_new_alloc :: proc(name: ^_c.char) -> ^PxDefaultFileInputData ---

    @(link_name = "PxDefaultFileInputData_delete")
    default_file_input_data_delete :: proc(self_: ^PxDefaultFileInputData) ---

    @(link_name = "PxDefaultFileInputData_read_mut")
    default_file_input_data_read_mut :: proc(self_: ^PxDefaultFileInputData, dest: rawptr, count: _c.uint32_t) -> _c.uint32_t ---

    @(link_name = "PxDefaultFileInputData_seek_mut")
    default_file_input_data_seek_mut :: proc(self_: ^PxDefaultFileInputData, pos: _c.uint32_t) ---

    @(link_name = "PxDefaultFileInputData_tell")
    default_file_input_data_tell :: proc(self_: ^PxDefaultFileInputData) -> _c.uint32_t ---

    @(link_name = "PxDefaultFileInputData_getLength")
    default_file_input_data_get_length :: proc(self_: ^PxDefaultFileInputData) -> _c.uint32_t ---

    @(link_name = "PxDefaultFileInputData_isValid")
    default_file_input_data_is_valid :: proc(self_: ^PxDefaultFileInputData) -> _c.bool ---

    @(link_name = "phys_platformAlignedAlloc")
    platform_aligned_alloc :: proc(size: _c.size_t) -> rawptr ---

    @(link_name = "phys_platformAlignedFree")
    platform_aligned_free :: proc(ptr: rawptr) ---

    @(link_name = "PxDefaultAllocator_allocate_mut")
    default_allocator_allocate_mut :: proc(self_: ^PxDefaultAllocator, size: _c.size_t, anon_param1: ^_c.char, anon_param2: ^_c.char, anon_param3: _c.int32_t) -> rawptr ---

    @(link_name = "PxDefaultAllocator_deallocate_mut")
    default_allocator_deallocate_mut :: proc(self_: ^PxDefaultAllocator, ptr: rawptr) ---

    @(link_name = "PxDefaultAllocator_delete")
    default_allocator_delete :: proc(self_: ^PxDefaultAllocator) ---

    /// Set the actors for this joint.
    ///
    /// An actor may be NULL to indicate the world frame. At most one of the actors may be NULL.
    @(link_name = "PxJoint_setActors_mut")
    joint_set_actors_mut :: proc(self_: ^PxJoint, actor0: ^PxRigidActor, actor1: ^PxRigidActor) ---

    /// Get the actors for this joint.
    @(link_name = "PxJoint_getActors")
    joint_get_actors :: proc(self_: ^PxJoint, actor0: ^^PxRigidActor, actor1: ^^PxRigidActor) ---

    /// Set the joint local pose for an actor.
    ///
    /// This is the relative pose which locates the joint frame relative to the actor.
    @(link_name = "PxJoint_setLocalPose_mut")
    joint_set_local_pose_mut :: proc(self_: ^PxJoint, actor: PxJointActorIndex, #by_ptr localPose: PxTransform) ---

    /// get the joint local pose for an actor.
    ///
    /// return the local pose for this joint
    @(link_name = "PxJoint_getLocalPose")
    joint_get_local_pose :: proc(self_: ^PxJoint, actor: PxJointActorIndex) -> PxTransform ---

    /// get the relative pose for this joint
    ///
    /// This function returns the pose of the joint frame of actor1 relative to actor0
    @(link_name = "PxJoint_getRelativeTransform")
    joint_get_relative_transform :: proc(self_: ^PxJoint) -> PxTransform ---

    /// get the relative linear velocity of the joint
    ///
    /// This function returns the linear velocity of the origin of the constraint frame of actor1, relative to the origin of the constraint
    /// frame of actor0. The value is returned in the constraint frame of actor0
    @(link_name = "PxJoint_getRelativeLinearVelocity")
    joint_get_relative_linear_velocity :: proc(self_: ^PxJoint) -> PxVec3 ---

    /// get the relative angular velocity of the joint
    ///
    /// This function returns the angular velocity of  actor1 relative to actor0. The value is returned in the constraint frame of actor0
    @(link_name = "PxJoint_getRelativeAngularVelocity")
    joint_get_relative_angular_velocity :: proc(self_: ^PxJoint) -> PxVec3 ---

    /// set the break force for this joint.
    ///
    /// if the constraint force or torque on the joint exceeds the specified values, the joint will break,
    /// at which point it will not constrain the two actors and the flag PxConstraintFlag::eBROKEN will be set. The
    /// force and torque are measured in the joint frame of the first actor
    @(link_name = "PxJoint_setBreakForce_mut")
    joint_set_break_force_mut :: proc(self_: ^PxJoint, force: _c.float, torque: _c.float) ---

    /// get the break force for this joint.
    @(link_name = "PxJoint_getBreakForce")
    joint_get_break_force :: proc(self_: ^PxJoint, force: ^_c.float, torque: ^_c.float) ---

    /// set the constraint flags for this joint.
    @(link_name = "PxJoint_setConstraintFlags_mut")
    joint_set_constraint_flags_mut :: proc(self_: ^PxJoint, flags: PxConstraintFlags_Set) ---

    /// set a constraint flags for this joint to a specified value.
    @(link_name = "PxJoint_setConstraintFlag_mut")
    joint_set_constraint_flag_mut :: proc(self_: ^PxJoint, flag: PxConstraintFlag, value: _c.bool) ---

    /// get the constraint flags for this joint.
    ///
    /// the constraint flags
    @(link_name = "PxJoint_getConstraintFlags")
    joint_get_constraint_flags :: proc(self_: ^PxJoint) -> PxConstraintFlags_Set ---

    /// set the inverse mass scale for actor0.
    @(link_name = "PxJoint_setInvMassScale0_mut")
    joint_set_inv_mass_scale0_mut :: proc(self_: ^PxJoint, invMassScale: _c.float) ---

    /// get the inverse mass scale for actor0.
    ///
    /// inverse mass scale for actor0
    @(link_name = "PxJoint_getInvMassScale0")
    joint_get_inv_mass_scale0 :: proc(self_: ^PxJoint) -> _c.float ---

    /// set the inverse inertia scale for actor0.
    @(link_name = "PxJoint_setInvInertiaScale0_mut")
    joint_set_inv_inertia_scale0_mut :: proc(self_: ^PxJoint, invInertiaScale: _c.float) ---

    /// get the inverse inertia scale for actor0.
    ///
    /// inverse inertia scale for actor0
    @(link_name = "PxJoint_getInvInertiaScale0")
    joint_get_inv_inertia_scale0 :: proc(self_: ^PxJoint) -> _c.float ---

    /// set the inverse mass scale for actor1.
    @(link_name = "PxJoint_setInvMassScale1_mut")
    joint_set_inv_mass_scale1_mut :: proc(self_: ^PxJoint, invMassScale: _c.float) ---

    /// get the inverse mass scale for actor1.
    ///
    /// inverse mass scale for actor1
    @(link_name = "PxJoint_getInvMassScale1")
    joint_get_inv_mass_scale1 :: proc(self_: ^PxJoint) -> _c.float ---

    /// set the inverse inertia scale for actor1.
    @(link_name = "PxJoint_setInvInertiaScale1_mut")
    joint_set_inv_inertia_scale1_mut :: proc(self_: ^PxJoint, invInertiaScale: _c.float) ---

    /// get the inverse inertia scale for actor1.
    ///
    /// inverse inertia scale for actor1
    @(link_name = "PxJoint_getInvInertiaScale1")
    joint_get_inv_inertia_scale1 :: proc(self_: ^PxJoint) -> _c.float ---

    /// Retrieves the PxConstraint corresponding to this joint.
    ///
    /// This can be used to determine, among other things, the force applied at the joint.
    ///
    /// the constraint
    @(link_name = "PxJoint_getConstraint")
    joint_get_constraint :: proc(self_: ^PxJoint) -> ^PxConstraint ---

    /// Sets a name string for the object that can be retrieved with getName().
    ///
    /// This is for debugging and is not used by the SDK. The string is not copied by the SDK,
    /// only the pointer is stored.
    @(link_name = "PxJoint_setName_mut")
    joint_set_name_mut :: proc(self_: ^PxJoint, name: ^_c.char) ---

    /// Retrieves the name string set with setName().
    ///
    /// Name string associated with object.
    @(link_name = "PxJoint_getName")
    joint_get_name :: proc(self_: ^PxJoint) -> ^_c.char ---

    /// Deletes the joint.
    ///
    /// This call does not wake up the connected rigid bodies.
    @(link_name = "PxJoint_release_mut")
    joint_release_mut :: proc(self_: ^PxJoint) ---

    /// Retrieves the scene which this joint belongs to.
    ///
    /// Owner Scene. NULL if not part of a scene.
    @(link_name = "PxJoint_getScene")
    joint_get_scene :: proc(self_: ^PxJoint) -> ^PxScene ---

    /// Put class meta data in stream, used for serialization
    @(link_name = "PxJoint_getBinaryMetaData")
    joint_get_binary_meta_data :: proc(stream: ^PxOutputStream) ---

    @(link_name = "PxSpring_new")
    spring_new :: proc(stiffness_: _c.float, damping_: _c.float) -> PxSpring ---

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
    set_joint_global_frame :: proc(joint: ^PxJoint, wsAnchor: ^PxVec3, wsAxis: ^PxVec3) ---

    /// Create a distance Joint.
    @(link_name = "phys_PxDistanceJointCreate")
    distance_joint_create :: proc(physics: ^PxPhysics, actor0: ^PxRigidActor, #by_ptr localFrame0: PxTransform, actor1: ^PxRigidActor, #by_ptr localFrame1: PxTransform) -> ^PxDistanceJoint ---

    /// Return the current distance of the joint
    @(link_name = "PxDistanceJoint_getDistance")
    distance_joint_get_distance :: proc(self_: ^PxDistanceJoint) -> _c.float ---

    /// Set the allowed minimum distance for the joint.
    ///
    /// The minimum distance must be no more than the maximum distance
    ///
    /// Default
    /// 0.0f
    /// Range
    /// [0, PX_MAX_F32)
    @(link_name = "PxDistanceJoint_setMinDistance_mut")
    distance_joint_set_min_distance_mut :: proc(self_: ^PxDistanceJoint, distance: _c.float) ---

    /// Get the allowed minimum distance for the joint.
    ///
    /// the allowed minimum distance
    @(link_name = "PxDistanceJoint_getMinDistance")
    distance_joint_get_min_distance :: proc(self_: ^PxDistanceJoint) -> _c.float ---

    /// Set the allowed maximum distance for the joint.
    ///
    /// The maximum distance must be no less than the minimum distance.
    ///
    /// Default
    /// 0.0f
    /// Range
    /// [0, PX_MAX_F32)
    @(link_name = "PxDistanceJoint_setMaxDistance_mut")
    distance_joint_set_max_distance_mut :: proc(self_: ^PxDistanceJoint, distance: _c.float) ---

    /// Get the allowed maximum distance for the joint.
    ///
    /// the allowed maximum distance
    @(link_name = "PxDistanceJoint_getMaxDistance")
    distance_joint_get_max_distance :: proc(self_: ^PxDistanceJoint) -> _c.float ---

    /// Set the error tolerance of the joint.
    @(link_name = "PxDistanceJoint_setTolerance_mut")
    distance_joint_set_tolerance_mut :: proc(self_: ^PxDistanceJoint, tolerance: _c.float) ---

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
    distance_joint_get_tolerance :: proc(self_: ^PxDistanceJoint) -> _c.float ---

    /// Set the strength of the joint spring.
    ///
    /// The spring is used if enabled, and the distance exceeds the range [min-error, max+error].
    ///
    /// Default
    /// 0.0f
    /// Range
    /// [0, PX_MAX_F32)
    @(link_name = "PxDistanceJoint_setStiffness_mut")
    distance_joint_set_stiffness_mut :: proc(self_: ^PxDistanceJoint, stiffness: _c.float) ---

    /// Get the strength of the joint spring.
    ///
    /// stiffness the spring strength of the joint
    @(link_name = "PxDistanceJoint_getStiffness")
    distance_joint_get_stiffness :: proc(self_: ^PxDistanceJoint) -> _c.float ---

    /// Set the damping of the joint spring.
    ///
    /// The spring is used if enabled, and the distance exceeds the range [min-error, max+error].
    ///
    /// Default
    /// 0.0f
    /// Range
    /// [0, PX_MAX_F32)
    @(link_name = "PxDistanceJoint_setDamping_mut")
    distance_joint_set_damping_mut :: proc(self_: ^PxDistanceJoint, damping: _c.float) ---

    /// Get the damping of the joint spring.
    ///
    /// the degree of damping of the joint spring of the joint
    @(link_name = "PxDistanceJoint_getDamping")
    distance_joint_get_damping :: proc(self_: ^PxDistanceJoint) -> _c.float ---

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
    distance_joint_set_contact_distance_mut :: proc(self_: ^PxDistanceJoint, contactDistance: _c.float) ---

    /// Get the contact distance.
    ///
    /// the contact distance
    @(link_name = "PxDistanceJoint_getContactDistance")
    distance_joint_get_contact_distance :: proc(self_: ^PxDistanceJoint) -> _c.float ---

    /// Set the flags specific to the Distance Joint.
    ///
    /// Default
    /// PxDistanceJointFlag::eMAX_DISTANCE_ENABLED
    @(link_name = "PxDistanceJoint_setDistanceJointFlags_mut")
    distance_joint_set_distance_joint_flags_mut :: proc(self_: ^PxDistanceJoint, flags: PxDistanceJointFlags_Set) ---

    /// Set a single flag specific to a Distance Joint to true or false.
    @(link_name = "PxDistanceJoint_setDistanceJointFlag_mut")
    distance_joint_set_distance_joint_flag_mut :: proc(self_: ^PxDistanceJoint, flag: PxDistanceJointFlag, value: _c.bool) ---

    /// Get the flags specific to the Distance Joint.
    ///
    /// the joint flags
    @(link_name = "PxDistanceJoint_getDistanceJointFlags")
    distance_joint_get_distance_joint_flags :: proc(self_: ^PxDistanceJoint) -> PxDistanceJointFlags_Set ---

    /// Returns string name of PxDistanceJoint, used for serialization
    @(link_name = "PxDistanceJoint_getConcreteTypeName")
    distance_joint_get_concrete_type_name :: proc(self_: ^PxDistanceJoint) -> ^_c.char ---

    /// Create a distance Joint.
    @(link_name = "phys_PxContactJointCreate")
    contact_joint_create :: proc(physics: ^PxPhysics, actor0: ^PxRigidActor, #by_ptr localFrame0: PxTransform, actor1: ^PxRigidActor, #by_ptr localFrame1: PxTransform) -> ^PxContactJoint ---

    @(link_name = "PxJacobianRow_new")
    jacobian_row_new :: proc() -> PxJacobianRow ---

    @(link_name = "PxJacobianRow_new_1")
    jacobian_row_new_1 :: proc(#by_ptr lin0: PxVec3, #by_ptr lin1: PxVec3, #by_ptr ang0: PxVec3, #by_ptr ang1: PxVec3) -> PxJacobianRow ---

    /// Set the current contact of the joint
    @(link_name = "PxContactJoint_setContact_mut")
    contact_joint_set_contact_mut :: proc(self_: ^PxContactJoint, #by_ptr contact: PxVec3) ---

    /// Set the current contact normal of the joint
    @(link_name = "PxContactJoint_setContactNormal_mut")
    contact_joint_set_contact_normal_mut :: proc(self_: ^PxContactJoint, #by_ptr contactNormal: PxVec3) ---

    /// Set the current penetration of the joint
    @(link_name = "PxContactJoint_setPenetration_mut")
    contact_joint_set_penetration_mut :: proc(self_: ^PxContactJoint, penetration: _c.float) ---

    /// Return the current contact of the joint
    @(link_name = "PxContactJoint_getContact")
    contact_joint_get_contact :: proc(self_: ^PxContactJoint) -> PxVec3 ---

    /// Return the current contact normal of the joint
    @(link_name = "PxContactJoint_getContactNormal")
    contact_joint_get_contact_normal :: proc(self_: ^PxContactJoint) -> PxVec3 ---

    /// Return the current penetration value of the joint
    @(link_name = "PxContactJoint_getPenetration")
    contact_joint_get_penetration :: proc(self_: ^PxContactJoint) -> _c.float ---

    @(link_name = "PxContactJoint_getRestitution")
    contact_joint_get_restitution :: proc(self_: ^PxContactJoint) -> _c.float ---

    @(link_name = "PxContactJoint_setRestitution_mut")
    contact_joint_set_restitution_mut :: proc(self_: ^PxContactJoint, restitution: _c.float) ---

    @(link_name = "PxContactJoint_getBounceThreshold")
    contact_joint_get_bounce_threshold :: proc(self_: ^PxContactJoint) -> _c.float ---

    @(link_name = "PxContactJoint_setBounceThreshold_mut")
    contact_joint_set_bounce_threshold_mut :: proc(self_: ^PxContactJoint, bounceThreshold: _c.float) ---

    /// Returns string name of PxContactJoint, used for serialization
    @(link_name = "PxContactJoint_getConcreteTypeName")
    contact_joint_get_concrete_type_name :: proc(self_: ^PxContactJoint) -> ^_c.char ---

    @(link_name = "PxContactJoint_computeJacobians")
    contact_joint_compute_jacobians :: proc(self_: ^PxContactJoint, jacobian: ^PxJacobianRow) ---

    @(link_name = "PxContactJoint_getNbJacobianRows")
    contact_joint_get_nb_jacobian_rows :: proc(self_: ^PxContactJoint) -> _c.uint32_t ---

    /// Create a fixed joint.
    @(link_name = "phys_PxFixedJointCreate")
    fixed_joint_create :: proc(physics: ^PxPhysics, actor0: ^PxRigidActor, #by_ptr localFrame0: PxTransform, actor1: ^PxRigidActor, #by_ptr localFrame1: PxTransform) -> ^PxFixedJoint ---

    /// Returns string name of PxFixedJoint, used for serialization
    @(link_name = "PxFixedJoint_getConcreteTypeName")
    fixed_joint_get_concrete_type_name :: proc(self_: ^PxFixedJoint) -> ^_c.char ---

    @(link_name = "PxJointLimitParameters_new_alloc")
    joint_limit_parameters_new_alloc :: proc() -> ^PxJointLimitParameters ---

    /// Returns true if the current settings are valid.
    ///
    /// true if the current settings are valid
    @(link_name = "PxJointLimitParameters_isValid")
    joint_limit_parameters_is_valid :: proc(self_: ^PxJointLimitParameters) -> _c.bool ---

    @(link_name = "PxJointLimitParameters_isSoft")
    joint_limit_parameters_is_soft :: proc(self_: ^PxJointLimitParameters) -> _c.bool ---

    /// construct a linear hard limit
    @(link_name = "PxJointLinearLimit_new")
    joint_linear_limit_new :: proc(#by_ptr scale: PxTolerancesScale, extent: _c.float, contactDist_deprecated: _c.float) -> PxJointLinearLimit ---

    /// construct a linear soft limit
    @(link_name = "PxJointLinearLimit_new_1")
    joint_linear_limit_new_1 :: proc(extent: _c.float, spring: ^PxSpring) -> PxJointLinearLimit ---

    /// Returns true if the limit is valid
    ///
    /// true if the current settings are valid
    @(link_name = "PxJointLinearLimit_isValid")
    joint_linear_limit_is_valid :: proc(self_: ^PxJointLinearLimit) -> _c.bool ---

    @(link_name = "PxJointLinearLimit_delete")
    joint_linear_limit_delete :: proc(self_: ^PxJointLinearLimit) ---

    /// Construct a linear hard limit pair. The lower distance value must be less than the upper distance value.
    @(link_name = "PxJointLinearLimitPair_new")
    joint_linear_limit_pair_new :: proc(#by_ptr scale: PxTolerancesScale, lowerLimit: _c.float, upperLimit: _c.float, contactDist_deprecated: _c.float) -> PxJointLinearLimitPair ---

    /// construct a linear soft limit pair
    @(link_name = "PxJointLinearLimitPair_new_1")
    joint_linear_limit_pair_new_1 :: proc(lowerLimit: _c.float, upperLimit: _c.float, spring: ^PxSpring) -> PxJointLinearLimitPair ---

    /// Returns true if the limit is valid.
    ///
    /// true if the current settings are valid
    @(link_name = "PxJointLinearLimitPair_isValid")
    joint_linear_limit_pair_is_valid :: proc(self_: ^PxJointLinearLimitPair) -> _c.bool ---

    @(link_name = "PxJointLinearLimitPair_delete")
    joint_linear_limit_pair_delete :: proc(self_: ^PxJointLinearLimitPair) ---

    /// construct an angular hard limit pair.
    ///
    /// The lower value must be less than the upper value.
    @(link_name = "PxJointAngularLimitPair_new")
    joint_angular_limit_pair_new :: proc(lowerLimit: _c.float, upperLimit: _c.float, contactDist_deprecated: _c.float) -> PxJointAngularLimitPair ---

    /// construct an angular soft limit pair.
    ///
    /// The lower value must be less than the upper value.
    @(link_name = "PxJointAngularLimitPair_new_1")
    joint_angular_limit_pair_new_1 :: proc(lowerLimit: _c.float, upperLimit: _c.float, spring: ^PxSpring) -> PxJointAngularLimitPair ---

    /// Returns true if the limit is valid.
    ///
    /// true if the current settings are valid
    @(link_name = "PxJointAngularLimitPair_isValid")
    joint_angular_limit_pair_is_valid :: proc(self_: ^PxJointAngularLimitPair) -> _c.bool ---

    @(link_name = "PxJointAngularLimitPair_delete")
    joint_angular_limit_pair_delete :: proc(self_: ^PxJointAngularLimitPair) ---

    /// Construct a cone hard limit.
    @(link_name = "PxJointLimitCone_new")
    joint_limit_cone_new :: proc(yLimitAngle: _c.float, zLimitAngle: _c.float, contactDist_deprecated: _c.float) -> PxJointLimitCone ---

    /// Construct a cone soft limit.
    @(link_name = "PxJointLimitCone_new_1")
    joint_limit_cone_new_1 :: proc(yLimitAngle: _c.float, zLimitAngle: _c.float, spring: ^PxSpring) -> PxJointLimitCone ---

    /// Returns true if the limit is valid.
    ///
    /// true if the current settings are valid
    @(link_name = "PxJointLimitCone_isValid")
    joint_limit_cone_is_valid :: proc(self_: ^PxJointLimitCone) -> _c.bool ---

    @(link_name = "PxJointLimitCone_delete")
    joint_limit_cone_delete :: proc(self_: ^PxJointLimitCone) ---

    /// Construct a pyramid hard limit.
    @(link_name = "PxJointLimitPyramid_new")
    joint_limit_pyramid_new :: proc(yLimitAngleMin: _c.float, yLimitAngleMax: _c.float, zLimitAngleMin: _c.float, zLimitAngleMax: _c.float, contactDist_deprecated: _c.float) -> PxJointLimitPyramid ---

    /// Construct a pyramid soft limit.
    @(link_name = "PxJointLimitPyramid_new_1")
    joint_limit_pyramid_new_1 :: proc(yLimitAngleMin: _c.float, yLimitAngleMax: _c.float, zLimitAngleMin: _c.float, zLimitAngleMax: _c.float, spring: ^PxSpring) -> PxJointLimitPyramid ---

    /// Returns true if the limit is valid.
    ///
    /// true if the current settings are valid
    @(link_name = "PxJointLimitPyramid_isValid")
    joint_limit_pyramid_is_valid :: proc(self_: ^PxJointLimitPyramid) -> _c.bool ---

    @(link_name = "PxJointLimitPyramid_delete")
    joint_limit_pyramid_delete :: proc(self_: ^PxJointLimitPyramid) ---

    /// Create a prismatic joint.
    @(link_name = "phys_PxPrismaticJointCreate")
    prismatic_joint_create :: proc(physics: ^PxPhysics, actor0: ^PxRigidActor, #by_ptr localFrame0: PxTransform, actor1: ^PxRigidActor, #by_ptr localFrame1: PxTransform) -> ^PxPrismaticJoint ---

    /// returns the displacement of the joint along its axis.
    @(link_name = "PxPrismaticJoint_getPosition")
    prismatic_joint_get_position :: proc(self_: ^PxPrismaticJoint) -> _c.float ---

    /// returns the velocity of the joint along its axis
    @(link_name = "PxPrismaticJoint_getVelocity")
    prismatic_joint_get_velocity :: proc(self_: ^PxPrismaticJoint) -> _c.float ---

    /// sets the joint limit  parameters.
    ///
    /// The limit range is [-PX_MAX_F32, PX_MAX_F32], but note that the width of the limit (upper-lower) must also be
    /// a valid float.
    @(link_name = "PxPrismaticJoint_setLimit_mut")
    prismatic_joint_set_limit_mut :: proc(self_: ^PxPrismaticJoint, #by_ptr anon_param0: PxJointLinearLimitPair) ---

    /// gets the joint limit  parameters.
    @(link_name = "PxPrismaticJoint_getLimit")
    prismatic_joint_get_limit :: proc(self_: ^PxPrismaticJoint) -> PxJointLinearLimitPair ---

    /// Set the flags specific to the Prismatic Joint.
    ///
    /// Default
    /// PxPrismaticJointFlags(0)
    @(link_name = "PxPrismaticJoint_setPrismaticJointFlags_mut")
    prismatic_joint_set_prismatic_joint_flags_mut :: proc(self_: ^PxPrismaticJoint, flags: PxPrismaticJointFlags_Set) ---

    /// Set a single flag specific to a Prismatic Joint to true or false.
    @(link_name = "PxPrismaticJoint_setPrismaticJointFlag_mut")
    prismatic_joint_set_prismatic_joint_flag_mut :: proc(self_: ^PxPrismaticJoint, flag: PxPrismaticJointFlag, value: _c.bool) ---

    /// Get the flags specific to the Prismatic Joint.
    ///
    /// the joint flags
    @(link_name = "PxPrismaticJoint_getPrismaticJointFlags")
    prismatic_joint_get_prismatic_joint_flags :: proc(self_: ^PxPrismaticJoint) -> PxPrismaticJointFlags_Set ---

    /// Returns string name of PxPrismaticJoint, used for serialization
    @(link_name = "PxPrismaticJoint_getConcreteTypeName")
    prismatic_joint_get_concrete_type_name :: proc(self_: ^PxPrismaticJoint) -> ^_c.char ---

    /// Create a revolute joint.
    @(link_name = "phys_PxRevoluteJointCreate")
    revolute_joint_create :: proc(physics: ^PxPhysics, actor0: ^PxRigidActor, #by_ptr localFrame0: PxTransform, actor1: ^PxRigidActor, #by_ptr localFrame1: PxTransform) -> ^PxRevoluteJoint ---

    /// return the angle of the joint, in the range (-2*Pi, 2*Pi]
    @(link_name = "PxRevoluteJoint_getAngle")
    revolute_joint_get_angle :: proc(self_: ^PxRevoluteJoint) -> _c.float ---

    /// return the velocity of the joint
    @(link_name = "PxRevoluteJoint_getVelocity")
    revolute_joint_get_velocity :: proc(self_: ^PxRevoluteJoint) -> _c.float ---

    /// set the joint limit parameters.
    ///
    /// The limit is activated using the flag PxRevoluteJointFlag::eLIMIT_ENABLED
    ///
    /// The limit angle range is (-2*Pi, 2*Pi).
    @(link_name = "PxRevoluteJoint_setLimit_mut")
    revolute_joint_set_limit_mut :: proc(self_: ^PxRevoluteJoint, #by_ptr limits: PxJointAngularLimitPair) ---

    /// get the joint limit parameters.
    ///
    /// the joint limit parameters
    @(link_name = "PxRevoluteJoint_getLimit")
    revolute_joint_get_limit :: proc(self_: ^PxRevoluteJoint) -> PxJointAngularLimitPair ---

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
    revolute_joint_set_drive_velocity_mut :: proc(self_: ^PxRevoluteJoint, velocity: _c.float, autowake: _c.bool) ---

    /// gets the target velocity for the drive model.
    ///
    /// the drive target velocity
    @(link_name = "PxRevoluteJoint_getDriveVelocity")
    revolute_joint_get_drive_velocity :: proc(self_: ^PxRevoluteJoint) -> _c.float ---

    /// sets the maximum torque the drive can exert.
    ///
    /// The value set here may be used either as an impulse limit or a force limit, depending on the flag PxConstraintFlag::eDRIVE_LIMITS_ARE_FORCES
    ///
    /// Range:
    /// [0, PX_MAX_F32)
    /// Default:
    /// PX_MAX_F32
    @(link_name = "PxRevoluteJoint_setDriveForceLimit_mut")
    revolute_joint_set_drive_force_limit_mut :: proc(self_: ^PxRevoluteJoint, limit: _c.float) ---

    /// gets the maximum torque the drive can exert.
    ///
    /// the torque limit
    @(link_name = "PxRevoluteJoint_getDriveForceLimit")
    revolute_joint_get_drive_force_limit :: proc(self_: ^PxRevoluteJoint) -> _c.float ---

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
    revolute_joint_set_drive_gear_ratio_mut :: proc(self_: ^PxRevoluteJoint, ratio: _c.float) ---

    /// gets the gear ratio.
    ///
    /// the drive gear ratio
    @(link_name = "PxRevoluteJoint_getDriveGearRatio")
    revolute_joint_get_drive_gear_ratio :: proc(self_: ^PxRevoluteJoint) -> _c.float ---

    /// sets the flags specific to the Revolute Joint.
    ///
    /// Default
    /// PxRevoluteJointFlags(0)
    @(link_name = "PxRevoluteJoint_setRevoluteJointFlags_mut")
    revolute_joint_set_revolute_joint_flags_mut :: proc(self_: ^PxRevoluteJoint, flags: PxRevoluteJointFlags_Set) ---

    /// sets a single flag specific to a Revolute Joint.
    @(link_name = "PxRevoluteJoint_setRevoluteJointFlag_mut")
    revolute_joint_set_revolute_joint_flag_mut :: proc(self_: ^PxRevoluteJoint, flag: PxRevoluteJointFlag, value: _c.bool) ---

    /// gets the flags specific to the Revolute Joint.
    ///
    /// the joint flags
    @(link_name = "PxRevoluteJoint_getRevoluteJointFlags")
    revolute_joint_get_revolute_joint_flags :: proc(self_: ^PxRevoluteJoint) -> PxRevoluteJointFlags_Set ---

    /// Returns string name of PxRevoluteJoint, used for serialization
    @(link_name = "PxRevoluteJoint_getConcreteTypeName")
    revolute_joint_get_concrete_type_name :: proc(self_: ^PxRevoluteJoint) -> ^_c.char ---

    /// Create a spherical joint.
    @(link_name = "phys_PxSphericalJointCreate")
    spherical_joint_create :: proc(physics: ^PxPhysics, actor0: ^PxRigidActor, #by_ptr localFrame0: PxTransform, actor1: ^PxRigidActor, #by_ptr localFrame1: PxTransform) -> ^PxSphericalJoint ---

    /// Set the limit cone.
    ///
    /// If enabled, the limit cone will constrain the angular movement of the joint to lie
    /// within an elliptical cone.
    ///
    /// the limit cone
    @(link_name = "PxSphericalJoint_getLimitCone")
    spherical_joint_get_limit_cone :: proc(self_: ^PxSphericalJoint) -> PxJointLimitCone ---

    /// Get the limit cone.
    @(link_name = "PxSphericalJoint_setLimitCone_mut")
    spherical_joint_set_limit_cone_mut :: proc(self_: ^PxSphericalJoint, #by_ptr limit: PxJointLimitCone) ---

    /// get the swing angle of the joint from the Y axis
    @(link_name = "PxSphericalJoint_getSwingYAngle")
    spherical_joint_get_swing_y_angle :: proc(self_: ^PxSphericalJoint) -> _c.float ---

    /// get the swing angle of the joint from the Z axis
    @(link_name = "PxSphericalJoint_getSwingZAngle")
    spherical_joint_get_swing_z_angle :: proc(self_: ^PxSphericalJoint) -> _c.float ---

    /// Set the flags specific to the Spherical Joint.
    ///
    /// Default
    /// PxSphericalJointFlags(0)
    @(link_name = "PxSphericalJoint_setSphericalJointFlags_mut")
    spherical_joint_set_spherical_joint_flags_mut :: proc(self_: ^PxSphericalJoint, flags: PxSphericalJointFlags_Set) ---

    /// Set a single flag specific to a Spherical Joint to true or false.
    @(link_name = "PxSphericalJoint_setSphericalJointFlag_mut")
    spherical_joint_set_spherical_joint_flag_mut :: proc(self_: ^PxSphericalJoint, flag: PxSphericalJointFlag, value: _c.bool) ---

    /// Get the flags specific to the Spherical Joint.
    ///
    /// the joint flags
    @(link_name = "PxSphericalJoint_getSphericalJointFlags")
    spherical_joint_get_spherical_joint_flags :: proc(self_: ^PxSphericalJoint) -> PxSphericalJointFlags_Set ---

    /// Returns string name of PxSphericalJoint, used for serialization
    @(link_name = "PxSphericalJoint_getConcreteTypeName")
    spherical_joint_get_concrete_type_name :: proc(self_: ^PxSphericalJoint) -> ^_c.char ---

    /// Create a D6 joint.
    @(link_name = "phys_PxD6JointCreate")
    d6_joint_create :: proc(physics: ^PxPhysics, actor0: ^PxRigidActor, #by_ptr localFrame0: PxTransform, actor1: ^PxRigidActor, #by_ptr localFrame1: PxTransform) -> ^PxD6Joint ---

    /// default constructor for PxD6JointDrive.
    @(link_name = "PxD6JointDrive_new")
    d6_joint_drive_new :: proc() -> PxD6JointDrive ---

    /// constructor a PxD6JointDrive.
    @(link_name = "PxD6JointDrive_new_1")
    d6_joint_drive_new_1 :: proc(driveStiffness: _c.float, driveDamping: _c.float, driveForceLimit: _c.float, isAcceleration: _c.bool) -> PxD6JointDrive ---

    /// returns true if the drive is valid
    @(link_name = "PxD6JointDrive_isValid")
    d6_joint_drive_is_valid :: proc(self_: ^PxD6JointDrive) -> _c.bool ---

    /// Set the motion type around the specified axis.
    ///
    /// Each axis may independently specify that the degree of freedom is locked (blocking relative movement
    /// along or around this axis), limited by the corresponding limit, or free.
    ///
    /// Default:
    /// all degrees of freedom are locked
    @(link_name = "PxD6Joint_setMotion_mut")
    d6_joint_set_motion_mut :: proc(self_: ^PxD6Joint, axis: PxD6Axis, type: PxD6Motion) ---

    /// Get the motion type around the specified axis.
    ///
    /// the motion type around the specified axis
    @(link_name = "PxD6Joint_getMotion")
    d6_joint_get_motion :: proc(self_: ^PxD6Joint, axis: PxD6Axis) -> PxD6Motion ---

    /// get the twist angle of the joint, in the range (-2*Pi, 2*Pi]
    @(link_name = "PxD6Joint_getTwistAngle")
    d6_joint_get_twist_angle :: proc(self_: ^PxD6Joint) -> _c.float ---

    /// get the swing angle of the joint from the Y axis
    @(link_name = "PxD6Joint_getSwingYAngle")
    d6_joint_get_swing_y_angle :: proc(self_: ^PxD6Joint) -> _c.float ---

    /// get the swing angle of the joint from the Z axis
    @(link_name = "PxD6Joint_getSwingZAngle")
    d6_joint_get_swing_z_angle :: proc(self_: ^PxD6Joint) -> _c.float ---

    /// Set the distance limit for the joint.
    ///
    /// A single limit constraints all linear limited degrees of freedom, forming a linear, circular
    /// or spherical constraint on motion depending on the number of limited degrees. This is similar
    /// to a distance limit.
    @(link_name = "PxD6Joint_setDistanceLimit_mut")
    d6_joint_set_distance_limit_mut :: proc(self_: ^PxD6Joint, #by_ptr limit: PxJointLinearLimit) ---

    /// Get the distance limit for the joint.
    ///
    /// the distance limit structure
    @(link_name = "PxD6Joint_getDistanceLimit")
    d6_joint_get_distance_limit :: proc(self_: ^PxD6Joint) -> PxJointLinearLimit ---

    /// Set the linear limit for a given linear axis.
    ///
    /// This function extends the previous setDistanceLimit call with the following features:
    /// - there can be a different limit for each linear axis
    /// - each limit is defined by two values, i.e. it can now be asymmetric
    ///
    /// This can be used to create prismatic joints similar to PxPrismaticJoint, or point-in-quad joints,
    /// or point-in-box joints.
    @(link_name = "PxD6Joint_setLinearLimit_mut")
    d6_joint_set_linear_limit_mut :: proc(self_: ^PxD6Joint, axis: PxD6Axis, #by_ptr limit: PxJointLinearLimitPair) ---

    /// Get the linear limit for a given linear axis.
    ///
    /// the linear limit pair structure from desired axis
    @(link_name = "PxD6Joint_getLinearLimit")
    d6_joint_get_linear_limit :: proc(self_: ^PxD6Joint, axis: PxD6Axis) -> PxJointLinearLimitPair ---

    /// Set the twist limit for the joint.
    ///
    /// The twist limit controls the range of motion around the twist axis.
    ///
    /// The limit angle range is (-2*Pi, 2*Pi).
    @(link_name = "PxD6Joint_setTwistLimit_mut")
    d6_joint_set_twist_limit_mut :: proc(self_: ^PxD6Joint, #by_ptr limit: PxJointAngularLimitPair) ---

    /// Get the twist limit for the joint.
    ///
    /// the twist limit structure
    @(link_name = "PxD6Joint_getTwistLimit")
    d6_joint_get_twist_limit :: proc(self_: ^PxD6Joint) -> PxJointAngularLimitPair ---

    /// Set the swing cone limit for the joint.
    ///
    /// The cone limit is used if either or both swing axes are limited. The extents are
    /// symmetrical and measured in the frame of the parent. If only one swing degree of freedom
    /// is limited, the corresponding value from the cone limit defines the limit range.
    @(link_name = "PxD6Joint_setSwingLimit_mut")
    d6_joint_set_swing_limit_mut :: proc(self_: ^PxD6Joint, #by_ptr limit: PxJointLimitCone) ---

    /// Get the cone limit for the joint.
    ///
    /// the swing limit structure
    @(link_name = "PxD6Joint_getSwingLimit")
    d6_joint_get_swing_limit :: proc(self_: ^PxD6Joint) -> PxJointLimitCone ---

    /// Set a pyramidal swing limit for the joint.
    ///
    /// The pyramid limits will only be used in the following cases:
    /// - both swing Y and Z are limited. The limit shape is then a pyramid.
    /// - Y is limited and Z is locked, or vice versa. The limit shape is an asymmetric angular section, similar to
    /// what is supported for the twist axis.
    /// The remaining cases (Y limited and Z is free, or vice versa) are not supported.
    @(link_name = "PxD6Joint_setPyramidSwingLimit_mut")
    d6_joint_set_pyramid_swing_limit_mut :: proc(self_: ^PxD6Joint, #by_ptr limit: PxJointLimitPyramid) ---

    /// Get the pyramidal swing limit for the joint.
    ///
    /// the swing limit structure
    @(link_name = "PxD6Joint_getPyramidSwingLimit")
    d6_joint_get_pyramid_swing_limit :: proc(self_: ^PxD6Joint) -> PxJointLimitPyramid ---

    /// Set the drive parameters for the specified drive type.
    ///
    /// Default
    /// The default drive spring and damping values are zero, the force limit is zero, and no flags are set.
    @(link_name = "PxD6Joint_setDrive_mut")
    d6_joint_set_drive_mut :: proc(self_: ^PxD6Joint, index: PxD6Drive, #by_ptr drive: PxD6JointDrive) ---

    /// Get the drive parameters for the specified drive type.
    @(link_name = "PxD6Joint_getDrive")
    d6_joint_get_drive :: proc(self_: ^PxD6Joint, index: PxD6Drive) -> PxD6JointDrive ---

    /// Set the drive goal pose
    ///
    /// The goal is relative to the constraint frame of actor[0]
    ///
    /// Default
    /// the identity transform
    @(link_name = "PxD6Joint_setDrivePosition_mut")
    d6_joint_set_drive_position_mut :: proc(self_: ^PxD6Joint, #by_ptr pose: PxTransform, autowake: _c.bool) ---

    /// Get the drive goal pose.
    @(link_name = "PxD6Joint_getDrivePosition")
    d6_joint_get_drive_position :: proc(self_: ^PxD6Joint) -> PxTransform ---

    /// Set the target goal velocity for drive.
    ///
    /// The velocity is measured in the constraint frame of actor[0]
    @(link_name = "PxD6Joint_setDriveVelocity_mut")
    d6_joint_set_drive_velocity_mut :: proc(self_: ^PxD6Joint, #by_ptr linear: PxVec3, #by_ptr angular: PxVec3, autowake: _c.bool) ---

    /// Get the target goal velocity for joint drive.
    @(link_name = "PxD6Joint_getDriveVelocity")
    d6_joint_get_drive_velocity :: proc(self_: ^PxD6Joint, linear: ^PxVec3, angular: ^PxVec3) ---

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
    d6_joint_set_projection_linear_tolerance_mut :: proc(self_: ^PxD6Joint, tolerance: _c.float) ---

    /// Get the linear tolerance threshold for projection.
    ///
    /// the linear tolerance threshold
    @(link_name = "PxD6Joint_getProjectionLinearTolerance")
    d6_joint_get_projection_linear_tolerance :: proc(self_: ^PxD6Joint) -> _c.float ---

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
    d6_joint_set_projection_angular_tolerance_mut :: proc(self_: ^PxD6Joint, tolerance: _c.float) ---

    /// Get the angular tolerance threshold for projection.
    ///
    /// tolerance the angular tolerance threshold in radians
    @(link_name = "PxD6Joint_getProjectionAngularTolerance")
    d6_joint_get_projection_angular_tolerance :: proc(self_: ^PxD6Joint) -> _c.float ---

    /// Returns string name of PxD6Joint, used for serialization
    @(link_name = "PxD6Joint_getConcreteTypeName")
    d6_joint_get_concrete_type_name :: proc(self_: ^PxD6Joint) -> ^_c.char ---

    /// Create a gear Joint.
    @(link_name = "phys_PxGearJointCreate")
    gear_joint_create :: proc(physics: ^PxPhysics, actor0: ^PxRigidActor, #by_ptr localFrame0: PxTransform, actor1: ^PxRigidActor, #by_ptr localFrame1: PxTransform) -> ^PxGearJoint ---

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
    gear_joint_set_hinges_mut :: proc(self_: ^PxGearJoint, hinge0: ^PxBase, hinge1: ^PxBase) -> _c.bool ---

    /// Set the desired gear ratio.
    ///
    /// For two gears with n0 and n1 teeth respectively, the gear ratio is n0/n1.
    ///
    /// You may need to use a negative gear ratio if the joint frames of involved actors are not oriented in the same direction.
    ///
    /// Calling this function resets the internal positional error correction term.
    @(link_name = "PxGearJoint_setGearRatio_mut")
    gear_joint_set_gear_ratio_mut :: proc(self_: ^PxGearJoint, ratio: _c.float) ---

    /// Get the gear ratio.
    ///
    /// Current ratio
    @(link_name = "PxGearJoint_getGearRatio")
    gear_joint_get_gear_ratio :: proc(self_: ^PxGearJoint) -> _c.float ---

    @(link_name = "PxGearJoint_getConcreteTypeName")
    gear_joint_get_concrete_type_name :: proc(self_: ^PxGearJoint) -> ^_c.char ---

    /// Create a rack
    /// &
    /// pinion Joint.
    @(link_name = "phys_PxRackAndPinionJointCreate")
    rack_and_pinion_joint_create :: proc(physics: ^PxPhysics, actor0: ^PxRigidActor, #by_ptr localFrame0: PxTransform, actor1: ^PxRigidActor, #by_ptr localFrame1: PxTransform) -> ^PxRackAndPinionJoint ---

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
    rack_and_pinion_joint_set_joints_mut :: proc(self_: ^PxRackAndPinionJoint, hinge: ^PxBase, prismatic: ^PxBase) -> _c.bool ---

    /// Set the desired ratio directly.
    ///
    /// You may need to use a negative gear ratio if the joint frames of involved actors are not oriented in the same direction.
    ///
    /// Calling this function resets the internal positional error correction term.
    @(link_name = "PxRackAndPinionJoint_setRatio_mut")
    rack_and_pinion_joint_set_ratio_mut :: proc(self_: ^PxRackAndPinionJoint, ratio: _c.float) ---

    /// Get the ratio.
    ///
    /// Current ratio
    @(link_name = "PxRackAndPinionJoint_getRatio")
    rack_and_pinion_joint_get_ratio :: proc(self_: ^PxRackAndPinionJoint) -> _c.float ---

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
    rack_and_pinion_joint_set_data_mut :: proc(self_: ^PxRackAndPinionJoint, nbRackTeeth: _c.uint32_t, nbPinionTeeth: _c.uint32_t, rackLength: _c.float) -> _c.bool ---

    @(link_name = "PxRackAndPinionJoint_getConcreteTypeName")
    rack_and_pinion_joint_get_concrete_type_name :: proc(self_: ^PxRackAndPinionJoint) -> ^_c.char ---

    @(link_name = "PxGroupsMask_new_alloc")
    groups_mask_new_alloc :: proc() -> ^PxGroupsMask ---

    @(link_name = "PxGroupsMask_delete")
    groups_mask_delete :: proc(self_: ^PxGroupsMask) ---

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
    default_simulation_filter_shader :: proc(attributes0: _c.uint32_t, filterData0: PxFilterData, attributes1: _c.uint32_t, filterData1: PxFilterData, pairFlags: ^PxPairFlags_Set, constantBlock: rawptr, constantBlockSize: _c.uint32_t) -> PxFilterFlags_Set ---

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
    get_group :: proc(actor: ^PxActor) -> _c.uint16_t ---

    /// Sets which collision group this actor is part of
    ///
    /// Collision group is an integer between 0 and 31.
    @(link_name = "phys_PxSetGroup")
    set_group :: proc(actor: ^PxActor, collisionGroup: _c.uint16_t) ---

    /// Retrieves filtering operation. See comments for PxGroupsMask
    @(link_name = "phys_PxGetFilterOps")
    get_filter_ops :: proc(op0: ^PxFilterOp, op1: ^PxFilterOp, op2: ^PxFilterOp) ---

    /// Setups filtering operations. See comments for PxGroupsMask
    @(link_name = "phys_PxSetFilterOps")
    set_filter_ops :: proc(#by_ptr op0: PxFilterOp, #by_ptr op1: PxFilterOp, #by_ptr op2: PxFilterOp) ---

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
    get_filter_constants :: proc(c0: ^PxGroupsMask, c1: ^PxGroupsMask) ---

    /// Setups filtering's K0 and K1 value. See comments for PxGroupsMask
    @(link_name = "phys_PxSetFilterConstants")
    set_filter_constants :: proc(#by_ptr c0: PxGroupsMask, #by_ptr c1: PxGroupsMask) ---

    /// Gets 64-bit mask used for collision filtering. See comments for PxGroupsMask
    ///
    /// The group mask for the actor.
    @(link_name = "phys_PxGetGroupsMask")
    get_groups_mask :: proc(actor: ^PxActor) -> PxGroupsMask ---

    /// Sets 64-bit mask used for collision filtering. See comments for PxGroupsMask
    @(link_name = "phys_PxSetGroupsMask")
    set_groups_mask :: proc(actor: ^PxActor, #by_ptr mask: PxGroupsMask) ---

    @(link_name = "PxDefaultErrorCallback_new_alloc")
    default_error_callback_new_alloc :: proc() -> ^PxDefaultErrorCallback ---

    @(link_name = "PxDefaultErrorCallback_delete")
    default_error_callback_delete :: proc(self_: ^PxDefaultErrorCallback) ---

    @(link_name = "PxDefaultErrorCallback_reportError_mut")
    default_error_callback_report_error_mut :: proc(self_: ^PxDefaultErrorCallback, code: PxErrorCode, message: ^_c.char, file: ^_c.char, line: _c.int32_t) ---

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
    rigid_actor_ext_create_exclusive_shape :: proc(actor: ^PxRigidActor, geometry: ^PxGeometry, materials: ^^PxMaterial, materialCount: _c.uint16_t, shapeFlags: PxShapeFlags_Set) -> ^PxShape ---

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
    rigid_actor_ext_create_exclusive_shape_1 :: proc(actor: ^PxRigidActor, geometry: ^PxGeometry, #by_ptr material: PxMaterial, shapeFlags: PxShapeFlags_Set) -> ^PxShape ---

    /// Gets a list of bounds based on shapes in rigid actor. This list can be used to cook/create
    /// bounding volume hierarchy though PxCooking API.
    @(link_name = "PxRigidActorExt_getRigidActorShapeLocalBoundsList")
    rigid_actor_ext_get_rigid_actor_shape_local_bounds_list :: proc(actor: ^PxRigidActor, numBounds: ^_c.uint32_t) -> ^PxBounds3 ---

    /// Convenience function to create a PxBVH object from a PxRigidActor.
    ///
    /// The computed PxBVH can then be used in PxScene::addActor() or PxAggregate::addActor().
    /// After adding the actor
    /// &
    /// BVH to the scene/aggregate, release the PxBVH object by calling PxBVH::release().
    ///
    /// The PxBVH for this actor.
    @(link_name = "PxRigidActorExt_createBVHFromActor")
    rigid_actor_ext_create_b_v_h_from_actor :: proc(physics: ^PxPhysics, actor: ^PxRigidActor) -> ^PxBVH ---

    /// Default constructor.
    @(link_name = "PxMassProperties_new")
    mass_properties_new :: proc() -> PxMassProperties ---

    /// Construct from individual elements.
    @(link_name = "PxMassProperties_new_1")
    mass_properties_new_1 :: proc(m: _c.float, #by_ptr inertiaT: PxMat33, #by_ptr com: PxVec3) -> PxMassProperties ---

    /// Compute mass properties based on a provided geometry structure.
    ///
    /// This constructor assumes the geometry has a density of 1. Mass and inertia tensor scale linearly with density.
    @(link_name = "PxMassProperties_new_2")
    mass_properties_new_2 :: proc(geometry: ^PxGeometry) -> PxMassProperties ---

    /// Translate the center of mass by a given vector and adjust the inertia tensor accordingly.
    @(link_name = "PxMassProperties_translate_mut")
    mass_properties_translate_mut :: proc(self_: ^PxMassProperties, #by_ptr t: PxVec3) ---

    /// Get the entries of the diagonalized inertia tensor and the corresponding reference rotation.
    ///
    /// The entries of the diagonalized inertia tensor.
    @(link_name = "PxMassProperties_getMassSpaceInertia")
    mass_properties_get_mass_space_inertia :: proc(#by_ptr inertia: PxMat33, massFrame: ^PxQuat) -> PxVec3 ---

    /// Translate an inertia tensor using the parallel axis theorem
    ///
    /// The translated inertia tensor.
    @(link_name = "PxMassProperties_translateInertia")
    mass_properties_translate_inertia :: proc(#by_ptr inertia: PxMat33, mass: _c.float, #by_ptr t: PxVec3) -> PxMat33 ---

    /// Rotate an inertia tensor around the center of mass
    ///
    /// The rotated inertia tensor.
    @(link_name = "PxMassProperties_rotateInertia")
    mass_properties_rotate_inertia :: proc(#by_ptr inertia: PxMat33, #by_ptr q: PxQuat) -> PxMat33 ---

    /// Non-uniform scaling of the inertia tensor
    ///
    /// The scaled inertia tensor.
    @(link_name = "PxMassProperties_scaleInertia")
    mass_properties_scale_inertia :: proc(#by_ptr inertia: PxMat33, #by_ptr scaleRotation: PxQuat, #by_ptr scale: PxVec3) -> PxMat33 ---

    /// Sum up individual mass properties.
    ///
    /// The summed up mass properties.
    @(link_name = "PxMassProperties_sum")
    mass_properties_sum :: proc(props: ^PxMassProperties, transforms: ^PxTransform, count: _c.uint32_t) -> PxMassProperties ---

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
    rigid_body_ext_update_mass_and_inertia :: proc(body: ^PxRigidBody, shapeDensities: ^_c.float, shapeDensityCount: _c.uint32_t, massLocalPose: ^PxVec3, includeNonSimShapes: _c.bool) -> _c.bool ---

    /// Computation of mass properties for a rigid body actor
    ///
    /// See previous method for details.
    ///
    /// Boolean. True on success else false.
    @(link_name = "PxRigidBodyExt_updateMassAndInertia_1")
    rigid_body_ext_update_mass_and_inertia_1 :: proc(body: ^PxRigidBody, density: _c.float, massLocalPose: ^PxVec3, includeNonSimShapes: _c.bool) -> _c.bool ---

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
    rigid_body_ext_set_mass_and_update_inertia :: proc(body: ^PxRigidBody, shapeMasses: ^_c.float, shapeMassCount: _c.uint32_t, massLocalPose: ^PxVec3, includeNonSimShapes: _c.bool) -> _c.bool ---

    /// Computation of mass properties for a rigid body actor
    ///
    /// This method sets the mass, inertia and center of mass of a rigid body. The mass is set to the user-supplied
    /// value, and the inertia and center of mass are computed according to the rigid body's shapes and the input mass.
    ///
    /// If no collision shapes are found, the inertia tensor is set to (1,1,1)
    ///
    /// Boolean. True on success else false.
    @(link_name = "PxRigidBodyExt_setMassAndUpdateInertia_1")
    rigid_body_ext_set_mass_and_update_inertia_1 :: proc(body: ^PxRigidBody, mass: _c.float, massLocalPose: ^PxVec3, includeNonSimShapes: _c.bool) -> _c.bool ---

    /// Compute the mass, inertia tensor and center of mass from a list of shapes.
    ///
    /// The mass properties from the combined shapes.
    @(link_name = "PxRigidBodyExt_computeMassPropertiesFromShapes")
    rigid_body_ext_compute_mass_properties_from_shapes :: proc(shapes: ^^PxShape, shapeCount: _c.uint32_t) -> PxMassProperties ---

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
    rigid_body_ext_add_force_at_pos :: proc(body: ^PxRigidBody, #by_ptr force: PxVec3, #by_ptr pos: PxVec3, mode: PxForceMode, wakeup: _c.bool) ---

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
    rigid_body_ext_add_force_at_local_pos :: proc(body: ^PxRigidBody, #by_ptr force: PxVec3, #by_ptr pos: PxVec3, mode: PxForceMode, wakeup: _c.bool) ---

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
    rigid_body_ext_add_local_force_at_pos :: proc(body: ^PxRigidBody, #by_ptr force: PxVec3, #by_ptr pos: PxVec3, mode: PxForceMode, wakeup: _c.bool) ---

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
    rigid_body_ext_add_local_force_at_local_pos :: proc(body: ^PxRigidBody, #by_ptr force: PxVec3, #by_ptr pos: PxVec3, mode: PxForceMode, wakeup: _c.bool) ---

    /// Computes the velocity of a point given in world coordinates if it were attached to the
    /// specified body and moving with it.
    ///
    /// The velocity of point in the global frame.
    @(link_name = "PxRigidBodyExt_getVelocityAtPos")
    rigid_body_ext_get_velocity_at_pos :: proc(body: ^PxRigidBody, #by_ptr pos: PxVec3) -> PxVec3 ---

    /// Computes the velocity of a point given in local coordinates if it were attached to the
    /// specified body and moving with it.
    ///
    /// The velocity of point in the local frame.
    @(link_name = "PxRigidBodyExt_getLocalVelocityAtLocalPos")
    rigid_body_ext_get_local_velocity_at_local_pos :: proc(body: ^PxRigidBody, #by_ptr pos: PxVec3) -> PxVec3 ---

    /// Computes the velocity of a point (offset from the origin of the body) given in world coordinates if it were attached to the
    /// specified body and moving with it.
    ///
    /// The velocity of point (offset from the origin of the body) in the global frame.
    @(link_name = "PxRigidBodyExt_getVelocityAtOffset")
    rigid_body_ext_get_velocity_at_offset :: proc(body: ^PxRigidBody, #by_ptr pos: PxVec3) -> PxVec3 ---

    /// Compute the change to linear and angular velocity that would occur if an impulsive force and torque were to be applied to a specified rigid body.
    ///
    /// The rigid body is left unaffected unless a subsequent independent call is executed that actually applies the computed changes to velocity and angular velocity.
    ///
    /// if this call is used to determine the velocity delta for an articulation link, only the mass properties of the link are taken into account.
    @(link_name = "PxRigidBodyExt_computeVelocityDeltaFromImpulse")
    rigid_body_ext_compute_velocity_delta_from_impulse :: proc(body: ^PxRigidBody, #by_ptr impulsiveForce: PxVec3, #by_ptr impulsiveTorque: PxVec3, deltaLinearVelocity: ^PxVec3, deltaAngularVelocity: ^PxVec3) ---

    /// Computes the linear and angular velocity change vectors for a given impulse at a world space position taking a mass and inertia scale into account
    ///
    /// This function is useful for extracting the respective linear and angular velocity changes from a contact or joint when the mass/inertia ratios have been adjusted.
    ///
    /// if this call is used to determine the velocity delta for an articulation link, only the mass properties of the link are taken into account.
    @(link_name = "PxRigidBodyExt_computeVelocityDeltaFromImpulse_1")
    rigid_body_ext_compute_velocity_delta_from_impulse_1 :: proc(body: ^PxRigidBody, #by_ptr globalPose: PxTransform, #by_ptr point: PxVec3, #by_ptr impulse: PxVec3, invMassScale: _c.float, invInertiaScale: _c.float, deltaLinearVelocity: ^PxVec3, deltaAngularVelocity: ^PxVec3) ---

    /// Computes the linear and angular impulse vectors for a given impulse at a world space position taking a mass and inertia scale into account
    ///
    /// This function is useful for extracting the respective linear and angular impulses from a contact or joint when the mass/inertia ratios have been adjusted.
    @(link_name = "PxRigidBodyExt_computeLinearAngularImpulse")
    rigid_body_ext_compute_linear_angular_impulse :: proc(body: ^PxRigidBody, #by_ptr globalPose: PxTransform, #by_ptr point: PxVec3, #by_ptr impulse: PxVec3, invMassScale: _c.float, invInertiaScale: _c.float, linearImpulse: ^PxVec3, angularImpulse: ^PxVec3) ---

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
    rigid_body_ext_linear_sweep_single :: proc(body: ^PxRigidBody, scene: ^PxScene, #by_ptr unitDir: PxVec3, distance: _c.float, outputFlags: PxHitFlags_Set, closestHit: ^PxSweepHit, shapeIndex: ^_c.uint32_t, #by_ptr filterData: PxQueryFilterData, filterCall: ^PxQueryFilterCallback, cache: ^PxQueryCache, inflation: _c.float) -> _c.bool ---

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
    rigid_body_ext_linear_sweep_multiple :: proc(body: ^PxRigidBody, scene: ^PxScene, #by_ptr unitDir: PxVec3, distance: _c.float, outputFlags: PxHitFlags_Set, touchHitBuffer: ^PxSweepHit, touchHitShapeIndices: ^_c.uint32_t, touchHitBufferSize: _c.uint32_t, block: ^PxSweepHit, blockingShapeIndex: ^_c.int32_t, overflow: ^_c.bool, #by_ptr filterData: PxQueryFilterData, filterCall: ^PxQueryFilterCallback, cache: ^PxQueryCache, inflation: _c.float) -> _c.uint32_t ---

    /// Retrieves the world space pose of the shape.
    ///
    /// Global pose of shape.
    @(link_name = "PxShapeExt_getGlobalPose")
    shape_ext_get_global_pose :: proc(#by_ptr shape: PxShape, actor: ^PxRigidActor) -> PxTransform ---

    /// Raycast test against the shape.
    ///
    /// Number of hits between the ray and the shape
    @(link_name = "PxShapeExt_raycast")
    shape_ext_raycast :: proc(#by_ptr shape: PxShape, actor: ^PxRigidActor, #by_ptr rayOrigin: PxVec3, #by_ptr rayDir: PxVec3, maxDist: _c.float, hitFlags: PxHitFlags_Set, maxHits: _c.uint32_t, rayHits: ^PxRaycastHit) -> _c.uint32_t ---

    /// Test overlap between the shape and a geometry object
    ///
    /// True if the shape overlaps the geometry object
    @(link_name = "PxShapeExt_overlap")
    shape_ext_overlap :: proc(#by_ptr shape: PxShape, actor: ^PxRigidActor, otherGeom: ^PxGeometry, #by_ptr otherGeomPose: PxTransform) -> _c.bool ---

    /// Sweep a geometry object against the shape.
    ///
    /// Currently only box, sphere, capsule and convex mesh shapes are supported, i.e. the swept geometry object must be one of those types.
    ///
    /// True if the swept geometry object hits the shape
    @(link_name = "PxShapeExt_sweep")
    shape_ext_sweep :: proc(#by_ptr shape: PxShape, actor: ^PxRigidActor, #by_ptr unitDir: PxVec3, distance: _c.float, otherGeom: ^PxGeometry, #by_ptr otherGeomPose: PxTransform, sweepHit: ^PxSweepHit, hitFlags: PxHitFlags_Set) -> _c.bool ---

    /// Retrieves the axis aligned bounding box enclosing the shape.
    ///
    /// The shape's bounding box.
    @(link_name = "PxShapeExt_getWorldBounds")
    shape_ext_get_world_bounds :: proc(#by_ptr shape: PxShape, actor: ^PxRigidActor, inflation: _c.float) -> PxBounds3 ---

    @(link_name = "PxMeshOverlapUtil_new_alloc")
    mesh_overlap_util_new_alloc :: proc() -> ^PxMeshOverlapUtil ---

    @(link_name = "PxMeshOverlapUtil_delete")
    mesh_overlap_util_delete :: proc(self_: ^PxMeshOverlapUtil) ---

    /// Find the mesh triangles which touch the specified geometry object.
    ///
    /// Number of overlaps found. Triangle indices can then be accessed through the [`getResults`]() function.
    @(link_name = "PxMeshOverlapUtil_findOverlap_mut")
    mesh_overlap_util_find_overlap_mut :: proc(self_: ^PxMeshOverlapUtil, geom: ^PxGeometry, #by_ptr geomPose: PxTransform, #by_ptr meshGeom: PxTriangleMeshGeometry, #by_ptr meshPose: PxTransform) -> _c.uint32_t ---

    /// Find the height field triangles which touch the specified geometry object.
    ///
    /// Number of overlaps found. Triangle indices can then be accessed through the [`getResults`]() function.
    @(link_name = "PxMeshOverlapUtil_findOverlap_mut_1")
    mesh_overlap_util_find_overlap_mut_1 :: proc(self_: ^PxMeshOverlapUtil, geom: ^PxGeometry, #by_ptr geomPose: PxTransform, #by_ptr hfGeom: PxHeightFieldGeometry, #by_ptr hfPose: PxTransform) -> _c.uint32_t ---

    /// Retrieves array of triangle indices after a findOverlap call.
    ///
    /// Indices of touched triangles
    @(link_name = "PxMeshOverlapUtil_getResults")
    mesh_overlap_util_get_results :: proc(self_: ^PxMeshOverlapUtil) -> ^_c.uint32_t ---

    /// Retrieves number of triangle indices after a findOverlap call.
    ///
    /// Number of touched triangles
    @(link_name = "PxMeshOverlapUtil_getNbResults")
    mesh_overlap_util_get_nb_results :: proc(self_: ^PxMeshOverlapUtil) -> _c.uint32_t ---

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
    compute_triangle_mesh_penetration :: proc(direction: ^PxVec3, depth: ^_c.float, geom: ^PxGeometry, #by_ptr geomPose: PxTransform, #by_ptr meshGeom: PxTriangleMeshGeometry, #by_ptr meshPose: PxTransform, maxIter: _c.uint32_t, usedIter: ^_c.uint32_t) -> _c.bool ---

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
    compute_height_field_penetration :: proc(direction: ^PxVec3, depth: ^_c.float, geom: ^PxGeometry, #by_ptr geomPose: PxTransform, #by_ptr heightFieldGeom: PxHeightFieldGeometry, #by_ptr heightFieldPose: PxTransform, maxIter: _c.uint32_t, usedIter: ^_c.uint32_t) -> _c.bool ---

    @(link_name = "PxXmlMiscParameter_new")
    xml_misc_parameter_new :: proc() -> PxXmlMiscParameter ---

    @(link_name = "PxXmlMiscParameter_new_1")
    xml_misc_parameter_new_1 :: proc(inUpVector: ^PxVec3, inScale: PxTolerancesScale) -> PxXmlMiscParameter ---

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
    serialization_is_serializable :: proc(collection: ^PxCollection, sr: ^PxSerializationRegistry, externalReferences: ^PxCollection) -> _c.bool ---

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
    serialization_complete :: proc(collection: ^PxCollection, sr: ^PxSerializationRegistry, exceptFor: ^PxCollection, followJoints: _c.bool) ---

    /// Creates PxSerialObjectId values for unnamed objects in a collection.
    ///
    /// Creates PxSerialObjectId names for unnamed objects in a collection starting at a base value and incrementing,
    /// skipping values that are already assigned to objects in the collection.
    @(link_name = "PxSerialization_createSerialObjectIds")
    serialization_create_serial_object_ids :: proc(collection: ^PxCollection, base: _c.uint64_t) ---

    /// Creates a PxCollection from XML data.
    ///
    /// a pointer to a PxCollection if successful or NULL if it failed.
    @(link_name = "PxSerialization_createCollectionFromXml")
    serialization_create_collection_from_xml :: proc(inputData: ^PxInputData, cooking: ^PxCooking, sr: ^PxSerializationRegistry, externalRefs: ^PxCollection, stringTable: ^PxStringTable, outArgs: ^PxXmlMiscParameter) -> ^PxCollection ---

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
    serialization_create_collection_from_binary :: proc(memBlock: rawptr, sr: ^PxSerializationRegistry, externalRefs: ^PxCollection) -> ^PxCollection ---

    /// Serializes a physics collection to an XML output stream.
    ///
    /// The collection to be serialized needs to be complete
    ///
    /// Serialization of objects in a scene that is simultaneously being simulated is not supported and leads to undefined behavior.
    ///
    /// true if the collection is successfully serialized.
    @(link_name = "PxSerialization_serializeCollectionToXml")
    serialization_serialize_collection_to_xml :: proc(outputStream: ^PxOutputStream, collection: ^PxCollection, sr: ^PxSerializationRegistry, cooking: ^PxCooking, externalRefs: ^PxCollection, inArgs: ^PxXmlMiscParameter) -> _c.bool ---

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
    serialization_serialize_collection_to_binary :: proc(outputStream: ^PxOutputStream, collection: ^PxCollection, sr: ^PxSerializationRegistry, externalRefs: ^PxCollection, exportNames: _c.bool) -> _c.bool ---

    /// Creates an application managed registry for serialization.
    ///
    /// PxSerializationRegistry instance.
    @(link_name = "PxSerialization_createSerializationRegistry")
    serialization_create_serialization_registry :: proc(physics: ^PxPhysics) -> ^PxSerializationRegistry ---

    /// Deletes the dispatcher.
    ///
    /// Do not keep a reference to the deleted instance.
    @(link_name = "PxDefaultCpuDispatcher_release_mut")
    default_cpu_dispatcher_release_mut :: proc(self_: ^PxDefaultCpuDispatcher) ---

    /// Enables profiling at task level.
    ///
    /// By default enabled only in profiling builds.
    @(link_name = "PxDefaultCpuDispatcher_setRunProfiled_mut")
    default_cpu_dispatcher_set_run_profiled_mut :: proc(self_: ^PxDefaultCpuDispatcher, runProfiled: _c.bool) ---

    /// Checks if profiling is enabled at task level.
    ///
    /// True if tasks should be profiled.
    @(link_name = "PxDefaultCpuDispatcher_getRunProfiled")
    default_cpu_dispatcher_get_run_profiled :: proc(self_: ^PxDefaultCpuDispatcher) -> _c.bool ---

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
    default_cpu_dispatcher_create :: proc(numThreads: _c.uint32_t, affinityMasks: ^_c.uint32_t, mode: PxDefaultCpuDispatcherWaitForWorkMode, yieldProcessorCount: _c.uint32_t) -> ^PxDefaultCpuDispatcher ---

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
    build_smooth_normals :: proc(nbTris: _c.uint32_t, nbVerts: _c.uint32_t, verts: ^PxVec3, dFaces: ^_c.uint32_t, wFaces: ^_c.uint16_t, normals: ^PxVec3, flip: _c.bool) -> _c.bool ---

    /// simple method to create a PxRigidDynamic actor with a single PxShape.
    ///
    /// a new dynamic actor with the PxRigidBodyFlag, or NULL if it could
    /// not be constructed
    @(link_name = "phys_PxCreateDynamic")
    create_dynamic :: proc(sdk: ^PxPhysics, #by_ptr transform: PxTransform, geometry: ^PxGeometry, material: ^PxMaterial, density: _c.float, #by_ptr shapeOffset: PxTransform) -> ^PxRigidDynamic ---

    /// simple method to create a PxRigidDynamic actor with a single PxShape.
    ///
    /// a new dynamic actor with the PxRigidBodyFlag, or NULL if it could
    /// not be constructed
    @(link_name = "phys_PxCreateDynamic_1")
    create_dynamic_1 :: proc(sdk: ^PxPhysics, #by_ptr transform: PxTransform, shape: ^PxShape, density: _c.float) -> ^PxRigidDynamic ---

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
    create_kinematic :: proc(sdk: ^PxPhysics, #by_ptr transform: PxTransform, geometry: ^PxGeometry, material: ^PxMaterial, density: _c.float, #by_ptr shapeOffset: PxTransform) -> ^PxRigidDynamic ---

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
    create_kinematic_1 :: proc(sdk: ^PxPhysics, #by_ptr transform: PxTransform, shape: ^PxShape, density: _c.float) -> ^PxRigidDynamic ---

    /// simple method to create a PxRigidStatic actor with a single PxShape.
    ///
    /// a new static actor, or NULL if it could not be constructed
    @(link_name = "phys_PxCreateStatic")
    create_static :: proc(sdk: ^PxPhysics, #by_ptr transform: PxTransform, geometry: ^PxGeometry, material: ^PxMaterial, #by_ptr shapeOffset: PxTransform) -> ^PxRigidStatic ---

    /// simple method to create a PxRigidStatic actor with a single PxShape.
    ///
    /// a new static actor, or NULL if it could not be constructed
    @(link_name = "phys_PxCreateStatic_1")
    create_static_1 :: proc(sdk: ^PxPhysics, #by_ptr transform: PxTransform, shape: ^PxShape) -> ^PxRigidStatic ---

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
    clone_shape :: proc(physicsSDK: ^PxPhysics, #by_ptr shape: PxShape, isExclusive: _c.bool) -> ^PxShape ---

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
    clone_static :: proc(physicsSDK: ^PxPhysics, #by_ptr transform: PxTransform, actor: ^PxRigidActor) -> ^PxRigidStatic ---

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
    clone_dynamic :: proc(physicsSDK: ^PxPhysics, #by_ptr transform: PxTransform, #by_ptr body: PxRigidDynamic) -> ^PxRigidDynamic ---

    /// create a plane actor. The plane equation is n.x + d = 0
    ///
    /// a new static actor, or NULL if it could not be constructed
    @(link_name = "phys_PxCreatePlane")
    create_plane :: proc(sdk: ^PxPhysics, #by_ptr plane: PxPlane, material: ^PxMaterial) -> ^PxRigidStatic ---

    /// scale a rigid actor by a uniform scale
    ///
    /// The geometry and relative positions of the actor are multiplied by the given scale value. If the actor is a rigid body or an
    /// articulation link and the scaleMassProps value is true, the mass properties are scaled assuming the density is constant: the
    /// center of mass is linearly scaled, the mass is multiplied by the cube of the scale, and the inertia tensor by the fifth power of the scale.
    @(link_name = "phys_PxScaleRigidActor")
    scale_rigid_actor :: proc(actor: ^PxRigidActor, scale: _c.float, scaleMassProps: _c.bool) ---

    @(link_name = "PxStringTableExt_createStringTable")
    string_table_ext_create_string_table :: proc(inAllocator: ^PxAllocatorCallback) -> ^PxStringTable ---

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
    broad_phase_ext_create_regions_from_world_bounds :: proc(regions: ^PxBounds3, #by_ptr globalBounds: PxBounds3, nbSubdiv: _c.uint32_t, upAxis: _c.uint32_t) -> _c.uint32_t ---

    /// Raycast returning any blocking hit, not necessarily the closest.
    ///
    /// Returns whether any rigid actor is hit along the ray.
    ///
    /// Shooting a ray from within an object leads to different results depending on the shape type. Please check the details in article SceneQuery. User can ignore such objects by using one of the provided filter mechanisms.
    ///
    /// True if a blocking hit was found.
    @(link_name = "PxSceneQueryExt_raycastAny")
    scene_query_ext_raycast_any :: proc(#by_ptr scene: PxScene, #by_ptr origin: PxVec3, #by_ptr unitDir: PxVec3, distance: _c.float, hit: ^PxQueryHit, #by_ptr filterData: PxQueryFilterData, filterCall: ^PxQueryFilterCallback, cache: ^PxQueryCache) -> _c.bool ---

    /// Raycast returning a single result.
    ///
    /// Returns the first rigid actor that is hit along the ray. Data for a blocking hit will be returned as specified by the outputFlags field. Touching hits will be ignored.
    ///
    /// Shooting a ray from within an object leads to different results depending on the shape type. Please check the details in article SceneQuery. User can ignore such objects by using one of the provided filter mechanisms.
    ///
    /// True if a blocking hit was found.
    @(link_name = "PxSceneQueryExt_raycastSingle")
    scene_query_ext_raycast_single :: proc(#by_ptr scene: PxScene, #by_ptr origin: PxVec3, #by_ptr unitDir: PxVec3, distance: _c.float, outputFlags: PxHitFlags_Set, hit: ^PxRaycastHit, #by_ptr filterData: PxQueryFilterData, filterCall: ^PxQueryFilterCallback, cache: ^PxQueryCache) -> _c.bool ---

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
    scene_query_ext_raycast_multiple :: proc(#by_ptr scene: PxScene, #by_ptr origin: PxVec3, #by_ptr unitDir: PxVec3, distance: _c.float, outputFlags: PxHitFlags_Set, hitBuffer: ^PxRaycastHit, hitBufferSize: _c.uint32_t, blockingHit: ^_c.bool, #by_ptr filterData: PxQueryFilterData, filterCall: ^PxQueryFilterCallback, cache: ^PxQueryCache) -> _c.int32_t ---

    /// Sweep returning any blocking hit, not necessarily the closest.
    ///
    /// Returns whether any rigid actor is hit along the sweep path.
    ///
    /// If a shape from the scene is already overlapping with the query shape in its starting position, behavior is controlled by the PxSceneQueryFlag::eINITIAL_OVERLAP flag.
    ///
    /// True if a blocking hit was found.
    @(link_name = "PxSceneQueryExt_sweepAny")
    scene_query_ext_sweep_any :: proc(#by_ptr scene: PxScene, geometry: ^PxGeometry, #by_ptr pose: PxTransform, #by_ptr unitDir: PxVec3, distance: _c.float, queryFlags: PxHitFlags_Set, hit: ^PxQueryHit, #by_ptr filterData: PxQueryFilterData, filterCall: ^PxQueryFilterCallback, cache: ^PxQueryCache, inflation: _c.float) -> _c.bool ---

    /// Sweep returning a single result.
    ///
    /// Returns the first rigid actor that is hit along the ray. Data for a blocking hit will be returned as specified by the outputFlags field. Touching hits will be ignored.
    ///
    /// If a shape from the scene is already overlapping with the query shape in its starting position, behavior is controlled by the PxSceneQueryFlag::eINITIAL_OVERLAP flag.
    ///
    /// True if a blocking hit was found.
    @(link_name = "PxSceneQueryExt_sweepSingle")
    scene_query_ext_sweep_single :: proc(#by_ptr scene: PxScene, geometry: ^PxGeometry, #by_ptr pose: PxTransform, #by_ptr unitDir: PxVec3, distance: _c.float, outputFlags: PxHitFlags_Set, hit: ^PxSweepHit, #by_ptr filterData: PxQueryFilterData, filterCall: ^PxQueryFilterCallback, cache: ^PxQueryCache, inflation: _c.float) -> _c.bool ---

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
    scene_query_ext_sweep_multiple :: proc(#by_ptr scene: PxScene, geometry: ^PxGeometry, #by_ptr pose: PxTransform, #by_ptr unitDir: PxVec3, distance: _c.float, outputFlags: PxHitFlags_Set, hitBuffer: ^PxSweepHit, hitBufferSize: _c.uint32_t, blockingHit: ^_c.bool, #by_ptr filterData: PxQueryFilterData, filterCall: ^PxQueryFilterCallback, cache: ^PxQueryCache, inflation: _c.float) -> _c.int32_t ---

    /// Test overlap between a geometry and objects in the scene.
    ///
    /// Filtering: Overlap tests do not distinguish between touching and blocking hit types. Both get written to the hit buffer.
    ///
    /// PxHitFlag::eMESH_MULTIPLE and PxHitFlag::eMESH_BOTH_SIDES have no effect in this case
    ///
    /// Number of hits in the buffer, or -1 if the buffer overflowed.
    @(link_name = "PxSceneQueryExt_overlapMultiple")
    scene_query_ext_overlap_multiple :: proc(#by_ptr scene: PxScene, geometry: ^PxGeometry, #by_ptr pose: PxTransform, hitBuffer: ^PxOverlapHit, hitBufferSize: _c.uint32_t, #by_ptr filterData: PxQueryFilterData, filterCall: ^PxQueryFilterCallback) -> _c.int32_t ---

    /// Test returning, for a given geometry, any overlapping object in the scene.
    ///
    /// Filtering: Overlap tests do not distinguish between touching and blocking hit types. Both trigger a hit.
    ///
    /// PxHitFlag::eMESH_MULTIPLE and PxHitFlag::eMESH_BOTH_SIDES have no effect in this case
    ///
    /// True if an overlap was found.
    @(link_name = "PxSceneQueryExt_overlapAny")
    scene_query_ext_overlap_any :: proc(#by_ptr scene: PxScene, geometry: ^PxGeometry, #by_ptr pose: PxTransform, hit: ^PxOverlapHit, #by_ptr filterData: PxQueryFilterData, filterCall: ^PxQueryFilterCallback) -> _c.bool ---

    @(link_name = "PxBatchQueryExt_release_mut")
    batch_query_ext_release_mut :: proc(self_: ^PxBatchQueryExt) ---

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
    batch_query_ext_raycast_mut :: proc(self_: ^PxBatchQueryExt, #by_ptr origin: PxVec3, #by_ptr unitDir: PxVec3, distance: _c.float, maxNbTouches: _c.uint16_t, hitFlags: PxHitFlags_Set, #by_ptr filterData: PxQueryFilterData, cache: ^PxQueryCache) -> ^PxRaycastBuffer ---

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
    batch_query_ext_sweep_mut :: proc(self_: ^PxBatchQueryExt, geometry: ^PxGeometry, #by_ptr pose: PxTransform, #by_ptr unitDir: PxVec3, distance: _c.float, maxNbTouches: _c.uint16_t, hitFlags: PxHitFlags_Set, #by_ptr filterData: PxQueryFilterData, cache: ^PxQueryCache, inflation: _c.float) -> ^PxSweepBuffer ---

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
    batch_query_ext_overlap_mut :: proc(self_: ^PxBatchQueryExt, geometry: ^PxGeometry, #by_ptr pose: PxTransform, maxNbTouches: _c.uint16_t, #by_ptr filterData: PxQueryFilterData, cache: ^PxQueryCache) -> ^PxOverlapBuffer ---

    @(link_name = "PxBatchQueryExt_execute_mut")
    batch_query_ext_execute_mut :: proc(self_: ^PxBatchQueryExt) ---

    /// Create a PxBatchQueryExt without the need for pre-allocated result or touch buffers.
    ///
    /// Returns a PxBatchQueryExt instance. A NULL pointer will be returned if the subsequent allocations fail or if any of the arguments are illegal.
    /// In the event that a NULL pointer is returned a corresponding error will be issued to the error stream.
    @(link_name = "phys_PxCreateBatchQueryExt")
    create_batch_query_ext :: proc(#by_ptr scene: PxScene, queryFilterCallback: ^PxQueryFilterCallback, maxNbRaycasts: _c.uint32_t, maxNbRaycastTouches: _c.uint32_t, maxNbSweeps: _c.uint32_t, maxNbSweepTouches: _c.uint32_t, maxNbOverlaps: _c.uint32_t, maxNbOverlapTouches: _c.uint32_t) -> ^PxBatchQueryExt ---

    /// Create a PxBatchQueryExt with user-supplied result and touch buffers.
    ///
    /// Returns a PxBatchQueryExt instance. A NULL pointer will be returned if the subsequent allocations fail or if any of the arguments are illegal.
    /// In the event that a NULL pointer is returned a corresponding error will be issued to the error stream.
    @(link_name = "phys_PxCreateBatchQueryExt_1")
    create_batch_query_ext_1 :: proc(#by_ptr scene: PxScene, queryFilterCallback: ^PxQueryFilterCallback, raycastBuffers: ^PxRaycastBuffer, maxNbRaycasts: _c.uint32_t, raycastTouches: ^PxRaycastHit, maxNbRaycastTouches: _c.uint32_t, sweepBuffers: ^PxSweepBuffer, maxNbSweeps: _c.uint32_t, sweepTouches: ^PxSweepHit, maxNbSweepTouches: _c.uint32_t, overlapBuffers: ^PxOverlapBuffer, maxNbOverlaps: _c.uint32_t, overlapTouches: ^PxOverlapHit, maxNbOverlapTouches: _c.uint32_t) -> ^PxBatchQueryExt ---

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
    create_external_scene_query_system :: proc(desc: ^PxSceneQueryDesc, contextID: _c.uint64_t) -> ^PxSceneQuerySystem ---

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
    custom_scene_query_system_add_pruner_mut :: proc(self_: ^PxCustomSceneQuerySystem, primaryType: PxPruningStructureType, secondaryType: PxDynamicTreeSecondaryPruner, preallocated: _c.uint32_t) -> _c.uint32_t ---

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
    custom_scene_query_system_start_custom_buildstep_mut :: proc(self_: ^PxCustomSceneQuerySystem) -> _c.uint32_t ---

    /// Perform a custom build-step for a given pruner.
    @(link_name = "PxCustomSceneQuerySystem_customBuildstep_mut")
    custom_scene_query_system_custom_buildstep_mut :: proc(self_: ^PxCustomSceneQuerySystem, index: _c.uint32_t) ---

    /// Finish custom build-steps
    ///
    /// Call this function once after all the customBuildstep() calls are done.
    @(link_name = "PxCustomSceneQuerySystem_finishCustomBuildstep_mut")
    custom_scene_query_system_finish_custom_buildstep_mut :: proc(self_: ^PxCustomSceneQuerySystem) ---

    @(link_name = "PxCustomSceneQuerySystemAdapter_delete")
    custom_scene_query_system_adapter_delete :: proc(self_: ^PxCustomSceneQuerySystemAdapter) ---

    /// Gets a pruner index for an actor/shape.
    ///
    /// This user-defined function tells the system in which pruner a given actor/shape should go.
    ///
    /// The returned index must be valid, i.e. it must have been previously returned to users by PxCustomSceneQuerySystem::addPruner.
    ///
    /// A pruner index for this actor/shape.
    @(link_name = "PxCustomSceneQuerySystemAdapter_getPrunerIndex")
    custom_scene_query_system_adapter_get_pruner_index :: proc(self_: ^PxCustomSceneQuerySystemAdapter, actor: ^PxRigidActor, #by_ptr shape: PxShape) -> _c.uint32_t ---

    /// Pruner filtering callback.
    ///
    /// This will be called for each query to validate whether it should process a given pruner.
    ///
    /// True to process the pruner, false to skip it entirely
    @(link_name = "PxCustomSceneQuerySystemAdapter_processPruner")
    custom_scene_query_system_adapter_process_pruner :: proc(self_: ^PxCustomSceneQuerySystemAdapter, prunerIndex: _c.uint32_t, context_: ^PxQueryThreadContext, #by_ptr filterData: PxQueryFilterData, filterCall: ^PxQueryFilterCallback) -> _c.bool ---

    /// Creates a custom scene query system.
    ///
    /// This is similar to PxCreateExternalSceneQuerySystem, except this function creates a PxCustomSceneQuerySystem object.
    /// It can be plugged to PxScene the same way, via PxSceneDesc::sceneQuerySystem.
    ///
    /// A custom SQ system instance
    @(link_name = "phys_PxCreateCustomSceneQuerySystem")
    create_custom_scene_query_system :: proc(sceneQueryUpdateMode: PxSceneQueryUpdateMode, contextID: _c.uint64_t, #by_ptr adapter: PxCustomSceneQuerySystemAdapter, usesTreeOfPruners: _c.bool) -> ^PxCustomSceneQuerySystem ---

    /// Computes closest polygon of the convex hull geometry for a given impact point
    /// and impact direction. When doing sweeps against a scene, one might want to delay
    /// the rather expensive computation of the hit face index for convexes until it is clear
    /// the information is really needed and then use this method to get the corresponding
    /// face index.
    ///
    /// Closest face index of the convex geometry.
    @(link_name = "phys_PxFindFaceIndex")
    find_face_index :: proc(#by_ptr convexGeom: PxConvexMeshGeometry, #by_ptr geomPose: PxTransform, #by_ptr impactPos: PxVec3, #by_ptr unitDir: PxVec3) -> _c.uint32_t ---

    /// Sets the sampling radius
    ///
    /// Returns true if the sampling was successful and false if there was a problem. Usually an internal overflow is the problem for very big meshes or very small sampling radii.
    @(link_name = "PxPoissonSampler_setSamplingRadius_mut")
    poisson_sampler_set_sampling_radius_mut :: proc(self_: ^PxPoissonSampler, samplingRadius: _c.float) -> _c.bool ---

    /// Adds new Poisson Samples inside the sphere specified
    @(link_name = "PxPoissonSampler_addSamplesInSphere_mut")
    poisson_sampler_add_samples_in_sphere_mut :: proc(self_: ^PxPoissonSampler, #by_ptr sphereCenter: PxVec3, sphereRadius: _c.float, createVolumeSamples: _c.bool) ---

    /// Adds new Poisson Samples inside the box specified
    @(link_name = "PxPoissonSampler_addSamplesInBox_mut")
    poisson_sampler_add_samples_in_box_mut :: proc(self_: ^PxPoissonSampler, #by_ptr axisAlignedBox: PxBounds3, #by_ptr boxOrientation: PxQuat, createVolumeSamples: _c.bool) ---

    @(link_name = "PxPoissonSampler_delete")
    poisson_sampler_delete :: proc(self_: ^PxPoissonSampler) ---

    /// Creates a shape sampler
    ///
    /// Returns the sampler
    @(link_name = "phys_PxCreateShapeSampler")
    create_shape_sampler :: proc(geometry: ^PxGeometry, #by_ptr transform: PxTransform, #by_ptr worldBounds: PxBounds3, initialSamplingRadius: _c.float, numSampleAttemptsAroundPoint: _c.int32_t) -> ^PxPoissonSampler ---

    /// Checks whether a point is inside the triangle mesh
    ///
    /// Returns true if the point is inside the triangle mesh
    @(link_name = "PxTriangleMeshPoissonSampler_isPointInTriangleMesh_mut")
    triangle_mesh_poisson_sampler_is_point_in_triangle_mesh_mut :: proc(self_: ^PxTriangleMeshPoissonSampler, #by_ptr p: PxVec3) -> _c.bool ---

    @(link_name = "PxTriangleMeshPoissonSampler_delete")
    triangle_mesh_poisson_sampler_delete :: proc(self_: ^PxTriangleMeshPoissonSampler) ---

    /// Creates a triangle mesh sampler
    ///
    /// Returns the sampler
    @(link_name = "phys_PxCreateTriangleMeshSampler")
    create_triangle_mesh_sampler :: proc(triangles: ^_c.uint32_t, numTriangles: _c.uint32_t, vertices: ^PxVec3, numVertices: _c.uint32_t, initialSamplingRadius: _c.float, numSampleAttemptsAroundPoint: _c.int32_t) -> ^PxTriangleMeshPoissonSampler ---

    /// Returns the index of the tetrahedron that contains a point
    ///
    /// The index of the tetrahedon containing the point, -1 if not tetrahedron contains the opoint
    @(link_name = "PxTetrahedronMeshExt_findTetrahedronContainingPoint")
    tetrahedron_mesh_ext_find_tetrahedron_containing_point :: proc(mesh: ^PxTetrahedronMesh, #by_ptr point: PxVec3, bary: ^PxVec4, tolerance: _c.float) -> _c.int32_t ---

    /// Returns the index of the tetrahedron closest to a point
    ///
    /// The index of the tetrahedon closest to the point
    @(link_name = "PxTetrahedronMeshExt_findTetrahedronClosestToPoint")
    tetrahedron_mesh_ext_find_tetrahedron_closest_to_point :: proc(mesh: ^PxTetrahedronMesh, #by_ptr point: PxVec3, bary: ^PxVec4) -> _c.int32_t ---

    /// Initialize the PhysXExtensions library.
    ///
    /// This should be called before calling any functions or methods in extensions which may require allocation.
    ///
    /// This function does not need to be called before creating a PxDefaultAllocator object.
    @(link_name = "phys_PxInitExtensions")
    init_extensions :: proc(physics: ^PxPhysics, pvd: ^PxPvd) -> _c.bool ---

    /// Shut down the PhysXExtensions library.
    ///
    /// This function should be called to cleanly shut down the PhysXExtensions library before application exit.
    ///
    /// This function is required to be called to release foundation usage.
    @(link_name = "phys_PxCloseExtensions")
    close_extensions :: proc() ---

    @(link_name = "PxRepXObject_new")
    rep_x_object_new :: proc(inTypeName: ^_c.char, inSerializable: rawptr, inId: _c.uint64_t) -> PxRepXObject ---

    @(link_name = "PxRepXObject_isValid")
    rep_x_object_is_valid :: proc(self_: ^PxRepXObject) -> _c.bool ---

    @(link_name = "PxRepXInstantiationArgs_new")
    rep_x_instantiation_args_new :: proc(inPhysics: ^PxPhysics, inCooking: ^PxCooking, inStringTable: ^PxStringTable) -> PxRepXInstantiationArgs ---

    /// The type this Serializer is meant to operate on.
    @(link_name = "PxRepXSerializer_getTypeName_mut")
    rep_x_serializer_get_type_name_mut :: proc(self_: ^PxRepXSerializer) -> ^_c.char ---

    /// Convert from a RepX object to a key-value pair hierarchy
    @(link_name = "PxRepXSerializer_objectToFile_mut")
    rep_x_serializer_object_to_file_mut :: proc(self_: ^PxRepXSerializer, #by_ptr inLiveObject: PxRepXObject, inCollection: ^PxCollection, inWriter: ^XmlWriter, inTempBuffer: ^MemoryBuffer, inArgs: ^PxRepXInstantiationArgs) ---

    /// Convert from a descriptor to a live object.  Must be an object of this Serializer type.
    ///
    /// The new live object.  It can be an invalid object if the instantiation cannot take place.
    @(link_name = "PxRepXSerializer_fileToObject_mut")
    rep_x_serializer_file_to_object_mut :: proc(self_: ^PxRepXSerializer, inReader: ^XmlReader, inAllocator: ^XmlMemoryAllocator, inArgs: ^PxRepXInstantiationArgs, inCollection: ^PxCollection) -> PxRepXObject ---

    /// Connects the SDK to the PhysX Visual Debugger application.
    @(link_name = "PxPvd_connect_mut")
    pvd_connect_mut :: proc(self_: ^PxPvd, transport: ^PxPvdTransport, flags: PxPvdInstrumentationFlags_Set) -> _c.bool ---

    /// Disconnects the SDK from the PhysX Visual Debugger application.
    /// If we are still connected, this will kill the entire debugger connection.
    @(link_name = "PxPvd_disconnect_mut")
    pvd_disconnect_mut :: proc(self_: ^PxPvd) ---

    /// Return if connection to PVD is created.
    @(link_name = "PxPvd_isConnected_mut")
    pvd_is_connected_mut :: proc(self_: ^PxPvd, useCachedStatus: _c.bool) -> _c.bool ---

    /// returns the PVD data transport
    /// returns NULL if no transport is present.
    @(link_name = "PxPvd_getTransport_mut")
    pvd_get_transport_mut :: proc(self_: ^PxPvd) -> ^PxPvdTransport ---

    /// Retrieves the PVD flags. See PxPvdInstrumentationFlags.
    @(link_name = "PxPvd_getInstrumentationFlags_mut")
    pvd_get_instrumentation_flags_mut :: proc(self_: ^PxPvd) -> PxPvdInstrumentationFlags_Set ---

    /// Releases the pvd instance.
    @(link_name = "PxPvd_release_mut")
    pvd_release_mut :: proc(self_: ^PxPvd) ---

    /// Create a pvd instance.
    @(link_name = "phys_PxCreatePvd")
    create_pvd :: proc(foundation: ^PxFoundation) -> ^PxPvd ---

    /// Connects to the Visual Debugger application.
    /// return True if success
    @(link_name = "PxPvdTransport_connect_mut")
    pvd_transport_connect_mut :: proc(self_: ^PxPvdTransport) -> _c.bool ---

    /// Disconnects from the Visual Debugger application.
    /// If we are still connected, this will kill the entire debugger connection.
    @(link_name = "PxPvdTransport_disconnect_mut")
    pvd_transport_disconnect_mut :: proc(self_: ^PxPvdTransport) ---

    /// Return if connection to PVD is created.
    @(link_name = "PxPvdTransport_isConnected_mut")
    pvd_transport_is_connected_mut :: proc(self_: ^PxPvdTransport) -> _c.bool ---

    /// write bytes to the other endpoint of the connection. should lock before witre. If an error occurs
    /// this connection will assume to be dead.
    @(link_name = "PxPvdTransport_write_mut")
    pvd_transport_write_mut :: proc(self_: ^PxPvdTransport, inBytes: ^_c.uint8_t, inLength: _c.uint32_t) -> _c.bool ---

    @(link_name = "PxPvdTransport_lock_mut")
    pvd_transport_lock_mut :: proc(self_: ^PxPvdTransport) -> ^PxPvdTransport ---

    @(link_name = "PxPvdTransport_unlock_mut")
    pvd_transport_unlock_mut :: proc(self_: ^PxPvdTransport) ---

    /// send any data and block until we know it is at least on the wire.
    @(link_name = "PxPvdTransport_flush_mut")
    pvd_transport_flush_mut :: proc(self_: ^PxPvdTransport) ---

    /// Return size of written data.
    @(link_name = "PxPvdTransport_getWrittenDataSize_mut")
    pvd_transport_get_written_data_size_mut :: proc(self_: ^PxPvdTransport) -> _c.uint64_t ---

    @(link_name = "PxPvdTransport_release_mut")
    pvd_transport_release_mut :: proc(self_: ^PxPvdTransport) ---

    /// Create a default socket transport.
    @(link_name = "phys_PxDefaultPvdSocketTransportCreate")
    default_pvd_socket_transport_create :: proc(host: ^_c.char, port: _c.int32_t, timeoutInMilliseconds: _c.uint32_t) -> ^PxPvdTransport ---

    /// Create a default file transport.
    @(link_name = "phys_PxDefaultPvdFileTransportCreate")
    default_pvd_file_transport_create :: proc(name: ^_c.char) -> ^PxPvdTransport ---

}
