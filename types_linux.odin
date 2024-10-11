package physx
import _c "core:c"
/// enum for empty constructor tag
EMPTY :: enum _c.int32_t {
    Empty = 0,
}

/// enum for zero constructor tag for vectors and matrices
ZERO :: enum _c.int32_t {
    Zero = 0,
}

/// enum for identity constructor flag for quaternions, transforms, and matrices
IDENTITY :: enum _c.int32_t {
    Identity = 0,
}

/// Error codes
///
/// These error codes are passed to [`PxErrorCallback`]
ErrorCode :: enum _c.int32_t {
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

ThreadPriority :: enum _c.uint32_t {
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
DebugColor :: enum _c.uint32_t {
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
ConcreteType :: enum _c.int32_t {
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
BaseFlag :: enum _c.int32_t {
    OwnsMemory = 0,
    IsReleasable = 1,
}

/// Flags for [`PxBaseFlag`]
BaseFlags_Set :: bit_set[BaseFlag; _c.uint16_t]


/// Flags used to configure binary meta data entries, typically set through PX_DEF_BIN_METADATA defines.
MetaDataFlag :: enum _c.int32_t {
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
TaskType :: enum _c.int32_t {
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
GeometryType :: enum _c.int32_t {
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
GeometryQueryFlag :: enum _c.int32_t {
    SimdGuard = 0,
}

/// Flags for [`PxGeometryQueryFlag`]
GeometryQueryFlags_Set :: bit_set[GeometryQueryFlag; _c.uint32_t]


/// Desired build strategy for bounding-volume hierarchies
BVHBuildStrategy :: enum _c.int32_t {
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
ConvexMeshGeometryFlag :: enum _c.int32_t {
    TightBounds = 0,
}

/// Flags for [`PxConvexMeshGeometryFlag`]
ConvexMeshGeometryFlags_Set :: bit_set[ConvexMeshGeometryFlag; _c.uint8_t]


/// Flags controlling the simulated behavior of the triangle mesh geometry.
///
/// Used in ::PxMeshGeometryFlags.
MeshGeometryFlag :: enum _c.int32_t {
    TightBounds = 0,
    DoubleSided = 1,
}

/// Flags for [`PxMeshGeometryFlag`]
MeshGeometryFlags_Set :: bit_set[MeshGeometryFlag; _c.uint8_t]


/// Identifies the solver to use for a particle system.
ParticleSolverType :: enum _c.int32_t {
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
HitFlag :: enum _c.int32_t {
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
HitFlag_Default :: HitFlag.Position | HitFlag.Normal | HitFlag.FaceIndex
HitFlag_ModifiableFlags :: HitFlag.AssumeNoInitialOverlap | HitFlag.MeshMultiple | HitFlag.MeshBothSides | HitFlag.PreciseSweep

/// Flags for [`PxHitFlag`]
HitFlags_Set :: bit_set[HitFlag; _c.uint16_t]


/// Describes the format of height field samples.
HeightFieldFormat :: enum _c.int32_t {
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
HeightFieldTessFlag :: enum _c.int32_t {
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
HeightFieldFlag :: enum _c.int32_t {
    NoBoundaryEdges = 0,
}

/// Flags for [`PxHeightFieldFlag`]
HeightFieldFlags_Set :: bit_set[HeightFieldFlag; _c.uint16_t]


/// Special material index values for height field samples.
HeightFieldMaterial :: enum _c.int32_t {
    /// A material indicating that the triangle should be treated as a hole in the mesh.
    Hole = 127,
}

MeshMeshQueryFlag :: enum _c.int32_t {
    DiscardCoplanar = 0,
}
MeshMeshQueryFlag_Default :: 0

/// Flags for [`PxMeshMeshQueryFlag`]
MeshMeshQueryFlags_Set :: bit_set[MeshMeshQueryFlag; _c.uint32_t]


/// Enum with flag values to be used in PxSimpleTriangleMesh::flags.
MeshFlag :: enum _c.int32_t {
    Flipnormals = 0,
    E16BitIndices = 1,
}

/// Flags for [`PxMeshFlag`]
MeshFlags_Set :: bit_set[MeshFlag; _c.uint16_t]


/// Mesh midphase structure. This enum is used to select the desired acceleration structure for midphase queries
/// (i.e. raycasts, overlaps, sweeps vs triangle meshes).
///
/// The PxMeshMidPhase::eBVH33 structure is the one used in recent PhysX versions (up to PhysX 3.3). It has great performance and is
/// supported on all platforms. It is deprecated since PhysX 5.x.
///
/// The PxMeshMidPhase::eBVH34 structure is a revisited implementation introduced in PhysX 3.4. It can be significantly faster both
/// in terms of cooking performance and runtime performance.
MeshMidPhase :: enum _c.int32_t {
    /// Default midphase mesh structure, as used up to PhysX 3.3 (deprecated)
    Bvh33 = 0,
    /// New midphase mesh structure, introduced in PhysX 3.4
    Bvh34 = 1,
    Last = 2,
}

/// Flags for the mesh geometry properties.
///
/// Used in ::PxTriangleMeshFlags.
TriangleMeshFlag :: enum _c.int32_t {
    E16BitIndices = 1,
    AdjacencyInfo = 2,
    PreferNoSdfProj = 3,
}

/// Flags for [`PxTriangleMeshFlag`]
TriangleMeshFlags_Set :: bit_set[TriangleMeshFlag; _c.uint8_t]


TetrahedronMeshFlag :: enum _c.int32_t {
    E16BitIndices = 1,
}

/// Flags for [`PxTetrahedronMeshFlag`]
TetrahedronMeshFlags_Set :: bit_set[TetrahedronMeshFlag; _c.uint8_t]


/// Flags which control the behavior of an actor.
ActorFlag :: enum _c.int32_t {
    Visualization = 0,
    DisableGravity = 1,
    SendSleepNotifies = 2,
    DisableSimulation = 3,
}

/// Flags for [`PxActorFlag`]
ActorFlags_Set :: bit_set[ActorFlag; _c.uint8_t]


/// Identifies each type of actor.
ActorType :: enum _c.int32_t {
    /// A static rigid body
    RigidStatic = 0,
    /// A dynamic rigid body
    RigidDynamic = 1,
    /// An articulation link
    ArticulationLink = 2,
}

AggregateType :: enum _c.int32_t {
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
OneDConstraintFlag :: enum _c.int32_t {
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
OneDConstraintFlags_Set :: bit_set[OneDConstraintFlag; _c.uint16_t]


/// Constraint type hints which the solver uses to optimize constraint handling
ConstraintSolveHint :: enum _c.int32_t {
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
ConstraintVisualizationFlag :: enum _c.int32_t {
    /// visualize constraint frames
    LocalFrames = 1,
    /// visualize constraint limits
    Limits = 2,
}

/// Flags for determining how PVD should serialize a constraint update
PvdUpdateType :: enum _c.int32_t {
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
ArticulationAxis :: enum _c.int32_t {
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

ArticulationMotion :: enum _c.int32_t {
    Limited = 0,
    Free = 1,
}
ArticulationMotion_Locked :: 0

/// Flags for [`PxArticulationMotion`]
ArticulationMotions_Set :: bit_set[ArticulationMotion; _c.uint8_t]


ArticulationJointType :: enum _c.int32_t {
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

ArticulationFlag :: enum _c.int32_t {
    FixBase = 0,
    DriveLimitsAreForces = 1,
    DisableSelfCollision = 2,
    ComputeJointForces = 3,
}

/// Flags for [`PxArticulationFlag`]
ArticulationFlags_Set :: bit_set[ArticulationFlag; _c.uint8_t]


ArticulationDriveType :: enum _c.int32_t {
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
ArticulationGpuDataType :: enum _c.int32_t {
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
ArticulationCacheFlag :: enum _c.int32_t {
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
ArticulationCacheFlag_All :: ArticulationCacheFlag.Velocity | ArticulationCacheFlag.Acceleration | ArticulationCacheFlag.Position | ArticulationCacheFlag.LinkVelocity | ArticulationCacheFlag.LinkAcceleration | ArticulationCacheFlag.RootTransform | ArticulationCacheFlag.RootVelocities

/// Flags for [`PxArticulationCacheFlag`]
ArticulationCacheFlags_Set :: bit_set[ArticulationCacheFlag; _c.uint32_t]


/// Flags to configure the forces reported by articulation link sensors.
ArticulationSensorFlag :: enum _c.int32_t {
    ForwardDynamicsForces = 0,
    ConstraintSolverForces = 1,
    WorldFrame = 2,
}

/// Flags for [`PxArticulationSensorFlag`]
ArticulationSensorFlags_Set :: bit_set[ArticulationSensorFlag; _c.uint8_t]


/// Flag that configures articulation-state updates by PxArticulationReducedCoordinate::updateKinematic.
ArticulationKinematicFlag :: enum _c.int32_t {
    Position = 0,
    Velocity = 1,
}

/// Flags for [`PxArticulationKinematicFlag`]
ArticulationKinematicFlags_Set :: bit_set[ArticulationKinematicFlag; _c.uint8_t]


/// Flags which affect the behavior of PxShapes.
ShapeFlag :: enum _c.int32_t {
    SimulationShape = 0,
    SceneQueryShape = 1,
    TriggerShape = 2,
    Visualization = 3,
}

/// Flags for [`PxShapeFlag`]
ShapeFlags_Set :: bit_set[ShapeFlag; _c.uint8_t]


/// Parameter to addForce() and addTorque() calls, determines the exact operation that is carried out.
ForceMode :: enum _c.int32_t {
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
RigidBodyFlag :: enum _c.int32_t {
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
RigidBodyFlags_Set :: bit_set[RigidBodyFlag; _c.uint16_t]


/// constraint flags
///
/// eBROKEN is a read only flag
ConstraintFlag :: enum _c.int32_t {
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
ConstraintFlag_Projection :: ConstraintFlag.ProjectToActor0 | ConstraintFlag.ProjectToActor1

/// Flags for [`PxConstraintFlag`]
ConstraintFlags_Set :: bit_set[ConstraintFlag; _c.uint16_t]


/// Header for a contact patch where all points share same material and normal
ContactPatchFlags :: enum _c.int32_t {
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
DeletionEventFlag :: enum _c.int32_t {
    UserRelease = 0,
    MemoryRelease = 1,
}

/// Flags for [`PxDeletionEventFlag`]
DeletionEventFlags_Set :: bit_set[DeletionEventFlag; _c.uint8_t]


/// Collection of flags describing the actions to take for a collision pair.
PairFlag :: enum _c.int32_t {
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
PairFlag_ContactDefault :: PairFlag.SolveContact | PairFlag.DetectDiscreteContact
PairFlag_TriggerDefault :: PairFlag.NotifyTouchFound | PairFlag.NotifyTouchLost | PairFlag.DetectDiscreteContact

/// Flags for [`PxPairFlag`]
PairFlags_Set :: bit_set[PairFlag; _c.uint16_t]


/// Collection of flags describing the filter actions to take for a collision pair.
FilterFlag :: enum _c.int32_t {
    Kill = 0,
    Suppress = 1,
    Callback = 2,
}
FilterFlag_Notify :: FilterFlag.Callback
FilterFlag_Default :: 0

/// Flags for [`PxFilterFlag`]
FilterFlags_Set :: bit_set[FilterFlag; _c.uint16_t]


/// Identifies each type of filter object.
FilterObjectType :: enum _c.int32_t {
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

FilterObjectFlag :: enum _c.int32_t {
    Kinematic = 16,
    Trigger = 32,
}

PairFilteringMode :: enum _c.int32_t {
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

DataAccessFlag :: enum _c.int32_t {
    Readable = 0,
    Writable = 1,
    Device = 2,
}

/// Flags for [`PxDataAccessFlag`]
DataAccessFlags_Set :: bit_set[DataAccessFlag; _c.uint8_t]


/// Flags which control the behavior of a material.
MaterialFlag :: enum _c.int32_t {
    DisableFriction = 0,
    DisableStrongFriction = 1,
    ImprovedPatchFriction = 2,
    CompliantContact = 3,
}

/// Flags for [`PxMaterialFlag`]
MaterialFlags_Set :: bit_set[MaterialFlag; _c.uint16_t]


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
CombineMode :: enum _c.int32_t {
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
ParticleBufferFlag :: enum _c.int32_t {
    UpdatePosition = 0,
    UpdateVelocity = 1,
    UpdatePhase = 2,
    UpdateRestposition = 3,
    UpdateCloth = 5,
    UpdateRigid = 6,
    UpdateDiffuseParam = 7,
    UpdateAttachments = 8,
}
ParticleBufferFlag_None :: 0
ParticleBufferFlag_All :: ParticleBufferFlag.UpdatePosition | ParticleBufferFlag.UpdateVelocity | ParticleBufferFlag.UpdatePhase | ParticleBufferFlag.UpdateRestposition | ParticleBufferFlag.UpdateCloth | ParticleBufferFlag.UpdateRigid | ParticleBufferFlag.UpdateDiffuseParam | ParticleBufferFlag.UpdateAttachments

/// Flags for [`PxParticleBufferFlag`]
ParticleBufferFlags_Set :: bit_set[ParticleBufferFlag; _c.uint32_t]


/// Identifies per-particle behavior for a PxParticleSystem.
///
/// See [`PxParticleSystem::createPhase`]().
ParticlePhaseFlag :: enum _c.uint32_t {
    ParticlePhaseSelfCollide = 20,
    ParticlePhaseSelfCollideFilter = 21,
    ParticlePhaseFluid = 22,
}
ParticlePhaseFlag_ParticlePhaseGroupMask :: 0x000fffff
ParticlePhaseFlag_ParticlePhaseFlagsMask :: ParticlePhaseFlag.ParticlePhaseSelfCollide | ParticlePhaseFlag.ParticlePhaseSelfCollideFilter | ParticlePhaseFlag.ParticlePhaseFluid

/// Flags for [`PxParticlePhaseFlag`]
ParticlePhaseFlags_Set :: bit_set[ParticlePhaseFlag; _c.uint32_t]


/// Specifies memory space for a PxBuffer instance.
BufferType :: enum _c.int32_t {
    Host = 0,
    Device = 1,
}

/// Filtering flags for scene queries.
QueryFlag :: enum _c.int32_t {
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
QueryFlags_Set :: bit_set[QueryFlag; _c.uint16_t]


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
QueryHitType :: enum _c.int32_t {
    /// the query should ignore this shape
    None = 0,
    /// a hit on the shape touches the intersection geometry of the query but does not block it
    Touch = 1,
    /// a hit on the shape blocks the query (does not block overlap queries)
    Block = 2,
}

/// Collection of flags providing a mechanism to lock motion along/around a specific axis.
RigidDynamicLockFlag :: enum _c.int32_t {
    LockLinearX = 0,
    LockLinearY = 1,
    LockLinearZ = 2,
    LockAngularX = 3,
    LockAngularY = 4,
    LockAngularZ = 5,
}

/// Flags for [`PxRigidDynamicLockFlag`]
RigidDynamicLockFlags_Set :: bit_set[RigidDynamicLockFlag; _c.uint8_t]


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
PruningStructureType :: enum _c.int32_t {
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
DynamicTreeSecondaryPruner :: enum _c.int32_t {
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
SceneQueryUpdateMode :: enum _c.int32_t {
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
ScenePrunerIndex :: enum _c.uint32_t {
    ScenePrunerStatic = 0,
    ScenePrunerDynamic = 1,
    SceneCompoundPruner = 4294967295,
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
BroadPhaseType :: enum _c.int32_t {
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
FrictionType :: enum _c.int32_t {
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
SolverType :: enum _c.int32_t {
    /// Projected Gauss-Seidel iterative solver
    Pgs = 0,
    /// Default Temporal Gauss-Seidel solver
    Tgs = 1,
}

/// flags for configuring properties of the scene
SceneFlag :: enum _c.int32_t {
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
SceneFlag_MutableFlags :: SceneFlag.EnableActiveActors | SceneFlag.ExcludeKinematicsFromActiveActors | SceneFlag.SuppressReadback

/// Flags for [`PxSceneFlag`]
SceneFlags_Set :: bit_set[SceneFlag; _c.uint32_t]


/// Debug visualization parameters.
///
/// [`PxVisualizationParameter::eSCALE`] is the master switch for enabling visualization, please read the corresponding documentation
/// for further details.
VisualizationParameter :: enum _c.int32_t {
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
SoftBodyDataFlag :: enum _c.int32_t {
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
HairSystemData :: enum _c.int32_t {
    PositionInvmass = 0,
    Velocity = 1,
}
HairSystemData_None :: 0
HairSystemData_All :: HairSystemData.PositionInvmass | HairSystemData.Velocity

/// Flags for [`PxHairSystemData`]
HairSystemDataFlags_Set :: bit_set[HairSystemData; _c.uint32_t]


/// Binary settings for hair system simulation
HairSystemFlag :: enum _c.int32_t {
    DisableSelfCollision = 0,
    DisableExternalCollision = 1,
    DisableTwosidedAttachment = 2,
}

/// Flags for [`PxHairSystemFlag`]
HairSystemFlags_Set :: bit_set[HairSystemFlag; _c.uint32_t]


/// Identifies each type of information for retrieving from actor.
ActorCacheFlag :: enum _c.int32_t {
    ActorData = 0,
    Force = 2,
    Torque = 3,
}

/// Flags for [`PxActorCacheFlag`]
ActorCacheFlags_Set :: bit_set[ActorCacheFlag; _c.uint16_t]


/// PVD scene Flags. They are disabled by default, and only works if PxPvdInstrumentationFlag::eDEBUG is set.
PvdSceneFlag :: enum _c.int32_t {
    TransmitContacts = 0,
    TransmitScenequeries = 1,
    TransmitConstraints = 2,
}

/// Flags for [`PxPvdSceneFlag`]
PvdSceneFlags_Set :: bit_set[PvdSceneFlag; _c.uint8_t]


/// Identifies each type of actor for retrieving actors from a scene.
///
/// [`PxArticulationLink`] objects are not supported. Use the #PxArticulationReducedCoordinate object to retrieve all its links.
ActorTypeFlag :: enum _c.int32_t {
    RigidStatic = 0,
    RigidDynamic = 1,
}

/// Flags for [`PxActorTypeFlag`]
ActorTypeFlags_Set :: bit_set[ActorTypeFlag; _c.uint16_t]


/// Extra data item types for contact pairs.
ContactPairExtraDataType :: enum _c.int32_t {
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
ContactPairHeaderFlag :: enum _c.int32_t {
    RemovedActor0 = 0,
    RemovedActor1 = 1,
}

/// Flags for [`PxContactPairHeaderFlag`]
ContactPairHeaderFlags_Set :: bit_set[ContactPairHeaderFlag; _c.uint16_t]


/// Collection of flags providing information on contact report pairs.
ContactPairFlag :: enum _c.int32_t {
    RemovedShape0 = 0,
    RemovedShape1 = 1,
    ActorPairHasFirstTouch = 2,
    ActorPairLostTouch = 3,
    InternalHasImpulses = 4,
    InternalContactsAreFlipped = 5,
}

/// Flags for [`PxContactPairFlag`]
ContactPairFlags_Set :: bit_set[ContactPairFlag; _c.uint16_t]


/// Collection of flags providing information on trigger report pairs.
TriggerPairFlag :: enum _c.int32_t {
    RemovedShapeTrigger = 0,
    RemovedShapeOther = 1,
    NextFree = 2,
}

/// Flags for [`PxTriggerPairFlag`]
TriggerPairFlags_Set :: bit_set[TriggerPairFlag; _c.uint8_t]


/// Identifies input and output buffers for PxSoftBody.
SoftBodyData :: enum _c.int32_t {
    PositionInvmass = 0,
    SimPositionInvmass = 2,
    SimVelocity = 3,
    SimKinematicTarget = 4,
}
SoftBodyData_None :: 0
SoftBodyData_All :: SoftBodyData.PositionInvmass | SoftBodyData.SimPositionInvmass | SoftBodyData.SimVelocity | SoftBodyData.SimKinematicTarget

/// Flags for [`PxSoftBodyData`]
SoftBodyDataFlags_Set :: bit_set[SoftBodyData; _c.uint32_t]


/// Flags to enable or disable special modes of a SoftBody
SoftBodyFlag :: enum _c.int32_t {
    DisableSelfCollision = 0,
    ComputeStressTensor = 1,
    EnableCcd = 2,
    DisplaySimMesh = 3,
    Kinematic = 4,
    PartiallyKinematic = 5,
}

/// Flags for [`PxSoftBodyFlag`]
SoftBodyFlags_Set :: bit_set[SoftBodyFlag; _c.uint32_t]


/// The type of controller, eg box, sphere or capsule.
ControllerShapeType :: enum _c.int32_t {
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
ControllerNonWalkableMode :: enum _c.int32_t {
    /// Stops character from climbing up non-walkable slopes, but doesn't move it otherwise
    PreventClimbing = 0,
    /// Stops character from climbing up non-walkable slopes, and forces it to slide down those slopes
    PreventClimbingAndForceSliding = 1,
}

/// specifies which sides a character is colliding with.
ControllerCollisionFlag :: enum _c.int32_t {
    CollisionSides = 0,
    CollisionUp = 1,
    CollisionDown = 2,
}

/// Flags for [`PxControllerCollisionFlag`]
ControllerCollisionFlags_Set :: bit_set[ControllerCollisionFlag; _c.uint8_t]


CapsuleClimbingMode :: enum _c.int32_t {
    /// Standard mode, let the capsule climb over surfaces according to impact normal
    Easy = 0,
    /// Constrained mode, try to limit climbing according to the step offset
    Constrained = 1,
    Last = 2,
}

/// specifies controller behavior
ControllerBehaviorFlag :: enum _c.int32_t {
    CctCanRideOnObject = 0,
    CctSlide = 1,
    CctUserDefinedRide = 2,
}

/// Flags for [`PxControllerBehaviorFlag`]
ControllerBehaviorFlags_Set :: bit_set[ControllerBehaviorFlag; _c.uint8_t]


/// specifies debug-rendering flags
ControllerDebugRenderFlag :: enum _c.uint32_t {
    TemporalBv = 0,
    CachedBv = 1,
    Obstacles = 2,
}
ControllerDebugRenderFlag_None :: 0
ControllerDebugRenderFlag_All :: ControllerDebugRenderFlag.TemporalBv | ControllerDebugRenderFlag.CachedBv | ControllerDebugRenderFlag.Obstacles

/// Flags for [`PxControllerDebugRenderFlag`]
ControllerDebugRenderFlags_Set :: bit_set[ControllerDebugRenderFlag; _c.uint32_t]


/// Defines the number of bits per subgrid pixel
SdfBitsPerSubgridPixel :: enum _c.int32_t {
    /// 8 bit per subgrid pixel (values will be stored as normalized integers)
    E8BitPerPixel = 1,
    /// 16 bit per subgrid pixel (values will be stored as normalized integers)
    E16BitPerPixel = 2,
    /// 32 bit per subgrid pixel (values will be stored as floats in world scale units)
    E32BitPerPixel = 4,
}

/// Flags which describe the format and behavior of a convex mesh.
ConvexFlag :: enum _c.int32_t {
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
ConvexFlags_Set :: bit_set[ConvexFlag; _c.uint16_t]


/// Defines the tetrahedron structure of a mesh.
MeshFormat :: enum _c.int32_t {
    /// Normal tetmesh with arbitrary tetrahedra
    TetMesh = 0,
    /// 6 tetrahedra in a row will form a hexahedron
    HexMesh = 1,
}

/// Desired build strategy for PxMeshMidPhase::eBVH34
BVH34BuildStrategy :: enum _c.int32_t {
    /// Fast build strategy. Fast build speed, good runtime performance in most cases. Recommended for runtime mesh cooking.
    Fast = 0,
    /// Default build strategy. Medium build speed, good runtime performance in all cases.
    Default = 1,
    /// SAH build strategy. Slower builds, slightly improved runtime performance in some cases.
    Sah = 2,
    Last = 3,
}

/// Result from convex cooking.
ConvexMeshCookingResult :: enum _c.int32_t {
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
ConvexMeshCookingType :: enum _c.int32_t {
    /// The Quickhull algorithm constructs the hull from the given input points. The resulting hull
    /// will only contain a subset of the input points.
    Quickhull = 0,
}

/// Result from triangle mesh cooking
TriangleMeshCookingResult :: enum _c.int32_t {
    /// Everything is A-OK.
    Success = 0,
    /// a triangle is too large for well-conditioned results. Tessellate the mesh for better behavior, see the user guide section on cooking for more details.
    LargeTriangle = 1,
    /// Something unrecoverable happened. Check the error stream to find out what.
    Failure = 2,
}

/// Enum for the set of mesh pre-processing parameters.
MeshPreprocessingFlag :: enum _c.int32_t {
    WeldVertices = 0,
    DisableCleanMesh = 1,
    DisableActiveEdgesPrecompute = 2,
    Force32bitIndices = 3,
    EnableVertMapping = 4,
    EnableInertia = 5,
}

/// Flags for [`PxMeshPreprocessingFlag`]
MeshPreprocessingFlags_Set :: bit_set[MeshPreprocessingFlag; _c.uint32_t]


/// Unique identifiers for extensions classes which implement a constraint based on PxConstraint.
///
/// Users which want to create their own custom constraint types should choose an ID larger or equal to eNEXT_FREE_ID
/// and not eINVALID_ID.
ConstraintExtIDs :: enum _c.int32_t {
    Joint = 0,
    VehicleSuspLimitDeprecated = 1,
    VehicleStickyTyreDeprecated = 2,
    VehicleJoint = 3,
    NextFreeId = 4,
    InvalidId = 2147483647,
}

/// an enumeration of PhysX' built-in joint types
JointConcreteType :: enum _c.int32_t {
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
JointActorIndex :: enum _c.int32_t {
    Actor0 = 0,
    Actor1 = 1,
    Count = 2,
}

/// flags for configuring the drive of a PxDistanceJoint
DistanceJointFlag :: enum _c.int32_t {
    MaxDistanceEnabled = 1,
    MinDistanceEnabled = 2,
    SpringEnabled = 3,
}

/// Flags for [`PxDistanceJointFlag`]
DistanceJointFlags_Set :: bit_set[DistanceJointFlag; _c.uint16_t]


/// Flags specific to the prismatic joint.
PrismaticJointFlag :: enum _c.int32_t {
    LimitEnabled = 1,
}

/// Flags for [`PxPrismaticJointFlag`]
PrismaticJointFlags_Set :: bit_set[PrismaticJointFlag; _c.uint16_t]


/// Flags specific to the Revolute Joint.
RevoluteJointFlag :: enum _c.int32_t {
    LimitEnabled = 0,
    DriveEnabled = 1,
    DriveFreespin = 2,
}

/// Flags for [`PxRevoluteJointFlag`]
RevoluteJointFlags_Set :: bit_set[RevoluteJointFlag; _c.uint16_t]


/// Flags specific to the spherical joint.
SphericalJointFlag :: enum _c.int32_t {
    LimitEnabled = 1,
}

/// Flags for [`PxSphericalJointFlag`]
SphericalJointFlags_Set :: bit_set[SphericalJointFlag; _c.uint16_t]


/// Used to specify one of the degrees of freedom of  a D6 joint.
D6Axis :: enum _c.int32_t {
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
D6Motion :: enum _c.int32_t {
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
D6Drive :: enum _c.int32_t {
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
D6JointDriveFlag :: enum _c.int32_t {
    Acceleration = 0,
}

/// Flags for [`PxD6JointDriveFlag`]
D6JointDriveFlags_Set :: bit_set[D6JointDriveFlag; _c.uint32_t]


/// Collision filtering operations.
FilterOp :: enum _c.int32_t {
    FilteropAnd = 0,
    FilteropOr = 1,
    FilteropXor = 2,
    FilteropNand = 3,
    FilteropNor = 4,
    FilteropNxor = 5,
    FilteropSwapAnd = 6,
}

/// If a thread ends up waiting for work it will find itself in a spin-wait loop until work becomes available.
/// Three strategies are available to limit wasted cycles.
/// The strategies are as follows:
/// a) wait until a work task signals the end of the spin-wait period.
/// b) yield the thread by providing a hint to reschedule thread execution, thereby allowing other threads to run.
/// c) yield the processor by informing it that it is waiting for work and requesting it to more efficiently use compute resources.
DefaultCpuDispatcherWaitForWorkMode :: enum _c.int32_t {
    WaitForWork = 0,
    YieldThread = 1,
    YieldProcessor = 2,
}

BatchQueryStatus :: enum _c.int32_t {
    /// This is the initial state before a query starts.
    Pending = 0,
    /// The query is finished; results have been written into the result and hit buffers.
    Success = 1,
    /// The query results were incomplete due to touch hit buffer overflow. Blocking hit is still correct.
    Overflow = 2,
}

/// types of instrumentation that PVD can do.
PvdInstrumentationFlag :: enum _c.int32_t {
    Debug = 0,
    Profile = 1,
    Memory = 2,
}
PvdInstrumentationFlag_All :: PvdInstrumentationFlag.Debug | PvdInstrumentationFlag.Profile | PvdInstrumentationFlag.Memory

/// Flags for [`PxPvdInstrumentationFlag`]
PvdInstrumentationFlags_Set :: bit_set[PvdInstrumentationFlag; _c.uint8_t]


Mat34 :: distinct rawptr 

AllocatorCallback :: struct {
    _pad0: [8]u8,
}


AssertHandler :: struct {
    _pad0: [8]u8,
}


Foundation :: struct {
    _pad0: [8]u8,
}


Allocator :: struct {
};

RawAllocator :: struct {
};

VirtualAllocatorCallback :: struct {
    _pad0: [8]u8,
}


VirtualAllocator :: struct {
    _pad0: [16]u8,
}


UserAllocated :: struct {
};

TempAllocatorChunk :: struct #raw_union {
    mNext: ^TempAllocatorChunk,
    mIndex: _c.uint32_t,
    mPad: [16]_c.uint8_t,
};

TempAllocator :: struct {
};

LogTwo :: distinct rawptr 

UnConst :: distinct rawptr 

BitAndByte :: struct {
    _pad0: [1]u8,
}


BitMap :: struct {
    _pad0: [16]u8,
}


Vec3 :: struct {
    x: _c.float,
    y: _c.float,
    z: _c.float,
}


Vec3Padded :: struct {
    using _: Vec3,
    padding: _c.uint32_t,
}


Quat :: struct {
    x: _c.float,
    y: _c.float,
    z: _c.float,
    w: _c.float,
}


Transform :: struct {
    q: Quat,
    p: Vec3,
}


TransformPadded :: struct {
    transform: Transform,
    padding: _c.uint32_t,
}


Mat33 :: struct {
    column0: Vec3,
    column1: Vec3,
    column2: Vec3,
}


Bounds3 :: struct {
    minimum: Vec3,
    maximum: Vec3,
}


ErrorCallback :: struct {
    _pad0: [8]u8,
}


AllocationListener :: struct {
    _pad0: [8]u8,
}


BroadcastingAllocator :: struct {
    using _: AllocatorCallback,
    _pad1: [168]u8,
}


BroadcastingErrorCallback :: struct {
    using _: ErrorCallback,
    _pad1: [152]u8,
}


Hash :: distinct rawptr 

InputStream :: struct {
    _pad0: [8]u8,
}


InputData :: struct {
    using _: InputStream,
}


OutputStream :: struct {
    _pad0: [8]u8,
}


Vec4 :: struct {
    x: _c.float,
    y: _c.float,
    z: _c.float,
    w: _c.float,
}


Mat44 :: struct {
    column0: Vec4,
    column1: Vec4,
    column2: Vec4,
    column3: Vec4,
}


Plane :: struct {
    n: Vec3,
    d: _c.float,
}


Interpolation :: struct {
};

MutexImpl :: struct {
};

ReadWriteLock :: struct {
    _pad0: [8]u8,
}


ProfilerCallback :: struct {
    _pad0: [8]u8,
}


ProfileScoped :: struct {
    mCallback: ^ProfilerCallback,
    mEventName: ^_c.char,
    mProfilerData: rawptr,
    mContextId: _c.uint64_t,
    mDetached: _c.bool,
    _pad5: [7]u8,
}


SListEntry :: struct #align(16){
    _pad0: [16]u8,
}


SListImpl :: struct {
};

SyncImpl :: struct {
};

Runnable :: struct {
    _pad0: [8]u8,
}


CounterFrequencyToTensOfNanos :: struct {
    mNumerator: _c.uint64_t,
    mDenominator: _c.uint64_t,
}


Time :: struct {
    _pad0: [8]u8,
}


Vec2 :: struct {
    x: _c.float,
    y: _c.float,
}


StridedData :: struct {
    stride: _c.uint32_t,
    _pad1: [4]u8,
    data: rawptr,
}


BoundedData :: struct {
    using _: StridedData,
    count: _c.uint32_t,
    _pad4: [4]u8,
}


DebugPoint :: struct {
    pos: Vec3,
    color: _c.uint32_t,
}


DebugLine :: struct {
    pos0: Vec3,
    color0: _c.uint32_t,
    pos1: Vec3,
    color1: _c.uint32_t,
}


DebugTriangle :: struct {
    pos0: Vec3,
    color0: _c.uint32_t,
    pos1: Vec3,
    color1: _c.uint32_t,
    pos2: Vec3,
    color2: _c.uint32_t,
}


DebugText :: struct {
    position: Vec3,
    size: _c.float,
    color: _c.uint32_t,
    _pad3: [4]u8,
    string: ^_c.char,
}


RenderBuffer :: struct {
    _pad0: [8]u8,
}


ProcessPxBaseCallback :: struct {
    _pad0: [8]u8,
}


SerializationContext :: struct {
    _pad0: [8]u8,
}


DeserializationContext :: struct {
    _pad0: [16]u8,
}


SerializationRegistry :: struct {
    _pad0: [8]u8,
}


Collection :: struct {
    _pad0: [8]u8,
}


TypeInfo :: distinct rawptr 

FEMSoftBodyMaterial :: distinct rawptr 

FEMClothMaterial :: distinct rawptr 

PBDMaterial :: distinct rawptr 

FLIPMaterial :: distinct rawptr 

MPMMaterial :: distinct rawptr 

CustomMaterial :: distinct rawptr 

BVH33TriangleMesh :: distinct rawptr 

ParticleSystem :: distinct rawptr 

PBDParticleSystem :: distinct rawptr 

FLIPParticleSystem :: distinct rawptr 

MPMParticleSystem :: distinct rawptr 

CustomParticleSystem :: distinct rawptr 

SoftBody :: distinct rawptr 

FEMCloth :: distinct rawptr 

HairSystem :: distinct rawptr 

ParticleBuffer :: distinct rawptr 

ParticleAndDiffuseBuffer :: distinct rawptr 

ParticleClothBuffer :: distinct rawptr 

ParticleRigidBuffer :: distinct rawptr 

Base :: struct {
    _pad0: [16]u8,
}


RefCounted :: struct {
    using _: Base,
}


TolerancesScale :: struct {
    length: _c.float,
    speed: _c.float,
}


StringTable :: struct {
    _pad0: [8]u8,
}


Serializer :: struct {
    _pad0: [8]u8,
}


MetaDataEntry :: struct {
    type: ^_c.char,
    name: ^_c.char,
    offset: _c.uint32_t,
    size: _c.uint32_t,
    count: _c.uint32_t,
    offsetSize: _c.uint32_t,
    flags: _c.uint32_t,
    alignment: _c.uint32_t,
}


InsertionCallback :: struct {
    _pad0: [8]u8,
}


TaskManager :: struct {
    _pad0: [8]u8,
}


CpuDispatcher :: struct {
    _pad0: [8]u8,
}


BaseTask :: struct {
    _pad0: [24]u8,
}


Task :: struct {
    using _: BaseTask,
    _pad1: [8]u8,
}


LightCpuTask :: struct {
    using _: BaseTask,
    _pad1: [16]u8,
}


Geometry :: struct {
    _pad0: [4]u8,
    mTypePadding: _c.float,
}


BoxGeometry :: struct {
    using _: Geometry,
    halfExtents: Vec3,
}


BVHRaycastCallback :: struct {
    _pad0: [8]u8,
}


BVHOverlapCallback :: struct {
    _pad0: [8]u8,
}


BVHTraversalCallback :: struct {
    _pad0: [8]u8,
}


BVH :: struct {
    using _: Base,
}


CapsuleGeometry :: struct {
    using _: Geometry,
    radius: _c.float,
    halfHeight: _c.float,
}


HullPolygon :: struct {
    mPlane: [4]_c.float,
    mNbVerts: _c.uint16_t,
    mIndexBase: _c.uint16_t,
}


ConvexMesh :: struct {
    using _: RefCounted,
}


MeshScale :: struct {
    scale: Vec3,
    rotation: Quat,
}


ConvexMeshGeometry :: struct {
    using _: Geometry,
    scale: MeshScale,
    _pad3: [4]u8,
    convexMesh: ^ConvexMesh,
    meshFlags: ConvexMeshGeometryFlags_Set,
    _pad6: [7]u8,
}


SphereGeometry :: struct {
    using _: Geometry,
    radius: _c.float,
}


PlaneGeometry :: struct {
    using _: Geometry,
}


TriangleMeshGeometry :: struct {
    using _: Geometry,
    scale: MeshScale,
    meshFlags: MeshGeometryFlags_Set,
    _pad4: [3]u8,
    triangleMesh: ^TriangleMesh,
}


HeightFieldGeometry :: struct {
    using _: Geometry,
    heightField: ^HeightField,
    heightScale: _c.float,
    rowScale: _c.float,
    columnScale: _c.float,
    heightFieldFlags: MeshGeometryFlags_Set,
    _pad7: [3]u8,
}


ParticleSystemGeometry :: struct {
    using _: Geometry,
    mSolverType: ParticleSolverType,
}


HairSystemGeometry :: struct {
    using _: Geometry,
}


TetrahedronMeshGeometry :: struct {
    using _: Geometry,
    tetrahedronMesh: ^TetrahedronMesh,
}


QueryHit :: struct {
    faceIndex: _c.uint32_t,
}


LocationHit :: struct {
    using _: QueryHit,
    flags: HitFlags_Set,
    _pad2: [2]u8,
    position: Vec3,
    normal: Vec3,
    distance: _c.float,
}


GeomRaycastHit :: struct {
    using _: LocationHit,
    u: _c.float,
    v: _c.float,
}


GeomOverlapHit :: struct {
    using _: QueryHit,
}


GeomSweepHit :: struct {
    using _: LocationHit,
}


GeomIndexPair :: struct {
    id0: _c.uint32_t,
    id1: _c.uint32_t,
}


QueryThreadContext :: struct {
};

ContactBuffer :: distinct rawptr 

RenderOutput :: distinct rawptr 

CustomGeometryType :: struct {
    _pad0: [4]u8,
}


CustomGeometryCallbacks :: struct {
    _pad0: [8]u8,
}


CustomGeometry :: struct {
    using _: Geometry,
    callbacks: ^CustomGeometryCallbacks,
}


GeometryHolder :: struct {
    _pad0: [56]u8,
}


GeometryQuery :: struct {
};

HeightFieldSample :: struct {
    height: _c.int16_t,
    materialIndex0: BitAndByte,
    materialIndex1: BitAndByte,
}


HeightField :: struct {
    using _: RefCounted,
}


HeightFieldDesc :: struct {
    nbRows: _c.uint32_t,
    nbColumns: _c.uint32_t,
    format: HeightFieldFormat,
    _pad3: [4]u8,
    samples: StridedData,
    convexEdgeThreshold: _c.float,
    flags: HeightFieldFlags_Set,
    _pad7: [2]u8,
}


MeshQuery :: struct {
};

SimpleTriangleMesh :: struct {
    points: BoundedData,
    triangles: BoundedData,
    flags: MeshFlags_Set,
    _pad3: [6]u8,
}


Triangle :: struct {
    verts: [3]Vec3,
}


TrianglePadded :: struct {
    using _: Triangle,
    padding: _c.uint32_t,
}


TriangleMesh :: struct {
    using _: RefCounted,
}


BVH34TriangleMesh :: struct {
    using _: TriangleMesh,
}


Tetrahedron :: struct {
    verts: [4]Vec3,
}


SoftBodyAuxData :: struct {
    using _: RefCounted,
}


TetrahedronMesh :: struct {
    using _: RefCounted,
}


SoftBodyMesh :: struct {
    using _: RefCounted,
}


CollisionMeshMappingData :: struct {
    using _: UserAllocated,
    _pad0: [8]u8,
}


SoftBodyCollisionData :: struct {
    using _: UserAllocated,
};

TetrahedronMeshData :: struct {
    using _: UserAllocated,
};

SoftBodySimulationData :: struct {
    using _: UserAllocated,
};

CollisionTetrahedronMeshData :: struct {
    using _: UserAllocated,
    _pad0: [8]u8,
}


SimulationTetrahedronMeshData :: struct {
    using _: UserAllocated,
    _pad0: [8]u8,
}


Actor :: struct {
    using _: Base,
    userData: rawptr,
}


Aggregate :: struct {
    using _: Base,
    userData: rawptr,
}


SpringModifiers :: struct #align(16){
    stiffness: _c.float,
    damping: _c.float,
    _pad2: [8]u8,
}


RestitutionModifiers :: struct #align(16){
    restitution: _c.float,
    velocityThreshold: _c.float,
    _pad2: [8]u8,
}


OneDConstraintMods :: struct #raw_union {
    spring: SpringModifiers,
    bounce: RestitutionModifiers,
};

OneDConstraint :: struct {
    linear0: Vec3,
    geometricError: _c.float,
    angular0: Vec3,
    velocityTarget: _c.float,
    linear1: Vec3,
    minImpulse: _c.float,
    angular1: Vec3,
    maxImpulse: _c.float,
    mods: OneDConstraintMods,
    forInternalUse: _c.float,
    flags: _c.uint16_t,
    solveHint: _c.uint16_t,
    _pad12: [8]u8,
}


ConstraintInvMassScale :: struct #align(16){
    linear0: _c.float,
    angular0: _c.float,
    linear1: _c.float,
    angular1: _c.float,
}


ConstraintVisualizer :: struct {
    _pad0: [8]u8,
}


ConstraintConnector :: struct {
    _pad0: [8]u8,
}


ContactPoint :: struct #align(16){
    normal: Vec3,
    separation: _c.float,
    point: Vec3,
    maxImpulse: _c.float,
    targetVel: Vec3,
    staticFriction: _c.float,
    materialFlags: _c.uint8_t,
    _pad7: [3]u8,
    internalFaceIndex1: _c.uint32_t,
    dynamicFriction: _c.float,
    restitution: _c.float,
    damping: _c.float,
    _pad12: [12]u8,
}


SolverBody :: struct {
    linearVelocity: Vec3,
    maxSolverNormalProgress: _c.uint16_t,
    maxSolverFrictionProgress: _c.uint16_t,
    angularState: Vec3,
    solverProgress: _c.uint32_t,
}


SolverBodyData :: struct {
    linearVelocity: Vec3,
    invMass: _c.float,
    angularVelocity: Vec3,
    reportThreshold: _c.float,
    sqrtInvInertia: Mat33,
    penBiasClamp: _c.float,
    nodeIndex: _c.uint32_t,
    maxContactImpulse: _c.float,
    body2World: Transform,
    pad: _c.uint16_t,
    _pad10: [2]u8,
}


ConstraintBatchHeader :: struct {
    startIndex: _c.uint32_t,
    stride: _c.uint16_t,
    constraintType: _c.uint16_t,
}


SolverConstraintDesc :: struct {
    bodyA: ^SolverBody,
    bodyB: ^SolverBody,
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


SolverConstraintPrepDescBase :: struct {
    invMassScales: ConstraintInvMassScale,
    desc: ^SolverConstraintDesc,
    body0: ^SolverBody,
    body1: ^SolverBody,
    data0: ^SolverBodyData,
    data1: ^SolverBodyData,
    bodyFrame0: Transform,
    bodyFrame1: Transform,
    bodyState0: BodyState,
    bodyState1: BodyState,
    _pad10: [8]u8,
}


SolverConstraintPrepDesc :: struct {
    using _: SolverConstraintPrepDescBase,
    rows: ^OneDConstraint,
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
    body0WorldOffset: Vec3Padded,
    _pad25: [8]u8,
}


SolverContactDesc :: struct {
    invMassScales: ConstraintInvMassScale,
    desc: ^SolverConstraintDesc,
    body0: ^SolverBody,
    body1: ^SolverBody,
    data0: ^SolverBodyData,
    data1: ^SolverBodyData,
    bodyFrame0: Transform,
    bodyFrame1: Transform,
    bodyState0: BodyState,
    bodyState1: BodyState,
    shapeInteraction: rawptr,
    contacts: ^ContactPoint,
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


ConstraintAllocator :: struct {
    _pad0: [8]u8,
}


ArticulationLimit :: struct {
    low: _c.float,
    high: _c.float,
}


ArticulationDrive :: struct {
    stiffness: _c.float,
    damping: _c.float,
    maxForce: _c.float,
    driveType: ArticulationDriveType,
}


TGSSolverBodyVel :: struct {
    linearVelocity: Vec3,
    nbStaticInteractions: _c.uint16_t,
    maxDynamicPartition: _c.uint16_t,
    angularVelocity: Vec3,
    partitionMask: _c.uint32_t,
    deltaAngDt: Vec3,
    maxAngVel: _c.float,
    deltaLinDt: Vec3,
    lockFlags: _c.uint16_t,
    isKinematic: _c.bool,
    pad: _c.uint8_t,
}


TGSSolverBodyTxInertia :: struct {
    deltaBody2World: Transform,
    sqrtInvInertia: Mat33,
}


TGSSolverBodyData :: struct {
    originalLinearVelocity: Vec3,
    maxContactImpulse: _c.float,
    originalAngularVelocity: Vec3,
    penBiasClamp: _c.float,
    invMass: _c.float,
    nodeIndex: _c.uint32_t,
    reportThreshold: _c.float,
    pad: _c.uint32_t,
}


TGSSolverConstraintPrepDescBase :: struct {
    invMassScales: ConstraintInvMassScale,
    desc: ^SolverConstraintDesc,
    body0: ^TGSSolverBodyVel,
    body1: ^TGSSolverBodyVel,
    body0TxI: ^TGSSolverBodyTxInertia,
    body1TxI: ^TGSSolverBodyTxInertia,
    bodyData0: ^TGSSolverBodyData,
    bodyData1: ^TGSSolverBodyData,
    bodyFrame0: Transform,
    bodyFrame1: Transform,
    bodyState0: BodyState,
    bodyState1: BodyState,
    _pad12: [8]u8,
}


TGSSolverConstraintPrepDesc :: struct {
    invMassScales: ConstraintInvMassScale,
    desc: ^SolverConstraintDesc,
    body0: ^TGSSolverBodyVel,
    body1: ^TGSSolverBodyVel,
    body0TxI: ^TGSSolverBodyTxInertia,
    body1TxI: ^TGSSolverBodyTxInertia,
    bodyData0: ^TGSSolverBodyData,
    bodyData1: ^TGSSolverBodyData,
    bodyFrame0: Transform,
    bodyFrame1: Transform,
    bodyState0: BodyState,
    bodyState1: BodyState,
    rows: ^OneDConstraint,
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
    body0WorldOffset: Vec3Padded,
    cA2w: Vec3Padded,
    cB2w: Vec3Padded,
}


TGSSolverContactDesc :: struct {
    invMassScales: ConstraintInvMassScale,
    desc: ^SolverConstraintDesc,
    body0: ^TGSSolverBodyVel,
    body1: ^TGSSolverBodyVel,
    body0TxI: ^TGSSolverBodyTxInertia,
    body1TxI: ^TGSSolverBodyTxInertia,
    bodyData0: ^TGSSolverBodyData,
    bodyData1: ^TGSSolverBodyData,
    bodyFrame0: Transform,
    bodyFrame1: Transform,
    bodyState0: BodyState,
    bodyState1: BodyState,
    shapeInteraction: rawptr,
    contacts: ^ContactPoint,
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


ArticulationTendonLimit :: struct {
    lowLimit: _c.float,
    highLimit: _c.float,
}


ArticulationAttachment :: struct {
    using _: Base,
    userData: rawptr,
}


ArticulationTendonJoint :: struct {
    using _: Base,
    userData: rawptr,
}


ArticulationTendon :: struct {
    using _: Base,
    userData: rawptr,
}


ArticulationSpatialTendon :: struct {
    using _: ArticulationTendon,
}


ArticulationFixedTendon :: struct {
    using _: ArticulationTendon,
}


SpatialForce :: struct {
    force: Vec3,
    pad0: _c.float,
    torque: Vec3,
    pad1: _c.float,
}


SpatialVelocity :: struct {
    linear: Vec3,
    pad0: _c.float,
    angular: Vec3,
    pad1: _c.float,
}


ArticulationRootLinkData :: struct {
    transform: Transform,
    worldLinVel: Vec3,
    worldAngVel: Vec3,
    worldLinAccel: Vec3,
    worldAngAccel: Vec3,
}


ArticulationCache :: struct {
    externalForces: ^SpatialForce,
    denseJacobian: ^_c.float,
    massMatrix: ^_c.float,
    jointVelocity: ^_c.float,
    jointAcceleration: ^_c.float,
    jointPosition: ^_c.float,
    jointForce: ^_c.float,
    jointSolverForces: ^_c.float,
    linkVelocity: ^SpatialVelocity,
    linkAcceleration: ^SpatialVelocity,
    rootLinkData: ^ArticulationRootLinkData,
    sensorForces: ^SpatialForce,
    coefficientMatrix: ^_c.float,
    lambda: ^_c.float,
    scratchMemory: rawptr,
    scratchAllocator: rawptr,
    version: _c.uint32_t,
    _pad17: [4]u8,
}


ArticulationSensor :: struct {
    using _: Base,
    userData: rawptr,
}


ArticulationReducedCoordinate :: struct {
    using _: Base,
    userData: rawptr,
}


ArticulationJointReducedCoordinate :: struct {
    using _: Base,
    userData: rawptr,
}


Shape :: struct {
    using _: RefCounted,
    userData: rawptr,
}


RigidActor :: struct {
    using _: Actor,
}


NodeIndex :: struct {
    _pad0: [8]u8,
}


RigidBody :: struct {
    using _: RigidActor,
}


ArticulationLink :: struct {
    using _: RigidBody,
}


ConeLimitedConstraint :: struct {
    mAxis: Vec3,
    mAngle: _c.float,
    mLowLimit: _c.float,
    mHighLimit: _c.float,
}


ConeLimitParams :: struct {
    lowHighLimits: Vec4,
    axisAngle: Vec4,
}


ConstraintShaderTable :: struct {
    solverPrep: rawptr,
    _pad1: [8]u8,
    visualize: rawptr,
    flag: ConstraintFlag,
    _pad4: [4]u8,
}


Constraint :: struct {
    using _: Base,
    userData: rawptr,
}


MassModificationProps :: struct {
    mInvMassScale0: _c.float,
    mInvInertiaScale0: _c.float,
    mInvMassScale1: _c.float,
    mInvInertiaScale1: _c.float,
}


ContactPatch :: struct {
    mMassModification: MassModificationProps,
    normal: Vec3,
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


Contact :: struct {
    contact: Vec3,
    separation: _c.float,
}


ExtendedContact :: struct {
    using _: Contact,
    targetVelocity: Vec3,
    maxImpulse: _c.float,
}


ModifiableContact :: struct {
    using _: ExtendedContact,
    normal: Vec3,
    restitution: _c.float,
    materialFlags: _c.uint32_t,
    materialIndex0: _c.uint16_t,
    materialIndex1: _c.uint16_t,
    staticFriction: _c.float,
    dynamicFriction: _c.float,
}


ContactStreamIterator :: struct {
    zero: Vec3,
    _pad1: [4]u8,
    patch: ^ContactPatch,
    contact: ^Contact,
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


GpuContactPair :: struct {
    contactPatches: ^_c.uint8_t,
    contactPoints: ^_c.uint8_t,
    contactForces: ^_c.float,
    transformCacheRef0: _c.uint32_t,
    transformCacheRef1: _c.uint32_t,
    nodeIndex0: NodeIndex,
    nodeIndex1: NodeIndex,
    actor0: ^Actor,
    actor1: ^Actor,
    nbContacts: _c.uint16_t,
    nbPatches: _c.uint16_t,
    _pad11: [4]u8,
}


ContactSet :: struct {
    _pad0: [16]u8,
}


ContactModifyPair :: struct {
    actor: [2]^RigidActor,
    shape: [2]^Shape,
    transform: [2]Transform,
    contacts: ContactSet,
}


ContactModifyCallback :: struct {
    _pad0: [8]u8,
}


CCDContactModifyCallback :: struct {
    _pad0: [8]u8,
}


DeletionListener :: struct {
    _pad0: [8]u8,
}


BaseMaterial :: struct {
    using _: RefCounted,
    userData: rawptr,
}


FEMMaterial :: struct {
    using _: BaseMaterial,
}


FilterData :: struct {
    word0: _c.uint32_t,
    word1: _c.uint32_t,
    word2: _c.uint32_t,
    word3: _c.uint32_t,
}


SimulationFilterCallback :: struct {
    _pad0: [8]u8,
}


ParticleRigidFilterPair :: struct {
    mID0: _c.uint64_t,
    mID1: _c.uint64_t,
}


LockedData :: struct {
    _pad0: [8]u8,
}


Material :: struct {
    using _: BaseMaterial,
}


GpuParticleBufferIndexPair :: struct {
    systemIndex: _c.uint32_t,
    bufferIndex: _c.uint32_t,
}


CudaContextManager :: distinct rawptr 

ParticleRigidAttachment :: distinct rawptr 

ParticleVolume :: struct {
    bound: Bounds3,
    particleIndicesOffset: _c.uint32_t,
    numParticles: _c.uint32_t,
}


DiffuseParticleParams :: struct {
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


ParticleSpring :: struct {
    ind0: _c.uint32_t,
    ind1: _c.uint32_t,
    length: _c.float,
    stiffness: _c.float,
    damping: _c.float,
    pad: _c.float,
}


ParticleMaterial :: struct {
    using _: BaseMaterial,
}


OmniPvd :: distinct rawptr 

Physics :: struct {
    _pad0: [8]u8,
}


ActorShape :: struct {
    actor: ^RigidActor,
    shape: ^Shape,
}


RaycastHit :: struct {
    using _: GeomRaycastHit,
    using _: ActorShape,
}


OverlapHit :: struct {
    using _: GeomOverlapHit,
    using _: ActorShape,
}


SweepHit :: struct {
    using _: GeomSweepHit,
    using _: ActorShape,
}


RaycastCallback :: struct {
    _pad0: [8]u8,
    block: RaycastHit,
    hasBlock: _c.bool,
    _pad3: [7]u8,
    touches: ^RaycastHit,
    maxNbTouches: _c.uint32_t,
    nbTouches: _c.uint32_t,
}


OverlapCallback :: struct {
    _pad0: [8]u8,
    block: OverlapHit,
    hasBlock: _c.bool,
    _pad3: [7]u8,
    touches: ^OverlapHit,
    maxNbTouches: _c.uint32_t,
    nbTouches: _c.uint32_t,
}


SweepCallback :: struct {
    _pad0: [8]u8,
    block: SweepHit,
    hasBlock: _c.bool,
    _pad3: [7]u8,
    touches: ^SweepHit,
    maxNbTouches: _c.uint32_t,
    nbTouches: _c.uint32_t,
}


RaycastBuffer :: struct {
    _pad0: [8]u8,
    block: RaycastHit,
    hasBlock: _c.bool,
    _pad3: [7]u8,
    touches: ^RaycastHit,
    maxNbTouches: _c.uint32_t,
    nbTouches: _c.uint32_t,
}


OverlapBuffer :: struct {
    _pad0: [8]u8,
    block: OverlapHit,
    hasBlock: _c.bool,
    _pad3: [7]u8,
    touches: ^OverlapHit,
    maxNbTouches: _c.uint32_t,
    nbTouches: _c.uint32_t,
}


SweepBuffer :: struct {
    _pad0: [8]u8,
    block: SweepHit,
    hasBlock: _c.bool,
    _pad3: [7]u8,
    touches: ^SweepHit,
    maxNbTouches: _c.uint32_t,
    nbTouches: _c.uint32_t,
}


QueryCache :: struct {
    shape: ^Shape,
    actor: ^RigidActor,
    faceIndex: _c.uint32_t,
    _pad3: [4]u8,
}


QueryFilterData :: struct {
    data: FilterData,
    flags: QueryFlags_Set,
    _pad2: [2]u8,
}


QueryFilterCallback :: struct {
    _pad0: [8]u8,
}


RigidDynamic :: struct {
    using _: RigidBody,
}


RigidStatic :: struct {
    using _: RigidActor,
}


SceneQueryDesc :: struct {
    staticStructure: PruningStructureType,
    dynamicStructure: PruningStructureType,
    dynamicTreeRebuildRateHint: _c.uint32_t,
    dynamicTreeSecondaryPruner: DynamicTreeSecondaryPruner,
    staticBVHBuildStrategy: BVHBuildStrategy,
    dynamicBVHBuildStrategy: BVHBuildStrategy,
    staticNbObjectsPerNode: _c.uint32_t,
    dynamicNbObjectsPerNode: _c.uint32_t,
    sceneQueryUpdateMode: SceneQueryUpdateMode,
}


SceneQuerySystemBase :: struct {
    _pad0: [8]u8,
}


SceneSQSystem :: struct {
    using _: SceneQuerySystemBase,
}


SceneQuerySystem :: struct {
    using _: SceneQuerySystemBase,
}


BroadPhaseRegion :: struct {
    mBounds: Bounds3,
    mUserData: rawptr,
}


BroadPhaseRegionInfo :: struct {
    mRegion: BroadPhaseRegion,
    mNbStaticObjects: _c.uint32_t,
    mNbDynamicObjects: _c.uint32_t,
    mActive: _c.bool,
    mOverlap: _c.bool,
    _pad5: [6]u8,
}


BroadPhaseCaps :: struct {
    mMaxNbRegions: _c.uint32_t,
}


BroadPhaseDesc :: struct {
    mType: BroadPhaseType,
    _pad1: [4]u8,
    mContextID: _c.uint64_t,
    _pad3: [8]u8,
    mFoundLostPairsCapacity: _c.uint32_t,
    mDiscardStaticVsKinematic: _c.bool,
    mDiscardKinematicVsKinematic: _c.bool,
    _pad7: [2]u8,
}


BroadPhaseUpdateData :: struct {
    mCreated: ^_c.uint32_t,
    mNbCreated: _c.uint32_t,
    _pad2: [4]u8,
    mUpdated: ^_c.uint32_t,
    mNbUpdated: _c.uint32_t,
    _pad5: [4]u8,
    mRemoved: ^_c.uint32_t,
    mNbRemoved: _c.uint32_t,
    _pad8: [4]u8,
    mBounds: ^Bounds3,
    mGroups: ^_c.uint32_t,
    mDistances: ^_c.float,
    mCapacity: _c.uint32_t,
    _pad13: [4]u8,
}


BroadPhasePair :: struct {
    mID0: _c.uint32_t,
    mID1: _c.uint32_t,
}


BroadPhaseResults :: struct {
    mNbCreatedPairs: _c.uint32_t,
    _pad1: [4]u8,
    mCreatedPairs: ^BroadPhasePair,
    mNbDeletedPairs: _c.uint32_t,
    _pad4: [4]u8,
    mDeletedPairs: ^BroadPhasePair,
}


BroadPhaseRegions :: struct {
    _pad0: [8]u8,
}


BroadPhase :: struct {
    _pad0: [8]u8,
}


AABBManager :: struct {
    _pad0: [8]u8,
}


SceneLimits :: struct {
    maxNbActors: _c.uint32_t,
    maxNbBodies: _c.uint32_t,
    maxNbStaticShapes: _c.uint32_t,
    maxNbDynamicShapes: _c.uint32_t,
    maxNbAggregates: _c.uint32_t,
    maxNbConstraints: _c.uint32_t,
    maxNbRegions: _c.uint32_t,
    maxNbBroadPhaseOverlaps: _c.uint32_t,
}


gDynamicsMemoryConfig :: struct {
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


SceneDesc :: struct {
    using _: SceneQueryDesc,
    gravity: Vec3,
    simulationEventCallback: ^SimulationEventCallback,
    contactModifyCallback: ^ContactModifyCallback,
    ccdContactModifyCallback: ^CCDContactModifyCallback,
    filterShaderData: rawptr,
    filterShaderDataSize: _c.uint32_t,
    _pad15: [4]u8,
    filterShader: rawptr,
    filterCallback: ^SimulationFilterCallback,
    kineKineFilteringMode: PairFilteringMode,
    staticKineFilteringMode: PairFilteringMode,
    broadPhaseType: BroadPhaseType,
    _pad21: [4]u8,
    broadPhaseCallback: ^BroadPhaseCallback,
    limits: SceneLimits,
    frictionType: FrictionType,
    solverType: SolverType,
    bounceThresholdVelocity: _c.float,
    frictionOffsetThreshold: _c.float,
    frictionCorrelationDistance: _c.float,
    flags: SceneFlags_Set,
    cpuDispatcher: ^CpuDispatcher,
    _pad31: [8]u8,
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
    sanityBounds: Bounds3,
    gpuDynamicsConfig: gDynamicsMemoryConfig,
    gpuMaxNumPartitions: _c.uint32_t,
    gpuMaxNumStaticPartitions: _c.uint32_t,
    gpuComputeVersion: _c.uint32_t,
    contactPairSlabSize: _c.uint32_t,
    sceneQuerySystem: ^SceneQuerySystem,
    _pad50: [8]u8,
}


SimulationStatistics :: struct {
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


GpuBodyData :: struct {
    quat: Quat,
    pos: Vec4,
    linVel: Vec4,
    angVel: Vec4,
}


GpuActorPair :: struct {
    srcIndex: _c.uint32_t,
    _pad1: [4]u8,
    nodeIndex: NodeIndex,
}


IndexDataPair :: struct {
    index: _c.uint32_t,
    _pad1: [4]u8,
    data: rawptr,
}


PvdSceneClient :: struct {
    _pad0: [8]u8,
}


DominanceGroupPair :: struct {
    dominance0: _c.uint8_t,
    dominance1: _c.uint8_t,
}


BroadPhaseCallback :: struct {
    _pad0: [8]u8,
}


Scene :: struct {
    using _: SceneSQSystem,
    userData: rawptr,
}


SceneReadLock :: struct {
    _pad0: [8]u8,
}


SceneWriteLock :: struct {
    _pad0: [8]u8,
}


ContactPairExtraDataItem :: struct {
    type: _c.uint8_t,
}


ContactPairVelocity :: struct {
    using _: ContactPairExtraDataItem,
    _pad1: [2]u8,
    linearVelocity: [2]Vec3,
    angularVelocity: [2]Vec3,
}


ContactPairPose :: struct {
    using _: ContactPairExtraDataItem,
    _pad1: [2]u8,
    globalPose: [2]Transform,
}


ContactPairIndex :: struct {
    using _: ContactPairExtraDataItem,
    index: _c.uint16_t,
}


ContactPairExtraDataIterator :: struct {
    currPtr: ^_c.uint8_t,
    endPtr: ^_c.uint8_t,
    preSolverVelocity: ^ContactPairVelocity,
    postSolverVelocity: ^ContactPairVelocity,
    eventPose: ^ContactPairPose,
    contactPairIndex: _c.uint32_t,
    _pad6: [4]u8,
}


ContactPairHeader :: struct {
    actors: [2]^Actor,
    extraDataStream: ^_c.uint8_t,
    extraDataStreamSize: _c.uint16_t,
    flags: ContactPairHeaderFlags_Set,
    _pad4: [4]u8,
    pairs: ^ContactPair,
    nbPairs: _c.uint32_t,
    _pad7: [4]u8,
}


ContactPairPoint :: struct {
    position: Vec3,
    separation: _c.float,
    normal: Vec3,
    internalFaceIndex0: _c.uint32_t,
    impulse: Vec3,
    internalFaceIndex1: _c.uint32_t,
}


ContactPair :: struct {
    shapes: [2]^Shape,
    contactPatches: ^_c.uint8_t,
    contactPoints: ^_c.uint8_t,
    contactImpulses: ^_c.float,
    requiredBufferSize: _c.uint32_t,
    contactCount: _c.uint8_t,
    patchCount: _c.uint8_t,
    contactStreamSize: _c.uint16_t,
    flags: ContactPairFlags_Set,
    events: PairFlags_Set,
    internalData: [2]_c.uint32_t,
    _pad11: [4]u8,
}


TriggerPair :: struct {
    triggerShape: ^Shape,
    triggerActor: ^Actor,
    otherShape: ^Shape,
    otherActor: ^Actor,
    status: PairFlag,
    flags: TriggerPairFlags_Set,
    _pad6: [3]u8,
}


ConstraintInfo :: struct {
    constraint: ^Constraint,
    externalReference: rawptr,
    type: _c.uint32_t,
    _pad3: [4]u8,
}


SimulationEventCallback :: struct {
    _pad0: [8]u8,
}


FEMParameters :: struct {
    velocityDamping: _c.float,
    settlingThreshold: _c.float,
    sleepThreshold: _c.float,
    sleepDamping: _c.float,
    selfCollisionFilterDistance: _c.float,
    selfCollisionStressTolerance: _c.float,
}


PruningStructure :: struct {
    using _: Base,
}


ExtendedVec3 :: struct {
    x: _c.double,
    y: _c.double,
    z: _c.double,
}


Obstacle :: struct {
    _pad0: [8]u8,
    mUserData: rawptr,
    mPos: ExtendedVec3,
    mRot: Quat,
}


BoxObstacle :: struct {
    using _: Obstacle,
    mHalfExtents: Vec3,
    _pad5: [4]u8,
}


CapsuleObstacle :: struct {
    using _: Obstacle,
    mHalfHeight: _c.float,
    mRadius: _c.float,
}


ObstacleContext :: struct {
    _pad0: [8]u8,
}


ControllerState :: struct {
    deltaXP: Vec3,
    _pad1: [4]u8,
    touchedShape: ^Shape,
    touchedActor: ^RigidActor,
    touchedObstacleHandle: _c.uint32_t,
    collisionFlags: _c.uint32_t,
    standOnAnotherCCT: _c.bool,
    standOnObstacle: _c.bool,
    isMovingUp: _c.bool,
    _pad9: [5]u8,
}


ControllerStats :: struct {
    nbIterations: _c.uint16_t,
    nbFullUpdates: _c.uint16_t,
    nbPartialUpdates: _c.uint16_t,
    nbTessellation: _c.uint16_t,
}


ControllerHit :: struct {
    controller: ^Controller,
    worldPos: ExtendedVec3,
    worldNormal: Vec3,
    dir: Vec3,
    length: _c.float,
    _pad5: [4]u8,
}


ControllerShapeHit :: struct {
    using _: ControllerHit,
    shape: ^Shape,
    actor: ^RigidActor,
    triangleIndex: _c.uint32_t,
    _pad10: [4]u8,
}


ControllersHit :: struct {
    using _: ControllerHit,
    other: ^Controller,
}


ControllerObstacleHit :: struct {
    using _: ControllerHit,
    userData: rawptr,
}


UserControllerHitReport :: struct {
    _pad0: [8]u8,
}


ControllerFilterCallback :: struct {
    _pad0: [8]u8,
}


ControllerFilters :: struct {
    mFilterData: ^FilterData,
    mFilterCallback: ^QueryFilterCallback,
    mFilterFlags: QueryFlags_Set,
    _pad3: [6]u8,
    mCCTFilterCallback: ^ControllerFilterCallback,
}


ControllerDesc :: struct #packed {
    _pad0: [8]u8,
    position: ExtendedVec3,
    upDirection: Vec3,
    slopeLimit: _c.float,
    invisibleWallHeight: _c.float,
    maxJumpHeight: _c.float,
    contactOffset: _c.float,
    stepOffset: _c.float,
    density: _c.float,
    scaleCoeff: _c.float,
    volumeGrowth: _c.float,
    _pad11: [4]u8,
    reportCallback: ^UserControllerHitReport,
    behaviorCallback: ^ControllerBehaviorCallback,
    nonWalkableMode: ControllerNonWalkableMode,
    _pad15: [4]u8,
    material: ^Material,
    registerDeletionListener: _c.bool,
    clientID: _c.uint8_t,
    _pad19: [6]u8,
    userData: rawptr,
    _pad21: [4]u8,
}


Controller :: struct {
    _pad0: [8]u8,
}


BoxControllerDesc :: struct {
    using _: ControllerDesc,
    halfHeight: _c.float,
    halfSideExtent: _c.float,
    halfForwardExtent: _c.float,
}


BoxController :: struct {
    using _: Controller,
}


CapsuleControllerDesc :: struct {
    using _: ControllerDesc,
    radius: _c.float,
    height: _c.float,
    climbingMode: CapsuleClimbingMode,
}


CapsuleController :: struct {
    using _: Controller,
}


ControllerBehaviorCallback :: struct {
    _pad0: [8]u8,
}


ControllerManager :: struct {
    _pad0: [8]u8,
}


Dim3 :: struct {
    x: _c.uint32_t,
    y: _c.uint32_t,
    z: _c.uint32_t,
}


SDFDesc :: struct {
    sdf: BoundedData,
    dims: Dim3,
    meshLower: Vec3,
    spacing: _c.float,
    subgridSize: _c.uint32_t,
    bitsPerSubgridPixel: SdfBitsPerSubgridPixel,
    sdfSubgrids3DTexBlockDim: Dim3,
    sdfSubgrids: BoundedData,
    sdfStartSlots: BoundedData,
    subgridsMinSdfValue: _c.float,
    subgridsMaxSdfValue: _c.float,
    sdfBounds: Bounds3,
    narrowBandThicknessRelativeToSdfBoundsDiagonal: _c.float,
    numThreadsForSdfConstruction: _c.uint32_t,
}


ConvexMeshDesc :: struct {
    points: BoundedData,
    polygons: BoundedData,
    indices: BoundedData,
    flags: ConvexFlags_Set,
    vertexLimit: _c.uint16_t,
    polygonLimit: _c.uint16_t,
    quantizedCount: _c.uint16_t,
    sdfDesc: ^SDFDesc,
}


TriangleMeshDesc :: struct {
    using _: SimpleTriangleMesh,
    materialIndices: [16]_c.char,
    sdfDesc: ^SDFDesc,
}


TetrahedronMeshDesc :: struct {
    materialIndices: [16]_c.char,
    points: BoundedData,
    tetrahedrons: BoundedData,
    flags: MeshFlags_Set,
    tetsPerElement: _c.uint16_t,
    _pad5: [4]u8,
}


SoftBodySimulationDataDesc :: struct {
    vertexToTet: BoundedData,
}


BVH34MidphaseDesc :: struct {
    numPrimsPerLeaf: _c.uint32_t,
    buildStrategy: BVH34BuildStrategy,
    quantized: _c.bool,
    _pad3: [3]u8,
}


MidphaseDesc :: struct {
    mBVH34Desc: [12]_c.char,
    _pad1: [4]u8,
}


BVHDesc :: struct {
    bounds: BoundedData,
    enlargement: _c.float,
    numPrimsPerLeaf: _c.uint32_t,
    buildStrategy: BVHBuildStrategy,
    _pad4: [4]u8,
}


CookingParams :: struct {
    areaTestEpsilon: _c.float,
    planeTolerance: _c.float,
    convexMeshCookingType: ConvexMeshCookingType,
    suppressTriangleMeshRemapTable: _c.bool,
    buildTriangleAdjacencies: _c.bool,
    buildGPUData: _c.bool,
    _pad6: [1]u8,
    scale: TolerancesScale,
    meshPreprocessParams: MeshPreprocessingFlags_Set,
    meshWeldTolerance: _c.float,
    midphaseDesc: MidphaseDesc,
    gaussMapLimit: _c.uint32_t,
    maxWeightRatioInTet: _c.float,
}


DefaultMemoryOutputStream :: struct {
    using _: OutputStream,
    _pad1: [24]u8,
}


DefaultMemoryInputData :: struct {
    using _: InputData,
    _pad1: [24]u8,
}


DefaultFileOutputStream :: struct {
    using _: OutputStream,
    _pad1: [8]u8,
}


DefaultFileInputData :: struct {
    using _: InputData,
    _pad1: [16]u8,
}


DefaultAllocator :: struct {
    using _: AllocatorCallback,
}


Joint :: struct {
    using _: Base,
    userData: rawptr,
}


Spring :: struct {
    stiffness: _c.float,
    damping: _c.float,
}


DistanceJoint :: struct {
    using _: Joint,
}


JacobianRow :: struct {
    linear0: Vec3,
    linear1: Vec3,
    angular0: Vec3,
    angular1: Vec3,
}


ContactJoint :: struct {
    using _: Joint,
}


FixedJoint :: struct {
    using _: Joint,
}


JointLimitParameters :: struct {
    restitution: _c.float,
    bounceThreshold: _c.float,
    stiffness: _c.float,
    damping: _c.float,
    contactDistance_deprecated: _c.float,
}


JointLinearLimit :: struct {
    using _: JointLimitParameters,
    value: _c.float,
}


JointLinearLimitPair :: struct {
    using _: JointLimitParameters,
    upper: _c.float,
    lower: _c.float,
}


JointAngularLimitPair :: struct {
    using _: JointLimitParameters,
    upper: _c.float,
    lower: _c.float,
}


JointLimitCone :: struct {
    using _: JointLimitParameters,
    yAngle: _c.float,
    zAngle: _c.float,
}


JointLimitPyramid :: struct {
    using _: JointLimitParameters,
    yAngleMin: _c.float,
    yAngleMax: _c.float,
    zAngleMin: _c.float,
    zAngleMax: _c.float,
}


PrismaticJoint :: struct {
    using _: Joint,
}


RevoluteJoint :: struct {
    using _: Joint,
}


SphericalJoint :: struct {
    using _: Joint,
}


D6JointDrive :: struct {
    using _: Spring,
    forceLimit: _c.float,
    flags: D6JointDriveFlags_Set,
}


D6Joint :: struct {
    using _: Joint,
}


GearJoint :: struct {
    using _: Joint,
}


RackAndPinionJoint :: struct {
    using _: Joint,
}


GroupsMask :: struct {
    bits0: _c.uint16_t,
    bits1: _c.uint16_t,
    bits2: _c.uint16_t,
    bits3: _c.uint16_t,
}


DefaultErrorCallback :: struct {
    using _: ErrorCallback,
}


RigidActorExt :: struct {
};

MassProperties :: struct {
    inertiaTensor: Mat33,
    centerOfMass: Vec3,
    mass: _c.float,
}


RigidBodyExt :: struct {
};

ShapeExt :: struct {
};

MeshOverlapUtil :: struct {
    _pad0: [1040]u8,
}


BinaryConverter :: distinct rawptr 

XmlMiscParameter :: struct {
    upVector: Vec3,
    scale: TolerancesScale,
}


Serialization :: struct {
};

DefaultCpuDispatcher :: struct {
    using _: CpuDispatcher,
}


StringTableExt :: struct {
};

BroadPhaseExt :: struct {
};

SceneQueryExt :: struct {
};

BatchQueryExt :: struct {
    _pad0: [8]u8,
}


CustomSceneQuerySystem :: struct {
    using _: SceneQuerySystem,
}


CustomSceneQuerySystemAdapter :: struct {
    _pad0: [8]u8,
}


SamplingExt :: struct {
};

PoissonSampler :: struct {
    using _: UserAllocated,
    _pad0: [8]u8,
}


TriangleMeshPoissonSampler :: struct {
    using _: PoissonSampler,
}


TetrahedronMeshExt :: struct {
};

RepXObject :: struct {
    typeName: ^_c.char,
    serializable: rawptr,
    id: _c.uint64_t,
}


Cooking :: distinct rawptr 

RepXInstantiationArgs :: struct {
    _pad0: [8]u8,
    cooker: ^Cooking,
    stringTable: ^StringTable,
}


XmlMemoryAllocator :: distinct rawptr 

XmlWriter :: distinct rawptr 

XmlReader :: distinct rawptr 

MemoryBuffer :: distinct rawptr 

RepXSerializer :: struct {
    _pad0: [8]u8,
}


VehicleWheels4SimData :: distinct rawptr 

VehicleWheels4DynData :: distinct rawptr 

VehicleTireForceCalculator :: distinct rawptr 

VehicleDrivableSurfaceToTireFrictionPairs :: distinct rawptr 

VehicleTelemetryData :: distinct rawptr 

Pvd :: struct {
    using _: ProfilerCallback,
}


PvdTransport :: struct {
    _pad0: [8]u8,
}

