package physx
import _c "core:c"
/// enum for empty constructor tag
PxEMPTY :: enum {
    PxEmpty = 0,
}

/// enum for zero constructor tag for vectors and matrices
PxZERO :: enum {
    PxZero = 0,
}

/// enum for identity constructor flag for quaternions, transforms, and matrices
PxIDENTITY :: enum {
    PxIdentity = 0,
}

/// Error codes
///
/// These error codes are passed to [`PxErrorCallback`]
PxErrorCode :: enum {
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

PxThreadPriority :: enum {
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
PxDebugColor :: enum {
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
PxConcreteType :: enum {
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
PxBaseFlag :: enum {
    OwnsMemory = 0,
    IsReleasable = 1,
}

/// Flags for [`PxBaseFlag`]
PxBaseFlags_Set :: bit_set[PxBaseFlag; _c.uint16_t]


/// Flags used to configure binary meta data entries, typically set through PX_DEF_BIN_METADATA defines.
PxMetaDataFlag :: enum {
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
PxTaskType :: enum {
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
PxGeometryType :: enum {
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
PxGeometryQueryFlag :: enum {
    SimdGuard = 0,
}

/// Flags for [`PxGeometryQueryFlag`]
PxGeometryQueryFlags_Set :: bit_set[PxGeometryQueryFlag; _c.uint32_t]


/// Desired build strategy for bounding-volume hierarchies
PxBVHBuildStrategy :: enum {
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
PxConvexMeshGeometryFlag :: enum {
    TightBounds = 0,
}

/// Flags for [`PxConvexMeshGeometryFlag`]
PxConvexMeshGeometryFlags_Set :: bit_set[PxConvexMeshGeometryFlag; _c.uint8_t]


/// Flags controlling the simulated behavior of the triangle mesh geometry.
///
/// Used in ::PxMeshGeometryFlags.
PxMeshGeometryFlag :: enum {
    TightBounds = 0,
    DoubleSided = 1,
}

/// Flags for [`PxMeshGeometryFlag`]
PxMeshGeometryFlags_Set :: bit_set[PxMeshGeometryFlag; _c.uint8_t]


/// Identifies the solver to use for a particle system.
PxParticleSolverType :: enum {
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
PxHitFlag :: enum {
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
PxHeightFieldFormat :: enum {
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
PxHeightFieldTessFlag :: enum {
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
PxHeightFieldFlag :: enum {
    NoBoundaryEdges = 0,
}

/// Flags for [`PxHeightFieldFlag`]
PxHeightFieldFlags_Set :: bit_set[PxHeightFieldFlag; _c.uint16_t]


/// Special material index values for height field samples.
PxHeightFieldMaterial :: enum {
    /// A material indicating that the triangle should be treated as a hole in the mesh.
    Hole = 127,
}

PxMeshMeshQueryFlag :: enum {
    DiscardCoplanar = 0,
}
PxMeshMeshQueryFlag_Default :: 0

/// Flags for [`PxMeshMeshQueryFlag`]
PxMeshMeshQueryFlags_Set :: bit_set[PxMeshMeshQueryFlag; _c.uint32_t]


/// Enum with flag values to be used in PxSimpleTriangleMesh::flags.
PxMeshFlag :: enum {
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
PxMeshMidPhase :: enum {
    /// Default midphase mesh structure, as used up to PhysX 3.3 (deprecated)
    Bvh33 = 0,
    /// New midphase mesh structure, introduced in PhysX 3.4
    Bvh34 = 1,
    Last = 2,
}

/// Flags for the mesh geometry properties.
///
/// Used in ::PxTriangleMeshFlags.
PxTriangleMeshFlag :: enum {
    E16BitIndices = 1,
    AdjacencyInfo = 2,
    PreferNoSdfProj = 3,
}

/// Flags for [`PxTriangleMeshFlag`]
PxTriangleMeshFlags_Set :: bit_set[PxTriangleMeshFlag; _c.uint8_t]


PxTetrahedronMeshFlag :: enum {
    E16BitIndices = 1,
}

/// Flags for [`PxTetrahedronMeshFlag`]
PxTetrahedronMeshFlags_Set :: bit_set[PxTetrahedronMeshFlag; _c.uint8_t]


/// Flags which control the behavior of an actor.
PxActorFlag :: enum {
    Visualization = 0,
    DisableGravity = 1,
    SendSleepNotifies = 2,
    DisableSimulation = 3,
}

/// Flags for [`PxActorFlag`]
PxActorFlags_Set :: bit_set[PxActorFlag; _c.uint8_t]


/// Identifies each type of actor.
PxActorType :: enum {
    /// A static rigid body
    RigidStatic = 0,
    /// A dynamic rigid body
    RigidDynamic = 1,
    /// An articulation link
    ArticulationLink = 2,
}

PxAggregateType :: enum {
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
Px1DConstraintFlag :: enum {
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
PxConstraintSolveHint :: enum {
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
PxConstraintVisualizationFlag :: enum {
    /// visualize constraint frames
    LocalFrames = 1,
    /// visualize constraint limits
    Limits = 2,
}

/// Flags for determining how PVD should serialize a constraint update
PxPvdUpdateType :: enum {
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
ConstraintType :: enum {
    /// Defines this pair is a contact constraint
    ContactConstraint = 0,
    /// Defines this pair is a joint constraint
    JointConstraint = 1,
}

/// Data structure used for preparing constraints before solving them
BodyState :: enum {
    DynamicBody = 1,
    StaticBody = 2,
    KinematicBody = 4,
    Articulation = 8,
}

/// @
/// {
PxArticulationAxis :: enum {
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

PxArticulationMotion :: enum {
    Limited = 0,
    Free = 1,
}
PxArticulationMotion_Locked :: 0

/// Flags for [`PxArticulationMotion`]
PxArticulationMotions_Set :: bit_set[PxArticulationMotion; _c.uint8_t]


PxArticulationJointType :: enum {
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

PxArticulationFlag :: enum {
    FixBase = 0,
    DriveLimitsAreForces = 1,
    DisableSelfCollision = 2,
    ComputeJointForces = 3,
}

/// Flags for [`PxArticulationFlag`]
PxArticulationFlags_Set :: bit_set[PxArticulationFlag; _c.uint8_t]


PxArticulationDriveType :: enum {
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
PxArticulationGpuDataType :: enum {
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
PxArticulationCacheFlag :: enum {
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
PxArticulationSensorFlag :: enum {
    ForwardDynamicsForces = 0,
    ConstraintSolverForces = 1,
    WorldFrame = 2,
}

/// Flags for [`PxArticulationSensorFlag`]
PxArticulationSensorFlags_Set :: bit_set[PxArticulationSensorFlag; _c.uint8_t]


/// Flag that configures articulation-state updates by PxArticulationReducedCoordinate::updateKinematic.
PxArticulationKinematicFlag :: enum {
    Position = 0,
    Velocity = 1,
}

/// Flags for [`PxArticulationKinematicFlag`]
PxArticulationKinematicFlags_Set :: bit_set[PxArticulationKinematicFlag; _c.uint8_t]


/// Flags which affect the behavior of PxShapes.
PxShapeFlag :: enum {
    SimulationShape = 0,
    SceneQueryShape = 1,
    TriggerShape = 2,
    Visualization = 3,
}

/// Flags for [`PxShapeFlag`]
PxShapeFlags_Set :: bit_set[PxShapeFlag; _c.uint8_t]


/// Parameter to addForce() and addTorque() calls, determines the exact operation that is carried out.
PxForceMode :: enum {
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
PxRigidBodyFlag :: enum {
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
PxConstraintFlag :: enum {
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
PxContactPatchFlags :: enum {
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
StreamFormat :: enum {
    SimpleStream = 0,
    ModifiableStream = 1,
    CompressedModifiableStream = 2,
}

/// Flags specifying deletion event types.
PxDeletionEventFlag :: enum {
    UserRelease = 0,
    MemoryRelease = 1,
}

/// Flags for [`PxDeletionEventFlag`]
PxDeletionEventFlags_Set :: bit_set[PxDeletionEventFlag; _c.uint8_t]


/// Collection of flags describing the actions to take for a collision pair.
PxPairFlag :: enum {
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
PxFilterFlag :: enum {
    Kill = 0,
    Suppress = 1,
    Callback = 2,
}
PxFilterFlag_Notify :: PxFilterFlag.Callback
PxFilterFlag_Default :: 0

/// Flags for [`PxFilterFlag`]
PxFilterFlags_Set :: bit_set[PxFilterFlag; _c.uint16_t]


/// Identifies each type of filter object.
PxFilterObjectType :: enum {
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

PxFilterObjectFlag :: enum {
    Kinematic = 16,
    Trigger = 32,
}

PxPairFilteringMode :: enum {
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

PxDataAccessFlag :: enum {
    Readable = 0,
    Writable = 1,
    Device = 2,
}

/// Flags for [`PxDataAccessFlag`]
PxDataAccessFlags_Set :: bit_set[PxDataAccessFlag; _c.uint8_t]


/// Flags which control the behavior of a material.
PxMaterialFlag :: enum {
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
PxCombineMode :: enum {
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
PxParticleBufferFlag :: enum {
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
PxParticlePhaseFlag :: enum {
    ParticlePhaseSelfCollide = 20,
    ParticlePhaseSelfCollideFilter = 21,
    ParticlePhaseFluid = 22,
}
PxParticlePhaseFlag_ParticlePhaseGroupMask :: 0x000fffff
PxParticlePhaseFlag_ParticlePhaseFlagsMask :: PxParticlePhaseFlag.ParticlePhaseSelfCollide | PxParticlePhaseFlag.ParticlePhaseSelfCollideFilter | PxParticlePhaseFlag.ParticlePhaseFluid

/// Flags for [`PxParticlePhaseFlag`]
PxParticlePhaseFlags_Set :: bit_set[PxParticlePhaseFlag; _c.uint32_t]


/// Specifies memory space for a PxBuffer instance.
PxBufferType :: enum {
    Host = 0,
    Device = 1,
}

/// Filtering flags for scene queries.
PxQueryFlag :: enum {
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
PxQueryHitType :: enum {
    /// the query should ignore this shape
    None = 0,
    /// a hit on the shape touches the intersection geometry of the query but does not block it
    Touch = 1,
    /// a hit on the shape blocks the query (does not block overlap queries)
    Block = 2,
}

/// Collection of flags providing a mechanism to lock motion along/around a specific axis.
PxRigidDynamicLockFlag :: enum {
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
PxPruningStructureType :: enum {
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
PxDynamicTreeSecondaryPruner :: enum {
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
PxSceneQueryUpdateMode :: enum {
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
PxScenePrunerIndex :: enum {
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
PxBroadPhaseType :: enum {
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
PxFrictionType :: enum {
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
PxSolverType :: enum {
    /// Projected Gauss-Seidel iterative solver
    Pgs = 0,
    /// Default Temporal Gauss-Seidel solver
    Tgs = 1,
}

/// flags for configuring properties of the scene
PxSceneFlag :: enum {
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
PxVisualizationParameter :: enum {
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
RbPairStatsType :: enum {
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
PxSoftBodyDataFlag :: enum {
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
PxHairSystemData :: enum {
    PositionInvmass = 0,
    Velocity = 1,
}
PxHairSystemData_None :: 0
PxHairSystemData_All :: PxHairSystemData.PositionInvmass | PxHairSystemData.Velocity

/// Flags for [`PxHairSystemData`]
PxHairSystemDataFlags_Set :: bit_set[PxHairSystemData; _c.uint32_t]


/// Binary settings for hair system simulation
PxHairSystemFlag :: enum {
    DisableSelfCollision = 0,
    DisableExternalCollision = 1,
    DisableTwosidedAttachment = 2,
}

/// Flags for [`PxHairSystemFlag`]
PxHairSystemFlags_Set :: bit_set[PxHairSystemFlag; _c.uint32_t]


/// Identifies each type of information for retrieving from actor.
PxActorCacheFlag :: enum {
    ActorData = 0,
    Force = 2,
    Torque = 3,
}

/// Flags for [`PxActorCacheFlag`]
PxActorCacheFlags_Set :: bit_set[PxActorCacheFlag; _c.uint16_t]


/// PVD scene Flags. They are disabled by default, and only works if PxPvdInstrumentationFlag::eDEBUG is set.
PxPvdSceneFlag :: enum {
    TransmitContacts = 0,
    TransmitScenequeries = 1,
    TransmitConstraints = 2,
}

/// Flags for [`PxPvdSceneFlag`]
PxPvdSceneFlags_Set :: bit_set[PxPvdSceneFlag; _c.uint8_t]


/// Identifies each type of actor for retrieving actors from a scene.
///
/// [`PxArticulationLink`] objects are not supported. Use the #PxArticulationReducedCoordinate object to retrieve all its links.
PxActorTypeFlag :: enum {
    RigidStatic = 0,
    RigidDynamic = 1,
}

/// Flags for [`PxActorTypeFlag`]
PxActorTypeFlags_Set :: bit_set[PxActorTypeFlag; _c.uint16_t]


/// Extra data item types for contact pairs.
PxContactPairExtraDataType :: enum {
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
PxContactPairHeaderFlag :: enum {
    RemovedActor0 = 0,
    RemovedActor1 = 1,
}

/// Flags for [`PxContactPairHeaderFlag`]
PxContactPairHeaderFlags_Set :: bit_set[PxContactPairHeaderFlag; _c.uint16_t]


/// Collection of flags providing information on contact report pairs.
PxContactPairFlag :: enum {
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
PxTriggerPairFlag :: enum {
    RemovedShapeTrigger = 0,
    RemovedShapeOther = 1,
    NextFree = 2,
}

/// Flags for [`PxTriggerPairFlag`]
PxTriggerPairFlags_Set :: bit_set[PxTriggerPairFlag; _c.uint8_t]


/// Identifies input and output buffers for PxSoftBody.
PxSoftBodyData :: enum {
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
PxSoftBodyFlag :: enum {
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
PxControllerShapeType :: enum {
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
PxControllerNonWalkableMode :: enum {
    /// Stops character from climbing up non-walkable slopes, but doesn't move it otherwise
    PreventClimbing = 0,
    /// Stops character from climbing up non-walkable slopes, and forces it to slide down those slopes
    PreventClimbingAndForceSliding = 1,
}

/// specifies which sides a character is colliding with.
PxControllerCollisionFlag :: enum {
    CollisionSides = 0,
    CollisionUp = 1,
    CollisionDown = 2,
}

/// Flags for [`PxControllerCollisionFlag`]
PxControllerCollisionFlags_Set :: bit_set[PxControllerCollisionFlag; _c.uint8_t]


PxCapsuleClimbingMode :: enum {
    /// Standard mode, let the capsule climb over surfaces according to impact normal
    Easy = 0,
    /// Constrained mode, try to limit climbing according to the step offset
    Constrained = 1,
    Last = 2,
}

/// specifies controller behavior
PxControllerBehaviorFlag :: enum {
    CctCanRideOnObject = 0,
    CctSlide = 1,
    CctUserDefinedRide = 2,
}

/// Flags for [`PxControllerBehaviorFlag`]
PxControllerBehaviorFlags_Set :: bit_set[PxControllerBehaviorFlag; _c.uint8_t]


/// specifies debug-rendering flags
PxControllerDebugRenderFlag :: enum {
    TemporalBv = 0,
    CachedBv = 1,
    Obstacles = 2,
}
PxControllerDebugRenderFlag_None :: 0
PxControllerDebugRenderFlag_All :: PxControllerDebugRenderFlag.TemporalBv | PxControllerDebugRenderFlag.CachedBv | PxControllerDebugRenderFlag.Obstacles

/// Flags for [`PxControllerDebugRenderFlag`]
PxControllerDebugRenderFlags_Set :: bit_set[PxControllerDebugRenderFlag; _c.uint32_t]


/// Defines the number of bits per subgrid pixel
PxSdfBitsPerSubgridPixel :: enum {
    /// 8 bit per subgrid pixel (values will be stored as normalized integers)
    E8BitPerPixel = 1,
    /// 16 bit per subgrid pixel (values will be stored as normalized integers)
    E16BitPerPixel = 2,
    /// 32 bit per subgrid pixel (values will be stored as floats in world scale units)
    E32BitPerPixel = 4,
}

/// Flags which describe the format and behavior of a convex mesh.
PxConvexFlag :: enum {
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
PxMeshFormat :: enum {
    /// Normal tetmesh with arbitrary tetrahedra
    TetMesh = 0,
    /// 6 tetrahedra in a row will form a hexahedron
    HexMesh = 1,
}

/// Desired build strategy for PxMeshMidPhase::eBVH34
PxBVH34BuildStrategy :: enum {
    /// Fast build strategy. Fast build speed, good runtime performance in most cases. Recommended for runtime mesh cooking.
    Fast = 0,
    /// Default build strategy. Medium build speed, good runtime performance in all cases.
    Default = 1,
    /// SAH build strategy. Slower builds, slightly improved runtime performance in some cases.
    Sah = 2,
    Last = 3,
}

/// Result from convex cooking.
PxConvexMeshCookingResult :: enum {
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
PxConvexMeshCookingType :: enum {
    /// The Quickhull algorithm constructs the hull from the given input points. The resulting hull
    /// will only contain a subset of the input points.
    Quickhull = 0,
}

/// Result from triangle mesh cooking
PxTriangleMeshCookingResult :: enum {
    /// Everything is A-OK.
    Success = 0,
    /// a triangle is too large for well-conditioned results. Tessellate the mesh for better behavior, see the user guide section on cooking for more details.
    LargeTriangle = 1,
    /// Something unrecoverable happened. Check the error stream to find out what.
    Failure = 2,
}

/// Enum for the set of mesh pre-processing parameters.
PxMeshPreprocessingFlag :: enum {
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
PxConstraintExtIDs :: enum {
    Joint = 0,
    VehicleSuspLimitDeprecated = 1,
    VehicleStickyTyreDeprecated = 2,
    VehicleJoint = 3,
    NextFreeId = 4,
    InvalidId = 2147483647,
}

/// an enumeration of PhysX' built-in joint types
PxJointConcreteType :: enum {
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
PxJointActorIndex :: enum {
    Actor0 = 0,
    Actor1 = 1,
    Count = 2,
}

/// flags for configuring the drive of a PxDistanceJoint
PxDistanceJointFlag :: enum {
    MaxDistanceEnabled = 1,
    MinDistanceEnabled = 2,
    SpringEnabled = 3,
}

/// Flags for [`PxDistanceJointFlag`]
PxDistanceJointFlags_Set :: bit_set[PxDistanceJointFlag; _c.uint16_t]


/// Flags specific to the prismatic joint.
PxPrismaticJointFlag :: enum {
    LimitEnabled = 1,
}

/// Flags for [`PxPrismaticJointFlag`]
PxPrismaticJointFlags_Set :: bit_set[PxPrismaticJointFlag; _c.uint16_t]


/// Flags specific to the Revolute Joint.
PxRevoluteJointFlag :: enum {
    LimitEnabled = 0,
    DriveEnabled = 1,
    DriveFreespin = 2,
}

/// Flags for [`PxRevoluteJointFlag`]
PxRevoluteJointFlags_Set :: bit_set[PxRevoluteJointFlag; _c.uint16_t]


/// Flags specific to the spherical joint.
PxSphericalJointFlag :: enum {
    LimitEnabled = 1,
}

/// Flags for [`PxSphericalJointFlag`]
PxSphericalJointFlags_Set :: bit_set[PxSphericalJointFlag; _c.uint16_t]


/// Used to specify one of the degrees of freedom of  a D6 joint.
PxD6Axis :: enum {
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
PxD6Motion :: enum {
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
PxD6Drive :: enum {
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
PxD6JointDriveFlag :: enum {
    Acceleration = 0,
}

/// Flags for [`PxD6JointDriveFlag`]
PxD6JointDriveFlags_Set :: bit_set[PxD6JointDriveFlag; _c.uint32_t]


/// Collision filtering operations.
PxFilterOp :: enum {
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
PxDefaultCpuDispatcherWaitForWorkMode :: enum {
    WaitForWork = 0,
    YieldThread = 1,
    YieldProcessor = 2,
}

PxBatchQueryStatus :: enum {
    /// This is the initial state before a query starts.
    Pending = 0,
    /// The query is finished; results have been written into the result and hit buffers.
    Success = 1,
    /// The query results were incomplete due to touch hit buffer overflow. Blocking hit is still correct.
    Overflow = 2,
}

/// types of instrumentation that PVD can do.
PxPvdInstrumentationFlag :: enum {
    Debug = 0,
    Profile = 1,
    Memory = 2,
}
PxPvdInstrumentationFlag_All :: PxPvdInstrumentationFlag.Debug | PxPvdInstrumentationFlag.Profile | PxPvdInstrumentationFlag.Memory

/// Flags for [`PxPvdInstrumentationFlag`]
PxPvdInstrumentationFlags_Set :: bit_set[PxPvdInstrumentationFlag; _c.uint8_t]


PxMat34 :: struct {
    _unused: [0]u8,
}

PxAllocatorCallback :: struct {
    vtable_: rawptr,
};

PxAssertHandler :: struct {
    vtable_: rawptr,
};

PxFoundation :: struct {
    vtable_: rawptr,
};

PxAllocator :: struct {
    _pad0: [1]u8,
    unused0: [1]u8,
}

PxRawAllocator :: struct {
    _pad0: [1]u8,
    unused0: [1]u8,
}

PxVirtualAllocatorCallback :: struct {
    vtable_: rawptr,
};

PxVirtualAllocator :: struct {
    _pad0: [16]u8,
    unused0: [1]u8,
}

PxUserAllocated :: struct {
    _pad0: [1]u8,
    unused0: [1]u8,
}

PxTempAllocatorChunk :: struct #raw_union {
    mNext: ^PxTempAllocatorChunk,
    mIndex: _c.uint32_t,
    mPad: [16]_c.uint8_t,
};

PxTempAllocator :: struct {
    _pad0: [1]u8,
    unused0: [1]u8,
}

PxLogTwo :: struct {
    _unused: [0]u8,
}

PxUnConst :: struct {
    _unused: [0]u8,
}

PxBitAndByte :: struct {
    _pad0: [1]u8,
    unused0: [1]u8,
}

PxBitMap :: struct {
    _pad0: [16]u8,
    unused0: [1]u8,
}

PxVec3 :: struct {
    x: _c.float,
    y: _c.float,
    z: _c.float,
}

PxVec3Padded :: struct {
    x: _c.float,
    y: _c.float,
    z: _c.float,
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
    vtable_: rawptr,
};

PxAllocationListener :: struct {
    vtable_: rawptr,
};

PxBroadcastingAllocator :: struct {
    _vtable: rawptr,
    _pad0: [176]u8,
}

PxBroadcastingErrorCallback :: struct {
    _vtable: rawptr,
    _pad0: [160]u8,
}

PxHash :: struct {
    _unused: [0]u8,
}

PxInputStream :: struct {
    vtable_: rawptr,
};

PxInputData :: struct {
    vtable_: rawptr,
};

PxOutputStream :: struct {
    vtable_: rawptr,
};

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
    _pad0: [1]u8,
    unused0: [1]u8,
}

PxMutexImpl :: struct {
    _pad0: [1]u8,
    unused0: [1]u8,
}

PxReadWriteLock :: struct {
    _pad0: [8]u8,
    unused0: [1]u8,
}

PxProfilerCallback :: struct {
    vtable_: rawptr,
};

PxProfileScoped :: struct {
    mCallback: ^PxProfilerCallback,
    mEventName: ^_c.char,
    mProfilerData: rawptr,
    mContextId: _c.uint64_t,
    mDetached: _c.bool,
    _pad0: [7]u8,
}

PxSListEntry :: struct {
    _pad0: [16]u8,
    unused0: [1]u8,
}

PxSListImpl :: struct {
    _pad0: [1]u8,
    unused0: [1]u8,
}

PxSyncImpl :: struct {
    _pad0: [1]u8,
    unused0: [1]u8,
}

PxRunnable :: struct {
    vtable_: rawptr,
};

PxCounterFrequencyToTensOfNanos :: struct {
    mNumerator: _c.uint64_t,
    mDenominator: _c.uint64_t,
}

PxTime :: struct {
    _pad0: [8]u8,
    unused0: [1]u8,
}

PxVec2 :: struct {
    x: _c.float,
    y: _c.float,
}

PxStridedData :: struct {
    stride: _c.uint32_t,
    _pad0: [4]u8,
    data: rawptr,
}

PxBoundedData :: struct {
    stride: _c.uint32_t,
    _pad0: [4]u8,
    data: rawptr,
    count: _c.uint32_t,
    _pad1: [4]u8,
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
    _pad0: [4]u8,
    string: ^_c.char,
}

PxRenderBuffer :: struct {
    vtable_: rawptr,
};

PxProcessPxBaseCallback :: struct {
    vtable_: rawptr,
};

PxSerializationContext :: struct {
    vtable_: rawptr,
};

PxDeserializationContext :: struct {
    _vtable: rawptr,
    _pad0: [16]u8,
}

PxSerializationRegistry :: struct {
    vtable_: rawptr,
};

PxCollection :: struct {
    vtable_: rawptr,
};

PxTypeInfo :: struct {
    _unused: [0]u8,
}

PxFEMSoftBodyMaterial :: struct {
    _unused: [0]u8,
}

PxFEMClothMaterial :: struct {
    _unused: [0]u8,
}

PxPBDMaterial :: struct {
    _unused: [0]u8,
}

PxFLIPMaterial :: struct {
    _unused: [0]u8,
}

PxMPMMaterial :: struct {
    _unused: [0]u8,
}

PxCustomMaterial :: struct {
    _unused: [0]u8,
}

PxBVH33TriangleMesh :: struct {
    _unused: [0]u8,
}

PxParticleSystem :: struct {
    _unused: [0]u8,
}

PxPBDParticleSystem :: struct {
    _unused: [0]u8,
}

PxFLIPParticleSystem :: struct {
    _unused: [0]u8,
}

PxMPMParticleSystem :: struct {
    _unused: [0]u8,
}

PxCustomParticleSystem :: struct {
    _unused: [0]u8,
}

PxSoftBody :: struct {
    _unused: [0]u8,
}

PxFEMCloth :: struct {
    _unused: [0]u8,
}

PxHairSystem :: struct {
    _unused: [0]u8,
}

PxParticleBuffer :: struct {
    _unused: [0]u8,
}

PxParticleAndDiffuseBuffer :: struct {
    _unused: [0]u8,
}

PxParticleClothBuffer :: struct {
    _unused: [0]u8,
}

PxParticleRigidBuffer :: struct {
    _unused: [0]u8,
}

PxBase :: struct {
    _vtable: rawptr,
    _pad0: [16]u8,
}

PxRefCounted :: struct {
    _vtable: rawptr,
    _pad0: [16]u8,
}

PxTolerancesScale :: struct {
    length: _c.float,
    speed: _c.float,
}

PxStringTable :: struct {
    vtable_: rawptr,
};

PxSerializer :: struct {
    vtable_: rawptr,
};

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
    vtable_: rawptr,
};

PxTaskManager :: struct {
    vtable_: rawptr,
};

PxCpuDispatcher :: struct {
    vtable_: rawptr,
};

PxBaseTask :: struct {
    _vtable: rawptr,
    _pad0: [24]u8,
}

PxTask :: struct {
    _vtable: rawptr,
    _pad0: [32]u8,
}

PxLightCpuTask :: struct {
    _vtable: rawptr,
    _pad0: [40]u8,
}

PxGeometry :: struct {
    _pad0: [4]u8,
    mTypePadding: _c.float,
}

PxBoxGeometry :: struct {
    _pad0: [4]u8,
    mTypePadding: _c.float,
    halfExtents: PxVec3,
}

PxBVHRaycastCallback :: struct {
    vtable_: rawptr,
};

PxBVHOverlapCallback :: struct {
    vtable_: rawptr,
};

PxBVHTraversalCallback :: struct {
    vtable_: rawptr,
};

PxBVH :: struct {
    _vtable: rawptr,
    _pad0: [16]u8,
}

PxCapsuleGeometry :: struct {
    _pad0: [4]u8,
    mTypePadding: _c.float,
    radius: _c.float,
    halfHeight: _c.float,
}

PxHullPolygon :: struct {
    mPlane: [4]_c.float,
    mNbVerts: _c.uint16_t,
    mIndexBase: _c.uint16_t,
}

PxConvexMesh :: struct {
    _vtable: rawptr,
    _pad0: [16]u8,
}

PxMeshScale :: struct {
    scale: PxVec3,
    rotation: PxQuat,
}

PxConvexMeshGeometry :: struct {
    _pad0: [4]u8,
    mTypePadding: _c.float,
    scale: PxMeshScale,
    _pad1: [4]u8,
    convexMesh: ^PxConvexMesh,
    meshFlags: _c.uint8_t,
    _pad2: [7]u8,
}

PxSphereGeometry :: struct {
    _pad0: [4]u8,
    mTypePadding: _c.float,
    radius: _c.float,
}

PxPlaneGeometry :: struct {
    _pad0: [4]u8,
    mTypePadding: _c.float,
}

PxTriangleMeshGeometry :: struct {
    _pad0: [4]u8,
    mTypePadding: _c.float,
    scale: PxMeshScale,
    meshFlags: _c.uint8_t,
    _pad1: [3]u8,
    triangleMesh: ^PxTriangleMesh,
}

PxHeightFieldGeometry :: struct {
    _pad0: [4]u8,
    mTypePadding: _c.float,
    heightField: ^PxHeightField,
    heightScale: _c.float,
    rowScale: _c.float,
    columnScale: _c.float,
    heightFieldFlags: _c.uint8_t,
    _pad1: [3]u8,
}

PxParticleSystemGeometry :: struct {
    _pad0: [4]u8,
    mTypePadding: _c.float,
    mSolverType: _c.int32_t,
}

PxHairSystemGeometry :: struct {
    _pad0: [4]u8,
    mTypePadding: _c.float,
}

PxTetrahedronMeshGeometry :: struct {
    _pad0: [4]u8,
    mTypePadding: _c.float,
    tetrahedronMesh: ^PxTetrahedronMesh,
}

PxQueryHit :: struct {
    faceIndex: _c.uint32_t,
}

PxLocationHit :: struct {
    faceIndex: _c.uint32_t,
    flags: _c.uint16_t,
    _pad0: [2]u8,
    position: PxVec3,
    normal: PxVec3,
    distance: _c.float,
}

PxGeomRaycastHit :: struct {
    faceIndex: _c.uint32_t,
    flags: _c.uint16_t,
    _pad0: [2]u8,
    position: PxVec3,
    normal: PxVec3,
    distance: _c.float,
    u: _c.float,
    v: _c.float,
}

PxGeomOverlapHit :: struct {
    faceIndex: _c.uint32_t,
}

PxGeomSweepHit :: struct {
    faceIndex: _c.uint32_t,
    flags: _c.uint16_t,
    _pad0: [2]u8,
    position: PxVec3,
    normal: PxVec3,
    distance: _c.float,
}

PxGeomIndexPair :: struct {
    id0: _c.uint32_t,
    id1: _c.uint32_t,
}

PxQueryThreadContext :: struct {
    _pad0: [1]u8,
    unused0: [1]u8,
}

PxContactBuffer :: struct {
    _unused: [0]u8,
}

PxRenderOutput :: struct {
    _unused: [0]u8,
}

PxCustomGeometryType :: struct {
    _pad0: [4]u8,
    unused0: [1]u8,
}

PxCustomGeometryCallbacks :: struct {
    vtable_: rawptr,
};

PxCustomGeometry :: struct {
    _pad0: [4]u8,
    mTypePadding: _c.float,
    callbacks: ^PxCustomGeometryCallbacks,
}

PxGeometryHolder :: struct {
    _pad0: [56]u8,
    unused0: [1]u8,
}

PxGeometryQuery :: struct {
    _pad0: [1]u8,
    unused0: [1]u8,
}

PxHeightFieldSample :: struct {
    height: _c.int16_t,
    materialIndex0: PxBitAndByte,
    materialIndex1: PxBitAndByte,
}

PxHeightField :: struct {
    _vtable: rawptr,
    _pad0: [16]u8,
}

PxHeightFieldDesc :: struct {
    nbRows: _c.uint32_t,
    nbColumns: _c.uint32_t,
    format: _c.int32_t,
    _pad0: [4]u8,
    samples: PxStridedData,
    convexEdgeThreshold: _c.float,
    flags: _c.uint16_t,
    _pad1: [2]u8,
}

PxMeshQuery :: struct {
    _pad0: [1]u8,
    unused0: [1]u8,
}

PxSimpleTriangleMesh :: struct {
    points: PxBoundedData,
    triangles: PxBoundedData,
    flags: _c.uint16_t,
    _pad0: [6]u8,
}

PxTriangle :: struct {
    verts: [3]PxVec3,
}

PxTrianglePadded :: struct {
    verts: [3]PxVec3,
    padding: _c.uint32_t,
}

PxTriangleMesh :: struct {
    _vtable: rawptr,
    _pad0: [16]u8,
}

PxBVH34TriangleMesh :: struct {
    _vtable: rawptr,
    _pad0: [16]u8,
}

PxTetrahedron :: struct {
    verts: [4]PxVec3,
}

PxSoftBodyAuxData :: struct {
    _vtable: rawptr,
    _pad0: [16]u8,
}

PxTetrahedronMesh :: struct {
    _vtable: rawptr,
    _pad0: [16]u8,
}

PxSoftBodyMesh :: struct {
    _vtable: rawptr,
    _pad0: [16]u8,
}

PxCollisionMeshMappingData :: struct {
    _vtable: rawptr,
    _pad0: [8]u8,
}

PxSoftBodyCollisionData :: struct {
    _pad0: [1]u8,
    unused0: [1]u8,
}

PxTetrahedronMeshData :: struct {
    _pad0: [1]u8,
    unused0: [1]u8,
}

PxSoftBodySimulationData :: struct {
    _pad0: [1]u8,
    unused0: [1]u8,
}

PxCollisionTetrahedronMeshData :: struct {
    _vtable: rawptr,
    _pad0: [8]u8,
}

PxSimulationTetrahedronMeshData :: struct {
    _vtable: rawptr,
    _pad0: [8]u8,
}

PxActor :: struct {
    _vtable: rawptr,
    _pad0: [16]u8,
    userData: rawptr,
}

PxAggregate :: struct {
    _vtable: rawptr,
    _pad0: [16]u8,
    userData: rawptr,
}

PxSpringModifiers :: struct {
    stiffness: _c.float,
    damping: _c.float,
    _pad0: [8]u8,
}

PxRestitutionModifiers :: struct {
    restitution: _c.float,
    velocityThreshold: _c.float,
    _pad0: [8]u8,
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
    _pad0: [8]u8,
}

PxConstraintInvMassScale :: struct {
    linear0: _c.float,
    angular0: _c.float,
    linear1: _c.float,
    angular1: _c.float,
}

PxConstraintVisualizer :: struct {
    vtable_: rawptr,
};

PxConstraintConnector :: struct {
    vtable_: rawptr,
};

PxContactPoint :: struct {
    normal: PxVec3,
    separation: _c.float,
    point: PxVec3,
    maxImpulse: _c.float,
    targetVel: PxVec3,
    staticFriction: _c.float,
    materialFlags: _c.uint8_t,
    _pad0: [3]u8,
    internalFaceIndex1: _c.uint32_t,
    dynamicFriction: _c.float,
    restitution: _c.float,
    damping: _c.float,
    _pad1: [12]u8,
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
    _pad0: [2]u8,
}

PxConstraintBatchHeader :: struct {
    startIndex: _c.uint32_t,
    stride: _c.uint16_t,
    constraintType: _c.uint16_t,
}

PxSolverConstraintDesc :: struct {
    _pad0: [16]u8,
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
    bodyState0: _c.int32_t,
    bodyState1: _c.int32_t,
    _pad0: [8]u8,
}

PxSolverConstraintPrepDesc :: struct {
    invMassScales: PxConstraintInvMassScale,
    desc: ^PxSolverConstraintDesc,
    body0: ^PxSolverBody,
    body1: ^PxSolverBody,
    data0: ^PxSolverBodyData,
    data1: ^PxSolverBodyData,
    bodyFrame0: PxTransform,
    bodyFrame1: PxTransform,
    bodyState0: _c.int32_t,
    bodyState1: _c.int32_t,
    _pad0: [8]u8,
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
    _pad1: [3]u8,
    body0WorldOffset: PxVec3Padded,
    _pad2: [8]u8,
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
    bodyState0: _c.int32_t,
    bodyState1: _c.int32_t,
    shapeInteraction: rawptr,
    contacts: ^PxContactPoint,
    numContacts: _c.uint32_t,
    hasMaxImpulse: _c.bool,
    disableStrongFriction: _c.bool,
    hasForceThresholds: _c.bool,
    _pad0: [1]u8,
    restDistance: _c.float,
    maxCCDSeparation: _c.float,
    frictionPtr: ^_c.uint8_t,
    frictionCount: _c.uint8_t,
    _pad1: [7]u8,
    contactForces: ^_c.float,
    startFrictionPatchIndex: _c.uint32_t,
    numFrictionPatches: _c.uint32_t,
    startContactPatchIndex: _c.uint32_t,
    numContactPatches: _c.uint16_t,
    axisConstraintCount: _c.uint16_t,
    offsetSlop: _c.float,
    _pad2: [12]u8,
}

PxConstraintAllocator :: struct {
    vtable_: rawptr,
};

PxArticulationLimit :: struct {
    low: _c.float,
    high: _c.float,
}

PxArticulationDrive :: struct {
    stiffness: _c.float,
    damping: _c.float,
    maxForce: _c.float,
    driveType: _c.int32_t,
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
    bodyState0: _c.int32_t,
    bodyState1: _c.int32_t,
    _pad0: [8]u8,
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
    bodyState0: _c.int32_t,
    bodyState1: _c.int32_t,
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
    _pad0: [3]u8,
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
    bodyState0: _c.int32_t,
    bodyState1: _c.int32_t,
    shapeInteraction: rawptr,
    contacts: ^PxContactPoint,
    numContacts: _c.uint32_t,
    hasMaxImpulse: _c.bool,
    disableStrongFriction: _c.bool,
    hasForceThresholds: _c.bool,
    _pad0: [1]u8,
    restDistance: _c.float,
    maxCCDSeparation: _c.float,
    frictionPtr: ^_c.uint8_t,
    frictionCount: _c.uint8_t,
    _pad1: [7]u8,
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
    _vtable: rawptr,
    _pad0: [16]u8,
    userData: rawptr,
}

PxArticulationTendonJoint :: struct {
    _vtable: rawptr,
    _pad0: [16]u8,
    userData: rawptr,
}

PxArticulationTendon :: struct {
    _vtable: rawptr,
    _pad0: [16]u8,
    userData: rawptr,
}

PxArticulationSpatialTendon :: struct {
    _vtable: rawptr,
    _pad0: [16]u8,
    userData: rawptr,
}

PxArticulationFixedTendon :: struct {
    _vtable: rawptr,
    _pad0: [16]u8,
    userData: rawptr,
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
    _pad0: [4]u8,
}

PxArticulationSensor :: struct {
    _vtable: rawptr,
    _pad0: [16]u8,
    userData: rawptr,
}

PxArticulationReducedCoordinate :: struct {
    _vtable: rawptr,
    _pad0: [16]u8,
    userData: rawptr,
}

PxArticulationJointReducedCoordinate :: struct {
    _vtable: rawptr,
    _pad0: [16]u8,
    userData: rawptr,
}

PxShape :: struct {
    _vtable: rawptr,
    _pad0: [16]u8,
    userData: rawptr,
}

PxRigidActor :: struct {
    _vtable: rawptr,
    _pad0: [16]u8,
    userData: rawptr,
}

PxNodeIndex :: struct {
    _pad0: [8]u8,
    unused0: [1]u8,
}

PxRigidBody :: struct {
    _vtable: rawptr,
    _pad0: [16]u8,
    userData: rawptr,
}

PxArticulationLink :: struct {
    _vtable: rawptr,
    _pad0: [16]u8,
    userData: rawptr,
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
    _pad0: [8]u8,
    visualize: rawptr,
    flag: _c.int32_t,
    _pad1: [4]u8,
}

PxConstraint :: struct {
    _vtable: rawptr,
    _pad0: [16]u8,
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
    contact: PxVec3,
    separation: _c.float,
    targetVelocity: PxVec3,
    maxImpulse: _c.float,
}

PxModifiableContact :: struct {
    contact: PxVec3,
    separation: _c.float,
    targetVelocity: PxVec3,
    maxImpulse: _c.float,
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
    _pad0: [4]u8,
    patch: ^PxContactPatch,
    contact: ^PxContact,
    faceIndice: ^_c.uint32_t,
    totalPatches: _c.uint32_t,
    totalContacts: _c.uint32_t,
    nextContactIndex: _c.uint32_t,
    nextPatchIndex: _c.uint32_t,
    contactPatchHeaderSize: _c.uint32_t,
    contactPointSize: _c.uint32_t,
    mStreamFormat: _c.int32_t,
    forceNoResponse: _c.uint32_t,
    pointStepped: _c.bool,
    _pad1: [3]u8,
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
    _pad0: [4]u8,
}

PxContactSet :: struct {
    _pad0: [16]u8,
    unused0: [1]u8,
}

PxContactModifyPair :: struct {
    actor: [2]^PxRigidActor,
    shape: [2]^PxShape,
    transform: [2]PxTransform,
    contacts: PxContactSet,
}

PxContactModifyCallback :: struct {
    vtable_: rawptr,
};

PxCCDContactModifyCallback :: struct {
    vtable_: rawptr,
};

PxDeletionListener :: struct {
    vtable_: rawptr,
};

PxBaseMaterial :: struct {
    _vtable: rawptr,
    _pad0: [16]u8,
    userData: rawptr,
}

PxFEMMaterial :: struct {
    _vtable: rawptr,
    _pad0: [16]u8,
    userData: rawptr,
}

PxFilterData :: struct {
    word0: _c.uint32_t,
    word1: _c.uint32_t,
    word2: _c.uint32_t,
    word3: _c.uint32_t,
}

PxSimulationFilterCallback :: struct {
    vtable_: rawptr,
};

PxParticleRigidFilterPair :: struct {
    mID0: _c.uint64_t,
    mID1: _c.uint64_t,
}

PxLockedData :: struct {
    vtable_: rawptr,
};

PxMaterial :: struct {
    _vtable: rawptr,
    _pad0: [16]u8,
    userData: rawptr,
}

PxGpuParticleBufferIndexPair :: struct {
    systemIndex: _c.uint32_t,
    bufferIndex: _c.uint32_t,
}

PxCudaContextManager :: struct {
    _unused: [0]u8,
}

PxParticleRigidAttachment :: struct {
    _unused: [0]u8,
}

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
    _pad0: [3]u8,
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
    _vtable: rawptr,
    _pad0: [16]u8,
    userData: rawptr,
}

PxOmniPvd :: struct {
    _unused: [0]u8,
}

PxPhysics :: struct {
    vtable_: rawptr,
};

PxActorShape :: struct {
    actor: ^PxRigidActor,
    shape: ^PxShape,
}

PxRaycastHit :: struct {
    faceIndex: _c.uint32_t,
    flags: _c.uint16_t,
    _pad0: [2]u8,
    position: PxVec3,
    normal: PxVec3,
    distance: _c.float,
    u: _c.float,
    v: _c.float,
    _pad1: [4]u8,
    actor: ^PxRigidActor,
    shape: ^PxShape,
}

PxOverlapHit :: struct {
    faceIndex: _c.uint32_t,
    _pad0: [4]u8,
    actor: ^PxRigidActor,
    shape: ^PxShape,
}

PxSweepHit :: struct {
    faceIndex: _c.uint32_t,
    flags: _c.uint16_t,
    _pad0: [2]u8,
    position: PxVec3,
    normal: PxVec3,
    distance: _c.float,
    _pad1: [4]u8,
    actor: ^PxRigidActor,
    shape: ^PxShape,
}

PxRaycastCallback :: struct {
    _vtable: rawptr,
    _pad0: [8]u8,
    block: PxRaycastHit,
    hasBlock: _c.bool,
    _pad1: [7]u8,
    touches: ^PxRaycastHit,
    maxNbTouches: _c.uint32_t,
    nbTouches: _c.uint32_t,
}

PxOverlapCallback :: struct {
    _vtable: rawptr,
    _pad0: [8]u8,
    block: PxOverlapHit,
    hasBlock: _c.bool,
    _pad1: [7]u8,
    touches: ^PxOverlapHit,
    maxNbTouches: _c.uint32_t,
    nbTouches: _c.uint32_t,
}

PxSweepCallback :: struct {
    _vtable: rawptr,
    _pad0: [8]u8,
    block: PxSweepHit,
    hasBlock: _c.bool,
    _pad1: [7]u8,
    touches: ^PxSweepHit,
    maxNbTouches: _c.uint32_t,
    nbTouches: _c.uint32_t,
}

PxRaycastBuffer :: struct {
    _vtable: rawptr,
    _pad0: [8]u8,
    block: PxRaycastHit,
    hasBlock: _c.bool,
    _pad1: [7]u8,
    touches: ^PxRaycastHit,
    maxNbTouches: _c.uint32_t,
    nbTouches: _c.uint32_t,
}

PxOverlapBuffer :: struct {
    _vtable: rawptr,
    _pad0: [8]u8,
    block: PxOverlapHit,
    hasBlock: _c.bool,
    _pad1: [7]u8,
    touches: ^PxOverlapHit,
    maxNbTouches: _c.uint32_t,
    nbTouches: _c.uint32_t,
}

PxSweepBuffer :: struct {
    _vtable: rawptr,
    _pad0: [8]u8,
    block: PxSweepHit,
    hasBlock: _c.bool,
    _pad1: [7]u8,
    touches: ^PxSweepHit,
    maxNbTouches: _c.uint32_t,
    nbTouches: _c.uint32_t,
}

PxQueryCache :: struct {
    shape: ^PxShape,
    actor: ^PxRigidActor,
    faceIndex: _c.uint32_t,
    _pad0: [4]u8,
}

PxQueryFilterData :: struct {
    data: PxFilterData,
    flags: _c.uint16_t,
    _pad0: [2]u8,
}

PxQueryFilterCallback :: struct {
    vtable_: rawptr,
};

PxRigidDynamic :: struct {
    _vtable: rawptr,
    _pad0: [16]u8,
    userData: rawptr,
}

PxRigidStatic :: struct {
    _vtable: rawptr,
    _pad0: [16]u8,
    userData: rawptr,
}

PxSceneQueryDesc :: struct {
    staticStructure: _c.int32_t,
    dynamicStructure: _c.int32_t,
    dynamicTreeRebuildRateHint: _c.uint32_t,
    dynamicTreeSecondaryPruner: _c.int32_t,
    staticBVHBuildStrategy: _c.int32_t,
    dynamicBVHBuildStrategy: _c.int32_t,
    staticNbObjectsPerNode: _c.uint32_t,
    dynamicNbObjectsPerNode: _c.uint32_t,
    sceneQueryUpdateMode: _c.int32_t,
}

PxSceneQuerySystemBase :: struct {
    vtable_: rawptr,
};

PxSceneSQSystem :: struct {
    vtable_: rawptr,
};

PxSceneQuerySystem :: struct {
    vtable_: rawptr,
};

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
    _pad0: [6]u8,
}

PxBroadPhaseCaps :: struct {
    mMaxNbRegions: _c.uint32_t,
}

PxBroadPhaseDesc :: struct {
    mType: _c.int32_t,
    _pad0: [4]u8,
    mContextID: _c.uint64_t,
    _pad1: [8]u8,
    mFoundLostPairsCapacity: _c.uint32_t,
    mDiscardStaticVsKinematic: _c.bool,
    mDiscardKinematicVsKinematic: _c.bool,
    _pad2: [2]u8,
}

PxBroadPhaseUpdateData :: struct {
    mCreated: ^_c.uint32_t,
    mNbCreated: _c.uint32_t,
    _pad0: [4]u8,
    mUpdated: ^_c.uint32_t,
    mNbUpdated: _c.uint32_t,
    _pad1: [4]u8,
    mRemoved: ^_c.uint32_t,
    mNbRemoved: _c.uint32_t,
    _pad2: [4]u8,
    mBounds: ^PxBounds3,
    mGroups: ^_c.uint32_t,
    mDistances: ^_c.float,
    mCapacity: _c.uint32_t,
    _pad3: [4]u8,
}

PxBroadPhasePair :: struct {
    mID0: _c.uint32_t,
    mID1: _c.uint32_t,
}

PxBroadPhaseResults :: struct {
    mNbCreatedPairs: _c.uint32_t,
    _pad0: [4]u8,
    mCreatedPairs: ^PxBroadPhasePair,
    mNbDeletedPairs: _c.uint32_t,
    _pad1: [4]u8,
    mDeletedPairs: ^PxBroadPhasePair,
}

PxBroadPhaseRegions :: struct {
    vtable_: rawptr,
};

PxBroadPhase :: struct {
    vtable_: rawptr,
};

PxAABBManager :: struct {
    vtable_: rawptr,
};

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
    staticStructure: _c.int32_t,
    dynamicStructure: _c.int32_t,
    dynamicTreeRebuildRateHint: _c.uint32_t,
    dynamicTreeSecondaryPruner: _c.int32_t,
    staticBVHBuildStrategy: _c.int32_t,
    dynamicBVHBuildStrategy: _c.int32_t,
    staticNbObjectsPerNode: _c.uint32_t,
    dynamicNbObjectsPerNode: _c.uint32_t,
    sceneQueryUpdateMode: _c.int32_t,
    gravity: PxVec3,
    simulationEventCallback: ^PxSimulationEventCallback,
    contactModifyCallback: ^PxContactModifyCallback,
    ccdContactModifyCallback: ^PxCCDContactModifyCallback,
    filterShaderData: rawptr,
    filterShaderDataSize: _c.uint32_t,
    _pad0: [4]u8,
    filterShader: rawptr,
    filterCallback: ^PxSimulationFilterCallback,
    kineKineFilteringMode: _c.int32_t,
    staticKineFilteringMode: _c.int32_t,
    broadPhaseType: _c.int32_t,
    _pad1: [4]u8,
    broadPhaseCallback: ^PxBroadPhaseCallback,
    limits: PxSceneLimits,
    frictionType: _c.int32_t,
    solverType: _c.int32_t,
    bounceThresholdVelocity: _c.float,
    frictionOffsetThreshold: _c.float,
    frictionCorrelationDistance: _c.float,
    flags: _c.uint32_t,
    cpuDispatcher: ^PxCpuDispatcher,
    _pad2: [8]u8,
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
    _pad3: [8]u8,
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
    _pad0: [4]u8,
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
    _pad0: [4]u8,
    nodeIndex: PxNodeIndex,
}

PxIndexDataPair :: struct {
    index: _c.uint32_t,
    _pad0: [4]u8,
    data: rawptr,
}

PxPvdSceneClient :: struct {
    vtable_: rawptr,
};

PxDominanceGroupPair :: struct {
    dominance0: _c.uint8_t,
    dominance1: _c.uint8_t,
}

PxBroadPhaseCallback :: struct {
    vtable_: rawptr,
};

PxScene :: struct {
    _vtable: rawptr,
    _pad0: [8]u8,
    userData: rawptr,
}

PxSceneReadLock :: struct {
    _pad0: [8]u8,
    unused0: [1]u8,
}

PxSceneWriteLock :: struct {
    _pad0: [8]u8,
    unused0: [1]u8,
}

PxContactPairExtraDataItem :: struct {
    type: _c.uint8_t,
}

PxContactPairVelocity :: struct {
    type: _c.uint8_t,
    _pad0: [3]u8,
    linearVelocity: [2]PxVec3,
    angularVelocity: [2]PxVec3,
}

PxContactPairPose :: struct {
    type: _c.uint8_t,
    _pad0: [3]u8,
    globalPose: [2]PxTransform,
}

PxContactPairIndex :: struct {
    type: _c.uint8_t,
    _pad0: [1]u8,
    index: _c.uint16_t,
}

PxContactPairExtraDataIterator :: struct {
    currPtr: ^_c.uint8_t,
    endPtr: ^_c.uint8_t,
    preSolverVelocity: ^PxContactPairVelocity,
    postSolverVelocity: ^PxContactPairVelocity,
    eventPose: ^PxContactPairPose,
    contactPairIndex: _c.uint32_t,
    _pad0: [4]u8,
}

PxContactPairHeader :: struct {
    actors: [2]^PxActor,
    extraDataStream: ^_c.uint8_t,
    extraDataStreamSize: _c.uint16_t,
    flags: _c.uint16_t,
    _pad0: [4]u8,
    pairs: ^PxContactPair,
    nbPairs: _c.uint32_t,
    _pad1: [4]u8,
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
    flags: _c.uint16_t,
    events: _c.uint16_t,
    internalData: [2]_c.uint32_t,
    _pad0: [4]u8,
}

PxTriggerPair :: struct {
    triggerShape: ^PxShape,
    triggerActor: ^PxActor,
    otherShape: ^PxShape,
    otherActor: ^PxActor,
    status: _c.int32_t,
    flags: _c.uint8_t,
    _pad0: [3]u8,
}

PxConstraintInfo :: struct {
    constraint: ^PxConstraint,
    externalReference: rawptr,
    type: _c.uint32_t,
    _pad0: [4]u8,
}

PxSimulationEventCallback :: struct {
    vtable_: rawptr,
};

PxFEMParameters :: struct {
    velocityDamping: _c.float,
    settlingThreshold: _c.float,
    sleepThreshold: _c.float,
    sleepDamping: _c.float,
    selfCollisionFilterDistance: _c.float,
    selfCollisionStressTolerance: _c.float,
}

PxPruningStructure :: struct {
    _vtable: rawptr,
    _pad0: [16]u8,
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
    _pad0: [8]u8,
    mUserData: rawptr,
    mPos: PxExtendedVec3,
    mRot: PxQuat,
    mHalfExtents: PxVec3,
    _pad1: [4]u8,
}

PxCapsuleObstacle :: struct {
    _pad0: [8]u8,
    mUserData: rawptr,
    mPos: PxExtendedVec3,
    mRot: PxQuat,
    mHalfHeight: _c.float,
    mRadius: _c.float,
}

PxObstacleContext :: struct {
    vtable_: rawptr,
};

PxControllerState :: struct {
    deltaXP: PxVec3,
    _pad0: [4]u8,
    touchedShape: ^PxShape,
    touchedActor: ^PxRigidActor,
    touchedObstacleHandle: _c.uint32_t,
    collisionFlags: _c.uint32_t,
    standOnAnotherCCT: _c.bool,
    standOnObstacle: _c.bool,
    isMovingUp: _c.bool,
    _pad1: [5]u8,
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
    _pad0: [4]u8,
}

PxControllerShapeHit :: struct {
    controller: ^PxController,
    worldPos: PxExtendedVec3,
    worldNormal: PxVec3,
    dir: PxVec3,
    length: _c.float,
    _pad0: [4]u8,
    shape: ^PxShape,
    actor: ^PxRigidActor,
    triangleIndex: _c.uint32_t,
    _pad1: [4]u8,
}

PxControllersHit :: struct {
    controller: ^PxController,
    worldPos: PxExtendedVec3,
    worldNormal: PxVec3,
    dir: PxVec3,
    length: _c.float,
    _pad0: [4]u8,
    other: ^PxController,
}

PxControllerObstacleHit :: struct {
    controller: ^PxController,
    worldPos: PxExtendedVec3,
    worldNormal: PxVec3,
    dir: PxVec3,
    length: _c.float,
    _pad0: [4]u8,
    userData: rawptr,
}

PxUserControllerHitReport :: struct {
    vtable_: rawptr,
};

PxControllerFilterCallback :: struct {
    vtable_: rawptr,
};

PxControllerFilters :: struct {
    mFilterData: ^PxFilterData,
    mFilterCallback: ^PxQueryFilterCallback,
    mFilterFlags: _c.uint16_t,
    _pad0: [6]u8,
    mCCTFilterCallback: ^PxControllerFilterCallback,
}

PxControllerDesc :: struct {
    _vtable: rawptr,
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
    _pad1: [4]u8,
    reportCallback: ^PxUserControllerHitReport,
    behaviorCallback: ^PxControllerBehaviorCallback,
    nonWalkableMode: _c.int32_t,
    _pad2: [4]u8,
    material: ^PxMaterial,
    registerDeletionListener: _c.bool,
    clientID: _c.uint8_t,
    _pad3: [6]u8,
    userData: rawptr,
    _pad4: [8]u8,
}

PxController :: struct {
    vtable_: rawptr,
};

PxBoxControllerDesc :: struct {
    _vtable: rawptr,
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
    _pad1: [4]u8,
    reportCallback: ^PxUserControllerHitReport,
    behaviorCallback: ^PxControllerBehaviorCallback,
    nonWalkableMode: _c.int32_t,
    _pad2: [4]u8,
    material: ^PxMaterial,
    registerDeletionListener: _c.bool,
    clientID: _c.uint8_t,
    _pad3: [6]u8,
    userData: rawptr,
    _pad4: [4]u8,
    halfHeight: _c.float,
    halfSideExtent: _c.float,
    halfForwardExtent: _c.float,
}

PxBoxController :: struct {
    vtable_: rawptr,
};

PxCapsuleControllerDesc :: struct {
    _vtable: rawptr,
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
    _pad1: [4]u8,
    reportCallback: ^PxUserControllerHitReport,
    behaviorCallback: ^PxControllerBehaviorCallback,
    nonWalkableMode: _c.int32_t,
    _pad2: [4]u8,
    material: ^PxMaterial,
    registerDeletionListener: _c.bool,
    clientID: _c.uint8_t,
    _pad3: [6]u8,
    userData: rawptr,
    _pad4: [4]u8,
    radius: _c.float,
    height: _c.float,
    climbingMode: _c.int32_t,
}

PxCapsuleController :: struct {
    vtable_: rawptr,
};

PxControllerBehaviorCallback :: struct {
    vtable_: rawptr,
};

PxControllerManager :: struct {
    vtable_: rawptr,
};

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
    bitsPerSubgridPixel: _c.int32_t,
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
    flags: _c.uint16_t,
    vertexLimit: _c.uint16_t,
    polygonLimit: _c.uint16_t,
    quantizedCount: _c.uint16_t,
    sdfDesc: ^PxSDFDesc,
}

PxTriangleMeshDesc :: struct {
    points: PxBoundedData,
    triangles: PxBoundedData,
    flags: _c.uint16_t,
    _pad0: [22]u8,
    sdfDesc: ^PxSDFDesc,
}

PxTetrahedronMeshDesc :: struct {
    _pad0: [16]u8,
    points: PxBoundedData,
    tetrahedrons: PxBoundedData,
    flags: _c.uint16_t,
    tetsPerElement: _c.uint16_t,
    _pad1: [4]u8,
}

PxSoftBodySimulationDataDesc :: struct {
    vertexToTet: PxBoundedData,
}

PxBVH34MidphaseDesc :: struct {
    numPrimsPerLeaf: _c.uint32_t,
    buildStrategy: _c.int32_t,
    quantized: _c.bool,
    _pad0: [3]u8,
}

PxMidphaseDesc :: struct {
    _pad0: [16]u8,
    unused0: [1]u8,
}

PxBVHDesc :: struct {
    bounds: PxBoundedData,
    enlargement: _c.float,
    numPrimsPerLeaf: _c.uint32_t,
    buildStrategy: _c.int32_t,
    _pad0: [4]u8,
}

PxCookingParams :: struct {
    areaTestEpsilon: _c.float,
    planeTolerance: _c.float,
    convexMeshCookingType: _c.int32_t,
    suppressTriangleMeshRemapTable: _c.bool,
    buildTriangleAdjacencies: _c.bool,
    buildGPUData: _c.bool,
    _pad0: [1]u8,
    scale: PxTolerancesScale,
    meshPreprocessParams: _c.uint32_t,
    meshWeldTolerance: _c.float,
    midphaseDesc: PxMidphaseDesc,
    gaussMapLimit: _c.uint32_t,
    maxWeightRatioInTet: _c.float,
}

PxDefaultMemoryOutputStream :: struct {
    _vtable: rawptr,
    _pad0: [32]u8,
}

PxDefaultMemoryInputData :: struct {
    _vtable: rawptr,
    _pad0: [32]u8,
}

PxDefaultFileOutputStream :: struct {
    _vtable: rawptr,
    _pad0: [16]u8,
}

PxDefaultFileInputData :: struct {
    _vtable: rawptr,
    _pad0: [24]u8,
}

PxDefaultAllocator :: struct {
    vtable_: rawptr,
};

PxJoint :: struct {
    _vtable: rawptr,
    _pad0: [16]u8,
    userData: rawptr,
}

PxSpring :: struct {
    stiffness: _c.float,
    damping: _c.float,
}

PxDistanceJoint :: struct {
    _vtable: rawptr,
    _pad0: [16]u8,
    userData: rawptr,
}

PxJacobianRow :: struct {
    linear0: PxVec3,
    linear1: PxVec3,
    angular0: PxVec3,
    angular1: PxVec3,
}

PxContactJoint :: struct {
    _vtable: rawptr,
    _pad0: [16]u8,
    userData: rawptr,
}

PxFixedJoint :: struct {
    _vtable: rawptr,
    _pad0: [16]u8,
    userData: rawptr,
}

PxJointLimitParameters :: struct {
    restitution: _c.float,
    bounceThreshold: _c.float,
    stiffness: _c.float,
    damping: _c.float,
    contactDistance_deprecated: _c.float,
}

PxJointLinearLimit :: struct {
    restitution: _c.float,
    bounceThreshold: _c.float,
    stiffness: _c.float,
    damping: _c.float,
    contactDistance_deprecated: _c.float,
    value: _c.float,
}

PxJointLinearLimitPair :: struct {
    restitution: _c.float,
    bounceThreshold: _c.float,
    stiffness: _c.float,
    damping: _c.float,
    contactDistance_deprecated: _c.float,
    upper: _c.float,
    lower: _c.float,
}

PxJointAngularLimitPair :: struct {
    restitution: _c.float,
    bounceThreshold: _c.float,
    stiffness: _c.float,
    damping: _c.float,
    contactDistance_deprecated: _c.float,
    upper: _c.float,
    lower: _c.float,
}

PxJointLimitCone :: struct {
    restitution: _c.float,
    bounceThreshold: _c.float,
    stiffness: _c.float,
    damping: _c.float,
    contactDistance_deprecated: _c.float,
    yAngle: _c.float,
    zAngle: _c.float,
}

PxJointLimitPyramid :: struct {
    restitution: _c.float,
    bounceThreshold: _c.float,
    stiffness: _c.float,
    damping: _c.float,
    contactDistance_deprecated: _c.float,
    yAngleMin: _c.float,
    yAngleMax: _c.float,
    zAngleMin: _c.float,
    zAngleMax: _c.float,
}

PxPrismaticJoint :: struct {
    _vtable: rawptr,
    _pad0: [16]u8,
    userData: rawptr,
}

PxRevoluteJoint :: struct {
    _vtable: rawptr,
    _pad0: [16]u8,
    userData: rawptr,
}

PxSphericalJoint :: struct {
    _vtable: rawptr,
    _pad0: [16]u8,
    userData: rawptr,
}

PxD6JointDrive :: struct {
    stiffness: _c.float,
    damping: _c.float,
    forceLimit: _c.float,
    flags: _c.uint32_t,
}

PxD6Joint :: struct {
    _vtable: rawptr,
    _pad0: [16]u8,
    userData: rawptr,
}

PxGearJoint :: struct {
    _vtable: rawptr,
    _pad0: [16]u8,
    userData: rawptr,
}

PxRackAndPinionJoint :: struct {
    _vtable: rawptr,
    _pad0: [16]u8,
    userData: rawptr,
}

PxGroupsMask :: struct {
    bits0: _c.uint16_t,
    bits1: _c.uint16_t,
    bits2: _c.uint16_t,
    bits3: _c.uint16_t,
}

PxDefaultErrorCallback :: struct {
    vtable_: rawptr,
};

PxRigidActorExt :: struct {
    _pad0: [1]u8,
    unused0: [1]u8,
}

PxMassProperties :: struct {
    inertiaTensor: PxMat33,
    centerOfMass: PxVec3,
    mass: _c.float,
}

PxRigidBodyExt :: struct {
    _pad0: [1]u8,
    unused0: [1]u8,
}

PxShapeExt :: struct {
    _pad0: [1]u8,
    unused0: [1]u8,
}

PxMeshOverlapUtil :: struct {
    _pad0: [1040]u8,
    unused0: [1]u8,
}

PxBinaryConverter :: struct {
    _unused: [0]u8,
}

PxXmlMiscParameter :: struct {
    upVector: PxVec3,
    scale: PxTolerancesScale,
}

PxSerialization :: struct {
    _pad0: [1]u8,
    unused0: [1]u8,
}

PxDefaultCpuDispatcher :: struct {
    vtable_: rawptr,
};

PxStringTableExt :: struct {
    _pad0: [1]u8,
    unused0: [1]u8,
}

PxBroadPhaseExt :: struct {
    _pad0: [1]u8,
    unused0: [1]u8,
}

PxSceneQueryExt :: struct {
    _pad0: [1]u8,
    unused0: [1]u8,
}

PxBatchQueryExt :: struct {
    vtable_: rawptr,
};

PxCustomSceneQuerySystem :: struct {
    vtable_: rawptr,
};

PxCustomSceneQuerySystemAdapter :: struct {
    vtable_: rawptr,
};

PxSamplingExt :: struct {
    _pad0: [1]u8,
    unused0: [1]u8,
}

PxPoissonSampler :: struct {
    _vtable: rawptr,
    _pad0: [8]u8,
}

PxTriangleMeshPoissonSampler :: struct {
    _vtable: rawptr,
    _pad0: [8]u8,
}

PxTetrahedronMeshExt :: struct {
    _pad0: [1]u8,
    unused0: [1]u8,
}

PxRepXObject :: struct {
    typeName: ^_c.char,
    serializable: rawptr,
    id: _c.uint64_t,
}

PxCooking :: struct {
    _unused: [0]u8,
}

PxRepXInstantiationArgs :: struct {
    _pad0: [8]u8,
    cooker: ^PxCooking,
    stringTable: ^PxStringTable,
}

XmlMemoryAllocator :: struct {
    _unused: [0]u8,
}

XmlWriter :: struct {
    _unused: [0]u8,
}

XmlReader :: struct {
    _unused: [0]u8,
}

MemoryBuffer :: struct {
    _unused: [0]u8,
}

PxRepXSerializer :: struct {
    vtable_: rawptr,
};

PxVehicleWheels4SimData :: struct {
    _unused: [0]u8,
}

PxVehicleWheels4DynData :: struct {
    _unused: [0]u8,
}

PxVehicleTireForceCalculator :: struct {
    _unused: [0]u8,
}

PxVehicleDrivableSurfaceToTireFrictionPairs :: struct {
    _unused: [0]u8,
}

PxVehicleTelemetryData :: struct {
    _unused: [0]u8,
}

PxPvd :: struct {
    vtable_: rawptr,
};

PxPvdTransport :: struct {
    vtable_: rawptr,
};
when ODIN_OS == .Linux do foreign import libphysx "libphysx.a"

@(default_calling_convention = "c")
foreign libphysx {
    PxAllocatorCallback_delete :: proc(self_: ^PxAllocatorCallback) ---

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
    PxAllocatorCallback_allocate_mut :: proc(self_: ^PxAllocatorCallback, size: _c.size_t, typeName: ^_c.char, filename: ^_c.char, line: _c.int32_t) -> rawptr ---

    /// Frees memory previously allocated by allocate().
    ///
    /// Threading:
    /// This function should be thread safe as it can be called in the context of the user thread
    /// and physics processing thread(s).
    PxAllocatorCallback_deallocate_mut :: proc(self_: ^PxAllocatorCallback, ptr: rawptr) ---

    PxAssertHandler_delete :: proc(self_: ^PxAssertHandler) ---

    phys_PxGetAssertHandler :: proc() -> ^PxAssertHandler ---

    phys_PxSetAssertHandler :: proc(handler: ^PxAssertHandler) ---

    /// Destroys the instance it is called on.
    ///
    /// The operation will fail, if there are still modules referencing the foundation object. Release all dependent modules
    /// prior to calling this method.
    PxFoundation_release_mut :: proc(self_: ^PxFoundation) ---

    /// retrieves error callback
    PxFoundation_getErrorCallback_mut :: proc(self_: ^PxFoundation) -> ^PxErrorCallback ---

    /// Sets mask of errors to report.
    PxFoundation_setErrorLevel_mut :: proc(self_: ^PxFoundation, mask: _c.uint32_t) ---

    /// Retrieves mask of errors to be reported.
    PxFoundation_getErrorLevel :: proc(self_: ^PxFoundation) -> _c.uint32_t ---

    /// Retrieves the allocator this object was created with.
    PxFoundation_getAllocatorCallback_mut :: proc(self_: ^PxFoundation) -> ^PxAllocatorCallback ---

    /// Retrieves if allocation names are being passed to allocator callback.
    PxFoundation_getReportAllocationNames :: proc(self_: ^PxFoundation) -> _c.bool ---

    /// Set if allocation names are being passed to allocator callback.
    ///
    /// Enabled by default in debug and checked build, disabled by default in profile and release build.
    PxFoundation_setReportAllocationNames_mut :: proc(self_: ^PxFoundation, value: _c.bool) ---

    PxFoundation_registerAllocationListener_mut :: proc(self_: ^PxFoundation, listener: ^PxAllocationListener) ---

    PxFoundation_deregisterAllocationListener_mut :: proc(self_: ^PxFoundation, listener: ^PxAllocationListener) ---

    PxFoundation_registerErrorCallback_mut :: proc(self_: ^PxFoundation, callback: ^PxErrorCallback) ---

    PxFoundation_deregisterErrorCallback_mut :: proc(self_: ^PxFoundation, callback: ^PxErrorCallback) ---

    /// Creates an instance of the foundation class
    ///
    /// The foundation class is needed to initialize higher level SDKs. There may be only one instance per process.
    /// Calling this method after an instance has been created already will result in an error message and NULL will be
    /// returned.
    ///
    /// Foundation instance on success, NULL if operation failed
    phys_PxCreateFoundation :: proc(version: _c.uint32_t, allocator: ^PxAllocatorCallback, errorCallback: ^PxErrorCallback) -> ^PxFoundation ---

    phys_PxSetFoundationInstance :: proc(foundation: ^PxFoundation) ---

    phys_PxGetFoundation :: proc() -> ^PxFoundation ---

    /// Get the callback that will be used for all profiling.
    phys_PxGetProfilerCallback :: proc() -> ^PxProfilerCallback ---

    /// Set the callback that will be used for all profiling.
    phys_PxSetProfilerCallback :: proc(profiler: ^PxProfilerCallback) ---

    /// Get the allocator callback
    phys_PxGetAllocatorCallback :: proc() -> ^PxAllocatorCallback ---

    /// Get the broadcasting allocator callback
    phys_PxGetBroadcastAllocator :: proc() -> ^PxAllocatorCallback ---

    /// Get the error callback
    phys_PxGetErrorCallback :: proc() -> ^PxErrorCallback ---

    /// Get the broadcasting error callback
    phys_PxGetBroadcastError :: proc() -> ^PxErrorCallback ---

    /// Get the warn once timestamp
    phys_PxGetWarnOnceTimeStamp :: proc() -> _c.uint32_t ---

    /// Decrement the ref count of PxFoundation
    phys_PxDecFoundationRefCount :: proc() ---

    /// Increment the ref count of PxFoundation
    phys_PxIncFoundationRefCount :: proc() ---

    PxAllocator_new :: proc(anon_param0: ^_c.char) -> PxAllocator ---

    PxAllocator_allocate_mut :: proc(self_: ^PxAllocator, size: _c.size_t, file: ^_c.char, line: _c.int32_t) -> rawptr ---

    PxAllocator_deallocate_mut :: proc(self_: ^PxAllocator, ptr: rawptr) ---

    PxRawAllocator_new :: proc(anon_param0: ^_c.char) -> PxRawAllocator ---

    PxRawAllocator_allocate_mut :: proc(self_: ^PxRawAllocator, size: _c.size_t, anon_param1: ^_c.char, anon_param2: _c.int32_t) -> rawptr ---

    PxRawAllocator_deallocate_mut :: proc(self_: ^PxRawAllocator, ptr: rawptr) ---

    PxVirtualAllocatorCallback_delete :: proc(self_: ^PxVirtualAllocatorCallback) ---

    PxVirtualAllocatorCallback_allocate_mut :: proc(self_: ^PxVirtualAllocatorCallback, size: _c.size_t, group: _c.int32_t, file: ^_c.char, line: _c.int32_t) -> rawptr ---

    PxVirtualAllocatorCallback_deallocate_mut :: proc(self_: ^PxVirtualAllocatorCallback, ptr: rawptr) ---

    PxVirtualAllocator_new :: proc(callback: ^PxVirtualAllocatorCallback, group: _c.int32_t) -> PxVirtualAllocator ---

    PxVirtualAllocator_allocate_mut :: proc(self_: ^PxVirtualAllocator, size: _c.size_t, file: ^_c.char, line: _c.int32_t) -> rawptr ---

    PxVirtualAllocator_deallocate_mut :: proc(self_: ^PxVirtualAllocator, ptr: rawptr) ---

    PxTempAllocatorChunk_new :: proc() -> PxTempAllocatorChunk ---

    PxTempAllocator_new :: proc(anon_param0: ^_c.char) -> PxTempAllocator ---

    PxTempAllocator_allocate_mut :: proc(self_: ^PxTempAllocator, size: _c.size_t, file: ^_c.char, line: _c.int32_t) -> rawptr ---

    PxTempAllocator_deallocate_mut :: proc(self_: ^PxTempAllocator, ptr: rawptr) ---

    /// Sets the bytes of the provided buffer to zero.
    ///
    /// Pointer to memory block (same as input)
    phys_PxMemZero :: proc(dest: rawptr, count: _c.uint32_t) -> rawptr ---

    /// Sets the bytes of the provided buffer to the specified value.
    ///
    /// Pointer to memory block (same as input)
    phys_PxMemSet :: proc(dest: rawptr, c: _c.int32_t, count: _c.uint32_t) -> rawptr ---

    /// Copies the bytes of one memory block to another. The memory blocks must not overlap.
    ///
    /// Use [`PxMemMove`] if memory blocks overlap.
    ///
    /// Pointer to destination memory block
    phys_PxMemCopy :: proc(dest: rawptr, src: rawptr, count: _c.uint32_t) -> rawptr ---

    /// Copies the bytes of one memory block to another. The memory blocks can overlap.
    ///
    /// Use [`PxMemCopy`] if memory blocks do not overlap.
    ///
    /// Pointer to destination memory block
    phys_PxMemMove :: proc(dest: rawptr, src: rawptr, count: _c.uint32_t) -> rawptr ---

    /// Mark a specified amount of memory with 0xcd pattern. This is used to check that the meta data
    /// definition for serialized classes is complete in checked builds.
    phys_PxMarkSerializedMemory :: proc(ptr: rawptr, byteSize: _c.uint32_t) ---

    phys_PxMemoryBarrier :: proc() ---

    /// Return the index of the highest set bit. Undefined for zero arg.
    phys_PxHighestSetBitUnsafe :: proc(v: _c.uint32_t) -> _c.uint32_t ---

    /// Return the index of the highest set bit. Undefined for zero arg.
    phys_PxLowestSetBitUnsafe :: proc(v: _c.uint32_t) -> _c.uint32_t ---

    /// Returns the index of the highest set bit. Returns 32 for v=0.
    phys_PxCountLeadingZeros :: proc(v: _c.uint32_t) -> _c.uint32_t ---

    /// Prefetch aligned 64B x86, 32b ARM around
    phys_PxPrefetchLine :: proc(ptr: rawptr, offset: _c.uint32_t) ---

    /// Prefetch
    /// bytes starting at
    phys_PxPrefetch :: proc(ptr: rawptr, count: _c.uint32_t) ---

    phys_PxBitCount :: proc(v: _c.uint32_t) -> _c.uint32_t ---

    phys_PxIsPowerOfTwo :: proc(x: _c.uint32_t) -> _c.bool ---

    phys_PxNextPowerOfTwo :: proc(x: _c.uint32_t) -> _c.uint32_t ---

    /// Return the index of the highest set bit. Not valid for zero arg.
    phys_PxLowestSetBit :: proc(x: _c.uint32_t) -> _c.uint32_t ---

    /// Return the index of the highest set bit. Not valid for zero arg.
    phys_PxHighestSetBit :: proc(x: _c.uint32_t) -> _c.uint32_t ---

    phys_PxILog2 :: proc(num: _c.uint32_t) -> _c.uint32_t ---

    /// default constructor leaves data uninitialized.
    PxVec3_new :: proc() -> PxVec3 ---

    /// zero constructor.
    PxVec3_new_1 :: proc(anon_param0: _c.int32_t) -> PxVec3 ---

    /// Assigns scalar parameter to all elements.
    ///
    /// Useful to initialize to zero or one.
    PxVec3_new_2 :: proc(a: _c.float) -> PxVec3 ---

    /// Initializes from 3 scalar parameters.
    PxVec3_new_3 :: proc(nx: _c.float, ny: _c.float, nz: _c.float) -> PxVec3 ---

    /// tests for exact zero vector
    PxVec3_isZero :: proc(self_: ^PxVec3) -> _c.bool ---

    /// returns true if all 3 elems of the vector are finite (not NAN or INF, etc.)
    PxVec3_isFinite :: proc(self_: ^PxVec3) -> _c.bool ---

    /// is normalized - used by API parameter validation
    PxVec3_isNormalized :: proc(self_: ^PxVec3) -> _c.bool ---

    /// returns the squared magnitude
    ///
    /// Avoids calling PxSqrt()!
    PxVec3_magnitudeSquared :: proc(self_: ^PxVec3) -> _c.float ---

    /// returns the magnitude
    PxVec3_magnitude :: proc(self_: ^PxVec3) -> _c.float ---

    /// returns the scalar product of this and other.
    PxVec3_dot :: proc(self_: ^PxVec3, v: ^PxVec3) -> _c.float ---

    /// cross product
    PxVec3_cross :: proc(self_: ^PxVec3, v: ^PxVec3) -> PxVec3 ---

    /// returns a unit vector
    PxVec3_getNormalized :: proc(self_: ^PxVec3) -> PxVec3 ---

    /// normalizes the vector in place
    PxVec3_normalize_mut :: proc(self_: ^PxVec3) -> _c.float ---

    /// normalizes the vector in place. Does nothing if vector magnitude is under PX_NORMALIZATION_EPSILON.
    /// Returns vector magnitude if >= PX_NORMALIZATION_EPSILON and 0.0f otherwise.
    PxVec3_normalizeSafe_mut :: proc(self_: ^PxVec3) -> _c.float ---

    /// normalizes the vector in place. Asserts if vector magnitude is under PX_NORMALIZATION_EPSILON.
    /// returns vector magnitude.
    PxVec3_normalizeFast_mut :: proc(self_: ^PxVec3) -> _c.float ---

    /// a[i] * b[i], for all i.
    PxVec3_multiply :: proc(self_: ^PxVec3, a: ^PxVec3) -> PxVec3 ---

    /// element-wise minimum
    PxVec3_minimum :: proc(self_: ^PxVec3, v: ^PxVec3) -> PxVec3 ---

    /// returns MIN(x, y, z);
    PxVec3_minElement :: proc(self_: ^PxVec3) -> _c.float ---

    /// element-wise maximum
    PxVec3_maximum :: proc(self_: ^PxVec3, v: ^PxVec3) -> PxVec3 ---

    /// returns MAX(x, y, z);
    PxVec3_maxElement :: proc(self_: ^PxVec3) -> _c.float ---

    /// returns absolute values of components;
    PxVec3_abs :: proc(self_: ^PxVec3) -> PxVec3 ---

    PxVec3Padded_new_alloc :: proc() -> ^PxVec3Padded ---

    PxVec3Padded_delete :: proc(self_: ^PxVec3Padded) ---

    PxVec3Padded_new_alloc_1 :: proc(p: ^PxVec3) -> ^PxVec3Padded ---

    PxVec3Padded_new_alloc_2 :: proc(f: _c.float) -> ^PxVec3Padded ---

    /// Default constructor, does not do any initialization.
    PxQuat_new :: proc() -> PxQuat ---

    /// identity constructor
    PxQuat_new_1 :: proc(anon_param0: _c.int32_t) -> PxQuat ---

    /// Constructor from a scalar: sets the real part w to the scalar value, and the imaginary parts (x,y,z) to zero
    PxQuat_new_2 :: proc(r: _c.float) -> PxQuat ---

    /// Constructor. Take note of the order of the elements!
    PxQuat_new_3 :: proc(nx: _c.float, ny: _c.float, nz: _c.float, nw: _c.float) -> PxQuat ---

    /// Creates from angle-axis representation.
    ///
    /// Axis must be normalized!
    ///
    /// Angle is in radians!
    ///
    /// Unit:
    /// Radians
    PxQuat_new_4 :: proc(angleRadians: _c.float, unitAxis: ^PxVec3) -> PxQuat ---

    /// Creates from orientation matrix.
    PxQuat_new_5 :: proc(m: ^PxMat33) -> PxQuat ---

    /// returns true if quat is identity
    PxQuat_isIdentity :: proc(self_: ^PxQuat) -> _c.bool ---

    /// returns true if all elements are finite (not NAN or INF, etc.)
    PxQuat_isFinite :: proc(self_: ^PxQuat) -> _c.bool ---

    /// returns true if finite and magnitude is close to unit
    PxQuat_isUnit :: proc(self_: ^PxQuat) -> _c.bool ---

    /// returns true if finite and magnitude is reasonably close to unit to allow for some accumulation of error vs
    /// isValid
    PxQuat_isSane :: proc(self_: ^PxQuat) -> _c.bool ---

    /// converts this quaternion to angle-axis representation
    PxQuat_toRadiansAndUnitAxis :: proc(self_: ^PxQuat, angle: ^_c.float, axis: ^PxVec3) ---

    /// Gets the angle between this quat and the identity quaternion.
    ///
    /// Unit:
    /// Radians
    PxQuat_getAngle :: proc(self_: ^PxQuat) -> _c.float ---

    /// Gets the angle between this quat and the argument
    ///
    /// Unit:
    /// Radians
    PxQuat_getAngle_1 :: proc(self_: ^PxQuat, q: ^PxQuat) -> _c.float ---

    /// This is the squared 4D vector length, should be 1 for unit quaternions.
    PxQuat_magnitudeSquared :: proc(self_: ^PxQuat) -> _c.float ---

    /// returns the scalar product of this and other.
    PxQuat_dot :: proc(self_: ^PxQuat, v: ^PxQuat) -> _c.float ---

    PxQuat_getNormalized :: proc(self_: ^PxQuat) -> PxQuat ---

    PxQuat_magnitude :: proc(self_: ^PxQuat) -> _c.float ---

    /// maps to the closest unit quaternion.
    PxQuat_normalize_mut :: proc(self_: ^PxQuat) -> _c.float ---

    PxQuat_getConjugate :: proc(self_: ^PxQuat) -> PxQuat ---

    PxQuat_getImaginaryPart :: proc(self_: ^PxQuat) -> PxVec3 ---

    /// brief computes rotation of x-axis
    PxQuat_getBasisVector0 :: proc(self_: ^PxQuat) -> PxVec3 ---

    /// brief computes rotation of y-axis
    PxQuat_getBasisVector1 :: proc(self_: ^PxQuat) -> PxVec3 ---

    /// brief computes rotation of z-axis
    PxQuat_getBasisVector2 :: proc(self_: ^PxQuat) -> PxVec3 ---

    /// rotates passed vec by this (assumed unitary)
    PxQuat_rotate :: proc(self_: ^PxQuat, v: ^PxVec3) -> PxVec3 ---

    /// inverse rotates passed vec by this (assumed unitary)
    PxQuat_rotateInv :: proc(self_: ^PxQuat, v: ^PxVec3) -> PxVec3 ---

    PxTransform_new :: proc() -> PxTransform ---

    PxTransform_new_1 :: proc(position: ^PxVec3) -> PxTransform ---

    PxTransform_new_2 :: proc(anon_param0: _c.int32_t) -> PxTransform ---

    PxTransform_new_3 :: proc(orientation: ^PxQuat) -> PxTransform ---

    PxTransform_new_4 :: proc(x: _c.float, y: _c.float, z: _c.float, aQ: PxQuat) -> PxTransform ---

    PxTransform_new_5 :: proc(p0: ^PxVec3, q0: ^PxQuat) -> PxTransform ---

    PxTransform_new_6 :: proc(m: ^PxMat44) -> PxTransform ---

    PxTransform_getInverse :: proc(self_: ^PxTransform) -> PxTransform ---

    PxTransform_transform :: proc(self_: ^PxTransform, input: ^PxVec3) -> PxVec3 ---

    PxTransform_transformInv :: proc(self_: ^PxTransform, input: ^PxVec3) -> PxVec3 ---

    PxTransform_rotate :: proc(self_: ^PxTransform, input: ^PxVec3) -> PxVec3 ---

    PxTransform_rotateInv :: proc(self_: ^PxTransform, input: ^PxVec3) -> PxVec3 ---

    /// Transform transform to parent (returns compound transform: first src, then *this)
    PxTransform_transform_1 :: proc(self_: ^PxTransform, src: ^PxTransform) -> PxTransform ---

    /// returns true if finite and q is a unit quaternion
    PxTransform_isValid :: proc(self_: ^PxTransform) -> _c.bool ---

    /// returns true if finite and quat magnitude is reasonably close to unit to allow for some accumulation of error
    /// vs isValid
    PxTransform_isSane :: proc(self_: ^PxTransform) -> _c.bool ---

    /// returns true if all elems are finite (not NAN or INF, etc.)
    PxTransform_isFinite :: proc(self_: ^PxTransform) -> _c.bool ---

    /// Transform transform from parent (returns compound transform: first src, then this->inverse)
    PxTransform_transformInv_1 :: proc(self_: ^PxTransform, src: ^PxTransform) -> PxTransform ---

    /// return a normalized transform (i.e. one in which the quaternion has unit magnitude)
    PxTransform_getNormalized :: proc(self_: ^PxTransform) -> PxTransform ---

    /// Default constructor
    PxMat33_new :: proc() -> PxMat33 ---

    /// identity constructor
    PxMat33_new_1 :: proc(anon_param0: _c.int32_t) -> PxMat33 ---

    /// zero constructor
    PxMat33_new_2 :: proc(anon_param0: _c.int32_t) -> PxMat33 ---

    /// Construct from three base vectors
    PxMat33_new_3 :: proc(col0: ^PxVec3, col1: ^PxVec3, col2: ^PxVec3) -> PxMat33 ---

    /// constructor from a scalar, which generates a multiple of the identity matrix
    PxMat33_new_4 :: proc(r: _c.float) -> PxMat33 ---

    /// Construct from float[9]
    PxMat33_new_5 :: proc(values: ^_c.float) -> PxMat33 ---

    /// Construct from a quaternion
    PxMat33_new_6 :: proc(q: ^PxQuat) -> PxMat33 ---

    /// Construct from diagonal, off-diagonals are zero.
    PxMat33_createDiagonal :: proc(d: ^PxVec3) -> PxMat33 ---

    /// Computes the outer product of two vectors
    PxMat33_outer :: proc(a: ^PxVec3, b: ^PxVec3) -> PxMat33 ---

    /// Get transposed matrix
    PxMat33_getTranspose :: proc(self_: ^PxMat33) -> PxMat33 ---

    /// Get the real inverse
    PxMat33_getInverse :: proc(self_: ^PxMat33) -> PxMat33 ---

    /// Get determinant
    PxMat33_getDeterminant :: proc(self_: ^PxMat33) -> _c.float ---

    /// Transform vector by matrix, equal to v' = M*v
    PxMat33_transform :: proc(self_: ^PxMat33, other: ^PxVec3) -> PxVec3 ---

    /// Transform vector by matrix transpose, v' = M^t*v
    PxMat33_transformTranspose :: proc(self_: ^PxMat33, other: ^PxVec3) -> PxVec3 ---

    PxMat33_front :: proc(self_: ^PxMat33) -> ^_c.float ---

    /// Default constructor, not performing any initialization for performance reason.
    ///
    /// Use empty() function below to construct empty bounds.
    PxBounds3_new :: proc() -> PxBounds3 ---

    /// Construct from two bounding points
    PxBounds3_new_1 :: proc(minimum: ^PxVec3, maximum: ^PxVec3) -> PxBounds3 ---

    /// Return empty bounds.
    PxBounds3_empty :: proc() -> PxBounds3 ---

    /// returns the AABB containing v0 and v1.
    PxBounds3_boundsOfPoints :: proc(v0: ^PxVec3, v1: ^PxVec3) -> PxBounds3 ---

    /// returns the AABB from center and extents vectors.
    PxBounds3_centerExtents :: proc(center: ^PxVec3, extent: ^PxVec3) -> PxBounds3 ---

    /// Construct from center, extent, and (not necessarily orthogonal) basis
    PxBounds3_basisExtent :: proc(center: ^PxVec3, basis: ^PxMat33, extent: ^PxVec3) -> PxBounds3 ---

    /// Construct from pose and extent
    PxBounds3_poseExtent :: proc(pose: ^PxTransform, extent: ^PxVec3) -> PxBounds3 ---

    /// gets the transformed bounds of the passed AABB (resulting in a bigger AABB).
    ///
    /// This version is safe to call for empty bounds.
    PxBounds3_transformSafe :: proc(matrix_: ^PxMat33, bounds: ^PxBounds3) -> PxBounds3 ---

    /// gets the transformed bounds of the passed AABB (resulting in a bigger AABB).
    ///
    /// Calling this method for empty bounds leads to undefined behavior. Use [`transformSafe`]() instead.
    PxBounds3_transformFast :: proc(matrix_: ^PxMat33, bounds: ^PxBounds3) -> PxBounds3 ---

    /// gets the transformed bounds of the passed AABB (resulting in a bigger AABB).
    ///
    /// This version is safe to call for empty bounds.
    PxBounds3_transformSafe_1 :: proc(transform: ^PxTransform, bounds: ^PxBounds3) -> PxBounds3 ---

    /// gets the transformed bounds of the passed AABB (resulting in a bigger AABB).
    ///
    /// Calling this method for empty bounds leads to undefined behavior. Use [`transformSafe`]() instead.
    PxBounds3_transformFast_1 :: proc(transform: ^PxTransform, bounds: ^PxBounds3) -> PxBounds3 ---

    /// Sets empty to true
    PxBounds3_setEmpty_mut :: proc(self_: ^PxBounds3) ---

    /// Sets the bounds to maximum size [-PX_MAX_BOUNDS_EXTENTS, PX_MAX_BOUNDS_EXTENTS].
    PxBounds3_setMaximal_mut :: proc(self_: ^PxBounds3) ---

    /// expands the volume to include v
    PxBounds3_include_mut :: proc(self_: ^PxBounds3, v: ^PxVec3) ---

    /// expands the volume to include b.
    PxBounds3_include_mut_1 :: proc(self_: ^PxBounds3, b: ^PxBounds3) ---

    PxBounds3_isEmpty :: proc(self_: ^PxBounds3) -> _c.bool ---

    /// indicates whether the intersection of this and b is empty or not.
    PxBounds3_intersects :: proc(self_: ^PxBounds3, b: ^PxBounds3) -> _c.bool ---

    /// computes the 1D-intersection between two AABBs, on a given axis.
    PxBounds3_intersects1D :: proc(self_: ^PxBounds3, a: ^PxBounds3, axis: _c.uint32_t) -> _c.bool ---

    /// indicates if these bounds contain v.
    PxBounds3_contains :: proc(self_: ^PxBounds3, v: ^PxVec3) -> _c.bool ---

    /// checks a box is inside another box.
    PxBounds3_isInside :: proc(self_: ^PxBounds3, box: ^PxBounds3) -> _c.bool ---

    /// returns the center of this axis aligned box.
    PxBounds3_getCenter :: proc(self_: ^PxBounds3) -> PxVec3 ---

    /// get component of the box's center along a given axis
    PxBounds3_getCenter_1 :: proc(self_: ^PxBounds3, axis: _c.uint32_t) -> _c.float ---

    /// get component of the box's extents along a given axis
    PxBounds3_getExtents :: proc(self_: ^PxBounds3, axis: _c.uint32_t) -> _c.float ---

    /// returns the dimensions (width/height/depth) of this axis aligned box.
    PxBounds3_getDimensions :: proc(self_: ^PxBounds3) -> PxVec3 ---

    /// returns the extents, which are half of the width/height/depth.
    PxBounds3_getExtents_1 :: proc(self_: ^PxBounds3) -> PxVec3 ---

    /// scales the AABB.
    ///
    /// This version is safe to call for empty bounds.
    PxBounds3_scaleSafe_mut :: proc(self_: ^PxBounds3, scale: _c.float) ---

    /// scales the AABB.
    ///
    /// Calling this method for empty bounds leads to undefined behavior. Use [`scaleSafe`]() instead.
    PxBounds3_scaleFast_mut :: proc(self_: ^PxBounds3, scale: _c.float) ---

    /// fattens the AABB in all 3 dimensions by the given distance.
    ///
    /// This version is safe to call for empty bounds.
    PxBounds3_fattenSafe_mut :: proc(self_: ^PxBounds3, distance: _c.float) ---

    /// fattens the AABB in all 3 dimensions by the given distance.
    ///
    /// Calling this method for empty bounds leads to undefined behavior. Use [`fattenSafe`]() instead.
    PxBounds3_fattenFast_mut :: proc(self_: ^PxBounds3, distance: _c.float) ---

    /// checks that the AABB values are not NaN
    PxBounds3_isFinite :: proc(self_: ^PxBounds3) -> _c.bool ---

    /// checks that the AABB values describe a valid configuration.
    PxBounds3_isValid :: proc(self_: ^PxBounds3) -> _c.bool ---

    /// Finds the closest point in the box to the point p. If p is contained, this will be p, otherwise it
    /// will be the closest point on the surface of the box.
    PxBounds3_closestPoint :: proc(self_: ^PxBounds3, p: ^PxVec3) -> PxVec3 ---

    PxErrorCallback_delete :: proc(self_: ^PxErrorCallback) ---

    /// Reports an error code.
    PxErrorCallback_reportError_mut :: proc(self_: ^PxErrorCallback, code: _c.int32_t, message: ^_c.char, file: ^_c.char, line: _c.int32_t) ---

    /// callback when memory is allocated.
    PxAllocationListener_onAllocation_mut :: proc(self_: ^PxAllocationListener, size: _c.size_t, typeName: ^_c.char, filename: ^_c.char, line: _c.int32_t, allocatedMemory: rawptr) ---

    /// callback when memory is deallocated.
    PxAllocationListener_onDeallocation_mut :: proc(self_: ^PxAllocationListener, allocatedMemory: rawptr) ---

    /// The default constructor.
    PxBroadcastingAllocator_new_alloc :: proc(allocator: ^PxAllocatorCallback, error: ^PxErrorCallback) -> ^PxBroadcastingAllocator ---

    /// The default constructor.
    PxBroadcastingAllocator_delete :: proc(self_: ^PxBroadcastingAllocator) ---

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
    PxBroadcastingAllocator_allocate_mut :: proc(self_: ^PxBroadcastingAllocator, size: _c.size_t, typeName: ^_c.char, filename: ^_c.char, line: _c.int32_t) -> rawptr ---

    /// Frees memory previously allocated by allocate().
    ///
    /// Threading:
    /// This function should be thread safe as it can be called in the context of the user thread
    /// and physics processing thread(s).
    PxBroadcastingAllocator_deallocate_mut :: proc(self_: ^PxBroadcastingAllocator, ptr: rawptr) ---

    /// The default constructor.
    PxBroadcastingErrorCallback_new_alloc :: proc(errorCallback: ^PxErrorCallback) -> ^PxBroadcastingErrorCallback ---

    /// The default destructor.
    PxBroadcastingErrorCallback_delete :: proc(self_: ^PxBroadcastingErrorCallback) ---

    /// Reports an error code.
    PxBroadcastingErrorCallback_reportError_mut :: proc(self_: ^PxBroadcastingErrorCallback, code: _c.int32_t, message: ^_c.char, file: ^_c.char, line: _c.int32_t) ---

    /// Enables floating point exceptions for the scalar and SIMD unit
    phys_PxEnableFPExceptions :: proc() ---

    /// Disables floating point exceptions for the scalar and SIMD unit
    phys_PxDisableFPExceptions :: proc() ---

    /// read from the stream. The number of bytes read may be less than the number requested.
    ///
    /// the number of bytes read from the stream.
    PxInputStream_read_mut :: proc(self_: ^PxInputStream, dest: rawptr, count: _c.uint32_t) -> _c.uint32_t ---

    PxInputStream_delete :: proc(self_: ^PxInputStream) ---

    /// return the length of the input data
    ///
    /// size in bytes of the input data
    PxInputData_getLength :: proc(self_: ^PxInputData) -> _c.uint32_t ---

    /// seek to the given offset from the start of the data.
    PxInputData_seek_mut :: proc(self_: ^PxInputData, offset: _c.uint32_t) ---

    /// return the current offset from the start of the data
    ///
    /// the offset to seek to.
    PxInputData_tell :: proc(self_: ^PxInputData) -> _c.uint32_t ---

    PxInputData_delete :: proc(self_: ^PxInputData) ---

    /// write to the stream. The number of bytes written may be less than the number sent.
    ///
    /// the number of bytes written to the stream by this call.
    PxOutputStream_write_mut :: proc(self_: ^PxOutputStream, src: rawptr, count: _c.uint32_t) -> _c.uint32_t ---

    PxOutputStream_delete :: proc(self_: ^PxOutputStream) ---

    /// default constructor leaves data uninitialized.
    PxVec4_new :: proc() -> PxVec4 ---

    /// zero constructor.
    PxVec4_new_1 :: proc(anon_param0: _c.int32_t) -> PxVec4 ---

    /// Assigns scalar parameter to all elements.
    ///
    /// Useful to initialize to zero or one.
    PxVec4_new_2 :: proc(a: _c.float) -> PxVec4 ---

    /// Initializes from 3 scalar parameters.
    PxVec4_new_3 :: proc(nx: _c.float, ny: _c.float, nz: _c.float, nw: _c.float) -> PxVec4 ---

    /// Initializes from 3 scalar parameters.
    PxVec4_new_4 :: proc(v: ^PxVec3, nw: _c.float) -> PxVec4 ---

    /// Initializes from an array of scalar parameters.
    PxVec4_new_5 :: proc(v: ^_c.float) -> PxVec4 ---

    /// tests for exact zero vector
    PxVec4_isZero :: proc(self_: ^PxVec4) -> _c.bool ---

    /// returns true if all 3 elems of the vector are finite (not NAN or INF, etc.)
    PxVec4_isFinite :: proc(self_: ^PxVec4) -> _c.bool ---

    /// is normalized - used by API parameter validation
    PxVec4_isNormalized :: proc(self_: ^PxVec4) -> _c.bool ---

    /// returns the squared magnitude
    ///
    /// Avoids calling PxSqrt()!
    PxVec4_magnitudeSquared :: proc(self_: ^PxVec4) -> _c.float ---

    /// returns the magnitude
    PxVec4_magnitude :: proc(self_: ^PxVec4) -> _c.float ---

    /// returns the scalar product of this and other.
    PxVec4_dot :: proc(self_: ^PxVec4, v: ^PxVec4) -> _c.float ---

    /// returns a unit vector
    PxVec4_getNormalized :: proc(self_: ^PxVec4) -> PxVec4 ---

    /// normalizes the vector in place
    PxVec4_normalize_mut :: proc(self_: ^PxVec4) -> _c.float ---

    /// a[i] * b[i], for all i.
    PxVec4_multiply :: proc(self_: ^PxVec4, a: ^PxVec4) -> PxVec4 ---

    /// element-wise minimum
    PxVec4_minimum :: proc(self_: ^PxVec4, v: ^PxVec4) -> PxVec4 ---

    /// element-wise maximum
    PxVec4_maximum :: proc(self_: ^PxVec4, v: ^PxVec4) -> PxVec4 ---

    PxVec4_getXYZ :: proc(self_: ^PxVec4) -> PxVec3 ---

    /// Default constructor
    PxMat44_new :: proc() -> PxMat44 ---

    /// identity constructor
    PxMat44_new_1 :: proc(anon_param0: _c.int32_t) -> PxMat44 ---

    /// zero constructor
    PxMat44_new_2 :: proc(anon_param0: _c.int32_t) -> PxMat44 ---

    /// Construct from four 4-vectors
    PxMat44_new_3 :: proc(col0: ^PxVec4, col1: ^PxVec4, col2: ^PxVec4, col3: ^PxVec4) -> PxMat44 ---

    /// constructor that generates a multiple of the identity matrix
    PxMat44_new_4 :: proc(r: _c.float) -> PxMat44 ---

    /// Construct from three base vectors and a translation
    PxMat44_new_5 :: proc(col0: ^PxVec3, col1: ^PxVec3, col2: ^PxVec3, col3: ^PxVec3) -> PxMat44 ---

    /// Construct from float[16]
    PxMat44_new_6 :: proc(values: ^_c.float) -> PxMat44 ---

    /// Construct from a quaternion
    PxMat44_new_7 :: proc(q: ^PxQuat) -> PxMat44 ---

    /// Construct from a diagonal vector
    PxMat44_new_8 :: proc(diagonal: ^PxVec4) -> PxMat44 ---

    /// Construct from Mat33 and a translation
    PxMat44_new_9 :: proc(axes: ^PxMat33, position: ^PxVec3) -> PxMat44 ---

    PxMat44_new_10 :: proc(t: ^PxTransform) -> PxMat44 ---

    /// Get transposed matrix
    PxMat44_getTranspose :: proc(self_: ^PxMat44) -> PxMat44 ---

    /// Transform vector by matrix, equal to v' = M*v
    PxMat44_transform :: proc(self_: ^PxMat44, other: ^PxVec4) -> PxVec4 ---

    /// Transform vector by matrix, equal to v' = M*v
    PxMat44_transform_1 :: proc(self_: ^PxMat44, other: ^PxVec3) -> PxVec3 ---

    /// Rotate vector by matrix, equal to v' = M*v
    PxMat44_rotate :: proc(self_: ^PxMat44, other: ^PxVec4) -> PxVec4 ---

    /// Rotate vector by matrix, equal to v' = M*v
    PxMat44_rotate_1 :: proc(self_: ^PxMat44, other: ^PxVec3) -> PxVec3 ---

    PxMat44_getBasis :: proc(self_: ^PxMat44, num: _c.uint32_t) -> PxVec3 ---

    PxMat44_getPosition :: proc(self_: ^PxMat44) -> PxVec3 ---

    PxMat44_setPosition_mut :: proc(self_: ^PxMat44, position: ^PxVec3) ---

    PxMat44_front :: proc(self_: ^PxMat44) -> ^_c.float ---

    PxMat44_scale_mut :: proc(self_: ^PxMat44, p: ^PxVec4) ---

    PxMat44_inverseRT :: proc(self_: ^PxMat44) -> PxMat44 ---

    PxMat44_isFinite :: proc(self_: ^PxMat44) -> _c.bool ---

    /// Constructor
    PxPlane_new :: proc() -> PxPlane ---

    /// Constructor from a normal and a distance
    PxPlane_new_1 :: proc(nx: _c.float, ny: _c.float, nz: _c.float, distance: _c.float) -> PxPlane ---

    /// Constructor from a normal and a distance
    PxPlane_new_2 :: proc(normal: ^PxVec3, distance: _c.float) -> PxPlane ---

    /// Constructor from a point on the plane and a normal
    PxPlane_new_3 :: proc(point: ^PxVec3, normal: ^PxVec3) -> PxPlane ---

    /// Constructor from three points
    PxPlane_new_4 :: proc(p0: ^PxVec3, p1: ^PxVec3, p2: ^PxVec3) -> PxPlane ---

    PxPlane_distance :: proc(self_: ^PxPlane, p: ^PxVec3) -> _c.float ---

    PxPlane_contains :: proc(self_: ^PxPlane, p: ^PxVec3) -> _c.bool ---

    /// projects p into the plane
    PxPlane_project :: proc(self_: ^PxPlane, p: ^PxVec3) -> PxVec3 ---

    /// find an arbitrary point in the plane
    PxPlane_pointInPlane :: proc(self_: ^PxPlane) -> PxVec3 ---

    /// equivalent plane with unit normal
    PxPlane_normalize_mut :: proc(self_: ^PxPlane) ---

    /// transform plane
    PxPlane_transform :: proc(self_: ^PxPlane, pose: ^PxTransform) -> PxPlane ---

    /// inverse-transform plane
    PxPlane_inverseTransform :: proc(self_: ^PxPlane, pose: ^PxTransform) -> PxPlane ---

    /// finds the shortest rotation between two vectors.
    ///
    /// a rotation about an axis normal to the two vectors which takes one to the other via the shortest path
    phys_PxShortestRotation :: proc(from: ^PxVec3, target: ^PxVec3) -> PxQuat ---

    phys_PxDiagonalize :: proc(m: ^PxMat33, axes: ^PxQuat) -> PxVec3 ---

    /// creates a transform from the endpoints of a segment, suitable for an actor transform for a PxCapsuleGeometry
    ///
    /// A PxTransform which will transform the vector (1,0,0) to the capsule axis shrunk by the halfHeight
    phys_PxTransformFromSegment :: proc(p0: ^PxVec3, p1: ^PxVec3, halfHeight: ^_c.float) -> PxTransform ---

    /// creates a transform from a plane equation, suitable for an actor transform for a PxPlaneGeometry
    ///
    /// a PxTransform which will transform the plane PxPlane(1,0,0,0) to the specified plane
    phys_PxTransformFromPlaneEquation :: proc(plane: ^PxPlane) -> PxTransform ---

    /// creates a plane equation from a transform, such as the actor transform for a PxPlaneGeometry
    ///
    /// the plane
    phys_PxPlaneEquationFromTransform :: proc(pose: ^PxTransform) -> PxPlane ---

    /// Spherical linear interpolation of two quaternions.
    ///
    /// Returns left when t=0, right when t=1 and a linear interpolation of left and right when 0
    /// <
    /// t
    /// <
    /// 1.
    /// Returns angle between -PI and PI in radians
    phys_PxSlerp :: proc(t: _c.float, left: ^PxQuat, right: ^PxQuat) -> PxQuat ---

    /// integrate transform.
    phys_PxIntegrateTransform :: proc(curTrans: ^PxTransform, linvel: ^PxVec3, angvel: ^PxVec3, timeStep: _c.float, result: ^PxTransform) ---

    /// Compute the exponent of a PxVec3
    phys_PxExp :: proc(v: ^PxVec3) -> PxQuat ---

    /// computes a oriented bounding box around the scaled basis.
    ///
    /// Bounding box extent.
    phys_PxOptimizeBoundingBox :: proc(basis: ^PxMat33) -> PxVec3 ---

    /// return Returns the log of a PxQuat
    phys_PxLog :: proc(q: ^PxQuat) -> PxVec3 ---

    /// return Returns 0 if v.x is largest element of v, 1 if v.y is largest element, 2 if v.z is largest element.
    phys_PxLargestAxis :: proc(v: ^PxVec3) -> _c.uint32_t ---

    /// Compute tan(theta/2) given sin(theta) and cos(theta) as inputs.
    ///
    /// Returns tan(theta/2)
    phys_PxTanHalf :: proc(sin: _c.float, cos: _c.float) -> _c.float ---

    /// Compute the closest point on an 2d ellipse to a given 2d point.
    ///
    /// Returns the 2d position on the surface of the ellipse that is closest to point.
    phys_PxEllipseClamp :: proc(point: ^PxVec3, radii: ^PxVec3) -> PxVec3 ---

    /// Compute from an input quaternion q a pair of quaternions (swing, twist) such that
    /// q = swing * twist
    /// with the caveats that swing.x = twist.y = twist.z = 0.
    phys_PxSeparateSwingTwist :: proc(q: ^PxQuat, swing: ^PxQuat, twist: ^PxQuat) ---

    /// Compute the angle between two non-unit vectors
    ///
    /// Returns the angle (in radians) between the two vector v0 and v1.
    phys_PxComputeAngle :: proc(v0: ^PxVec3, v1: ^PxVec3) -> _c.float ---

    /// Compute two normalized vectors (right and up) that are perpendicular to an input normalized vector (dir).
    phys_PxComputeBasisVectors :: proc(dir: ^PxVec3, right: ^PxVec3, up: ^PxVec3) ---

    /// Compute three normalized vectors (dir, right and up) that are parallel to (dir) and perpendicular to (right, up) the
    /// normalized direction vector (p1 - p0)/||p1 - p0||.
    phys_PxComputeBasisVectors_1 :: proc(p0: ^PxVec3, p1: ^PxVec3, dir: ^PxVec3, right: ^PxVec3, up: ^PxVec3) ---

    /// Compute (i+1)%3
    phys_PxGetNextIndex3 :: proc(i: _c.uint32_t) -> _c.uint32_t ---

    phys_computeBarycentric :: proc(a: ^PxVec3, b: ^PxVec3, c: ^PxVec3, d: ^PxVec3, p: ^PxVec3, bary: ^PxVec4) ---

    phys_computeBarycentric_1 :: proc(a: ^PxVec3, b: ^PxVec3, c: ^PxVec3, p: ^PxVec3, bary: ^PxVec4) ---

    Interpolation_PxLerp :: proc(a: _c.float, b: _c.float, t: _c.float) -> _c.float ---

    Interpolation_PxBiLerp :: proc(f00: _c.float, f10: _c.float, f01: _c.float, f11: _c.float, tx: _c.float, ty: _c.float) -> _c.float ---

    Interpolation_PxTriLerp :: proc(f000: _c.float, f100: _c.float, f010: _c.float, f110: _c.float, f001: _c.float, f101: _c.float, f011: _c.float, f111: _c.float, tx: _c.float, ty: _c.float, tz: _c.float) -> _c.float ---

    Interpolation_PxSDFIdx :: proc(i: _c.uint32_t, j: _c.uint32_t, k: _c.uint32_t, nbX: _c.uint32_t, nbY: _c.uint32_t) -> _c.uint32_t ---

    Interpolation_PxSDFSampleImpl :: proc(sdf: ^_c.float, localPos: ^PxVec3, sdfBoxLower: ^PxVec3, sdfBoxHigher: ^PxVec3, sdfDx: _c.float, invSdfDx: _c.float, dimX: _c.uint32_t, dimY: _c.uint32_t, dimZ: _c.uint32_t, tolerance: _c.float) -> _c.float ---

    phys_PxSdfSample :: proc(sdf: ^_c.float, localPos: ^PxVec3, sdfBoxLower: ^PxVec3, sdfBoxHigher: ^PxVec3, sdfDx: _c.float, invSdfDx: _c.float, dimX: _c.uint32_t, dimY: _c.uint32_t, dimZ: _c.uint32_t, gradient: ^PxVec3, tolerance: _c.float) -> _c.float ---

    /// The constructor for Mutex creates a mutex. It is initially unlocked.
    PxMutexImpl_new_alloc :: proc() -> ^PxMutexImpl ---

    /// The destructor for Mutex deletes the mutex.
    PxMutexImpl_delete :: proc(self_: ^PxMutexImpl) ---

    /// Acquire (lock) the mutex. If the mutex is already locked
    /// by another thread, this method blocks until the mutex is
    /// unlocked.
    PxMutexImpl_lock_mut :: proc(self_: ^PxMutexImpl) ---

    /// Acquire (lock) the mutex. If the mutex is already locked
    /// by another thread, this method returns false without blocking.
    PxMutexImpl_trylock_mut :: proc(self_: ^PxMutexImpl) -> _c.bool ---

    /// Release (unlock) the mutex.
    PxMutexImpl_unlock_mut :: proc(self_: ^PxMutexImpl) ---

    /// Size of this class.
    PxMutexImpl_getSize :: proc() -> _c.uint32_t ---

    PxReadWriteLock_new_alloc :: proc() -> ^PxReadWriteLock ---

    PxReadWriteLock_delete :: proc(self_: ^PxReadWriteLock) ---

    PxReadWriteLock_lockReader_mut :: proc(self_: ^PxReadWriteLock, takeLock: _c.bool) ---

    PxReadWriteLock_lockWriter_mut :: proc(self_: ^PxReadWriteLock) ---

    PxReadWriteLock_unlockReader_mut :: proc(self_: ^PxReadWriteLock) ---

    PxReadWriteLock_unlockWriter_mut :: proc(self_: ^PxReadWriteLock) ---

    /// Mark the beginning of a nested profile block
    ///
    /// Returns implementation-specific profiler data for this event
    PxProfilerCallback_zoneStart_mut :: proc(self_: ^PxProfilerCallback, eventName: ^_c.char, detached: _c.bool, contextId: _c.uint64_t) -> rawptr ---

    /// Mark the end of a nested profile block
    ///
    /// eventName plus contextId can be used to uniquely match up start and end of a zone.
    PxProfilerCallback_zoneEnd_mut :: proc(self_: ^PxProfilerCallback, profilerData: rawptr, eventName: ^_c.char, detached: _c.bool, contextId: _c.uint64_t) ---

    PxProfileScoped_new_alloc :: proc(callback: ^PxProfilerCallback, eventName: ^_c.char, detached: _c.bool, contextId: _c.uint64_t) -> ^PxProfileScoped ---

    PxProfileScoped_delete :: proc(self_: ^PxProfileScoped) ---

    PxSListEntry_new :: proc() -> PxSListEntry ---

    PxSListEntry_next_mut :: proc(self_: ^PxSListEntry) -> ^PxSListEntry ---

    PxSListImpl_new_alloc :: proc() -> ^PxSListImpl ---

    PxSListImpl_delete :: proc(self_: ^PxSListImpl) ---

    PxSListImpl_push_mut :: proc(self_: ^PxSListImpl, entry: ^PxSListEntry) ---

    PxSListImpl_pop_mut :: proc(self_: ^PxSListImpl) -> ^PxSListEntry ---

    PxSListImpl_flush_mut :: proc(self_: ^PxSListImpl) -> ^PxSListEntry ---

    PxSListImpl_getSize :: proc() -> _c.uint32_t ---

    PxSyncImpl_new_alloc :: proc() -> ^PxSyncImpl ---

    PxSyncImpl_delete :: proc(self_: ^PxSyncImpl) ---

    /// Wait on the object for at most the given number of ms. Returns
    /// true if the object is signaled. Sync::waitForever will block forever
    /// or until the object is signaled.
    PxSyncImpl_wait_mut :: proc(self_: ^PxSyncImpl, milliseconds: _c.uint32_t) -> _c.bool ---

    /// Signal the synchronization object, waking all threads waiting on it
    PxSyncImpl_set_mut :: proc(self_: ^PxSyncImpl) ---

    /// Reset the synchronization object
    PxSyncImpl_reset_mut :: proc(self_: ^PxSyncImpl) ---

    /// Size of this class.
    PxSyncImpl_getSize :: proc() -> _c.uint32_t ---

    PxRunnable_new_alloc :: proc() -> ^PxRunnable ---

    PxRunnable_delete :: proc(self_: ^PxRunnable) ---

    PxRunnable_execute_mut :: proc(self_: ^PxRunnable) ---

    phys_PxTlsAlloc :: proc() -> _c.uint32_t ---

    phys_PxTlsFree :: proc(index: _c.uint32_t) ---

    phys_PxTlsGet :: proc(index: _c.uint32_t) -> rawptr ---

    phys_PxTlsGetValue :: proc(index: _c.uint32_t) -> _c.size_t ---

    phys_PxTlsSet :: proc(index: _c.uint32_t, value: rawptr) -> _c.uint32_t ---

    phys_PxTlsSetValue :: proc(index: _c.uint32_t, value: _c.size_t) -> _c.uint32_t ---

    PxCounterFrequencyToTensOfNanos_new :: proc(inNum: _c.uint64_t, inDenom: _c.uint64_t) -> PxCounterFrequencyToTensOfNanos ---

    PxCounterFrequencyToTensOfNanos_toTensOfNanos :: proc(self_: ^PxCounterFrequencyToTensOfNanos, inCounter: _c.uint64_t) -> _c.uint64_t ---

    PxTime_getBootCounterFrequency :: proc() -> ^PxCounterFrequencyToTensOfNanos ---

    PxTime_getCounterFrequency :: proc() -> PxCounterFrequencyToTensOfNanos ---

    PxTime_getCurrentCounterValue :: proc() -> _c.uint64_t ---

    PxTime_getCurrentTimeInTensOfNanoSeconds :: proc() -> _c.uint64_t ---

    PxTime_new :: proc() -> PxTime ---

    PxTime_getElapsedSeconds_mut :: proc(self_: ^PxTime) -> _c.double ---

    PxTime_peekElapsedSeconds_mut :: proc(self_: ^PxTime) -> _c.double ---

    PxTime_getLastTime :: proc(self_: ^PxTime) -> _c.double ---

    /// default constructor leaves data uninitialized.
    PxVec2_new :: proc() -> PxVec2 ---

    /// zero constructor.
    PxVec2_new_1 :: proc(anon_param0: _c.int32_t) -> PxVec2 ---

    /// Assigns scalar parameter to all elements.
    ///
    /// Useful to initialize to zero or one.
    PxVec2_new_2 :: proc(a: _c.float) -> PxVec2 ---

    /// Initializes from 2 scalar parameters.
    PxVec2_new_3 :: proc(nx: _c.float, ny: _c.float) -> PxVec2 ---

    /// tests for exact zero vector
    PxVec2_isZero :: proc(self_: ^PxVec2) -> _c.bool ---

    /// returns true if all 2 elems of the vector are finite (not NAN or INF, etc.)
    PxVec2_isFinite :: proc(self_: ^PxVec2) -> _c.bool ---

    /// is normalized - used by API parameter validation
    PxVec2_isNormalized :: proc(self_: ^PxVec2) -> _c.bool ---

    /// returns the squared magnitude
    ///
    /// Avoids calling PxSqrt()!
    PxVec2_magnitudeSquared :: proc(self_: ^PxVec2) -> _c.float ---

    /// returns the magnitude
    PxVec2_magnitude :: proc(self_: ^PxVec2) -> _c.float ---

    /// returns the scalar product of this and other.
    PxVec2_dot :: proc(self_: ^PxVec2, v: ^PxVec2) -> _c.float ---

    /// returns a unit vector
    PxVec2_getNormalized :: proc(self_: ^PxVec2) -> PxVec2 ---

    /// normalizes the vector in place
    PxVec2_normalize_mut :: proc(self_: ^PxVec2) -> _c.float ---

    /// a[i] * b[i], for all i.
    PxVec2_multiply :: proc(self_: ^PxVec2, a: ^PxVec2) -> PxVec2 ---

    /// element-wise minimum
    PxVec2_minimum :: proc(self_: ^PxVec2, v: ^PxVec2) -> PxVec2 ---

    /// returns MIN(x, y);
    PxVec2_minElement :: proc(self_: ^PxVec2) -> _c.float ---

    /// element-wise maximum
    PxVec2_maximum :: proc(self_: ^PxVec2, v: ^PxVec2) -> PxVec2 ---

    /// returns MAX(x, y);
    PxVec2_maxElement :: proc(self_: ^PxVec2) -> _c.float ---

    PxStridedData_new :: proc() -> PxStridedData ---

    PxBoundedData_new :: proc() -> PxBoundedData ---

    PxDebugPoint_new :: proc(p: ^PxVec3, c: ^_c.uint32_t) -> PxDebugPoint ---

    PxDebugLine_new :: proc(p0: ^PxVec3, p1: ^PxVec3, c: ^_c.uint32_t) -> PxDebugLine ---

    PxDebugTriangle_new :: proc(p0: ^PxVec3, p1: ^PxVec3, p2: ^PxVec3, c: ^_c.uint32_t) -> PxDebugTriangle ---

    PxDebugText_new :: proc() -> PxDebugText ---

    PxDebugText_new_1 :: proc(pos: ^PxVec3, sz: ^_c.float, clr: ^_c.uint32_t, str: ^_c.char) -> PxDebugText ---

    PxRenderBuffer_delete :: proc(self_: ^PxRenderBuffer) ---

    PxRenderBuffer_getNbPoints :: proc(self_: ^PxRenderBuffer) -> _c.uint32_t ---

    PxRenderBuffer_getPoints :: proc(self_: ^PxRenderBuffer) -> ^PxDebugPoint ---

    PxRenderBuffer_addPoint_mut :: proc(self_: ^PxRenderBuffer, point: ^PxDebugPoint) ---

    PxRenderBuffer_getNbLines :: proc(self_: ^PxRenderBuffer) -> _c.uint32_t ---

    PxRenderBuffer_getLines :: proc(self_: ^PxRenderBuffer) -> ^PxDebugLine ---

    PxRenderBuffer_addLine_mut :: proc(self_: ^PxRenderBuffer, line: ^PxDebugLine) ---

    PxRenderBuffer_reserveLines_mut :: proc(self_: ^PxRenderBuffer, nbLines: _c.uint32_t) -> ^PxDebugLine ---

    PxRenderBuffer_reservePoints_mut :: proc(self_: ^PxRenderBuffer, nbLines: _c.uint32_t) -> ^PxDebugPoint ---

    PxRenderBuffer_getNbTriangles :: proc(self_: ^PxRenderBuffer) -> _c.uint32_t ---

    PxRenderBuffer_getTriangles :: proc(self_: ^PxRenderBuffer) -> ^PxDebugTriangle ---

    PxRenderBuffer_addTriangle_mut :: proc(self_: ^PxRenderBuffer, triangle: ^PxDebugTriangle) ---

    PxRenderBuffer_append_mut :: proc(self_: ^PxRenderBuffer, other: ^PxRenderBuffer) ---

    PxRenderBuffer_clear_mut :: proc(self_: ^PxRenderBuffer) ---

    PxRenderBuffer_shift_mut :: proc(self_: ^PxRenderBuffer, delta: ^PxVec3) ---

    PxRenderBuffer_empty :: proc(self_: ^PxRenderBuffer) -> _c.bool ---

    PxProcessPxBaseCallback_delete :: proc(self_: ^PxProcessPxBaseCallback) ---

    PxProcessPxBaseCallback_process_mut :: proc(self_: ^PxProcessPxBaseCallback, anon_param0: ^PxBase) ---

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
    PxSerializationContext_registerReference_mut :: proc(self_: ^PxSerializationContext, base: ^PxBase, kind: _c.uint32_t, reference: _c.size_t) ---

    /// Returns the collection that is being serialized.
    PxSerializationContext_getCollection :: proc(self_: ^PxSerializationContext) -> ^PxCollection ---

    /// Serializes object data and object extra data.
    ///
    /// This function is assumed to be called within the implementation of PxSerializer::exportData and PxSerializer::exportExtraData.
    PxSerializationContext_writeData_mut :: proc(self_: ^PxSerializationContext, data: rawptr, size: _c.uint32_t) ---

    /// Aligns the serialized data.
    ///
    /// This function is assumed to be called within the implementation of PxSerializer::exportData and PxSerializer::exportExtraData.
    PxSerializationContext_alignData_mut :: proc(self_: ^PxSerializationContext, alignment: _c.uint32_t) ---

    /// Helper function to write a name to the extraData if serialization is configured to save names.
    ///
    /// This function is assumed to be called within the implementation of PxSerializer::exportExtraData.
    PxSerializationContext_writeName_mut :: proc(self_: ^PxSerializationContext, name: ^_c.char) ---

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
    PxDeserializationContext_resolveReference :: proc(self_: ^PxDeserializationContext, kind: _c.uint32_t, reference: _c.size_t) -> ^PxBase ---

    /// Helper function to read a name from the extra data during deserialization.
    ///
    /// This function is assumed to be called within the implementation of PxSerializer::createObject.
    PxDeserializationContext_readName_mut :: proc(self_: ^PxDeserializationContext, name: ^^_c.char) ---

    /// Function to align the extra data stream to a power of 2 alignment
    ///
    /// This function is assumed to be called within the implementation of PxSerializer::createObject.
    PxDeserializationContext_alignExtraData_mut :: proc(self_: ^PxDeserializationContext, alignment: _c.uint32_t) ---

    /// Register a serializer for a concrete type
    PxSerializationRegistry_registerSerializer_mut :: proc(self_: ^PxSerializationRegistry, type: _c.uint16_t, serializer: ^PxSerializer) ---

    /// Unregister a serializer for a concrete type, and retrieves the corresponding serializer object.
    ///
    /// Unregistered serializer corresponding to type, NULL for types for which no serializer has been registered.
    PxSerializationRegistry_unregisterSerializer_mut :: proc(self_: ^PxSerializationRegistry, type: _c.uint16_t) -> ^PxSerializer ---

    /// Returns PxSerializer corresponding to type
    ///
    /// Registered PxSerializer object corresponding to type
    PxSerializationRegistry_getSerializer :: proc(self_: ^PxSerializationRegistry, type: _c.uint16_t) -> ^PxSerializer ---

    /// Register a RepX serializer for a concrete type
    PxSerializationRegistry_registerRepXSerializer_mut :: proc(self_: ^PxSerializationRegistry, type: _c.uint16_t, serializer: ^PxRepXSerializer) ---

    /// Unregister a RepX serializer for a concrete type, and retrieves the corresponding serializer object.
    ///
    /// Unregistered PxRepxSerializer corresponding to type, NULL for types for which no RepX serializer has been registered.
    PxSerializationRegistry_unregisterRepXSerializer_mut :: proc(self_: ^PxSerializationRegistry, type: _c.uint16_t) -> ^PxRepXSerializer ---

    /// Returns RepX serializer given the corresponding type name
    ///
    /// Registered PxRepXSerializer object corresponding to type name
    PxSerializationRegistry_getRepXSerializer :: proc(self_: ^PxSerializationRegistry, typeName: ^_c.char) -> ^PxRepXSerializer ---

    /// Releases PxSerializationRegistry instance.
    ///
    /// This unregisters all PhysX and PhysXExtension serializers. Make sure to unregister all custom type
    /// serializers before releasing the PxSerializationRegistry.
    PxSerializationRegistry_release_mut :: proc(self_: ^PxSerializationRegistry) ---

    /// Adds a PxBase object to the collection.
    ///
    /// Adds a PxBase object to the collection. Optionally a PxSerialObjectId can be provided
    /// in order to resolve dependencies between collections. A PxSerialObjectId value of PX_SERIAL_OBJECT_ID_INVALID
    /// means the object remains without id. Objects can be added regardless of other objects they require. If the object
    /// is already in the collection, the ID will be set if it was PX_SERIAL_OBJECT_ID_INVALID previously, otherwise the
    /// operation fails.
    PxCollection_add_mut :: proc(self_: ^PxCollection, object: ^PxBase, id: _c.uint64_t) ---

    /// Removes a PxBase member object from the collection.
    ///
    /// Object needs to be contained by the collection.
    PxCollection_remove_mut :: proc(self_: ^PxCollection, object: ^PxBase) ---

    /// Returns whether the collection contains a certain PxBase object.
    ///
    /// Whether object is contained.
    PxCollection_contains :: proc(self_: ^PxCollection, object: ^PxBase) -> _c.bool ---

    /// Adds an id to a member PxBase object.
    ///
    /// If the object is already associated with an id within the collection, the id is replaced.
    /// May only be called for objects that are members of the collection. The id needs to be unique
    /// within the collection.
    PxCollection_addId_mut :: proc(self_: ^PxCollection, object: ^PxBase, id: _c.uint64_t) ---

    /// Removes id from a contained PxBase object.
    ///
    /// May only be called for ids that are associated with an object in the collection.
    PxCollection_removeId_mut :: proc(self_: ^PxCollection, id: _c.uint64_t) ---

    /// Adds all PxBase objects and their ids of collection to this collection.
    ///
    /// PxBase objects already in this collection are ignored. Object ids need to be conflict
    /// free, i.e. the same object may not have two different ids within the two collections.
    PxCollection_add_mut_1 :: proc(self_: ^PxCollection, collection: ^PxCollection) ---

    /// Removes all PxBase objects of collection from this collection.
    ///
    /// PxBase objects not present in this collection are ignored. Ids of objects
    /// which are removed are also removed.
    PxCollection_remove_mut_1 :: proc(self_: ^PxCollection, collection: ^PxCollection) ---

    /// Gets number of PxBase objects in this collection.
    ///
    /// Number of objects in this collection
    PxCollection_getNbObjects :: proc(self_: ^PxCollection) -> _c.uint32_t ---

    /// Gets the PxBase object of this collection given its index.
    ///
    /// PxBase object at index index
    PxCollection_getObject :: proc(self_: ^PxCollection, index: _c.uint32_t) -> ^PxBase ---

    /// Copies member PxBase pointers to a user specified buffer.
    ///
    /// number of members PxBase objects that have been written to the userBuffer
    PxCollection_getObjects :: proc(self_: ^PxCollection, userBuffer: ^^PxBase, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Looks for a PxBase object given a PxSerialObjectId value.
    ///
    /// If there is no PxBase object in the collection with the given id, NULL is returned.
    ///
    /// PxBase object with the given id value or NULL
    PxCollection_find :: proc(self_: ^PxCollection, id: _c.uint64_t) -> ^PxBase ---

    /// Gets number of PxSerialObjectId names in this collection.
    ///
    /// Number of PxSerialObjectId names in this collection
    PxCollection_getNbIds :: proc(self_: ^PxCollection) -> _c.uint32_t ---

    /// Copies member PxSerialObjectId values to a user specified buffer.
    ///
    /// number of members PxSerialObjectId values that have been written to the userBuffer
    PxCollection_getIds :: proc(self_: ^PxCollection, userBuffer: ^_c.uint64_t, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Gets the PxSerialObjectId name of a PxBase object within the collection.
    ///
    /// The PxBase object needs to be a member of the collection.
    ///
    /// PxSerialObjectId name of the object or PX_SERIAL_OBJECT_ID_INVALID if the object is unnamed
    PxCollection_getId :: proc(self_: ^PxCollection, object: ^PxBase) -> _c.uint64_t ---

    /// Deletes a collection object.
    ///
    /// This function only deletes the collection object, i.e. the container class. It doesn't delete objects
    /// that are part of the collection.
    PxCollection_release_mut :: proc(self_: ^PxCollection) ---

    /// Creates a collection object.
    ///
    /// Objects can only be serialized or deserialized through a collection.
    /// For serialization, users must add objects to the collection and serialize the collection as a whole.
    /// For deserialization, the system gives back a collection of deserialized objects to users.
    ///
    /// The new collection object.
    phys_PxCreateCollection :: proc() -> ^PxCollection ---

    /// Releases the PxBase instance, please check documentation of release in derived class.
    PxBase_release_mut :: proc(self_: ^PxBase) ---

    /// Returns string name of dynamic type.
    ///
    /// Class name of most derived type of this object.
    PxBase_getConcreteTypeName :: proc(self_: ^PxBase) -> ^_c.char ---

    /// Returns concrete type of object.
    ///
    /// PxConcreteType::Enum of serialized object
    PxBase_getConcreteType :: proc(self_: ^PxBase) -> _c.uint16_t ---

    /// Set PxBaseFlag
    PxBase_setBaseFlag_mut :: proc(self_: ^PxBase, flag: _c.int32_t, value: _c.bool) ---

    /// Set PxBaseFlags
    PxBase_setBaseFlags_mut :: proc(self_: ^PxBase, inFlags: _c.uint16_t) ---

    /// Returns PxBaseFlags
    ///
    /// PxBaseFlags
    PxBase_getBaseFlags :: proc(self_: ^PxBase) -> _c.uint16_t ---

    /// Whether the object is subordinate.
    ///
    /// A class is subordinate, if it can only be instantiated in the context of another class.
    ///
    /// Whether the class is subordinate
    PxBase_isReleasable :: proc(self_: ^PxBase) -> _c.bool ---

    /// Decrements the reference count of the object and releases it if the new reference count is zero.
    PxRefCounted_release_mut :: proc(self_: ^PxRefCounted) ---

    /// Returns the reference count of the object.
    ///
    /// At creation, the reference count of the object is 1. Every other object referencing this object increments the
    /// count by 1. When the reference count reaches 0, and only then, the object gets destroyed automatically.
    ///
    /// the current reference count.
    PxRefCounted_getReferenceCount :: proc(self_: ^PxRefCounted) -> _c.uint32_t ---

    /// Acquires a counted reference to this object.
    ///
    /// This method increases the reference count of the object by 1. Decrement the reference count by calling release()
    PxRefCounted_acquireReference_mut :: proc(self_: ^PxRefCounted) ---

    /// constructor sets to default
    PxTolerancesScale_new :: proc(defaultLength: _c.float, defaultSpeed: _c.float) -> PxTolerancesScale ---

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid (returns always true).
    PxTolerancesScale_isValid :: proc(self_: ^PxTolerancesScale) -> _c.bool ---

    /// Allocate a new string.
    ///
    /// *Always* a valid null terminated string.  "" is returned if "" or null is passed in.
    PxStringTable_allocateStr_mut :: proc(self_: ^PxStringTable, inSrc: ^_c.char) -> ^_c.char ---

    /// Release the string table and all the strings associated with it.
    PxStringTable_release_mut :: proc(self_: ^PxStringTable) ---

    /// Returns string name of dynamic type.
    ///
    /// Class name of most derived type of this object.
    PxSerializer_getConcreteTypeName :: proc(self_: ^PxSerializer) -> ^_c.char ---

    /// Adds required objects to the collection.
    ///
    /// This method does not add the required objects recursively, e.g. objects required by required objects.
    PxSerializer_requiresObjects :: proc(self_: ^PxSerializer, anon_param0: ^PxBase, anon_param1: ^PxProcessPxBaseCallback) ---

    /// Whether the object is subordinate.
    ///
    /// A class is subordinate, if it can only be instantiated in the context of another class.
    ///
    /// Whether the class is subordinate
    PxSerializer_isSubordinate :: proc(self_: ^PxSerializer) -> _c.bool ---

    /// Exports object's extra data to stream.
    PxSerializer_exportExtraData :: proc(self_: ^PxSerializer, anon_param0: ^PxBase, anon_param1: ^PxSerializationContext) ---

    /// Exports object's data to stream.
    PxSerializer_exportData :: proc(self_: ^PxSerializer, anon_param0: ^PxBase, anon_param1: ^PxSerializationContext) ---

    /// Register references that the object maintains to other objects.
    PxSerializer_registerReferences :: proc(self_: ^PxSerializer, obj: ^PxBase, s: ^PxSerializationContext) ---

    /// Returns size needed to create the class instance.
    ///
    /// sizeof class instance.
    PxSerializer_getClassSize :: proc(self_: ^PxSerializer) -> _c.size_t ---

    /// Create object at a given address, resolve references and import extra data.
    ///
    /// Created PxBase pointer (needs to be identical to address before increment).
    PxSerializer_createObject :: proc(self_: ^PxSerializer, address: ^^_c.uint8_t, context_: ^PxDeserializationContext) -> ^PxBase ---

    /// *******************************************************************************************************************
    PxSerializer_delete :: proc(self_: ^PxSerializer) ---

    /// Builds object (TriangleMesh, Heightfield, ConvexMesh or BVH) from given data in PxPhysics.
    ///
    /// PxBase Created object in PxPhysics.
    PxInsertionCallback_buildObjectFromData_mut :: proc(self_: ^PxInsertionCallback, type: _c.int32_t, data: rawptr) -> ^PxBase ---

    /// Set the user-provided dispatcher object for CPU tasks
    PxTaskManager_setCpuDispatcher_mut :: proc(self_: ^PxTaskManager, ref: ^PxCpuDispatcher) ---

    /// Get the user-provided dispatcher object for CPU tasks
    ///
    /// The CPU dispatcher object.
    PxTaskManager_getCpuDispatcher :: proc(self_: ^PxTaskManager) -> ^PxCpuDispatcher ---

    /// Reset any dependencies between Tasks
    ///
    /// Will be called at the start of every frame before tasks are submitted.
    PxTaskManager_resetDependencies_mut :: proc(self_: ^PxTaskManager) ---

    /// Called by the owning scene to start the task graph.
    ///
    /// All tasks with ref count of 1 will be dispatched.
    PxTaskManager_startSimulation_mut :: proc(self_: ^PxTaskManager) ---

    /// Called by the owning scene at the end of a simulation step.
    PxTaskManager_stopSimulation_mut :: proc(self_: ^PxTaskManager) ---

    /// Called by the worker threads to inform the PxTaskManager that a task has completed processing.
    PxTaskManager_taskCompleted_mut :: proc(self_: ^PxTaskManager, task: ^PxTask) ---

    /// Retrieve a task by name
    ///
    /// The ID of the task with that name, or eNOT_PRESENT if not found
    PxTaskManager_getNamedTask_mut :: proc(self_: ^PxTaskManager, name: ^_c.char) -> _c.uint32_t ---

    /// Submit a task with a unique name.
    ///
    /// The ID of the task with that name, or eNOT_PRESENT if not found
    PxTaskManager_submitNamedTask_mut :: proc(self_: ^PxTaskManager, task: ^PxTask, name: ^_c.char, type: _c.int32_t) -> _c.uint32_t ---

    /// Submit an unnamed task.
    ///
    /// The ID of the task with that name, or eNOT_PRESENT if not found
    PxTaskManager_submitUnnamedTask_mut :: proc(self_: ^PxTaskManager, task: ^PxTask, type: _c.int32_t) -> _c.uint32_t ---

    /// Retrieve a task given a task ID
    ///
    /// The task associated with the ID
    PxTaskManager_getTaskFromID_mut :: proc(self_: ^PxTaskManager, id: _c.uint32_t) -> ^PxTask ---

    /// Release the PxTaskManager object, referenced dispatchers will not be released
    PxTaskManager_release_mut :: proc(self_: ^PxTaskManager) ---

    /// Construct a new PxTaskManager instance with the given [optional] dispatchers
    PxTaskManager_createTaskManager :: proc(errorCallback: ^PxErrorCallback, anon_param1: ^PxCpuDispatcher) -> ^PxTaskManager ---

    /// Called by the TaskManager when a task is to be queued for execution.
    ///
    /// Upon receiving a task, the dispatcher should schedule the task to run.
    /// After the task has been run, it should call the release() method and
    /// discard its pointer.
    PxCpuDispatcher_submitTask_mut :: proc(self_: ^PxCpuDispatcher, task: ^PxBaseTask) ---

    /// Returns the number of available worker threads for this dispatcher.
    ///
    /// The SDK will use this count to control how many tasks are submitted. By
    /// matching the number of tasks with the number of execution units task
    /// overhead can be reduced.
    PxCpuDispatcher_getWorkerCount :: proc(self_: ^PxCpuDispatcher) -> _c.uint32_t ---

    PxCpuDispatcher_delete :: proc(self_: ^PxCpuDispatcher) ---

    /// The user-implemented run method where the task's work should be performed
    ///
    /// run() methods must be thread safe, stack friendly (no alloca, etc), and
    /// must never block.
    PxBaseTask_run_mut :: proc(self_: ^PxBaseTask) ---

    /// Return a user-provided task name for profiling purposes.
    ///
    /// It does not have to be unique, but unique names are helpful.
    ///
    /// The name of this task
    PxBaseTask_getName :: proc(self_: ^PxBaseTask) -> ^_c.char ---

    /// Implemented by derived implementation classes
    PxBaseTask_addReference_mut :: proc(self_: ^PxBaseTask) ---

    /// Implemented by derived implementation classes
    PxBaseTask_removeReference_mut :: proc(self_: ^PxBaseTask) ---

    /// Implemented by derived implementation classes
    PxBaseTask_getReference :: proc(self_: ^PxBaseTask) -> _c.int32_t ---

    /// Implemented by derived implementation classes
    ///
    /// A task may assume in its release() method that the task system no longer holds
    /// references to it - so it may safely run its destructor, recycle itself, etc.
    /// provided no additional user references to the task exist
    PxBaseTask_release_mut :: proc(self_: ^PxBaseTask) ---

    /// Return PxTaskManager to which this task was submitted
    ///
    /// Note, can return NULL if task was not submitted, or has been
    /// completed.
    PxBaseTask_getTaskManager :: proc(self_: ^PxBaseTask) -> ^PxTaskManager ---

    PxBaseTask_setContextId_mut :: proc(self_: ^PxBaseTask, id: _c.uint64_t) ---

    PxBaseTask_getContextId :: proc(self_: ^PxBaseTask) -> _c.uint64_t ---

    /// Release method implementation
    PxTask_release_mut :: proc(self_: ^PxTask) ---

    /// Inform the PxTaskManager this task must finish before the given
    PxTask_finishBefore_mut :: proc(self_: ^PxTask, taskID: _c.uint32_t) ---

    /// Inform the PxTaskManager this task cannot start until the given
    PxTask_startAfter_mut :: proc(self_: ^PxTask, taskID: _c.uint32_t) ---

    /// Manually increment this task's reference count. The task will
    /// not be allowed to run until removeReference() is called.
    PxTask_addReference_mut :: proc(self_: ^PxTask) ---

    /// Manually decrement this task's reference count. If the reference
    /// count reaches zero, the task will be dispatched.
    PxTask_removeReference_mut :: proc(self_: ^PxTask) ---

    /// Return the ref-count for this task
    PxTask_getReference :: proc(self_: ^PxTask) -> _c.int32_t ---

    /// Return the unique ID for this task
    PxTask_getTaskID :: proc(self_: ^PxTask) -> _c.uint32_t ---

    /// Called by PxTaskManager at submission time for initialization
    ///
    /// Perform simulation step initialization here.
    PxTask_submitted_mut :: proc(self_: ^PxTask) ---

    /// Initialize this task and specify the task that will have its ref count decremented on completion.
    ///
    /// Submission is deferred until the task's mRefCount is decremented to zero.
    /// Note that we only use the PxTaskManager to query the appropriate dispatcher.
    PxLightCpuTask_setContinuation_mut :: proc(self_: ^PxLightCpuTask, tm: ^PxTaskManager, c: ^PxBaseTask) ---

    /// Initialize this task and specify the task that will have its ref count decremented on completion.
    ///
    /// This overload of setContinuation() queries the PxTaskManager from the continuation
    /// task, which cannot be NULL.
    PxLightCpuTask_setContinuation_mut_1 :: proc(self_: ^PxLightCpuTask, c: ^PxBaseTask) ---

    /// Retrieves continuation task
    PxLightCpuTask_getContinuation :: proc(self_: ^PxLightCpuTask) -> ^PxBaseTask ---

    /// Manually decrement this task's reference count. If the reference
    /// count reaches zero, the task will be dispatched.
    PxLightCpuTask_removeReference_mut :: proc(self_: ^PxLightCpuTask) ---

    /// Return the ref-count for this task
    PxLightCpuTask_getReference :: proc(self_: ^PxLightCpuTask) -> _c.int32_t ---

    /// Manually increment this task's reference count. The task will
    /// not be allowed to run until removeReference() is called.
    PxLightCpuTask_addReference_mut :: proc(self_: ^PxLightCpuTask) ---

    /// called by CpuDispatcher after run method has completed
    ///
    /// Decrements the continuation task's reference count, if specified.
    PxLightCpuTask_release_mut :: proc(self_: ^PxLightCpuTask) ---

    /// Returns the type of the geometry.
    ///
    /// The type of the object.
    PxGeometry_getType :: proc(self_: ^PxGeometry) -> _c.int32_t ---

    /// Constructor to initialize half extents from scalar parameters.
    PxBoxGeometry_new :: proc(hx: _c.float, hy: _c.float, hz: _c.float) -> PxBoxGeometry ---

    /// Constructor to initialize half extents from vector parameter.
    PxBoxGeometry_new_1 :: proc(halfExtents_: PxVec3) -> PxBoxGeometry ---

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid
    ///
    /// A valid box has a positive extent in each direction (halfExtents.x > 0, halfExtents.y > 0, halfExtents.z > 0).
    /// It is illegal to call PxRigidActor::createShape and PxPhysics::createShape with a box that has zero extent in any direction.
    PxBoxGeometry_isValid :: proc(self_: ^PxBoxGeometry) -> _c.bool ---

    PxBVHRaycastCallback_delete :: proc(self_: ^PxBVHRaycastCallback) ---

    PxBVHRaycastCallback_reportHit_mut :: proc(self_: ^PxBVHRaycastCallback, boundsIndex: _c.uint32_t, distance: ^_c.float) -> _c.bool ---

    PxBVHOverlapCallback_delete :: proc(self_: ^PxBVHOverlapCallback) ---

    PxBVHOverlapCallback_reportHit_mut :: proc(self_: ^PxBVHOverlapCallback, boundsIndex: _c.uint32_t) -> _c.bool ---

    PxBVHTraversalCallback_delete :: proc(self_: ^PxBVHTraversalCallback) ---

    PxBVHTraversalCallback_visitNode_mut :: proc(self_: ^PxBVHTraversalCallback, bounds: ^PxBounds3) -> _c.bool ---

    PxBVHTraversalCallback_reportLeaf_mut :: proc(self_: ^PxBVHTraversalCallback, nbPrims: _c.uint32_t, prims: ^_c.uint32_t) -> _c.bool ---

    /// Raycast test against a BVH.
    ///
    /// false if query has been aborted
    PxBVH_raycast :: proc(self_: ^PxBVH, origin: ^PxVec3, unitDir: ^PxVec3, maxDist: _c.float, cb: ^PxBVHRaycastCallback, queryFlags: _c.uint32_t) -> _c.bool ---

    /// Sweep test against a BVH.
    ///
    /// false if query has been aborted
    PxBVH_sweep :: proc(self_: ^PxBVH, geom: ^PxGeometry, pose: ^PxTransform, unitDir: ^PxVec3, maxDist: _c.float, cb: ^PxBVHRaycastCallback, queryFlags: _c.uint32_t) -> _c.bool ---

    /// Overlap test against a BVH.
    ///
    /// false if query has been aborted
    PxBVH_overlap :: proc(self_: ^PxBVH, geom: ^PxGeometry, pose: ^PxTransform, cb: ^PxBVHOverlapCallback, queryFlags: _c.uint32_t) -> _c.bool ---

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
    PxBVH_cull :: proc(self_: ^PxBVH, nbPlanes: _c.uint32_t, planes: ^PxPlane, cb: ^PxBVHOverlapCallback, queryFlags: _c.uint32_t) -> _c.bool ---

    /// Returns the number of bounds in the BVH.
    ///
    /// You can use [`getBounds`]() to retrieve the bounds.
    ///
    /// These are the user-defined bounds passed to the BVH builder, not the internal bounds around each BVH node.
    ///
    /// Number of bounds in the BVH.
    PxBVH_getNbBounds :: proc(self_: ^PxBVH) -> _c.uint32_t ---

    /// Retrieve the read-only bounds in the BVH.
    ///
    /// These are the user-defined bounds passed to the BVH builder, not the internal bounds around each BVH node.
    PxBVH_getBounds :: proc(self_: ^PxBVH) -> ^PxBounds3 ---

    /// Retrieve the bounds in the BVH.
    ///
    /// These bounds can be modified. Call refit() after modifications are done.
    ///
    /// These are the user-defined bounds passed to the BVH builder, not the internal bounds around each BVH node.
    PxBVH_getBoundsForModification_mut :: proc(self_: ^PxBVH) -> ^PxBounds3 ---

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
    PxBVH_refit_mut :: proc(self_: ^PxBVH) ---

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
    PxBVH_updateBounds_mut :: proc(self_: ^PxBVH, boundsIndex: _c.uint32_t, newBounds: ^PxBounds3) -> _c.bool ---

    /// Refits subset of marked nodes.
    ///
    /// This is an alternative to the refit() function, to be called after updateBounds() calls.
    /// See updateBounds() for details.
    PxBVH_partialRefit_mut :: proc(self_: ^PxBVH) ---

    /// Generic BVH traversal function.
    ///
    /// This can be used to implement custom BVH traversal functions if provided ones are not enough.
    /// In particular this can be used to visualize the tree's bounds.
    ///
    /// false if query has been aborted
    PxBVH_traverse :: proc(self_: ^PxBVH, cb: ^PxBVHTraversalCallback) -> _c.bool ---

    PxBVH_getConcreteTypeName :: proc(self_: ^PxBVH) -> ^_c.char ---

    /// Constructor, initializes to a capsule with passed radius and half height.
    PxCapsuleGeometry_new :: proc(radius_: _c.float, halfHeight_: _c.float) -> PxCapsuleGeometry ---

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid.
    ///
    /// A valid capsule has radius > 0, halfHeight >= 0.
    /// It is illegal to call PxRigidActor::createShape and PxPhysics::createShape with a capsule that has zero radius or height.
    PxCapsuleGeometry_isValid :: proc(self_: ^PxCapsuleGeometry) -> _c.bool ---

    /// Returns the number of vertices.
    ///
    /// Number of vertices.
    PxConvexMesh_getNbVertices :: proc(self_: ^PxConvexMesh) -> _c.uint32_t ---

    /// Returns the vertices.
    ///
    /// Array of vertices.
    PxConvexMesh_getVertices :: proc(self_: ^PxConvexMesh) -> ^PxVec3 ---

    /// Returns the index buffer.
    ///
    /// Index buffer.
    PxConvexMesh_getIndexBuffer :: proc(self_: ^PxConvexMesh) -> ^_c.uint8_t ---

    /// Returns the number of polygons.
    ///
    /// Number of polygons.
    PxConvexMesh_getNbPolygons :: proc(self_: ^PxConvexMesh) -> _c.uint32_t ---

    /// Returns the polygon data.
    ///
    /// True if success.
    PxConvexMesh_getPolygonData :: proc(self_: ^PxConvexMesh, index: _c.uint32_t, data: ^PxHullPolygon) -> _c.bool ---

    /// Decrements the reference count of a convex mesh and releases it if the new reference count is zero.
    PxConvexMesh_release_mut :: proc(self_: ^PxConvexMesh) ---

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
    PxConvexMesh_getMassInformation :: proc(self_: ^PxConvexMesh, mass: ^_c.float, localInertia: ^PxMat33, localCenterOfMass: ^PxVec3) ---

    /// Returns the local-space (vertex space) AABB from the convex mesh.
    ///
    /// local-space bounds
    PxConvexMesh_getLocalBounds :: proc(self_: ^PxConvexMesh) -> PxBounds3 ---

    /// Returns the local-space Signed Distance Field for this mesh if it has one.
    ///
    /// local-space SDF.
    PxConvexMesh_getSDF :: proc(self_: ^PxConvexMesh) -> ^_c.float ---

    PxConvexMesh_getConcreteTypeName :: proc(self_: ^PxConvexMesh) -> ^_c.char ---

    /// This method decides whether a convex mesh is gpu compatible. If the total number of vertices are more than 64 or any number of vertices in a polygon is more than 32, or
    /// convex hull data was not cooked with GPU data enabled during cooking or was loaded from a serialized collection, the convex hull is incompatible with GPU collision detection. Otherwise
    /// it is compatible.
    ///
    /// True if the convex hull is gpu compatible
    PxConvexMesh_isGpuCompatible :: proc(self_: ^PxConvexMesh) -> _c.bool ---

    /// Constructor initializes to identity scale.
    PxMeshScale_new :: proc() -> PxMeshScale ---

    /// Constructor from scalar.
    PxMeshScale_new_1 :: proc(r: _c.float) -> PxMeshScale ---

    /// Constructor to initialize to arbitrary scale and identity scale rotation.
    PxMeshScale_new_2 :: proc(s: ^PxVec3) -> PxMeshScale ---

    /// Constructor to initialize to arbitrary scaling.
    PxMeshScale_new_3 :: proc(s: ^PxVec3, r: ^PxQuat) -> PxMeshScale ---

    /// Returns true if the scaling is an identity transformation.
    PxMeshScale_isIdentity :: proc(self_: ^PxMeshScale) -> _c.bool ---

    /// Returns the inverse of this scaling transformation.
    PxMeshScale_getInverse :: proc(self_: ^PxMeshScale) -> PxMeshScale ---

    /// Converts this transformation to a 3x3 matrix representation.
    PxMeshScale_toMat33 :: proc(self_: ^PxMeshScale) -> PxMat33 ---

    /// Returns true if combination of negative scale components will cause the triangle normal to flip. The SDK will flip the normals internally.
    PxMeshScale_hasNegativeDeterminant :: proc(self_: ^PxMeshScale) -> _c.bool ---

    PxMeshScale_transform :: proc(self_: ^PxMeshScale, v: ^PxVec3) -> PxVec3 ---

    PxMeshScale_isValidForTriangleMesh :: proc(self_: ^PxMeshScale) -> _c.bool ---

    PxMeshScale_isValidForConvexMesh :: proc(self_: ^PxMeshScale) -> _c.bool ---

    /// Constructor. By default creates an empty object with a NULL mesh and identity scale.
    PxConvexMeshGeometry_new :: proc(mesh: ^PxConvexMesh, scaling: ^PxMeshScale, flags: _c.uint8_t) -> PxConvexMeshGeometry ---

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid for shape creation.
    ///
    /// A valid convex mesh has a positive scale value in each direction (scale.x > 0, scale.y > 0, scale.z > 0).
    /// It is illegal to call PxRigidActor::createShape and PxPhysics::createShape with a convex that has zero extent in any direction.
    PxConvexMeshGeometry_isValid :: proc(self_: ^PxConvexMeshGeometry) -> _c.bool ---

    /// Constructor.
    PxSphereGeometry_new :: proc(ir: _c.float) -> PxSphereGeometry ---

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid
    ///
    /// A valid sphere has radius > 0.
    /// It is illegal to call PxRigidActor::createShape and PxPhysics::createShape with a sphere that has zero radius.
    PxSphereGeometry_isValid :: proc(self_: ^PxSphereGeometry) -> _c.bool ---

    /// Constructor.
    PxPlaneGeometry_new :: proc() -> PxPlaneGeometry ---

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid
    PxPlaneGeometry_isValid :: proc(self_: ^PxPlaneGeometry) -> _c.bool ---

    /// Constructor. By default creates an empty object with a NULL mesh and identity scale.
    PxTriangleMeshGeometry_new :: proc(mesh: ^PxTriangleMesh, scaling: ^PxMeshScale, flags: _c.uint8_t) -> PxTriangleMeshGeometry ---

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid for shape creation.
    ///
    /// A valid triangle mesh has a positive scale value in each direction (scale.scale.x > 0, scale.scale.y > 0, scale.scale.z > 0).
    /// It is illegal to call PxRigidActor::createShape and PxPhysics::createShape with a triangle mesh that has zero extents in any direction.
    PxTriangleMeshGeometry_isValid :: proc(self_: ^PxTriangleMeshGeometry) -> _c.bool ---

    /// Constructor.
    PxHeightFieldGeometry_new :: proc(hf: ^PxHeightField, flags: _c.uint8_t, heightScale_: _c.float, rowScale_: _c.float, columnScale_: _c.float) -> PxHeightFieldGeometry ---

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid
    ///
    /// A valid height field has a positive scale value in each direction (heightScale > 0, rowScale > 0, columnScale > 0).
    /// It is illegal to call PxRigidActor::createShape and PxPhysics::createShape with a height field that has zero extents in any direction.
    PxHeightFieldGeometry_isValid :: proc(self_: ^PxHeightFieldGeometry) -> _c.bool ---

    /// Default constructor.
    ///
    /// Creates an empty object with no particles.
    PxParticleSystemGeometry_new :: proc() -> PxParticleSystemGeometry ---

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid for shape creation.
    PxParticleSystemGeometry_isValid :: proc(self_: ^PxParticleSystemGeometry) -> _c.bool ---

    /// Default constructor.
    PxHairSystemGeometry_new :: proc() -> PxHairSystemGeometry ---

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid for shape creation.
    PxHairSystemGeometry_isValid :: proc(self_: ^PxHairSystemGeometry) -> _c.bool ---

    /// Constructor. By default creates an empty object with a NULL mesh and identity scale.
    PxTetrahedronMeshGeometry_new :: proc(mesh: ^PxTetrahedronMesh) -> PxTetrahedronMeshGeometry ---

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid for shape creation.
    ///
    /// A valid tetrahedron mesh has a positive scale value in each direction (scale.scale.x > 0, scale.scale.y > 0, scale.scale.z > 0).
    /// It is illegal to call PxRigidActor::createShape and PxPhysics::createShape with a tetrahedron mesh that has zero extents in any direction.
    PxTetrahedronMeshGeometry_isValid :: proc(self_: ^PxTetrahedronMeshGeometry) -> _c.bool ---

    PxQueryHit_new :: proc() -> PxQueryHit ---

    PxLocationHit_new :: proc() -> PxLocationHit ---

    /// For raycast hits: true for shapes overlapping with raycast origin.
    ///
    /// For sweep hits: true for shapes overlapping at zero sweep distance.
    PxLocationHit_hadInitialOverlap :: proc(self_: ^PxLocationHit) -> _c.bool ---

    PxGeomRaycastHit_new :: proc() -> PxGeomRaycastHit ---

    PxGeomOverlapHit_new :: proc() -> PxGeomOverlapHit ---

    PxGeomSweepHit_new :: proc() -> PxGeomSweepHit ---

    PxGeomIndexPair_new :: proc() -> PxGeomIndexPair ---

    PxGeomIndexPair_new_1 :: proc(_id0: _c.uint32_t, _id1: _c.uint32_t) -> PxGeomIndexPair ---

    /// For internal use
    phys_PxCustomGeometry_getUniqueID :: proc() -> _c.uint32_t ---

    /// Default constructor
    PxCustomGeometryType_new :: proc() -> PxCustomGeometryType ---

    /// Invalid type
    PxCustomGeometryType_INVALID :: proc() -> PxCustomGeometryType ---

    /// Return custom type. The type purpose is for user to differentiate custom geometries. Not used by PhysX.
    ///
    /// Unique ID of a custom geometry type.
    ///
    /// User should use DECLARE_CUSTOM_GEOMETRY_TYPE and IMPLEMENT_CUSTOM_GEOMETRY_TYPE intead of overwriting this function.
    PxCustomGeometryCallbacks_getCustomType :: proc(self_: ^PxCustomGeometryCallbacks) -> PxCustomGeometryType ---

    /// Return local bounds.
    ///
    /// Bounding box in the geometry local space.
    PxCustomGeometryCallbacks_getLocalBounds :: proc(self_: ^PxCustomGeometryCallbacks, geometry: ^PxGeometry) -> PxBounds3 ---

    /// Raycast. Cast a ray against the geometry in given pose.
    ///
    /// Number of hits.
    PxCustomGeometryCallbacks_raycast :: proc(self_: ^PxCustomGeometryCallbacks, origin: ^PxVec3, unitDir: ^PxVec3, geom: ^PxGeometry, pose: ^PxTransform, maxDist: _c.float, hitFlags: _c.uint16_t, maxHits: _c.uint32_t, rayHits: ^PxGeomRaycastHit, stride: _c.uint32_t, threadContext: ^PxQueryThreadContext) -> _c.uint32_t ---

    /// Overlap. Test if geometries overlap.
    ///
    /// True if there is overlap. False otherwise.
    PxCustomGeometryCallbacks_overlap :: proc(self_: ^PxCustomGeometryCallbacks, geom0: ^PxGeometry, pose0: ^PxTransform, geom1: ^PxGeometry, pose1: ^PxTransform, threadContext: ^PxQueryThreadContext) -> _c.bool ---

    /// Sweep. Sweep one geometry against the other.
    ///
    /// True if there is hit. False otherwise.
    PxCustomGeometryCallbacks_sweep :: proc(self_: ^PxCustomGeometryCallbacks, unitDir: ^PxVec3, maxDist: _c.float, geom0: ^PxGeometry, pose0: ^PxTransform, geom1: ^PxGeometry, pose1: ^PxTransform, sweepHit: ^PxGeomSweepHit, hitFlags: _c.uint16_t, inflation: _c.float, threadContext: ^PxQueryThreadContext) -> _c.bool ---

    /// Compute custom geometry mass properties. For geometries usable with dynamic rigidbodies.
    PxCustomGeometryCallbacks_computeMassProperties :: proc(self_: ^PxCustomGeometryCallbacks, geometry: ^PxGeometry, massProperties: ^PxMassProperties) ---

    /// Compatible with PhysX's PCM feature. Allows to optimize contact generation.
    PxCustomGeometryCallbacks_usePersistentContactManifold :: proc(self_: ^PxCustomGeometryCallbacks, geometry: ^PxGeometry, breakingThreshold: ^_c.float) -> _c.bool ---

    PxCustomGeometryCallbacks_delete :: proc(self_: ^PxCustomGeometryCallbacks) ---

    /// Default constructor.
    ///
    /// Creates an empty object with a NULL callbacks pointer.
    PxCustomGeometry_new :: proc() -> PxCustomGeometry ---

    /// Constructor.
    PxCustomGeometry_new_1 :: proc(_callbacks: ^PxCustomGeometryCallbacks) -> PxCustomGeometry ---

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid for shape creation.
    PxCustomGeometry_isValid :: proc(self_: ^PxCustomGeometry) -> _c.bool ---

    /// Returns the custom type of the custom geometry.
    PxCustomGeometry_getCustomType :: proc(self_: ^PxCustomGeometry) -> PxCustomGeometryType ---

    PxGeometryHolder_getType :: proc(self_: ^PxGeometryHolder) -> _c.int32_t ---

    PxGeometryHolder_any_mut :: proc(self_: ^PxGeometryHolder) -> ^PxGeometry ---

    PxGeometryHolder_any :: proc(self_: ^PxGeometryHolder) -> ^PxGeometry ---

    PxGeometryHolder_sphere_mut :: proc(self_: ^PxGeometryHolder) -> ^PxSphereGeometry ---

    PxGeometryHolder_sphere :: proc(self_: ^PxGeometryHolder) -> ^PxSphereGeometry ---

    PxGeometryHolder_plane_mut :: proc(self_: ^PxGeometryHolder) -> ^PxPlaneGeometry ---

    PxGeometryHolder_plane :: proc(self_: ^PxGeometryHolder) -> ^PxPlaneGeometry ---

    PxGeometryHolder_capsule_mut :: proc(self_: ^PxGeometryHolder) -> ^PxCapsuleGeometry ---

    PxGeometryHolder_capsule :: proc(self_: ^PxGeometryHolder) -> ^PxCapsuleGeometry ---

    PxGeometryHolder_box_mut :: proc(self_: ^PxGeometryHolder) -> ^PxBoxGeometry ---

    PxGeometryHolder_box :: proc(self_: ^PxGeometryHolder) -> ^PxBoxGeometry ---

    PxGeometryHolder_convexMesh_mut :: proc(self_: ^PxGeometryHolder) -> ^PxConvexMeshGeometry ---

    PxGeometryHolder_convexMesh :: proc(self_: ^PxGeometryHolder) -> ^PxConvexMeshGeometry ---

    PxGeometryHolder_tetMesh_mut :: proc(self_: ^PxGeometryHolder) -> ^PxTetrahedronMeshGeometry ---

    PxGeometryHolder_tetMesh :: proc(self_: ^PxGeometryHolder) -> ^PxTetrahedronMeshGeometry ---

    PxGeometryHolder_triangleMesh_mut :: proc(self_: ^PxGeometryHolder) -> ^PxTriangleMeshGeometry ---

    PxGeometryHolder_triangleMesh :: proc(self_: ^PxGeometryHolder) -> ^PxTriangleMeshGeometry ---

    PxGeometryHolder_heightField_mut :: proc(self_: ^PxGeometryHolder) -> ^PxHeightFieldGeometry ---

    PxGeometryHolder_heightField :: proc(self_: ^PxGeometryHolder) -> ^PxHeightFieldGeometry ---

    PxGeometryHolder_particleSystem_mut :: proc(self_: ^PxGeometryHolder) -> ^PxParticleSystemGeometry ---

    PxGeometryHolder_particleSystem :: proc(self_: ^PxGeometryHolder) -> ^PxParticleSystemGeometry ---

    PxGeometryHolder_hairSystem_mut :: proc(self_: ^PxGeometryHolder) -> ^PxHairSystemGeometry ---

    PxGeometryHolder_hairSystem :: proc(self_: ^PxGeometryHolder) -> ^PxHairSystemGeometry ---

    PxGeometryHolder_custom_mut :: proc(self_: ^PxGeometryHolder) -> ^PxCustomGeometry ---

    PxGeometryHolder_custom :: proc(self_: ^PxGeometryHolder) -> ^PxCustomGeometry ---

    PxGeometryHolder_storeAny_mut :: proc(self_: ^PxGeometryHolder, geometry: ^PxGeometry) ---

    PxGeometryHolder_new :: proc() -> PxGeometryHolder ---

    PxGeometryHolder_new_1 :: proc(geometry: ^PxGeometry) -> PxGeometryHolder ---

    /// Raycast test against a geometry object.
    ///
    /// All geometry types are supported except PxParticleSystemGeometry, PxTetrahedronMeshGeometry and PxHairSystemGeometry.
    ///
    /// Number of hits between the ray and the geometry object
    PxGeometryQuery_raycast :: proc(origin: ^PxVec3, unitDir: ^PxVec3, geom: ^PxGeometry, pose: ^PxTransform, maxDist: _c.float, hitFlags: _c.uint16_t, maxHits: _c.uint32_t, rayHits: ^PxGeomRaycastHit, stride: _c.uint32_t, queryFlags: _c.uint32_t, threadContext: ^PxQueryThreadContext) -> _c.uint32_t ---

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
    PxGeometryQuery_overlap :: proc(geom0: ^PxGeometry, pose0: ^PxTransform, geom1: ^PxGeometry, pose1: ^PxTransform, queryFlags: _c.uint32_t, threadContext: ^PxQueryThreadContext) -> _c.bool ---

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
    PxGeometryQuery_sweep :: proc(unitDir: ^PxVec3, maxDist: _c.float, geom0: ^PxGeometry, pose0: ^PxTransform, geom1: ^PxGeometry, pose1: ^PxTransform, sweepHit: ^PxGeomSweepHit, hitFlags: _c.uint16_t, inflation: _c.float, queryFlags: _c.uint32_t, threadContext: ^PxQueryThreadContext) -> _c.bool ---

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
    PxGeometryQuery_computePenetration :: proc(direction: ^PxVec3, depth: ^_c.float, geom0: ^PxGeometry, pose0: ^PxTransform, geom1: ^PxGeometry, pose1: ^PxTransform, queryFlags: _c.uint32_t) -> _c.bool ---

    /// Computes distance between a point and a geometry object.
    ///
    /// Currently supported geometry objects: box, sphere, capsule, convex, mesh.
    ///
    /// For meshes, only the BVH34 midphase data-structure is supported.
    ///
    /// Square distance between the point and the geom object, or 0.0 if the point is inside the object, or -1.0 if an error occured (geometry type is not supported, or invalid pose)
    PxGeometryQuery_pointDistance :: proc(point: ^PxVec3, geom: ^PxGeometry, pose: ^PxTransform, closestPoint: ^PxVec3, closestIndex: ^_c.uint32_t, queryFlags: _c.uint32_t) -> _c.float ---

    /// computes the bounds for a geometry object
    PxGeometryQuery_computeGeomBounds :: proc(bounds: ^PxBounds3, geom: ^PxGeometry, pose: ^PxTransform, offset: _c.float, inflation: _c.float, queryFlags: _c.uint32_t) ---

    /// Checks if provided geometry is valid.
    ///
    /// True if geometry is valid.
    PxGeometryQuery_isValid :: proc(geom: ^PxGeometry) -> _c.bool ---

    PxHeightFieldSample_tessFlag :: proc(self_: ^PxHeightFieldSample) -> _c.uint8_t ---

    PxHeightFieldSample_setTessFlag_mut :: proc(self_: ^PxHeightFieldSample) ---

    PxHeightFieldSample_clearTessFlag_mut :: proc(self_: ^PxHeightFieldSample) ---

    /// Decrements the reference count of a height field and releases it if the new reference count is zero.
    PxHeightField_release_mut :: proc(self_: ^PxHeightField) ---

    /// Writes out the sample data array.
    ///
    /// The user provides destBufferSize bytes storage at destBuffer.
    /// The data is formatted and arranged as PxHeightFieldDesc.samples.
    ///
    /// The number of bytes written.
    PxHeightField_saveCells :: proc(self_: ^PxHeightField, destBuffer: rawptr, destBufferSize: _c.uint32_t) -> _c.uint32_t ---

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
    PxHeightField_modifySamples_mut :: proc(self_: ^PxHeightField, startCol: _c.int32_t, startRow: _c.int32_t, subfieldDesc: ^PxHeightFieldDesc, shrinkBounds: _c.bool) -> _c.bool ---

    /// Retrieves the number of sample rows in the samples array.
    ///
    /// The number of sample rows in the samples array.
    PxHeightField_getNbRows :: proc(self_: ^PxHeightField) -> _c.uint32_t ---

    /// Retrieves the number of sample columns in the samples array.
    ///
    /// The number of sample columns in the samples array.
    PxHeightField_getNbColumns :: proc(self_: ^PxHeightField) -> _c.uint32_t ---

    /// Retrieves the format of the sample data.
    ///
    /// The format of the sample data.
    PxHeightField_getFormat :: proc(self_: ^PxHeightField) -> _c.int32_t ---

    /// Retrieves the offset in bytes between consecutive samples in the array.
    ///
    /// The offset in bytes between consecutive samples in the array.
    PxHeightField_getSampleStride :: proc(self_: ^PxHeightField) -> _c.uint32_t ---

    /// Retrieves the convex edge threshold.
    ///
    /// The convex edge threshold.
    PxHeightField_getConvexEdgeThreshold :: proc(self_: ^PxHeightField) -> _c.float ---

    /// Retrieves the flags bits, combined from values of the enum ::PxHeightFieldFlag.
    ///
    /// The flags bits, combined from values of the enum ::PxHeightFieldFlag.
    PxHeightField_getFlags :: proc(self_: ^PxHeightField) -> _c.uint16_t ---

    /// Retrieves the height at the given coordinates in grid space.
    ///
    /// The height at the given coordinates or 0 if the coordinates are out of range.
    PxHeightField_getHeight :: proc(self_: ^PxHeightField, x: _c.float, z: _c.float) -> _c.float ---

    /// Returns material table index of given triangle
    ///
    /// This function takes a post cooking triangle index.
    ///
    /// Material table index, or 0xffff if no per-triangle materials are used
    PxHeightField_getTriangleMaterialIndex :: proc(self_: ^PxHeightField, triangleIndex: _c.uint32_t) -> _c.uint16_t ---

    /// Returns a triangle face normal for a given triangle index
    ///
    /// This function takes a post cooking triangle index.
    ///
    /// Triangle normal for a given triangle index
    PxHeightField_getTriangleNormal :: proc(self_: ^PxHeightField, triangleIndex: _c.uint32_t) -> PxVec3 ---

    /// Returns heightfield sample of given row and column
    ///
    /// Heightfield sample
    PxHeightField_getSample :: proc(self_: ^PxHeightField, row: _c.uint32_t, column: _c.uint32_t) -> ^PxHeightFieldSample ---

    /// Returns the number of times the heightfield data has been modified
    ///
    /// This method returns the number of times modifySamples has been called on this heightfield, so that code that has
    /// retained state that depends on the heightfield can efficiently determine whether it has been modified.
    ///
    /// the number of times the heightfield sample data has been modified.
    PxHeightField_getTimestamp :: proc(self_: ^PxHeightField) -> _c.uint32_t ---

    PxHeightField_getConcreteTypeName :: proc(self_: ^PxHeightField) -> ^_c.char ---

    /// Constructor sets to default.
    PxHeightFieldDesc_new :: proc() -> PxHeightFieldDesc ---

    /// (re)sets the structure to the default.
    PxHeightFieldDesc_setToDefault_mut :: proc(self_: ^PxHeightFieldDesc) ---

    /// Returns true if the descriptor is valid.
    ///
    /// True if the current settings are valid.
    PxHeightFieldDesc_isValid :: proc(self_: ^PxHeightFieldDesc) -> _c.bool ---

    /// Retrieves triangle data from a triangle ID.
    ///
    /// This function can be used together with [`findOverlapTriangleMesh`]() to retrieve triangle properties.
    ///
    /// This function will flip the triangle normal whenever triGeom.scale.hasNegativeDeterminant() is true.
    PxMeshQuery_getTriangle :: proc(triGeom: ^PxTriangleMeshGeometry, transform: ^PxTransform, triangleIndex: _c.uint32_t, triangle: ^PxTriangle, vertexIndices: ^_c.uint32_t, adjacencyIndices: ^_c.uint32_t) ---

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
    PxMeshQuery_getTriangle_1 :: proc(hfGeom: ^PxHeightFieldGeometry, transform: ^PxTransform, triangleIndex: _c.uint32_t, triangle: ^PxTriangle, vertexIndices: ^_c.uint32_t, adjacencyIndices: ^_c.uint32_t) ---

    /// Find the mesh triangles which touch the specified geometry object.
    ///
    /// For mesh-vs-mesh overlap tests, please use the specialized function below.
    ///
    /// Returned triangle indices can be used with [`getTriangle`]() to retrieve the triangle properties.
    ///
    /// Number of overlaps found, i.e. number of elements written to the results buffer
    PxMeshQuery_findOverlapTriangleMesh :: proc(geom: ^PxGeometry, geomPose: ^PxTransform, meshGeom: ^PxTriangleMeshGeometry, meshPose: ^PxTransform, results: ^_c.uint32_t, maxResults: _c.uint32_t, startIndex: _c.uint32_t, overflow: ^_c.bool, queryFlags: _c.uint32_t) -> _c.uint32_t ---

    /// Find the height field triangles which touch the specified geometry object.
    ///
    /// Returned triangle indices can be used with [`getTriangle`]() to retrieve the triangle properties.
    ///
    /// Number of overlaps found, i.e. number of elements written to the results buffer
    PxMeshQuery_findOverlapHeightField :: proc(geom: ^PxGeometry, geomPose: ^PxTransform, hfGeom: ^PxHeightFieldGeometry, hfPose: ^PxTransform, results: ^_c.uint32_t, maxResults: _c.uint32_t, startIndex: _c.uint32_t, overflow: ^_c.bool, queryFlags: _c.uint32_t) -> _c.uint32_t ---

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
    PxMeshQuery_sweep :: proc(unitDir: ^PxVec3, distance: _c.float, geom: ^PxGeometry, pose: ^PxTransform, triangleCount: _c.uint32_t, triangles: ^PxTriangle, sweepHit: ^PxGeomSweepHit, hitFlags: _c.uint16_t, cachedIndex: ^_c.uint32_t, inflation: _c.float, doubleSided: _c.bool, queryFlags: _c.uint32_t) -> _c.bool ---

    /// constructor sets to default.
    PxSimpleTriangleMesh_new :: proc() -> PxSimpleTriangleMesh ---

    /// (re)sets the structure to the default.
    PxSimpleTriangleMesh_setToDefault_mut :: proc(self_: ^PxSimpleTriangleMesh) ---

    /// returns true if the current settings are valid
    PxSimpleTriangleMesh_isValid :: proc(self_: ^PxSimpleTriangleMesh) -> _c.bool ---

    /// Constructor
    PxTriangle_new_alloc :: proc() -> ^PxTriangle ---

    /// Constructor
    PxTriangle_new_alloc_1 :: proc(p0: ^PxVec3, p1: ^PxVec3, p2: ^PxVec3) -> ^PxTriangle ---

    /// Destructor
    PxTriangle_delete :: proc(self_: ^PxTriangle) ---

    /// Compute the normal of the Triangle.
    PxTriangle_normal :: proc(self_: ^PxTriangle, _normal: ^PxVec3) ---

    /// Compute the unnormalized normal of the triangle.
    PxTriangle_denormalizedNormal :: proc(self_: ^PxTriangle, _normal: ^PxVec3) ---

    /// Compute the area of the triangle.
    ///
    /// Area of the triangle.
    PxTriangle_area :: proc(self_: ^PxTriangle) -> _c.float ---

    /// Computes a point on the triangle from u and v barycentric coordinates.
    PxTriangle_pointFromUV :: proc(self_: ^PxTriangle, u: _c.float, v: _c.float) -> PxVec3 ---

    PxTrianglePadded_new_alloc :: proc() -> ^PxTrianglePadded ---

    PxTrianglePadded_delete :: proc(self_: ^PxTrianglePadded) ---

    /// Returns the number of vertices.
    ///
    /// number of vertices
    PxTriangleMesh_getNbVertices :: proc(self_: ^PxTriangleMesh) -> _c.uint32_t ---

    /// Returns the vertices.
    ///
    /// array of vertices
    PxTriangleMesh_getVertices :: proc(self_: ^PxTriangleMesh) -> ^PxVec3 ---

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
    PxTriangleMesh_getVerticesForModification_mut :: proc(self_: ^PxTriangleMesh) -> ^PxVec3 ---

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
    PxTriangleMesh_refitBVH_mut :: proc(self_: ^PxTriangleMesh) -> PxBounds3 ---

    /// Returns the number of triangles.
    ///
    /// number of triangles
    PxTriangleMesh_getNbTriangles :: proc(self_: ^PxTriangleMesh) -> _c.uint32_t ---

    /// Returns the triangle indices.
    ///
    /// The indices can be 16 or 32bit depending on the number of triangles in the mesh.
    /// Call getTriangleMeshFlags() to know if the indices are 16 or 32 bits.
    ///
    /// The number of indices is the number of triangles * 3.
    ///
    /// array of triangles
    PxTriangleMesh_getTriangles :: proc(self_: ^PxTriangleMesh) -> rawptr ---

    /// Reads the PxTriangleMesh flags.
    ///
    /// See the list of flags [`PxTriangleMeshFlag`]
    ///
    /// The values of the PxTriangleMesh flags.
    PxTriangleMesh_getTriangleMeshFlags :: proc(self_: ^PxTriangleMesh) -> _c.uint8_t ---

    /// Returns the triangle remapping table.
    ///
    /// The triangles are internally sorted according to various criteria. Hence the internal triangle order
    /// does not always match the original (user-defined) order. The remapping table helps finding the old
    /// indices knowing the new ones:
    ///
    /// remapTable[ internalTriangleIndex ] = originalTriangleIndex
    ///
    /// the remapping table (or NULL if 'PxCookingParams::suppressTriangleMeshRemapTable' has been used)
    PxTriangleMesh_getTrianglesRemap :: proc(self_: ^PxTriangleMesh) -> ^_c.uint32_t ---

    /// Decrements the reference count of a triangle mesh and releases it if the new reference count is zero.
    PxTriangleMesh_release_mut :: proc(self_: ^PxTriangleMesh) ---

    /// Returns material table index of given triangle
    ///
    /// This function takes a post cooking triangle index.
    ///
    /// Material table index, or 0xffff if no per-triangle materials are used
    PxTriangleMesh_getTriangleMaterialIndex :: proc(self_: ^PxTriangleMesh, triangleIndex: _c.uint32_t) -> _c.uint16_t ---

    /// Returns the local-space (vertex space) AABB from the triangle mesh.
    ///
    /// local-space bounds
    PxTriangleMesh_getLocalBounds :: proc(self_: ^PxTriangleMesh) -> PxBounds3 ---

    /// Returns the local-space Signed Distance Field for this mesh if it has one.
    ///
    /// local-space SDF.
    PxTriangleMesh_getSDF :: proc(self_: ^PxTriangleMesh) -> ^_c.float ---

    /// Returns the resolution of the local-space dense SDF.
    PxTriangleMesh_getSDFDimensions :: proc(self_: ^PxTriangleMesh, numX: ^_c.uint32_t, numY: ^_c.uint32_t, numZ: ^_c.uint32_t) ---

    /// Sets whether this mesh should be preferred for SDF projection.
    ///
    /// By default, meshes are flagged as preferring projection and the decisions on which mesh to project is based on the triangle and vertex
    /// count. The model with the fewer triangles is projected onto the SDF of the more detailed mesh.
    /// If one of the meshes is set to prefer SDF projection (default) and the other is set to not prefer SDF projection, model flagged as
    /// preferring SDF projection will be projected onto the model flagged as not preferring, regardless of the detail of the respective meshes.
    /// Where both models are flagged as preferring no projection, the less detailed model will be projected as before.
    PxTriangleMesh_setPreferSDFProjection_mut :: proc(self_: ^PxTriangleMesh, preferProjection: _c.bool) ---

    /// Returns whether this mesh prefers SDF projection.
    ///
    /// whether this mesh prefers SDF projection.
    PxTriangleMesh_getPreferSDFProjection :: proc(self_: ^PxTriangleMesh) -> _c.bool ---

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
    PxTriangleMesh_getMassInformation :: proc(self_: ^PxTriangleMesh, mass: ^_c.float, localInertia: ^PxMat33, localCenterOfMass: ^PxVec3) ---

    /// Constructor
    PxTetrahedron_new_alloc :: proc() -> ^PxTetrahedron ---

    /// Constructor
    PxTetrahedron_new_alloc_1 :: proc(p0: ^PxVec3, p1: ^PxVec3, p2: ^PxVec3, p3: ^PxVec3) -> ^PxTetrahedron ---

    /// Destructor
    PxTetrahedron_delete :: proc(self_: ^PxTetrahedron) ---

    /// Decrements the reference count of a tetrahedron mesh and releases it if the new reference count is zero.
    PxSoftBodyAuxData_release_mut :: proc(self_: ^PxSoftBodyAuxData) ---

    /// Returns the number of vertices.
    ///
    /// number of vertices
    PxTetrahedronMesh_getNbVertices :: proc(self_: ^PxTetrahedronMesh) -> _c.uint32_t ---

    /// Returns the vertices
    ///
    /// array of vertices
    PxTetrahedronMesh_getVertices :: proc(self_: ^PxTetrahedronMesh) -> ^PxVec3 ---

    /// Returns the number of tetrahedrons.
    ///
    /// number of tetrahedrons
    PxTetrahedronMesh_getNbTetrahedrons :: proc(self_: ^PxTetrahedronMesh) -> _c.uint32_t ---

    /// Returns the tetrahedron indices.
    ///
    /// The indices can be 16 or 32bit depending on the number of tetrahedrons in the mesh.
    /// Call getTetrahedronMeshFlags() to know if the indices are 16 or 32 bits.
    ///
    /// The number of indices is the number of tetrahedrons * 4.
    ///
    /// array of tetrahedrons
    PxTetrahedronMesh_getTetrahedrons :: proc(self_: ^PxTetrahedronMesh) -> rawptr ---

    /// Reads the PxTetrahedronMesh flags.
    ///
    /// See the list of flags [`PxTetrahedronMeshFlags`]
    ///
    /// The values of the PxTetrahedronMesh flags.
    PxTetrahedronMesh_getTetrahedronMeshFlags :: proc(self_: ^PxTetrahedronMesh) -> _c.uint8_t ---

    /// Returns the tetrahedra remapping table.
    ///
    /// The tetrahedra are internally sorted according to various criteria. Hence the internal tetrahedron order
    /// does not always match the original (user-defined) order. The remapping table helps finding the old
    /// indices knowing the new ones:
    ///
    /// remapTable[ internalTetrahedronIndex ] = originalTetrahedronIndex
    ///
    /// the remapping table (or NULL if 'PxCookingParams::suppressTriangleMeshRemapTable' has been used)
    PxTetrahedronMesh_getTetrahedraRemap :: proc(self_: ^PxTetrahedronMesh) -> ^_c.uint32_t ---

    /// Returns the local-space (vertex space) AABB from the tetrahedron mesh.
    ///
    /// local-space bounds
    PxTetrahedronMesh_getLocalBounds :: proc(self_: ^PxTetrahedronMesh) -> PxBounds3 ---

    /// Decrements the reference count of a tetrahedron mesh and releases it if the new reference count is zero.
    PxTetrahedronMesh_release_mut :: proc(self_: ^PxTetrahedronMesh) ---

    /// Const accecssor to the softbody's collision mesh.
    PxSoftBodyMesh_getCollisionMesh :: proc(self_: ^PxSoftBodyMesh) -> ^PxTetrahedronMesh ---

    /// Accecssor to the softbody's collision mesh.
    PxSoftBodyMesh_getCollisionMesh_mut :: proc(self_: ^PxSoftBodyMesh) -> ^PxTetrahedronMesh ---

    /// Const accessor to the softbody's simulation mesh.
    PxSoftBodyMesh_getSimulationMesh :: proc(self_: ^PxSoftBodyMesh) -> ^PxTetrahedronMesh ---

    /// Accecssor to the softbody's simulation mesh.
    PxSoftBodyMesh_getSimulationMesh_mut :: proc(self_: ^PxSoftBodyMesh) -> ^PxTetrahedronMesh ---

    /// Const accessor to the softbodies simulation state.
    PxSoftBodyMesh_getSoftBodyAuxData :: proc(self_: ^PxSoftBodyMesh) -> ^PxSoftBodyAuxData ---

    /// Accessor to the softbody's auxilary data like mass and rest pose information
    PxSoftBodyMesh_getSoftBodyAuxData_mut :: proc(self_: ^PxSoftBodyMesh) -> ^PxSoftBodyAuxData ---

    /// Decrements the reference count of a tetrahedron mesh and releases it if the new reference count is zero.
    PxSoftBodyMesh_release_mut :: proc(self_: ^PxSoftBodyMesh) ---

    PxCollisionMeshMappingData_release_mut :: proc(self_: ^PxCollisionMeshMappingData) ---

    PxCollisionTetrahedronMeshData_getMesh :: proc(self_: ^PxCollisionTetrahedronMeshData) -> ^PxTetrahedronMeshData ---

    PxCollisionTetrahedronMeshData_getMesh_mut :: proc(self_: ^PxCollisionTetrahedronMeshData) -> ^PxTetrahedronMeshData ---

    PxCollisionTetrahedronMeshData_getData :: proc(self_: ^PxCollisionTetrahedronMeshData) -> ^PxSoftBodyCollisionData ---

    PxCollisionTetrahedronMeshData_getData_mut :: proc(self_: ^PxCollisionTetrahedronMeshData) -> ^PxSoftBodyCollisionData ---

    PxCollisionTetrahedronMeshData_release_mut :: proc(self_: ^PxCollisionTetrahedronMeshData) ---

    PxSimulationTetrahedronMeshData_getMesh_mut :: proc(self_: ^PxSimulationTetrahedronMeshData) -> ^PxTetrahedronMeshData ---

    PxSimulationTetrahedronMeshData_getData_mut :: proc(self_: ^PxSimulationTetrahedronMeshData) -> ^PxSoftBodySimulationData ---

    PxSimulationTetrahedronMeshData_release_mut :: proc(self_: ^PxSimulationTetrahedronMeshData) ---

    /// Deletes the actor.
    ///
    /// Do not keep a reference to the deleted instance.
    ///
    /// If the actor belongs to a [`PxAggregate`] object, it is automatically removed from the aggregate.
    PxActor_release_mut :: proc(self_: ^PxActor) ---

    /// Retrieves the type of actor.
    ///
    /// The actor type of the actor.
    PxActor_getType :: proc(self_: ^PxActor) -> _c.int32_t ---

    /// Retrieves the scene which this actor belongs to.
    ///
    /// Owner Scene. NULL if not part of a scene.
    PxActor_getScene :: proc(self_: ^PxActor) -> ^PxScene ---

    /// Sets a name string for the object that can be retrieved with getName().
    ///
    /// This is for debugging and is not used by the SDK. The string is not copied by the SDK,
    /// only the pointer is stored.
    ///
    /// Default:
    /// NULL
    PxActor_setName_mut :: proc(self_: ^PxActor, name: ^_c.char) ---

    /// Retrieves the name string set with setName().
    ///
    /// Name string associated with object.
    PxActor_getName :: proc(self_: ^PxActor) -> ^_c.char ---

    /// Retrieves the axis aligned bounding box enclosing the actor.
    ///
    /// It is not allowed to use this method while the simulation is running (except during PxScene::collide(),
    /// in PxContactModifyCallback or in contact report callbacks).
    ///
    /// The actor's bounding box.
    PxActor_getWorldBounds :: proc(self_: ^PxActor, inflation: _c.float) -> PxBounds3 ---

    /// Raises or clears a particular actor flag.
    ///
    /// See the list of flags [`PxActorFlag`]
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the actor up automatically.
    PxActor_setActorFlag_mut :: proc(self_: ^PxActor, flag: _c.int32_t, value: _c.bool) ---

    /// Sets the actor flags.
    ///
    /// See the list of flags [`PxActorFlag`]
    PxActor_setActorFlags_mut :: proc(self_: ^PxActor, inFlags: _c.uint8_t) ---

    /// Reads the PxActor flags.
    ///
    /// See the list of flags [`PxActorFlag`]
    ///
    /// The values of the PxActor flags.
    PxActor_getActorFlags :: proc(self_: ^PxActor) -> _c.uint8_t ---

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
    PxActor_setDominanceGroup_mut :: proc(self_: ^PxActor, dominanceGroup: _c.uint8_t) ---

    /// Retrieves the value set with setDominanceGroup().
    ///
    /// The dominance group of this actor.
    PxActor_getDominanceGroup :: proc(self_: ^PxActor) -> _c.uint8_t ---

    /// Sets the owner client of an actor.
    ///
    /// This cannot be done once the actor has been placed into a scene.
    ///
    /// Default:
    /// PX_DEFAULT_CLIENT
    PxActor_setOwnerClient_mut :: proc(self_: ^PxActor, inClient: _c.uint8_t) ---

    /// Returns the owner client that was specified at creation time.
    ///
    /// This value cannot be changed once the object is placed into the scene.
    PxActor_getOwnerClient :: proc(self_: ^PxActor) -> _c.uint8_t ---

    /// Retrieves the aggregate the actor might be a part of.
    ///
    /// The aggregate the actor is a part of, or NULL if the actor does not belong to an aggregate.
    PxActor_getAggregate :: proc(self_: ^PxActor) -> ^PxAggregate ---

    phys_PxGetAggregateFilterHint :: proc(type: _c.int32_t, enableSelfCollision: _c.bool) -> _c.uint32_t ---

    phys_PxGetAggregateSelfCollisionBit :: proc(hint: _c.uint32_t) -> _c.uint32_t ---

    phys_PxGetAggregateType :: proc(hint: _c.uint32_t) -> _c.int32_t ---

    /// Deletes the aggregate object.
    ///
    /// Deleting the PxAggregate object does not delete the aggregated actors. If the PxAggregate object
    /// belongs to a scene, the aggregated actors are automatically re-inserted in that scene. If you intend
    /// to delete both the PxAggregate and its actors, it is best to release the actors first, then release
    /// the PxAggregate when it is empty.
    PxAggregate_release_mut :: proc(self_: ^PxAggregate) ---

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
    PxAggregate_addActor_mut :: proc(self_: ^PxAggregate, actor: ^PxActor, bvh: ^PxBVH) -> _c.bool ---

    /// Removes an actor from the aggregate object.
    ///
    /// A warning is output if the incoming actor does not belong to the aggregate. Otherwise the actor is
    /// removed from the aggregate. If the aggregate belongs to a scene, the actor is reinserted in that
    /// scene. If you intend to delete the actor, it is best to call [`PxActor::release`]() directly. That way
    /// the actor will be automatically removed from its aggregate (if any) and not reinserted in a scene.
    PxAggregate_removeActor_mut :: proc(self_: ^PxAggregate, actor: ^PxActor) -> _c.bool ---

    /// Adds an articulation to the aggregate object.
    ///
    /// A warning is output if the total number of actors is reached (every articulation link counts as an actor),
    /// or if the incoming articulation already belongs to an aggregate.
    ///
    /// If the aggregate belongs to a scene, adding an articulation to the aggregate also adds the articulation to that scene.
    ///
    /// If the articulation already belongs to a scene, a warning is output and the call is ignored. You need to remove
    /// the articulation from the scene first, before adding it to the aggregate.
    PxAggregate_addArticulation_mut :: proc(self_: ^PxAggregate, articulation: ^PxArticulationReducedCoordinate) -> _c.bool ---

    /// Removes an articulation from the aggregate object.
    ///
    /// A warning is output if the incoming articulation does not belong to the aggregate. Otherwise the articulation is
    /// removed from the aggregate. If the aggregate belongs to a scene, the articulation is reinserted in that
    /// scene. If you intend to delete the articulation, it is best to call [`PxArticulationReducedCoordinate::release`]() directly. That way
    /// the articulation will be automatically removed from its aggregate (if any) and not reinserted in a scene.
    PxAggregate_removeArticulation_mut :: proc(self_: ^PxAggregate, articulation: ^PxArticulationReducedCoordinate) -> _c.bool ---

    /// Returns the number of actors contained in the aggregate.
    ///
    /// You can use [`getActors`]() to retrieve the actor pointers.
    ///
    /// Number of actors contained in the aggregate.
    PxAggregate_getNbActors :: proc(self_: ^PxAggregate) -> _c.uint32_t ---

    /// Retrieves max amount of shapes that can be contained in the aggregate.
    ///
    /// Max shape size.
    PxAggregate_getMaxNbShapes :: proc(self_: ^PxAggregate) -> _c.uint32_t ---

    /// Retrieve all actors contained in the aggregate.
    ///
    /// You can retrieve the number of actor pointers by calling [`getNbActors`]()
    ///
    /// Number of actor pointers written to the buffer.
    PxAggregate_getActors :: proc(self_: ^PxAggregate, userBuffer: ^^PxActor, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Retrieves the scene which this aggregate belongs to.
    ///
    /// Owner Scene. NULL if not part of a scene.
    PxAggregate_getScene_mut :: proc(self_: ^PxAggregate) -> ^PxScene ---

    /// Retrieves aggregate's self-collision flag.
    ///
    /// self-collision flag
    PxAggregate_getSelfCollision :: proc(self_: ^PxAggregate) -> _c.bool ---

    PxAggregate_getConcreteTypeName :: proc(self_: ^PxAggregate) -> ^_c.char ---

    PxConstraintInvMassScale_new :: proc() -> PxConstraintInvMassScale ---

    PxConstraintInvMassScale_new_1 :: proc(lin0: _c.float, ang0: _c.float, lin1: _c.float, ang1: _c.float) -> PxConstraintInvMassScale ---

    /// Visualize joint frames
    PxConstraintVisualizer_visualizeJointFrames_mut :: proc(self_: ^PxConstraintVisualizer, parent: ^PxTransform, child: ^PxTransform) ---

    /// Visualize joint linear limit
    PxConstraintVisualizer_visualizeLinearLimit_mut :: proc(self_: ^PxConstraintVisualizer, t0: ^PxTransform, t1: ^PxTransform, value: _c.float, active: _c.bool) ---

    /// Visualize joint angular limit
    PxConstraintVisualizer_visualizeAngularLimit_mut :: proc(self_: ^PxConstraintVisualizer, t0: ^PxTransform, lower: _c.float, upper: _c.float, active: _c.bool) ---

    /// Visualize limit cone
    PxConstraintVisualizer_visualizeLimitCone_mut :: proc(self_: ^PxConstraintVisualizer, t: ^PxTransform, tanQSwingY: _c.float, tanQSwingZ: _c.float, active: _c.bool) ---

    /// Visualize joint double cone
    PxConstraintVisualizer_visualizeDoubleCone_mut :: proc(self_: ^PxConstraintVisualizer, t: ^PxTransform, angle: _c.float, active: _c.bool) ---

    /// Visualize line
    PxConstraintVisualizer_visualizeLine_mut :: proc(self_: ^PxConstraintVisualizer, p0: ^PxVec3, p1: ^PxVec3, color: _c.uint32_t) ---

    /// Pre-simulation data preparation
    /// when the constraint is marked dirty, this function is called at the start of the simulation
    /// step for the SDK to copy the constraint data block.
    PxConstraintConnector_prepareData_mut :: proc(self_: ^PxConstraintConnector) -> rawptr ---

    /// Constraint release callback
    ///
    /// When the SDK deletes a PxConstraint object this function is called by the SDK. In general
    /// custom constraints should not be deleted directly by applications: rather, the constraint
    /// should respond to a release() request by calling PxConstraint::release(), then wait for
    /// this call to release its own resources.
    ///
    /// This function is also called when a PxConstraint object is deleted on cleanup due to
    /// destruction of the PxPhysics object.
    PxConstraintConnector_onConstraintRelease_mut :: proc(self_: ^PxConstraintConnector) ---

    /// Center-of-mass shift callback
    ///
    /// This function is called by the SDK when the CoM of one of the actors is moved. Since the
    /// API specifies constraint positions relative to actors, and the constraint shader functions
    /// are supplied with coordinates relative to bodies, some synchronization is usually required
    /// when the application moves an object's center of mass.
    PxConstraintConnector_onComShift_mut :: proc(self_: ^PxConstraintConnector, actor: _c.uint32_t) ---

    /// Origin shift callback
    ///
    /// This function is called by the SDK when the scene origin gets shifted and allows to adjust
    /// custom data which contains world space transforms.
    ///
    /// If the adjustments affect constraint shader data, it is necessary to call PxConstraint::markDirty()
    /// to make sure that the data gets synced at the beginning of the next simulation step.
    PxConstraintConnector_onOriginShift_mut :: proc(self_: ^PxConstraintConnector, shift: ^PxVec3) ---

    /// Obtain a reference to a PxBase interface if the constraint has one.
    ///
    /// If the constraint does not implement the PxBase interface, it should return NULL.
    PxConstraintConnector_getSerializable_mut :: proc(self_: ^PxConstraintConnector) -> ^PxBase ---

    /// Obtain the pointer to the constraint's constant data
    PxConstraintConnector_getConstantBlock :: proc(self_: ^PxConstraintConnector) -> rawptr ---

    /// Let the connector know it has been connected to a constraint.
    PxConstraintConnector_connectToConstraint_mut :: proc(self_: ^PxConstraintConnector, anon_param0: ^PxConstraint) ---

    /// virtual destructor
    PxConstraintConnector_delete :: proc(self_: ^PxConstraintConnector) ---

    PxSolverBody_new :: proc() -> PxSolverBody ---

    PxSolverBodyData_projectVelocity :: proc(self_: ^PxSolverBodyData, lin: ^PxVec3, ang: ^PxVec3) -> _c.float ---

    PxSolverConstraintPrepDesc_delete :: proc(self_: ^PxSolverConstraintPrepDesc) ---

    /// Allocates constraint data. It is the application's responsibility to release this memory after PxSolveConstraints has completed.
    ///
    /// The allocated memory. This address must be 16-byte aligned.
    PxConstraintAllocator_reserveConstraintData_mut :: proc(self_: ^PxConstraintAllocator, byteSize: _c.uint32_t) -> ^_c.uint8_t ---

    /// Allocates friction data. Friction data can be retained by the application for a given pair and provided as an input to PxSolverContactDesc to improve simulation stability.
    /// It is the application's responsibility to release this memory. If this memory is released, the application should ensure it does not pass pointers to this memory to PxSolverContactDesc.
    ///
    /// The allocated memory. This address must be 4-byte aligned.
    PxConstraintAllocator_reserveFrictionData_mut :: proc(self_: ^PxConstraintAllocator, byteSize: _c.uint32_t) -> ^_c.uint8_t ---

    PxConstraintAllocator_delete :: proc(self_: ^PxConstraintAllocator) ---

    PxArticulationLimit_new :: proc() -> PxArticulationLimit ---

    PxArticulationLimit_new_1 :: proc(low_: _c.float, high_: _c.float) -> PxArticulationLimit ---

    PxArticulationDrive_new :: proc() -> PxArticulationDrive ---

    PxArticulationDrive_new_1 :: proc(stiffness_: _c.float, damping_: _c.float, maxForce_: _c.float, driveType_: _c.int32_t) -> PxArticulationDrive ---

    PxTGSSolverBodyVel_projectVelocity :: proc(self_: ^PxTGSSolverBodyVel, lin: ^PxVec3, ang: ^PxVec3) -> _c.float ---

    PxTGSSolverBodyData_projectVelocity :: proc(self_: ^PxTGSSolverBodyData, linear: ^PxVec3, angular: ^PxVec3) -> _c.float ---

    PxTGSSolverConstraintPrepDesc_delete :: proc(self_: ^PxTGSSolverConstraintPrepDesc) ---

    /// Sets the spring rest length for the sub-tendon from the root to this leaf attachment.
    ///
    /// Setting this on non-leaf attachments has no effect.
    PxArticulationAttachment_setRestLength_mut :: proc(self_: ^PxArticulationAttachment, restLength: _c.float) ---

    /// Gets the spring rest length for the sub-tendon from the root to this leaf attachment.
    ///
    /// The rest length.
    PxArticulationAttachment_getRestLength :: proc(self_: ^PxArticulationAttachment) -> _c.float ---

    /// Sets the low and high limit on the length of the sub-tendon from the root to this leaf attachment.
    ///
    /// Setting this on non-leaf attachments has no effect.
    PxArticulationAttachment_setLimitParameters_mut :: proc(self_: ^PxArticulationAttachment, parameters: ^PxArticulationTendonLimit) ---

    /// Gets the low and high limit on the length of the sub-tendon from the root to this leaf attachment.
    ///
    /// Struct with the low and high limit.
    PxArticulationAttachment_getLimitParameters :: proc(self_: ^PxArticulationAttachment) -> PxArticulationTendonLimit ---

    /// Sets the attachment's relative offset in the link actor frame.
    PxArticulationAttachment_setRelativeOffset_mut :: proc(self_: ^PxArticulationAttachment, offset: ^PxVec3) ---

    /// Gets the attachment's relative offset in the link actor frame.
    ///
    /// The relative offset in the link actor frame.
    PxArticulationAttachment_getRelativeOffset :: proc(self_: ^PxArticulationAttachment) -> PxVec3 ---

    /// Sets the attachment coefficient.
    PxArticulationAttachment_setCoefficient_mut :: proc(self_: ^PxArticulationAttachment, coefficient: _c.float) ---

    /// Gets the attachment coefficient.
    ///
    /// The scale that the distance between this attachment and its parent is multiplied by when summing up the spatial tendon's length.
    PxArticulationAttachment_getCoefficient :: proc(self_: ^PxArticulationAttachment) -> _c.float ---

    /// Gets the articulation link.
    ///
    /// The articulation link that this attachment is attached to.
    PxArticulationAttachment_getLink :: proc(self_: ^PxArticulationAttachment) -> ^PxArticulationLink ---

    /// Gets the parent attachment.
    ///
    /// The parent attachment.
    PxArticulationAttachment_getParent :: proc(self_: ^PxArticulationAttachment) -> ^PxArticulationAttachment ---

    /// Indicates that this attachment is a leaf, and thus defines a sub-tendon from the root to this attachment.
    ///
    /// True: This attachment is a leaf and has zero children; False: Not a leaf.
    PxArticulationAttachment_isLeaf :: proc(self_: ^PxArticulationAttachment) -> _c.bool ---

    /// Gets the spatial tendon that the attachment is a part of.
    ///
    /// The tendon.
    PxArticulationAttachment_getTendon :: proc(self_: ^PxArticulationAttachment) -> ^PxArticulationSpatialTendon ---

    /// Releases the attachment.
    ///
    /// Releasing the attachment is not allowed while the articulation is in a scene. In order to
    /// release the attachment, remove and then re-add the articulation to the scene.
    PxArticulationAttachment_release_mut :: proc(self_: ^PxArticulationAttachment) ---

    /// Returns the string name of the dynamic type.
    ///
    /// The string name.
    PxArticulationAttachment_getConcreteTypeName :: proc(self_: ^PxArticulationAttachment) -> ^_c.char ---

    /// Sets the tendon joint coefficient.
    ///
    /// RecipCoefficient is commonly expected to be 1/coefficient, but it can be set to different values to tune behavior; for example, zero can be used to
    /// have a joint axis only participate in the length computation of the tendon, but not have any tendon force applied to it.
    PxArticulationTendonJoint_setCoefficient_mut :: proc(self_: ^PxArticulationTendonJoint, axis: _c.int32_t, coefficient: _c.float, recipCoefficient: _c.float) ---

    /// Gets the tendon joint coefficient.
    PxArticulationTendonJoint_getCoefficient :: proc(self_: ^PxArticulationTendonJoint, axis: ^_c.int32_t, coefficient: ^_c.float, recipCoefficient: ^_c.float) ---

    /// Gets the articulation link.
    ///
    /// The articulation link (and its incoming joint in particular) that this tendon joint is associated with.
    PxArticulationTendonJoint_getLink :: proc(self_: ^PxArticulationTendonJoint) -> ^PxArticulationLink ---

    /// Gets the parent tendon joint.
    ///
    /// The parent tendon joint.
    PxArticulationTendonJoint_getParent :: proc(self_: ^PxArticulationTendonJoint) -> ^PxArticulationTendonJoint ---

    /// Gets the tendon that the joint is a part of.
    ///
    /// The tendon.
    PxArticulationTendonJoint_getTendon :: proc(self_: ^PxArticulationTendonJoint) -> ^PxArticulationFixedTendon ---

    /// Releases a tendon joint.
    ///
    /// Releasing a tendon joint is not allowed while the articulation is in a scene. In order to
    /// release the joint, remove and then re-add the articulation to the scene.
    PxArticulationTendonJoint_release_mut :: proc(self_: ^PxArticulationTendonJoint) ---

    /// Returns the string name of the dynamic type.
    ///
    /// The string name.
    PxArticulationTendonJoint_getConcreteTypeName :: proc(self_: ^PxArticulationTendonJoint) -> ^_c.char ---

    /// Sets the spring stiffness term acting on the tendon length.
    PxArticulationTendon_setStiffness_mut :: proc(self_: ^PxArticulationTendon, stiffness: _c.float) ---

    /// Gets the spring stiffness of the tendon.
    ///
    /// The spring stiffness.
    PxArticulationTendon_getStiffness :: proc(self_: ^PxArticulationTendon) -> _c.float ---

    /// Sets the damping term acting both on the tendon length and tendon-length limits.
    PxArticulationTendon_setDamping_mut :: proc(self_: ^PxArticulationTendon, damping: _c.float) ---

    /// Gets the damping term acting both on the tendon length and tendon-length limits.
    ///
    /// The damping term.
    PxArticulationTendon_getDamping :: proc(self_: ^PxArticulationTendon) -> _c.float ---

    /// Sets the limit stiffness term acting on the tendon's length limits.
    ///
    /// For spatial tendons, this parameter applies to all its leaf attachments / sub-tendons.
    PxArticulationTendon_setLimitStiffness_mut :: proc(self_: ^PxArticulationTendon, stiffness: _c.float) ---

    /// Gets the limit stiffness term acting on the tendon's length limits.
    ///
    /// For spatial tendons, this parameter applies to all its leaf attachments / sub-tendons.
    ///
    /// The limit stiffness term.
    PxArticulationTendon_getLimitStiffness :: proc(self_: ^PxArticulationTendon) -> _c.float ---

    /// Sets the length offset term for the tendon.
    ///
    /// An offset defines an amount to be added to the accumulated length computed for the tendon. It allows the
    /// application to actuate the tendon by shortening or lengthening it.
    PxArticulationTendon_setOffset_mut :: proc(self_: ^PxArticulationTendon, offset: _c.float, autowake: _c.bool) ---

    /// Gets the length offset term for the tendon.
    ///
    /// The offset term.
    PxArticulationTendon_getOffset :: proc(self_: ^PxArticulationTendon) -> _c.float ---

    /// Gets the articulation that the tendon is a part of.
    ///
    /// The articulation.
    PxArticulationTendon_getArticulation :: proc(self_: ^PxArticulationTendon) -> ^PxArticulationReducedCoordinate ---

    /// Releases a tendon to remove it from the articulation and free its associated memory.
    ///
    /// When an articulation is released, its attached tendons are automatically released.
    ///
    /// Releasing a tendon is not allowed while the articulation is in a scene. In order to
    /// release the tendon, remove and then re-add the articulation to the scene.
    PxArticulationTendon_release_mut :: proc(self_: ^PxArticulationTendon) ---

    /// Creates an articulation attachment and adds it to the list of children in the parent attachment.
    ///
    /// Creating an attachment is not allowed while the articulation is in a scene. In order to
    /// add the attachment, remove and then re-add the articulation to the scene.
    ///
    /// The newly-created attachment if creation was successful, otherwise a null pointer.
    PxArticulationSpatialTendon_createAttachment_mut :: proc(self_: ^PxArticulationSpatialTendon, parent: ^PxArticulationAttachment, coefficient: _c.float, relativeOffset: PxVec3, link: ^PxArticulationLink) -> ^PxArticulationAttachment ---

    /// Fills a user-provided buffer of attachment pointers with the set of attachments.
    ///
    /// The number of attachments that were filled into the user buffer.
    PxArticulationSpatialTendon_getAttachments :: proc(self_: ^PxArticulationSpatialTendon, userBuffer: ^^PxArticulationAttachment, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Returns the number of attachments in the tendon.
    ///
    /// The number of attachments.
    PxArticulationSpatialTendon_getNbAttachments :: proc(self_: ^PxArticulationSpatialTendon) -> _c.uint32_t ---

    /// Returns the string name of the dynamic type.
    ///
    /// The string name.
    PxArticulationSpatialTendon_getConcreteTypeName :: proc(self_: ^PxArticulationSpatialTendon) -> ^_c.char ---

    /// Creates an articulation tendon joint and adds it to the list of children in the parent tendon joint.
    ///
    /// Creating a tendon joint is not allowed while the articulation is in a scene. In order to
    /// add the joint, remove and then re-add the articulation to the scene.
    ///
    /// The newly-created tendon joint if creation was successful, otherwise a null pointer.
    ///
    /// - The axis motion must not be configured as PxArticulationMotion::eLOCKED.
    /// - The axis cannot be part of a fixed joint, i.e. joint configured as PxArticulationJointType::eFIX.
    PxArticulationFixedTendon_createTendonJoint_mut :: proc(self_: ^PxArticulationFixedTendon, parent: ^PxArticulationTendonJoint, axis: _c.int32_t, coefficient: _c.float, recipCoefficient: _c.float, link: ^PxArticulationLink) -> ^PxArticulationTendonJoint ---

    /// Fills a user-provided buffer of tendon-joint pointers with the set of tendon joints.
    ///
    /// The number of tendon joints filled into the user buffer.
    PxArticulationFixedTendon_getTendonJoints :: proc(self_: ^PxArticulationFixedTendon, userBuffer: ^^PxArticulationTendonJoint, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Returns the number of tendon joints in the tendon.
    ///
    /// The number of tendon joints.
    PxArticulationFixedTendon_getNbTendonJoints :: proc(self_: ^PxArticulationFixedTendon) -> _c.uint32_t ---

    /// Sets the spring rest length of the tendon.
    ///
    /// The accumulated "length" of a fixed tendon is a linear combination of the joint axis positions that the tendon is
    /// associated with, scaled by the respective tendon joints' coefficients. As such, when the joint positions of all
    /// joints are zero, the accumulated length of a fixed tendon is zero.
    ///
    /// The spring of the tendon is not exerting any force on the articulation when the rest length is equal to the
    /// tendon's accumulated length plus the tendon offset.
    PxArticulationFixedTendon_setRestLength_mut :: proc(self_: ^PxArticulationFixedTendon, restLength: _c.float) ---

    /// Gets the spring rest length of the tendon.
    ///
    /// The spring rest length of the tendon.
    PxArticulationFixedTendon_getRestLength :: proc(self_: ^PxArticulationFixedTendon) -> _c.float ---

    /// Sets the low and high limit on the length of the tendon.
    ///
    /// The limits, together with the damping and limit stiffness parameters, act on the accumulated length of the tendon.
    PxArticulationFixedTendon_setLimitParameters_mut :: proc(self_: ^PxArticulationFixedTendon, parameter: ^PxArticulationTendonLimit) ---

    /// Gets the low and high limit on the length of the tendon.
    ///
    /// Struct with the low and high limit.
    PxArticulationFixedTendon_getLimitParameters :: proc(self_: ^PxArticulationFixedTendon) -> PxArticulationTendonLimit ---

    /// Returns the string name of the dynamic type.
    ///
    /// The string name.
    PxArticulationFixedTendon_getConcreteTypeName :: proc(self_: ^PxArticulationFixedTendon) -> ^_c.char ---

    PxArticulationCache_new :: proc() -> PxArticulationCache ---

    /// Releases an articulation cache.
    PxArticulationCache_release_mut :: proc(self_: ^PxArticulationCache) ---

    /// Releases the sensor.
    ///
    /// Releasing a sensor is not allowed while the articulation is in a scene. In order to
    /// release a sensor, remove and then re-add the articulation to the scene.
    PxArticulationSensor_release_mut :: proc(self_: ^PxArticulationSensor) ---

    /// Returns the spatial force in the local frame of the sensor.
    ///
    /// The spatial force.
    ///
    /// This call is not allowed while the simulation is running except in a split simulation during [`PxScene::collide`]() and up to #PxScene::advance(),
    /// and in PxContactModifyCallback or in contact report callbacks.
    PxArticulationSensor_getForces :: proc(self_: ^PxArticulationSensor) -> PxSpatialForce ---

    /// Returns the relative pose between this sensor and the body frame of the link that the sensor is attached to.
    ///
    /// The link body frame is at the center of mass and aligned with the principal axes of inertia, see PxRigidBody::getCMassLocalPose.
    ///
    /// The transform link body frame -> sensor frame.
    PxArticulationSensor_getRelativePose :: proc(self_: ^PxArticulationSensor) -> PxTransform ---

    /// Sets the relative pose between this sensor and the body frame of the link that the sensor is attached to.
    ///
    /// The link body frame is at the center of mass and aligned with the principal axes of inertia, see PxRigidBody::getCMassLocalPose.
    ///
    /// Setting the sensor relative pose is not allowed while the articulation is in a scene. In order to
    /// set the pose, remove and then re-add the articulation to the scene.
    PxArticulationSensor_setRelativePose_mut :: proc(self_: ^PxArticulationSensor, pose: ^PxTransform) ---

    /// Returns the link that this sensor is attached to.
    ///
    /// A pointer to the link.
    PxArticulationSensor_getLink :: proc(self_: ^PxArticulationSensor) -> ^PxArticulationLink ---

    /// Returns the index of this sensor inside the articulation.
    ///
    /// The return value is only valid for sensors attached to articulations that are in a scene.
    ///
    /// The low-level index, or 0xFFFFFFFF if the articulation is not in a scene.
    PxArticulationSensor_getIndex :: proc(self_: ^PxArticulationSensor) -> _c.uint32_t ---

    /// Returns the articulation that this sensor is part of.
    ///
    /// A pointer to the articulation.
    PxArticulationSensor_getArticulation :: proc(self_: ^PxArticulationSensor) -> ^PxArticulationReducedCoordinate ---

    /// Returns the sensor's flags.
    ///
    /// The current set of flags of the sensor.
    PxArticulationSensor_getFlags :: proc(self_: ^PxArticulationSensor) -> _c.uint8_t ---

    /// Sets a flag of the sensor.
    ///
    /// Setting the sensor flags is not allowed while the articulation is in a scene. In order to
    /// set the flags, remove and then re-add the articulation to the scene.
    PxArticulationSensor_setFlag_mut :: proc(self_: ^PxArticulationSensor, flag: _c.int32_t, enabled: _c.bool) ---

    /// Returns the string name of the dynamic type.
    ///
    /// The string name.
    PxArticulationSensor_getConcreteTypeName :: proc(self_: ^PxArticulationSensor) -> ^_c.char ---

    /// Returns the scene which this articulation belongs to.
    ///
    /// Owner Scene. NULL if not part of a scene.
    PxArticulationReducedCoordinate_getScene :: proc(self_: ^PxArticulationReducedCoordinate) -> ^PxScene ---

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
    PxArticulationReducedCoordinate_setSolverIterationCounts_mut :: proc(self_: ^PxArticulationReducedCoordinate, minPositionIters: _c.uint32_t, minVelocityIters: _c.uint32_t) ---

    /// Returns the solver iteration counts.
    PxArticulationReducedCoordinate_getSolverIterationCounts :: proc(self_: ^PxArticulationReducedCoordinate, minPositionIters: ^_c.uint32_t, minVelocityIters: ^_c.uint32_t) ---

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
    PxArticulationReducedCoordinate_isSleeping :: proc(self_: ^PxArticulationReducedCoordinate) -> _c.bool ---

    /// Sets the mass-normalized energy threshold below which the articulation may go to sleep.
    ///
    /// The articulation will sleep if the energy of each link is below this threshold.
    ///
    /// This call may not be made during simulation.
    PxArticulationReducedCoordinate_setSleepThreshold_mut :: proc(self_: ^PxArticulationReducedCoordinate, threshold: _c.float) ---

    /// Returns the mass-normalized energy below which the articulation may go to sleep.
    ///
    /// The energy threshold for sleeping.
    PxArticulationReducedCoordinate_getSleepThreshold :: proc(self_: ^PxArticulationReducedCoordinate) -> _c.float ---

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
    PxArticulationReducedCoordinate_setStabilizationThreshold_mut :: proc(self_: ^PxArticulationReducedCoordinate, threshold: _c.float) ---

    /// Returns the mass-normalized kinetic energy below which the articulation may participate in stabilization.
    ///
    /// Articulations whose kinetic energy divided by their mass is above this threshold will not participate in stabilization.
    ///
    /// The energy threshold for participating in stabilization.
    PxArticulationReducedCoordinate_getStabilizationThreshold :: proc(self_: ^PxArticulationReducedCoordinate) -> _c.float ---

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
    PxArticulationReducedCoordinate_setWakeCounter_mut :: proc(self_: ^PxArticulationReducedCoordinate, wakeCounterValue: _c.float) ---

    /// Returns the wake counter of the articulation in seconds.
    ///
    /// The wake counter of the articulation in seconds.
    ///
    /// This call may not be made during simulation, except in a split simulation in-between [`PxScene::fetchCollision`] and #PxScene::advance.
    PxArticulationReducedCoordinate_getWakeCounter :: proc(self_: ^PxArticulationReducedCoordinate) -> _c.float ---

    /// Wakes up the articulation if it is sleeping.
    ///
    /// - The articulation will get woken up and might cause other touching objects to wake up as well during the next simulation step.
    /// - This will set the wake counter of the articulation to the value specified in [`PxSceneDesc::wakeCounterResetValue`].
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation,
    /// except in a split simulation in-between [`PxScene::fetchCollision`] and #PxScene::advance.
    PxArticulationReducedCoordinate_wakeUp_mut :: proc(self_: ^PxArticulationReducedCoordinate) ---

    /// Forces the articulation to sleep.
    ///
    /// - The articulation will stay asleep during the next simulation step if not touched by another non-sleeping actor.
    /// - This will set any applied force, the velocity, and the wake counter of all bodies in the articulation to zero.
    ///
    /// This call may not be made during simulation, and may only be made on articulations that are in a scene.
    PxArticulationReducedCoordinate_putToSleep_mut :: proc(self_: ^PxArticulationReducedCoordinate) ---

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
    PxArticulationReducedCoordinate_setMaxCOMLinearVelocity_mut :: proc(self_: ^PxArticulationReducedCoordinate, maxLinearVelocity: _c.float) ---

    /// Gets the limit on the magnitude of the linear velocity of the articulation's center of mass.
    ///
    /// The maximal linear velocity magnitude.
    PxArticulationReducedCoordinate_getMaxCOMLinearVelocity :: proc(self_: ^PxArticulationReducedCoordinate) -> _c.float ---

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
    PxArticulationReducedCoordinate_setMaxCOMAngularVelocity_mut :: proc(self_: ^PxArticulationReducedCoordinate, maxAngularVelocity: _c.float) ---

    /// Gets the limit on the magnitude of the angular velocity at the articulation's center of mass.
    ///
    /// The maximal angular velocity magnitude.
    PxArticulationReducedCoordinate_getMaxCOMAngularVelocity :: proc(self_: ^PxArticulationReducedCoordinate) -> _c.float ---

    /// Adds a link to the articulation with default attribute values.
    ///
    /// The new link, or NULL if the link cannot be created.
    ///
    /// Creating a link is not allowed while the articulation is in a scene. In order to add a link,
    /// remove and then re-add the articulation to the scene.
    PxArticulationReducedCoordinate_createLink_mut :: proc(self_: ^PxArticulationReducedCoordinate, parent: ^PxArticulationLink, pose: ^PxTransform) -> ^PxArticulationLink ---

    /// Releases the articulation, and all its links and corresponding joints.
    ///
    /// Attached sensors and tendons are released automatically when the articulation is released.
    ///
    /// This call may not be made during simulation.
    PxArticulationReducedCoordinate_release_mut :: proc(self_: ^PxArticulationReducedCoordinate) ---

    /// Returns the number of links in the articulation.
    ///
    /// The number of links.
    PxArticulationReducedCoordinate_getNbLinks :: proc(self_: ^PxArticulationReducedCoordinate) -> _c.uint32_t ---

    /// Returns the set of links in the articulation in the order that they were added to the articulation using createLink.
    ///
    /// The number of links written into the buffer.
    PxArticulationReducedCoordinate_getLinks :: proc(self_: ^PxArticulationReducedCoordinate, userBuffer: ^^PxArticulationLink, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Returns the number of shapes in the articulation.
    ///
    /// The number of shapes.
    PxArticulationReducedCoordinate_getNbShapes :: proc(self_: ^PxArticulationReducedCoordinate) -> _c.uint32_t ---

    /// Sets a name string for the articulation that can be retrieved with getName().
    ///
    /// This is for debugging and is not used by the SDK. The string is not copied by the SDK,
    /// only the pointer is stored.
    PxArticulationReducedCoordinate_setName_mut :: proc(self_: ^PxArticulationReducedCoordinate, name: ^_c.char) ---

    /// Returns the name string set with setName().
    ///
    /// Name string associated with the articulation.
    PxArticulationReducedCoordinate_getName :: proc(self_: ^PxArticulationReducedCoordinate) -> ^_c.char ---

    /// Returns the axis-aligned bounding box enclosing the articulation.
    ///
    /// The articulation's bounding box.
    ///
    /// It is not allowed to use this method while the simulation is running, except in a split simulation
    /// during [`PxScene::collide`]() and up to #PxScene::advance(), and in PxContactModifyCallback or in contact report callbacks.
    PxArticulationReducedCoordinate_getWorldBounds :: proc(self_: ^PxArticulationReducedCoordinate, inflation: _c.float) -> PxBounds3 ---

    /// Returns the aggregate the articulation might be a part of.
    ///
    /// The aggregate the articulation is a part of, or NULL if the articulation does not belong to an aggregate.
    PxArticulationReducedCoordinate_getAggregate :: proc(self_: ^PxArticulationReducedCoordinate) -> ^PxAggregate ---

    /// Sets flags on the articulation.
    ///
    /// This call may not be made during simulation.
    PxArticulationReducedCoordinate_setArticulationFlags_mut :: proc(self_: ^PxArticulationReducedCoordinate, flags: _c.uint8_t) ---

    /// Raises or clears a flag on the articulation.
    ///
    /// This call may not be made during simulation.
    PxArticulationReducedCoordinate_setArticulationFlag_mut :: proc(self_: ^PxArticulationReducedCoordinate, flag: _c.int32_t, value: _c.bool) ---

    /// Returns the articulation's flags.
    ///
    /// The flags.
    PxArticulationReducedCoordinate_getArticulationFlags :: proc(self_: ^PxArticulationReducedCoordinate) -> _c.uint8_t ---

    /// Returns the total number of joint degrees-of-freedom (DOFs) of the articulation.
    ///
    /// - The six DOFs of the base of a floating-base articulation are not included in this count.
    /// - Example: Both a fixed-base and a floating-base double-pendulum with two revolute joints will have getDofs() == 2.
    /// - The return value is only valid for articulations that are in a scene.
    ///
    /// The number of joint DOFs, or 0xFFFFFFFF if the articulation is not in a scene.
    PxArticulationReducedCoordinate_getDofs :: proc(self_: ^PxArticulationReducedCoordinate) -> _c.uint32_t ---

    /// Creates an articulation cache that can be used to read and write internal articulation data.
    ///
    /// - When the structure of the articulation changes (e.g. adding a link or sensor) after the cache was created,
    /// the cache needs to be released and recreated.
    /// - Free the memory allocated for the cache by calling the release() method on the cache.
    /// - Caches can only be created by articulations that are in a scene.
    ///
    /// The cache, or NULL if the articulation is not in a scene.
    PxArticulationReducedCoordinate_createCache :: proc(self_: ^PxArticulationReducedCoordinate) -> ^PxArticulationCache ---

    /// Returns the size of the articulation cache in bytes.
    ///
    /// - The size does not include: the user-allocated memory for the coefficient matrix or lambda values;
    /// the scratch-related memory/members; and the cache version. See comment in [`PxArticulationCache`].
    /// - The return value is only valid for articulations that are in a scene.
    ///
    /// The byte size of the cache, or 0xFFFFFFFF if the articulation is not in a scene.
    PxArticulationReducedCoordinate_getCacheDataSize :: proc(self_: ^PxArticulationReducedCoordinate) -> _c.uint32_t ---

    /// Zeroes all data in the articulation cache, except user-provided and scratch memory, and cache version.
    ///
    /// This call may only be made on articulations that are in a scene.
    PxArticulationReducedCoordinate_zeroCache :: proc(self_: ^PxArticulationReducedCoordinate, cache: ^PxArticulationCache) ---

    /// Applies the data in the cache to the articulation.
    ///
    /// This call wakes the articulation if it is sleeping, and the autowake parameter is true (default) or:
    /// - a nonzero joint velocity is applied or
    /// - a nonzero joint force is applied or
    /// - a nonzero root velocity is applied
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
    PxArticulationReducedCoordinate_applyCache_mut :: proc(self_: ^PxArticulationReducedCoordinate, cache: ^PxArticulationCache, flags: _c.uint32_t, autowake: _c.bool) ---

    /// Copies internal data of the articulation to the cache.
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
    PxArticulationReducedCoordinate_copyInternalStateToCache :: proc(self_: ^PxArticulationReducedCoordinate, cache: ^PxArticulationCache, flags: _c.uint32_t) ---

    /// Converts maximal-coordinate joint DOF data to reduced coordinates.
    ///
    /// - Indexing into the maximal joint DOF data is via the link's low-level index minus 1 (the root link is not included).
    /// - The reduced-coordinate data follows the cache indexing convention, see PxArticulationCache::jointVelocity.
    ///
    /// The articulation must be in a scene.
    PxArticulationReducedCoordinate_packJointData :: proc(self_: ^PxArticulationReducedCoordinate, maximum: ^_c.float, reduced: ^_c.float) ---

    /// Converts reduced-coordinate joint DOF data to maximal coordinates.
    ///
    /// - Indexing into the maximal joint DOF data is via the link's low-level index minus 1 (the root link is not included).
    /// - The reduced-coordinate data follows the cache indexing convention, see PxArticulationCache::jointVelocity.
    ///
    /// The articulation must be in a scene.
    PxArticulationReducedCoordinate_unpackJointData :: proc(self_: ^PxArticulationReducedCoordinate, reduced: ^_c.float, maximum: ^_c.float) ---

    /// Prepares common articulation data based on articulation pose for inverse dynamics calculations.
    ///
    /// Usage:
    /// 1. Set articulation pose (joint positions and base transform) via articulation cache and applyCache().
    /// 1. Call commonInit.
    /// 1. Call inverse dynamics computation method.
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
    PxArticulationReducedCoordinate_commonInit :: proc(self_: ^PxArticulationReducedCoordinate) ---

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
    PxArticulationReducedCoordinate_computeGeneralizedGravityForce :: proc(self_: ^PxArticulationReducedCoordinate, cache: ^PxArticulationCache) ---

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
    PxArticulationReducedCoordinate_computeCoriolisAndCentrifugalForce :: proc(self_: ^PxArticulationReducedCoordinate, cache: ^PxArticulationCache) ---

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
    PxArticulationReducedCoordinate_computeGeneralizedExternalForce :: proc(self_: ^PxArticulationReducedCoordinate, cache: ^PxArticulationCache) ---

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
    PxArticulationReducedCoordinate_computeJointAcceleration :: proc(self_: ^PxArticulationReducedCoordinate, cache: ^PxArticulationCache) ---

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
    PxArticulationReducedCoordinate_computeJointForce :: proc(self_: ^PxArticulationReducedCoordinate, cache: ^PxArticulationCache) ---

    /// Compute the dense Jacobian for the articulation in world space, including the DOFs of a potentially floating base.
    ///
    /// This computes the dense representation of an inherently sparse matrix. Multiplication with this matrix maps
    /// joint space velocities to world-space linear and angular (i.e. spatial) velocities of the centers of mass of the links.
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
    PxArticulationReducedCoordinate_computeDenseJacobian :: proc(self_: ^PxArticulationReducedCoordinate, cache: ^PxArticulationCache, nRows: ^_c.uint32_t, nCols: ^_c.uint32_t) ---

    /// Computes the coefficient matrix for contact forces.
    ///
    /// - The matrix dimension is getCoefficientMatrixSize() = getDofs() * getNbLoopJoints(), and the DOF (column) indexing follows the internal DOF order, see PxArticulationCache::jointVelocity.
    /// - Each column in the matrix is the joint forces effected by a contact based on impulse strength 1.
    /// - The user must allocate memory for PxArticulationCache::coefficientMatrix where the required size of the PxReal array is equal to getCoefficientMatrixSize().
    /// - commonInit() must be called before the computation, and after setting the articulation pose via applyCache().
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
    PxArticulationReducedCoordinate_computeCoefficientMatrix :: proc(self_: ^PxArticulationReducedCoordinate, cache: ^PxArticulationCache) ---

    /// Computes the lambda values when the test impulse is 1.
    ///
    /// - The user must allocate memory for PxArticulationCache::lambda where the required size of the PxReal array is equal to getNbLoopJoints().
    /// - commonInit() must be called before the computation, and after setting the articulation pose via applyCache().
    ///
    /// True if convergence was achieved within maxIter; False if convergence was not achieved or the operation failed otherwise.
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
    PxArticulationReducedCoordinate_computeLambda :: proc(self_: ^PxArticulationReducedCoordinate, cache: ^PxArticulationCache, initialState: ^PxArticulationCache, jointTorque: ^_c.float, maxIter: _c.uint32_t) -> _c.bool ---

    /// Compute the joint-space inertia matrix that maps joint accelerations to joint forces: forces = M * accelerations.
    ///
    /// - Inputs - Articulation pose (joint positions and base transform).
    /// - Outputs - Mass matrix (in cache).
    ///
    /// commonInit() must be called before the computation, and after setting the articulation pose via applyCache().
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
    PxArticulationReducedCoordinate_computeGeneralizedMassMatrix :: proc(self_: ^PxArticulationReducedCoordinate, cache: ^PxArticulationCache) ---

    /// Adds a loop joint to the articulation system for inverse dynamics.
    ///
    /// This call may not be made during simulation.
    PxArticulationReducedCoordinate_addLoopJoint_mut :: proc(self_: ^PxArticulationReducedCoordinate, joint: ^PxConstraint) ---

    /// Removes a loop joint from the articulation for inverse dynamics.
    ///
    /// This call may not be made during simulation.
    PxArticulationReducedCoordinate_removeLoopJoint_mut :: proc(self_: ^PxArticulationReducedCoordinate, joint: ^PxConstraint) ---

    /// Returns the number of loop joints in the articulation for inverse dynamics.
    ///
    /// The number of loop joints.
    PxArticulationReducedCoordinate_getNbLoopJoints :: proc(self_: ^PxArticulationReducedCoordinate) -> _c.uint32_t ---

    /// Returns the set of loop constraints (i.e. joints) in the articulation.
    ///
    /// The number of constraints written into the buffer.
    PxArticulationReducedCoordinate_getLoopJoints :: proc(self_: ^PxArticulationReducedCoordinate, userBuffer: ^^PxConstraint, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Returns the required size of the coefficient matrix in the articulation.
    ///
    /// Size of the coefficient matrix (equal to getDofs() * getNbLoopJoints()).
    ///
    /// This call may only be made on articulations that are in a scene.
    PxArticulationReducedCoordinate_getCoefficientMatrixSize :: proc(self_: ^PxArticulationReducedCoordinate) -> _c.uint32_t ---

    /// Sets the root link transform (world to actor frame).
    ///
    /// - For performance, prefer PxArticulationCache::rootLinkData to set the root link transform in a batch articulation state update.
    /// - Use updateKinematic() after all state updates to the articulation via non-cache API such as this method,
    /// in order to update link states for the next simulation frame or querying.
    ///
    /// This call may not be made during simulation.
    PxArticulationReducedCoordinate_setRootGlobalPose_mut :: proc(self_: ^PxArticulationReducedCoordinate, pose: ^PxTransform, autowake: _c.bool) ---

    /// Returns the root link transform (world to actor frame).
    ///
    /// For performance, prefer PxArticulationCache::rootLinkData to get the root link transform in a batch query.
    ///
    /// The root link transform.
    ///
    /// This call is not allowed while the simulation is running except in a split simulation during [`PxScene::collide`]() and up to #PxScene::advance(),
    /// and in PxContactModifyCallback or in contact report callbacks.
    PxArticulationReducedCoordinate_getRootGlobalPose :: proc(self_: ^PxArticulationReducedCoordinate) -> PxTransform ---

    /// Sets the root link linear center-of-mass velocity.
    ///
    /// - The linear velocity is with respect to the link's center of mass and not the actor frame origin.
    /// - For performance, prefer PxArticulationCache::rootLinkData to set the root link velocity in a batch articulation state update.
    /// - The articulation is woken up if the input velocity is nonzero (ignoring autowake) and the articulation is in a scene.
    /// - Use updateKinematic() after all state updates to the articulation via non-cache API such as this method,
    /// in order to update link states for the next simulation frame or querying.
    ///
    /// This call may not be made during simulation, except in a split simulation in-between [`PxScene::fetchCollision`] and #PxScene::advance.
    PxArticulationReducedCoordinate_setRootLinearVelocity_mut :: proc(self_: ^PxArticulationReducedCoordinate, linearVelocity: ^PxVec3, autowake: _c.bool) ---

    /// Gets the root link center-of-mass linear velocity.
    ///
    /// - The linear velocity is with respect to the link's center of mass and not the actor frame origin.
    /// - For performance, prefer PxArticulationCache::rootLinkData to get the root link velocity in a batch query.
    ///
    /// The root link center-of-mass linear velocity.
    ///
    /// This call is not allowed while the simulation is running except in a split simulation during [`PxScene::collide`]() and up to #PxScene::advance(),
    /// and in PxContactModifyCallback or in contact report callbacks.
    PxArticulationReducedCoordinate_getRootLinearVelocity :: proc(self_: ^PxArticulationReducedCoordinate) -> PxVec3 ---

    /// Sets the root link angular velocity.
    ///
    /// - For performance, prefer PxArticulationCache::rootLinkData to set the root link velocity in a batch articulation state update.
    /// - The articulation is woken up if the input velocity is nonzero (ignoring autowake) and the articulation is in a scene.
    /// - Use updateKinematic() after all state updates to the articulation via non-cache API such as this method,
    /// in order to update link states for the next simulation frame or querying.
    ///
    /// This call may not be made during simulation, except in a split simulation in-between [`PxScene::fetchCollision`] and #PxScene::advance.
    PxArticulationReducedCoordinate_setRootAngularVelocity_mut :: proc(self_: ^PxArticulationReducedCoordinate, angularVelocity: ^PxVec3, autowake: _c.bool) ---

    /// Gets the root link angular velocity.
    ///
    /// For performance, prefer PxArticulationCache::rootLinkData to get the root link velocity in a batch query.
    ///
    /// The root link angular velocity.
    ///
    /// This call is not allowed while the simulation is running except in a split simulation during [`PxScene::collide`]() and up to #PxScene::advance(),
    /// and in PxContactModifyCallback or in contact report callbacks.
    PxArticulationReducedCoordinate_getRootAngularVelocity :: proc(self_: ^PxArticulationReducedCoordinate) -> PxVec3 ---

    /// Returns the (classical) link acceleration in world space for the given low-level link index.
    ///
    /// - The returned acceleration is not a spatial, but a classical, i.e. body-fixed acceleration (https://en.wikipedia.org/wiki/Spatial_acceleration).
    /// - The (linear) acceleration is with respect to the link's center of mass and not the actor frame origin.
    ///
    /// The link's center-of-mass classical acceleration, or 0 if the call is made before the articulation participated in a first simulation step.
    ///
    /// This call may only be made on articulations that are in a scene, and it is not allowed to use this method while the simulation
    /// is running except in a split simulation during [`PxScene::collide`]() and up to #PxScene::advance(), and in PxContactModifyCallback or in contact report callbacks.
    PxArticulationReducedCoordinate_getLinkAcceleration_mut :: proc(self_: ^PxArticulationReducedCoordinate, linkId: _c.uint32_t) -> PxSpatialVelocity ---

    /// Returns the GPU articulation index.
    ///
    /// The GPU index, or 0xFFFFFFFF if the articulation is not in a scene or PxSceneFlag::eSUPPRESS_READBACK is not set.
    PxArticulationReducedCoordinate_getGpuArticulationIndex_mut :: proc(self_: ^PxArticulationReducedCoordinate) -> _c.uint32_t ---

    /// Creates a spatial tendon to attach to the articulation with default attribute values.
    ///
    /// The new spatial tendon.
    ///
    /// Creating a spatial tendon is not allowed while the articulation is in a scene. In order to
    /// add the tendon, remove and then re-add the articulation to the scene.
    PxArticulationReducedCoordinate_createSpatialTendon_mut :: proc(self_: ^PxArticulationReducedCoordinate) -> ^PxArticulationSpatialTendon ---

    /// Creates a fixed tendon to attach to the articulation with default attribute values.
    ///
    /// The new fixed tendon.
    ///
    /// Creating a fixed tendon is not allowed while the articulation is in a scene. In order to
    /// add the tendon, remove and then re-add the articulation to the scene.
    PxArticulationReducedCoordinate_createFixedTendon_mut :: proc(self_: ^PxArticulationReducedCoordinate) -> ^PxArticulationFixedTendon ---

    /// Creates a force sensor attached to a link of the articulation.
    ///
    /// The new sensor.
    ///
    /// Creating a sensor is not allowed while the articulation is in a scene. In order to
    /// add the sensor, remove and then re-add the articulation to the scene.
    PxArticulationReducedCoordinate_createSensor_mut :: proc(self_: ^PxArticulationReducedCoordinate, link: ^PxArticulationLink, relativePose: ^PxTransform) -> ^PxArticulationSensor ---

    /// Returns the spatial tendons attached to the articulation.
    ///
    /// The order of the tendons in the buffer is not necessarily identical to the order in which the tendons were added to the articulation.
    ///
    /// The number of tendons written into the buffer.
    PxArticulationReducedCoordinate_getSpatialTendons :: proc(self_: ^PxArticulationReducedCoordinate, userBuffer: ^^PxArticulationSpatialTendon, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Returns the number of spatial tendons in the articulation.
    ///
    /// The number of tendons.
    PxArticulationReducedCoordinate_getNbSpatialTendons_mut :: proc(self_: ^PxArticulationReducedCoordinate) -> _c.uint32_t ---

    /// Returns the fixed tendons attached to the articulation.
    ///
    /// The order of the tendons in the buffer is not necessarily identical to the order in which the tendons were added to the articulation.
    ///
    /// The number of tendons written into the buffer.
    PxArticulationReducedCoordinate_getFixedTendons :: proc(self_: ^PxArticulationReducedCoordinate, userBuffer: ^^PxArticulationFixedTendon, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Returns the number of fixed tendons in the articulation.
    ///
    /// The number of tendons.
    PxArticulationReducedCoordinate_getNbFixedTendons_mut :: proc(self_: ^PxArticulationReducedCoordinate) -> _c.uint32_t ---

    /// Returns the sensors attached to the articulation.
    ///
    /// The order of the sensors in the buffer is not necessarily identical to the order in which the sensors were added to the articulation.
    ///
    /// The number of sensors written into the buffer.
    PxArticulationReducedCoordinate_getSensors :: proc(self_: ^PxArticulationReducedCoordinate, userBuffer: ^^PxArticulationSensor, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Returns the number of sensors in the articulation.
    ///
    /// The number of sensors.
    PxArticulationReducedCoordinate_getNbSensors_mut :: proc(self_: ^PxArticulationReducedCoordinate) -> _c.uint32_t ---

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
    PxArticulationReducedCoordinate_updateKinematic_mut :: proc(self_: ^PxArticulationReducedCoordinate, flags: _c.uint8_t) ---

    /// Gets the parent articulation link of this joint.
    ///
    /// The parent link.
    PxArticulationJointReducedCoordinate_getParentArticulationLink :: proc(self_: ^PxArticulationJointReducedCoordinate) -> ^PxArticulationLink ---

    /// Sets the joint pose in the parent link actor frame.
    ///
    /// This call is not allowed while the simulation is running.
    PxArticulationJointReducedCoordinate_setParentPose_mut :: proc(self_: ^PxArticulationJointReducedCoordinate, pose: ^PxTransform) ---

    /// Gets the joint pose in the parent link actor frame.
    ///
    /// The joint pose.
    PxArticulationJointReducedCoordinate_getParentPose :: proc(self_: ^PxArticulationJointReducedCoordinate) -> PxTransform ---

    /// Gets the child articulation link of this joint.
    ///
    /// The child link.
    PxArticulationJointReducedCoordinate_getChildArticulationLink :: proc(self_: ^PxArticulationJointReducedCoordinate) -> ^PxArticulationLink ---

    /// Sets the joint pose in the child link actor frame.
    ///
    /// This call is not allowed while the simulation is running.
    PxArticulationJointReducedCoordinate_setChildPose_mut :: proc(self_: ^PxArticulationJointReducedCoordinate, pose: ^PxTransform) ---

    /// Gets the joint pose in the child link actor frame.
    ///
    /// The joint pose.
    PxArticulationJointReducedCoordinate_getChildPose :: proc(self_: ^PxArticulationJointReducedCoordinate) -> PxTransform ---

    /// Sets the joint type (e.g. revolute).
    ///
    /// Setting the joint type is not allowed while the articulation is in a scene.
    /// In order to set the joint type, remove and then re-add the articulation to the scene.
    PxArticulationJointReducedCoordinate_setJointType_mut :: proc(self_: ^PxArticulationJointReducedCoordinate, jointType: _c.int32_t) ---

    /// Gets the joint type.
    ///
    /// The joint type.
    PxArticulationJointReducedCoordinate_getJointType :: proc(self_: ^PxArticulationJointReducedCoordinate) -> _c.int32_t ---

    /// Sets the joint motion for a given axis.
    ///
    /// Setting the motion of joint axes is not allowed while the articulation is in a scene.
    /// In order to set the motion, remove and then re-add the articulation to the scene.
    PxArticulationJointReducedCoordinate_setMotion_mut :: proc(self_: ^PxArticulationJointReducedCoordinate, axis: _c.int32_t, motion: _c.int32_t) ---

    /// Returns the joint motion for the given axis.
    ///
    /// The joint motion of the given axis.
    PxArticulationJointReducedCoordinate_getMotion :: proc(self_: ^PxArticulationJointReducedCoordinate, axis: _c.int32_t) -> _c.int32_t ---

    /// Sets the joint limits for a given axis.
    ///
    /// - The motion of the corresponding axis should be set to PxArticulationMotion::eLIMITED in order for the limits to be enforced.
    /// - The lower limit should be strictly smaller than the higher limit. If the limits should be equal, use PxArticulationMotion::eLOCKED
    /// and an appropriate offset in the parent/child joint frames.
    ///
    /// This call is not allowed while the simulation is running.
    ///
    /// For spherical joints, limit.min and limit.max must both be in range [-Pi, Pi].
    PxArticulationJointReducedCoordinate_setLimitParams_mut :: proc(self_: ^PxArticulationJointReducedCoordinate, axis: _c.int32_t, limit: ^PxArticulationLimit) ---

    /// Returns the joint limits for a given axis.
    ///
    /// The joint limits.
    PxArticulationJointReducedCoordinate_getLimitParams :: proc(self_: ^PxArticulationJointReducedCoordinate, axis: _c.int32_t) -> PxArticulationLimit ---

    /// Configures a joint drive for the given axis.
    ///
    /// See PxArticulationDrive for parameter details; and the manual for further information, and the drives' implicit spring-damper (i.e. PD control) implementation in particular.
    ///
    /// This call is not allowed while the simulation is running.
    PxArticulationJointReducedCoordinate_setDriveParams_mut :: proc(self_: ^PxArticulationJointReducedCoordinate, axis: _c.int32_t, drive: ^PxArticulationDrive) ---

    /// Gets the joint drive configuration for the given axis.
    ///
    /// The drive parameters.
    PxArticulationJointReducedCoordinate_getDriveParams :: proc(self_: ^PxArticulationJointReducedCoordinate, axis: _c.int32_t) -> PxArticulationDrive ---

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
    PxArticulationJointReducedCoordinate_setDriveTarget_mut :: proc(self_: ^PxArticulationJointReducedCoordinate, axis: _c.int32_t, target: _c.float, autowake: _c.bool) ---

    /// Returns the joint drive position target for the given axis.
    ///
    /// The target position.
    PxArticulationJointReducedCoordinate_getDriveTarget :: proc(self_: ^PxArticulationJointReducedCoordinate, axis: _c.int32_t) -> _c.float ---

    /// Sets the joint drive velocity target for the given axis.
    ///
    /// The target units are linear units (equivalent to scene units) per second for a translational axis, or radians per second for a rotational axis.
    ///
    /// This call is not allowed while the simulation is running.
    PxArticulationJointReducedCoordinate_setDriveVelocity_mut :: proc(self_: ^PxArticulationJointReducedCoordinate, axis: _c.int32_t, targetVel: _c.float, autowake: _c.bool) ---

    /// Returns the joint drive velocity target for the given axis.
    ///
    /// The target velocity.
    PxArticulationJointReducedCoordinate_getDriveVelocity :: proc(self_: ^PxArticulationJointReducedCoordinate, axis: _c.int32_t) -> _c.float ---

    /// Sets the joint armature for the given axis.
    ///
    /// - The armature is directly added to the joint-space spatial inertia of the corresponding axis.
    /// - The armature is in mass units for a prismatic (i.e. linear) joint, and in mass units * (scene linear units)^2 for a rotational joint.
    ///
    /// This call is not allowed while the simulation is running.
    PxArticulationJointReducedCoordinate_setArmature_mut :: proc(self_: ^PxArticulationJointReducedCoordinate, axis: _c.int32_t, armature: _c.float) ---

    /// Gets the joint armature for the given axis.
    ///
    /// The armature set on the given axis.
    PxArticulationJointReducedCoordinate_getArmature :: proc(self_: ^PxArticulationJointReducedCoordinate, axis: _c.int32_t) -> _c.float ---

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
    PxArticulationJointReducedCoordinate_setFrictionCoefficient_mut :: proc(self_: ^PxArticulationJointReducedCoordinate, coefficient: _c.float) ---

    /// Gets the joint friction coefficient.
    ///
    /// The joint friction coefficient.
    PxArticulationJointReducedCoordinate_getFrictionCoefficient :: proc(self_: ^PxArticulationJointReducedCoordinate) -> _c.float ---

    /// Sets the maximal joint velocity enforced for all axes.
    ///
    /// - The solver will apply appropriate joint-space impulses in order to enforce the per-axis joint-velocity limit.
    /// - The velocity units are linear units (equivalent to scene units) per second for a translational axis, or radians per second for a rotational axis.
    ///
    /// This call is not allowed while the simulation is running.
    PxArticulationJointReducedCoordinate_setMaxJointVelocity_mut :: proc(self_: ^PxArticulationJointReducedCoordinate, maxJointV: _c.float) ---

    /// Gets the maximal joint velocity enforced for all axes.
    ///
    /// The maximal per-axis joint velocity.
    PxArticulationJointReducedCoordinate_getMaxJointVelocity :: proc(self_: ^PxArticulationJointReducedCoordinate) -> _c.float ---

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
    PxArticulationJointReducedCoordinate_setJointPosition_mut :: proc(self_: ^PxArticulationJointReducedCoordinate, axis: _c.int32_t, jointPos: _c.float) ---

    /// Gets the joint position for the given axis, i.e. joint degree of freedom (DOF).
    ///
    /// For performance, prefer PxArticulationCache::jointPosition to get joint positions in a batch query.
    ///
    /// The joint position in linear units (equivalent to scene units) for a translational axis, or radians for a rotational axis.
    ///
    /// This call is not allowed while the simulation is running except in a split simulation during [`PxScene::collide`]() and up to #PxScene::advance(),
    /// and in PxContactModifyCallback or in contact report callbacks.
    PxArticulationJointReducedCoordinate_getJointPosition :: proc(self_: ^PxArticulationJointReducedCoordinate, axis: _c.int32_t) -> _c.float ---

    /// Sets the joint velocity for the given axis.
    ///
    /// - For performance, prefer PxArticulationCache::jointVelocity to set joint velocities in a batch articulation state update.
    /// - Use PxArticulationReducedCoordinate::updateKinematic after all state updates to the articulation via non-cache API such as this method,
    /// in order to update link states for the next simulation frame or querying.
    ///
    /// This call is not allowed while the simulation is running.
    PxArticulationJointReducedCoordinate_setJointVelocity_mut :: proc(self_: ^PxArticulationJointReducedCoordinate, axis: _c.int32_t, jointVel: _c.float) ---

    /// Gets the joint velocity for the given axis.
    ///
    /// For performance, prefer PxArticulationCache::jointVelocity to get joint velocities in a batch query.
    ///
    /// The joint velocity in linear units (equivalent to scene units) per second for a translational axis, or radians per second for a rotational axis.
    ///
    /// This call is not allowed while the simulation is running except in a split simulation during [`PxScene::collide`]() and up to #PxScene::advance(),
    /// and in PxContactModifyCallback or in contact report callbacks.
    PxArticulationJointReducedCoordinate_getJointVelocity :: proc(self_: ^PxArticulationJointReducedCoordinate, axis: _c.int32_t) -> _c.float ---

    /// Returns the string name of the dynamic type.
    ///
    /// The string name.
    PxArticulationJointReducedCoordinate_getConcreteTypeName :: proc(self_: ^PxArticulationJointReducedCoordinate) -> ^_c.char ---

    /// Decrements the reference count of a shape and releases it if the new reference count is zero.
    ///
    /// Note that in releases prior to PhysX 3.3 this method did not have reference counting semantics and was used to destroy a shape
    /// created with PxActor::createShape(). In PhysX 3.3 and above, this usage is deprecated, instead, use PxRigidActor::detachShape() to detach
    /// a shape from an actor. If the shape to be detached was created with PxActor::createShape(), the actor holds the only counted reference,
    /// and so when the shape is detached it will also be destroyed.
    PxShape_release_mut :: proc(self_: ^PxShape) ---

    /// Adjust the geometry of the shape.
    ///
    /// The type of the passed in geometry must match the geometry type of the shape.
    ///
    /// It is not allowed to change the geometry type of a shape.
    ///
    /// This function does not guarantee correct/continuous behavior when objects are resting on top of old or new geometry.
    PxShape_setGeometry_mut :: proc(self_: ^PxShape, geometry: ^PxGeometry) ---

    /// Retrieve a reference to the shape's geometry.
    ///
    /// The returned reference has the same lifetime as the PxShape it comes from.
    ///
    /// Reference to internal PxGeometry object.
    PxShape_getGeometry :: proc(self_: ^PxShape) -> ^PxGeometry ---

    /// Retrieves the actor which this shape is associated with.
    ///
    /// The actor this shape is associated with, if it is an exclusive shape, else NULL
    PxShape_getActor :: proc(self_: ^PxShape) -> ^PxRigidActor ---

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
    PxShape_setLocalPose_mut :: proc(self_: ^PxShape, pose: ^PxTransform) ---

    /// Retrieves the pose of the shape in actor space, i.e. relative to the actor they are owned by.
    ///
    /// This transformation is identity by default.
    ///
    /// Pose of shape relative to the actor's frame.
    PxShape_getLocalPose :: proc(self_: ^PxShape) -> PxTransform ---

    /// Sets the user definable collision filter data.
    ///
    /// Sleeping:
    /// Does wake up the actor if the filter data change causes a formerly suppressed
    /// collision pair to be enabled.
    ///
    /// Default:
    /// (0,0,0,0)
    PxShape_setSimulationFilterData_mut :: proc(self_: ^PxShape, data: ^PxFilterData) ---

    /// Retrieves the shape's collision filter data.
    PxShape_getSimulationFilterData :: proc(self_: ^PxShape) -> PxFilterData ---

    /// Sets the user definable query filter data.
    ///
    /// Default:
    /// (0,0,0,0)
    PxShape_setQueryFilterData_mut :: proc(self_: ^PxShape, data: ^PxFilterData) ---

    /// Retrieves the shape's Query filter data.
    PxShape_getQueryFilterData :: proc(self_: ^PxShape) -> PxFilterData ---

    /// Assigns material(s) to the shape. Will remove existing materials from the shape.
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the associated actor up automatically.
    PxShape_setMaterials_mut :: proc(self_: ^PxShape, materials: ^^PxMaterial, materialCount: _c.uint16_t) ---

    /// Returns the number of materials assigned to the shape.
    ///
    /// You can use [`getMaterials`]() to retrieve the material pointers.
    ///
    /// Number of materials associated with this shape.
    PxShape_getNbMaterials :: proc(self_: ^PxShape) -> _c.uint16_t ---

    /// Retrieve all the material pointers associated with the shape.
    ///
    /// You can retrieve the number of material pointers by calling [`getNbMaterials`]()
    ///
    /// Note: The returned data may contain invalid pointers if you release materials using [`PxMaterial::release`]().
    ///
    /// Number of material pointers written to the buffer.
    PxShape_getMaterials :: proc(self_: ^PxShape, userBuffer: ^^PxMaterial, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

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
    PxShape_getMaterialFromInternalFaceIndex :: proc(self_: ^PxShape, faceIndex: _c.uint32_t) -> ^PxBaseMaterial ---

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
    PxShape_setContactOffset_mut :: proc(self_: ^PxShape, contactOffset: _c.float) ---

    /// Retrieves the contact offset.
    ///
    /// The contact offset of the shape.
    PxShape_getContactOffset :: proc(self_: ^PxShape) -> _c.float ---

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
    PxShape_setRestOffset_mut :: proc(self_: ^PxShape, restOffset: _c.float) ---

    /// Retrieves the rest offset.
    ///
    /// The rest offset of the shape.
    PxShape_getRestOffset :: proc(self_: ^PxShape) -> _c.float ---

    /// Sets the density used to interact with fluids.
    ///
    /// To be physically accurate, the density of a rigid body should be computed as its mass divided by its volume. To
    /// simplify tuning the interaction of fluid and rigid bodies, the density for fluid can differ from the real density. This
    /// allows to create floating bodies, even if they are supposed to sink with their mass and volume.
    ///
    /// Default:
    /// 800.0f
    PxShape_setDensityForFluid_mut :: proc(self_: ^PxShape, densityForFluid: _c.float) ---

    /// Retrieves the density used to interact with fluids.
    ///
    /// The density of the body when interacting with fluid.
    PxShape_getDensityForFluid :: proc(self_: ^PxShape) -> _c.float ---

    /// Sets torsional patch radius.
    ///
    /// This defines the radius of the contact patch used to apply torsional friction. If the radius is 0, no torsional friction
    /// will be applied. If the radius is > 0, some torsional friction will be applied. This is proportional to the penetration depth
    /// so, if the shapes are separated or penetration is zero, no torsional friction will be applied. It is used to approximate
    /// rotational friction introduced by the compression of contacting surfaces.
    ///
    /// Default:
    /// 0.0
    PxShape_setTorsionalPatchRadius_mut :: proc(self_: ^PxShape, radius: _c.float) ---

    /// Gets torsional patch radius.
    ///
    /// This defines the radius of the contact patch used to apply torsional friction. If the radius is 0, no torsional friction
    /// will be applied. If the radius is > 0, some torsional friction will be applied. This is proportional to the penetration depth
    /// so, if the shapes are separated or penetration is zero, no torsional friction will be applied. It is used to approximate
    /// rotational friction introduced by the compression of contacting surfaces.
    ///
    /// The torsional patch radius of the shape.
    PxShape_getTorsionalPatchRadius :: proc(self_: ^PxShape) -> _c.float ---

    /// Sets minimum torsional patch radius.
    ///
    /// This defines the minimum radius of the contact patch used to apply torsional friction. If the radius is 0, the amount of torsional friction
    /// that will be applied will be entirely dependent on the value of torsionalPatchRadius.
    ///
    /// If the radius is > 0, some torsional friction will be applied regardless of the value of torsionalPatchRadius or the amount of penetration.
    ///
    /// Default:
    /// 0.0
    PxShape_setMinTorsionalPatchRadius_mut :: proc(self_: ^PxShape, radius: _c.float) ---

    /// Gets minimum torsional patch radius.
    ///
    /// This defines the minimum radius of the contact patch used to apply torsional friction. If the radius is 0, the amount of torsional friction
    /// that will be applied will be entirely dependent on the value of torsionalPatchRadius.
    ///
    /// If the radius is > 0, some torsional friction will be applied regardless of the value of torsionalPatchRadius or the amount of penetration.
    ///
    /// The minimum torsional patch radius of the shape.
    PxShape_getMinTorsionalPatchRadius :: proc(self_: ^PxShape) -> _c.float ---

    /// Sets shape flags
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the associated actor up automatically.
    ///
    /// Default:
    /// PxShapeFlag::eVISUALIZATION | PxShapeFlag::eSIMULATION_SHAPE | PxShapeFlag::eSCENE_QUERY_SHAPE
    PxShape_setFlag_mut :: proc(self_: ^PxShape, flag: _c.int32_t, value: _c.bool) ---

    /// Sets shape flags
    PxShape_setFlags_mut :: proc(self_: ^PxShape, inFlags: _c.uint8_t) ---

    /// Retrieves shape flags.
    ///
    /// The values of the shape flags.
    PxShape_getFlags :: proc(self_: ^PxShape) -> _c.uint8_t ---

    /// Returns true if the shape is exclusive to an actor.
    PxShape_isExclusive :: proc(self_: ^PxShape) -> _c.bool ---

    /// Sets a name string for the object that can be retrieved with [`getName`]().
    ///
    /// This is for debugging and is not used by the SDK.
    /// The string is not copied by the SDK, only the pointer is stored.
    ///
    /// Default:
    /// NULL
    PxShape_setName_mut :: proc(self_: ^PxShape, name: ^_c.char) ---

    /// retrieves the name string set with setName().
    ///
    /// The name associated with the shape.
    PxShape_getName :: proc(self_: ^PxShape) -> ^_c.char ---

    PxShape_getConcreteTypeName :: proc(self_: ^PxShape) -> ^_c.char ---

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
    PxRigidActor_release_mut :: proc(self_: ^PxRigidActor) ---

    /// Returns the internal actor index.
    ///
    /// This is only defined for actors that have been added to a scene.
    ///
    /// The internal actor index, or 0xffffffff if the actor is not part of a scene.
    PxRigidActor_getInternalActorIndex :: proc(self_: ^PxRigidActor) -> _c.uint32_t ---

    /// Retrieves the actors world space transform.
    ///
    /// The getGlobalPose() method retrieves the actor's current actor space to world space transformation.
    ///
    /// It is not allowed to use this method while the simulation is running (except during PxScene::collide(),
    /// in PxContactModifyCallback or in contact report callbacks).
    ///
    /// Global pose of object.
    PxRigidActor_getGlobalPose :: proc(self_: ^PxRigidActor) -> PxTransform ---

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
    PxRigidActor_setGlobalPose_mut :: proc(self_: ^PxRigidActor, pose: ^PxTransform, autowake: _c.bool) ---

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
    PxRigidActor_attachShape_mut :: proc(self_: ^PxRigidActor, shape: ^PxShape) -> _c.bool ---

    /// Detach a shape from an actor.
    ///
    /// This will also decrement the reference count of the PxShape, and if the reference count is zero, will cause it to be deleted.
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the actor up automatically.
    PxRigidActor_detachShape_mut :: proc(self_: ^PxRigidActor, shape: ^PxShape, wakeOnLostTouch: _c.bool) ---

    /// Returns the number of shapes assigned to the actor.
    ///
    /// You can use [`getShapes`]() to retrieve the shape pointers.
    ///
    /// Number of shapes associated with this actor.
    PxRigidActor_getNbShapes :: proc(self_: ^PxRigidActor) -> _c.uint32_t ---

    /// Retrieve all the shape pointers belonging to the actor.
    ///
    /// These are the shapes used by the actor for collision detection.
    ///
    /// You can retrieve the number of shape pointers by calling [`getNbShapes`]()
    ///
    /// Note: Removing shapes with [`PxShape::release`]() will invalidate the pointer of the released shape.
    ///
    /// Number of shape pointers written to the buffer.
    PxRigidActor_getShapes :: proc(self_: ^PxRigidActor, userBuffer: ^^PxShape, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Returns the number of constraint shaders attached to the actor.
    ///
    /// You can use [`getConstraints`]() to retrieve the constraint shader pointers.
    ///
    /// Number of constraint shaders attached to this actor.
    PxRigidActor_getNbConstraints :: proc(self_: ^PxRigidActor) -> _c.uint32_t ---

    /// Retrieve all the constraint shader pointers belonging to the actor.
    ///
    /// You can retrieve the number of constraint shader pointers by calling [`getNbConstraints`]()
    ///
    /// Note: Removing constraint shaders with [`PxConstraint::release`]() will invalidate the pointer of the released constraint.
    ///
    /// Number of constraint shader pointers written to the buffer.
    PxRigidActor_getConstraints :: proc(self_: ^PxRigidActor, userBuffer: ^^PxConstraint, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    PxNodeIndex_new :: proc(id: _c.uint32_t, articLinkId: _c.uint32_t) -> PxNodeIndex ---

    PxNodeIndex_new_1 :: proc(id: _c.uint32_t) -> PxNodeIndex ---

    PxNodeIndex_index :: proc(self_: ^PxNodeIndex) -> _c.uint32_t ---

    PxNodeIndex_articulationLinkId :: proc(self_: ^PxNodeIndex) -> _c.uint32_t ---

    PxNodeIndex_isArticulation :: proc(self_: ^PxNodeIndex) -> _c.uint32_t ---

    PxNodeIndex_isStaticBody :: proc(self_: ^PxNodeIndex) -> _c.bool ---

    PxNodeIndex_isValid :: proc(self_: ^PxNodeIndex) -> _c.bool ---

    PxNodeIndex_setIndices_mut :: proc(self_: ^PxNodeIndex, index: _c.uint32_t, articLinkId: _c.uint32_t) ---

    PxNodeIndex_setIndices_mut_1 :: proc(self_: ^PxNodeIndex, index: _c.uint32_t) ---

    PxNodeIndex_getInd :: proc(self_: ^PxNodeIndex) -> _c.uint64_t ---

    /// Sets the pose of the center of mass relative to the actor.
    ///
    /// Changing this transform will not move the actor in the world!
    ///
    /// Setting an unrealistic center of mass which is a long way from the body can make it difficult for
    /// the SDK to solve constraints. Perhaps leading to instability and jittering bodies.
    ///
    /// Default:
    /// the identity transform
    PxRigidBody_setCMassLocalPose_mut :: proc(self_: ^PxRigidBody, pose: ^PxTransform) ---

    /// Retrieves the center of mass pose relative to the actor frame.
    ///
    /// The center of mass pose relative to the actor frame.
    PxRigidBody_getCMassLocalPose :: proc(self_: ^PxRigidBody) -> PxTransform ---

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
    PxRigidBody_setMass_mut :: proc(self_: ^PxRigidBody, mass: _c.float) ---

    /// Retrieves the mass of the actor.
    ///
    /// A value of 0 is interpreted as infinite mass.
    ///
    /// The mass of this actor.
    PxRigidBody_getMass :: proc(self_: ^PxRigidBody) -> _c.float ---

    /// Retrieves the inverse mass of the actor.
    ///
    /// The inverse mass of this actor.
    PxRigidBody_getInvMass :: proc(self_: ^PxRigidBody) -> _c.float ---

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
    PxRigidBody_setMassSpaceInertiaTensor_mut :: proc(self_: ^PxRigidBody, m: ^PxVec3) ---

    /// Retrieves the diagonal inertia tensor of the actor relative to the mass coordinate frame.
    ///
    /// This method retrieves a mass frame inertia vector.
    ///
    /// The mass space inertia tensor of this actor.
    ///
    /// A value of 0 in an element is interpreted as infinite inertia along that axis.
    PxRigidBody_getMassSpaceInertiaTensor :: proc(self_: ^PxRigidBody) -> PxVec3 ---

    /// Retrieves the diagonal inverse inertia tensor of the actor relative to the mass coordinate frame.
    ///
    /// This method retrieves a mass frame inverse inertia vector.
    ///
    /// A value of 0 in an element is interpreted as infinite inertia along that axis.
    ///
    /// The mass space inverse inertia tensor of this actor.
    PxRigidBody_getMassSpaceInvInertiaTensor :: proc(self_: ^PxRigidBody) -> PxVec3 ---

    /// Sets the linear damping coefficient.
    ///
    /// Zero represents no damping. The damping coefficient must be nonnegative.
    ///
    /// Default:
    /// 0.0
    PxRigidBody_setLinearDamping_mut :: proc(self_: ^PxRigidBody, linDamp: _c.float) ---

    /// Retrieves the linear damping coefficient.
    ///
    /// The linear damping coefficient associated with this actor.
    PxRigidBody_getLinearDamping :: proc(self_: ^PxRigidBody) -> _c.float ---

    /// Sets the angular damping coefficient.
    ///
    /// Zero represents no damping.
    ///
    /// The angular damping coefficient must be nonnegative.
    ///
    /// Default:
    /// 0.05
    PxRigidBody_setAngularDamping_mut :: proc(self_: ^PxRigidBody, angDamp: _c.float) ---

    /// Retrieves the angular damping coefficient.
    ///
    /// The angular damping coefficient associated with this actor.
    PxRigidBody_getAngularDamping :: proc(self_: ^PxRigidBody) -> _c.float ---

    /// Retrieves the linear velocity of an actor.
    ///
    /// It is not allowed to use this method while the simulation is running (except during PxScene::collide(),
    /// in PxContactModifyCallback or in contact report callbacks).
    ///
    /// The linear velocity of the actor.
    PxRigidBody_getLinearVelocity :: proc(self_: ^PxRigidBody) -> PxVec3 ---

    /// Retrieves the angular velocity of the actor.
    ///
    /// It is not allowed to use this method while the simulation is running (except during PxScene::collide(),
    /// in PxContactModifyCallback or in contact report callbacks).
    ///
    /// The angular velocity of the actor.
    PxRigidBody_getAngularVelocity :: proc(self_: ^PxRigidBody) -> PxVec3 ---

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
    PxRigidBody_setMaxLinearVelocity_mut :: proc(self_: ^PxRigidBody, maxLinVel: _c.float) ---

    /// Retrieves the maximum angular velocity permitted for this actor.
    ///
    /// The maximum allowed angular velocity for this actor.
    PxRigidBody_getMaxLinearVelocity :: proc(self_: ^PxRigidBody) -> _c.float ---

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
    PxRigidBody_setMaxAngularVelocity_mut :: proc(self_: ^PxRigidBody, maxAngVel: _c.float) ---

    /// Retrieves the maximum angular velocity permitted for this actor.
    ///
    /// The maximum allowed angular velocity for this actor.
    PxRigidBody_getMaxAngularVelocity :: proc(self_: ^PxRigidBody) -> _c.float ---

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
    PxRigidBody_addForce_mut :: proc(self_: ^PxRigidBody, force: ^PxVec3, mode: _c.int32_t, autowake: _c.bool) ---

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
    PxRigidBody_addTorque_mut :: proc(self_: ^PxRigidBody, torque: ^PxVec3, mode: _c.int32_t, autowake: _c.bool) ---

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
    PxRigidBody_clearForce_mut :: proc(self_: ^PxRigidBody, mode: _c.int32_t) ---

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
    PxRigidBody_clearTorque_mut :: proc(self_: ^PxRigidBody, mode: _c.int32_t) ---

    /// Sets the impulsive force and torque defined in the global coordinate frame to the actor.
    ///
    /// ::PxForceMode determines if the cleared torque is to be conventional or impulsive.
    ///
    /// The force modes PxForceMode::eIMPULSE and PxForceMode::eVELOCITY_CHANGE can not be applied to articulation links.
    ///
    /// It is invalid to use this method if the actor has not been added to a scene already or if PxActorFlag::eDISABLE_SIMULATION is set.
    PxRigidBody_setForceAndTorque_mut :: proc(self_: ^PxRigidBody, force: ^PxVec3, torque: ^PxVec3, mode: _c.int32_t) ---

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
    PxRigidBody_setRigidBodyFlag_mut :: proc(self_: ^PxRigidBody, flag: _c.int32_t, value: _c.bool) ---

    PxRigidBody_setRigidBodyFlags_mut :: proc(self_: ^PxRigidBody, inFlags: _c.uint16_t) ---

    /// Reads the PxRigidBody flags.
    ///
    /// See the list of flags [`PxRigidBodyFlag`]
    ///
    /// The values of the PxRigidBody flags.
    PxRigidBody_getRigidBodyFlags :: proc(self_: ^PxRigidBody) -> _c.uint16_t ---

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
    PxRigidBody_setMinCCDAdvanceCoefficient_mut :: proc(self_: ^PxRigidBody, advanceCoefficient: _c.float) ---

    /// Gets the CCD minimum advance coefficient.
    ///
    /// The value of the CCD min advance coefficient.
    PxRigidBody_getMinCCDAdvanceCoefficient :: proc(self_: ^PxRigidBody) -> _c.float ---

    /// Sets the maximum depenetration velocity permitted to be introduced by the solver.
    /// This value controls how much velocity the solver can introduce to correct for penetrations in contacts.
    PxRigidBody_setMaxDepenetrationVelocity_mut :: proc(self_: ^PxRigidBody, biasClamp: _c.float) ---

    /// Returns the maximum depenetration velocity the solver is permitted to introduced.
    /// This value controls how much velocity the solver can introduce to correct for penetrations in contacts.
    ///
    /// The maximum penetration bias applied by the solver.
    PxRigidBody_getMaxDepenetrationVelocity :: proc(self_: ^PxRigidBody) -> _c.float ---

    /// Sets a limit on the impulse that may be applied at a contact. The maximum impulse at a contact between two dynamic or kinematic
    /// bodies will be the minimum of the two limit values. For a collision between a static and a dynamic body, the impulse is limited
    /// by the value for the dynamic body.
    PxRigidBody_setMaxContactImpulse_mut :: proc(self_: ^PxRigidBody, maxImpulse: _c.float) ---

    /// Returns the maximum impulse that may be applied at a contact.
    ///
    /// The maximum impulse that may be applied at a contact
    PxRigidBody_getMaxContactImpulse :: proc(self_: ^PxRigidBody) -> _c.float ---

    /// Sets a distance scale whereby the angular influence of a contact on the normal constraint in a contact is
    /// zeroed if normal.cross(offset) falls below this tolerance. Rather than acting as an absolute value, this tolerance
    /// is scaled by the ratio rXn.dot(angVel)/normal.dot(linVel) such that contacts that have relatively larger angular velocity
    /// than linear normal velocity (e.g. rolling wheels) achieve larger slop values as the angular velocity increases.
    PxRigidBody_setContactSlopCoefficient_mut :: proc(self_: ^PxRigidBody, slopCoefficient: _c.float) ---

    /// Returns the contact slop coefficient.
    ///
    /// The contact slop coefficient.
    PxRigidBody_getContactSlopCoefficient :: proc(self_: ^PxRigidBody) -> _c.float ---

    /// Returns the island node index
    ///
    /// The island node index.
    PxRigidBody_getInternalIslandNodeIndex :: proc(self_: ^PxRigidBody) -> PxNodeIndex ---

    /// Releases the link from the articulation.
    ///
    /// Only a leaf articulation link can be released.
    ///
    /// Releasing a link is not allowed while the articulation link is in a scene. In order to release a link,
    /// remove and then re-add the corresponding articulation to the scene.
    PxArticulationLink_release_mut :: proc(self_: ^PxArticulationLink) ---

    /// Gets the articulation that the link is a part of.
    ///
    /// The articulation.
    PxArticulationLink_getArticulation :: proc(self_: ^PxArticulationLink) -> ^PxArticulationReducedCoordinate ---

    /// Gets the joint which connects this link to its parent.
    ///
    /// The joint connecting the link to the parent. NULL for the root link.
    PxArticulationLink_getInboundJoint :: proc(self_: ^PxArticulationLink) -> ^PxArticulationJointReducedCoordinate ---

    /// Gets the number of degrees of freedom of the joint which connects this link to its parent.
    ///
    /// - The root link DOF-count is defined to be 0 regardless of PxArticulationFlag::eFIX_BASE.
    /// - The return value is only valid for articulations that are in a scene.
    ///
    /// The number of degrees of freedom, or 0xFFFFFFFF if the articulation is not in a scene.
    PxArticulationLink_getInboundJointDof :: proc(self_: ^PxArticulationLink) -> _c.uint32_t ---

    /// Gets the number of child links.
    ///
    /// The number of child links.
    PxArticulationLink_getNbChildren :: proc(self_: ^PxArticulationLink) -> _c.uint32_t ---

    /// Gets the low-level link index that may be used to index into members of PxArticulationCache.
    ///
    /// The return value is only valid for articulations that are in a scene.
    ///
    /// The low-level index, or 0xFFFFFFFF if the articulation is not in a scene.
    PxArticulationLink_getLinkIndex :: proc(self_: ^PxArticulationLink) -> _c.uint32_t ---

    /// Retrieves the child links.
    ///
    /// The number of articulation links written to the buffer.
    PxArticulationLink_getChildren :: proc(self_: ^PxArticulationLink, userBuffer: ^^PxArticulationLink, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

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
    PxArticulationLink_setCfmScale_mut :: proc(self_: ^PxArticulationLink, cfm: _c.float) ---

    /// Get the constraint-force-mixing scale term.
    ///
    /// The constraint-force-mixing scale term.
    PxArticulationLink_getCfmScale :: proc(self_: ^PxArticulationLink) -> _c.float ---

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
    PxArticulationLink_getLinearVelocity :: proc(self_: ^PxArticulationLink) -> PxVec3 ---

    /// Get the angular velocity of the link.
    ///
    /// - For performance, prefer PxArticulationCache::linkVelocity to get link spatial velocities in a batch query.
    /// - When the articulation state is updated via non-cache API, use PxArticulationReducedCoordinate::updateKinematic before querying velocity.
    ///
    /// The angular velocity of the link.
    ///
    /// This call is not allowed while the simulation is running except in a split simulation during [`PxScene::collide`]() and up to #PxScene::advance(),
    /// and in PxContactModifyCallback or in contact report callbacks.
    PxArticulationLink_getAngularVelocity :: proc(self_: ^PxArticulationLink) -> PxVec3 ---

    /// Returns the string name of the dynamic type.
    ///
    /// The string name.
    PxArticulationLink_getConcreteTypeName :: proc(self_: ^PxArticulationLink) -> ^_c.char ---

    PxConeLimitedConstraint_new :: proc() -> PxConeLimitedConstraint ---

    /// Releases a PxConstraint instance.
    ///
    /// This call does not wake up the connected rigid bodies.
    PxConstraint_release_mut :: proc(self_: ^PxConstraint) ---

    /// Retrieves the scene which this constraint belongs to.
    ///
    /// Owner Scene. NULL if not part of a scene.
    PxConstraint_getScene :: proc(self_: ^PxConstraint) -> ^PxScene ---

    /// Retrieves the actors for this constraint.
    PxConstraint_getActors :: proc(self_: ^PxConstraint, actor0: ^^PxRigidActor, actor1: ^^PxRigidActor) ---

    /// Sets the actors for this constraint.
    PxConstraint_setActors_mut :: proc(self_: ^PxConstraint, actor0: ^PxRigidActor, actor1: ^PxRigidActor) ---

    /// Notify the scene that the constraint shader data has been updated by the application
    PxConstraint_markDirty_mut :: proc(self_: ^PxConstraint) ---

    /// Retrieve the flags for this constraint
    ///
    /// the constraint flags
    PxConstraint_getFlags :: proc(self_: ^PxConstraint) -> _c.uint16_t ---

    /// Set the flags for this constraint
    ///
    /// default: PxConstraintFlag::eDRIVE_LIMITS_ARE_FORCES
    PxConstraint_setFlags_mut :: proc(self_: ^PxConstraint, flags: _c.uint16_t) ---

    /// Set a flag for this constraint
    PxConstraint_setFlag_mut :: proc(self_: ^PxConstraint, flag: _c.int32_t, value: _c.bool) ---

    /// Retrieve the constraint force most recently applied to maintain this constraint.
    ///
    /// It is not allowed to use this method while the simulation is running (except during PxScene::collide(),
    /// in PxContactModifyCallback or in contact report callbacks).
    PxConstraint_getForce :: proc(self_: ^PxConstraint, linear: ^PxVec3, angular: ^PxVec3) ---

    /// whether the constraint is valid.
    ///
    /// A constraint is valid if it has at least one dynamic rigid body or articulation link. A constraint that
    /// is not valid may not be inserted into a scene, and therefore a static actor to which an invalid constraint
    /// is attached may not be inserted into a scene.
    ///
    /// Invalid constraints arise only when an actor to which the constraint is attached has been deleted.
    PxConstraint_isValid :: proc(self_: ^PxConstraint) -> _c.bool ---

    /// Set the break force and torque thresholds for this constraint.
    ///
    /// If either the force or torque measured at the constraint exceed these thresholds the constraint will break.
    PxConstraint_setBreakForce_mut :: proc(self_: ^PxConstraint, linear: _c.float, angular: _c.float) ---

    /// Retrieve the constraint break force and torque thresholds
    PxConstraint_getBreakForce :: proc(self_: ^PxConstraint, linear: ^_c.float, angular: ^_c.float) ---

    /// Set the minimum response threshold for a constraint row
    ///
    /// When using mass modification for a joint or infinite inertia for a jointed body, very stiff solver constraints can be generated which
    /// can destabilize simulation. Setting this value to a small positive value (e.g. 1e-8) will cause constraint rows to be ignored if very
    /// large changes in impulses will generate only small changes in velocity. When setting this value, also set
    /// PxConstraintFlag::eDISABLE_PREPROCESSING. The solver accuracy for this joint may be reduced.
    PxConstraint_setMinResponseThreshold_mut :: proc(self_: ^PxConstraint, threshold: _c.float) ---

    /// Retrieve the constraint break force and torque thresholds
    ///
    /// the minimum response threshold for a constraint row
    PxConstraint_getMinResponseThreshold :: proc(self_: ^PxConstraint) -> _c.float ---

    /// Fetch external owner of the constraint.
    ///
    /// Provides a reference to the external owner of a constraint and a unique owner type ID.
    ///
    /// Reference to the external object which owns the constraint.
    PxConstraint_getExternalReference_mut :: proc(self_: ^PxConstraint, typeID: ^_c.uint32_t) -> rawptr ---

    /// Set the constraint functions for this constraint
    PxConstraint_setConstraintFunctions_mut :: proc(self_: ^PxConstraint, connector: ^PxConstraintConnector, shaders: ^PxConstraintShaderTable) ---

    PxConstraint_getConcreteTypeName :: proc(self_: ^PxConstraint) -> ^_c.char ---

    /// Constructor
    PxContactStreamIterator_new :: proc(contactPatches: ^_c.uint8_t, contactPoints: ^_c.uint8_t, contactFaceIndices: ^_c.uint32_t, nbPatches: _c.uint32_t, nbContacts: _c.uint32_t) -> PxContactStreamIterator ---

    /// Returns whether there are more patches in this stream.
    ///
    /// Whether there are more patches in this stream.
    PxContactStreamIterator_hasNextPatch :: proc(self_: ^PxContactStreamIterator) -> _c.bool ---

    /// Returns the total contact count.
    ///
    /// Total contact count.
    PxContactStreamIterator_getTotalContactCount :: proc(self_: ^PxContactStreamIterator) -> _c.uint32_t ---

    /// Returns the total patch count.
    ///
    /// Total patch count.
    PxContactStreamIterator_getTotalPatchCount :: proc(self_: ^PxContactStreamIterator) -> _c.uint32_t ---

    /// Advances iterator to next contact patch.
    PxContactStreamIterator_nextPatch_mut :: proc(self_: ^PxContactStreamIterator) ---

    /// Returns if the current patch has more contacts.
    ///
    /// If there are more contacts in the current patch.
    PxContactStreamIterator_hasNextContact :: proc(self_: ^PxContactStreamIterator) -> _c.bool ---

    /// Advances to the next contact in the patch.
    PxContactStreamIterator_nextContact_mut :: proc(self_: ^PxContactStreamIterator) ---

    /// Gets the current contact's normal
    ///
    /// The current contact's normal.
    PxContactStreamIterator_getContactNormal :: proc(self_: ^PxContactStreamIterator) -> ^PxVec3 ---

    /// Gets the inverse mass scale for body 0.
    ///
    /// The inverse mass scale for body 0.
    PxContactStreamIterator_getInvMassScale0 :: proc(self_: ^PxContactStreamIterator) -> _c.float ---

    /// Gets the inverse mass scale for body 1.
    ///
    /// The inverse mass scale for body 1.
    PxContactStreamIterator_getInvMassScale1 :: proc(self_: ^PxContactStreamIterator) -> _c.float ---

    /// Gets the inverse inertia scale for body 0.
    ///
    /// The inverse inertia scale for body 0.
    PxContactStreamIterator_getInvInertiaScale0 :: proc(self_: ^PxContactStreamIterator) -> _c.float ---

    /// Gets the inverse inertia scale for body 1.
    ///
    /// The inverse inertia scale for body 1.
    PxContactStreamIterator_getInvInertiaScale1 :: proc(self_: ^PxContactStreamIterator) -> _c.float ---

    /// Gets the contact's max impulse.
    ///
    /// The contact's max impulse.
    PxContactStreamIterator_getMaxImpulse :: proc(self_: ^PxContactStreamIterator) -> _c.float ---

    /// Gets the contact's target velocity.
    ///
    /// The contact's target velocity.
    PxContactStreamIterator_getTargetVel :: proc(self_: ^PxContactStreamIterator) -> ^PxVec3 ---

    /// Gets the contact's contact point.
    ///
    /// The contact's contact point.
    PxContactStreamIterator_getContactPoint :: proc(self_: ^PxContactStreamIterator) -> ^PxVec3 ---

    /// Gets the contact's separation.
    ///
    /// The contact's separation.
    PxContactStreamIterator_getSeparation :: proc(self_: ^PxContactStreamIterator) -> _c.float ---

    /// Gets the contact's face index for shape 0.
    ///
    /// The contact's face index for shape 0.
    PxContactStreamIterator_getFaceIndex0 :: proc(self_: ^PxContactStreamIterator) -> _c.uint32_t ---

    /// Gets the contact's face index for shape 1.
    ///
    /// The contact's face index for shape 1.
    PxContactStreamIterator_getFaceIndex1 :: proc(self_: ^PxContactStreamIterator) -> _c.uint32_t ---

    /// Gets the contact's static friction coefficient.
    ///
    /// The contact's static friction coefficient.
    PxContactStreamIterator_getStaticFriction :: proc(self_: ^PxContactStreamIterator) -> _c.float ---

    /// Gets the contact's dynamic friction coefficient.
    ///
    /// The contact's dynamic friction coefficient.
    PxContactStreamIterator_getDynamicFriction :: proc(self_: ^PxContactStreamIterator) -> _c.float ---

    /// Gets the contact's restitution coefficient.
    ///
    /// The contact's restitution coefficient.
    PxContactStreamIterator_getRestitution :: proc(self_: ^PxContactStreamIterator) -> _c.float ---

    /// Gets the contact's damping value.
    ///
    /// The contact's damping value.
    PxContactStreamIterator_getDamping :: proc(self_: ^PxContactStreamIterator) -> _c.float ---

    /// Gets the contact's material flags.
    ///
    /// The contact's material flags.
    PxContactStreamIterator_getMaterialFlags :: proc(self_: ^PxContactStreamIterator) -> _c.uint32_t ---

    /// Gets the contact's material index for shape 0.
    ///
    /// The contact's material index for shape 0.
    PxContactStreamIterator_getMaterialIndex0 :: proc(self_: ^PxContactStreamIterator) -> _c.uint16_t ---

    /// Gets the contact's material index for shape 1.
    ///
    /// The contact's material index for shape 1.
    PxContactStreamIterator_getMaterialIndex1 :: proc(self_: ^PxContactStreamIterator) -> _c.uint16_t ---

    /// Advances the contact stream iterator to a specific contact index.
    ///
    /// True if advancing was possible
    PxContactStreamIterator_advanceToIndex_mut :: proc(self_: ^PxContactStreamIterator, initialIndex: _c.uint32_t) -> _c.bool ---

    /// Get the position of a specific contact point in the set.
    ///
    /// Position to the requested point in world space
    PxContactSet_getPoint :: proc(self_: ^PxContactSet, i: _c.uint32_t) -> ^PxVec3 ---

    /// Alter the position of a specific contact point in the set.
    PxContactSet_setPoint_mut :: proc(self_: ^PxContactSet, i: _c.uint32_t, p: ^PxVec3) ---

    /// Get the contact normal of a specific contact point in the set.
    ///
    /// The requested normal in world space
    PxContactSet_getNormal :: proc(self_: ^PxContactSet, i: _c.uint32_t) -> ^PxVec3 ---

    /// Alter the contact normal of a specific contact point in the set.
    ///
    /// Changing the normal can cause contact points to be ignored.
    PxContactSet_setNormal_mut :: proc(self_: ^PxContactSet, i: _c.uint32_t, n: ^PxVec3) ---

    /// Get the separation distance of a specific contact point in the set.
    ///
    /// The separation. Negative implies penetration.
    PxContactSet_getSeparation :: proc(self_: ^PxContactSet, i: _c.uint32_t) -> _c.float ---

    /// Alter the separation of a specific contact point in the set.
    PxContactSet_setSeparation_mut :: proc(self_: ^PxContactSet, i: _c.uint32_t, s: _c.float) ---

    /// Get the target velocity of a specific contact point in the set.
    ///
    /// The target velocity in world frame
    PxContactSet_getTargetVelocity :: proc(self_: ^PxContactSet, i: _c.uint32_t) -> ^PxVec3 ---

    /// Alter the target velocity of a specific contact point in the set.
    PxContactSet_setTargetVelocity_mut :: proc(self_: ^PxContactSet, i: _c.uint32_t, v: ^PxVec3) ---

    /// Get the face index with respect to the first shape of the pair for a specific contact point in the set.
    ///
    /// The face index of the first shape
    ///
    /// At the moment, the first shape is never a tri-mesh, therefore this function always returns PXC_CONTACT_NO_FACE_INDEX
    PxContactSet_getInternalFaceIndex0 :: proc(self_: ^PxContactSet, i: _c.uint32_t) -> _c.uint32_t ---

    /// Get the face index with respect to the second shape of the pair for a specific contact point in the set.
    ///
    /// The face index of the second shape
    PxContactSet_getInternalFaceIndex1 :: proc(self_: ^PxContactSet, i: _c.uint32_t) -> _c.uint32_t ---

    /// Get the maximum impulse for a specific contact point in the set.
    ///
    /// The maximum impulse
    PxContactSet_getMaxImpulse :: proc(self_: ^PxContactSet, i: _c.uint32_t) -> _c.float ---

    /// Alter the maximum impulse for a specific contact point in the set.
    ///
    /// Must be nonnegative. If set to zero, the contact point will be ignored
    PxContactSet_setMaxImpulse_mut :: proc(self_: ^PxContactSet, i: _c.uint32_t, s: _c.float) ---

    /// Get the restitution coefficient for a specific contact point in the set.
    ///
    /// The restitution coefficient
    PxContactSet_getRestitution :: proc(self_: ^PxContactSet, i: _c.uint32_t) -> _c.float ---

    /// Alter the restitution coefficient for a specific contact point in the set.
    ///
    /// Valid ranges [0,1]
    PxContactSet_setRestitution_mut :: proc(self_: ^PxContactSet, i: _c.uint32_t, r: _c.float) ---

    /// Get the static friction coefficient for a specific contact point in the set.
    ///
    /// The friction coefficient (dimensionless)
    PxContactSet_getStaticFriction :: proc(self_: ^PxContactSet, i: _c.uint32_t) -> _c.float ---

    /// Alter the static friction coefficient for a specific contact point in the set.
    PxContactSet_setStaticFriction_mut :: proc(self_: ^PxContactSet, i: _c.uint32_t, f: _c.float) ---

    /// Get the static friction coefficient for a specific contact point in the set.
    ///
    /// The friction coefficient
    PxContactSet_getDynamicFriction :: proc(self_: ^PxContactSet, i: _c.uint32_t) -> _c.float ---

    /// Alter the static dynamic coefficient for a specific contact point in the set.
    PxContactSet_setDynamicFriction_mut :: proc(self_: ^PxContactSet, i: _c.uint32_t, f: _c.float) ---

    /// Ignore the contact point.
    ///
    /// If a contact point is ignored then no force will get applied at this point. This can be used to disable collision in certain areas of a shape, for example.
    PxContactSet_ignore_mut :: proc(self_: ^PxContactSet, i: _c.uint32_t) ---

    /// The number of contact points in the set.
    PxContactSet_size :: proc(self_: ^PxContactSet) -> _c.uint32_t ---

    /// Returns the invMassScale of body 0
    ///
    /// A value
    /// <
    /// 1.0 makes this contact treat the body as if it had larger mass. A value of 0.f makes this contact
    /// treat the body as if it had infinite mass. Any value > 1.f makes this contact treat the body as if it had smaller mass.
    PxContactSet_getInvMassScale0 :: proc(self_: ^PxContactSet) -> _c.float ---

    /// Returns the invMassScale of body 1
    ///
    /// A value
    /// <
    /// 1.0 makes this contact treat the body as if it had larger mass. A value of 0.f makes this contact
    /// treat the body as if it had infinite mass. Any value > 1.f makes this contact treat the body as if it had smaller mass.
    PxContactSet_getInvMassScale1 :: proc(self_: ^PxContactSet) -> _c.float ---

    /// Returns the invInertiaScale of body 0
    ///
    /// A value
    /// <
    /// 1.0 makes this contact treat the body as if it had larger inertia. A value of 0.f makes this contact
    /// treat the body as if it had infinite inertia. Any value > 1.f makes this contact treat the body as if it had smaller inertia.
    PxContactSet_getInvInertiaScale0 :: proc(self_: ^PxContactSet) -> _c.float ---

    /// Returns the invInertiaScale of body 1
    ///
    /// A value
    /// <
    /// 1.0 makes this contact treat the body as if it had larger inertia. A value of 0.f makes this contact
    /// treat the body as if it had infinite inertia. Any value > 1.f makes this contact treat the body as if it had smaller inertia.
    PxContactSet_getInvInertiaScale1 :: proc(self_: ^PxContactSet) -> _c.float ---

    /// Sets the invMassScale of body 0
    ///
    /// This can be set to any value in the range [0, PX_MAX_F32). A value
    /// <
    /// 1.0 makes this contact treat the body as if it had larger mass. A value of 0.f makes this contact
    /// treat the body as if it had infinite mass. Any value > 1.f makes this contact treat the body as if it had smaller mass.
    PxContactSet_setInvMassScale0_mut :: proc(self_: ^PxContactSet, scale: _c.float) ---

    /// Sets the invMassScale of body 1
    ///
    /// This can be set to any value in the range [0, PX_MAX_F32). A value
    /// <
    /// 1.0 makes this contact treat the body as if it had larger mass. A value of 0.f makes this contact
    /// treat the body as if it had infinite mass. Any value > 1.f makes this contact treat the body as if it had smaller mass.
    PxContactSet_setInvMassScale1_mut :: proc(self_: ^PxContactSet, scale: _c.float) ---

    /// Sets the invInertiaScale of body 0
    ///
    /// This can be set to any value in the range [0, PX_MAX_F32). A value
    /// <
    /// 1.0 makes this contact treat the body as if it had larger inertia. A value of 0.f makes this contact
    /// treat the body as if it had infinite inertia. Any value > 1.f makes this contact treat the body as if it had smaller inertia.
    PxContactSet_setInvInertiaScale0_mut :: proc(self_: ^PxContactSet, scale: _c.float) ---

    /// Sets the invInertiaScale of body 1
    ///
    /// This can be set to any value in the range [0, PX_MAX_F32). A value
    /// <
    /// 1.0 makes this contact treat the body as if it had larger inertia. A value of 0.f makes this contact
    /// treat the body as if it had infinite inertia. Any value > 1.f makes this contact treat the body as if it had smaller inertia.
    PxContactSet_setInvInertiaScale1_mut :: proc(self_: ^PxContactSet, scale: _c.float) ---

    /// Passes modifiable arrays of contacts to the application.
    ///
    /// The initial contacts are regenerated from scratch each frame by collision detection.
    ///
    /// The number of contacts can not be changed, so you cannot add your own contacts.  You may however
    /// disable contacts using PxContactSet::ignore().
    PxContactModifyCallback_onContactModify_mut :: proc(self_: ^PxContactModifyCallback, pairs: ^PxContactModifyPair, count: _c.uint32_t) ---

    /// Passes modifiable arrays of contacts to the application.
    ///
    /// The initial contacts are regenerated from scratch each frame by collision detection.
    ///
    /// The number of contacts can not be changed, so you cannot add your own contacts.  You may however
    /// disable contacts using PxContactSet::ignore().
    PxCCDContactModifyCallback_onCCDContactModify_mut :: proc(self_: ^PxCCDContactModifyCallback, pairs: ^PxContactModifyPair, count: _c.uint32_t) ---

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
    PxDeletionListener_onRelease_mut :: proc(self_: ^PxDeletionListener, observed: ^PxBase, userData: rawptr, deletionEvent: _c.int32_t) ---

    PxBaseMaterial_isKindOf :: proc(self_: ^PxBaseMaterial, name: ^_c.char) -> _c.bool ---

    /// Sets young's modulus which defines the body's stiffness
    PxFEMMaterial_setYoungsModulus_mut :: proc(self_: ^PxFEMMaterial, young: _c.float) ---

    /// Retrieves the young's modulus value.
    ///
    /// The young's modulus value.
    PxFEMMaterial_getYoungsModulus :: proc(self_: ^PxFEMMaterial) -> _c.float ---

    /// Sets the Poisson's ratio which defines the body's volume preservation. Completely incompressible materials have a poisson ratio of 0.5. Its value should not be set to exactly 0.5 because this leads to numerical problems.
    PxFEMMaterial_setPoissons_mut :: proc(self_: ^PxFEMMaterial, poisson: _c.float) ---

    /// Retrieves the Poisson's ratio.
    ///
    /// The Poisson's ratio.
    PxFEMMaterial_getPoissons :: proc(self_: ^PxFEMMaterial) -> _c.float ---

    /// Sets the dynamic friction value which defines the strength of resistance when two objects slide relative to each other while in contact.
    PxFEMMaterial_setDynamicFriction_mut :: proc(self_: ^PxFEMMaterial, dynamicFriction: _c.float) ---

    /// Retrieves the dynamic friction value
    ///
    /// The dynamic friction value
    PxFEMMaterial_getDynamicFriction :: proc(self_: ^PxFEMMaterial) -> _c.float ---

    PxFilterData_new :: proc(anon_param0: _c.int32_t) -> PxFilterData ---

    /// Default constructor.
    PxFilterData_new_1 :: proc() -> PxFilterData ---

    /// Constructor to set filter data initially.
    PxFilterData_new_2 :: proc(w0: _c.uint32_t, w1: _c.uint32_t, w2: _c.uint32_t, w3: _c.uint32_t) -> PxFilterData ---

    /// (re)sets the structure to the default.
    PxFilterData_setToDefault_mut :: proc(self_: ^PxFilterData) ---

    /// Extract filter object type from the filter attributes of a collision pair object
    ///
    /// The type of the collision pair object.
    phys_PxGetFilterObjectType :: proc(attr: _c.uint32_t) -> _c.int32_t ---

    /// Specifies whether the collision object belongs to a kinematic rigid body
    ///
    /// True if the object belongs to a kinematic rigid body, else false
    phys_PxFilterObjectIsKinematic :: proc(attr: _c.uint32_t) -> _c.bool ---

    /// Specifies whether the collision object is a trigger shape
    ///
    /// True if the object is a trigger shape, else false
    phys_PxFilterObjectIsTrigger :: proc(attr: _c.uint32_t) -> _c.bool ---

    /// Filter method to specify how a pair of potentially colliding objects should be processed.
    ///
    /// This method gets called when the filter flags returned by the filter shader (see [`PxSimulationFilterShader`])
    /// indicate that the filter callback should be invoked ([`PxFilterFlag::eCALLBACK`] or #PxFilterFlag::eNOTIFY set).
    /// Return the PxFilterFlag flags and set the PxPairFlag flags to define what the simulation should do with the given
    /// collision pair.
    ///
    /// Filter flags defining whether the pair should be discarded, temporarily ignored or processed and whether the pair
    /// should be tracked and send a report on pair deletion through the filter callback
    PxSimulationFilterCallback_pairFound_mut :: proc(self_: ^PxSimulationFilterCallback, pairID: _c.uint32_t, attributes0: _c.uint32_t, filterData0: PxFilterData, a0: ^PxActor, s0: ^PxShape, attributes1: _c.uint32_t, filterData1: PxFilterData, a1: ^PxActor, s1: ^PxShape, pairFlags: ^_c.uint16_t) -> _c.uint16_t ---

    /// Callback to inform that a tracked collision pair is gone.
    ///
    /// This method gets called when a collision pair disappears or gets re-filtered. Only applies to
    /// collision pairs which have been marked as filter callback pairs ([`PxFilterFlag::eNOTIFY`] set in #pairFound()).
    PxSimulationFilterCallback_pairLost_mut :: proc(self_: ^PxSimulationFilterCallback, pairID: _c.uint32_t, attributes0: _c.uint32_t, filterData0: PxFilterData, attributes1: _c.uint32_t, filterData1: PxFilterData, objectRemoved: _c.bool) ---

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
    PxSimulationFilterCallback_statusChange_mut :: proc(self_: ^PxSimulationFilterCallback, pairID: ^_c.uint32_t, pairFlags: ^_c.uint16_t, filterFlags: ^_c.uint16_t) -> _c.bool ---

    /// Any combination of PxDataAccessFlag::eREADABLE and PxDataAccessFlag::eWRITABLE
    PxLockedData_getDataAccessFlags_mut :: proc(self_: ^PxLockedData) -> _c.uint8_t ---

    /// Unlocks the bulk data.
    PxLockedData_unlock_mut :: proc(self_: ^PxLockedData) ---

    /// virtual destructor
    PxLockedData_delete :: proc(self_: ^PxLockedData) ---

    /// Sets the coefficient of dynamic friction.
    ///
    /// The coefficient of dynamic friction should be in [0, PX_MAX_F32). If set to greater than staticFriction, the effective value of staticFriction will be increased to match.
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake any actors which may be affected.
    PxMaterial_setDynamicFriction_mut :: proc(self_: ^PxMaterial, coef: _c.float) ---

    /// Retrieves the DynamicFriction value.
    ///
    /// The coefficient of dynamic friction.
    PxMaterial_getDynamicFriction :: proc(self_: ^PxMaterial) -> _c.float ---

    /// Sets the coefficient of static friction
    ///
    /// The coefficient of static friction should be in the range [0, PX_MAX_F32)
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake any actors which may be affected.
    PxMaterial_setStaticFriction_mut :: proc(self_: ^PxMaterial, coef: _c.float) ---

    /// Retrieves the coefficient of static friction.
    ///
    /// The coefficient of static friction.
    PxMaterial_getStaticFriction :: proc(self_: ^PxMaterial) -> _c.float ---

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
    PxMaterial_setRestitution_mut :: proc(self_: ^PxMaterial, rest: _c.float) ---

    /// Retrieves the coefficient of restitution.
    ///
    /// See [`setRestitution`].
    ///
    /// The coefficient of restitution.
    PxMaterial_getRestitution :: proc(self_: ^PxMaterial) -> _c.float ---

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
    PxMaterial_setDamping_mut :: proc(self_: ^PxMaterial, damping: _c.float) ---

    /// Retrieves the coefficient of damping.
    ///
    /// See [`setDamping`].
    ///
    /// The coefficient of damping.
    PxMaterial_getDamping :: proc(self_: ^PxMaterial) -> _c.float ---

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
    PxMaterial_setFlag_mut :: proc(self_: ^PxMaterial, flag: _c.int32_t, b: _c.bool) ---

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
    PxMaterial_setFlags_mut :: proc(self_: ^PxMaterial, flags: _c.uint16_t) ---

    /// Retrieves the flags. See [`PxMaterialFlag`].
    ///
    /// The material flags.
    PxMaterial_getFlags :: proc(self_: ^PxMaterial) -> _c.uint16_t ---

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
    PxMaterial_setFrictionCombineMode_mut :: proc(self_: ^PxMaterial, combMode: _c.int32_t) ---

    /// Retrieves the friction combine mode.
    ///
    /// See [`setFrictionCombineMode`].
    ///
    /// The friction combine mode for this material.
    PxMaterial_getFrictionCombineMode :: proc(self_: ^PxMaterial) -> _c.int32_t ---

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
    PxMaterial_setRestitutionCombineMode_mut :: proc(self_: ^PxMaterial, combMode: _c.int32_t) ---

    /// Retrieves the restitution combine mode.
    ///
    /// See [`setRestitutionCombineMode`].
    ///
    /// The coefficient of restitution combine mode for this material.
    PxMaterial_getRestitutionCombineMode :: proc(self_: ^PxMaterial) -> _c.int32_t ---

    PxMaterial_getConcreteTypeName :: proc(self_: ^PxMaterial) -> ^_c.char ---

    /// Construct parameters with default values.
    PxDiffuseParticleParams_new :: proc() -> PxDiffuseParticleParams ---

    /// (re)sets the structure to the default.
    PxDiffuseParticleParams_setToDefault_mut :: proc(self_: ^PxDiffuseParticleParams) ---

    /// Sets friction
    PxParticleMaterial_setFriction_mut :: proc(self_: ^PxParticleMaterial, friction: _c.float) ---

    /// Retrieves the friction value.
    ///
    /// The friction value.
    PxParticleMaterial_getFriction :: proc(self_: ^PxParticleMaterial) -> _c.float ---

    /// Sets velocity damping term
    PxParticleMaterial_setDamping_mut :: proc(self_: ^PxParticleMaterial, damping: _c.float) ---

    /// Retrieves the velocity damping term
    ///
    /// The velocity damping term.
    PxParticleMaterial_getDamping :: proc(self_: ^PxParticleMaterial) -> _c.float ---

    /// Sets adhesion term
    PxParticleMaterial_setAdhesion_mut :: proc(self_: ^PxParticleMaterial, adhesion: _c.float) ---

    /// Retrieves the adhesion term
    ///
    /// The adhesion term.
    PxParticleMaterial_getAdhesion :: proc(self_: ^PxParticleMaterial) -> _c.float ---

    /// Sets gravity scale term
    PxParticleMaterial_setGravityScale_mut :: proc(self_: ^PxParticleMaterial, scale: _c.float) ---

    /// Retrieves the gravity scale term
    ///
    /// The gravity scale term.
    PxParticleMaterial_getGravityScale :: proc(self_: ^PxParticleMaterial) -> _c.float ---

    /// Sets material adhesion radius scale. This is multiplied by the particle rest offset to compute the fall-off distance
    /// at which point adhesion ceases to operate.
    PxParticleMaterial_setAdhesionRadiusScale_mut :: proc(self_: ^PxParticleMaterial, scale: _c.float) ---

    /// Retrieves the adhesion radius scale.
    ///
    /// The adhesion radius scale.
    PxParticleMaterial_getAdhesionRadiusScale :: proc(self_: ^PxParticleMaterial) -> _c.float ---

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
    PxPhysics_release_mut :: proc(self_: ^PxPhysics) ---

    /// Retrieves the Foundation instance.
    ///
    /// A reference to the Foundation object.
    PxPhysics_getFoundation_mut :: proc(self_: ^PxPhysics) -> ^PxFoundation ---

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
    PxPhysics_createAggregate_mut :: proc(self_: ^PxPhysics, maxActor: _c.uint32_t, maxShape: _c.uint32_t, filterHint: _c.uint32_t) -> ^PxAggregate ---

    /// Returns the simulation tolerance parameters.
    ///
    /// The current simulation tolerance parameters.
    PxPhysics_getTolerancesScale :: proc(self_: ^PxPhysics) -> ^PxTolerancesScale ---

    /// Creates a triangle mesh object.
    ///
    /// This can then be instanced into [`PxShape`] objects.
    ///
    /// The new triangle mesh.
    PxPhysics_createTriangleMesh_mut :: proc(self_: ^PxPhysics, stream: ^PxInputStream) -> ^PxTriangleMesh ---

    /// Return the number of triangle meshes that currently exist.
    ///
    /// Number of triangle meshes.
    PxPhysics_getNbTriangleMeshes :: proc(self_: ^PxPhysics) -> _c.uint32_t ---

    /// Writes the array of triangle mesh pointers to a user buffer.
    ///
    /// Returns the number of pointers written.
    ///
    /// The ordering of the triangle meshes in the array is not specified.
    ///
    /// The number of triangle mesh pointers written to userBuffer, this should be less or equal to bufferSize.
    PxPhysics_getTriangleMeshes :: proc(self_: ^PxPhysics, userBuffer: ^^PxTriangleMesh, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Creates a tetrahedron mesh object.
    ///
    /// This can then be instanced into [`PxShape`] objects.
    ///
    /// The new tetrahedron mesh.
    PxPhysics_createTetrahedronMesh_mut :: proc(self_: ^PxPhysics, stream: ^PxInputStream) -> ^PxTetrahedronMesh ---

    /// Creates a softbody mesh object.
    ///
    /// The new softbody mesh.
    PxPhysics_createSoftBodyMesh_mut :: proc(self_: ^PxPhysics, stream: ^PxInputStream) -> ^PxSoftBodyMesh ---

    /// Return the number of tetrahedron meshes that currently exist.
    ///
    /// Number of tetrahedron meshes.
    PxPhysics_getNbTetrahedronMeshes :: proc(self_: ^PxPhysics) -> _c.uint32_t ---

    /// Writes the array of tetrahedron mesh pointers to a user buffer.
    ///
    /// Returns the number of pointers written.
    ///
    /// The ordering of the tetrahedron meshes in the array is not specified.
    ///
    /// The number of tetrahedron mesh pointers written to userBuffer, this should be less or equal to bufferSize.
    PxPhysics_getTetrahedronMeshes :: proc(self_: ^PxPhysics, userBuffer: ^^PxTetrahedronMesh, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Creates a heightfield object from previously cooked stream.
    ///
    /// This can then be instanced into [`PxShape`] objects.
    ///
    /// The new heightfield.
    PxPhysics_createHeightField_mut :: proc(self_: ^PxPhysics, stream: ^PxInputStream) -> ^PxHeightField ---

    /// Return the number of heightfields that currently exist.
    ///
    /// Number of heightfields.
    PxPhysics_getNbHeightFields :: proc(self_: ^PxPhysics) -> _c.uint32_t ---

    /// Writes the array of heightfield pointers to a user buffer.
    ///
    /// Returns the number of pointers written.
    ///
    /// The ordering of the heightfields in the array is not specified.
    ///
    /// The number of heightfield pointers written to userBuffer, this should be less or equal to bufferSize.
    PxPhysics_getHeightFields :: proc(self_: ^PxPhysics, userBuffer: ^^PxHeightField, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Creates a convex mesh object.
    ///
    /// This can then be instanced into [`PxShape`] objects.
    ///
    /// The new convex mesh.
    PxPhysics_createConvexMesh_mut :: proc(self_: ^PxPhysics, stream: ^PxInputStream) -> ^PxConvexMesh ---

    /// Return the number of convex meshes that currently exist.
    ///
    /// Number of convex meshes.
    PxPhysics_getNbConvexMeshes :: proc(self_: ^PxPhysics) -> _c.uint32_t ---

    /// Writes the array of convex mesh pointers to a user buffer.
    ///
    /// Returns the number of pointers written.
    ///
    /// The ordering of the convex meshes in the array is not specified.
    ///
    /// The number of convex mesh pointers written to userBuffer, this should be less or equal to bufferSize.
    PxPhysics_getConvexMeshes :: proc(self_: ^PxPhysics, userBuffer: ^^PxConvexMesh, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Creates a bounding volume hierarchy.
    ///
    /// The new BVH.
    PxPhysics_createBVH_mut :: proc(self_: ^PxPhysics, stream: ^PxInputStream) -> ^PxBVH ---

    /// Return the number of bounding volume hierarchies that currently exist.
    ///
    /// Number of bounding volume hierarchies.
    PxPhysics_getNbBVHs :: proc(self_: ^PxPhysics) -> _c.uint32_t ---

    /// Writes the array of bounding volume hierarchy pointers to a user buffer.
    ///
    /// Returns the number of pointers written.
    ///
    /// The ordering of the BVHs in the array is not specified.
    ///
    /// The number of BVH pointers written to userBuffer, this should be less or equal to bufferSize.
    PxPhysics_getBVHs :: proc(self_: ^PxPhysics, userBuffer: ^^PxBVH, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Creates a scene.
    ///
    /// Every scene uses a Thread Local Storage slot. This imposes a platform specific limit on the
    /// number of scenes that can be created.
    ///
    /// The new scene object.
    PxPhysics_createScene_mut :: proc(self_: ^PxPhysics, sceneDesc: ^PxSceneDesc) -> ^PxScene ---

    /// Gets number of created scenes.
    ///
    /// The number of scenes created.
    PxPhysics_getNbScenes :: proc(self_: ^PxPhysics) -> _c.uint32_t ---

    /// Writes the array of scene pointers to a user buffer.
    ///
    /// Returns the number of pointers written.
    ///
    /// The ordering of the scene pointers in the array is not specified.
    ///
    /// The number of scene pointers written to userBuffer, this should be less or equal to bufferSize.
    PxPhysics_getScenes :: proc(self_: ^PxPhysics, userBuffer: ^^PxScene, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Creates a static rigid actor with the specified pose and all other fields initialized
    /// to their default values.
    PxPhysics_createRigidStatic_mut :: proc(self_: ^PxPhysics, pose: ^PxTransform) -> ^PxRigidStatic ---

    /// Creates a dynamic rigid actor with the specified pose and all other fields initialized
    /// to their default values.
    PxPhysics_createRigidDynamic_mut :: proc(self_: ^PxPhysics, pose: ^PxTransform) -> ^PxRigidDynamic ---

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
    PxPhysics_createPruningStructure_mut :: proc(self_: ^PxPhysics, actors: ^^PxRigidActor, nbActors: _c.uint32_t) -> ^PxPruningStructure ---

    /// Creates a shape which may be attached to multiple actors
    ///
    /// The shape will be created with a reference count of 1.
    ///
    /// The shape
    ///
    /// Shared shapes are not mutable when they are attached to an actor
    PxPhysics_createShape_mut :: proc(self_: ^PxPhysics, geometry: ^PxGeometry, material: ^PxMaterial, isExclusive: _c.bool, shapeFlags: _c.uint8_t) -> ^PxShape ---

    /// Creates a shape which may be attached to multiple actors
    ///
    /// The shape will be created with a reference count of 1.
    ///
    /// The shape
    ///
    /// Shared shapes are not mutable when they are attached to an actor
    ///
    /// Shapes created from *SDF* triangle-mesh geometries do not support more than one material.
    PxPhysics_createShape_mut_1 :: proc(self_: ^PxPhysics, geometry: ^PxGeometry, materials: ^^PxMaterial, materialCount: _c.uint16_t, isExclusive: _c.bool, shapeFlags: _c.uint8_t) -> ^PxShape ---

    /// Return the number of shapes that currently exist.
    ///
    /// Number of shapes.
    PxPhysics_getNbShapes :: proc(self_: ^PxPhysics) -> _c.uint32_t ---

    /// Writes the array of shape pointers to a user buffer.
    ///
    /// Returns the number of pointers written.
    ///
    /// The ordering of the shapes in the array is not specified.
    ///
    /// The number of shape pointers written to userBuffer, this should be less or equal to bufferSize.
    PxPhysics_getShapes :: proc(self_: ^PxPhysics, userBuffer: ^^PxShape, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Creates a constraint shader.
    ///
    /// A constraint shader will get added automatically to the scene the two linked actors belong to. Either, but not both, of actor0 and actor1 may
    /// be NULL to denote attachment to the world.
    ///
    /// The new constraint shader.
    PxPhysics_createConstraint_mut :: proc(self_: ^PxPhysics, actor0: ^PxRigidActor, actor1: ^PxRigidActor, connector: ^PxConstraintConnector, shaders: ^PxConstraintShaderTable, dataSize: _c.uint32_t) -> ^PxConstraint ---

    /// Creates a reduced-coordinate articulation with all fields initialized to their default values.
    ///
    /// the new articulation
    PxPhysics_createArticulationReducedCoordinate_mut :: proc(self_: ^PxPhysics) -> ^PxArticulationReducedCoordinate ---

    /// Creates a new rigid body material with certain default properties.
    ///
    /// The new rigid body material.
    PxPhysics_createMaterial_mut :: proc(self_: ^PxPhysics, staticFriction: _c.float, dynamicFriction: _c.float, restitution: _c.float) -> ^PxMaterial ---

    /// Return the number of rigid body materials that currently exist.
    ///
    /// Number of rigid body materials.
    PxPhysics_getNbMaterials :: proc(self_: ^PxPhysics) -> _c.uint32_t ---

    /// Writes the array of rigid body material pointers to a user buffer.
    ///
    /// Returns the number of pointers written.
    ///
    /// The ordering of the materials in the array is not specified.
    ///
    /// The number of material pointers written to userBuffer, this should be less or equal to bufferSize.
    PxPhysics_getMaterials :: proc(self_: ^PxPhysics, userBuffer: ^^PxMaterial, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Register a deletion listener. Listeners will be called whenever an object is deleted.
    ///
    /// It is illegal to register or unregister a deletion listener while deletions are being processed.
    ///
    /// By default a registered listener will receive events from all objects. Set the restrictedObjectSet parameter to true on registration and use [`registerDeletionListenerObjects`] to restrict the received events to specific objects.
    ///
    /// The deletion events are only supported on core PhysX objects. In general, objects in extension modules do not provide this functionality, however, in the case of PxJoint objects, the underlying PxConstraint will send the events.
    PxPhysics_registerDeletionListener_mut :: proc(self_: ^PxPhysics, observer: ^PxDeletionListener, deletionEvents: ^_c.uint8_t, restrictedObjectSet: _c.bool) ---

    /// Unregister a deletion listener.
    ///
    /// It is illegal to register or unregister a deletion listener while deletions are being processed.
    PxPhysics_unregisterDeletionListener_mut :: proc(self_: ^PxPhysics, observer: ^PxDeletionListener) ---

    /// Register specific objects for deletion events.
    ///
    /// This method allows for a deletion listener to limit deletion events to specific objects only.
    ///
    /// It is illegal to register or unregister objects while deletions are being processed.
    ///
    /// The deletion listener has to be registered through [`registerDeletionListener`]() and configured to support restricted object sets prior to this method being used.
    PxPhysics_registerDeletionListenerObjects_mut :: proc(self_: ^PxPhysics, observer: ^PxDeletionListener, observables: ^^PxBase, observableCount: _c.uint32_t) ---

    /// Unregister specific objects for deletion events.
    ///
    /// This method allows to clear previously registered objects for a deletion listener (see [`registerDeletionListenerObjects`]()).
    ///
    /// It is illegal to register or unregister objects while deletions are being processed.
    ///
    /// The deletion listener has to be registered through [`registerDeletionListener`]() and configured to support restricted object sets prior to this method being used.
    PxPhysics_unregisterDeletionListenerObjects_mut :: proc(self_: ^PxPhysics, observer: ^PxDeletionListener, observables: ^^PxBase, observableCount: _c.uint32_t) ---

    /// Gets PxPhysics object insertion interface.
    ///
    /// The insertion interface is needed for PxCreateTriangleMesh, PxCooking::createTriangleMesh etc., this allows runtime mesh creation.
    PxPhysics_getPhysicsInsertionCallback_mut :: proc(self_: ^PxPhysics) -> ^PxInsertionCallback ---

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
    phys_PxCreatePhysics :: proc(version: _c.uint32_t, foundation: ^PxFoundation, scale: ^PxTolerancesScale, trackOutstandingAllocations: _c.bool, pvd: ^PxPvd, omniPvd: ^PxOmniPvd) -> ^PxPhysics ---

    phys_PxGetPhysics :: proc() -> ^PxPhysics ---

    PxActorShape_new :: proc() -> PxActorShape ---

    PxActorShape_new_1 :: proc(a: ^PxRigidActor, s: ^PxShape) -> PxActorShape ---

    /// constructor sets to default
    PxQueryCache_new :: proc() -> PxQueryCache ---

    /// constructor to set properties
    PxQueryCache_new_1 :: proc(s: ^PxShape, findex: _c.uint32_t) -> PxQueryCache ---

    /// default constructor
    PxQueryFilterData_new :: proc() -> PxQueryFilterData ---

    /// constructor to set both filter data and filter flags
    PxQueryFilterData_new_1 :: proc(fd: ^PxFilterData, f: _c.uint16_t) -> PxQueryFilterData ---

    /// constructor to set filter flags only
    PxQueryFilterData_new_2 :: proc(f: _c.uint16_t) -> PxQueryFilterData ---

    /// This filter callback is executed before the exact intersection test if PxQueryFlag::ePREFILTER flag was set.
    ///
    /// the updated type for this hit  (see [`PxQueryHitType`])
    PxQueryFilterCallback_preFilter_mut :: proc(self_: ^PxQueryFilterCallback, filterData: ^PxFilterData, shape: ^PxShape, actor: ^PxRigidActor, queryFlags: ^_c.uint16_t) -> _c.int32_t ---

    /// This filter callback is executed if the exact intersection test returned true and PxQueryFlag::ePOSTFILTER flag was set.
    ///
    /// the updated hit type for this hit  (see [`PxQueryHitType`])
    PxQueryFilterCallback_postFilter_mut :: proc(self_: ^PxQueryFilterCallback, filterData: ^PxFilterData, hit: ^PxQueryHit, shape: ^PxShape, actor: ^PxRigidActor) -> _c.int32_t ---

    /// virtual destructor
    PxQueryFilterCallback_delete :: proc(self_: ^PxQueryFilterCallback) ---

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
    PxRigidDynamic_setKinematicTarget_mut :: proc(self_: ^PxRigidDynamic, destination: ^PxTransform) ---

    /// Get target pose of a kinematically controlled dynamic actor.
    ///
    /// True if the actor is a kinematically controlled dynamic and the target has been set, else False.
    PxRigidDynamic_getKinematicTarget :: proc(self_: ^PxRigidDynamic, target: ^PxTransform) -> _c.bool ---

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
    PxRigidDynamic_isSleeping :: proc(self_: ^PxRigidDynamic) -> _c.bool ---

    /// Sets the mass-normalized kinetic energy threshold below which an actor may go to sleep.
    ///
    /// Actors whose kinetic energy divided by their mass is below this threshold will be candidates for sleeping.
    ///
    /// Default:
    /// 5e-5f * PxTolerancesScale::speed * PxTolerancesScale::speed
    PxRigidDynamic_setSleepThreshold_mut :: proc(self_: ^PxRigidDynamic, threshold: _c.float) ---

    /// Returns the mass-normalized kinetic energy below which an actor may go to sleep.
    ///
    /// The energy threshold for sleeping.
    PxRigidDynamic_getSleepThreshold :: proc(self_: ^PxRigidDynamic) -> _c.float ---

    /// Sets the mass-normalized kinetic energy threshold below which an actor may participate in stabilization.
    ///
    /// Actors whose kinetic energy divided by their mass is above this threshold will not participate in stabilization.
    ///
    /// This value has no effect if PxSceneFlag::eENABLE_STABILIZATION was not enabled on the PxSceneDesc.
    ///
    /// Default:
    /// 1e-5f * PxTolerancesScale::speed * PxTolerancesScale::speed
    PxRigidDynamic_setStabilizationThreshold_mut :: proc(self_: ^PxRigidDynamic, threshold: _c.float) ---

    /// Returns the mass-normalized kinetic energy below which an actor may participate in stabilization.
    ///
    /// Actors whose kinetic energy divided by their mass is above this threshold will not participate in stabilization.
    ///
    /// The energy threshold for participating in stabilization.
    PxRigidDynamic_getStabilizationThreshold :: proc(self_: ^PxRigidDynamic) -> _c.float ---

    /// Reads the PxRigidDynamic lock flags.
    ///
    /// See the list of flags [`PxRigidDynamicLockFlag`]
    ///
    /// The values of the PxRigidDynamicLock flags.
    PxRigidDynamic_getRigidDynamicLockFlags :: proc(self_: ^PxRigidDynamic) -> _c.uint8_t ---

    /// Raises or clears a particular rigid dynamic lock flag.
    ///
    /// See the list of flags [`PxRigidDynamicLockFlag`]
    ///
    /// Default:
    /// no flags are set
    PxRigidDynamic_setRigidDynamicLockFlag_mut :: proc(self_: ^PxRigidDynamic, flag: _c.int32_t, value: _c.bool) ---

    PxRigidDynamic_setRigidDynamicLockFlags_mut :: proc(self_: ^PxRigidDynamic, flags: _c.uint8_t) ---

    /// Retrieves the linear velocity of an actor.
    ///
    /// It is not allowed to use this method while the simulation is running (except during PxScene::collide(),
    /// in PxContactModifyCallback or in contact report callbacks).
    ///
    /// The linear velocity of the actor.
    PxRigidDynamic_getLinearVelocity :: proc(self_: ^PxRigidDynamic) -> PxVec3 ---

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
    PxRigidDynamic_setLinearVelocity_mut :: proc(self_: ^PxRigidDynamic, linVel: ^PxVec3, autowake: _c.bool) ---

    /// Retrieves the angular velocity of the actor.
    ///
    /// It is not allowed to use this method while the simulation is running (except during PxScene::collide(),
    /// in PxContactModifyCallback or in contact report callbacks).
    ///
    /// The angular velocity of the actor.
    PxRigidDynamic_getAngularVelocity :: proc(self_: ^PxRigidDynamic) -> PxVec3 ---

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
    PxRigidDynamic_setAngularVelocity_mut :: proc(self_: ^PxRigidDynamic, angVel: ^PxVec3, autowake: _c.bool) ---

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
    PxRigidDynamic_setWakeCounter_mut :: proc(self_: ^PxRigidDynamic, wakeCounterValue: _c.float) ---

    /// Returns the wake counter of the actor.
    ///
    /// It is not allowed to use this method while the simulation is running.
    ///
    /// The wake counter of the actor.
    PxRigidDynamic_getWakeCounter :: proc(self_: ^PxRigidDynamic) -> _c.float ---

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
    PxRigidDynamic_wakeUp_mut :: proc(self_: ^PxRigidDynamic) ---

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
    PxRigidDynamic_putToSleep_mut :: proc(self_: ^PxRigidDynamic) ---

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
    PxRigidDynamic_setSolverIterationCounts_mut :: proc(self_: ^PxRigidDynamic, minPositionIters: _c.uint32_t, minVelocityIters: _c.uint32_t) ---

    /// Retrieves the solver iteration counts.
    PxRigidDynamic_getSolverIterationCounts :: proc(self_: ^PxRigidDynamic, minPositionIters: ^_c.uint32_t, minVelocityIters: ^_c.uint32_t) ---

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
    PxRigidDynamic_getContactReportThreshold :: proc(self_: ^PxRigidDynamic) -> _c.float ---

    /// Sets the force threshold for contact reports.
    ///
    /// See [`getContactReportThreshold`]().
    PxRigidDynamic_setContactReportThreshold_mut :: proc(self_: ^PxRigidDynamic, threshold: _c.float) ---

    PxRigidDynamic_getConcreteTypeName :: proc(self_: ^PxRigidDynamic) -> ^_c.char ---

    PxRigidStatic_getConcreteTypeName :: proc(self_: ^PxRigidStatic) -> ^_c.char ---

    /// constructor sets to default.
    PxSceneQueryDesc_new :: proc() -> PxSceneQueryDesc ---

    /// (re)sets the structure to the default.
    PxSceneQueryDesc_setToDefault_mut :: proc(self_: ^PxSceneQueryDesc) ---

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid.
    PxSceneQueryDesc_isValid :: proc(self_: ^PxSceneQueryDesc) -> _c.bool ---

    /// Sets the rebuild rate of the dynamic tree pruning structures.
    PxSceneQuerySystemBase_setDynamicTreeRebuildRateHint_mut :: proc(self_: ^PxSceneQuerySystemBase, dynamicTreeRebuildRateHint: _c.uint32_t) ---

    /// Retrieves the rebuild rate of the dynamic tree pruning structures.
    ///
    /// The rebuild rate of the dynamic tree pruning structures.
    PxSceneQuerySystemBase_getDynamicTreeRebuildRateHint :: proc(self_: ^PxSceneQuerySystemBase) -> _c.uint32_t ---

    /// Forces dynamic trees to be immediately rebuilt.
    ///
    /// PxScene will call this function with the PX_SCENE_PRUNER_STATIC or PX_SCENE_PRUNER_DYNAMIC value.
    PxSceneQuerySystemBase_forceRebuildDynamicTree_mut :: proc(self_: ^PxSceneQuerySystemBase, prunerIndex: _c.uint32_t) ---

    /// Sets scene query update mode
    PxSceneQuerySystemBase_setUpdateMode_mut :: proc(self_: ^PxSceneQuerySystemBase, updateMode: _c.int32_t) ---

    /// Gets scene query update mode
    ///
    /// Current scene query update mode.
    PxSceneQuerySystemBase_getUpdateMode :: proc(self_: ^PxSceneQuerySystemBase) -> _c.int32_t ---

    /// Retrieves the system's internal scene query timestamp, increased each time a change to the
    /// static scene query structure is performed.
    ///
    /// scene query static timestamp
    PxSceneQuerySystemBase_getStaticTimestamp :: proc(self_: ^PxSceneQuerySystemBase) -> _c.uint32_t ---

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
    PxSceneQuerySystemBase_flushUpdates_mut :: proc(self_: ^PxSceneQuerySystemBase) ---

    /// Performs a raycast against objects in the scene, returns results in a PxRaycastBuffer object
    /// or via a custom user callback implementation inheriting from PxRaycastCallback.
    ///
    /// Touching hits are not ordered.
    ///
    /// Shooting a ray from within an object leads to different results depending on the shape type. Please check the details in user guide article SceneQuery. User can ignore such objects by employing one of the provided filter mechanisms.
    ///
    /// True if any touching or blocking hits were found or any hit was found in case PxQueryFlag::eANY_HIT was specified.
    PxSceneQuerySystemBase_raycast :: proc(self_: ^PxSceneQuerySystemBase, origin: ^PxVec3, unitDir: ^PxVec3, distance: _c.float, hitCall: ^PxRaycastCallback, hitFlags: _c.uint16_t, filterData: ^PxQueryFilterData, filterCall: ^PxQueryFilterCallback, cache: ^PxQueryCache, queryFlags: _c.uint32_t) -> _c.bool ---

    /// Performs a sweep test against objects in the scene, returns results in a PxSweepBuffer object
    /// or via a custom user callback implementation inheriting from PxSweepCallback.
    ///
    /// Touching hits are not ordered.
    ///
    /// If a shape from the scene is already overlapping with the query shape in its starting position,
    /// the hit is returned unless eASSUME_NO_INITIAL_OVERLAP was specified.
    ///
    /// True if any touching or blocking hits were found or any hit was found in case PxQueryFlag::eANY_HIT was specified.
    PxSceneQuerySystemBase_sweep :: proc(self_: ^PxSceneQuerySystemBase, geometry: ^PxGeometry, pose: ^PxTransform, unitDir: ^PxVec3, distance: _c.float, hitCall: ^PxSweepCallback, hitFlags: _c.uint16_t, filterData: ^PxQueryFilterData, filterCall: ^PxQueryFilterCallback, cache: ^PxQueryCache, inflation: _c.float, queryFlags: _c.uint32_t) -> _c.bool ---

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
    PxSceneQuerySystemBase_overlap :: proc(self_: ^PxSceneQuerySystemBase, geometry: ^PxGeometry, pose: ^PxTransform, hitCall: ^PxOverlapCallback, filterData: ^PxQueryFilterData, filterCall: ^PxQueryFilterCallback, cache: ^PxQueryCache, queryFlags: _c.uint32_t) -> _c.bool ---

    /// Sets scene query update mode
    PxSceneSQSystem_setSceneQueryUpdateMode_mut :: proc(self_: ^PxSceneSQSystem, updateMode: _c.int32_t) ---

    /// Gets scene query update mode
    ///
    /// Current scene query update mode.
    PxSceneSQSystem_getSceneQueryUpdateMode :: proc(self_: ^PxSceneSQSystem) -> _c.int32_t ---

    /// Retrieves the scene's internal scene query timestamp, increased each time a change to the
    /// static scene query structure is performed.
    ///
    /// scene query static timestamp
    PxSceneSQSystem_getSceneQueryStaticTimestamp :: proc(self_: ^PxSceneSQSystem) -> _c.uint32_t ---

    /// Flushes any changes to the scene query representation.
    PxSceneSQSystem_flushQueryUpdates_mut :: proc(self_: ^PxSceneSQSystem) ---

    /// Forces dynamic trees to be immediately rebuilt.
    PxSceneSQSystem_forceDynamicTreeRebuild_mut :: proc(self_: ^PxSceneSQSystem, rebuildStaticStructure: _c.bool, rebuildDynamicStructure: _c.bool) ---

    /// Return the value of PxSceneQueryDesc::staticStructure that was set when creating the scene with PxPhysics::createScene
    PxSceneSQSystem_getStaticStructure :: proc(self_: ^PxSceneSQSystem) -> _c.int32_t ---

    /// Return the value of PxSceneQueryDesc::dynamicStructure that was set when creating the scene with PxPhysics::createScene
    PxSceneSQSystem_getDynamicStructure :: proc(self_: ^PxSceneSQSystem) -> _c.int32_t ---

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
    PxSceneSQSystem_sceneQueriesUpdate_mut :: proc(self_: ^PxSceneSQSystem, completionTask: ^PxBaseTask, controlSimulation: _c.bool) ---

    /// This checks to see if the scene queries update has completed.
    ///
    /// This does not cause the data available for reading to be updated with the results of the scene queries update, it is simply a status check.
    /// The bool will allow it to either return immediately or block waiting for the condition to be met so that it can return true
    ///
    /// True if the results are available.
    PxSceneSQSystem_checkQueries_mut :: proc(self_: ^PxSceneSQSystem, block: _c.bool) -> _c.bool ---

    /// This method must be called after sceneQueriesUpdate. It will wait for the scene queries update to finish. If the user makes an illegal scene queries update call,
    /// the SDK will issue an error message.
    ///
    /// If a new AABB tree build finished, then during fetchQueries the current tree within the pruning structure is swapped with the new tree.
    PxSceneSQSystem_fetchQueries_mut :: proc(self_: ^PxSceneSQSystem, block: _c.bool) -> _c.bool ---

    /// Decrements the reference count of the object and releases it if the new reference count is zero.
    PxSceneQuerySystem_release_mut :: proc(self_: ^PxSceneQuerySystem) ---

    /// Acquires a counted reference to this object.
    ///
    /// This method increases the reference count of the object by 1. Decrement the reference count by calling release()
    PxSceneQuerySystem_acquireReference_mut :: proc(self_: ^PxSceneQuerySystem) ---

    /// Preallocates internal arrays to minimize the amount of reallocations.
    ///
    /// The system does not prevent more allocations than given numbers. It is legal to not call this function at all,
    /// or to add more shapes to the system than the preallocated amounts.
    PxSceneQuerySystem_preallocate_mut :: proc(self_: ^PxSceneQuerySystem, prunerIndex: _c.uint32_t, nbShapes: _c.uint32_t) ---

    /// Frees internal memory that may not be in-use anymore.
    ///
    /// This is an entry point for reclaiming transient memory allocated at some point by the SQ system,
    /// but which wasn't been immediately freed for performance reason. Calling this function might free
    /// some memory, but it might also produce a new set of allocations in the next frame.
    PxSceneQuerySystem_flushMemory_mut :: proc(self_: ^PxSceneQuerySystem) ---

    /// Adds a shape to the SQ system.
    ///
    /// The same function is used to add either a regular shape, or a SQ compound shape.
    PxSceneQuerySystem_addSQShape_mut :: proc(self_: ^PxSceneQuerySystem, actor: ^PxRigidActor, shape: ^PxShape, bounds: ^PxBounds3, transform: ^PxTransform, compoundHandle: ^_c.uint32_t, hasPruningStructure: _c.bool) ---

    /// Removes a shape from the SQ system.
    ///
    /// The same function is used to remove either a regular shape, or a SQ compound shape.
    PxSceneQuerySystem_removeSQShape_mut :: proc(self_: ^PxSceneQuerySystem, actor: ^PxRigidActor, shape: ^PxShape) ---

    /// Updates a shape in the SQ system.
    ///
    /// The same function is used to update either a regular shape, or a SQ compound shape.
    ///
    /// The transforms are eager-evaluated, but the bounds are lazy-evaluated. This means that
    /// the updated transform has to be passed to the update function, while the bounds are automatically
    /// recomputed by the system whenever needed.
    PxSceneQuerySystem_updateSQShape_mut :: proc(self_: ^PxSceneQuerySystem, actor: ^PxRigidActor, shape: ^PxShape, transform: ^PxTransform) ---

    /// Adds a compound to the SQ system.
    ///
    /// SQ compound handle
    PxSceneQuerySystem_addSQCompound_mut :: proc(self_: ^PxSceneQuerySystem, actor: ^PxRigidActor, shapes: ^^PxShape, bvh: ^PxBVH, transforms: ^PxTransform) -> _c.uint32_t ---

    /// Removes a compound from the SQ system.
    PxSceneQuerySystem_removeSQCompound_mut :: proc(self_: ^PxSceneQuerySystem, compoundHandle: _c.uint32_t) ---

    /// Updates a compound in the SQ system.
    ///
    /// The compound structures are immediately updated when the call occurs.
    PxSceneQuerySystem_updateSQCompound_mut :: proc(self_: ^PxSceneQuerySystem, compoundHandle: _c.uint32_t, compoundTransform: ^PxTransform) ---

    /// Shift the data structures' origin by the specified vector.
    ///
    /// Please refer to the notes of the similar function in PxScene.
    PxSceneQuerySystem_shiftOrigin_mut :: proc(self_: ^PxSceneQuerySystem, shift: ^PxVec3) ---

    /// Merges a pruning structure with the SQ system's internal pruners.
    PxSceneQuerySystem_merge_mut :: proc(self_: ^PxSceneQuerySystem, pruningStructure: ^PxPruningStructure) ---

    /// Shape to SQ-pruner-handle mapping function.
    ///
    /// This function finds and returns the SQ pruner handle associated with a given (actor/shape) couple
    /// that was previously added to the system. This is needed for the sync function.
    ///
    /// Associated SQ pruner handle.
    PxSceneQuerySystem_getHandle :: proc(self_: ^PxSceneQuerySystem, actor: ^PxRigidActor, shape: ^PxShape, prunerIndex: ^_c.uint32_t) -> _c.uint32_t ---

    /// Synchronizes the scene-query system with another system that references the same objects.
    ///
    /// This function is used when the scene-query objects also exist in another system that can also update them. For example the scene-query objects
    /// (used for raycast, overlap or sweep queries) might be driven by equivalent objects in an external rigid-body simulation engine. In this case
    /// the rigid-body simulation engine computes the new poses and transforms, and passes them to the scene-query system using this function. It is
    /// more efficient than calling updateSQShape on each object individually, since updateSQShape would end up recomputing the bounds already available
    /// in the rigid-body engine.
    PxSceneQuerySystem_sync_mut :: proc(self_: ^PxSceneQuerySystem, prunerIndex: _c.uint32_t, handles: ^_c.uint32_t, indices: ^_c.uint32_t, bounds: ^PxBounds3, transforms: ^PxTransformPadded, count: _c.uint32_t, ignoredIndices: ^PxBitMap) ---

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
    PxSceneQuerySystem_finalizeUpdates_mut :: proc(self_: ^PxSceneQuerySystem) ---

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
    PxSceneQuerySystem_prepareSceneQueryBuildStep_mut :: proc(self_: ^PxSceneQuerySystem, prunerIndex: _c.uint32_t) -> rawptr ---

    /// Executes asynchronous build step.
    ///
    /// This is directly called (asynchronously) by PxSceneSQSystem::sceneQueriesUpdate(). See the comments there.
    ///
    /// This function incrementally builds the internal trees/pruners. It is called asynchronously, i.e. this can be
    /// called from different threads for building multiple trees at the same time.
    PxSceneQuerySystem_sceneQueryBuildStep_mut :: proc(self_: ^PxSceneQuerySystem, handle: rawptr) ---

    PxBroadPhaseDesc_new :: proc(type: _c.int32_t) -> PxBroadPhaseDesc ---

    PxBroadPhaseDesc_isValid :: proc(self_: ^PxBroadPhaseDesc) -> _c.bool ---

    /// Retrieves the filter group for static objects.
    ///
    /// Mark static objects with this group when adding them to the broadphase.
    /// Overlaps between static objects will not be detected. All static objects
    /// should have the same group.
    ///
    /// Filter group for static objects.
    phys_PxGetBroadPhaseStaticFilterGroup :: proc() -> _c.uint32_t ---

    /// Retrieves a filter group for dynamic objects.
    ///
    /// Mark dynamic objects with this group when adding them to the broadphase.
    /// Each dynamic object must have an ID, and overlaps between dynamic objects that have
    /// the same ID will not be detected. This is useful to dismiss overlaps between shapes
    /// of the same (compound) actor directly within the broadphase.
    ///
    /// Filter group for the object.
    phys_PxGetBroadPhaseDynamicFilterGroup :: proc(id: _c.uint32_t) -> _c.uint32_t ---

    /// Retrieves a filter group for kinematic objects.
    ///
    /// Mark kinematic objects with this group when adding them to the broadphase.
    /// Each kinematic object must have an ID, and overlaps between kinematic objects that have
    /// the same ID will not be detected.
    ///
    /// Filter group for the object.
    phys_PxGetBroadPhaseKinematicFilterGroup :: proc(id: _c.uint32_t) -> _c.uint32_t ---

    PxBroadPhaseUpdateData_new :: proc(created: ^_c.uint32_t, nbCreated: _c.uint32_t, updated: ^_c.uint32_t, nbUpdated: _c.uint32_t, removed: ^_c.uint32_t, nbRemoved: _c.uint32_t, bounds: ^PxBounds3, groups: ^_c.uint32_t, distances: ^_c.float, capacity: _c.uint32_t) -> PxBroadPhaseUpdateData ---

    PxBroadPhaseResults_new :: proc() -> PxBroadPhaseResults ---

    /// Returns number of regions currently registered in the broad-phase.
    ///
    /// Number of regions
    PxBroadPhaseRegions_getNbRegions :: proc(self_: ^PxBroadPhaseRegions) -> _c.uint32_t ---

    /// Gets broad-phase regions.
    ///
    /// Number of written out regions.
    PxBroadPhaseRegions_getRegions :: proc(self_: ^PxBroadPhaseRegions, userBuffer: ^PxBroadPhaseRegionInfo, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

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
    PxBroadPhaseRegions_addRegion_mut :: proc(self_: ^PxBroadPhaseRegions, region: ^PxBroadPhaseRegion, populateRegion: _c.bool, bounds: ^PxBounds3, distances: ^_c.float) -> _c.uint32_t ---

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
    PxBroadPhaseRegions_removeRegion_mut :: proc(self_: ^PxBroadPhaseRegions, handle: _c.uint32_t) -> _c.bool ---

    PxBroadPhaseRegions_getNbOutOfBoundsObjects :: proc(self_: ^PxBroadPhaseRegions) -> _c.uint32_t ---

    PxBroadPhaseRegions_getOutOfBoundsObjects :: proc(self_: ^PxBroadPhaseRegions) -> ^_c.uint32_t ---

    PxBroadPhase_release_mut :: proc(self_: ^PxBroadPhase) ---

    /// Gets the broadphase type.
    ///
    /// Broadphase type.
    PxBroadPhase_getType :: proc(self_: ^PxBroadPhase) -> _c.int32_t ---

    /// Gets broad-phase caps.
    PxBroadPhase_getCaps :: proc(self_: ^PxBroadPhase, caps: ^PxBroadPhaseCaps) ---

    /// Retrieves the regions API if applicable.
    ///
    /// For broadphases that do not use explicit user-defined regions, this call returns NULL.
    ///
    /// Region API, or NULL.
    PxBroadPhase_getRegions_mut :: proc(self_: ^PxBroadPhase) -> ^PxBroadPhaseRegions ---

    /// Retrieves the broadphase allocator.
    ///
    /// User-provided buffers should ideally be allocated with this allocator, for best performance.
    /// This is especially true for the GPU broadphases, whose buffers need to be allocated in CUDA
    /// host memory.
    ///
    /// The broadphase allocator.
    PxBroadPhase_getAllocator_mut :: proc(self_: ^PxBroadPhase) -> ^PxAllocatorCallback ---

    /// Retrieves the profiler's context ID.
    ///
    /// The context ID.
    PxBroadPhase_getContextID :: proc(self_: ^PxBroadPhase) -> _c.uint64_t ---

    /// Sets a scratch buffer
    ///
    /// Some broadphases might take advantage of a scratch buffer to limit runtime allocations.
    ///
    /// All broadphases still work without providing a scratch buffer, this is an optional function
    /// that can potentially reduce runtime allocations.
    PxBroadPhase_setScratchBlock_mut :: proc(self_: ^PxBroadPhase, scratchBlock: rawptr, size: _c.uint32_t) ---

    /// Updates the broadphase and computes the lists of created/deleted pairs.
    ///
    /// The provided update data describes changes to objects since the last broadphase update.
    ///
    /// To benefit from potentially multithreaded implementations, it is necessary to provide a continuation
    /// task to the function. It is legal to pass NULL there, but the underlying (CPU) implementations will
    /// then run single-threaded.
    PxBroadPhase_update_mut :: proc(self_: ^PxBroadPhase, updateData: ^PxBroadPhaseUpdateData, continuation: ^PxBaseTask) ---

    /// Retrieves the broadphase results after an update.
    ///
    /// This should be called once after each update call to retrieve the results of the broadphase. The
    /// results are incremental, i.e. the system only returns new and lost pairs, not all current pairs.
    PxBroadPhase_fetchResults_mut :: proc(self_: ^PxBroadPhase, results: ^PxBroadPhaseResults) ---

    /// Helper for single-threaded updates.
    ///
    /// This short helper function performs a single-theaded update and reports the results in a single call.
    PxBroadPhase_update_mut_1 :: proc(self_: ^PxBroadPhase, results: ^PxBroadPhaseResults, updateData: ^PxBroadPhaseUpdateData) ---

    /// Broadphase factory function.
    ///
    /// Use this function to create a new standalone broadphase.
    ///
    /// Newly created broadphase, or NULL
    phys_PxCreateBroadPhase :: proc(desc: ^PxBroadPhaseDesc) -> ^PxBroadPhase ---

    PxAABBManager_release_mut :: proc(self_: ^PxAABBManager) ---

    /// Retrieves the underlying broadphase.
    ///
    /// The managed broadphase.
    PxAABBManager_getBroadPhase_mut :: proc(self_: ^PxAABBManager) -> ^PxBroadPhase ---

    /// Retrieves the managed bounds.
    ///
    /// This is needed as input parameters to functions like PxBroadPhaseRegions::addRegion.
    ///
    /// The managed object bounds.
    PxAABBManager_getBounds :: proc(self_: ^PxAABBManager) -> ^PxBounds3 ---

    /// Retrieves the managed distances.
    ///
    /// This is needed as input parameters to functions like PxBroadPhaseRegions::addRegion.
    ///
    /// The managed object distances.
    PxAABBManager_getDistances :: proc(self_: ^PxAABBManager) -> ^_c.float ---

    /// Retrieves the managed filter groups.
    ///
    /// The managed object groups.
    PxAABBManager_getGroups :: proc(self_: ^PxAABBManager) -> ^_c.uint32_t ---

    /// Retrieves the managed buffers' capacity.
    ///
    /// Bounds, distances and groups buffers have the same capacity.
    ///
    /// The managed buffers' capacity.
    PxAABBManager_getCapacity :: proc(self_: ^PxAABBManager) -> _c.uint32_t ---

    /// Adds an object to the manager.
    ///
    /// Objects' indices are externally managed, i.e. they must be provided by users (as opposed to handles
    /// that could be returned by this manager). The design allows users to identify an object by a single ID,
    /// and use the same ID in multiple sub-systems.
    PxAABBManager_addObject_mut :: proc(self_: ^PxAABBManager, index: _c.uint32_t, bounds: ^PxBounds3, group: _c.uint32_t, distance: _c.float) ---

    /// Removes an object from the manager.
    PxAABBManager_removeObject_mut :: proc(self_: ^PxAABBManager, index: _c.uint32_t) ---

    /// Updates an object in the manager.
    ///
    /// This call can update an object's bounds, distance, or both.
    /// It is not possible to update an object's filter group.
    PxAABBManager_updateObject_mut :: proc(self_: ^PxAABBManager, index: _c.uint32_t, bounds: ^PxBounds3, distance: ^_c.float) ---

    /// Updates the broadphase and computes the lists of created/deleted pairs.
    ///
    /// The data necessary for updating the broadphase is internally computed by the AABB manager.
    ///
    /// To benefit from potentially multithreaded implementations, it is necessary to provide a continuation
    /// task to the function. It is legal to pass NULL there, but the underlying (CPU) implementations will
    /// then run single-threaded.
    PxAABBManager_update_mut :: proc(self_: ^PxAABBManager, continuation: ^PxBaseTask) ---

    /// Retrieves the broadphase results after an update.
    ///
    /// This should be called once after each update call to retrieve the results of the broadphase. The
    /// results are incremental, i.e. the system only returns new and lost pairs, not all current pairs.
    PxAABBManager_fetchResults_mut :: proc(self_: ^PxAABBManager, results: ^PxBroadPhaseResults) ---

    /// Helper for single-threaded updates.
    ///
    /// This short helper function performs a single-theaded update and reports the results in a single call.
    PxAABBManager_update_mut_1 :: proc(self_: ^PxAABBManager, results: ^PxBroadPhaseResults) ---

    /// AABB manager factory function.
    ///
    /// Use this function to create a new standalone high-level broadphase.
    ///
    /// Newly created AABB manager, or NULL
    phys_PxCreateAABBManager :: proc(broadphase: ^PxBroadPhase) -> ^PxAABBManager ---

    /// constructor sets to default
    PxSceneLimits_new :: proc() -> PxSceneLimits ---

    /// (re)sets the structure to the default
    PxSceneLimits_setToDefault_mut :: proc(self_: ^PxSceneLimits) ---

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid.
    PxSceneLimits_isValid :: proc(self_: ^PxSceneLimits) -> _c.bool ---

    PxgDynamicsMemoryConfig_new :: proc() -> PxgDynamicsMemoryConfig ---

    PxgDynamicsMemoryConfig_isValid :: proc(self_: ^PxgDynamicsMemoryConfig) -> _c.bool ---

    /// constructor sets to default.
    PxSceneDesc_new :: proc(scale: ^PxTolerancesScale) -> PxSceneDesc ---

    /// (re)sets the structure to the default.
    PxSceneDesc_setToDefault_mut :: proc(self_: ^PxSceneDesc, scale: ^PxTolerancesScale) ---

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid.
    PxSceneDesc_isValid :: proc(self_: ^PxSceneDesc) -> _c.bool ---

    PxSceneDesc_getTolerancesScale :: proc(self_: ^PxSceneDesc) -> ^PxTolerancesScale ---

    /// Get number of broadphase volumes added for the current simulation step.
    ///
    /// Number of broadphase volumes added.
    PxSimulationStatistics_getNbBroadPhaseAdds :: proc(self_: ^PxSimulationStatistics) -> _c.uint32_t ---

    /// Get number of broadphase volumes removed for the current simulation step.
    ///
    /// Number of broadphase volumes removed.
    PxSimulationStatistics_getNbBroadPhaseRemoves :: proc(self_: ^PxSimulationStatistics) -> _c.uint32_t ---

    /// Get number of shape collision pairs of a certain type processed for the current simulation step.
    ///
    /// There is an entry for each geometry pair type.
    ///
    /// entry[i][j] = entry[j][i], hence, if you want the sum of all pair
    /// types, you need to discard the symmetric entries
    ///
    /// Number of processed pairs of the specified geometry types.
    PxSimulationStatistics_getRbPairStats :: proc(self_: ^PxSimulationStatistics, pairType: _c.int32_t, g0: _c.int32_t, g1: _c.int32_t) -> _c.uint32_t ---

    PxSimulationStatistics_new :: proc() -> PxSimulationStatistics ---

    /// Sets the PVD flag. See PxPvdSceneFlag.
    PxPvdSceneClient_setScenePvdFlag_mut :: proc(self_: ^PxPvdSceneClient, flag: _c.int32_t, value: _c.bool) ---

    /// Sets the PVD flags. See PxPvdSceneFlags.
    PxPvdSceneClient_setScenePvdFlags_mut :: proc(self_: ^PxPvdSceneClient, flags: _c.uint8_t) ---

    /// Retrieves the PVD flags. See PxPvdSceneFlags.
    PxPvdSceneClient_getScenePvdFlags :: proc(self_: ^PxPvdSceneClient) -> _c.uint8_t ---

    /// update camera on PVD application's render window
    PxPvdSceneClient_updateCamera_mut :: proc(self_: ^PxPvdSceneClient, name: ^_c.char, origin: ^PxVec3, up: ^PxVec3, target: ^PxVec3) ---

    /// draw points on PVD application's render window
    PxPvdSceneClient_drawPoints_mut :: proc(self_: ^PxPvdSceneClient, points: ^PxDebugPoint, count: _c.uint32_t) ---

    /// draw lines on PVD application's render window
    PxPvdSceneClient_drawLines_mut :: proc(self_: ^PxPvdSceneClient, lines: ^PxDebugLine, count: _c.uint32_t) ---

    /// draw triangles on PVD application's render window
    PxPvdSceneClient_drawTriangles_mut :: proc(self_: ^PxPvdSceneClient, triangles: ^PxDebugTriangle, count: _c.uint32_t) ---

    /// draw text on PVD application's render window
    PxPvdSceneClient_drawText_mut :: proc(self_: ^PxPvdSceneClient, text: ^PxDebugText) ---

    PxDominanceGroupPair_new :: proc(a: _c.uint8_t, b: _c.uint8_t) -> PxDominanceGroupPair ---

    PxBroadPhaseCallback_delete :: proc(self_: ^PxBroadPhaseCallback) ---

    /// Out-of-bounds notification.
    ///
    /// This function is called when an object leaves the broad-phase.
    PxBroadPhaseCallback_onObjectOutOfBounds_mut :: proc(self_: ^PxBroadPhaseCallback, shape: ^PxShape, actor: ^PxActor) ---

    /// Out-of-bounds notification.
    ///
    /// This function is called when an aggregate leaves the broad-phase.
    PxBroadPhaseCallback_onObjectOutOfBounds_mut_1 :: proc(self_: ^PxBroadPhaseCallback, aggregate: ^PxAggregate) ---

    /// Deletes the scene.
    ///
    /// Removes any actors and constraint shaders from this scene
    /// (if the user hasn't already done so).
    ///
    /// Be sure to not keep a reference to this object after calling release.
    /// Avoid release calls while the scene is simulating (in between simulate() and fetchResults() calls).
    PxScene_release_mut :: proc(self_: ^PxScene) ---

    /// Sets a scene flag. You can only set one flag at a time.
    ///
    /// Not all flags are mutable and changing some will result in an error. Please check [`PxSceneFlag`] to see which flags can be changed.
    PxScene_setFlag_mut :: proc(self_: ^PxScene, flag: _c.int32_t, value: _c.bool) ---

    /// Get the scene flags.
    ///
    /// The scene flags. See [`PxSceneFlag`]
    PxScene_getFlags :: proc(self_: ^PxScene) -> _c.uint32_t ---

    /// Set new scene limits.
    ///
    /// Increase the maximum capacity of various data structures in the scene. The new capacities will be
    /// at least as large as required to deal with the objects currently in the scene. Further, these values
    /// are for preallocation and do not represent hard limits.
    PxScene_setLimits_mut :: proc(self_: ^PxScene, limits: ^PxSceneLimits) ---

    /// Get current scene limits.
    ///
    /// Current scene limits.
    PxScene_getLimits :: proc(self_: ^PxScene) -> PxSceneLimits ---

    /// Call this method to retrieve the Physics SDK.
    ///
    /// The physics SDK this scene is associated with.
    PxScene_getPhysics_mut :: proc(self_: ^PxScene) -> ^PxPhysics ---

    /// Retrieves the scene's internal timestamp, increased each time a simulation step is completed.
    ///
    /// scene timestamp
    PxScene_getTimestamp :: proc(self_: ^PxScene) -> _c.uint32_t ---

    /// Adds an articulation to this scene.
    ///
    /// If the articulation is already assigned to a scene (see [`PxArticulationReducedCoordinate::getScene`]), the call is ignored and an error is issued.
    ///
    /// True if success
    PxScene_addArticulation_mut :: proc(self_: ^PxScene, articulation: ^PxArticulationReducedCoordinate) -> _c.bool ---

    /// Removes an articulation from this scene.
    ///
    /// If the articulation is not part of this scene (see [`PxArticulationReducedCoordinate::getScene`]), the call is ignored and an error is issued.
    ///
    /// If the articulation is in an aggregate it will be removed from the aggregate.
    PxScene_removeArticulation_mut :: proc(self_: ^PxScene, articulation: ^PxArticulationReducedCoordinate, wakeOnLostTouch: _c.bool) ---

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
    PxScene_addActor_mut :: proc(self_: ^PxScene, actor: ^PxActor, bvh: ^PxBVH) -> _c.bool ---

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
    PxScene_addActors_mut :: proc(self_: ^PxScene, actors: ^^PxActor, nbActors: _c.uint32_t) -> _c.bool ---

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
    PxScene_addActors_mut_1 :: proc(self_: ^PxScene, pruningStructure: ^PxPruningStructure) -> _c.bool ---

    /// Removes an actor from this scene.
    ///
    /// If the actor is not part of this scene (see [`PxActor::getScene`]), the call is ignored and an error is issued.
    ///
    /// You can not remove individual articulation links (see [`PxArticulationLink`]) from the scene. Use #removeArticulation() instead.
    ///
    /// If the actor is a PxRigidActor then all assigned PxConstraint objects will get removed from the scene automatically.
    ///
    /// If the actor is in an aggregate it will be removed from the aggregate.
    PxScene_removeActor_mut :: proc(self_: ^PxScene, actor: ^PxActor, wakeOnLostTouch: _c.bool) ---

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
    PxScene_removeActors_mut :: proc(self_: ^PxScene, actors: ^^PxActor, nbActors: _c.uint32_t, wakeOnLostTouch: _c.bool) ---

    /// Adds an aggregate to this scene.
    ///
    /// If the aggregate is already assigned to a scene (see [`PxAggregate::getScene`]), the call is ignored and an error is issued.
    ///
    /// If the aggregate contains an actor with an invalid constraint, in checked builds the call is ignored and an error is issued.
    ///
    /// If the aggregate already contains actors, those actors are added to the scene as well.
    ///
    /// True if success
    PxScene_addAggregate_mut :: proc(self_: ^PxScene, aggregate: ^PxAggregate) -> _c.bool ---

    /// Removes an aggregate from this scene.
    ///
    /// If the aggregate is not part of this scene (see [`PxAggregate::getScene`]), the call is ignored and an error is issued.
    ///
    /// If the aggregate contains actors, those actors are removed from the scene as well.
    PxScene_removeAggregate_mut :: proc(self_: ^PxScene, aggregate: ^PxAggregate, wakeOnLostTouch: _c.bool) ---

    /// Adds objects in the collection to this scene.
    ///
    /// This function adds the following types of objects to this scene: PxRigidActor (except PxArticulationLink), PxAggregate, PxArticulationReducedCoordinate.
    /// This method is typically used after deserializing the collection in order to populate the scene with deserialized objects.
    ///
    /// If the collection contains an actor with an invalid constraint, in checked builds the call is ignored and an error is issued.
    ///
    /// True if success
    PxScene_addCollection_mut :: proc(self_: ^PxScene, collection: ^PxCollection) -> _c.bool ---

    /// Retrieve the number of actors of certain types in the scene. For supported types, see PxActorTypeFlags.
    ///
    /// the number of actors.
    PxScene_getNbActors :: proc(self_: ^PxScene, types: _c.uint16_t) -> _c.uint32_t ---

    /// Retrieve an array of all the actors of certain types in the scene. For supported types, see PxActorTypeFlags.
    ///
    /// Number of actors written to the buffer.
    PxScene_getActors :: proc(self_: ^PxScene, types: _c.uint16_t, userBuffer: ^^PxActor, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Queries the PxScene for a list of the PxActors whose transforms have been
    /// updated during the previous simulation step. Only includes actors of type PxRigidDynamic and PxArticulationLink.
    ///
    /// PxSceneFlag::eENABLE_ACTIVE_ACTORS must be set.
    ///
    /// Do not use this method while the simulation is running. Calls to this method while the simulation is running will be ignored and NULL will be returned.
    ///
    /// A pointer to the list of active PxActors generated during the last call to fetchResults().
    PxScene_getActiveActors_mut :: proc(self_: ^PxScene, nbActorsOut: ^_c.uint32_t) -> ^^PxActor ---

    /// Returns the number of articulations in the scene.
    ///
    /// the number of articulations in this scene.
    PxScene_getNbArticulations :: proc(self_: ^PxScene) -> _c.uint32_t ---

    /// Retrieve all the articulations in the scene.
    ///
    /// Number of articulations written to the buffer.
    PxScene_getArticulations :: proc(self_: ^PxScene, userBuffer: ^^PxArticulationReducedCoordinate, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Returns the number of constraint shaders in the scene.
    ///
    /// the number of constraint shaders in this scene.
    PxScene_getNbConstraints :: proc(self_: ^PxScene) -> _c.uint32_t ---

    /// Retrieve all the constraint shaders in the scene.
    ///
    /// Number of constraint shaders written to the buffer.
    PxScene_getConstraints :: proc(self_: ^PxScene, userBuffer: ^^PxConstraint, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Returns the number of aggregates in the scene.
    ///
    /// the number of aggregates in this scene.
    PxScene_getNbAggregates :: proc(self_: ^PxScene) -> _c.uint32_t ---

    /// Retrieve all the aggregates in the scene.
    ///
    /// Number of aggregates written to the buffer.
    PxScene_getAggregates :: proc(self_: ^PxScene, userBuffer: ^^PxAggregate, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

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
    PxScene_setDominanceGroupPair_mut :: proc(self_: ^PxScene, group1: _c.uint8_t, group2: _c.uint8_t, dominance: ^PxDominanceGroupPair) ---

    /// Samples the dominance matrix.
    PxScene_getDominanceGroupPair :: proc(self_: ^PxScene, group1: _c.uint8_t, group2: _c.uint8_t) -> PxDominanceGroupPair ---

    /// Return the cpu dispatcher that was set in PxSceneDesc::cpuDispatcher when creating the scene with PxPhysics::createScene
    PxScene_getCpuDispatcher :: proc(self_: ^PxScene) -> ^PxCpuDispatcher ---

    /// Reserves a new client ID.
    ///
    /// PX_DEFAULT_CLIENT is always available as the default clientID.
    /// Additional clients are returned by this function. Clients cannot be released once created.
    /// An error is reported when more than a supported number of clients (currently 128) are created.
    PxScene_createClient_mut :: proc(self_: ^PxScene) -> _c.uint8_t ---

    /// Sets a user notify object which receives special simulation events when they occur.
    ///
    /// Do not set the callback while the simulation is running. Calls to this method while the simulation is running will be ignored.
    PxScene_setSimulationEventCallback_mut :: proc(self_: ^PxScene, callback: ^PxSimulationEventCallback) ---

    /// Retrieves the simulationEventCallback pointer set with setSimulationEventCallback().
    ///
    /// The current user notify pointer. See [`PxSimulationEventCallback`].
    PxScene_getSimulationEventCallback :: proc(self_: ^PxScene) -> ^PxSimulationEventCallback ---

    /// Sets a user callback object, which receives callbacks on all contacts generated for specified actors.
    ///
    /// Do not set the callback while the simulation is running. Calls to this method while the simulation is running will be ignored.
    PxScene_setContactModifyCallback_mut :: proc(self_: ^PxScene, callback: ^PxContactModifyCallback) ---

    /// Sets a user callback object, which receives callbacks on all CCD contacts generated for specified actors.
    ///
    /// Do not set the callback while the simulation is running. Calls to this method while the simulation is running will be ignored.
    PxScene_setCCDContactModifyCallback_mut :: proc(self_: ^PxScene, callback: ^PxCCDContactModifyCallback) ---

    /// Retrieves the PxContactModifyCallback pointer set with setContactModifyCallback().
    ///
    /// The current user contact modify callback pointer. See [`PxContactModifyCallback`].
    PxScene_getContactModifyCallback :: proc(self_: ^PxScene) -> ^PxContactModifyCallback ---

    /// Retrieves the PxCCDContactModifyCallback pointer set with setContactModifyCallback().
    ///
    /// The current user contact modify callback pointer. See [`PxContactModifyCallback`].
    PxScene_getCCDContactModifyCallback :: proc(self_: ^PxScene) -> ^PxCCDContactModifyCallback ---

    /// Sets a broad-phase user callback object.
    ///
    /// Do not set the callback while the simulation is running. Calls to this method while the simulation is running will be ignored.
    PxScene_setBroadPhaseCallback_mut :: proc(self_: ^PxScene, callback: ^PxBroadPhaseCallback) ---

    /// Retrieves the PxBroadPhaseCallback pointer set with setBroadPhaseCallback().
    ///
    /// The current broad-phase callback pointer. See [`PxBroadPhaseCallback`].
    PxScene_getBroadPhaseCallback :: proc(self_: ^PxScene) -> ^PxBroadPhaseCallback ---

    /// Sets the shared global filter data which will get passed into the filter shader.
    ///
    /// It is the user's responsibility to ensure that changing the shared global filter data does not change the filter output value for existing pairs.
    /// If the filter output for existing pairs does change nonetheless then such a change will not take effect until the pair gets refiltered.
    /// resetFiltering() can be used to explicitly refilter the pairs of specific objects.
    ///
    /// The provided data will get copied to internal buffers and this copy will be used for filtering calls.
    ///
    /// Do not use this method while the simulation is running. Calls to this method while the simulation is running will be ignored.
    PxScene_setFilterShaderData_mut :: proc(self_: ^PxScene, data: rawptr, dataSize: _c.uint32_t) ---

    /// Gets the shared global filter data in use for this scene.
    ///
    /// The reference points to a copy of the original filter data specified in [`PxSceneDesc`].filterShaderData or provided by #setFilterShaderData().
    ///
    /// Shared filter data for filter shader.
    PxScene_getFilterShaderData :: proc(self_: ^PxScene) -> rawptr ---

    /// Gets the size of the shared global filter data ([`PxSceneDesc`].filterShaderData)
    ///
    /// Size of shared filter data [bytes].
    PxScene_getFilterShaderDataSize :: proc(self_: ^PxScene) -> _c.uint32_t ---

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
    PxScene_resetFiltering_mut :: proc(self_: ^PxScene, actor: ^PxActor) -> _c.bool ---

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
    PxScene_resetFiltering_mut_1 :: proc(self_: ^PxScene, actor: ^PxRigidActor, shapes: ^^PxShape, shapeCount: _c.uint32_t) -> _c.bool ---

    /// Gets the pair filtering mode for kinematic-kinematic pairs.
    ///
    /// Filtering mode for kinematic-kinematic pairs.
    PxScene_getKinematicKinematicFilteringMode :: proc(self_: ^PxScene) -> _c.int32_t ---

    /// Gets the pair filtering mode for static-kinematic pairs.
    ///
    /// Filtering mode for static-kinematic pairs.
    PxScene_getStaticKinematicFilteringMode :: proc(self_: ^PxScene) -> _c.int32_t ---

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
    PxScene_simulate_mut :: proc(self_: ^PxScene, elapsedTime: _c.float, completionTask: ^PxBaseTask, scratchMemBlock: rawptr, scratchMemBlockSize: _c.uint32_t, controlSimulation: _c.bool) -> _c.bool ---

    /// Performs dynamics phase of the simulation pipeline.
    ///
    /// Calls to advance() should follow calls to fetchCollision(). An error message will be issued if this sequence is not followed.
    ///
    /// True if success
    PxScene_advance_mut :: proc(self_: ^PxScene, completionTask: ^PxBaseTask) -> _c.bool ---

    /// Performs collision detection for the scene over elapsedTime
    ///
    /// Calls to collide() should be the first method called to simulate a frame.
    ///
    /// True if success
    PxScene_collide_mut :: proc(self_: ^PxScene, elapsedTime: _c.float, completionTask: ^PxBaseTask, scratchMemBlock: rawptr, scratchMemBlockSize: _c.uint32_t, controlSimulation: _c.bool) -> _c.bool ---

    /// This checks to see if the simulation run has completed.
    ///
    /// This does not cause the data available for reading to be updated with the results of the simulation, it is simply a status check.
    /// The bool will allow it to either return immediately or block waiting for the condition to be met so that it can return true
    ///
    /// True if the results are available.
    PxScene_checkResults_mut :: proc(self_: ^PxScene, block: _c.bool) -> _c.bool ---

    /// This method must be called after collide() and before advance(). It will wait for the collision phase to finish. If the user makes an illegal simulation call, the SDK will issue an error
    /// message.
    PxScene_fetchCollision_mut :: proc(self_: ^PxScene, block: _c.bool) -> _c.bool ---

    /// This is the big brother to checkResults() it basically does the following:
    ///
    /// True if the results have been fetched.
    PxScene_fetchResults_mut :: proc(self_: ^PxScene, block: _c.bool, errorState: ^_c.uint32_t) -> _c.bool ---

    /// This call performs the first section of fetchResults, and returns a pointer to the contact streams output by the simulation. It can be used to process contact pairs in parallel, which is often a limiting factor
    /// for fetchResults() performance.
    ///
    /// After calling this function and processing the contact streams, call fetchResultsFinish(). Note that writes to the simulation are not
    /// permitted between the start of fetchResultsStart() and the end of fetchResultsFinish().
    ///
    /// True if the results have been fetched.
    PxScene_fetchResultsStart_mut :: proc(self_: ^PxScene, contactPairs: ^^PxContactPairHeader, nbContactPairs: ^_c.uint32_t, block: _c.bool) -> _c.bool ---

    /// This call processes all event callbacks in parallel. It takes a continuation task, which will be executed once all callbacks have been processed.
    ///
    /// This is a utility function to make it easier to process callbacks in parallel using the PhysX task system. It can only be used in conjunction with
    /// fetchResultsStart(...) and fetchResultsFinish(...)
    PxScene_processCallbacks_mut :: proc(self_: ^PxScene, continuation: ^PxBaseTask) ---

    /// This call performs the second section of fetchResults.
    ///
    /// It must be called after fetchResultsStart() returns and contact reports have been processed.
    ///
    /// Note that once fetchResultsFinish() has been called, the contact streams returned in fetchResultsStart() will be invalid.
    PxScene_fetchResultsFinish_mut :: proc(self_: ^PxScene, errorState: ^_c.uint32_t) ---

    /// This call performs the synchronization of particle system data copies.
    PxScene_fetchResultsParticleSystem_mut :: proc(self_: ^PxScene) ---

    /// Clear internal buffers and free memory.
    ///
    /// This method can be used to clear buffers and free internal memory without having to destroy the scene. Can be useful if
    /// the physics data gets streamed in and a checkpoint with a clean state should be created.
    ///
    /// It is not allowed to call this method while the simulation is running. The call will fail.
    PxScene_flushSimulation_mut :: proc(self_: ^PxScene, sendPendingReports: _c.bool) ---

    /// Sets a constant gravity for the entire scene.
    ///
    /// Do not use this method while the simulation is running.
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the actor up automatically.
    PxScene_setGravity_mut :: proc(self_: ^PxScene, vec: ^PxVec3) ---

    /// Retrieves the current gravity setting.
    ///
    /// The current gravity for the scene.
    PxScene_getGravity :: proc(self_: ^PxScene) -> PxVec3 ---

    /// Set the bounce threshold velocity.  Collision speeds below this threshold will not cause a bounce.
    ///
    /// Do not use this method while the simulation is running.
    PxScene_setBounceThresholdVelocity_mut :: proc(self_: ^PxScene, t: _c.float) ---

    /// Return the bounce threshold velocity.
    PxScene_getBounceThresholdVelocity :: proc(self_: ^PxScene) -> _c.float ---

    /// Sets the maximum number of CCD passes
    ///
    /// Do not use this method while the simulation is running.
    PxScene_setCCDMaxPasses_mut :: proc(self_: ^PxScene, ccdMaxPasses: _c.uint32_t) ---

    /// Gets the maximum number of CCD passes.
    ///
    /// The maximum number of CCD passes.
    PxScene_getCCDMaxPasses :: proc(self_: ^PxScene) -> _c.uint32_t ---

    /// Set the maximum CCD separation.
    ///
    /// Do not use this method while the simulation is running.
    PxScene_setCCDMaxSeparation_mut :: proc(self_: ^PxScene, t: _c.float) ---

    /// Gets the maximum CCD separation.
    ///
    /// The maximum CCD separation.
    PxScene_getCCDMaxSeparation :: proc(self_: ^PxScene) -> _c.float ---

    /// Set the CCD threshold.
    ///
    /// Do not use this method while the simulation is running.
    PxScene_setCCDThreshold_mut :: proc(self_: ^PxScene, t: _c.float) ---

    /// Gets the CCD threshold.
    ///
    /// The CCD threshold.
    PxScene_getCCDThreshold :: proc(self_: ^PxScene) -> _c.float ---

    /// Set the max bias coefficient.
    ///
    /// Do not use this method while the simulation is running.
    PxScene_setMaxBiasCoefficient_mut :: proc(self_: ^PxScene, t: _c.float) ---

    /// Gets the max bias coefficient.
    ///
    /// The max bias coefficient.
    PxScene_getMaxBiasCoefficient :: proc(self_: ^PxScene) -> _c.float ---

    /// Set the friction offset threshold.
    ///
    /// Do not use this method while the simulation is running.
    PxScene_setFrictionOffsetThreshold_mut :: proc(self_: ^PxScene, t: _c.float) ---

    /// Gets the friction offset threshold.
    PxScene_getFrictionOffsetThreshold :: proc(self_: ^PxScene) -> _c.float ---

    /// Set the friction correlation distance.
    ///
    /// Do not use this method while the simulation is running.
    PxScene_setFrictionCorrelationDistance_mut :: proc(self_: ^PxScene, t: _c.float) ---

    /// Gets the friction correlation distance.
    PxScene_getFrictionCorrelationDistance :: proc(self_: ^PxScene) -> _c.float ---

    /// Return the friction model.
    PxScene_getFrictionType :: proc(self_: ^PxScene) -> _c.int32_t ---

    /// Return the solver model.
    PxScene_getSolverType :: proc(self_: ^PxScene) -> _c.int32_t ---

    /// Function that lets you set debug visualization parameters.
    ///
    /// Returns false if the value passed is out of range for usage specified by the enum.
    ///
    /// Do not use this method while the simulation is running.
    ///
    /// False if the parameter is out of range.
    PxScene_setVisualizationParameter_mut :: proc(self_: ^PxScene, param: _c.int32_t, value: _c.float) -> _c.bool ---

    /// Function that lets you query debug visualization parameters.
    ///
    /// The value of the parameter.
    PxScene_getVisualizationParameter :: proc(self_: ^PxScene, paramEnum: _c.int32_t) -> _c.float ---

    /// Defines a box in world space to which visualization geometry will be (conservatively) culled. Use a non-empty culling box to enable the feature, and an empty culling box to disable it.
    ///
    /// Do not use this method while the simulation is running.
    PxScene_setVisualizationCullingBox_mut :: proc(self_: ^PxScene, box: ^PxBounds3) ---

    /// Retrieves the visualization culling box.
    ///
    /// the box to which the geometry will be culled.
    PxScene_getVisualizationCullingBox :: proc(self_: ^PxScene) -> PxBounds3 ---

    /// Retrieves the render buffer.
    ///
    /// This will contain the results of any active visualization for this scene.
    ///
    /// Do not use this method while the simulation is running. Calls to this method while the simulation is running will result in undefined behaviour.
    ///
    /// The render buffer.
    PxScene_getRenderBuffer_mut :: proc(self_: ^PxScene) -> ^PxRenderBuffer ---

    /// Call this method to retrieve statistics for the current simulation step.
    ///
    /// Do not use this method while the simulation is running. Calls to this method while the simulation is running will be ignored.
    PxScene_getSimulationStatistics :: proc(self_: ^PxScene, stats: ^PxSimulationStatistics) ---

    /// Returns broad-phase type.
    ///
    /// Broad-phase type
    PxScene_getBroadPhaseType :: proc(self_: ^PxScene) -> _c.int32_t ---

    /// Gets broad-phase caps.
    ///
    /// True if success
    PxScene_getBroadPhaseCaps :: proc(self_: ^PxScene, caps: ^PxBroadPhaseCaps) -> _c.bool ---

    /// Returns number of regions currently registered in the broad-phase.
    ///
    /// Number of regions
    PxScene_getNbBroadPhaseRegions :: proc(self_: ^PxScene) -> _c.uint32_t ---

    /// Gets broad-phase regions.
    ///
    /// Number of written out regions
    PxScene_getBroadPhaseRegions :: proc(self_: ^PxScene, userBuffer: ^PxBroadPhaseRegionInfo, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

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
    PxScene_addBroadPhaseRegion_mut :: proc(self_: ^PxScene, region: ^PxBroadPhaseRegion, populateRegion: _c.bool) -> _c.uint32_t ---

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
    PxScene_removeBroadPhaseRegion_mut :: proc(self_: ^PxScene, handle: _c.uint32_t) -> _c.bool ---

    /// Get the task manager associated with this scene
    ///
    /// the task manager associated with the scene
    PxScene_getTaskManager :: proc(self_: ^PxScene) -> ^PxTaskManager ---

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
    PxScene_lockRead_mut :: proc(self_: ^PxScene, file: ^_c.char, line: _c.uint32_t) ---

    /// Unlock the scene from reading.
    ///
    /// Each unlockRead() must be paired with a lockRead() from the same thread.
    PxScene_unlockRead_mut :: proc(self_: ^PxScene) ---

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
    PxScene_lockWrite_mut :: proc(self_: ^PxScene, file: ^_c.char, line: _c.uint32_t) ---

    /// Unlock the scene from writing.
    ///
    /// Each unlockWrite() must be paired with a lockWrite() from the same thread.
    PxScene_unlockWrite_mut :: proc(self_: ^PxScene) ---

    /// set the cache blocks that can be used during simulate().
    ///
    /// Each frame the simulation requires memory to store contact, friction, and contact cache data. This memory is used in blocks of 16K.
    /// Each frame the blocks used by the previous frame are freed, and may be retrieved by the application using PxScene::flushSimulation()
    ///
    /// This call will force allocation of cache blocks if the numBlocks parameter is greater than the currently allocated number
    /// of blocks, and less than the max16KContactDataBlocks parameter specified at scene creation time.
    ///
    /// Do not use this method while the simulation is running.
    PxScene_setNbContactDataBlocks_mut :: proc(self_: ^PxScene, numBlocks: _c.uint32_t) ---

    /// get the number of cache blocks currently used by the scene
    ///
    /// This function may not be called while the scene is simulating
    ///
    /// the number of cache blocks currently used by the scene
    PxScene_getNbContactDataBlocksUsed :: proc(self_: ^PxScene) -> _c.uint32_t ---

    /// get the maximum number of cache blocks used by the scene
    ///
    /// This function may not be called while the scene is simulating
    ///
    /// the maximum number of cache blocks everused by the scene
    PxScene_getMaxNbContactDataBlocksUsed :: proc(self_: ^PxScene) -> _c.uint32_t ---

    /// Return the value of PxSceneDesc::contactReportStreamBufferSize that was set when creating the scene with PxPhysics::createScene
    PxScene_getContactReportStreamBufferSize :: proc(self_: ^PxScene) -> _c.uint32_t ---

    /// Sets the number of actors required to spawn a separate rigid body solver thread.
    ///
    /// Do not use this method while the simulation is running.
    PxScene_setSolverBatchSize_mut :: proc(self_: ^PxScene, solverBatchSize: _c.uint32_t) ---

    /// Retrieves the number of actors required to spawn a separate rigid body solver thread.
    ///
    /// Current number of actors required to spawn a separate rigid body solver thread.
    PxScene_getSolverBatchSize :: proc(self_: ^PxScene) -> _c.uint32_t ---

    /// Sets the number of articulations required to spawn a separate rigid body solver thread.
    ///
    /// Do not use this method while the simulation is running.
    PxScene_setSolverArticulationBatchSize_mut :: proc(self_: ^PxScene, solverBatchSize: _c.uint32_t) ---

    /// Retrieves the number of articulations required to spawn a separate rigid body solver thread.
    ///
    /// Current number of articulations required to spawn a separate rigid body solver thread.
    PxScene_getSolverArticulationBatchSize :: proc(self_: ^PxScene) -> _c.uint32_t ---

    /// Returns the wake counter reset value.
    ///
    /// Wake counter reset value
    PxScene_getWakeCounterResetValue :: proc(self_: ^PxScene) -> _c.float ---

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
    PxScene_shiftOrigin_mut :: proc(self_: ^PxScene, shift: ^PxVec3) ---

    /// Returns the Pvd client associated with the scene.
    ///
    /// the client, NULL if no PVD supported.
    PxScene_getScenePvdClient_mut :: proc(self_: ^PxScene) -> ^PxPvdSceneClient ---

    /// Copy GPU articulation data from the internal GPU buffer to a user-provided device buffer.
    PxScene_copyArticulationData_mut :: proc(self_: ^PxScene, data: rawptr, index: rawptr, dataType: _c.int32_t, nbCopyArticulations: _c.uint32_t, copyEvent: rawptr) ---

    /// Apply GPU articulation data from a user-provided device buffer to the internal GPU buffer.
    PxScene_applyArticulationData_mut :: proc(self_: ^PxScene, data: rawptr, index: rawptr, dataType: _c.int32_t, nbUpdatedArticulations: _c.uint32_t, waitEvent: rawptr, signalEvent: rawptr) ---

    /// Copy GPU softbody data from the internal GPU buffer to a user-provided device buffer.
    PxScene_copySoftBodyData_mut :: proc(self_: ^PxScene, data: ^rawptr, dataSizes: rawptr, softBodyIndices: rawptr, flag: _c.int32_t, nbCopySoftBodies: _c.uint32_t, maxSize: _c.uint32_t, copyEvent: rawptr) ---

    /// Apply user-provided data to the internal softbody system.
    PxScene_applySoftBodyData_mut :: proc(self_: ^PxScene, data: ^rawptr, dataSizes: rawptr, softBodyIndices: rawptr, flag: _c.int32_t, nbUpdatedSoftBodies: _c.uint32_t, maxSize: _c.uint32_t, applyEvent: rawptr) ---

    /// Copy contact data from the internal GPU buffer to a user-provided device buffer.
    ///
    /// The contact data contains pointers to internal state and is only valid until the next call to simulate().
    PxScene_copyContactData_mut :: proc(self_: ^PxScene, data: rawptr, maxContactPairs: _c.uint32_t, numContactPairs: rawptr, copyEvent: rawptr) ---

    /// Copy GPU rigid body data from the internal GPU buffer to a user-provided device buffer.
    PxScene_copyBodyData_mut :: proc(self_: ^PxScene, data: ^PxGpuBodyData, index: ^PxGpuActorPair, nbCopyActors: _c.uint32_t, copyEvent: rawptr) ---

    /// Apply user-provided data to rigid body.
    PxScene_applyActorData_mut :: proc(self_: ^PxScene, data: rawptr, index: ^PxGpuActorPair, flag: _c.int32_t, nbUpdatedActors: _c.uint32_t, waitEvent: rawptr, signalEvent: rawptr) ---

    /// Compute dense Jacobian matrices for specified articulations on the GPU.
    ///
    /// The size of Jacobians can vary by articulation, since it depends on the number of links, degrees-of-freedom, and whether the base is fixed.
    ///
    /// The size is determined using these formulas:
    /// nCols = (fixedBase ? 0 : 6) + dofCount
    /// nRows = (fixedBase ? 0 : 6) + (linkCount - 1) * 6;
    ///
    /// The user must ensure that adequate space is provided for each Jacobian matrix.
    PxScene_computeDenseJacobians_mut :: proc(self_: ^PxScene, indices: ^PxIndexDataPair, nbIndices: _c.uint32_t, computeEvent: rawptr) ---

    /// Compute the joint-space inertia matrices that maps joint accelerations to joint forces: forces = M * accelerations on the GPU.
    ///
    /// The size of matrices can vary by articulation, since it depends on the number of links and degrees-of-freedom.
    ///
    /// The size is determined using this formula:
    /// sizeof(float) * dofCount * dofCount
    ///
    /// The user must ensure that adequate space is provided for each mass matrix.
    PxScene_computeGeneralizedMassMatrices_mut :: proc(self_: ^PxScene, indices: ^PxIndexDataPair, nbIndices: _c.uint32_t, computeEvent: rawptr) ---

    /// Computes the joint DOF forces required to counteract gravitational forces for the given articulation pose.
    ///
    /// The size of the result can vary by articulation, since it depends on the number of links and degrees-of-freedom.
    ///
    /// The size is determined using this formula:
    /// sizeof(float) * dofCount
    ///
    /// The user must ensure that adequate space is provided for each articulation.
    PxScene_computeGeneralizedGravityForces_mut :: proc(self_: ^PxScene, indices: ^PxIndexDataPair, nbIndices: _c.uint32_t, computeEvent: rawptr) ---

    /// Computes the joint DOF forces required to counteract coriolis and centrifugal forces for the given articulation pose.
    ///
    /// The size of the result can vary by articulation, since it depends on the number of links and degrees-of-freedom.
    ///
    /// The size is determined using this formula:
    /// sizeof(float) * dofCount
    ///
    /// The user must ensure that adequate space is provided for each articulation.
    PxScene_computeCoriolisAndCentrifugalForces_mut :: proc(self_: ^PxScene, indices: ^PxIndexDataPair, nbIndices: _c.uint32_t, computeEvent: rawptr) ---

    PxScene_getGpuDynamicsConfig :: proc(self_: ^PxScene) -> PxgDynamicsMemoryConfig ---

    /// Apply user-provided data to particle buffers.
    ///
    /// This function should be used if the particle buffer flags are already on the device. Otherwise, use PxParticleBuffer::raiseFlags()
    /// from the CPU.
    ///
    /// This assumes the data has been changed directly in the PxParticleBuffer.
    PxScene_applyParticleBufferData_mut :: proc(self_: ^PxScene, indices: ^_c.uint32_t, bufferIndexPair: ^PxGpuParticleBufferIndexPair, flags: ^_c.uint32_t, nbUpdatedBuffers: _c.uint32_t, waitEvent: rawptr, signalEvent: rawptr) ---

    /// Constructor
    PxSceneReadLock_new_alloc :: proc(scene: ^PxScene, file: ^_c.char, line: _c.uint32_t) -> ^PxSceneReadLock ---

    PxSceneReadLock_delete :: proc(self_: ^PxSceneReadLock) ---

    /// Constructor
    PxSceneWriteLock_new_alloc :: proc(scene: ^PxScene, file: ^_c.char, line: _c.uint32_t) -> ^PxSceneWriteLock ---

    PxSceneWriteLock_delete :: proc(self_: ^PxSceneWriteLock) ---

    PxContactPairExtraDataItem_new :: proc() -> PxContactPairExtraDataItem ---

    PxContactPairVelocity_new :: proc() -> PxContactPairVelocity ---

    PxContactPairPose_new :: proc() -> PxContactPairPose ---

    PxContactPairIndex_new :: proc() -> PxContactPairIndex ---

    /// Constructor
    PxContactPairExtraDataIterator_new :: proc(stream: ^_c.uint8_t, size: _c.uint32_t) -> PxContactPairExtraDataIterator ---

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
    PxContactPairExtraDataIterator_nextItemSet_mut :: proc(self_: ^PxContactPairExtraDataIterator) -> _c.bool ---

    PxContactPairHeader_new :: proc() -> PxContactPairHeader ---

    PxContactPair_new :: proc() -> PxContactPair ---

    /// Extracts the contact points from the stream and stores them in a convenient format.
    ///
    /// Number of contact points written to the buffer.
    PxContactPair_extractContacts :: proc(self_: ^PxContactPair, userBuffer: ^PxContactPairPoint, bufferSize: _c.uint32_t) -> _c.uint32_t ---

    /// Helper method to clone the contact pair and copy the contact data stream into a user buffer.
    ///
    /// The contact data stream is only accessible during the contact report callback. This helper function provides copy functionality
    /// to buffer the contact stream information such that it can get accessed at a later stage.
    PxContactPair_bufferContacts :: proc(self_: ^PxContactPair, newPair: ^PxContactPair, bufferMemory: ^_c.uint8_t) ---

    PxContactPair_getInternalFaceIndices :: proc(self_: ^PxContactPair) -> ^_c.uint32_t ---

    PxTriggerPair_new :: proc() -> PxTriggerPair ---

    PxConstraintInfo_new :: proc() -> PxConstraintInfo ---

    PxConstraintInfo_new_1 :: proc(c: ^PxConstraint, extRef: rawptr, t: _c.uint32_t) -> PxConstraintInfo ---

    /// This is called when a breakable constraint breaks.
    ///
    /// The user should not release the constraint shader inside this call!
    ///
    /// No event will get reported if the constraint breaks but gets deleted while the time step is still being simulated.
    PxSimulationEventCallback_onConstraintBreak_mut :: proc(self_: ^PxSimulationEventCallback, constraints: ^PxConstraintInfo, count: _c.uint32_t) ---

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
    PxSimulationEventCallback_onWake_mut :: proc(self_: ^PxSimulationEventCallback, actors: ^^PxActor, count: _c.uint32_t) ---

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
    PxSimulationEventCallback_onSleep_mut :: proc(self_: ^PxSimulationEventCallback, actors: ^^PxActor, count: _c.uint32_t) ---

    /// This is called when certain contact events occur.
    ///
    /// The method will be called for a pair of actors if one of the colliding shape pairs requested contact notification.
    /// You request which events are reported using the filter shader/callback mechanism (see [`PxSimulationFilterShader`],
    /// [`PxSimulationFilterCallback`], #PxPairFlag).
    ///
    /// Do not keep references to the passed objects, as they will be
    /// invalid after this function returns.
    PxSimulationEventCallback_onContact_mut :: proc(self_: ^PxSimulationEventCallback, pairHeader: ^PxContactPairHeader, pairs: ^PxContactPair, nbPairs: _c.uint32_t) ---

    /// This is called with the current trigger pair events.
    ///
    /// Shapes which have been marked as triggers using PxShapeFlag::eTRIGGER_SHAPE will send events
    /// according to the pair flag specification in the filter shader (see [`PxPairFlag`], #PxSimulationFilterShader).
    ///
    /// Trigger shapes will no longer send notification events for interactions with other trigger shapes.
    PxSimulationEventCallback_onTrigger_mut :: proc(self_: ^PxSimulationEventCallback, pairs: ^PxTriggerPair, count: _c.uint32_t) ---

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
    PxSimulationEventCallback_onAdvance_mut :: proc(self_: ^PxSimulationEventCallback, bodyBuffer: ^^PxRigidBody, poseBuffer: ^PxTransform, count: _c.uint32_t) ---

    PxSimulationEventCallback_delete :: proc(self_: ^PxSimulationEventCallback) ---

    PxFEMParameters_new :: proc() -> PxFEMParameters ---

    /// Release this object.
    PxPruningStructure_release_mut :: proc(self_: ^PxPruningStructure) ---

    /// Retrieve rigid actors in the pruning structure.
    ///
    /// You can retrieve the number of rigid actor pointers by calling [`getNbRigidActors`]()
    ///
    /// Number of rigid actor pointers written to the buffer.
    PxPruningStructure_getRigidActors :: proc(self_: ^PxPruningStructure, userBuffer: ^^PxRigidActor, bufferSize: _c.uint32_t, startIndex: _c.uint32_t) -> _c.uint32_t ---

    /// Returns the number of rigid actors in the pruning structure.
    ///
    /// You can use [`getRigidActors`]() to retrieve the rigid actor pointers.
    ///
    /// Number of rigid actors in the pruning structure.
    PxPruningStructure_getNbRigidActors :: proc(self_: ^PxPruningStructure) -> _c.uint32_t ---

    /// Gets the merge data for static actors
    ///
    /// This is mainly called by the PxSceneQuerySystem::merge() function to merge a PxPruningStructure
    /// with the internal data-structures of the scene-query system.
    ///
    /// Implementation-dependent merge data for static actors.
    PxPruningStructure_getStaticMergeData :: proc(self_: ^PxPruningStructure) -> rawptr ---

    /// Gets the merge data for dynamic actors
    ///
    /// This is mainly called by the PxSceneQuerySystem::merge() function to merge a PxPruningStructure
    /// with the internal data-structures of the scene-query system.
    ///
    /// Implementation-dependent merge data for dynamic actors.
    PxPruningStructure_getDynamicMergeData :: proc(self_: ^PxPruningStructure) -> rawptr ---

    PxPruningStructure_getConcreteTypeName :: proc(self_: ^PxPruningStructure) -> ^_c.char ---

    PxExtendedVec3_new :: proc() -> PxExtendedVec3 ---

    PxExtendedVec3_new_1 :: proc(_x: _c.double, _y: _c.double, _z: _c.double) -> PxExtendedVec3 ---

    PxExtendedVec3_isZero :: proc(self_: ^PxExtendedVec3) -> _c.bool ---

    PxExtendedVec3_dot :: proc(self_: ^PxExtendedVec3, v: ^PxVec3) -> _c.double ---

    PxExtendedVec3_distanceSquared :: proc(self_: ^PxExtendedVec3, v: ^PxExtendedVec3) -> _c.double ---

    PxExtendedVec3_magnitudeSquared :: proc(self_: ^PxExtendedVec3) -> _c.double ---

    PxExtendedVec3_magnitude :: proc(self_: ^PxExtendedVec3) -> _c.double ---

    PxExtendedVec3_normalize_mut :: proc(self_: ^PxExtendedVec3) -> _c.double ---

    PxExtendedVec3_isFinite :: proc(self_: ^PxExtendedVec3) -> _c.bool ---

    PxExtendedVec3_maximum_mut :: proc(self_: ^PxExtendedVec3, v: ^PxExtendedVec3) ---

    PxExtendedVec3_minimum_mut :: proc(self_: ^PxExtendedVec3, v: ^PxExtendedVec3) ---

    PxExtendedVec3_set_mut :: proc(self_: ^PxExtendedVec3, x_: _c.double, y_: _c.double, z_: _c.double) ---

    PxExtendedVec3_setPlusInfinity_mut :: proc(self_: ^PxExtendedVec3) ---

    PxExtendedVec3_setMinusInfinity_mut :: proc(self_: ^PxExtendedVec3) ---

    PxExtendedVec3_cross_mut :: proc(self_: ^PxExtendedVec3, left: ^PxExtendedVec3, right: ^PxVec3) ---

    PxExtendedVec3_cross_mut_1 :: proc(self_: ^PxExtendedVec3, left: ^PxExtendedVec3, right: ^PxExtendedVec3) ---

    PxExtendedVec3_cross :: proc(self_: ^PxExtendedVec3, v: ^PxExtendedVec3) -> PxExtendedVec3 ---

    PxExtendedVec3_cross_mut_2 :: proc(self_: ^PxExtendedVec3, left: ^PxVec3, right: ^PxExtendedVec3) ---

    phys_toVec3 :: proc(v: ^PxExtendedVec3) -> PxVec3 ---

    PxObstacle_getType :: proc(self_: ^PxObstacle) -> _c.int32_t ---

    PxBoxObstacle_new :: proc() -> PxBoxObstacle ---

    PxCapsuleObstacle_new :: proc() -> PxCapsuleObstacle ---

    /// Releases the context.
    PxObstacleContext_release_mut :: proc(self_: ^PxObstacleContext) ---

    /// Retrieves the controller manager associated with this context.
    ///
    /// The associated controller manager
    PxObstacleContext_getControllerManager :: proc(self_: ^PxObstacleContext) -> ^PxControllerManager ---

    /// Adds an obstacle to the context.
    ///
    /// Handle for newly-added obstacle
    PxObstacleContext_addObstacle_mut :: proc(self_: ^PxObstacleContext, obstacle: ^PxObstacle) -> _c.uint32_t ---

    /// Removes an obstacle from the context.
    ///
    /// True if success
    PxObstacleContext_removeObstacle_mut :: proc(self_: ^PxObstacleContext, handle: _c.uint32_t) -> _c.bool ---

    /// Updates data for an existing obstacle.
    ///
    /// True if success
    PxObstacleContext_updateObstacle_mut :: proc(self_: ^PxObstacleContext, handle: _c.uint32_t, obstacle: ^PxObstacle) -> _c.bool ---

    /// Retrieves number of obstacles in the context.
    ///
    /// Number of obstacles in the context
    PxObstacleContext_getNbObstacles :: proc(self_: ^PxObstacleContext) -> _c.uint32_t ---

    /// Retrieves desired obstacle.
    ///
    /// Desired obstacle
    PxObstacleContext_getObstacle :: proc(self_: ^PxObstacleContext, i: _c.uint32_t) -> ^PxObstacle ---

    /// Retrieves desired obstacle by given handle.
    ///
    /// Desired obstacle
    PxObstacleContext_getObstacleByHandle :: proc(self_: ^PxObstacleContext, handle: _c.uint32_t) -> ^PxObstacle ---

    /// Called when current controller hits a shape.
    ///
    /// This is called when the CCT moves and hits a shape. This will not be called when a moving shape hits a non-moving CCT.
    PxUserControllerHitReport_onShapeHit_mut :: proc(self_: ^PxUserControllerHitReport, hit: ^PxControllerShapeHit) ---

    /// Called when current controller hits another controller.
    PxUserControllerHitReport_onControllerHit_mut :: proc(self_: ^PxUserControllerHitReport, hit: ^PxControllersHit) ---

    /// Called when current controller hits a user-defined obstacle.
    PxUserControllerHitReport_onObstacleHit_mut :: proc(self_: ^PxUserControllerHitReport, hit: ^PxControllerObstacleHit) ---

    PxControllerFilterCallback_delete :: proc(self_: ^PxControllerFilterCallback) ---

    /// Filtering method for CCT-vs-CCT.
    ///
    /// true to keep the pair, false to filter it out
    PxControllerFilterCallback_filter_mut :: proc(self_: ^PxControllerFilterCallback, a: ^PxController, b: ^PxController) -> _c.bool ---

    PxControllerFilters_new :: proc(filterData: ^PxFilterData, cb: ^PxQueryFilterCallback, cctFilterCb: ^PxControllerFilterCallback) -> PxControllerFilters ---

    /// returns true if the current settings are valid
    ///
    /// True if the descriptor is valid.
    PxControllerDesc_isValid :: proc(self_: ^PxControllerDesc) -> _c.bool ---

    /// Returns the character controller type
    ///
    /// The controllers type.
    PxControllerDesc_getType :: proc(self_: ^PxControllerDesc) -> _c.int32_t ---

    /// Return the type of controller
    PxController_getType :: proc(self_: ^PxController) -> _c.int32_t ---

    /// Releases the controller.
    PxController_release_mut :: proc(self_: ^PxController) ---

    /// Moves the character using a "collide-and-slide" algorithm.
    ///
    /// Collision flags, collection of ::PxControllerCollisionFlags
    PxController_move_mut :: proc(self_: ^PxController, disp: ^PxVec3, minDist: _c.float, elapsedTime: _c.float, filters: ^PxControllerFilters, obstacles: ^PxObstacleContext) -> _c.uint8_t ---

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
    PxController_setPosition_mut :: proc(self_: ^PxController, position: ^PxExtendedVec3) -> _c.bool ---

    /// Retrieve the raw position of the controller.
    ///
    /// The position retrieved by this function is the center of the collision shape. To retrieve the bottom position of the shape,
    /// a.k.a. the foot position, use the getFootPosition() function.
    ///
    /// The position is updated by calls to move(). Calling this method without calling
    /// move() will return the last position or the initial position of the controller.
    ///
    /// The controller's center position
    PxController_getPosition :: proc(self_: ^PxController) -> ^PxExtendedVec3 ---

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
    PxController_setFootPosition_mut :: proc(self_: ^PxController, position: ^PxExtendedVec3) -> _c.bool ---

    /// Retrieve the "foot" position of the controller, i.e. the position of the bottom of the CCT's shape.
    ///
    /// The foot position takes the contact offset into account
    ///
    /// The controller's foot position
    PxController_getFootPosition :: proc(self_: ^PxController) -> PxExtendedVec3 ---

    /// Get the rigid body actor associated with this controller (see PhysX documentation).
    /// The behavior upon manually altering this actor is undefined, you should primarily
    /// use it for reading const properties.
    ///
    /// the actor associated with the controller.
    PxController_getActor :: proc(self_: ^PxController) -> ^PxRigidDynamic ---

    /// The step height.
    PxController_setStepOffset_mut :: proc(self_: ^PxController, offset: _c.float) ---

    /// Retrieve the step height.
    ///
    /// The step offset for the controller.
    PxController_getStepOffset :: proc(self_: ^PxController) -> _c.float ---

    /// Sets the non-walkable mode for the CCT.
    PxController_setNonWalkableMode_mut :: proc(self_: ^PxController, flag: _c.int32_t) ---

    /// Retrieves the non-walkable mode for the CCT.
    ///
    /// The current non-walkable mode.
    PxController_getNonWalkableMode :: proc(self_: ^PxController) -> _c.int32_t ---

    /// Retrieve the contact offset.
    ///
    /// The contact offset for the controller.
    PxController_getContactOffset :: proc(self_: ^PxController) -> _c.float ---

    /// Sets the contact offset.
    PxController_setContactOffset_mut :: proc(self_: ^PxController, offset: _c.float) ---

    /// Retrieve the 'up' direction.
    ///
    /// The up direction for the controller.
    PxController_getUpDirection :: proc(self_: ^PxController) -> PxVec3 ---

    /// Sets the 'up' direction.
    PxController_setUpDirection_mut :: proc(self_: ^PxController, up: ^PxVec3) ---

    /// Retrieve the slope limit.
    ///
    /// The slope limit for the controller.
    PxController_getSlopeLimit :: proc(self_: ^PxController) -> _c.float ---

    /// Sets the slope limit.
    ///
    /// This feature can not be enabled at runtime, i.e. if the slope limit is zero when creating the CCT
    /// (which disables the feature) then changing the slope limit at runtime will not have any effect, and the call
    /// will be ignored.
    PxController_setSlopeLimit_mut :: proc(self_: ^PxController, slopeLimit: _c.float) ---

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
    PxController_invalidateCache_mut :: proc(self_: ^PxController) ---

    /// Retrieve the scene associated with the controller.
    ///
    /// The physics scene
    PxController_getScene_mut :: proc(self_: ^PxController) -> ^PxScene ---

    /// Returns the user data associated with this controller.
    ///
    /// The user pointer associated with the controller.
    PxController_getUserData :: proc(self_: ^PxController) -> rawptr ---

    /// Sets the user data associated with this controller.
    PxController_setUserData_mut :: proc(self_: ^PxController, userData: rawptr) ---

    /// Returns information about the controller's internal state.
    PxController_getState :: proc(self_: ^PxController, state: ^PxControllerState) ---

    /// Returns the controller's internal statistics.
    PxController_getStats :: proc(self_: ^PxController, stats: ^PxControllerStats) ---

    /// Resizes the controller.
    ///
    /// This function attempts to resize the controller to a given size, while making sure the bottom
    /// position of the controller remains constant. In other words the function modifies both the
    /// height and the (center) position of the controller. This is a helper function that can be used
    /// to implement a 'crouch' functionality for example.
    PxController_resize_mut :: proc(self_: ^PxController, height: _c.float) ---

    /// constructor sets to default.
    PxBoxControllerDesc_new_alloc :: proc() -> ^PxBoxControllerDesc ---

    PxBoxControllerDesc_delete :: proc(self_: ^PxBoxControllerDesc) ---

    /// (re)sets the structure to the default.
    PxBoxControllerDesc_setToDefault_mut :: proc(self_: ^PxBoxControllerDesc) ---

    /// returns true if the current settings are valid
    ///
    /// True if the descriptor is valid.
    PxBoxControllerDesc_isValid :: proc(self_: ^PxBoxControllerDesc) -> _c.bool ---

    /// Gets controller's half height.
    ///
    /// The half height of the controller.
    PxBoxController_getHalfHeight :: proc(self_: ^PxBoxController) -> _c.float ---

    /// Gets controller's half side extent.
    ///
    /// The half side extent of the controller.
    PxBoxController_getHalfSideExtent :: proc(self_: ^PxBoxController) -> _c.float ---

    /// Gets controller's half forward extent.
    ///
    /// The half forward extent of the controller.
    PxBoxController_getHalfForwardExtent :: proc(self_: ^PxBoxController) -> _c.float ---

    /// Sets controller's half height.
    ///
    /// this doesn't check for collisions.
    ///
    /// Currently always true.
    PxBoxController_setHalfHeight_mut :: proc(self_: ^PxBoxController, halfHeight: _c.float) -> _c.bool ---

    /// Sets controller's half side extent.
    ///
    /// this doesn't check for collisions.
    ///
    /// Currently always true.
    PxBoxController_setHalfSideExtent_mut :: proc(self_: ^PxBoxController, halfSideExtent: _c.float) -> _c.bool ---

    /// Sets controller's half forward extent.
    ///
    /// this doesn't check for collisions.
    ///
    /// Currently always true.
    PxBoxController_setHalfForwardExtent_mut :: proc(self_: ^PxBoxController, halfForwardExtent: _c.float) -> _c.bool ---

    /// constructor sets to default.
    PxCapsuleControllerDesc_new_alloc :: proc() -> ^PxCapsuleControllerDesc ---

    PxCapsuleControllerDesc_delete :: proc(self_: ^PxCapsuleControllerDesc) ---

    /// (re)sets the structure to the default.
    PxCapsuleControllerDesc_setToDefault_mut :: proc(self_: ^PxCapsuleControllerDesc) ---

    /// returns true if the current settings are valid
    ///
    /// True if the descriptor is valid.
    PxCapsuleControllerDesc_isValid :: proc(self_: ^PxCapsuleControllerDesc) -> _c.bool ---

    /// Gets controller's radius.
    ///
    /// The radius of the controller.
    PxCapsuleController_getRadius :: proc(self_: ^PxCapsuleController) -> _c.float ---

    /// Sets controller's radius.
    ///
    /// this doesn't check for collisions.
    ///
    /// Currently always true.
    PxCapsuleController_setRadius_mut :: proc(self_: ^PxCapsuleController, radius: _c.float) -> _c.bool ---

    /// Gets controller's height.
    ///
    /// The height of the capsule controller.
    PxCapsuleController_getHeight :: proc(self_: ^PxCapsuleController) -> _c.float ---

    /// Resets controller's height.
    ///
    /// this doesn't check for collisions.
    ///
    /// Currently always true.
    PxCapsuleController_setHeight_mut :: proc(self_: ^PxCapsuleController, height: _c.float) -> _c.bool ---

    /// Gets controller's climbing mode.
    ///
    /// The capsule controller's climbing mode.
    PxCapsuleController_getClimbingMode :: proc(self_: ^PxCapsuleController) -> _c.int32_t ---

    /// Sets controller's climbing mode.
    PxCapsuleController_setClimbingMode_mut :: proc(self_: ^PxCapsuleController, mode: _c.int32_t) -> _c.bool ---

    /// Retrieve behavior flags for a shape.
    ///
    /// When the CCT touches a shape, the CCT's behavior w.r.t. this shape can be customized by users.
    /// This function retrieves the desired PxControllerBehaviorFlag flags capturing the desired behavior.
    ///
    /// See comments about deprecated functions at the start of this class
    ///
    /// Desired behavior flags for the given shape
    PxControllerBehaviorCallback_getBehaviorFlags_mut :: proc(self_: ^PxControllerBehaviorCallback, shape: ^PxShape, actor: ^PxActor) -> _c.uint8_t ---

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
    PxControllerBehaviorCallback_getBehaviorFlags_mut_1 :: proc(self_: ^PxControllerBehaviorCallback, controller: ^PxController) -> _c.uint8_t ---

    /// Retrieve behavior flags for an obstacle.
    ///
    /// When the CCT touches an obstacle, the CCT's behavior w.r.t. this obstacle can be customized by users.
    /// This function retrieves the desired PxControllerBehaviorFlag flags capturing the desired behavior.
    ///
    /// See comments about deprecated functions at the start of this class
    ///
    /// Desired behavior flags for the given obstacle
    PxControllerBehaviorCallback_getBehaviorFlags_mut_2 :: proc(self_: ^PxControllerBehaviorCallback, obstacle: ^PxObstacle) -> _c.uint8_t ---

    /// Releases the controller manager.
    ///
    /// This will release all associated controllers and obstacle contexts.
    ///
    /// This function is required to be called to release foundation usage.
    PxControllerManager_release_mut :: proc(self_: ^PxControllerManager) ---

    /// Returns the scene the manager is adding the controllers to.
    ///
    /// The associated physics scene.
    PxControllerManager_getScene :: proc(self_: ^PxControllerManager) -> ^PxScene ---

    /// Returns the number of controllers that are being managed.
    ///
    /// The number of controllers.
    PxControllerManager_getNbControllers :: proc(self_: ^PxControllerManager) -> _c.uint32_t ---

    /// Retrieve one of the controllers in the manager.
    ///
    /// The controller with the specified index.
    PxControllerManager_getController_mut :: proc(self_: ^PxControllerManager, index: _c.uint32_t) -> ^PxController ---

    /// Creates a new character controller.
    ///
    /// The new controller
    PxControllerManager_createController_mut :: proc(self_: ^PxControllerManager, desc: ^PxControllerDesc) -> ^PxController ---

    /// Releases all the controllers that are being managed.
    PxControllerManager_purgeControllers_mut :: proc(self_: ^PxControllerManager) ---

    /// Retrieves debug data.
    ///
    /// The render buffer filled with debug-render data
    PxControllerManager_getRenderBuffer_mut :: proc(self_: ^PxControllerManager) -> ^PxRenderBuffer ---

    /// Sets debug rendering flags
    PxControllerManager_setDebugRenderingFlags_mut :: proc(self_: ^PxControllerManager, flags: _c.uint32_t) ---

    /// Returns the number of obstacle contexts that are being managed.
    ///
    /// The number of obstacle contexts.
    PxControllerManager_getNbObstacleContexts :: proc(self_: ^PxControllerManager) -> _c.uint32_t ---

    /// Retrieve one of the obstacle contexts in the manager.
    ///
    /// The obstacle context with the specified index.
    PxControllerManager_getObstacleContext_mut :: proc(self_: ^PxControllerManager, index: _c.uint32_t) -> ^PxObstacleContext ---

    /// Creates an obstacle context.
    ///
    /// New obstacle context
    PxControllerManager_createObstacleContext_mut :: proc(self_: ^PxControllerManager) -> ^PxObstacleContext ---

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
    PxControllerManager_computeInteractions_mut :: proc(self_: ^PxControllerManager, elapsedTime: _c.float, cctFilterCb: ^PxControllerFilterCallback) ---

    /// Enables or disables runtime tessellation.
    ///
    /// Large triangles can create accuracy issues in the sweep code, which in turn can lead to characters not sliding smoothly
    /// against geometries, or even penetrating them. This feature allows one to reduce those issues by tessellating large
    /// triangles at runtime, before performing sweeps against them. The amount of tessellation is controlled by the 'maxEdgeLength' parameter.
    /// Any triangle with at least one edge length greater than the maxEdgeLength will get recursively tessellated, until resulting triangles are small enough.
    ///
    /// This features only applies to triangle meshes, convex meshes, heightfields and boxes.
    PxControllerManager_setTessellation_mut :: proc(self_: ^PxControllerManager, flag: _c.bool, maxEdgeLength: _c.float) ---

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
    PxControllerManager_setOverlapRecoveryModule_mut :: proc(self_: ^PxControllerManager, flag: _c.bool) ---

    /// Enables or disables the precise sweeps.
    ///
    /// Precise sweeps are more accurate, but also potentially slower than regular sweeps.
    ///
    /// By default, precise sweeps are enabled.
    PxControllerManager_setPreciseSweeps_mut :: proc(self_: ^PxControllerManager, flag: _c.bool) ---

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
    PxControllerManager_setPreventVerticalSlidingAgainstCeiling_mut :: proc(self_: ^PxControllerManager, flag: _c.bool) ---

    /// Shift the origin of the character controllers and obstacle objects by the specified vector.
    ///
    /// The positions of all character controllers, obstacle objects and the corresponding data structures will get adjusted to reflect the shifted origin location
    /// (the shift vector will get subtracted from all character controller and obstacle object positions).
    ///
    /// It is the user's responsibility to keep track of the summed total origin shift and adjust all input/output to/from PhysXCharacterKinematic accordingly.
    ///
    /// This call will not automatically shift the PhysX scene and its objects. You need to call PxScene::shiftOrigin() seperately to keep the systems in sync.
    PxControllerManager_shiftOrigin_mut :: proc(self_: ^PxControllerManager, shift: ^PxVec3) ---

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
    phys_PxCreateControllerManager :: proc(scene: ^PxScene, lockingEnabled: _c.bool) -> ^PxControllerManager ---

    PxDim3_new :: proc() -> PxDim3 ---

    /// Constructor
    PxSDFDesc_new :: proc() -> PxSDFDesc ---

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid
    PxSDFDesc_isValid :: proc(self_: ^PxSDFDesc) -> _c.bool ---

    /// constructor sets to default.
    PxConvexMeshDesc_new :: proc() -> PxConvexMeshDesc ---

    /// (re)sets the structure to the default.
    PxConvexMeshDesc_setToDefault_mut :: proc(self_: ^PxConvexMeshDesc) ---

    /// Returns true if the descriptor is valid.
    ///
    /// True if the current settings are valid
    PxConvexMeshDesc_isValid :: proc(self_: ^PxConvexMeshDesc) -> _c.bool ---

    /// Constructor sets to default.
    PxTriangleMeshDesc_new :: proc() -> PxTriangleMeshDesc ---

    /// (re)sets the structure to the default.
    PxTriangleMeshDesc_setToDefault_mut :: proc(self_: ^PxTriangleMeshDesc) ---

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid
    PxTriangleMeshDesc_isValid :: proc(self_: ^PxTriangleMeshDesc) -> _c.bool ---

    /// Constructor to build an empty tetmesh description
    PxTetrahedronMeshDesc_new :: proc() -> PxTetrahedronMeshDesc ---

    PxTetrahedronMeshDesc_isValid :: proc(self_: ^PxTetrahedronMeshDesc) -> _c.bool ---

    /// Constructor to build an empty simulation description
    PxSoftBodySimulationDataDesc_new :: proc() -> PxSoftBodySimulationDataDesc ---

    PxSoftBodySimulationDataDesc_isValid :: proc(self_: ^PxSoftBodySimulationDataDesc) -> _c.bool ---

    /// Desc initialization to default value.
    PxBVH34MidphaseDesc_setToDefault_mut :: proc(self_: ^PxBVH34MidphaseDesc) ---

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid.
    PxBVH34MidphaseDesc_isValid :: proc(self_: ^PxBVH34MidphaseDesc) -> _c.bool ---

    PxMidphaseDesc_new :: proc() -> PxMidphaseDesc ---

    /// Returns type of midphase mesh structure.
    ///
    /// PxMeshMidPhase::Enum
    PxMidphaseDesc_getType :: proc(self_: ^PxMidphaseDesc) -> _c.int32_t ---

    /// Initialize the midphase mesh structure descriptor
    PxMidphaseDesc_setToDefault_mut :: proc(self_: ^PxMidphaseDesc, type: _c.int32_t) ---

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid.
    PxMidphaseDesc_isValid :: proc(self_: ^PxMidphaseDesc) -> _c.bool ---

    PxBVHDesc_new :: proc() -> PxBVHDesc ---

    /// Initialize the BVH descriptor
    PxBVHDesc_setToDefault_mut :: proc(self_: ^PxBVHDesc) ---

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid.
    PxBVHDesc_isValid :: proc(self_: ^PxBVHDesc) -> _c.bool ---

    PxCookingParams_new :: proc(sc: ^PxTolerancesScale) -> PxCookingParams ---

    phys_PxGetStandaloneInsertionCallback :: proc() -> ^PxInsertionCallback ---

    /// Cooks a bounding volume hierarchy. The results are written to the stream.
    ///
    /// PxCookBVH() allows a BVH description to be cooked into a binary stream
    /// suitable for loading and performing BVH detection at runtime.
    ///
    /// true on success.
    phys_PxCookBVH :: proc(desc: ^PxBVHDesc, stream: ^PxOutputStream) -> _c.bool ---

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
    phys_PxCreateBVH :: proc(desc: ^PxBVHDesc, insertionCallback: ^PxInsertionCallback) -> ^PxBVH ---

    /// Cooks a heightfield. The results are written to the stream.
    ///
    /// To create a heightfield object there is an option to precompute some of calculations done while loading the heightfield data.
    ///
    /// cookHeightField() allows a heightfield description to be cooked into a binary stream
    /// suitable for loading and performing collision detection at runtime.
    ///
    /// true on success
    phys_PxCookHeightField :: proc(desc: ^PxHeightFieldDesc, stream: ^PxOutputStream) -> _c.bool ---

    /// Cooks and creates a heightfield mesh and inserts it into PxPhysics.
    ///
    /// PxHeightField pointer on success
    phys_PxCreateHeightField :: proc(desc: ^PxHeightFieldDesc, insertionCallback: ^PxInsertionCallback) -> ^PxHeightField ---

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
    phys_PxCookConvexMesh :: proc(params: ^PxCookingParams, desc: ^PxConvexMeshDesc, stream: ^PxOutputStream, condition: ^_c.int32_t) -> _c.bool ---

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
    phys_PxCreateConvexMesh :: proc(params: ^PxCookingParams, desc: ^PxConvexMeshDesc, insertionCallback: ^PxInsertionCallback, condition: ^_c.int32_t) -> ^PxConvexMesh ---

    /// Verifies if the convex mesh is valid. Prints an error message for each inconsistency found.
    ///
    /// The convex mesh descriptor must contain an already created convex mesh - the vertices, indices and polygons must be provided.
    ///
    /// This function should be used if PxConvexFlag::eDISABLE_MESH_VALIDATION is planned to be used in release builds.
    ///
    /// true if all the validity conditions hold, false otherwise.
    phys_PxValidateConvexMesh :: proc(params: ^PxCookingParams, desc: ^PxConvexMeshDesc) -> _c.bool ---

    /// Computed hull polygons from given vertices and triangles. Polygons are needed for PxConvexMeshDesc rather than triangles.
    ///
    /// Please note that the resulting polygons may have different number of vertices. Some vertices may be removed.
    /// The output vertices, indices and polygons must be used to construct a hull.
    ///
    /// The provided PxAllocatorCallback does allocate the out array's. It is the user responsibility to deallocated those
    /// array's.
    ///
    /// true on success
    phys_PxComputeHullPolygons :: proc(params: ^PxCookingParams, mesh: ^PxSimpleTriangleMesh, inCallback: ^PxAllocatorCallback, nbVerts: ^_c.uint32_t, vertices: ^^PxVec3, nbIndices: ^_c.uint32_t, indices: ^^_c.uint32_t, nbPolygons: ^_c.uint32_t, hullPolygons: ^^PxHullPolygon) -> _c.bool ---

    /// Verifies if the triangle mesh is valid. Prints an error message for each inconsistency found.
    ///
    /// The following conditions are true for a valid triangle mesh:
    /// 1. There are no duplicate vertices (within specified vertexWeldTolerance. See PxCookingParams::meshWeldTolerance)
    /// 2. There are no large triangles (within specified PxTolerancesScale.)
    ///
    /// true if all the validity conditions hold, false otherwise.
    phys_PxValidateTriangleMesh :: proc(params: ^PxCookingParams, desc: ^PxTriangleMeshDesc) -> _c.bool ---

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
    phys_PxCreateTriangleMesh :: proc(params: ^PxCookingParams, desc: ^PxTriangleMeshDesc, insertionCallback: ^PxInsertionCallback, condition: ^_c.int32_t) -> ^PxTriangleMesh ---

    /// Cooks a triangle mesh. The results are written to the stream.
    ///
    /// To create a triangle mesh object it is necessary to first 'cook' the mesh data into
    /// a form which allows the SDK to perform efficient collision detection.
    ///
    /// PxCookTriangleMesh() allows a mesh description to be cooked into a binary stream
    /// suitable for loading and performing collision detection at runtime.
    ///
    /// true on success
    phys_PxCookTriangleMesh :: proc(params: ^PxCookingParams, desc: ^PxTriangleMeshDesc, stream: ^PxOutputStream, condition: ^_c.int32_t) -> _c.bool ---

    PxDefaultMemoryOutputStream_new_alloc :: proc(allocator: ^PxAllocatorCallback) -> ^PxDefaultMemoryOutputStream ---

    PxDefaultMemoryOutputStream_delete :: proc(self_: ^PxDefaultMemoryOutputStream) ---

    PxDefaultMemoryOutputStream_write_mut :: proc(self_: ^PxDefaultMemoryOutputStream, src: rawptr, count: _c.uint32_t) -> _c.uint32_t ---

    PxDefaultMemoryOutputStream_getSize :: proc(self_: ^PxDefaultMemoryOutputStream) -> _c.uint32_t ---

    PxDefaultMemoryOutputStream_getData :: proc(self_: ^PxDefaultMemoryOutputStream) -> ^_c.uint8_t ---

    PxDefaultMemoryInputData_new_alloc :: proc(data: ^_c.uint8_t, length: _c.uint32_t) -> ^PxDefaultMemoryInputData ---

    PxDefaultMemoryInputData_read_mut :: proc(self_: ^PxDefaultMemoryInputData, dest: rawptr, count: _c.uint32_t) -> _c.uint32_t ---

    PxDefaultMemoryInputData_getLength :: proc(self_: ^PxDefaultMemoryInputData) -> _c.uint32_t ---

    PxDefaultMemoryInputData_seek_mut :: proc(self_: ^PxDefaultMemoryInputData, pos: _c.uint32_t) ---

    PxDefaultMemoryInputData_tell :: proc(self_: ^PxDefaultMemoryInputData) -> _c.uint32_t ---

    PxDefaultFileOutputStream_new_alloc :: proc(name: ^_c.char) -> ^PxDefaultFileOutputStream ---

    PxDefaultFileOutputStream_delete :: proc(self_: ^PxDefaultFileOutputStream) ---

    PxDefaultFileOutputStream_write_mut :: proc(self_: ^PxDefaultFileOutputStream, src: rawptr, count: _c.uint32_t) -> _c.uint32_t ---

    PxDefaultFileOutputStream_isValid_mut :: proc(self_: ^PxDefaultFileOutputStream) -> _c.bool ---

    PxDefaultFileInputData_new_alloc :: proc(name: ^_c.char) -> ^PxDefaultFileInputData ---

    PxDefaultFileInputData_delete :: proc(self_: ^PxDefaultFileInputData) ---

    PxDefaultFileInputData_read_mut :: proc(self_: ^PxDefaultFileInputData, dest: rawptr, count: _c.uint32_t) -> _c.uint32_t ---

    PxDefaultFileInputData_seek_mut :: proc(self_: ^PxDefaultFileInputData, pos: _c.uint32_t) ---

    PxDefaultFileInputData_tell :: proc(self_: ^PxDefaultFileInputData) -> _c.uint32_t ---

    PxDefaultFileInputData_getLength :: proc(self_: ^PxDefaultFileInputData) -> _c.uint32_t ---

    PxDefaultFileInputData_isValid :: proc(self_: ^PxDefaultFileInputData) -> _c.bool ---

    phys_platformAlignedAlloc :: proc(size: _c.size_t) -> rawptr ---

    phys_platformAlignedFree :: proc(ptr: rawptr) ---

    PxDefaultAllocator_allocate_mut :: proc(self_: ^PxDefaultAllocator, size: _c.size_t, anon_param1: ^_c.char, anon_param2: ^_c.char, anon_param3: _c.int32_t) -> rawptr ---

    PxDefaultAllocator_deallocate_mut :: proc(self_: ^PxDefaultAllocator, ptr: rawptr) ---

    PxDefaultAllocator_delete :: proc(self_: ^PxDefaultAllocator) ---

    /// Set the actors for this joint.
    ///
    /// An actor may be NULL to indicate the world frame. At most one of the actors may be NULL.
    PxJoint_setActors_mut :: proc(self_: ^PxJoint, actor0: ^PxRigidActor, actor1: ^PxRigidActor) ---

    /// Get the actors for this joint.
    PxJoint_getActors :: proc(self_: ^PxJoint, actor0: ^^PxRigidActor, actor1: ^^PxRigidActor) ---

    /// Set the joint local pose for an actor.
    ///
    /// This is the relative pose which locates the joint frame relative to the actor.
    PxJoint_setLocalPose_mut :: proc(self_: ^PxJoint, actor: _c.int32_t, localPose: ^PxTransform) ---

    /// get the joint local pose for an actor.
    ///
    /// return the local pose for this joint
    PxJoint_getLocalPose :: proc(self_: ^PxJoint, actor: _c.int32_t) -> PxTransform ---

    /// get the relative pose for this joint
    ///
    /// This function returns the pose of the joint frame of actor1 relative to actor0
    PxJoint_getRelativeTransform :: proc(self_: ^PxJoint) -> PxTransform ---

    /// get the relative linear velocity of the joint
    ///
    /// This function returns the linear velocity of the origin of the constraint frame of actor1, relative to the origin of the constraint
    /// frame of actor0. The value is returned in the constraint frame of actor0
    PxJoint_getRelativeLinearVelocity :: proc(self_: ^PxJoint) -> PxVec3 ---

    /// get the relative angular velocity of the joint
    ///
    /// This function returns the angular velocity of  actor1 relative to actor0. The value is returned in the constraint frame of actor0
    PxJoint_getRelativeAngularVelocity :: proc(self_: ^PxJoint) -> PxVec3 ---

    /// set the break force for this joint.
    ///
    /// if the constraint force or torque on the joint exceeds the specified values, the joint will break,
    /// at which point it will not constrain the two actors and the flag PxConstraintFlag::eBROKEN will be set. The
    /// force and torque are measured in the joint frame of the first actor
    PxJoint_setBreakForce_mut :: proc(self_: ^PxJoint, force: _c.float, torque: _c.float) ---

    /// get the break force for this joint.
    PxJoint_getBreakForce :: proc(self_: ^PxJoint, force: ^_c.float, torque: ^_c.float) ---

    /// set the constraint flags for this joint.
    PxJoint_setConstraintFlags_mut :: proc(self_: ^PxJoint, flags: _c.uint16_t) ---

    /// set a constraint flags for this joint to a specified value.
    PxJoint_setConstraintFlag_mut :: proc(self_: ^PxJoint, flag: _c.int32_t, value: _c.bool) ---

    /// get the constraint flags for this joint.
    ///
    /// the constraint flags
    PxJoint_getConstraintFlags :: proc(self_: ^PxJoint) -> _c.uint16_t ---

    /// set the inverse mass scale for actor0.
    PxJoint_setInvMassScale0_mut :: proc(self_: ^PxJoint, invMassScale: _c.float) ---

    /// get the inverse mass scale for actor0.
    ///
    /// inverse mass scale for actor0
    PxJoint_getInvMassScale0 :: proc(self_: ^PxJoint) -> _c.float ---

    /// set the inverse inertia scale for actor0.
    PxJoint_setInvInertiaScale0_mut :: proc(self_: ^PxJoint, invInertiaScale: _c.float) ---

    /// get the inverse inertia scale for actor0.
    ///
    /// inverse inertia scale for actor0
    PxJoint_getInvInertiaScale0 :: proc(self_: ^PxJoint) -> _c.float ---

    /// set the inverse mass scale for actor1.
    PxJoint_setInvMassScale1_mut :: proc(self_: ^PxJoint, invMassScale: _c.float) ---

    /// get the inverse mass scale for actor1.
    ///
    /// inverse mass scale for actor1
    PxJoint_getInvMassScale1 :: proc(self_: ^PxJoint) -> _c.float ---

    /// set the inverse inertia scale for actor1.
    PxJoint_setInvInertiaScale1_mut :: proc(self_: ^PxJoint, invInertiaScale: _c.float) ---

    /// get the inverse inertia scale for actor1.
    ///
    /// inverse inertia scale for actor1
    PxJoint_getInvInertiaScale1 :: proc(self_: ^PxJoint) -> _c.float ---

    /// Retrieves the PxConstraint corresponding to this joint.
    ///
    /// This can be used to determine, among other things, the force applied at the joint.
    ///
    /// the constraint
    PxJoint_getConstraint :: proc(self_: ^PxJoint) -> ^PxConstraint ---

    /// Sets a name string for the object that can be retrieved with getName().
    ///
    /// This is for debugging and is not used by the SDK. The string is not copied by the SDK,
    /// only the pointer is stored.
    PxJoint_setName_mut :: proc(self_: ^PxJoint, name: ^_c.char) ---

    /// Retrieves the name string set with setName().
    ///
    /// Name string associated with object.
    PxJoint_getName :: proc(self_: ^PxJoint) -> ^_c.char ---

    /// Deletes the joint.
    ///
    /// This call does not wake up the connected rigid bodies.
    PxJoint_release_mut :: proc(self_: ^PxJoint) ---

    /// Retrieves the scene which this joint belongs to.
    ///
    /// Owner Scene. NULL if not part of a scene.
    PxJoint_getScene :: proc(self_: ^PxJoint) -> ^PxScene ---

    /// Put class meta data in stream, used for serialization
    PxJoint_getBinaryMetaData :: proc(stream: ^PxOutputStream) ---

    PxSpring_new :: proc(stiffness_: _c.float, damping_: _c.float) -> PxSpring ---

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
    phys_PxSetJointGlobalFrame :: proc(joint: ^PxJoint, wsAnchor: ^PxVec3, wsAxis: ^PxVec3) ---

    /// Create a distance Joint.
    phys_PxDistanceJointCreate :: proc(physics: ^PxPhysics, actor0: ^PxRigidActor, localFrame0: ^PxTransform, actor1: ^PxRigidActor, localFrame1: ^PxTransform) -> ^PxDistanceJoint ---

    /// Return the current distance of the joint
    PxDistanceJoint_getDistance :: proc(self_: ^PxDistanceJoint) -> _c.float ---

    /// Set the allowed minimum distance for the joint.
    ///
    /// The minimum distance must be no more than the maximum distance
    ///
    /// Default
    /// 0.0f
    /// Range
    /// [0, PX_MAX_F32)
    PxDistanceJoint_setMinDistance_mut :: proc(self_: ^PxDistanceJoint, distance: _c.float) ---

    /// Get the allowed minimum distance for the joint.
    ///
    /// the allowed minimum distance
    PxDistanceJoint_getMinDistance :: proc(self_: ^PxDistanceJoint) -> _c.float ---

    /// Set the allowed maximum distance for the joint.
    ///
    /// The maximum distance must be no less than the minimum distance.
    ///
    /// Default
    /// 0.0f
    /// Range
    /// [0, PX_MAX_F32)
    PxDistanceJoint_setMaxDistance_mut :: proc(self_: ^PxDistanceJoint, distance: _c.float) ---

    /// Get the allowed maximum distance for the joint.
    ///
    /// the allowed maximum distance
    PxDistanceJoint_getMaxDistance :: proc(self_: ^PxDistanceJoint) -> _c.float ---

    /// Set the error tolerance of the joint.
    PxDistanceJoint_setTolerance_mut :: proc(self_: ^PxDistanceJoint, tolerance: _c.float) ---

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
    PxDistanceJoint_getTolerance :: proc(self_: ^PxDistanceJoint) -> _c.float ---

    /// Set the strength of the joint spring.
    ///
    /// The spring is used if enabled, and the distance exceeds the range [min-error, max+error].
    ///
    /// Default
    /// 0.0f
    /// Range
    /// [0, PX_MAX_F32)
    PxDistanceJoint_setStiffness_mut :: proc(self_: ^PxDistanceJoint, stiffness: _c.float) ---

    /// Get the strength of the joint spring.
    ///
    /// stiffness the spring strength of the joint
    PxDistanceJoint_getStiffness :: proc(self_: ^PxDistanceJoint) -> _c.float ---

    /// Set the damping of the joint spring.
    ///
    /// The spring is used if enabled, and the distance exceeds the range [min-error, max+error].
    ///
    /// Default
    /// 0.0f
    /// Range
    /// [0, PX_MAX_F32)
    PxDistanceJoint_setDamping_mut :: proc(self_: ^PxDistanceJoint, damping: _c.float) ---

    /// Get the damping of the joint spring.
    ///
    /// the degree of damping of the joint spring of the joint
    PxDistanceJoint_getDamping :: proc(self_: ^PxDistanceJoint) -> _c.float ---

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
    PxDistanceJoint_setContactDistance_mut :: proc(self_: ^PxDistanceJoint, contactDistance: _c.float) ---

    /// Get the contact distance.
    ///
    /// the contact distance
    PxDistanceJoint_getContactDistance :: proc(self_: ^PxDistanceJoint) -> _c.float ---

    /// Set the flags specific to the Distance Joint.
    ///
    /// Default
    /// PxDistanceJointFlag::eMAX_DISTANCE_ENABLED
    PxDistanceJoint_setDistanceJointFlags_mut :: proc(self_: ^PxDistanceJoint, flags: _c.uint16_t) ---

    /// Set a single flag specific to a Distance Joint to true or false.
    PxDistanceJoint_setDistanceJointFlag_mut :: proc(self_: ^PxDistanceJoint, flag: _c.int32_t, value: _c.bool) ---

    /// Get the flags specific to the Distance Joint.
    ///
    /// the joint flags
    PxDistanceJoint_getDistanceJointFlags :: proc(self_: ^PxDistanceJoint) -> _c.uint16_t ---

    /// Returns string name of PxDistanceJoint, used for serialization
    PxDistanceJoint_getConcreteTypeName :: proc(self_: ^PxDistanceJoint) -> ^_c.char ---

    /// Create a distance Joint.
    phys_PxContactJointCreate :: proc(physics: ^PxPhysics, actor0: ^PxRigidActor, localFrame0: ^PxTransform, actor1: ^PxRigidActor, localFrame1: ^PxTransform) -> ^PxContactJoint ---

    PxJacobianRow_new :: proc() -> PxJacobianRow ---

    PxJacobianRow_new_1 :: proc(lin0: ^PxVec3, lin1: ^PxVec3, ang0: ^PxVec3, ang1: ^PxVec3) -> PxJacobianRow ---

    /// Set the current contact of the joint
    PxContactJoint_setContact_mut :: proc(self_: ^PxContactJoint, contact: ^PxVec3) ---

    /// Set the current contact normal of the joint
    PxContactJoint_setContactNormal_mut :: proc(self_: ^PxContactJoint, contactNormal: ^PxVec3) ---

    /// Set the current penetration of the joint
    PxContactJoint_setPenetration_mut :: proc(self_: ^PxContactJoint, penetration: _c.float) ---

    /// Return the current contact of the joint
    PxContactJoint_getContact :: proc(self_: ^PxContactJoint) -> PxVec3 ---

    /// Return the current contact normal of the joint
    PxContactJoint_getContactNormal :: proc(self_: ^PxContactJoint) -> PxVec3 ---

    /// Return the current penetration value of the joint
    PxContactJoint_getPenetration :: proc(self_: ^PxContactJoint) -> _c.float ---

    PxContactJoint_getRestitution :: proc(self_: ^PxContactJoint) -> _c.float ---

    PxContactJoint_setRestitution_mut :: proc(self_: ^PxContactJoint, restitution: _c.float) ---

    PxContactJoint_getBounceThreshold :: proc(self_: ^PxContactJoint) -> _c.float ---

    PxContactJoint_setBounceThreshold_mut :: proc(self_: ^PxContactJoint, bounceThreshold: _c.float) ---

    /// Returns string name of PxContactJoint, used for serialization
    PxContactJoint_getConcreteTypeName :: proc(self_: ^PxContactJoint) -> ^_c.char ---

    PxContactJoint_computeJacobians :: proc(self_: ^PxContactJoint, jacobian: ^PxJacobianRow) ---

    PxContactJoint_getNbJacobianRows :: proc(self_: ^PxContactJoint) -> _c.uint32_t ---

    /// Create a fixed joint.
    phys_PxFixedJointCreate :: proc(physics: ^PxPhysics, actor0: ^PxRigidActor, localFrame0: ^PxTransform, actor1: ^PxRigidActor, localFrame1: ^PxTransform) -> ^PxFixedJoint ---

    /// Returns string name of PxFixedJoint, used for serialization
    PxFixedJoint_getConcreteTypeName :: proc(self_: ^PxFixedJoint) -> ^_c.char ---

    PxJointLimitParameters_new_alloc :: proc() -> ^PxJointLimitParameters ---

    /// Returns true if the current settings are valid.
    ///
    /// true if the current settings are valid
    PxJointLimitParameters_isValid :: proc(self_: ^PxJointLimitParameters) -> _c.bool ---

    PxJointLimitParameters_isSoft :: proc(self_: ^PxJointLimitParameters) -> _c.bool ---

    /// construct a linear hard limit
    PxJointLinearLimit_new :: proc(scale: ^PxTolerancesScale, extent: _c.float, contactDist_deprecated: _c.float) -> PxJointLinearLimit ---

    /// construct a linear soft limit
    PxJointLinearLimit_new_1 :: proc(extent: _c.float, spring: ^PxSpring) -> PxJointLinearLimit ---

    /// Returns true if the limit is valid
    ///
    /// true if the current settings are valid
    PxJointLinearLimit_isValid :: proc(self_: ^PxJointLinearLimit) -> _c.bool ---

    PxJointLinearLimit_delete :: proc(self_: ^PxJointLinearLimit) ---

    /// Construct a linear hard limit pair. The lower distance value must be less than the upper distance value.
    PxJointLinearLimitPair_new :: proc(scale: ^PxTolerancesScale, lowerLimit: _c.float, upperLimit: _c.float, contactDist_deprecated: _c.float) -> PxJointLinearLimitPair ---

    /// construct a linear soft limit pair
    PxJointLinearLimitPair_new_1 :: proc(lowerLimit: _c.float, upperLimit: _c.float, spring: ^PxSpring) -> PxJointLinearLimitPair ---

    /// Returns true if the limit is valid.
    ///
    /// true if the current settings are valid
    PxJointLinearLimitPair_isValid :: proc(self_: ^PxJointLinearLimitPair) -> _c.bool ---

    PxJointLinearLimitPair_delete :: proc(self_: ^PxJointLinearLimitPair) ---

    /// construct an angular hard limit pair.
    ///
    /// The lower value must be less than the upper value.
    PxJointAngularLimitPair_new :: proc(lowerLimit: _c.float, upperLimit: _c.float, contactDist_deprecated: _c.float) -> PxJointAngularLimitPair ---

    /// construct an angular soft limit pair.
    ///
    /// The lower value must be less than the upper value.
    PxJointAngularLimitPair_new_1 :: proc(lowerLimit: _c.float, upperLimit: _c.float, spring: ^PxSpring) -> PxJointAngularLimitPair ---

    /// Returns true if the limit is valid.
    ///
    /// true if the current settings are valid
    PxJointAngularLimitPair_isValid :: proc(self_: ^PxJointAngularLimitPair) -> _c.bool ---

    PxJointAngularLimitPair_delete :: proc(self_: ^PxJointAngularLimitPair) ---

    /// Construct a cone hard limit.
    PxJointLimitCone_new :: proc(yLimitAngle: _c.float, zLimitAngle: _c.float, contactDist_deprecated: _c.float) -> PxJointLimitCone ---

    /// Construct a cone soft limit.
    PxJointLimitCone_new_1 :: proc(yLimitAngle: _c.float, zLimitAngle: _c.float, spring: ^PxSpring) -> PxJointLimitCone ---

    /// Returns true if the limit is valid.
    ///
    /// true if the current settings are valid
    PxJointLimitCone_isValid :: proc(self_: ^PxJointLimitCone) -> _c.bool ---

    PxJointLimitCone_delete :: proc(self_: ^PxJointLimitCone) ---

    /// Construct a pyramid hard limit.
    PxJointLimitPyramid_new :: proc(yLimitAngleMin: _c.float, yLimitAngleMax: _c.float, zLimitAngleMin: _c.float, zLimitAngleMax: _c.float, contactDist_deprecated: _c.float) -> PxJointLimitPyramid ---

    /// Construct a pyramid soft limit.
    PxJointLimitPyramid_new_1 :: proc(yLimitAngleMin: _c.float, yLimitAngleMax: _c.float, zLimitAngleMin: _c.float, zLimitAngleMax: _c.float, spring: ^PxSpring) -> PxJointLimitPyramid ---

    /// Returns true if the limit is valid.
    ///
    /// true if the current settings are valid
    PxJointLimitPyramid_isValid :: proc(self_: ^PxJointLimitPyramid) -> _c.bool ---

    PxJointLimitPyramid_delete :: proc(self_: ^PxJointLimitPyramid) ---

    /// Create a prismatic joint.
    phys_PxPrismaticJointCreate :: proc(physics: ^PxPhysics, actor0: ^PxRigidActor, localFrame0: ^PxTransform, actor1: ^PxRigidActor, localFrame1: ^PxTransform) -> ^PxPrismaticJoint ---

    /// returns the displacement of the joint along its axis.
    PxPrismaticJoint_getPosition :: proc(self_: ^PxPrismaticJoint) -> _c.float ---

    /// returns the velocity of the joint along its axis
    PxPrismaticJoint_getVelocity :: proc(self_: ^PxPrismaticJoint) -> _c.float ---

    /// sets the joint limit  parameters.
    ///
    /// The limit range is [-PX_MAX_F32, PX_MAX_F32], but note that the width of the limit (upper-lower) must also be
    /// a valid float.
    PxPrismaticJoint_setLimit_mut :: proc(self_: ^PxPrismaticJoint, anon_param0: ^PxJointLinearLimitPair) ---

    /// gets the joint limit  parameters.
    PxPrismaticJoint_getLimit :: proc(self_: ^PxPrismaticJoint) -> PxJointLinearLimitPair ---

    /// Set the flags specific to the Prismatic Joint.
    ///
    /// Default
    /// PxPrismaticJointFlags(0)
    PxPrismaticJoint_setPrismaticJointFlags_mut :: proc(self_: ^PxPrismaticJoint, flags: _c.uint16_t) ---

    /// Set a single flag specific to a Prismatic Joint to true or false.
    PxPrismaticJoint_setPrismaticJointFlag_mut :: proc(self_: ^PxPrismaticJoint, flag: _c.int32_t, value: _c.bool) ---

    /// Get the flags specific to the Prismatic Joint.
    ///
    /// the joint flags
    PxPrismaticJoint_getPrismaticJointFlags :: proc(self_: ^PxPrismaticJoint) -> _c.uint16_t ---

    /// Returns string name of PxPrismaticJoint, used for serialization
    PxPrismaticJoint_getConcreteTypeName :: proc(self_: ^PxPrismaticJoint) -> ^_c.char ---

    /// Create a revolute joint.
    phys_PxRevoluteJointCreate :: proc(physics: ^PxPhysics, actor0: ^PxRigidActor, localFrame0: ^PxTransform, actor1: ^PxRigidActor, localFrame1: ^PxTransform) -> ^PxRevoluteJoint ---

    /// return the angle of the joint, in the range (-2*Pi, 2*Pi]
    PxRevoluteJoint_getAngle :: proc(self_: ^PxRevoluteJoint) -> _c.float ---

    /// return the velocity of the joint
    PxRevoluteJoint_getVelocity :: proc(self_: ^PxRevoluteJoint) -> _c.float ---

    /// set the joint limit parameters.
    ///
    /// The limit is activated using the flag PxRevoluteJointFlag::eLIMIT_ENABLED
    ///
    /// The limit angle range is (-2*Pi, 2*Pi).
    PxRevoluteJoint_setLimit_mut :: proc(self_: ^PxRevoluteJoint, limits: ^PxJointAngularLimitPair) ---

    /// get the joint limit parameters.
    ///
    /// the joint limit parameters
    PxRevoluteJoint_getLimit :: proc(self_: ^PxRevoluteJoint) -> PxJointAngularLimitPair ---

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
    PxRevoluteJoint_setDriveVelocity_mut :: proc(self_: ^PxRevoluteJoint, velocity: _c.float, autowake: _c.bool) ---

    /// gets the target velocity for the drive model.
    ///
    /// the drive target velocity
    PxRevoluteJoint_getDriveVelocity :: proc(self_: ^PxRevoluteJoint) -> _c.float ---

    /// sets the maximum torque the drive can exert.
    ///
    /// The value set here may be used either as an impulse limit or a force limit, depending on the flag PxConstraintFlag::eDRIVE_LIMITS_ARE_FORCES
    ///
    /// Range:
    /// [0, PX_MAX_F32)
    /// Default:
    /// PX_MAX_F32
    PxRevoluteJoint_setDriveForceLimit_mut :: proc(self_: ^PxRevoluteJoint, limit: _c.float) ---

    /// gets the maximum torque the drive can exert.
    ///
    /// the torque limit
    PxRevoluteJoint_getDriveForceLimit :: proc(self_: ^PxRevoluteJoint) -> _c.float ---

    /// sets the gear ratio for the drive.
    ///
    /// When setting up the drive constraint, the velocity of the first actor is scaled by this value, and its response to drive torque is scaled down.
    /// So if the drive target velocity is zero, the second actor will be driven to the velocity of the first scaled by the gear ratio
    ///
    /// Range:
    /// [0, PX_MAX_F32)
    /// Default:
    /// 1.0
    PxRevoluteJoint_setDriveGearRatio_mut :: proc(self_: ^PxRevoluteJoint, ratio: _c.float) ---

    /// gets the gear ratio.
    ///
    /// the drive gear ratio
    PxRevoluteJoint_getDriveGearRatio :: proc(self_: ^PxRevoluteJoint) -> _c.float ---

    /// sets the flags specific to the Revolute Joint.
    ///
    /// Default
    /// PxRevoluteJointFlags(0)
    PxRevoluteJoint_setRevoluteJointFlags_mut :: proc(self_: ^PxRevoluteJoint, flags: _c.uint16_t) ---

    /// sets a single flag specific to a Revolute Joint.
    PxRevoluteJoint_setRevoluteJointFlag_mut :: proc(self_: ^PxRevoluteJoint, flag: _c.int32_t, value: _c.bool) ---

    /// gets the flags specific to the Revolute Joint.
    ///
    /// the joint flags
    PxRevoluteJoint_getRevoluteJointFlags :: proc(self_: ^PxRevoluteJoint) -> _c.uint16_t ---

    /// Returns string name of PxRevoluteJoint, used for serialization
    PxRevoluteJoint_getConcreteTypeName :: proc(self_: ^PxRevoluteJoint) -> ^_c.char ---

    /// Create a spherical joint.
    phys_PxSphericalJointCreate :: proc(physics: ^PxPhysics, actor0: ^PxRigidActor, localFrame0: ^PxTransform, actor1: ^PxRigidActor, localFrame1: ^PxTransform) -> ^PxSphericalJoint ---

    /// Set the limit cone.
    ///
    /// If enabled, the limit cone will constrain the angular movement of the joint to lie
    /// within an elliptical cone.
    ///
    /// the limit cone
    PxSphericalJoint_getLimitCone :: proc(self_: ^PxSphericalJoint) -> PxJointLimitCone ---

    /// Get the limit cone.
    PxSphericalJoint_setLimitCone_mut :: proc(self_: ^PxSphericalJoint, limit: ^PxJointLimitCone) ---

    /// get the swing angle of the joint from the Y axis
    PxSphericalJoint_getSwingYAngle :: proc(self_: ^PxSphericalJoint) -> _c.float ---

    /// get the swing angle of the joint from the Z axis
    PxSphericalJoint_getSwingZAngle :: proc(self_: ^PxSphericalJoint) -> _c.float ---

    /// Set the flags specific to the Spherical Joint.
    ///
    /// Default
    /// PxSphericalJointFlags(0)
    PxSphericalJoint_setSphericalJointFlags_mut :: proc(self_: ^PxSphericalJoint, flags: _c.uint16_t) ---

    /// Set a single flag specific to a Spherical Joint to true or false.
    PxSphericalJoint_setSphericalJointFlag_mut :: proc(self_: ^PxSphericalJoint, flag: _c.int32_t, value: _c.bool) ---

    /// Get the flags specific to the Spherical Joint.
    ///
    /// the joint flags
    PxSphericalJoint_getSphericalJointFlags :: proc(self_: ^PxSphericalJoint) -> _c.uint16_t ---

    /// Returns string name of PxSphericalJoint, used for serialization
    PxSphericalJoint_getConcreteTypeName :: proc(self_: ^PxSphericalJoint) -> ^_c.char ---

    /// Create a D6 joint.
    phys_PxD6JointCreate :: proc(physics: ^PxPhysics, actor0: ^PxRigidActor, localFrame0: ^PxTransform, actor1: ^PxRigidActor, localFrame1: ^PxTransform) -> ^PxD6Joint ---

    /// default constructor for PxD6JointDrive.
    PxD6JointDrive_new :: proc() -> PxD6JointDrive ---

    /// constructor a PxD6JointDrive.
    PxD6JointDrive_new_1 :: proc(driveStiffness: _c.float, driveDamping: _c.float, driveForceLimit: _c.float, isAcceleration: _c.bool) -> PxD6JointDrive ---

    /// returns true if the drive is valid
    PxD6JointDrive_isValid :: proc(self_: ^PxD6JointDrive) -> _c.bool ---

    /// Set the motion type around the specified axis.
    ///
    /// Each axis may independently specify that the degree of freedom is locked (blocking relative movement
    /// along or around this axis), limited by the corresponding limit, or free.
    ///
    /// Default:
    /// all degrees of freedom are locked
    PxD6Joint_setMotion_mut :: proc(self_: ^PxD6Joint, axis: _c.int32_t, type: _c.int32_t) ---

    /// Get the motion type around the specified axis.
    ///
    /// the motion type around the specified axis
    PxD6Joint_getMotion :: proc(self_: ^PxD6Joint, axis: _c.int32_t) -> _c.int32_t ---

    /// get the twist angle of the joint, in the range (-2*Pi, 2*Pi]
    PxD6Joint_getTwistAngle :: proc(self_: ^PxD6Joint) -> _c.float ---

    /// get the swing angle of the joint from the Y axis
    PxD6Joint_getSwingYAngle :: proc(self_: ^PxD6Joint) -> _c.float ---

    /// get the swing angle of the joint from the Z axis
    PxD6Joint_getSwingZAngle :: proc(self_: ^PxD6Joint) -> _c.float ---

    /// Set the distance limit for the joint.
    ///
    /// A single limit constraints all linear limited degrees of freedom, forming a linear, circular
    /// or spherical constraint on motion depending on the number of limited degrees. This is similar
    /// to a distance limit.
    PxD6Joint_setDistanceLimit_mut :: proc(self_: ^PxD6Joint, limit: ^PxJointLinearLimit) ---

    /// Get the distance limit for the joint.
    ///
    /// the distance limit structure
    PxD6Joint_getDistanceLimit :: proc(self_: ^PxD6Joint) -> PxJointLinearLimit ---

    /// Set the linear limit for a given linear axis.
    ///
    /// This function extends the previous setDistanceLimit call with the following features:
    /// - there can be a different limit for each linear axis
    /// - each limit is defined by two values, i.e. it can now be asymmetric
    ///
    /// This can be used to create prismatic joints similar to PxPrismaticJoint, or point-in-quad joints,
    /// or point-in-box joints.
    PxD6Joint_setLinearLimit_mut :: proc(self_: ^PxD6Joint, axis: _c.int32_t, limit: ^PxJointLinearLimitPair) ---

    /// Get the linear limit for a given linear axis.
    ///
    /// the linear limit pair structure from desired axis
    PxD6Joint_getLinearLimit :: proc(self_: ^PxD6Joint, axis: _c.int32_t) -> PxJointLinearLimitPair ---

    /// Set the twist limit for the joint.
    ///
    /// The twist limit controls the range of motion around the twist axis.
    ///
    /// The limit angle range is (-2*Pi, 2*Pi).
    PxD6Joint_setTwistLimit_mut :: proc(self_: ^PxD6Joint, limit: ^PxJointAngularLimitPair) ---

    /// Get the twist limit for the joint.
    ///
    /// the twist limit structure
    PxD6Joint_getTwistLimit :: proc(self_: ^PxD6Joint) -> PxJointAngularLimitPair ---

    /// Set the swing cone limit for the joint.
    ///
    /// The cone limit is used if either or both swing axes are limited. The extents are
    /// symmetrical and measured in the frame of the parent. If only one swing degree of freedom
    /// is limited, the corresponding value from the cone limit defines the limit range.
    PxD6Joint_setSwingLimit_mut :: proc(self_: ^PxD6Joint, limit: ^PxJointLimitCone) ---

    /// Get the cone limit for the joint.
    ///
    /// the swing limit structure
    PxD6Joint_getSwingLimit :: proc(self_: ^PxD6Joint) -> PxJointLimitCone ---

    /// Set a pyramidal swing limit for the joint.
    ///
    /// The pyramid limits will only be used in the following cases:
    /// - both swing Y and Z are limited. The limit shape is then a pyramid.
    /// - Y is limited and Z is locked, or vice versa. The limit shape is an asymmetric angular section, similar to
    /// what is supported for the twist axis.
    /// The remaining cases (Y limited and Z is free, or vice versa) are not supported.
    PxD6Joint_setPyramidSwingLimit_mut :: proc(self_: ^PxD6Joint, limit: ^PxJointLimitPyramid) ---

    /// Get the pyramidal swing limit for the joint.
    ///
    /// the swing limit structure
    PxD6Joint_getPyramidSwingLimit :: proc(self_: ^PxD6Joint) -> PxJointLimitPyramid ---

    /// Set the drive parameters for the specified drive type.
    ///
    /// Default
    /// The default drive spring and damping values are zero, the force limit is zero, and no flags are set.
    PxD6Joint_setDrive_mut :: proc(self_: ^PxD6Joint, index: _c.int32_t, drive: ^PxD6JointDrive) ---

    /// Get the drive parameters for the specified drive type.
    PxD6Joint_getDrive :: proc(self_: ^PxD6Joint, index: _c.int32_t) -> PxD6JointDrive ---

    /// Set the drive goal pose
    ///
    /// The goal is relative to the constraint frame of actor[0]
    ///
    /// Default
    /// the identity transform
    PxD6Joint_setDrivePosition_mut :: proc(self_: ^PxD6Joint, pose: ^PxTransform, autowake: _c.bool) ---

    /// Get the drive goal pose.
    PxD6Joint_getDrivePosition :: proc(self_: ^PxD6Joint) -> PxTransform ---

    /// Set the target goal velocity for drive.
    ///
    /// The velocity is measured in the constraint frame of actor[0]
    PxD6Joint_setDriveVelocity_mut :: proc(self_: ^PxD6Joint, linear: ^PxVec3, angular: ^PxVec3, autowake: _c.bool) ---

    /// Get the target goal velocity for joint drive.
    PxD6Joint_getDriveVelocity :: proc(self_: ^PxD6Joint, linear: ^PxVec3, angular: ^PxVec3) ---

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
    PxD6Joint_setProjectionLinearTolerance_mut :: proc(self_: ^PxD6Joint, tolerance: _c.float) ---

    /// Get the linear tolerance threshold for projection.
    ///
    /// the linear tolerance threshold
    PxD6Joint_getProjectionLinearTolerance :: proc(self_: ^PxD6Joint) -> _c.float ---

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
    PxD6Joint_setProjectionAngularTolerance_mut :: proc(self_: ^PxD6Joint, tolerance: _c.float) ---

    /// Get the angular tolerance threshold for projection.
    ///
    /// tolerance the angular tolerance threshold in radians
    PxD6Joint_getProjectionAngularTolerance :: proc(self_: ^PxD6Joint) -> _c.float ---

    /// Returns string name of PxD6Joint, used for serialization
    PxD6Joint_getConcreteTypeName :: proc(self_: ^PxD6Joint) -> ^_c.char ---

    /// Create a gear Joint.
    phys_PxGearJointCreate :: proc(physics: ^PxPhysics, actor0: ^PxRigidActor, localFrame0: ^PxTransform, actor1: ^PxRigidActor, localFrame1: ^PxTransform) -> ^PxGearJoint ---

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
    PxGearJoint_setHinges_mut :: proc(self_: ^PxGearJoint, hinge0: ^PxBase, hinge1: ^PxBase) -> _c.bool ---

    /// Set the desired gear ratio.
    ///
    /// For two gears with n0 and n1 teeth respectively, the gear ratio is n0/n1.
    ///
    /// You may need to use a negative gear ratio if the joint frames of involved actors are not oriented in the same direction.
    ///
    /// Calling this function resets the internal positional error correction term.
    PxGearJoint_setGearRatio_mut :: proc(self_: ^PxGearJoint, ratio: _c.float) ---

    /// Get the gear ratio.
    ///
    /// Current ratio
    PxGearJoint_getGearRatio :: proc(self_: ^PxGearJoint) -> _c.float ---

    PxGearJoint_getConcreteTypeName :: proc(self_: ^PxGearJoint) -> ^_c.char ---

    /// Create a rack
    /// &
    /// pinion Joint.
    phys_PxRackAndPinionJointCreate :: proc(physics: ^PxPhysics, actor0: ^PxRigidActor, localFrame0: ^PxTransform, actor1: ^PxRigidActor, localFrame1: ^PxTransform) -> ^PxRackAndPinionJoint ---

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
    PxRackAndPinionJoint_setJoints_mut :: proc(self_: ^PxRackAndPinionJoint, hinge: ^PxBase, prismatic: ^PxBase) -> _c.bool ---

    /// Set the desired ratio directly.
    ///
    /// You may need to use a negative gear ratio if the joint frames of involved actors are not oriented in the same direction.
    ///
    /// Calling this function resets the internal positional error correction term.
    PxRackAndPinionJoint_setRatio_mut :: proc(self_: ^PxRackAndPinionJoint, ratio: _c.float) ---

    /// Get the ratio.
    ///
    /// Current ratio
    PxRackAndPinionJoint_getRatio :: proc(self_: ^PxRackAndPinionJoint) -> _c.float ---

    /// Set the desired ratio indirectly.
    ///
    /// This is a simple helper function that computes the ratio from passed data:
    ///
    /// ratio = (PI*2*nbRackTeeth)/(rackLength*nbPinionTeeth)
    ///
    /// Calling this function resets the internal positional error correction term.
    ///
    /// true if success
    PxRackAndPinionJoint_setData_mut :: proc(self_: ^PxRackAndPinionJoint, nbRackTeeth: _c.uint32_t, nbPinionTeeth: _c.uint32_t, rackLength: _c.float) -> _c.bool ---

    PxRackAndPinionJoint_getConcreteTypeName :: proc(self_: ^PxRackAndPinionJoint) -> ^_c.char ---

    PxGroupsMask_new_alloc :: proc() -> ^PxGroupsMask ---

    PxGroupsMask_delete :: proc(self_: ^PxGroupsMask) ---

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
    phys_PxDefaultSimulationFilterShader :: proc(attributes0: _c.uint32_t, filterData0: PxFilterData, attributes1: _c.uint32_t, filterData1: PxFilterData, pairFlags: ^_c.uint16_t, constantBlock: rawptr, constantBlockSize: _c.uint32_t) -> _c.uint16_t ---

    /// Determines if collision detection is performed between a pair of groups
    ///
    /// Collision group is an integer between 0 and 31.
    ///
    /// True if the groups could collide
    phys_PxGetGroupCollisionFlag :: proc(group1: _c.uint16_t, group2: _c.uint16_t) -> _c.bool ---

    /// Specifies if collision should be performed by a pair of groups
    ///
    /// Collision group is an integer between 0 and 31.
    phys_PxSetGroupCollisionFlag :: proc(group1: _c.uint16_t, group2: _c.uint16_t, enable: _c.bool) ---

    /// Retrieves the value set with PxSetGroup()
    ///
    /// Collision group is an integer between 0 and 31.
    ///
    /// The collision group this actor belongs to
    phys_PxGetGroup :: proc(actor: ^PxActor) -> _c.uint16_t ---

    /// Sets which collision group this actor is part of
    ///
    /// Collision group is an integer between 0 and 31.
    phys_PxSetGroup :: proc(actor: ^PxActor, collisionGroup: _c.uint16_t) ---

    /// Retrieves filtering operation. See comments for PxGroupsMask
    phys_PxGetFilterOps :: proc(op0: ^_c.int32_t, op1: ^_c.int32_t, op2: ^_c.int32_t) ---

    /// Setups filtering operations. See comments for PxGroupsMask
    phys_PxSetFilterOps :: proc(op0: ^_c.int32_t, op1: ^_c.int32_t, op2: ^_c.int32_t) ---

    /// Retrieves filtering's boolean value. See comments for PxGroupsMask
    ///
    /// flag Boolean value for filter.
    phys_PxGetFilterBool :: proc() -> _c.bool ---

    /// Setups filtering's boolean value. See comments for PxGroupsMask
    phys_PxSetFilterBool :: proc(enable: _c.bool) ---

    /// Gets filtering constant K0 and K1. See comments for PxGroupsMask
    phys_PxGetFilterConstants :: proc(c0: ^PxGroupsMask, c1: ^PxGroupsMask) ---

    /// Setups filtering's K0 and K1 value. See comments for PxGroupsMask
    phys_PxSetFilterConstants :: proc(c0: ^PxGroupsMask, c1: ^PxGroupsMask) ---

    /// Gets 64-bit mask used for collision filtering. See comments for PxGroupsMask
    ///
    /// The group mask for the actor.
    phys_PxGetGroupsMask :: proc(actor: ^PxActor) -> PxGroupsMask ---

    /// Sets 64-bit mask used for collision filtering. See comments for PxGroupsMask
    phys_PxSetGroupsMask :: proc(actor: ^PxActor, mask: ^PxGroupsMask) ---

    PxDefaultErrorCallback_new_alloc :: proc() -> ^PxDefaultErrorCallback ---

    PxDefaultErrorCallback_delete :: proc(self_: ^PxDefaultErrorCallback) ---

    PxDefaultErrorCallback_reportError_mut :: proc(self_: ^PxDefaultErrorCallback, code: _c.int32_t, message: ^_c.char, file: ^_c.char, line: _c.int32_t) ---

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
    PxRigidActorExt_createExclusiveShape :: proc(actor: ^PxRigidActor, geometry: ^PxGeometry, materials: ^^PxMaterial, materialCount: _c.uint16_t, shapeFlags: _c.uint8_t) -> ^PxShape ---

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
    PxRigidActorExt_createExclusiveShape_1 :: proc(actor: ^PxRigidActor, geometry: ^PxGeometry, material: ^PxMaterial, shapeFlags: _c.uint8_t) -> ^PxShape ---

    /// Gets a list of bounds based on shapes in rigid actor. This list can be used to cook/create
    /// bounding volume hierarchy though PxCooking API.
    PxRigidActorExt_getRigidActorShapeLocalBoundsList :: proc(actor: ^PxRigidActor, numBounds: ^_c.uint32_t) -> ^PxBounds3 ---

    /// Convenience function to create a PxBVH object from a PxRigidActor.
    ///
    /// The computed PxBVH can then be used in PxScene::addActor() or PxAggregate::addActor().
    /// After adding the actor
    /// &
    /// BVH to the scene/aggregate, release the PxBVH object by calling PxBVH::release().
    ///
    /// The PxBVH for this actor.
    PxRigidActorExt_createBVHFromActor :: proc(physics: ^PxPhysics, actor: ^PxRigidActor) -> ^PxBVH ---

    /// Default constructor.
    PxMassProperties_new :: proc() -> PxMassProperties ---

    /// Construct from individual elements.
    PxMassProperties_new_1 :: proc(m: _c.float, inertiaT: ^PxMat33, com: ^PxVec3) -> PxMassProperties ---

    /// Compute mass properties based on a provided geometry structure.
    ///
    /// This constructor assumes the geometry has a density of 1. Mass and inertia tensor scale linearly with density.
    PxMassProperties_new_2 :: proc(geometry: ^PxGeometry) -> PxMassProperties ---

    /// Translate the center of mass by a given vector and adjust the inertia tensor accordingly.
    PxMassProperties_translate_mut :: proc(self_: ^PxMassProperties, t: ^PxVec3) ---

    /// Get the entries of the diagonalized inertia tensor and the corresponding reference rotation.
    ///
    /// The entries of the diagonalized inertia tensor.
    PxMassProperties_getMassSpaceInertia :: proc(inertia: ^PxMat33, massFrame: ^PxQuat) -> PxVec3 ---

    /// Translate an inertia tensor using the parallel axis theorem
    ///
    /// The translated inertia tensor.
    PxMassProperties_translateInertia :: proc(inertia: ^PxMat33, mass: _c.float, t: ^PxVec3) -> PxMat33 ---

    /// Rotate an inertia tensor around the center of mass
    ///
    /// The rotated inertia tensor.
    PxMassProperties_rotateInertia :: proc(inertia: ^PxMat33, q: ^PxQuat) -> PxMat33 ---

    /// Non-uniform scaling of the inertia tensor
    ///
    /// The scaled inertia tensor.
    PxMassProperties_scaleInertia :: proc(inertia: ^PxMat33, scaleRotation: ^PxQuat, scale: ^PxVec3) -> PxMat33 ---

    /// Sum up individual mass properties.
    ///
    /// The summed up mass properties.
    PxMassProperties_sum :: proc(props: ^PxMassProperties, transforms: ^PxTransform, count: _c.uint32_t) -> PxMassProperties ---

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
    PxRigidBodyExt_updateMassAndInertia :: proc(body: ^PxRigidBody, shapeDensities: ^_c.float, shapeDensityCount: _c.uint32_t, massLocalPose: ^PxVec3, includeNonSimShapes: _c.bool) -> _c.bool ---

    /// Computation of mass properties for a rigid body actor
    ///
    /// See previous method for details.
    ///
    /// Boolean. True on success else false.
    PxRigidBodyExt_updateMassAndInertia_1 :: proc(body: ^PxRigidBody, density: _c.float, massLocalPose: ^PxVec3, includeNonSimShapes: _c.bool) -> _c.bool ---

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
    PxRigidBodyExt_setMassAndUpdateInertia :: proc(body: ^PxRigidBody, shapeMasses: ^_c.float, shapeMassCount: _c.uint32_t, massLocalPose: ^PxVec3, includeNonSimShapes: _c.bool) -> _c.bool ---

    /// Computation of mass properties for a rigid body actor
    ///
    /// This method sets the mass, inertia and center of mass of a rigid body. The mass is set to the user-supplied
    /// value, and the inertia and center of mass are computed according to the rigid body's shapes and the input mass.
    ///
    /// If no collision shapes are found, the inertia tensor is set to (1,1,1)
    ///
    /// Boolean. True on success else false.
    PxRigidBodyExt_setMassAndUpdateInertia_1 :: proc(body: ^PxRigidBody, mass: _c.float, massLocalPose: ^PxVec3, includeNonSimShapes: _c.bool) -> _c.bool ---

    /// Compute the mass, inertia tensor and center of mass from a list of shapes.
    ///
    /// The mass properties from the combined shapes.
    PxRigidBodyExt_computeMassPropertiesFromShapes :: proc(shapes: ^^PxShape, shapeCount: _c.uint32_t) -> PxMassProperties ---

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
    PxRigidBodyExt_addForceAtPos :: proc(body: ^PxRigidBody, force: ^PxVec3, pos: ^PxVec3, mode: _c.int32_t, wakeup: _c.bool) ---

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
    PxRigidBodyExt_addForceAtLocalPos :: proc(body: ^PxRigidBody, force: ^PxVec3, pos: ^PxVec3, mode: _c.int32_t, wakeup: _c.bool) ---

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
    PxRigidBodyExt_addLocalForceAtPos :: proc(body: ^PxRigidBody, force: ^PxVec3, pos: ^PxVec3, mode: _c.int32_t, wakeup: _c.bool) ---

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
    PxRigidBodyExt_addLocalForceAtLocalPos :: proc(body: ^PxRigidBody, force: ^PxVec3, pos: ^PxVec3, mode: _c.int32_t, wakeup: _c.bool) ---

    /// Computes the velocity of a point given in world coordinates if it were attached to the
    /// specified body and moving with it.
    ///
    /// The velocity of point in the global frame.
    PxRigidBodyExt_getVelocityAtPos :: proc(body: ^PxRigidBody, pos: ^PxVec3) -> PxVec3 ---

    /// Computes the velocity of a point given in local coordinates if it were attached to the
    /// specified body and moving with it.
    ///
    /// The velocity of point in the local frame.
    PxRigidBodyExt_getLocalVelocityAtLocalPos :: proc(body: ^PxRigidBody, pos: ^PxVec3) -> PxVec3 ---

    /// Computes the velocity of a point (offset from the origin of the body) given in world coordinates if it were attached to the
    /// specified body and moving with it.
    ///
    /// The velocity of point (offset from the origin of the body) in the global frame.
    PxRigidBodyExt_getVelocityAtOffset :: proc(body: ^PxRigidBody, pos: ^PxVec3) -> PxVec3 ---

    /// Compute the change to linear and angular velocity that would occur if an impulsive force and torque were to be applied to a specified rigid body.
    ///
    /// The rigid body is left unaffected unless a subsequent independent call is executed that actually applies the computed changes to velocity and angular velocity.
    ///
    /// if this call is used to determine the velocity delta for an articulation link, only the mass properties of the link are taken into account.
    PxRigidBodyExt_computeVelocityDeltaFromImpulse :: proc(body: ^PxRigidBody, impulsiveForce: ^PxVec3, impulsiveTorque: ^PxVec3, deltaLinearVelocity: ^PxVec3, deltaAngularVelocity: ^PxVec3) ---

    /// Computes the linear and angular velocity change vectors for a given impulse at a world space position taking a mass and inertia scale into account
    ///
    /// This function is useful for extracting the respective linear and angular velocity changes from a contact or joint when the mass/inertia ratios have been adjusted.
    ///
    /// if this call is used to determine the velocity delta for an articulation link, only the mass properties of the link are taken into account.
    PxRigidBodyExt_computeVelocityDeltaFromImpulse_1 :: proc(body: ^PxRigidBody, globalPose: ^PxTransform, point: ^PxVec3, impulse: ^PxVec3, invMassScale: _c.float, invInertiaScale: _c.float, deltaLinearVelocity: ^PxVec3, deltaAngularVelocity: ^PxVec3) ---

    /// Computes the linear and angular impulse vectors for a given impulse at a world space position taking a mass and inertia scale into account
    ///
    /// This function is useful for extracting the respective linear and angular impulses from a contact or joint when the mass/inertia ratios have been adjusted.
    PxRigidBodyExt_computeLinearAngularImpulse :: proc(body: ^PxRigidBody, globalPose: ^PxTransform, point: ^PxVec3, impulse: ^PxVec3, invMassScale: _c.float, invInertiaScale: _c.float, linearImpulse: ^PxVec3, angularImpulse: ^PxVec3) ---

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
    PxRigidBodyExt_linearSweepSingle :: proc(body: ^PxRigidBody, scene: ^PxScene, unitDir: ^PxVec3, distance: _c.float, outputFlags: _c.uint16_t, closestHit: ^PxSweepHit, shapeIndex: ^_c.uint32_t, filterData: ^PxQueryFilterData, filterCall: ^PxQueryFilterCallback, cache: ^PxQueryCache, inflation: _c.float) -> _c.bool ---

    /// Performs a linear sweep through space with the body's geometry objects, returning all overlaps.
    ///
    /// Supported geometries are: box, sphere, capsule, convex. Other geometry types will be ignored.
    ///
    /// This function sweeps all shapes attached to a given rigid body through space and reports all
    /// objects in the scene that intersect any of the shapes' swept paths until there are no more objects to report
    /// or a blocking hit is encountered.
    ///
    /// the number of touching hits. If overflow is set to true, the results are incomplete. In case of overflow there are also no guarantees that all touching hits returned are closer than the blocking hit.
    PxRigidBodyExt_linearSweepMultiple :: proc(body: ^PxRigidBody, scene: ^PxScene, unitDir: ^PxVec3, distance: _c.float, outputFlags: _c.uint16_t, touchHitBuffer: ^PxSweepHit, touchHitShapeIndices: ^_c.uint32_t, touchHitBufferSize: _c.uint32_t, block: ^PxSweepHit, blockingShapeIndex: ^_c.int32_t, overflow: ^_c.bool, filterData: ^PxQueryFilterData, filterCall: ^PxQueryFilterCallback, cache: ^PxQueryCache, inflation: _c.float) -> _c.uint32_t ---

    /// Retrieves the world space pose of the shape.
    ///
    /// Global pose of shape.
    PxShapeExt_getGlobalPose :: proc(shape: ^PxShape, actor: ^PxRigidActor) -> PxTransform ---

    /// Raycast test against the shape.
    ///
    /// Number of hits between the ray and the shape
    PxShapeExt_raycast :: proc(shape: ^PxShape, actor: ^PxRigidActor, rayOrigin: ^PxVec3, rayDir: ^PxVec3, maxDist: _c.float, hitFlags: _c.uint16_t, maxHits: _c.uint32_t, rayHits: ^PxRaycastHit) -> _c.uint32_t ---

    /// Test overlap between the shape and a geometry object
    ///
    /// True if the shape overlaps the geometry object
    PxShapeExt_overlap :: proc(shape: ^PxShape, actor: ^PxRigidActor, otherGeom: ^PxGeometry, otherGeomPose: ^PxTransform) -> _c.bool ---

    /// Sweep a geometry object against the shape.
    ///
    /// Currently only box, sphere, capsule and convex mesh shapes are supported, i.e. the swept geometry object must be one of those types.
    ///
    /// True if the swept geometry object hits the shape
    PxShapeExt_sweep :: proc(shape: ^PxShape, actor: ^PxRigidActor, unitDir: ^PxVec3, distance: _c.float, otherGeom: ^PxGeometry, otherGeomPose: ^PxTransform, sweepHit: ^PxSweepHit, hitFlags: _c.uint16_t) -> _c.bool ---

    /// Retrieves the axis aligned bounding box enclosing the shape.
    ///
    /// The shape's bounding box.
    PxShapeExt_getWorldBounds :: proc(shape: ^PxShape, actor: ^PxRigidActor, inflation: _c.float) -> PxBounds3 ---

    PxMeshOverlapUtil_new_alloc :: proc() -> ^PxMeshOverlapUtil ---

    PxMeshOverlapUtil_delete :: proc(self_: ^PxMeshOverlapUtil) ---

    /// Find the mesh triangles which touch the specified geometry object.
    ///
    /// Number of overlaps found. Triangle indices can then be accessed through the [`getResults`]() function.
    PxMeshOverlapUtil_findOverlap_mut :: proc(self_: ^PxMeshOverlapUtil, geom: ^PxGeometry, geomPose: ^PxTransform, meshGeom: ^PxTriangleMeshGeometry, meshPose: ^PxTransform) -> _c.uint32_t ---

    /// Find the height field triangles which touch the specified geometry object.
    ///
    /// Number of overlaps found. Triangle indices can then be accessed through the [`getResults`]() function.
    PxMeshOverlapUtil_findOverlap_mut_1 :: proc(self_: ^PxMeshOverlapUtil, geom: ^PxGeometry, geomPose: ^PxTransform, hfGeom: ^PxHeightFieldGeometry, hfPose: ^PxTransform) -> _c.uint32_t ---

    /// Retrieves array of triangle indices after a findOverlap call.
    ///
    /// Indices of touched triangles
    PxMeshOverlapUtil_getResults :: proc(self_: ^PxMeshOverlapUtil) -> ^_c.uint32_t ---

    /// Retrieves number of triangle indices after a findOverlap call.
    ///
    /// Number of touched triangles
    PxMeshOverlapUtil_getNbResults :: proc(self_: ^PxMeshOverlapUtil) -> _c.uint32_t ---

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
    phys_PxComputeTriangleMeshPenetration :: proc(direction: ^PxVec3, depth: ^_c.float, geom: ^PxGeometry, geomPose: ^PxTransform, meshGeom: ^PxTriangleMeshGeometry, meshPose: ^PxTransform, maxIter: _c.uint32_t, usedIter: ^_c.uint32_t) -> _c.bool ---

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
    phys_PxComputeHeightFieldPenetration :: proc(direction: ^PxVec3, depth: ^_c.float, geom: ^PxGeometry, geomPose: ^PxTransform, heightFieldGeom: ^PxHeightFieldGeometry, heightFieldPose: ^PxTransform, maxIter: _c.uint32_t, usedIter: ^_c.uint32_t) -> _c.bool ---

    PxXmlMiscParameter_new :: proc() -> PxXmlMiscParameter ---

    PxXmlMiscParameter_new_1 :: proc(inUpVector: ^PxVec3, inScale: PxTolerancesScale) -> PxXmlMiscParameter ---

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
    PxSerialization_isSerializable :: proc(collection: ^PxCollection, sr: ^PxSerializationRegistry, externalReferences: ^PxCollection) -> _c.bool ---

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
    PxSerialization_complete :: proc(collection: ^PxCollection, sr: ^PxSerializationRegistry, exceptFor: ^PxCollection, followJoints: _c.bool) ---

    /// Creates PxSerialObjectId values for unnamed objects in a collection.
    ///
    /// Creates PxSerialObjectId names for unnamed objects in a collection starting at a base value and incrementing,
    /// skipping values that are already assigned to objects in the collection.
    PxSerialization_createSerialObjectIds :: proc(collection: ^PxCollection, base: _c.uint64_t) ---

    /// Creates a PxCollection from XML data.
    ///
    /// a pointer to a PxCollection if successful or NULL if it failed.
    PxSerialization_createCollectionFromXml :: proc(inputData: ^PxInputData, cooking: ^PxCooking, sr: ^PxSerializationRegistry, externalRefs: ^PxCollection, stringTable: ^PxStringTable, outArgs: ^PxXmlMiscParameter) -> ^PxCollection ---

    /// Deserializes a PxCollection from memory.
    ///
    /// Creates a collection from memory. If the collection has external dependencies another collection
    /// can be provided to resolve these.
    ///
    /// The memory block provided has to be 128 bytes aligned and contain a contiguous serialized collection as written
    /// by PxSerialization::serializeCollectionToBinary. The contained binary data needs to be compatible with the current binary format version
    /// which is defined by "PX_PHYSICS_VERSION_MAJOR.PX_PHYSICS_VERSION_MINOR.PX_PHYSICS_VERSION_BUGFIX-PX_BINARY_SERIAL_VERSION".
    /// For a list of compatible sdk releases refer to the documentation of PX_BINARY_SERIAL_VERSION.
    PxSerialization_createCollectionFromBinary :: proc(memBlock: rawptr, sr: ^PxSerializationRegistry, externalRefs: ^PxCollection) -> ^PxCollection ---

    /// Serializes a physics collection to an XML output stream.
    ///
    /// The collection to be serialized needs to be complete
    ///
    /// Serialization of objects in a scene that is simultaneously being simulated is not supported and leads to undefined behavior.
    ///
    /// true if the collection is successfully serialized.
    PxSerialization_serializeCollectionToXml :: proc(outputStream: ^PxOutputStream, collection: ^PxCollection, sr: ^PxSerializationRegistry, cooking: ^PxCooking, externalRefs: ^PxCollection, inArgs: ^PxXmlMiscParameter) -> _c.bool ---

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
    PxSerialization_serializeCollectionToBinary :: proc(outputStream: ^PxOutputStream, collection: ^PxCollection, sr: ^PxSerializationRegistry, externalRefs: ^PxCollection, exportNames: _c.bool) -> _c.bool ---

    /// Creates an application managed registry for serialization.
    ///
    /// PxSerializationRegistry instance.
    PxSerialization_createSerializationRegistry :: proc(physics: ^PxPhysics) -> ^PxSerializationRegistry ---

    /// Deletes the dispatcher.
    ///
    /// Do not keep a reference to the deleted instance.
    PxDefaultCpuDispatcher_release_mut :: proc(self_: ^PxDefaultCpuDispatcher) ---

    /// Enables profiling at task level.
    ///
    /// By default enabled only in profiling builds.
    PxDefaultCpuDispatcher_setRunProfiled_mut :: proc(self_: ^PxDefaultCpuDispatcher, runProfiled: _c.bool) ---

    /// Checks if profiling is enabled at task level.
    ///
    /// True if tasks should be profiled.
    PxDefaultCpuDispatcher_getRunProfiled :: proc(self_: ^PxDefaultCpuDispatcher) -> _c.bool ---

    /// Create default dispatcher, extensions SDK needs to be initialized first.
    ///
    /// numThreads may be zero in which case no worker thread are initialized and
    /// simulation tasks will be executed on the thread that calls PxScene::simulate()
    ///
    /// yieldProcessorCount must be greater than zero if eYIELD_PROCESSOR is the chosen mode and equal to zero for all other modes.
    ///
    /// eYIELD_THREAD and eYIELD_PROCESSOR modes will use compute resources even if the simulation is not running.
    /// It is left to users to keep threads inactive, if so desired, when no simulation is running.
    phys_PxDefaultCpuDispatcherCreate :: proc(numThreads: _c.uint32_t, affinityMasks: ^_c.uint32_t, mode: _c.int32_t, yieldProcessorCount: _c.uint32_t) -> ^PxDefaultCpuDispatcher ---

    /// Builds smooth vertex normals over a mesh.
    ///
    /// - "smooth" because smoothing groups are not supported here
    /// - takes angles into account for correct cube normals computation
    ///
    /// To use 32bit indices pass a pointer in dFaces and set wFaces to zero. Alternatively pass a pointer to
    /// wFaces and set dFaces to zero.
    ///
    /// True on success.
    phys_PxBuildSmoothNormals :: proc(nbTris: _c.uint32_t, nbVerts: _c.uint32_t, verts: ^PxVec3, dFaces: ^_c.uint32_t, wFaces: ^_c.uint16_t, normals: ^PxVec3, flip: _c.bool) -> _c.bool ---

    /// simple method to create a PxRigidDynamic actor with a single PxShape.
    ///
    /// a new dynamic actor with the PxRigidBodyFlag, or NULL if it could
    /// not be constructed
    phys_PxCreateDynamic :: proc(sdk: ^PxPhysics, transform: ^PxTransform, geometry: ^PxGeometry, material: ^PxMaterial, density: _c.float, shapeOffset: ^PxTransform) -> ^PxRigidDynamic ---

    /// simple method to create a PxRigidDynamic actor with a single PxShape.
    ///
    /// a new dynamic actor with the PxRigidBodyFlag, or NULL if it could
    /// not be constructed
    phys_PxCreateDynamic_1 :: proc(sdk: ^PxPhysics, transform: ^PxTransform, shape: ^PxShape, density: _c.float) -> ^PxRigidDynamic ---

    /// simple method to create a kinematic PxRigidDynamic actor with a single PxShape.
    ///
    /// unlike PxCreateDynamic, the geometry is not restricted to box, capsule, sphere or convex. However,
    /// kinematics of other geometry types may not participate in simulation collision and may be used only for
    /// triggers or scene queries of moving objects under animation control. In this case the density parameter
    /// will be ignored and the created shape will be set up as a scene query only shape (see [`PxShapeFlag::eSCENE_QUERY_SHAPE`])
    ///
    /// a new dynamic actor with the PxRigidBodyFlag::eKINEMATIC set, or NULL if it could
    /// not be constructed
    phys_PxCreateKinematic :: proc(sdk: ^PxPhysics, transform: ^PxTransform, geometry: ^PxGeometry, material: ^PxMaterial, density: _c.float, shapeOffset: ^PxTransform) -> ^PxRigidDynamic ---

    /// simple method to create a kinematic PxRigidDynamic actor with a single PxShape.
    ///
    /// unlike PxCreateDynamic, the geometry is not restricted to box, capsule, sphere or convex. However,
    /// kinematics of other geometry types may not participate in simulation collision and may be used only for
    /// triggers or scene queries of moving objects under animation control. In this case the density parameter
    /// will be ignored and the created shape will be set up as a scene query only shape (see [`PxShapeFlag::eSCENE_QUERY_SHAPE`])
    ///
    /// a new dynamic actor with the PxRigidBodyFlag::eKINEMATIC set, or NULL if it could
    /// not be constructed
    phys_PxCreateKinematic_1 :: proc(sdk: ^PxPhysics, transform: ^PxTransform, shape: ^PxShape, density: _c.float) -> ^PxRigidDynamic ---

    /// simple method to create a PxRigidStatic actor with a single PxShape.
    ///
    /// a new static actor, or NULL if it could not be constructed
    phys_PxCreateStatic :: proc(sdk: ^PxPhysics, transform: ^PxTransform, geometry: ^PxGeometry, material: ^PxMaterial, shapeOffset: ^PxTransform) -> ^PxRigidStatic ---

    /// simple method to create a PxRigidStatic actor with a single PxShape.
    ///
    /// a new static actor, or NULL if it could not be constructed
    phys_PxCreateStatic_1 :: proc(sdk: ^PxPhysics, transform: ^PxTransform, shape: ^PxShape) -> ^PxRigidStatic ---

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
    phys_PxCloneShape :: proc(physicsSDK: ^PxPhysics, shape: ^PxShape, isExclusive: _c.bool) -> ^PxShape ---

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
    phys_PxCloneStatic :: proc(physicsSDK: ^PxPhysics, transform: ^PxTransform, actor: ^PxRigidActor) -> ^PxRigidStatic ---

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
    phys_PxCloneDynamic :: proc(physicsSDK: ^PxPhysics, transform: ^PxTransform, body: ^PxRigidDynamic) -> ^PxRigidDynamic ---

    /// create a plane actor. The plane equation is n.x + d = 0
    ///
    /// a new static actor, or NULL if it could not be constructed
    phys_PxCreatePlane :: proc(sdk: ^PxPhysics, plane: ^PxPlane, material: ^PxMaterial) -> ^PxRigidStatic ---

    /// scale a rigid actor by a uniform scale
    ///
    /// The geometry and relative positions of the actor are multiplied by the given scale value. If the actor is a rigid body or an
    /// articulation link and the scaleMassProps value is true, the mass properties are scaled assuming the density is constant: the
    /// center of mass is linearly scaled, the mass is multiplied by the cube of the scale, and the inertia tensor by the fifth power of the scale.
    phys_PxScaleRigidActor :: proc(actor: ^PxRigidActor, scale: _c.float, scaleMassProps: _c.bool) ---

    PxStringTableExt_createStringTable :: proc(inAllocator: ^PxAllocatorCallback) -> ^PxStringTable ---

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
    PxBroadPhaseExt_createRegionsFromWorldBounds :: proc(regions: ^PxBounds3, globalBounds: ^PxBounds3, nbSubdiv: _c.uint32_t, upAxis: _c.uint32_t) -> _c.uint32_t ---

    /// Raycast returning any blocking hit, not necessarily the closest.
    ///
    /// Returns whether any rigid actor is hit along the ray.
    ///
    /// Shooting a ray from within an object leads to different results depending on the shape type. Please check the details in article SceneQuery. User can ignore such objects by using one of the provided filter mechanisms.
    ///
    /// True if a blocking hit was found.
    PxSceneQueryExt_raycastAny :: proc(scene: ^PxScene, origin: ^PxVec3, unitDir: ^PxVec3, distance: _c.float, hit: ^PxQueryHit, filterData: ^PxQueryFilterData, filterCall: ^PxQueryFilterCallback, cache: ^PxQueryCache) -> _c.bool ---

    /// Raycast returning a single result.
    ///
    /// Returns the first rigid actor that is hit along the ray. Data for a blocking hit will be returned as specified by the outputFlags field. Touching hits will be ignored.
    ///
    /// Shooting a ray from within an object leads to different results depending on the shape type. Please check the details in article SceneQuery. User can ignore such objects by using one of the provided filter mechanisms.
    ///
    /// True if a blocking hit was found.
    PxSceneQueryExt_raycastSingle :: proc(scene: ^PxScene, origin: ^PxVec3, unitDir: ^PxVec3, distance: _c.float, outputFlags: _c.uint16_t, hit: ^PxRaycastHit, filterData: ^PxQueryFilterData, filterCall: ^PxQueryFilterCallback, cache: ^PxQueryCache) -> _c.bool ---

    /// Raycast returning multiple results.
    ///
    /// Find all rigid actors that get hit along the ray. Each result contains data as specified by the outputFlags field.
    ///
    /// Touching hits are not ordered.
    ///
    /// Shooting a ray from within an object leads to different results depending on the shape type. Please check the details in article SceneQuery. User can ignore such objects by using one of the provided filter mechanisms.
    ///
    /// Number of hits in the buffer, or -1 if the buffer overflowed.
    PxSceneQueryExt_raycastMultiple :: proc(scene: ^PxScene, origin: ^PxVec3, unitDir: ^PxVec3, distance: _c.float, outputFlags: _c.uint16_t, hitBuffer: ^PxRaycastHit, hitBufferSize: _c.uint32_t, blockingHit: ^_c.bool, filterData: ^PxQueryFilterData, filterCall: ^PxQueryFilterCallback, cache: ^PxQueryCache) -> _c.int32_t ---

    /// Sweep returning any blocking hit, not necessarily the closest.
    ///
    /// Returns whether any rigid actor is hit along the sweep path.
    ///
    /// If a shape from the scene is already overlapping with the query shape in its starting position, behavior is controlled by the PxSceneQueryFlag::eINITIAL_OVERLAP flag.
    ///
    /// True if a blocking hit was found.
    PxSceneQueryExt_sweepAny :: proc(scene: ^PxScene, geometry: ^PxGeometry, pose: ^PxTransform, unitDir: ^PxVec3, distance: _c.float, queryFlags: _c.uint16_t, hit: ^PxQueryHit, filterData: ^PxQueryFilterData, filterCall: ^PxQueryFilterCallback, cache: ^PxQueryCache, inflation: _c.float) -> _c.bool ---

    /// Sweep returning a single result.
    ///
    /// Returns the first rigid actor that is hit along the ray. Data for a blocking hit will be returned as specified by the outputFlags field. Touching hits will be ignored.
    ///
    /// If a shape from the scene is already overlapping with the query shape in its starting position, behavior is controlled by the PxSceneQueryFlag::eINITIAL_OVERLAP flag.
    ///
    /// True if a blocking hit was found.
    PxSceneQueryExt_sweepSingle :: proc(scene: ^PxScene, geometry: ^PxGeometry, pose: ^PxTransform, unitDir: ^PxVec3, distance: _c.float, outputFlags: _c.uint16_t, hit: ^PxSweepHit, filterData: ^PxQueryFilterData, filterCall: ^PxQueryFilterCallback, cache: ^PxQueryCache, inflation: _c.float) -> _c.bool ---

    /// Sweep returning multiple results.
    ///
    /// Find all rigid actors that get hit along the sweep. Each result contains data as specified by the outputFlags field.
    ///
    /// Touching hits are not ordered.
    ///
    /// If a shape from the scene is already overlapping with the query shape in its starting position, behavior is controlled by the PxSceneQueryFlag::eINITIAL_OVERLAP flag.
    ///
    /// Number of hits in the buffer, or -1 if the buffer overflowed.
    PxSceneQueryExt_sweepMultiple :: proc(scene: ^PxScene, geometry: ^PxGeometry, pose: ^PxTransform, unitDir: ^PxVec3, distance: _c.float, outputFlags: _c.uint16_t, hitBuffer: ^PxSweepHit, hitBufferSize: _c.uint32_t, blockingHit: ^_c.bool, filterData: ^PxQueryFilterData, filterCall: ^PxQueryFilterCallback, cache: ^PxQueryCache, inflation: _c.float) -> _c.int32_t ---

    /// Test overlap between a geometry and objects in the scene.
    ///
    /// Filtering: Overlap tests do not distinguish between touching and blocking hit types. Both get written to the hit buffer.
    ///
    /// PxHitFlag::eMESH_MULTIPLE and PxHitFlag::eMESH_BOTH_SIDES have no effect in this case
    ///
    /// Number of hits in the buffer, or -1 if the buffer overflowed.
    PxSceneQueryExt_overlapMultiple :: proc(scene: ^PxScene, geometry: ^PxGeometry, pose: ^PxTransform, hitBuffer: ^PxOverlapHit, hitBufferSize: _c.uint32_t, filterData: ^PxQueryFilterData, filterCall: ^PxQueryFilterCallback) -> _c.int32_t ---

    /// Test returning, for a given geometry, any overlapping object in the scene.
    ///
    /// Filtering: Overlap tests do not distinguish between touching and blocking hit types. Both trigger a hit.
    ///
    /// PxHitFlag::eMESH_MULTIPLE and PxHitFlag::eMESH_BOTH_SIDES have no effect in this case
    ///
    /// True if an overlap was found.
    PxSceneQueryExt_overlapAny :: proc(scene: ^PxScene, geometry: ^PxGeometry, pose: ^PxTransform, hit: ^PxOverlapHit, filterData: ^PxQueryFilterData, filterCall: ^PxQueryFilterCallback) -> _c.bool ---

    PxBatchQueryExt_release_mut :: proc(self_: ^PxBatchQueryExt) ---

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
    PxBatchQueryExt_raycast_mut :: proc(self_: ^PxBatchQueryExt, origin: ^PxVec3, unitDir: ^PxVec3, distance: _c.float, maxNbTouches: _c.uint16_t, hitFlags: _c.uint16_t, filterData: ^PxQueryFilterData, cache: ^PxQueryCache) -> ^PxRaycastBuffer ---

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
    PxBatchQueryExt_sweep_mut :: proc(self_: ^PxBatchQueryExt, geometry: ^PxGeometry, pose: ^PxTransform, unitDir: ^PxVec3, distance: _c.float, maxNbTouches: _c.uint16_t, hitFlags: _c.uint16_t, filterData: ^PxQueryFilterData, cache: ^PxQueryCache, inflation: _c.float) -> ^PxSweepBuffer ---

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
    PxBatchQueryExt_overlap_mut :: proc(self_: ^PxBatchQueryExt, geometry: ^PxGeometry, pose: ^PxTransform, maxNbTouches: _c.uint16_t, filterData: ^PxQueryFilterData, cache: ^PxQueryCache) -> ^PxOverlapBuffer ---

    PxBatchQueryExt_execute_mut :: proc(self_: ^PxBatchQueryExt) ---

    /// Create a PxBatchQueryExt without the need for pre-allocated result or touch buffers.
    ///
    /// Returns a PxBatchQueryExt instance. A NULL pointer will be returned if the subsequent allocations fail or if any of the arguments are illegal.
    /// In the event that a NULL pointer is returned a corresponding error will be issued to the error stream.
    phys_PxCreateBatchQueryExt :: proc(scene: ^PxScene, queryFilterCallback: ^PxQueryFilterCallback, maxNbRaycasts: _c.uint32_t, maxNbRaycastTouches: _c.uint32_t, maxNbSweeps: _c.uint32_t, maxNbSweepTouches: _c.uint32_t, maxNbOverlaps: _c.uint32_t, maxNbOverlapTouches: _c.uint32_t) -> ^PxBatchQueryExt ---

    /// Create a PxBatchQueryExt with user-supplied result and touch buffers.
    ///
    /// Returns a PxBatchQueryExt instance. A NULL pointer will be returned if the subsequent allocations fail or if any of the arguments are illegal.
    /// In the event that a NULL pointer is returned a corresponding error will be issued to the error stream.
    phys_PxCreateBatchQueryExt_1 :: proc(scene: ^PxScene, queryFilterCallback: ^PxQueryFilterCallback, raycastBuffers: ^PxRaycastBuffer, maxNbRaycasts: _c.uint32_t, raycastTouches: ^PxRaycastHit, maxNbRaycastTouches: _c.uint32_t, sweepBuffers: ^PxSweepBuffer, maxNbSweeps: _c.uint32_t, sweepTouches: ^PxSweepHit, maxNbSweepTouches: _c.uint32_t, overlapBuffers: ^PxOverlapBuffer, maxNbOverlaps: _c.uint32_t, overlapTouches: ^PxOverlapHit, maxNbOverlapTouches: _c.uint32_t) -> ^PxBatchQueryExt ---

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
    phys_PxCreateExternalSceneQuerySystem :: proc(desc: ^PxSceneQueryDesc, contextID: _c.uint64_t) -> ^PxSceneQuerySystem ---

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
    PxCustomSceneQuerySystem_addPruner_mut :: proc(self_: ^PxCustomSceneQuerySystem, primaryType: _c.int32_t, secondaryType: _c.int32_t, preallocated: _c.uint32_t) -> _c.uint32_t ---

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
    PxCustomSceneQuerySystem_startCustomBuildstep_mut :: proc(self_: ^PxCustomSceneQuerySystem) -> _c.uint32_t ---

    /// Perform a custom build-step for a given pruner.
    PxCustomSceneQuerySystem_customBuildstep_mut :: proc(self_: ^PxCustomSceneQuerySystem, index: _c.uint32_t) ---

    /// Finish custom build-steps
    ///
    /// Call this function once after all the customBuildstep() calls are done.
    PxCustomSceneQuerySystem_finishCustomBuildstep_mut :: proc(self_: ^PxCustomSceneQuerySystem) ---

    PxCustomSceneQuerySystemAdapter_delete :: proc(self_: ^PxCustomSceneQuerySystemAdapter) ---

    /// Gets a pruner index for an actor/shape.
    ///
    /// This user-defined function tells the system in which pruner a given actor/shape should go.
    ///
    /// The returned index must be valid, i.e. it must have been previously returned to users by PxCustomSceneQuerySystem::addPruner.
    ///
    /// A pruner index for this actor/shape.
    PxCustomSceneQuerySystemAdapter_getPrunerIndex :: proc(self_: ^PxCustomSceneQuerySystemAdapter, actor: ^PxRigidActor, shape: ^PxShape) -> _c.uint32_t ---

    /// Pruner filtering callback.
    ///
    /// This will be called for each query to validate whether it should process a given pruner.
    ///
    /// True to process the pruner, false to skip it entirely
    PxCustomSceneQuerySystemAdapter_processPruner :: proc(self_: ^PxCustomSceneQuerySystemAdapter, prunerIndex: _c.uint32_t, context_: ^PxQueryThreadContext, filterData: ^PxQueryFilterData, filterCall: ^PxQueryFilterCallback) -> _c.bool ---

    /// Creates a custom scene query system.
    ///
    /// This is similar to PxCreateExternalSceneQuerySystem, except this function creates a PxCustomSceneQuerySystem object.
    /// It can be plugged to PxScene the same way, via PxSceneDesc::sceneQuerySystem.
    ///
    /// A custom SQ system instance
    phys_PxCreateCustomSceneQuerySystem :: proc(sceneQueryUpdateMode: _c.int32_t, contextID: _c.uint64_t, adapter: ^PxCustomSceneQuerySystemAdapter, usesTreeOfPruners: _c.bool) -> ^PxCustomSceneQuerySystem ---

    /// Computes closest polygon of the convex hull geometry for a given impact point
    /// and impact direction. When doing sweeps against a scene, one might want to delay
    /// the rather expensive computation of the hit face index for convexes until it is clear
    /// the information is really needed and then use this method to get the corresponding
    /// face index.
    ///
    /// Closest face index of the convex geometry.
    phys_PxFindFaceIndex :: proc(convexGeom: ^PxConvexMeshGeometry, geomPose: ^PxTransform, impactPos: ^PxVec3, unitDir: ^PxVec3) -> _c.uint32_t ---

    /// Sets the sampling radius
    ///
    /// Returns true if the sampling was successful and false if there was a problem. Usually an internal overflow is the problem for very big meshes or very small sampling radii.
    PxPoissonSampler_setSamplingRadius_mut :: proc(self_: ^PxPoissonSampler, samplingRadius: _c.float) -> _c.bool ---

    /// Adds new Poisson Samples inside the sphere specified
    PxPoissonSampler_addSamplesInSphere_mut :: proc(self_: ^PxPoissonSampler, sphereCenter: ^PxVec3, sphereRadius: _c.float, createVolumeSamples: _c.bool) ---

    /// Adds new Poisson Samples inside the box specified
    PxPoissonSampler_addSamplesInBox_mut :: proc(self_: ^PxPoissonSampler, axisAlignedBox: ^PxBounds3, boxOrientation: ^PxQuat, createVolumeSamples: _c.bool) ---

    PxPoissonSampler_delete :: proc(self_: ^PxPoissonSampler) ---

    /// Creates a shape sampler
    ///
    /// Returns the sampler
    phys_PxCreateShapeSampler :: proc(geometry: ^PxGeometry, transform: ^PxTransform, worldBounds: ^PxBounds3, initialSamplingRadius: _c.float, numSampleAttemptsAroundPoint: _c.int32_t) -> ^PxPoissonSampler ---

    /// Checks whether a point is inside the triangle mesh
    ///
    /// Returns true if the point is inside the triangle mesh
    PxTriangleMeshPoissonSampler_isPointInTriangleMesh_mut :: proc(self_: ^PxTriangleMeshPoissonSampler, p: ^PxVec3) -> _c.bool ---

    PxTriangleMeshPoissonSampler_delete :: proc(self_: ^PxTriangleMeshPoissonSampler) ---

    /// Creates a triangle mesh sampler
    ///
    /// Returns the sampler
    phys_PxCreateTriangleMeshSampler :: proc(triangles: ^_c.uint32_t, numTriangles: _c.uint32_t, vertices: ^PxVec3, numVertices: _c.uint32_t, initialSamplingRadius: _c.float, numSampleAttemptsAroundPoint: _c.int32_t) -> ^PxTriangleMeshPoissonSampler ---

    /// Returns the index of the tetrahedron that contains a point
    ///
    /// The index of the tetrahedon containing the point, -1 if not tetrahedron contains the opoint
    PxTetrahedronMeshExt_findTetrahedronContainingPoint :: proc(mesh: ^PxTetrahedronMesh, point: ^PxVec3, bary: ^PxVec4, tolerance: _c.float) -> _c.int32_t ---

    /// Returns the index of the tetrahedron closest to a point
    ///
    /// The index of the tetrahedon closest to the point
    PxTetrahedronMeshExt_findTetrahedronClosestToPoint :: proc(mesh: ^PxTetrahedronMesh, point: ^PxVec3, bary: ^PxVec4) -> _c.int32_t ---

    /// Initialize the PhysXExtensions library.
    ///
    /// This should be called before calling any functions or methods in extensions which may require allocation.
    ///
    /// This function does not need to be called before creating a PxDefaultAllocator object.
    phys_PxInitExtensions :: proc(physics: ^PxPhysics, pvd: ^PxPvd) -> _c.bool ---

    /// Shut down the PhysXExtensions library.
    ///
    /// This function should be called to cleanly shut down the PhysXExtensions library before application exit.
    ///
    /// This function is required to be called to release foundation usage.
    phys_PxCloseExtensions :: proc() ---

    PxRepXObject_new :: proc(inTypeName: ^_c.char, inSerializable: rawptr, inId: _c.uint64_t) -> PxRepXObject ---

    PxRepXObject_isValid :: proc(self_: ^PxRepXObject) -> _c.bool ---

    PxRepXInstantiationArgs_new :: proc(inPhysics: ^PxPhysics, inCooking: ^PxCooking, inStringTable: ^PxStringTable) -> PxRepXInstantiationArgs ---

    /// The type this Serializer is meant to operate on.
    PxRepXSerializer_getTypeName_mut :: proc(self_: ^PxRepXSerializer) -> ^_c.char ---

    /// Convert from a RepX object to a key-value pair hierarchy
    PxRepXSerializer_objectToFile_mut :: proc(self_: ^PxRepXSerializer, inLiveObject: ^PxRepXObject, inCollection: ^PxCollection, inWriter: ^XmlWriter, inTempBuffer: ^MemoryBuffer, inArgs: ^PxRepXInstantiationArgs) ---

    /// Convert from a descriptor to a live object.  Must be an object of this Serializer type.
    ///
    /// The new live object.  It can be an invalid object if the instantiation cannot take place.
    PxRepXSerializer_fileToObject_mut :: proc(self_: ^PxRepXSerializer, inReader: ^XmlReader, inAllocator: ^XmlMemoryAllocator, inArgs: ^PxRepXInstantiationArgs, inCollection: ^PxCollection) -> PxRepXObject ---

    /// Connects the SDK to the PhysX Visual Debugger application.
    PxPvd_connect_mut :: proc(self_: ^PxPvd, transport: ^PxPvdTransport, flags: _c.uint8_t) -> _c.bool ---

    /// Disconnects the SDK from the PhysX Visual Debugger application.
    /// If we are still connected, this will kill the entire debugger connection.
    PxPvd_disconnect_mut :: proc(self_: ^PxPvd) ---

    /// Return if connection to PVD is created.
    PxPvd_isConnected_mut :: proc(self_: ^PxPvd, useCachedStatus: _c.bool) -> _c.bool ---

    /// returns the PVD data transport
    /// returns NULL if no transport is present.
    PxPvd_getTransport_mut :: proc(self_: ^PxPvd) -> ^PxPvdTransport ---

    /// Retrieves the PVD flags. See PxPvdInstrumentationFlags.
    PxPvd_getInstrumentationFlags_mut :: proc(self_: ^PxPvd) -> _c.uint8_t ---

    /// Releases the pvd instance.
    PxPvd_release_mut :: proc(self_: ^PxPvd) ---

    /// Create a pvd instance.
    phys_PxCreatePvd :: proc(foundation: ^PxFoundation) -> ^PxPvd ---

    /// Connects to the Visual Debugger application.
    /// return True if success
    PxPvdTransport_connect_mut :: proc(self_: ^PxPvdTransport) -> _c.bool ---

    /// Disconnects from the Visual Debugger application.
    /// If we are still connected, this will kill the entire debugger connection.
    PxPvdTransport_disconnect_mut :: proc(self_: ^PxPvdTransport) ---

    /// Return if connection to PVD is created.
    PxPvdTransport_isConnected_mut :: proc(self_: ^PxPvdTransport) -> _c.bool ---

    /// write bytes to the other endpoint of the connection. should lock before witre. If an error occurs
    /// this connection will assume to be dead.
    PxPvdTransport_write_mut :: proc(self_: ^PxPvdTransport, inBytes: ^_c.uint8_t, inLength: _c.uint32_t) -> _c.bool ---

    PxPvdTransport_lock_mut :: proc(self_: ^PxPvdTransport) -> ^PxPvdTransport ---

    PxPvdTransport_unlock_mut :: proc(self_: ^PxPvdTransport) ---

    /// send any data and block until we know it is at least on the wire.
    PxPvdTransport_flush_mut :: proc(self_: ^PxPvdTransport) ---

    /// Return size of written data.
    PxPvdTransport_getWrittenDataSize_mut :: proc(self_: ^PxPvdTransport) -> _c.uint64_t ---

    PxPvdTransport_release_mut :: proc(self_: ^PxPvdTransport) ---

    /// Create a default socket transport.
    phys_PxDefaultPvdSocketTransportCreate :: proc(host: ^_c.char, port: _c.int32_t, timeoutInMilliseconds: _c.uint32_t) -> ^PxPvdTransport ---

    /// Create a default file transport.
    phys_PxDefaultPvdFileTransportCreate :: proc(name: ^_c.char) -> ^PxPvdTransport ---

}
