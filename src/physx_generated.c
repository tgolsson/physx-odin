/// enum for empty constructor tag
typedef enum PxEMPTY: int32_t {
    PxEmpty = 0,
} PxEMPTY;

/// enum for zero constructor tag for vectors and matrices
typedef enum PxZERO: int32_t {
    PxZero = 0,
} PxZERO;

/// enum for identity constructor flag for quaternions, transforms, and matrices
typedef enum PxIDENTITY: int32_t {
    PxIdentity = 0,
} PxIDENTITY;

/// Error codes
///
/// These error codes are passed to [`PxErrorCallback`]
typedef enum PxErrorCode: int32_t {
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
} PxErrorCode;

typedef enum PxThreadPriority: uint32_t {
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
} PxThreadPriority;

/// Default color values used for debug rendering.
typedef enum PxDebugColor: uint32_t {
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
} PxDebugColor;

/// an enumeration of concrete classes inheriting from PxBase
///
/// Enumeration space is reserved for future PhysX core types, PhysXExtensions,
/// PhysXVehicle and Custom application types.
typedef enum PxConcreteType: int32_t {
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
} PxConcreteType;

impl From<uint16_t> for PxConcreteType {
    fn from(val: uint16_t) -> Self {
        #[allow(clippy::match_same_arms)]
        match val {
            0 => Self::Undefined,
            1 => Self::Heightfield,
            2 => Self::ConvexMesh,
            3 => Self::TriangleMeshBvh33,
            4 => Self::TriangleMeshBvh34,
            5 => Self::TetrahedronMesh,
            6 => Self::SoftbodyMesh,
            7 => Self::RigidDynamic,
            8 => Self::RigidStatic,
            9 => Self::Shape,
            10 => Self::Material,
            11 => Self::SoftbodyMaterial,
            12 => Self::ClothMaterial,
            13 => Self::PbdMaterial,
            14 => Self::FlipMaterial,
            15 => Self::MpmMaterial,
            16 => Self::CustomMaterial,
            17 => Self::Constraint,
            18 => Self::Aggregate,
            19 => Self::ArticulationReducedCoordinate,
            20 => Self::ArticulationLink,
            21 => Self::ArticulationJointReducedCoordinate,
            22 => Self::ArticulationSensor,
            23 => Self::ArticulationSpatialTendon,
            24 => Self::ArticulationFixedTendon,
            25 => Self::ArticulationAttachment,
            26 => Self::ArticulationTendonJoint,
            27 => Self::PruningStructure,
            28 => Self::Bvh,
            29 => Self::SoftBody,
            30 => Self::SoftBodyState,
            31 => Self::PbdParticlesystem,
            32 => Self::FlipParticlesystem,
            33 => Self::MpmParticlesystem,
            34 => Self::CustomParticlesystem,
            35 => Self::FemCloth,
            36 => Self::HairSystem,
            37 => Self::ParticleBuffer,
            38 => Self::ParticleDiffuseBuffer,
            39 => Self::ParticleClothBuffer,
            40 => Self::ParticleRigidBuffer,
            41 => Self::PhysxCoreCount,
            256 => Self::FirstPhysxExtension,
            512 => Self::FirstVehicleExtension,
            1024 => Self::FirstUserExtension,
            _ => Self::Undefined,
        }
    }
}

/// Flags for PxBase.
typedef enum PxBaseFlag: int32_t {
    OwnsMemory = 1,
    IsReleasable = 2,
} PxBaseFlag;

/// Flags for [`PxBaseFlag`]
typedef enum PxBaseFlags: uint16_t {
    OwnsMemory = 1 << 0,
    IsReleasable = 1 << 1,
} PxBaseFlags;


/// Flags used to configure binary meta data entries, typically set through PX_DEF_BIN_METADATA defines.
typedef enum PxMetaDataFlag: int32_t {
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
} PxMetaDataFlag;

/// Identifies the type of each heavyweight PxTask object
typedef enum PxTaskType: int32_t {
    /// PxTask will be run on the CPU
    Cpu = 0,
    /// Return code when attempting to find a task that does not exist
    NotPresent = 1,
    /// PxTask execution has been completed
    Completed = 2,
} PxTaskType;

/// A geometry type.
///
/// Used to distinguish the type of a ::PxGeometry object.
typedef enum PxGeometryType: int32_t {
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
} PxGeometryType;

/// Geometry-level query flags.
typedef enum PxGeometryQueryFlag: int32_t {
    /// Saves/restores SIMD control word for each query (safer but slower). Omit this if you took care of it yourself in your app.
    SimdGuard = 1,
} PxGeometryQueryFlag;

/// Flags for [`PxGeometryQueryFlag`]
typedef enum PxGeometryQueryFlags: uint32_t {
    SimdGuard = 1 << 0,
} PxGeometryQueryFlags;


/// Desired build strategy for bounding-volume hierarchies
typedef enum PxBVHBuildStrategy: int32_t {
    /// Fast build strategy. Fast build speed, good runtime performance in most cases. Recommended for runtime cooking.
    Fast = 0,
    /// Default build strategy. Medium build speed, good runtime performance in all cases.
    Default = 1,
    /// SAH build strategy. Slower builds, slightly improved runtime performance in some cases.
    Sah = 2,
    Last = 3,
} PxBVHBuildStrategy;

/// Flags controlling the simulated behavior of the convex mesh geometry.
///
/// Used in ::PxConvexMeshGeometryFlags.
typedef enum PxConvexMeshGeometryFlag: int32_t {
    /// Use tighter (but more expensive to compute) bounds around the convex geometry.
    TightBounds = 1,
} PxConvexMeshGeometryFlag;

/// Flags for [`PxConvexMeshGeometryFlag`]
typedef enum PxConvexMeshGeometryFlags: uint8_t {
    TightBounds = 1 << 0,
} PxConvexMeshGeometryFlags;


/// Flags controlling the simulated behavior of the triangle mesh geometry.
///
/// Used in ::PxMeshGeometryFlags.
typedef enum PxMeshGeometryFlag: int32_t {
    /// Use tighter (but more expensive to compute) bounds around the triangle mesh geometry.
    TightBounds = 1,
    /// Meshes with this flag set are treated as double-sided.
    /// This flag is currently only used for raycasts and sweeps (it is ignored for overlap queries).
    /// For detailed specifications of this flag for meshes and heightfields please refer to the Geometry Query section of the user guide.
    DoubleSided = 2,
} PxMeshGeometryFlag;

/// Flags for [`PxMeshGeometryFlag`]
typedef enum PxMeshGeometryFlags: uint8_t {
    TightBounds = 1 << 0,
    DoubleSided = 1 << 1,
} PxMeshGeometryFlags;


/// Identifies the solver to use for a particle system.
typedef enum PxParticleSolverType: int32_t {
    /// The position based dynamics solver that can handle fluid, granular material, cloth, inflatables etc. See [`PxPBDParticleSystem`].
    Pbd = 1,
    /// The FLIP fluid solver. See [`PxFLIPParticleSystem`].
    Flip = 2,
    /// The MPM (material point method) solver that can handle a variety of materials. See [`PxMPMParticleSystem`].
    Mpm = 4,
    /// Custom solver. The user needs to specify the interaction of the particle by providing appropriate functions. Can be used e.g. for molecular dynamics simulations. See [`PxCustomParticleSystem`].
    Custom = 8,
} PxParticleSolverType;

/// Scene query and geometry query behavior flags.
///
/// PxHitFlags are used for 3 different purposes:
///
/// 1) To request hit fields to be filled in by scene queries (such as hit position, normal, face index or UVs).
/// 2) Once query is completed, to indicate which fields are valid (note that a query may produce more valid fields than requested).
/// 3) To specify additional options for the narrow phase and mid-phase intersection routines.
///
/// All these flags apply to both scene queries and geometry queries (PxGeometryQuery).
typedef enum PxHitFlag: int32_t {
    /// "position" member of [`PxQueryHit`] is valid
    Position = 1,
    /// "normal" member of [`PxQueryHit`] is valid
    Normal = 2,
    /// "u" and "v" barycentric coordinates of [`PxQueryHit`] are valid. Not applicable to sweep queries.
    Uv = 8,
    /// Performance hint flag for sweeps when it is known upfront there's no initial overlap.
    /// NOTE: using this flag may cause undefined results if shapes are initially overlapping.
    AssumeNoInitialOverlap = 16,
    /// Report any first hit. Used for geometries that contain more than one primitive. For meshes,
    /// if neither eMESH_MULTIPLE nor eANY_HIT is specified, a single closest hit will be reported.
    AnyHit = 32,
    /// Report all hits for meshes rather than just the first. Not applicable to sweep queries.
    MeshMultiple = 64,
    /// Report hits with back faces of mesh triangles. Also report hits for raycast
    /// originating on mesh surface and facing away from the surface normal. Not applicable to sweep queries.
    /// Please refer to the user guide for heightfield-specific differences.
    MeshBothSides = 128,
    /// Use more accurate but slower narrow phase sweep tests.
    /// May provide better compatibility with PhysX 3.2 sweep behavior.
    PreciseSweep = 256,
    /// Report the minimum translation depth, normal and contact point.
    Mtd = 512,
    /// "face index" member of [`PxQueryHit`] is valid
    FaceIndex = 1024,
    Default = 1027,
    /// Only this subset of flags can be modified by pre-filter. Other modifications will be discarded.
    ModifiableFlags = 464,
} PxHitFlag;

/// Flags for [`PxHitFlag`]
typedef enum PxHitFlags: uint16_t {
    Position = 1 << 0,
    Normal = 1 << 1,
    Uv = 1 << 3,
    AssumeNoInitialOverlap = 1 << 4,
    AnyHit = 1 << 5,
    MeshMultiple = 1 << 6,
    MeshBothSides = 1 << 7,
    PreciseSweep = 1 << 8,
    Mtd = 1 << 9,
    FaceIndex = 1 << 10,
    Default = Position | Normal | FaceIndex,
    ModifiableFlags = AssumeNoInitialOverlap | MeshMultiple | MeshBothSides | PreciseSweep,
} PxHitFlags;


/// Describes the format of height field samples.
typedef enum PxHeightFieldFormat: int32_t {
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
} PxHeightFieldFormat;

/// Determines the tessellation of height field cells.
typedef enum PxHeightFieldTessFlag: int32_t {
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
} PxHeightFieldTessFlag;

/// Enum with flag values to be used in PxHeightFieldDesc.flags.
typedef enum PxHeightFieldFlag: int32_t {
    /// Disable collisions with height field with boundary edges.
    ///
    /// Raise this flag if several terrain patches are going to be placed adjacent to each other,
    /// to avoid a bump when sliding across.
    ///
    /// This flag is ignored in contact generation with sphere and capsule shapes.
    NoBoundaryEdges = 1,
} PxHeightFieldFlag;

/// Flags for [`PxHeightFieldFlag`]
typedef enum PxHeightFieldFlags: uint16_t {
    NoBoundaryEdges = 1 << 0,
} PxHeightFieldFlags;


/// Special material index values for height field samples.
typedef enum PxHeightFieldMaterial: int32_t {
    /// A material indicating that the triangle should be treated as a hole in the mesh.
    Hole = 127,
} PxHeightFieldMaterial;

typedef enum PxMeshMeshQueryFlag: int32_t {
    /// Report all overlaps
    Default = 0,
    /// Ignore coplanar triangle-triangle overlaps
    DiscardCoplanar = 1,
} PxMeshMeshQueryFlag;

/// Flags for [`PxMeshMeshQueryFlag`]
typedef enum PxMeshMeshQueryFlags: uint32_t {
    Default = 0 << 0,
    DiscardCoplanar = 1 << 0,
} PxMeshMeshQueryFlags;


/// Enum with flag values to be used in PxSimpleTriangleMesh::flags.
typedef enum PxMeshFlag: int32_t {
    /// Specifies if the SDK should flip normals.
    ///
    /// The PhysX libraries assume that the face normal of a triangle with vertices [a,b,c] can be computed as:
    /// edge1 = b-a
    /// edge2 = c-a
    /// face_normal = edge1 x edge2.
    ///
    /// Note: This is the same as a counterclockwise winding in a right handed coordinate system or
    /// alternatively a clockwise winding order in a left handed coordinate system.
    ///
    /// If this does not match the winding order for your triangles, raise the below flag.
    Flipnormals = 1,
    /// Denotes the use of 16-bit vertex indices
    E16BitIndices = 2,
} PxMeshFlag;

/// Flags for [`PxMeshFlag`]
typedef enum PxMeshFlags: uint16_t {
    Flipnormals = 1 << 0,
    E16BitIndices = 1 << 1,
} PxMeshFlags;


/// Mesh midphase structure. This enum is used to select the desired acceleration structure for midphase queries
/// (i.e. raycasts, overlaps, sweeps vs triangle meshes).
///
/// The PxMeshMidPhase::eBVH33 structure is the one used in recent PhysX versions (up to PhysX 3.3). It has great performance and is
/// supported on all platforms. It is deprecated since PhysX 5.x.
///
/// The PxMeshMidPhase::eBVH34 structure is a revisited implementation introduced in PhysX 3.4. It can be significantly faster both
/// in terms of cooking performance and runtime performance.
typedef enum PxMeshMidPhase: int32_t {
    /// Default midphase mesh structure, as used up to PhysX 3.3 (deprecated)
    Bvh33 = 0,
    /// New midphase mesh structure, introduced in PhysX 3.4
    Bvh34 = 1,
    Last = 2,
} PxMeshMidPhase;

/// Flags for the mesh geometry properties.
///
/// Used in ::PxTriangleMeshFlags.
typedef enum PxTriangleMeshFlag: int32_t {
    /// The triangle mesh has 16bits vertex indices.
    E16BitIndices = 2,
    /// The triangle mesh has adjacency information build.
    AdjacencyInfo = 4,
    /// Indicates that this mesh would preferably not be the mesh projected for mesh-mesh collision. This can indicate that the mesh is not well tessellated.
    PreferNoSdfProj = 8,
} PxTriangleMeshFlag;

/// Flags for [`PxTriangleMeshFlag`]
typedef enum PxTriangleMeshFlags: uint8_t {
    E16BitIndices = 1 << 1,
    AdjacencyInfo = 1 << 2,
    PreferNoSdfProj = 1 << 3,
} PxTriangleMeshFlags;


typedef enum PxTetrahedronMeshFlag: int32_t {
    /// The tetrahedron mesh has 16bits vertex indices
    E16BitIndices = 2,
} PxTetrahedronMeshFlag;

/// Flags for [`PxTetrahedronMeshFlag`]
typedef enum PxTetrahedronMeshFlags: uint8_t {
    E16BitIndices = 1 << 1,
} PxTetrahedronMeshFlags;


/// Flags which control the behavior of an actor.
typedef enum PxActorFlag: int32_t {
    /// Enable debug renderer for this actor
    Visualization = 1,
    /// Disables scene gravity for this actor
    DisableGravity = 2,
    /// Enables the sending of PxSimulationEventCallback::onWake() and PxSimulationEventCallback::onSleep() notify events
    SendSleepNotifies = 4,
    /// Disables simulation for the actor.
    ///
    /// This is only supported by PxRigidStatic and PxRigidDynamic actors and can be used to reduce the memory footprint when rigid actors are
    /// used for scene queries only.
    ///
    /// Setting this flag will remove all constraints attached to the actor from the scene.
    ///
    /// If this flag is set, the following calls are forbidden:
    ///
    /// PxRigidBody: setLinearVelocity(), setAngularVelocity(), addForce(), addTorque(), clearForce(), clearTorque(), setForceAndTorque()
    ///
    /// PxRigidDynamic: setKinematicTarget(), setWakeCounter(), wakeUp(), putToSleep()
    ///
    /// Sleeping:
    /// Raising this flag will set all velocities and the wake counter to 0, clear all forces, clear the kinematic target, put the actor
    /// to sleep and wake up all touching actors from the previous frame.
    DisableSimulation = 8,
} PxActorFlag;

/// Flags for [`PxActorFlag`]
typedef enum PxActorFlags: uint8_t {
    Visualization = 1 << 0,
    DisableGravity = 1 << 1,
    SendSleepNotifies = 1 << 2,
    DisableSimulation = 1 << 3,
} PxActorFlags;


/// Identifies each type of actor.
typedef enum PxActorType: int32_t {
    /// A static rigid body
    RigidStatic = 0,
    /// A dynamic rigid body
    RigidDynamic = 1,
    /// An articulation link
    ArticulationLink = 2,
} PxActorType;

typedef enum PxAggregateType: int32_t {
    /// Aggregate will contain various actors of unspecified types
    Generic = 0,
    /// Aggregate will only contain static actors
    Static = 1,
    /// Aggregate will only contain kinematic actors
    Kinematic = 2,
} PxAggregateType;

/// Constraint row flags
///
/// These flags configure the post-processing of constraint rows and the behavior of the solver while solving constraints
typedef enum Px1DConstraintFlag: int32_t {
    /// whether the constraint is a spring. Mutually exclusive with eRESTITUTION. If set, eKEEPBIAS is ignored.
    Spring = 1,
    /// whether the constraint is a force or acceleration spring. Only valid if eSPRING is set.
    AccelerationSpring = 2,
    /// whether the restitution model should be applied to generate the target velocity. Mutually exclusive with eSPRING. If restitution causes a bounces, eKEEPBIAS is ignored
    Restitution = 4,
    /// whether to keep the error term when solving for velocity. Ignored if restitution generates bounce, or eSPRING is set.
    Keepbias = 8,
    /// whether to accumulate the force value from this constraint in the force total that is reported for the constraint and tested for breakage
    OutputForce = 16,
    /// whether the constraint has a drive force limit (which will be scaled by dt unless PxConstraintFlag::eLIMITS_ARE_FORCES is set)
    HasDriveLimit = 32,
    /// whether this is an angular or linear constraint
    AngularConstraint = 64,
    /// whether the constraint's geometric error should drive the target velocity
    DriveRow = 128,
} Px1DConstraintFlag;

/// Flags for [`Px1DConstraintFlag`]
typedef enum Px1DConstraintFlags: uint16_t {
    Spring = 1 << 0,
    AccelerationSpring = 1 << 1,
    Restitution = 1 << 2,
    Keepbias = 1 << 3,
    OutputForce = 1 << 4,
    HasDriveLimit = 1 << 5,
    AngularConstraint = 1 << 6,
    DriveRow = 1 << 7,
} Px1DConstraintFlags;


/// Constraint type hints which the solver uses to optimize constraint handling
typedef enum PxConstraintSolveHint: int32_t {
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
} PxConstraintSolveHint;

/// Flags for determining which components of the constraint should be visualized.
typedef enum PxConstraintVisualizationFlag: int32_t {
    /// visualize constraint frames
    LocalFrames = 1,
    /// visualize constraint limits
    Limits = 2,
} PxConstraintVisualizationFlag;

/// Flags for determining how PVD should serialize a constraint update
typedef enum PxPvdUpdateType: int32_t {
    /// triggers createPvdInstance call, creates an instance of a constraint
    CreateInstance = 0,
    /// triggers releasePvdInstance call, releases an instance of a constraint
    ReleaseInstance = 1,
    /// triggers updatePvdProperties call, updates all properties of a constraint
    UpdateAllProperties = 2,
    /// triggers simUpdate call, updates all simulation properties of a constraint
    UpdateSimProperties = 3,
} PxPvdUpdateType;

/// Constraint descriptor used inside the solver
typedef enum ConstraintType: int32_t {
    /// Defines this pair is a contact constraint
    ContactConstraint = 0,
    /// Defines this pair is a joint constraint
    JointConstraint = 1,
} ConstraintType;

/// Data structure used for preparing constraints before solving them
typedef enum BodyState: int32_t {
    DynamicBody = 1,
    StaticBody = 2,
    KinematicBody = 4,
    Articulation = 8,
} BodyState;

/// @
/// {
typedef enum PxArticulationAxis: int32_t {
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
} PxArticulationAxis;

typedef enum PxArticulationMotion: int32_t {
    /// Locked axis, i.e. degree of freedom (DOF)
    Locked = 0,
    /// Limited DOF - set limits of joint DOF together with this flag, see PxArticulationJointReducedCoordinate::setLimitParams
    Limited = 1,
    /// Free DOF
    Free = 2,
} PxArticulationMotion;

/// Flags for [`PxArticulationMotion`]
typedef enum PxArticulationMotions: uint8_t {
    Locked = 0 << 0,
    Limited = 1 << 0,
    Free = 1 << 1,
} PxArticulationMotions;


typedef enum PxArticulationJointType: int32_t {
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
} PxArticulationJointType;

typedef enum PxArticulationFlag: int32_t {
    /// Set articulation base to be fixed.
    FixBase = 1,
    /// Limits for drive effort are forces and torques rather than impulses, see PxArticulationDrive::maxForce.
    DriveLimitsAreForces = 2,
    /// Disable collisions between the articulation's links (note that parent/child collisions are disabled internally in either case).
    DisableSelfCollision = 4,
    /// Enable in order to be able to query joint solver (i.e. constraint) forces using PxArticulationCache::jointSolverForces.
    ComputeJointForces = 8,
} PxArticulationFlag;

/// Flags for [`PxArticulationFlag`]
typedef enum PxArticulationFlags: uint8_t {
    FixBase = 1 << 0,
    DriveLimitsAreForces = 1 << 1,
    DisableSelfCollision = 1 << 2,
    ComputeJointForces = 1 << 3,
} PxArticulationFlags;


typedef enum PxArticulationDriveType: int32_t {
    /// The output of the implicit spring drive controller is a force/torque.
    Force = 0,
    /// The output of the implicit spring drive controller is a joint acceleration (use this to get (spatial)-inertia-invariant behavior of the drive).
    Acceleration = 1,
    /// Sets the drive gains internally to track a target position almost kinematically (i.e. with very high drive gains).
    Target = 2,
    /// Sets the drive gains internally to track a target velocity almost kinematically (i.e. with very high drive gains).
    Velocity = 3,
    None = 4,
} PxArticulationDriveType;

/// A description of the types of articulation data that may be directly written to and read from the GPU using the functions
/// PxScene::copyArticulationData() and PxScene::applyArticulationData(). Types that are read-only may only be used in conjunction with
/// PxScene::copyArticulationData(). Types that are write-only may only be used in conjunction with PxScene::applyArticulationData().
/// A subset of data types may be used in conjunction with both PxScene::applyArticulationData() and PxScene::applyArticulationData().
typedef enum PxArticulationGpuDataType: int32_t {
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
} PxArticulationGpuDataType;

/// These flags determine what data is read or written to the internal articulation data via cache.
typedef enum PxArticulationCacheFlag: int32_t {
    /// The joint velocities, see PxArticulationCache::jointVelocity.
    Velocity = 1,
    /// The joint accelerations, see PxArticulationCache::jointAcceleration.
    Acceleration = 2,
    /// The joint positions, see PxArticulationCache::jointPosition.
    Position = 4,
    /// The joint forces, see PxArticulationCache::jointForce.
    Force = 8,
    /// The link velocities, see PxArticulationCache::linkVelocity.
    LinkVelocity = 16,
    /// The link accelerations, see PxArticulationCache::linkAcceleration.
    LinkAcceleration = 32,
    /// Root link transform, see PxArticulationCache::rootLinkData.
    RootTransform = 64,
    /// Root link velocities (read/write) and accelerations (read), see PxArticulationCache::rootLinkData.
    RootVelocities = 128,
    /// The spatial sensor forces, see PxArticulationCache::sensorForces.
    SensorForces = 256,
    /// Solver constraint joint forces, see PxArticulationCache::jointSolverForces.
    JointSolverForces = 512,
    All = 247,
} PxArticulationCacheFlag;

/// Flags for [`PxArticulationCacheFlag`]
typedef enum PxArticulationCacheFlags: uint32_t {
    Velocity = 1 << 0,
    Acceleration = 1 << 1,
    Position = 1 << 2,
    Force = 1 << 3,
    LinkVelocity = 1 << 4,
    LinkAcceleration = 1 << 5,
    RootTransform = 1 << 6,
    RootVelocities = 1 << 7,
    SensorForces = 1 << 8,
    JointSolverForces = 1 << 9,
    All = Velocity | Acceleration | Position | LinkVelocity | LinkAcceleration | RootTransform | RootVelocities,
} PxArticulationCacheFlags;


/// Flags to configure the forces reported by articulation link sensors.
typedef enum PxArticulationSensorFlag: int32_t {
    /// Raise to receive forces from forward dynamics.
    ForwardDynamicsForces = 1,
    /// Raise to receive forces from constraint solver.
    ConstraintSolverForces = 2,
    /// Raise to receive forces in the world rotation frame, otherwise they will be reported in the sensor's local frame.
    WorldFrame = 4,
} PxArticulationSensorFlag;

/// Flags for [`PxArticulationSensorFlag`]
typedef enum PxArticulationSensorFlags: uint8_t {
    ForwardDynamicsForces = 1 << 0,
    ConstraintSolverForces = 1 << 1,
    WorldFrame = 1 << 2,
} PxArticulationSensorFlags;


/// Flag that configures articulation-state updates by PxArticulationReducedCoordinate::updateKinematic.
typedef enum PxArticulationKinematicFlag: int32_t {
    /// Raise after any changes to the articulation root or joint positions using non-cache API calls. Updates links' positions and velocities.
    Position = 1,
    /// Raise after velocity-only changes to the articulation root or joints using non-cache API calls. Updates links' velocities.
    Velocity = 2,
} PxArticulationKinematicFlag;

/// Flags for [`PxArticulationKinematicFlag`]
typedef enum PxArticulationKinematicFlags: uint8_t {
    Position = 1 << 0,
    Velocity = 1 << 1,
} PxArticulationKinematicFlags;


/// Flags which affect the behavior of PxShapes.
typedef enum PxShapeFlag: int32_t {
    /// The shape will partake in collision in the physical simulation.
    ///
    /// It is illegal to raise the eSIMULATION_SHAPE and eTRIGGER_SHAPE flags.
    /// In the event that one of these flags is already raised the sdk will reject any
    /// attempt to raise the other.  To raise the eSIMULATION_SHAPE first ensure that
    /// eTRIGGER_SHAPE is already lowered.
    ///
    /// This flag has no effect if simulation is disabled for the corresponding actor (see [`PxActorFlag::eDISABLE_SIMULATION`]).
    SimulationShape = 1,
    /// The shape will partake in scene queries (ray casts, overlap tests, sweeps, ...).
    SceneQueryShape = 2,
    /// The shape is a trigger which can send reports whenever other shapes enter/leave its volume.
    ///
    /// Triangle meshes and heightfields can not be triggers. Shape creation will fail in these cases.
    ///
    /// Shapes marked as triggers do not collide with other objects. If an object should act both
    /// as a trigger shape and a collision shape then create a rigid body with two shapes, one being a
    /// trigger shape and the other a collision shape. It is illegal to raise the eTRIGGER_SHAPE and
    /// eSIMULATION_SHAPE flags on a single PxShape instance.  In the event that one of these flags is already
    /// raised the sdk will reject any attempt to raise the other.  To raise the eTRIGGER_SHAPE flag first
    /// ensure that eSIMULATION_SHAPE flag is already lowered.
    ///
    /// Trigger shapes will no longer send notification events for interactions with other trigger shapes.
    ///
    /// Shapes marked as triggers are allowed to participate in scene queries, provided the eSCENE_QUERY_SHAPE flag is set.
    ///
    /// This flag has no effect if simulation is disabled for the corresponding actor (see [`PxActorFlag::eDISABLE_SIMULATION`]).
    TriggerShape = 4,
    /// Enable debug renderer for this shape
    Visualization = 8,
} PxShapeFlag;

/// Flags for [`PxShapeFlag`]
typedef enum PxShapeFlags: uint8_t {
    SimulationShape = 1 << 0,
    SceneQueryShape = 1 << 1,
    TriggerShape = 1 << 2,
    Visualization = 1 << 3,
} PxShapeFlags;


/// Parameter to addForce() and addTorque() calls, determines the exact operation that is carried out.
typedef enum PxForceMode: int32_t {
    /// parameter has unit of mass * length / time^2, i.e., a force
    Force = 0,
    /// parameter has unit of mass * length / time, i.e., force * time
    Impulse = 1,
    /// parameter has unit of length / time, i.e., the effect is mass independent: a velocity change.
    VelocityChange = 2,
    /// parameter has unit of length/ time^2, i.e., an acceleration. It gets treated just like a force except the mass is not divided out before integration.
    Acceleration = 3,
} PxForceMode;

/// Collection of flags describing the behavior of a rigid body.
typedef enum PxRigidBodyFlag: int32_t {
    /// Enable kinematic mode for the body.
    Kinematic = 1,
    /// Use the kinematic target transform for scene queries.
    ///
    /// If this flag is raised, then scene queries will treat the kinematic target transform as the current pose
    /// of the body (instead of using the actual pose). Without this flag, the kinematic target will only take
    /// effect with respect to scene queries after a simulation step.
    UseKinematicTargetForSceneQueries = 2,
    /// Enable CCD for the body.
    EnableCcd = 4,
    /// Enabled CCD in swept integration for the actor.
    ///
    /// If this flag is raised and CCD is enabled, CCD interactions will simulate friction. By default, friction is disabled in CCD interactions because
    /// CCD friction has been observed to introduce some simulation artifacts. CCD friction was enabled in previous versions of the SDK. Raising this flag will result in behavior
    /// that is a closer match for previous versions of the SDK.
    ///
    /// This flag requires PxRigidBodyFlag::eENABLE_CCD to be raised to have any effect.
    EnableCcdFriction = 8,
    /// Register a rigid body to dynamically adjust contact offset based on velocity. This can be used to achieve a CCD effect.
    ///
    /// If both eENABLE_CCD and eENABLE_SPECULATIVE_CCD are set on the same body, then angular motions are handled by speculative
    /// contacts (eENABLE_SPECULATIVE_CCD) while linear motions are handled by sweeps (eENABLE_CCD).
    EnableSpeculativeCcd = 16,
    /// Register a rigid body for reporting pose changes by the simulation at an early stage.
    ///
    /// Sometimes it might be advantageous to get access to the new pose of a rigid body as early as possible and
    /// not wait until the call to fetchResults() returns. Setting this flag will schedule the rigid body to get reported
    /// in [`PxSimulationEventCallback::onAdvance`](). Please refer to the documentation of that callback to understand
    /// the behavior and limitations of this functionality.
    EnablePoseIntegrationPreview = 32,
    /// Permit CCD to limit maxContactImpulse. This is useful for use-cases like a destruction system but can cause visual artefacts so is not enabled by default.
    EnableCcdMaxContactImpulse = 64,
    /// Carries over forces/accelerations between frames, rather than clearing them
    RetainAccelerations = 128,
    /// Forces kinematic-kinematic pairs notifications for this actor.
    ///
    /// This flag overrides the global scene-level PxPairFilteringMode setting for kinematic actors.
    /// This is equivalent to having PxPairFilteringMode::eKEEP for pairs involving this actor.
    ///
    /// A particular use case is when you have a large amount of kinematic actors, but you are only
    /// interested in interactions between a few of them. In this case it is best to use
    /// PxSceneDesc.kineKineFilteringMode = PxPairFilteringMode::eKILL, and then raise the
    /// eFORCE_KINE_KINE_NOTIFICATIONS flag on the small set of kinematic actors that need
    /// notifications.
    ///
    /// This has no effect if PxRigidBodyFlag::eKINEMATIC is not set.
    ///
    /// Changing this flag at runtime will not have an effect until you remove and re-add the actor to the scene.
    ForceKineKineNotifications = 256,
    /// Forces static-kinematic pairs notifications for this actor.
    ///
    /// Similar to eFORCE_KINE_KINE_NOTIFICATIONS, but for static-kinematic interactions.
    ///
    /// This has no effect if PxRigidBodyFlag::eKINEMATIC is not set.
    ///
    /// Changing this flag at runtime will not have an effect until you remove and re-add the actor to the scene.
    ForceStaticKineNotifications = 512,
    /// Enables computation of gyroscopic forces on the rigid body.
    EnableGyroscopicForces = 1024,
} PxRigidBodyFlag;

/// Flags for [`PxRigidBodyFlag`]
typedef enum PxRigidBodyFlags: uint16_t {
    Kinematic = 1 << 0,
    UseKinematicTargetForSceneQueries = 1 << 1,
    EnableCcd = 1 << 2,
    EnableCcdFriction = 1 << 3,
    EnableSpeculativeCcd = 1 << 4,
    EnablePoseIntegrationPreview = 1 << 5,
    EnableCcdMaxContactImpulse = 1 << 6,
    RetainAccelerations = 1 << 7,
    ForceKineKineNotifications = 1 << 8,
    ForceStaticKineNotifications = 1 << 9,
    EnableGyroscopicForces = 1 << 10,
} PxRigidBodyFlags;


/// constraint flags
///
/// eBROKEN is a read only flag
typedef enum PxConstraintFlag: int32_t {
    /// whether the constraint is broken
    Broken = 1,
    /// whether actor1 should get projected to actor0 for this constraint (note: projection of a static/kinematic actor to a dynamic actor will be ignored)
    ProjectToActor0 = 2,
    /// whether actor0 should get projected to actor1 for this constraint (note: projection of a static/kinematic actor to a dynamic actor will be ignored)
    ProjectToActor1 = 4,
    /// whether the actors should get projected for this constraint (the direction will be chosen by PhysX)
    Projection = 6,
    /// whether contacts should be generated between the objects this constraint constrains
    CollisionEnabled = 8,
    /// whether this constraint should be visualized, if constraint visualization is turned on
    Visualization = 16,
    /// limits for drive strength are forces rather than impulses
    DriveLimitsAreForces = 32,
    /// perform preprocessing for improved accuracy on D6 Slerp Drive (this flag will be removed in a future release when preprocessing is no longer required)
    ImprovedSlerp = 128,
    /// suppress constraint preprocessing, intended for use with rowResponseThreshold. May result in worse solver accuracy for ill-conditioned constraints.
    DisablePreprocessing = 256,
    /// enables extended limit ranges for angular limits (e.g., limit values > PxPi or
    /// <
    /// -PxPi)
    EnableExtendedLimits = 512,
    /// the constraint type is supported by gpu dynamics
    GpuCompatible = 1024,
    /// updates the constraint each frame
    AlwaysUpdate = 2048,
    /// disables the constraint. SolverPrep functions won't be called for this constraint.
    DisableConstraint = 4096,
} PxConstraintFlag;

/// Flags for [`PxConstraintFlag`]
typedef enum PxConstraintFlags: uint16_t {
    Broken = 1 << 0,
    ProjectToActor0 = 1 << 1,
    ProjectToActor1 = 1 << 2,
    Projection = ProjectToActor0 | ProjectToActor1,
    CollisionEnabled = 1 << 3,
    Visualization = 1 << 4,
    DriveLimitsAreForces = 1 << 5,
    ImprovedSlerp = 1 << 7,
    DisablePreprocessing = 1 << 8,
    EnableExtendedLimits = 1 << 9,
    GpuCompatible = 1 << 10,
    AlwaysUpdate = 1 << 11,
    DisableConstraint = 1 << 12,
} PxConstraintFlags;


/// Header for a contact patch where all points share same material and normal
typedef enum PxContactPatchFlags: int32_t {
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
} PxContactPatchFlags;

/// A class to iterate over a compressed contact stream. This supports read-only access to the various contact formats.
typedef enum StreamFormat: int32_t {
    SimpleStream = 0,
    ModifiableStream = 1,
    CompressedModifiableStream = 2,
} StreamFormat;

/// Flags specifying deletion event types.
typedef enum PxDeletionEventFlag: int32_t {
    /// The user has called release on an object.
    UserRelease = 1,
    /// The destructor of an object has been called and the memory has been released.
    MemoryRelease = 2,
} PxDeletionEventFlag;

/// Flags for [`PxDeletionEventFlag`]
typedef enum PxDeletionEventFlags: uint8_t {
    UserRelease = 1 << 0,
    MemoryRelease = 1 << 1,
} PxDeletionEventFlags;


/// Collection of flags describing the actions to take for a collision pair.
typedef enum PxPairFlag: int32_t {
    /// Process the contacts of this collision pair in the dynamics solver.
    ///
    /// Only takes effect if the colliding actors are rigid bodies.
    SolveContact = 1,
    /// Call contact modification callback for this collision pair
    ///
    /// Only takes effect if the colliding actors are rigid bodies.
    ModifyContacts = 2,
    /// Call contact report callback or trigger callback when this collision pair starts to be in contact.
    ///
    /// If one of the two collision objects is a trigger shape (see [`PxShapeFlag::eTRIGGER_SHAPE`])
    /// then the trigger callback will get called as soon as the other object enters the trigger volume.
    /// If none of the two collision objects is a trigger shape then the contact report callback will get
    /// called when the actors of this collision pair start to be in contact.
    ///
    /// Only takes effect if the colliding actors are rigid bodies.
    ///
    /// Only takes effect if eDETECT_DISCRETE_CONTACT or eDETECT_CCD_CONTACT is raised
    NotifyTouchFound = 4,
    /// Call contact report callback while this collision pair is in contact
    ///
    /// If none of the two collision objects is a trigger shape then the contact report callback will get
    /// called while the actors of this collision pair are in contact.
    ///
    /// Triggers do not support this event. Persistent trigger contacts need to be tracked separately by observing eNOTIFY_TOUCH_FOUND/eNOTIFY_TOUCH_LOST events.
    ///
    /// Only takes effect if the colliding actors are rigid bodies.
    ///
    /// No report will get sent if the objects in contact are sleeping.
    ///
    /// Only takes effect if eDETECT_DISCRETE_CONTACT or eDETECT_CCD_CONTACT is raised
    ///
    /// If this flag gets enabled while a pair is in touch already, there will be no eNOTIFY_TOUCH_PERSISTS events until the pair loses and regains touch.
    NotifyTouchPersists = 8,
    /// Call contact report callback or trigger callback when this collision pair stops to be in contact
    ///
    /// If one of the two collision objects is a trigger shape (see [`PxShapeFlag::eTRIGGER_SHAPE`])
    /// then the trigger callback will get called as soon as the other object leaves the trigger volume.
    /// If none of the two collision objects is a trigger shape then the contact report callback will get
    /// called when the actors of this collision pair stop to be in contact.
    ///
    /// Only takes effect if the colliding actors are rigid bodies.
    ///
    /// This event will also get triggered if one of the colliding objects gets deleted.
    ///
    /// Only takes effect if eDETECT_DISCRETE_CONTACT or eDETECT_CCD_CONTACT is raised
    NotifyTouchLost = 16,
    /// Call contact report callback when this collision pair is in contact during CCD passes.
    ///
    /// If CCD with multiple passes is enabled, then a fast moving object might bounce on and off the same
    /// object multiple times. Hence, the same pair might be in contact multiple times during a simulation step.
    /// This flag will make sure that all the detected collision during CCD will get reported. For performance
    /// reasons, the system can not always tell whether the contact pair lost touch in one of the previous CCD
    /// passes and thus can also not always tell whether the contact is new or has persisted. eNOTIFY_TOUCH_CCD
    /// just reports when the two collision objects were detected as being in contact during a CCD pass.
    ///
    /// Only takes effect if the colliding actors are rigid bodies.
    ///
    /// Trigger shapes are not supported.
    ///
    /// Only takes effect if eDETECT_CCD_CONTACT is raised
    NotifyTouchCcd = 32,
    /// Call contact report callback when the contact force between the actors of this collision pair exceeds one of the actor-defined force thresholds.
    ///
    /// Only takes effect if the colliding actors are rigid bodies.
    ///
    /// Only takes effect if eDETECT_DISCRETE_CONTACT or eDETECT_CCD_CONTACT is raised
    NotifyThresholdForceFound = 64,
    /// Call contact report callback when the contact force between the actors of this collision pair continues to exceed one of the actor-defined force thresholds.
    ///
    /// Only takes effect if the colliding actors are rigid bodies.
    ///
    /// If a pair gets re-filtered and this flag has previously been disabled, then the report will not get fired in the same frame even if the force threshold has been reached in the
    /// previous one (unless [`eNOTIFY_THRESHOLD_FORCE_FOUND`] has been set in the previous frame).
    ///
    /// Only takes effect if eDETECT_DISCRETE_CONTACT or eDETECT_CCD_CONTACT is raised
    NotifyThresholdForcePersists = 128,
    /// Call contact report callback when the contact force between the actors of this collision pair falls below one of the actor-defined force thresholds (includes the case where this collision pair stops being in contact).
    ///
    /// Only takes effect if the colliding actors are rigid bodies.
    ///
    /// If a pair gets re-filtered and this flag has previously been disabled, then the report will not get fired in the same frame even if the force threshold has been reached in the
    /// previous one (unless [`eNOTIFY_THRESHOLD_FORCE_FOUND`] or #eNOTIFY_THRESHOLD_FORCE_PERSISTS has been set in the previous frame).
    ///
    /// Only takes effect if eDETECT_DISCRETE_CONTACT or eDETECT_CCD_CONTACT is raised
    NotifyThresholdForceLost = 256,
    /// Provide contact points in contact reports for this collision pair.
    ///
    /// Only takes effect if the colliding actors are rigid bodies and if used in combination with the flags eNOTIFY_TOUCH_... or eNOTIFY_THRESHOLD_FORCE_...
    ///
    /// Only takes effect if eDETECT_DISCRETE_CONTACT or eDETECT_CCD_CONTACT is raised
    NotifyContactPoints = 512,
    /// This flag is used to indicate whether this pair generates discrete collision detection contacts.
    ///
    /// Contacts are only responded to if eSOLVE_CONTACT is enabled.
    DetectDiscreteContact = 1024,
    /// This flag is used to indicate whether this pair generates CCD contacts.
    ///
    /// The contacts will only be responded to if eSOLVE_CONTACT is enabled on this pair.
    ///
    /// The scene must have PxSceneFlag::eENABLE_CCD enabled to use this feature.
    ///
    /// Non-static bodies of the pair should have PxRigidBodyFlag::eENABLE_CCD specified for this feature to work correctly.
    ///
    /// This flag is not supported with trigger shapes. However, CCD trigger events can be emulated using non-trigger shapes
    /// and requesting eNOTIFY_TOUCH_FOUND and eNOTIFY_TOUCH_LOST and not raising eSOLVE_CONTACT on the pair.
    DetectCcdContact = 2048,
    /// Provide pre solver velocities in contact reports for this collision pair.
    ///
    /// If the collision pair has contact reports enabled, the velocities of the rigid bodies before contacts have been solved
    /// will be provided in the contact report callback unless the pair lost touch in which case no data will be provided.
    ///
    /// Usually it is not necessary to request these velocities as they will be available by querying the velocity from the provided
    /// PxRigidActor object directly. However, it might be the case that the velocity of a rigid body gets set while the simulation is running
    /// in which case the PxRigidActor would return this new velocity in the contact report callback and not the velocity the simulation used.
    PreSolverVelocity = 4096,
    /// Provide post solver velocities in contact reports for this collision pair.
    ///
    /// If the collision pair has contact reports enabled, the velocities of the rigid bodies after contacts have been solved
    /// will be provided in the contact report callback unless the pair lost touch in which case no data will be provided.
    PostSolverVelocity = 8192,
    /// Provide rigid body poses in contact reports for this collision pair.
    ///
    /// If the collision pair has contact reports enabled, the rigid body poses at the contact event will be provided
    /// in the contact report callback unless the pair lost touch in which case no data will be provided.
    ///
    /// Usually it is not necessary to request these poses as they will be available by querying the pose from the provided
    /// PxRigidActor object directly. However, it might be the case that the pose of a rigid body gets set while the simulation is running
    /// in which case the PxRigidActor would return this new pose in the contact report callback and not the pose the simulation used.
    /// Another use case is related to CCD with multiple passes enabled, A fast moving object might bounce on and off the same
    /// object multiple times. This flag can be used to request the rigid body poses at the time of impact for each such collision event.
    ContactEventPose = 16384,
    /// For internal use only.
    NextFree = 32768,
    /// Provided default flag to do simple contact processing for this collision pair.
    ContactDefault = 1025,
    /// Provided default flag to get commonly used trigger behavior for this collision pair.
    TriggerDefault = 1044,
} PxPairFlag;

/// Flags for [`PxPairFlag`]
typedef enum PxPairFlags: uint16_t {
    SolveContact = 1 << 0,
    ModifyContacts = 1 << 1,
    NotifyTouchFound = 1 << 2,
    NotifyTouchPersists = 1 << 3,
    NotifyTouchLost = 1 << 4,
    NotifyTouchCcd = 1 << 5,
    NotifyThresholdForceFound = 1 << 6,
    NotifyThresholdForcePersists = 1 << 7,
    NotifyThresholdForceLost = 1 << 8,
    NotifyContactPoints = 1 << 9,
    DetectDiscreteContact = 1 << 10,
    DetectCcdContact = 1 << 11,
    PreSolverVelocity = 1 << 12,
    PostSolverVelocity = 1 << 13,
    ContactEventPose = 1 << 14,
    NextFree = 1 << 15,
    ContactDefault = SolveContact | DetectDiscreteContact,
    TriggerDefault = NotifyTouchFound | NotifyTouchLost | DetectDiscreteContact,
} PxPairFlags;


/// Collection of flags describing the filter actions to take for a collision pair.
typedef enum PxFilterFlag: int32_t {
    /// Ignore the collision pair as long as the bounding volumes of the pair objects overlap.
    ///
    /// Killed pairs will be ignored by the simulation and won't run through the filter again until one
    /// of the following occurs:
    ///
    /// The bounding volumes of the two objects overlap again (after being separated)
    ///
    /// The user enforces a re-filtering (see [`PxScene::resetFiltering`]())
    Kill = 1,
    /// Ignore the collision pair as long as the bounding volumes of the pair objects overlap or until filtering relevant data changes for one of the collision objects.
    ///
    /// Suppressed pairs will be ignored by the simulation and won't make another filter request until one
    /// of the following occurs:
    ///
    /// Same conditions as for killed pairs (see [`eKILL`])
    ///
    /// The filter data or the filter object attributes change for one of the collision objects
    Suppress = 2,
    /// Invoke the filter callback ([`PxSimulationFilterCallback::pairFound`]()) for this collision pair.
    Callback = 4,
    /// Track this collision pair with the filter callback mechanism.
    ///
    /// When the bounding volumes of the collision pair lose contact, the filter callback [`PxSimulationFilterCallback::pairLost`]()
    /// will be invoked. Furthermore, the filter status of the collision pair can be adjusted through [`PxSimulationFilterCallback::statusChange`]()
    /// once per frame (until a pairLost() notification occurs).
    Notify = 12,
    /// Provided default to get standard behavior:
    ///
    /// The application configure the pair's collision properties once when bounding volume overlap is found and
    /// doesn't get asked again about that pair until overlap status or filter properties changes, or re-filtering is requested.
    ///
    /// No notification is provided when bounding volume overlap is lost
    ///
    /// The pair will not be killed or suppressed, so collision detection will be processed
    Default = 0,
} PxFilterFlag;

/// Flags for [`PxFilterFlag`]
typedef enum PxFilterFlags: uint16_t {
    Kill = 1 << 0,
    Suppress = 1 << 1,
    Callback = 1 << 2,
    Notify = Callback,
    Default = 0 << 0,
} PxFilterFlags;


/// Identifies each type of filter object.
typedef enum PxFilterObjectType: int32_t {
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
} PxFilterObjectType;

typedef enum PxFilterObjectFlag: int32_t {
    Kinematic = 16,
    Trigger = 32,
} PxFilterObjectFlag;

typedef enum PxPairFilteringMode: int32_t {
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
} PxPairFilteringMode;

typedef enum PxDataAccessFlag: int32_t {
    Readable = 1,
    Writable = 2,
    Device = 4,
} PxDataAccessFlag;

/// Flags for [`PxDataAccessFlag`]
typedef enum PxDataAccessFlags: uint8_t {
    Readable = 1 << 0,
    Writable = 1 << 1,
    Device = 1 << 2,
} PxDataAccessFlags;


/// Flags which control the behavior of a material.
typedef enum PxMaterialFlag: int32_t {
    /// If this flag is set, friction computations are always skipped between shapes with this material and any other shape.
    DisableFriction = 1,
    /// Whether to use strong friction.
    /// The difference between "normal" and "strong" friction is that the strong friction feature
    /// remembers the "friction error" between simulation steps. The friction is a force trying to
    /// hold objects in place (or slow them down) and this is handled in the solver. But since the
    /// solver is only an approximation, the result of the friction calculation can include a small
    /// "error" - e.g. a box resting on a slope should not move at all if the static friction is in
    /// action, but could slowly glide down the slope because of a small friction error in each
    /// simulation step. The strong friction counter-acts this by remembering the small error and
    /// taking it to account during the next simulation step.
    ///
    /// However, in some cases the strong friction could cause problems, and this is why it is
    /// possible to disable the strong friction feature by setting this flag. One example is
    /// raycast vehicles that are sliding fast across the surface, but still need a precise
    /// steering behavior. It may be a good idea to reenable the strong friction when objects
    /// are coming to a rest, to prevent them from slowly creeping down inclines.
    ///
    /// Note: This flag only has an effect if the PxMaterialFlag::eDISABLE_FRICTION bit is 0.
    DisableStrongFriction = 2,
    /// Whether to use the patch friction model.
    /// This flag only has an effect if PxFrictionType::ePATCH friction model is used.
    ///
    /// When using the patch friction model, up to 2 friction anchors are generated per patch. As the number of friction anchors
    /// can be smaller than the number of contacts, the normal force is accumulated over all contacts and used to compute friction
    /// for all anchors. Where there are more than 2 anchors, this can produce frictional behavior that is too strong (approximately 2x as strong
    /// as analytical models suggest).
    ///
    /// This flag causes the normal force to be distributed between the friction anchors such that the total amount of friction applied does not
    /// exceed the analytical results.
    ImprovedPatchFriction = 4,
    /// This flag has the effect of enabling an implicit spring model for the normal force computation.
    CompliantContact = 8,
} PxMaterialFlag;

/// Flags for [`PxMaterialFlag`]
typedef enum PxMaterialFlags: uint16_t {
    DisableFriction = 1 << 0,
    DisableStrongFriction = 1 << 1,
    ImprovedPatchFriction = 1 << 2,
    CompliantContact = 1 << 3,
} PxMaterialFlags;


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
typedef enum PxCombineMode: int32_t {
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
} PxCombineMode;

/// Identifies dirty particle buffers that need to be updated in the particle system.
///
/// This flag can be used mark the device user buffers that are dirty and need to be written to the particle system.
typedef enum PxParticleBufferFlag: int32_t {
    /// No data specified
    None = 0,
    /// Specifies the position (first 3 floats) and inverse mass (last float) data (array of PxVec4 * number of particles)
    UpdatePosition = 1,
    /// Specifies the velocity (first 3 floats) data (array of PxVec4 * number of particles)
    UpdateVelocity = 2,
    /// Specifies the per-particle phase flag data (array of PxU32 * number of particles)
    UpdatePhase = 4,
    /// Specifies the rest position (first 3 floats) data for cloth buffers
    UpdateRestposition = 8,
    /// Specifies the cloth buffer (see PxParticleClothBuffer)
    UpdateCloth = 32,
    /// Specifies the rigid buffer (see PxParticleRigidBuffer)
    UpdateRigid = 64,
    /// Specifies the diffuse particle parameter buffer (see PxDiffuseParticleParams)
    UpdateDiffuseParam = 128,
    /// Specifies the attachments.
    UpdateAttachments = 256,
    All = 495,
} PxParticleBufferFlag;

/// Flags for [`PxParticleBufferFlag`]
typedef enum PxParticleBufferFlags: uint32_t {
    None = 0 << 0,
    UpdatePosition = 1 << 0,
    UpdateVelocity = 1 << 1,
    UpdatePhase = 1 << 2,
    UpdateRestposition = 1 << 3,
    UpdateCloth = 1 << 5,
    UpdateRigid = 1 << 6,
    UpdateDiffuseParam = 1 << 7,
    UpdateAttachments = 1 << 8,
    All = UpdatePosition | UpdateVelocity | UpdatePhase | UpdateRestposition | UpdateCloth | UpdateRigid | UpdateDiffuseParam | UpdateAttachments,
} PxParticleBufferFlags;


/// Identifies per-particle behavior for a PxParticleSystem.
///
/// See [`PxParticleSystem::createPhase`]().
typedef enum PxParticlePhaseFlag: uint32_t {
    /// Bits [ 0, 19] represent the particle group for controlling collisions
    ParticlePhaseGroupMask = 1048575,
    /// Bits [20, 23] hold flags about how the particle behave
    ParticlePhaseFlagsMask = 4293918720,
    /// If set this particle will interact with particles of the same group
    ParticlePhaseSelfCollide = 1048576,
    /// If set this particle will ignore collisions with particles closer than the radius in the rest pose, this flag should not be specified unless valid rest positions have been specified using setRestParticles()
    ParticlePhaseSelfCollideFilter = 2097152,
    /// If set this particle will generate fluid density constraints for its overlapping neighbors
    ParticlePhaseFluid = 4194304,
} PxParticlePhaseFlag;

/// Flags for [`PxParticlePhaseFlag`]
typedef enum PxParticlePhaseFlags: uint32_t {
    ParticlePhaseGroupMask = 0x000fffff,
    ParticlePhaseFlagsMask = ParticlePhaseSelfCollide | ParticlePhaseSelfCollideFilter | ParticlePhaseFluid,
    ParticlePhaseSelfCollide = 1 << 20,
    ParticlePhaseSelfCollideFilter = 1 << 21,
    ParticlePhaseFluid = 1 << 22,
} PxParticlePhaseFlags;


/// Specifies memory space for a PxBuffer instance.
typedef enum PxBufferType: int32_t {
    Host = 0,
    Device = 1,
} PxBufferType;

/// Filtering flags for scene queries.
typedef enum PxQueryFlag: int32_t {
    /// Traverse static shapes
    Static = 1,
    /// Traverse dynamic shapes
    Dynamic = 2,
    /// Run the pre-intersection-test filter (see [`PxQueryFilterCallback::preFilter`]())
    Prefilter = 4,
    /// Run the post-intersection-test filter (see [`PxQueryFilterCallback::postFilter`]())
    Postfilter = 8,
    /// Abort traversal as soon as any hit is found and return it via callback.block.
    /// Helps query performance. Both eTOUCH and eBLOCK hitTypes are considered hits with this flag.
    AnyHit = 16,
    /// All hits are reported as touching. Overrides eBLOCK returned from user filters with eTOUCH.
    /// This is also an optimization hint that may improve query performance.
    NoBlock = 32,
    /// Same as eBATCH_QUERY_LEGACY_BEHAVIOUR, more explicit name making it clearer that this can also be used
    /// with regular/non-batched queries if needed.
    DisableHardcodedFilter = 64,
    /// Reserved for internal use
    Reserved = 32768,
} PxQueryFlag;

/// Flags for [`PxQueryFlag`]
typedef enum PxQueryFlags: uint16_t {
    Static = 1 << 0,
    Dynamic = 1 << 1,
    Prefilter = 1 << 2,
    Postfilter = 1 << 3,
    AnyHit = 1 << 4,
    NoBlock = 1 << 5,
    DisableHardcodedFilter = 1 << 6,
    Reserved = 1 << 15,
} PxQueryFlags;


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
typedef enum PxQueryHitType: int32_t {
    /// the query should ignore this shape
    None = 0,
    /// a hit on the shape touches the intersection geometry of the query but does not block it
    Touch = 1,
    /// a hit on the shape blocks the query (does not block overlap queries)
    Block = 2,
} PxQueryHitType;

/// Collection of flags providing a mechanism to lock motion along/around a specific axis.
typedef enum PxRigidDynamicLockFlag: int32_t {
    LockLinearX = 1,
    LockLinearY = 2,
    LockLinearZ = 4,
    LockAngularX = 8,
    LockAngularY = 16,
    LockAngularZ = 32,
} PxRigidDynamicLockFlag;

/// Flags for [`PxRigidDynamicLockFlag`]
typedef enum PxRigidDynamicLockFlags: uint8_t {
    LockLinearX = 1 << 0,
    LockLinearY = 1 << 1,
    LockLinearZ = 1 << 2,
    LockAngularX = 1 << 3,
    LockAngularY = 1 << 4,
    LockAngularZ = 1 << 5,
} PxRigidDynamicLockFlags;


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
typedef enum PxPruningStructureType: int32_t {
    /// Using a simple data structure
    None = 0,
    /// Using a dynamic AABB tree
    DynamicAabbTree = 1,
    /// Using a static AABB tree
    StaticAabbTree = 2,
    Last = 3,
} PxPruningStructureType;

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
typedef enum PxDynamicTreeSecondaryPruner: int32_t {
    /// no secondary pruner, new objects aren't visible to SQ for a few frames
    None = 0,
    /// bucket-based secondary pruner, faster updates, slower query time
    Bucket = 1,
    /// incremental-BVH secondary pruner, faster query time, slower updates
    Incremental = 2,
    /// PxBVH-based secondary pruner, good overall performance
    Bvh = 3,
    Last = 4,
} PxDynamicTreeSecondaryPruner;

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
typedef enum PxSceneQueryUpdateMode: int32_t {
    /// Both scene query build and commit are executed.
    BuildEnabledCommitEnabled = 0,
    /// Scene query build only is executed.
    BuildEnabledCommitDisabled = 1,
    /// No work is done, no update of scene queries
    BuildDisabledCommitDisabled = 2,
} PxSceneQueryUpdateMode;

/// Built-in enum for default PxScene pruners
///
/// This is passed as a pruner index to various functions in the following APIs.
typedef enum PxScenePrunerIndex: uint32_t {
    PxScenePrunerStatic = 0,
    PxScenePrunerDynamic = 1,
    PxSceneCompoundPruner = 4294967295,
} PxScenePrunerIndex;

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
typedef enum PxBroadPhaseType: int32_t {
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
} PxBroadPhaseType;

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
typedef enum PxFrictionType: int32_t {
    /// Select default patch-friction model.
    Patch = 0,
    /// Select one directional per-contact friction model.
    OneDirectional = 1,
    /// Select two directional per-contact friction model.
    TwoDirectional = 2,
    /// The total number of friction models supported by the SDK.
    FrictionCount = 3,
} PxFrictionType;

/// Enum for selecting the type of solver used for the simulation.
///
/// [`PxSolverType::ePGS`] selects the iterative sequential impulse solver. This is the same kind of solver used in PhysX 3.4 and earlier releases.
///
/// [`PxSolverType::eTGS`] selects a non linear iterative solver. This kind of solver can lead to improved convergence and handle large mass ratios, long chains and jointed systems better. It is slightly more expensive than the default solver and can introduce more energy to correct joint and contact errors.
typedef enum PxSolverType: int32_t {
    /// Projected Gauss-Seidel iterative solver
    Pgs = 0,
    /// Default Temporal Gauss-Seidel solver
    Tgs = 1,
} PxSolverType;

/// flags for configuring properties of the scene
typedef enum PxSceneFlag: int32_t {
    /// Enable Active Actors Notification.
    ///
    /// This flag enables the Active Actor Notification feature for a scene.  This
    /// feature defaults to disabled.  When disabled, the function
    /// PxScene::getActiveActors() will always return a NULL list.
    ///
    /// There may be a performance penalty for enabling the Active Actor Notification, hence this flag should
    /// only be enabled if the application intends to use the feature.
    ///
    /// Default:
    /// False
    EnableActiveActors = 1,
    /// Enables a second broad phase check after integration that makes it possible to prevent objects from tunneling through eachother.
    ///
    /// PxPairFlag::eDETECT_CCD_CONTACT requires this flag to be specified.
    ///
    /// For this feature to be effective for bodies that can move at a significant velocity, the user should raise the flag PxRigidBodyFlag::eENABLE_CCD for them.
    ///
    /// This flag is not mutable, and must be set in PxSceneDesc at scene creation.
    ///
    /// Default:
    /// False
    EnableCcd = 2,
    /// Enables a simplified swept integration strategy, which sacrifices some accuracy for improved performance.
    ///
    /// This simplified swept integration approach makes certain assumptions about the motion of objects that are not made when using a full swept integration.
    /// These assumptions usually hold but there are cases where they could result in incorrect behavior between a set of fast-moving rigid bodies. A key issue is that
    /// fast-moving dynamic objects may tunnel through each-other after a rebound. This will not happen if this mode is disabled. However, this approach will be potentially
    /// faster than a full swept integration because it will perform significantly fewer sweeps in non-trivial scenes involving many fast-moving objects. This approach
    /// should successfully resist objects passing through the static environment.
    ///
    /// PxPairFlag::eDETECT_CCD_CONTACT requires this flag to be specified.
    ///
    /// This scene flag requires eENABLE_CCD to be enabled as well. If it is not, this scene flag will do nothing.
    ///
    /// For this feature to be effective for bodies that can move at a significant velocity, the user should raise the flag PxRigidBodyFlag::eENABLE_CCD for them.
    ///
    /// This flag is not mutable, and must be set in PxSceneDesc at scene creation.
    ///
    /// Default:
    /// False
    DisableCcdResweep = 4,
    /// Enable GJK-based distance collision detection system.
    ///
    /// This flag is not mutable, and must be set in PxSceneDesc at scene creation.
    ///
    /// Default:
    /// true
    EnablePcm = 64,
    /// Disable contact report buffer resize. Once the contact buffer is full, the rest of the contact reports will
    /// not be buffered and sent.
    ///
    /// This flag is not mutable, and must be set in PxSceneDesc at scene creation.
    ///
    /// Default:
    /// false
    DisableContactReportBufferResize = 128,
    /// Disable contact cache.
    ///
    /// Contact caches are used internally to provide faster contact generation. You can disable all contact caches
    /// if memory usage for this feature becomes too high.
    ///
    /// This flag is not mutable, and must be set in PxSceneDesc at scene creation.
    ///
    /// Default:
    /// false
    DisableContactCache = 256,
    /// Require scene-level locking
    ///
    /// When set to true this requires that threads accessing the PxScene use the
    /// multi-threaded lock methods.
    ///
    /// This flag is not mutable, and must be set in PxSceneDesc at scene creation.
    ///
    /// Default:
    /// false
    RequireRwLock = 512,
    /// Enables additional stabilization pass in solver
    ///
    /// When set to true, this enables additional stabilization processing to improve that stability of complex interactions between large numbers of bodies.
    ///
    /// Note that this flag is not mutable and must be set in PxSceneDesc at scene creation. Also, this is an experimental feature which does result in some loss of momentum.
    EnableStabilization = 1024,
    /// Enables average points in contact manifolds
    ///
    /// When set to true, this enables additional contacts to be generated per manifold to represent the average point in a manifold. This can stabilize stacking when only a small
    /// number of solver iterations is used.
    ///
    /// Note that this flag is not mutable and must be set in PxSceneDesc at scene creation.
    EnableAveragePoint = 2048,
    /// Do not report kinematics in list of active actors.
    ///
    /// Since the target pose for kinematics is set by the user, an application can track the activity state directly and use
    /// this flag to avoid that kinematics get added to the list of active actors.
    ///
    /// This flag has only an effect in combination with eENABLE_ACTIVE_ACTORS.
    ///
    /// Default:
    /// false
    ExcludeKinematicsFromActiveActors = 4096,
    /// Do not report kinematics in list of active actors.
    ///
    /// Since the target pose for kinematics is set by the user, an application can track the activity state directly and use
    /// this flag to avoid that kinematics get added to the list of active actors.
    ///
    /// This flag has only an effect in combination with eENABLE_ACTIVE_ACTORS.
    ///
    /// Default:
    /// false
    EnableGpuDynamics = 8192,
    /// Provides improved determinism at the expense of performance.
    ///
    /// By default, PhysX provides limited determinism guarantees. Specifically, PhysX guarantees that the exact scene (same actors created in the same order) and simulated using the same
    /// time-stepping scheme should provide the exact same behaviour.
    ///
    /// However, if additional actors are added to the simulation, this can affect the behaviour of the existing actors in the simulation, even if the set of new actors do not interact with
    /// the existing actors.
    ///
    /// This flag provides an additional level of determinism that guarantees that the simulation will not change if additional actors are added to the simulation, provided those actors do not interfere
    /// with the existing actors in the scene. Determinism is only guaranteed if the actors are inserted in a consistent order each run in a newly-created scene and simulated using a consistent time-stepping
    /// scheme.
    ///
    /// Note that this flag is not mutable and must be set at scene creation.
    ///
    /// Note that enabling this flag can have a negative impact on performance.
    ///
    /// Note that this feature is not currently supported on GPU.
    ///
    /// Default
    /// false
    EnableEnhancedDeterminism = 16384,
    /// Controls processing friction in all solver iterations
    ///
    /// By default, PhysX processes friction only in the final 3 position iterations, and all velocity
    /// iterations. This flag enables friction processing in all position and velocity iterations.
    ///
    /// The default behaviour provides a good trade-off between performance and stability and is aimed
    /// primarily at game development.
    ///
    /// When simulating more complex frictional behaviour, such as grasping of complex geometries with
    /// a robotic manipulator, better results can be achieved by enabling friction in all solver iterations.
    ///
    /// This flag only has effect with the default solver. The TGS solver always performs friction per-iteration.
    EnableFrictionEveryIteration = 32768,
    /// Controls processing friction in all solver iterations
    ///
    /// By default, PhysX processes friction only in the final 3 position iterations, and all velocity
    /// iterations. This flag enables friction processing in all position and velocity iterations.
    ///
    /// The default behaviour provides a good trade-off between performance and stability and is aimed
    /// primarily at game development.
    ///
    /// When simulating more complex frictional behaviour, such as grasping of complex geometries with
    /// a robotic manipulator, better results can be achieved by enabling friction in all solver iterations.
    ///
    /// This flag only has effect with the default solver. The TGS solver always performs friction per-iteration.
    SuppressReadback = 65536,
    /// Controls processing friction in all solver iterations
    ///
    /// By default, PhysX processes friction only in the final 3 position iterations, and all velocity
    /// iterations. This flag enables friction processing in all position and velocity iterations.
    ///
    /// The default behaviour provides a good trade-off between performance and stability and is aimed
    /// primarily at game development.
    ///
    /// When simulating more complex frictional behaviour, such as grasping of complex geometries with
    /// a robotic manipulator, better results can be achieved by enabling friction in all solver iterations.
    ///
    /// This flag only has effect with the default solver. The TGS solver always performs friction per-iteration.
    ForceReadback = 131072,
    /// Controls processing friction in all solver iterations
    ///
    /// By default, PhysX processes friction only in the final 3 position iterations, and all velocity
    /// iterations. This flag enables friction processing in all position and velocity iterations.
    ///
    /// The default behaviour provides a good trade-off between performance and stability and is aimed
    /// primarily at game development.
    ///
    /// When simulating more complex frictional behaviour, such as grasping of complex geometries with
    /// a robotic manipulator, better results can be achieved by enabling friction in all solver iterations.
    ///
    /// This flag only has effect with the default solver. The TGS solver always performs friction per-iteration.
    MutableFlags = 69633,
} PxSceneFlag;

/// Flags for [`PxSceneFlag`]
typedef enum PxSceneFlags: uint32_t {
    EnableActiveActors = 1 << 0,
    EnableCcd = 1 << 1,
    DisableCcdResweep = 1 << 2,
    EnablePcm = 1 << 6,
    DisableContactReportBufferResize = 1 << 7,
    DisableContactCache = 1 << 8,
    RequireRwLock = 1 << 9,
    EnableStabilization = 1 << 10,
    EnableAveragePoint = 1 << 11,
    ExcludeKinematicsFromActiveActors = 1 << 12,
    EnableGpuDynamics = 1 << 13,
    EnableEnhancedDeterminism = 1 << 14,
    EnableFrictionEveryIteration = 1 << 15,
    SuppressReadback = 1 << 16,
    ForceReadback = 1 << 17,
    MutableFlags = EnableActiveActors | ExcludeKinematicsFromActiveActors | SuppressReadback,
} PxSceneFlags;


/// Debug visualization parameters.
///
/// [`PxVisualizationParameter::eSCALE`] is the master switch for enabling visualization, please read the corresponding documentation
/// for further details.
typedef enum PxVisualizationParameter: int32_t {
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
} PxVisualizationParameter;

/// Different types of rigid body collision pair statistics.
typedef enum RbPairStatsType: int32_t {
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
} RbPairStatsType;

/// These flags determine what data is read or written to the gpu softbody.
typedef enum PxSoftBodyDataFlag: int32_t {
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
} PxSoftBodyDataFlag;

/// Identifies input and output buffers for PxHairSystem
typedef enum PxHairSystemData: int32_t {
    /// No data specified
    None = 0,
    /// Specifies the position (first 3 floats) and inverse mass (last float) data (array of PxVec4 * max number of vertices)
    PositionInvmass = 1,
    /// Specifies the velocity (first 3 floats) data (array of PxVec4 * max number of vertices)
    Velocity = 2,
    /// Specifies everything
    All = 3,
} PxHairSystemData;

/// Flags for [`PxHairSystemData`]
typedef enum PxHairSystemDataFlags: uint32_t {
    None = 0 << 0,
    PositionInvmass = 1 << 0,
    Velocity = 1 << 1,
    All = PositionInvmass | Velocity,
} PxHairSystemDataFlags;


/// Binary settings for hair system simulation
typedef enum PxHairSystemFlag: int32_t {
    /// Determines if self-collision between hair vertices is ignored
    DisableSelfCollision = 1,
    /// Determines if collision between hair and external bodies is ignored
    DisableExternalCollision = 2,
    /// Determines if attachment constraint is also felt by body to which the hair is attached
    DisableTwosidedAttachment = 4,
} PxHairSystemFlag;

/// Flags for [`PxHairSystemFlag`]
typedef enum PxHairSystemFlags: uint32_t {
    DisableSelfCollision = 1 << 0,
    DisableExternalCollision = 1 << 1,
    DisableTwosidedAttachment = 1 << 2,
} PxHairSystemFlags;


/// Identifies each type of information for retrieving from actor.
typedef enum PxActorCacheFlag: int32_t {
    ActorData = 1,
    Force = 4,
    Torque = 8,
} PxActorCacheFlag;

/// Flags for [`PxActorCacheFlag`]
typedef enum PxActorCacheFlags: uint16_t {
    ActorData = 1 << 0,
    Force = 1 << 2,
    Torque = 1 << 3,
} PxActorCacheFlags;


/// PVD scene Flags. They are disabled by default, and only works if PxPvdInstrumentationFlag::eDEBUG is set.
typedef enum PxPvdSceneFlag: int32_t {
    TransmitContacts = 1,
    /// Transmits contact stream to PVD.
    TransmitScenequeries = 2,
    /// Transmits scene query stream to PVD.
    TransmitConstraints = 4,
} PxPvdSceneFlag;

/// Flags for [`PxPvdSceneFlag`]
typedef enum PxPvdSceneFlags: uint8_t {
    TransmitContacts = 1 << 0,
    TransmitScenequeries = 1 << 1,
    TransmitConstraints = 1 << 2,
} PxPvdSceneFlags;


/// Identifies each type of actor for retrieving actors from a scene.
///
/// [`PxArticulationLink`] objects are not supported. Use the #PxArticulationReducedCoordinate object to retrieve all its links.
typedef enum PxActorTypeFlag: int32_t {
    /// A static rigid body
    RigidStatic = 1,
    /// A dynamic rigid body
    RigidDynamic = 2,
} PxActorTypeFlag;

/// Flags for [`PxActorTypeFlag`]
typedef enum PxActorTypeFlags: uint16_t {
    RigidStatic = 1 << 0,
    RigidDynamic = 1 << 1,
} PxActorTypeFlags;


/// Extra data item types for contact pairs.
typedef enum PxContactPairExtraDataType: int32_t {
    /// see [`PxContactPairVelocity`]
    PreSolverVelocity = 0,
    /// see [`PxContactPairVelocity`]
    PostSolverVelocity = 1,
    /// see [`PxContactPairPose`]
    ContactEventPose = 2,
    /// see [`PxContactPairIndex`]
    ContactPairIndex = 3,
} PxContactPairExtraDataType;

/// Collection of flags providing information on contact report pairs.
typedef enum PxContactPairHeaderFlag: int32_t {
    /// The actor with index 0 has been removed from the scene.
    RemovedActor0 = 1,
    /// The actor with index 1 has been removed from the scene.
    RemovedActor1 = 2,
} PxContactPairHeaderFlag;

/// Flags for [`PxContactPairHeaderFlag`]
typedef enum PxContactPairHeaderFlags: uint16_t {
    RemovedActor0 = 1 << 0,
    RemovedActor1 = 1 << 1,
} PxContactPairHeaderFlags;


/// Collection of flags providing information on contact report pairs.
typedef enum PxContactPairFlag: int32_t {
    /// The shape with index 0 has been removed from the actor/scene.
    RemovedShape0 = 1,
    /// The shape with index 1 has been removed from the actor/scene.
    RemovedShape1 = 2,
    /// First actor pair contact.
    ///
    /// The provided shape pair marks the first contact between the two actors, no other shape pair has been touching prior to the current simulation frame.
    ///
    /// : This info is only available if [`PxPairFlag::eNOTIFY_TOUCH_FOUND`] has been declared for the pair.
    ActorPairHasFirstTouch = 4,
    /// All contact between the actor pair was lost.
    ///
    /// All contact between the two actors has been lost, no shape pairs remain touching after the current simulation frame.
    ActorPairLostTouch = 8,
    /// Internal flag, used by [`PxContactPair`].extractContacts()
    ///
    /// The applied contact impulses are provided for every contact point.
    /// This is the case if [`PxPairFlag::eSOLVE_CONTACT`] has been set for the pair.
    InternalHasImpulses = 16,
    /// Internal flag, used by [`PxContactPair`].extractContacts()
    ///
    /// The provided contact point information is flipped with regards to the shapes of the contact pair. This mainly concerns the order of the internal triangle indices.
    InternalContactsAreFlipped = 32,
} PxContactPairFlag;

/// Flags for [`PxContactPairFlag`]
typedef enum PxContactPairFlags: uint16_t {
    RemovedShape0 = 1 << 0,
    RemovedShape1 = 1 << 1,
    ActorPairHasFirstTouch = 1 << 2,
    ActorPairLostTouch = 1 << 3,
    InternalHasImpulses = 1 << 4,
    InternalContactsAreFlipped = 1 << 5,
} PxContactPairFlags;


/// Collection of flags providing information on trigger report pairs.
typedef enum PxTriggerPairFlag: int32_t {
    /// The trigger shape has been removed from the actor/scene.
    RemovedShapeTrigger = 1,
    /// The shape causing the trigger event has been removed from the actor/scene.
    RemovedShapeOther = 2,
    /// For internal use only.
    NextFree = 4,
} PxTriggerPairFlag;

/// Flags for [`PxTriggerPairFlag`]
typedef enum PxTriggerPairFlags: uint8_t {
    RemovedShapeTrigger = 1 << 0,
    RemovedShapeOther = 1 << 1,
    NextFree = 1 << 2,
} PxTriggerPairFlags;


/// Identifies input and output buffers for PxSoftBody.
typedef enum PxSoftBodyData: int32_t {
    None = 0,
    /// Flag to request access to the collision mesh's positions; read only
    PositionInvmass = 1,
    /// Flag to request access to the simulation mesh's positions and inverse masses
    SimPositionInvmass = 4,
    /// Flag to request access to the simulation mesh's velocities and inverse masses
    SimVelocity = 8,
    /// Flag to request access to the simulation mesh's kinematic target position
    SimKinematicTarget = 16,
    All = 29,
} PxSoftBodyData;

/// Flags for [`PxSoftBodyData`]
typedef enum PxSoftBodyDataFlags: uint32_t {
    None = 0 << 0,
    PositionInvmass = 1 << 0,
    SimPositionInvmass = 1 << 2,
    SimVelocity = 1 << 3,
    SimKinematicTarget = 1 << 4,
    All = PositionInvmass | SimPositionInvmass | SimVelocity | SimKinematicTarget,
} PxSoftBodyDataFlags;


/// Flags to enable or disable special modes of a SoftBody
typedef enum PxSoftBodyFlag: int32_t {
    /// Determines if self collision will be detected and resolved
    DisableSelfCollision = 1,
    /// Enables computation of a Cauchy stress tensor for every tetrahedron in the simulation mesh. The tensors can be accessed through the softbody direct API
    ComputeStressTensor = 2,
    /// Enables support for continuous collision detection
    EnableCcd = 4,
    /// Enable debug rendering to display the simulation mesh
    DisplaySimMesh = 8,
    /// Enables support for kinematic motion of the collision and simulation mesh.
    Kinematic = 16,
    /// Enables partially kinematic motion of the collisios and simulation mesh.
    PartiallyKinematic = 32,
} PxSoftBodyFlag;

/// Flags for [`PxSoftBodyFlag`]
typedef enum PxSoftBodyFlags: uint32_t {
    DisableSelfCollision = 1 << 0,
    ComputeStressTensor = 1 << 1,
    EnableCcd = 1 << 2,
    DisplaySimMesh = 1 << 3,
    Kinematic = 1 << 4,
    PartiallyKinematic = 1 << 5,
} PxSoftBodyFlags;


/// The type of controller, eg box, sphere or capsule.
typedef enum PxControllerShapeType: int32_t {
    /// A box controller.
    Box = 0,
    /// A capsule controller
    Capsule = 1,
    /// A capsule controller
    ForceDword = 2147483647,
} PxControllerShapeType;

/// specifies how a CCT interacts with non-walkable parts.
///
/// This is only used when slopeLimit is non zero. It is currently enabled for static actors only, and not supported for spheres or capsules.
typedef enum PxControllerNonWalkableMode: int32_t {
    /// Stops character from climbing up non-walkable slopes, but doesn't move it otherwise
    PreventClimbing = 0,
    /// Stops character from climbing up non-walkable slopes, and forces it to slide down those slopes
    PreventClimbingAndForceSliding = 1,
} PxControllerNonWalkableMode;

/// specifies which sides a character is colliding with.
typedef enum PxControllerCollisionFlag: int32_t {
    /// Character is colliding to the sides.
    CollisionSides = 1,
    /// Character has collision above.
    CollisionUp = 2,
    /// Character has collision below.
    CollisionDown = 4,
} PxControllerCollisionFlag;

/// Flags for [`PxControllerCollisionFlag`]
typedef enum PxControllerCollisionFlags: uint8_t {
    CollisionSides = 1 << 0,
    CollisionUp = 1 << 1,
    CollisionDown = 1 << 2,
} PxControllerCollisionFlags;


typedef enum PxCapsuleClimbingMode: int32_t {
    /// Standard mode, let the capsule climb over surfaces according to impact normal
    Easy = 0,
    /// Constrained mode, try to limit climbing according to the step offset
    Constrained = 1,
    Last = 2,
} PxCapsuleClimbingMode;

/// specifies controller behavior
typedef enum PxControllerBehaviorFlag: int32_t {
    /// Controller can ride on touched object (i.e. when this touched object is moving horizontally).
    ///
    /// The CCT vs. CCT case is not supported.
    CctCanRideOnObject = 1,
    /// Controller should slide on touched object
    CctSlide = 2,
    /// Disable all code dealing with controllers riding on objects, let users define it outside of the SDK.
    CctUserDefinedRide = 4,
} PxControllerBehaviorFlag;

/// Flags for [`PxControllerBehaviorFlag`]
typedef enum PxControllerBehaviorFlags: uint8_t {
    CctCanRideOnObject = 1 << 0,
    CctSlide = 1 << 1,
    CctUserDefinedRide = 1 << 2,
} PxControllerBehaviorFlags;


/// specifies debug-rendering flags
typedef enum PxControllerDebugRenderFlag: uint32_t {
    /// Temporal bounding volume around controllers
    TemporalBv = 1,
    /// Cached bounding volume around controllers
    CachedBv = 2,
    /// User-defined obstacles
    Obstacles = 4,
    None = 0,
    All = 4294967295,
} PxControllerDebugRenderFlag;

/// Flags for [`PxControllerDebugRenderFlag`]
typedef enum PxControllerDebugRenderFlags: uint32_t {
    TemporalBv = 1 << 0,
    CachedBv = 1 << 1,
    Obstacles = 1 << 2,
    None = 0 << 0,
    All = TemporalBv | CachedBv | Obstacles,
} PxControllerDebugRenderFlags;


/// Defines the number of bits per subgrid pixel
typedef enum PxSdfBitsPerSubgridPixel: int32_t {
    /// 8 bit per subgrid pixel (values will be stored as normalized integers)
    E8BitPerPixel = 1,
    /// 16 bit per subgrid pixel (values will be stored as normalized integers)
    E16BitPerPixel = 2,
    /// 32 bit per subgrid pixel (values will be stored as floats in world scale units)
    E32BitPerPixel = 4,
} PxSdfBitsPerSubgridPixel;

/// Flags which describe the format and behavior of a convex mesh.
typedef enum PxConvexFlag: int32_t {
    /// Denotes the use of 16-bit vertex indices in PxConvexMeshDesc::triangles or PxConvexMeshDesc::polygons.
    /// (otherwise, 32-bit indices are assumed)
    E16BitIndices = 1,
    /// Automatically recomputes the hull from the vertices. If this flag is not set, you must provide the entire geometry manually.
    ///
    /// There are two different algorithms for hull computation, please see PxConvexMeshCookingType.
    ComputeConvex = 2,
    /// Checks and removes almost zero-area triangles during convex hull computation.
    /// The rejected area size is specified in PxCookingParams::areaTestEpsilon
    ///
    /// This flag is only used in combination with eCOMPUTE_CONVEX.
    CheckZeroAreaTriangles = 4,
    /// Quantizes the input vertices using the k-means clustering
    ///
    /// The input vertices are quantized to PxConvexMeshDesc::quantizedCount
    /// see http://en.wikipedia.org/wiki/K-means_clustering
    QuantizeInput = 8,
    /// Disables the convex mesh validation to speed-up hull creation. Please use separate validation
    /// function in checked/debug builds. Creating a convex mesh with invalid input data without prior validation
    /// may result in undefined behavior.
    DisableMeshValidation = 16,
    /// Enables plane shifting vertex limit algorithm.
    ///
    /// Plane shifting is an alternative algorithm for the case when the computed hull has more vertices
    /// than the specified vertex limit.
    ///
    /// The default algorithm computes the full hull, and an OBB around the input vertices. This OBB is then sliced
    /// with the hull planes until the vertex limit is reached.The default algorithm requires the vertex limit
    /// to be set to at least 8, and typically produces results that are much better quality than are produced
    /// by plane shifting.
    ///
    /// When plane shifting is enabled, the hull computation stops when vertex limit is reached. The hull planes
    /// are then shifted to contain all input vertices, and the new plane intersection points are then used to
    /// generate the final hull with the given vertex limit.Plane shifting may produce sharp edges to vertices
    /// very far away from the input cloud, and does not guarantee that all input vertices are inside the resulting
    /// hull.However, it can be used with a vertex limit as low as 4.
    PlaneShifting = 32,
    /// Inertia tensor computation is faster using SIMD code, but the precision is lower, which may result
    /// in incorrect inertia for very thin hulls.
    FastInertiaComputation = 64,
    /// Convex hulls are created with respect to GPU simulation limitations. Vertex limit and polygon limit
    /// is set to 64 and vertex limit per face is internally set to 32.
    ///
    /// Can be used only with eCOMPUTE_CONVEX flag.
    GpuCompatible = 128,
    /// Convex hull input vertices are shifted to be around origin to provide better computation stability.
    /// It is recommended to provide input vertices around the origin, otherwise use this flag to improve
    /// numerical stability.
    ///
    /// Is used only with eCOMPUTE_CONVEX flag.
    ShiftVertices = 256,
} PxConvexFlag;

/// Flags for [`PxConvexFlag`]
typedef enum PxConvexFlags: uint16_t {
    E16BitIndices = 1 << 0,
    ComputeConvex = 1 << 1,
    CheckZeroAreaTriangles = 1 << 2,
    QuantizeInput = 1 << 3,
    DisableMeshValidation = 1 << 4,
    PlaneShifting = 1 << 5,
    FastInertiaComputation = 1 << 6,
    GpuCompatible = 1 << 7,
    ShiftVertices = 1 << 8,
} PxConvexFlags;


/// Defines the tetrahedron structure of a mesh.
typedef enum PxMeshFormat: int32_t {
    /// Normal tetmesh with arbitrary tetrahedra
    TetMesh = 0,
    /// 6 tetrahedra in a row will form a hexahedron
    HexMesh = 1,
} PxMeshFormat;

/// Desired build strategy for PxMeshMidPhase::eBVH34
typedef enum PxBVH34BuildStrategy: int32_t {
    /// Fast build strategy. Fast build speed, good runtime performance in most cases. Recommended for runtime mesh cooking.
    Fast = 0,
    /// Default build strategy. Medium build speed, good runtime performance in all cases.
    Default = 1,
    /// SAH build strategy. Slower builds, slightly improved runtime performance in some cases.
    Sah = 2,
    Last = 3,
} PxBVH34BuildStrategy;

/// Result from convex cooking.
typedef enum PxConvexMeshCookingResult: int32_t {
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
} PxConvexMeshCookingResult;

/// Enumeration for convex mesh cooking algorithms.
typedef enum PxConvexMeshCookingType: int32_t {
    /// The Quickhull algorithm constructs the hull from the given input points. The resulting hull
    /// will only contain a subset of the input points.
    Quickhull = 0,
} PxConvexMeshCookingType;

/// Result from triangle mesh cooking
typedef enum PxTriangleMeshCookingResult: int32_t {
    /// Everything is A-OK.
    Success = 0,
    /// a triangle is too large for well-conditioned results. Tessellate the mesh for better behavior, see the user guide section on cooking for more details.
    LargeTriangle = 1,
    /// Something unrecoverable happened. Check the error stream to find out what.
    Failure = 2,
} PxTriangleMeshCookingResult;

/// Enum for the set of mesh pre-processing parameters.
typedef enum PxMeshPreprocessingFlag: int32_t {
    /// When set, mesh welding is performed. See PxCookingParams::meshWeldTolerance. Clean mesh must be enabled.
    WeldVertices = 1,
    /// When set, mesh cleaning is disabled. This makes cooking faster.
    ///
    /// When clean mesh is not performed, mesh welding is also not performed.
    ///
    /// It is recommended to use only meshes that passed during validateTriangleMesh.
    DisableCleanMesh = 2,
    /// When set, active edges are set for each triangle edge. This makes cooking faster but slow up contact generation.
    DisableActiveEdgesPrecompute = 4,
    /// When set, 32-bit indices will always be created regardless of triangle count.
    ///
    /// By default mesh will be created with 16-bit indices for triangle count
    /// <
    /// = 0xFFFF and 32-bit otherwise.
    Force32bitIndices = 8,
    /// When set, a list of triangles will be created for each associated vertex in the mesh
    EnableVertMapping = 16,
    /// When set, inertia tensor is calculated for the mesh
    EnableInertia = 32,
} PxMeshPreprocessingFlag;

/// Flags for [`PxMeshPreprocessingFlag`]
typedef enum PxMeshPreprocessingFlags: uint32_t {
    WeldVertices = 1 << 0,
    DisableCleanMesh = 1 << 1,
    DisableActiveEdgesPrecompute = 1 << 2,
    Force32bitIndices = 1 << 3,
    EnableVertMapping = 1 << 4,
    EnableInertia = 1 << 5,
} PxMeshPreprocessingFlags;


/// Unique identifiers for extensions classes which implement a constraint based on PxConstraint.
///
/// Users which want to create their own custom constraint types should choose an ID larger or equal to eNEXT_FREE_ID
/// and not eINVALID_ID.
typedef enum PxConstraintExtIDs: int32_t {
    Joint = 0,
    VehicleSuspLimitDeprecated = 1,
    VehicleStickyTyreDeprecated = 2,
    VehicleJoint = 3,
    NextFreeId = 4,
    InvalidId = 2147483647,
} PxConstraintExtIDs;

/// an enumeration of PhysX' built-in joint types
typedef enum PxJointConcreteType: int32_t {
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
} PxJointConcreteType;

/// an enumeration for specifying one or other of the actors referenced by a joint
typedef enum PxJointActorIndex: int32_t {
    Actor0 = 0,
    Actor1 = 1,
    Count = 2,
} PxJointActorIndex;

/// flags for configuring the drive of a PxDistanceJoint
typedef enum PxDistanceJointFlag: int32_t {
    MaxDistanceEnabled = 2,
    MinDistanceEnabled = 4,
    SpringEnabled = 8,
} PxDistanceJointFlag;

/// Flags for [`PxDistanceJointFlag`]
typedef enum PxDistanceJointFlags: uint16_t {
    MaxDistanceEnabled = 1 << 1,
    MinDistanceEnabled = 1 << 2,
    SpringEnabled = 1 << 3,
} PxDistanceJointFlags;


/// Flags specific to the prismatic joint.
typedef enum PxPrismaticJointFlag: int32_t {
    LimitEnabled = 2,
} PxPrismaticJointFlag;

/// Flags for [`PxPrismaticJointFlag`]
typedef enum PxPrismaticJointFlags: uint16_t {
    LimitEnabled = 1 << 1,
} PxPrismaticJointFlags;


/// Flags specific to the Revolute Joint.
typedef enum PxRevoluteJointFlag: int32_t {
    /// enable the limit
    LimitEnabled = 1,
    /// enable the drive
    DriveEnabled = 2,
    /// if the existing velocity is beyond the drive velocity, do not add force
    DriveFreespin = 4,
} PxRevoluteJointFlag;

/// Flags for [`PxRevoluteJointFlag`]
typedef enum PxRevoluteJointFlags: uint16_t {
    LimitEnabled = 1 << 0,
    DriveEnabled = 1 << 1,
    DriveFreespin = 1 << 2,
} PxRevoluteJointFlags;


/// Flags specific to the spherical joint.
typedef enum PxSphericalJointFlag: int32_t {
    /// the cone limit for the spherical joint is enabled
    LimitEnabled = 2,
} PxSphericalJointFlag;

/// Flags for [`PxSphericalJointFlag`]
typedef enum PxSphericalJointFlags: uint16_t {
    LimitEnabled = 1 << 1,
} PxSphericalJointFlags;


/// Used to specify one of the degrees of freedom of  a D6 joint.
typedef enum PxD6Axis: int32_t {
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
} PxD6Axis;

/// Used to specify the range of motions allowed for a degree of freedom in a D6 joint.
typedef enum PxD6Motion: int32_t {
    /// The DOF is locked, it does not allow relative motion.
    Locked = 0,
    /// The DOF is limited, it only allows motion within a specific range.
    Limited = 1,
    /// The DOF is free and has its full range of motion.
    Free = 2,
} PxD6Motion;

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
typedef enum PxD6Drive: int32_t {
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
} PxD6Drive;

impl From<size_t> for PxD6Drive {
    fn from(val: size_t) -> Self {
        #[allow(clippy::match_same_arms)]
        match val {
            0 => Self::X,
            1 => Self::Y,
            2 => Self::Z,
            3 => Self::Swing,
            4 => Self::Twist,
            5 => Self::Slerp,
            6 => Self::Count,
            _ => Self::Count,
        }
    }
}

/// flags for configuring the drive model of a PxD6Joint
typedef enum PxD6JointDriveFlag: int32_t {
    /// drive spring is for the acceleration at the joint (rather than the force)
    Acceleration = 1,
} PxD6JointDriveFlag;

/// Flags for [`PxD6JointDriveFlag`]
typedef enum PxD6JointDriveFlags: uint32_t {
    Acceleration = 1 << 0,
} PxD6JointDriveFlags;


/// Collision filtering operations.
typedef enum PxFilterOp: int32_t {
    PxFilteropAnd = 0,
    PxFilteropOr = 1,
    PxFilteropXor = 2,
    PxFilteropNand = 3,
    PxFilteropNor = 4,
    PxFilteropNxor = 5,
    PxFilteropSwapAnd = 6,
} PxFilterOp;

/// If a thread ends up waiting for work it will find itself in a spin-wait loop until work becomes available.
/// Three strategies are available to limit wasted cycles.
/// The strategies are as follows:
/// a) wait until a work task signals the end of the spin-wait period.
/// b) yield the thread by providing a hint to reschedule thread execution, thereby allowing other threads to run.
/// c) yield the processor by informing it that it is waiting for work and requesting it to more efficiently use compute resources.
typedef enum PxDefaultCpuDispatcherWaitForWorkMode: int32_t {
    WaitForWork = 0,
    YieldThread = 1,
    YieldProcessor = 2,
} PxDefaultCpuDispatcherWaitForWorkMode;

typedef enum PxBatchQueryStatus: int32_t {
    /// This is the initial state before a query starts.
    Pending = 0,
    /// The query is finished; results have been written into the result and hit buffers.
    Success = 1,
    /// The query results were incomplete due to touch hit buffer overflow. Blocking hit is still correct.
    Overflow = 2,
} PxBatchQueryStatus;

/// types of instrumentation that PVD can do.
typedef enum PxPvdInstrumentationFlag: int32_t {
    /// Send debugging information to PVD.
    ///
    /// This information is the actual object data of the rigid statics, shapes,
    /// articulations, etc.  Sending this information has a noticeable impact on
    /// performance and thus this flag should not be set if you want an accurate
    /// performance profile.
    Debug = 1,
    /// Send profile information to PVD.
    ///
    /// This information populates PVD's profile view.  It has (at this time) negligible
    /// cost compared to Debug information and makes PVD *much* more useful so it is quite
    /// highly recommended.
    ///
    /// This flag works together with a PxCreatePhysics parameter.
    /// Using it allows the SDK to send profile events to PVD.
    Profile = 2,
    /// Send memory information to PVD.
    ///
    /// The PVD sdk side hooks into the Foundation memory controller and listens to
    /// allocation/deallocation events.  This has a noticable hit on the first frame,
    /// however, this data is somewhat compressed and the PhysX SDK doesn't allocate much
    /// once it hits a steady state.  This information also has a fairly negligible
    /// impact and thus is also highly recommended.
    ///
    /// This flag works together with a PxCreatePhysics parameter,
    /// trackOutstandingAllocations.  Using both of them together allows users to have
    /// an accurate view of the overall memory usage of the simulation at the cost of
    /// a hashtable lookup per allocation/deallocation.  Again, PhysX makes a best effort
    /// attempt not to allocate or deallocate during simulation so this hashtable lookup
    /// tends to have no effect past the first frame.
    ///
    /// Sending memory information without tracking outstanding allocations means that
    /// PVD will accurate information about the state of the memory system before the
    /// actual connection happened.
    Memory = 4,
    /// Send memory information to PVD.
    ///
    /// The PVD sdk side hooks into the Foundation memory controller and listens to
    /// allocation/deallocation events.  This has a noticable hit on the first frame,
    /// however, this data is somewhat compressed and the PhysX SDK doesn't allocate much
    /// once it hits a steady state.  This information also has a fairly negligible
    /// impact and thus is also highly recommended.
    ///
    /// This flag works together with a PxCreatePhysics parameter,
    /// trackOutstandingAllocations.  Using both of them together allows users to have
    /// an accurate view of the overall memory usage of the simulation at the cost of
    /// a hashtable lookup per allocation/deallocation.  Again, PhysX makes a best effort
    /// attempt not to allocate or deallocate during simulation so this hashtable lookup
    /// tends to have no effect past the first frame.
    ///
    /// Sending memory information without tracking outstanding allocations means that
    /// PVD will accurate information about the state of the memory system before the
    /// actual connection happened.
    All = 7,
} PxPvdInstrumentationFlag;

/// Flags for [`PxPvdInstrumentationFlag`]
typedef enum PxPvdInstrumentationFlags: uint8_t {
    Debug = 1 << 0,
    Profile = 1 << 1,
    Memory = 1 << 2,
    All = Debug | Profile | Memory,
} PxPvdInstrumentationFlags;


typedef struct PxMat34 {
    _unused: [u8; 0],
} PxMat34;

typedef struct PxAllocatorCallback {
    vtable_: *const std::ffi::c_void,
} PxAllocatorCallback;

typedef struct PxAssertHandler {
    vtable_: *const std::ffi::c_void,
} PxAssertHandler;

typedef struct PxFoundation {
    vtable_: *const std::ffi::c_void,
} PxFoundation;

typedef struct PxVirtualAllocatorCallback {
    vtable_: *const std::ffi::c_void,
} PxVirtualAllocatorCallback;

typedef union PxTempAllocatorChunk {
    mNext: physx_PxTempAllocatorChunk_Pod*,
    mIndex: uint32_t,
    mPad: uint8_t[16],
} PxTempAllocatorChunk;

typedef struct PxLogTwo {
    _unused: [u8; 0],
} PxLogTwo;

typedef struct PxUnConst {
    _unused: [u8; 0],
} PxUnConst;

typedef struct PxErrorCallback {
    vtable_: *const std::ffi::c_void,
} PxErrorCallback;

typedef struct PxAllocationListener {
    vtable_: *const std::ffi::c_void,
} PxAllocationListener;

typedef struct PxHash {
    _unused: [u8; 0],
} PxHash;

typedef struct PxInputStream {
    vtable_: *const std::ffi::c_void,
} PxInputStream;

typedef struct PxInputData {
    vtable_: *const std::ffi::c_void,
} PxInputData;

typedef struct PxOutputStream {
    vtable_: *const std::ffi::c_void,
} PxOutputStream;

typedef struct PxProfilerCallback {
    vtable_: *const std::ffi::c_void,
} PxProfilerCallback;

typedef struct PxRunnable {
    vtable_: *const std::ffi::c_void,
} PxRunnable;

typedef struct PxRenderBuffer {
    vtable_: *const std::ffi::c_void,
} PxRenderBuffer;

typedef struct PxProcessPxBaseCallback {
    vtable_: *const std::ffi::c_void,
} PxProcessPxBaseCallback;

typedef struct PxSerializationContext {
    vtable_: *const std::ffi::c_void,
} PxSerializationContext;

typedef struct PxSerializationRegistry {
    vtable_: *const std::ffi::c_void,
} PxSerializationRegistry;

typedef struct PxCollection {
    vtable_: *const std::ffi::c_void,
} PxCollection;

typedef struct PxTypeInfo {
    _unused: [u8; 0],
} PxTypeInfo;

typedef struct PxFEMSoftBodyMaterial {
    _unused: [u8; 0],
} PxFEMSoftBodyMaterial;

typedef struct PxFEMClothMaterial {
    _unused: [u8; 0],
} PxFEMClothMaterial;

typedef struct PxPBDMaterial {
    _unused: [u8; 0],
} PxPBDMaterial;

typedef struct PxFLIPMaterial {
    _unused: [u8; 0],
} PxFLIPMaterial;

typedef struct PxMPMMaterial {
    _unused: [u8; 0],
} PxMPMMaterial;

typedef struct PxCustomMaterial {
    _unused: [u8; 0],
} PxCustomMaterial;

typedef struct PxBVH33TriangleMesh {
    _unused: [u8; 0],
} PxBVH33TriangleMesh;

typedef struct PxParticleSystem {
    _unused: [u8; 0],
} PxParticleSystem;

typedef struct PxPBDParticleSystem {
    _unused: [u8; 0],
} PxPBDParticleSystem;

typedef struct PxFLIPParticleSystem {
    _unused: [u8; 0],
} PxFLIPParticleSystem;

typedef struct PxMPMParticleSystem {
    _unused: [u8; 0],
} PxMPMParticleSystem;

typedef struct PxCustomParticleSystem {
    _unused: [u8; 0],
} PxCustomParticleSystem;

typedef struct PxSoftBody {
    _unused: [u8; 0],
} PxSoftBody;

typedef struct PxFEMCloth {
    _unused: [u8; 0],
} PxFEMCloth;

typedef struct PxHairSystem {
    _unused: [u8; 0],
} PxHairSystem;

typedef struct PxParticleBuffer {
    _unused: [u8; 0],
} PxParticleBuffer;

typedef struct PxParticleAndDiffuseBuffer {
    _unused: [u8; 0],
} PxParticleAndDiffuseBuffer;

typedef struct PxParticleClothBuffer {
    _unused: [u8; 0],
} PxParticleClothBuffer;

typedef struct PxParticleRigidBuffer {
    _unused: [u8; 0],
} PxParticleRigidBuffer;

typedef struct PxStringTable {
    vtable_: *const std::ffi::c_void,
} PxStringTable;

typedef struct PxSerializer {
    vtable_: *const std::ffi::c_void,
} PxSerializer;

typedef struct PxInsertionCallback {
    vtable_: *const std::ffi::c_void,
} PxInsertionCallback;

typedef struct PxTaskManager {
    vtable_: *const std::ffi::c_void,
} PxTaskManager;

typedef struct PxCpuDispatcher {
    vtable_: *const std::ffi::c_void,
} PxCpuDispatcher;

typedef struct PxBVHRaycastCallback {
    vtable_: *const std::ffi::c_void,
} PxBVHRaycastCallback;

typedef struct PxBVHOverlapCallback {
    vtable_: *const std::ffi::c_void,
} PxBVHOverlapCallback;

typedef struct PxBVHTraversalCallback {
    vtable_: *const std::ffi::c_void,
} PxBVHTraversalCallback;

typedef struct PxContactBuffer {
    _unused: [u8; 0],
} PxContactBuffer;

typedef struct PxRenderOutput {
    _unused: [u8; 0],
} PxRenderOutput;

typedef struct PxCustomGeometryCallbacks {
    vtable_: *const std::ffi::c_void,
} PxCustomGeometryCallbacks;

typedef union Px1DConstraintMods {
    spring: physx_PxSpringModifiers_Pod,
    bounce: physx_PxRestitutionModifiers_Pod,
} Px1DConstraintMods;

typedef struct PxConstraintVisualizer {
    vtable_: *const std::ffi::c_void,
} PxConstraintVisualizer;

typedef struct PxConstraintConnector {
    vtable_: *const std::ffi::c_void,
} PxConstraintConnector;

typedef struct PxConstraintAllocator {
    vtable_: *const std::ffi::c_void,
} PxConstraintAllocator;

typedef struct PxContactModifyCallback {
    vtable_: *const std::ffi::c_void,
} PxContactModifyCallback;

typedef struct PxCCDContactModifyCallback {
    vtable_: *const std::ffi::c_void,
} PxCCDContactModifyCallback;

typedef struct PxDeletionListener {
    vtable_: *const std::ffi::c_void,
} PxDeletionListener;

typedef struct PxSimulationFilterCallback {
    vtable_: *const std::ffi::c_void,
} PxSimulationFilterCallback;

typedef struct PxLockedData {
    vtable_: *const std::ffi::c_void,
} PxLockedData;

typedef struct PxCudaContextManager {
    _unused: [u8; 0],
} PxCudaContextManager;

typedef struct PxParticleRigidAttachment {
    _unused: [u8; 0],
} PxParticleRigidAttachment;

typedef struct PxOmniPvd {
    _unused: [u8; 0],
} PxOmniPvd;

typedef struct PxPhysics {
    vtable_: *const std::ffi::c_void,
} PxPhysics;

typedef struct PxQueryFilterCallback {
    vtable_: *const std::ffi::c_void,
} PxQueryFilterCallback;

typedef struct PxSceneQuerySystemBase {
    vtable_: *const std::ffi::c_void,
} PxSceneQuerySystemBase;

typedef struct PxSceneSQSystem {
    vtable_: *const std::ffi::c_void,
} PxSceneSQSystem;

typedef struct PxSceneQuerySystem {
    vtable_: *const std::ffi::c_void,
} PxSceneQuerySystem;

typedef struct PxBroadPhaseRegions {
    vtable_: *const std::ffi::c_void,
} PxBroadPhaseRegions;

typedef struct PxBroadPhase {
    vtable_: *const std::ffi::c_void,
} PxBroadPhase;

typedef struct PxAABBManager {
    vtable_: *const std::ffi::c_void,
} PxAABBManager;

typedef struct PxPvdSceneClient {
    vtable_: *const std::ffi::c_void,
} PxPvdSceneClient;

typedef struct PxBroadPhaseCallback {
    vtable_: *const std::ffi::c_void,
} PxBroadPhaseCallback;

typedef struct PxSimulationEventCallback {
    vtable_: *const std::ffi::c_void,
} PxSimulationEventCallback;

typedef struct PxObstacleContext {
    vtable_: *const std::ffi::c_void,
} PxObstacleContext;

typedef struct PxUserControllerHitReport {
    vtable_: *const std::ffi::c_void,
} PxUserControllerHitReport;

typedef struct PxControllerFilterCallback {
    vtable_: *const std::ffi::c_void,
} PxControllerFilterCallback;

typedef struct PxController {
    vtable_: *const std::ffi::c_void,
} PxController;

typedef struct PxBoxController {
    vtable_: *const std::ffi::c_void,
} PxBoxController;

typedef struct PxCapsuleController {
    vtable_: *const std::ffi::c_void,
} PxCapsuleController;

typedef struct PxControllerBehaviorCallback {
    vtable_: *const std::ffi::c_void,
} PxControllerBehaviorCallback;

typedef struct PxControllerManager {
    vtable_: *const std::ffi::c_void,
} PxControllerManager;

typedef struct PxDefaultAllocator {
    vtable_: *const std::ffi::c_void,
} PxDefaultAllocator;

typedef struct PxDefaultErrorCallback {
    vtable_: *const std::ffi::c_void,
} PxDefaultErrorCallback;

typedef struct PxBinaryConverter {
    _unused: [u8; 0],
} PxBinaryConverter;

typedef struct PxDefaultCpuDispatcher {
    vtable_: *const std::ffi::c_void,
} PxDefaultCpuDispatcher;

typedef struct PxBatchQueryExt {
    vtable_: *const std::ffi::c_void,
} PxBatchQueryExt;

typedef struct PxCustomSceneQuerySystem {
    vtable_: *const std::ffi::c_void,
} PxCustomSceneQuerySystem;

typedef struct PxCustomSceneQuerySystemAdapter {
    vtable_: *const std::ffi::c_void,
} PxCustomSceneQuerySystemAdapter;

typedef struct PxCooking {
    _unused: [u8; 0],
} PxCooking;

typedef struct XmlMemoryAllocator {
    _unused: [u8; 0],
} XmlMemoryAllocator;

typedef struct XmlWriter {
    _unused: [u8; 0],
} XmlWriter;

typedef struct XmlReader {
    _unused: [u8; 0],
} XmlReader;

typedef struct MemoryBuffer {
    _unused: [u8; 0],
} MemoryBuffer;

typedef struct PxRepXSerializer {
    vtable_: *const std::ffi::c_void,
} PxRepXSerializer;

typedef struct PxVehicleWheels4SimData {
    _unused: [u8; 0],
} PxVehicleWheels4SimData;

typedef struct PxVehicleWheels4DynData {
    _unused: [u8; 0],
} PxVehicleWheels4DynData;

typedef struct PxVehicleTireForceCalculator {
    _unused: [u8; 0],
} PxVehicleTireForceCalculator;

typedef struct PxVehicleDrivableSurfaceToTireFrictionPairs {
    _unused: [u8; 0],
} PxVehicleDrivableSurfaceToTireFrictionPairs;

typedef struct PxVehicleTelemetryData {
    _unused: [u8; 0],
} PxVehicleTelemetryData;

typedef struct PxPvd {
    vtable_: *const std::ffi::c_void,
} PxPvd;

typedef struct PxPvdTransport {
    vtable_: *const std::ffi::c_void,
} PxPvdTransport;
