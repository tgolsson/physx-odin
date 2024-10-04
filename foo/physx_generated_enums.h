#include <stdint.h>
/// enum for empty constructor tag
typedef enum PxEMPTY: int32_t {
    PxEMPTY_PxEmpty = 0,
} PxEMPTY;

/// enum for zero constructor tag for vectors and matrices
typedef enum PxZERO: int32_t {
    PxZERO_PxZero = 0,
} PxZERO;

/// enum for identity constructor flag for quaternions, transforms, and matrices
typedef enum PxIDENTITY: int32_t {
    PxIDENTITY_PxIdentity = 0,
} PxIDENTITY;

/// Error codes
///
/// These error codes are passed to [`PxErrorCallback`]
typedef enum PxErrorCode: int32_t {
    PxErrorCode_NoError = 0,
    /// An informational message.
    PxErrorCode_DebugInfo = 1,
    /// a warning message for the user to help with debugging
    PxErrorCode_DebugWarning = 2,
    /// method called with invalid parameter(s)
    PxErrorCode_InvalidParameter = 4,
    /// method was called at a time when an operation is not possible
    PxErrorCode_InvalidOperation = 8,
    /// method failed to allocate some memory
    PxErrorCode_OutOfMemory = 16,
    /// The library failed for some reason.
    /// Possibly you have passed invalid values like NaNs, which are not checked for.
    PxErrorCode_InternalError = 32,
    /// An unrecoverable error, execution should be halted and log output flushed
    PxErrorCode_Abort = 64,
    /// The SDK has determined that an operation may result in poor performance.
    PxErrorCode_PerfWarning = 128,
    /// A bit mask for including all errors
    PxErrorCode_MaskAll = -1,
} PxErrorCode;

typedef enum PxThreadPriority: uint32_t {
    /// High priority
    PxThreadPriority_High = 0,
    /// Above Normal priority
    PxThreadPriority_AboveNormal = 1,
    /// Normal/default priority
    PxThreadPriority_Normal = 2,
    /// Below Normal priority
    PxThreadPriority_BelowNormal = 3,
    /// Low priority.
    PxThreadPriority_Low = 4,
    PxThreadPriority_ForceDword = 4294967295,
} PxThreadPriority;

/// Default color values used for debug rendering.
typedef enum PxDebugColor: uint32_t {
    PxDebugColor_ArgbBlack = 4278190080,
    PxDebugColor_ArgbRed = 4294901760,
    PxDebugColor_ArgbGreen = 4278255360,
    PxDebugColor_ArgbBlue = 4278190335,
    PxDebugColor_ArgbYellow = 4294967040,
    PxDebugColor_ArgbMagenta = 4294902015,
    PxDebugColor_ArgbCyan = 4278255615,
    PxDebugColor_ArgbWhite = 4294967295,
    PxDebugColor_ArgbGrey = 4286611584,
    PxDebugColor_ArgbDarkred = 4287102976,
    PxDebugColor_ArgbDarkgreen = 4278224896,
    PxDebugColor_ArgbDarkblue = 4278190216,
} PxDebugColor;

/// an enumeration of concrete classes inheriting from PxBase
///
/// Enumeration space is reserved for future PhysX core types, PhysXExtensions,
/// PhysXVehicle and Custom application types.
typedef enum PxConcreteType: int32_t {
    PxConcreteType_Undefined = 0,
    PxConcreteType_Heightfield = 1,
    PxConcreteType_ConvexMesh = 2,
    PxConcreteType_TriangleMeshBvh33 = 3,
    PxConcreteType_TriangleMeshBvh34 = 4,
    PxConcreteType_TetrahedronMesh = 5,
    PxConcreteType_SoftbodyMesh = 6,
    PxConcreteType_RigidDynamic = 7,
    PxConcreteType_RigidStatic = 8,
    PxConcreteType_Shape = 9,
    PxConcreteType_Material = 10,
    PxConcreteType_SoftbodyMaterial = 11,
    PxConcreteType_ClothMaterial = 12,
    PxConcreteType_PbdMaterial = 13,
    PxConcreteType_FlipMaterial = 14,
    PxConcreteType_MpmMaterial = 15,
    PxConcreteType_CustomMaterial = 16,
    PxConcreteType_Constraint = 17,
    PxConcreteType_Aggregate = 18,
    PxConcreteType_ArticulationReducedCoordinate = 19,
    PxConcreteType_ArticulationLink = 20,
    PxConcreteType_ArticulationJointReducedCoordinate = 21,
    PxConcreteType_ArticulationSensor = 22,
    PxConcreteType_ArticulationSpatialTendon = 23,
    PxConcreteType_ArticulationFixedTendon = 24,
    PxConcreteType_ArticulationAttachment = 25,
    PxConcreteType_ArticulationTendonJoint = 26,
    PxConcreteType_PruningStructure = 27,
    PxConcreteType_Bvh = 28,
    PxConcreteType_SoftBody = 29,
    PxConcreteType_SoftBodyState = 30,
    PxConcreteType_PbdParticlesystem = 31,
    PxConcreteType_FlipParticlesystem = 32,
    PxConcreteType_MpmParticlesystem = 33,
    PxConcreteType_CustomParticlesystem = 34,
    PxConcreteType_FemCloth = 35,
    PxConcreteType_HairSystem = 36,
    PxConcreteType_ParticleBuffer = 37,
    PxConcreteType_ParticleDiffuseBuffer = 38,
    PxConcreteType_ParticleClothBuffer = 39,
    PxConcreteType_ParticleRigidBuffer = 40,
    PxConcreteType_PhysxCoreCount = 41,
    PxConcreteType_FirstPhysxExtension = 256,
    PxConcreteType_FirstVehicleExtension = 512,
    PxConcreteType_FirstUserExtension = 1024,
} PxConcreteType;


/// Flags for PxBase.
typedef enum PxBaseFlag: int32_t {
    PxBaseFlag_OwnsMemory = 1,
    PxBaseFlag_IsReleasable = 2,
} PxBaseFlag;


/// Flags used to configure binary meta data entries, typically set through PX_DEF_BIN_METADATA defines.
typedef enum PxMetaDataFlag: int32_t {
    /// declares a class
    PxMetaDataFlag_Class = 1,
    /// declares class to be virtual
    PxMetaDataFlag_Virtual = 2,
    /// declares a typedef
    PxMetaDataFlag_Typedef = 4,
    /// declares a pointer
    PxMetaDataFlag_Ptr = 8,
    /// declares a handle
    PxMetaDataFlag_Handle = 16,
    /// declares extra data exported with PxSerializer::exportExtraData
    PxMetaDataFlag_ExtraData = 32,
    /// specifies one element of extra data
    PxMetaDataFlag_ExtraItem = 64,
    /// specifies an array of extra data
    PxMetaDataFlag_ExtraItems = 128,
    /// specifies a name of extra data
    PxMetaDataFlag_ExtraName = 256,
    /// declares a union
    PxMetaDataFlag_Union = 512,
    /// declares explicit padding data
    PxMetaDataFlag_Padding = 1024,
    /// declares aligned data
    PxMetaDataFlag_Alignment = 2048,
    /// specifies that the count value's most significant bit needs to be masked out
    PxMetaDataFlag_CountMaskMsb = 4096,
    /// specifies that the count value is treated as zero for a variable value of one - special case for single triangle meshes
    PxMetaDataFlag_CountSkipIfOne = 8192,
    /// specifies that the control value is the negate of the variable value
    PxMetaDataFlag_ControlFlip = 16384,
    /// specifies that the control value is masked - mask bits are assumed to be within eCONTROL_MASK_RANGE
    PxMetaDataFlag_ControlMask = 32768,
    /// mask range allowed for eCONTROL_MASK
    PxMetaDataFlag_ControlMaskRange = 255,
    PxMetaDataFlag_ForceDword = 2147483647,
} PxMetaDataFlag;

/// Identifies the type of each heavyweight PxTask object
typedef enum PxTaskType: int32_t {
    /// PxTask will be run on the CPU
    PxTaskType_Cpu = 0,
    /// Return code when attempting to find a task that does not exist
    PxTaskType_NotPresent = 1,
    /// PxTask execution has been completed
    PxTaskType_Completed = 2,
} PxTaskType;

/// A geometry type.
///
/// Used to distinguish the type of a ::PxGeometry object.
typedef enum PxGeometryType: int32_t {
    PxGeometryType_Sphere = 0,
    PxGeometryType_Plane = 1,
    PxGeometryType_Capsule = 2,
    PxGeometryType_Box = 3,
    PxGeometryType_Convexmesh = 4,
    PxGeometryType_Particlesystem = 5,
    PxGeometryType_Tetrahedronmesh = 6,
    PxGeometryType_Trianglemesh = 7,
    PxGeometryType_Heightfield = 8,
    PxGeometryType_Hairsystem = 9,
    PxGeometryType_Custom = 10,
    /// internal use only!
    PxGeometryType_GeometryCount = 11,
    /// internal use only!
    PxGeometryType_Invalid = -1,
} PxGeometryType;

/// Geometry-level query flags.
typedef enum PxGeometryQueryFlag: int32_t {
    /// Saves/restores SIMD control word for each query (safer but slower). Omit this if you took care of it yourself in your app.
    PxGeometryQueryFlag_SimdGuard = 1,
} PxGeometryQueryFlag;


/// Desired build strategy for bounding-volume hierarchies
typedef enum PxBVHBuildStrategy: int32_t {
    /// Fast build strategy. Fast build speed, good runtime performance in most cases. Recommended for runtime cooking.
    PxBVHBuildStrategy_Fast = 0,
    /// Default build strategy. Medium build speed, good runtime performance in all cases.
    PxBVHBuildStrategy_Default = 1,
    /// SAH build strategy. Slower builds, slightly improved runtime performance in some cases.
    PxBVHBuildStrategy_Sah = 2,
    PxBVHBuildStrategy_Last = 3,
} PxBVHBuildStrategy;

/// Flags controlling the simulated behavior of the convex mesh geometry.
///
/// Used in ::PxConvexMeshGeometryFlags.
typedef enum PxConvexMeshGeometryFlag: int32_t {
    /// Use tighter (but more expensive to compute) bounds around the convex geometry.
    PxConvexMeshGeometryFlag_TightBounds = 1,
} PxConvexMeshGeometryFlag;


/// Flags controlling the simulated behavior of the triangle mesh geometry.
///
/// Used in ::PxMeshGeometryFlags.
typedef enum PxMeshGeometryFlag: int32_t {
    /// Use tighter (but more expensive to compute) bounds around the triangle mesh geometry.
    PxMeshGeometryFlag_TightBounds = 1,
    /// Meshes with this flag set are treated as double-sided.
    /// This flag is currently only used for raycasts and sweeps (it is ignored for overlap queries).
    /// For detailed specifications of this flag for meshes and heightfields please refer to the Geometry Query section of the user guide.
    PxMeshGeometryFlag_DoubleSided = 2,
} PxMeshGeometryFlag;


/// Identifies the solver to use for a particle system.
typedef enum PxParticleSolverType: int32_t {
    /// The position based dynamics solver that can handle fluid, granular material, cloth, inflatables etc. See [`PxPBDParticleSystem`].
    PxParticleSolverType_Pbd = 1,
    /// The FLIP fluid solver. See [`PxFLIPParticleSystem`].
    PxParticleSolverType_Flip = 2,
    /// The MPM (material point method) solver that can handle a variety of materials. See [`PxMPMParticleSystem`].
    PxParticleSolverType_Mpm = 4,
    /// Custom solver. The user needs to specify the interaction of the particle by providing appropriate functions. Can be used e.g. for molecular dynamics simulations. See [`PxCustomParticleSystem`].
    PxParticleSolverType_Custom = 8,
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
    PxHitFlag_Position = 1,
    /// "normal" member of [`PxQueryHit`] is valid
    PxHitFlag_Normal = 2,
    /// "u" and "v" barycentric coordinates of [`PxQueryHit`] are valid. Not applicable to sweep queries.
    PxHitFlag_Uv = 8,
    /// Performance hint flag for sweeps when it is known upfront there's no initial overlap.
    /// NOTE: using this flag may cause undefined results if shapes are initially overlapping.
    PxHitFlag_AssumeNoInitialOverlap = 16,
    /// Report any first hit. Used for geometries that contain more than one primitive. For meshes,
    /// if neither eMESH_MULTIPLE nor eANY_HIT is specified, a single closest hit will be reported.
    PxHitFlag_AnyHit = 32,
    /// Report all hits for meshes rather than just the first. Not applicable to sweep queries.
    PxHitFlag_MeshMultiple = 64,
    /// Report hits with back faces of mesh triangles. Also report hits for raycast
    /// originating on mesh surface and facing away from the surface normal. Not applicable to sweep queries.
    /// Please refer to the user guide for heightfield-specific differences.
    PxHitFlag_MeshBothSides = 128,
    /// Use more accurate but slower narrow phase sweep tests.
    /// May provide better compatibility with PhysX 3.2 sweep behavior.
    PxHitFlag_PreciseSweep = 256,
    /// Report the minimum translation depth, normal and contact point.
    PxHitFlag_Mtd = 512,
    /// "face index" member of [`PxQueryHit`] is valid
    PxHitFlag_FaceIndex = 1024,
    PxHitFlag_Default = 1027,
    /// Only this subset of flags can be modified by pre-filter. Other modifications will be discarded.
    PxHitFlag_ModifiableFlags = 464,
} PxHitFlag;


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
    PxHeightFieldFormat_S16Tm = 1,
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
    PxHeightFieldTessFlag_E0ThVertexShared = 1,
} PxHeightFieldTessFlag;

/// Enum with flag values to be used in PxHeightFieldDesc.flags.
typedef enum PxHeightFieldFlag: int32_t {
    /// Disable collisions with height field with boundary edges.
    ///
    /// Raise this flag if several terrain patches are going to be placed adjacent to each other,
    /// to avoid a bump when sliding across.
    ///
    /// This flag is ignored in contact generation with sphere and capsule shapes.
    PxHeightFieldFlag_NoBoundaryEdges = 1,
} PxHeightFieldFlag;


/// Special material index values for height field samples.
typedef enum PxHeightFieldMaterial: int32_t {
    /// A material indicating that the triangle should be treated as a hole in the mesh.
    PxHeightFieldMaterial_Hole = 127,
} PxHeightFieldMaterial;

typedef enum PxMeshMeshQueryFlag: int32_t {
    /// Report all overlaps
    PxMeshMeshQueryFlag_Default = 0,
    /// Ignore coplanar triangle-triangle overlaps
    PxMeshMeshQueryFlag_DiscardCoplanar = 1,
} PxMeshMeshQueryFlag;


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
    PxMeshFlag_Flipnormals = 1,
    /// Denotes the use of 16-bit vertex indices
    PxMeshFlag_E16BitIndices = 2,
} PxMeshFlag;


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
    PxMeshMidPhase_Bvh33 = 0,
    /// New midphase mesh structure, introduced in PhysX 3.4
    PxMeshMidPhase_Bvh34 = 1,
    PxMeshMidPhase_Last = 2,
} PxMeshMidPhase;

/// Flags for the mesh geometry properties.
///
/// Used in ::PxTriangleMeshFlags.
typedef enum PxTriangleMeshFlag: int32_t {
    /// The triangle mesh has 16bits vertex indices.
    PxTriangleMeshFlag_E16BitIndices = 2,
    /// The triangle mesh has adjacency information build.
    PxTriangleMeshFlag_AdjacencyInfo = 4,
    /// Indicates that this mesh would preferably not be the mesh projected for mesh-mesh collision. This can indicate that the mesh is not well tessellated.
    PxTriangleMeshFlag_PreferNoSdfProj = 8,
} PxTriangleMeshFlag;


typedef enum PxTetrahedronMeshFlag: int32_t {
    /// The tetrahedron mesh has 16bits vertex indices
    PxTetrahedronMeshFlag_E16BitIndices = 2,
} PxTetrahedronMeshFlag;


/// Flags which control the behavior of an actor.
typedef enum PxActorFlag: int32_t {
    /// Enable debug renderer for this actor
    PxActorFlag_Visualization = 1,
    /// Disables scene gravity for this actor
    PxActorFlag_DisableGravity = 2,
    /// Enables the sending of PxSimulationEventCallback::onWake() and PxSimulationEventCallback::onSleep() notify events
    PxActorFlag_SendSleepNotifies = 4,
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
    PxActorFlag_DisableSimulation = 8,
} PxActorFlag;


/// Identifies each type of actor.
typedef enum PxActorType: int32_t {
    /// A static rigid body
    PxActorType_RigidStatic = 0,
    /// A dynamic rigid body
    PxActorType_RigidDynamic = 1,
    /// An articulation link
    PxActorType_ArticulationLink = 2,
} PxActorType;

typedef enum PxAggregateType: int32_t {
    /// Aggregate will contain various actors of unspecified types
    PxAggregateType_Generic = 0,
    /// Aggregate will only contain static actors
    PxAggregateType_Static = 1,
    /// Aggregate will only contain kinematic actors
    PxAggregateType_Kinematic = 2,
} PxAggregateType;

/// Constraint row flags
///
/// These flags configure the post-processing of constraint rows and the behavior of the solver while solving constraints
typedef enum Px1DConstraintFlag: int32_t {
    /// whether the constraint is a spring. Mutually exclusive with eRESTITUTION. If set, eKEEPBIAS is ignored.
    Px1DConstraintFlag_Spring = 1,
    /// whether the constraint is a force or acceleration spring. Only valid if eSPRING is set.
    Px1DConstraintFlag_AccelerationSpring = 2,
    /// whether the restitution model should be applied to generate the target velocity. Mutually exclusive with eSPRING. If restitution causes a bounces, eKEEPBIAS is ignored
    Px1DConstraintFlag_Restitution = 4,
    /// whether to keep the error term when solving for velocity. Ignored if restitution generates bounce, or eSPRING is set.
    Px1DConstraintFlag_Keepbias = 8,
    /// whether to accumulate the force value from this constraint in the force total that is reported for the constraint and tested for breakage
    Px1DConstraintFlag_OutputForce = 16,
    /// whether the constraint has a drive force limit (which will be scaled by dt unless PxConstraintFlag::eLIMITS_ARE_FORCES is set)
    Px1DConstraintFlag_HasDriveLimit = 32,
    /// whether this is an angular or linear constraint
    Px1DConstraintFlag_AngularConstraint = 64,
    /// whether the constraint's geometric error should drive the target velocity
    Px1DConstraintFlag_DriveRow = 128,
} Px1DConstraintFlag;


/// Constraint type hints which the solver uses to optimize constraint handling
typedef enum PxConstraintSolveHint: int32_t {
    /// no special properties
    PxConstraintSolveHint_None = 0,
    /// a group of acceleration drive constraints with the same stiffness and drive parameters
    PxConstraintSolveHint_Acceleration1 = 256,
    /// temporary special value to identify SLERP drive rows
    PxConstraintSolveHint_SlerpSpring = 258,
    /// a group of acceleration drive constraints with the same stiffness and drive parameters
    PxConstraintSolveHint_Acceleration2 = 512,
    /// a group of acceleration drive constraints with the same stiffness and drive parameters
    PxConstraintSolveHint_Acceleration3 = 768,
    /// rotational equality constraints with no force limit and no velocity target
    PxConstraintSolveHint_RotationalEquality = 1024,
    /// rotational inequality constraints with (0, PX_MAX_FLT) force limits
    PxConstraintSolveHint_RotationalInequality = 1025,
    /// equality constraints with no force limit and no velocity target
    PxConstraintSolveHint_Equality = 2048,
    /// inequality constraints with (0, PX_MAX_FLT) force limits
    PxConstraintSolveHint_Inequality = 2049,
} PxConstraintSolveHint;

/// Flags for determining which components of the constraint should be visualized.
typedef enum PxConstraintVisualizationFlag: int32_t {
    /// visualize constraint frames
    PxConstraintVisualizationFlag_LocalFrames = 1,
    /// visualize constraint limits
    PxConstraintVisualizationFlag_Limits = 2,
} PxConstraintVisualizationFlag;

/// Flags for determining how PVD should serialize a constraint update
typedef enum PxPvdUpdateType: int32_t {
    /// triggers createPvdInstance call, creates an instance of a constraint
    PxPvdUpdateType_CreateInstance = 0,
    /// triggers releasePvdInstance call, releases an instance of a constraint
    PxPvdUpdateType_ReleaseInstance = 1,
    /// triggers updatePvdProperties call, updates all properties of a constraint
    PxPvdUpdateType_UpdateAllProperties = 2,
    /// triggers simUpdate call, updates all simulation properties of a constraint
    PxPvdUpdateType_UpdateSimProperties = 3,
} PxPvdUpdateType;

/// Constraint descriptor used inside the solver
typedef enum ConstraintType: int32_t {
    /// Defines this pair is a contact constraint
    ConstraintType_ContactConstraint = 0,
    /// Defines this pair is a joint constraint
    ConstraintType_JointConstraint = 1,
} ConstraintType;

/// Data structure used for preparing constraints before solving them
typedef enum BodyState: int32_t {
    BodyState_DynamicBody = 1,
    BodyState_StaticBody = 2,
    BodyState_KinematicBody = 4,
    BodyState_Articulation = 8,
} BodyState;

/// @
/// {
typedef enum PxArticulationAxis: int32_t {
    /// Rotational about eX
    PxArticulationAxis_Twist = 0,
    /// Rotational about eY
    PxArticulationAxis_Swing1 = 1,
    /// Rotational about eZ
    PxArticulationAxis_Swing2 = 2,
    /// Linear in eX
    PxArticulationAxis_X = 3,
    /// Linear in eY
    PxArticulationAxis_Y = 4,
    /// Linear in eZ
    PxArticulationAxis_Z = 5,
    PxArticulationAxis_Count = 6,
} PxArticulationAxis;

typedef enum PxArticulationMotion: int32_t {
    /// Locked axis, i.e. degree of freedom (DOF)
    PxArticulationMotion_Locked = 0,
    /// Limited DOF - set limits of joint DOF together with this flag, see PxArticulationJointReducedCoordinate::setLimitParams
    PxArticulationMotion_Limited = 1,
    /// Free DOF
    PxArticulationMotion_Free = 2,
} PxArticulationMotion;


typedef enum PxArticulationJointType: int32_t {
    /// All joint axes, i.e. degrees of freedom (DOFs) locked
    PxArticulationJointType_Fix = 0,
    /// Single linear DOF, e.g. cart on a rail
    PxArticulationJointType_Prismatic = 1,
    /// Single rotational DOF, e.g. an elbow joint or a rotational motor, position wrapped at 2pi radians
    PxArticulationJointType_Revolute = 2,
    /// Single rotational DOF, e.g. an elbow joint or a rotational motor, position not wrapped
    PxArticulationJointType_RevoluteUnwrapped = 3,
    /// Ball and socket joint with two or three DOFs
    PxArticulationJointType_Spherical = 4,
    PxArticulationJointType_Undefined = 5,
} PxArticulationJointType;

typedef enum PxArticulationFlag: int32_t {
    /// Set articulation base to be fixed.
    PxArticulationFlag_FixBase = 1,
    /// Limits for drive effort are forces and torques rather than impulses, see PxArticulationDrive::maxForce.
    PxArticulationFlag_DriveLimitsAreForces = 2,
    /// Disable collisions between the articulation's links (note that parent/child collisions are disabled internally in either case).
    PxArticulationFlag_DisableSelfCollision = 4,
    /// Enable in order to be able to query joint solver (i.e. constraint) forces using PxArticulationCache::jointSolverForces.
    PxArticulationFlag_ComputeJointForces = 8,
} PxArticulationFlag;


typedef enum PxArticulationDriveType: int32_t {
    /// The output of the implicit spring drive controller is a force/torque.
    PxArticulationDriveType_Force = 0,
    /// The output of the implicit spring drive controller is a joint acceleration (use this to get (spatial)-inertia-invariant behavior of the drive).
    PxArticulationDriveType_Acceleration = 1,
    /// Sets the drive gains internally to track a target position almost kinematically (i.e. with very high drive gains).
    PxArticulationDriveType_Target = 2,
    /// Sets the drive gains internally to track a target velocity almost kinematically (i.e. with very high drive gains).
    PxArticulationDriveType_Velocity = 3,
    PxArticulationDriveType_None = 4,
} PxArticulationDriveType;

/// A description of the types of articulation data that may be directly written to and read from the GPU using the functions
/// PxScene::copyArticulationData() and PxScene::applyArticulationData(). Types that are read-only may only be used in conjunction with
/// PxScene::copyArticulationData(). Types that are write-only may only be used in conjunction with PxScene::applyArticulationData().
/// A subset of data types may be used in conjunction with both PxScene::applyArticulationData() and PxScene::applyArticulationData().
typedef enum PxArticulationGpuDataType: int32_t {
    /// The joint positions, read and write, see PxScene::copyArticulationData(), PxScene::applyArticulationData()
    PxArticulationGpuDataType_JointPosition = 0,
    /// The joint velocities, read and write,  see PxScene::copyArticulationData(), PxScene::applyArticulationData()
    PxArticulationGpuDataType_JointVelocity = 1,
    /// The joint accelerations, read only, see PxScene::copyArticulationData()
    PxArticulationGpuDataType_JointAcceleration = 2,
    /// The applied joint forces, write only, see PxScene::applyArticulationData()
    PxArticulationGpuDataType_JointForce = 3,
    /// The computed joint constraint solver forces, read only, see PxScene::copyArticulationData()()
    PxArticulationGpuDataType_JointSolverForce = 4,
    /// The velocity targets for the joint drives, write only, see PxScene::applyArticulationData()
    PxArticulationGpuDataType_JointTargetVelocity = 5,
    /// The position targets for the joint drives, write only, see PxScene::applyArticulationData()
    PxArticulationGpuDataType_JointTargetPosition = 6,
    /// The spatial sensor forces, read only, see PxScene::copyArticulationData()
    PxArticulationGpuDataType_SensorForce = 7,
    /// The root link transform, read and write, see PxScene::copyArticulationData(), PxScene::applyArticulationData()
    PxArticulationGpuDataType_RootTransform = 8,
    /// The root link velocity, read and write, see PxScene::copyArticulationData(), PxScene::applyArticulationData()
    PxArticulationGpuDataType_RootVelocity = 9,
    /// The link transforms including root link, read only, see PxScene::copyArticulationData()
    PxArticulationGpuDataType_LinkTransform = 10,
    /// The link velocities including root link, read only, see PxScene::copyArticulationData()
    PxArticulationGpuDataType_LinkVelocity = 11,
    /// The forces to apply to links, write only, see PxScene::applyArticulationData()
    PxArticulationGpuDataType_LinkForce = 12,
    /// The torques to apply to links, write only, see PxScene::applyArticulationData()
    PxArticulationGpuDataType_LinkTorque = 13,
    /// Fixed tendon data, write only, see PxScene::applyArticulationData()
    PxArticulationGpuDataType_FixedTendon = 14,
    /// Fixed tendon joint data, write only, see PxScene::applyArticulationData()
    PxArticulationGpuDataType_FixedTendonJoint = 15,
    /// Spatial tendon data, write only, see PxScene::applyArticulationData()
    PxArticulationGpuDataType_SpatialTendon = 16,
    /// Spatial tendon attachment data, write only, see PxScene::applyArticulationData()
    PxArticulationGpuDataType_SpatialTendonAttachment = 17,
} PxArticulationGpuDataType;

/// These flags determine what data is read or written to the internal articulation data via cache.
typedef enum PxArticulationCacheFlag: int32_t {
    /// The joint velocities, see PxArticulationCache::jointVelocity.
    PxArticulationCacheFlag_Velocity = 1,
    /// The joint accelerations, see PxArticulationCache::jointAcceleration.
    PxArticulationCacheFlag_Acceleration = 2,
    /// The joint positions, see PxArticulationCache::jointPosition.
    PxArticulationCacheFlag_Position = 4,
    /// The joint forces, see PxArticulationCache::jointForce.
    PxArticulationCacheFlag_Force = 8,
    /// The link velocities, see PxArticulationCache::linkVelocity.
    PxArticulationCacheFlag_LinkVelocity = 16,
    /// The link accelerations, see PxArticulationCache::linkAcceleration.
    PxArticulationCacheFlag_LinkAcceleration = 32,
    /// Root link transform, see PxArticulationCache::rootLinkData.
    PxArticulationCacheFlag_RootTransform = 64,
    /// Root link velocities (read/write) and accelerations (read), see PxArticulationCache::rootLinkData.
    PxArticulationCacheFlag_RootVelocities = 128,
    /// The spatial sensor forces, see PxArticulationCache::sensorForces.
    PxArticulationCacheFlag_SensorForces = 256,
    /// Solver constraint joint forces, see PxArticulationCache::jointSolverForces.
    PxArticulationCacheFlag_JointSolverForces = 512,
    PxArticulationCacheFlag_All = 247,
} PxArticulationCacheFlag;


/// Flags to configure the forces reported by articulation link sensors.
typedef enum PxArticulationSensorFlag: int32_t {
    /// Raise to receive forces from forward dynamics.
    PxArticulationSensorFlag_ForwardDynamicsForces = 1,
    /// Raise to receive forces from constraint solver.
    PxArticulationSensorFlag_ConstraintSolverForces = 2,
    /// Raise to receive forces in the world rotation frame, otherwise they will be reported in the sensor's local frame.
    PxArticulationSensorFlag_WorldFrame = 4,
} PxArticulationSensorFlag;


/// Flag that configures articulation-state updates by PxArticulationReducedCoordinate::updateKinematic.
typedef enum PxArticulationKinematicFlag: int32_t {
    /// Raise after any changes to the articulation root or joint positions using non-cache API calls. Updates links' positions and velocities.
    PxArticulationKinematicFlag_Position = 1,
    /// Raise after velocity-only changes to the articulation root or joints using non-cache API calls. Updates links' velocities.
    PxArticulationKinematicFlag_Velocity = 2,
} PxArticulationKinematicFlag;


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
    PxShapeFlag_SimulationShape = 1,
    /// The shape will partake in scene queries (ray casts, overlap tests, sweeps, ...).
    PxShapeFlag_SceneQueryShape = 2,
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
    PxShapeFlag_TriggerShape = 4,
    /// Enable debug renderer for this shape
    PxShapeFlag_Visualization = 8,
} PxShapeFlag;


/// Parameter to addForce() and addTorque() calls, determines the exact operation that is carried out.
typedef enum PxForceMode: int32_t {
    /// parameter has unit of mass * length / time^2, i.e., a force
    PxForceMode_Force = 0,
    /// parameter has unit of mass * length / time, i.e., force * time
    PxForceMode_Impulse = 1,
    /// parameter has unit of length / time, i.e., the effect is mass independent: a velocity change.
    PxForceMode_VelocityChange = 2,
    /// parameter has unit of length/ time^2, i.e., an acceleration. It gets treated just like a force except the mass is not divided out before integration.
    PxForceMode_Acceleration = 3,
} PxForceMode;

/// Collection of flags describing the behavior of a rigid body.
typedef enum PxRigidBodyFlag: int32_t {
    /// Enable kinematic mode for the body.
    PxRigidBodyFlag_Kinematic = 1,
    /// Use the kinematic target transform for scene queries.
    ///
    /// If this flag is raised, then scene queries will treat the kinematic target transform as the current pose
    /// of the body (instead of using the actual pose). Without this flag, the kinematic target will only take
    /// effect with respect to scene queries after a simulation step.
    PxRigidBodyFlag_UseKinematicTargetForSceneQueries = 2,
    /// Enable CCD for the body.
    PxRigidBodyFlag_EnableCcd = 4,
    /// Enabled CCD in swept integration for the actor.
    ///
    /// If this flag is raised and CCD is enabled, CCD interactions will simulate friction. By default, friction is disabled in CCD interactions because
    /// CCD friction has been observed to introduce some simulation artifacts. CCD friction was enabled in previous versions of the SDK. Raising this flag will result in behavior
    /// that is a closer match for previous versions of the SDK.
    ///
    /// This flag requires PxRigidBodyFlag::eENABLE_CCD to be raised to have any effect.
    PxRigidBodyFlag_EnableCcdFriction = 8,
    /// Register a rigid body to dynamically adjust contact offset based on velocity. This can be used to achieve a CCD effect.
    ///
    /// If both eENABLE_CCD and eENABLE_SPECULATIVE_CCD are set on the same body, then angular motions are handled by speculative
    /// contacts (eENABLE_SPECULATIVE_CCD) while linear motions are handled by sweeps (eENABLE_CCD).
    PxRigidBodyFlag_EnableSpeculativeCcd = 16,
    /// Register a rigid body for reporting pose changes by the simulation at an early stage.
    ///
    /// Sometimes it might be advantageous to get access to the new pose of a rigid body as early as possible and
    /// not wait until the call to fetchResults() returns. Setting this flag will schedule the rigid body to get reported
    /// in [`PxSimulationEventCallback::onAdvance`](). Please refer to the documentation of that callback to understand
    /// the behavior and limitations of this functionality.
    PxRigidBodyFlag_EnablePoseIntegrationPreview = 32,
    /// Permit CCD to limit maxContactImpulse. This is useful for use-cases like a destruction system but can cause visual artefacts so is not enabled by default.
    PxRigidBodyFlag_EnableCcdMaxContactImpulse = 64,
    /// Carries over forces/accelerations between frames, rather than clearing them
    PxRigidBodyFlag_RetainAccelerations = 128,
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
    PxRigidBodyFlag_ForceKineKineNotifications = 256,
    /// Forces static-kinematic pairs notifications for this actor.
    ///
    /// Similar to eFORCE_KINE_KINE_NOTIFICATIONS, but for static-kinematic interactions.
    ///
    /// This has no effect if PxRigidBodyFlag::eKINEMATIC is not set.
    ///
    /// Changing this flag at runtime will not have an effect until you remove and re-add the actor to the scene.
    PxRigidBodyFlag_ForceStaticKineNotifications = 512,
    /// Enables computation of gyroscopic forces on the rigid body.
    PxRigidBodyFlag_EnableGyroscopicForces = 1024,
} PxRigidBodyFlag;


/// constraint flags
///
/// eBROKEN is a read only flag
typedef enum PxConstraintFlag: int32_t {
    /// whether the constraint is broken
    PxConstraintFlag_Broken = 1,
    /// whether actor1 should get projected to actor0 for this constraint (note: projection of a static/kinematic actor to a dynamic actor will be ignored)
    PxConstraintFlag_ProjectToActor0 = 2,
    /// whether actor0 should get projected to actor1 for this constraint (note: projection of a static/kinematic actor to a dynamic actor will be ignored)
    PxConstraintFlag_ProjectToActor1 = 4,
    /// whether the actors should get projected for this constraint (the direction will be chosen by PhysX)
    PxConstraintFlag_Projection = 6,
    /// whether contacts should be generated between the objects this constraint constrains
    PxConstraintFlag_CollisionEnabled = 8,
    /// whether this constraint should be visualized, if constraint visualization is turned on
    PxConstraintFlag_Visualization = 16,
    /// limits for drive strength are forces rather than impulses
    PxConstraintFlag_DriveLimitsAreForces = 32,
    /// perform preprocessing for improved accuracy on D6 Slerp Drive (this flag will be removed in a future release when preprocessing is no longer required)
    PxConstraintFlag_ImprovedSlerp = 128,
    /// suppress constraint preprocessing, intended for use with rowResponseThreshold. May result in worse solver accuracy for ill-conditioned constraints.
    PxConstraintFlag_DisablePreprocessing = 256,
    /// enables extended limit ranges for angular limits (e.g., limit values > PxPi or
    /// <
    /// -PxPi)
    PxConstraintFlag_EnableExtendedLimits = 512,
    /// the constraint type is supported by gpu dynamics
    PxConstraintFlag_GpuCompatible = 1024,
    /// updates the constraint each frame
    PxConstraintFlag_AlwaysUpdate = 2048,
    /// disables the constraint. SolverPrep functions won't be called for this constraint.
    PxConstraintFlag_DisableConstraint = 4096,
} PxConstraintFlag;


/// Header for a contact patch where all points share same material and normal
typedef enum PxContactPatchFlags: int32_t {
    /// Indicates this contact stream has face indices.
    PxContactPatchFlags_HasFaceIndices = 1,
    /// Indicates this contact stream is modifiable.
    PxContactPatchFlags_Modifiable = 2,
    /// Indicates this contact stream is notify-only (no contact response).
    PxContactPatchFlags_ForceNoResponse = 4,
    /// Indicates this contact stream has modified mass ratios
    PxContactPatchFlags_HasModifiedMassRatios = 8,
    /// Indicates this contact stream has target velocities set
    PxContactPatchFlags_HasTargetVelocity = 16,
    /// Indicates this contact stream has max impulses set
    PxContactPatchFlags_HasMaxImpulse = 32,
    /// Indicates this contact stream needs patches re-generated. This is required if the application modified either the contact normal or the material properties
    PxContactPatchFlags_RegeneratePatches = 64,
    PxContactPatchFlags_CompressedModifiedContact = 128,
} PxContactPatchFlags;

/// A class to iterate over a compressed contact stream. This supports read-only access to the various contact formats.
typedef enum StreamFormat: int32_t {
    StreamFormat_SimpleStream = 0,
    StreamFormat_ModifiableStream = 1,
    StreamFormat_CompressedModifiableStream = 2,
} StreamFormat;

/// Flags specifying deletion event types.
typedef enum PxDeletionEventFlag: int32_t {
    /// The user has called release on an object.
    PxDeletionEventFlag_UserRelease = 1,
    /// The destructor of an object has been called and the memory has been released.
    PxDeletionEventFlag_MemoryRelease = 2,
} PxDeletionEventFlag;


/// Collection of flags describing the actions to take for a collision pair.
typedef enum PxPairFlag: int32_t {
    /// Process the contacts of this collision pair in the dynamics solver.
    ///
    /// Only takes effect if the colliding actors are rigid bodies.
    PxPairFlag_SolveContact = 1,
    /// Call contact modification callback for this collision pair
    ///
    /// Only takes effect if the colliding actors are rigid bodies.
    PxPairFlag_ModifyContacts = 2,
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
    PxPairFlag_NotifyTouchFound = 4,
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
    PxPairFlag_NotifyTouchPersists = 8,
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
    PxPairFlag_NotifyTouchLost = 16,
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
    PxPairFlag_NotifyTouchCcd = 32,
    /// Call contact report callback when the contact force between the actors of this collision pair exceeds one of the actor-defined force thresholds.
    ///
    /// Only takes effect if the colliding actors are rigid bodies.
    ///
    /// Only takes effect if eDETECT_DISCRETE_CONTACT or eDETECT_CCD_CONTACT is raised
    PxPairFlag_NotifyThresholdForceFound = 64,
    /// Call contact report callback when the contact force between the actors of this collision pair continues to exceed one of the actor-defined force thresholds.
    ///
    /// Only takes effect if the colliding actors are rigid bodies.
    ///
    /// If a pair gets re-filtered and this flag has previously been disabled, then the report will not get fired in the same frame even if the force threshold has been reached in the
    /// previous one (unless [`eNOTIFY_THRESHOLD_FORCE_FOUND`] has been set in the previous frame).
    ///
    /// Only takes effect if eDETECT_DISCRETE_CONTACT or eDETECT_CCD_CONTACT is raised
    PxPairFlag_NotifyThresholdForcePersists = 128,
    /// Call contact report callback when the contact force between the actors of this collision pair falls below one of the actor-defined force thresholds (includes the case where this collision pair stops being in contact).
    ///
    /// Only takes effect if the colliding actors are rigid bodies.
    ///
    /// If a pair gets re-filtered and this flag has previously been disabled, then the report will not get fired in the same frame even if the force threshold has been reached in the
    /// previous one (unless [`eNOTIFY_THRESHOLD_FORCE_FOUND`] or #eNOTIFY_THRESHOLD_FORCE_PERSISTS has been set in the previous frame).
    ///
    /// Only takes effect if eDETECT_DISCRETE_CONTACT or eDETECT_CCD_CONTACT is raised
    PxPairFlag_NotifyThresholdForceLost = 256,
    /// Provide contact points in contact reports for this collision pair.
    ///
    /// Only takes effect if the colliding actors are rigid bodies and if used in combination with the flags eNOTIFY_TOUCH_... or eNOTIFY_THRESHOLD_FORCE_...
    ///
    /// Only takes effect if eDETECT_DISCRETE_CONTACT or eDETECT_CCD_CONTACT is raised
    PxPairFlag_NotifyContactPoints = 512,
    /// This flag is used to indicate whether this pair generates discrete collision detection contacts.
    ///
    /// Contacts are only responded to if eSOLVE_CONTACT is enabled.
    PxPairFlag_DetectDiscreteContact = 1024,
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
    PxPairFlag_DetectCcdContact = 2048,
    /// Provide pre solver velocities in contact reports for this collision pair.
    ///
    /// If the collision pair has contact reports enabled, the velocities of the rigid bodies before contacts have been solved
    /// will be provided in the contact report callback unless the pair lost touch in which case no data will be provided.
    ///
    /// Usually it is not necessary to request these velocities as they will be available by querying the velocity from the provided
    /// PxRigidActor object directly. However, it might be the case that the velocity of a rigid body gets set while the simulation is running
    /// in which case the PxRigidActor would return this new velocity in the contact report callback and not the velocity the simulation used.
    PxPairFlag_PreSolverVelocity = 4096,
    /// Provide post solver velocities in contact reports for this collision pair.
    ///
    /// If the collision pair has contact reports enabled, the velocities of the rigid bodies after contacts have been solved
    /// will be provided in the contact report callback unless the pair lost touch in which case no data will be provided.
    PxPairFlag_PostSolverVelocity = 8192,
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
    PxPairFlag_ContactEventPose = 16384,
    /// For internal use only.
    PxPairFlag_NextFree = 32768,
    /// Provided default flag to do simple contact processing for this collision pair.
    PxPairFlag_ContactDefault = 1025,
    /// Provided default flag to get commonly used trigger behavior for this collision pair.
    PxPairFlag_TriggerDefault = 1044,
} PxPairFlag;


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
    PxFilterFlag_Kill = 1,
    /// Ignore the collision pair as long as the bounding volumes of the pair objects overlap or until filtering relevant data changes for one of the collision objects.
    ///
    /// Suppressed pairs will be ignored by the simulation and won't make another filter request until one
    /// of the following occurs:
    ///
    /// Same conditions as for killed pairs (see [`eKILL`])
    ///
    /// The filter data or the filter object attributes change for one of the collision objects
    PxFilterFlag_Suppress = 2,
    /// Invoke the filter callback ([`PxSimulationFilterCallback::pairFound`]()) for this collision pair.
    PxFilterFlag_Callback = 4,
    /// Track this collision pair with the filter callback mechanism.
    ///
    /// When the bounding volumes of the collision pair lose contact, the filter callback [`PxSimulationFilterCallback::pairLost`]()
    /// will be invoked. Furthermore, the filter status of the collision pair can be adjusted through [`PxSimulationFilterCallback::statusChange`]()
    /// once per frame (until a pairLost() notification occurs).
    PxFilterFlag_Notify = 12,
    /// Provided default to get standard behavior:
    ///
    /// The application configure the pair's collision properties once when bounding volume overlap is found and
    /// doesn't get asked again about that pair until overlap status or filter properties changes, or re-filtering is requested.
    ///
    /// No notification is provided when bounding volume overlap is lost
    ///
    /// The pair will not be killed or suppressed, so collision detection will be processed
    PxFilterFlag_Default = 0,
} PxFilterFlag;


/// Identifies each type of filter object.
typedef enum PxFilterObjectType: int32_t {
    /// A static rigid body
    PxFilterObjectType_RigidStatic = 0,
    /// A dynamic rigid body
    PxFilterObjectType_RigidDynamic = 1,
    /// An articulation
    PxFilterObjectType_Articulation = 2,
    /// A particle system
    PxFilterObjectType_Particlesystem = 3,
    /// A FEM-based soft body
    PxFilterObjectType_Softbody = 4,
    /// A FEM-based cloth
    ///
    /// In development
    PxFilterObjectType_Femcloth = 5,
    /// A hair system
    ///
    /// In development
    PxFilterObjectType_Hairsystem = 6,
    /// internal use only!
    PxFilterObjectType_MaxTypeCount = 16,
    /// internal use only!
    PxFilterObjectType_Undefined = 15,
} PxFilterObjectType;

typedef enum PxFilterObjectFlag: int32_t {
    PxFilterObjectFlag_Kinematic = 16,
    PxFilterObjectFlag_Trigger = 32,
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
    PxPairFilteringMode_Keep = 0,
    /// Output pair from BP, create interaction marker. Can be later switched to regular interaction.
    PxPairFilteringMode_Suppress = 1,
    /// Don't output pair from BP. Cannot be later switched to regular interaction, needs "resetFiltering" call.
    PxPairFilteringMode_Kill = 2,
} PxPairFilteringMode;

typedef enum PxDataAccessFlag: int32_t {
    PxDataAccessFlag_Readable = 1,
    PxDataAccessFlag_Writable = 2,
    PxDataAccessFlag_Device = 4,
} PxDataAccessFlag;


/// Flags which control the behavior of a material.
typedef enum PxMaterialFlag: int32_t {
    /// If this flag is set, friction computations are always skipped between shapes with this material and any other shape.
    PxMaterialFlag_DisableFriction = 1,
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
    PxMaterialFlag_DisableStrongFriction = 2,
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
    PxMaterialFlag_ImprovedPatchFriction = 4,
    /// This flag has the effect of enabling an implicit spring model for the normal force computation.
    PxMaterialFlag_CompliantContact = 8,
} PxMaterialFlag;


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
    PxCombineMode_Average = 0,
    /// Minimum: minimum(a,b)
    PxCombineMode_Min = 1,
    /// Multiply: a*b
    PxCombineMode_Multiply = 2,
    /// Maximum: maximum(a,b)
    PxCombineMode_Max = 3,
    /// This is not a valid combine mode, it is a sentinel to denote the number of possible values. We assert that the variable's value is smaller than this.
    PxCombineMode_NValues = 4,
    /// This is not a valid combine mode, it is to assure that the size of the enum type is big enough.
    PxCombineMode_Pad32 = 2147483647,
} PxCombineMode;

/// Identifies dirty particle buffers that need to be updated in the particle system.
///
/// This flag can be used mark the device user buffers that are dirty and need to be written to the particle system.
typedef enum PxParticleBufferFlag: int32_t {
    /// No data specified
    PxParticleBufferFlag_None = 0,
    /// Specifies the position (first 3 floats) and inverse mass (last float) data (array of PxVec4 * number of particles)
    PxParticleBufferFlag_UpdatePosition = 1,
    /// Specifies the velocity (first 3 floats) data (array of PxVec4 * number of particles)
    PxParticleBufferFlag_UpdateVelocity = 2,
    /// Specifies the per-particle phase flag data (array of PxU32 * number of particles)
    PxParticleBufferFlag_UpdatePhase = 4,
    /// Specifies the rest position (first 3 floats) data for cloth buffers
    PxParticleBufferFlag_UpdateRestposition = 8,
    /// Specifies the cloth buffer (see PxParticleClothBuffer)
    PxParticleBufferFlag_UpdateCloth = 32,
    /// Specifies the rigid buffer (see PxParticleRigidBuffer)
    PxParticleBufferFlag_UpdateRigid = 64,
    /// Specifies the diffuse particle parameter buffer (see PxDiffuseParticleParams)
    PxParticleBufferFlag_UpdateDiffuseParam = 128,
    /// Specifies the attachments.
    PxParticleBufferFlag_UpdateAttachments = 256,
    PxParticleBufferFlag_All = 495,
} PxParticleBufferFlag;


/// Identifies per-particle behavior for a PxParticleSystem.
///
/// See [`PxParticleSystem::createPhase`]().
typedef enum PxParticlePhaseFlag: uint32_t {
    /// Bits [ 0, 19] represent the particle group for controlling collisions
    PxParticlePhaseFlag_ParticlePhaseGroupMask = 1048575,
    /// Bits [20, 23] hold flags about how the particle behave
    PxParticlePhaseFlag_ParticlePhaseFlagsMask = 4293918720,
    /// If set this particle will interact with particles of the same group
    PxParticlePhaseFlag_ParticlePhaseSelfCollide = 1048576,
    /// If set this particle will ignore collisions with particles closer than the radius in the rest pose, this flag should not be specified unless valid rest positions have been specified using setRestParticles()
    PxParticlePhaseFlag_ParticlePhaseSelfCollideFilter = 2097152,
    /// If set this particle will generate fluid density constraints for its overlapping neighbors
    PxParticlePhaseFlag_ParticlePhaseFluid = 4194304,
} PxParticlePhaseFlag;


/// Specifies memory space for a PxBuffer instance.
typedef enum PxBufferType: int32_t {
    PxBufferType_Host = 0,
    PxBufferType_Device = 1,
} PxBufferType;

/// Filtering flags for scene queries.
typedef enum PxQueryFlag: int32_t {
    /// Traverse static shapes
    PxQueryFlag_Static = 1,
    /// Traverse dynamic shapes
    PxQueryFlag_Dynamic = 2,
    /// Run the pre-intersection-test filter (see [`PxQueryFilterCallback::preFilter`]())
    PxQueryFlag_Prefilter = 4,
    /// Run the post-intersection-test filter (see [`PxQueryFilterCallback::postFilter`]())
    PxQueryFlag_Postfilter = 8,
    /// Abort traversal as soon as any hit is found and return it via callback.block.
    /// Helps query performance. Both eTOUCH and eBLOCK hitTypes are considered hits with this flag.
    PxQueryFlag_AnyHit = 16,
    /// All hits are reported as touching. Overrides eBLOCK returned from user filters with eTOUCH.
    /// This is also an optimization hint that may improve query performance.
    PxQueryFlag_NoBlock = 32,
    /// Same as eBATCH_QUERY_LEGACY_BEHAVIOUR, more explicit name making it clearer that this can also be used
    /// with regular/non-batched queries if needed.
    PxQueryFlag_DisableHardcodedFilter = 64,
    /// Reserved for internal use
    PxQueryFlag_Reserved = 32768,
} PxQueryFlag;


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
    PxQueryHitType_None = 0,
    /// a hit on the shape touches the intersection geometry of the query but does not block it
    PxQueryHitType_Touch = 1,
    /// a hit on the shape blocks the query (does not block overlap queries)
    PxQueryHitType_Block = 2,
} PxQueryHitType;

/// Collection of flags providing a mechanism to lock motion along/around a specific axis.
typedef enum PxRigidDynamicLockFlag: int32_t {
    PxRigidDynamicLockFlag_LockLinearX = 1,
    PxRigidDynamicLockFlag_LockLinearY = 2,
    PxRigidDynamicLockFlag_LockLinearZ = 4,
    PxRigidDynamicLockFlag_LockAngularX = 8,
    PxRigidDynamicLockFlag_LockAngularY = 16,
    PxRigidDynamicLockFlag_LockAngularZ = 32,
} PxRigidDynamicLockFlag;


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
    PxPruningStructureType_None = 0,
    /// Using a dynamic AABB tree
    PxPruningStructureType_DynamicAabbTree = 1,
    /// Using a static AABB tree
    PxPruningStructureType_StaticAabbTree = 2,
    PxPruningStructureType_Last = 3,
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
    PxDynamicTreeSecondaryPruner_None = 0,
    /// bucket-based secondary pruner, faster updates, slower query time
    PxDynamicTreeSecondaryPruner_Bucket = 1,
    /// incremental-BVH secondary pruner, faster query time, slower updates
    PxDynamicTreeSecondaryPruner_Incremental = 2,
    /// PxBVH-based secondary pruner, good overall performance
    PxDynamicTreeSecondaryPruner_Bvh = 3,
    PxDynamicTreeSecondaryPruner_Last = 4,
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
    PxSceneQueryUpdateMode_BuildEnabledCommitEnabled = 0,
    /// Scene query build only is executed.
    PxSceneQueryUpdateMode_BuildEnabledCommitDisabled = 1,
    /// No work is done, no update of scene queries
    PxSceneQueryUpdateMode_BuildDisabledCommitDisabled = 2,
} PxSceneQueryUpdateMode;

/// Built-in enum for default PxScene pruners
///
/// This is passed as a pruner index to various functions in the following APIs.
typedef enum PxScenePrunerIndex: uint32_t {
    PxScenePrunerIndex_PxScenePrunerStatic = 0,
    PxScenePrunerIndex_PxScenePrunerDynamic = 1,
    PxScenePrunerIndex_PxSceneCompoundPruner = 4294967295,
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
    PxBroadPhaseType_Sap = 0,
    /// Multi box pruning
    PxBroadPhaseType_Mbp = 1,
    /// Automatic box pruning
    PxBroadPhaseType_Abp = 2,
    /// Parallel automatic box pruning
    PxBroadPhaseType_Pabp = 3,
    /// GPU broad phase
    PxBroadPhaseType_Gpu = 4,
    PxBroadPhaseType_Last = 5,
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
    PxFrictionType_Patch = 0,
    /// Select one directional per-contact friction model.
    PxFrictionType_OneDirectional = 1,
    /// Select two directional per-contact friction model.
    PxFrictionType_TwoDirectional = 2,
    /// The total number of friction models supported by the SDK.
    PxFrictionType_FrictionCount = 3,
} PxFrictionType;

/// Enum for selecting the type of solver used for the simulation.
///
/// [`PxSolverType::ePGS`] selects the iterative sequential impulse solver. This is the same kind of solver used in PhysX 3.4 and earlier releases.
///
/// [`PxSolverType::eTGS`] selects a non linear iterative solver. This kind of solver can lead to improved convergence and handle large mass ratios, long chains and jointed systems better. It is slightly more expensive than the default solver and can introduce more energy to correct joint and contact errors.
typedef enum PxSolverType: int32_t {
    /// Projected Gauss-Seidel iterative solver
    PxSolverType_Pgs = 0,
    /// Default Temporal Gauss-Seidel solver
    PxSolverType_Tgs = 1,
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
    PxSceneFlag_EnableActiveActors = 1,
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
    PxSceneFlag_EnableCcd = 2,
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
    PxSceneFlag_DisableCcdResweep = 4,
    /// Enable GJK-based distance collision detection system.
    ///
    /// This flag is not mutable, and must be set in PxSceneDesc at scene creation.
    ///
    /// Default:
    /// true
    PxSceneFlag_EnablePcm = 64,
    /// Disable contact report buffer resize. Once the contact buffer is full, the rest of the contact reports will
    /// not be buffered and sent.
    ///
    /// This flag is not mutable, and must be set in PxSceneDesc at scene creation.
    ///
    /// Default:
    /// false
    PxSceneFlag_DisableContactReportBufferResize = 128,
    /// Disable contact cache.
    ///
    /// Contact caches are used internally to provide faster contact generation. You can disable all contact caches
    /// if memory usage for this feature becomes too high.
    ///
    /// This flag is not mutable, and must be set in PxSceneDesc at scene creation.
    ///
    /// Default:
    /// false
    PxSceneFlag_DisableContactCache = 256,
    /// Require scene-level locking
    ///
    /// When set to true this requires that threads accessing the PxScene use the
    /// multi-threaded lock methods.
    ///
    /// This flag is not mutable, and must be set in PxSceneDesc at scene creation.
    ///
    /// Default:
    /// false
    PxSceneFlag_RequireRwLock = 512,
    /// Enables additional stabilization pass in solver
    ///
    /// When set to true, this enables additional stabilization processing to improve that stability of complex interactions between large numbers of bodies.
    ///
    /// Note that this flag is not mutable and must be set in PxSceneDesc at scene creation. Also, this is an experimental feature which does result in some loss of momentum.
    PxSceneFlag_EnableStabilization = 1024,
    /// Enables average points in contact manifolds
    ///
    /// When set to true, this enables additional contacts to be generated per manifold to represent the average point in a manifold. This can stabilize stacking when only a small
    /// number of solver iterations is used.
    ///
    /// Note that this flag is not mutable and must be set in PxSceneDesc at scene creation.
    PxSceneFlag_EnableAveragePoint = 2048,
    /// Do not report kinematics in list of active actors.
    ///
    /// Since the target pose for kinematics is set by the user, an application can track the activity state directly and use
    /// this flag to avoid that kinematics get added to the list of active actors.
    ///
    /// This flag has only an effect in combination with eENABLE_ACTIVE_ACTORS.
    ///
    /// Default:
    /// false
    PxSceneFlag_ExcludeKinematicsFromActiveActors = 4096,
    /// Do not report kinematics in list of active actors.
    ///
    /// Since the target pose for kinematics is set by the user, an application can track the activity state directly and use
    /// this flag to avoid that kinematics get added to the list of active actors.
    ///
    /// This flag has only an effect in combination with eENABLE_ACTIVE_ACTORS.
    ///
    /// Default:
    /// false
    PxSceneFlag_EnableGpuDynamics = 8192,
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
    PxSceneFlag_EnableEnhancedDeterminism = 16384,
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
    PxSceneFlag_EnableFrictionEveryIteration = 32768,
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
    PxSceneFlag_SuppressReadback = 65536,
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
    PxSceneFlag_ForceReadback = 131072,
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
    PxSceneFlag_MutableFlags = 69633,
} PxSceneFlag;


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
    PxVisualizationParameter_Scale = 0,
    /// Visualize the world axes.
    PxVisualizationParameter_WorldAxes = 1,
    /// Visualize a bodies axes.
    PxVisualizationParameter_BodyAxes = 2,
    /// Visualize a body's mass axes.
    ///
    /// This visualization is also useful for visualizing the sleep state of bodies. Sleeping bodies are drawn in
    /// black, while awake bodies are drawn in white. If the body is sleeping and part of a sleeping group, it is
    /// drawn in red.
    PxVisualizationParameter_BodyMassAxes = 3,
    /// Visualize the bodies linear velocity.
    PxVisualizationParameter_BodyLinVelocity = 4,
    /// Visualize the bodies angular velocity.
    PxVisualizationParameter_BodyAngVelocity = 5,
    /// Visualize contact points. Will enable contact information.
    PxVisualizationParameter_ContactPoint = 6,
    /// Visualize contact normals. Will enable contact information.
    PxVisualizationParameter_ContactNormal = 7,
    /// Visualize contact errors. Will enable contact information.
    PxVisualizationParameter_ContactError = 8,
    /// Visualize Contact forces. Will enable contact information.
    PxVisualizationParameter_ContactForce = 9,
    /// Visualize actor axes.
    PxVisualizationParameter_ActorAxes = 10,
    /// Visualize bounds (AABBs in world space)
    PxVisualizationParameter_CollisionAabbs = 11,
    /// Shape visualization
    PxVisualizationParameter_CollisionShapes = 12,
    /// Shape axis visualization
    PxVisualizationParameter_CollisionAxes = 13,
    /// Compound visualization (compound AABBs in world space)
    PxVisualizationParameter_CollisionCompounds = 14,
    /// Mesh
    /// &
    /// convex face normals
    PxVisualizationParameter_CollisionFnormals = 15,
    /// Active edges for meshes
    PxVisualizationParameter_CollisionEdges = 16,
    /// Static pruning structures
    PxVisualizationParameter_CollisionStatic = 17,
    /// Dynamic pruning structures
    PxVisualizationParameter_CollisionDynamic = 18,
    /// Joint local axes
    PxVisualizationParameter_JointLocalFrames = 19,
    /// Joint limits
    PxVisualizationParameter_JointLimits = 20,
    /// Visualize culling box
    PxVisualizationParameter_CullBox = 21,
    /// MBP regions
    PxVisualizationParameter_MbpRegions = 22,
    /// Renders the simulation mesh instead of the collision mesh (only available for tetmeshes)
    PxVisualizationParameter_SimulationMesh = 23,
    /// Renders the SDF of a mesh instead of the collision mesh (only available for triangle meshes with SDFs)
    PxVisualizationParameter_Sdf = 24,
    /// This is not a parameter, it just records the current number of parameters (as maximum(PxVisualizationParameter)+1) for use in loops.
    PxVisualizationParameter_NumValues = 25,
    /// This is not a parameter, it just records the current number of parameters (as maximum(PxVisualizationParameter)+1) for use in loops.
    PxVisualizationParameter_ForceDword = 2147483647,
} PxVisualizationParameter;

/// Different types of rigid body collision pair statistics.
typedef enum RbPairStatsType: int32_t {
    /// Shape pairs processed as discrete contact pairs for the current simulation step.
    RbPairStatsType_DiscreteContactPairs = 0,
    /// Shape pairs processed as swept integration pairs for the current simulation step.
    ///
    /// Counts the pairs for which special CCD (continuous collision detection) work was actually done and NOT the number of pairs which were configured for CCD.
    /// Furthermore, there can be multiple CCD passes and all processed pairs of all passes are summed up, hence the number can be larger than the amount of pairs which have been configured for CCD.
    RbPairStatsType_CcdPairs = 1,
    /// Shape pairs processed with user contact modification enabled for the current simulation step.
    RbPairStatsType_ModifiedContactPairs = 2,
    /// Trigger shape pairs processed for the current simulation step.
    RbPairStatsType_TriggerPairs = 3,
} RbPairStatsType;

/// These flags determine what data is read or written to the gpu softbody.
typedef enum PxSoftBodyDataFlag: int32_t {
    /// The collision mesh tetrahedron indices (quadruples of int32)
    PxSoftBodyDataFlag_TetIndices = 0,
    /// The collision mesh cauchy stress tensors (float 3x3 matrices)
    PxSoftBodyDataFlag_TetStress = 1,
    /// The collision mesh tetrahedron von Mises stress (float scalar)
    PxSoftBodyDataFlag_TetStresscoeff = 2,
    /// The collision mesh tetrahedron rest poses (float 3x3 matrices)
    PxSoftBodyDataFlag_TetRestPoses = 3,
    /// The collision mesh tetrahedron orientations (quaternions, quadruples of float)
    PxSoftBodyDataFlag_TetRotations = 4,
    /// The collision mesh vertex positions and their inverted mass in the 4th component (quadruples of float)
    PxSoftBodyDataFlag_TetPositionInvMass = 5,
    /// The simulation mesh tetrahedron indices (quadruples of int32)
    PxSoftBodyDataFlag_SimTetIndices = 6,
    /// The simulation mesh vertex velocities and their inverted mass in the 4th component (quadruples of float)
    PxSoftBodyDataFlag_SimVelocityInvMass = 7,
    /// The simulation mesh vertex positions and their inverted mass in the 4th component (quadruples of float)
    PxSoftBodyDataFlag_SimPositionInvMass = 8,
    /// The simulation mesh kinematic target positions
    PxSoftBodyDataFlag_SimKinematicTarget = 9,
} PxSoftBodyDataFlag;

/// Identifies input and output buffers for PxHairSystem
typedef enum PxHairSystemData: int32_t {
    /// No data specified
    PxHairSystemData_None = 0,
    /// Specifies the position (first 3 floats) and inverse mass (last float) data (array of PxVec4 * max number of vertices)
    PxHairSystemData_PositionInvmass = 1,
    /// Specifies the velocity (first 3 floats) data (array of PxVec4 * max number of vertices)
    PxHairSystemData_Velocity = 2,
    /// Specifies everything
    PxHairSystemData_All = 3,
} PxHairSystemData;


/// Binary settings for hair system simulation
typedef enum PxHairSystemFlag: int32_t {
    /// Determines if self-collision between hair vertices is ignored
    PxHairSystemFlag_DisableSelfCollision = 1,
    /// Determines if collision between hair and external bodies is ignored
    PxHairSystemFlag_DisableExternalCollision = 2,
    /// Determines if attachment constraint is also felt by body to which the hair is attached
    PxHairSystemFlag_DisableTwosidedAttachment = 4,
} PxHairSystemFlag;


/// Identifies each type of information for retrieving from actor.
typedef enum PxActorCacheFlag: int32_t {
    PxActorCacheFlag_ActorData = 1,
    PxActorCacheFlag_Force = 4,
    PxActorCacheFlag_Torque = 8,
} PxActorCacheFlag;


/// PVD scene Flags. They are disabled by default, and only works if PxPvdInstrumentationFlag::eDEBUG is set.
typedef enum PxPvdSceneFlag: int32_t {
    PxPvdSceneFlag_TransmitContacts = 1,
    /// Transmits contact stream to PVD.
    PxPvdSceneFlag_TransmitScenequeries = 2,
    /// Transmits scene query stream to PVD.
    PxPvdSceneFlag_TransmitConstraints = 4,
} PxPvdSceneFlag;


/// Identifies each type of actor for retrieving actors from a scene.
///
/// [`PxArticulationLink`] objects are not supported. Use the #PxArticulationReducedCoordinate object to retrieve all its links.
typedef enum PxActorTypeFlag: int32_t {
    /// A static rigid body
    PxActorTypeFlag_RigidStatic = 1,
    /// A dynamic rigid body
    PxActorTypeFlag_RigidDynamic = 2,
} PxActorTypeFlag;


/// Extra data item types for contact pairs.
typedef enum PxContactPairExtraDataType: int32_t {
    /// see [`PxContactPairVelocity`]
    PxContactPairExtraDataType_PreSolverVelocity = 0,
    /// see [`PxContactPairVelocity`]
    PxContactPairExtraDataType_PostSolverVelocity = 1,
    /// see [`PxContactPairPose`]
    PxContactPairExtraDataType_ContactEventPose = 2,
    /// see [`PxContactPairIndex`]
    PxContactPairExtraDataType_ContactPairIndex = 3,
} PxContactPairExtraDataType;

/// Collection of flags providing information on contact report pairs.
typedef enum PxContactPairHeaderFlag: int32_t {
    /// The actor with index 0 has been removed from the scene.
    PxContactPairHeaderFlag_RemovedActor0 = 1,
    /// The actor with index 1 has been removed from the scene.
    PxContactPairHeaderFlag_RemovedActor1 = 2,
} PxContactPairHeaderFlag;


/// Collection of flags providing information on contact report pairs.
typedef enum PxContactPairFlag: int32_t {
    /// The shape with index 0 has been removed from the actor/scene.
    PxContactPairFlag_RemovedShape0 = 1,
    /// The shape with index 1 has been removed from the actor/scene.
    PxContactPairFlag_RemovedShape1 = 2,
    /// First actor pair contact.
    ///
    /// The provided shape pair marks the first contact between the two actors, no other shape pair has been touching prior to the current simulation frame.
    ///
    /// : This info is only available if [`PxPairFlag::eNOTIFY_TOUCH_FOUND`] has been declared for the pair.
    PxContactPairFlag_ActorPairHasFirstTouch = 4,
    /// All contact between the actor pair was lost.
    ///
    /// All contact between the two actors has been lost, no shape pairs remain touching after the current simulation frame.
    PxContactPairFlag_ActorPairLostTouch = 8,
    /// Internal flag, used by [`PxContactPair`].extractContacts()
    ///
    /// The applied contact impulses are provided for every contact point.
    /// This is the case if [`PxPairFlag::eSOLVE_CONTACT`] has been set for the pair.
    PxContactPairFlag_InternalHasImpulses = 16,
    /// Internal flag, used by [`PxContactPair`].extractContacts()
    ///
    /// The provided contact point information is flipped with regards to the shapes of the contact pair. This mainly concerns the order of the internal triangle indices.
    PxContactPairFlag_InternalContactsAreFlipped = 32,
} PxContactPairFlag;


/// Collection of flags providing information on trigger report pairs.
typedef enum PxTriggerPairFlag: int32_t {
    /// The trigger shape has been removed from the actor/scene.
    PxTriggerPairFlag_RemovedShapeTrigger = 1,
    /// The shape causing the trigger event has been removed from the actor/scene.
    PxTriggerPairFlag_RemovedShapeOther = 2,
    /// For internal use only.
    PxTriggerPairFlag_NextFree = 4,
} PxTriggerPairFlag;


/// Identifies input and output buffers for PxSoftBody.
typedef enum PxSoftBodyData: int32_t {
    PxSoftBodyData_None = 0,
    /// Flag to request access to the collision mesh's positions; read only
    PxSoftBodyData_PositionInvmass = 1,
    /// Flag to request access to the simulation mesh's positions and inverse masses
    PxSoftBodyData_SimPositionInvmass = 4,
    /// Flag to request access to the simulation mesh's velocities and inverse masses
    PxSoftBodyData_SimVelocity = 8,
    /// Flag to request access to the simulation mesh's kinematic target position
    PxSoftBodyData_SimKinematicTarget = 16,
    PxSoftBodyData_All = 29,
} PxSoftBodyData;


/// Flags to enable or disable special modes of a SoftBody
typedef enum PxSoftBodyFlag: int32_t {
    /// Determines if self collision will be detected and resolved
    PxSoftBodyFlag_DisableSelfCollision = 1,
    /// Enables computation of a Cauchy stress tensor for every tetrahedron in the simulation mesh. The tensors can be accessed through the softbody direct API
    PxSoftBodyFlag_ComputeStressTensor = 2,
    /// Enables support for continuous collision detection
    PxSoftBodyFlag_EnableCcd = 4,
    /// Enable debug rendering to display the simulation mesh
    PxSoftBodyFlag_DisplaySimMesh = 8,
    /// Enables support for kinematic motion of the collision and simulation mesh.
    PxSoftBodyFlag_Kinematic = 16,
    /// Enables partially kinematic motion of the collisios and simulation mesh.
    PxSoftBodyFlag_PartiallyKinematic = 32,
} PxSoftBodyFlag;


/// The type of controller, eg box, sphere or capsule.
typedef enum PxControllerShapeType: int32_t {
    /// A box controller.
    PxControllerShapeType_Box = 0,
    /// A capsule controller
    PxControllerShapeType_Capsule = 1,
    /// A capsule controller
    PxControllerShapeType_ForceDword = 2147483647,
} PxControllerShapeType;

/// specifies how a CCT interacts with non-walkable parts.
///
/// This is only used when slopeLimit is non zero. It is currently enabled for static actors only, and not supported for spheres or capsules.
typedef enum PxControllerNonWalkableMode: int32_t {
    /// Stops character from climbing up non-walkable slopes, but doesn't move it otherwise
    PxControllerNonWalkableMode_PreventClimbing = 0,
    /// Stops character from climbing up non-walkable slopes, and forces it to slide down those slopes
    PxControllerNonWalkableMode_PreventClimbingAndForceSliding = 1,
} PxControllerNonWalkableMode;

/// specifies which sides a character is colliding with.
typedef enum PxControllerCollisionFlag: int32_t {
    /// Character is colliding to the sides.
    PxControllerCollisionFlag_CollisionSides = 1,
    /// Character has collision above.
    PxControllerCollisionFlag_CollisionUp = 2,
    /// Character has collision below.
    PxControllerCollisionFlag_CollisionDown = 4,
} PxControllerCollisionFlag;


typedef enum PxCapsuleClimbingMode: int32_t {
    /// Standard mode, let the capsule climb over surfaces according to impact normal
    PxCapsuleClimbingMode_Easy = 0,
    /// Constrained mode, try to limit climbing according to the step offset
    PxCapsuleClimbingMode_Constrained = 1,
    PxCapsuleClimbingMode_Last = 2,
} PxCapsuleClimbingMode;

/// specifies controller behavior
typedef enum PxControllerBehaviorFlag: int32_t {
    /// Controller can ride on touched object (i.e. when this touched object is moving horizontally).
    ///
    /// The CCT vs. CCT case is not supported.
    PxControllerBehaviorFlag_CctCanRideOnObject = 1,
    /// Controller should slide on touched object
    PxControllerBehaviorFlag_CctSlide = 2,
    /// Disable all code dealing with controllers riding on objects, let users define it outside of the SDK.
    PxControllerBehaviorFlag_CctUserDefinedRide = 4,
} PxControllerBehaviorFlag;


/// specifies debug-rendering flags
typedef enum PxControllerDebugRenderFlag: uint32_t {
    /// Temporal bounding volume around controllers
    PxControllerDebugRenderFlag_TemporalBv = 1,
    /// Cached bounding volume around controllers
    PxControllerDebugRenderFlag_CachedBv = 2,
    /// User-defined obstacles
    PxControllerDebugRenderFlag_Obstacles = 4,
    PxControllerDebugRenderFlag_None = 0,
    PxControllerDebugRenderFlag_All = 4294967295,
} PxControllerDebugRenderFlag;


/// Defines the number of bits per subgrid pixel
typedef enum PxSdfBitsPerSubgridPixel: int32_t {
    /// 8 bit per subgrid pixel (values will be stored as normalized integers)
    PxSdfBitsPerSubgridPixel_E8BitPerPixel = 1,
    /// 16 bit per subgrid pixel (values will be stored as normalized integers)
    PxSdfBitsPerSubgridPixel_E16BitPerPixel = 2,
    /// 32 bit per subgrid pixel (values will be stored as floats in world scale units)
    PxSdfBitsPerSubgridPixel_E32BitPerPixel = 4,
} PxSdfBitsPerSubgridPixel;

/// Flags which describe the format and behavior of a convex mesh.
typedef enum PxConvexFlag: int32_t {
    /// Denotes the use of 16-bit vertex indices in PxConvexMeshDesc::triangles or PxConvexMeshDesc::polygons.
    /// (otherwise, 32-bit indices are assumed)
    PxConvexFlag_E16BitIndices = 1,
    /// Automatically recomputes the hull from the vertices. If this flag is not set, you must provide the entire geometry manually.
    ///
    /// There are two different algorithms for hull computation, please see PxConvexMeshCookingType.
    PxConvexFlag_ComputeConvex = 2,
    /// Checks and removes almost zero-area triangles during convex hull computation.
    /// The rejected area size is specified in PxCookingParams::areaTestEpsilon
    ///
    /// This flag is only used in combination with eCOMPUTE_CONVEX.
    PxConvexFlag_CheckZeroAreaTriangles = 4,
    /// Quantizes the input vertices using the k-means clustering
    ///
    /// The input vertices are quantized to PxConvexMeshDesc::quantizedCount
    /// see http://en.wikipedia.org/wiki/K-means_clustering
    PxConvexFlag_QuantizeInput = 8,
    /// Disables the convex mesh validation to speed-up hull creation. Please use separate validation
    /// function in checked/debug builds. Creating a convex mesh with invalid input data without prior validation
    /// may result in undefined behavior.
    PxConvexFlag_DisableMeshValidation = 16,
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
    PxConvexFlag_PlaneShifting = 32,
    /// Inertia tensor computation is faster using SIMD code, but the precision is lower, which may result
    /// in incorrect inertia for very thin hulls.
    PxConvexFlag_FastInertiaComputation = 64,
    /// Convex hulls are created with respect to GPU simulation limitations. Vertex limit and polygon limit
    /// is set to 64 and vertex limit per face is internally set to 32.
    ///
    /// Can be used only with eCOMPUTE_CONVEX flag.
    PxConvexFlag_GpuCompatible = 128,
    /// Convex hull input vertices are shifted to be around origin to provide better computation stability.
    /// It is recommended to provide input vertices around the origin, otherwise use this flag to improve
    /// numerical stability.
    ///
    /// Is used only with eCOMPUTE_CONVEX flag.
    PxConvexFlag_ShiftVertices = 256,
} PxConvexFlag;


/// Defines the tetrahedron structure of a mesh.
typedef enum PxMeshFormat: int32_t {
    /// Normal tetmesh with arbitrary tetrahedra
    PxMeshFormat_TetMesh = 0,
    /// 6 tetrahedra in a row will form a hexahedron
    PxMeshFormat_HexMesh = 1,
} PxMeshFormat;

/// Desired build strategy for PxMeshMidPhase::eBVH34
typedef enum PxBVH34BuildStrategy: int32_t {
    /// Fast build strategy. Fast build speed, good runtime performance in most cases. Recommended for runtime mesh cooking.
    PxBVH34BuildStrategy_Fast = 0,
    /// Default build strategy. Medium build speed, good runtime performance in all cases.
    PxBVH34BuildStrategy_Default = 1,
    /// SAH build strategy. Slower builds, slightly improved runtime performance in some cases.
    PxBVH34BuildStrategy_Sah = 2,
    PxBVH34BuildStrategy_Last = 3,
} PxBVH34BuildStrategy;

/// Result from convex cooking.
typedef enum PxConvexMeshCookingResult: int32_t {
    /// Convex mesh cooking succeeded.
    PxConvexMeshCookingResult_Success = 0,
    /// Convex mesh cooking failed, algorithm couldn't find 4 initial vertices without a small triangle.
    PxConvexMeshCookingResult_ZeroAreaTestFailed = 1,
    /// Convex mesh cooking succeeded, but the algorithm has reached the 255 polygons limit.
    /// The produced hull does not contain all input vertices. Try to simplify the input vertices
    /// or try to use the eINFLATE_CONVEX or the eQUANTIZE_INPUT flags.
    PxConvexMeshCookingResult_PolygonsLimitReached = 2,
    /// Something unrecoverable happened. Check the error stream to find out what.
    PxConvexMeshCookingResult_Failure = 3,
} PxConvexMeshCookingResult;

/// Enumeration for convex mesh cooking algorithms.
typedef enum PxConvexMeshCookingType: int32_t {
    /// The Quickhull algorithm constructs the hull from the given input points. The resulting hull
    /// will only contain a subset of the input points.
    PxConvexMeshCookingType_Quickhull = 0,
} PxConvexMeshCookingType;

/// Result from triangle mesh cooking
typedef enum PxTriangleMeshCookingResult: int32_t {
    /// Everything is A-OK.
    PxTriangleMeshCookingResult_Success = 0,
    /// a triangle is too large for well-conditioned results. Tessellate the mesh for better behavior, see the user guide section on cooking for more details.
    PxTriangleMeshCookingResult_LargeTriangle = 1,
    /// Something unrecoverable happened. Check the error stream to find out what.
    PxTriangleMeshCookingResult_Failure = 2,
} PxTriangleMeshCookingResult;

/// Enum for the set of mesh pre-processing parameters.
typedef enum PxMeshPreprocessingFlag: int32_t {
    /// When set, mesh welding is performed. See PxCookingParams::meshWeldTolerance. Clean mesh must be enabled.
    PxMeshPreprocessingFlag_WeldVertices = 1,
    /// When set, mesh cleaning is disabled. This makes cooking faster.
    ///
    /// When clean mesh is not performed, mesh welding is also not performed.
    ///
    /// It is recommended to use only meshes that passed during validateTriangleMesh.
    PxMeshPreprocessingFlag_DisableCleanMesh = 2,
    /// When set, active edges are set for each triangle edge. This makes cooking faster but slow up contact generation.
    PxMeshPreprocessingFlag_DisableActiveEdgesPrecompute = 4,
    /// When set, 32-bit indices will always be created regardless of triangle count.
    ///
    /// By default mesh will be created with 16-bit indices for triangle count
    /// <
    /// = 0xFFFF and 32-bit otherwise.
    PxMeshPreprocessingFlag_Force32bitIndices = 8,
    /// When set, a list of triangles will be created for each associated vertex in the mesh
    PxMeshPreprocessingFlag_EnableVertMapping = 16,
    /// When set, inertia tensor is calculated for the mesh
    PxMeshPreprocessingFlag_EnableInertia = 32,
} PxMeshPreprocessingFlag;


/// Unique identifiers for extensions classes which implement a constraint based on PxConstraint.
///
/// Users which want to create their own custom constraint types should choose an ID larger or equal to eNEXT_FREE_ID
/// and not eINVALID_ID.
typedef enum PxConstraintExtIDs: int32_t {
    PxConstraintExtIDs_Joint = 0,
    PxConstraintExtIDs_VehicleSuspLimitDeprecated = 1,
    PxConstraintExtIDs_VehicleStickyTyreDeprecated = 2,
    PxConstraintExtIDs_VehicleJoint = 3,
    PxConstraintExtIDs_NextFreeId = 4,
    PxConstraintExtIDs_InvalidId = 2147483647,
} PxConstraintExtIDs;

/// an enumeration of PhysX' built-in joint types
typedef enum PxJointConcreteType: int32_t {
    PxJointConcreteType_Spherical = 256,
    PxJointConcreteType_Revolute = 257,
    PxJointConcreteType_Prismatic = 258,
    PxJointConcreteType_Fixed = 259,
    PxJointConcreteType_Distance = 260,
    PxJointConcreteType_D6 = 261,
    PxJointConcreteType_Contact = 262,
    PxJointConcreteType_Gear = 263,
    PxJointConcreteType_RackAndPinion = 264,
    PxJointConcreteType_Last = 265,
} PxJointConcreteType;

/// an enumeration for specifying one or other of the actors referenced by a joint
typedef enum PxJointActorIndex: int32_t {
    PxJointActorIndex_Actor0 = 0,
    PxJointActorIndex_Actor1 = 1,
    PxJointActorIndex_Count = 2,
} PxJointActorIndex;

/// flags for configuring the drive of a PxDistanceJoint
typedef enum PxDistanceJointFlag: int32_t {
    PxDistanceJointFlag_MaxDistanceEnabled = 2,
    PxDistanceJointFlag_MinDistanceEnabled = 4,
    PxDistanceJointFlag_SpringEnabled = 8,
} PxDistanceJointFlag;


/// Flags specific to the prismatic joint.
typedef enum PxPrismaticJointFlag: int32_t {
    PxPrismaticJointFlag_LimitEnabled = 2,
} PxPrismaticJointFlag;


/// Flags specific to the Revolute Joint.
typedef enum PxRevoluteJointFlag: int32_t {
    /// enable the limit
    PxRevoluteJointFlag_LimitEnabled = 1,
    /// enable the drive
    PxRevoluteJointFlag_DriveEnabled = 2,
    /// if the existing velocity is beyond the drive velocity, do not add force
    PxRevoluteJointFlag_DriveFreespin = 4,
} PxRevoluteJointFlag;


/// Flags specific to the spherical joint.
typedef enum PxSphericalJointFlag: int32_t {
    /// the cone limit for the spherical joint is enabled
    PxSphericalJointFlag_LimitEnabled = 2,
} PxSphericalJointFlag;


/// Used to specify one of the degrees of freedom of  a D6 joint.
typedef enum PxD6Axis: int32_t {
    /// motion along the X axis
    PxD6Axis_X = 0,
    /// motion along the Y axis
    PxD6Axis_Y = 1,
    /// motion along the Z axis
    PxD6Axis_Z = 2,
    /// motion around the X axis
    PxD6Axis_Twist = 3,
    /// motion around the Y axis
    PxD6Axis_Swing1 = 4,
    /// motion around the Z axis
    PxD6Axis_Swing2 = 5,
    PxD6Axis_Count = 6,
} PxD6Axis;

/// Used to specify the range of motions allowed for a degree of freedom in a D6 joint.
typedef enum PxD6Motion: int32_t {
    /// The DOF is locked, it does not allow relative motion.
    PxD6Motion_Locked = 0,
    /// The DOF is limited, it only allows motion within a specific range.
    PxD6Motion_Limited = 1,
    /// The DOF is free and has its full range of motion.
    PxD6Motion_Free = 2,
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
    PxD6Drive_X = 0,
    /// drive along the Y-axis
    PxD6Drive_Y = 1,
    /// drive along the Z-axis
    PxD6Drive_Z = 2,
    /// drive of displacement from the X-axis
    PxD6Drive_Swing = 3,
    /// drive of the displacement around the X-axis
    PxD6Drive_Twist = 4,
    /// drive of all three angular degrees along a SLERP-path
    PxD6Drive_Slerp = 5,
    PxD6Drive_Count = 6,
} PxD6Drive;


/// flags for configuring the drive model of a PxD6Joint
typedef enum PxD6JointDriveFlag: int32_t {
    /// drive spring is for the acceleration at the joint (rather than the force)
    PxD6JointDriveFlag_Acceleration = 1,
} PxD6JointDriveFlag;


/// Collision filtering operations.
typedef enum PxFilterOp: int32_t {
    PxFilterOp_PxFilteropAnd = 0,
    PxFilterOp_PxFilteropOr = 1,
    PxFilterOp_PxFilteropXor = 2,
    PxFilterOp_PxFilteropNand = 3,
    PxFilterOp_PxFilteropNor = 4,
    PxFilterOp_PxFilteropNxor = 5,
    PxFilterOp_PxFilteropSwapAnd = 6,
} PxFilterOp;

/// If a thread ends up waiting for work it will find itself in a spin-wait loop until work becomes available.
/// Three strategies are available to limit wasted cycles.
/// The strategies are as follows:
/// a) wait until a work task signals the end of the spin-wait period.
/// b) yield the thread by providing a hint to reschedule thread execution, thereby allowing other threads to run.
/// c) yield the processor by informing it that it is waiting for work and requesting it to more efficiently use compute resources.
typedef enum PxDefaultCpuDispatcherWaitForWorkMode: int32_t {
    PxDefaultCpuDispatcherWaitForWorkMode_WaitForWork = 0,
    PxDefaultCpuDispatcherWaitForWorkMode_YieldThread = 1,
    PxDefaultCpuDispatcherWaitForWorkMode_YieldProcessor = 2,
} PxDefaultCpuDispatcherWaitForWorkMode;

typedef enum PxBatchQueryStatus: int32_t {
    /// This is the initial state before a query starts.
    PxBatchQueryStatus_Pending = 0,
    /// The query is finished; results have been written into the result and hit buffers.
    PxBatchQueryStatus_Success = 1,
    /// The query results were incomplete due to touch hit buffer overflow. Blocking hit is still correct.
    PxBatchQueryStatus_Overflow = 2,
} PxBatchQueryStatus;

/// types of instrumentation that PVD can do.
typedef enum PxPvdInstrumentationFlag: int32_t {
    /// Send debugging information to PVD.
    ///
    /// This information is the actual object data of the rigid statics, shapes,
    /// articulations, etc.  Sending this information has a noticeable impact on
    /// performance and thus this flag should not be set if you want an accurate
    /// performance profile.
    PxPvdInstrumentationFlag_Debug = 1,
    /// Send profile information to PVD.
    ///
    /// This information populates PVD's profile view.  It has (at this time) negligible
    /// cost compared to Debug information and makes PVD *much* more useful so it is quite
    /// highly recommended.
    ///
    /// This flag works together with a PxCreatePhysics parameter.
    /// Using it allows the SDK to send profile events to PVD.
    PxPvdInstrumentationFlag_Profile = 2,
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
    PxPvdInstrumentationFlag_Memory = 4,
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
    PxPvdInstrumentationFlag_All = 7,
} PxPvdInstrumentationFlag;

